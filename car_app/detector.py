"""YOLOv8 识别（地平线 BPU）：抓帧 → letterbox NV12 → hobot_dnn 推理 → DFL 解码 + 按类 NMS。

20 类：1~10 嫌疑人、11~20 刀具。置信度过滤阈值在 config.YOLO_CONF_THRESHOLD。
后处理（sigmoid/DFL/NMS）是纯 numpy，可桌面单测；hobot_dnn 的输入布局/加载
API/返回类型已实机验证（一维 NV12 输入、dnn.load 返回列表、forward 返回带 .buffer 的张量）。

关键修复：
- DFL 的 l/t/r/b 距离乘对应 stride（原来漏乘导致框缩 8/16/32 倍）。
- 类别通道按 runtime-manifest.json 的 ``class_names`` 映射（模型通道序是
  1,10,11,...,9，不是 index+1）。
- 预处理用 letterbox（127 填充）+ I420 交错 UV 成 NV12（不再拉伸 + YUV444 降采样）。
- 按类别分别 NMS，再按分数截断到 manifest 的 max_detections。
"""

from __future__ import annotations

import json
from dataclasses import dataclass
import math
from pathlib import Path
import queue
import threading
import time
import uuid

import numpy as np

import config
import runlog


@dataclass(frozen=True)
class Detection:
    class_id: int      # 1..20（1~10 嫌疑人、11~20 刀具）
    score: float
    box: tuple[float, float, float, float]  # x1,y1,x2,y2（像素）


@dataclass(frozen=True)
class _OutputSchema:
    """runtime-manifest 中单个推理输出的冻结约束。"""

    name: str
    role: str
    stride: int
    dtype: np.dtype
    shape: tuple[int, int, int, int]


# ---------------------------------------------------------------------------
# 后处理（纯 numpy）
# ---------------------------------------------------------------------------
def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def _softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
    e = np.exp(x - x.max(axis=axis, keepdims=True))
    return e / e.sum(axis=axis, keepdims=True)


def dfl_decode(box_logits: np.ndarray, reg_bins: int = 16) -> np.ndarray:
    """DFL 解码：[H,W,64] → [H,W,4]（l,t,r,b 距离，单位：cell）。"""
    h, w = box_logits.shape[:2]
    x = box_logits.reshape(h, w, 4, reg_bins).astype(np.float32)
    w_ = _softmax(x, axis=-1)
    bin_idx = np.arange(reg_bins, dtype=np.float32).reshape(1, 1, 1, reg_bins)
    return (w_ * bin_idx).sum(axis=-1)  # [h,w,4]


def yolo_decode_one(
    cls_logits: np.ndarray,
    box_logits: np.ndarray,
    stride: int,
    conf_thresh: float,
    class_names: list[str],
    reg_bins: int = 16,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """单个 stride 的解码，返回 (boxes, scores, class_ids)。

    class_ids 是业务类别号（1..20），由 manifest 的 class_names 映射而来。
    """
    cls = np.asarray(cls_logits).squeeze(0).astype(np.float32)   # [H,W,20]
    box = np.asarray(box_logits).squeeze(0).astype(np.float32)   # [H,W,64]
    h, w = cls.shape[:2]
    scores = _sigmoid(cls)                                       # [H,W,20]
    dist = dfl_decode(box, reg_bins) * stride                    # l,t,r,b 转像素
    xs = (np.arange(w, dtype=np.float32) + 0.5) * stride
    ys = (np.arange(h, dtype=np.float32) + 0.5) * stride
    cx = np.broadcast_to(xs[None, :], (h, w))
    cy = np.broadcast_to(ys[:, None], (h, w))
    l, t, r, b = dist[..., 0], dist[..., 1], dist[..., 2], dist[..., 3]
    boxes = np.stack([cx - l, cy - t, cx + r, cy + b], axis=-1)   # [H,W,4] xyxy
    rows, cols, channels = np.where(scores >= conf_thresh)
    class_ids = np.array([int(class_names[c]) for c in channels], dtype=np.int64)
    return boxes[rows, cols], scores[rows, cols, channels], class_ids


def nms(boxes: np.ndarray, scores: np.ndarray, iou_thresh: float) -> np.ndarray:
    """简单 numpy NMS，返回保留的索引（相对输入数组）。"""
    if len(boxes) == 0:
        return np.array([], dtype=np.int64)
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep: list[int] = []
    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[i] + areas[order[1:]] - inter
        iou = inter / np.maximum(union, 1e-9)
        order = order[1:][iou <= iou_thresh]
    return np.asarray(keep, dtype=np.int64)


def postprocess_yolo(
    outputs: list[np.ndarray],
    conf_thresh: float,
    iou_thresh: float,
    class_names: list[str],
    max_detections: int,
    reg_bins: int = 16,
    strides: tuple[int, ...] = (8, 16, 32),
) -> list[Detection]:
    """把 6 个输出张量解码成检测列表（按类别 NMS + top-k）。

    输出顺序（按 runtime-manifest）：output0/326 是 stride8 的 cls/box，
    334/342 是 stride16，350/358 是 stride32。
    """
    all_boxes: list[np.ndarray] = []
    all_scores: list[np.ndarray] = []
    all_cls: list[np.ndarray] = []
    if len(outputs) != len(strides) * 2:
        raise RuntimeError(
            f"模型输出张量数 {len(outputs)} != {len(strides) * 2}（预期 cls/box 成对输出）"
        )
    for idx, stride in enumerate(strides):
        boxes, scores, cls = yolo_decode_one(
            outputs[idx * 2], outputs[idx * 2 + 1], stride, conf_thresh, class_names, reg_bins
        )
        all_boxes.append(boxes)
        all_scores.append(scores)
        all_cls.append(cls)
    boxes = np.concatenate(all_boxes) if all_boxes else np.zeros((0, 4), dtype=np.float32)
    scores = np.concatenate(all_scores) if all_scores else np.zeros((0,), dtype=np.float32)
    cls = np.concatenate(all_cls) if all_cls else np.zeros((0,), dtype=np.int64)

    # 按类别分别 NMS：重叠的不同类别互不删除（例如刀具不被大的人脸框压掉）
    keep_idx: list[int] = []
    for c in np.unique(cls):
        mask = cls == c
        local_keep = nms(boxes[mask], scores[mask], iou_thresh)
        global_idx = np.where(mask)[0][local_keep]
        keep_idx.extend(int(i) for i in global_idx)

    keep_idx = np.asarray(keep_idx, dtype=np.int64)
    if keep_idx.size > 0:
        order = scores[keep_idx].argsort()[::-1]
        keep_idx = keep_idx[order][:max_detections]

    return [
        Detection(int(cls[i]), float(scores[i]), tuple(float(v) for v in boxes[i]))
        for i in keep_idx
    ]


# ---------------------------------------------------------------------------
# 图像 → NV12（letterbox + I420 交错 UV）
# ---------------------------------------------------------------------------
def _letterbox(
    bgr: np.ndarray, target_w: int, target_h: int, pad: int = 127
) -> tuple[np.ndarray, float, int, int]:
    """等比缩放 + 居中填充，返回 (canvas, scale, dx, dy)（scale/dx/dy 用于框逆映射）。"""
    import cv2

    h, w = bgr.shape[:2]
    scale = min(target_w / w, target_h / h)
    nw = max(1, int(round(w * scale)))
    nh = max(1, int(round(h * scale)))
    resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((target_h, target_w, 3), pad, dtype=np.uint8)
    dy = (target_h - nh) // 2
    dx = (target_w - nw) // 2
    canvas[dy:dy + nh, dx:dx + nw] = resized
    return canvas, scale, dx, dy


def letterbox(bgr: np.ndarray, target_w: int, target_h: int, pad: int = 127) -> np.ndarray:
    """等比缩放 + 居中填充，返回 (target_h, target_w, 3) 的 BGR。"""
    return _letterbox(bgr, target_w, target_h, pad)[0]


def bgr_to_nv12_with_meta(bgr: np.ndarray) -> tuple[np.ndarray, float, int, int]:
    """BGR → (NV12 字节串, scale, dx, dy)。

    NV12 为一维 uint8（长度 h*w*3//2）；scale/dx/dy 用于把框坐标逆映射回原图。
    已实机验证：一维 uint8 NV12 数组正是 hobot_dnn 接受的输入布局。
    """
    import cv2

    H, W = config.YOLO_INPUT_H, config.YOLO_INPUT_W
    lb, scale, dx, dy = _letterbox(bgr, W, H, pad=127)
    h, w = lb.shape[:2]
    area = h * w
    i420 = cv2.cvtColor(lb, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
    y = i420[:area]
    uv_planar = i420[area:].reshape((2, area // 4))
    uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))
    nv12 = np.empty((area * 3 // 2,), dtype=np.uint8)
    nv12[:area] = y
    nv12[area:] = uv_packed
    return nv12, scale, dx, dy


def bgr_to_nv12(bgr: np.ndarray) -> np.ndarray:
    """BGR → NV12 字节串（一维 uint8，长度 h*w*3//2）。"""
    return bgr_to_nv12_with_meta(bgr)[0]


def remap_letterbox_box(
    box: tuple[float, float, float, float],
    scale: float,
    dx: int,
    dy: int,
    src_w: int,
    src_h: int,
) -> tuple[float, float, float, float]:
    """把 letterbox（640×640 填充空间）的 xyxy 框逆映射回原图坐标并裁剪。"""
    x1, y1, x2, y2 = box
    nx1 = max(0.0, min(float(src_w), (x1 - dx) / scale))
    ny1 = max(0.0, min(float(src_h), (y1 - dy) / scale))
    nx2 = max(0.0, min(float(src_w), (x2 - dx) / scale))
    ny2 = max(0.0, min(float(src_h), (y2 - dy) / scale))
    return (nx1, ny1, nx2, ny2)


# ---------------------------------------------------------------------------
# 推理封装（hobot_dnn API 已实机验证）
# ---------------------------------------------------------------------------
class Detector:
    def __init__(
        self,
        model_bin: str | None = None,
        conf_thresh: float | None = None,
        iou_thresh: float | None = None,
        *,
        manifest_path: str | Path | None = None,
        class_names: tuple[str, ...] | None = None,
        profile_timing: bool = False,
    ) -> None:
        self.model_bin = model_bin or str(config.MODEL_DIR / config.MODEL_BIN)
        self.manifest_path = (config.MODEL_DIR / "runtime-manifest.json"
                              if manifest_path is None else Path(manifest_path))
        self._expected_class_names = (tuple(str(n) for n in range(1, config.YOLO_CLASS_COUNT + 1))
                                      if class_names is None else tuple(str(n) for n in class_names))
        self.conf_thresh = config.YOLO_CONF_THRESHOLD if conf_thresh is None else float(conf_thresh)
        if not math.isfinite(self.conf_thresh) or not config.YOLO_CONF_THRESHOLD <= self.conf_thresh <= 1.0:
            raise ValueError("confidence threshold must be finite and at least 0.60, at most 1")
        self.iou_thresh = iou_thresh or config.YOLO_IOU_THRESHOLD
        self._model = None
        self._class_names: list[str] = []
        self._max_detections = 300
        self._reg_bins = 16
        self._input_width = 0
        self._input_height = 0
        self._output_schema: tuple[_OutputSchema, ...] = ()
        self._requests: queue.Queue = queue.Queue(maxsize=1)
        self._request_lock = threading.Lock()
        self._stop = threading.Event()
        self._worker: threading.Thread | None = None
        self._disabled = False
        self.last_error: str | None = None
        self._profile_timing = bool(profile_timing)
        self._timing_instance = uuid.uuid4().hex if self._profile_timing else None
        self._timing_sequence = 0
        self.last_diagnostics: dict = {}

    def _load_manifest(self) -> None:
        """加载并冻结完整 runtime schema；在推理前拒绝布局/顺序漂移。"""
        path = self.manifest_path
        try:
            doc = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"无法读取 runtime manifest {path}: {exc}") from exc
        if not isinstance(doc, dict):
            raise RuntimeError("runtime manifest 根节点必须是对象")

        class_names = doc.get("class_names")
        postprocess = doc.get("postprocess")
        input_doc = doc.get("input")
        output_names = doc.get("output_names")
        output_docs = doc.get("outputs")
        if not isinstance(class_names, list):
            raise RuntimeError("manifest class_names 必须是数组")
        if not isinstance(postprocess, dict) or not isinstance(input_doc, dict):
            raise RuntimeError("manifest 缺少 input 或 postprocess 对象")
        if not isinstance(output_names, list) or not isinstance(output_docs, list):
            raise RuntimeError("manifest output_names / outputs 必须是数组")

        self._class_names = [str(n) for n in class_names]
        if set(self._class_names) != set(self._expected_class_names):
            raise RuntimeError("manifest class_names 必须唯一映射到全部业务类别")
        try:
            self._max_detections = int(postprocess["max_detections"])
            self._reg_bins = int(postprocess["reg_bins"])
            self._input_width = int(input_doc["width"])
            self._input_height = int(input_doc["height"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"manifest 基础字段非法: {exc}") from exc
        if len(self._class_names) != len(self._expected_class_names):
            raise RuntimeError(
                f"manifest class_names 数量 {len(self._class_names)} != {len(self._expected_class_names)}"
            )
        if self._reg_bins <= 0 or self._max_detections <= 0:
            raise RuntimeError("manifest reg_bins / max_detections 必须为正数")
        if (self._input_width, self._input_height) != (
            config.YOLO_INPUT_W,
            config.YOLO_INPUT_H,
        ):
            raise RuntimeError(
                "manifest 输入尺寸与 config 不一致: "
                f"manifest={self._input_width}x{self._input_height}, "
                f"config={config.YOLO_INPUT_W}x{config.YOLO_INPUT_H}"
            )
        input_shape = input_doc.get("tensor_shape")
        expected_input_shape = [1, 3, self._input_height, self._input_width]
        if input_shape != expected_input_shape:
            raise RuntimeError(
                f"manifest input.tensor_shape={input_shape!r} != {expected_input_shape!r}"
            )
        if input_doc.get("format") != "NV12":
            raise RuntimeError(
                f"manifest input.format={input_doc.get('format')!r}，仅支持 NV12"
            )
        try:
            input_dtype = np.dtype(input_doc["tensor_dtype"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"manifest input.tensor_dtype 非法: {exc}") from exc
        if input_dtype != np.dtype(np.uint8):
            raise RuntimeError(
                f"manifest input.tensor_dtype={input_dtype}，仅支持 uint8 NV12"
            )

        if len(output_names) != 6 or len(output_docs) != 6:
            raise RuntimeError("manifest 必须定义 6 个有序 cls/box 输出")
        schema: list[_OutputSchema] = []
        for index, raw in enumerate(output_docs):
            if not isinstance(raw, dict):
                raise RuntimeError(f"manifest outputs[{index}] 必须是对象")
            name = raw.get("name")
            role = raw.get("role")
            layout = raw.get("layout")
            raw_shape = raw.get("tensor_shape")
            try:
                stride = int(raw["stride"])
                dtype = np.dtype(raw["tensor_dtype"])
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError(f"manifest outputs[{index}] schema 非法: {exc}") from exc
            if name != output_names[index]:
                raise RuntimeError(
                    f"manifest 输出顺序不一致: output_names[{index}]={output_names[index]!r}, "
                    f"outputs[{index}].name={name!r}"
                )
            if layout != "NHWC":
                raise RuntimeError(f"manifest outputs[{index}] layout={layout!r}，仅支持 NHWC")
            if dtype != np.dtype(np.float32):
                raise RuntimeError(
                    f"manifest outputs[{index}] dtype={dtype}，仅支持 float32"
                )
            if (
                not isinstance(raw_shape, list)
                or len(raw_shape) != 4
                or any(isinstance(value, bool) or not isinstance(value, int) for value in raw_shape)
            ):
                raise RuntimeError(
                    f"manifest outputs[{index}].tensor_shape 必须是四维整数数组"
                )
            shape = tuple(raw_shape)
            expected_role = "CLASS_LOGITS" if index % 2 == 0 else "DFL_BOX_LOGITS"
            expected_channels = (
                len(self._expected_class_names) if index % 2 == 0 else 4 * self._reg_bins
            )
            if role != expected_role:
                raise RuntimeError(
                    f"manifest outputs[{index}] role={role!r}，预期 {expected_role}"
                )
            if stride <= 0 or self._input_width % stride or self._input_height % stride:
                raise RuntimeError(
                    f"manifest outputs[{index}] stride={stride} 与输入尺寸不整除"
                )
            expected_shape = (
                1,
                self._input_height // stride,
                self._input_width // stride,
                expected_channels,
            )
            if shape != expected_shape:
                raise RuntimeError(
                    f"manifest outputs[{index}] shape={shape} != {expected_shape}"
                )
            schema.append(_OutputSchema(str(name), str(role), stride, dtype, shape))

        for index in range(0, len(schema), 2):
            cls_spec, box_spec = schema[index], schema[index + 1]
            if cls_spec.stride != box_spec.stride or cls_spec.shape[:3] != box_spec.shape[:3]:
                raise RuntimeError(
                    f"manifest stride{cls_spec.stride} 的 cls/box 网格或 stride 不匹配"
                )
        manifest_strides = postprocess.get("strides")
        expected_strides = [spec.stride for spec in schema[::2]]
        if manifest_strides != expected_strides:
            raise RuntimeError(
                f"manifest postprocess.strides={manifest_strides!r} != {expected_strides!r}"
            )
        self._output_schema = tuple(schema)

    def _load_sync(self) -> None:
        from hobot_dnn import pyeasy_dnn as dnn  # type: ignore

        self._load_manifest()
        models = dnn.load(self.model_bin)
        if not models:
            raise RuntimeError("hobot_dnn did not load a model")
        self._model = models[0]

    def _work(self) -> None:
        try:
            while not self._stop.is_set():
                request = self._requests.get()
                if request is None:
                    break
                operation, bgr, ready, result = request
                timing = result.get("timing")
                if timing is not None:
                    timing["_execution_started_at"] = time.perf_counter()
                    timing["execution_status"] = "running"
                try:
                    if self._stop.is_set():
                        raise RuntimeError("detector is closed")
                    if timing is None:
                        result["value"] = self._load_sync() if operation == "load" else self._infer_sync(bgr)
                    else:
                        result["value"] = self._infer_sync(bgr, timing=timing)
                except Exception as exc:
                    result["error"] = exc
                    self.last_error = str(exc)
                    runlog.exception("detector_failed", f"detector {operation} failed", exc)
                finally:
                    if timing is not None:
                        timing["_execution_finished_at"] = time.perf_counter()
                        timing["execution_total_s"] = (timing["_execution_finished_at"]
                                                       - timing["_execution_started_at"])
                        timing["execution_status"] = "error" if "error" in result else "finished"
                    ready.set()
                    if timing is not None:
                        self._emit_timing("detector_execution_timing", timing,
                                          timing["execution_status"], result.get("error"))
        finally:
            # Only this worker owns and releases the native model.
            self._model = None

    def _request(self, operation: str, bgr, timeout_s: float, *, input_copy_s=None):
        if not self._request_lock.acquire(blocking=False):
            raise RuntimeError("detector is already processing a request")
        try:
            if operation == "infer":
                self.last_diagnostics = {}
            if self._disabled or self._stop.is_set():
                raise RuntimeError(self.last_error or "detector is closed")
            if self._worker is None:
                self._worker = threading.Thread(target=self._work, name="detector-infer", daemon=True)
                self._worker.start()
            ready = threading.Event()
            result: dict = {}
            timing = None
            if self._profile_timing and operation == "infer":
                self._timing_sequence += 1
                timing = result["timing"] = {
                    "detector_instance": self._timing_instance, "request_id": self._timing_sequence,
                    "operation": operation, "timeout_s": max(0.0, timeout_s),
                    "input_copy_s": input_copy_s,
                    "_requested_at": time.perf_counter(), "_stage_started_at": None,
                    "stage": "not_started", "preprocess_s": None, "forward_s": None,
                    "postprocess_s": None, "execution_total_s": None, "request_total_s": None,
                    "execution_status": "not_started", "caller_timed_out": False,
                }
            self._requests.put_nowait((operation, bgr, ready, result))
            if not ready.wait(max(0.0, timeout_s)):
                if timing is not None:
                    timing["caller_timed_out"] = True
                self._disabled = True
                self._stop.set()
                self.last_error = f"detector {operation} timed out after {timeout_s:.3f}s"
                runlog.emit("detector_timeout", self.last_error, **(
                    {"detector_instance": timing["detector_instance"], "request_id": timing["request_id"]}
                    if timing is not None else {}))
                if timing is not None:
                    timing["request_total_s"] = time.perf_counter() - timing["_requested_at"]
                    self._emit_timing("detector_request_timing", timing, "timeout", self.last_error)
                raise TimeoutError(self.last_error)
            if "error" in result:
                if timing is not None:
                    timing["request_total_s"] = time.perf_counter() - timing["_requested_at"]
                    self._emit_timing("detector_request_timing", timing, "error", result["error"])
                raise result["error"]
            self.last_error = None
            if timing is not None:
                self.last_diagnostics = timing.get("diagnostics", {}).copy()
                timing["request_total_s"] = time.perf_counter() - timing["_requested_at"]
                self._emit_timing("detector_request_timing", timing, "returned")
            return result.get("value")
        finally:
            self._request_lock.release()

    @staticmethod
    def _emit_timing(event, timing, status, error=None):
        snapshot = timing.copy()
        finished = snapshot.get("_execution_finished_at")
        now = time.perf_counter()
        stage_started = snapshot["_stage_started_at"]
        fields = {key: value for key, value in snapshot.items() if not key.startswith("_")}
        fields.update(status=status,
                      caller_total_s=(None if snapshot["request_total_s"] is None else
                                      snapshot["request_total_s"] + (snapshot["input_copy_s"] or 0.0)),
                      stage_elapsed_s=(None if stage_started is None
                                       else (finished if finished is not None else now) - stage_started),
                      completed_late=(None if finished is None else snapshot["caller_timed_out"]
                                      or finished - snapshot["_requested_at"] > snapshot["timeout_s"]))
        if error is not None:
            fields["error"] = str(error)
        runlog.emit(event, "检测请求与底层执行耗时", terminal=False, **fields)

    def load(self, timeout_s: float | None = None) -> None:
        timeout = getattr(config, "DETECTOR_LOAD_TIMEOUT_S", 15.0) if timeout_s is None else timeout_s
        self._request("load", None, timeout)

    def infer_bgr(self, bgr: np.ndarray, timeout_s: float | None = None) -> list[Detection]:
        timeout = getattr(config, "DETECTOR_INFER_TIMEOUT_S", 0.15) if timeout_s is None else timeout_s
        if not self._profile_timing:
            return self._request("infer", np.asarray(bgr).copy(), timeout)
        started = time.perf_counter()
        image = np.asarray(bgr).copy()
        return self._request("infer", image, timeout, input_copy_s=time.perf_counter() - started)

    def close(self) -> None:
        self._stop.set()
        try:
            self._requests.put_nowait(None)
        except queue.Full:
            pass  # A queued request observes _stop before touching the model.
        if self._worker is not None:
            self._worker.join(timeout=max(0.0, getattr(config, "IO_CLOSE_TIMEOUT_S", 0.2)))
            if self._worker.is_alive():
                runlog.emit("detector_close_pending", "inference worker is still in a native call")

    def _infer_sync(self, bgr: np.ndarray, timing=None) -> list[Detection]:
        if self._model is None:
            self._load_sync()
        if bgr.ndim != 3 or bgr.shape[2] != 3 or not bgr.shape[0] or not bgr.shape[1]:
            raise RuntimeError("inference requires a nonempty HWC BGR image")
        src_h, src_w = bgr.shape[:2]
        if timing is not None:
            timing["stage"], timing["_stage_started_at"] = "preprocess", time.perf_counter()
        nv12, scale, dx, dy = bgr_to_nv12_with_meta(bgr)
        if timing is not None:
            now = time.perf_counter()
            timing["preprocess_s"] = now - timing["_stage_started_at"]
            timing["stage"], timing["_stage_started_at"] = "forward", now
        raw_outputs = self._model.forward(nv12)
        if timing is not None:
            now = time.perf_counter()
            timing["forward_s"] = now - timing["_stage_started_at"]
            timing["stage"], timing["_stage_started_at"] = "postprocess", now
        # pyDNNTensor 用 .buffer 取 numpy 数组（已实机验证；np.asarray(t) 会得到坏的 scalar object）
        outputs: list[np.ndarray] = []
        for t in raw_outputs:
            if hasattr(t, "buffer"):
                outputs.append(np.asarray(t.buffer))
            else:
                raise RuntimeError(f"hobot_dnn 输出张量缺少 .buffer 属性: {type(t)}")
        self._validate_outputs(outputs)
        dets = postprocess_yolo(
            outputs,
            self.conf_thresh,
            self.iou_thresh,
            self._class_names,
            self._max_detections,
            self._reg_bins,
            tuple(spec.stride for spec in self._output_schema[::2]),
        )
        detections = [
            Detection(d.class_id, d.score, remap_letterbox_box(d.box, scale, dx, dy, src_w, src_h))
            for d in dets
        ]
        if timing is not None:
            highest_logit = highest_class_id = None
            heads = []
            threshold_logit = (math.inf if self.conf_thresh == 1.0 else
                               math.log(self.conf_thresh / (1.0 - self.conf_thresh)))
            for logits, spec in zip(outputs[::2], self._output_schema[::2]):
                best_index = int(np.argmax(logits))
                best_logit = float(logits.flat[best_index])
                heads.append({"stride": spec.stride, "logit_min": float(np.min(logits)),
                              "logit_max": best_logit,
                              "candidates_ge_threshold": int(np.count_nonzero(logits >= threshold_logit))})
                if highest_logit is None or best_logit > highest_logit:
                    highest_logit = best_logit
                    highest_class_id = int(self._class_names[best_index % logits.shape[-1]])
            highest_score = None
            if highest_logit is not None:
                if highest_logit >= 0:
                    highest_score = 1.0 / (1.0 + math.exp(-highest_logit))
                else:
                    exponential = math.exp(highest_logit)
                    highest_score = exponential / (1.0 + exponential)
            timing["diagnostics"] = {
                "detector_instance": timing["detector_instance"], "request_id": timing["request_id"],
                "confidence_threshold": self.conf_thresh, "input_shape": list(nv12.shape),
                "input_dtype": str(nv12.dtype), "highest_logit": highest_logit,
                "highest_score": highest_score, "highest_class_id": highest_class_id,
                "heads": heads,
                "candidates_ge_threshold": sum(head["candidates_ge_threshold"] for head in heads),
                "postprocess_detection_count": len(detections),
            }
            timing["postprocess_s"] = time.perf_counter() - timing["_stage_started_at"]
            timing["stage"], timing["_stage_started_at"] = "finished", None
        return detections

    def _validate_outputs(self, outputs: list[np.ndarray]) -> None:
        """严格比对运行时输出与 manifest 保存的完整有序 schema。"""
        if not self._output_schema:
            self._load_manifest()
        if len(outputs) != len(self._output_schema):
            raise RuntimeError(
                f"模型输出张量数 {len(outputs)} != {len(self._output_schema)}（预期 manifest cls/box 输出）"
            )
        for index, (tensor, spec) in enumerate(zip(outputs, self._output_schema, strict=True)):
            actual = np.asarray(tensor)
            if actual.ndim != 4:
                raise RuntimeError(
                    f"输出[{index}] {spec.name} rank={actual.ndim}，预期 rank=4 NHWC {spec.shape}"
                )
            if tuple(actual.shape) != spec.shape:
                raise RuntimeError(
                    f"输出[{index}] {spec.name} shape={tuple(actual.shape)} != {spec.shape} "
                    f"(role={spec.role}, stride={spec.stride})"
                )
            if actual.dtype != spec.dtype:
                raise RuntimeError(
                    f"输出[{index}] {spec.name} dtype={actual.dtype} != {spec.dtype}"
                )
            if not np.all(np.isfinite(actual)):
                raise RuntimeError(f"输出[{index}] {spec.name} contains non-finite logits")
