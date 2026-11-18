"""阶段二：实时障碍预览、直穿路口、掉头后返回路口刹停；不启用RFID。"""
from __future__ import annotations

import json
import math
import ctypes
import multiprocessing
import os
import select
import signal
import struct
import threading
import time
import zlib
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import config
import obstacle
import runlog


STAGE2_PAGE = r"""<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>阶段二障碍识别实时画面</title><style>
body{font-family:system-ui;margin:24px;background:#171a1e;color:#eee}
img{display:block;max-width:100%;height:auto} #state{margin:16px 0;white-space:pre-line}
.error{color:#ff9388} #last{max-width:640px} h2{font-size:18px}
</style><h1>阶段二 · 实时障碍识别</h1>
<div id="target-guide">识别前方颜色目标。关闭页面不会中断已开始的小车行驶。</div>
<div id="state">等待新画面…</div><img id="video" src="/stream.mjpg" alt="实时障碍框">
<h2>最后一次确认障碍</h2><div id="last-label">尚未确认障碍</div>
<img id="last" hidden alt="最后一次确认障碍图片">
<script>
const state=document.getElementById('state'), video=document.getElementById('video');
const last=document.getElementById('last'), lastLabel=document.getElementById('last-label');
let lastId=0;
const targetNames={color_obstacle:'颜色障碍',blue_red_sign:'蓝红禁行牌',orange_sign:'橙色施工牌',wood_fence:'木栅栏',blue_yellow_sign:'蓝黄警示牌',blue_patch:'蓝色色块',red_patch:'红色色块',warm_patch:'黄橙／棕色色块'};
const channelNames={blue_red:'蓝红并集',orange:'橙黄',brown:'木栏棕'};
video.onerror=()=>setTimeout(()=>{video.src='/stream.mjpg?t='+Date.now()},1000);
async function poll(){try{
  const r=await fetch('/status',{cache:'no-store',signal:AbortSignal.timeout(2000)});
  if(!r.ok)throw new Error('服务不可用');const s=await r.json();
  document.getElementById('target-guide').textContent=s.distance_profile?
    ({near:'近距离',far:'远距离',combined:'近远联合'}[s.distance_profile]+'统一面积区间；取蓝红并集、橙黄、木栏棕面积的最大值。0.5秒内最近3帧命中2帧后刹车，允许最大颜色组变化。关闭页面不会中断已开始的小车行驶。'):s.color_obstacle_enabled?
    '蓝、红、黄橙和棕色色块独立触发；蓝黄牌也触发。0.5秒内最近3帧同位置目标命中2帧后刹车。关闭页面不会中断已开始的小车行驶。':
    '检测前方蓝红禁行牌、橙色施工牌和木栅栏；当前模式蓝黄警示牌不触发停车。关闭页面不会中断已开始的小车行驶。';
  let target=s.confirmed?'障碍已确认':s.timed_out?'候选确认超时':s.candidate_active?'候选核验中':s.detected?'画面检测到目标':'未检测到障碍';
  state.className=s.fault_reason||!s.live?'error':'';
  state.textContent=s.action+'\n'+target+' · 新帧检测 '+s.fps.toFixed(1)+' FPS · 图龄 '+(s.age_s===null?'未知':s.age_s.toFixed(2)+' 秒')
    +(s.target_kind?'\n'+(targetNames[s.target_kind]||s.target_kind)+' · 实际色块 '+(100*(s.area_ratio||0)).toFixed(1)+'%'
      +(s.color_obstacle_enabled?' · 最近3帧命中2帧后刹车':s.slowdown_ratio!==null?' · 接近门槛 '+(100*s.slowdown_ratio).toFixed(0)+'% · 同目标连续两帧触底刹车':' · 本轮不触发停车'):'')
    +(s.distance_profile&&s.dominant_channel?'\n最大通道 '+channelNames[s.dominant_channel]+' · 判断面积 '+(100*s.judgment_area_ratio).toFixed(2)+'%'
      +(s.area_interval?' · 允许 '+(100*s.area_interval[0]).toFixed(2)+'%～'+(100*s.area_interval[1]).toFixed(2)+'%':''):'')
    +(s.approach_phase&&s.approach_phase!=='NORMAL'?'\n障碍接近：'+(s.approach_phase==='SLOWING'?'正在减速':'低速接近'):'')
    +(s.bottom_contact?' · 下边缘真实色块 '+s.bottom_hits+'/2帧':'')
    +(s.startup_reason?'\n视觉提示：'+s.startup_reason:'')
    +(s.fault_reason?'\n视觉故障：'+s.fault_reason:'')+(s.preview_error?'\n预览提示：'+s.preview_error:'');
  video.style.opacity=s.live?'1':'.3';
  if(s.last_obstacle_id&&s.last_obstacle_id!==lastId){lastId=s.last_obstacle_id;
    last.src='/last-obstacle.jpg?id='+lastId;last.hidden=false;lastLabel.textContent='确认时的原始画面与障碍框';}
  else if(!s.last_obstacle_id){lastId=0;last.hidden=true;lastLabel.textContent='尚未确认障碍';}
}catch(e){state.className='error';state.textContent='预览连接已中断，请检查RDK终端';video.style.opacity='.3';}
setTimeout(poll,400);}poll();
</script></html>""".encode("utf-8")


@dataclass(frozen=True)
class Stage2VisionSnapshot:
    candidate_active: bool
    confirmed: obstacle.ObstacleResult | None
    timed_out: bool
    fault_reason: str | None
    current: obstacle.ObstacleResult | None
    sequence: int
    captured_at: float | None
    candidate_started_sequence: int | None = None
    ready: bool = False
    startup_reason: str | None = None
    generation: int = 0
    observation_interrupted: bool = False
    observation_generation: int = 0
    last_successful_capture_at: float | None = None
    last_successful_sequence: int | None = None
    last_successful_source_generation: int | None = None
    availability_kind: str = "starting"
    runtime_stop_generation: int = 0
    runtime_stop_reason: str | None = None
    source_generation: int | None = None
    candidate_hits: int = 0
    approach_phase: str = "NORMAL"
    area_ratio: float | None = None
    approach_generation: int = 0
    target_kind: str | None = None
    stop_ratio: float | None = None
    slowdown_ratio: float | None = None
    clipped_stop_eligible: bool = False
    slowdown_hits: int = 0
    clipped_hits: int = 0
    bottom_contact: bool = False
    bottom_hits: int = 0
    soft_ready: bool = False
    last_reliable_capture_at: float | None = None
    recovery_hits: int = 0


STAGE2_OBSTACLE_HIT_STREAK = 2
STAGE2_OBSTACLE_VERIFY_TIMEOUT_S = 1.0
STAGE2_COLOR_VERIFY_WINDOW_S = .5
STAGE2_COLOR_MIN_COMPONENT_PIXELS = 500
STAGE2_RUNTIME_VISION_TIMEOUT_S = 2.0
STAGE2_OBSTACLE_SLOWDOWN_RATIOS = {
    "blue_red_sign": .12,
    "orange_sign": .10,
    "wood_fence": .19,
}
STAGE2_OBSTACLE_APPROACH_SPEED_MMPS = 91
STAGE2_UTURN_WHEEL_SPEED_MMPS = 91
STAGE2_TURN_STATIONARY_WAIT_S = 1.0


class _Stage2ObstacleCandidate:
    """Two consecutive distinct detections in the original fixed window."""

    def __init__(self):
        self.window_s = STAGE2_OBSTACLE_VERIFY_TIMEOUT_S
        self.started_at = self.last = self.last_frame = None
        self.hits = self.clears = 0

    @property
    def active(self):
        return self.started_at is not None

    def expired(self, now):
        return self.active and now - self.started_at >= STAGE2_OBSTACLE_VERIFY_TIMEOUT_S

    def update(self, result, frame_token, now, *, same_target=None):
        if not result.available:
            return "fault"
        if self.expired(now):
            return "timeout"
        if frame_token == self.last_frame:
            return "pending" if self.active else "idle"
        self.last_frame = frame_token
        if not result.detected:
            active = self.active
            self.started_at = self.last = None
            self.hits = self.clears = 0
            return "cleared" if active else "idle"
        first = not self.active
        if first:
            self.started_at = now
        same = obstacle.same_obstacle(self.last, result) if same_target is None else same_target
        self.hits = self.hits + 1 if same else 1
        self.last = result
        if self.hits >= STAGE2_OBSTACLE_HIT_STREAK:
            return "confirmed"
        return "first" if first else "pending"


class _Stage2ColorCandidate:
    """Two matching hits among the latest three valid source frames in 0.5 s."""

    def __init__(self):
        self.window_s = STAGE2_COLOR_VERIFY_WINDOW_S
        self.started_at = self.last = self.last_frame = None
        self.hits = self.clears = 0
        self._frames = deque(maxlen=3)

    @property
    def active(self):
        return self.started_at is not None

    def expired(self, now):
        return self.active and now - self.started_at >= self.window_s - 1e-9

    def _recount(self):
        hits = [(at, result) for at, result in self._frames if result is not None]
        self.hits = len(hits)
        self.clears = len(self._frames) - self.hits
        self.started_at = hits[0][0] if hits else None
        self.last = hits[-1][1] if hits else None

    def update(self, result, frame_token, now, *, same_target=None):
        if not result.available:
            return "fault"
        if frame_token == self.last_frame:
            return "pending" if self.active else "idle"
        self.last_frame = frame_token
        was_active = self.active
        while self._frames and now - self._frames[0][0] >= self.window_s - 1e-9:
            self._frames.popleft()
        self._recount()
        first = result.detected and not self.active
        if result.detected:
            same = obstacle.same_obstacle(self.last, result) if same_target is None else same_target
            if self.active and not same:
                self._frames.clear()
                first = True
            self._frames.append((now, result))
        else:
            self._frames.append((now, None))
        self._recount()
        if self.hits >= 2:
            return "confirmed"
        if first:
            return "first"
        if self.active:
            return "pending"
        return "cleared" if was_active else "idle"


@dataclass(frozen=True)
class _Stage2CapturedFrame:
    bgr: object
    sequence: int
    captured_monotonic: float
    source_generation: int


class _Stage2StartupRecoveryRequired(Exception):
    """Recovery invalidated the parked sample before the first wheel write."""


class _Stage2FrameSlot:
    """Two bounded raw slots; neither reader nor writer acquires a shared lock.

    An odd version denotes an unfinished write. A complete header, matching
    version before/after the copy and payload CRC reject interrupted/torn copies.
    A replacement owner gets new slots, so a retired process cannot publish into
    the new generation. Readers make two attempts and never wait for a publisher.
    """

    CAPACITY = 32 * 1024 * 1024
    HEADER = struct.Struct("<QdIIII")

    def __init__(self, context):
        self.buffers = [context.RawArray(ctypes.c_ubyte, self.CAPACITY + self.HEADER.size)
                        for _ in range(2)]
        self.versions = context.RawArray(ctypes.c_uint64, 2)
        self.latest = context.RawValue(ctypes.c_int, -1)
        self._read_cache = None
        self.rejection = 0

    def publish(self, frame):
        import numpy as np
        image = np.asarray(frame.bgr)
        self.rejection = 0
        if (image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3
                or not all(image.shape)):
            self.rejection = 3
            return False
        if image.nbytes > self.CAPACITY:
            self.rejection = 4
            return False
        if (isinstance(frame.sequence, bool) or not isinstance(frame.sequence, int)
                or frame.sequence < 0 or frame.sequence >= 2 ** 64
                or not Stage2Vision._fresh(frame.captured_monotonic)):
            self.rejection = 5
            return False
        payload = np.ascontiguousarray(image).tobytes()
        index = 1 - self.latest.value if self.latest.value >= 0 else 0
        version = int(self.versions[index])
        odd = version + 1 if not version % 2 else version + 2
        self.versions[index] = odd
        header = self.HEADER.pack(frame.sequence, frame.captured_monotonic,
                                  image.shape[0], image.shape[1], len(payload),
                                  zlib.crc32(payload))
        ctypes.memmove(ctypes.addressof(self.buffers[index]), header, len(header))
        ctypes.memmove(ctypes.addressof(self.buffers[index]) + self.HEADER.size,
                       payload, len(payload))
        self.versions[index] = odd + 1
        self.latest.value = index
        return True

    def read(self, generation):
        import numpy as np
        for _ in range(2):
            index = self.latest.value
            if index not in (0, 1):
                return None
            before = int(self.versions[index])
            if not before or before % 2:
                continue
            cached = self._read_cache
            if cached is not None and cached[:3] == (index, before, generation):
                return cached[3] if Stage2Vision._fresh(cached[3].captured_monotonic) else None
            address = ctypes.addressof(self.buffers[index])
            header = ctypes.string_at(address, self.HEADER.size)
            seq, captured, height, width, length, crc = self.HEADER.unpack(header)
            if not (0 < length <= self.CAPACITY and height > 0 and width > 0
                    and height * width * 3 == length):
                continue
            payload = ctypes.string_at(address + self.HEADER.size, length)
            if (before != int(self.versions[index]) or before % 2
                    or zlib.crc32(payload) != crc or not Stage2Vision._fresh(captured)):
                continue
            image = np.frombuffer(payload, dtype=np.uint8).reshape(height, width, 3).copy()
            frame = _Stage2CapturedFrame(image, seq, captured, generation)
            self._read_cache = (index, before, generation, frame)
            return frame
        return None


class _Stage2CameraStatus:
    """Bounded, single-writer diagnostics without child-owned locks or pipes."""

    CAPACITY = 8192

    def __init__(self, context):
        self.phase = context.RawValue(ctypes.c_int, 0)
        self.version = context.RawValue(ctypes.c_uint64, 0)
        self.length = context.RawValue(ctypes.c_int, 0)
        self.buffer = context.RawArray(ctypes.c_ubyte, self.CAPACITY)
        self._fault_count = 0
        self._fault_key = self._last_fault = None

    @property
    def value(self):
        return self.phase.value

    @value.setter
    def value(self, value):
        self.phase.value = value

    def publish(self, **fields):
        # A quick failed read followed by a successful frame must remain visible
        # to the supervisor even if it never polls the intermediate phase.
        phase = fields.get("phase")
        if phase in ("read-failed", "failed", "invalid-frame", "close-failed"):
            key = (phase, fields.get("error"))
            if key != self._fault_key:
                self._fault_count += 1
                self._last_fault = str(fields.get("error", phase))
            self._fault_key = key
        else:
            self._fault_key = None
        fields.update(fault_count=self._fault_count, fault_reason=self._last_fault)
        data = json.dumps(fields, ensure_ascii=False, default=str).encode("utf-8")
        if len(data) > self.CAPACITY:
            data = json.dumps({"phase": fields.get("phase"), "error": str(fields)[:1500],
                               "fault_count": self._fault_count,
                               "fault_reason": (self._last_fault or "")[:500]},
                              ensure_ascii=False).encode("utf-8")
        version = int(self.version.value)
        odd = version + 1 if not version % 2 else version + 2
        self.version.value = odd
        ctypes.memmove(ctypes.addressof(self.buffer), data, len(data))
        self.length.value = len(data)
        self.version.value = odd + 1

    def read(self):
        for _ in range(2):
            version = int(self.version.value)
            length = self.length.value
            if not version or version % 2 or not 0 < length <= self.CAPACITY:
                continue
            payload = ctypes.string_at(ctypes.addressof(self.buffer), length)
            if version != self.version.value:
                continue
            try:
                return json.loads(payload)
            except (UnicodeError, ValueError):
                continue
        return {}


@dataclass(frozen=True)
class _Stage2UsbCameraFactory:
    index: int = 0
    device: str | None = None

    def __call__(self):
        return _Stage2UsbCamera(self.index, self.device)


class _Stage2UsbCamera:
    """Stage 2 USB driver; the spawned owner calls open/read/release directly."""

    def __init__(self, index=0, device=None):
        self.index, self.device = index, device
        self.cap = None
        self.cv2 = None
        self.sequence = 0
        self.last_error = None

    @property
    def selection(self):
        return self.device if self.device is not None else self.index

    def prepare_driver(self):
        import cv2
        self.cv2 = cv2

    def open(self):
        if self.cv2 is None:
            self.prepare_driver()
        cv2 = self.cv2
        # V4L2 is explicit on the RDK. Desktop fakes remain injectable; no
        # fallback may silently select another physical camera.
        backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(self.selection, backend)
        if not self.cap.isOpened():
            raise RuntimeError(f"USB摄像头{self.selection!r}打开失败（backend={backend}）")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def grab_frame(self):
        ok, bgr = self.cap.read()
        captured_at = time.monotonic()
        if not ok or bgr is None:
            self.last_error = "USB摄像头read返回空帧"
            return None
        self.last_error = None
        self.sequence += 1
        return camera.CameraFrame(bgr, self.sequence, captured_at)

    def close(self):
        cap, self.cap = self.cap, None
        if cap is not None:
            cap.release()


class _Stage2VideoNodeMissing(RuntimeError):
    """A uniquely identified camera exists, but its video node is unavailable."""


class _Stage2UsbRecovery:
    """Resolve the selected UVC interface; never reset a hub or another camera."""

    def __init__(self, identity_path=None, *, sys_root="/sys", dev_root="/dev"):
        self.identity_path = Path(identity_path or config.APP_ROOT / "runtime" / "stage2_camera_identity.json")
        self.sys_root, self.dev_root = Path(sys_root), Path(dev_root)
        self.identity = None
        if self.identity_path.is_file():
            try:
                self.identity = self._validate(json.loads(self.identity_path.read_text(encoding="utf-8")))
            except (OSError, ValueError, TypeError) as exc:
                raise RuntimeError("相机身份文件无效，保持停车：" + str(exc)) from exc

    @staticmethod
    def _text(path):
        return path.read_text(encoding="ascii").strip()

    @staticmethod
    def _validate(value):
        if not isinstance(value, dict):
            raise ValueError("相机身份需要JSON对象")
        result = {key: value.get(key) for key in (
            "vid", "pid", "serial", "port", "interface", "video_index")}
        if (any(not isinstance(result[k], str) or not result[k] for k in
                ("vid", "pid", "port", "interface", "video_index"))
                or any(c not in "0123456789abcdef" for k in ("vid", "pid") for c in result[k])
                or any(len(result[k]) != 4 for k in ("vid", "pid"))
                or not result["video_index"].isdigit()
                or not isinstance(result["serial"], (str, type(None)))):
            raise ValueError("相机身份字段非法")
        # Sysfs names, rather than paths, are the only values allowed in driver writes.
        if (not all(c.isdigit() or c in "-." for c in result["port"])
                or not all(c.isdigit() or c in "-.:" for c in result["interface"])
                or not result["interface"].startswith(result["port"] + ":")):
            raise ValueError("相机USB端口或接口非法")
        return result

    def _usb_identity(self, usb, interface, *, video_index="0"):
        serial = usb / "serial"
        return self._validate(dict(vid=self._text(usb / "idVendor").lower(),
            pid=self._text(usb / "idProduct").lower(),
            serial=self._text(serial) if serial.is_file() else None,
            port=usb.name, interface=interface.name, video_index=video_index))

    def _matches(self, candidate):
        known = self.identity
        if known is None:
            return False
        if (candidate["vid"], candidate["pid"]) != (known["vid"], known["pid"]):
            return False
        if known["serial"]:
            return candidate["serial"] == known["serial"]
        return (candidate["port"], candidate["interface"]) == (known["port"], known["interface"])

    def _device(self, node):
        entry = self.sys_root / "class" / "video4linux" / node.name
        target = (entry / "device").resolve(strict=True)
        usb = next((p for p in (target, *target.parents)
                    if (p / "idVendor").is_file() and (p / "idProduct").is_file()), None)
        interface = next((p for p in (target, *target.parents)
                          if p.name.startswith((usb.name if usb else "") + ":")), None)
        if usb is None or interface is None:
            raise RuntimeError("所选视频节点无法唯一对应USB相机接口")
        driver = (interface / "driver").resolve(strict=True)
        if driver.name != "uvcvideo":
            raise RuntimeError("所选相机不属于UVC接口，不能执行USB恢复")
        index_file = entry / "index"
        identity = self._usb_identity(usb, interface,
                                     video_index=self._text(index_file) if index_file.is_file() else "0")
        return dict(identity=identity, usb=usb, interface=interface, node=node)

    def check_selection(self, selection):
        if self.identity is None:
            return
        try:
            selected = self._device(Path(selection).resolve())
        except (OSError, RuntimeError):
            selected = None
        if selected is not None and not self._matches(selected["identity"]):
            raise RuntimeError("显式视频节点与已保存相机身份不同，保持停车")

    def resolve(self, selection, *, enforce_selection=False):
        if self.identity is None:
            node = (self.dev_root / f"video{selection}" if isinstance(selection, int)
                    else Path(selection).resolve())
            target = self._device(node)
            self.identity = target["identity"]
        elif enforce_selection:
            self.check_selection(selection)
        target = self.resolve_usb()
        candidates = []
        video_root = self.sys_root / "class" / "video4linux"
        for entry in sorted(video_root.glob("video*")):
            try:
                candidate = self._device(self.dev_root / entry.name)
            except (OSError, RuntimeError, ValueError):
                continue
            if (self._matches(candidate["identity"])
                    and candidate["identity"]["interface"].split(":", 1)[-1]
                    == self.identity["interface"].split(":", 1)[-1]
                    and candidate["identity"]["video_index"] == self.identity["video_index"]):
                candidates.append(candidate)
        if len(candidates) > 1:
            raise RuntimeError("相机身份对应多个视频节点，保持停车")
        if candidates:
            target = candidates[0]
            target["node"] = str(target["node"])
        else:
            target["node"] = None
        return target

    def resolve_usb(self):
        if self.identity is None:
            raise RuntimeError("没有可验证的相机USB身份，保持停车")
        matches = []
        for usb in (self.sys_root / "bus" / "usb" / "devices").iterdir():
            if not (usb / "idVendor").is_file():
                continue
            interface_name = usb.name + ":" + self.identity["interface"].split(":", 1)[-1]
            interface = self.sys_root / "bus" / "usb" / "devices" / interface_name
            try:
                identity = self._usb_identity(usb, interface, video_index=self.identity["video_index"])
            except (OSError, ValueError):
                continue
            if self._matches(identity):
                matches.append(dict(identity=identity, usb=usb.resolve(),
                                    interface=interface.resolve(), node=None))
        if len(matches) != 1:
            raise RuntimeError("相机USB设备已消失" if not matches else "相机USB身份不唯一，保持停车")
        target = matches[0]
        usb_class = target["usb"] / "bDeviceClass"
        if usb_class.is_file() and self._text(usb_class).lower() == "09":
            raise RuntimeError("目标是USB集线器，拒绝恢复")
        interface_class = target["interface"] / "bInterfaceClass"
        if not interface_class.is_file() or self._text(interface_class).lower() != "0e":
            raise RuntimeError("目标USB接口不属于视频设备，拒绝恢复")
        return target

    def save(self):
        self.identity_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.identity_path.with_name(self.identity_path.name + f".{os.getpid()}.tmp")
        try:
            temporary.write_text(json.dumps(self.identity, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            os.replace(temporary, self.identity_path)
        finally:
            temporary.unlink(missing_ok=True)


def _stage2_usb_recovery_process(identity, action, status, authorized, *,
                                  sys_root="/sys", dev_root="/dev"):
    """Potentially blocking ioctl/sysfs writes are owned by this bounded child."""
    try:
        import fcntl
        resolver = _Stage2UsbRecovery.__new__(_Stage2UsbRecovery)
        resolver.sys_root, resolver.dev_root = Path(sys_root), Path(dev_root)
        resolver.identity = resolver._validate(identity)
        target = resolver.resolve_usb()
        if not authorized.value:
            raise RuntimeError("严格停稳授权已撤销")
        if action == "reset":
            usb = target["usb"]
            node = Path(dev_root) / "bus" / "usb" / f"{int(resolver._text(usb / 'busnum')):03}" / f"{int(resolver._text(usb / 'devnum')):03}"
            # USBDEVFS_RESET = _IO('U', 20); reset only the verified USB device.
            with node.open("wb", buffering=0) as handle:
                if not authorized.value:
                    raise RuntimeError("严格停稳授权已撤销")
                fcntl.ioctl(handle.fileno(), (ord("U") << 8) | 20, 0)
        elif action == "rebind":
            interface = target["interface"]
            driver = (interface / "driver").resolve(strict=True)
            if driver.name != "uvcvideo":
                raise RuntimeError("目标接口不是UVC，拒绝重新绑定")
            if not authorized.value:
                raise RuntimeError("严格停稳授权已撤销")
            (driver / "unbind").write_text(interface.name, encoding="ascii")
            # Restore the same interface even if shutdown arrived after unbind.
            (driver / "bind").write_text(interface.name, encoding="ascii")
        else:
            raise RuntimeError("未知USB恢复操作")
        status.publish(phase="usb-recovered", action=action, identity=target["identity"])
        status.value = 1
    except Exception as exc:
        status.publish(phase="usb-recovery-failed", action=action,
                       error=str(exc), error_type=type(exc).__name__)
        status.value = 2


def _stage2_capture_process(factory, initial, slot, status, stop):
    """Only this spawned process owns the camera and potentially blocking driver."""
    # The parent owns Ctrl-C and coordinates a bounded cooperative shutdown.
    # A signal delivered inside native cap.read() must not print a child traceback.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    current = None
    try:
        status.publish(phase="loading-driver", started_at=time.monotonic())
        current = initial if initial is not None else (factory or _Stage2UsbCameraFactory())()
        prepare_driver = getattr(current, "prepare_driver", None)
        if callable(prepare_driver):
            prepare_driver()
        status.value = 6
        status.publish(phase="opening", device=getattr(current, "selection", None),
                       started_at=time.monotonic())
        current.open()
        status.value = 1
        status.publish(phase="waiting-first-frame", device=getattr(current, "selection", None),
                       opened_at=time.monotonic())
        previous_sequence = previous_time = None
        while not stop.value:
            frame = current.grab_frame()
            if frame is not None:
                source = frame.captured_monotonic
                seq = frame.sequence
                now = time.monotonic()
                if (isinstance(source, bool) or not isinstance(source, (int, float))
                        or not math.isfinite(source) or source > now
                        or isinstance(seq, bool) or not isinstance(seq, int) or not 0 <= seq < 2 ** 64):
                    status.value = 5
                    status.publish(phase="invalid-frame", error="摄像头源时间或帧序号非法")
                    return
                if not Stage2Vision._fresh(source, now):
                    # Valid old acquisitions supply neither new evidence nor a
                    # driver error; the original one-second watchdog governs.
                    time.sleep(.005)
                    continue
                if (previous_sequence is None or seq > previous_sequence) and (
                        previous_time is None or isinstance(source, (int, float))
                        and not isinstance(source, bool) and source > previous_time):
                    if slot.publish(frame):
                        previous_sequence, previous_time = seq, source
                        status.publish(phase="capturing", device=getattr(current, "selection", None),
                                       sequence=seq, captured_at=source)
                    elif slot.rejection:
                        status.value = slot.rejection
                        status.publish(phase="invalid-frame", sequence=seq, error=f"slot rejection {slot.rejection}")
                        return
            elif getattr(current, "last_error", None):
                status.publish(phase="read-failed", device=getattr(current, "selection", None),
                               error=current.last_error)
            time.sleep(.005)
    except Exception as exc:
        import traceback
        status.publish(phase="failed", device=getattr(current, "selection", None),
                       error=str(exc), error_type=type(exc).__name__, traceback=traceback.format_exc())
        status.value = 2
    finally:
        if current is not None:
            try:
                current.close()
            except Exception as exc:
                status.publish(phase="close-failed", error=str(exc), error_type=type(exc).__name__)


class _Stage2CameraSource:
    """Supervise one spawned owner; driver stalls never block the control thread."""

    START_TIMEOUT_S = 10.0
    OPEN_TIMEOUT_S = 3.0
    FIRST_FRAME_TIMEOUT_S = 5.0
    FRAME_TIMEOUT_S = STAGE2_RUNTIME_VISION_TIMEOUT_S
    RETRY_INTERVAL_S = 1.0
    OWNER_JOIN_S = .5
    USB_COOLDOWN_S = 30.0
    USB_HELPER_TIMEOUT_S = 3.0
    USB_REENUMERATE_S = 2.0
    STATIONARY_LEASE_S = .5

    def __init__(self, camera_factory=None, initial_camera=None, *, context=None,
                 camera_index=0, camera_device=None, identity_path=None, usb_recovery=None):
        self._factory = camera_factory or _Stage2UsbCameraFactory(camera_index, camera_device)
        self._initial = initial_camera
        self.selection = camera_device if camera_device is not None else camera_index
        self._explicit_selection = camera_device is not None
        self._context = context or multiprocessing.get_context("spawn")
        self._lock = threading.RLock()
        self._stop, self._retry = threading.Event(), threading.Event()
        self._thread = self._process = None
        self._slot = self._status = self._owner_stop = None
        self._generation = 0
        self._error = "正在启动摄像头"
        self._opened = False
        self._armed = False
        self._status_emitter = None
        self._detail = {"phase": "starting", "device": self.selection}
        self._fault_generation = 0
        self._fault_reason = None
        self._usb = (usb_recovery if usb_recovery is not None else
                     _Stage2UsbRecovery(identity_path) if camera_factory is None
                     and initial_camera is None and sys.platform.startswith("linux") else None)
        self._usb_authorized = self._context.RawValue(ctypes.c_int, 0)
        self._stationary_at = float("-inf")
        self._stationary_barrier = None
        self._usb_active = False
        self._usb_helper = None
        self._usb_last_action_at = float("-inf")
        self._startup_failures = 0
        self._identity_saved = False
        self._usb_rebind_pending = False

    @property
    def diagnostics(self):
        # The supervisor replaces these published values; reading them must
        # never make UART control wait behind a USB helper's owner lock.
        return dict(self._detail, generation=self._generation, error=self._error)

    @property
    def generation(self):
        return self._generation

    @property
    def last_error(self):
        return self._error

    @property
    def fault_generation(self):
        return self._fault_generation

    @property
    def fault_reason(self):
        return self._fault_reason

    def start(self):
        with self._lock:
            if self._thread is None and not self._stop.is_set():
                self._thread = threading.Thread(target=self._run, name="stage2-camera-supervisor", daemon=True)
                self._thread.start()

    def open(self):
        self.start()

    def request_retry(self, reason):
        # Revoking a source is an atomic publication, not owner-process work.
        # This may be called by watchdog publication while USB recovery is busy.
        if not self._retry.is_set():
            self._fault_generation += 1
            self._fault_reason = str(reason)
        self._error = str(reason)
        self._opened = False
        self._retry.set()

    def arm_motion(self):
        # Permission revocation is monotonic and needs no owner lock. It must
        # also happen when that lock is busy immediately before a motor write.
        self._armed = True
        self._usb_authorized.value = 0
        self._stationary_at = float("-inf")

    def authorize_stationary(self, stationary, *, barrier=None):
        """Only the control thread may grant this after strict wheel stability."""
        if not stationary:
            self._usb_authorized.value = 0
            self._stationary_at = float("-inf")
            self._stationary_barrier = None
            return
        if not self._lock.acquire(timeout=.005):
            # A skipped parked grant never grants USB work while moving.
            return
        try:
            self._usb_authorized.value = int(bool(stationary) and not self._stop.is_set())
            self._stationary_at = time.monotonic() if stationary else float("-inf")
            self._stationary_barrier = barrier if stationary else None
        finally:
            self._lock.release()

    def _stationary_authorized(self):
        return bool(self._usb_authorized.value and not self._stop.is_set()
                    and 0 <= time.monotonic() - self._stationary_at <= self.STATIONARY_LEASE_S)

    @contextmanager
    def motion_write(self):
        if not self._lock.acquire(timeout=.005):
            raise sequence._CenteringProposalExpired("相机最终发令检查繁忙，重新计算目标")
        try:
            if self._usb_active:
                raise sequence._CenteringProposalExpired("相机USB恢复尚未结束，保持停车")
            yield
        finally:
            self._lock.release()

    def grab_frame(self):
        with self._lock:
            slot, generation = self._slot, self._generation
            available = self._opened and not self._retry.is_set() and not self._stop.is_set()
        if not available or slot is None:
            return None
        frame = slot.read(generation)
        with self._lock:
            return frame if slot is self._slot and generation == self._generation else None

    def _retire(self, owner):
        # Raw flag instead of multiprocessing.Event: killing a waiter must not
        # strand a child-owned synchronization lock in the parent.
        self._owner_stop.value = 1
        owner.join(self.OWNER_JOIN_S)
        if owner.is_alive():
            owner.terminate()
            owner.join(self.OWNER_JOIN_S)
        if owner.is_alive():
            owner.kill()
            owner.join(self.OWNER_JOIN_S)
        if owner.is_alive():
            runlog.emit("stage2_camera_owner_wait", "旧采集进程尚未退出，暂不重开设备",
                        source_generation=self.generation)
            return False
        owner.close()
        with self._lock:
            self._process = self._slot = None
        return True

    def _recover_usb(self):
        if self._usb is None or self._startup_failures < 2:
            return
        with self._lock:
            if (self._process is not None or not self._stationary_authorized()
                    or time.monotonic() - self._usb_last_action_at < self.USB_COOLDOWN_S):
                return
        try:
            if self._explicit_selection:
                self._usb.check_selection(self.selection)
            target = self._usb.resolve_usb()
        except Exception as exc:
            with self._lock:
                self._error = "USB恢复不可用，保持停车：" + str(exc)
                self._detail = dict(phase="usb-recovery-blocked", error=self._error)
            return
        with self._lock:
            if (self._process is not None or not self._stationary_authorized()
                    or time.monotonic() - self._usb_last_action_at < self.USB_COOLDOWN_S):
                return
            action = "rebind" if self._usb_rebind_pending else "reset"
            status = _Stage2CameraStatus(self._context)
            helper = self._context.Process(target=_stage2_usb_recovery_process,
                args=(target["identity"], action, status, self._usb_authorized),
                kwargs=dict(sys_root=str(self._usb.sys_root), dev_root=str(self._usb.dev_root)),
                name="stage2-usb-recovery", daemon=True)
            self._usb_active, self._usb_helper = True, helper
            self._usb_last_action_at = time.monotonic()
            self._error = "严格停稳后正在执行相机USB" + action
            self._detail = dict(phase="usb-recovering", action=action, identity=target["identity"])
        runlog.emit("stage2_camera_usb_recovery", "严格停稳且旧采集进程已退出，恢复所选相机",
                    action=action, identity=target["identity"], stable_barrier=self._stationary_barrier)
        try:
            helper.start()
            started = time.monotonic()
            while helper.is_alive() and not self._stop.wait(.01):
                with self._lock:
                    if not self._stationary_authorized():
                        self._usb_authorized.value = 0
                if time.monotonic() - started >= self.USB_HELPER_TIMEOUT_S:
                    break
            helper.join(.05)
            if helper.is_alive():
                helper.terminate()
                helper.join(self.OWNER_JOIN_S)
            if helper.is_alive():
                helper.kill()
                helper.join(self.OWNER_JOIN_S)
            detail = status.read()
            if helper.is_alive():
                raise RuntimeError("USB恢复进程尚未退出，保持停车")
            with self._lock:
                self._detail = detail or dict(phase="usb-recovery-timeout", action=action)
                self._error = None if status.value == 1 else "USB恢复未完成，保持停车：" + str(detail.get("error", "操作超时"))
                self._usb_rebind_pending = action == "reset" and status.value == 1
            until = time.monotonic() + self.USB_REENUMERATE_S
            while not self._stop.is_set() and time.monotonic() < until:
                try:
                    if self._usb.resolve(self.selection)["node"] is not None:
                        self._usb_rebind_pending = False
                        break
                except (OSError, RuntimeError, ValueError):
                    pass
                self._stop.wait(.05)
            runlog.emit("stage2_camera_usb_result", "相机USB恢复结束", action=action,
                        diagnostics=self._detail, rebind_pending=self._usb_rebind_pending)
        except Exception as exc:
            with self._lock:
                self._error = "USB恢复失败，保持停车：" + str(exc)
                self._detail = dict(phase="usb-recovery-failed", error=self._error)
            runlog.exception("stage2_camera_usb_failed", "所选相机USB恢复失败", exc)
        finally:
            if helper.pid is None or not helper.is_alive():
                helper.close()
                with self._lock:
                    self._usb_helper = None
                    self._usb_active = False

    def _selected_factory(self):
        if self._usb is None:
            return self._factory
        target = self._usb.resolve(self.selection, enforce_selection=self._explicit_selection)
        if target["node"] is None:
            raise _Stage2VideoNodeMissing("所选USB相机没有视频节点，保持停车等待恢复")
        return _Stage2UsbCameraFactory(device=target["node"])

    def _run(self):
        owner = None
        last_key = None
        try:
            while not self._stop.is_set():
                if owner is None:
                    self._recover_usb()
                    if self._stop.is_set():
                        break
                    if self._usb_active:
                        self._stop.wait(.05)
                        continue
                    try:
                        factory = self._selected_factory()
                    except Exception as exc:
                        if isinstance(exc, _Stage2VideoNodeMissing):
                            self._startup_failures += 1
                        else:
                            self._startup_failures = 0
                        with self._lock:
                            self._error = "相机身份或设备不可用，保持停车：" + str(exc)
                            self._detail = dict(phase="camera-identity-unavailable", error=self._error)
                        self._stop.wait(self.RETRY_INTERVAL_S)
                        continue
                    slot = _Stage2FrameSlot(self._context)
                    status = _Stage2CameraStatus(self._context)
                    stop = self._context.RawValue(ctypes.c_int, 0)
                    initial, self._initial = self._initial, None
                    owner = self._context.Process(target=_stage2_capture_process,
                        args=(factory, initial, slot, status, stop),
                        name="stage2-camera-owner", daemon=True)
                    with self._lock:
                        self._generation += 1
                        self._process, self._slot = owner, slot
                        self._status, self._owner_stop = status, stop
                        self._opened = False
                        self._retry.clear()
                        self._error = "正在打开摄像头"
                        self._detail = {"phase": "starting", "device": self.selection}
                    runlog.emit("stage2_camera_open_attempt", "后台尝试打开摄像头",
                                source_generation=self.generation, camera_device=self.selection)
                    try:
                        owner.start()
                    except Exception as exc:
                        owner.close()
                        with self._lock:
                            self._error = "摄像头进程启动失败：" + str(exc)
                            self._process = self._slot = None
                        owner = None
                        self._stop.wait(self.RETRY_INTERVAL_S)
                        continue
                    opened_at, opening_at, last_new = None, None, time.monotonic()
                    started_at, last_key = last_new, None
                    owner_fault_count = 0
                now = time.monotonic()
                detail = status.read()
                if detail:
                    with self._lock:
                        self._detail = detail
                        fault_count = int(detail.get("fault_count", 0))
                        if fault_count > owner_fault_count:
                            self._fault_generation += fault_count - owner_fault_count
                            self._fault_reason = str(detail.get("fault_reason", "摄像头读取失败"))
                        owner_fault_count = fault_count
                        if detail.get("phase") == "read-failed":
                            self._error = "摄像头读取失败：" + str(detail.get("error", "未知异常"))
                if status.value == 6 and opening_at is None:
                    opening_at = now
                if status.value == 1 and opened_at is None:
                    opened_at = now
                    last_new = now
                    with self._lock:
                        self._opened = True
                if opened_at is not None:
                    frame = slot.read(self.generation)
                    key = (frame.sequence, frame.captured_monotonic) if frame is not None else None
                    if key is not None and key != last_key:
                        last_key, last_new = key, frame.captured_monotonic
                        self._startup_failures = 0
                        self._usb_rebind_pending = False
                        if self._usb is not None and not self._identity_saved:
                            try:
                                self._usb.save()
                                self._identity_saved = True
                            except OSError as exc:
                                runlog.exception("stage2_camera_identity_save_failed", "相机身份保存失败", exc)
                        with self._lock:
                            self._error = None
                reason = ("摄像头采集失败：" + str(detail.get("error", "未知异常")) if status.value == 2 else
                          "摄像头图像格式非法，需要uint8三通道BGR" if status.value == 3 else
                          "摄像头单帧超过32MiB有界传图容量" if status.value == 4 else
                          "摄像头源时间或帧序号非法" if status.value == 5 else
                          "摄像头采集进程已结束" if not owner.is_alive() else
                          "摄像头采集进程启动超时" if opening_at is None and opened_at is None and now - started_at >= self.START_TIMEOUT_S else
                          "摄像头设备打开超时" if opened_at is None and opening_at is not None and now - opening_at >= self.OPEN_TIMEOUT_S else
                          "摄像头首帧等待超时" if opened_at is not None and last_key is None and now - opened_at >= self.FIRST_FRAME_TIMEOUT_S else
                          "摄像头持续没有新帧" if opened_at is not None and last_key is not None and now - last_new >= self.FRAME_TIMEOUT_S else None)
                if reason:
                    runlog.emit("stage2_camera_failed", reason, source_generation=self.generation,
                                camera_device=self.selection, camera_diagnostics=detail)
                    self.request_retry(reason)
                if self._retry.is_set():
                    if not self._retire(owner):
                        self._stop.wait(.05)
                        continue
                    if last_key is None:
                        self._startup_failures += 1
                    owner = None
                    self._stop.wait(self.RETRY_INTERVAL_S)
                else:
                    self._stop.wait(.01)
        except Exception as exc:
            with self._lock:
                self._error = "摄像头监督异常：" + str(exc)
            runlog.exception("stage2_camera_supervisor_failed", "摄像头监督异常", exc)
        finally:
            with self._lock:
                self._opened = False
            if owner is not None:
                self._retire(owner)

    def close(self):
        self._stop.set()
        self._usb_authorized.value = 0
        self._opened = False
        worker = self._thread
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=3 * self.OWNER_JOIN_S + .5)

STAGE2_COLOR_PROFILE = {
    # OpenCV H=0..179. Stage 2 alone owns this calibrated profile; ROI and
    # full-frame completion consume the same masks. Dark ranges require more
    # saturation so neutral shadows cannot become colored evidence.
    "blue": (((94, 65, 18), (140, 255, 255)), ((94, 100, 8), (140, 255, 17))),
    "yellow": (((8, 80, 18), (42, 255, 255)),),
    "brown": (((3, 50, 18), (30, 255, 255)), ((3, 100, 8), (30, 255, 17))),
    "red": (((0, 70, 18), (11, 255, 255)), ((167, 70, 18), (179, 255, 255))),
}

# Only the standalone color-trigger mode uses these photo-calibrated ranges
# and local color-support filters. Legacy/competition and camera settings stay
# independent; shaded low-saturation wood is genuine color evidence.
STAGE2_COLOR_MOTION_PROFILE = {
    "blue": (((94, 120, 18), (140, 255, 255)), ((94, 120, 8), (140, 255, 17))),
    "red": STAGE2_COLOR_PROFILE["red"],
    "yellow": (((8, 80, 18), (42, 255, 255)),),
    "brown": (((3, 100, 8), (45, 255, 255)),
              ((3, 20, 8), (45, 255, 120))),
}


def _stage2_color_motion_filter(raw_hsv, corrected_hsv, effective_masks, cv2, np):
    """Reject neutral gaps and dark print without erasing dark wooden paint."""
    hue, saturation, value = (corrected_hsv[:, :, channel] for channel in range(3))
    reference_scale = max(raw_hsv.shape[:2]) / float(config.OBSTACLE_REFERENCE_EDGE_PX)
    minimum_support = 50 * reference_scale ** 2
    # Slightly tinted white gaps are brighter than low-chroma shaded wood.
    # Keep both raw/adjusted color support, using the bounded correction only
    # to reject neutral evidence rather than filling missing painted pixels.
    brown_evidence = ((saturation >= 52)
                      | ((saturation >= 20) & (value <= 80)
                         & (raw_hsv[:, :, 1] >= 45))
                      | ((hue <= 12) & (saturation >= 40)
                         & (value >= 85) & (value <= 103)
                         & (raw_hsv[:, :, 1] >= 50)))
    surrounding = cv2.medianBlur(raw_hsv[:, :, 2], 31)
    effective_masks["brown"][~brown_evidence] = 0
    warm = effective_masks["yellow"] | effective_masks["brown"]
    structure = cv2.morphologyEx(warm, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(structure)
    paint = np.zeros_like(warm)
    for label in range(1, count):
        x, y, width, height, _ = stats[label]
        selected = ((labels[y:y + height, x:x + width] == label)
                    & (warm[y:y + height, x:x + width] != 0))
        area = np.count_nonzero(selected)
        if area < minimum_support:
            continue
        local_saturation = corrected_hsv[y:y + height, x:x + width, 1]
        if np.count_nonzero(selected & (local_saturation >= 80)) < max(minimum_support, area * .05):
            # A neutral rail/shadow may acquire an unstable brown hue. Shaded
            # wooden pixels need genuine chromatic support in their connected
            # color region; neutral gaps cannot serve as that support.
            neutral = labels[y:y + height, x:x + width] == label
            for color in ("yellow", "brown"):
                effective_masks[color][y:y + height, x:x + width][neutral] = 0
            continue
        # Solid saturated paint supports a local brightness reference. The
        # low-saturation, naturally shaded wooden surface keeps its true pixels.
        if (width > 30 and height > 30
                and np.median(corrected_hsv[y:y + height, x:x + width, 1][selected]) >= 180):
            # The reference must fit inside this color region's outer bounds;
            # a small isolated color patch cannot use a brighter wall to reject
            # itself, and the outside wall must not erase the painted border.
            paint[y + 15:y + height - 15, x + 15:x + width - 15] = 255
    dark_print = (paint != 0) & (raw_hsv[:, :, 2] < surrounding * .85)
    # Blue has an independent high-chroma range and no measured print leakage.
    # Tinted low-chroma print can cross the red hue boundary after correction.
    for color in ("yellow", "brown"):
        effective_masks[color][dark_print] = 0
    effective_masks["red"][dark_print & (saturation < 200)] = 0
    return {"paint_reference_saturation_min": 180,
            "print_reference_kernel_px": 31, "print_minimum_brightness_ratio": .85,
            "brown_normal_saturation_min": 52, "brown_shadow_saturation_min": 20,
            "brown_shadow_value_max": 80, "warm_chromatic_saturation_min": 80,
            "brown_shadow_raw_saturation_min": 45,
            "brown_mid_value_hue_max": 12, "brown_mid_value_saturation_min": 40,
            "brown_mid_value_min": 85, "brown_mid_value_max": 103,
            "brown_mid_value_raw_saturation_min": 50,
            "print_red_low_saturation_max_exclusive": 200,
            "warm_chromatic_support_min_pixels": minimum_support,
            "warm_chromatic_support_min_reference_pixels": 50,
            "warm_chromatic_support_min_fraction": .05}


def _stage2_color_masks(bgr, *, profile=None):
    """One local color pass; morphology never contributes area evidence."""
    import cv2
    import numpy as np

    if (bgr is None or getattr(bgr, "ndim", 0) != 3 or not bgr.size
            or bgr.shape[2] != 3 or bgr.dtype != np.uint8):
        raise ValueError("摄像头帧必须为非空 uint8 BGR 图像")
    original_height, original_width = bgr.shape[:2]
    max_edge = int(config.OBSTACLE_WORK_MAX_EDGE_PX)
    if max_edge <= 0 or config.OBSTACLE_REFERENCE_EDGE_PX <= 0:
        raise ValueError("障碍检测处理尺度配置无效")
    scale = min(1.0, max_edge / max(original_height, original_width))
    working = bgr if scale == 1.0 else cv2.resize(
        bgr, (max(1, round(original_width * scale)), max(1, round(original_height * scale))),
        interpolation=cv2.INTER_AREA)
    height, width = working.shape[:2]
    bounds = obstacle._roi_bounds(height, width)
    if bounds is None:
        raise ValueError("障碍检测 ROI 配置无效")
    corrected, lighting = obstacle._correct_lighting(working, bounds, cv2, np)
    raw_hsv = cv2.cvtColor(working, cv2.COLOR_BGR2HSV)
    corrected_hsv = cv2.cvtColor(corrected, cv2.COLOR_BGR2HSV)
    profile = STAGE2_COLOR_PROFILE if profile is None else profile
    reference_scale = max(height, width) / float(config.OBSTACLE_REFERENCE_EDGE_PX)
    kernel_size = max(1, round(config.OBSTACLE_MORPH_KERNEL_PX * reference_scale))
    kernel_size += int(kernel_size % 2 == 0)
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    x0, y0, x1, y1 = bounds
    minimum_area = max(config.OBSTACLE_MIN_COMPONENT_AREA_PX * reference_scale ** 2,
                       (x1 - x0) * (y1 - y0) * config.OBSTACLE_MIN_COMPONENT_AREA_RATIO)
    masks, effective_masks = {}, {}
    clipped = ((raw_hsv[:, :, 1] <= config.OBSTACLE_WB_MAX_SATURATION)
               & (raw_hsv[:, :, 2] > config.OBSTACLE_WB_MAX_VALUE))
    for color, ranges in profile.items():
        raw = np.zeros((height, width), np.uint8)
        normalized = np.zeros_like(raw)
        for lower, upper in ranges:
            raw |= cv2.inRange(raw_hsv, lower, upper)
            normalized |= cv2.inRange(corrected_hsv, lower, upper)
        if lighting.get("white_balance_applied"):
            raw[corrected_hsv[:, :, 1] < min(lower[1] for lower, _ in ranges)] = 0
        effective = raw | normalized
        effective[clipped] = 0
        effective_masks[color] = effective
    if profile is STAGE2_COLOR_MOTION_PROFILE:
        lighting.update(_stage2_color_motion_filter(raw_hsv, corrected_hsv,
                                                   effective_masks, cv2, np))
    for color, effective in effective_masks.items():
        structure = cv2.morphologyEx(effective, cv2.MORPH_OPEN, kernel)
        masks[color] = cv2.morphologyEx(structure, cv2.MORPH_CLOSE, kernel)
    return {"working_bgr": working, "corrected_bgr": corrected,
            "working_roi": bounds, "roi": obstacle._roi_bounds(original_height, original_width),
            "full_masks": masks, "effective_masks": effective_masks,
            "masks": {color: mask[y0:y1, x0:x1] for color, mask in masks.items()},
            "parameters": dict(lighting, reference_scale=reference_scale,
                               minimum_work_area_px=minimum_area,
                               morph_work_kernel_px=kernel_size, local_color_profile=profile),
            "raw_hsv": raw_hsv, "corrected_hsv": corrected_hsv}


def _stage2_effective_color_pixels(bgr, parameters, colors):
    """Apply the current frame's lighting estimates to actual source pixels."""
    import cv2
    import numpy as np
    raw_hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    corrected = np.clip(bgr.astype(np.float32) * np.asarray(
        parameters["white_balance_bgr_gains"], dtype=np.float32), 0, 255).astype(np.uint8)
    normalized = cv2.cvtColor(corrected, cv2.COLOR_BGR2HSV)
    lut = np.clip(255.0 * (np.minimum(np.arange(256) * parameters["brightness_gain"], 255.) / 255.)
                  ** parameters["gamma"], 0, 255).astype(np.uint8)
    normalized[:, :, 2] = cv2.LUT(normalized[:, :, 2], lut)
    target = np.zeros(bgr.shape[:2], np.uint8)
    for color in colors:
        ranges = parameters["local_color_profile"][color]
        raw, adjusted = np.zeros_like(target), np.zeros_like(target)
        for lower, upper in ranges:
            raw |= cv2.inRange(raw_hsv, lower, upper)
            adjusted |= cv2.inRange(normalized, lower, upper)
        if parameters["white_balance_applied"]:
            raw[normalized[:, :, 1] < min(lo[1] for lo, _ in ranges)] = 0
        target |= raw | adjusted
    target[(raw_hsv[:, :, 1] <= config.OBSTACLE_WB_MAX_SATURATION)
           & (raw_hsv[:, :, 2] > config.OBSTACLE_WB_MAX_VALUE)] = 0
    return target


def _stage2_roi_seed(detail, *, only_color=None):
    """Reuse geometric gates, with local-profile evidence for every seed."""
    import cv2

    working = detail["working_bgr"]
    original_x0, original_y0, original_x1, original_y1 = detail["roi"]
    x0, y0, x1, y1 = detail["working_roi"]
    scale_x = (x1 - x0) / (original_x1 - original_x0)
    scale_y = (y1 - y0) / (original_y1 - original_y0)
    reference_scale = detail["parameters"]["reference_scale"]
    minimum_area = detail["parameters"]["minimum_work_area_px"]
    minimum_width = config.OBSTACLE_MIN_WIDTH_PX * reference_scale
    minimum_height = config.OBSTACLE_MIN_HEIGHT_PX * reference_scale
    choices = []
    for color, mask in detail["masks"].items():
        if only_color is not None and color != only_color:
            continue
        fragments = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            px, py, pw, ph = cv2.boundingRect(contour)
            area = float(cv2.countNonZero(mask[py:py + ph, px:px + pw]))
            hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
            solidity = cv2.contourArea(contour) / hull_area if hull_area else 0.0
            if area / (pw * ph) < config.OBSTACLE_MIN_COLOR_FILL_RATIO or solidity < config.OBSTACLE_MIN_SOLIDITY:
                continue
            item = {"color": color, "bbox": (px, py, pw, ph), "area": area}
            if area >= minimum_area and pw >= minimum_width and ph >= minimum_height:
                choices.append(item)
            if color == "brown":
                plank = (ph >= 1.5 * minimum_height and pw >= .4 * minimum_width
                         and ph / pw >= config.OBSTACLE_FENCE_PLANK_MIN_ASPECT)
                beam = (pw >= 2 * minimum_width and ph >= .4 * minimum_height
                        and pw / ph >= config.OBSTACLE_FENCE_BEAM_MIN_ASPECT)
                if (area >= max(minimum_area * config.OBSTACLE_FENCE_FRAGMENT_MIN_AREA_RATIO,
                                config.OBSTACLE_FENCE_FRAGMENT_MIN_AREA_PX * reference_scale ** 2)
                        and area / (pw * ph) >= .35 and (plank or beam)):
                    fragments.append(dict(item, plank=plank))
        for box in obstacle._fence_groups(fragments):
            px, py, pw, ph = box
            area = float(cv2.countNonZero(mask[py:py + ph, px:px + pw]))
            if area >= minimum_area and pw >= minimum_width and ph >= minimum_height:
                choices.append({"color": color, "bbox": box, "area": area})
    if not choices:
        return obstacle.ObstacleResult(False, None, 0.0, None, None, True)
    # Orange and brown share hue; prefer the orange mask only for a solid
    # upright face. The later structural pass separates fence geometry.
    best = max(choices, key=lambda item: (item["area"], item["color"] == "brown"))
    px, py, pw, ph = best["bbox"]
    left, top = round((px + x0) / scale_x), round((py + y0) / scale_y)
    right, bottom = round((px + x0 + pw) / scale_x), round((py + y0 + ph) / scale_y)
    box = (left, top, right - left, bottom - top)
    return obstacle.ObstacleResult(True, best["color"], best["area"] / (scale_x * scale_y),
                                   box, obstacle._bbox_center(box), True)


def _stage2_link_clipped_fence_sections(components, fragments, labels, height,
                                       minimum_width, reference_scale):
    """Attach a broken beam only to its proven, bottom-clipped fence."""
    import cv2
    import numpy as np

    parents = [item for item in fragments if item["assembly"]
               and item["bbox"][1] >= .55 * height
               and item["bbox"][3] >= .25 * height
               and item["bbox"][1] + item["bbox"][3] == height]
    envelopes = {item["label"]: item["bbox"] for item in parents}
    parent_labels = set(envelopes)
    strip = max(1, round(3 * reference_scale))
    minimum_beam_run = max(1, round(3 * reference_scale))
    for item in components:
        if item["label"] in parent_labels:
            continue
        bx, by, bw, bh = item["bbox"]
        if by + bh != height or bh < .25 * height or bw < 2 * minimum_width:
            continue
        pixels = (labels[by:by + bh, bx:bx + bw] == item["label"]).astype(np.uint8)
        contours, _ = cv2.findContours(pixels, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = max(contours, key=cv2.contourArea)
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        if not hull_area or cv2.contourArea(contour) / hull_area >= .90:
            continue  # An adjacent solid board is not a missing fence section.
        for parent in parents:
            ax, ay, aw, ah = parent["bbox"]
            gap = max(0, max(ax, bx) - min(ax + aw, bx + bw))
            if (gap > .5 * minimum_width or abs(by - ay) > .1 * min(ah, bh)
                    or max(ah, bh) / min(ah, bh) > 1.2):
                continue
            if ax + aw <= bx:
                first = np.any(labels[:, ax + aw - strip:ax + aw] == parent["label"], axis=1)
                second = np.any(labels[:, bx:bx + strip] == item["label"], axis=1)
            elif bx + bw <= ax:
                first = np.any(labels[:, ax:ax + strip] == parent["label"], axis=1)
                second = np.any(labels[:, bx + bw - strip:bx + bw] == item["label"], axis=1)
            else:
                continue
            edges = np.diff(np.r_[False, first & second, False].astype(np.int8))
            runs = np.flatnonzero(edges == -1) - np.flatnonzero(edges == 1)
            if not len(runs) or max(runs) < minimum_beam_run:
                continue
            fragments[:] = [part for part in fragments if part["label"] != item["label"]]
            fragments.append(dict(item, plank=True, assembly=False, clipped_section=True,
                                  parent_fence=parent["label"]))
            envelopes[parent["label"]] = obstacle._union_box([envelopes[parent["label"]], item["bbox"]])
            break
    # The linked beam now proves a wider fence envelope. Its bottom-clipped
    # front-post caps can be separate color components but still belong to it.
    for item in components:
        if item["label"] in parent_labels:
            continue
        bx, by, bw, bh = item["bbox"]
        for parent in parents:
            ax, ay, aw, ah = envelopes[parent["label"]]
            if (by + bh == height and ax <= bx and bx + bw <= ax + aw
                    and .2 * ah <= bh < .6 * ah and minimum_width <= bw <= .3 * aw):
                fragments[:] = [part for part in fragments if part["label"] != item["label"]]
                fragments.append(dict(item, plank=True, assembly=False, clipped_cap=True,
                                      parent_fence=parent["label"]))
                break


def _stage2_complete_obstacle(bgr, *, diagnostics=None, prepared=None, selected_seed=None) -> obstacle.ObstacleResult:
    """Complete one road-ROI target; count only its actual colored pixels."""
    if diagnostics is not None:
        diagnostics.clear()
        diagnostics.update({"target_kind": None, "stop_eligible": False,
                            "target_color_area_px": 0.0, "edge_contacts": (),
                            "seed_source": "stage2_local_profile_roi", "completion_applied": False})
    try:
        import cv2
        import numpy as np

        detail = _stage2_color_masks(bgr) if prepared is None else prepared
        seed = _stage2_roi_seed(detail) if selected_seed is None else selected_seed
        if diagnostics is not None:
            diagnostics.update(detail)
            diagnostics["seed_result"] = seed
        working = detail["working_bgr"]
        height, width = working.shape[:2]
        original_height, original_width = bgr.shape[:2]
        scale_x, scale_y = width / original_width, height / original_height
        kernel_size = detail["parameters"]["morph_work_kernel_px"]
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)

        def full_mask(color):
            return detail["full_masks"][color]

        minimum_area = float(detail["parameters"]["minimum_work_area_px"])
        reference_scale = float(detail["parameters"]["reference_scale"])
        minimum_width = config.OBSTACLE_MIN_WIDTH_PX * reference_scale
        minimum_height = config.OBSTACLE_MIN_HEIGHT_PX * reference_scale
        x0, y0, x1, y1 = detail["working_roi"]
        if not seed.detected:
            dark_mask = full_mask("blue")
            gate = dark_mask[y0:y1, x0:x1]
            contours, _ = cv2.findContours(gate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            dark_candidates = []
            for contour in contours:
                px, py, pw, ph = cv2.boundingRect(contour)
                area = float(cv2.countNonZero(gate[py:py + ph, px:px + pw]))
                hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
                solidity = float(cv2.contourArea(contour)) / hull_area if hull_area else 0.0
                if (area >= minimum_area and pw >= minimum_width and ph >= minimum_height
                        and area / (pw * ph) >= 0.45 and solidity >= 0.75):
                    dark_candidates.append((area, (px + x0, py + y0, pw, ph)))
            if not dark_candidates:
                return seed
            area, (px, py, pw, ph) = max(dark_candidates)
            left, top = round(px / scale_x), round(py / scale_y)
            right, bottom = round((px + pw) / scale_x), round((py + ph) / scale_y)
            box = (left, top, right - left, bottom - top)
            seed = obstacle.ObstacleResult(True, "blue", area / (scale_x * scale_y),
                                           box, obstacle._bbox_center(box), True)
            detail["masks"]["blue"] = gate
            if diagnostics is not None:
                diagnostics.update({"seed_result": seed, "seed_source": "stage2_dark_blue_roi"})
        mask = full_mask(seed.color)

        x, y, seed_width, seed_height = seed.bbox
        left = max(x0, int(round(x * scale_x)))
        top = max(y0, int(round(y * scale_y)))
        right = min(x1, int(round((x + seed_width) * scale_x)))
        bottom = min(y1, int(round((y + seed_height) * scale_y)))
        # 精确颜色像素重合提供锚点；仅框重叠不能把另一个物体并入。
        gate_mask = detail["masks"][seed.color]
        anchor = np.zeros_like(mask)
        anchor[top:bottom, left:right] = gate_mask[top - y0:bottom - y0,
                                                   left - x0:right - x0]
        component_count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        anchored = np.bincount(labels[anchor != 0], minlength=component_count)
        components, fragments = [], []
        for index in range(1, component_count):
            px, py, pw, ph, area = (int(value) for value in stats[index])
            fill = area / (pw * ph)
            if fill < config.OBSTACLE_MIN_COLOR_FILL_RATIO:
                continue
            item = {"color": seed.color, "bbox": (px, py, pw, ph),
                    "area": float(area), "label": index,
                    "anchor_area": int(anchored[index])}
            if area >= minimum_area and pw >= minimum_width and ph >= minimum_height:
                components.append(item)
            if seed.color == "brown":
                fragment_minimum = max(
                    minimum_area * config.OBSTACLE_FENCE_FRAGMENT_MIN_AREA_RATIO,
                    config.OBSTACLE_FENCE_FRAGMENT_MIN_AREA_PX * reference_scale ** 2,
                )
                plank = (ph >= 1.5 * minimum_height and pw >= 0.4 * minimum_width
                         and ph / pw >= config.OBSTACLE_FENCE_PLANK_MIN_ASPECT)
                beam = (pw >= 2.0 * minimum_width and ph >= 0.4 * minimum_height
                        and pw / ph >= config.OBSTACLE_FENCE_BEAM_MIN_ASPECT)
                assembly = False
                if pw >= 2.0 * minimum_width and ph >= 1.5 * minimum_height:
                    pixels = labels[py:py + ph, px:px + pw] == index
                    vertical_core = cv2.morphologyEx(
                        pixels.astype(np.uint8), cv2.MORPH_OPEN,
                        np.ones((max(3, int(round(0.55 * ph))), 1), dtype=np.uint8),
                    )
                    vertical = np.count_nonzero(vertical_core, axis=0) >= 0.55 * ph
                    edges = np.diff(np.r_[False, vertical, False].astype(np.int8))
                    starts, stops = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
                    posts = sum(stop - start >= 0.4 * minimum_width
                                for start, stop in zip(starts, stops))
                    beam_rows = np.count_nonzero(pixels, axis=1) >= 0.65 * pw
                    # 宽连通体须独立显示多根纵条及横梁；实心牌不算栅栏整体。
                    assembly = posts >= 2 and np.count_nonzero(beam_rows) >= 0.4 * minimum_height
                    if not assembly and py + ph == height:
                        # In a close bottom-clipped fence both front and rear
                        # posts are shorter than half the visible box. Require
                        # three separate short posts plus real cross-beam rows.
                        short_length = max(3, int(round(.25 * ph)))
                        short_core = cv2.morphologyEx(pixels.astype(np.uint8), cv2.MORPH_OPEN,
                                                     np.ones((short_length, 1), np.uint8))
                        vertical = np.count_nonzero(short_core, axis=0) >= short_length
                        edges = np.diff(np.r_[False, vertical, False].astype(np.int8))
                        short_posts = sum(stop - start >= .4 * minimum_width for start, stop in
                                          zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))
                        assembly = short_posts >= 3 and np.count_nonzero(beam_rows) >= .4 * minimum_height
                if (area >= fragment_minimum and (fill >= .35 or assembly)
                        and (plank or beam or assembly)):
                    fragments.append(dict(item, plank=plank, assembly=assembly))

        if seed.color == "brown":
            _stage2_link_clipped_fence_sections(components, fragments, labels, height,
                                               minimum_width, reference_scale)
            # 暗端木条可能仍被纹理切碎。只为已证实多柱横梁的整体寻找
            # 紧邻且等高的端部，不让任意宽棕块获得组装能力。
            known_labels = {item["label"] for item in fragments}
            assemblies = [item for item in fragments if item["assembly"]]
            for item in components:
                if item["label"] in known_labels:
                    continue
                bx, by, bw, bh = item["bbox"]
                for assembly in assemblies:
                    ax, ay, aw, ah = assembly["bbox"]
                    # Front-post caps become wide trapezoids when their bodies
                    # leave the bottom edge. Only a proven enclosing fence can
                    # own these clipped pieces; unrelated boards remain out.
                    if (by + bh == height and ay + ah == height and ax <= bx
                            and bx + bw <= ax + aw and bh >= .2 * ah
                            and minimum_width <= bw <= .3 * aw):
                        fragments.append(dict(item, plank=True, assembly=False, clipped_cap=True))
                        break
                    if bh < 1.5 * minimum_height or bh / bw < 1.5:
                        continue
                    overlap = max(0, min(ay + ah, by + bh) - max(ay, by))
                    gap = max(0, max(ax, bx) - min(ax + aw, bx + bw))
                    if (max(ah, bh) / min(ah, bh) <= 1.5
                            and overlap / min(ah, bh) >= 0.75
                            and gap <= 0.5 * minimum_width):
                        fragments.append(dict(item, plank=True, assembly=False))
                        break

        def fence_compatible(first, second):
            if first["assembly"] or second["assembly"]:
                assembly, other = (first, second) if first["assembly"] else (second, first)
                if not other["plank"]:
                    return False
                ax, ay, aw, ah = assembly["bbox"]
                bx, by, bw, bh = other["bbox"]
                if other.get("parent_fence") is not None:
                    return other["parent_fence"] == assembly["label"]
                if other.get("clipped_cap"):
                    return ax <= bx and bx + bw <= ax + aw and ay <= by and by + bh == ay + ah
                overlap = max(0, min(ay + ah, by + bh) - max(ay, by))
                gap = max(0, max(ax, bx) - min(ax + aw, bx + bw))
                return (max(ah, bh) / min(ah, bh) <= config.OBSTACLE_FENCE_MAX_SIZE_RATIO
                        and overlap / min(ah, bh) >= config.OBSTACLE_FENCE_MIN_VERTICAL_OVERLAP
                        and gap <= bw * config.OBSTACLE_FENCE_MAX_GAP_WIDTH_RATIO)
            return obstacle._fence_fragments_compatible(first, second)

        choices = [dict(item, labels=[item["label"]]) for item in components
                   if item["anchor_area"] > 0]
        if seed.color == "brown":
            # Overexposed pale beams can split a very close fence into pointed
            # rear posts and bottom-clipped front caps. Recognize that two-row
            # geometry before regrouping the actual color components; no filled
            # beam pixels are invented and the group still needs ROI evidence.
            near = [item for item in components if item["bbox"][1] >= .55 * height]
            caps = [item for item in near if item["bbox"][1] + item["bbox"][3] == height]
            rear_posts = []
            for item in near:
                px, py, pw, ph = item["bbox"]
                if py + ph == height or not (1.1 <= ph / pw <= 5):
                    continue
                pixels = labels[py:py + ph, px:px + pw] == item["label"]
                row_widths = np.count_nonzero(pixels, axis=1)
                tip = max(1, round(.2 * ph))
                if np.mean(row_widths[:tip]) <= .8 * np.mean(row_widths[tip:]):
                    rear_posts.append(item)
            if len(caps) >= 2 and len(rear_posts) >= 2:
                rear_y = float(np.median([item["bbox"][1] for item in rear_posts]))
                members = [item for item in near if item in caps or abs(item["bbox"][1] - rear_y) <= .12 * height]
                box = obstacle._union_box([item["bbox"] for item in members])
                anchor_area = sum(item["anchor_area"] for item in members)
                if box[2] >= .55 * width and anchor_area:
                    choices.append({"bbox": box, "area": sum(item["area"] for item in members),
                                    "anchor_area": anchor_area, "fence_group": True,
                                    "labels": [item["label"] for item in members]})
        # 不相连木条只能通过高度、间隙与横梁关系成组；组必须含 ROI 种子。
        remaining = set(range(len(fragments)))
        while remaining:
            group = [remaining.pop()]
            pending = list(group)
            while pending:
                index = pending.pop()
                matches = [other for other in remaining
                           if fence_compatible(fragments[index], fragments[other])]
                remaining.difference_update(matches)
                group.extend(matches)
                pending.extend(matches)
            members = [fragments[index] for index in group]
            anchor_area = sum(item["anchor_area"] for item in members)
            if (len(members) < 2 or not anchor_area
                    or not any(item["plank"] for item in members)):
                continue
            box = obstacle._union_box([item["bbox"] for item in members])
            area = sum(item["area"] for item in members)
            if (area >= minimum_area and box[2] >= minimum_width and box[3] >= minimum_height
                    and area / (box[2] * box[3]) >= config.OBSTACLE_MIN_COLOR_FILL_RATIO):
                choices.append({"bbox": box, "area": area, "anchor_area": anchor_area,
                                "labels": [item["label"] for item in members]})
        if not choices:
            return seed
        # 优先覆盖原 ROI 中的颜色证据，避免大旁侧块凭全帧面积胜出。
        best = max(choices, key=lambda item: (item["anchor_area"], item["area"]))
        # Structure determines ownership; only the original valid color mask
        # contributes pixels. Opening/closing and filled white glyph holes do
        # not inflate the area used by the motion gate.
        ownership = np.isin(labels, best["labels"]).astype(np.uint8)
        ownership = cv2.dilate(ownership, kernel)
        target_mask = np.where(ownership != 0, detail["effective_masks"][seed.color], 0).astype(np.uint8)
        target_kind = None
        if seed.color == "blue":
            # A circular red patch inside a blue-tinted wall does not prove a
            # board. Require the blue component itself to show a painted planar
            # face before it can authorize stopping for a blue/red sign.
            ax, ay, aw, ah = best["bbox"]
            blue_structure = np.isin(labels[ay:ay + ah, ax:ax + aw], best["labels"]).astype(np.uint8)
            blue_contours, _ = cv2.findContours(blue_structure, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
            outer_fill = outer_solidity = 0.0
            corners = 0
            planar_face = False
            clipped_dense_face = False
            frame_clipped = ax == 0 or ay == 0 or ax + aw == width or ay + ah == height
            if blue_contours:
                outer = max(blue_contours, key=cv2.contourArea)
                contour_area = float(cv2.contourArea(outer))
                hull_area = float(cv2.contourArea(cv2.convexHull(outer)))
                outer_fill = contour_area / (aw * ah)
                outer_solidity = contour_area / hull_area if hull_area else 0.0
                polygon = cv2.approxPolyDP(outer, .02 * cv2.arcLength(outer, True), True)
                corners = len(polygon)
                # Intersecting a convex four-sided board with the camera frame
                # can yield three to eight visible corners. Do not require the
                # off-screen corners or fill the missing face into the mask.
                visible_corners = 3 <= corners <= 8 if frame_clipped else corners == 4
                planar_face = (outer_fill >= (.45 if frame_clipped else .80)
                               and outer_solidity >= .92 and visible_corners
                               and cv2.isContourConvex(polygon))
                # White glyphs can join a clipped image edge, opening holes in
                # the color contour. A dense clipped face still proves a board
                # without inventing those missing pixels in target_mask.
                clipped_dense_face = frame_clipped and outer_fill >= .80 and outer_solidity >= .80
                planar_face = planar_face or clipped_dense_face
            if diagnostics is not None:
                diagnostics["target_shape_evidence"] = {"outer_fill": outer_fill,
                                                       "outer_solidity": outer_solidity,
                                                       "corners": corners,
                                                       "frame_clipped": frame_clipped,
                                                       "clipped_dense_face": clipped_dense_face,
                                                       "planar_face": planar_face}
            # 蓝牌内红圈 / 红面须有真实红像素落在牌面内；外侧空心红框
            # 的包围盒包含蓝牌不能证明同一物体。
            red_mask = full_mask("red")
            red_count, red_labels, red_stats, _ = cv2.connectedComponentsWithStats(red_mask)
            blue_box = best["bbox"]
            ax, ay, aw, ah = blue_box
            compatible = []
            accepted_red = np.zeros_like(red_mask)
            for index in range(1, red_count):
                bx, by, bw, bh, red_area = (int(value) for value in red_stats[index])
                if (red_area < minimum_area or bw < minimum_width or bh < minimum_height
                        or red_area / (bw * bh) < config.OBSTACLE_MIN_COLOR_FILL_RATIO
                        or bw > 1.25 * aw or bh > 1.25 * ah
                        or not planar_face or bw < .15 * aw or bh < .10 * ah
                        or red_area < .015 * aw * ah):
                    continue
                pixels = (red_labels[by:by + bh, bx:bx + bw] == index).astype(np.uint8)
                contours, _ = cv2.findContours(pixels, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contour = max(contours, key=cv2.contourArea)
                hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
                solidity = float(cv2.contourArea(contour)) / hull_area if hull_area else 0.0
                if solidity < config.OBSTACLE_MIN_SOLIDITY:
                    continue
                perimeter = float(cv2.arcLength(contour, True))
                circularity = (4 * np.pi * cv2.contourArea(contour) / (perimeter * perimeter)
                               if perimeter else 0.0)
                if not (.6 <= bw / bh <= 1.5 and solidity >= .70 and circularity >= .55):
                    continue  # Wood-colored posts inside a blue wall are not a red symbol.
                intersection = (max(0, min(ax + aw, bx + bw) - max(ax, bx))
                                * max(0, min(ay + ah, by + bh) - max(ay, by)))
                red_inside_blue = int(np.count_nonzero(red_labels[ay:ay + ah, ax:ax + aw] == index))
                if (intersection / min(aw * ah, bw * bh) >= config.OBSTACLE_BLUE_RED_MIN_OVERLAP_RATIO
                        and red_inside_blue / red_area >= config.OBSTACLE_BLUE_RED_MIN_OVERLAP_RATIO):
                    compatible.append((bx, by, bw, bh))
                    accepted_red[red_labels == index] = 255
            if compatible:
                best["bbox"] = obstacle._union_box([blue_box, *compatible])
                target_mask |= cv2.dilate(accepted_red, kernel) & detail["effective_masks"]["red"]
                target_kind = "blue_red_sign"
            else:
                # Blue/yellow warning boards are displayed, but cannot obtain
                # any slowing or parking eligibility.
                yellow = detail["effective_masks"]["yellow"]
                inside_yellow = np.zeros_like(yellow)
                inside_yellow[ay:ay + ah, ax:ax + aw] = yellow[ay:ay + ah, ax:ax + aw]
                if cv2.countNonZero(inside_yellow) >= max(60 * reference_scale ** 2, .025 * aw * ah):
                    target_mask |= inside_yellow
                    target_kind = "blue_yellow_sign"
        elif seed.color in ("brown", "yellow"):
            px, py, pw, ph = best["bbox"]
            fill = cv2.countNonZero(target_mask[py:py + ph, px:px + pw]) / (pw * ph)
            # Lettering may occupy more than 30% of a painted board and can
            # mimic posts/beams. Its external contour still proves one planar
            # convex quadrilateral; fence gaps and pointed posts do not.
            owned_structure = np.isin(labels[py:py + ph, px:px + pw], best["labels"]).astype(np.uint8)
            face_contours, _ = cv2.findContours(owned_structure, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)
            outer_fill = outer_solidity = 0.0
            corners = 0
            planar_face = False
            if face_contours:
                face_contour = max(face_contours, key=cv2.contourArea)
                contour_area = float(cv2.contourArea(face_contour))
                hull_area = float(cv2.contourArea(cv2.convexHull(face_contour)))
                outer_fill = contour_area / (pw * ph)
                outer_solidity = contour_area / hull_area if hull_area else 0.0
                polygon = cv2.approxPolyDP(face_contour, .02 * cv2.arcLength(face_contour, True), True)
                corners = len(polygon)
                planar_face = (outer_fill >= .80 and outer_solidity >= .92
                               and corners == 4 and cv2.isContourConvex(polygon))
            if diagnostics is not None:
                diagnostics["target_shape_evidence"] = {"outer_fill": outer_fill,
                                                       "outer_solidity": outer_solidity,
                                                       "corners": corners,
                                                       "planar_face": planar_face}
            if .35 <= pw / ph <= 1.35 and (fill >= .72 or planar_face):
                target_kind = "orange_sign"
                # Black lettering retains some hue at very low luminance in
                # JPEGs. Compare V with the face's own lit color level so dark
                # words are excluded under global exposure changes as well.
                values = detail["raw_hsv"][:, :, 2]
                face_values = values[target_mask != 0]
                black_limit = .38 * float(np.percentile(face_values, 85)) if face_values.size else 255
                target_mask[values < black_limit] = 0
            elif seed.color == "brown" and (best.get("fence_group") or len(best["labels"]) >= 2 or any(
                    item["label"] in best["labels"] and item["assembly"] for item in fragments)):
                target_kind = "wood_fence"
        # Restore raw edge pixels removed during morphology, bounded to the
        # selected structure. This box therefore describes visible color.
        nonzero = cv2.findNonZero(target_mask)
        if nonzero is not None:
            best["bbox"] = cv2.boundingRect(nonzero)
        best["area"] = float(cv2.countNonZero(target_mask))
        x, y, box_width, box_height = best["bbox"]
        left, top = max(0, round(x / scale_x)), max(0, round(y / scale_y))
        right = min(original_width, round((x + box_width) / scale_x))
        bottom = min(original_height, round((y + box_height) / scale_y))
        box = (left, top, right - left, bottom - top)
        result_color = "yellow" if target_kind == "orange_sign" else seed.color
        result = obstacle.ObstacleResult(True, result_color, best["area"] / (scale_x * scale_y),
                                        box, obstacle._bbox_center(box), True)
        if diagnostics is not None:
            # Only selected target color supplies edge evidence. A single
            # actual pixel on the final row is sufficient; padded boxes are not.
            contacts = []
            for edge, band, axis, edge_length in (
                    ("left", target_mask[:, :1], 1, height),
                    ("right", target_mask[:, -1:], 1, height),
                    ("top", target_mask[:1, :], 0, width),
                    ("bottom", target_mask[-1:, :], 0, width)):
                if np.any(band != 0):
                    contacts.append(edge)
            bottom_contact = bool(np.any(target_mask[-1] != 0))
            if working is not bgr:
                # Downsampling can mix a white last row with color above it.
                # Check the actual full-frame row using this frame's calibrated
                # color profile, restricted to the selected ownership footprint.
                colors = (seed.color, "red") if target_kind == "blue_red_sign" else (seed.color,)
                raw_target = _stage2_effective_color_pixels(bgr[-1:], detail["parameters"], colors)
                footprint = cv2.resize((ownership[-1:] != 0).astype(np.uint8),
                                       (original_width, 1), interpolation=cv2.INTER_NEAREST)
                if target_kind == "blue_red_sign":
                    # Accepted red symbols belong inside the selected blue face.
                    footprint |= cv2.resize((target_mask[-1:] != 0).astype(np.uint8),
                                            (original_width, 1), interpolation=cv2.INTER_NEAREST)
                bottom_contact = bool(np.any((footprint != 0) & (raw_target != 0)))
                contacts = [edge for edge in contacts if edge != "bottom"]
                if bottom_contact:
                    contacts.append("bottom")
            diagnostics.update({"completion_applied": True, "completion_mask": target_mask,
                                "completion_work_bbox": best["bbox"],
                                "completion_components": len(best["labels"]),
                                "completion_candidates": choices,
                                "completion_fragments": fragments,
                                "completion_result": result,
                                "target_kind": target_kind,
                                "stop_eligible": target_kind in ("blue_red_sign", "orange_sign", "wood_fence"),
                                "target_color_area_px": result.area_px,
                                "bottom_contact": bottom_contact,
                                "edge_contacts": tuple(contacts)})
        return result
    except Exception as exc:
        return obstacle.ObstacleResult.unavailable(f"阶段二障碍补框异常: {exc}")


def detect_stage2_obstacle(bgr, *, diagnostics=None) -> obstacle.ObstacleResult:
    """Prefer verified target structure over a larger incidental color region."""
    try:
        detail = _stage2_color_masks(bgr)
        results = []
        for color in ("blue", "brown", "yellow", "red"):
            seed = _stage2_roi_seed(detail, only_color=color)
            if not seed.detected:
                continue
            candidate_detail = {}
            result = _stage2_complete_obstacle(bgr, diagnostics=candidate_detail,
                                              prepared=detail, selected_seed=seed)
            if not result.available:
                if diagnostics is not None:
                    diagnostics.clear()
                    diagnostics.update(candidate_detail)
                return result
            rank = (2 if candidate_detail.get("stop_eligible") else
                    1 if candidate_detail.get("target_kind") == "blue_yellow_sign" else 0)
            results.append((rank, result.area_px or 0.0, result, candidate_detail))
        if results:
            _, _, result, chosen = max(results, key=lambda item: (item[0], item[1]))
        else:
            result = obstacle.ObstacleResult(False, None, 0.0, None, None, True)
            chosen = dict(detail, target_kind=None, stop_eligible=False, target_color_area_px=0.0,
                          edge_contacts=(), seed_source="stage2_local_profile_roi", completion_applied=False)
        if diagnostics is not None:
            diagnostics.clear()
            diagnostics.update(chosen)
            diagnostics["structured_candidates"] = [
                {"kind": item[3].get("target_kind"), "stop_eligible": item[3].get("stop_eligible", False),
                 "area_px": item[2].area_px, "bbox": item[2].bbox} for item in results]
        return result
    except Exception as exc:
        if diagnostics is not None:
            diagnostics.clear()
        return obstacle.ObstacleResult.unavailable(f"阶段二颜色识别异常: {exc}")


def detect_stage2_color_obstacle(bgr, *, diagnostics=None) -> obstacle.ObstacleResult:
    """Independent road color patches; geometric continuity may cross colors."""
    try:
        import cv2
        import numpy as np

        detail = _stage2_color_masks(bgr, profile=STAGE2_COLOR_MOTION_PROFILE)
        height, width = detail["working_bgr"].shape[:2]
        original_height, original_width = bgr.shape[:2]
        scale_x, scale_y = width / original_width, height / original_height
        minimum = STAGE2_COLOR_MIN_COMPONENT_PIXELS * detail["parameters"]["reference_scale"] ** 2
        x0, y0, x1, y1 = detail["working_roi"]
        raw = {color: mask[y0:y1, x0:x1] for color, mask in detail["effective_masks"].items()}
        kernel = np.ones((3, 3), dtype=np.uint8)

        def components(mask):
            structure = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            count, labels, stats, _ = cv2.connectedComponentsWithStats(structure)
            areas = np.bincount(labels[mask != 0].ravel(), minlength=count)
            return labels, stats, areas

        choices = []
        for color in ("blue", "red", "yellow", "brown"):
            labels, stats, areas = components(raw[color])
            for label in range(1, len(stats)):
                if areas[label] >= minimum:
                    kind = color + "_patch" if color in ("blue", "red") else "warm_patch"
                    choices.append((int(areas[label]), kind, color,
                                    label, labels, raw[color]))

        mask = np.zeros((height, width), dtype=np.uint8)
        kind = None
        if choices:
            area, kind, color, label, labels, source = max(choices, key=lambda item: item[0])
            owned = (labels == label) & (source != 0)
            mask[y0:y1, x0:x1][owned] = 255
            x, y, w, h = cv2.boundingRect(cv2.findNonZero(mask))
            left, top = round(x / scale_x), round(y / scale_y)
            right, bottom = round((x + w) / scale_x), round((y + h) / scale_y)
            box = (left, top, right - left, bottom - top)
            result = obstacle.ObstacleResult(True, color, area / (scale_x * scale_y),
                                            box, obstacle._bbox_center(box), True)
        else:
            result = obstacle.ObstacleResult(False, None, 0.0, None, None, True)
        if diagnostics is not None:
            diagnostics.clear()
            diagnostics.update(detail)
            diagnostics.update(target_kind=kind, stop_eligible=result.detected,
                               target_color_area_px=result.area_px, completion_mask=mask,
                                minimum_work_area_px=minimum, bottom_contact=False, edge_contacts=(),
                               color_candidates=[{"area_work_px": item[0], "kind": item[1],
                                                  "color": item[2]} for item in choices])
        return result
    except Exception as exc:
        if diagnostics is not None:
            diagnostics.clear()
        return obstacle.ObstacleResult.unavailable(f"阶段二色块识别异常: {exc}")


def _stage2_annotate(bgr, result, fps, phase):
    """Annotate a private copy, always with the result from this exact image."""
    import cv2

    output = bgr.copy()
    if result.detected and result.bbox is not None:
        height, width = output.shape[:2]
        x, y, w, h = result.bbox
        left, top = max(0, min(width - 1, int(x))), max(0, min(height - 1, int(y)))
        right = max(0, min(width - 1, int(x + w)))
        bottom = max(0, min(height - 1, int(y + h)))
        if right > left and bottom > top:
            color = (70, 220, 90) if phase == "confirmed" else (0, 190, 255)
            cv2.rectangle(output, (left, top), (right, bottom), color, 2)
            cv2.putText(output, f"obstacle | {phase}",
                        (left, max(18, top - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, color, 2, cv2.LINE_AA)
    cv2.putText(output, f"Obstacle detection: {fps:.1f} FPS | {phase}", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return output


class _Stage2PreviewServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    allow_reuse_address = True
    request_queue_size = 8

    def __init__(self, address, vision):
        self.vision = vision
        self.slots = threading.BoundedSemaphore(8)
        # Long-lived video clients cannot consume the entire request budget.
        self.stream_slots = threading.BoundedSemaphore(4)
        super().__init__(address, _Stage2PreviewHandler)

    def process_request(self, request, client_address):
        request.settimeout(2.0)
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class _Stage2PreviewHandler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def _reply(self, data, content_type):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)
        self.wfile.flush()

    def _client_closed(self):
        connection = getattr(self, "connection", None)
        if connection is None:
            return False
        try:
            readable, _, _ = select.select([connection], [], [], 0)
            if not readable:
                return False
            timeout = connection.gettimeout()
            try:
                connection.setblocking(False)
                return not connection.recv(1, socket.MSG_PEEK)
            finally:
                connection.settimeout(timeout)
        except BlockingIOError:
            return False
        except OSError:
            return True

    def _stream(self, vision):
        if not self.server.stream_slots.acquire(blocking=False):
            self.send_error(503, "Too many preview streams")
            return
        try:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            sequence = -1
            while not vision._stop.is_set():
                if self._client_closed():
                    return
                item = vision._next_jpeg(sequence)
                if item is None:
                    continue
                sequence, jpeg, captured_at, generation = item
                with vision._condition:
                    vision._sync_source_generation_locked()
                    if generation != vision._visual_generation:
                        continue
                if not vision._fresh(captured_at):
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpeg)).encode("ascii") + b"\r\n\r\n" + jpeg + b"\r\n")
                self.wfile.flush()
                with vision._condition:
                    vision._sync_source_generation_locked()
                    if (generation == vision._visual_generation
                            and vision._source_generation_current_locked()
                            and vision._fresh(captured_at)):
                        vision._browser_received = True
                        vision._browser_captured_at = captured_at
                        vision._browser_generation = generation
        finally:
            self.server.stream_slots.release()

    def do_GET(self):
        vision = self.server.vision
        path = self.path.split("?", 1)[0]
        try:
            if path == "/":
                self._reply(STAGE2_PAGE, "text/html; charset=utf-8")
            elif path == "/status":
                self._reply(json.dumps(vision._status(), ensure_ascii=False).encode("utf-8"),
                            "application/json; charset=utf-8")
            elif path == "/last-obstacle.jpg":
                with vision._condition:
                    jpeg = vision._last_obstacle_jpeg
                if jpeg is None:
                    self.send_error(404, "No confirmed obstacle image")
                else:
                    self._reply(jpeg, "image/jpeg")
            elif path == "/stream.mjpg":
                self._stream(vision)
            else:
                self.send_error(404)
        except (OSError, TimeoutError):
            # A closed/slow browser affects only this bounded client worker.
            pass
        except Exception as exc:
            vision._preview_failed(f"HTTP预览异常：{exc}")


class Stage2Vision:
    """Independent color detection, latest-image JPEG encoding and bounded HTTP preview."""

    def __init__(self, camera, *, host="0.0.0.0", port=8080, frame_observer=None,
                 obstacle_approach=False, preview_enabled=True, distance_profile=None,
                 evidence_writer=None):
        self.camera, self.host, self.port = camera, host, port
        self.preview_enabled = bool(preview_enabled)
        self._frame_observer = frame_observer
        self.distance_profile = None
        self._distance_detector = None
        self._distance_metrics = {}
        self._evidence_writer = evidence_writer
        self._next_detection_diagnostic_at = 0.
        self._condition = threading.Condition()
        self._deferred_diagnostics = deque()
        self._stop = threading.Event()
        self._threads = []
        self._server = None
        self._http_thread = None
        self._started_at = None
        self._started = False
        self._closed = False
        self._epoch = None
        self.color_obstacle_enabled = False
        self._generation = 0
        self._epoch_since = time.monotonic()
        self._candidate = _Stage2ObstacleCandidate()
        self._clipped_candidate = _Stage2ObstacleCandidate()
        self._candidate_kind = self._clipped_candidate_kind = None
        self._slowdown_hits = self._approach_absent_hits = 0
        self._slowdown_last = self._approach_target = None
        self._slowdown_kind = self._approach_target_kind = None
        self._target_kind = self._stop_ratio = self._slowdown_ratio = None
        self._bottom_contact = False
        self._tracked_same_target = self._target_identity_uncertain = False
        self._clipped_stop_eligible = False
        self.obstacle_approach_enabled = bool(obstacle_approach)
        self._approach_phase = "NORMAL"
        self._approach_generation = 0
        self._approach_trigger_at = None
        self._approach_applied_barrier = None
        self._area_ratio = None
        self._candidate_started_sequence = None
        self._candidate_consumed_sequence = None
        self._candidate_restart_barrier = None
        self._confirmed = None
        self._timed_out = False
        self._fault_reason = None
        self._startup_reason = "等待摄像头新鲜画面"
        self._motion_sent = False
        self._status_emitter = None
        self._ready = False
        self._visual_generation = 0
        self._observation_generation = 0
        self._source_generation = None
        self._source_fault_generation = None
        self._last_successful_capture_at = None
        self._last_successful_sequence = None
        self._last_successful_source_generation = None
        self._last_reliable_capture_at = None
        self._soft_ready = False
        self._recovery_keys = deque(maxlen=2)
        self._recovery_barrier = self._epoch_since
        self._availability_kind = "starting"
        self._runtime_stop_generation = 0
        self._runtime_stop_reason = None
        self._runtime_outage_active = False
        self._active_fault_key = None
        self._timeout_capture_key = None
        self._preview_ack_generation = None
        self._ever_acknowledged = False
        self._preview_error = None
        self._current = None
        self._sequence = 0
        self._captured_at = None
        self._observed_at = None
        self._last_frame_sequence = None
        self._last_frame_time = None
        self._pending = None
        self._detect_pending = None
        self._raw_latest = None
        self._pending_confirmation = None
        self._jpeg = None
        self._jpeg_sequence = 0
        self._jpeg_captured_at = None
        self._jpeg_generation = None
        self._last_obstacle_jpeg = None
        self._last_obstacle_id = 0
        self._browser_received = False
        self._browser_captured_at = None
        self._browser_generation = None
        self._action = "等待摄像头画面与速度输入"
        self._completed = deque(maxlen=240)
        self._successful_evidence = deque(maxlen=16)
        self._fps = 0.0
        if distance_profile is not None:
            self.enable_distance_obstacles(distance_profile)

    @staticmethod
    def _fresh(captured_at, now=None):
        if (isinstance(captured_at, bool) or not isinstance(captured_at, (int, float))
                or not math.isfinite(captured_at)):
            return False
        age = (time.monotonic() if now is None else now) - captured_at
        return 0.0 <= age <= config.CAMERA_FRAME_MAX_AGE_S

    @contextmanager
    def control_read(self):
        """A busy publisher invalidates a proposal, never stalls UART polling."""
        if not self._condition.acquire(timeout=.005):
            raise sequence._CenteringProposalExpired("视觉发布繁忙，继续接收UART后重新计算目标")
        try:
            yield
        finally:
            self._condition.release()

    def _diagnostic_locked(self, event, message, **fields):
        self._deferred_diagnostics.append((event, message, fields))

    def _flush_diagnostics(self):
        # Only camera/detection workers call this outside publication locks.
        with self._condition:
            pending = tuple(self._deferred_diagnostics)
            self._deferred_diagnostics.clear()
        for event, message, fields in pending:
            runlog.emit(event, message, **fields)

    def start(self):
        with self._condition:
            if self._closed:
                raise RuntimeError("阶段二视觉已关闭")
            if self._started:
                return
            self._started = True
            self._started_at = time.monotonic()
            if self.preview_enabled:
                try:
                    self._server = _Stage2PreviewServer((self.host, self.port), self)
                    self.port = self._server.server_port
                except Exception as exc:
                    self._preview_failed("HTTP预览启动失败：" + str(exc))
        capture = threading.Thread(target=self._capture_loop, name="stage2-frame-ingest", daemon=True)
        detection = threading.Thread(target=self._detect_loop, name="stage2-detection", daemon=True)
        workers = [capture, detection]
        if self.preview_enabled:
            workers.append(threading.Thread(target=self._encode_loop, name="stage2-jpeg", daemon=True))
        if self._server is not None:
            self._http_thread = threading.Thread(target=self._serve, name="stage2-http", daemon=True)
            workers.append(self._http_thread)
        for worker in workers:
            self._threads.append(worker)
            try:
                worker.start()
            except Exception as exc:
                self._preview_failed(f"视觉后台线程启动失败（{worker.name}）：{exc}")
        opener = getattr(self.camera, "start", None)
        if callable(opener):
            try:
                opener()
            except Exception as exc:
                self._fault("摄像头监督启动失败：" + str(exc))
        elif callable(getattr(self.camera, "open", None)):
            def open_in_background():
                try:
                    self.camera.open()
                except Exception as exc:
                    self._fault("摄像头打开失败：" + str(exc))
            worker = threading.Thread(target=open_in_background, name="stage2-camera-open", daemon=True)
            self._threads.append(worker)
            try:
                worker.start()
            except Exception as exc:
                self._fault("摄像头后台打开线程启动失败：" + str(exc))

    def _serve(self):
        try:
            self._server.serve_forever(poll_interval=0.05)
        except Exception as exc:
            self._preview_failed(f"HTTP预览停止：{exc}")

    def set_epoch(self, epoch):
        if epoch not in (None, "outbound", "return"):
            raise ValueError("视觉运动阶段必须为 outbound、return 或 None")
        with self.control_read():
            if epoch == self._epoch:
                return
            self._epoch = epoch
            self._generation += 1
            self._epoch_since = time.monotonic()
            self._candidate = _Stage2ObstacleCandidate()
            self._candidate_started_sequence = None
            self._candidate_consumed_sequence = None
            self._candidate_restart_barrier = None
            self._confirmed = None
            self._timed_out = False
            self._reset_approach_locked()
            self._clear_unconfirmed_locked()

    def _reset_approach_locked(self, *, preserve_slowdown=False):
        changed = self._approach_phase != "NORMAL" or self._approach_trigger_at is not None
        self._approach_phase = "NORMAL"
        if changed:
            self._approach_generation += 1
        self._approach_trigger_at = self._approach_applied_barrier = None
        self._approach_absent_hits = 0
        self._approach_target = self._approach_target_kind = None
        if not preserve_slowdown:
            self._slowdown_hits = 0
            self._slowdown_last = self._slowdown_kind = None

    def enable_obstacle_approach(self, enabled=True):
        """Opt in at the obstacle entrypoint; recording/Stage 3 keep preview behavior."""
        with self._condition:
            self.obstacle_approach_enabled = bool(enabled)
            if enabled:
                self.color_obstacle_enabled = False
            self._discard_decision_locked()

    def enable_color_obstacles(self, enabled=True):
        """Dedicated standalone obstacle test; shared callers retain their defaults."""
        with self._condition:
            self.color_obstacle_enabled = bool(enabled)
            if enabled:
                self.obstacle_approach_enabled = False
            self._discard_decision_locked()

    def enable_distance_obstacles(self, profile="combined", *, evidence_writer=None):
        """Opt in without changing the recorder or standalone Stage 3 contract."""
        from stage2_distance import DistanceColorDetector
        detector = DistanceColorDetector(profile=profile)
        with self._condition:
            self.distance_profile = profile
            self._distance_detector = detector
            if evidence_writer is not None:
                self._evidence_writer = evidence_writer
            self.color_obstacle_enabled = True
            self.obstacle_approach_enabled = False
            self._discard_decision_locked()

    def _new_obstacle_candidate(self):
        return _Stage2ColorCandidate() if self.color_obstacle_enabled else _Stage2ObstacleCandidate()

    def mark_approach_applied(self, generation, applied_at, evidence_at):
        """Record slowing application; it never authorizes or delays BRAKE."""
        with self.control_read():
            if (not self.obstacle_approach_enabled or generation != self._approach_generation
                    or self._approach_phase != "SLOWING"
                    or any(isinstance(v, bool) or not isinstance(v, (int, float))
                           or not math.isfinite(v) for v in (applied_at, evidence_at))):
                return False
            self._approach_applied_barrier = max(applied_at, evidence_at, self._approach_trigger_at)
            self._approach_phase = "VERIFYING"
            return True

    @property
    def approach_active(self):
        with self.control_read():
            return self.obstacle_approach_enabled and self._approach_phase != "NORMAL"

    def suspend_approach_confirmation(self):
        """Discard pre-BRAKE observations while preserving the slow goal."""
        with self.control_read():
            if self.obstacle_approach_enabled and self._approach_phase != "NORMAL":
                self._approach_phase = "SLOWING"
                self._approach_generation += 1
                self._approach_applied_barrier = None
                self._clear_unconfirmed_locked()

    def try_suspend_approach_confirmation(self):
        if not self._condition.acquire(blocking=False):
            return False
        try:
            self.suspend_approach_confirmation()
            return True
        finally:
            self._condition.release()

    def set_action(self, text):
        with self.control_read():
            self._action = str(text)

    @property
    def preview_error(self):
        with self._condition:
            return self._preview_error

    @property
    def listener_ready(self):
        with self._condition:
            return (not self._closed and self._server is not None
                    and self._http_thread is not None and self._http_thread.is_alive())

    def _clear_unconfirmed_locked(self):
        if self._confirmed is None:
            self._candidate = self._new_obstacle_candidate()
            self._clipped_candidate = _Stage2ObstacleCandidate()
            self._candidate_kind = self._clipped_candidate_kind = None
            self._slowdown_hits = self._approach_absent_hits = 0
            self._slowdown_last = self._slowdown_kind = None
            self._candidate_started_sequence = self._candidate_consumed_sequence = None

    def _discard_decision_locked(self):
        self._confirmed = None
        self._pending_confirmation = None
        self._timed_out = False
        self._candidate_restart_barrier = None
        self._clear_unconfirmed_locked()
        self._reset_approach_locked()

    def discard_pending_decision(self):
        with self.control_read():
            self._discard_decision_locked()
            self._observation_generation += 1
            self._epoch_since = time.monotonic()

    def _latch_runtime_stop_locked(self, reason, key):
        if key == self._active_fault_key:
            return
        self._active_fault_key = key
        self._runtime_stop_generation += 1
        self._observation_generation += 1
        self._runtime_stop_reason = str(reason)
        self._runtime_outage_active = True
        self._soft_ready = False
        self._recovery_keys.clear()
        self._recovery_barrier = time.monotonic()
        # A committed stop survives vision recovery; only uncommitted evidence
        # is invalidated by a real stop event.
        self._clear_unconfirmed_locked()
        self._diagnostic_locked("stage2_vision_stop_requested", str(reason),
                    runtime_stop_generation=self._runtime_stop_generation,
                    source_generation=self._source_generation,
                    last_successful_capture_at=self._last_successful_capture_at,
                    last_successful_sequence=self._last_successful_sequence)

    def _runtime_timeout_locked(self, now):
        # Acquiring images and completing enabled recognition have independent
        # deadlines. Neither a new owner nor a retry renews these source clocks.
        if self._runtime_outage_active:
            return
        for family, source in (("capture", self._last_reliable_capture_at),
                               ("detection", self._last_successful_capture_at)):
            if source is None or now - source < STAGE2_RUNTIME_VISION_TIMEOUT_S:
                continue
            key = family, source
            if key != self._timeout_capture_key:
                self._timeout_capture_key = key
                self._latch_runtime_stop_locked(
                    "连续2秒没有有效新画面" if family == "capture" else "连续2秒没有新鲜成功障碍检测",
                    ("timeout", key))
                self._availability_kind = "fault"
            return

    def _expire_candidate_locked(self, now):
        if self.color_obstacle_enabled:
            # This rolling window belongs to completed frame source times.
            # Processing time and raw arrivals cannot expire queued evidence or
            # turn one unconfirmed color patch into a legacy timeout decision.
            return
        expired = [candidate for candidate in (self._candidate, self._clipped_candidate)
                   if candidate.expired(now)]
        if self._confirmed is None and expired:
            self._candidate_restart_barrier = max(candidate.started_at
                + candidate.window_s for candidate in expired)
            self._timed_out = True
            self._clear_unconfirmed_locked()

    def _sync_source_generation_locked(self):
        generation = getattr(self.camera, "generation", None)
        if generation is not None and generation != self._source_generation:
            previous = self._source_generation
            self._source_generation = generation
            self._visual_generation += 1
            self._observation_generation += 1
            self._ready = False
            self._recovery_keys.clear()
            self._availability_kind = "temporary" if previous is not None else "starting"
            self._startup_reason = "等待新摄像头代际的成功检测"
            self._current = None
            self._last_frame_sequence = self._last_frame_time = None
            self._observed_at = self._captured_at = None
            self._detect_pending = self._pending = self._raw_latest = None
            self._jpeg = self._jpeg_captured_at = self._jpeg_generation = None
            self._browser_received = False
            # Retain the last delivered source clock for the short recovery
            # grace. It is never current-generation preview evidence.
            self._browser_generation = None
            self._clear_unconfirmed_locked()

    def _source_generation_current_locked(self):
        return getattr(self.camera, "generation", None) == self._source_generation

    def _recover_startup(self, reason, *, retry_camera=False, kind="fault", event_key=None, **details):
        # A monotonic interruption counter lets the runner notice even a gap
        # which recovered between two polls; interrupted decisions need new evidence.
        with self._condition:
            self._sync_source_generation_locked()
            if kind in ("temporary", "starting") and self._availability_kind == "fault" and not self._soft_ready:
                kind = "fault"
                event_key = self._active_fault_key
            changed = self._startup_reason != str(reason) or self._ready
            self._ready = False
            self._availability_kind = kind
            self._startup_reason = str(reason)
            self._current = self._captured_at = None
            if kind == "fault":
                self._clear_unconfirmed_locked()
            # Driver/detector faults start background recovery, but only the
            # original two-second source deadlines authorize a visual stop.
            if kind == "fault" and self._last_successful_capture_at is not None and not self._runtime_outage_active:
                self._availability_kind = "temporary"
            if changed:
                self._diagnostic_locked("stage2_vision_unavailable", "视觉暂不可用：" + str(reason),
                            generation=self._visual_generation, **details)
            if retry_camera:
                retry = getattr(self.camera, "request_retry", None)
                if callable(retry):
                    retry(reason)
            self._condition.notify_all()

    def _fault(self, reason, **details):
        self._recover_startup(reason, **details)

    def mark_motion_sent(self):
        with self.control_read():
            self._motion_sent = True
            arm = getattr(self.camera, "arm_motion", None)
            if callable(arm):
                arm()

    @contextmanager
    def motion_write(self):
        # Serialize detector publication with the final short UART write. A
        # busy observer only invalidates this proposal; it creates no stop.
        if not self._condition.acquire(timeout=.005):
            raise sequence._CenteringProposalExpired("视觉最终发令检查繁忙，重新计算目标")
        try:
            # Source-time expiration is pure publication. Driver work and log
            # flushing stay with the workers outside the final write lock.
            self._sync_source_generation_locked()
            self._runtime_timeout_locked(time.monotonic())
            camera_guard = getattr(self.camera, "motion_write", None)
            if callable(camera_guard):
                with camera_guard():
                    yield self._snapshot_locked()
            else:
                yield self._snapshot_locked()
        finally:
            self._condition.release()

    def acknowledge_preview(self):
        with self._condition:
            self._watchdog_locked(time.monotonic())
            if not self._preview_ready_locked():
                return False
            self._preview_ack_generation = self._visual_generation
            self._ever_acknowledged = True
            return True

    @property
    def needs_reconfirmation(self):
        with self._condition:
            self._watchdog_locked(time.monotonic())
            return self._ever_acknowledged and (
                self._browser_captured_at is None
                or time.monotonic() - self._browser_captured_at >= STAGE2_RUNTIME_VISION_TIMEOUT_S)

    def _preview_failed(self, reason):
        with self._condition:
            self._preview_error = str(reason)
        runlog.emit("stage2_preview_failed", str(reason))

    def _watchdog_locked(self, now):
        if self._closed or self._started_at is None:
            return
        self._sync_source_generation_locked()
        self._runtime_timeout_locked(now)
        fault_generation = getattr(self.camera, "fault_generation", None)
        if fault_generation is not None and fault_generation != self._source_fault_generation:
            old_fault_generation = self._source_fault_generation
            self._source_fault_generation = fault_generation
            if fault_generation or old_fault_generation is not None:
                self._recover_startup("摄像头提示：" + str(getattr(self.camera, "fault_reason", None)
                                      or "摄像头读取失败"), event_key=("driver", fault_generation))
        reason = getattr(self.camera, "last_error", None)
        if reason:
            diagnostics = getattr(self.camera, "diagnostics", None)
            phase = diagnostics.get("phase") if isinstance(diagnostics, dict) else None
            starting = (phase in ("starting", "loading-driver", "opening", "waiting-first-frame")
                        and not fault_generation)
            self._recover_startup("摄像头提示：" + str(reason), kind="starting" if starting else "fault")
        elif self._observed_at is None or not self._fresh(self._observed_at, now):
            self._recover_startup(self._startup_reason or "摄像头没有新鲜帧",
                                  kind="temporary" if self._last_successful_capture_at is not None else "starting")
        elif self._captured_at is not None and not self._fresh(self._captured_at, now):
            self._recover_startup("障碍检测结果超龄", kind="temporary")
        if self._epoch is not None:
            self._expire_candidate_locked(now)

    def _snapshot_locked(self, *, candidate_pending=False):
        active = (self._candidate.active or self._clipped_candidate.active) and self._epoch is not None
        return Stage2VisionSnapshot(active, self._confirmed, self._timed_out, self._fault_reason,
                                    self._current, self._sequence, self._captured_at,
                                    self._candidate_started_sequence, self._ready,
                                    self._startup_reason, self._visual_generation, not self._ready,
                                    self._observation_generation, self._last_successful_capture_at,
                                    self._last_successful_sequence, self._last_successful_source_generation,
                                    self._availability_kind, self._runtime_stop_generation,
                                    self._runtime_stop_reason, self._source_generation,
                                    max(self._candidate.hits, self._clipped_candidate.hits),
                                    self._approach_phase, self._area_ratio, self._approach_generation,
                                    self._target_kind, self._stop_ratio, self._slowdown_ratio,
                                    self._clipped_stop_eligible, self._slowdown_hits,
                                    self._clipped_candidate.hits, self._bottom_contact,
                                    self._clipped_candidate.hits, self._soft_ready,
                                    self._last_reliable_capture_at, len(self._recovery_keys))

    def poll(self):
        """Read current UI/health state without consuming motion events."""
        with self.control_read():
            self._watchdog_locked(time.monotonic())
            return self._snapshot_locked()

    def consume_control_snapshot(self):
        """Expose current observations; candidates are display-only evidence."""
        with self.control_read():
            self._watchdog_locked(time.monotonic())
            snapshot = self._snapshot_locked()
            self._candidate_consumed_sequence = self._candidate_started_sequence
            return snapshot

    def try_control_snapshot(self):
        """Read published observations without delaying UART reception."""
        if not self._condition.acquire(blocking=False):
            return None
        try:
            # Runtime freshness is checked from these original source times.
            # Camera access and watchdog publication remain in the worker and
            # the authoritative final write guard, outside the receive hook.
            snapshot = self._snapshot_locked()
            self._candidate_consumed_sequence = self._candidate_started_sequence
            return snapshot
        finally:
            self._condition.release()

    def invalidate_control_candidates(self, barrier):
        """A new MCU session cannot reuse unconfirmed pre-parking frames."""
        with self.control_read():
            self._clear_unconfirmed_locked()
            self._candidate_restart_barrier = barrier
            self._epoch_since = max(self._epoch_since, barrier)
            self._observation_generation += 1
            if self._confirmed is None:
                self._pending_confirmation = None
                self._timed_out = False

    def try_mark_approach_applied(self, generation, applied_at, evidence_at):
        if not self._condition.acquire(blocking=False):
            return False
        try:
            return self.mark_approach_applied(generation, applied_at, evidence_at)
        finally:
            self._condition.release()

    def preview_ready(self):
        with self._condition:
            self._watchdog_locked(time.monotonic())
            return self._preview_ready_locked()

    def _preview_ready_locked(self):
        return (self.listener_ready and self._browser_received and self._jpeg is not None
                and self._jpeg_generation == self._visual_generation
                and self._browser_generation == self._visual_generation
                and self._fresh(self._jpeg_captured_at)
                and self._fresh(self._browser_captured_at)
                and self._source_generation_current_locked())

    def _status(self):
        snapshot = self.poll()
        with self._condition:
            age = None if self._jpeg_captured_at is None else max(0.0, time.monotonic() - self._jpeg_captured_at)
            live = (not self._closed and self._fresh(self._jpeg_captured_at)
                    and snapshot.fault_reason is None)
            current = snapshot.confirmed or snapshot.current
            return {"live": live, "fps": self._fps if live else 0.0, "age_s": age,
                    "epoch": self._epoch, "action": self._action,
                    "candidate_active": snapshot.candidate_active, "confirmed": snapshot.confirmed is not None,
                    "timed_out": snapshot.timed_out, "fault_reason": snapshot.fault_reason,
                    "startup_reason": snapshot.startup_reason, "generation": snapshot.generation,
                    "ready": snapshot.ready,
                    "listener_ready": self.listener_ready,
                    "browser_received": self._browser_received,
                    "observation_generation": snapshot.observation_generation,
                    "availability_kind": snapshot.availability_kind,
                    "runtime_stop_generation": snapshot.runtime_stop_generation,
                    "runtime_stop_reason": snapshot.runtime_stop_reason,
                    "last_successful_capture_at": snapshot.last_successful_capture_at,
                    "source_generation": snapshot.source_generation,
                    "candidate_hits": snapshot.candidate_hits,
                    "color_obstacle_enabled": self.color_obstacle_enabled,
                    "distance_profile": self.distance_profile,
                    "dominant_channel": self._distance_metrics.get("dominant_channel") if snapshot.current else None,
                    "judgment_area_ratio": self._distance_metrics.get("judgment_area_ratio") if snapshot.current else None,
                    "area_interval": self._distance_metrics.get("area_interval") if snapshot.current else None,
                    "perchannel": self._distance_metrics.get("perchannel") if snapshot.current else None,
                    "approach_phase": snapshot.approach_phase, "area_ratio": snapshot.area_ratio,
                    "target_kind": snapshot.target_kind, "stop_ratio": snapshot.stop_ratio,
                    "slowdown_ratio": snapshot.slowdown_ratio,
                    "clipped_stop_eligible": snapshot.clipped_stop_eligible,
                    "slowdown_hits": snapshot.slowdown_hits, "clipped_hits": snapshot.clipped_hits,
                    "bottom_contact": snapshot.bottom_contact, "bottom_hits": snapshot.bottom_hits,
                    "camera": getattr(self.camera, "diagnostics", None),
                    "captured_sequence": self._last_frame_sequence,
                    "jpeg_sequence": self._jpeg_sequence,
                    "preview_error": self._preview_error, "detected": bool(current and current.detected),
                    "color": current.color if current else None, "sequence": snapshot.sequence,
                    "last_obstacle_id": self._last_obstacle_id if self._last_obstacle_jpeg is not None else 0}

    def _capture_loop(self):
        while not self._stop.is_set():
            try:
                captured = self.camera.grab_frame()
                if captured is not None:
                    self._ingest_frame(captured)
                self.poll()
            except Exception as exc:
                self._fault("摄像头取帧异常：" + str(exc))
            self._flush_diagnostics()
            self._stop.wait(.005)

    def _ingest_frame(self, captured):
        now = time.monotonic()
        with self._condition:
            if self._closed:
                return None
            source_generation = getattr(captured, "source_generation", None)
            self._sync_source_generation_locked()
            if (source_generation is not None
                    and (source_generation != self._source_generation
                         or not self._source_generation_current_locked())):
                self._sync_source_generation_locked()
                return None
            # Observe an interruption against the previous acquisitions before
            # a new raw frame could conceal it; raw input cannot renew success.
            self._watchdog_locked(now)
            source, seq = captured.captured_monotonic, captured.sequence
            if not self._fresh(source, now):
                valid = (isinstance(source, (int, float)) and not isinstance(source, bool)
                         and math.isfinite(source) and source <= now)
                self._recover_startup("摄像头新帧超龄" if valid else "摄像头源时间非法",
                                      retry_camera=not valid, kind="temporary" if valid else "fault",
                                      frame_sequence=seq)
                return None
            if (isinstance(seq, bool) or not isinstance(seq, int) or seq < 0):
                self._recover_startup("摄像头帧序号非法", retry_camera=True)
                return None
            if ((self._last_frame_sequence is not None and seq <= self._last_frame_sequence)
                    or (self._last_frame_time is not None and source <= self._last_frame_time)):
                self._watchdog_locked(now)
                return None
            image = captured.bgr
            if (getattr(image, "ndim", None) != 3 or image.shape[2] != 3
                    or not all(image.shape) or str(image.dtype) != "uint8"):
                self._recover_startup("摄像头图像格式非法")
                return None
            if not self._source_generation_current_locked():
                self._sync_source_generation_locked()
                return None
            self._last_frame_sequence, self._last_frame_time = seq, source
            self._observed_at = source
            self._last_reliable_capture_at = source
            token = (self._generation, self._visual_generation, source_generation, now,
                     self._runtime_stop_generation)
            self._raw_latest = captured
            self._detect_pending = (captured, token)
            clear = obstacle.ObstacleResult(False, None, None, None, None, True)
            if self.preview_enabled and not self.color_obstacle_enabled:
                self._pending = (captured, clear, self._fps, "raw", self._visual_generation)
            self._condition.notify_all()
        if self._frame_observer is not None:
            observed = _Stage2CapturedFrame(image.copy(), seq, source, source_generation)
            try:
                self._frame_observer(observed)
            except Exception as exc:
                with self._condition:
                    self._latch_runtime_stop_locked("原始画面接收失败：" + str(exc),
                                                    ("frame-observer", source_generation, seq))
                runlog.exception("stage2_frame_observer_failed", "原始画面接收失败", exc)
        return token

    def _detect_loop(self):
        while not self._stop.is_set():
            with self._condition:
                self._condition.wait_for(lambda: self._stop.is_set() or self._detect_pending is not None,
                                         timeout=.1)
                if self._stop.is_set():
                    return
                pending, self._detect_pending = self._detect_pending, None
            if pending is None:
                continue
            captured, token = pending
            try:
                self._detect_frame(captured, token)
            except Exception as exc:
                self._fault("障碍检测故障：" + str(exc))

    def _process_frame(self, captured):
        """Synchronous helper; production uses separate ingestion and detection."""
        token = self._ingest_frame(captured)
        if token is not None:
            self._detect_frame(captured, token)

    def _detect_frame(self, captured, token):
        generation, visual_generation, source_generation, started, stop_generation = token
        detail = {}
        detector_started = time.perf_counter()
        try:
            if self._distance_detector is not None:
                with self._condition:
                    detecting_candidate = self._candidate
                    continuation = bool(self._epoch is not None and generation == self._generation
                        and visual_generation == self._visual_generation
                        and stop_generation == self._runtime_stop_generation
                        and source_generation == self._source_generation
                        and captured.captured_monotonic > self._epoch_since
                        and (self._candidate_restart_barrier is None
                             or captured.captured_monotonic >= self._candidate_restart_barrier)
                        and self._candidate.active
                        and not self._candidate.expired(captured.captured_monotonic)
                        and self._candidate.last is not None)
                result = self._distance_detector.detect(captured.bgr, diagnostics=detail,
                                                        continuation_active=continuation)
            else:
                result = (detect_stage2_color_obstacle(captured.bgr, diagnostics=detail)
                      if self.color_obstacle_enabled else
                      detect_stage2_obstacle(captured.bgr, diagnostics=detail)
                      if self.obstacle_approach_enabled else detect_stage2_obstacle(captured.bgr))
            if self.obstacle_approach_enabled and result.available:
                result = self._track_approach_target(captured, result, detail)
        except Exception as exc:
            with self._condition:
                self._sync_source_generation_locked()
                if (visual_generation == self._visual_generation
                        and (source_generation is None
                             or source_generation == getattr(self.camera, "generation", source_generation))):
                    self._fault("障碍检测故障：" + str(exc))
            return
        completed_at = time.monotonic()
        timing_due = completed_at >= self._next_detection_diagnostic_at
        detection_duration = time.perf_counter() - detector_started
        with self._condition:
            if timing_due:
                self._next_detection_diagnostic_at = completed_at + 1.
                self._diagnostic_locked("stage2_detection_timing", "检测耗时与源帧年龄",
                    source_sequence=captured.sequence,
                    source_generation=getattr(captured, "source_generation", source_generation),
                    source_age_s=completed_at - captured.captured_monotonic,
                    detection_duration_s=detection_duration,
                    detection_queue_s=max(0., completed_at - started - detection_duration),
                    ingest_age_s=started - captured.captured_monotonic,
                    distance_profile=self.distance_profile,
                    preview_enabled=self.preview_enabled)
            if self._closed:
                return
            self._sync_source_generation_locked()
            if (visual_generation != self._visual_generation
                    or source_generation is not None
                    and source_generation != getattr(self.camera, "generation", source_generation)):
                return
            # Check the old successful source before publishing a fresh result:
            # recovery cannot erase a stop that became due between polls.
            self._runtime_timeout_locked(completed_at)
            self._expire_candidate_locked(completed_at)
            self._watchdog_locked(completed_at)
            if (visual_generation != self._visual_generation
                    or not self._source_generation_current_locked()):
                self._sync_source_generation_locked()
                return
            if self._distance_detector is not None and detecting_candidate is not self._candidate:
                # A same-owner detector fault may clear a candidate while this
                # detection is running. Its old continuation allowance cannot
                # create the first hit of the replacement candidate.
                return
            continuous_observation = stop_generation == self._runtime_stop_generation
            if not self._fresh(captured.captured_monotonic, completed_at):
                self._recover_startup("障碍检测完成时图像已超龄", kind="temporary")
                return
            if not result.available:
                self._recover_startup(result.reason or "障碍检测不可用")
                return
            if getattr(self.camera, "last_error", None):
                self._watchdog_locked(completed_at)
                return
            self._ready, self._startup_reason = True, None
            self._availability_kind = "ready"
            self._active_fault_key = None
            self._current, self._sequence = result, captured.sequence
            self._set_obstacle_metrics_locked(captured, result, detail)
            self._captured_at = captured.captured_monotonic
            self._record_success_locked(captured)
            self._completed.append(completed_at)
            while len(self._completed) > 1 and self._completed[0] < completed_at - 2.:
                self._completed.popleft()
            elapsed = completed_at - self._completed[0]
            self._fps = (len(self._completed) - 1) / elapsed if elapsed > 0 else 0.
            phase = "preview"
            newly_confirmed = False
            if (self._epoch is not None and generation == self._generation
                    and captured.captured_monotonic > self._epoch_since
                    and (self._candidate_restart_barrier is None
                         or captured.captured_monotonic >= self._candidate_restart_barrier)
                    and continuous_observation
                    and self._confirmed is None):
                frame_key = captured.sequence, captured.captured_monotonic
                if self.obstacle_approach_enabled:
                    phase = self._update_approach_locked(result, frame_key)
                else:
                    same_target = (self._candidate.last is not None
                                   and self._candidate.last.detected and result.detected)
                    phase = self._candidate.update(result, frame_key, captured.captured_monotonic,
                        same_target=same_target if self._distance_detector is not None else None)
                if phase == "first":
                    self._candidate_started_sequence = captured.sequence
                    self._timed_out = False
                elif phase == "cleared":
                    self._candidate_started_sequence = None
                elif phase == "confirmed":
                    self._confirmed = result
                    newly_confirmed = True
                    if self.color_obstacle_enabled:
                        self._diagnostic_locked("stage2_color_obstacle_confirmed",
                                    ("最近0.5秒三帧内颜色面积命中两帧，立即请求BRAKE"
                                     if self._distance_detector is not None else
                                     "最近0.5秒三帧内同一色块命中两帧，立即请求BRAKE"),
                                    frame_sequence=captured.sequence,
                                    captured_at=captured.captured_monotonic,
                                    color=result.color, color_area_px=result.area_px)
                    if self.preview_enabled:
                        self._pending_confirmation = (captured, result, self._fps, phase, visual_generation)
                elif phase == "timeout":
                    self._timed_out = True
                    self._clear_unconfirmed_locked()
            elif self._confirmed is not None:
                phase = "confirmed" if result.detected else "clear-after-confirmation"
            elif self._timed_out:
                phase = "timeout"
            # Never replace a newer raw image with an old detector completion.
            if self.preview_enabled and (self.color_obstacle_enabled or self._raw_latest is captured):
                self._pending = (captured, result, self._fps, phase, visual_generation)
            evidence_epoch = (self._epoch, self._generation, self._visual_generation)
            save_evidence = self._confirmed is None or newly_confirmed
            self._condition.notify_all()
        self._flush_diagnostics()
        if self._evidence_writer is not None and save_evidence:
            try:
                self._evidence_writer.observe(captured, result, detail, phase, epoch=evidence_epoch)
            except Exception as exc:
                runlog.exception("stage2_obstacle_evidence_failed", "障碍证据入队失败", exc,
                                 source_sequence=captured.sequence)

    def _record_success_locked(self, captured):
        """Publish a genuine completed recognition, retaining source-time clocks."""
        self._last_successful_capture_at = captured.captured_monotonic
        self._last_successful_sequence = captured.sequence
        self._last_successful_source_generation = self._source_generation
        key = self._source_generation, captured.sequence, captured.captured_monotonic
        self._successful_evidence.append((*key, self._runtime_stop_generation))
        if captured.captured_monotonic > self._recovery_barrier and key not in self._recovery_keys:
            self._recovery_keys.append(key)
        if not self._soft_ready and len(self._recovery_keys) >= 2:
            self._soft_ready = True
            self._runtime_outage_active = False
            self._timeout_capture_key = None
            self._diagnostic_locked("stage2_vision_ready", "两组新鲜成功识别已恢复", generation=self._visual_generation)

    def _set_obstacle_metrics_locked(self, captured, result, detail):
        """One color/area/edge contract shared with the official entrypoint."""
        if self._distance_detector is not None:
            self._distance_metrics = {key: detail.get(key) for key in
                ("dominant_channel", "judgment_area_ratio", "area_interval", "perchannel")}
        height, width = captured.bgr.shape[:2]
        area = detail.get("target_color_area_px", result.area_px)
        self._area_ratio = (max(0., min(1., float(area) / (width * height)))
                            if result.detected and not isinstance(area, bool)
                            and isinstance(area, (int, float)) and math.isfinite(area)
                            and width > 0 and height > 0 else None)
        self._target_kind = detail.get("target_kind")
        self._slowdown_ratio = (STAGE2_OBSTACLE_SLOWDOWN_RATIOS.get(self._target_kind)
                                if not self.color_obstacle_enabled
                                and detail.get("stop_eligible") is True else None)
        # No amount of color alone authorizes a stop. Old fields stay available
        # for injected preview adapters, but carry no area-parking threshold.
        self._stop_ratio = None
        self._bottom_contact = (detail.get("bottom_contact") is True
                                if "bottom_contact" in detail
                                else "bottom" in detail.get("edge_contacts", ()))
        self._clipped_stop_eligible = False
        self._tracked_same_target = detail.get("tracked_same_target") is True
        self._target_identity_uncertain = detail.get("target_identity_uncertain") is True

    def _track_approach_target(self, captured, result, detail):
        """Follow an armed target's actual pixels after structural clipping.

        Initial qualification still requires a classified, adequately sized
        target. Tracking is restricted to its previous pixel footprint; an
        unrelated component elsewhere cannot inherit that qualification.
        """
        with self._condition:
            previous, kind = self._approach_target, self._approach_target_kind
            armed = self.obstacle_approach_enabled and self._approach_phase != "NORMAL"
        if not armed or previous is None or previous.bbox is None:
            return result
        detected_kind = detail.get("target_kind")
        if (detected_kind in (*STAGE2_OBSTACLE_SLOWDOWN_RATIOS, "blue_yellow_sign")
                and detected_kind != kind and result.detected):
            # A newly classified material is authoritative even when its HSV
            # ranges overlap the old target. Only missing/uncertain structure
            # may use the old footprint to recover a clipped target.
            return result
        if (detail.get("target_kind") == kind and result.detected
                and obstacle.same_obstacle(previous, result)):
            detail["tracked_same_target"] = True
            return result
        parameters = detail.get("parameters")
        if not parameters or previous.color not in STAGE2_COLOR_PROFILE:
            return result
        import cv2
        import numpy as np
        height, width = captured.bgr.shape[:2]
        x, y, box_width, box_height = previous.bbox
        left, top = max(0, x), max(0, y)
        right = min(width, x + box_width)
        # A close object can slide down and clip while its visible footprint
        # shrinks. Permit the existing continuity displacement below that
        # footprint, without claiming ownership of separate lateral objects.
        displacement = max(config.OBSTACLE_CONTINUITY_MIN_CENTER_SHIFT_PX,
                           math.hypot(box_width, box_height)
                           * config.OBSTACLE_CONTINUITY_MAX_CENTER_SHIFT_RATIO)
        bottom = min(height, y + box_height + math.ceil(displacement))
        if right <= left or bottom <= top:
            return result
        # Raw source pixels retain tiny remnants lost by processing-size
        # resampling or morphology. No width, height or area threshold applies.
        colors = (previous.color, "red") if kind == "blue_red_sign" else (previous.color,)
        owned = _stage2_effective_color_pixels(captured.bgr[top:bottom, left:right], parameters, colors)
        points = cv2.findNonZero(owned)
        if points is None:
            evidence = detail.get("effective_masks", {})
            if not result.detected and any(np.any(evidence.get(color, ())) for color in colors):
                detail["target_identity_uncertain"] = True
            return result
        px, py, pw, ph = cv2.boundingRect(points)
        box = (left + px, top + py, pw, ph)
        tracked = obstacle.ObstacleResult(True, previous.color, float(cv2.countNonZero(owned)),
                                          box, obstacle._bbox_center(box), True)
        detail.update(target_kind=kind, stop_eligible=True, tracked_same_target=True,
                      target_color_area_px=tracked.area_px,
                      bottom_contact=bottom == height and bool(np.any(owned[-1] != 0)),
                      tracked_pixel_bbox=box)
        return tracked

    def _update_approach_locked(self, result, frame_key):
        captured_at = frame_key[1]
        eligible = bool(result.detected and self._slowdown_ratio is not None
                        and self._area_ratio is not None)
        kind = self._target_kind
        slow_ready = eligible and self._area_ratio >= self._slowdown_ratio - 1e-9
        if kind != self._candidate_kind:
            self._candidate = _Stage2ObstacleCandidate()
            self._candidate_kind = kind
        slow_phase = self._candidate.update(replace(result, detected=slow_ready),
                                            frame_key, captured_at)
        self._slowdown_hits = self._candidate.hits
        if slow_ready:
            self._slowdown_last, self._slowdown_kind = result, kind
        else:
            self._slowdown_last = self._slowdown_kind = None

        armed_same = (self._approach_phase != "NORMAL" and eligible
                      and kind == self._approach_target_kind
                      and (self._tracked_same_target
                           or obstacle.same_obstacle(self._approach_target, result)))
        if self._approach_phase != "NORMAL":
            # A successfully detected but unclassified color patch is uncertain,
            # not proof that the armed target vanished. A known different target
            # or a clear successful frame does provide disappearance evidence.
            definitely_absent = (not self._target_identity_uncertain and (not result.detected or
                                 eligible and not armed_same or
                                 kind == "blue_yellow_sign"))
            self._approach_absent_hits = (self._approach_absent_hits + 1
                                         if definitely_absent else 0)
            if armed_same:
                self._approach_target = result
            if self._approach_absent_hits >= STAGE2_OBSTACLE_HIT_STREAK:
                self._reset_approach_locked(preserve_slowdown=True)
                if slow_phase != "confirmed":
                    self._diagnostic_locked("stage2_obstacle_approach_cleared",
                                "连续两张有效新画面明确目标消失，恢复原速度")

        # Each new material/object must independently qualify. A newly qualified
        # target can replace the previous one without an intermediate fast goal.
        if slow_phase == "confirmed" and not armed_same:
            self._approach_phase = "SLOWING"
            self._approach_generation += 1
            self._approach_trigger_at = captured_at
            self._approach_applied_barrier = None
            self._approach_target, self._approach_target_kind = result, kind
            self._approach_absent_hits = 0
            armed_same = True
            self._diagnostic_locked("stage2_obstacle_approach_started",
                        "两帧达到面积接近门槛，前进基准上限91mm/s",
                        target_kind=kind, area_ratio=self._area_ratio,
                        slowdown_ratio=self._slowdown_ratio)
        if armed_same:
            # Its original area qualification is committed. Continuing to see
            # that target cannot expire a completed slowdown candidate.
            self._candidate = _Stage2ObstacleCandidate()

        bottom_ready = bool(self._bottom_contact and (armed_same or slow_ready))
        self._clipped_stop_eligible = bottom_ready
        if kind != self._clipped_candidate_kind:
            self._clipped_candidate = _Stage2ObstacleCandidate()
            self._clipped_candidate_kind = kind
        bottom_phase = self._clipped_candidate.update(
            replace(result, detected=bottom_ready), frame_key, captured_at,
            same_target=True if bottom_ready and armed_same and self._tracked_same_target else None)
        if bottom_phase == "confirmed" and armed_same:
            self._diagnostic_locked("stage2_obstacle_stop_threshold",
                        "同一接近目标连续两帧真实颜色触底，BRAKE优先于减速确认",
                        target_kind=kind, area_ratio=self._area_ratio,
                        slowdown_ratio=self._slowdown_ratio, reason="bottom",
                        frame_sequence=frame_key[0], captured_at=captured_at)
            return "confirmed"
        if "timeout" in (slow_phase, bottom_phase):
            return "timeout"
        self._timed_out = False
        if "first" in (slow_phase, bottom_phase):
            return "first"
        if self._candidate.active or self._clipped_candidate.active:
            return "pending"
        return "slowing" if self._approach_phase != "NORMAL" else "distant" if eligible else "cleared"

    def _encode_loop(self):
        try:
            import cv2
        except Exception as exc:
            self._preview_failed(f"JPEG依赖不可用：{exc}")
            return
        while not self._stop.is_set():
            with self._condition:
                self._condition.wait_for(lambda: self._stop.is_set() or self._pending is not None
                                         or self._pending_confirmation is not None, timeout=0.2)
                if self._stop.is_set():
                    return
                historical = self._pending_confirmation is not None
                if historical:
                    item, self._pending_confirmation = self._pending_confirmation, None
                else:
                    item, self._pending = self._pending, None
            if item is None:
                continue
            captured, result, fps, phase, generation = item
            try:
                output = _stage2_annotate(captured.bgr, result, fps, phase)
                ok, encoded = cv2.imencode(".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    raise RuntimeError("无法编码JPEG画面")
                jpeg = encoded.tobytes()
                with self._condition:
                    if self._closed:
                        return
                    self._sync_source_generation_locked()
                    if generation != self._visual_generation:
                        continue
                    if not self._source_generation_current_locked():
                        self._sync_source_generation_locked()
                        continue
                    if historical:
                        self._last_obstacle_jpeg = jpeg
                        self._last_obstacle_id += 1
                    if (self._fresh(captured.captured_monotonic)
                            and (self._jpeg_captured_at is None
                                 or captured.captured_monotonic >= self._jpeg_captured_at)):
                        self._jpeg = jpeg
                        self._jpeg_sequence += 1
                        self._jpeg_captured_at = captured.captured_monotonic
                        self._jpeg_generation = generation
                        if self._preview_error and self._preview_error.startswith("JPEG"):
                            self._preview_error = None
                    self._condition.notify_all()
            except Exception as exc:
                self._preview_failed(f"JPEG预览编码失败：{exc}")
                # Preview failure does not stop independent on-board detection.

    def _next_jpeg(self, sequence):
        with self._condition:
            self._condition.wait_for(lambda: self._stop.is_set() or self._jpeg_sequence > sequence,
                                     timeout=0.2)
            if self._stop.is_set() or self._jpeg is None or self._jpeg_sequence <= sequence:
                return None
            return self._jpeg_sequence, self._jpeg, self._jpeg_captured_at, self._jpeg_generation

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if self._condition.acquire(timeout=.005):
            try:
                self._pending = self._detect_pending = self._raw_latest = None
                self._pending_confirmation = None
                self._condition.notify_all()
            finally:
                self._condition.release()
        timeout = max(0.0, getattr(config, "IO_CLOSE_TIMEOUT_S", 0.2))
        server = self._server
        if server is not None:
            if self._http_thread is not None and self._http_thread.is_alive():
                shutdown = threading.Thread(target=server.shutdown, name="stage2-http-close", daemon=True)
                shutdown.start()
                shutdown.join(timeout=timeout)
            server.server_close()
        for worker in self._threads:
            if worker.ident is not None and worker is not threading.current_thread():
                worker.join(timeout=timeout)
        # Camera lifecycle remains owned by the stage runner (one capture owner).


import math
import time
from collections import deque
from dataclasses import asdict, replace

import mcu
import runlog
import test_motion_sequence as sequence
from test_stage1_patrol import (
    PatrolCenteringController, MovingHeadingSampler, PatrolDisplay,
    PatrolJunctionDetector, PatrolJunctionEvidence, _check_patrol_sides,
    cancel_motion_commands_after_brake, wait_brake_applied_in_fault,
    FaultStopTracker, turn_recovery_decision,
)
from drivers.protocol_v2 import (
    AckStatus, MessageType, MotionActionCode, SequenceDisposition, StateFlag, StopReason,
    MotionAction, MotionActionFlag,
    WireGimbalPose, WireRangeStatus, classify_sequence, decode_control_target, decode_motion_action,
)


class _Stage2TurnNotReady(mcu.McuError):
    """An unsent pivot must earn a new strict-stop and one-second wait."""


class _Stage2RemainingTurnGuard(sequence._TurnDirectionGuard):
    """Direction proof for accepted/applied pivot CONTROL, without TURN flags."""

    def observe(self, state, generated_at, now, *, applied=False):
        identity = getattr(state, "session_nonce", None), getattr(state, "boot_id", None)
        uptime, raw = getattr(state, "uptime_ms", None), getattr(state, "yaw_tenth_deg", None)
        flags = getattr(state, "state_flags", 0)
        if (not applied or identity != self.identity
                or not flags & StateFlag.APPLIED_CONTROL_PRESENT
                or not flags & StateFlag.ODOM_VALID
                or isinstance(uptime, bool) or not isinstance(uptime, int)
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in (raw, generated_at, now))
                or not 0 <= now - generated_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9):
            return None
        if self.uptime is not None and (not 0 < ((uptime - self.uptime) & 0xFFFFFFFF) < 0x80000000
                                        or generated_at <= self.source_at):
            return None
        yaw = raw / 10.0
        self.yaw += sequence._normalize_yaw(yaw - self.yaw)
        self.uptime, self.source_at = uptime, generated_at
        progress = self.sign * (self.yaw - self.start_yaw)
        if progress <= -1.0 + 1e-9:
            return f"阶段二剩余左转已应用后反向航向累计{-progress:.2f}°达到1°，立即制动；禁止自动反号重试"
        return None


class Stage2Link(sequence.MotionSequenceLink):
    """Local passage policy; normal firmware and command evidence stay intact."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stage2_mode = "normal"
        self._recoverable_link_diagnostic_mask = 0x00000F3F
        self.stage2_turn_failure = None
        self.stage2_return = False
        self.stage2_opening = None
        self.stage2_junction_evidence = None
        self.stage2_gate_identity = None
        self.stage2_gate_sources = None
        self.stage2_gate_keys = None
        self.stage2_runtime_poll = None
        self.stage2_visual_health_gate = None
        self.stage2_visual_stop_pending = None
        self.stage2_visual_snapshot_gate = None
        self.stage2_target_guard = None
        self.stage2_vision = None
        self.stage2_startup_generation = None
        self.stage2_gimbal_monitor = None
        self.stage2_gimbal_pending = None
        self._stage2_gimbal_wire = self._stage2_gimbal_requested = None
        self._stage2_turn_target = None
        self.stage2_fault_stopping = False
        self.stage2_committed_turn_active = False
        self.stage2_world_yaw_offset = 0.0
        self._stage2_world_yaw = self._stage2_previous_raw_yaw = None
        self.stage2_recovery_pivot_first = None
        self.stage2_confirmed_pwm = None
        self.stage2_gimbal_state_waiting = False
        self.stage2_world_yaw_unknown = False
        self.stage2_confirmation_timeout = None
        self.stage2_recovery_block_reason = None
        self._stage2_timeout_reasons = set()
        self._stage2_uart_sends = {}
        self.stage2_recovery_active = False

    def _reset_link_state(self):
        super()._reset_link_state()
        self.stage2_opening = self.stage2_junction_evidence = None
        self.stage2_gate_identity = self.stage2_gate_sources = self.stage2_gate_keys = None
        self.stage2_return = False
        self.stage2_mode = "stopped"
        self.side_age_required = False
        self.stage2_gimbal_monitor = None
        self.stage2_gimbal_pending = None
        self._stage2_gimbal_wire = self._stage2_gimbal_requested = None
        self._stage2_turn_target = None
        self._stage2_previous_raw_yaw = None
        self.stage2_confirmation_timeout = None
        self.stage2_recovery_block_reason = None
        self._stage2_timeout_reasons.clear()
        self._stage2_uart_sends.clear()
        self.stage2_turn_failure = None

    def _latch_hard_fault(self, reason):
        self.stage2_opening = self.stage2_junction_evidence = None
        if reason == "转向被下位机中止" and getattr(self, "_stage2_fault_frame_family", None) == "state":
            if self._active_fault_bits():
                reason = ("下位机本地停机锁存" if self.state.fault_bits & (1 << 20) else "MCU固件故障")
            elif (self.stage2_committed_turn_active and self._stage2_turn_target is not None
                    and sequence._recoverable_turn_abort(self)):
                self.stage2_turn_failure = dict(identity=(self.session_nonce, self.boot_id),
                    reason=reason, source_at=self._sample_time("state"),
                    applied_sequence=self.state.applied_control_sequence,
                    diagnostic_history=self._turn_abort_evidence["record"][10]["turn_diagnostic_history"])
        if (reason == "转向被下位机中止" and self.stage2_fault_stopping
                and getattr(self, "_stage2_fault_frame_family", None) == "state"):
            state = self.state
            if state is not None and state.fault_bits & (1 << 20):
                reason = "下位机本地停机锁存"
            elif self._stage2_expected_brake_abort(state):
                # BRAKE deliberately cancels a partially completed turn. The
                # exact BRAKE must still earn its bounded ACK/application and
                # 500 ms stop proof; this flag grants no movement permission.
                return
        if ((getattr(self, "stage2_confirmation_timeout", None) is not None
                or getattr(self, "stage2_recovery_active", False))
                and reason not in self._stage2_timeout_reasons):
            state, diag = self.state, self.diagnostics
            encoder = (state is not None and state.fault_bits & (1 << 20) and diag is not None
                and diag.last_stop_reason is sequence.WireSafetyStopReason.ENCODER_FAULT
                and (diag.session_nonce, diag.boot_id) == (self.session_nonce, self.boot_id)
                and self._sample_is_fresh("diagnostics"))
            encoder_report = (getattr(self, "_stage2_fault_frame_family", None) == "state"
                and reason == "下位机本地停机锁存"
                or getattr(self, "_stage2_fault_frame_family", None) == "diagnostics"
                and encoder)
            if (not encoder or not encoder_report) and not (
                    self.stage2_turn_failure is not None and reason == self.stage2_turn_failure["reason"]):
                self.stage2_recovery_block_reason = reason
        return super()._latch_hard_fault(reason)

    def _stage2_expected_brake_abort(self, state):
        key = MessageType.MOTION_ACTION, self._last_brake_sequence
        record = self._command_ledger.get(key)
        if state is None or record is None:
            return False
        received = self._frame_received_at
        baseline = record[1]
        elapsed = None if baseline is None else (state.uptime_ms - baseline) & 0xffffffff
        return (record[5] == (MessageType.MOTION_ACTION, MotionActionCode.BRAKE, 0, 0, 0)
            and tuple(record[6:8]) == (state.session_nonce, state.boot_id)
            == (self.session_nonce, self.boot_id)
            and not sequence._active_runtime_fault_bits(self, state)
            and state.odom_status == 0 and state.state_flags & StateFlag.ODOM_VALID
            and state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT
            and state.applied_control_sequence == key[1]
            and not state.target_linear_mms and not state.target_angular_mrads
            and received is not None and received >= record[0]
            and (elapsed is None or 0 < elapsed < 0x80000000))

    def _record_confirmation_timeout(self, key, record, deadline, now):
        super()._record_confirmation_timeout(key, record, deadline, now)
        if record is None or tuple(record[6:8]) != (self.session_nonce, self.boot_id):
            return
        ack = record[8]
        if ack is not None and ack.status is not AckStatus.ACCEPTED:
            return
        missing = tuple(name for name, proof, received in (
            ("ack", ack, record[9]), ("state", record[2], record[3]))
            if proof is None or received is None or received > deadline + 1e-9)
        if not missing:
            return
        reason = f"{key[0].name} seq={key[1]}未在确认窗口内应用"
        self._stage2_timeout_reasons.add(reason)
        record[10]["stage2_confirmation_timeout"] = True
        if self._hard_fault_reason is None and self.stage2_confirmation_timeout is None:
            self.stage2_confirmation_timeout = dict(key=key, identity=tuple(record[6:8]),
                target=record[5], reason=reason, sent_at=record[0], deadline=deadline,
                checked_at=now, missing_or_late=missing)

    def _stop_failed_confirmation(self, reason):
        if (self.stage2_confirmation_timeout is not None
                and reason in self._stage2_timeout_reasons
                and self.stage2_recovery_block_reason is None):
            self._latch_hard_fault(reason)
            return
        return super()._stop_failed_confirmation(reason)

    def send_stop(self, reason=StopReason.MISSION):
        # The shared fixed-turn wait unwinds before the coordinator can BRAKE.
        # Only its classified confirmation-timeout cleanup may defer STOP.
        if (reason is StopReason.FAULT and getattr(self, "_turn_waiting", False)
                and self.stage2_confirmation_timeout is not None
                and self.stage2_recovery_block_reason is None
                and self._hard_fault_reason in self._stage2_timeout_reasons):
            runlog.emit("stage2_confirmation_stop_deferred", "确认超时转向等待退出后转入BRAKE停车恢复",
                        original_fault=self.stage2_confirmation_timeout)
            return None
        return super().send_stop(reason)

    def _retire_failed_recovery_zero(self, key, reason):
        record = self._pending_commands.get(key)
        if record is None:
            return False
        target = record[5]
        zero = (key[0] is MessageType.MOTION_ACTION and len(target) > 1
                and target[1] is MotionActionCode.BRAKE
                or key[0] is MessageType.CONTROL_TARGET and len(target) >= 3
                and not target[1] and not target[2])
        if not zero:
            return False
        if record[8] is not None and record[8].status is not AckStatus.ACCEPTED:
            return False
        if time.monotonic() < record[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S - 1e-9:
            return False
        record[10].update(disposition="recovery_zero_timeout", failure_reason=str(reason),
                          terminal_at=time.monotonic(), original_deadline=record[0]
                          + config.MCU_COMMAND_CONFIRM_TIMEOUT_S)
        runlog.emit("stage2_recovery_zero_timeout", "本次零目标停车确认失败，保留证据后终结待确认项",
                    sequence=key[1], identity=tuple(record[6:8]), target=target,
                    sent_at=record[0], ack_received_at=record[9], state_received_at=record[3],
                    reason=str(reason))
        self._pending_commands.pop(key, None)
        return True

    class _WriteClock:
        def __init__(self, serial_port, before_write=None):
            self.serial_port, self.started_at = serial_port, None
            self.before_write, self.write_attempted = before_write, False

        def __getattr__(self, name):
            return getattr(self.serial_port, name)

        def write(self, data):
            # Disarm first: lease failure may itself write a safe nested STOP.
            guard, self.before_write = self.before_write, None
            if guard is not None:
                guard()
            self.write_attempted = True
            self.started_at = time.monotonic()
            return self.serial_port.write(data)

    def _stage2_transport_send(self, message_type, payload, *, before_write=None):
        original = self._ser
        if original is None:
            return super()._send(message_type, payload)
        clock = self._WriteClock(original, before_write)
        original_sequence = self._seq
        self._ser = clock
        try:
            number = super()._send(message_type, payload)
        except Exception:
            if not clock.write_attempted:
                self._seq = original_sequence
            raise
        finally:
            self._ser = original
        # Only a complete successful write may acquire send/lease evidence.
        self._stage2_uart_sends[(message_type, number)] = clock.started_at
        while len(self._stage2_uart_sends) > 256:
            self._stage2_uart_sends.pop(next(iter(self._stage2_uart_sends)))
        return number

    def _track_command(self, kind, number, sent_at, baseline, target):
        sent_at = self._stage2_uart_sends.get((kind, number), sent_at)
        raw = self._action_sends.get(number) if kind is MessageType.MOTION_ACTION else None
        if raw is not None:
            self._action_sends[number] = (raw[0], sent_at, *raw[2:])
            if raw[0].action is MotionActionCode.BRAKE:
                self._last_brake_sent_at = sent_at
            else:
                self._active_motion_sent_at = sent_at
        elif kind is MessageType.CONTROL_TARGET and len(target) == 3 and any(target[1:3]):
            self._active_motion_sent_at = sent_at
        return super()._track_command(kind, number, sent_at, baseline, target)

    def pump(self):
        hook = self.stage2_runtime_poll
        result = super().pump()
        if self.motion_checks_enabled:
            self._check_motion_lease()
            if self.motion_health_reason() is not None:
                self.check_stage2_gimbal_output()
                return result
        if hook is not None:
            hook()
        self.check_stage2_gimbal_output()
        return result

    def _accept_state(self, state):
        previous = self.state
        probe = getattr(self, "_stage2_clock_probe", None)
        if probe is not None:
            number, sent_at, baseline = probe
            received = self._frame_received_at
            elapsed = None if baseline is None else (state.uptime_ms - baseline) & 0xffffffff
            if ((state.session_nonce, state.boot_id) == (self.session_nonce, self.boot_id)
                    and state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT
                    and state.applied_control_sequence == number
                    and not state.target_linear_mms and not state.target_angular_mrads
                    and received is not None and sent_at <= received
                    and received <= sent_at + config.MCU_COMMAND_CONFIRM_TIMEOUT_S + 1e-9
                    and (elapsed is None or 0 < elapsed < 0x80000000)):
                reference_raw = self._uptime_reference_raw
                uptime = state.uptime_ms if reference_raw is None else self._uptime_reference_ms + (
                    ((state.uptime_ms - reference_raw + 0x80000000) & 0xffffffff) - 0x80000000)
                lower_bound = sent_at - uptime / 1000.0
                self._clock_offset_s = (lower_bound if self._clock_offset_s is None
                                        else max(self._clock_offset_s, lower_bound))
        self._stage2_fault_frame_family = "state"
        try:
            super()._accept_state(state)
        finally:
            self._stage2_fault_frame_family = None
        monitor = self.stage2_gimbal_monitor
        if monitor is None and self.stage2_gimbal_pending is not None:
            command = self.stage2_gimbal_pending
            record = self._command_ledger.get((MessageType.CONTROL_TARGET, command))
            # Start observing at the first matching application, while the
            # original ACK/STATE wait is still decoding this UART batch.
            if record is not None and record[2] is not None:
                monitor = command, record[5][4], tuple(record[6:8])
        if self.state is state and previous is not state and monitor is not None:
            command, pose, identity = monitor
            reason = self._stage2_gimbal_reason(state, pose, identity)
            if reason is not None and self._hard_fault_reason is None:
                # Keep a transient output loss even if a later STATE in this
                # same UART batch re-enables it; decoding itself stays intact.
                self._stage2_gimbal_failed(reason, command=command, state=state, raise_error=False)
        if self.state is state and previous is not state and state.state_flags & StateFlag.ODOM_VALID:
            raw = state.yaw_tenth_deg / 10.0
            last = getattr(self, "_stage2_previous_raw_yaw", None)
            world = getattr(self, "_stage2_world_yaw", None)
            self._stage2_world_yaw = (raw + getattr(self, "stage2_world_yaw_offset", 0.0)
                                    if last is None or world is None else world + sequence._normalize_yaw(raw - last))
            self._stage2_previous_raw_yaw = raw

    def _accept_diagnostics(self, sample):
        self._stage2_fault_frame_family = "diagnostics"
        try:
            result = super()._accept_diagnostics(sample)
        finally:
            self._stage2_fault_frame_family = None
        if (self.diagnostics is sample and self.stage2_committed_turn_active
                and sequence._recoverable_turn_abort(self)):
            self.stage2_turn_failure = dict(identity=(self.session_nonce, self.boot_id),
                reason="转向被下位机中止", source_at=self._turn_abort_evidence["source_at"],
                applied_sequence=self._turn_abort_evidence["state"].applied_control_sequence,
                diagnostic_history=self._turn_abort_evidence["record"][10]["turn_diagnostic_history"])
        return result

    @property
    def stage2_world_yaw_deg(self):
        return getattr(self, "_stage2_world_yaw", None) if self.yaw_deg is not None else None

    def _stage2_before_nonzero(self, *, linear_mmps=None):
        if self.stage2_runtime_poll is not None:
            self.stage2_runtime_poll()
        health_gate = getattr(self, "stage2_visual_health_gate", None)
        if health_gate is not None and not getattr(self, "stage2_committed_turn_active", False):
            health_gate()
        if not getattr(self, "stage2_committed_turn_active", False) and self.stage2_visual_stop_pending is not None and self.stage2_visual_stop_pending():
            raise _Stage2VisionInterrupt()
        guard = getattr(self, "stage2_target_guard", None)
        if guard is not None:
            guard()

    def _stage2_require_turn_ready(self, command):
        """Pure reads only: this also runs inside the final visual write lock."""
        identity = self.session_nonce, self.boot_id
        target = (MessageType.MOTION_ACTION, command.action, command.speed_mms,
                  int(command.flags), command.turn_angle_tenth_deg)
        if getattr(self, "_turn_waiting", False):
            if (self._current_target != target
                    or self._stage2_turn_target != (identity, target)):
                raise mcu.McuError("原地掉头执行中仅允许当前会话同参数UTURN续约")
            return
        now = time.monotonic()
        key = MessageType.MOTION_ACTION, self._last_brake_sequence
        record = self._command_ledger.get(key)
        witness_key = self._superseded_commands.get(key, key)
        witness = self._command_ledger.get(witness_key)
        evidence_at = self._confirmed_evidence_times.get(key)
        tracker = getattr(self, "_stop_tracker", None)
        barrier = getattr(self, "_stable_barrier", None)
        brake_target = (MessageType.MOTION_ACTION, MotionActionCode.BRAKE, 0, 0, 0)
        valid_brake = (self._last_brake_identity == identity and None not in identity
            and record is not None and witness is not None
            and tuple(record[6:8]) == identity and witness[4:8] == record[4:8]
            and record[5] == brake_target and witness[0] >= record[0]
            and witness[2] is not None and witness[8] is not None
            and witness[8].status is AckStatus.ACCEPTED
            and not witness[2].target_linear_mms and not witness[2].target_angular_mrads
            and witness[3] is not None and witness[9] is not None
            and evidence_at is not None and math.isfinite(evidence_at)
            and evidence_at == max(witness[3], witness[9])
            and record[0] <= evidence_at <= record[0] + config.MCU_COMMAND_CONFIRM_TIMEOUT_S + 1e-9)
        if not valid_brake:
            raise _Stage2TurnNotReady("UTURN缺少本次BRAKE在原200ms内的接受与零目标应用证据")
        valid_stop = (tracker is not None and barrier is not None
            and tracker.identity == identity and tracker.source_barrier == evidence_at
            and self._brake_evidence_barrier == evidence_at
            and tracker.quiet_since is not None and tracker.last_state_source is not None
            and tracker.count >= 2
            and tracker.last_state_source - tracker.quiet_since + 1e-9 >= sequence.STOP_OBSERVATION_S
            and 0 <= now - tracker.last_state_source <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9
            and barrier >= tracker.quiet_since + sequence.STOP_OBSERVATION_S - 1e-9)
        if not valid_stop:
            raise _Stage2TurnNotReady("UTURN缺少本次BRAKE后双轮连续500ms停稳证据")
        if now - barrier < STAGE2_TURN_STATIONARY_WAIT_S - 1e-9:
            raise _Stage2TurnNotReady("UTURN严格停稳后尚未额外静止满1秒")
        state = self.state
        if (state is None or (state.session_nonce, state.boot_id) != identity
                or not state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT
                or state.applied_control_sequence not in (key[1], witness_key[1])
                or state.target_linear_mms or state.target_angular_mrads
                or self.odom_mm != tracker.previous):
            raise _Stage2TurnNotReady("UTURN发令前本次BRAKE零目标或逐轮静止证据已改变")
        previous_source, previous_token = tracker.last_state_source, tracker.token
        for pending, source in sorted(self._state_events, key=lambda item: item[1]):
            elapsed = (pending.uptime_ms - previous_token) & 0xffffffff
            if source < previous_source or not 0 < elapsed < 0x80000000:
                continue
            if ((pending.session_nonce, pending.boot_id) != identity
                    or (pending.left_mm, pending.right_mm) != tracker.previous
                    or source - previous_source > config.TELEMETRY_STALE_MS / 1000.0 + 1e-9
                    or not pending.state_flags & StateFlag.APPLIED_CONTROL_PRESENT
                    or pending.applied_control_sequence not in (key[1], witness_key[1])
                    or pending.target_linear_mms or pending.target_angular_mrads):
                raise _Stage2TurnNotReady("UTURN发令前新STATE出现运动、断档或BRAKE应用失效")
            previous_source, previous_token = source, pending.uptime_ms
        for trace, source in self._trace_events:
            if (source <= tracker.source_barrier + 1e-9
                    or tracker.last_trace_source is not None and source < tracker.last_trace_source
                    or tracker.last_trace_token is not None and classify_sequence(
                        trace.sample_sequence, tracker.last_trace_token) is not SequenceDisposition.NEWER):
                continue
            if trace.left_encoder_delta or trace.right_encoder_delta:
                raise _Stage2TurnNotReady("UTURN发令前未消费轮迹仍有单轮运动")

    def _send(self, message_type, payload):
        # Recheck after all command preparation and runtime callbacks, before
        # the transport can consume a sequence or write a nonzero target.
        nonzero = (message_type in (MessageType.CONTROL_TARGET, MessageType.MOTION_ACTION)
                    and getattr(self, "_sending_nonzero", False))
        action = decode_motion_action(payload) if message_type is MessageType.MOTION_ACTION else None
        pivot = action is not None and action.action is MotionActionCode.UTURN_LEFT
        recovery_pivot = getattr(self, "stage2_recovery_pivot_first", None)
        write_vision = None

        def before_write():
            # Encoding/preparation may have consumed the remaining budget.
            # The ordinary branch still owns its publication lock here; use
            # only existing pure reads, without polling or another lock wait.
            self._check_motion_deadline()
            self._check_motion_lease()
            reason = self.motion_health_reason()
            if reason is not None:
                raise mcu.McuError(reason)
            if write_vision is not None:
                snapshot_gate = getattr(self, "stage2_visual_snapshot_gate", None)
                if snapshot_gate is not None:
                    snapshot_gate(write_vision._snapshot_locked())
            if not getattr(self, "stage2_committed_turn_active", False):
                stop_pending = getattr(self, "stage2_visual_stop_pending", None)
                if stop_pending is not None and stop_pending():
                    raise _Stage2VisionInterrupt()
            guard = getattr(self, "stage2_target_guard", None)
            if guard is not None:
                guard()
            if pivot or recovery_pivot is not None:
                self._stage2_require_turn_ready(action if pivot else recovery_pivot)

        if nonzero:
            health_gate = getattr(self, "stage2_visual_health_gate", None)
            if health_gate is not None and not getattr(self, "stage2_committed_turn_active", False):
                health_gate()
            # The final visual checkpoint can itself deliver a newly confirmed
            # obstacle after the earlier send checks.
            stop_pending = getattr(self, "stage2_visual_stop_pending", None)
            if not getattr(self, "stage2_committed_turn_active", False) and stop_pending is not None and stop_pending():
                raise _Stage2VisionInterrupt()
            # Visual polling can wait for a lock and consume the remaining
            # freshness/deadline budget after the earlier motion checks.
            # Recheck the existing motion gates, then the controller witness,
            # without another polling callback before the UART write.
            self._check_motion_deadline()
            self._check_motion_lease()
            reason = self.motion_health_reason()
            if reason is not None:
                raise mcu.McuError(reason)
            guard = getattr(self, "stage2_target_guard", None)
            if guard is not None:
                guard()
        vision = getattr(self, "stage2_vision", None)
        if vision is not None and nonzero and not getattr(self, "stage2_committed_turn_active", False):
            with vision.motion_write():
                write_vision = vision
                # The bounded lock acquisition can spend part of the original
                # motion budget. These production guards are pure reads, with
                # no input, UART pump, or detection callback inside this lock.
                self._check_motion_deadline()
                self._check_motion_lease()
                reason = self.motion_health_reason()
                if reason is not None:
                    raise mcu.McuError(reason)
                guard = getattr(self, "stage2_target_guard", None)
                if guard is not None:
                    guard()
                # Sample after the last target guard, then serialize this write
                # against the detector's confirmation publication. No callback
                # may publish visual evidence between this snapshot and UART.
                snapshot = vision._snapshot_locked()
                snapshot_gate = getattr(self, "stage2_visual_snapshot_gate", None)
                if snapshot_gate is not None:
                    snapshot_gate(snapshot)
                # Keep motion budgets valid after the snapshot gate, then read
                # the same publication without driver or log work before UART.
                self._check_motion_deadline()
                self._check_motion_lease()
                reason = self.motion_health_reason()
                if reason is not None:
                    raise mcu.McuError(reason)
                if guard is not None:
                    guard()
                if snapshot_gate is not None:
                    snapshot_gate(vision._snapshot_locked())
                if pivot:
                    self._stage2_require_turn_ready(action)
                sent = self._stage2_transport_send(message_type, payload, before_write=before_write)
                self.stage2_gimbal_monitor = None
                vision.mark_motion_sent()
                self._stage2_trace_gimbal_wire(message_type, payload, sent)
                return sent
        if nonzero and (pivot or recovery_pivot is not None):
            self._stage2_require_turn_ready(action if pivot else recovery_pivot)
        sent = self._stage2_transport_send(message_type, payload,
                                           before_write=before_write if nonzero else None)
        if recovery_pivot is not None and nonzero:
            self.stage2_recovery_pivot_first = None
        if nonzero:
            self.stage2_gimbal_monitor = None
        self._stage2_trace_gimbal_wire(message_type, payload, sent)
        return sent

    def send_motion(self, linear_mmps, angular_mradps, **kwargs):
        if linear_mmps or angular_mradps:
            self._stage2_before_nonzero(linear_mmps=linear_mmps)
        return super().send_motion(linear_mmps, angular_mradps, **kwargs)

    def synchronize_clock(self, timeout_s=None):
        if not self.connected or self._hard_fault_reason is not None:
            raise mcu.McuError(self._hard_fault_reason or "时钟同步前必须建立HELLO会话")
        if self._clock_sync_attempted:
            raise mcu.McuError("当前会话已经尝试零速时钟同步")
        self._clock_sync_attempted = True
        baseline = self._family_uptime_ms.get("state")
        number = self.send_motion(0, 0)
        sent_at = self._stage2_uart_sends[(MessageType.CONTROL_TARGET, number)]
        self._track_command(MessageType.CONTROL_TARGET, number, sent_at, baseline,
                            (MessageType.CONTROL_TARGET, 0, 0))
        self._stage2_clock_probe = (number, sent_at, baseline)
        try:
            state = self._wait_applied(MessageType.CONTROL_TARGET, number, timeout_s)
        except mcu.McuError as exc:
            self._retire_failed_recovery_zero((MessageType.CONTROL_TARGET, number), exc)
            raise
        finally:
            self._stage2_clock_probe = None
        if state.target_linear_mms or state.target_angular_mrads:
            raise mcu.McuError("时钟同步应用STATE并非零底盘目标")
        record = self._command_ledger[(MessageType.CONTROL_TARGET, number)]
        reference_raw = self._uptime_reference_raw
        uptime = (None if reference_raw is None else self._uptime_reference_ms + (
            ((state.uptime_ms - reference_raw + 0x80000000) & 0xffffffff) - 0x80000000))
        if uptime is None:
            raise mcu.McuError("零速同步缺少当前STATE时钟")
        previous = self._clock_offset_s
        lower_bound = sent_at - uptime / 1000.0
        self._clock_offset_s = lower_bound if previous is None else max(previous, lower_bound)
        runlog.emit("mcu_clock_synchronized", "零速精确ACK/应用回读已收紧MCU采集时间下界",
                    sequence=number, issued_monotonic=sent_at,
                    confirmation_window_s=self._confirm_timeout(timeout_s),
                    reply_uptime_ms=state.uptime_ms, reply_uptime_expanded_ms=uptime,
                    reply_received_monotonic=record[3], previous_offset_s=previous,
                    offset_s=self._clock_offset_s)
        return state

    def send_motion_action(self, action, speed_mms=0, **kwargs):
        if action is not MotionActionCode.BRAKE:
            self._stage2_before_nonzero()
        sent = super().send_motion_action(action, speed_mms, **kwargs)
        if action is MotionActionCode.BRAKE:
            self._stage2_turn_target = None
            cancel_motion_commands_after_brake(self, sent, self._last_brake_sent_at)
        elif action is MotionActionCode.UTURN_LEFT:
            self._stage2_turn_target = ((self.session_nonce, self.boot_id), self._current_target)
        return sent

    def _wait_applied(self, kind, number, timeout_s=None):
        if getattr(self, "stage2_fault_stopping", False) and kind is MessageType.MOTION_ACTION:
            record = self._action_sends.get(number)
            if record is not None and record[0].action is MotionActionCode.BRAKE:
                return wait_brake_applied_in_fault(self, number, timeout_s)
        return super()._wait_applied(kind, number, timeout_s)

    def wait_stage2_parked_pwm(self, number, pwm, input_source=None, timeout_s=.2):
        """Confirm only the last parked PWM without clearing an older fault."""
        if tuple(pwm) != self.stage2_confirmed_pwm:
            raise mcu.McuError("故障停车只能恢复最后确认的PWM")
        key = MessageType.CONTROL_TARGET, number
        record = self._command_ledger.get(key)
        expected = (MessageType.CONTROL_TARGET, 0, 0, "gimbal", int(pwm[0]),
                    int(pwm[1]), int(pwm[2]))
        if record is None or record[5] != expected:
            raise mcu.McuError("停车PWM缺少本次精确云台命令")
        identity, deadline = tuple(record[6:8]), record[0] + self._confirm_timeout(timeout_s)
        while True:
            sequence._check_input_exit(input_source)
            if not self.connected or (self.session_nonce, self.boot_id) != identity:
                raise mcu.McuError("停车PWM确认期间会话变化")
            ack, state = record[8], record[2]
            if ack is not None and ack.status is not AckStatus.ACCEPTED:
                raise mcu.McuError("停车PWM命令被拒：" + ack.status.name)
            if ack is not None and state is not None:
                reason = self._stage2_gimbal_reason(state, int(pwm[0]), identity)
                if reason is not None:
                    raise mcu.McuError(reason)
                if (ack.acknowledged_type is key[0] and ack.acknowledged_sequence == number
                        and (ack.session_nonce, ack.boot_id) == identity
                        and state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT
                        and state.applied_control_sequence == number
                        and record[3] is not None and record[9] is not None
                        and record[0] - 1e-9 <= min(record[3], record[9])
                        and max(record[3], record[9]) <= deadline + 1e-9):
                    self._confirmed_commands[key] = state
                    self._confirmed_send_times[key] = record[0]
                    self._confirmed_evidence_times[key] = max(record[3], record[9])
                    self._pending_commands.pop(key, None)
                    self.stage2_gimbal_pending = None
                    runlog.emit("stage2_parked_pwm_restored", "停车PWM原命令ACK/STATE及输出启用已确认",
                                sequence=number, pan_us=pwm[1], tilt_us=pwm[2],
                                original_fault=self._hard_fault_reason, mechanical_angle_verified=False)
                    return state
            if time.monotonic() >= deadline:
                self._record_confirmation_timeout(key, record, deadline, time.monotonic())
                self._retire_failed_recovery_zero(key, "停车PWM未在原200ms确认窗口内应用")
                raise mcu.McuError("停车PWM未在原200ms确认窗口内应用")
            self.pump()
            time.sleep(.005)

    def send_gimbal(self, pose, pan_us, tilt_us):
        # A new PWM command may legitimately change pose; it must earn its own
        # output evidence before monitoring the new setting.
        self.stage2_gimbal_monitor = None
        self.stage2_gimbal_pending = None
        sent_at = time.monotonic()
        baseline = getattr(self.state, "uptime_ms", None)
        self._stage2_gimbal_requested = (int(pose), pan_us, tilt_us)
        command = super().send_gimbal(pose, pan_us, tilt_us)
        # Gimbal CONTROL_TARGET carries zero wheel fields and never renews the
        # motion lease. It still requires this exact ACK and applied STATE in200ms.
        target = (MessageType.CONTROL_TARGET, 0, 0, "gimbal", int(pose),
                  int(pan_us), int(tilt_us))
        self._track_command(MessageType.CONTROL_TARGET, command, sent_at, baseline, target)
        self.stage2_gimbal_pending = command
        return command

    def _stage2_trace_gimbal_wire(self, message_type, payload, command):
        """Diagnostic only: decoded bytes sent, never evidence of servo motion."""
        # Logging after a successful UART write cannot turn that write into a
        # failed command or prevent its normal sequence/evidence bookkeeping.
        try:
            previous = getattr(self, "_stage2_gimbal_wire", None)
            target = decode_control_target(payload) if message_type is MessageType.CONTROL_TARGET else None
            from drivers.protocol_v2 import ControlFlag
            if target is not None and target.flags & ControlFlag.GIMBAL_VALID:
                self._stage2_gimbal_wire = (command, target, False)
                requested = getattr(self, "_stage2_gimbal_requested", None)
                runlog.emit("stage2_gimbal_wire", "本次云台实际发送字段；机械动作待观察",
                            sequence=command, message_type=int(message_type),
                            session_nonce=target.session_nonce, boot_id=self.boot_id,
                            requested_pose=None if requested is None else requested[0],
                            requested_pan_us=None if requested is None else requested[1],
                            requested_tilt_us=None if requested is None else requested[2],
                            wire_pose=int(target.pose_id), wire_flags=int(target.flags),
                            wire_pan_pwm_us=target.pan_pwm_us, wire_tilt_pwm_us=target.tilt_pwm_us,
                            ch1_pwm_us=target.pan_pwm_us, ch2_pwm_us=target.tilt_pwm_us,
                            linear_mms=target.linear_mms, angular_mrads=target.angular_mrads,
                            replaces_sequence=None if previous is None else previous[0],
                            host_trace_at_s=time.monotonic(), mechanical_angle_verified=False)
            elif previous is not None and not previous[2] and message_type in (
                    MessageType.CONTROL_TARGET, MessageType.MOTION_ACTION, MessageType.STOP):
                self._stage2_gimbal_wire = (previous[0], previous[1], True)
                runlog.emit("stage2_gimbal_following_command", "云台命令后的首条控制指令；完整后续见原始串口记录",
                            gimbal_sequence=previous[0], sequence=command, message_type=int(message_type),
                            session_nonce=self.session_nonce, boot_id=self.boot_id,
                            wire_flags=None if target is None else int(target.flags),
                            wire_pan_pwm_us=None if target is None else target.pan_pwm_us,
                            wire_tilt_pwm_us=None if target is None else target.tilt_pwm_us,
                            gimbal_valid=False, host_trace_at_s=time.monotonic())
        except Exception:
            pass

    @staticmethod
    def _stage2_gimbal_reason(state, pose, identity):
        if state is None:
            return "尚无云台STATE"
        if (state.session_nonce, state.boot_id) != identity:
            return "云台STATE会话或boot不匹配"
        if not state.state_flags & StateFlag.GIMBAL_PRESENT:
            return "STATE未确认云台存在(GIMBAL_PRESENT)"
        if not state.state_flags & StateFlag.GIMBAL_PWM_ENABLED:
            return "STATE显示云台PWM输出关闭(GIMBAL_PWM_ENABLED)"
        if int(state.gimbal_pose) != pose:
            return "STATE云台pose与本次PWM命令不匹配"
        if state.target_linear_mms or state.target_angular_mrads:
            return "停车调整期间STATE底盘目标非零"
        return None

    def _stage2_gimbal_failed(self, reason, *, command=None, state=None, raise_error=True):
        record = self._command_ledger.get((MessageType.CONTROL_TARGET, command))
        runlog.emit("stage2_gimbal_output_failed", "云台PWM证据未通过，保持停车",
                    reason=reason, sequence=command,
                    target=None if record is None else record[5],
                    ack_received_at_s=None if record is None else record[9],
                    state_received_at_s=None if record is None else record[3],
                    current_identity=(self.session_nonce, self.boot_id),
                    state_identity=None if state is None else (state.session_nonce, state.boot_id),
                    state_uptime_ms=getattr(state, "uptime_ms", None),
                    state_flags=None if state is None else int(state.state_flags),
                    gimbal_pose=None if state is None else int(state.gimbal_pose),
                    gimbal_motion_state=None if state is None else int(state.gimbal_motion_state),
                    applied_control_sequence=getattr(state, "applied_control_sequence", None),
                    target_linear_mms=getattr(state, "target_linear_mms", None),
                    target_angular_mrads=getattr(state, "target_angular_mrads", None))
        self._stop_failed_confirmation("云台PWM未确认：" + reason)
        if raise_error:
            raise mcu.McuError(self._hard_fault_reason)

    def wait_for_control_applied(self, command, timeout_s=None):
        applied = super().wait_for_control_applied(command, timeout_s)
        record = self._command_ledger.get((MessageType.CONTROL_TARGET, command))
        if record is None or len(record[5]) < 4 or record[5][3] != "gimbal":
            return applied
        # The original command's first matching applied STATE is authoritative.
        # A newer STATE or a same-target command cannot repair missing flags,
        # wrong pose, nonzero chassis output or late original evidence.
        state, ack = record[2], record[8]
        deadline = record[0] + self._confirm_timeout(timeout_s)
        identity = tuple(record[6:8])
        reason = self._stage2_gimbal_reason(state, record[5][4], identity)
        if reason is None and (ack is None or ack.status is not AckStatus.ACCEPTED
                or (ack.session_nonce, ack.boot_id) != identity
                or ack.acknowledged_type is not MessageType.CONTROL_TARGET
                or ack.acknowledged_sequence != command):
            reason = "本次云台命令缺少匹配ACCEPTED ACK"
        if reason is None and (state.applied_control_sequence != command
                or not state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT):
            reason = "本次云台命令缺少匹配应用STATE"
        if reason is None and (record[3] is None or record[9] is None
                or max(record[3], record[9]) > deadline + 1e-9):
            reason = "本次云台ACK/STATE未在原200ms确认窗口到达"
        if reason is not None:
            self._stage2_gimbal_failed(reason, command=command, state=state)
        self.stage2_gimbal_monitor = (command, record[5][4], identity)
        self.stage2_confirmed_pwm = (WireGimbalPose(record[5][4]), *record[5][5:7])
        self.stage2_gimbal_pending = None
        runlog.emit("stage2_gimbal_output_confirmed", "MCU确认本次云台命令且PWM输出启用",
                    sequence=command, pose=record[5][4], pan_us=record[5][5], tilt_us=record[5][6],
                    state_flags=int(state.state_flags), state_uptime_ms=state.uptime_ms,
                    ack_received_at_s=record[9], state_received_at_s=record[3],
                    command_sent_at_s=record[0], confirm_deadline_s=deadline,
                    applied_control_sequence=state.applied_control_sequence,
                    ack_status=int(ack.status), state_reports_pwm_us=False,
                    mechanical_angle_verified=False)
        self.check_stage2_gimbal_output()
        return state

    def check_stage2_gimbal_output(self):
        monitor = getattr(self, "stage2_gimbal_monitor", None)
        if monitor is None:
            return
        if self._hard_fault_reason is not None:
            raise mcu.McuError(self._hard_fault_reason)
        command, pose, identity = monitor
        if self.connected and not self._sample_is_fresh("state"):
            # A temporary parked telemetry gap invalidates permission to write,
            # but does not invent a loss of an already confirmed PWM output.
            self.stage2_gimbal_state_waiting = True
            return
        self.stage2_gimbal_state_waiting = False
        reason = self._stage2_gimbal_reason(self.state, pose, identity)
        if reason is None and (not self.connected or not self._sample_is_fresh("state")):
            reason = "停车调整期间云台STATE超龄或连接丢失"
        if reason is not None:
            self._stage2_gimbal_failed(reason, command=command, state=self.state)

    def set_stage2_mode(self, mode):
        if mode not in ("normal", "passage", "departure", "turn", "stopped"):
            raise ValueError("非法阶段二激光门禁")
        self.stage2_mode = mode
        # Legal missing readings use the stage-one heading policy, including
        # normal roads and turn departure; they never renew position evidence.
        self.side_age_required = False
        self.side_departure_active = False
        if mode != "departure":
            sequence.MotionSequenceLink.finish_departure(self)
        self.side_cached_active = False
        # The base junction latch is never authority for this local itinerary.
        sequence._clear_junction(self)
        if mode != "normal":
            sequence._clear_side_recovery(self)

    def begin_stage2_passage(self, opening, evidence=None):
        identity = self.session_nonce, self.boot_id
        now = time.monotonic()
        if evidence is None and opening is self.stage2_opening:
            evidence = getattr(self, "stage2_junction_evidence", None)
        if evidence is None:
            if (not opening.candidates or not all(
                    0 <= now - opening.source_times[index] <= .250 + 1e-9
                    for index, side in enumerate(("left", "right"))
                    if side in opening.candidates)):
                raise mcu.McuError("穿口必须使用同会话真正新鲜开口或已保存路口证据")
            at = opening.received_at - opening.receive_delay_ms / 1000.0
            evidence = PatrolJunctionEvidence("explicit_opening", identity, at,
                opening.snapshot_uptime_ms, tuple(sorted(opening.candidates)), None, None)
        if (opening.source_identity != identity or None in identity
                or evidence.identity != identity or not evidence.openings
                or evidence.kind not in ("explicit_opening", "missing_inference")
                or getattr(self, "_hard_fault_reason", None) is not None):
            raise mcu.McuError("穿口路口证据身份失效或已有真实故障")
        _check_patrol_sides(opening, now)
        self.stage2_gate_identity = identity
        self.stage2_gate_sources = tuple(opening.source_times)
        self.stage2_gate_keys = opening.source_keys
        self.stage2_opening = opening
        self.stage2_junction_evidence = evidence
        self.set_stage2_mode("passage")
        return max(evidence.confirmed_at, *opening.source_times)

    def begin_stage2_return(self, barrier):
        sample = self._latest_side_sample
        self.stage2_return = True
        self.stage2_opening = None
        self.stage2_junction_evidence = None
        self.stage2_gate_identity = self.session_nonce, self.boot_id
        self.stage2_gate_sources = (barrier, barrier)
        self.stage2_gate_keys = sample.source_keys if sample is not None else None
        self.set_stage2_mode("departure")
        # TURN-era samples cannot authorize either the next walls or a return
        # junction, even if they were queued during the final strict stop.
        self._side_events.clear()
        self._latest_side_sample = None
        sequence.MotionSequenceLink.finish_departure(self)

    def check_patrol_sides(self, sample, now=None, **_kwargs):
        _check_patrol_sides(sample, now)
        if sample.source_identity != (self.session_nonce, self.boot_id):
            raise mcu.McuError("下层激光样本身份不属于当前会话")

    def cached_side_feedback(self, now=None):
        self.side_cached_active = False
        return False

    def _accept_directional(self, directional):
        if not self._session_matches(directional.session_nonce, directional.boot_id):
            if self.connected and directional.boot_id != self.boot_id:
                self._latch_hard_fault("MCU boot 变化（复位）")
            return
        if (self._last_directional_seq is not None
                and directional.directional_cycle_seq != self._last_directional_seq
                and classify_sequence(directional.directional_cycle_seq, self._last_directional_seq)
                is not SequenceDisposition.NEWER):
            return
        if not self._accept_uptime("directional", directional.uptime_ms):
            return
        self._last_directional_seq = directional.directional_cycle_seq
        self.directional = directional
        sample = sequence._annotate_side_sample(self, sequence._side_sample(self))
        self._latest_side_sample = sample
        try:
            self.check_patrol_sides(sample, sample.received_at)
            if (self.motion_checks_enabled and self.side_candidates_enabled
                    and self.stage2_mode == "normal" and sample.candidates):
                if self.stage2_opening is None:
                    self.stage2_opening = sample
                    at = sample.received_at - sample.receive_delay_ms / 1000.0
                    self.stage2_junction_evidence = PatrolJunctionEvidence(
                        "explicit_opening", sample.source_identity, at,
                        sample.snapshot_uptime_ms, tuple(sorted(sample.candidates)), None, None)
                    if self.stage2_return:
                        # Receive-time stop authority prevents a late nonzero renewal.
                        self._side_brake_pending = True
                    else:
                        self.begin_stage2_passage(sample)
        except mcu.McuError as exc:
            if self.motion_checks_enabled:
                self._latch_hard_fault(str(exc))
        if len(self._side_events) >= 256:
            self._latch_hard_fault("下层激光快照队列溢出，测量证据不完整")
            return
        self._side_events.append(sample)

    def _laser_stale_reason(self, *, require_upper=True):
        try:
            if self._latest_side_sample is not None:
                self.check_patrol_sides(self._latest_side_sample, time.monotonic())
            evidence = self.stage2_junction_evidence
            if (evidence is not None
                    and evidence.identity != (self.session_nonce, self.boot_id)):
                self.stage2_opening = self.stage2_junction_evidence = None
                return "阶段二已保存路口证据身份失效"
        except mcu.McuError as exc:
            return str(exc)
        return None


def _stage2_fresh_position_feedback(link, controller, now):
    identity = link.session_nonce, link.boot_id
    sample = getattr(link, "_latest_side_sample", None)
    sources = controller.latest_source_times
    # Raw VALID caches can advance before their delayed STATE arrives.
    # They cannot renew the age of the last actually aligned position pair.
    return bool(sample is not None and sample.dual_wall
                and sample.source_identity == identity and None not in identity
                and controller._state_session == identity
                and controller._usable_pair and controller._last_pair_pose is not None
                and sources is not None
                and all(0 <= now - source <= .250 + 1e-9
                        and (controller.source_barrier is None
                             or source > controller.source_barrier + 1e-9)
                        and controller._pose_at(source) is not None for source in sources)
                and all(0 <= now - source <= .250 + 1e-9 for source in sample.source_times)
                and controller.control_available(now))


class Stage2CenteringController(PatrolCenteringController):
    """Stage-two forward steering, independent of changing patrol policy.

    Real, STATE-aligned laser pairs provide the raw half difference and trend.
    Steering/speed changes remain proposals until their UART write succeeds.
    """

    HEADING_LIMIT_DEG = 10.0
    HEADING_RETURN_EXIT_DEG = 5.0
    ANGULAR_LIMIT_MRADPS = 768
    ANGULAR_STEP_MRADPS = 32
    FILTER_TAU_S = .15
    TREND_HORIZON_S = .5
    LOOKAHEAD_MM = 150.0

    def __init__(self, now=None, *, link=None):
        self.stage2_link = link
        super().__init__(now)

    def reset(self, now, source_barrier=None):
        # Retain the adapter type expected by the display/runner, but do not
        # inherit the independently evolving stage-one control policy.
        self._patrol_cruise_policy = False
        self._stage2_heading_recovery = False
        self._stage2_correction_phase = "HOLD"
        sequence.CenteringController.reset(self, now, source_barrier=source_barrier)

    def observe_state(self, state, generated_at):
        previous = self._state_at
        sequence.CenteringController.observe_state(self, state, generated_at)
        if self._state_at != previous:
            psi = self._heading_error()
            if psi is not None:
                if abs(psi) >= self.HEADING_LIMIT_DEG - 1e-9:
                    self._stage2_heading_recovery = True
                elif abs(psi) <= self.HEADING_RETURN_EXIT_DEG + 1e-9:
                    self._stage2_heading_recovery = False

    @property
    def heading_recovery_active(self):
        psi = self._heading_error()
        return bool(self._stage2_heading_recovery or psi is not None
                    and abs(psi) >= self.HEADING_LIMIT_DEG - 1e-9)

    def retry_pending_pair(self, sample=None, now=None):
        if now is not None:
            self.refresh_yaw_rate(now)
        if sample is not None:
            return self.observe_side(sample)
        consumed = False
        while self._pending_pair_queue:
            candidate, histories, keys = self._pending_pair_queue[0]
            sources = tuple(history[-1][0] for history in histories)
            at = min(sources)
            if (candidate.source_identity is not None and candidate.source_identity != self._state_session
                    or self.source_barrier is not None and any(s <= self.source_barrier + 1e-9 for s in sources)
                    or any(not 0 <= self._rate_now - s <= .250 + 1e-9 for s in sources)
                    or self._pair_at is not None and at <= self._pair_at + 1e-9
                    or not all(sequence._source_key_newer(key, old) for key, old in zip(keys, self._paired_keys))):
                self._pending_pair_queue.popleft()
                continue
            if not self._poses or at > self._poses[-1].at + 1e-9:
                break
            self._pending_pair_queue.popleft()
            pose = self._pose_at(at)
            aligned = [self._side_at(history, at) for history in histories]
            if pose is None or any(item is None for item in aligned):
                continue
            difference = aligned[0][0] - aligned[1][0]
            observed = difference / 2.0
            error = observed
            if self._pair_at is not None and self._error is not None:
                dt = at - self._pair_at
                alpha = -math.expm1(-dt / self.FILTER_TAU_S)
                previous = self._error
                error = previous + alpha * (observed - previous)
                self._error_rate += alpha * ((error - previous) / dt - self._error_rate)
            self._pair_at, self._latest_source_times = at, sources
            self._last_pair_pose = pose
            self._raw_error = self._observed_error = observed
            self._last_raw_difference = difference
            self._error, self._usable_pair = error, True
            self._paired_keys = list(keys)
            self.observation_token += 1
            self._lateral_sources.append((at, pose.travel_mm, observed))
            self._compensated_sources.append((at, sources, tuple(keys), observed, pose.yaw))
            consumed = True
        self._pending_pair_sample = self._pending_pair_queue[0][0] if self._pending_pair_queue else None
        return consumed

    def predicted_error_mm(self, now):
        # The observed front-sensor difference is the control quantity. Odom
        # and axle geometry cannot replace or extend this source evidence.
        return self._error

    def _consume_pending_means(self, now):
        consumed = sequence.CenteringController._consume_pending_means(self, now)
        self._mean_reason = self._mean_reason.replace("车轴补偿均值差", "原始左右差均值")
        return consumed

    def _quantized_angular(self, requested):
        magnitude = min(self.ANGULAR_LIMIT_MRADPS,
                        math.floor(abs(requested) / 16.0 + .5) * 16)
        return int(math.copysign(magnitude, requested)) if requested else 0

    @staticmethod
    def _positive_action(wanted, angular, floor, direction=1):
        return sequence.CenteringController._positive_action(wanted, angular, floor, direction)

    def _speed_proposal(self, goal, now, angular, *, cruise, frozen, departure=False):
        return sequence.CenteringController._speed_proposal(
            self, goal, now, angular, cruise=cruise, frozen=frozen, departure=departure)

    def _estimate_speed(self, now):
        return sequence.CenteringController._estimate_speed(self, now)

    def _slew_angular(self, requested, now):
        # A delayed loop gets one step, never accumulated catch-up authority.
        allowance = (self.ANGULAR_STEP_MRADPS if self._last_sent_at is None
                     or now - self._last_sent_at >= .1 - 1e-9 else 0)
        selected = self._quantized_angular(requested)
        return max(self._last_sent_angular - allowance,
                   min(self._last_sent_angular + allowance, selected))

    def _anticipate_heading_return(self, angular, now, psi, rate, limit):
        horizon = (config.MCU_COMMAND_CONFIRM_TIMEOUT_S + sequence.CONTROL_INTERVAL_S
                   + max(0., now - self._state_at))
        for sign in (-1, 1):
            # An executing outward command still needs unloading when this
            # proposal is zero or already countersteering. Check both sides.
            pending = max(0, sign * angular, sign * self._last_sent_angular,
                          sign * self._applied_angular)
            outward_rate = max(0., sign * (rate or 0.), math.degrees(pending / 1000.))
            unload_angle = 0.0
            while pending:
                unload_angle += math.degrees(pending / 1000.) * .1
                pending = max(0, pending - self.ANGULAR_STEP_MRADPS)
            if sign * psi + outward_rate * horizon + unload_angle >= limit - 1e-9:
                return True
        return False

    def _forward_target(self, xx, now, *, heading_only=False, cached_feedback=False,
                        departure_only=False, fixed_reference_deg=None,
                        correction_heading_limit_deg=10.0, obstacle_approach=False):
        self.refresh_yaw_rate(now)
        self._consume_pending_means(now)
        pose = self._poses[-1] if self._poses else None
        fixed = fixed_reference_deg is not None
        if fixed and (isinstance(fixed_reference_deg, bool)
                      or not isinstance(fixed_reference_deg, (int, float))
                      or not math.isfinite(fixed_reference_deg)):
            raise ValueError("固定航向参考必须为有限角度")
        departure_only = departure_only or fixed
        if (pose is None or self._road_reference is None or self._state_at is None
                or not 0 <= now - self._state_at <= config.TELEMETRY_STALE_MS / 1000.0 + 1e-9
                or not departure_only and not self.control_available(now)):
            return sequence.CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(),
                                            self._effective_yaw_rate(), self.axis_trusted, False)
        controls_fresh = all(item is not None and 0 <= now - item[0] <= .250 + 1e-9
                             for item in self._control_cache)
        cached = not departure_only and (cached_feedback or not controls_fresh)
        if cached and not self._normal_motion_qualified:
            return sequence.CenteringTarget(0, 0, "WAIT", self._error, self._heading_error(),
                                            self._effective_yaw_rate(), self.axis_trusted, False)
        limit = min(self.HEADING_LIMIT_DEG, self.HEADING_LIMIT_DEG
                    if correction_heading_limit_deg is None else abs(correction_heading_limit_deg))
        reference = self._angle(fixed_reference_deg) if fixed else self._road_reference
        psi = self._angle(pose.yaw - reference)
        road_psi = self._angle(pose.yaw - self._road_reference)
        rate, error = self._effective_yaw_rate(now), self._error
        recovery = self.heading_recovery_active
        desired, phase = 0.0, "RETURN" if recovery else "HOLD"
        if not (heading_only or departure_only or cached or recovery):
            position = math.copysign(max(0., abs(error or 0.) - 2.5), error or 0.)
            # Desired heading is expressed in the fixed road frame. Adding
            # the current heading here would cancel the very heading return
            # needed when the car is centered but tilted.
            desired = math.degrees(math.atan2(
                position + self.TREND_HORIZON_S * self._error_rate, self.LOOKAHEAD_MM))
            phase = "SHIFT" if abs(self._raw_error or 0.) > 2.5 else "HOLD"
        desired = max(-limit, min(limit, desired))
        capture = (not heading_only and not departure_only and not cached and not recovery
                   and abs(self._raw_error or 0.) <= 2.5 + 1e-9
                   and abs(self._error_rate) <= .5 + 1e-9
                   and abs(road_psi) <= .2 + 1e-9
                   and abs(rate or 0.) <= .5 + 1e-9)
        requested = (0.0 if capture else
                     (4.0 * math.radians(desired - psi) - .2 * math.radians(rate or 0.)) * 1000.)
        # Keep the requested differential inside the final common-speed domain.
        # A newly committed approach cap takes priority over an old faster
        # steering envelope. Ordinary steering retains its successful-send slew.
        approach_limit = self.ANGULAR_LIMIT_MRADPS
        if obstacle_approach:
            approach_limit = max((value for value in range(0, self.ANGULAR_LIMIT_MRADPS + 1, 16)
                                  if math.trunc(value * sequence.WHEEL_BASE_MM / 2000)
                                  <= min(xx - 61, 310 - xx)), default=0)
            requested = max(-approach_limit, min(approach_limit, requested))
        confirmed = (self._applied_linear > 0
                     and sequence._motion_wheels(self._applied_linear, self._applied_angular)
                     == self._last_sent_wheels)
        angular, residual, residual_sign = self._fine_angular_proposal(
            requested, now, enabled=confirmed and not capture and not cached)
        if obstacle_approach and abs(angular) > approach_limit:
            angular = max(-approach_limit, min(approach_limit, angular))
            residual, residual_sign = 0., 0
        quantized = angular
        angular = self._slew_angular(quantized, now)
        if angular != quantized:
            residual, residual_sign = 0., 0
        if self._anticipate_heading_return(angular, now, road_psi, rate, limit):
            desired, phase, recovery = 0.0, "RETURN", True
            requested = (-4.0 * math.radians(road_psi) - .2 * math.radians(rate or 0.)) * 1000.
            if obstacle_approach:
                requested = max(-approach_limit, min(approach_limit, requested))
            angular, residual, residual_sign = self._fine_angular_proposal(
                requested, now, enabled=confirmed and not cached)
            if obstacle_approach and abs(angular) > approach_limit:
                angular = max(-approach_limit, min(approach_limit, angular))
                residual, residual_sign = 0., 0
            quantized = angular
            angular = self._slew_angular(quantized, now)
            if angular != quantized:
                residual, residual_sign = 0., 0
        linear, angular, reason, state, estimate = self._speed_proposal(
            xx, now, angular, cruise=True, frozen=cached, departure=departure_only)
        if obstacle_approach:
            linear = min(xx, max(61, round(state["_ramp_speed"])))
            final_limit = max((value for value in range(0, self.ANGULAR_LIMIT_MRADPS + 1, 16)
                               if math.trunc(value * sequence.WHEEL_BASE_MM / 2000)
                               <= min(linear - 61, 310 - linear)), default=0)
            if abs(angular) > final_limit:
                angular = max(-final_limit, min(final_limit, angular))
                residual, residual_sign = 0., 0
            state["_ramp_speed"] = float(linear)
            state["_speed_trim"] = 0.0
            reason = "障碍接近：基准上限优先，差速保持逐轮合法；实测速仅记录"
        state.update(_stage2_heading_recovery=recovery, _stage2_correction_phase=phase,
                     _position_integral_deg=0.0, _position_integral_key=None,
                     _position_integral_source_at=None, _hold_heading=self._road_reference,
                     _hold_locked=False, _angular_residual_mradps=residual,
                     _angular_residual_request_sign=residual_sign)
        state["_control_feedback"] = dict(
            raw_difference_mm=self._last_raw_difference,
            raw_half_difference_mm=self._raw_error, filtered_half_difference_mm=error,
            fixed_reference_deg=self._road_reference, hold_heading_deg=reference,
            desired_heading_deg=desired, actual_heading_offset_deg=road_psi,
            yaw_rate_degps=rate, selected_linear_mmps=linear, selected_angular_mradps=angular,
            cached_feedback=cached, correction_phase=phase, heading_recovery_active=recovery,
            correction_heading_limit_deg=limit, position_integral_limit_deg=0.,
            position_control=None if heading_only or departure_only or cached else dict(
                laser_error_mm=error, laser_error_rate_mmps=self._error_rate,
                trend_horizon_s=self.TREND_HORIZON_S, source_at=self._pair_at,
                source_keys=tuple(self._paired_keys)),
            basis="阶段二原始左右差与趋势；固定参考±10°，成功发令每100ms最多32mrad/s")
        normal_allowed = bool(not departure_only and (cached and self._normal_motion_qualified
                              or not cached and self._usable_pair))
        proposal = dict(epoch=self._motion_epoch, token=self.observation_token, at=now, state=state,
                        identity=self._state_session, departure=departure_only, cached=cached,
                        normal=normal_allowed, qualifies=normal_allowed, capture_hold=False,
                        adjustment=angular != 0)
        mode = ("HEADING_FIXED" if fixed else "HEADING_DEPARTURE" if heading_only and departure_only
                else "HEADING_CACHE" if heading_only and cached else "HEADING_ONLY" if heading_only
                else "DEPARTURE" if departure_only else "CACHE" if cached else "CRUISE_CORRECT")
        return sequence.CenteringTarget(
            int(linear), int(angular), mode, error, road_psi, rate, self.axis_trusted,
            self.centered and not heading_only and not cached and not departure_only,
            proposal, reason, estimate, state["_speed_qualified"], normal_allowed)

    def target(self, xx, now, *, cached_feedback=False, departure_only=False,
               correction_heading_limit_deg=10.0, obstacle_approach=False):
        return self._forward_target(xx, now, cached_feedback=cached_feedback,
                                    departure_only=departure_only,
                                    correction_heading_limit_deg=correction_heading_limit_deg,
                                    obstacle_approach=obstacle_approach)

    def target_heading_only(self, xx, now, *, cached_feedback=False, departure_only=False,
                            fixed_reference_deg=None, obstacle_approach=False):
        return self._forward_target(xx, now, heading_only=True, cached_feedback=cached_feedback,
                                    departure_only=departure_only, fixed_reference_deg=fixed_reference_deg,
                                    obstacle_approach=obstacle_approach)

    def confirm_target_sent(self, target):
        return sequence.CenteringController.confirm_target_sent(self, target)

    def notify_target_applied(self, linear_mmps, angular_mradps, applied_at=None, *, source_at=None):
        if (linear_mmps == 0 and angular_mradps
                and getattr(self, "_stage2_remaining_turn_active", False)):
            # This confirmed pivot belongs to the saved turn endpoint, not to
            # ordinary forward centering. Its wire and wheel proof stay in the
            # command ledger and the turn's response tracker.
            return
        sequence.CenteringController.notify_target_applied(
            self, linear_mmps, angular_mradps, applied_at, source_at=source_at)

    def notify_stopped(self, now, *, stable=True):
        steering = self._last_sent_angular, self._last_sent_at
        sequence.CenteringController.notify_stopped(self, now, stable=stable)
        if stable:
            self._stage2_heading_recovery = False
            self._stage2_correction_phase = "HOLD"
        else:
            # Requesting BRAKE is not a proof that the preceding turn rate has
            # vanished. Only the strict stop barrier resets steering authority.
            self._last_sent_angular = self._last_angular = steering[0]
            self._last_sent_at = steering[1]

    def _adopt_reference(self, reference, now, *, moving):
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in (reference, now)):
            return False
        self.refresh_yaw_rate(now)
        post_turn_stopped = bool(not moving and self._stopped and self._applied_linear == 0
                                 and self._last_sent_wheels == (0, 0))
        if (self.heading_recovery_active and not post_turn_stopped
                or self._state_session is None or None in self._state_session
                or self._road_reference is None or moving and not self.centered):
            return False
        self._road_reference = self._hold_heading = self._angle(float(reference))
        self._initial_reference_fixed = True
        self._stage2_heading_recovery = False
        self._stage2_correction_phase = "HOLD"
        self._hold_locked = False
        self._motion_epoch += 1
        self._position_integral_deg = 0.0
        self._position_integral_key = self._position_integral_source_at = None
        self._good_windows = 0
        self._mean_last_end = None
        self._mean_source_keys = (None, None)
        self._pending_means.clear()
        self._compensated_sources.clear()
        self._last_mean_difference = self._last_mean_heading = None
        self._mean_reason = "1秒航向均值已采纳；原始激光源时间和转向阶梯保持"
        self._control_feedback = None
        return True

    def adopt_moving_reference(self, mean_deg, now):
        return self._adopt_reference(mean_deg, now, moving=True)

    def adopt_post_turn_reference(self, mean_deg, now):
        return self._adopt_reference(mean_deg, now, moving=False)

    def position_feedback_available(self, now=None):
        return bool(self.stage2_link is not None and _stage2_fresh_position_feedback(
            self.stage2_link, self, time.monotonic() if now is None else now))

    @property
    def centered(self):
        return bool(not self.heading_recovery_active and self.position_feedback_available()
                    and sequence.CenteringController.centered.fget(self))

    def target_send_ready(self, target, now):
        proposal = target._proposal or {}
        if (self.heading_recovery_active
                and not proposal.get("state", {}).get("_stage2_heading_recovery")):
            return False
        changed = target.angular_mradps != self._last_sent_angular
        if (abs(target.angular_mradps - self._last_sent_angular) > self.ANGULAR_STEP_MRADPS
                or changed and self._last_sent_at is not None
                and now - self._last_sent_at < .1 - 1e-9):
            return False
        if (not (target._proposal or {}).get("departure")
                and not self.position_feedback_available(now)):
            return False
        return sequence.CenteringController.target_send_ready(self, target, now)

    def stage_snapshot(self):
        result = sequence.CenteringController.stage_snapshot(self)
        result.update(position_feedback_available=self.position_feedback_available(),
                      heading_recovery_active=self.heading_recovery_active,
                      correction_phase=self._stage2_correction_phase,
                      correction_heading_limit_deg=self.HEADING_LIMIT_DEG,
                      position_integral_limit_deg=0.)
        return result


class Stage2Display(PatrolDisplay):
    """Stage-one source handling with the local stage-two turn/link policy."""

    def __init__(self, link, emitter=None):
        super().__init__(link, emitter=emitter)
        self.controller = Stage2CenteringController(link=link)

    def print_snapshot(self):
        if self._snapshot_text is not None:
            runlog.emit("stage2_detail", self._snapshot_text)


class _Stage2VisionInterrupt(Exception):
    """A stopped recovery can yield to a latched visual stop decision."""


class _Stage2Input:
    def __init__(self, runner):
        self.runner = runner

    def poll(self):
        self.runner._checkpoint()
        return None


def _stage2_value(snapshot, name, default=None):
    return snapshot.get(name, default) if isinstance(snapshot, dict) else getattr(snapshot, name, default)


def _stage2_brake_context_data(link, display, snapshot, *, reason, road=None, phase=None, returning=False):
    """Freeze original motion evidence without logging or peripheral work."""
    now = time.monotonic()
    controller = getattr(display, "controller", None)
    source_times = getattr(controller, "latest_source_times", None)
    captured = _stage2_value(snapshot, "last_successful_capture_at",
                             _stage2_value(snapshot, "captured_at"))
    valid_capture = (not isinstance(captured, bool) and isinstance(captured, (int, float))
                     and math.isfinite(captured))
    return dict(reason=reason,
                road=road, phase=phase, returning=returning,
                control_mode=getattr(getattr(display, "centering_target", None), "mode", None),
                sent_wheel_targets=tuple(getattr(display, "wheel_targets", (0, 0))),
                measured_wheel_speeds=getattr(link, "wheel_speeds", None),
                laser_source_times=source_times,
                laser_source_ages_s=None if source_times is None else tuple(now - at for at in source_times),
                visual_frame_age_s=now - captured if valid_capture else None,
                visual_sequence=_stage2_value(snapshot, "last_successful_sequence",
                                              _stage2_value(snapshot, "sequence")),
                visual_availability=_stage2_value(snapshot, "availability_kind"),
                candidate_hits=_stage2_value(snapshot, "candidate_hits", 0),
                candidate_active=bool(_stage2_value(snapshot, "candidate_active", False)),
                obstacle_confirmed=_stage2_value(snapshot, "confirmed") is not None)


def _stage2_brake_context(link, display, snapshot, *, reason, road=None, phase=None, returning=False):
    runlog.emit("stage2_brake_requested", "阶段二请求严格停车",
                **_stage2_brake_context_data(link, display, snapshot, reason=reason,
                                            road=road, phase=phase, returning=returning))


def _stage2_authorize_camera_stopped(link, stationary):
    source = getattr(link, "stage2_camera_source", None)
    if source is None:
        source = getattr(getattr(link, "stage2_vision", None), "camera", None)
    grant = getattr(source, "authorize_stationary", None)
    if callable(grant):
        grant(stationary, barrier=getattr(link, "_stable_barrier", None))


class _Stage2ParkedRecovery:
    """Retain one fault/parking anchor across every legal recovery attempt."""

    def __init__(self, link, display, input_source=None):
        self.link, self.display, self.input_source = link, display, input_source
        self.original_fault = self.anchor = None
        self.saved_mode = self.saved_reference = self.saved_pwm = None
        self.world_yaw = None
        self.attempts = 0
        self.blocked_reason = None
        self.stage = "BRAKE"
        self._resumable = bool(getattr(link, "stage2_resumable_parked_recovery", False))
        self._checkpoint = 0
        self._checkpoint_identity = None
        self._raw_yaw_shift = None
        self._clock_complete = False

    def can_recover(self):
        link = self.link
        if getattr(link, "stage2_recovery_block_reason", None) is not None:
            return False
        if self.original_fault is not None and self.stage != "COMPLETE":
            try:
                self._check_secondary_fault()
            except mcu.McuError:
                return False
        timeout = getattr(link, "stage2_confirmation_timeout", None)
        state, diag = getattr(link, "state", None), getattr(link, "diagnostics", None)
        bits = getattr(state, "fault_bits", 0)
        encoder = (state is not None and diag is not None
            and bool(bits & (1 << 20))
            and not bits & ~(sequence._ignored_runtime_fault_bits(link) | (1 << 20))
            and getattr(diag, "last_stop_reason", None) is sequence.WireSafetyStopReason.ENCODER_FAULT
            and (getattr(diag, "session_nonce", None), getattr(diag, "boot_id", None))
            == (link.session_nonce, link.boot_id)
            and link._sample_is_fresh("diagnostics"))
        if self.original_fault is not None and self.stage != "COMPLETE":
            return True
        turn_failure = getattr(link, "stage2_turn_failure", None)
        turn = (turn_failure is not None
                and tuple(turn_failure["identity"]) == (link.session_nonce, link.boot_id)
                and not sequence._active_runtime_fault_bits(link)
                and link.stage2_committed_turn_active)
        return bool(encoder or turn or timeout is not None
                    and timeout["identity"] == (link.session_nonce, link.boot_id))

    def _capture(self):
        if self.stage == "COMPLETE":
            self.original_fault = self.anchor = None
            self.blocked_reason = None
            self.stage = "BRAKE"
            self._checkpoint = 0
            self._checkpoint_identity = None
            self._raw_yaw_shift = None
            self._clock_complete = False
        if self.original_fault is not None:
            return
        if not self.can_recover():
            raise mcu.McuError(self.link.motion_health_reason() or "当前故障不属于可恢复确认超时或编码器锁存")
        link = self.link
        self.original_fault = dict(
            identity=(link.session_nonce, link.boot_id), reason=link._hard_fault_reason,
            confirmation=getattr(link, "stage2_confirmation_timeout", None),
            turn_failure=getattr(link, "stage2_turn_failure", None),
            fault_bits=getattr(link.state, "fault_bits", None))
        self.saved_mode = link.stage2_mode
        self.saved_reference = self.display.controller.heading_reference_deg
        self.saved_pwm = getattr(link, "stage2_confirmed_pwm", None)

    def _check_continuity(self):
        if self.anchor is None or self.link.state is None or not self.link._sample_is_fresh("state"):
            return
        link = self.link
        if link.boot_id != self.anchor["boot_id"]:
            self.blocked_reason = "MCU复位，世界航向与轮程连续性无法恢复，保持停车"
        elif link.odom_mm is not None and link.odom_mm != self.anchor["positions"]:
            self.blocked_reason = "HELLO前后双轮累计轮程改变，停车坐标连续性无法恢复，保持停车"
        if self.blocked_reason is not None:
            link.stage2_world_yaw_unknown = True
            raise mcu.McuError(self.blocked_reason)

    def _check_secondary_fault(self):
        link = self.link
        block = getattr(link, "stage2_recovery_block_reason", None)
        if block is not None:
            raise mcu.McuError(block)
        if (link.state is not None and link.state.fault_bits
                & ~(sequence._ignored_runtime_fault_bits(link) | (1 << 20))):
            raise mcu.McuError("停车恢复出现新的真实固件故障")
        if (getattr(link, "last_parse_error", None) or "").startswith("READ_ERROR:"):
            raise mcu.McuError(link.last_parse_error)
        diag = getattr(link, "diagnostics", None)
        if (diag is not None and (diag.session_nonce, diag.boot_id) == (link.session_nonce, link.boot_id)
                and link._sample_is_fresh("diagnostics")):
            status = int(diag.gyro_status)
            if status in (1, 5, 6):
                raise mcu.McuError("停车恢复出现真实陀螺仪故障：" + diag.gyro_status.name)
            saved_turn = None if self.original_fault is None else self.original_fault.get("turn_failure")
            source = None if saved_turn is None else saved_turn.get("source_at")
            history = None if saved_turn is None else saved_turn.get("diagnostic_history")
            identity = link.session_nonce, link.boot_id
            original_identity = None if self.original_fault is None else tuple(self.original_fault["identity"])
            pure_turn_failure = (saved_turn is not None and history is not None
                and tuple(saved_turn.get("identity", ())) == original_identity
                and original_identity[1] == identity[1]
                and (identity == original_identity or self._checkpoint >= 1
                     and self._checkpoint_identity == identity)
                and not isinstance(source, bool) and isinstance(source, (float, int))
                and math.isfinite(source) and source <= time.monotonic()
                and type(saved_turn.get("applied_sequence")) is int
                and 0 <= saved_turn["applied_sequence"] <= 0xffff
                and tuple(history.get("identity", ())) == original_identity
                and not isinstance(history.get("source_at"), bool)
                and isinstance(history.get("source_at"), (float, int))
                and math.isfinite(history["source_at"]) and history["source_at"] <= source + 1e-9
                and history.get("reason") == diag.last_stop_reason
                and link.stage2_committed_turn_active
                and not sequence._active_runtime_fault_bits(link)
                and sequence._gyro_health_reason(link) is None)
            if diag.last_stop_reason not in (sequence.WireSafetyStopReason.NONE,
                    sequence.WireSafetyStopReason.BOOT, sequence.WireSafetyStopReason.HOST,
                    sequence.WireSafetyStopReason.LEASE_EXPIRED, sequence.WireSafetyStopReason.ENCODER_FAULT) and not pure_turn_failure:
                raise mcu.McuError("停车恢复出现真实下位机故障：" + diag.last_stop_reason.name)
        hard = getattr(link, "_hard_fault_reason", None)
        if hard is not None and self.original_fault is not None:
            known = hard == self.original_fault["reason"] or hard in link._stage2_timeout_reasons
            diag, state = getattr(link, "diagnostics", None), getattr(link, "state", None)
            encoder = (state is not None and state.fault_bits & (1 << 20) and diag is not None
                and diag.last_stop_reason is sequence.WireSafetyStopReason.ENCODER_FAULT
                and (diag.session_nonce, diag.boot_id) == (link.session_nonce, link.boot_id)
                and link._sample_is_fresh("diagnostics"))
            if not known and not encoder:
                raise mcu.McuError(hard)

    def _drain_parked_auxiliary(self, *, states=False):
        # Parked samples cannot create new road/opening authority. Draining
        # them also keeps unlimited retries from overflowing receive queues.
        names = ("drain_side_samples", "drain_diagnostics_samples")
        if states:
            names += ("drain_state_samples", "drain_wheel_trace_samples")
        for name in names:
            drain = getattr(self.link, name, None)
            if callable(drain):
                drain()

    def park(self):
        """One bounded BRAKE + genuine 500 ms stop observation, never HELLO."""
        link, display = self.link, self.display
        previous_hook = link.stage2_runtime_poll
        previous_stopping = link.stage2_fault_stopping
        link.stage2_runtime_poll = lambda: sequence._check_input_exit(self.input_source)
        link.stage2_gimbal_monitor = None
        link._turn_direction_guard = None
        link.stage2_fault_stopping = True
        link.set_stage2_mode("stopped")
        try:
            self.stage = "BRAKE"
            deadline = time.monotonic() + 5.0
            try:
                sequence._apply_brake(link, display, deadline)
            except mcu.McuError as exc:
                key = MessageType.MOTION_ACTION, link._last_brake_sequence
                link._retire_failed_recovery_zero(key, exc)
                raise
            self.stage = "STATIONARY"
            observer = FaultStopTracker(link, link._last_brake_sequence)
            while True:
                sequence._check_input_exit(self.input_source)
                sequence._remaining(deadline, "停车恢复：5秒内未取得有效连续500ms静止证据，保持停车")
                link.pump()
                self._check_secondary_fault()
                anchor = observer.update()
                self._drain_parked_auxiliary()
                if anchor is not None:
                    link._stop_tracker = observer.tracker
                    link._stable_barrier = time.monotonic()
                    _stage2_authorize_camera_stopped(link, True)
                    return anchor
                time.sleep(sequence.CONTROL_INTERVAL_S)
        finally:
            link.stage2_runtime_poll = previous_hook
            link.stage2_fault_stopping = previous_stopping

    def attempt(self):
        if self._resumable:
            return self._attempt_resumable()
        self._capture()
        if self.blocked_reason is not None:
            raise mcu.McuError(self.blocked_reason)
        self._check_secondary_fault()
        self._check_continuity()
        link, display = self.link, self.display
        previous_hook = link.stage2_runtime_poll
        previous_stopping = link.stage2_fault_stopping
        link.stage2_runtime_poll = lambda: sequence._check_input_exit(self.input_source)
        link.stage2_gimbal_monitor = None
        link.stage2_fault_stopping = True
        link.stage2_recovery_active = True
        try:
            # Before the first HELLO only an exact accepted/applied BRAKE and
            # fresh stationary observation may establish the permanent anchor.
            if self.anchor is None:
                self.anchor = self.park()
                self.world_yaw = link.stage2_world_yaw_deg
                if self.world_yaw is None:
                    self.anchor = None
                    raise mcu.McuError("恢复缺少连续世界航向，保持停车")
            elif link.connected and link.state is not None:
                self.park()
                self._check_continuity()
            self._check_secondary_fault()
            self.attempts += 1
            self.stage = "HELLO"
            runlog.emit("stage2_control_recovery", "已确认零输出与500ms静止，合法HELLO恢复原任务",
                        attempt=self.attempts, anchor=self.anchor, original_fault=self.original_fault,
                        saved_pwm=self.saved_pwm, world_yaw_deg=self.world_yaw)
            link.hello(timeout_s=1.0)
            self.stage = "ZERO_SYNC"
            link.stage2_world_yaw_unknown = True
            link.synchronize_clock()
            self._check_continuity()
            deadline = time.monotonic() + config.MCU_SELFCHECK_READY_TIMEOUT_S
            while True:
                sequence._check_input_exit(self.input_source)
                link.pump()
                self._drain_parked_auxiliary(states=True)
                self._check_continuity()
                self._check_secondary_fault()
                reason = link.motion_health_reason() or sequence._gyro_health_reason(link)
                if (reason is None and link.state is not None and link.yaw_deg is not None
                        and not link.state.target_linear_mms and not link.state.target_angular_mrads):
                    break
                sequence._remaining(deadline, "HELLO后未取得当前新鲜健康零输出遥测，保持停车")
                time.sleep(sequence.CONTROL_INTERVAL_S)
            raw = link.yaw_deg
            shift = sequence._normalize_yaw(raw - self.anchor["raw_yaw"])
            link.stage2_world_yaw_offset = self.world_yaw - raw
            link._stage2_world_yaw, link._stage2_previous_raw_yaw = self.world_yaw, raw
            link.stage2_world_yaw_unknown = False
            self.park()
            self.stage = "PWM"
            if self.saved_pwm is not None:
                _stage2_confirm_parked_pwm(link, self.saved_pwm, self.input_source)
            # A gimbal command changes the applied sequence; acquire the final
            # BRAKE proof again so a saved partial turn can pass its write gate.
            self.park()
            self._check_continuity()
            self._check_secondary_fault()
            if self.saved_pwm is not None:
                pwm_reason = link._stage2_gimbal_reason(link.state, int(self.saved_pwm[0]),
                                                       (link.session_nonce, link.boot_id))
                if pwm_reason is not None:
                    link._stage2_gimbal_failed(pwm_reason)
            barrier = link._stable_barrier
            display.reset_averages(source_barrier=barrier)
            if self.saved_reference is not None:
                display.controller.set_initial_reference(sequence._normalize_yaw(self.saved_reference + shift))
            link.set_stage2_mode(self.saved_mode)
            self.stage = "COMPLETE"
            link.stage2_recovery_active = False
            return dict(barrier=barrier, raw_yaw_shift=shift, world_yaw_deg=self.world_yaw,
                        saved_mode=self.saved_mode, saved_reference=self.saved_reference,
                        saved_pwm=self.saved_pwm, session=(link.session_nonce, link.boot_id))
        finally:
            link.stage2_runtime_poll = previous_hook
            link.stage2_fault_stopping = previous_stopping

    def _can_resume_checkpoint(self):
        """A completed step can be reused only in its fresh, stationary session."""
        link = self.link
        state = link.state
        return bool(self._checkpoint and link.connected and state is not None
                    and self._checkpoint_identity == (link.session_nonce, link.boot_id)
                    and link._sample_is_fresh("state")
                    and not state.target_linear_mms and not state.target_angular_mrads
                    and self.anchor is not None and link.odom_mm == self.anchor["positions"]
                    and not sequence._active_runtime_fault_bits(link, state))

    def _attempt_resumable(self):
        """Resume confirmed HELLO/PWM phases while keeping the final BRAKE gate."""
        self._capture()
        if self.blocked_reason is not None:
            raise mcu.McuError(self.blocked_reason)
        self._check_secondary_fault()
        self._check_continuity()
        link, display = self.link, self.display
        previous_hook = link.stage2_runtime_poll
        previous_stopping = link.stage2_fault_stopping
        link.stage2_runtime_poll = lambda: sequence._check_input_exit(self.input_source)
        link.stage2_gimbal_monitor = None
        link.stage2_fault_stopping = True
        link.stage2_recovery_active = True
        try:
            if self.anchor is None:
                self.anchor = self.park()
                self.world_yaw = link.stage2_world_yaw_deg
                if self.world_yaw is None:
                    self.anchor = None
                    raise mcu.McuError("恢复缺少连续世界航向，保持停车")
            elif not self._can_resume_checkpoint():
                self._checkpoint = 0
                self._checkpoint_identity = None
                self._clock_complete = False
                if link.connected and link.state is not None:
                    self.park()
                    self._check_continuity()
            self.attempts += 1
            if self._checkpoint == 0:
                self.stage = "HELLO"
                runlog.emit("stage2_control_recovery", "停车恢复HELLO阶段",
                            attempt=self.attempts, anchor=self.anchor,
                            original_fault=self.original_fault, saved_pwm=self.saved_pwm)
                link.hello(timeout_s=1.0)
                self._checkpoint_identity = (link.session_nonce, link.boot_id)
                self._checkpoint = 1
            if self._checkpoint == 1:
                self.stage = "ZERO_SYNC"
                link.stage2_world_yaw_unknown = True
                if not self._clock_complete:
                    try:
                        link.synchronize_clock()
                    except mcu.McuError:
                        # Each HELLO nonce permits one clock probe. A failed
                        # probe needs a new legal HELLO; it cannot be replayed.
                        self._checkpoint = 0
                        self._checkpoint_identity = None
                        self._clock_complete = False
                        raise
                    self._clock_complete = True
                self._check_continuity()
                deadline = time.monotonic() + config.MCU_SELFCHECK_READY_TIMEOUT_S
                while True:
                    sequence._check_input_exit(self.input_source)
                    link.pump()
                    self._drain_parked_auxiliary(states=True)
                    self._check_continuity()
                    self._check_secondary_fault()
                    reason = link.motion_health_reason() or sequence._gyro_health_reason(link)
                    if (reason is None and link.state is not None and link.yaw_deg is not None
                            and not link.state.target_linear_mms and not link.state.target_angular_mrads):
                        break
                    sequence._remaining(deadline, "HELLO后未取得当前新鲜健康零输出遥测，保持停车")
                    time.sleep(sequence.CONTROL_INTERVAL_S)
                raw = link.yaw_deg
                self._raw_yaw_shift = sequence._normalize_yaw(raw - self.anchor["raw_yaw"])
                link.stage2_world_yaw_offset = self.world_yaw - raw
                link._stage2_world_yaw, link._stage2_previous_raw_yaw = self.world_yaw, raw
                link.stage2_world_yaw_unknown = False
                self._checkpoint = 2
            if self._checkpoint == 2:
                self.park()
                self._check_continuity()
                self._checkpoint = 3
            if self._checkpoint == 3:
                self.stage = "PWM"
                if self.saved_pwm is not None:
                    _stage2_confirm_parked_pwm(link, self.saved_pwm, self.input_source)
                self._checkpoint = 4
            if self._checkpoint == 4:
                # Mandatory even when the PWM phase was completed on an earlier
                # attempt: only this BRAKE owns the resumed motion's stop proof.
                self.park()
                self._checkpoint = 5
            self._check_continuity()
            self._check_secondary_fault()
            if self.saved_pwm is not None:
                reason = link._stage2_gimbal_reason(link.state, int(self.saved_pwm[0]),
                                                   (link.session_nonce, link.boot_id))
                if reason is not None:
                    link._stage2_gimbal_failed(reason)
            barrier = link._stable_barrier
            display.reset_averages(source_barrier=barrier)
            shift = self._raw_yaw_shift
            if self.saved_reference is not None:
                display.controller.set_initial_reference(sequence._normalize_yaw(self.saved_reference + shift))
            link.set_stage2_mode(self.saved_mode)
            self.stage = "COMPLETE"
            link.stage2_recovery_active = False
            runlog.emit("stage2_control_recovery_complete", "停车恢复已续接完成",
                        attempts=self.attempts, session=self._checkpoint_identity,
                        final_stop_barrier=barrier, saved_pwm=self.saved_pwm)
            return dict(barrier=barrier, raw_yaw_shift=shift, world_yaw_deg=self.world_yaw,
                        saved_mode=self.saved_mode, saved_reference=self.saved_reference,
                        saved_pwm=self.saved_pwm, session=self._checkpoint_identity)
        finally:
            link.stage2_runtime_poll = previous_hook
            link.stage2_fault_stopping = previous_stopping

    def recover(self):
        self._capture()
        notified = False
        blocked_notified = False
        last_report = None
        while True:
            sequence._check_input_exit(self.input_source)
            try:
                return self.attempt()
            except mcu.McuError as exc:
                # A changed physical coordinate or boot can never establish a
                # replacement origin. Keep the original task parked until q.
                if self.blocked_reason is None:
                    self._check_secondary_fault()
                    if self.stage in ("BRAKE", "STATIONARY"):
                        try:
                            self.link.send_stop(StopReason.FAULT)
                        except mcu.McuError:
                            pass
                report = (self.stage, self.attempts, str(exc), self.blocked_reason is not None)
                if report != last_report:
                    runlog.emit("stage2_control_retry_wait", "恢复证据未齐，保留原任务并保持停车", stage=self.stage,
                                attempt=self.attempts, reason=str(exc), blocked=self.blocked_reason is not None)
                    last_report = report
                if not notified:
                    emitter = getattr(self.display, "emitter", None)
                    if callable(emitter):
                        emitter("控制确认中断；保持停车并自动恢复，输入q结束。")
                    notified = True
                if self.blocked_reason is not None and not blocked_notified:
                    emitter = getattr(self.display, "emitter", None)
                    if callable(emitter):
                        emitter(self.blocked_reason + "；请输入q退出后重新运行。")
                    blocked_notified = True
                sequence._check_input_exit(self.input_source)
                time.sleep(sequence.CONTROL_INTERVAL_S)


class Stage2Runner:
    """A → straight passage → B → one left U-turn → first return opening."""

    def __init__(self, link, display, vision, xx, *, input_source=None, duration_s=None):
        if isinstance(xx, bool) or not isinstance(xx, int) or not 61 <= xx <= 310:
            raise ValueError("阶段二直行速度必须是61～310整数")
        if duration_s is not None and (isinstance(duration_s, bool)
                or not isinstance(duration_s, (int, float)) or not math.isfinite(duration_s)
                or duration_s <= 0):
            raise ValueError("duration必须是有限正数")
        if not isinstance(display.controller, PatrolCenteringController):
            raise TypeError("阶段二需要PatrolCenteringController")
        self.link, self.display, self.vision = link, display, vision
        self.xx, self.input_source, self.duration_s = xx, input_source, duration_s
        self.deadline, self.started_at = math.inf, None
        self.phase, self.road, self.returning, self.armed = "ALIGN", "A", False, False
        self.sampler = MovingHeadingSampler()
        self.reference_fixed = False
        self.cutoff_reference = None
        self.centering_cutoff = False
        self.passage_barrier = display.controller.source_barrier
        self.pending_wall = None
        self._deferred_openings = deque()
        self.visual_confirmed = None
        self.visual_timeout = False
        self.visual_candidate = False
        self._candidate_started_sequence = None
        self._candidate_pending = False
        self._candidate_raw_active = False
        self._candidate_last_active = False
        self.visual_snapshot = None
        self._waiting_recovery = False
        self._safety_stop = False
        self.sent_at, self.next_send = None, time.monotonic()
        self.progress = sequence._RoadProgress()
        self.response = sequence._WheelResponseTracker(link, self.next_send)
        self._runtime_input = _Stage2Input(self)
        self._previous_observer = display._distance_observer
        self._previous_runtime_poll = getattr(link, "stage2_runtime_poll", None)
        self._previous_visual_gate = getattr(link, "stage2_visual_stop_pending", None)
        self._last_passage_key = None
        self._pending_next_road = False
        self.detector = PatrolJunctionDetector()
        self.detector.reset(None, None, (link.session_nonce, link.boot_id), require_window=False)
        self.junction_evidence = None
        self.first_wall_source = None
        self._localization_samples = deque(maxlen=256)
        self._visual_generation = None
        self._visual_epoch = None
        self._visual_health_token = None
        self._vision_pause_pending = False
        self._vision_pause_reason = None
        self._vision_waiting = False
        self._turn_motion_active = False
        self._turn_decision = None
        self._turn_stop_notified = False
        self._previous_health_gate = getattr(link, "stage2_visual_health_gate", None)
        self._previous_snapshot_gate = getattr(link, "stage2_visual_snapshot_gate", None)
        self._approach_commands = {}
        self._loop_initialized = False
        self._turn_goal_world = None
        self._turn_road_reference_world = None
        self._encoder_recovery_count = 0
        self._control_recovery = None
        self._visual_snapshot_busy = False
        self._pending_visual_suspension = False
        self._turn_stop_ready = False
        self._return_finish_evidence = None
        self._post_turn_front_barrier = None

    def _known_encoder_fault(self):
        state, diag = getattr(self.link, "state", None), getattr(self.link, "diagnostics", None)
        return (self._encoder_fault_candidate() and diag is not None
                and getattr(diag, "last_stop_reason", None) is sequence.WireSafetyStopReason.ENCODER_FAULT
                and self.link._sample_is_fresh("diagnostics")
                and (diag.session_nonce, diag.boot_id) == (self.link.session_nonce, self.link.boot_id))

    def _encoder_fault_candidate(self):
        state = getattr(self.link, "state", None)
        hard = getattr(self.link, "_hard_fault_reason", None) or ""
        return (state is not None
                and bool(getattr(state, "fault_bits", 0) & (1 << 20))
                and not getattr(state, "fault_bits", 0) & ~(sequence._ignored_runtime_fault_bits(self.link) | (1 << 20))
                and not (getattr(self.link, "last_parse_error", None) or "").startswith("READ_ERROR:")
                and "非法" not in hard and "boot 变化" not in hard and "被拒" not in hard)

    def _await_encoder_fault_diagnosis(self):
        """STATE may precede DIAG; first make output zero, then classify fresh reason."""
        link = self.link
        fault_source = link._sample_time("state")
        link.stage2_gimbal_monitor = None
        link.stage2_fault_stopping = True
        try:
            sequence._apply_brake(link, self.display, time.monotonic() + 5.0)
            self._encoder_pending_brake = link._last_brake_sequence
            deadline = time.monotonic() + 1.0
            while True:
                sequence._check_input_exit(self.input_source)
                link.pump()
                if not self._encoder_fault_candidate():
                    raise mcu.McuError("停车后故障类型变化，保持停车")
                diag = link.diagnostics
                if (diag is not None and link._sample_is_fresh("diagnostics")
                        and link._sample_time("diagnostics") >= fault_source
                        and (diag.session_nonce, diag.boot_id) == (link.session_nonce, link.boot_id)):
                    if diag.last_stop_reason is sequence.WireSafetyStopReason.ENCODER_FAULT:
                        return
                    raise mcu.McuError("新鲜DIAG并非编码器故障，保持停车")
                sequence._remaining(deadline, "锁存STATE后1秒内缺少新鲜DIAG故障原因，保持停车")
                time.sleep(sequence.CONTROL_INTERVAL_S)
        finally:
            link.stage2_fault_stopping = False

    def _await_turn_abort_diagnosis(self):
        """Park before waiting for the DIAG paired with the saved abort event."""
        link = self.link
        evidence = getattr(link, "_turn_abort_evidence", None)
        if (self._turn_goal_world is None or evidence is None
                or not link.stage2_committed_turn_active or sequence._active_runtime_fault_bits(link)):
            return
        link.stage2_gimbal_monitor = None
        link.stage2_fault_stopping = True
        try:
            sequence._apply_brake(link, self.display, time.monotonic() + 5.0)
            until = time.monotonic() + config.TELEMETRY_STALE_MS / 1000.0
            while time.monotonic() < until:
                sequence._check_input_exit(self.input_source)
                if sequence._active_runtime_fault_bits(link):
                    return
                if sequence._recoverable_turn_abort(link):
                    link.stage2_turn_failure = dict(identity=(link.session_nonce, link.boot_id),
                        reason="转向被下位机中止", source_at=evidence["source_at"],
                        applied_sequence=evidence["state"].applied_control_sequence,
                        diagnostic_history=evidence["record"][10]["turn_diagnostic_history"])
                    return
                link.pump()
                time.sleep(sequence.CONTROL_INTERVAL_S)
        finally:
            link.stage2_fault_stopping = False

    def _resume_control_recovery(self, result):
        link = self.link
        self._encoder_pending_brake = None
        self._encoder_recovery_count += 1
        link.stage2_return = self.returning
        link.side_motion_active = True
        link.side_candidates_enabled = False
        self.sampler.cancel()
        self._approach_commands.clear()
        invalidate = getattr(self.vision, "invalidate_control_candidates", None)
        if callable(invalidate):
            invalidate(result["barrier"])
        self._candidate_started_sequence = None
        self._candidate_pending = self._candidate_raw_active = self._candidate_last_active = False
        self.visual_timeout = self.visual_candidate = False
        self.pending_wall = None
        self._deferred_openings.clear()
        self._localization_samples.clear()
        self.junction_evidence = None
        self.first_wall_source = None
        self.passage_barrier = result["barrier"]
        self.armed = False
        self.reference_fixed = result["saved_reference"] is not None
        self.detector.reset(None, None, (link.session_nonce, link.boot_id), require_window=False)
        self.sent_at, self.next_send = None, time.monotonic()
        self.response = sequence._WheelResponseTracker(link, self.next_send)
        # Include the actual braking tail at the existing origin. HELLO does
        # not create a new road or forgive wheel displacement before parking.
        if link.state is not None:
            self.progress.update(link.state, link._sample_time("state"))
        runlog.emit("stage2_control_resumed", "新会话健康与PWM已确认，接续原道路及原掉头终点",
                    attempt=self._encoder_recovery_count, goal_world=self._turn_goal_world,
                    raw_yaw_shift=result["raw_yaw_shift"], world_yaw_deg=result["world_yaw_deg"],
                    road=self.road, phase=self.phase, returning=self.returning,
                    origin_mm=self.progress.origin_mm, position_mm=self.progress.position_mm)
        self._control_recovery = None
        return result

    def _recover_confirmation_fault(self):
        if self._control_recovery is None:
            self._control_recovery = _Stage2ParkedRecovery(self.link, self.display, self.input_source)
        self._safety_stop = True
        try:
            return self._resume_control_recovery(self._control_recovery.recover())
        finally:
            self._safety_stop = False

    def _recover_encoder_fault(self):
        """One attempt retained for callers; run() owns unlimited parked retry."""
        if self._control_recovery is None:
            if not self._known_encoder_fault():
                raise mcu.McuError(self.link.motion_health_reason() or "当前故障不是可恢复编码器锁存")
            self._control_recovery = _Stage2ParkedRecovery(self.link, self.display, self.input_source)
        self._safety_stop = True
        try:
            return self._resume_control_recovery(self._control_recovery.attempt())
        finally:
            self._safety_stop = False

    def _approach_active(self, snapshot=None):
        snapshot = self.visual_snapshot if snapshot is None else snapshot
        return bool(getattr(self.vision, "obstacle_approach_enabled", False)
                    and _stage2_value(snapshot, "approach_phase", "NORMAL") != "NORMAL")

    def _sync_approach_applied(self, snapshot=None):
        if not getattr(self.vision, "obstacle_approach_enabled", False):
            return
        if snapshot is None:
            read = getattr(self.vision, "try_control_snapshot", None)
            snapshot = read() if callable(read) else self.vision.poll()
            if snapshot is None:
                return
        generation = _stage2_value(snapshot, "approach_generation", 0)
        identity = self.link.session_nonce, self.link.boot_id
        ledger = getattr(self.link, "_command_ledger", {})
        confirmed = getattr(self.link, "_confirmed_commands", {})
        evidence_times = getattr(self.link, "_confirmed_evidence_times", {})
        for key, token in list(self._approach_commands.items()):
            record = ledger.get(key)
            if (record is None or token != (*identity, generation, record[0])
                    or tuple(record[6:8]) != identity
                    or len(record) > 10 and record[10].get("drive_invalidated_by_brake")):
                self._approach_commands.pop(key, None)
                continue
            evidence_at = evidence_times.get(key)
            if (key not in confirmed or evidence_at is None
                    or evidence_at - record[0] > config.MCU_COMMAND_CONFIRM_TIMEOUT_S + 1e-9):
                continue
            witness_key = getattr(self.link, "_superseded_commands", {}).get(key, key)
            witness = ledger.get(witness_key, record)
            source_at = witness[10].get("state_source_at") if len(witness) > 10 else None
            if source_at is None:
                state = confirmed[key]
                reference_raw = getattr(self.link, "_uptime_reference_raw", None)
                offset = getattr(self.link, "_clock_offset_s", None)
                if reference_raw is not None and offset is not None:
                    delta = ((state.uptime_ms - reference_raw + 0x80000000) & 0xFFFFFFFF) - 0x80000000
                    source_at = offset + (getattr(self.link, "_uptime_reference_ms", reference_raw) + delta) / 1000.
            if source_at is not None:
                mark = getattr(self.vision, "try_mark_approach_applied", self.vision.mark_approach_applied)
                if mark(generation, source_at, evidence_at):
                    self._approach_commands.pop(key, None)

    def _visual_health(self, snapshot):
        kind = _stage2_value(snapshot, "availability_kind")
        if kind is None:
            # Existing injected adapters have no last-success contract. They
            # retain their stricter fresh-result gate rather than renewing a
            # grace period from an unqualified frame or retry.
            token = (_stage2_value(snapshot, "generation", 0),
                     _stage2_value(snapshot, "observation_generation", 0))
            captured = _stage2_value(snapshot, "captured_at")
            healthy = (bool(_stage2_value(snapshot, "ready", False))
                       and not _stage2_value(snapshot, "fault_reason")
                       and Stage2Vision._fresh(captured))
            return token, healthy, captured
        token = (None, _stage2_value(snapshot, "runtime_stop_generation", 0))
        captured = _stage2_value(snapshot, "last_successful_capture_at")
        raw = _stage2_value(snapshot, "last_reliable_capture_at", captured)
        healthy = (kind in ("ready", "temporary", "starting")
                   and not _stage2_value(snapshot, "fault_reason")
                   and not isinstance(captured, bool)
                   and isinstance(captured, (float, int)) and math.isfinite(captured)
                   and 0 <= time.monotonic() - captured < STAGE2_RUNTIME_VISION_TIMEOUT_S
                   and not isinstance(raw, bool) and isinstance(raw, (int, float))
                   and math.isfinite(raw)
                   and 0 <= time.monotonic() - raw < STAGE2_RUNTIME_VISION_TIMEOUT_S)
        return token, healthy, captured

    def _visual_pause_cause(self, snapshot, token, healthy):
        previous = self._visual_health_token
        if previous is not None and token != previous:
            if token[0] != previous[0]:
                return "camera_generation_changed"
            return _stage2_value(snapshot, "runtime_stop_reason") or "visual_stop_event"
        if not healthy:
            return (_stage2_value(snapshot, "runtime_stop_reason")
                    or _stage2_value(snapshot, "fault_reason")
                    or "visual_success_source_expired")
        return None

    def _guard_visual_motion(self):
        self._checkpoint()
        if self._pending_visual_suspension and not self._turn_motion_active:
            raise sequence._CenteringProposalExpired("停车后的旧视觉候选尚未撤销，保持停车重试")
        if self._visual_snapshot_busy and not self._turn_motion_active:
            raise sequence._CenteringProposalExpired("视觉快照繁忙，继续接收UART后重新计算目标")
        if self._vision_pause_pending and not self._turn_motion_active:
            raise _Stage2VisionInterrupt()
        if not self._turn_motion_active and not self._post_turn_front_ready(self.visual_snapshot):
            raise sequence._CenteringProposalExpired("转后尚无一张600前向新鲜无候选画面，保持停车")

    def _guard_visual_snapshot(self, snapshot):
        """Final write guard: consume already locked evidence without polling."""
        self._apply_visual_snapshot(snapshot)
        if self._pending_visual_suspension and not self._turn_motion_active:
            raise sequence._CenteringProposalExpired("停车后的旧视觉候选尚未撤销，保持停车重试")
        if not self._turn_motion_active and (
                self._vision_pause_pending or self.visual_confirmed is not None):
            raise _Stage2VisionInterrupt()
        if not self._turn_motion_active and not self._post_turn_front_ready(snapshot):
            raise sequence._CenteringProposalExpired("最终发令前转后清晰画面资格已失效，保持停车")

    def _post_turn_front_ready(self, snapshot):
        barrier = self._post_turn_front_barrier
        if barrier is None:
            return True
        current = _stage2_value(snapshot, "current")
        captured = _stage2_value(snapshot, "captured_at")
        return bool(current is not None and current.available and not current.detected
                    and _stage2_value(snapshot, "ready", False)
                    and not _stage2_value(snapshot, "fault_reason")
                    and not _stage2_value(snapshot, "candidate_active", False)
                    and _stage2_value(snapshot, "confirmed") is None
                    and Stage2Vision._fresh(captured) and captured > barrier)

    def _recover_vision(self):
        """Pause observation without resetting road progress or requiring new walls."""
        previous_phase = self.phase
        committed = self._turn_decision or self.visual_confirmed
        if committed is not None:
            self._turn_decision = committed
        elif not self._turn_motion_active:
            self.visual_confirmed = None
        self._vision_waiting = True
        try:
            if not sequence._assert_still_stable(self.link, self.display):
                self._strict_brake("视觉不可用：BRAKE并严格停稳，等待自动恢复",
                                   reason=self._vision_pause_reason or "visual_unavailable")
            else:
                self.sampler.cancel()
            if committed is None and not self._turn_motion_active:
                discard = getattr(self.vision, "suspend_approach_confirmation" if self._approach_active()
                                  else "discard_pending_decision", None)
                if callable(discard):
                    discard()
            self._notify_stopped_once(
                "已确认障碍并严格停稳；等待视觉恢复后继续掉头。" if committed is not None
                else "视觉不可用，已BRAKE并严格停稳；等待自动恢复。",
                committed=committed is not None)
            _stage2_authorize_camera_stopped(self.link, True)
            self.pending_wall = None
            self._deferred_openings.clear()
            self._action("保持停车等待摄像头与识别恢复；恢复后自动继续，q退出")
            barrier = self.link._stable_barrier
            resume_token = None
            resume_frames = set()
            runlog.emit("stage2_visual_pause", "视觉停车等待恢复", road=self.road,
                        phase=previous_phase, stable_barrier=barrier,
                        health_token=self._visual_health_token)
            while True:
                self._checkpoint()
                self.display.tick()
                self._consume(update=False, means=False)
                if not sequence._assert_still_stable(self.link, self.display):
                    _stage2_authorize_camera_stopped(self.link, False)
                    sequence._wait_stable(self.link, self.display, time.monotonic() + 5.0,
                                          input_source=self._runtime_input)
                    barrier = self.link._stable_barrier
                    resume_frames.clear()
                _stage2_authorize_camera_stopped(self.link, True)
                token, healthy, captured = self._visual_health(self.visual_snapshot)
                if token != resume_token:
                    resume_token = token
                    resume_frames.clear()
                if (healthy and bool(_stage2_value(self.visual_snapshot, "ready", False))
                        and Stage2Vision._fresh(captured) and captured > barrier):
                    sequence_number = _stage2_value(self.visual_snapshot, "last_successful_sequence",
                                                   _stage2_value(self.visual_snapshot, "sequence"))
                    resume_frames.add((sequence_number, captured))
                    if len(resume_frames) < 2:
                        time.sleep(sequence.CONTROL_INTERVAL_S)
                        continue
                    self._visual_health_token = token
                    self._vision_pause_pending = False
                    self._vision_pause_reason = None
                    self.visual_timeout = False
                    self.phase = previous_phase if previous_phase != "SAMPLE" else "ALIGN"
                    self.next_send = time.monotonic()
                    runlog.emit("stage2_visual_resumed", "视觉恢复，按原路线自动继续",
                                road=self.road, phase=self.phase, captured_at=captured,
                                stable_barrier=barrier, health_token=token)
                    emitter = getattr(self.vision, "_status_emitter", None)
                    if emitter is not None:
                        emitter("视觉已恢复；继续待执行的障碍掉头。" if self._turn_decision is not None
                                else "视觉已恢复；按原路线继续。")
                    return
                time.sleep(sequence.CONTROL_INTERVAL_S)
        finally:
            self._vision_waiting = False

    def _checkpoint(self):
        if self.input_source is not None:
            value = self.input_source.poll()
            if isinstance(value, str) and value.strip().lower() == "q":
                raise sequence._UserQuit("收到q，尝试停车")
            if getattr(self.input_source, "eof", False):
                raise sequence._UserQuit("终端输入结束，尝试停车")
        if not self._safety_stop and time.monotonic() >= self.deadline:
            raise mcu.McuError("阶段二运动时限已到，停车结束")
        if self._pending_visual_suspension:
            suspend = getattr(self.vision, "try_suspend_approach_confirmation", None)
            if callable(suspend) and suspend():
                self._pending_visual_suspension = False
        consume = getattr(self.vision, "try_control_snapshot", None)
        if not callable(consume):
            consume = getattr(self.vision, "consume_control_snapshot", None)
        snapshot = consume() if callable(consume) else self.vision.poll()
        self._visual_snapshot_busy = snapshot is None
        if snapshot is None:
            return
        self._sync_approach_applied(snapshot)
        self._apply_visual_snapshot(snapshot)

    def _apply_visual_snapshot(self, snapshot):
        self.visual_snapshot = snapshot
        self.link.stage2_last_visual_snapshot = snapshot
        health_token, healthy, _captured = self._visual_health(snapshot)
        health_changed = (self._visual_health_token is not None
                          and health_token != self._visual_health_token)
        decision_invalid = bool(health_changed or not healthy)
        if decision_invalid:
            self._vision_pause_pending = True
            if self._vision_pause_reason is None:
                self._vision_pause_reason = self._visual_pause_cause(snapshot, health_token, healthy)
        self._visual_health_token = health_token
        generation = _stage2_value(snapshot, "generation")
        if generation != self._visual_generation:
            if self._visual_generation is not None:
                decision_invalid = True
            self._visual_generation = generation
            self.visual_timeout = self.visual_candidate = False
            self._candidate_started_sequence = None
            self._candidate_pending = self._candidate_raw_active = self._candidate_last_active = False
        active = bool(_stage2_value(snapshot, "candidate_active", False))
        started = _stage2_value(snapshot, "candidate_started_sequence")
        if _stage2_value(snapshot, "observation_interrupted", False):
            self._candidate_pending = False
            self._candidate_started_sequence = None
            self.visual_timeout = False
        if (started is not None and started != self._candidate_started_sequence
                or started is None and active and not self._candidate_last_active):
            self._candidate_pending = False
        if started is not None:
            self._candidate_started_sequence = started
        self._candidate_last_active = self._candidate_raw_active = active
        self.visual_candidate = active
        confirmed = _stage2_value(snapshot, "confirmed")
        if confirmed is not None:
            self.visual_confirmed = confirmed
            self._turn_decision = confirmed
        self.visual_timeout = bool(_stage2_value(snapshot, "timed_out", False))
        if (self._waiting_recovery and not self._vision_waiting
                and (self._vision_pause_pending or self.visual_confirmed is not None)):
            raise _Stage2VisionInterrupt()

    def _epoch(self, value):
        self.vision.set_epoch(value)
        self._visual_epoch = value
        self._pending_visual_suspension = False
        self.visual_confirmed = None
        self.visual_timeout = self.visual_candidate = False
        self._candidate_started_sequence = None
        self._candidate_pending = self._candidate_raw_active = self._candidate_last_active = False

    def _action(self, text):
        self.display.action = text
        self.vision.set_action(text)

    def _notify_stopped_once(self, text, *, committed=False):
        """Operator output is issued only after actual strict-stop evidence."""
        if committed and self._turn_stop_notified:
            return
        if committed:
            self._turn_stop_notified = True
        emitter = getattr(self.vision, "_status_emitter", None)
        if emitter is not None:
            emitter(text)

    def _observe_distance(self, state):
        at = self.display.controller._state_at
        self.progress.update(state, time.monotonic() if at is None else at)
        if self._previous_observer is not None:
            self._previous_observer(state)

    def _position_at(self, source):
        history = self.progress._history
        if not history or not history[0][0] - 1e-9 <= source <= history[-1][0] + 1e-9:
            return None
        previous = history[0]
        for current in history:
            if current[0] + 1e-9 >= source:
                fraction = ((source - previous[0]) / (current[0] - previous[0])
                            if current[0] > previous[0] else 0.0)
                return tuple(a + fraction * (b - a) for a, b in zip(previous[1], current[1]))
            previous = current
        return None

    def _distance_at(self, source):
        position = self._position_at(source)
        origin = self.progress.origin_mm
        if position is None or origin is None:
            return None
        return sum(value - start for value, start in zip(position, origin)) / 2.0

    def _fresh_position_feedback(self, now):
        return _stage2_fresh_position_feedback(self.link, self.display.controller, now)

    def _fresh_walls(self, sample, now):
        identity = self.link.session_nonce, self.link.boot_id
        sources = getattr(self.link, "stage2_gate_sources", None)
        keys = getattr(self.link, "stage2_gate_keys", None)
        gate_identity = getattr(self.link, "stage2_gate_identity", None)
        if (not sample.dual_wall or sample.source_identity != identity or None in identity
                or gate_identity is not None and gate_identity != identity):
            return False
        for index, source in enumerate(sample.source_times):
            if (not 0 <= now - source <= .250 + 1e-9
                    or self.passage_barrier is not None and source <= self.passage_barrier + 1e-9
                    or sources is not None and source <= sources[index] + 1e-9
                    or keys is not None and (sample.source_keys is None
                        or not sequence._source_key_newer(sample.source_keys[index], keys[index]))):
                return False
        return True

    def _eligible_deferred_sides(self, sample, now, *, wall=None):
        identity = self.link.session_nonce, self.link.boot_id
        if sample.source_identity != identity or None in identity:
            return frozenset()
        eligible = set()
        gate_sources = getattr(self.link, "stage2_gate_sources", None)
        gate_keys = getattr(self.link, "stage2_gate_keys", None)
        for index, side in enumerate(("left", "right")):
            if side not in sample.candidates:
                continue
            source = sample.source_times[index]
            if (not 0 <= now - source <= .250 + 1e-9
                    or self.passage_barrier is not None and source <= self.passage_barrier + 1e-9
                    or gate_sources is not None and source <= gate_sources[index] + 1e-9
                    or gate_keys is not None and (sample.source_keys is None
                        or not sequence._source_key_newer(sample.source_keys[index], gate_keys[index]))):
                continue
            if wall is not None and (source <= wall.source_times[index] + 1e-9
                    or wall.source_keys is not None and (sample.source_keys is None
                        or not sequence._source_key_newer(sample.source_keys[index], wall.source_keys[index]))):
                continue
            eligible.add(side)
        return frozenset(eligible)

    def _defer_openings(self, samples, now):
        # Keep the newest qualified opening on each side. Older openings
        # cannot follow a wall which the newer one fails to follow. This
        # preserves receipt-time authority through delayed STATE without
        # imposing a queue-size timeout on a long open passage.
        latest = [None, None]
        for sample in (*self._deferred_openings, *samples):
            eligible = self._eligible_deferred_sides(sample, sample.received_at)
            for index, side in enumerate(("left", "right")):
                if side not in eligible:
                    continue
                previous = latest[index]
                key = sample.source_keys[index] if sample.source_keys is not None else sample.source_times[index]
                old_key = (None if previous is None else previous.source_keys[index]
                           if previous.source_keys is not None else previous.source_times[index])
                if sequence._source_key_newer(key, old_key):
                    latest[index] = sample
        retained, seen = [], set()
        for sample in latest:
            if sample is not None and id(sample) not in seen:
                retained.append(sample)
                seen.add(id(sample))
        self._deferred_openings = deque(sorted(retained, key=lambda sample: sample.received_at))

    def _replay_deferred_opening(self, wall, now):
        for sample in self._deferred_openings:
            eligible = self._eligible_deferred_sides(sample, sample.received_at, wall=wall)
            if eligible:
                self.link.stage2_opening = replace(sample, candidate_sides=eligible)
                at = sample.received_at - sample.receive_delay_ms / 1000.0
                self.junction_evidence = self.link.stage2_junction_evidence = PatrolJunctionEvidence(
                    "explicit_opening", sample.source_identity, at,
                    sample.snapshot_uptime_ms, tuple(sorted(eligible)), None, None)
                if self.returning:
                    self.link._side_brake_pending = True
                break
        self._deferred_openings.clear()

    def _arm_walls(self, samples, now):
        controller = self.display.controller
        self._defer_openings(samples, now)
        if self.pending_wall is not None and not self._fresh_walls(self.pending_wall, now):
            self.pending_wall = None
        for sample in samples:
            if self._fresh_walls(sample, now):
                controller.observe_side(sample)
                if self.pending_wall is None:
                    self.pending_wall = sample
        wall = self.pending_wall
        if wall is None:
            return
        controller.retry_pending_pair(now=now)
        latest = controller.latest_source_times
        if (latest is None or not all(self._position_at(at) is not None
                and latest[index] >= at - 1e-9 for index, at in enumerate(wall.source_times))):
            return
        if self._pending_next_road:
            # Preserve the complete STATE history while creating the new road origin.
            self.progress.origin_mm = self.progress.origin_source_at = None
            self.road = "B"
            self.centering_cutoff = self.reference_fixed = False
            self.cutoff_reference = None
            self._pending_next_road = False
        if not self.progress.establish(max(wall.source_times)):
            return
        self.armed = True
        self.pending_wall = None
        self.link.stage2_opening = None
        self.link.stage2_junction_evidence = self.junction_evidence = None
        self.first_wall_source = max(wall.source_times)
        self.detector.reset(None, None, wall.source_identity, require_window=False)
        self.detector._valid_seen = True
        self._localization_samples.clear()
        self.link.set_stage2_mode("normal")
        self.link.side_candidates_enabled = True
        controller.begin_alignment(now, source_barrier=self.passage_barrier)
        self.display.averages.reset(now, source_barrier=self.passage_barrier)
        self.display._ready_batches.clear()
        self.display.origin = self.progress.origin_mm
        self.phase = "RETURN" if self.returning else "ALIGN"
        self._replay_deferred_opening(wall, now)
        runlog.emit("stage2_road_armed", "真正新双墙建立道路原点并启用激光纠偏",
                    road=self.road, returning=self.returning, source_times=wall.source_times,
                    source_keys=wall.source_keys, origin_mm=self.progress.origin_mm)

    def _detect_junction(self, samples, now):
        if not self.armed:
            return None
        identity = self.link.session_nonce, self.link.boot_id
        if self.detector.identity != identity:
            self.detector.reset(None, None, identity, require_window=False)
            self._localization_samples.clear()
            self.junction_evidence = None
        saved = getattr(self.link, "stage2_junction_evidence", None)
        if saved is not None:
            if saved.identity != identity:
                raise mcu.McuError("阶段二已保存路口证据身份失效")
            self.junction_evidence = saved
            return self.link.stage2_opening
        # A new explicit 1/2 is authority at receipt, regardless of delayed
        # STATE. Consume the saved decision without applying age a second time.
        for sample in samples:
            if sample.source_identity != identity:
                continue
            at = sample.received_at - sample.receive_delay_ms / 1000.0
            evidence = self.detector.observe_explicit(sample, at, self._distance_at(at),
                now=sample.received_at, source_barrier=self.first_wall_source)
            if evidence is not None:
                eligible = tuple(side for side in evidence.openings if side in sample.candidates)
                if not eligible:
                    continue
                evidence = replace(evidence, openings=eligible)
                self.junction_evidence = self.link.stage2_junction_evidence = evidence
                self.link.stage2_opening = sample
                if self.returning:
                    self.link._side_brake_pending = True
                return sample
        for sample in samples:
            if sample.source_identity != identity:
                continue
            if len(self._localization_samples) == self._localization_samples.maxlen:
                raise mcu.McuError("阶段二路口定位快照队列溢出，禁止无依据续约")
            self._localization_samples.append(sample)
        while self._localization_samples:
            sample = self._localization_samples[0]
            at = sample.received_at - sample.receive_delay_ms / 1000.0
            if (sample.source_identity != identity or not 0 <= now - at <= .250 + 1e-9
                    or self.first_wall_source is not None and at <= self.first_wall_source + 1e-9):
                self._localization_samples.popleft()
                continue
            if not self.progress._history or at > self.progress._history[-1][0] + 1e-9:
                break
            self._localization_samples.popleft()
            position = self._position_at(at)
            if position is None:
                continue
            distance = self._distance_at(at)
            walls = sample.dual_wall and all(0 <= now - source <= .250 + 1e-9
                and self._position_at(source) is not None for source in sample.source_times)
            evidence = self.detector.observe(sample, at, distance,
                self.detector._valid_seen or walls, now=now)
            if evidence is not None:
                self.junction_evidence = self.link.stage2_junction_evidence = evidence
                self.link.stage2_opening = sample
                if self.returning:
                    self.link._side_brake_pending = True
                return sample
        return None

    def _check_cutoff(self):
        """Compatibility hook: road distance never disables lateral control."""

    def _begin_passage(self, opening):
        key = opening.source_identity, opening.source_keys, opening.source_times
        if key == self._last_passage_key or not self.armed:
            return
        self._last_passage_key = key
        evidence = getattr(self.link, "stage2_junction_evidence", None)
        self.passage_barrier = self.link.begin_stage2_passage(opening, evidence=evidence)
        self.junction_evidence = getattr(self.link, "stage2_junction_evidence", evidence)
        self._pending_next_road = True
        self.armed = False
        self.pending_wall = None
        self._deferred_openings.clear()
        self._localization_samples.clear()
        self.sampler.cancel()
        self.phase = "PASSAGE"
        self.display.controller.begin_alignment(time.monotonic(), source_barrier=self.passage_barrier)
        self.display.averages.reset(time.monotonic(), source_barrier=self.passage_barrier)
        self.display._ready_batches.clear()
        self.link.side_candidates_enabled = False
        runlog.emit("stage2_straight_passage", "已保存路口证据沿原道路参考直穿",
                    road=self.road, speed_mmps=self.xx, source_times=opening.source_times,
                    source_keys=opening.source_keys,
                    reference_deg=self.display.controller.heading_reference_deg,
                    evidence_kind=self.junction_evidence.kind,
                    evidence=asdict(self.junction_evidence),
                    raw_statuses=tuple(status.name for status in (opening.left_status, opening.right_status)))

    def _consume(self, *, update=True, means=True):
        samples = self.display.consume_samples(require_healthy=False, collect_means=means,
                                               update_controller=update, allow_stale=True)
        now = time.monotonic()
        for sample in samples:
            _check_patrol_sides(sample, now)
            if sample.source_identity != (self.link.session_nonce, self.link.boot_id):
                raise mcu.McuError("下层激光样本身份不属于当前会话")
        if not self.armed and self.link.stage2_mode in ("passage", "departure"):
            self._defer_openings(samples, now)
        reason = self.link._laser_stale_reason(require_upper=False)
        if reason is not None:
            raise mcu.McuError(reason)
        return samples

    def _strict_brake(self, action, *, reason=None):
        brake_context = _stage2_brake_context_data(
            self.link, self.display, self.visual_snapshot, reason=reason or action,
            road=self.road, phase=self.phase, returning=self.returning)
        self._pending_visual_suspension = True
        self._approach_commands.clear()
        _stage2_authorize_camera_stopped(self.link, False)
        self.sampler.cancel()
        self._deferred_openings.clear()
        self.pending_wall = None
        self.display.action = action
        local = time.monotonic() + 5.0
        # An elapsed optional driving duration must still permit safe BRAKE.
        self._safety_stop = True
        try:
            sequence._apply_brake(self.link, self.display, local)
            # The wire command and its original confirmation are authoritative.
            # Visual bookkeeping and diagnostic work follow actual BRAKE.
            runlog.emit("stage2_brake_requested", "阶段二请求严格停车", **brake_context)
            suspend = getattr(self.vision, "try_suspend_approach_confirmation", None)
            if callable(suspend):
                self._pending_visual_suspension = not suspend()
            else:
                suspend = getattr(self.vision, "suspend_approach_confirmation", None)
                if callable(suspend):
                    try:
                        suspend()
                        self._pending_visual_suspension = False
                    except sequence._CenteringProposalExpired:
                        pass
                else:
                    self._pending_visual_suspension = False
            try:
                self.vision.set_action(action)
            except sequence._CenteringProposalExpired:
                pass
            sequence._wait_stable(self.link, self.display, local, input_source=self._runtime_input)
            _stage2_authorize_camera_stopped(self.link, True)
        finally:
            self._safety_stop = False
        self.response.pause(self.link, time.monotonic())
        self.sent_at = None

    def _extra_stationary_second(self):
        limit = time.monotonic() + 5.0
        while True:
            self._checkpoint()
            self.display.tick()
            self._consume(update=False, means=False)
            stable = sequence._assert_still_stable(self.link, self.display)
            if not stable:
                _stage2_authorize_camera_stopped(self.link, False)
                sequence._wait_stable(self.link, self.display, limit, input_source=self._runtime_input)
                _stage2_authorize_camera_stopped(self.link, True)
            elif time.monotonic() - self.link._stable_barrier >= STAGE2_TURN_STATIONARY_WAIT_S - 1e-9:
                runlog.emit("stage2_turn_stationary_wait_complete", "双轮连续500ms停稳后完整静止1秒",
                            brake_sequence=self.link._last_brake_sequence,
                            brake_evidence_at=self.link._brake_evidence_barrier,
                            stable_barrier=self.link._stable_barrier,
                            wait_completed_at=time.monotonic(),
                            identity=(self.link.session_nonce, self.link.boot_id))
                return
            sequence._remaining(limit, "额外1秒期间无法保持严格静止")
            time.sleep(sequence.CONTROL_INTERVAL_S)

    def _recover(self, reason=None):
        self.sampler.cancel()
        self._strict_brake("控制目标不可用：BRAKE并严格停稳，保留道路原点",
                           reason=reason or "control_feedback_unavailable")
        barrier = self.link._stable_barrier
        self.display.reset_averages(source_barrier=barrier, reset_controller=False)
        if not self.armed:
            self.passage_barrier = barrier
            self.link.stage2_gate_sources = (barrier, barrier)
            self.pending_wall = None
        previous, verified = None, []
        recovery_deadline = self.deadline
        self._waiting_recovery = True
        try:
            while True:
                self._checkpoint()
                sequence._remaining(recovery_deadline, "阶段二运动时限已到，保持停车")
                self.display.tick()
                samples = self._consume(update=True, means=False)
                opening = self.link.stage2_opening
                if opening is not None and self.armed:
                    if self.returning:
                        return
                    self._begin_passage(opening)
                    return
                if not sequence._assert_still_stable(self.link, self.display):
                    sequence._wait_stable(self.link, self.display, time.monotonic() + 5.0,
                                          input_source=self._runtime_input)
                    barrier = self.link._stable_barrier
                    self.display.controller.source_barrier = barrier
                    if not self.armed:
                        self.passage_barrier = barrier
                        self.link.stage2_gate_sources = (barrier, barrier)
                        self.pending_wall = None
                        self._deferred_openings.clear()
                    previous, verified = None, []
                    continue
                now = time.monotonic()
                for sample in samples:
                    if (sample.dual_wall
                            and sequence._fresh_post_stop_pair(sample, now, barrier, previous)):
                        previous = sample
                        verified.append(sample)
                if len(verified) >= 2:
                    controller = self.display.controller
                    controller.retry_pending_pair(now=now)
                    if all(self._position_at(at) is not None for pair in verified[-2:]
                           for at in pair.source_times):
                        if self.link.stage2_mode == "departure":
                            # Only the source barrier is renewed; stage two has
                            # no 2-second departure laser deadline.
                            sequence.MotionSequenceLink.finish_departure(self.link)
                            self.passage_barrier = barrier
                            self.link.stage2_gate_sources = (barrier, barrier)
                            self.pending_wall = next((pair for pair in verified
                                if self._fresh_walls(pair, now)), None)
                        sequence._set_ready_feedback(self.link, verified[-2:], barrier)
                        sequence._clear_side_recovery(self.link)
                        controller.begin_alignment(now, source_barrier=barrier)
                        self.next_send = now
                        return
                time.sleep(sequence.CONTROL_INTERVAL_S)
        finally:
            self._waiting_recovery = False

    def _sample_reference(self, samples, now):
        if bool(getattr(self.display.controller, "heading_recovery_active", False)):
            self.sampler.cancel()
            self.phase = "HEADING_RECOVERY"
            return
        if self.phase == "HEADING_RECOVERY":
            self.phase = "RETURN" if self.returning else "CRUISE" if self.reference_fixed else "ALIGN"
        if self.returning or self.reference_fixed or not self.armed:
            return
        if self.sampler.active and any(not sample.dual_wall or any(
                now - source > .250 + 1e-9 for source in sample.source_times) for sample in samples):
            self.sampler.cancel()
        missing = self.link.side_cached_active or not self._fresh_position_feedback(now)
        mean = self.sampler.update(self.display.controller, self.display.state_samples, now,
                                   soft_missing=missing)
        self.phase = "SAMPLE" if self.sampler.active else "ALIGN"
        if mean is not None:
            if not self.display.controller.adopt_moving_reference(mean, now):
                raise mcu.McuError("运动1秒航向参考采纳失败")
            self.reference_fixed = True
            self.phase = "CRUISE"
            self.display.averages.reset(now, source_barrier=self.display.controller.source_barrier)
            self.display._ready_batches.clear()
            runlog.emit("stage2_reference_adopted", "本段原居中判据及移动1秒采样完成",
                        road=self.road, reference_deg=mean, elapsed_s=self.sampler.elapsed_s,
                        state_count=self.sampler.sample_count)

    def _target(self, now):
        controller = self.display.controller
        approach = self._approach_active()
        goal = min(self.xx, STAGE2_OBSTACLE_APPROACH_SPEED_MMPS) if approach else self.xx
        gyro_only = not self.armed or not self._fresh_position_feedback(now)
        self.display.input_speed_mmps = goal
        if gyro_only:
            options = {"obstacle_approach": True} if approach else {}
            target = controller.target_heading_only(goal, now, departure_only=True, **options)
        else:
            options = {"obstacle_approach": True} if approach else {}
            target = controller.target(goal, now, cached_feedback=self.link.side_cached_active, **options)
        if getattr(self.vision, "obstacle_approach_enabled", False) and target._proposal is not None:
            target._proposal["stage2_approach_generation"] = _stage2_value(
                self.visual_snapshot, "approach_generation", 0)
        return target

    def _send_target(self, target):
        def verify():
            now = time.monotonic()
            if (getattr(self.vision, "obstacle_approach_enabled", False)
                    and (target._proposal or {}).get("stage2_approach_generation")
                    != _stage2_value(self.visual_snapshot, "approach_generation", 0)):
                raise sequence._CenteringProposalExpired("障碍面积接近状态已改变，废止旧速度目标")
            if (not (target._proposal or {}).get("departure")
                    and not self._fresh_position_feedback(now)):
                raise sequence._CenteringProposalExpired(
                    "发令前真实STATE对齐双墙已超250ms，废止位置目标并重新计算道路航向目标")
            if not self.display.controller.target_send_ready(target, now):
                raise sequence._CenteringProposalExpired(
                    "发令前阶段二控制目标身份、航向或测量证据已改变")

        previous = getattr(self.link, "stage2_target_guard", None)
        self.link.stage2_target_guard = verify
        try:
            verify()
            command = sequence._send_feedback_target(
                self.link, self.display, target.linear_mmps, target.angular_mradps,
                self.deadline, target=target)
            if self._approach_active() and target.linear_mmps == min(self.xx, STAGE2_OBSTACLE_APPROACH_SPEED_MMPS):
                key = MessageType.CONTROL_TARGET, command
                record = getattr(self.link, "_command_ledger", {}).get(key)
                if record is not None:
                    self._approach_commands[key] = (self.link.session_nonce, self.link.boot_id,
                        _stage2_value(self.visual_snapshot, "approach_generation", 0), record[0])
            if any(target.wheel_targets):
                self._post_turn_front_barrier = None
            return command
        finally:
            self.link.stage2_target_guard = previous

    def _mark_started(self):
        if self.started_at is None:
            now = time.monotonic()
            sent_at = getattr(self.link, "_active_motion_sent_at", None)
            self.started_at = (float(sent_at) if not isinstance(sent_at, bool)
                and isinstance(sent_at, (int, float)) and math.isfinite(sent_at)
                and sent_at <= now else now)
            if self.duration_s is not None:
                self.deadline = self.started_at + self.duration_s
                self.link.set_motion_deadline(self.deadline)

    def _turn_back(self):
        link, display, controller = self.link, self.display, self.display.controller
        self._turn_decision = self.visual_confirmed or self._turn_decision
        self._checkpoint()
        if self._turn_decision is None:
            return
        link.stage2_committed_turn_active = True
        road_reference = controller.heading_reference_deg
        if road_reference is None:
            raise mcu.McuError("障碍掉头缺少正式道路参考")
        previous_epoch, previous_mode = self._visual_epoch, link.stage2_mode
        if not self._turn_stop_ready or not sequence._assert_still_stable(link, display):
            self._strict_brake("确认障碍：BRAKE、严格停稳，再额外静止1秒", reason="obstacle_confirmed")
            self._turn_stop_ready = True
        self._epoch(None)
        link.stage2_opening = link.stage2_junction_evidence = None
        self.junction_evidence = None
        self._localization_samples.clear()
        link.side_candidates_enabled = False
        link.set_stage2_mode("turn")
        self._notify_stopped_once("障碍已确认并严格停稳；额外静止1秒后左转180°。", committed=True)
        return_reference = sequence._normalize_yaw(road_reference + 180.0)
        recovering_turn = self._turn_goal_world is not None
        if recovering_turn:
            start_yaw = link.yaw_deg
            self._resume_turn_to_goal()
            target_yaw = sequence._normalize_yaw(self._turn_goal_world
                - (link.stage2_world_yaw_deg - link.yaw_deg))
            return_reference = sequence._normalize_yaw(self._turn_road_reference_world + 180.0
                - (link.stage2_world_yaw_deg - link.yaw_deg))
        while True:
            if recovering_turn:
                break
            self._extra_stationary_second()
            if self._turn_decision is None:
                self._epoch(previous_epoch)
                link.set_stage2_mode(previous_mode)
                return
            if not sequence._assert_still_stable(link, display):
                self._strict_brake("UTURN发令前静止证据改变：重新刹停并等待1秒",
                                   reason="uturn_stop_evidence_changed")
                continue
            start_yaw = link.yaw_deg
            if start_yaw is None:
                raise mcu.McuError("掉头前航向无效")
            target_yaw = sequence._normalize_yaw(start_yaw + 180.0)
            world = getattr(link, "stage2_world_yaw_deg", None)
            world = start_yaw if world is None else world
            self._turn_goal_world = world + 180.0
            self._turn_road_reference_world = world + sequence._normalize_yaw(road_reference - start_yaw)
            link._turn_direction_guard = sequence._TurnDirectionGuard(
                1, start_yaw, (link.session_nonce, link.boot_id))
            self._action(f"左180°原地掉头；请求轮速{STAGE2_UTURN_WHEEL_SPEED_MMPS}mm/s")
            display.turn_target_yaw = target_yaw
            try:
                action_seq = link.send_motion_action(MotionActionCode.UTURN_LEFT,
                    STAGE2_UTURN_WHEEL_SPEED_MMPS, brake_after_turn=True,
                    require_ranges=False, require_upper=False)
            except _Stage2TurnNotReady as exc:
                link._turn_direction_guard = None
                runlog.emit("stage2_uturn_send_blocked", "首次掉头尚未写入，重新严格停稳与1秒等待",
                            reason=str(exc), brake_sequence=link._last_brake_sequence,
                            stable_barrier=link._stable_barrier)
                self._strict_brake("UTURN最终发令门禁未通过：重新刹停并等待1秒",
                                   reason="uturn_final_stop_gate_changed")
                continue
            except BaseException:
                link._turn_direction_guard = None
                self._epoch(previous_epoch)
                link.set_stage2_mode(previous_mode)
                raise
            break
        self._turn_motion_active = not recovering_turn
        if not recovering_turn:
            self._mark_started()
        turn_wheels = (-STAGE2_UTURN_WHEEL_SPEED_MMPS, STAGE2_UTURN_WHEEL_SPEED_MMPS)
        turn_angular = 2 * STAGE2_UTURN_WHEEL_SPEED_MMPS * 1000 // config.FW_WHEEL_BASE_MM
        display.command, display.wheel_targets = (0, turn_angular), turn_wheels
        display.command_kind = MessageType.MOTION_ACTION
        turn_response = sequence._WheelResponseTracker(link, time.monotonic())
        turn_key = None if recovering_turn else (MessageType.MOTION_ACTION, action_seq)

        def on_poll():
            self._checkpoint()
            display.tick(pump=False)
            self._consume(update=False, means=False)
            confirmed = getattr(link, "_confirmed_evidence_times", {}).get(turn_key)
            applied = getattr(link, "_confirmed_commands", {}).get(turn_key)
            turn_response.update(link, turn_wheels, time.monotonic(), confirmed,
                                 display.state_samples, applied_state=applied)
            display.maybe_print()

        try:
            if not recovering_turn:
                link.wait_for_motion_action_complete(
                    action_seq, timeout_s=min(13.0, sequence._remaining(self.deadline)),
                    require_ranges=False, require_upper=False, on_poll=on_poll)
        finally:
            link._turn_direction_guard = None
            self._turn_motion_active = False
        self._strict_brake("掉头完成：显式BRAKE、严格停稳，再额外静止1秒", reason="uturn_complete")
        self._extra_stationary_second()
        actual = link.yaw_deg
        if actual is None or abs(sequence._normalize_yaw(actual - target_yaw)) > 5.0:
            raise mcu.McuError(f"180°掉头航向未达标：目标{target_yaw}，实际{actual}")
        barrier = link._stable_barrier
        display.reset_averages(source_barrier=barrier)
        controller.set_initial_reference(return_reference)
        link.begin_stage2_return(barrier)
        display._samples.clear()
        self.returning, self.armed, self.phase = True, False, "RETURN_DEPARTURE"
        self.centering_cutoff, self.cutoff_reference = False, None
        self.reference_fixed = True
        self.sampler.cancel()
        self.passage_barrier = barrier
        self.pending_wall = None
        self.first_wall_source = None
        self.detector.reset(None, None, (link.session_nonce, link.boot_id), require_window=False)
        self._deferred_openings.clear()
        self.progress = sequence._RoadProgress()
        display.road_progress = self.progress
        display.origin = None
        self.sent_at, self.next_send = None, time.monotonic()
        self.response = sequence._WheelResponseTracker(link, self.next_send)
        display.turn_target_yaw = None
        self._epoch("return")
        if getattr(self.vision, "distance_profile", None) is not None:
            self._post_turn_front_barrier = time.monotonic()
        self._turn_decision = None
        self._turn_goal_world = self._turn_road_reference_world = None
        link.stage2_committed_turn_active = False
        self._turn_stop_notified = False
        runlog.emit("stage2_return_started", "掉头核验完成，以道路正式参考加180度持续纠偏返回",
                    actual_start_yaw_deg=start_yaw, actual_target_yaw_deg=target_yaw,
                    road_reference_deg=road_reference, return_reference_deg=return_reference,
                    stable_barrier=barrier)

    def _resume_turn_to_goal(self):
        """Complete only the saved world-angle remainder using the existing 91 mm/s pivot."""
        link, display = self.link, self.display
        tail = 0.0
        local_deadline = math.inf
        while True:
            self._extra_stationary_second()
            self._checkpoint()
            display.tick()
            world = link.stage2_world_yaw_deg
            if world is None or link.stage2_world_yaw_unknown:
                raise mcu.McuError("剩余掉头缺少连续新鲜世界航向，保持停车")
            decision = turn_recovery_decision(self._turn_goal_world, world, 0.0,
                max(0.0, time.monotonic() - link._sample_time("state")), brake_tail_deg=tail)
            if decision["done"]:
                return
            if decision["overshot"]:
                raise mcu.McuError("剩余掉头已超过原终点5度，保持停车")
            sign = 1
            angular = sign * (2 * STAGE2_UTURN_WHEEL_SPEED_MMPS * 1000 // config.FW_WHEEL_BASE_MM)
            wheels = (-sign * STAGE2_UTURN_WHEEL_SPEED_MMPS, sign * STAGE2_UTURN_WHEEL_SPEED_MMPS)
            start_world = world
            allowed_arc = decision["remaining_deg"]
            link.stage2_recovery_pivot_first = MotionAction(
                link.session_nonce, MotionActionCode.UTURN_LEFT, MotionActionFlag.BRAKE_AFTER_TURN,
                STAGE2_UTURN_WHEEL_SPEED_MMPS, 1800)
            link._turn_direction_guard = _Stage2RemainingTurnGuard(sign, link.yaw_deg,
                (link.session_nonce, link.boot_id))
            self._turn_motion_active = True
            previous_remaining = getattr(display.controller, "_stage2_remaining_turn_active", False)
            display.controller._stage2_remaining_turn_active = True
            response = sequence._WheelResponseTracker(link, time.monotonic())
            next_send, sent_key, issued = time.monotonic(), None, set()
            try:
                while True:
                    self._checkpoint()
                    display.tick()
                    self._consume(update=False, means=False)
                    now = time.monotonic()
                    sequence._remaining(local_deadline, "剩余掉头在局部13秒内未达到原终点，停车")
                    if sent_key is not None:
                        for state, source_at in display.state_samples:
                            key = MessageType.CONTROL_TARGET, state.applied_control_sequence
                            record = link._command_ledger.get(key)
                            applied = (key in issued and key in link._confirmed_commands
                                and record is not None and tuple(record[6:8]) == (link.session_nonce, link.boot_id)
                                and record[5][1:3] == (0, angular)
                                and source_at >= record[0] - 1e-9
                                and state.target_linear_mms == 0 and state.target_angular_mrads == angular)
                            guard = link._turn_direction_guard
                            reason = guard.observe(state, source_at, now, applied=applied)
                            if reason is not None:
                                link._turn_direction_fault = True
                                runlog.emit("stage2_remaining_turn_direction_fault", reason,
                                    sequence=state.applied_control_sequence, source_at=source_at,
                                    goal_world=self._turn_goal_world, actual_unwrapped_yaw_deg=guard.yaw)
                                link._turn_direction_guard = None
                                self._strict_brake("剩余掉头实际错向：立即BRAKE", reason=reason)
                                raise mcu.McuError(reason)
                    world = link.stage2_world_yaw_deg
                    if world is None:
                        raise mcu.McuError("剩余掉头航向失效")
                    rate = display.controller._effective_yaw_rate(now)
                    if rate is None:
                        raise mcu.McuError("剩余掉头缺少新鲜角速")
                    decision = turn_recovery_decision(self._turn_goal_world, world, rate,
                        max(0.0, now - link._sample_time("state")), brake_tail_deg=tail,
                        allowed_arc_deg=allowed_arc, traveled_deg=abs(world - start_world))
                    if decision["overshot"]:
                        link._turn_direction_guard = None
                        self._strict_brake("剩余掉头超过原終点：停车", reason="remaining_turn_overshot")
                        raise mcu.McuError("剩余掉头已超过原终点5度，保持停车")
                    if sent_key is not None and (decision["done"] or decision["should_brake"]):
                        before = world
                        link._turn_direction_guard = None
                        self._strict_brake("剩余掉头：预测制动并核验原终点", reason="remaining_turn_brake")
                        after = link.stage2_world_yaw_deg
                        if after is None:
                            raise mcu.McuError("剩余掉头制动后航向失效")
                        tail = max(tail, abs(after - before))
                        runlog.emit("stage2_remaining_turn_braked", "剩余角预测制动，保留原世界终点",
                                    goal_world=self._turn_goal_world, before_world=before, after_world=after,
                                    prediction=decision, observed_brake_tail_deg=tail)
                        break
                    if sent_key is not None:
                        record = link._command_ledger.get(sent_key)
                        sent_at = record[0] if record is not None else now
                        executing, confirmed, applied = sequence._confirmed_wheel_response(link, sent_at, wheels)
                        response.update(link, executing, now, confirmed, display.state_samples, applied_state=applied)
                    if now >= next_send:
                        number = link.send_motion(0, angular, require_ranges=False, require_upper=False)
                        if not math.isfinite(local_deadline):
                            local_deadline = time.monotonic() + 13.0
                        sent_key = MessageType.CONTROL_TARGET, number
                        issued.add(sent_key)
                        display.command, display.wheel_targets = (0, angular), wheels
                        display.command_kind = MessageType.CONTROL_TARGET
                        next_send = time.monotonic() + sequence.COMMAND_INTERVAL_S
                        runlog.emit("stage2_remaining_turn_sent", "以91mm/s双轮继续原终点剩余角",
                                    sequence=number, goal_world=self._turn_goal_world, world_yaw=world,
                                    prediction=decision, angular_mradps=angular, wheel_targets=wheels)
                    time.sleep(sequence.CONTROL_INTERVAL_S)
            finally:
                display.controller._stage2_remaining_turn_active = previous_remaining
                link.stage2_recovery_pivot_first = None
                link._turn_direction_guard = None
                self._turn_motion_active = False

    def _finish_return(self, opening):
        evidence = getattr(self.link, "stage2_junction_evidence", None) or self.junction_evidence
        if evidence is None:
            at = opening.received_at - opening.receive_delay_ms / 1000.0
            evidence = PatrolJunctionEvidence("explicit_opening", opening.source_identity,
                at, opening.snapshot_uptime_ms, tuple(sorted(opening.candidates)), None, None)
        if (evidence.identity != (self.link.session_nonce, self.link.boot_id)
                or getattr(self.link, "_hard_fault_reason", None) is not None):
            raise mcu.McuError("回程路口证据身份失效或已有真实故障")
        self.junction_evidence = evidence
        # This is a committed business continuation. A new HELLO invalidates
        # raw opening identity, but cannot turn finishing into another drive.
        self.phase = "RETURN_FINISH"
        self._return_finish_evidence = evidence
        inferred = evidence.kind == "missing_inference"
        self._strict_brake("回程首个持续缺测推断路口：BRAKE并严格停稳，阶段二完成"
                           if inferred else "回程首个新鲜显式开口：BRAKE并严格停稳，阶段二完成",
                           reason="return_junction_missing" if inferred else "return_junction_opening")
        runlog.emit("stage2_return_junction_stopped", "回程路口停车结束",
                    evidence_kind=evidence.kind, evidence=asdict(evidence),
                    raw_statuses=tuple(status.name for status in (opening.left_status, opening.right_status)),
                    stable_barrier=self.link._stable_barrier)
        return True

    def _run_loop(self):
        link, display, controller = self.link, self.display, self.display.controller
        if not self._loop_initialized:
            self._loop_initialized = True
            link.side_motion_active = True
            link.side_candidates_enabled = False
            link.stage2_return = False
            link.set_stage2_mode("normal")
            display._distance_observer = self._observe_distance
            display.road_progress = self.progress
            self.progress.update(link.state, controller._state_at or time.monotonic())
            link.stage2_runtime_poll = self._checkpoint
            link.stage2_visual_health_gate = self._guard_visual_motion
            link.stage2_visual_snapshot_gate = self._guard_visual_snapshot
            link.stage2_visual_stop_pending = lambda: self.visual_confirmed is not None
            self._epoch("outbound")
        while True:
            self._checkpoint()
            if self.phase == "RETURN_FINISH":
                self._strict_brake("接续回程终点：BRAKE并严格停稳，阶段二完成",
                                   reason="return_finish_continuation")
                runlog.emit("stage2_return_junction_stopped", "恢复后接续原回程路口停车结束",
                            evidence=None if self._return_finish_evidence is None
                            else asdict(self._return_finish_evidence), stable_barrier=link._stable_barrier)
                return True
            display.tick()
            self._checkpoint()
            if self._visual_snapshot_busy and self._turn_decision is None:
                time.sleep(sequence.CONTROL_INTERVAL_S)
                continue
            if self._vision_pause_pending and self.visual_confirmed is None and self._turn_decision is None:
                self._recover_vision()
                continue
            # Visual decisions precede any nonzero target, including a passage renewal.
            if self.visual_confirmed is not None or self._turn_decision is not None:
                if self.returning:
                    self._strict_brake("回程再次确认障碍：停车结束，不执行第二次掉头",
                                       reason="return_obstacle_confirmed")
                    return False
                try:
                    self._turn_back()
                except _Stage2VisionInterrupt:
                    self._recover_vision()
                continue
            if not self._post_turn_front_ready(self.visual_snapshot):
                if not sequence._assert_still_stable(link, display):
                    self._strict_brake("转后前向识别等待：重新BRAKE并严格停稳",
                                       reason="post_turn_front_wait_unstable")
                    self._extra_stationary_second()
                    self._post_turn_front_barrier = time.monotonic()
                self._consume(update=False, means=False)
                self._action("转后保持停车，等待600前向新鲜无命中、无候选画面")
                time.sleep(sequence.CONTROL_INTERVAL_S)
                continue
            opening = link.stage2_opening
            if opening is not None and self.armed:
                if self.returning:
                    return self._finish_return(opening)
                self._begin_passage(opening)
            samples = self._consume(update=self.armed, means=self.armed)
            now = time.monotonic()
            if not self.armed:
                self._arm_walls(samples, now)
            self._detect_junction(samples, now)
            # UART may have delivered a new opening AFTER the first new walls,
            # while receive-time candidate authority was still disabled.
            opening = link.stage2_opening
            if opening is not None and self.armed:
                if self.returning:
                    return self._finish_return(opening)
                self._begin_passage(opening)
            if link.side_recovery_active:
                try:
                    self._recover(reason=getattr(link, "side_recovery_reason", None) or "laser_recovery_required")
                except _Stage2VisionInterrupt:
                    pass
                continue
            executing, confirmed, applied = sequence._confirmed_wheel_response(link, self.sent_at,
                                                                                display.wheel_targets)
            self.response.update(link, executing, now, confirmed, display.state_samples, applied_state=applied)
            self._sample_reference(samples, now)
            self._action(f"{'回程B→J' if self.returning else '去程道路' + self.road}；{self.phase}；"
                         f"目标{min(self.xx, STAGE2_OBSTACLE_APPROACH_SPEED_MMPS) if self._approach_active() else self.xx}mm/s"
                         + ("；障碍接近减速／核验" if self._approach_active() else ""))
            if now + 1e-9 < self.next_send:
                display.maybe_print()
                time.sleep(sequence.CONTROL_INTERVAL_S)
                continue
            try:
                self._checkpoint()
                if self.visual_confirmed is not None:
                    continue
                target = self._target(time.monotonic())
                if not any(target.wheel_targets):
                    if any(display.wheel_targets):
                        self._recover(reason="zero_control_target")
                    else:
                        time.sleep(sequence.CONTROL_INTERVAL_S)
                    continue
                if sequence._wheel_transition_requires_stop(display.wheel_targets, target.wheel_targets):
                    raise mcu.McuError("阶段二前进目标产生非法归零／换向")
                display.centering_target = target
                self._send_target(target)
                self._mark_started()
                self.sent_at = link._active_motion_sent_at or time.monotonic()
                controller.confirm_target_sent(target)
                self.next_send = self.sent_at + sequence.COMMAND_INTERVAL_S
            except sequence._CenteringProposalExpired:
                self.next_send = time.monotonic()
            except _Stage2VisionInterrupt:
                continue
            except sequence._SideRecoveryRequired as exc:
                try:
                    self._recover(reason=str(exc))
                except _Stage2VisionInterrupt:
                    pass
            display.maybe_print()
            time.sleep(sequence.CONTROL_INTERVAL_S)

    def run(self):
        try:
            while True:
                try:
                    return self._run_loop()
                except sequence._CenteringProposalExpired:
                    self.next_send = time.monotonic()
                    time.sleep(sequence.CONTROL_INTERVAL_S)
                except mcu.McuError:
                    if self._control_recovery is None:
                        if self._turn_goal_world is not None and getattr(self.link, "_turn_abort_evidence", None) is not None:
                            self._await_turn_abort_diagnosis()
                        if self._encoder_fault_candidate() and not self._known_encoder_fault():
                            self._await_encoder_fault_diagnosis()
                        candidate = _Stage2ParkedRecovery(self.link, self.display, self.input_source)
                        if not candidate.can_recover():
                            raise
                        self._control_recovery = candidate
                    self._recover_confirmation_fault()
        finally:
            self.sampler.cancel()
            self.display._distance_observer = self._previous_observer
            self.link.stage2_runtime_poll = self._previous_runtime_poll
            self.link.stage2_visual_stop_pending = self._previous_visual_gate
            self.link.stage2_visual_health_gate = self._previous_health_gate
            self.link.stage2_visual_snapshot_gate = self._previous_snapshot_gate
            self.link.side_motion_active = self.link.side_candidates_enabled = False
            self.vision.set_epoch(None)


import argparse
import socket
import sys

import camera
import gimbal
import gimbal_test
from motion_speed_input import TerminalSpeedInput


DEFAULT_DURATION_S = None
STAGE2_PAN_US = 1500
STAGE2_INITIAL_TILT_US = 800


class _Stage2Gimbal(gimbal.Gimbal):
    """One-shot PWM using Stage2Link's original ACK/STATE receive timestamps."""

    def set_pwm(self, pan_us, tilt_us, *, deadline=None):
        if self._deadline_expired(deadline):
            raise mcu.McuError("云台调整截止，未发送新的PWM命令")
        command = self.mcu.send_gimbal(WireGimbalPose.LEVEL_CENTER, pan_us, tilt_us)
        runlog.emit("gimbal.command", "已发送云台 PWM 命令", sequence=command,
                    action="set_pwm", pose=WireGimbalPose.LEVEL_CENTER.name,
                    pan_us=pan_us, tilt_us=tilt_us, ch1_pwm_us=tilt_us, ch2_pwm_us=pan_us)
        timeout = config.MCU_COMMAND_CONFIRM_TIMEOUT_S
        if deadline is not None:
            record = self.mcu._command_ledger[(MessageType.CONTROL_TARGET, command)]
            timeout = min(timeout, deadline - record[0])
            if timeout <= 0:
                raise mcu.McuError("云台确认时间已用尽")
        # Stage2Link requires both matching ACK and the original applied STATE;
        # a second wall-clock ACK wait would discard timely received evidence.
        self.mcu.wait_for_control_applied(command, timeout_s=timeout)
        end = time.monotonic() + config.SERVO_SETTLE_S
        if deadline is not None:
            end = min(end, deadline)
        while time.monotonic() < end:
            self.mcu.pump()
            time.sleep(max(0.0, min(.02, end - time.monotonic())))
        self.mcu.check_stage2_gimbal_output()
        if self._deadline_expired(deadline):
            raise mcu.McuError("云台未完成到位等待")
        runlog.emit("gimbal.applied", "MCU确认PWM输出启用，未验证机械角度",
                    sequence=command, applied=True, action="set_pwm", pan_us=pan_us,
                    tilt_us=tilt_us, ch1_pwm_us=tilt_us, ch2_pwm_us=pan_us)
        return command


class _Stage2RunLog(runlog.RunLog):
    """Keep diagnostic records in files and only operator messages on stdout."""

    def __init__(self, mode="stage2_obstacle", **kwargs):
        kwargs.setdefault("terminal", sequence.RawTerminalStream())
        super().__init__(mode, **kwargs)
        self._terminal_configured = False

    def emit(self, event, message, **fields):
        # Install before the first record is queued, including session_start.
        if self.terminal_sink is not None and not self._terminal_configured:
            self.terminal_sink.write = self._write_operator_record
            self._terminal_configured = True
        super().emit(event, message, **fields)

    def _write_operator_record(self, record):
        event, fields = record["event"], record["fields"]
        if event == "stage2_ui":
            text = record["message"]
        elif event in ("stage2_exit_brake_failed", "stage2_exit_stop_failed",
                       "stage2_close_failed", "unhandled_exception"):
            text = f"{record['message']}：{fields.get('error', '')}"
        else:
            return
        self.terminal.write(text + "\n")
        self.terminal.flush()


def _stage2_operator_message(message):
    if runlog.current_directory() is not None:
        runlog.emit("stage2_ui", message)
    else:
        print(message, flush=True)


class _Stage2CancellationInput:
    def __init__(self, console):
        self.console = console

    def poll(self):
        return self.console.poll_cancellation()


class _Stage2Console:
    """Keep image health checks alive in the original bounded motion waits."""

    def __init__(self, source, emitter=None):
        self.source = source
        self.emitter = emitter or (lambda _text: None)
        self.vision = None
        self.require_preview = False
        self.require_detection = False
        self.interrupt_on_detection = False
        self._preview_acknowledged = False
        self._detection_waiting = False
        self._pending_line = None
        self._request_prompt = None
        self._recovery_active = False
        self._recovery_prompt_open = False
        self._recovery_serial = 0
        self.retain_input_on_detection_recovery = False
        self.cancellation_input = _Stage2CancellationInput(self)

    def begin_request(self, prompt):
        self._pending_line = None
        self._request_prompt = prompt
        if not self._recovery_active:
            self.source.begin_request(prompt)

    def end_request(self):
        self._pending_line = None
        self._request_prompt = None
        if not self._recovery_active:
            self.source.end_request()

    def _read(self):
        # Runtime cancellation must not wait on the camera's publication lock.
        # The runner consumes its own nonblocking visual control snapshot.
        if not self.require_preview and not self.require_detection:
            return self.source.poll()
        snapshot = self.vision.poll() if self.vision is not None else None
        line = self.source.poll()
        if isinstance(line, str) and line.strip().lower() == "q":
            return line
        if self.vision is None:
            return line
        reconfirm = getattr(self.vision, "needs_reconfirmation", None)
        preview_lost = (self.require_preview and self._preview_acknowledged
                        and (bool(reconfirm) if isinstance(self.vision, Stage2Vision)
                             else not self.vision.preview_ready() or bool(reconfirm)))
        if preview_lost and not self._recovery_active:
            self._pending_line = None
            self._recovery_active = self._recovery_prompt_open = True
            self._recovery_serial += 1
            self.source.end_request()
            self.emitter("画面已中断；保持停车，当前PWM和已接受速度保留。请等待浏览器收到新画面后重新回车确认。")
            self.source.begin_request("画面恢复；看到新画面后回车确认，q退出：")
            line = None
        if self._recovery_active:
            self._pending_line = None
            if line == "" and self.vision.preview_ready():
                acknowledge = getattr(self.vision, "acknowledge_preview", None)
                if not callable(acknowledge) or acknowledge():
                    self.source.end_request()
                    self._recovery_active = self._recovery_prompt_open = False
                    if self._request_prompt is not None:
                        self.source.begin_request(self._request_prompt)
                    self.emitter("新画面已确认，继续原停车准备步骤。")
            elif line is not None:
                self.emitter("尚未收到新鲜浏览器画面，或尚未回车确认；车辆继续停车。")
                self.source.end_request()
                self.source.begin_request("画面恢复；看到新画面后回车确认，q退出：")
            if self.interrupt_on_detection:
                raise _Stage2StartupRecoveryRequired("画面中断，重新进行完整停车采样")
            return None
        if (self.require_preview and self._preview_acknowledged
                and isinstance(self.vision, Stage2Vision) and not self.vision.preview_ready()):
            # During the original two-second gap keep the request and typed
            # response; publish it only after a current fresh browser frame.
            if line is not None and self._pending_line is None:
                self._pending_line = line
            return None
        if self.require_detection and not _stage2_detection_ready(self.vision, snapshot, soft=True):
            if self.retain_input_on_detection_recovery:
                if line is not None and self._pending_line is None:
                    self._pending_line = line
            else:
                self._pending_line = None
            if self.interrupt_on_detection:
                raise _Stage2StartupRecoveryRequired("视觉检测中断，重新进行完整停车采样")
            if not self._detection_waiting:
                self.emitter("障碍识别暂不可用；保持停车，等待新鲜成功检测。")
                self._detection_waiting = True
            return None
        if self._detection_waiting:
            self.emitter("障碍识别已恢复，请继续当前输入。")
            self._detection_waiting = False
            return None
        return line

    def poll_cancellation(self):
        line = self._read()
        if isinstance(line, str) and line.strip().lower() == "q":
            self._pending_line = None
            return line
        if line is not None and self._pending_line is None:
            self._pending_line = line
        return None

    def poll(self):
        # Always read cancellation/errors before returning a buffered response.
        line = self._read()
        if isinstance(line, str) and line.strip().lower() == "q":
            self._pending_line = None
            return line
        if self._pending_line is not None:
            if (self.require_detection and self.retain_input_on_detection_recovery
                    and not _stage2_detection_ready(self.vision, soft=True)):
                return None
            if (self.require_preview and self._preview_acknowledged
                    and isinstance(self.vision, Stage2Vision) and not self.vision.preview_ready()):
                return None
            pending, self._pending_line = self._pending_line, None
            return pending
        return line

    def close(self):
        self._pending_line = None
        self.source.close()


def _components(*, camera_index=0, camera_device=None) -> dict:
    """只创建阶段二设备，不创建RFID读卡器或巡逻任务服务。"""
    link = Stage2Link()
    return {"mcu": link, "gimbal": _Stage2Gimbal(link),
            "camera": _Stage2CameraSource(camera_index=camera_index, camera_device=camera_device)}


def _prepare(components, display, input_source):
    link = components["mcu"]
    link.stage2_camera_source = components.get("camera")
    _stage2_authorize_camera_stopped(link, False)
    display.emitter("正在检查启动条件，小车保持停车。")
    runlog.emit("stage2_detail", "阶段二不使用RFID，无需连接读卡器。")
    link.set_motion_deadline(None)
    link.set_stage2_mode("stopped")
    if not getattr(link, "stage2_startup_link_ready", False):
        link.open()
        link.hello()
        link.synchronize_clock()
        link.send_stop(StopReason.MISSION)
        link.stage2_startup_link_ready = True
    deadline = time.monotonic() + config.MCU_SELFCHECK_READY_TIMEOUT_S
    telemetry_ready, next_diagnostic = False, float("-inf")
    while True:
        sequence._check_input_exit(input_source)
        link.pump()
        reason = link.motion_health_reason() or sequence._gyro_health_reason(link)
        now = time.monotonic()
        if reason is not None:
            if telemetry_ready or now >= deadline:
                raise mcu.McuError(f"运动遥测自检失败：{reason}")
        else:
            telemetry_ready = True
            display.tick(pump=False)
            sample = sequence._side_sample(link) if link.directional is not None else None
            display.consume_samples(collect_means=False)
            identity = link.session_nonce, link.boot_id
            blocked = []
            if sample is None:
                blocked.append("尚无下层激光快照")
            else:
                _check_patrol_sides(sample, now)
                if sample.source_identity != identity or None in identity:
                    blocked.append("激光身份不属于当前已建立会话")
                for index, side in enumerate(("左下CH2", "右下CH1")):
                    status = (sample.left_status, sample.right_status)[index]
                    age = (now - sample.source_times[index]) * 1000.0
                    if status != WireRangeStatus.VALID:
                        blocked.append(f"{side}状态{status.name}({int(status)})，需要VALID(0)")
                    if not 0 <= age <= 250.0 + 1e-6:
                        blocked.append(f"{side}真实源龄{age:.1f}ms，不满足0～250ms")
            fields = dict(current_identity=identity, blocking_reasons=blocked,
                          source_identity=sample.source_identity if sample is not None else None,
                          source_keys=sample.source_keys if sample is not None else None,
                          source_times=sample.source_times if sample is not None else None)
            if sample is not None:
                fields.update(sample.diagnostics(now))
            if not blocked:
                display.reset_averages(fresh_only=False)
                display.confirmation = "首次真正新鲜双墙有效；保持停车，继续云台调整"
                runlog.emit("stage2_startup_laser_ready", "首次双墙启动资格通过", **fields)
                break
            display.confirmation = "保持停车等待首次双墙：" + "；".join(blocked)
            if now >= next_diagnostic:
                next_diagnostic = now + 1.0
                detail = "尚无下层激光快照"
                if sample is not None:
                    parts = []
                    for index, side in enumerate(("左CH2", "右CH1")):
                        status = (sample.left_status, sample.right_status)[index]
                        distance = (sample.left_mm, sample.right_mm)[index]
                        age = (now - sample.source_times[index]) * 1000.0
                        key = sample.source_keys[index] if sample.source_keys is not None else None
                        parts.append(f"{side}={status.name}({int(status)})/{distance}mm/源龄{age:.1f}ms/源键{key}")
                    detail = "；".join(parts) + f"；样本身份={sample.source_identity}，当前身份={identity}"
                runlog.emit("stage2_startup_laser_wait", "云台前双墙等待诊断", **fields)
            display.maybe_print()
        time.sleep(sequence.CONTROL_INTERVAL_S)
    link.motion_checks_enabled = True
    sequence._brake(link, display, time.monotonic() + 5.0, input_source=input_source)
    _stage2_authorize_camera_stopped(link, True)


def _stage2_startup_retry(link, display, input_source, operation):
    """Resume one preparation step without replaying completed operator input."""
    while True:
        try:
            return operation()
        except (sequence._UserQuit, EOFError):
            raise
        except Exception:
            if not isinstance(link, Stage2Link):
                raise
            recovery = getattr(link, "stage2_startup_recovery", None)
            if recovery is None:
                recovery = _Stage2ParkedRecovery(link, display,
                    input_source=getattr(input_source, "cancellation_input", input_source))
                link.stage2_startup_recovery = recovery
            if not recovery.can_recover():
                raise
            runlog.emit("stage2_startup_confirmation_recovery", "启动步骤确认超时，停车恢复后续接",
                        input_speed_mmps=getattr(display, "input_speed_mmps", None),
                        confirmed_pwm=getattr(link, "stage2_confirmed_pwm", None))
            recovery.recover()
            link.stage2_startup_recovery = None
            link.stage2_startup_link_ready = True
            # A successful new session has its own coordinates and clock.
            # Only a complete new one-second sample may authorize departure.
            link.stage2_startup_generation = None


def _stopped_tick(link, display, input_source):
    return _stage2_startup_retry(link, display, input_source,
                                lambda: _stopped_tick_once(link, display, input_source))


def _stopped_tick_once(link, display, input_source):
    try:
        if (getattr(link, "stage2_gimbal_monitor", None) is not None
                and getattr(link, "_hard_fault_reason", None) is None
                and link.connected):
            link.pump()
            if not link._sample_is_fresh("state"):
                _stage2_authorize_camera_stopped(link, False)
                sequence._check_input_exit(getattr(input_source, "cancellation_input", input_source))
                display.action = "保持停车；等待当前新鲜STATE，保留已确认PWM"
                return False
        display.tick()
        display.consume_samples(collect_means=False)
        if not sequence._assert_still_stable(link, display):
            _stage2_authorize_camera_stopped(link, False)
        sequence._ensure_stable(link, display, time.monotonic() + 5.0,
                                input_source=getattr(input_source, "cancellation_input", input_source))
        _stage2_authorize_camera_stopped(link, True)
        return True
    except BaseException:
        _stage2_authorize_camera_stopped(link, False)
        raise


def _wait_stopped_line(link, display, input_source, prompt):
    input_source.begin_request(prompt)
    try:
        while True:
            if not _stopped_tick(link, display, input_source):
                time.sleep(sequence.CONTROL_INTERVAL_S)
                continue
            line = input_source.poll()
            if line is not None:
                if line.strip().lower() == "q":
                    raise sequence._UserQuit("收到 q，保持停车并退出")
                return line.strip()
            time.sleep(sequence.CONTROL_INTERVAL_S)
    finally:
        input_source.end_request()


def _configure_gimbal(components, display, input_source, emitter):
    link, controller = components["mcu"], components["gimbal"]
    current = STAGE2_INITIAL_TILT_US
    current_sequence = None
    display.action = "停车调整摄像头；左右固定1500 μs"

    def apply(value, *, initial=False):
        nonlocal current, current_sequence
        recovery_serial = getattr(input_source, "_recovery_serial", 0)
        runlog.emit("stage2_gimbal_request", "停车调整请求上下PWM",
                    requested_pan_us=STAGE2_PAN_US, requested_tilt_us=value,
                    previous_confirmed_tilt_us=None if initial else current,
                    previous_confirmed_sequence=current_sequence,
                    initial=initial, recovery_serial=recovery_serial)
        while True:
            _stopped_tick(link, display, input_source)
            if isinstance(input_source, _Stage2Console):
                sequence._check_input_exit(input_source.cancellation_input)
                preview_authorized = (not input_source.require_preview or (
                    input_source.vision is not None and input_source._preview_acknowledged
                    and not input_source._recovery_active and input_source.vision.preview_ready()
                    and not getattr(input_source.vision, "needs_reconfirmation", False)))
                if not preview_authorized or input_source._recovery_serial != recovery_serial:
                    if not initial:
                        runlog.emit("stage2_gimbal_request_discarded", "画面中断，未发送本次PWM输入",
                                    requested_tilt_us=value, retained_tilt_us=current,
                                    retained_sequence=current_sequence,
                                    request_recovery_serial=recovery_serial,
                                    current_recovery_serial=input_source._recovery_serial)
                        emitter("画面中断，本次上下PWM输入未执行；新画面确认后请重新输入。")
                        return False
                    time.sleep(sequence.CONTROL_INTERVAL_S)
                    # Initial positioning may run after reconfirmation; an
                    # already-read operator adjustment must never be replayed.
                    recovery_serial = input_source._recovery_serial
                    continue
            break
        current_sequence = _stage2_startup_retry(
            link, display, input_source, lambda: controller.set_pwm(STAGE2_PAN_US, value))
        current = value
        _stopped_tick(link, display, input_source)
        emitter(f"MCU已确认，PWM输出已启用：左右={STAGE2_PAN_US} μs，上下={current} μs。请观察实际动作和画面。")
        return True

    apply(current, initial=True)
    emitter("输入上下PWM整数（500～2500）并回车试调；可反复输入。mark完成调整，q退出。")
    while True:
        line = _wait_stopped_line(link, display, input_source, "上下PWM调整（mark继续）：")
        try:
            command, value = gimbal_test._parse_command(
                "set " + line if line.isascii() and line.isdecimal() else line, current, None)
        except ValueError as exc:
            emitter(str(exc))
            continue
        if command == "mark":
            check_output = getattr(link, "check_stage2_gimbal_output", None)
            if callable(check_output):
                check_output()
            runlog.emit("stage2_gimbal_mark", "确定阶段二本轮上下PWM", pan_us=STAGE2_PAN_US,
                        tilt_us=current, confirmed_sequence=current_sequence,
                        mechanical_angle_verified=False)
            link.stage2_gimbal_monitor = None
            return current
        if command == "help":
            emitter("输入500～2500整数或set 整数、+25/-25/+5/-5调整；center回800；mark完成后输入速度；q退出。")
        elif command == "":
            emitter("空回车不结束调整；请输入上下PWM，或输入mark确认当前值。")
        elif command == "center":
            apply(STAGE2_INITIAL_TILT_US)
        elif value is not None:
            apply(value)


def _preview_url(host, port):
    if host not in ("0.0.0.0", ""):
        return f"http://{host}:{port}/"
    try:
        # UDP connect only selects this host's route/interface; no packet is sent.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("192.0.2.1", 9))
            address = probe.getsockname()[0]
        return f"http://{address}:{port}/"
    except OSError:
        return f"http://<RDK-IP>:{port}/"


def _open_camera_stopped(components, display, input_source):
    """Compatibility entry: vehicle checks still apply, camera starts in background."""
    _stopped_tick(components["mcu"], display, input_source)
    poll = getattr(input_source, "poll_cancellation", input_source.poll)
    value = poll()
    if isinstance(value, str) and value.strip().lower() == "q":
        raise sequence._UserQuit("摄像头启动期间收到q，保持停车并退出")
    if getattr(input_source, "eof", False):
        raise EOFError("摄像头启动期间终端输入结束")
    def open_camera():
        try:
            components["camera"].open()
        except Exception as exc:
            runlog.exception("stage2_camera_open_failed", "摄像头后台打开失败", exc)
    threading.Thread(target=open_camera, name="stage2-camera-open", daemon=True).start()


def _wait_preview(components, display, input_source, vision, emitter, host, *, fixed_pwm=False):
    if not getattr(vision, "listener_ready", True):
        raise mcu.McuError("HTTP预览尚未监听：" + str(getattr(vision, "preview_error", None) or "服务线程未启动"))
    emitter(f"电脑浏览器打开：{_preview_url(host, vision.port)}")
    emitter("请确认浏览器显示持续更新的画面，再回车输入速度；q退出。" if fixed_pwm else
            "请确认浏览器显示持续更新的画面，再回车进入上下PWM调整；q退出。")
    while True:
        display.action = "停车等待浏览器新鲜画面及终端确认"
        line = _wait_stopped_line(components["mcu"], display, input_source, "画面预览；回车继续：")
        if not line:
            if not vision.preview_ready():
                snapshot = vision.poll()
                emitter("尚未向浏览器发送新鲜画面，保持停车：" + str(
                    getattr(vision, "preview_error", None) or getattr(snapshot, "startup_reason", None)
                    or "请打开预览网页并等待画面"))
                continue
            acknowledge = getattr(vision, "acknowledge_preview", None)
            if callable(acknowledge) and not acknowledge():
                emitter("确认时画面已过期，请等待新画面后再回车。")
                continue
            if isinstance(input_source, _Stage2Console):
                input_source._preview_acknowledged = True
            return
        emitter("请直接回车继续，q退出。")


def _stage2_detection_ready(vision, snapshot=None, *, soft=False):
    snapshot = vision.poll() if snapshot is None else snapshot
    if soft and isinstance(vision, Stage2Vision):
        return bool(snapshot.soft_ready) and not snapshot.fault_reason
    ready = getattr(snapshot, "ready", None)
    if ready is None:
        return not getattr(snapshot, "fault_reason", None) and vision.preview_ready()
    return (bool(ready) and not getattr(snapshot, "fault_reason", None)
            and Stage2Vision._fresh(getattr(snapshot, "captured_at", None)))


def _wait_startup_vision(components, display, source, vision):
    """Keep the chassis stopped until this camera generation has a fresh detection."""
    last_reason = None
    while True:
        _stopped_tick(components["mcu"], display, source)
        sequence._check_input_exit(getattr(source, "cancellation_input", source))
        snapshot = vision.poll()
        if _stage2_detection_ready(vision, snapshot) and not getattr(source, "_recovery_active", False):
            return
        reason = getattr(snapshot, "startup_reason", None) or "等待新鲜成功检测"
        display.action = "停车等待障碍识别恢复"
        if reason != last_reason:
            last_reason = reason
            source.emitter("保持停车等待视觉：" + str(reason))
        time.sleep(sequence.CONTROL_INTERVAL_S)


def _sample_stage2_start(components, display, source, vision):
    while True:
        _wait_startup_vision(components, display, source, vision)
        observation = getattr(vision.poll(), "observation_generation", 0)
        source.interrupt_on_detection = True
        try:
            _stage2_startup_retry(components["mcu"], display, source,
                lambda: sequence._sample_start(components["mcu"], display, input_source=source))
            snapshot = vision.poll()
            if (getattr(snapshot, "observation_generation", 0) != observation
                    or not _stage2_detection_ready(vision, snapshot)
                    or source._recovery_active):
                raise _Stage2StartupRecoveryRequired("采样期间视觉证据发生中断")
            components["mcu"].stage2_startup_generation = getattr(snapshot, "generation", 0)
            return
        except _Stage2StartupRecoveryRequired as exc:
            source.emitter(str(exc) + "；保留本轮PWM及速度，视觉恢复后重采1秒。")
        finally:
            source.interrupt_on_detection = False


def _stage2_confirm_parked_pwm(link, pwm, input_source=None):
    if pwm[1] != STAGE2_PAN_US:
        raise mcu.McuError("停车恢复云台左右PWM必须为1500 μs")
    number = link.send_gimbal(*pwm)
    return link.wait_stage2_parked_pwm(number, pwm, input_source=input_source)


def _stage2_exit_park(link, display):
    """Earn fresh BRAKE and 500 ms wheel evidence even under an older fault."""
    deadline = time.monotonic() + 5.0
    sequence._apply_brake(link, display, deadline)
    observer = FaultStopTracker(link, link._last_brake_sequence)
    anchor = None
    while anchor is None:
        sequence._remaining(deadline, "退出停车：5秒内未确认双轮连续500ms静止")
        link.pump()
        anchor = observer.update()
        if anchor is None:
            time.sleep(sequence.CONTROL_INTERVAL_S)
    link._stop_tracker = observer.tracker
    link._stable_barrier = time.monotonic()
    display.controller.notify_stopped(link._stable_barrier)
    _stage2_authorize_camera_stopped(link, True)
    runlog.emit("stage2_exit_parked", "退出BRAKE与双轮连续500ms静止已确认",
                brake_sequence=link._last_brake_sequence, anchor=anchor,
                stable_barrier=link._stable_barrier,
                original_fault=getattr(link, "_hard_fault_reason", None))


def _stage2_exit_hold_pwm(link, display):
    pwm = getattr(link, "stage2_confirmed_pwm", None)
    if pwm is None:
        # Before the operator's first confirmed setting there is nothing to
        # restore. A parked chassis may still close without another STOP.
        return
    if pwm[1] != STAGE2_PAN_US:
        raise mcu.McuError("退出云台左右PWM不是已确认的1500 μs")
    wire = getattr(link, "_stage2_gimbal_wire", None)
    # wire[2] only means a later command was logged; BRAKE does not change PWM.
    valid = (wire is not None
             and wire[1].session_nonce == link.session_nonce
             and int(wire[1].pose_id) == int(pwm[0])
             and wire[1].pan_pwm_us == pwm[2]
             and wire[1].tilt_pwm_us == pwm[1]
             and link._sample_is_fresh("state")
             and link._stage2_gimbal_reason(link.state, int(pwm[0]),
                                           (link.session_nonce, link.boot_id)) is None)
    if not valid:
        _stage2_confirm_parked_pwm(link, pwm)
        # The gimbal CONTROL changes applied sequence; earn a new BRAKE anchor.
        _stage2_exit_park(link, display)
    if (not link._sample_is_fresh("state")
            or link._stage2_gimbal_reason(link.state, int(pwm[0]),
                                         (link.session_nonce, link.boot_id)) is not None):
        raise mcu.McuError("退出时缺少当前PWM启用与零速证据")
    runlog.emit("stage2_exit_pwm_hold", "停车退出保持最后确认PWM，只接收收尾后关闭串口",
                pan_us=pwm[1], tilt_us=pwm[2], ch1_pwm_us=pwm[2], ch2_pwm_us=pwm[1],
                state_flags=int(link.state.state_flags),
                state_reports_pwm_us=False, mechanical_angle_verified=False)


def _shutdown(components, display, input_source, vision, *, failed, reason=None):
    link = components.get("mcu")
    production_link = isinstance(link, Stage2Link)
    parked_hold = False
    if link is not None:
        link.stage2_runtime_poll = None
        try:
            link.set_stage2_mode("stopped")
            if production_link and display is not None and getattr(link, "connected", False):
                link.stage2_gimbal_monitor = None
                link.stage2_gimbal_pending = None
                link._turn_direction_guard = None
                link.stage2_fault_stopping = True
                _stage2_brake_context(link, display,
                                     getattr(link, "stage2_last_visual_snapshot", None),
                                     reason=reason or "completed_exit", phase="EXIT")
                _stage2_exit_park(link, display)
                _stage2_exit_hold_pwm(link, display)
                parked_hold = True
            elif failed and display is not None and getattr(link, "connected", False):
                _stage2_brake_context(link, display,
                                     getattr(link, "stage2_last_visual_snapshot", None),
                                     reason=reason or "failed_exit", phase="EXIT")
                sequence._brake(link, display, time.monotonic() + 5.0)
        except (Exception, KeyboardInterrupt) as exc:
            runlog.exception("stage2_exit_brake_failed", "退出时BRAKE停稳未确认", exc)
        try:
            if not parked_hold and getattr(link, "connected", False):
                link.send_stop(StopReason.CLOSE)
        except (Exception, KeyboardInterrupt) as exc:
            runlog.exception("stage2_exit_stop_failed", "退出时STOP未发送", exc)
    for name, resource in (("preview", vision), ("camera", components.get("camera")),
                           ("input", input_source)):
        if resource is not None:
            try:
                resource.close()
            except Exception as exc:
                runlog.exception("stage2_close_failed", "关闭阶段二资源失败", exc, component=name)
    if link is not None:
        try:
            if production_link:
                try:
                    link.receive_shutdown_tail()
                finally:
                    link.close()
            else:
                link.shutdown()
        except (Exception, KeyboardInterrupt) as exc:
            runlog.exception("stage2_close_failed", "关闭阶段二串口失败", exc)


def run_obstacle_test(*, duration_s=None, components=None, input_source=None,
                      emitter=None, host="0.0.0.0", port=8080, vision_factory=None,
                      camera_index=0, camera_device=None, distance_profile="combined"):
    if distance_profile not in ("near", "far", "combined"):
        raise ValueError("distance-profile必须为near、far或combined")
    if duration_s is not None and (isinstance(duration_s, bool)
                                  or not math.isfinite(duration_s) or duration_s <= 0):
        raise ValueError("duration 必须是有限正数或None")
    if isinstance(port, bool) or not isinstance(port, int) or not 0 <= port <= 65535:
        raise ValueError("port 必须是0～65535整数")
    if isinstance(camera_index, bool) or not isinstance(camera_index, int) or camera_index < 0:
        raise ValueError("camera-index必须为非负整数")
    if camera_device is not None and (not isinstance(camera_device, str) or not camera_device.strip()):
        raise ValueError("camera-device必须为非空设备路径")
    if camera_device is not None and camera_index != 0:
        raise ValueError("camera-index与camera-device不能同时指定")
    write = emitter if emitter is not None else _stage2_operator_message
    c, display, vision, source = {}, None, None, None
    evidence_writer = None
    result_directory = None
    exit_reason = None
    succeeded = False
    try:
        import hashlib
        source_path = Path(__file__).resolve()
        runlog.emit("stage2_runtime_source", "阶段二本次运行文件指纹",
                    source_path=str(source_path), source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                    pwm_comparison_commit="5d6b1e7", pid=os.getpid())
        c = (_components() if camera_index == 0 and camera_device is None else
             _components(camera_index=camera_index, camera_device=camera_device)) if components is None else components
        c["mcu"].stage2_resumable_parked_recovery = True
        source = _Stage2Console(input_source if input_source is not None else TerminalSpeedInput(write_line=write),
                                emitter=write)
        source.retain_input_on_detection_recovery = True
        display = Stage2Display(c["mcu"], emitter=write)
        _stage2_startup_retry(c["mcu"], display, source, lambda: _prepare(c, display, source))
        runlog.emit("stage2_runtime_firmware", "阶段二握手后固件身份",
                    build_id=getattr(c["mcu"], "build_id", None),
                    expected_build_id=sequence.EXPECTED_BUILD_ID,
                    capabilities=getattr(c["mcu"], "capabilities", None),
                    expected_capabilities=sequence.EXPECTED_CAPABILITIES,
                    session_nonce=getattr(c["mcu"], "session_nonce", None),
                    boot_id=getattr(c["mcu"], "boot_id", None))
        _stage2_startup_retry(c["mcu"], display, source,
                             lambda: c["gimbal"].set_pwm(STAGE2_PAN_US, 600))
        check_output = getattr(c["mcu"], "check_stage2_gimbal_output", None)
        if callable(check_output):
            check_output()
        c["mcu"].stage2_gimbal_monitor = None
        write("摄像头PWM已固定：左右1500 μs，上下600 μs。")
        from obstacle_evidence import DistanceEvidenceWriter
        directory = runlog.current_directory()
        if directory is None:
            import uuid
            directory = Path(config.LOG_DIR) / ("stage2-obstacle-" + uuid.uuid4().hex[:12])
        result_directory = (Path(directory) / "obstacle-results").resolve()
        try:
            evidence_writer = DistanceEvidenceWriter(result_directory)
        except Exception as exc:
            runlog.exception("stage2_obstacle_evidence_open_failed", "障碍结果目录无法打开", exc,
                             result_directory=str(result_directory))
            write(f"障碍结果保存不可用：{exc}")
        vision = (vision_factory or Stage2Vision)(c["camera"], host=host, port=port)
        enable_distance = getattr(vision, "enable_distance_obstacles", None)
        if callable(enable_distance):
            enable_distance(distance_profile, evidence_writer=evidence_writer)
        if isinstance(vision, Stage2Vision):
            vision._status_emitter = write
        source.vision = vision
        source.require_preview = True
        try:
            vision.start()
        except Exception as exc:
            runlog.exception("stage2_preview_start_failed", "视觉预览未启动，保持停车退出", exc)
            raise mcu.McuError("视觉预览启动失败：" + str(exc)) from exc
        if isinstance(c["mcu"], Stage2Link) and isinstance(vision, Stage2Vision):
            c["mcu"].stage2_vision = vision
        c["mcu"].stage2_runtime_poll = lambda: sequence._check_input_exit(source.cancellation_input)
        _wait_preview(c, display, source, vision, write, host, fixed_pwm=True)
        # One explicit browser acknowledgement belongs to startup. Later camera
        # generations recover from fresh detection evidence while staying parked.
        source.require_preview = False
        source.require_detection = True
        _wait_startup_vision(c, display, source, vision)
        xx = _stage2_startup_retry(c["mcu"], display, source,
                                  lambda: sequence._wait_speed(c["mcu"], display, source))
        _sample_stage2_start(c, display, source, vision)
        source.require_preview = source.require_detection = False
        c["mcu"].stage2_runtime_poll = None
        succeeded = Stage2Runner(c["mcu"], display, vision, xx,
                                 input_source=source, duration_s=duration_s).run()
        write("阶段二完成：回程路口已刹车并确认停稳。" if succeeded else "阶段二未完成，已结束运动流程并尝试停车。")
    except (KeyboardInterrupt, EOFError, sequence._UserQuit) as exc:
        reason = "收到Ctrl-C" if isinstance(exc, KeyboardInterrupt) else str(exc)
        exit_reason = reason
        write(f"阶段二退出：{reason}；尝试安全停车。")
    except Exception as exc:
        exit_reason = str(exc)
        write(f"阶段二失败：{exc}；尝试安全停车。")
        runlog.exception("stage2_failed", "阶段二执行失败", exc)
    finally:
        _shutdown(c, display, source, vision, failed=not succeeded, reason=exit_reason)
        if evidence_writer is not None:
            try:
                evidence_summary = evidence_writer.close()
                if isinstance(evidence_summary, dict) and (evidence_summary.get("errors")
                        or evidence_summary.get("dropped_events") or evidence_summary.get("incomplete")):
                    runlog.emit("stage2_obstacle_evidence_incomplete", "障碍证据保存不完整",
                                **evidence_summary)
                    write("障碍结果保存不完整；请查看结果目录中的summary.json。")
            except Exception as exc:
                runlog.exception("stage2_obstacle_evidence_close_failed", "障碍证据保存未完整结束", exc)
                write(f"障碍结果保存失败：{exc}")
        if result_directory is not None:
            write(f"本次障碍结果：{result_directory}")
        directory = runlog.current_directory()
        if directory is not None:
            write(f"本次日志：{directory}")
    return succeeded


def _duration(value):
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("duration必须是秒数") from exc
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("duration必须是有限正数")
    return result


def _port(value):
    try:
        result = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("port必须为1～65535整数") from exc
    if not 1 <= result <= 65535:
        raise argparse.ArgumentTypeError("port必须为1～65535整数")
    return result


def _camera_index(value):
    try:
        result = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("camera-index必须为非负整数") from exc
    if result < 0:
        raise argparse.ArgumentTypeError("camera-index必须为非负整数")
    return result


def _parse_cli(argv=None):
    parser = argparse.ArgumentParser(description="阶段二：实时预览、直穿路口、四色障碍掉头、回程路口刹停")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP监听地址，默认0.0.0.0")
    parser.add_argument("--port", type=_port, default=8080, help="HTTP端口，默认8080")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--camera-index", type=_camera_index, default=None,
                           help="USB摄像头索引，默认0；不会自动切换设备")
    selection.add_argument("--camera-device", help="明确指定USB设备，例如/dev/video0；与索引互斥")
    parser.add_argument("--duration", type=_duration, default=None,
                        help="可选运动总秒数，从首次非零发令起计；默认无总时限")
    return parser.parse_args(argv)


def _run_cli(args, *, distance_profile="combined"):
    camera_kwargs = {}
    if args.camera_index is not None:
        camera_kwargs["camera_index"] = args.camera_index
    if args.camera_device is not None:
        camera_kwargs["camera_device"] = args.camera_device
    return 0 if run_obstacle_test(duration_s=args.duration, host=args.host, port=args.port,
                                 distance_profile=distance_profile, **camera_kwargs) else 1


def main_cli(argv=None, *, distance_profile="combined"):
    return _run_cli(_parse_cli(argv), distance_profile=distance_profile)


def _main(*, distance_profile="combined"):
    args = _parse_cli()
    with _Stage2RunLog():
        return _run_cli(args, distance_profile=distance_profile)


if __name__ == "__main__":
    sys.exit(_main())
