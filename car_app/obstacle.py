"""道路中部的颜色 / 结构障碍检测；有界光照适应，不依赖测距或白路面。"""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, log

import config


@dataclass(frozen=True)
class ObstacleResult:
    detected: bool
    color: str | None
    area_px: float | None
    bbox: tuple[int, int, int, int] | None  # 原图坐标 (x, y, width, height)
    center: tuple[float, float] | None
    available: bool
    reason: str | None = None

    @classmethod
    def unavailable(cls, reason: str) -> "ObstacleResult":
        return cls(False, None, None, None, None, False, reason)


_COLOR_RANGES = (
    ("blue", ("OBSTACLE_BLUE_RANGE",)),
    ("yellow", ("OBSTACLE_YELLOW_RANGE",)),
    ("brown", ("OBSTACLE_BROWN_RANGE",)),
    ("red", ("OBSTACLE_RED_LOW_RANGE", "OBSTACLE_RED_HIGH_RANGE")),
)


def _roi_bounds(height: int, width: int) -> tuple[int, int, int, int] | None:
    x0 = max(0, min(width, int(round(width * config.OBSTACLE_ROI_X_MIN_FRACTION))))
    x1 = max(0, min(width, int(round(width * config.OBSTACLE_ROI_X_MAX_FRACTION))))
    y0 = max(0, min(height, int(round(height * config.OBSTACLE_ROI_Y_MIN_FRACTION))))
    y1 = max(0, min(height, int(round(height * config.OBSTACLE_ROI_Y_MAX_FRACTION))))
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def _bbox_iou(
    first: tuple[int, int, int, int], second: tuple[int, int, int, int]
) -> float:
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    left, top = max(ax, bx), max(ay, by)
    right, bottom = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    intersection = max(0, right - left) * max(0, bottom - top)
    union = aw * ah + bw * bh - intersection
    return intersection / union if union > 0 else 0.0


def _bbox_center(box: tuple[int, int, int, int]) -> tuple[float, float]:
    x, y, width, height = box
    return x + width / 2.0, y + height / 2.0


def _bbox_area(box: tuple[int, int, int, int]) -> float:
    return float(box[2] * box[3])


def same_obstacle(previous: ObstacleResult | None, current: ObstacleResult) -> bool:
    """用框的位置与大小确认连续帧对应同一障碍，不依赖颜色标签。"""
    if not (
        previous
        and previous.detected
        and current.detected
        and previous.bbox
        and current.bbox
    ):
        return False

    if _bbox_iou(previous.bbox, current.bbox) >= config.OBSTACLE_CONTINUITY_MIN_IOU:
        return True

    previous_area = _bbox_area(previous.bbox)
    current_area = _bbox_area(current.bbox)
    if previous_area <= 0.0 or current_area <= 0.0:
        return False
    area_ratio = max(previous_area, current_area) / min(previous_area, current_area)
    if area_ratio > config.OBSTACLE_CONTINUITY_MAX_AREA_RATIO:
        return False

    previous_center = previous.center or _bbox_center(previous.bbox)
    current_center = current.center or _bbox_center(current.bbox)
    previous_diagonal = hypot(previous.bbox[2], previous.bbox[3])
    current_diagonal = hypot(current.bbox[2], current.bbox[3])
    maximum_shift = max(
        float(config.OBSTACLE_CONTINUITY_MIN_CENTER_SHIFT_PX),
        min(previous_diagonal, current_diagonal)
        * float(config.OBSTACLE_CONTINUITY_MAX_CENTER_SHIFT_RATIO),
    )
    return hypot(
        current_center[0] - previous_center[0],
        current_center[1] - previous_center[1],
    ) <= maximum_shift


class ObstacleCandidate:
    """Confirm one color target on distinct images within one fixed time window."""

    def __init__(self) -> None:
        self.started_at: float | None = None
        self.last: ObstacleResult | None = None
        self.last_frame: object = None
        self.hits = 0
        self.clears = 0

    @property
    def active(self) -> bool:
        return self.started_at is not None

    def expired(self, now: float) -> bool:
        return self.active and now - self.started_at >= config.OBSTACLE_VERIFY_TIMEOUT_S

    def update(self, result: ObstacleResult, frame_token: object, now: float) -> str:
        """Return first/pending/confirmed/cleared/idle/timeout/fault."""
        if not result.available:
            return "fault"
        if self.expired(now):
            return "timeout"
        if frame_token == self.last_frame:
            return "pending" if self.active else "idle"
        self.last_frame = frame_token
        if result.detected:
            first = not self.active
            if first:
                self.started_at = now
            self.hits = self.hits + 1 if same_obstacle(self.last, result) else 1
            self.last = result
            self.clears = 0
            if self.hits >= config.OBSTACLE_HIT_STREAK:
                return "confirmed"
            return "first" if first else "pending"
        if self.active:
            self.clears += 1
            self.hits = 0
            self.last = None
            if self.clears >= config.OBSTACLE_CLEAR_STREAK:
                self.started_at = None
                self.clears = 0
                return "cleared"
            return "pending"
        return "idle"


def _correct_lighting(bgr, bounds, cv2, np):
    """只根据当前帧估计有限校正；不设置摄像头参数，也不依赖历史帧。"""
    x0, y0, x1, y1 = bounds
    roi = bgr[y0:y1, x0:x1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    neutral = (
        (hsv[:, :, 1] <= config.OBSTACLE_WB_MAX_SATURATION)
        & (hsv[:, :, 2] >= config.OBSTACLE_WB_MIN_VALUE)
        & (hsv[:, :, 2] <= config.OBSTACLE_WB_MAX_VALUE)
    )
    neutral_count = int(np.count_nonzero(neutral))
    required = max(
        int(config.OBSTACLE_WB_MIN_PIXELS),
        int(round(neutral.size * config.OBSTACLE_WB_MIN_FRACTION)),
    )
    # 孤立弱色目标不能充当中性参考；真正的偏色墙面通常跨多个网格。
    grid_size = int(config.OBSTACLE_WB_GRID_SIZE)
    rows = np.linspace(0, neutral.shape[0], grid_size + 1, dtype=int)
    columns = np.linspace(0, neutral.shape[1], grid_size + 1, dtype=int)
    occupied_cells = 0
    for top, bottom in zip(rows[:-1], rows[1:]):
        for left, right in zip(columns[:-1], columns[1:]):
            cell = neutral[top:bottom, left:right]
            minimum = max(1, int(round(cell.size * config.OBSTACLE_WB_MIN_CELL_FRACTION)))
            occupied_cells += int(np.count_nonzero(cell) >= minimum)
    gains = np.ones(3, dtype=np.float32)
    white_balance = (neutral_count >= required
                     and occupied_cells >= config.OBSTACLE_WB_MIN_GRID_CELLS)
    if white_balance:
        channels = np.median(roi[neutral], axis=0)
        gains = np.clip(
            float(np.mean(channels)) / np.maximum(channels, 1.0),
            config.OBSTACLE_WB_GAIN_MIN,
            config.OBSTACLE_WB_GAIN_MAX,
        ).astype(np.float32)
    corrected = np.clip(bgr.astype(np.float32) * gains, 0, 255).astype(np.uint8)
    corrected_hsv = cv2.cvtColor(corrected, cv2.COLOR_BGR2HSV)
    values = corrected_hsv[y0:y1, x0:x1, 2]
    # 纯黑没有光照信息，不能让黑背景主导估计；黑帧仍会产生空颜色掩膜。
    usable = values[values >= 8]
    median_value = float(np.median(usable)) if usable.size else 0.0
    gain = gamma = 1.0
    if median_value > 0:
        gain = max(1.0, float(np.clip(
            config.OBSTACLE_BRIGHTNESS_TARGET / median_value,
            config.OBSTACLE_BRIGHTNESS_GAIN_MIN,
            config.OBSTACLE_BRIGHTNESS_GAIN_MAX,
        )))
        normalized = min(254.0, max(1.0, median_value * gain)) / 255.0
        target = min(254.0, max(1.0, float(config.OBSTACLE_BRIGHTNESS_TARGET))) / 255.0
        gamma = min(1.0, float(np.clip(
            log(target) / log(normalized), config.OBSTACLE_GAMMA_MIN,
            config.OBSTACLE_GAMMA_MAX,
        )))
        lut = np.clip(
            255.0 * (np.minimum(np.arange(256) * gain, 255.0) / 255.0) ** gamma,
            0, 255,
        ).astype(np.uint8)
        # 只改 V，保留白平衡后的色相、饱和度，避免逐通道 gamma 把木色冲成灰色。
        corrected_hsv[:, :, 2] = cv2.LUT(corrected_hsv[:, :, 2], lut)
        corrected = cv2.cvtColor(corrected_hsv, cv2.COLOR_HSV2BGR)
    return corrected, {
        "white_balance_applied": white_balance,
        "neutral_pixels": neutral_count,
        "neutral_required": required,
        "neutral_grid_cells": occupied_cells,
        "neutral_grid_cells_required": int(config.OBSTACLE_WB_MIN_GRID_CELLS),
        "white_balance_bgr_gains": [float(value) for value in gains],
        "brightness_median": median_value,
        "brightness_gain": gain,
        "gamma": gamma,
    }


def _color_mask(hsv, range_names, cv2, *, adaptive=False):
    mask = None
    for name in range_names:
        if adaptive:
            name = name.replace("_RANGE", "_ADAPTIVE_RANGE")
        lower, upper = getattr(config, name)
        current = cv2.inRange(hsv, tuple(lower), tuple(upper))
        mask = current if mask is None else cv2.bitwise_or(mask, current)
    return mask


def _component(mask, raw_mask, color, box, cv2):
    x, y, width, height = box
    return {
        "color": color,
        "bbox": box,
        "area": float(cv2.countNonZero(mask[y:y + height, x:x + width])),
        "raw_area": float(cv2.countNonZero(raw_mask[y:y + height, x:x + width])),
    }


def _union_box(boxes):
    x0 = min(box[0] for box in boxes)
    y0 = min(box[1] for box in boxes)
    x1 = max(box[0] + box[2] for box in boxes)
    y1 = max(box[1] + box[3] for box in boxes)
    return x0, y0, x1 - x0, y1 - y0


def _fence_fragments_compatible(first, second):
    ax, ay, aw, ah = first["bbox"]
    bx, by, bw, bh = second["bbox"]
    if first["plank"] and second["plank"]:
        if max(ah, bh) / min(ah, bh) > config.OBSTACLE_FENCE_MAX_SIZE_RATIO:
            return False
        if max(aw, bw) / min(aw, bw) > config.OBSTACLE_FENCE_MAX_SIZE_RATIO:
            return False
        overlap = max(0, min(ay + ah, by + bh) - max(ay, by))
        gap = max(0, max(ax, bx) - min(ax + aw, bx + bw))
        return (
            overlap / min(ah, bh) >= config.OBSTACLE_FENCE_MIN_VERTICAL_OVERLAP
            and gap <= min(aw, bw) * config.OBSTACLE_FENCE_MAX_GAP_WIDTH_RATIO
        )
    # 横梁必须横跨木条且与其纵向相交 / 紧邻；不能拿远处横色块串起一片噪声。
    if first["plank"] != second["plank"]:
        plank, beam = (first, second) if first["plank"] else (second, first)
        px, py, pw, ph = plank["bbox"]
        ex, ey, ew, eh = beam["bbox"]
        horizontal = max(0, min(px + pw, ex + ew) - max(px, ex))
        vertical_gap = max(0, max(py, ey) - min(py + ph, ey + eh))
        return horizontal >= 0.5 * pw and vertical_gap <= min(pw, eh)
    return False


def _fence_groups(fragments):
    remaining = set(range(len(fragments)))
    groups = []
    while remaining:
        group = [remaining.pop()]
        pending = list(group)
        while pending:
            index = pending.pop()
            matches = [other for other in remaining
                       if _fence_fragments_compatible(fragments[index], fragments[other])]
            remaining.difference_update(matches)
            group.extend(matches)
            pending.extend(matches)
        if len(group) >= 2 and any(fragments[index]["plank"] for index in group):
            groups.append(_union_box([fragments[index]["bbox"] for index in group]))
    return groups


def detect_colored_obstacle(bgr, *, diagnostics=None) -> ObstacleResult:
    """返回最大有效颜色证据，框 / 中心 / 面积为原图单位；无有效帧不能算畅通。

    可选 diagnostics 字典供离线工具查看中间图，不改变候选确认或生产调用接口。
    """
    if diagnostics is not None:
        diagnostics.clear()
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        return ObstacleResult.unavailable(f"障碍视觉依赖不可用: {exc}")

    if bgr is None:
        return ObstacleResult.unavailable("摄像头抓帧失败")
    if (getattr(bgr, "ndim", 0) != 3 or getattr(bgr, "size", 0) == 0
            or bgr.shape[2] != 3 or bgr.dtype != np.uint8):
        return ObstacleResult.unavailable("摄像头帧必须为非空 uint8 BGR 图像")
    try:
        original_height, original_width = bgr.shape[:2]
        original_bounds = _roi_bounds(original_height, original_width)
        max_edge = int(config.OBSTACLE_WORK_MAX_EDGE_PX)
        reference_edge = float(config.OBSTACLE_REFERENCE_EDGE_PX)
        if max_edge <= 0 or reference_edge <= 0:
            return ObstacleResult.unavailable("障碍检测处理尺度配置无效")
        resize_scale = min(1.0, max_edge / max(original_height, original_width))
        working = bgr if resize_scale == 1.0 else cv2.resize(
            bgr, (max(1, int(round(original_width * resize_scale))),
                  max(1, int(round(original_height * resize_scale)))),
            interpolation=cv2.INTER_AREA,
        )
        height, width = working.shape[:2]
        bounds = _roi_bounds(height, width)
        if bounds is None or original_bounds is None:
            return ObstacleResult.unavailable("障碍检测 ROI 配置无效")
        x0, y0, x1, y1 = bounds
        roi = working[y0:y1, x0:x1]
        raw_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        illumination = {"adaptive_enabled": bool(config.OBSTACLE_ADAPTIVE_ENABLED)}
        corrected = working
        corrected_hsv = raw_hsv
        if config.OBSTACLE_ADAPTIVE_ENABLED:
            corrected, info = _correct_lighting(working, bounds, cv2, np)
            illumination.update(info)
            corrected_hsv = cv2.cvtColor(corrected[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
        possible_neutral = raw_hsv[:, :, 1] <= config.OBSTACLE_WB_MAX_SATURATION
        # 弱色且已剪切的通道不能提供可靠色相，且不能用来估计白平衡。
        clipped_neutral = possible_neutral & (raw_hsv[:, :, 2] > config.OBSTACLE_WB_MAX_VALUE)
        illumination["excluded_neutral_clipped_pixels"] = (
            int(np.count_nonzero(clipped_neutral)) if config.OBSTACLE_ADAPTIVE_ENABLED else 0
        )
        reference_scale = max(height, width) / reference_edge
        kernel_size = max(1, int(round(config.OBSTACLE_MORPH_KERNEL_PX * reference_scale)))
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        iterations = max(1, int(config.OBSTACLE_MORPH_ITERATIONS))
        minimum_area = max(
            float(config.OBSTACLE_MIN_COMPONENT_AREA_PX) * reference_scale ** 2,
            roi.shape[0] * roi.shape[1] * config.OBSTACLE_MIN_COMPONENT_AREA_RATIO,
        )
        minimum_width = float(config.OBSTACLE_MIN_WIDTH_PX) * reference_scale
        minimum_height = float(config.OBSTACLE_MIN_HEIGHT_PX) * reference_scale
        candidates = []
        masks, raw_masks = {}, {}
        for color, range_names in _COLOR_RANGES:
            raw_mask = _color_mask(raw_hsv, range_names, cv2)
            mask = raw_mask.copy()
            if config.OBSTACLE_ADAPTIVE_ENABLED:
                # 有足够中性参考且校正证实为中性时，不再保留偏色灰墙的原始假证据。
                minimum_saturation = min(getattr(config, name)[0][1] for name in range_names)
                if illumination.get("white_balance_applied"):
                    neutral_proved = corrected_hsv[:, :, 1] < minimum_saturation
                    raw_mask[neutral_proved] = 0
                raw_mask[clipped_neutral] = 0
                mask = raw_mask.copy()
                mask |= _color_mask(corrected_hsv, range_names, cv2, adaptive=True)
                mask[clipped_neutral] = 0
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=iterations)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)
            masks[color], raw_masks[color] = mask, raw_mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            fragments = []
            for contour in contours:
                box = cv2.boundingRect(contour)
                x, y, component_width, component_height = box
                component = _component(mask, raw_mask, color, box, cv2)
                fill = component["area"] / (component_width * component_height)
                contour_area = float(cv2.contourArea(contour))
                hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
                solidity = contour_area / hull_area if hull_area else 0.0
                if fill < config.OBSTACLE_MIN_COLOR_FILL_RATIO or solidity < config.OBSTACLE_MIN_SOLIDITY:
                    continue
                if color == "brown":
                    fragment_area = max(
                        minimum_area * config.OBSTACLE_FENCE_FRAGMENT_MIN_AREA_RATIO,
                        config.OBSTACLE_FENCE_FRAGMENT_MIN_AREA_PX * reference_scale ** 2,
                    )
                    plank = (component_height >= 1.5 * minimum_height
                             and component_width >= 0.4 * minimum_width
                             and component_height / component_width >= config.OBSTACLE_FENCE_PLANK_MIN_ASPECT)
                    beam = (component_width >= 2.0 * minimum_width
                            and component_height >= 0.4 * minimum_height
                            and component_width / component_height >= config.OBSTACLE_FENCE_BEAM_MIN_ASPECT)
                    if component["area"] >= fragment_area and fill >= 0.35 and (plank or beam):
                        fragments.append(dict(component, plank=plank))
                if (component["area"] >= minimum_area and component_width >= minimum_width
                        and component_height >= minimum_height):
                    candidates.append(component)
            for box in _fence_groups(fragments):
                component = _component(mask, raw_mask, color, box, cv2)
                if (component["area"] >= minimum_area and box[2] >= minimum_width
                        and box[3] >= minimum_height
                        and component["area"] / (box[2] * box[3]) >= config.OBSTACLE_MIN_COLOR_FILL_RATIO):
                    candidates.append(component)

        # 禁行牌内部红圈与外部蓝牌在空间上包含 / 明显重叠才合并；不关联邻近色块。
        for blue in [item for item in candidates if item["color"] == "blue"]:
            for red in [item for item in candidates if item["color"] == "red"]:
                ax, ay, aw, ah = blue["bbox"]
                bx, by, bw, bh = red["bbox"]
                intersection = (max(0, min(ax + aw, bx + bw) - max(ax, bx))
                                * max(0, min(ay + ah, by + bh) - max(ay, by)))
                if intersection / min(aw * ah, bw * bh) < config.OBSTACLE_BLUE_RED_MIN_OVERLAP_RATIO:
                    continue
                box = _union_box([blue["bbox"], red["bbox"]])
                merged = _component(masks["blue"] | masks["red"],
                                    raw_masks["blue"] | raw_masks["red"], "blue", box, cv2)
                candidates.append(merged)

        scale_x, scale_y = width / original_width, height / original_height
        for candidate in candidates:
            x, y, component_width, component_height = candidate["bbox"]
            left, top = round((x + x0) / scale_x), round((y + y0) / scale_y)
            right = round((x + x0 + component_width) / scale_x)
            bottom = round((y + y0 + component_height) / scale_y)
            candidate["bbox"] = (left, top, right - left, bottom - top)
            candidate["area"] /= scale_x * scale_y
            candidate["raw_area"] /= scale_x * scale_y
        if diagnostics is not None:
            diagnostics.update({
                "working_bgr": working, "corrected_bgr": corrected,
                "masks": masks, "roi": original_bounds, "working_roi": bounds,
                "parameters": dict(illumination, normalization_scale=resize_scale,
                                   reference_scale=reference_scale,
                                   minimum_work_area_px=minimum_area,
                                   morph_work_kernel_px=kernel_size),
                "candidates": candidates,
            })
        if not candidates:
            return ObstacleResult(False, None, 0.0, None, None, True)
        best = max(candidates, key=lambda item: (item["area"], item["raw_area"], item["color"]))
        return ObstacleResult(True, best["color"], best["area"], best["bbox"],
                              _bbox_center(best["bbox"]), True)
    except Exception as exc:  # 无法完成检测时不能静默当作无障碍。
        return ObstacleResult.unavailable(f"颜色障碍识别异常: {exc}")
