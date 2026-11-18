"""导航原语：激光状态分类 + 突变检测 + 居中行驶 + 原地转弯闭环。

纯控制逻辑，接收遥测（mm + 状态），输出速度命令 (v, w)。实际收发由 mcu.py 完成，
顶层编排在 main.py。

安全约定：非 VALID 的测距只允许 NO_TARGET/OUT_OF_RANGE 判为「开阔」；其余故障
状态一律 fail-closed（停车），不把传感器故障当成「没墙」。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import config
from drivers.protocol_v2 import StopReason, WireRangeStatus


# ---------------------------------------------------------------------------
# 激光读数分类
# ---------------------------------------------------------------------------
def classify(distance_mm: int | None, status: WireRangeStatus | None) -> str:
    """Classify laser *status*; distance is used for centering, not events."""
    if status is None:
        return "fault"
    if status is not WireRangeStatus.VALID:
        if status in (WireRangeStatus.NO_TARGET, WireRangeStatus.OUT_OF_RANGE):
            return "open"
        return "fault"
    if distance_mm is None or not math.isfinite(distance_mm) or not 0 <= distance_mm < 65535:
        return "fault"
    return "near"


@dataclass(frozen=True)
class SideObservation:
    """One shared interpretation of the two bottom laser samples."""

    left_state: str
    right_state: str
    dual_wall: bool = False
    ambiguous: bool = False
    reason: str = ""
    total_mm: float | None = None


class SideObserver:
    """Separate a lateral offset from an opening using a trusted wall sum.

    A calibrated interval already includes its uncertainty. Without calibration,
    learn only from three distinct, stable samples with both walls close. Never
    move that reference to absorb an opening or an unexpected structure.
    """

    def __init__(self, total_range: tuple[float, float] | None = None) -> None:
        if total_range is not None:
            lower, upper = (float(value) for value in total_range)
            if (not all(math.isfinite(value) for value in (lower, upper))
                    or not 0 <= lower <= upper or upper <= 0):
                raise ValueError("side total range must be finite, nonnegative and contain positive distances")
            total_range = (lower, upper)
        self.total_range = total_range
        self._reference_samples: list[float] = []
        self._last_seq: int | None = None

    def update(
        self,
        directional_seq: int | None,
        left_mm: int | None,
        left_status: WireRangeStatus | None,
        right_mm: int | None,
        right_status: WireRangeStatus | None,
        *,
        yaw_error_deg: float | None = None,
    ) -> SideObservation:
        new_sample = directional_seq is not None and directional_seq != self._last_seq
        if directional_seq is not None:
            self._last_seq = directional_seq
        left = self._state(left_mm, left_status)
        right = self._state(right_mm, right_status)
        both_numeric = left == right == "near"
        total = float(left_mm + right_mm) if both_numeric else None
        if left == "fault" or right == "fault":
            if new_sample:
                self._reference_samples.clear()
            return SideObservation(left, right, reason="side_sensor_fault", total_mm=total)
        yaw_ok = yaw_error_deg is None or (
            math.isfinite(yaw_error_deg)
            and abs(yaw_error_deg) <= getattr(config, "SIDE_MAX_YAW_ERROR_DEG", 15.0)
        )
        if not yaw_ok:
            if new_sample:
                self._reference_samples.clear()
            return SideObservation(left, right, ambiguous=True, reason="side_heading_uncertain", total_mm=total)
        if self.total_range is None:
            if both_numeric:
                if new_sample:
                    self._reference_samples.append(total)
                    count = max(1, int(getattr(config, "SIDE_REFERENCE_SAMPLES", 3)))
                    self._reference_samples = self._reference_samples[-count:]
                    tolerance = max(0.0, float(getattr(config, "SIDE_TOTAL_TOLERANCE_MM", 10.0)))
                    if (len(self._reference_samples) >= count and
                            max(self._reference_samples) - min(self._reference_samples) <= tolerance):
                        mean = sum(self._reference_samples) / len(self._reference_samples)
                        self.total_range = (max(0.001, mean - tolerance), mean + tolerance)
                if self.total_range is None:
                    return SideObservation(left, right, reason="collecting_side_reference", total_mm=total)
            else:
                if new_sample:
                    self._reference_samples.clear()
                return SideObservation(left, right, reason="explicit_open", total_mm=total)
        if both_numeric:
            lower, upper = self.total_range
            if lower <= total <= upper:
                return SideObservation("near", "near", dual_wall=True,
                                       reason="consistent_wall_sum", total_mm=total)
            return SideObservation(left, right, ambiguous=True,
                                   reason="inconsistent_wall_sum", total_mm=total)
        return SideObservation(left, right, reason="explicit_open", total_mm=total)

    @staticmethod
    def _state(distance_mm, status) -> str:
        return classify(distance_mm, status)


class ChannelTracker:
    """单通道去抖：状态持续 debounce 次（遥测周期）才确认跳变，抑制单帧噪声。"""

    def __init__(self, name: str, debounce: int | None = None) -> None:
        self.name = name
        self.debounce = debounce or config.JUMP_WINDOW_SAMPLES
        self.current: str | None = None
        self._pending: str | None = None
        self._pending_streak = 0

    def update(self, state: str) -> tuple[str, str] | None:
        """喂入最新状态，若发生确认的跳变则返回 (old, new)。"""
        if state == "mid":          # 中间值不参与跳变判定，也不推翻当前态
            return None
        if state == "fault":        # 恢复后的第一帧只能重建基线。
            self.current = None
            self._pending = None
            self._pending_streak = 0
            return None
        if self.current is None:
            self.current = state
            return None
        if state == self.current:
            self._pending = None
            self._pending_streak = 0
            return None
        if self._pending == state:
            self._pending_streak += 1
        else:
            self._pending = state
            self._pending_streak = 1
        if self._pending_streak >= self.debounce:
            old, self.current = self.current, state
            self._pending = None
            self._pending_streak = 0
            return (old, state)
        return None


# ---------------------------------------------------------------------------
# 跳变事件
# ---------------------------------------------------------------------------
class JumpEvent:
    """一次激光跳变。"""

    def __init__(self, channel: str, old: str, new: str) -> None:
        self.channel = channel   # "left"/"right"/"upper_left"/"upper_right"
        self.old = old
        self.new = new

    @property
    def side_opened(self) -> bool:
        """侧方墙消失（near→open）：到路口。"""
        return self.channel in ("left", "right") and self.old == "near" and self.new == "open"

    @property
    def ceiling_appeared(self) -> bool:
        """顶出现（open→near）：进入涵洞/隧道。"""
        return self.channel in ("upper_left", "upper_right") and self.old == "open" and self.new == "near"

    @property
    def ceiling_opened(self) -> bool:
        """顶消失（near→open）：离开隧道/涵洞。"""
        return self.channel in ("upper_left", "upper_right") and self.old == "near" and self.new == "open"


class JumpDetector:
    """维护四个通道（左/右/上层左/上层右）的去抖跟踪，产出跳变事件。

    只在对应遥测族的 cycle_seq 更新时推进 tracker，杜绝「同一缓存帧读两次造假跳变」。
    """

    def __init__(self) -> None:
        self.trackers = {
            "left": ChannelTracker("left"),
            "right": ChannelTracker("right"),
            "upper_left": ChannelTracker("upper_left"),
            "upper_right": ChannelTracker("upper_right"),
        }
        self._last_dir_seq: int | None = None
        self._last_upper_seq: int | None = None
        self.side_candidates: tuple[str, ...] = ()

    def reset_side(self) -> None:
        """转弯后重置侧向 tracker，保留上层基线以免重复触发涵洞。"""
        self.trackers["left"] = ChannelTracker("left")
        self.trackers["right"] = ChannelTracker("right")
        self.side_candidates = ()

    def reset_upper(self) -> None:
        """上层失效或恢复时重建基线，不改变侧向路口状态。"""
        self.trackers["upper_left"] = ChannelTracker("upper_left")
        self.trackers["upper_right"] = ChannelTracker("upper_right")
        self._last_upper_seq = None

    def update(
        self,
        directional_seq: int | None,
        upper_seq: int | None,
        left_mm: int | None,
        left_status: WireRangeStatus | None,
        right_mm: int | None,
        right_status: WireRangeStatus | None,
        upper_left_mm: int | None,
        upper_left_status: WireRangeStatus | None,
        upper_right_mm: int | None,
        upper_right_status: WireRangeStatus | None,
        *,
        side_observation: SideObservation | None = None,
    ) -> list[JumpEvent]:
        events: list[JumpEvent] = []
        self.side_candidates = ()
        if directional_seq is not None and directional_seq != self._last_dir_seq:
            self._last_dir_seq = directional_seq
            states = (classify(left_mm, left_status), classify(right_mm, right_status))
            # Geometry events depend only on each channel's fresh wire status.
            candidates = []
            for name, state in zip(("left", "right"), states):
                tracker = self.trackers[name]
                if tracker.current == "near" and state == "open" and tracker._pending != "open":
                    candidates.append(name)
                result = tracker.update(state)
                if result is not None:
                    events.append(JumpEvent(name, result[0], result[1]))
            self.side_candidates = tuple(candidates)
        if upper_seq is not None and upper_seq != self._last_upper_seq:
            self._last_upper_seq = upper_seq
            self._advance(events, "upper_left", upper_left_mm, upper_left_status)
            self._advance(events, "upper_right", upper_right_mm, upper_right_status)
        return events

    def _advance(self, events: list[JumpEvent], name: str, mm: int | None, status) -> None:
        result = self.trackers[name].update(classify(mm, status))
        if result is not None:
            events.append(JumpEvent(name, result[0], result[1]))


def _upper_right_is_primary(
    upper_right_state: str,
    upper_right_has_numeric_value: bool | None,
) -> bool:
    """NO_TARGET and OUT_OF_RANGE are healthy CH5 observations."""
    return upper_right_state != "fault"


def ceiling_entered(
    events: list[JumpEvent],
    upper_right_state: str,
    *,
    upper_right_has_numeric_value: bool | None = None,
) -> bool:
    """进入涵洞/隧道（无目标→有效目标）：右侧健康时为主。"""
    right_is_primary = _upper_right_is_primary(upper_right_state, upper_right_has_numeric_value)
    for event in events:
        if not event.ceiling_appeared:
            continue
        if event.channel == "upper_right":
            return True
        if event.channel == "upper_left" and not right_is_primary:
            return True
    return False


def ceiling_left(
    events: list[JumpEvent],
    upper_right_state: str,
    *,
    upper_right_has_numeric_value: bool | None = None,
) -> bool:
    """离开涵洞/隧道（有效目标→无目标）：右侧健康时为主。"""
    right_is_primary = _upper_right_is_primary(upper_right_state, upper_right_has_numeric_value)
    for event in events:
        if not event.ceiling_opened:
            continue
        if event.channel == "upper_right":
            return True
        if event.channel == "upper_left" and not right_is_primary:
            return True
    return False


class CeilingSourceSelector:
    """把上层右主左备合成为一个顶棚状态，切源不重放旧源跳变。"""

    def __init__(self) -> None:
        self.source: str | None = None
        self.state: str | None = None

    def reset(self) -> None:
        """失效后忘记旧顶棚，恢复样本只建立新的主备基线。"""
        self.source = None
        self.state = None

    def update(
        self,
        events: list[JumpEvent],
        *,
        upper_right_has_numeric_value: bool,
        selected_state: str | None = None,
    ) -> tuple[bool, bool]:
        selected = "upper_right" if upper_right_has_numeric_value else "upper_left"
        stable_selected_state = selected_state if selected_state in {"near", "open"} else None
        if selected != self.source:
            self.source = selected
            self.state = stable_selected_state
            return False, False
        entered = any(
            getattr(event, "channel", None) == selected
            and bool(getattr(event, "ceiling_appeared", False))
            for event in events
        )
        left = any(
            getattr(event, "channel", None) == selected
            and bool(getattr(event, "ceiling_opened", False))
            for event in events
        )
        if entered:
            self.state = "near"
        elif left:
            self.state = "open"
        elif self.state is None:
            self.state = stable_selected_state
        return entered, left


# ---------------------------------------------------------------------------
# 居中行驶（PID）
# ---------------------------------------------------------------------------
class CenterFollower:
    """用左右测距误差 + 陀螺仪航向算角速度，让车保持在道路中心。

    返回 (linear_mmps, angular_mradps, stop_reason)；
    stop_reason 为 None / "fault" / "blind_timeout" / "side_uncertain"。

    每段行驶前调用 :meth:`begin` 记录该段起始航向作为参考；行驶中叠加陀螺仪
    航向校正让车走直（不因转弯残留航向误差漂移）。左右距离差与航向偏差都在
    死区内时不矫正（对应激光 1~3mm 噪声）。转弯期间（原地转）不调本方法，且
    转弯后双墙仍 open 时走盲走分支，天然避开转弯期。
    """

    def __init__(self) -> None:
        self._prev_err: float | None = None
        self._blind_since: float | None = None
        self._ref_yaw: float | None = None
        self._side_fault_streak = 0
        self._last_dir_seq: int | None = None
        self._d_err = 0.0
        self._side_bias_mm = float(config.CENTER_SIDE_BIAS_MM)
        self._recenter_requested = False
        self._recenter_started_at: float | None = None
        self._recenter_finished_at: float | None = None
        self._paused_at: float | None = None
        self._dual_streak = 0
        self._center_streak = 0
        self.recenter_complete = False
        self.timed_out = False

    def begin(
        self, yaw: float | None, *, recenter: bool = False,
        expected_yaw: float | None = None, side_bias_mm: float | None = None,
    ) -> None:
        """Start one road episode; use resume() after an inspection pause."""
        self._ref_yaw = yaw if expected_yaw is None else expected_yaw
        if side_bias_mm is None:
            side_bias_mm = config.CENTER_SIDE_BIAS_MM
        self._side_bias_mm = float(side_bias_mm)
        if not math.isfinite(self._side_bias_mm):
            raise ValueError("side bias must be finite")
        self._prev_err = None
        self._d_err = 0.0
        self._blind_since = None
        self._side_fault_streak = 0
        self._last_dir_seq = None
        self._recenter_requested = recenter
        self._recenter_started_at = None
        self._recenter_finished_at = None
        self._paused_at = None
        self._dual_streak = 0
        self._center_streak = 0
        self.recenter_complete = False
        self.timed_out = False

    @property
    def recenter_started(self) -> bool:
        return self._recenter_started_at is not None

    @property
    def recenter_elapsed_s(self) -> float:
        if self._recenter_started_at is None:
            return 0.0
        end = self._recenter_finished_at
        if end is None:
            end = self._paused_at if self._paused_at is not None else time.monotonic()
        return max(0.0, end - self._recenter_started_at)

    def pause(self) -> None:
        if self._paused_at is None:
            self._paused_at = time.monotonic()

    def resume(self) -> None:
        """Exclude a deliberate stationary pause without restarting the episode."""
        if self._paused_at is not None:
            elapsed = max(0.0, time.monotonic() - self._paused_at)
            if self._recenter_started_at is not None and not self.recenter_complete:
                self._recenter_started_at += elapsed
            if self._blind_since is not None:
                self._blind_since += elapsed
            self._paused_at = None
        self._prev_err = None
        self._d_err = 0.0
        self._dual_streak = 0
        self._center_streak = 0

    def _observe_recentering(self, *, new_frame: bool, dual_wall: bool,
                            error: float | None, heading_error: float | None) -> None:
        if not self._recenter_requested or self.recenter_complete or self._paused_at is not None:
            return
        required = max(1, int(getattr(config, "CENTER_RECENTER_STABLE_SAMPLES", 3)))
        if new_frame:
            self._dual_streak = self._dual_streak + 1 if dual_wall else 0
            if not self.recenter_started:
                if self._dual_streak >= required:
                    self._recenter_started_at = time.monotonic()
                return
            centered = (dual_wall and error is not None and heading_error is not None
                        and abs(error) <= config.CENTER_DEADBAND_MM
                        and abs(heading_error) <= config.CENTER_DEADBAND_DEG)
            self._center_streak = self._center_streak + 1 if centered else 0
            if self._center_streak >= required:
                self.recenter_complete = True
                self._recenter_finished_at = time.monotonic()
        if self.recenter_started and self.recenter_elapsed_s >= getattr(config, "CENTER_RECENTER_TIMEOUT_S", 5.0):
            self.timed_out = True

    def _limited_v(self, v: int) -> int:
        if self.timed_out and not self.recenter_complete:
            return min(v, int(getattr(config, "CENTER_RECENTER_LIMIT_MMPS", 60)))
        return v

    def control(
        self,
        left_mm: int | None,
        left_status: WireRangeStatus | None,
        right_mm: int | None,
        right_status: WireRangeStatus | None,
        yaw: float | None,
        directional_seq: int | None = None,
        *,
        side_observation: SideObservation | None = None,
    ) -> tuple[int, int, str | None]:
        v = config.CENTER_LINEAR_MMPS
        ls = classify(left_mm, left_status)
        rs = classify(right_mm, right_status)

        # 仅在新测距帧时更新侧向故障计数（避免同一缓存帧被多拍重复计数导致去抖失效）
        new_frame = directional_seq is not None and directional_seq != self._last_dir_seq
        if directional_seq is not None:
            self._last_dir_seq = directional_seq
        dual_wall = ls in {"near", "mid"} and rs in {"near", "mid"}
        if side_observation is not None:
            ls, rs = side_observation.left_state, side_observation.right_state
            dual_wall = side_observation.dual_wall

        if ls == "fault" or rs == "fault":
            if new_frame:
                self._side_fault_streak += 1
            self._prev_err = None
            self._d_err = 0.0
            self._observe_recentering(new_frame=new_frame, dual_wall=False, error=None, heading_error=None)
            if self._side_fault_streak >= config.SIDE_FAULT_TOLERANCE:
                self._blind_since = None
                return (0, 0, "fault")
            return (0, 0, None)  # 瞬时侧向故障：短暂停车，容忍单帧毛刺
        if new_frame:
            self._side_fault_streak = 0
        if side_observation is not None and side_observation.ambiguous:
            self._prev_err = None
            self._d_err = 0.0
            self._observe_recentering(new_frame=new_frame, dual_wall=False, error=None, heading_error=None)
            return (0, 0, "side_uncertain")
        if self._paused_at is not None:
            return (0, 0, None)

        # 陀螺仪航向误差（相对本段起始航向，归一到 ±180°）
        err_heading = 0.0
        if yaw is not None:
            if self._ref_yaw is None:
                self._ref_yaw = yaw  # 段起点 yaw 未就绪时，延迟到首个有效 yaw 再设参考
            else:
                err_heading = _unwrap(yaw - self._ref_yaw)
        heading_error = 0.0 if abs(err_heading) <= config.CENTER_DEADBAND_DEG else err_heading

        if ls == "open" and rs == "open":
            # 盲走：没有侧墙时仍用陀螺仪维持本段航向，超过盲走超时停车。
            now = time.monotonic()
            if self._blind_since is None:
                self._blind_since = now
            self._prev_err = None
            self._d_err = 0.0
            self._observe_recentering(new_frame=new_frame, dual_wall=False, error=None, heading_error=None)
            if now - self._blind_since >= config.CENTER_BLIND_TIMEOUT_S:
                return (0, 0, "blind_timeout")
            w = -config.CENTER_K_HEADING * heading_error
            w = max(-config.CENTER_MAX_ANGULAR_MRADPS, min(config.CENTER_MAX_ANGULAR_MRADPS, w))
            return (self._limited_v(v), int(round(w)), None)

        self._blind_since = None
        if ls == "open":                       # 只剩右墙：target - right
            err = config.CENTER_SINGLE_WALL_TARGET_MM - float(right_mm)
        elif rs == "open":                     # 只剩左墙：left - target
            err = float(left_mm) - config.CENTER_SINGLE_WALL_TARGET_MM
        else:                                   # 双墙：正=偏右，需左转
            err = float(left_mm - right_mm) - self._side_bias_mm

        self._observe_recentering(new_frame=new_frame, dual_wall=dual_wall, error=err,
                                  heading_error=err_heading if yaw is not None else None)
        v = self._limited_v(v)
        # 侧距和航向各自使用死区。侧距回到容差内时清空 PID 历史，避免下一帧
        # 仅由航向误差驱动时带入过期的侧向微分项。
        side_error = 0.0 if abs(err) <= config.CENTER_DEADBAND_MM else err
        if side_error == 0.0:
            self._prev_err = None
            self._d_err = 0.0
        elif new_frame:
            self._d_err = 0.0 if self._prev_err is None else (err - self._prev_err)
            self._prev_err = err
        if side_error == 0.0 and heading_error == 0.0:
            return (v, 0, None)

        w = (config.CENTER_KP * side_error + config.CENTER_KD * self._d_err
             - config.CENTER_K_HEADING * heading_error)
        w = max(-config.CENTER_MAX_ANGULAR_MRADPS, min(config.CENTER_MAX_ANGULAR_MRADPS, w))
        return (v, int(round(w)), None)


# ---------------------------------------------------------------------------
# 原地转弯闭环（陀螺仪 yaw）
# ---------------------------------------------------------------------------
def _unwrap(delta: float) -> float:
    """把有向增量归一到 [-180, 180)。"""
    return ((delta + 180.0) % 360.0) - 180.0


def _sign(x: float) -> int:
    return 1 if x >= 0 else -1


class TurnExecutor:
    """用 STM32 相对 yaw 闭环原地转到目标角度。

    仅接受 ODOM_VALID 且新鲜（mcu.yaw_deg 非 None）的 yaw；用有向增量累积，
    修复 +180° 跨边界被算成 ~360° 余量；带截止时间，超时/无 yaw 即停。
    """

    def __init__(self, mcu, ramp: VelocityRamp | None = None) -> None:
        self.mcu = mcu
        self.ramp = ramp
        self._target: float = 0.0
        self._accum: float = 0.0
        self._prev_yaw: float | None = None
        self._target_yaw_deg: float | None = None
        self._deadline: float = 0.0
        self._decelerating = False

    def begin(self, target_deg: float) -> None:
        self._target = target_deg
        self._accum = 0.0
        self._prev_yaw = None
        self._target_yaw_deg = None
        self._deadline = time.monotonic() + config.TURN_TIMEOUT_S
        self._decelerating = False

    @property
    def target_yaw_deg(self) -> float | None:
        """Keep the original target through final command and stop confirmation."""
        return self._target_yaw_deg

    def tick(self) -> str:
        """单步：仅以 yaw 闭环转向，返回 "done"/"running"/"timeout"/"no_yaw"。"""
        if time.monotonic() > self._deadline:
            self._hard_stop()
            return "timeout"
        yaw = self.mcu.yaw_deg  # 仅在 ODOM_VALID 且新鲜时非 None
        if yaw is None:
            # 无新鲜 yaw：不盲目转，零速等待（受截止时间约束）
            self._hard_stop()
            return "no_yaw"
        if self._prev_yaw is None:
            self._prev_yaw = yaw
            self._target_yaw_deg = _unwrap(yaw + self._target)
            self._send(0, _sign(self._target) * config.TURN_ANGULAR_MRADPS)
            return "running"
        self._accum += _unwrap(yaw - self._prev_yaw)
        self._prev_yaw = yaw
        remaining = self._target - self._accum
        # 预见式刹车：进入"减速距离"内即开始降角速度，让车在目标处停稳（消除减速过程 ~4° 过冲）。
        if self.ramp is None:
            stop_trigger = config.TURN_YAW_TOLERANCE_DEG  # 无 ramp：瞬间停，按容差判到位
        else:
            # 预测减速距离再小，也必须在最终航向容差窗口内开始停车。
            stop_trigger = max(config.TURN_YAW_TOLERANCE_DEG, self._decel_deg())
        if self._decelerating or abs(remaining) <= stop_trigger:
            self._decelerating = True
            self._send(0, 0)
            if self.ramp is None or self.ramp.at_rest():
                # 平滑停车后再以最终 yaw 累积值裁决，避免仅凭提前减速触发就宣称转到位。
                if abs(self._target - self._accum) <= config.TURN_YAW_TOLERANCE_DEG:
                    return "done"
                return "misaligned"
            return "running"
        self._send(0, _sign(remaining) * config.TURN_ANGULAR_MRADPS)
        return "running"

    def _decel_deg(self) -> float:
        """从当前角速度降到 0 还会再转过的角度（度）。无 ramp 时瞬间停 → 0。"""
        if self.ramp is None:
            return 0.0
        w = abs(self.ramp.w_mradps)
        if w == 0:
            return 0.0
        accel = config.RAMP_W_STEP_MRADPS / 0.02  # 每拍 20ms 减速 → 角减速度 mrad/s²
        return math.degrees((w * w) / (2.0 * accel) / 1000.0)

    def _send(self, v: int, w: int) -> None:
        """经平滑层下发（若无 ramp 则直接发），保证转弯角速度渐变。"""
        if self.ramp is None:
            command = (v, w)
        else:
            command = self.ramp.step(v, w)
        # McuLink 的纯旋转路径不要求四向/上层测距；兼容测试替身仍只提供
        # send_motion(v, w) 的旧接口。
        if command[0] == 0 and command[1] != 0:
            send_rotation = getattr(self.mcu, "send_rotation", None)
            if callable(send_rotation):
                send_rotation(command[1])
                return
        self.mcu.send_motion(*command)

    def _hard_stop(self) -> None:
        """Binary STOP bypasses the normal target ramp on a turn fault."""
        try:
            self.mcu.send_stop(StopReason.MISSION)
        finally:
            if self.ramp is not None:
                self.ramp.reset()


# ---------------------------------------------------------------------------
# 运动平滑（斜率限制器）+ 预见式刹车
# ---------------------------------------------------------------------------
def _approach(current: int, target: int, step: int) -> int:
    """朝 target 逼近一步（最多 step），step>0。"""
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return current


class VelocityRamp:
    """Limit each actually transmitted 100 ms target; STOP bypasses this ramp."""

    def __init__(self, max_dv: int | None = None, max_dw: int | None = None) -> None:
        self.max_dv = config.RAMP_V_STEP_MMPS if max_dv is None else max_dv
        self.max_dw = config.RAMP_W_STEP_MRADPS if max_dw is None else max_dw
        self._v = 0
        self._w = 0

    def reset(self) -> None:
        """把内部输出清零，使下一次 step 从静止起步（用于硬停后对齐状态）。"""
        self._v = 0
        self._w = 0

    def set_sent(self, linear_mmps: int, angular_mradps: int) -> None:
        """Align the ramp after an immediate safety speed cap was transmitted."""
        self._v, self._w = int(linear_mmps), int(angular_mradps)

    def step(self, target_v: int, target_w: int, *, max_linear_mmps: int | None = None) -> tuple[int, int]:
        if target_v == 0:
            self._v = 0
        else:
            down_step = int(getattr(config, "RAMP_V_DOWN_STEP_MMPS", 25))
            accelerating = (self._v == 0 or self._v * target_v < 0
                            or abs(target_v) > abs(self._v))
            self._v = _approach(self._v, target_v, self.max_dv if accelerating else down_step)
        self._w = _approach(self._w, target_w, self.max_dw)
        if max_linear_mmps is not None:
            # Safety ceilings constrain the command now, including the stored
            # ramp state, so a later tick cannot rebound from an older speed.
            limit = max(0, int(max_linear_mmps))
            self._v = max(-limit, min(limit, self._v))
        return (self._v, self._w)

    @property
    def w_mradps(self) -> int:
        """当前实际下发的角速度（转弯预见刹车估算减速距离用）。"""
        return self._w

    def at_rest(self) -> bool:
        """内部 (v,w) 是否已降到 0（转弯"停稳才算完成"）。"""
        return self._v == 0 and self._w == 0


class ApproachBrake:
    """预见式刹车：按「离路口剩余距离」提前把目标线速度降下来。

    纯逻辑，不碰硬件。侧激光 ``side_opened`` 仍是最终路口裁决；本类只决定何时
    开始减速，因此即使距离估计偏早/偏晚都只会"多刹一点"或"少刹一点"，不会错过路口。

    只降速、不提速：输入 v 已经 ≤ 下限或本就想停（瞬时侧向故障 v=0）时原样返回。
    """

    def __init__(
        self,
        road_len_mm: float,
        enabled: bool = True,
        brake_short_roads: bool = False,
    ) -> None:
        self.road_len_mm = float(road_len_mm)
        self.enabled = enabled
        self.brake_short_roads = brake_short_roads

    def scale_v(self, v: int, dist_traveled: float) -> int:
        if not self.enabled:
            return v
        if v <= 0:
            return v  # 车本就想停（瞬时侧向故障等），不强制提速
        if v <= config.BRAKE_MIN_MMPS:
            return v  # 已足够慢，不再降
        remaining = self.road_len_mm - float(dist_traveled)
        if remaining > config.BRAKE_START_MM:
            return v  # 还远，全速
        if remaining <= 0.0:
            return config.BRAKE_MIN_MMPS  # 预测越界后蠕行，等侧激光确认
        scaled = int(round(config.CENTER_LINEAR_MMPS * remaining / config.BRAKE_START_MM))
        return max(config.BRAKE_MIN_MMPS, min(v, scaled))
