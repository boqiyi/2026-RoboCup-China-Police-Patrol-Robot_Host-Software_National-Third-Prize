"""涵洞巡检：停车后水平扫描左右内壁，只播报稳定且无编号冲突的结果。"""

from __future__ import annotations

from dataclasses import dataclass
import inspect
import math
import time

import config
import gimbal as gimbal_mod
import runlog
from drivers.protocol_v2 import StopReason
from mcu import McuError


@dataclass(frozen=True)
class SideScanResult:
    class_ids: tuple[int, ...]
    frame_failures: int
    inference_failures: int
    valid_inference_frames: int
    post_move_frame_discarded: bool
    class_conflict: bool = False
    errors: tuple[str, ...] = ()
    interrupted: bool = False


@dataclass(frozen=True)
class CulvertInspectionResult:
    left: SideScanResult | None
    right: SideScanResult | None
    class_ids: tuple[int, ...]
    invalid_class_ids: tuple[int, ...]
    complete: bool
    navigation_view_restored: bool = False
    errors: tuple[str, ...] = ()
    queued_class_ids: tuple[int, ...] = ()
    audio_errors: tuple[str, ...] = ()

    @property
    def no_valid_target(self) -> bool:
        return self.complete and not self.class_ids

    @property
    def has_class_conflict(self) -> bool:
        return bool((self.left and self.left.class_conflict) or (self.right and self.right.class_conflict))

    def __iter__(self):
        return iter(self.class_ids)

    def __len__(self) -> int:
        return len(self.class_ids)

    def __bool__(self) -> bool:
        return bool(self.class_ids)


class CulvertFlow:
    def __init__(self, mcu, gimbal, camera, detector, audio, clues, *, require_upper: bool = False) -> None:
        self.mcu = mcu
        self.gimbal = gimbal
        self.camera = camera
        self.detector = detector
        self.audio = audio
        self.clues = clues
        self.require_upper = require_upper
        self._discarded_frame_sequence: int | None = None
        self._discarded_frame_monotonic: float | None = None
        self._discard_error: str | None = None

    def inspect(self, deadline: float | None = None, frame_observer=None) -> CulvertInspectionResult:
        """停车后依次扫描左右，最后恢复水平居中导航姿态；任一步失败则不完整。"""
        left: SideScanResult | None = None
        right: SideScanResult | None = None
        complete = False
        navigation_view_restored = False
        errors: list[str] = []
        observer_failed = False

        def observe_frame(frame, detections, sequence, captured_monotonic):
            nonlocal observer_failed
            if frame_observer is None or observer_failed:
                return
            try:
                frame_observer(frame, detections, sequence, captured_monotonic)
            except Exception as exc:  # 预览故障不能改变本次识别结果
                observer_failed = True
                runlog.exception("culvert.preview", "扫描预览已停止更新", exc)

        started = time.monotonic()
        local_deadline = started + getattr(config, "CULVERT_INSPECTION_TIMEOUT_S", 15.0)
        if deadline is not None and not math.isfinite(deadline):
            deadline = started
            errors.append("扫描截止时间不是有限值")
        deadline = local_deadline if deadline is None else min(deadline, local_deadline)
        restore_budget = config.SERVO_SETTLE_S + config.MCU_COMMAND_CONFIRM_TIMEOUT_S + config.MCU_WRITE_TIMEOUT_S
        scan_deadline = deadline - restore_budget
        runlog.status(phase="culvert_scan", side="stopping")
        runlog.emit("culvert.begin", "开始本次涵洞扫描", deadline=deadline)
        try:
            self.mcu.send_stop(StopReason.CULVERT)
            self._pump_wait(0.5, scan_deadline)
            if not self._deadline_expired(scan_deadline):
                runlog.status(phase="culvert_scan", side="left")
                self._move_gimbal("pan_left", scan_deadline)
                left = self._scan_side(
                    scan_deadline,
                    post_move_frame_discarded=self._discard_post_move_frame(),
                    frame_observer=observe_frame,
                )
            if left is not None and not left.interrupted and not self._deadline_expired(scan_deadline):
                runlog.status(phase="culvert_scan", side="right")
                self._move_gimbal("pan_right", scan_deadline)
                right = self._scan_side(
                    scan_deadline,
                    post_move_frame_discarded=self._discard_post_move_frame(),
                    frame_observer=observe_frame,
                )
            complete = not self._deadline_expired(scan_deadline) and self._scans_usable(left, right)
            if not complete:
                errors.append("扫描截止或左右有效帧不足/编号冲突，未完整确认")
        except Exception as exc:  # noqa: BLE001 - 云台/串口失败必须以未完成巡检收口
            complete = False
            errors.append(str(exc))
            runlog.exception("culvert.scan_exception", "涵洞扫描中断", exc,
                             left_completed=left is not None, right_completed=right is not None)
        finally:
            try:
                runlog.status(phase="culvert_scan", side="restoring_navigation")
                # 即使比赛刚截止，也要最后发一次水平居中恢复命令；Gimbal 在截止后不会继续等待。
                self._move_gimbal("tilt_look", deadline)
                navigation_view_restored = not self._deadline_expired(deadline)
            except Exception as exc:  # noqa: BLE001 - 无法确认恢复时留给主循环安全停车
                navigation_view_restored = False
                errors.append(f"导航视角恢复失败: {exc}")
                runlog.exception("culvert.restore_exception", "导航视角恢复失败", exc)
        return self._finish_inspection(
            left,
            right,
            complete=complete and navigation_view_restored,
            navigation_view_restored=navigation_view_restored,
            errors=tuple(errors),
        )

    @staticmethod
    def _deadline_expired(deadline: float | None) -> bool:
        return deadline is not None and time.monotonic() >= deadline

    @staticmethod
    def _scans_usable(left: SideScanResult | None, right: SideScanResult | None) -> bool:
        min_valid_frames = max(2, int(config.CULVERT_MIN_VALID_INFERENCE_FRAMES))
        return bool(
            left
            and right
            and left.post_move_frame_discarded
            and right.post_move_frame_discarded
            and left.valid_inference_frames >= min_valid_frames
            and right.valid_inference_frames >= min_valid_frames
            and not left.interrupted and not right.interrupted
            and not left.class_conflict and not right.class_conflict
        )

    def _move_gimbal(self, method_name: str, deadline: float | None) -> None:
        """把 deadline 交给新云台实现，同时兼容没有该关键字的既有 fake。"""
        method = getattr(self.gimbal, method_name)
        if deadline is None:
            method()
            return
        try:
            parameters = inspect.signature(method).parameters.values()
            accepts_deadline = any(
                parameter.name == "deadline"
                or parameter.kind is inspect.Parameter.VAR_KEYWORD
                for parameter in parameters
            )
        except (TypeError, ValueError):
            accepts_deadline = True
        if accepts_deadline:
            method(deadline=deadline)
        else:
            method()

    def _discard_post_move_frame(self) -> bool:
        """换向后主动取走一帧缓存；该帧绝不进入识别。"""
        self._discarded_frame_sequence = None
        self._discarded_frame_monotonic = None
        self._discard_error = None
        try:
            captured = self._grab_scan_frame()
            if captured is None:
                self._discard_error = "换向后没有有效缓存帧可丢弃"
                return False
            _frame, sequence, captured_monotonic = captured
            self._discarded_frame_sequence = sequence
            self._discarded_frame_monotonic = captured_monotonic
            return True
        except Exception as exc:  # noqa: BLE001 - 无法丢弃旧帧则本侧巡检不能完整
            self._discard_error = str(exc)
            runlog.exception("culvert.discard_frame", "换向后的缓存帧读取失败", exc)
            return False

    def _finish_inspection(
        self,
        left: SideScanResult | None,
        right: SideScanResult | None,
        *,
        complete: bool,
        navigation_view_restored: bool,
        errors: tuple[str, ...] = (),
    ) -> CulvertInspectionResult:
        observed = set((left.class_ids if left else ()) + (right.class_ids if right else ()))
        try:
            if self.clues is None:
                raise ValueError("目标类别与音频映射组件不可用")
            valid = sorted(cid for cid in observed if self.clues.detection(cid) is not None)
        except Exception as exc:  # noqa: BLE001
            valid = []
            complete = False
            errors += (str(exc),)
            runlog.exception("culvert.mapping", "涵洞识别结果映射失败", exc)
        invalid = sorted(cid for cid in observed if cid not in valid)
        queued: tuple[int, ...] = ()
        audio_errors: tuple[str, ...] = ()
        if complete and valid:
            queued, audio_errors = self._report(tuple(valid))
        result = CulvertInspectionResult(
            left=left,
            right=right,
            class_ids=tuple(valid),
            invalid_class_ids=tuple(invalid),
            complete=complete,
            navigation_view_restored=navigation_view_restored,
            errors=errors,
            queued_class_ids=queued,
            audio_errors=audio_errors,
        )
        runlog.emit("culvert.result", "涵洞扫描结束，音频记录表示入队而非播完",
                    complete=result.complete, class_ids=result.class_ids,
                    navigation_view_restored=result.navigation_view_restored,
                    queued_class_ids=queued, audio_errors=audio_errors, errors=errors,
                    left_errors=left.errors if left is not None else (),
                    right_errors=right.errors if right is not None else ())
        return result

    def _pump_wait(self, seconds: float, deadline: float | None = None) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self._deadline_expired(deadline):
                return
            self.mcu.pump()
            telemetry_ok = getattr(self.mcu, "telemetry_ok", None)
            if callable(telemetry_ok):
                reason = (telemetry_ok(require_upper=self.require_upper)
                          if gimbal_mod._accepts_keyword(telemetry_ok, "require_upper")
                          else telemetry_ok())
                if reason is not None:
                    raise McuError(f"涵洞等待期间遥测异常: {reason}")
            poll = getattr(self.audio, "poll", None)
            if callable(poll):
                try:
                    poll()
                except Exception as exc:  # noqa: BLE001 - 音频故障不得中断停车等待
                    runlog.exception("culvert.audio_poll", "扫描时音频队列处理失败", exc)
            remaining = end - time.monotonic()
            if deadline is not None:
                remaining = min(remaining, deadline - time.monotonic())
            time.sleep(max(0.0, min(0.02, remaining)))

    def _grab_scan_frame(self):
        """返回可推理帧及其元数据；旧 fake 仅有 grab_bgr 时保持兼容。"""
        grab_frame = getattr(self.camera, "grab_frame", None)
        if not callable(grab_frame):
            bgr = self.camera.grab_bgr()
            return (bgr, None, None) if bgr is not None else None
        captured = grab_frame()
        if captured is None:
            return None
        fresh = getattr(captured, "is_fresh", None)
        if callable(fresh) and not fresh(config.CAMERA_FRAME_MAX_AGE_S):
            return None
        sequence = getattr(captured, "sequence", None)
        captured_monotonic = getattr(captured, "captured_monotonic", None)
        if (
            not isinstance(sequence, int)
            or isinstance(sequence, bool)
            or not isinstance(captured_monotonic, (int, float))
            or isinstance(captured_monotonic, bool)
            or not math.isfinite(captured_monotonic)
        ):
            return None
        bgr = getattr(captured, "bgr", None)
        return (bgr, sequence, float(captured_monotonic)) if bgr is not None else None

    def _grab_bgr(self):
        captured = self._grab_scan_frame()
        return captured[0] if captured is not None else None

    @staticmethod
    def _class_group(class_id: int) -> str | None:
        if 1 <= class_id <= 10:
            return "suspect"
        if 11 <= class_id <= 20:
            return "knife"
        return None

    def _scan_side(
        self,
        deadline: float | None = None,
        *,
        post_move_frame_discarded: bool,
        frame_observer=None,
    ) -> SideScanResult:
        """同类别连续命中才确认；同一类别的冲突编号静默，人物和刀具可同时确认。"""
        current = {"suspect": {}, "knife": {}}
        maximum = {"suspect": {}, "knife": {}}
        conflicts: set[str] = set()
        frame_failures = 0
        inference_failures = 0
        valid_inference_frames = 0
        errors: list[str] = []
        interrupted = False
        if not post_move_frame_discarded:
            errors.append(self._discard_error or "未确认已丢弃换向缓存帧")
        previous_sequence = self._discarded_frame_sequence
        previous_captured_monotonic = self._discarded_frame_monotonic
        for _ in range(max(1, int(config.CULVERT_SCAN_FRAME_COUNT))):
            if self._deadline_expired(deadline):
                interrupted = True
                errors.append("本侧扫描到达截止时间")
                break
            try:
                self._pump_wait(config.CULVERT_FRAME_MIN_INTERVAL_S, deadline)
            except Exception as exc:  # noqa: BLE001
                errors.append(f"遥测等待失败: {exc}")
                interrupted = True
                runlog.exception("culvert.telemetry_wait", "本侧扫描遥测中断", exc)
                break
            if self._deadline_expired(deadline):
                interrupted = True
                errors.append("采样间隔等待后已到截止时间")
                break
            try:
                captured = self._grab_scan_frame()
            except Exception as exc:  # noqa: BLE001
                captured = None
                errors.append(f"抓帧异常: {exc}")
                runlog.exception("culvert.capture", "涵洞抓帧异常", exc)
            if captured is None:
                frame_failures += 1
                errors.append("无可用新鲜帧")
                for values in current.values():
                    values.clear()
                continue
            frame, sequence, captured_monotonic = captured
            if sequence is not None:
                independent = (
                    (previous_sequence is None or sequence > previous_sequence)
                    and (
                        previous_captured_monotonic is None
                        or captured_monotonic > previous_captured_monotonic
                    )
                )
                previous_sequence = sequence
                previous_captured_monotonic = captured_monotonic
                if not independent:
                    frame_failures += 1
                    errors.append("帧序号或采集时间未递增")
                    for values in current.values():
                        values.clear()
                    continue
            try:
                if self._deadline_expired(deadline):
                    raise TimeoutError("推理前已到扫描截止时间")
                infer = self.detector.infer_bgr
                timeout = getattr(config, "DETECTOR_INFER_TIMEOUT_S", 0.15)
                if deadline is not None:
                    timeout = min(timeout, deadline - time.monotonic())
                parameters = inspect.signature(infer).parameters.values()
                accepts_timeout = any(p.name == "timeout_s" or p.kind is inspect.Parameter.VAR_KEYWORD
                                      for p in parameters)
                detections = infer(frame, timeout_s=timeout) if accepts_timeout else infer(frame)
                if self._deadline_expired(deadline):
                    raise TimeoutError("推理返回时已到扫描截止时间")
                if not isinstance(detections, (list, tuple)):
                    raise ValueError("推理未返回检测列表")
            except Exception as exc:  # noqa: BLE001
                inference_failures += 1
                errors.append(f"推理异常: {exc}")
                runlog.exception("culvert.inference", "涵洞推理异常", exc)
                for values in current.values():
                    values.clear()
                if isinstance(exc, TimeoutError) or self._deadline_expired(deadline):
                    interrupted = True
                    break
                continue
            valid_inference_frames += 1
            if frame_observer is not None:
                frame_observer(frame, detections, sequence, captured_monotonic)

            seen = {"suspect": set(), "knife": set()}
            for det in detections:
                class_id = getattr(det, "class_id", None)
                try:
                    score = float(getattr(det, "score", None))
                except (TypeError, ValueError):
                    continue
                if (
                    not isinstance(class_id, int) or isinstance(class_id, bool)
                    or not math.isfinite(score)
                    or score < config.YOLO_CONF_THRESHOLD
                ):
                    continue
                group = self._class_group(class_id)
                if group is not None:
                    seen[group].add(class_id)

            for group, ids in seen.items():
                active = current[group]
                if len(ids) > 1:
                    conflicts.add(group)
                    active.clear()
                    continue
                if not ids:
                    active.clear()
                    continue
                class_id = next(iter(ids))
                for other in list(active):
                    if other != class_id:
                        active[other] = 0
                active[class_id] = active.get(class_id, 0) + 1
                maximum[group][class_id] = max(maximum[group].get(class_id, 0), active[class_id])

        found: list[int] = []
        for group, maximums in maximum.items():
            stable = [
                class_id
                for class_id, streak in maximums.items()
                if streak >= config.DETECT_HIT_STREAK
            ]
            if len(stable) > 1:
                conflicts.add(group)
            elif len(stable) == 1 and group not in conflicts:
                found.append(stable[0])
        return SideScanResult(
            tuple(sorted(found)),
            frame_failures,
            inference_failures,
            valid_inference_frames,
            post_move_frame_discarded,
            bool(conflicts),
            tuple(errors),
            interrupted,
        )

    def _report(self, class_ids: tuple[int, ...]) -> tuple[tuple[int, ...], tuple[str, ...]]:
        queued: list[int] = []
        errors: list[str] = []
        for class_id in class_ids:
            if self.clues.detection(class_id) is None:
                continue
            try:
                accepted = self.audio is not None and self.audio.play(self.clues.detection_audio_path(class_id))
                if accepted:
                    queued.append(class_id)
                    runlog.emit("culvert.audio_queued", "目标播报已入队，未声明播放完成", class_id=class_id)
                else:
                    errors.append(f"类别 {class_id} 播报未入队")
            except Exception as exc:  # noqa: BLE001 - 音频失败不得终止比赛
                errors.append(f"类别 {class_id} 播报异常: {exc}")
                runlog.exception("culvert.audio_enqueue", "目标播报入队失败", exc, class_id=class_id)
        return tuple(queued), tuple(errors)
