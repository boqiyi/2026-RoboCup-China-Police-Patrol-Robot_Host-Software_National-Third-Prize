"""Competition transport adapters on the verified Stage 1 v12 link.

The coordinator owns task precedence.  This module only makes its final motion
veto and parked gimbal evidence apply to the actual UART write.
"""

from __future__ import annotations

from contextlib import nullcontext
import math
import time

import config
import mcu
import runlog
from drivers.protocol_v2 import (
    AckStatus, ControlFlag, MessageType, MotionActionCode, StateFlag,
    decode_control_target, decode_motion_action,
)
import test_stage1_patrol as stage1
from test_stage1_patrol import PatrolLink


class CompetitionMotionBlocked(Exception):
    """A pending coordinator task prevented an unsent nonzero command."""
    motion_not_written = True


class _CompetitionGuardedSerial:
    def __init__(self, serial_port, guard):
        self.serial_port, self.guard = serial_port, guard
        self.armed, self.write_attempted = True, False
        self._host_write_started_at = None

    def __getattr__(self, name):
        return getattr(self.serial_port, name)

    def write(self, data):
        if self.armed:
            self.armed = False
            self.guard()
        self.write_attempted = True
        self._host_write_started_at = time.monotonic()
        return self.serial_port.write(data)


class CompetitionLink(PatrolLink):
    """One v12 session, Stage 1 safety, and exact parked PWM confirmation.

    ``competition_motion_guard`` is a zero-argument, read-only callback.  False
    or CompetitionMotionBlocked vetoes the pending UART write.  It must not
    pump UART, perform detection, or send commands.  A coordinator may supply
    ``competition_motion_context`` to serialize detection publication with the
    write, and ``competition_motion_sent`` to observe successful motion sends.
    """

    def __init__(self, *args, **kwargs):
        self.competition_motion_guard = None
        self.competition_motion_context = None
        self.competition_motion_sent = None
        self.competition_before_motion_write = None
        self.competition_write_intent = None
        self.competition_gimbal_monitor = None
        self.competition_gimbal_pending = None
        self.competition_recovery_progress = None
        super().__init__(*args, **kwargs)
        self.set_motion_deadline(None)

    def _reset_link_state(self):
        super()._reset_link_state()
        self.competition_gimbal_monitor = None
        self.competition_gimbal_pending = None
        self.competition_write_intent = None

    def recover_fault(self, display, *, input_source=None):
        """Resume confirmed recovery steps without resetting the saved world goal.

        Only this competition adapter uses the checkpoint. Successful clock
        synchronization survives later failures. A failed one-shot clock probe
        needs a new legal HELLO; a gap while proving stillness keeps the
        confirmed BRAKE and its original source barrier.
        """
        sequence = stage1.sequence
        self._patrol_display = display
        if input_source is not None:
            self._patrol_input = input_source
        progress = self.competition_recovery_progress
        if progress is None:
            progress = dict(stage="brake_before", identity=(self.session_nonce, self.boot_id),
                            before=None, stop=None, retry_at=0., migrated=False, pose_done=False)
            self.competition_recovery_progress = progress
        self._patrol_recovery_input = input_source
        self._patrol_recovery_display = display
        while True:
            sequence._check_input_exit(input_source)
            sequence._remaining(getattr(display, "patrol_deadline", math.inf))
            identity = (self.session_nonce, self.boot_id)
            if progress["identity"] != identity:
                self.patrol_manual_required = "恢复期间会话或boot意外变化，无法验证原坐标，等待人工"
            self.pump()
            try:
                display.tick(pump=False, check_health=False)
                display.consume_samples(require_healthy=False, collect_means=False, allow_stale=True)
            except mcu.McuError:
                pass
            display.action = "故障恢复等待：接续已确认步骤，保留原转弯终点"
            display.confirmation = self.patrol_manual_required or self._hard_fault_reason or "等待新鲜健康遥测"
            display.maybe_print()
            if self.patrol_manual_required:
                self._observe_uturn_manual_stop(display)
                time.sleep(sequence.CONTROL_INTERVAL_S)
                continue
            now = time.monotonic()
            if now < progress["retry_at"]:
                time.sleep(sequence.CONTROL_INTERVAL_S)
                continue
            try:
                phase = progress["stage"]
                if phase in ("brake_before", "brake_after", "brake_final"):
                    if not self.connected or self._ser is None:
                        time.sleep(sequence.CONTROL_INTERVAL_S)
                        continue
                    number = self.send_motion_action(MotionActionCode.BRAKE, 0,
                                                     require_ranges=False, require_upper=False)
                    display.set_stopped()
                    stage1.wait_brake_applied_in_fault(self, number)
                    self._brake_evidence_barrier = stage1._brake_evidence_at(self)
                    progress["stop"] = stage1.FaultStopTracker(self, number)
                    self._stop_tracker = progress["stop"].tracker
                    self._stable_barrier = None
                    progress["stage"] = ("wait_before" if phase == "brake_before" else
                                         "wait_final" if phase == "brake_final" else "wait_after")
                    runlog.emit("competition_recovery_step", "BRAKE已确认，接续严格500ms停稳",
                                stage=progress["stage"], sequence=number)
                if progress["stage"] in ("wait_before", "wait_after", "wait_final", "health"):
                    anchor = progress["stop"].update(getattr(display, "state_samples", None),
                                                     getattr(display, "trace_samples", None))
                    if anchor is None:
                        self._stable_barrier = None
                        time.sleep(sequence.CONTROL_INTERVAL_S)
                        continue
                    if self._stable_barrier is None:
                        self._stable_barrier = time.monotonic()
                    self._patrol_stationary_anchor = anchor
                    if progress["stage"] == "wait_after":
                        if progress["before"] is not None and not progress["migrated"]:
                            if not self.migrate_hello_anchor(progress["before"], anchor):
                                continue
                            progress["migrated"] = True
                        progress["stage"] = "health"
                    live = self._live_health_reason()
                    fault_at = self._patrol_fault_at if self._patrol_fault_at is not None else -math.inf
                    fresh = (self.state is not None and self.diagnostics is not None
                             and self._sample_time("state") > fault_at
                             and self._sample_time("diagnostics") > fault_at)
                    if live is None and fresh and not self._pending_commands:
                        self._hard_fault_reason = self._patrol_fault_at = None
                        self.last_parse_error = None
                        if progress["stage"] == "wait_final":
                            self.competition_recovery_progress = None
                            runlog.emit("competition_recovery_resumed", "PWM后最终BRAKE与严格停稳完成，接续原绝对终点",
                                        checkpoint=self.patrol_checkpoint, migrated=progress["migrated"])
                            return
                        progress["stage"] = "brake_final" if progress["pose_done"] else "pose"
                    # Missing telemetry after HELLO waits for the same session.
                    # A new real MCU latch may require another legal HELLO.
                    new_latch = (self.state is not None and
                                 self._active_fault_bits())
                    if (progress["stage"] == "wait_before" and self._hard_fault_reason is not None
                            or new_latch):
                        progress["before"] = dict(anchor)
                        progress["stage"] = "hello"
                        progress["pose_done"] = False
                if progress["stage"] == "hello":
                    # A failed zero-only clock probe cannot move the chassis.
                    # Retain the original 500ms parking anchor, but recheck
                    # current zero output/positions before another HELLO.
                    state, before = self.state, progress["before"]
                    if before is None or state is None or not self.connected:
                        time.sleep(sequence.CONTROL_INTERVAL_S)
                        continue
                    if (state.boot_id != before["boot_id"]
                            or (state.left_mm, state.right_mm) != before["positions"]):
                        self.patrol_manual_required = "恢复时boot或双轮里程改变，原静止坐标不可验证，等待人工"
                        continue
                    if (state.target_linear_mms or state.target_angular_mrads
                            or state.state_flags & StateFlag.TURN_ACTIVE
                            or not state.state_flags & StateFlag.ODOM_VALID or state.odom_status != 0
                            or (state.session_nonce, state.boot_id) != progress["identity"]
                            or time.monotonic() - self._last_state_recv > config.TELEMETRY_STALE_MS / 1000.):
                        time.sleep(sequence.CONTROL_INTERVAL_S)
                        continue
                    diagnostics = self.diagnostics
                    if (state.fault_bits & ~(stage1.sequence._ignored_runtime_fault_bits(self) | (1 << 20))
                            or diagnostics is None
                            or (diagnostics.session_nonce, diagnostics.boot_id) != progress["identity"]
                            or diagnostics.gyro_status != sequence.WireGyroStatus.READY
                            or not 0 <= diagnostics.gyro_sample_age_ms <= sequence.LASER_MAX_AGE_MS
                            or time.monotonic() - self._last_diagnostics_recv > config.TELEMETRY_STALE_MS / 1000.):
                        time.sleep(sequence.CONTROL_INTERVAL_S)
                        continue
                    self._patrol_session_history.append(dict(identity=identity,
                        ledger=dict(self._command_ledger), faults=list(self.patrol_fault_history)))
                    self._patrol_recovery_hello_active = True
                    try:
                        self.hello(timeout_s=1.0)
                    finally:
                        self._patrol_recovery_hello_active = False
                    progress["identity"] = (self.session_nonce, self.boot_id)
                    progress["migrated"] = False
                    progress["stage"] = "clock"
                if progress["stage"] == "clock":
                    self.synchronize_clock()
                    progress["stage"] = "brake_after"
                    progress["stop"] = None
                if progress["stage"] == "pose":
                    callback = getattr(self, "patrol_recovery_callback", None)
                    if callable(callback):
                        callback()
                    progress["pose_done"] = True
                    progress["stage"] = "brake_final"
                progress["retry_at"] = 0.
            except mcu.McuError as exc:
                self._latch_hard_fault(str(exc))
                # A failed HELLO may replace the provisional nonce. It has not
                # completed a step and is allowed to retry from the same stop.
                if progress["stage"] == "hello":
                    progress["identity"] = (self.session_nonce, self.boot_id)
                elif progress["stage"] == "clock":
                    # synchronize_clock permits one attempt per session. Its
                    # failed evidence cannot be made timely by a second probe.
                    progress["stage"] = "hello"
                elif progress["stage"] in ("health", "pose"):
                    # A failed PWM restoration may replace the applied zero
                    # sequence. Keep HELLO/clock, establish a new BRAKE barrier.
                    progress["stage"], progress["stop"] = "brake_after", None
                progress["retry_at"] = time.monotonic() + 1.
                runlog.emit("competition_recovery_step_wait", "保留已完成恢复步骤并重试当前步骤",
                            stage=progress["stage"], reason=str(exc))
            time.sleep(sequence.CONTROL_INTERVAL_S)

    @staticmethod
    def _payload_moves(message_type, payload):
        if message_type is MessageType.CONTROL_TARGET:
            target = decode_control_target(payload)
            return bool(target.flags & ControlFlag.MOTION_VALID
                        and (target.linear_mms or target.angular_mrads))
        if message_type is MessageType.MOTION_ACTION:
            return decode_motion_action(payload).action is not MotionActionCode.BRAKE
        return False

    def _send(self, message_type, payload):
        moving = self._payload_moves(message_type, payload)
        if not moving:
            return super()._send(message_type, payload)

        def before_write():
            guard = self.competition_motion_guard
            self.competition_write_intent = message_type, payload
            try:
                before = self.competition_before_motion_write
                if before is not None:
                    before()
                if guard is not None and guard() is False:
                    raise CompetitionMotionBlocked("比赛任务已抢占，禁止旧非零目标写入")
            finally:
                self.competition_write_intent = None
            # A callback may consume freshness time.  Recheck without another
            # coordinator callback between these reads and the serial write.
            self._check_motion_deadline()
            self._check_motion_lease()
            reason = self.motion_health_reason()
            if reason is not None:
                raise mcu.McuError(reason)

        original_serial, original_sequence = self._ser, self._seq
        if original_serial is None:
            before_write()
            return super()._send(message_type, payload)
        guarded_serial = _CompetitionGuardedSerial(original_serial, before_write)
        context_factory = self.competition_motion_context
        with context_factory() if context_factory is not None else nullcontext():
            self._ser = guarded_serial
            try:
                number = super()._send(message_type, payload)
            except Exception:
                if not guarded_serial.write_attempted:
                    self._seq = original_sequence
                raise
            finally:
                self._ser = original_serial
            # Disarm the parked monitor before the next moving STATE arrives.
            self.competition_gimbal_monitor = None
            self.competition_gimbal_pending = None
            callback = self.competition_motion_sent
            if callback is not None:
                callback()
            return number

    def send_gimbal(self, pose, pan_us, tilt_us):
        self.competition_gimbal_monitor = None
        self.competition_gimbal_pending = None
        sent_at = time.monotonic()
        baseline = getattr(self.state, "uptime_ms", None)
        # The base semantic API already maps pan/tilt to the firmware's CH2/CH1
        # wire fields.  No second swap belongs in the competition adapter.
        number = super().send_gimbal(pose, pan_us, tilt_us)
        target = (MessageType.CONTROL_TARGET, 0, 0, "gimbal", int(pose),
                  int(pan_us), int(tilt_us))
        self._track_command(MessageType.CONTROL_TARGET, number, sent_at, baseline, target)
        self.competition_gimbal_pending = number
        return number

    @staticmethod
    def _gimbal_state_reason(state, pose, identity):
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

    def _gimbal_failed(self, reason, *, command=None, state=None, raise_error=True):
        record = self._command_ledger.get((MessageType.CONTROL_TARGET, command))
        runlog.emit("competition_gimbal_output_failed", "云台PWM证据未通过，保持停车",
                    reason=reason, sequence=command,
                    target=None if record is None else record[5],
                    ack_received_at_s=None if record is None else record[9],
                    state_received_at_s=None if record is None else record[3],
                    state_uptime_ms=getattr(state, "uptime_ms", None))
        self._stop_failed_confirmation("云台PWM未确认：" + reason)
        if raise_error:
            raise mcu.McuError(self._hard_fault_reason)

    def _accept_state(self, state):
        previous = self.state
        super()._accept_state(state)
        monitor = self.competition_gimbal_monitor
        if monitor is None and self.competition_gimbal_pending is not None:
            number = self.competition_gimbal_pending
            record = self._command_ledger.get((MessageType.CONTROL_TARGET, number))
            if record is not None and record[2] is not None:
                monitor = number, record[5][4], tuple(record[6:8])
        if self.state is state and previous is not state and monitor is not None:
            number, pose, identity = monitor
            reason = self._gimbal_state_reason(state, pose, identity)
            if reason is not None and self._hard_fault_reason is None:
                self._gimbal_failed(reason, command=number, state=state, raise_error=False)

    def wait_for_control_applied(self, number, timeout_s=None):
        key = MessageType.CONTROL_TARGET, number
        record = self._command_ledger.get(key)
        if record is None or len(record[5]) < 4 or record[5][3] != "gimbal":
            return super().wait_for_control_applied(number, timeout_s)
        # Do not use the inherited same-target renewal proof: every PWM request
        # must have its own original ACK and first matching applied STATE.
        deadline = record[0] + min(config.MCU_COMMAND_CONFIRM_TIMEOUT_S,
                                   self._confirm_timeout(timeout_s))
        identity = tuple(record[6:8])
        while True:
            state, ack = record[2], record[8]
            reason = None
            if not self.connected or (self.session_nonce, self.boot_id) != identity:
                reason = "云台确认期间MCU会话变化"
            elif ack is not None and ack.status is not AckStatus.ACCEPTED:
                reason = "本次云台命令被拒：" + ack.status.name
            elif state is not None:
                reason = self._gimbal_state_reason(state, record[5][4], identity)
                if reason is None and (state.applied_control_sequence != number
                        or not state.state_flags & StateFlag.APPLIED_CONTROL_PRESENT):
                    reason = "本次云台命令缺少匹配应用STATE"
            if reason is not None:
                self._gimbal_failed(reason, command=number, state=state)
            if ack is not None and state is not None:
                if ((ack.session_nonce, ack.boot_id) != identity
                        or ack.acknowledged_type is not MessageType.CONTROL_TARGET
                        or ack.acknowledged_sequence != number):
                    self._gimbal_failed("本次云台命令缺少匹配ACCEPTED ACK", command=number, state=state)
                if (record[3] is None or record[9] is None
                        or max(record[3], record[9]) > deadline + 1e-9):
                    self._gimbal_failed("本次云台ACK/STATE未在原200ms确认窗口到达",
                                        command=number, state=state)
                if self._hard_fault_reason is not None:
                    raise mcu.McuError(self._hard_fault_reason)
                self.competition_gimbal_monitor = number, record[5][4], identity
                self.competition_gimbal_pending = None
                runlog.emit("competition_gimbal_output_confirmed", "本次云台PWM命令应用已确认",
                            sequence=number, pan_us=record[5][5], tilt_us=record[5][6],
                            ack_received_at_s=record[9], state_received_at_s=record[3],
                            command_sent_at_s=record[0], state_reports_pwm_us=False,
                            mechanical_angle_verified=False)
                self.check_competition_gimbal_output()
                return state
            if self._hard_fault_reason is not None:
                raise mcu.McuError(self._hard_fault_reason)
            if time.monotonic() >= deadline:
                self._record_confirmation_timeout(key, record, deadline, time.monotonic())
                self._gimbal_failed("本次云台ACK/STATE未在原200ms确认窗口到达",
                                    command=number, state=state)
            self.pump()
            if record[2] is None or record[8] is None:
                time.sleep(.005)

    def check_competition_gimbal_output(self):
        monitor = self.competition_gimbal_monitor
        if monitor is None:
            return
        if self._hard_fault_reason is not None:
            # Recovery and shutdown must still consume UART evidence. Motion
            # health remains responsible for vetoing commands while latched.
            return
        number, pose, identity = monitor
        if not self.connected or not self._sample_is_fresh("state"):
            # A previously accepted PWM request remains accepted while parked.
            # Fresh motor/diagnostic evidence is checked separately at departure.
            return
        reason = self._gimbal_state_reason(self.state, pose, identity)
        if reason is not None:
            self._gimbal_failed(reason, command=number, state=self.state, raise_error=False)

    def check_stage2_gimbal_output(self):
        """Compatibility for the verified Stage 3 parked gimbal adapter."""
        self.check_competition_gimbal_output()

    def pump(self):
        frames = super().pump()
        self.check_competition_gimbal_output()
        return frames
