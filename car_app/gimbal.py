"""云台控制（普通舵机无角度反馈）。

导航与扫描恢复均使用水平居中；pan 使用左/中/右三个 PWM 位置。
上下极限用于完整自检与手动标定；水平 PWM 使用 gimbal-test 实测后写入的共享配置。
上位机只能确认本次命令已应用，不能由 PWM 推断真实机械角度。
"""

from __future__ import annotations

import inspect
import time

import config
import runlog
from drivers.protocol_v2 import AckStatus, MessageType, WireGimbalPose
from mcu import McuError


def _accepts_keyword(method, name: str) -> bool:
    try:
        return any(
            parameter.name == name or parameter.kind is inspect.Parameter.VAR_KEYWORD
            for parameter in inspect.signature(method).parameters.values()
        )
    except (TypeError, ValueError):
        return True


def wait_for_ack(mcu, message_type, sequence: int, deadline: float) -> None:
    """仅本次会话内、类型和序号均匹配的 ACCEPTED ACK 可以确认命令。"""
    if isinstance(sequence, bool) or not isinstance(sequence, int):
        raise McuError("命令没有返回有效序号")
    while time.monotonic() < deadline:
        cached = getattr(mcu, "_cached_ack", None)
        ack = cached(message_type, sequence) if callable(cached) else getattr(mcu, "last_ack", None)
        if (
            ack is not None
            and ack.acknowledged_type == message_type
            and ack.acknowledged_sequence == sequence
        ):
            if ack.status != AckStatus.ACCEPTED:
                raise McuError(f"{message_type.name} seq={sequence} 被拒: {ack.status}")
            return
        mcu.pump()
        time.sleep(max(0.0, min(0.005, deadline - time.monotonic())))
    raise McuError(f"未收到本次 {message_type.name} seq={sequence} ACK")


class Gimbal:
    def __init__(self, mcu) -> None:
        self.mcu = mcu

    @staticmethod
    def _deadline_expired(deadline: float | None) -> bool:
        return deadline is not None and time.monotonic() >= deadline

    @staticmethod
    def _confirm_timeout(deadline: float | None) -> float | None:
        if deadline is None:
            return None
        return max(0.0, min(config.MCU_COMMAND_CONFIRM_TIMEOUT_S, deadline - time.monotonic()))

    def _pan(
        self,
        pose: WireGimbalPose,
        pan_us: int,
        tilt_us: int,
        *,
        action: str,
        deadline: float | None = None,
        restore: bool = False,
    ) -> int:
        """截止后只允许发送导航恢复命令，不再开始新的扫描动作。"""
        if self._deadline_expired(deadline) and not restore:
            raise McuError("比赛截止，未发送新的云台扫描命令")
        sequence = self.mcu.send_gimbal(pose, pan_us, tilt_us)
        runlog.emit("gimbal.command", "已发送云台 PWM 命令", sequence=sequence,
                    action=action, pose=pose.name, pan_us=pan_us, tilt_us=tilt_us,
                    ch1_pwm_us=tilt_us, ch2_pwm_us=pan_us)
        if self._deadline_expired(deadline):
            raise McuError("比赛截止，云台命令未能在时限内确认")

        confirm = getattr(self.mcu, "wait_for_control_applied", None)
        if callable(confirm):
            command_deadline = time.monotonic() + config.MCU_COMMAND_CONFIRM_TIMEOUT_S
            if deadline is not None:
                command_deadline = min(command_deadline, deadline)
            timeout_s = command_deadline - time.monotonic()
            if timeout_s <= 0:
                raise McuError("云台确认时间已用尽")
            if _accepts_keyword(confirm, "timeout_s"):
                confirm(sequence, timeout_s=timeout_s)
            else:
                confirm(sequence)
            # 正式 McuLink 提供 ACK 缓存；保留不含协议层的旧纯逻辑替身。
            if hasattr(self.mcu, "_cached_ack") or hasattr(self.mcu, "last_ack"):
                wait_for_ack(self.mcu, MessageType.CONTROL_TARGET, sequence, command_deadline)
        if self._deadline_expired(deadline):
            raise McuError("比赛截止，云台命令未能在时限内确认")

        # 到位等待期间继续泵遥测，避免巡检期间遥测超龄；绝不跨越比赛截止时间。
        end = time.monotonic() + config.SERVO_SETTLE_S
        if deadline is not None:
            end = min(end, deadline)
        while time.monotonic() < end:
            self.mcu.pump()
            time.sleep(max(0.0, min(0.02, end - time.monotonic())))
        if self._deadline_expired(deadline):
            raise McuError("比赛截止，云台未完成到位等待")
        runlog.emit("gimbal.applied", "PWM 命令应用确认结束，未验证机械角度",
                    sequence=sequence, applied=callable(confirm), action=action,
                    pan_us=pan_us, tilt_us=tilt_us,
                    ch1_pwm_us=tilt_us, ch2_pwm_us=pan_us)
        return sequence

    def center(self, deadline: float | None = None) -> int:
        """水平居中，用于左右扫描完成后的中间姿态。"""
        return self._pan(
            WireGimbalPose.LEVEL_CENTER,
            config.SERVO_PAN_CENTER_US,
            config.SERVO_TILT_CENTER_US,
            action="center",
            deadline=deadline,
        )

    def set_pwm(
        self,
        pan_us: int,
        tilt_us: int,
        *,
        deadline: float | None = None,
    ) -> int:
        """标定用原始 PWM；复用脉宽校验、命令确认和到位等待。

        LEVEL_CENTER 仅为现有协议的兼容标签，不表示任意 PWM 已机械水平。
        """
        return self._pan(
            WireGimbalPose.LEVEL_CENTER,
            pan_us,
            tilt_us,
            action="set_pwm",
            deadline=deadline,
        )

    def tilt_look(self, deadline: float | None = None) -> int:
        """兼容旧接口名，发送水平居中导航姿态。"""
        return self._pan(
            WireGimbalPose.LEVEL_CENTER,
            config.SERVO_PAN_CENTER_US,
            config.SERVO_TILT_CENTER_US,
            action="tilt_look",
            deadline=deadline,
            restore=True,
        )

    def pan_left(self, deadline: float | None = None) -> int:
        """在水平视角扫描最左侧涵洞内壁。"""
        return self._pan(
            WireGimbalPose.LEVEL_LEFT,
            config.SERVO_PAN_LEFT_US,
            config.SERVO_TILT_CENTER_US,
            action="pan_left",
            deadline=deadline,
        )

    def pan_right(self, deadline: float | None = None) -> int:
        """在水平视角扫描最右侧涵洞内壁。"""
        return self._pan(
            WireGimbalPose.LEVEL_RIGHT,
            config.SERVO_PAN_RIGHT_US,
            config.SERVO_TILT_CENTER_US,
            action="pan_right",
            deadline=deadline,
        )

    def tilt_down(self, deadline: float | None = None) -> int:
        """完整自检下打死，左右保持居中；沿用 LEVEL_CENTER 兼容标签。"""
        return self._pan(
            WireGimbalPose.LEVEL_CENTER,
            config.SERVO_PAN_CENTER_US,
            config.SERVO_TILT_MIN_US,
            action="tilt_down",
            deadline=deadline,
        )

    def tilt_up(self, deadline: float | None = None) -> int:
        """完整自检上打死，左右保持居中；沿用 LEVEL_CENTER 兼容标签。"""
        return self._pan(
            WireGimbalPose.LEVEL_CENTER,
            config.SERVO_PAN_CENTER_US,
            config.SERVO_TILT_MAX_US,
            action="tilt_up",
            deadline=deadline,
        )
