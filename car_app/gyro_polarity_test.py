#!/usr/bin/env python3
"""被动验证陀螺仪角速度正负号。

本工具只建立现有 protocol v2.3 HELLO 会话并接收遥测，不发送运动命令。
运行后手动向左、向右转动车体，观察每个新鲜 DIAGNOSTICS 样本的角速度
符号，以及相对于启动参考航向的 ``yaw`` 变化。

在 RDK 上运行::

    python3 -B gyro_polarity_test.py
    python3 -B gyro_polarity_test.py --port /dev/ttyUSB1

按 Ctrl+C 退出。脚本不会修改固件极性配置，也不会主动驱动车轮。
"""

from __future__ import annotations

import argparse
import math
import time

import config
import mcu
import runlog
from drivers.protocol_v2 import (
    DIAGNOSTICS_PAYLOAD_LENGTH,
    MessageType,
    ParserIssue,
    WireGyroStatus,
    decode_diagnostics,
    decode_hello_response,
)


# 下位机陀螺仪 Z 轴灵敏度：raw-bias / 131 = deg/s。
GYRO_LSB_PER_DEG_S = 131.0
GYRO_MAX_SOURCE_AGE_MS = 250.0
POLL_INTERVAL_S = 0.01

# 诊断工具只需要 protocol 2.3 的 STATE/DIAGNOSTICS。现有 RDK 实车可能是
# v9 或独立 v11；共享 McuLink 为正式 v9 任务保留严格单一 build 校验，
# 因此这里在脚本入口内显式允许这两个已知身份，不放宽到任意固件。
SUPPORTED_BUILD_IDS = frozenset((
    "car12-wheeltrace-v9",
    "car12-wheel-sync-v11",
))


class GyroPolarityLink(mcu.McuLink):
    """只为本诊断入口提供 v9/v11 build_id 白名单握手。"""

    @staticmethod
    def _decode_diagnostics(payload: bytes):
        # v11 adds TURN_SETTLE_ERROR=12 to the final stop-reason byte while the
        # remaining 60 diagnostic bytes keep the v2.3 layout.  The polarity
        # monitor does not use that reason, so validate the frozen payload with
        # the compatible NONE value instead of dropping the complete sample.
        if len(payload) == DIAGNOSTICS_PAYLOAD_LENGTH and payload[-1] == 12:
            payload = payload[:-1] + b"\x00"
        return decode_diagnostics(payload)

    def _handle(self, frame) -> None:
        if frame.message_type is not MessageType.DIAGNOSTICS:
            super()._handle(frame)
            return
        try:
            self._accept_diagnostics(self._decode_diagnostics(frame.payload))
        except Exception as exc:  # malformed diagnostics must remain non-fatal
            self._retain_parse_issues((ParserIssue("PAYLOAD_ERROR", str(exc)),))

    def _capture_frame(self, direction, frame, host_ms, **wire_context) -> None:
        if frame.message_type is not MessageType.DIAGNOSTICS:
            super()._capture_frame(direction, frame, host_ms, **wire_context)
            return
        decoded, error = None, None
        try:
            decoded = self._decode_diagnostics(frame.payload)
        except Exception as exc:
            error = str(exc)
        self._capture_call(
            "record_frame",
            direction,
            frame,
            host_ms,
            decoded=decoded,
            session_nonce=self.session_nonce,
            boot_id=self.boot_id,
            mcu_uptime_reference_ms=getattr(self.state, "uptime_ms", None),
            decode_allowed=True,
            decode_error=error,
            **wire_context,
        )

    def _complete_hello(self, payload: bytes) -> None:
        response = decode_hello_response(payload)
        if response.session_nonce != self._pending_hello_nonce:
            return
        if self._hello_response is not None:
            return
        required = int(config.MCU_REQUIRED_CAPABILITIES)
        if response.capabilities != required:
            raise mcu.McuError(
                f"固件能力不兼容 0x{response.capabilities:08X}，需要 0x{required:08X}"
            )
        if response.build_id not in SUPPORTED_BUILD_IDS:
            allowed = ", ".join(sorted(SUPPORTED_BUILD_IDS))
            raise mcu.McuError(
                f"固件标识不兼容 {response.build_id!r}，诊断工具只允许：{allowed}"
            )
        if response.boot_id == 0:
            raise mcu.McuError("固件 boot_id=0（身份初始化失败）")

        self.session_nonce = response.session_nonce
        self.boot_id = response.boot_id
        self.capabilities = response.capabilities
        self.build_id = response.build_id
        self.connected = True
        self._reset_link_state()
        self._session_sent_at = self._pending_hello_sent_at
        self.negotiated_required_capabilities = required
        self._hello_response = response
        self._capture_call(
            "update_metadata",
            build_id=self.build_id,
            boot_id=self.boot_id,
            session_nonce=self.session_nonce,
            negotiated_capabilities=required,
            supported_build_ids=sorted(SUPPORTED_BUILD_IDS),
        )
        self._capture_call(
            "event",
            "hello_accepted",
            session_nonce=self.session_nonce,
            boot_id=self.boot_id,
            build_id=self.build_id,
        )


def wrapped_delta_deg(current: float, reference: float) -> float:
    """Return the shortest signed ``current-reference`` angle in [-180, 180)."""
    delta = float(current) - float(reference)
    while delta >= 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return delta


def signed_rate_text(rate_deg_s: float) -> str:
    """Format a rate using the application's established sign convention."""
    rounded = round(float(rate_deg_s), 2)
    if rounded == 0:
        return "0.00°/s（显示精度内为零）"
    direction = "左转" if rounded > 0 else "右转"
    return f"{rounded:+.2f}°/s（{direction}）"


def diagnostic_source_age_ms(link: mcu.McuLink, diagnostics, now: float | None = None) -> float:
    """Estimate the age of the gyro sample represented by a DIAGNOSTICS frame.

    ``gyro_sample_age_ms`` is measured by the MCU when it creates the diagnostic
    frame.  Add the elapsed host time since that frame was generated so a frame
    that arrived late cannot be treated as fresh merely because its field age is
    below the limit.  McuLink's clock-synchronised sample time is used when
    available; the receive timestamp remains a safe fallback for test doubles.
    """
    now = time.monotonic() if now is None else now
    field_age = float(getattr(diagnostics, "gyro_sample_age_ms", math.inf))
    sample_time = getattr(link, "_sample_time", None)
    if callable(sample_time):
        try:
            generated_at = float(sample_time("diagnostics"))
        except (TypeError, ValueError, AttributeError):
            generated_at = float(getattr(link, "_last_diagnostics_recv", now))
    else:
        generated_at = float(getattr(link, "_last_diagnostics_recv", now))
    return max(0.0, field_age + (now - generated_at) * 1000.0)


def _yaw_text(link: mcu.McuLink, reference_yaw: float | None) -> tuple[str, float | None]:
    """Read current fresh yaw and format it against the fixed startup reference."""
    yaw = link.yaw_deg
    if yaw is None:
        return "yaw=等待新鲜 STATE", None
    if reference_yaw is None:
        return f"yaw={yaw:.1f}°（参考方向待建立）", None
    delta = wrapped_delta_deg(yaw, reference_yaw)
    return f"yaw={yaw:.1f}° Δyaw={delta:+.1f}°", delta


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="被动读取 STM32 陀螺仪，验证左转/右转对应的正负角速度。"
    )
    parser.add_argument(
        "--port",
        default=None,
        help=f"STM32 串口（默认使用 config.MCU_PORT={config.MCU_PORT}）",
    )
    return parser


def monitor(port: str | None = None) -> int:
    """Open the MCU link and print each new, valid gyro sample until interrupted."""
    link = GyroPolarityLink(port=port)
    reference_yaw: float | None = None
    seen_key: tuple[int, int, int] | None = None
    try:
        link.open()
        response = link.hello()
        print(
            f"已连接 STM32：build={response.build_id}；"
            f"session={link.session_nonce} boot={link.boot_id}"
        )
        print("请手动转动车体观察：正值按当前约定为左转，负值按当前约定为右转；Ctrl+C 退出。")

        while True:
            link.pump()
            now = time.monotonic()

            # 参考方向固定为会话启动后首个新鲜 STATE，不随手动转动更新。
            if reference_yaw is None:
                candidate = link.yaw_deg
                if candidate is not None:
                    reference_yaw = candidate
                    print(f"参考方向已建立：yaw={reference_yaw:.1f}°")

            diagnostics = link.diagnostics
            if diagnostics is None:
                time.sleep(POLL_INTERVAL_S)
                continue

            key = (
                int(getattr(diagnostics, "session_nonce", -1)),
                int(getattr(diagnostics, "boot_id", -1)),
                int(getattr(diagnostics, "uptime_ms", -1)),
            )
            if key == seen_key:
                time.sleep(POLL_INTERVAL_S)
                continue
            seen_key = key

            # McuLink 已经丢弃跨会话/倒序帧；这里再显式守一层，便于阅读和替身测试。
            if key[0] != link.session_nonce or key[1] != link.boot_id:
                continue

            raw_z = getattr(diagnostics, "gyro_raw_z", None)
            bias = getattr(diagnostics, "gyro_bias_lsb", None)
            status = getattr(diagnostics, "gyro_status", None)
            source_age_ms = diagnostic_source_age_ms(link, diagnostics, now)
            yaw_text, _ = _yaw_text(link, reference_yaw)
            status_name = getattr(status, "name", str(status))

            if status != WireGyroStatus.READY:
                print(
                    f"gyro_status={status_name} raw_z={raw_z} bias={bias} "
                    f"源龄={source_age_ms:.1f}ms；不判定"
                )
                time.sleep(POLL_INTERVAL_S)
                continue
            if source_age_ms > GYRO_MAX_SOURCE_AGE_MS:
                print(
                    f"gyro_status={status_name} raw_z={raw_z} bias={bias} "
                    f"源龄={source_age_ms:.1f}ms（>{GYRO_MAX_SOURCE_AGE_MS:.0f}ms）；不判定"
                )
                time.sleep(POLL_INTERVAL_S)
                continue

            try:
                rate_deg_s = (float(raw_z) - float(bias)) / GYRO_LSB_PER_DEG_S
            except (TypeError, ValueError):
                print(f"陀螺仪字段非法：raw_z={raw_z!r} bias={bias!r}；不判定")
                time.sleep(POLL_INTERVAL_S)
                continue

            print(
                f"gyro_status={status_name} raw_z={int(raw_z):+d} bias={int(bias):+d} "
                f"角速度={signed_rate_text(rate_deg_s)} "
                f"源龄={source_age_ms:.1f}ms；{yaw_text}"
            )
            time.sleep(POLL_INTERVAL_S)
    except KeyboardInterrupt:
        print("\n已停止陀螺仪监视。")
        return 0
    finally:
        # 被动工具只关闭串口和采集，不调用 shutdown()，因此不会额外发送 STOP。
        link.close()


def main() -> int:
    args = _build_parser().parse_args()
    return monitor(args.port)


if __name__ == "__main__":
    with runlog.session("gyro_polarity_test", periodic_status=False):
        raise SystemExit(main())
