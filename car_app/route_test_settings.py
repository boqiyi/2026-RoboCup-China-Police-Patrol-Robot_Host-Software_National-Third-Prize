"""路线实车测试和直路居中调参共用的人工设置；正式比赛入口不读取本文件。"""
from dataclasses import dataclass
import math

LEFT = "left"
RIGHT = "right"
STRAIGHT = "straight"

# 单位 mm/s；特殊路速度是上限，实际取 min(普通巡航, 特殊路)。
CRUISE_SPEED_MMPS = 150
SPECIAL_SPEED_MMPS = 100
TURN_SPEED_MMPS = 91
HOME_SPEED_MMPS = 61

# 原始左右激光差 <25 mm 使用小增益，>=25 mm 使用大增益。
SMALL_ERROR_GAIN_PER_S = 2.1
LARGE_ERROR_GAIN_PER_S = 3.7

# 从出发区遇到的第一个路口开始；最后一个 RIGHT 后按返家规则行驶160 mm。
ROUTE_ACTIONS = (
    RIGHT,     # 1：离开出发区右转
    LEFT,      # 2
    RIGHT,     # 3
    LEFT,      # 4
    STRAIGHT,  # 5
    STRAIGHT,  # 6
    LEFT,      # 7
    STRAIGHT,  # 8
    STRAIGHT,  # 9
    LEFT,      # 10
    LEFT,      # 11
    STRAIGHT,  # 12
    RIGHT,     # 13
    RIGHT,     # 14
    STRAIGHT,  # 15
    LEFT,      # 16
    LEFT,      # 17
    RIGHT,     # 18
    LEFT,      # 19
    RIGHT,     # 20：出发区右转，随后返家直行160 mm
)


@dataclass(frozen=True)
class Settings:
    cruise_speed_mmps: int = 150
    special_speed_mmps: int = 100
    turn_speed_mmps: int = 91
    home_speed_mmps: int = 61
    small_error_gain_per_s: float = 2.1
    large_error_gain_per_s: float = 3.7
    actions: tuple[str, ...] = ROUTE_ACTIONS

    def __post_init__(self):
        for name in ("cruise_speed_mmps", "special_speed_mmps", "turn_speed_mmps", "home_speed_mmps"):
            value = getattr(self, name)
            if type(value) is not int or not 61 <= value <= 310:
                raise ValueError(f"{name} 必须是61～310 mm/s的整数")
        for name in ("small_error_gain_per_s", "large_error_gain_per_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} 必须是有限正数")
        if not isinstance(self.actions, tuple) or not self.actions:
            raise ValueError("ROUTE_ACTIONS 必须是非空tuple，动作仅允许LEFT、RIGHT、STRAIGHT")
        for number, action in enumerate(self.actions, 1):
            if action not in (LEFT, RIGHT, STRAIGHT):
                raise ValueError(f"第{number}个动作非法：{action!r}；仅允许LEFT、RIGHT、STRAIGHT")


def load_settings() -> Settings:
    """每次启动读取当前常量，随后冻结，运行中不改变已授权速度或路线。"""
    return Settings(
        cruise_speed_mmps=CRUISE_SPEED_MMPS,
        special_speed_mmps=SPECIAL_SPEED_MMPS,
        turn_speed_mmps=TURN_SPEED_MMPS,
        home_speed_mmps=HOME_SPEED_MMPS,
        small_error_gain_per_s=SMALL_ERROR_GAIN_PER_S,
        large_error_gain_per_s=LARGE_ERROR_GAIN_PER_S,
        actions=ROUTE_ACTIONS,
    )
