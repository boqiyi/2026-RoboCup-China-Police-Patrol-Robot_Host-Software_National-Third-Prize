"""car_app 集中配置。

现场标定 / 换硬件时**只改这个文件**，不改其它模块逻辑。
所有待实车标定的值都标了 ``TODO(标定)``，先放了推荐数值。
"""

from __future__ import annotations

from pathlib import Path

# ---------------------------------------------------------------------------
# 路径
# ---------------------------------------------------------------------------
APP_ROOT = Path(__file__).resolve().parent

# 资产统一放在 car_app/assets/ 下，app 自包含、上传 RDK 时一并传。
ASSETS_ROOT = APP_ROOT / "assets"

# 2026 场地拓扑（多模态模型从规则图提取，已人工校对）
MAP_JSON = ASSETS_ROOT / "2026场地拓扑.json"

# 线索映射 + 音频（复用 robot/ 里已冻结好的资产）
CLUES_JSON = ASSETS_ROOT / "direct20-map.v1.json"
AUDIO_DIR = ASSETS_ROOT / "audio"

# 地平线 BPU 模型（20 类：1~10 嫌疑人、11~20 刀具）
MODEL_DIR = ASSETS_ROOT / "model"
MODEL_ONNX = "last_1_6.onnx"
MODEL_BIN = "last_1_6_bayese_640x640_nv12.bin"

# 日志目录
LOG_DIR = APP_ROOT / "logs"

# ---------------------------------------------------------------------------
# STM32 串口（RDK 检测到 2 个 CH340：/dev/ttyUSB0、/dev/ttyUSB1）
# ---------------------------------------------------------------------------
# 历史 2026-08-17 自检日志中的 ttyUSB0 曾完成 STM32 HELLO；
# 当前配置为 ttyUSB1=STM32、ttyUSB0=RFID，需现场核验实际身份。
# 换线/重启后应以 /dev/serial/by-id 下的实际枚举结果为准。
# 建议现场用 `ls /dev/serial/by-id` 固定设备名后替换，避免换口错乱。
MCU_PORT = "/dev/ttyUSB1"        # STM32 USART2（经 CH340 USB-TTL）
MCU_BAUD = 115200
MCU_TIMEOUT_S = 1.0
MCU_REQUIRED_CAPABILITIES = 0xEDB
MCU_EXPECTED_BUILD_ID = "car12-wheeltrace-v9"
# 最后一次停车后继续接收诊断：覆盖 1 秒尾流、控制采样与串口排队。
WHEEL_TRACE_TAIL_S = 1.1
# 写超时：远小于 500ms 心跳预算，避免 CH340/TTY 堵塞卡死唯一线程错过租约
MCU_WRITE_TIMEOUT_S = 0.05
# 运动租约与心跳：STM32 运动租约 500ms，心跳必须短于它
LEASE_MS = 500
HEARTBEAT_INTERVAL_S = 0.10
# 遥测健康：单族遥测超龄阈值 + 连续异常容忍次数（容忍 USB 串口偶发抖动）
TELEMETRY_STALE_MS = 500
TELEMETRY_FAULT_TOLERANCE = 5
# 单个激光样本的最大允许年龄：这是传感器样本年龄，不是串口帧接收年龄。
# 超过阈值后上位机统一按 STALE/fault 处理，不允许继续使用旧墙面或旧顶棚数据。
DIRECTIONAL_LASER_MAX_AGE_MS = 250
CULVERT_UPPER_LASER_MAX_AGE_MS = 250
# 命令 ACK / 实际状态回读的有界确认时间：低于 500ms 运动租约，避免等待确认时错过续租。
MCU_COMMAND_CONFIRM_TIMEOUT_S = 0.20
# 自检等待 STM32 完成 IMU 零偏标定与首轮传感器初始化；不会放宽任何运动安全门禁。
MCU_SELFCHECK_READY_TIMEOUT_S = 5.0
# 旧固件保留的 bit17 会随冻结协议上报；仅将该位作为兼容信息忽略。
# 其他已知位和未来未知位一律继续阻止运动。
MCU_IGNORED_RUNTIME_FAULT_BITS = 1 << 17

# ---------------------------------------------------------------------------
# 激光阈值（关键值已由用户确认；其余待实车标定）
# ---------------------------------------------------------------------------
# 冻结串口协议的槽 0/3 保留作兼容空槽，不参与上位机业务。
# 底盘侧激光的事件由状态变化判定；数值只用于居中与几何一致性检查。
# 上层测距（CULVERT_UPPER2）：用于涵洞/隧道顶检测。
WALL_NEAR_MM = 40           # 旧标定参考值，不参与事件判定
WALL_FAR_MM = 50            # 旧标定参考值，不参与事件判定
SIDE_FAULT_TOLERANCE = 3    # 侧向激光连续故障帧数才 fail-closed（容忍单帧毛刺）
# 障碍后安全回退：必须同时满足侧向路口、编码器里程和航向掉头证据才确认回到节点。
# TODO(标定)：按真实编码器误差与掉头漂移微调；任何不足均禁止节点图重规划。
ROLLBACK_ODOM_TOLERANCE_MM = 150
ROLLBACK_MIN_ODOM_DISTANCE_MM = 10
ROLLBACK_YAW_TOLERANCE_DEG = 20.0
# "突变"判定：在最近 N 个采样窗口内，从"有墙"翻到"没墙"算一次跳变
JUMP_WINDOW_SAMPLES = 5
# 只有离开起点路口一段可信轮程后，侧壁 near→open 才能确认下一节点。
JUNCTION_MIN_TRAVEL_RATIO = 0.10
JUNCTION_MIN_TRAVEL_MM = 20

# ---------------------------------------------------------------------------
# 居中行驶 PID（待实车测试后调整；若未另行告知则保持这些默认值）
# ---------------------------------------------------------------------------
FW_MAX_LINEAR_SPEED_MM_S = 310  # 固件单轮目标速度上限
FW_MIN_NONZERO_WHEEL_SPEED_MM_S = 31
FW_WHEEL_BASE_MM = 125
CENTER_LINEAR_MMPS = 250        # 直行线速度（mm/s，即 0.25 m/s；固件上限）
CENTER_KP = 0.8                 # 左右距离误差 → 角速度 的比例系数
CENTER_KD = 0.3                 # 阻尼项
CENTER_MAX_ANGULAR_MRADPS = 400  # 居中修正的角速度上限（mrad/s）
# 左右距离都失效（都远/无目标）时，允许继续直行的时间上限（秒），超过则停车
CENTER_BLIND_TIMEOUT_S = 1.5
# 陀螺仪航向校正增益（度误差 → mrad/s；TODO(标定) 按 5s 收敛调）
CENTER_K_HEADING = 30.0
# 居中死区：左右距离差和航向误差分别使用，避免两类正常噪声互相放大。
CENTER_DEADBAND_MM = 3.0
CENTER_DEADBAND_DEG = 2.0
CENTER_SIDE_BIAS_MM = 0.0       # 底盘安装造成的左右侧距固定偏置（待实车标定）
# 单墙跟随目标距离（待实车确认）
CENTER_SINGLE_WALL_TARGET_MM = 30
CENTER_RECENTER_TIMEOUT_S = 5.0
CENTER_RECENTER_STABLE_SAMPLES = 3
CENTER_RECENTER_LIMIT_MMPS = 60
SIDE_TOTAL_TOLERANCE_MM = 10.0
SIDE_REFERENCE_SAMPLES = 3
SIDE_MAX_YAW_ERROR_DEG = 15.0

# ---------------------------------------------------------------------------
# 转弯（原地转，v=0）
# ---------------------------------------------------------------------------
TURN_ANGULAR_MRADPS = 800       # 原地转角速度（mrad/s；正=左转）
TURN_ANGLE_DEG = 90             # 单次转弯角度
TURN_YAW_TOLERANCE_DEG = 5.0    # yaw 到位容差（度）
TURN_TIMEOUT_S = 13.0           # 固件固定转向的主机等待上限
TURN_ACTION_SPEED_MMPS = 80
# 原地转前的上位机停稳门禁：连续编码器样本位移不大于阈值才允许转向。
# TODO(标定)：实车确认惯性消退时间和编码器噪声后再收紧；不满足时保持停车。
STOP_STABLE_TIMEOUT_S = 0.50
STOP_STABLE_SAMPLES = 3
STOP_STABLE_DELTA_MM = 2.0

# ---------------------------------------------------------------------------
# 预见式刹车 / 运动平滑（TODO(标定)：现场实测后按需微调）
# ---------------------------------------------------------------------------
# 平滑层：渐变普通运动目标；颜色障碍候选、路口候选和通用故障直接 STOP，绕过渐变。
MOTION_SEND_INTERVAL_S = 0.10
RAMP_V_STEP_MMPS = 100          # 每次实际发包最多增加 100 mm/s
RAMP_V_DOWN_STEP_MMPS = 25      # 普通非零目标每次发包最多降低 25 mm/s
RAMP_W_STEP_MRADPS = 400        # 每次实际发包最多变化 400 mrad/s
# 预测层：离路口剩余距离 < BRAKE_START_MM 时开始降速，到路口蠕行至 BRAKE_MIN_MMPS。
BRAKE_ENABLED = True
BRAKE_START_MM = 250            # 离路口剩这么多距离开始减速（含地图 ±5% 与死推漂移余量）
BRAKE_MIN_MMPS = 60             # 蠕行速度下限；首个侧激光候选即 STOP
BRAKE_ODOM_SCALE = 1.0          # 编码器里程→真实 mm 标定系数（现场测一段已知路校准）
BRAKE_DR_DT_MAX_S = 0.1         # 死推时间步上限，防心跳异常时积分爆炸
JUNCTION_SOFTSTOP_MAX_TICKS = 15  # 兼容旧调用；实际停车直接 STOP

# ---------------------------------------------------------------------------
# 舵机 PWM（CH1 上下、CH2 左右；两路脉宽范围均为 500..2500 μs）
# ---------------------------------------------------------------------------
SERVO_PWM_MIN_US = 500
SERVO_PWM_MAX_US = 2500
SERVO_PAN_MIN_US = 500          # 左右轴 PWM 数值下限，不代表向左
SERVO_PAN_CENTER_US = 1500      # 左右居中 PWM
SERVO_PAN_MAX_US = 2500         # 左右轴 PWM 数值上限，不代表向右
SERVO_PAN_LEFT_US = 2500        # 朝小车自身左侧扫描（按实车观察修正）
SERVO_PAN_RIGHT_US = 500        # 朝小车自身右侧扫描（按实车观察修正）
SERVO_TILT_MIN_US = 500         # 下打死 PWM，仅完整自检使用
SERVO_TILT_CENTER_US = 800      # gimbal-test 实车确认的上下水平 PWM
SERVO_TILT_MAX_US = 2500        # 上打死 PWM，仅完整自检使用
SERVO_SETTLE_S = 0.6            # 舵机到位等待（普通舵机转到位的估算时间）

# ---------------------------------------------------------------------------
# 隧道补光（LED 灯带经 MOS 管，STM32 PA10/TIM1 CH3 PWM；仅隧道行驶时点亮）
# ---------------------------------------------------------------------------
LIGHT_DUTY_PERMILLE = 500       # 隧道补光固定亮度（0..1000，500=50%；用户确认现有方案可行，隧道内仍靠激光寻路、补光为辅助）

# ---------------------------------------------------------------------------
# 颜色障碍物：原始 / 有界光照校正 HSV 与牌面、木栅栏结构；不识字、不测距。
# ---------------------------------------------------------------------------
CAMERA_HEALTH_CHECK_INTERVAL_S = 0.10
OBSTACLE_CHECK_INTERVAL_S = 0.10
OBSTACLE_HIT_STREAK = 3
OBSTACLE_CLEAR_STREAK = 2
OBSTACLE_VERIFY_TIMEOUT_S = 1.0
JUNCTION_VERIFY_TIMEOUT_S = 2.0
OBSTACLE_MAX_BLOCKED = 12
OBSTACLE_ROI_FRACTION = 0.50    # 兼容旧读取方；实际由下列 x/y 边界定义。
OBSTACLE_ROI_X_MIN_FRACTION = 0.20
OBSTACLE_ROI_X_MAX_FRACTION = 0.80
OBSTACLE_ROI_Y_MIN_FRACTION = 0.15
OBSTACLE_ROI_Y_MAX_FRACTION = 1.00
OBSTACLE_MIN_PIXELS = 500       # 兼容旧读取方。
OBSTACLE_MIN_COMPONENT_AREA_PX = 500
OBSTACLE_MIN_COMPONENT_AREA_RATIO = 0.004
OBSTACLE_MIN_WIDTH_PX = 20
OBSTACLE_MIN_HEIGHT_PX = 20
OBSTACLE_MORPH_KERNEL_PX = 5
OBSTACLE_MORPH_ITERATIONS = 1
# 大小 / 形态学门槛均以最长边 640 为参考，低分辨率按比例换算，不上采样。
OBSTACLE_WORK_MAX_EDGE_PX = 640
OBSTACLE_REFERENCE_EDGE_PX = 640
OBSTACLE_MIN_COLOR_FILL_RATIO = 0.12
OBSTACLE_MIN_SOLIDITY = 0.25
OBSTACLE_CONTINUITY_MIN_IOU = 0.20
OBSTACLE_CONTINUITY_MAX_AREA_RATIO = 4.0
OBSTACLE_CONTINUITY_MIN_CENTER_SHIFT_PX = 20.0
OBSTACLE_CONTINUITY_MAX_CENTER_SHIFT_RATIO = 0.60
# OpenCV HSV: H=0..179, S/V=0..255. 保留原始分支初值，校正分支拓宽亮暗范围。
OBSTACLE_BROWN_RANGE = ((5, 60, 35), (25, 255, 200))
OBSTACLE_BLUE_RANGE = ((90, 60, 40), (135, 255, 255))
OBSTACLE_YELLOW_RANGE = ((18, 70, 70), (40, 255, 255))
OBSTACLE_RED_LOW_RANGE = ((0, 70, 50), (10, 255, 255))
OBSTACLE_RED_HIGH_RANGE = ((170, 70, 50), (179, 255, 255))
# 参考照片橙牌 H≈14、蓝牌 H≈109、木条 H≈11～20；不以手机图标定发现距离。
OBSTACLE_BLUE_ADAPTIVE_RANGE = ((88, 60, 22), (138, 255, 255))
OBSTACLE_YELLOW_ADAPTIVE_RANGE = ((10, 65, 55), (40, 255, 255))
OBSTACLE_BROWN_ADAPTIVE_RANGE = ((4, 65, 18), (28, 255, 210))
OBSTACLE_RED_LOW_ADAPTIVE_RANGE = ((0, 65, 28), (11, 255, 255))
OBSTACLE_RED_HIGH_ADAPTIVE_RANGE = ((168, 65, 28), (179, 255, 255))
OBSTACLE_ADAPTIVE_ENABLED = True
# 低饱和且未过曝的中性背景才参与白平衡；不足时跳过，不要求白墙。
# 校正确认中性时剔除原始偏色假证据；低饱和且 V>245 的剪切像素不作颜色证据。
OBSTACLE_WB_MAX_SATURATION = 90
OBSTACLE_WB_MIN_VALUE = 20
OBSTACLE_WB_MAX_VALUE = 245
OBSTACLE_WB_MIN_PIXELS = 128
OBSTACLE_WB_MIN_FRACTION = 0.03
# 参考必须分布在多个网格，孤立低饱和蓝牌不能被当作整幅照明的中性背景。
OBSTACLE_WB_GRID_SIZE = 4
OBSTACLE_WB_MIN_GRID_CELLS = 6
OBSTACLE_WB_MIN_CELL_FRACTION = 0.05
OBSTACLE_WB_GAIN_MIN = 0.80
OBSTACLE_WB_GAIN_MAX = 1.25
# 全 ROI 亮度中位数估计有限增益和 gamma；只提亮，避免亮背景使暗蓝牌更暗。
OBSTACLE_BRIGHTNESS_TARGET = 160
OBSTACLE_BRIGHTNESS_GAIN_MIN = 1.00
OBSTACLE_BRIGHTNESS_GAIN_MAX = 1.35
OBSTACLE_GAMMA_MIN = 0.65
OBSTACLE_GAMMA_MAX = 1.00
# 仅合并有足够高度 / 面积的相容木条；零散小斑点不参与拼接。
OBSTACLE_FENCE_FRAGMENT_MIN_AREA_RATIO = 0.15
OBSTACLE_FENCE_FRAGMENT_MIN_AREA_PX = 60
OBSTACLE_FENCE_PLANK_MIN_ASPECT = 1.8
OBSTACLE_FENCE_BEAM_MIN_ASPECT = 2.5
OBSTACLE_FENCE_MAX_SIZE_RATIO = 2.5
OBSTACLE_FENCE_MIN_VERTICAL_OVERLAP = 0.55
OBSTACLE_FENCE_MAX_GAP_WIDTH_RATIO = 2.5
OBSTACLE_BLUE_RED_MIN_OVERLAP_RATIO = 0.65
# ---------------------------------------------------------------------------
# YOLO 推理（地平线 BPU）
# ---------------------------------------------------------------------------
YOLO_CONF_THRESHOLD = 0.60      # 置信度阈值；等于 0.60 也允许播报
YOLO_IOU_THRESHOLD = 0.5
YOLO_INPUT_W = 640
YOLO_INPUT_H = 640
YOLO_CLASS_COUNT = 20           # 1~10 嫌疑人、11~20 刀具
# 涵洞扫描时，同类别连续命中多少次才确认（减少单帧误报）
DETECT_HIT_STREAK = 2
# 每侧至少有多少帧完成推理后，才能把该侧记为已完整扫描；不得低于 2。
CULVERT_MIN_VALID_INFERENCE_FRAMES = 2
# 同侧多帧确认必须留出真实采样间隔；与换向后的缓存丢帧一起避免复用同一图像误播。
CULVERT_SCAN_FRAME_COUNT = 5
CULVERT_FRAME_MIN_INTERVAL_S = 0.08

# ---------------------------------------------------------------------------
# 摄像头
# ---------------------------------------------------------------------------
CAMERA_INDEX = 0                # USB UVC 摄像头索引（openCV VideoCapture 用）
CAMERA_FRAME_MAX_AGE_S = 0.5     # 安全关键调用方允许使用的最大帧龄

# ---------------------------------------------------------------------------
# RFID（U13T，优灵科技 0x7F 私有协议，非 Modbus；经 CH340）
# ---------------------------------------------------------------------------
RFID_PORT = "/dev/ttyUSB0"
RFID_BAUD = 115200
# 后台线程有界读写；运动主循环只消费事件，不等待 RFID 串口。
RFID_TIMEOUT_S = 0.1
RFID_WRITE_TIMEOUT_S = 0.05
RFID_POLL_INTERVAL_S = 0.3
# 连续多少次无卡读才判定「卡离开」（去抖，避免信号边缘瞬时 miss 导致同卡重复播报）
RFID_NO_CARD_DEBOUNCE = 3
# 巡逻编号位于 0 扇区 1 块；默认密钥卡使用读卡器现有认证配置，不自动装载密钥。
RFID_PATROL_BLOCK = 1
# 完整 16 字节块内的偏移；第 0 字节必须是数值 0x01～0x0C，UID 不编码巡逻号。
RFID_PATROL_BYTE_INDEX = 0

# ---------------------------------------------------------------------------
# 音频输出（USB 喇叭）
# ---------------------------------------------------------------------------
AUDIO_MODE = "aplay"
AUDIO_DEVICE = "plughw:0,0"      # card 0 = USB 喇叭（CD002-AUDIO）

# ---------------------------------------------------------------------------
# 比赛
# ---------------------------------------------------------------------------
ROUND_TIME_LIMIT_S = 600

# Runtime deadlines are conservative desktop defaults, pending vehicle calibration.
RETURN_ESTIMATED_SPEED_MMPS = 150.0
RETURN_TURN_90_S = 14.0
RETURN_UTURN_S = 14.0
RETURN_EDGE_OVERHEAD_S = 1.0
RETURN_DARK_OVERHEAD_S = 1.0
RETURN_FINAL_SETTLE_S = 2.0
RETURN_MARGIN_S = 15.0
RETURN_TIME_FACTOR = 1.25
RETURN_BUDGET_CALIBRATED = False
ROAD_NO_PROGRESS_TIMEOUT_S = 3.0
ROAD_PROGRESS_MIN_MM = 5.0
ROAD_TIMEOUT_FACTOR = 2.0
ROAD_TIMEOUT_EXTRA_S = 3.0
ROAD_OVERRUN_RATIO = 1.5
ROAD_OVERRUN_MIN_MM = 100.0
CULVERT_ATTEMPT_TIMEOUT_S = 12.0
CULVERT_MAX_ATTEMPTS = 2
EXPECTED_CULVERT_COUNT = 8
SELFCHECK_TIME_LIMIT_S = 300.0
CULVERT_INSPECTION_TIMEOUT_S = 12.0
CAMERA_OPEN_TIMEOUT_S = 3.0
CAMERA_READ_WAIT_S = 0.02
IO_CLOSE_TIMEOUT_S = 0.2
DETECTOR_LOAD_TIMEOUT_S = 15.0
DETECTOR_INFER_TIMEOUT_S = 0.15
AUDIO_TIMEOUT_MARGIN_S = 2.0
AUDIO_MAX_CLIP_S = 30.0
AUDIO_UNKNOWN_DURATION_S = 10.0
AUDIO_QUEUE_LIMIT = 128
LOG_STATUS_INTERVAL_S = 1.0
LOG_TERMINAL_QUEUE_LIMIT = 4096
LOG_FILE_QUEUE_LIMIT = 16384

