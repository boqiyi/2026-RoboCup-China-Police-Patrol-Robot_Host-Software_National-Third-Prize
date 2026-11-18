# 2026 RoboCup 中国赛巡逻机器人上位机

*由AI根据作者意思写作并且作者没有检查。*

这是 2026 RoboCup 中国赛巡逻机器人的 Python 上位机，运行在地平线 RDK 上，通过串口与 STM32 底盘通信。

- 用侧向激光、陀螺仪和轮程估计道路方向与车身位置，执行沿路居中、路口转弯、巡逻和返航。
- 按场地图规划巡逻路线，发现障碍后更新可用道路。应用保留色块和 YOLO 障碍检测入口，也提供涵洞场景下的人脸与刀具识别。
- 读取 RFID 巡逻卡并播放对应语音。地图、视觉模型、语音和字体随应用提供。

## 在 RDK 上运行

需要兼容现有 BPU 模型的 RDK Linux 镜像和 Python 3，板卡环境应提供 `hobot_dnn`、`hobot_vio` 和 `Hobot.GPIO`。正式启动入口对 STM32 的握手要求为协议 2.3、固件标识 `car12-single-wheel-v12`、能力掩码 `0xEDB`。

```bash
git clone https://github.com/boqiyi/26robotcup-.git
cd 26robotcup-/car_app
# Python 依赖缺失时安装
python3 -m pip install -r requirements-car.txt
```

上车前核对 [config.py](car_app/config.py) 中的设备配置。默认 MCU 串口为 `/dev/ttyUSB1`，RFID 为 `/dev/ttyUSB0`。以 RDK 上实际设备和权限为准，并检查摄像头、云台和 `aplay` 所用的声卡。

**以下命令会启动比赛流程，可能驱动车轮和云台。** 先按 [实车验收清单](car_app/VEHICLE_ACCEPTANCE.md) 确认接线、车轮方向、停车和设备状态，再在可控场地执行：

```bash
python3 -u main.py start
```

分阶段实车入口和配置细节见 [应用说明](car_app/README.md)。其中部分历史记录引用本地桌面回归材料。

## 许可

原创上位机代码采用 [MIT](LICENSE)。随附的 Noto Sans SC 字体采用 [SIL Open Font License 1.1](car_app/assets/fonts/OFL-NotoSansSC.txt)。
