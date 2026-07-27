# OpenCV 香橙派循线

这是一个面向香橙派 Pi 4（Linux/aarch64）的 C++17 + OpenCV 循线项目。视觉流程参考
`Camera-recognition-for-smart-cars`：截取前方 ROI，将图像分成远、中、近三区分别做
Otsu 二值化，自车身附近向远处逐行搜索赛道左右边界，再由中线偏差输出转向量。
同时使用矩形探测框统计上、下、左、右出口，可识别左右角点、左右支路、T 字和十字
路口。

项目不绑定某一款电机驱动板。默认只显示识别结果和控制量；传入 `--serial` 后可把控制
指令发给下位机。这样不会在摄像头或参数尚未标定时意外驱动车辆。

## 目录

```text
opencv_xunji/
├── config/default.yaml       # 摄像头、视觉、PID 参数
├── include/                  # 视觉、控制器、串口接口
├── src/                      # C++ 实现和主程序
├── tests/                    # 合成赛道单元测试
└── CMakeLists.txt
```

## 在香橙派 Pi 4 上安装

以下命令适用于官方 Ubuntu/Debian 系统：

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev v4l-utils

cd /xunji/opencv_xunji
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

如果项目实际放在用户目录，请把 `/xunji/opencv_xunji` 换成真实路径。程序不含
x86 专用代码，在香橙派上原生构建后会生成 aarch64 可执行文件。

## 摄像头检查与运行

先确认 USB/CSI 摄像头对应的 V4L2 设备：

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --all
```

有桌面环境时先只看识别结果：

```bash
cd /xunji/opencv_xunji
./build/opencv_xunji --config config/default.yaml --camera 0
```

按 `q` 或 `Esc` 退出。SSH 或开机自启等无显示环境使用：

```bash
./build/opencv_xunji --config config/default.yaml --camera 0 --headless
```

也可以用录制的视频离线调参：

```bash
./build/opencv_xunji --config config/default.yaml --video track.mp4
```

若提示无权访问摄像头，将运行用户加入 `video` 组，重新登录后再试：

```bash
sudo usermod -aG video "$USER"
```

## 图像与控制参数

首先修改 `config/default.yaml`：

- 白色/浅色赛道、深色地面使用 `dark_line: 0`；黑线、浅色地面使用 `dark_line: 1`。
- `roi_top_ratio` 越大，忽略的远处区域越多。摄像头应向下固定，使 ROI 底部能看到车前赛道。
- `min_track_width` 用于排除细小亮斑，`max_track_width` 用于排除整行过曝。
- `search_radius` 决定相邻行允许的最大横向变化，急弯丢线时适当增大。
- `center_smoothing` 越小越稳，越大响应越快。
- 紫色矩形是路口探测框；`probe_inset_ratio` 控制它距离 ROI 边缘的位置。
- 绿色折线是完整中心轨迹；检测到 90° 转角时，橙色圆点和 `corner(x,y)` 标出角点。
- `probe_min_run` 是认定出口所需的最短连续像素，噪声误报时可适当增大。
- `feature_confirm_frames` 是路口确认帧数，`feature_clear_frames` 是离开路口解锁帧数。
- `route` 是节点动作序列，例如 `RIGHT,STRAIGHT,LEFT,STRAIGHT`，走完后循环。
- `base_speed`、`min_speed` 的单位是 m/s；先以较低速度测试，再调整 `kp`、`kd`。
- `max_omega_rad_s` 是普通循线最大角速度，`turn_omega_rad_s` 和
  `turn_duration_seconds` 控制直角转向的角速度和最短执行时间，必须结合实车调节。
- `max_accel_m_s2`、`max_decel_m_s2` 和 `max_omega_accel_rad_s2` 限制
  `v、w` 的突变；确定丢线或故障锁停不经过斜坡，直接输出零。
- 相邻视觉控制帧间隔超过 `control_timeout_seconds` 会锁停，防止摄像头或处理链
  卡顿后又拿旧状态重新启动车辆。
- 节点动作超过 `turn_timeout_seconds` / `straight_timeout_seconds` 仍未重新找到
  赛道时会锁停，不会一直盲走。重新启动程序前应先检查识别和参数。
- 动作达到最短时间后，还要连续 `maneuver_reacquire_frames` 帧满足“找到线且不再是
  路口”，且置信度、转向量满足 `reacquire_min_confidence`、
  `reacquire_max_steering`，才会回到普通循线。发现新赛道后会先交回视觉闭环校正，
  不再持续盲转。

控制符号约定：

- `error` 和 `steering` 范围均为 `-1.0 ~ +1.0`。
- 负数表示赛道在左侧/向左转，正数表示赛道在右侧/向右转。
- `speed` 单位为 m/s，弯道会自动降速。
- 连续丢线超过 `lost_stop_seconds` 后速度降为 0。

## 连接下位机

先根据香橙派所用系统的设备树配置启用 UART，并用 `ls -l /dev/ttyS* /dev/ttyUSB*`
确认实际设备名。不同系统镜像的 UART 编号可能不同，不要直接照抄示例设备名。

```bash
sudo usermod -aG dialout "$USER"
./build/opencv_xunji --config config/default.yaml --camera 0 \
  --serial /dev/ttyUSB0 --headless
```

通信严格使用 H7 UART7 底盘速度协议：`1,000,000 baud、8N1、无流控、3.3V TTL`。
视觉端 TX 连接 H7 UART7_RX，并且双方 GND 共地。不要把 5V 串口电平直接接入 H7。

每帧固定 8 字节，不发送 ASCII 文本：

| 字节 | 内容 |
|---:|---|
| 0 | `0xA5` |
| 1 | `0x5A` |
| 2～3 | `int16_t velocity_x_mm_s`，小端 |
| 4～5 | `int16_t omega_mrad_s`，小端 |
| 6～7 | 字节 0～5 的 CRC16/MODBUS，小端 |

符号规定：`Vx > 0` 前进；`Omega > 0` 左转；`Omega < 0` 右转；两者为 0 停止。
串口层只负责可靠传输最终 `v、w`，不直接承担决策。正常循线时，运动监督器把 PD
输出转换为 `v、w`；识别角点或路口后，根据 `route` 选择动作并进入节点状态机，
不再发送 `$NODE`。

默认转换如下：

```text
Vx = speed(m/s) × 1000
Omega = -steering × max_omega_rad_s × 1000
```

负号用于转换方向约定：视觉内部左转为负，而 H7 协议左转为正。左右转动作至少执行
`turn_duration_seconds`，直行节点至少执行 `straight_node_seconds`。达到最短时间后
不会立刻退出动作，而是等待视觉稳定重捕获普通赛道；未在各自超时时间内重捕获则
进入不可自动恢复的故障锁停。

程序随摄像头帧率发送，默认 30 FPS 即约 33 ms 一帧，满足协议建议的 20～50 ms。
串口后台线程只保留最新速度帧，避免积压过期控制量。串口刚打开以及程序正常退出、
异常退出时都会发送协议停车帧：

```text
A5 5A 00 00 00 00 40 E3
```

串口后台写失败会使主循环立即报错退出，H7 连续 200 ms 未收到 CRC 正确的帧会自动
停车。代码同时按协议限制
`|Vx| <= 1000 mm/s`、`|Omega| <= 6000 mrad/s`。

## 上车前检查

1. 先抬起驱动轮或断开电机电源，只检查画面中的绿点是否沿赛道中心。
2. 向左移动赛道，终端中的 `error` 应变负；向右移动应变正。
3. 确认下位机的左右方向与上述符号一致，方向相反时在下位机侧取负。
4. 低速落地测试，先调 `kp`，出现快速左右摆动后降低 `kp` 或增加少量 `kd`。
5. 加入物理急停、下位机通信超时停车和独立电机限幅后再提高速度。

## 命令行选项

```text
--config PATH      YAML 配置文件
--camera N         V4L2 摄像头序号
--video PATH       视频文件
--serial DEVICE    串口设备
--headless         不创建窗口
--max-frames N     处理指定帧数后退出
--help             帮助
```
