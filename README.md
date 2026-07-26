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
- 先以较低的 `base_speed` 测试，再依次调整 `kp`、`kd`；通常保持 `ki: 0.0`。

控制符号约定：

- `error` 和 `steering` 范围均为 `-1.0 ~ +1.0`。
- 负数表示赛道在左侧/向左转，正数表示赛道在右侧/向右转。
- `speed` 范围为 `0.0 ~ 1.0`，弯道会自动降速。
- 连续丢线超过 `lost_stop_seconds` 后速度降为 0。

## 连接下位机

先根据香橙派所用系统的设备树配置启用 UART，并用 `ls -l /dev/ttyS* /dev/ttyUSB*`
确认实际设备名。不同系统镜像的 UART 编号可能不同，不要直接照抄示例设备名。

```bash
sudo usermod -aG dialout "$USER"
./build/opencv_xunji --config config/default.yaml --camera 0 \
  --serial /dev/ttyUSB0 --headless
```

串口为 8N1，默认 115200 baud，每帧发送一行 ASCII：

```text
$CTRL,<steering>,<speed>,<confidence>
```

例如：

```text
$CTRL,-0.235,0.280,0.910
```

确认新的角点或路口时额外发送一次：

```text
$NODE,<feature>,<action>,<exits_mask>,<confidence>
```

例如：

```text
$NODE,CROSSROAD,RIGHT,15,0.960
```

`feature` 可为 `CORNER_LEFT`、`CORNER_RIGHT`、`BRANCH_LEFT`、
`BRANCH_RIGHT`、`T_JUNCTION` 或 `CROSSROAD`。`action` 可为 `LEFT`、
`RIGHT`、`STRAIGHT`，无可用出口时为 `STOP`。

普通角点动作由角点方向决定，不消耗 `route`；其余节点从 `route` 依次取动作。如果规划
动作在当前路口不存在，程序按直行、左、右的顺序选择一个实际存在的出口。

`exits_mask` 的位定义为 `bit0=下方来路`、`bit1=上方直行`、`bit2=左侧出口`、
`bit3=右侧出口`。常见值为：左角 `5`、右角 `9`、左支路 `7`、右支路 `11`、
T 字 `13`、十字 `15`。

下位机应将 `steering` 映射到舵机或差速转向，将 `speed` 映射到电机 PWM。收到
`$NODE` 后进入自己的转弯状态机，在转弯完成前可忽略后续 `$CTRL`，完成后再恢复普通
循线控制。下位机必须实现串口超时停车保护。程序正常退出时会额外发送
`$CTRL,0.000,0.000,0.000`。

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
