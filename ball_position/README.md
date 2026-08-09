# E题

这个代码可以用在 MaixCAM-Pro 和 MaixCAM2 上；`models` 同时包含两种设备的模型。

思路:
1. 使用 YOLO11 检测钢球
2. 固定maixcam到水管正上方, 固定水管, 计算小球在摄像头画面的像素位置, 反推出小球在水管上的位置
3. MaixCAM2 每处理一帧都通过 UART0 向 STM32F407 发送当前坐标或丢球状态

配置(可能没写全, 具体要自己看代码):

- 是否开启可选网络输出，推荐同时只开启一个，默认全部关闭
USE_RTSP=False
USE_JPEG=False

- 输入打印. 关掉后提升帧率
DEBUG_LOG=False

- 畸变校准
LENS_CORR_ENABLE=False
LENS_CORR_STRENGTH=0.6

## MaixVision 临时运行

板端运行入口是 `main.py`。当前版本已经把 UART 协议、坐标投影和滤波逻辑
内联到该文件中，MaixVision 临时运行时不再依赖同目录的其他 Python 文件。

启动后必须看到：

```text
build: YOLO11_CENTER_ZERO_UART0_MODELPATHFIX_XINV_ZERO464_CAL5_20260731
```

如果仍出现 `import ball_uart` 或 `import ball_position` 错误，说明 MaixVision
运行的还是旧标签页或旧副本，需要重新打开当前目录中的 `main.py`。

## 坐标方向

- 图像左端为 `-12.5 cm`。
- 尺面红色 `0` 为 `0 cm`，对应标定轴比例 `0.464`（MaixCAM2 模型坐标约 `x=297.3`）。
- 图像右端为 `+12.5 cm`。
- 超出标定轴的检测中心会先钳位到几何轴端点，再执行实测距离校正；校正后超出 UART `±12.5 cm` 量程时发送无效帧。

几何坐标之后按实测数据进行五点分段线性校正：

```python
POSITION_CALIBRATION_POINTS_CM = (
    (-9.36, -10.0),
    (-4.65, -5.0),
    (0.0, 0.0),
    (4.17, 5.0),
    (8.25, 10.0),
)
```

每一项是“校正前读数、实际厘米”。`-10 cm` 到 `+10 cm` 之间采用分段线性插值；
该范围之外按最外侧两个标定点线性外推，补充 `±12.5 cm` 实测点后可继续提高边缘精度。

## UART0

UART0 仅在 MaixCAM2 上自动启用；MaixCAM / MaixCAM-Pro 保留检测、显示和
图片采集功能，但不打开该串口。

| MaixCAM2 | STM32F407 | 方向 |
| --- | --- | --- |
| `U0T / UART0_TX` | `PC7 / USART6_RX` | MaixCAM2 发送坐标 |
| `U0R / UART0_RX` | `PC6 / USART6_TX` | 预留反向通信 |
| `GND` | `GND` | 必须共地 |

串口使用 `/dev/ttyS0`、115200、8N1、3.3 V TTL，不在应用中配置 pinmux。

```text
$B,<seq>,<x_0.1mm>,<valid>,<conf>*<XOR>\r\n
$B,42,-503,1,93*64\r\n
```

- 每处理一帧发送一次，坐标使用屏幕显示的滤波后 `position_cm`。
- 有球时 `valid=1`，坐标乘 100 后取整为 0.1 mm，置信度转换为 0-100。
- 丢球或坐标超出 `[-12.5, +12.5] cm` 时发送 `x=0,valid=0,conf=0`。
- 发送成功后序号加一并在 255 后回绕；写失败时保留序号并在屏幕显示错误。
- UART0 可能同时输出系统启动日志，STM32F407 应从 `$B,` 重新同步，并校验字段、行尾和 XOR。

## 图片采集

- 屏幕右下角蓝色 `START JPG`：开始或继续采集。
- 红色 `STOP JPG`：停止接收新图，橙色 `SAVING...` 消失后表示队列已写完。
- 所有图片直接保存在同一个目录：`/root/steel_ball_dataset/`，不再创建 `000001`、`000002` 子目录。
- 文件名使用 `frame_000001.jpg`、`frame_000002.jpg`……连续递增；每次重新开始或程序重启都会扫描已有编号，从下一个编号继续，不会覆盖旧数据。
- 保存内容是送入 YOLO 的干净 RGB 图像：包含可选的畸变校正，但不包含坐标轴、检测框、文字和采集按钮，可直接作为训练数据。
- 目标采样率为 30 张/秒；如果 YOLO 主循环低于 30 FPS，或存储写入跟不上，实际保存率会低于 30。屏幕左下角会显示：下一张图片编号、已保存/已入队、队列数、丢失数和实际保存 FPS。

相关配置：

```python
CAPTURE_ROOT = "/root/steel_ball_dataset"
CAPTURE_TARGET_FPS = 30
CAPTURE_JPEG_QUALITY = 95
CAPTURE_QUEUE_SIZE = 3
```

写盘使用小型非阻塞后台队列，队列满时丢弃采集帧而不等待，避免磁盘速度拖住 YOLO 主循环。
为了尽量达到 30 张/秒，工程默认关闭逐帧调试打印；需要看各阶段耗时时再把 `DEBUG_LOG` 改为 `True`，调试功能仍然保留。

## 两种设备

- MaixCAM2 优先使用板端 `/root/models/steel_ball_yolo11n_640x160_clean1077/steel_ball_yolo11n_640x160_clean1077.mud`（640x160 YOLO11 AXMODEL），并兼容旧目录 `/root/models/steel_ball_yolo11_640x160_maixcam2/`。
- `.mud`、`_npu.axmodel` 和 `_vnpu.axmodel` 三个模型文件必须放在上述同一目录中。
- MaixCAM-Pro（设备名 `maixcam` / `maixcam-pro` / `maixcam_pro`）自动使用 `320x320` YOLO11 CVIMODEL。
- MaixCAM2 模型使用板端绝对路径；MaixCAM-Pro 模型随工程的 `models` 目录发布。
- MaixCAM2 自动打开 UART0；MaixCAM / MaixCAM-Pro 不打开 UART0。
- MaixCAM-Pro 必须使用系统镜像与 MaixPy runtime 成套的官方版本；如果仍出现 `libcvi_bin_isp.so ... symbol not found`，那是系统库 ABI 不匹配，单独重装同版本号的 Python wheel 不能解决。

## 主机回归

在本目录执行：

```powershell
python -B -m unittest -v test_ball_uart.py
```

`ball_uart.py`、`ball_position.py` 和 `test_ball_uart.py` 只用于主机回归和源码
对照，不需要打包到 MaixCAM2。
