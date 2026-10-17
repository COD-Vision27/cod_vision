# sp_vision_25 相机配置、测试与排障指南

> 本文覆盖项目当前支持的海康工业相机、迈德威视工业相机和 V4L2/UVC USB 相机。重点说明每个配置参数实际怎样进入驱动、如何验证设备真正接受了参数，以及白屏、黑屏、帧率不足和无法打开时如何定位。构建与完整测试流程见 [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md)，几何标定见 [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md)。

## 目录

1. [先判断使用的是哪套相机后端](#1-先判断使用的是哪套相机后端)
2. [USB 相机白屏的直接结论](#2-usb-相机白屏的直接结论)
3. [曝光、增益、Gamma 和帧率](#3-曝光增益gamma-和帧率)
4. [USB/UVC 相机完整配置](#4-usbuvc-相机完整配置)
5. [海康 HikRobot 相机完整配置](#5-海康-hikrobot-相机完整配置)
6. [迈德威视相机配置](#6-迈德威视相机配置)
7. [图像质量的推荐调节顺序](#7-图像质量的推荐调节顺序)
8. [从取流测试到完整视觉链路](#8-从取流测试到完整视觉链路)
9. [相机标定与参数的关系](#9-相机标定与参数的关系)
10. [当前相机实现的工程限制](#10-当前相机实现的工程限制)
11. [故障速查表](#11-故障速查表)

## 1. 先判断使用的是哪套相机后端

项目中有两类相机入口，不能混为一谈。

### 1.1 主工业相机工厂 `io::Camera`

`io::Camera` 读取 YAML 的 `camera_name`，当前只接受：

```yaml
camera_name: hikrobot
```

或：

```yaml
camera_name: mindvision
```

它被 `standard`、`standard_mpc`、`camera_test`、`capture` 等程序使用。写成 `usb`、`video0` 或其他名称会抛出 `Unknown camera_name`。

### 1.2 全向感知 USB 相机 `io::USBCamera`

`USBCamera` 使用 Linux V4L2 打开 `/dev/video*`，由以下程序直接使用：

- `usbcamera_test`
- `usbcamera_detect_test`
- `multi_usbcamera_test`
- 哨兵全向感知模块

它不是 `CameraBase` 的子类，不能仅靠修改 YAML 替换完整主相机。

### 1.3 后端对比

| 项目 | USB/UVC | 海康 HikRobot | 迈德威视 |
|---|---|---|---|
| Linux 接口 | `/dev/video*`、V4L2 | 海康 MVS SDK | 迈德威视 SDK |
| 测试程序 | `usbcamera_test` | `camera_test` | `camera_test` |
| 配置前缀 | `usb_*` | `exposure_ms`、`gain` | `exposure_ms`、`gamma` |
| 自动曝光 | 代码写入 V4L2 手动模式 1 | 代码强制关闭 | 代码强制关闭 |
| 默认帧率处理 | 从 YAML 请求 | 源码固定请求 150 FPS | 源码设置低帧率档 1 |
| 设备选择 | `/dev/videoX` | SDK 枚举列表第一个设备 | SDK 枚举到的设备 |
| `vid_pid` 作用 | 不使用 | 仅掉线后 USB reset | 仅掉线后 USB reset |

## 2. USB 相机白屏的直接结论

### 2.1 当前环境的复现结果

当前连接的 `/dev/video4` 报告以下能力：

```text
1280x720 MJPG: 最高 60 FPS
exposure_time_absolute: 10..2500，默认 156
gamma: 90..150，默认 120
gain: 4..8，默认 4
sharpness: 默认 2
auto_exposure: 1=Manual，3=Aperture Priority
```

而 `configs/sentry.yaml` 当前 USB 配置为：

```yaml
image_width: 1280
image_height: 720
usb_frame_rate: 120
usb_exposure: 500
usb_gamma: 160
usb_gain: 10
```

实际运行后驱动状态为：

```text
帧率请求 120，实际接受 60 FPS
曝光 500，手动模式
Gamma 请求 160，实际钳位到 150
Gain 请求 10，实际钳位到 8
```

UVC 的 `exposure_time_absolute` 通常以 100 微秒为一个单位，因此：

```math
500\times100\ \mu s=50\ ms
```

50 ms 曝光不仅容易过曝，也只能支持约 20 FPS。实测程序输出约 20 FPS；对采样帧做亮度统计，平均亮度 `YAVG=251.8/255`，说明绝大部分像素已经接近纯白。因此这次白屏不是窗口绘制问题，而是曝光值过大。

### 2.2 当前设备的可用起点

对当前 `/dev/video4`，可以先从以下值开始，而不是继续使用 500：

```yaml
image_width: 1280
image_height: 720
usb_frame_rate: 60
usb_exposure: 20   # 常见 UVC 单位下约 2 ms
usb_gamma: 120
usb_gain: 4
```

这只是当前设备和当前照明下的排错起点，不是所有 USB 摄像头的通用参数。每个设备都必须先用 `v4l2-ctl` 查询自己的范围。

若画面仍偏白，先把 `usb_exposure` 降到 `10`；若明显偏暗，再按 `10` 为步长增加。只有曝光合理后，才逐步增加 gain。

### 2.3 为什么项目配置会产生白屏

`USBCamera::open` 的设置顺序是：

```text
打开 /dev/videoX
  -> 读取 sharpness
  -> 请求 MJPG
  -> 请求 FPS
  -> CAP_PROP_AUTO_EXPOSURE = 1
  -> 请求 gamma 和 gain
  -> 仅当 sharpness == 2 或 3 时设置分辨率和 exposure
```

在 Linux V4L2 中，`CAP_PROP_AUTO_EXPOSURE=1` 通常代表手动曝光，不是“开启自动曝光”。所以程序一定会尝试关闭自动曝光。

此外，只有 `sharpness` 恰好等于 2 或 3 时，代码才写入分辨率和曝光。这是项目用来区分左右全向相机的私有约定，不是 UVC 标准。其他摄像头可能出现：

- 自动曝光已经关闭；
- YAML 的 `usb_exposure` 却没有写入；
- 分辨率也没有按 YAML 设置；
- 最终沿用设备此前残留的手动曝光值。

这也是普通 USB 摄像头上画面异常的另一个重要原因。

### 2.4 重新插拔后不要继续假定设备是 `video5`

重新插拔 USB 摄像头后，Linux 可能重新分配 `/dev/videoX` 编号。当前这台设备的实际枚举结果是：

```text
/dev/video4  Video Capture，1280x720 MJPG，图像节点
/dev/video5  Metadata Capture，UVCH 元数据节点，不是图像
```

因此把 `--name=video5` 传给 `usbcamera_test` 是错误的，即使节点可以被打开，也不会得到普通图像。应先检查：

```bash
for d in /dev/video*; do
  echo "--- $d"
  v4l2-ctl -d "$d" --all 2>/dev/null | \
    grep -E 'Device Caps|Format (Video|Metadata)|Width/Height|Pixel Format'
done
```

当前机器还提供稳定路径：

```text
/dev/v4l/by-id/usb-XHH-260202-H_Integrated_Camera-video-index0 -> /dev/video4
/dev/v4l/by-id/usb-XHH-260202-H_Integrated_Camera-video-index1 -> /dev/video5
```

`video-index0` 是图像流，`video-index1` 是元数据流。推荐把稳定路径去掉开头的 `/dev/` 后传给程序：

```bash
./build/usbcamera_test \
  --name=v4l/by-id/usb-XHH-260202-H_Integrated_Camera-video-index0 \
  --display \
  configs/sentry.yaml
```

如果换到另一台电脑或再次插拔，必须重新执行 `readlink -f /dev/v4l/by-id/*`，不能把 `video4`、`video5` 当成永久设备名。

### 2.5 当前摄像头的真正根因：项目强制进入错误的手动曝光模式

对重新插拔后的图像节点 `/dev/video4` 实测：

| 状态 | V4L2 控制 | 亮度统计 |
|---|---|---:|
| 默认自动曝光 | `auto_exposure=3` | `YAVG` 约 221.9，能看到画面 |
| 项目启动后的手动状态 | `auto_exposure=1`、`exposure_time_absolute=20`、`gamma=120`、`gain=4` | `YAVG` 约 255，几乎纯白 |
| 手动曝光最小值 | `auto_exposure=1`、`exposure_time_absolute=10` | `YAVG` 仍约 253，仍然严重过曝 |

这说明本次问题不能靠继续把 `usb_exposure` 从 20 改成 10、5 或更小解决。该摄像头在当前固件和驱动下，进入手动模式后输出就已经异常；`usb_exposure` 的数值只是请求值，不能保证传感器真的按预期工作。

项目源码中的关键语句是：

```cpp
cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
```

在本机 OpenCV 4.5.4 的 V4L2 后端中，写入 `1` 后，驱动读回：

```text
auto_exposure: 1 (Manual Mode)
```

而该摄像头的正常自动曝光枚举值是 `3`：

```text
auto_exposure: 3 (Aperture Priority Mode)
```

这就是为什么“系统默认打开正常、项目打开后变白”：项目不是只读取摄像头参数，而是主动改变了摄像头的曝光模式。

### 2.6 不修改源码时的临时恢复办法

这是临时诊断手段，不是最终修复。先启动项目并确认使用图像节点，然后另开终端执行：

```bash
v4l2-ctl -d /dev/video4 --set-ctrl=auto_exposure=3
v4l2-ctl -d /dev/video4 --get-ctrl=auto_exposure,exposure_time_absolute,gamma,gain
```

本次实测，项目仍在取流时执行上述命令，画面从纯白恢复，亮度统计约为 `YAVG=170.4`。项目重启后会再次写入 `auto_exposure=1`，所以该命令需要每次重新启动项目后执行。

如果使用稳定路径：

```bash
v4l2-ctl \
  -d /dev/v4l/by-id/usb-XHH-260202-H_Integrated_Camera-video-index0 \
  --set-ctrl=auto_exposure=3
```

### 2.7 最终修复方向

当前后端没有 `usb_auto_exposure` 配置项，且无论 YAML 怎么改都会固定执行 `CAP_PROP_AUTO_EXPOSURE=1`。要正式支持这台摄像头，应在源码中增加明确的曝光模式配置，并按设备验证 OpenCV/V4L2 的枚举值，例如：

```yaml
usb_auto_exposure: true
```

对应逻辑应满足：

1. 自动曝光开启时写入设备实际需要的自动模式值 `3`，不要把 `1` 当作通用“自动”值；
2. 自动曝光开启时不要再写 `usb_exposure`，因为手动曝光控制在该模式下无效或会造成模式切换；
3. 手动曝光时才写 `usb_exposure`，并在写入后立即用 `get` 或 `v4l2-ctl` 回读；
4. 将 `CAP_PROP_AUTO_EXPOSURE`、曝光、Gamma、gain 的 `set()` 返回值记录到日志；
5. 最好允许按相机后端配置自动模式值，因为不同 V4L2 驱动对 OpenCV 属性值的映射并不统一。

在未完成这个后端修复前，不能把“修改 `sentry.yaml` 曝光值”当作这台摄像头的有效解决方案。

## 3. 曝光、增益、Gamma 和帧率

### 3.1 曝光时间

曝光时间决定传感器对一帧累计光线的时长：

- 曝光越长，画面越亮，但运动拖影越严重；
- 曝光越短，运动更清晰，但画面更暗；
- 曝光过长会让高亮区域达到像素上限，细节不可恢复。

曝光必须小于一帧周期。目标帧率为 `f` 时，帧周期为：

```math
T_{frame}=\frac{1}{f}
```

- `f`：目标帧率，单位 FPS，也就是 `s^-1`；
- `T_frame`：每帧可用时间，单位 s。

例如 60 FPS：

```math
T_{frame}=\frac{1}{60}\ s\approx16.67\ ms
```

如果 UVC 曝光单位为 100 微秒，则不拖低 60 FPS 的理论上限约为：

```math
\frac{16.67\ ms}{0.1\ ms}\approx166
```

自瞄目标快速运动，实际曝光应远小于理论上限。项目常见工业相机配置为 0.8 至 3 ms，就是为了降低装甲灯条拖影。

### 3.2 增益

增益放大传感器信号：

- 增益提高会让画面变亮；
- 同时放大噪声，降低颜色和边缘稳定性；
- 增益无法恢复已经过曝丢失的细节；
- 不同 SDK 的数值范围和单位不通用。

推荐先用曝光达到不过曝且运动清晰的状态，再用尽可能低的 gain 补亮。

### 3.3 Gamma

Gamma 是非线性亮度映射，主要改变中间亮度，不等于曝光：

- 它不会增加传感器收到的光；
- 不能修复纯白区域；
- 过高可能抬亮噪声并削弱明暗对比；
- 过低可能让暗部目标消失。

排查白屏时应先调曝光，再调 gain，最后才调 gamma。

### 3.4 白平衡

白平衡调整不同颜色通道的相对增益，使白色物体在图像中接近中性。它主要影响颜色判断，不负责总体亮度。海康驱动当前固定开启连续自动白平衡；USB 相机是否自动白平衡取决于设备当前控制状态，项目代码没有设置它。

### 3.5 分辨率、像素格式与帧率

同一摄像头在不同像素格式和分辨率下，最高帧率可能完全不同。例如当前 `/dev/video4`：

| 格式 | 分辨率 | 最高帧率 |
|---|---:|---:|
| MJPG | 1280×720 | 60 FPS |
| MJPG | 640×480 | 60 FPS |
| YUYV | 1280×720 | 10 FPS |
| YUYV | 640×480 | 30 FPS |

项目 USB 后端请求 MJPG，是为了降低 USB 带宽。YAML 中写入设备不支持的帧率时，驱动通常选择一个可用值，而不会保证请求值生效。

## 4. USB/UVC 相机完整配置

### 4.1 安装检查工具

```bash
sudo apt install v4l-utils ffmpeg
```

### 4.2 确认正确设备节点

```bash
v4l2-ctl --list-devices
ls -l /dev/video*
```

一台 UVC 摄像头可能暴露多个节点：

- `Video Capture`：正常图像节点；
- `Metadata Capture`：只输出帧元数据，不能当普通图像使用；
- 红外/灰度节点：可能只输出单通道图像。

检查某个节点：

```bash
v4l2-ctl -d /dev/video4 --all
```

输出必须包含 `Video Capture`。若只包含 `Metadata Capture`，换同一设备下的另一个 `/dev/videoX`。

### 4.3 查看设备真正支持的模式

```bash
v4l2-ctl -d /dev/video4 --list-formats-ext
```

根据输出选择一组真实存在的：

```text
像素格式 + 宽度 + 高度 + 帧率
```

不要只看产品宣传帧率。设备可能只在 MJPG 或较低分辨率下达到该帧率。

### 4.4 查看控制范围

```bash
v4l2-ctl -d /dev/video4 --list-ctrls-menus
```

重点寻找：

```text
auto_exposure
exposure_time_absolute
gain
gamma
white_balance_automatic
sharpness
```

记录每个控制的 `min`、`max`、`step`、`default` 和当前 `value`。如果设备根本没有 gain 或 gamma 控件，相应 YAML 项即使写了也不会生效。

### 4.5 先在项目外验证自动曝光

先让设备回到自动曝光，确认硬件可以产生正常画面：

```bash
v4l2-ctl -d /dev/video4 --set-ctrl=auto_exposure=3
ffplay -f v4l2 -input_format mjpeg -video_size 1280x720 -framerate 60 /dev/video4
```

如果自动曝光下仍是白屏，应检查镜头、驱动、设备固件、节点选择或图像格式，而不是继续调项目 YAML。

注意：启动 `usbcamera_test` 后，项目会再次写入 `auto_exposure=1`，切回手动模式。因此外部自动曝光只用于隔离硬件问题。

### 4.6 计算手动曝光起点

UVC `exposure_time_absolute` 标准单位通常是 100 微秒：

| YAML 值 | 常见等效曝光 |
|---:|---:|
| 10 | 1 ms |
| 20 | 2 ms |
| 30 | 3 ms |
| 50 | 5 ms |
| 100 | 10 ms |
| 500 | 50 ms |

仍应以设备驱动文档和实测为准。对于快速装甲，建议从 1 至 3 ms 开始；对于静态标定板，可以适当加长，但不能过曝。

### 4.7 准备配置

建议为单个 USB 相机新建独立配置，而不是直接修改完整哨兵参数。至少包含：

```yaml
image_width: 1280
image_height: 720
usb_frame_rate: 60
usb_exposure: 20
usb_gamma: 120
usb_gain: 4
```

如果运行 `usbcamera_detect_test`，同一个 YAML 还需要包含 YOLO、ROI 和传统检测相关字段。因此最方便的方式通常是复制 `configs/sentry.yaml`，再把 USB 参数改成设备真实范围。

#### 4.7.1 USB 配置字段与源码的准确对应关系

下表描述的是当前 `io/usbcamera/usbcamera.cpp` 的真实行为。YAML 中的值只是程序向驱动发出的请求，驱动可能拒绝、取最接近值或钳位到合法范围。

| YAML 字段 | 含义与常见单位 | 写入的 OpenCV/V4L2 属性 | 当前代码何时写入 | 注意事项 |
|---|---|---|---|---|
| `image_width` | 请求图像宽度，单位为像素 | `CAP_PROP_FRAME_WIDTH` | 仅当相机当前 `sharpness` 恰好为 2 或 3 | 必须与设备支持的分辨率组合一致；实际值要回读 |
| `image_height` | 请求图像高度，单位为像素 | `CAP_PROP_FRAME_HEIGHT` | 仅当 `sharpness` 为 2 或 3 | 必须与宽度、像素格式和 FPS 一起选择 |
| `usb_frame_rate` | 请求帧率，单位为帧每秒，即 FPS | `CAP_PROP_FPS` | 打开设备后总会尝试写入 | 当前设备在 1280x720 MJPG 下最高 60 FPS，写 120 不会得到 120 FPS |
| `usb_exposure` | 手动曝光控制原始值；UVC 设备常以 100 微秒为 1 个单位，但不是所有驱动都如此 | `CAP_PROP_EXPOSURE` | 仅当 `sharpness` 为 2 或 3 | 程序先写 `CAP_PROP_AUTO_EXPOSURE=1`，在当前 Linux/V4L2 设备上表示手动曝光 |
| `usb_gamma` | Gamma 控制原始值，无统一物理单位 | `CAP_PROP_GAMMA` | 总会尝试写入 | 当前设备范围为 90 至 150，配置 160 会被钳位为 150 |
| `usb_gain` | 传感器/数字增益控制原始值，无跨设备统一单位 | `CAP_PROP_GAIN` | 总会尝试写入 | 当前设备范围为 4 至 8，配置 10 会被钳位为 8 |

设置顺序是：打开设备、读取 sharpness、请求 MJPG、请求 FPS、切换手动曝光、设置 Gamma、设置 gain，最后才在 sharpness 为 2/3 时设置宽度、高度和曝光。某些驱动在改变格式或分辨率后会重置曝光等控制，因此程序启动后的回读比配置文件本身更可信。

`configs/sentry.yaml` 中类似 `# 1-80000`、`# 0-96` 的注释不能当作所有 USB 相机的合法范围。这些范围来自某个历史设备；当前接入设备的曝光范围是 10 至 2500、Gamma 范围是 90 至 150、gain 范围是 4 至 8。更换设备后必须重新运行：

```bash
v4l2-ctl -d /dev/video4 --list-ctrls-menus
```

#### 4.7.2 FOV 字段不负责设置相机

以下字段与 USB 相机写寄存器无关：

| YAML 字段 | 当前用途 |
|---|---|
| `fov_h` | 被全向感知 `Decider` 读取，但当前 `delta_angle()` 没有使用 |
| `fov_v` | 被 `Decider` 读取，但当前 `delta_angle()` 没有使用 |
| `new_fov_h` | 左、右 USB 相机从目标横向位置换算 yaw 偏角时使用，单位为度 |
| `new_fov_v` | 左、右 USB 相机从目标纵向位置换算 pitch 偏角时使用，单位为度 |

FOV 是 Field of View，即视场角。水平视场角表示画面最左边到最右边覆盖的角度范围，竖直视场角表示画面最上边到最下边覆盖的角度范围。它由传感器尺寸、镜头焦距、裁剪和缩放共同决定，不能靠修改 YAML 让镜头的真实视野变宽或变窄。

检测器先把装甲板中心像素归一化：

$$
u = \frac{x}{W}, \qquad v = \frac{y}{H}
$$

其中：

- $x$、$y$ 是装甲板中心的像素坐标；OpenCV 图像原点在左上角，$x$ 向右增大，$y$ 向下增大；
- $W$、$H$ 是当前实际图像的宽度和高度，单位为像素；
- $u$、$v$ 是无量纲归一化坐标，通常位于 0 至 1；画面中心约为 $(0.5, 0.5)$。

当前左相机使用：

$$
\Delta\mathrm{yaw}_{left} = 62^\circ + \left(\frac{1}{2}-u\right)F_h
$$

当前右相机使用：

$$
\Delta\mathrm{yaw}_{right} = -62^\circ + \left(\frac{1}{2}-u\right)F_h
$$

两台相机的俯仰偏角都使用：

$$
\Delta\mathrm{pitch} = \left(v-\frac{1}{2}\right)F_v
$$

其中：

- $F_h$ 是 `new_fov_h`，即用于线性换算的水平视场角，单位为度；
- $F_v$ 是 `new_fov_v`，即用于线性换算的竖直视场角，单位为度；
- $\Delta\mathrm{yaw}$ 是目标相对云台当前朝向的水平角偏差，单位为度；
- $\Delta\mathrm{pitch}$ 是目标相对图像中心的竖直角偏差，单位为度；
- $+62^\circ$ 和 $-62^\circ$ 是源码中写死的左右相机安装方位角，不是从 YAML 读取的标定结果。

后置工业相机分支同样采用线性近似，但水平视场角 `54.2` 度、竖直视场角 `44.5` 度和方位偏置 `170` 度全部硬编码在 `decider.cpp` 中。这些值若与实物安装不符，修改 `fov_h` 或 `fov_v` 也不会修正结果。

这种“归一化坐标乘视场角”的算法是近似模型。针孔相机严格的像素到视线角换算应使用相机内参，例如水平角可写为：

$$
\theta_x = \arctan\left(\frac{x-c_x}{f_x}\right)
$$

其中 $f_x$ 是水平方向焦距的像素表示，$c_x$ 是主点横坐标，$\theta_x$ 是相对光轴的水平角。视场角线性近似在画面边缘、广角镜头或畸变明显时误差会增大。

当前 `Decider` 还会读取 `image_width` 和 `image_height`，但上述 `delta_angle()` 实际使用的是检测器根据当前 `cv::Mat` 尺寸计算出的 $u$、$v$，没有使用这两个成员变量。字段仍不能随意删除，因为构造函数会通过 YAML 强制读取，缺失时可能直接抛出异常。

另外，部分配置中出现的 `new_image_width`、`new_image_height` 和 `new_usb_exposure` 在当前检索到的 C++ 源码中没有读取点，修改它们不会改变当前相机行为。它们更像历史配置残留；真正生效的是 `image_width`、`image_height` 和 `usb_exposure`。

### 4.8 运行取流测试

```bash
./build/usbcamera_test \
  --name=video4 \
  --display \
  configs/sentry.yaml
```

`--name` 会被代码拼成 `/dev/<name>`，所以常规写法是 `video4`，不要写 `/dev/video4`。

如使用稳定符号链接，也可以传入 `/dev/` 后面的部分，例如：

```bash
./build/usbcamera_test \
  --name=v4l/by-id/usb-camera-name-video-index0 \
  --display \
  configs/sentry.yaml
```

### 4.9 程序运行后复核真实参数

另开终端：

```bash
v4l2-ctl -d /dev/video4 \
  --get-ctrl=auto_exposure,exposure_time_absolute,gamma,gain,sharpness

v4l2-ctl -d /dev/video4 --get-fmt-video --get-parm
```

必须以这里读回的值为准，而不是以 YAML 为准。OpenCV `set` 可能：

- 返回失败但项目没有打印；
- 被驱动钳位到上下限；
- 选择最接近的帧率；
- 在改变分辨率后重置其他参数。

### 4.10 量化是否过曝

停止项目程序，使用 ffmpeg 读取一帧亮度统计：

```bash
ffmpeg -hide_banner \
  -f v4l2 \
  -input_format mjpeg \
  -video_size 1280x720 \
  -framerate 60 \
  -i /dev/video4 \
  -frames:v 1 \
  -vf signalstats,metadata=print \
  -f null - 2>&1 | grep 'lavfi.signalstats.Y'
```

关键字段：

- `YMIN`：最暗像素的亮度；
- `YAVG`：平均亮度；
- `YMAX`：最亮像素的亮度；
- 数值范围通常为 0 至 255。

`YAVG` 非常接近 255 且画面大面积失去纹理，说明严重过曝。单独出现 `YMAX=255` 并不一定有问题，因为装甲灯条等局部高亮本来可能饱和。

### 4.11 实时检测

取流正常后再运行：

```bash
./build/usbcamera_detect_test \
  --name=video4 \
  --display \
  configs/sentry.yaml
```

该程序验证 USB 取流和 YOLO，不执行 PnP、Tracker 或瞄准。画面正常但检测失败时，应转去检查模型、敌方颜色、ROI、曝光下的灯条形态和训练数据域。

### 4.12 USB 白屏排查顺序

1. 用 `v4l2-ctl --all` 确认选择的是 Video Capture 节点。
2. 用自动曝光加 `ffplay` 验证硬件原始画面。
3. 查询曝光、gain、gamma 的实际范围。
4. 把 YAML 曝光降到约 1 至 3 ms 对应值。
5. 把 gain 和 gamma 先恢复到设备默认值。
6. 把 YAML 帧率改成 `--list-formats-ext` 中真实支持的值。
7. 运行项目后用 `--get-ctrl` 读回真实参数。
8. 若 `sharpness` 不是 2 或 3，当前源码不会应用曝光和分辨率，应修改后端实现。

## 5. 海康 HikRobot 相机完整配置

### 5.1 当前后端支持范围

`HikRobot::capture_start` 只枚举：

```cpp
MV_USB_DEVICE
```

因此当前实现针对海康 USB 工业相机，不会枚举 GigE 相机。项目基准设备为 `MV-CS016-10UC`，当前机器通过 `lsusb` 显示：

```text
2bdf:0001 Hikrobot MV-CS016-10UC
```

### 5.2 安装和识别 MVS

先确认 SDK：

```bash
ls /opt/MVS/bin/MVS.sh
ldconfig -p | grep MvCameraControl
```

启动官方客户端：

```bash
/opt/MVS/bin/MVS.sh
```

在官方客户端中先完成三个检查：

1. 能枚举并打开相机；
2. 连续采集画面正常；
3. Feature Tree 中能读写曝光、增益、像素格式和帧率。

运行项目前关闭 MVS 客户端，避免相机被独占。若 MVS 客户端都不能打开，相机问题不在本项目算法层。

### 5.3 USB 权限

检查设备：

```bash
lsusb | grep -i '2bdf:0001'
```

SDK 安装通常创建类似规则：

```text
/etc/udev/rules.d/80-drivers-SDK-2bdf.rules
```

当前机器的规则将该厂商 USB 设备设为 `MODE="0666"` 并分配给 `plugdev`。规则修改后执行：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

然后重新插拔相机。不要长期依赖 `sudo` 运行视觉程序；那会掩盖权限问题，并导致生成的日志或录像属于 root。

### 5.4 海康配置字段

```yaml
camera_name: hikrobot
exposure_ms: 3
gain: 10.0
vid_pid: "2bdf:0001"
```

| 字段 | 源码中的实际作用 |
|---|---|
| `camera_name` | 选择 `HikRobot` 后端 |
| `exposure_ms` | 乘以 1000 后写入 SDK 的 `ExposureTime`，即 YAML 单位为 ms，SDK 写入单位为 us |
| `gain` | 写入 SDK 浮点节点 `Gain`，范围和单位由具体相机报告 |
| `vid_pid` | 解析成十六进制 VID/PID，仅在掉线恢复时执行 `libusb_reset_device` |

一个容易误解的点是：`vid_pid` 不参与相机选择。驱动始终使用 SDK 枚举列表中的第一个 USB 相机。如果同时连接多台海康相机，修改 `vid_pid` 并不能指定要打开哪一台。

### 5.5 驱动启动时实际设置的参数

打开相机后，源码依次执行：

```text
BalanceWhiteAuto = Continuous
ExposureAuto = Off
GainAuto = Off
ExposureTime = exposure_ms * 1000 us
Gain = gain
FrameRate = 150
StartGrabbing
```

因此：

- 曝光和增益始终是手动值；
- 白平衡是连续自动；
- 帧率不是 YAML 参数，而是在源码中固定请求 150 FPS；
- YAML 中其他 USB 相机字段不会影响海康相机；
- 源码没有显式设置 `TriggerMode` 和 `PixelFormat`。

### 5.6 在 MVS 客户端中准备相机状态

由于项目没有设置全部节点，建议先在 MVS 客户端确认：

```text
Acquisition Mode: Continuous
Trigger Mode: Off
Exposure Auto: Off
Gain Auto: Off
Pixel Format: BayerGR8 / BayerRG8 / BayerGB8 / BayerBG8 之一
```

当前代码的像素转换表只支持上述四种 8 位 Bayer 格式。若相机保留成 `Mono8`、`RGB8`、`BGR8`、YUV 或 10/12 位 Bayer，下面的查表会失败：

```cpp
type_map.at(pixel_type)
```

表现可能是取流线程异常退出或进程终止，而不是自动完成格式转换。

### 5.7 运行海康取流测试

准备专用配置，例如 `configs/camera.yaml`：

```yaml
camera_name: hikrobot
exposure_ms: 3
gain: 10.0
vid_pid: "2bdf:0001"
```

运行：

```bash
./build/camera_test \
  --config-path=configs/camera.yaml \
  --display
```

当前环境已经实测可以打开海康相机，日志会出现：

```text
HikRobot's daemon thread started.
HikRobot's capture thread started.
```

并能观察到约 150 FPS 的连续取流。第一条 FPS 通常受初始化间隔影响，不应拿第一帧数值判断稳定帧率。

### 5.8 海康曝光怎样设置

YAML 的单位明确是 ms：

| `exposure_ms` | SDK `ExposureTime` |
|---:|---:|
| 0.8 | 800 us |
| 2.0 | 2000 us |
| 2.5 | 2500 us |
| 3.0 | 3000 us |

项目配置中的 0.8 至 3 ms 是高速自瞄的实际量级。调节建议：

1. 室内静态检查可从 3 ms 开始。
2. 快速转动装甲出现拖影时，降到 2 ms、1 ms 或更低。
3. 画面变暗时优先改善照明，再适量提高 gain。
4. 出现大片白色饱和时先降低曝光，不要调 gamma。

目标 150 FPS 的帧周期约为：

```math
\frac{1000}{150}\approx6.67\ ms
```

曝光必须小于 6.67 ms 才可能维持 150 FPS，还要为传感器读出留余量。若把曝光设为 8 ms，即使相机接受参数，也不可能稳定达到 150 FPS。

### 5.9 海康 gain 怎样设置

项目把 `gain` 原样传给 SDK 的浮点节点。其有效范围应在 MVS 客户端 Feature Tree 中读取，因为不同相机型号、模拟/数字增益模式和固件可能不同。

调节原则：

- 从较低 gain 开始；
- 曝光已经足够短但画面偏暗时再增加；
- 检查暗部噪声是否让灯条轮廓和颜色分类抖动；
- 控制台出现 `MV_CC_SetFloatValue("Gain", ...) failed` 时，不要认为 YAML 已生效。

### 5.10 海康白屏和黑屏

白屏优先检查：

1. `exposure_ms` 是否过大；
2. gain 是否过高；
3. 镜头光圈是否完全打开且正对强光；
4. MVS 客户端中相同参数是否也过曝；
5. 自动白平衡是否造成单个颜色通道异常，但白平衡通常不会造成三通道全白。

黑屏优先检查：

1. 镜头盖和光圈；
2. 曝光是否过短；
3. gain 是否过低；
4. TriggerMode 是否残留为 On，但没有外部触发；
5. PixelFormat 是否为当前代码不支持的格式；
6. USB 供电和带宽。

### 5.11 海康“找不到相机”

按顺序检查：

```bash
lsusb | grep -i 2bdf
/opt/MVS/bin/MVS.sh
pgrep -af 'MVS|camera_test|standard|standard_mpc'
ldd build/camera_test | grep -E 'MvCameraControl|not found'
```

常见原因：

- MVS 客户端或另一个视觉进程占用相机；
- SDK/驱动版本与动态库不匹配；
- udev 权限尚未重新加载；
- USB 线或供电不稳定；
- 相机接在带宽不足的 Hub；
- 当前实现只枚举 USB，相机却是 GigE 型号。

### 5.12 海康错误码

日志以十六进制打印 SDK 返回值。常见值包括：

| 错误码 | 源码名称 | 含义 |
|---|---|---|
| `0x80000001` | `MV_E_SUPPORT` | 功能不支持 |
| `0x80000003` | `MV_E_CALLORDER` | API 调用顺序错误 |
| `0x80000102` | `MV_E_GC_RANGE` | 参数超出相机节点范围 |
| `0x80000106` | `MV_E_GC_ACCESS` | 当前节点不可访问或不可写 |
| `0x80000107` | `MV_E_GC_TIMEOUT` | GenICam 操作超时 |
| `0x80000300` | `MV_E_USB_READ` | USB 读取错误 |
| `0x80000302` | `MV_E_USB_DEVICE` | USB 设备异常 |
| `0x80000304` | `MV_E_USB_BANDWIDTH` | USB 带宽不足 |
| `0x80000305` | `MV_E_USB_DRIVER` | 驱动不匹配或未安装 |

出现范围或访问错误时，先在 MVS 客户端查看节点当前的最小值、最大值、增量和可写条件。

### 5.13 海康掉线恢复

取流失败后，守护线程会：

```text
停止取流
  -> 关闭并销毁句柄
  -> 根据 vid_pid 执行 USB reset
  -> 重新枚举并打开
```

`vid_pid` 必须与 `lsusb` 一致，否则 reset 阶段找不到设备，但 SDK 后续仍可能尝试重新枚举。USB reset 也受权限影响。

### 5.14 海康实时检测

取流稳定后再运行：

```bash
./build/camera_detect_test configs/standard4.yaml
```

传统检测路径：

```bash
./build/camera_detect_test \
  --tradition=true \
  configs/standard4.yaml
```

配置必须同时包含相机和检测器字段。该程序的 YOLO 构造使用 debug 模式，适合观察识别结果和推理速度。

## 6. 迈德威视相机配置

迈德威视使用：

```yaml
camera_name: mindvision
exposure_ms: 2
gamma: 0.5
vid_pid: "f622:d13a"
```

源码设置：

```text
关闭自动曝光
ExposureTime = exposure_ms * 1000 us
Gamma = gamma * 100
输出格式 = BGR8
TriggerMode = 0（连续采集）
FrameSpeed = 1（低帧率档）
```

与海康不同：

- YAML 使用 gamma，不读取 gain；
- 输出由 SDK 直接转换成 BGR8；
- 触发模式会被明确设置为连续采集；
- 宽高取相机能力中的最大分辨率；
- `vid_pid` 同样只用于掉线 USB reset，不选择设备。

测试命令仍是：

```bash
./build/camera_test \
  --config-path=configs/camera.yaml \
  --display
```

需先把 `configs/camera.yaml` 的活动配置切换为 mindvision，不能同时保留两个未注释的 `camera_name`。

## 7. 图像质量的推荐调节顺序

### 7.1 通用顺序

1. 确认像素格式、分辨率和帧率真实生效。
2. 关闭自动曝光，设置较短曝光。
3. 调曝光到高亮区域不大面积饱和。
4. 检查快速转动目标是否拖影。
5. 在曝光不能再增加时，用 gain 补亮。
6. 最后微调 gamma、白平衡或亮度。
7. 用实际检测器验证，而不是只追求“人眼看起来亮”。

### 7.2 自瞄图像与普通视频的目标不同

自瞄更看重：

- 灯条边缘清晰；
- 快速运动没有明显拖影；
- 红蓝颜色可区分；
- 装甲数字仍有纹理；
- 同一曝光下跨帧稳定。

画面整体略暗通常比灯条和装甲区域过曝更容易处理。纯白灯条会失去宽度、角点和颜色信息，直接影响关键点、PnP 和分类。

### 7.3 建议记录的指标

每组参数至少记录：

| 指标 | 作用 |
|---|---|
| 实际 FPS | 判断曝光或带宽是否限制帧率 |
| 平均处理延迟 | 判断相机缓存和推理延迟 |
| 检测召回率 | 判断目标是否经常漏检 |
| 关键点抖动 | 判断曝光、噪声和模型稳定性 |
| 饱和像素比例 | 判断是否过曝 |
| 快速运动拖影长度 | 判断曝光时间是否过长 |

只看静止画面无法完成自瞄相机调参。

## 8. 从取流测试到完整视觉链路

### 8.1 USB 相机

```text
v4l2-ctl / ffplay
  -> usbcamera_test
  -> usbcamera_detect_test
  -> 全向感知模块
```

USB 相机当前不能直接进入 `standard` 或 `standard_mpc`。

### 8.2 海康或迈德威视

```text
厂商客户端
  -> camera_test
  -> camera_detect_test
  -> capture + 标定
  -> debug 入口
  -> standard / standard_mpc
```

不要跳过厂商客户端和单相机测试，直接用完整入口排查相机。完整入口同时涉及通信、姿态、模型和状态机，会放大定位难度。

### 8.3 测试命令汇总

```bash
# USB 取流
./build/usbcamera_test --name=video4 --display configs/sentry.yaml

# USB + YOLO
./build/usbcamera_detect_test --name=video4 --display configs/sentry.yaml

# 主工业相机取流
./build/camera_test --config-path=configs/camera.yaml --display

# 主工业相机 + 检测
./build/camera_detect_test configs/standard4.yaml

# 标定采集：工业相机 + CAN/IMU
./build/capture --output-folder=assets/img_with_q configs/calibration.yaml
```

## 9. 相机标定与参数的关系

以下变化后必须重新做内参：

- 分辨率改变；
- 传感器 ROI 或裁剪改变；
- binning/decimation 改变；
- 镜头或焦距改变；
- 明显重新对焦；
- 图像进入算法前发生几何缩放。

仅改变曝光、gain、gamma 通常不改变几何内参，但会改变圆心和装甲关键点的检测精度。标定时的图像不能过曝或模糊。

USB 相机加入完整主链路后，也必须使用它自己的：

```text
camera_matrix
distort_coeffs
R_camera2gimbal
t_camera2gimbal
```

不能复用海康相机标定结果。

## 10. 当前相机实现的工程限制

### 10.1 USB 后端

- 用 sharpness 值 2/3 识别左右相机，不通用。
- 只有识别为左右相机后才设置宽高和曝光。
- 自动曝光固定写成手动模式，没有 YAML 开关。
- 不检查大部分 OpenCV `set` 的返回值。
- 不读回并打印实际曝光、gain、gamma、格式和帧率。
- 打开失败后消费者可能一直等待空队列。
- 设备路径依赖 `/dev/videoX`，默认不稳定。

建议后续增加：

```yaml
usb_device: /dev/v4l/by-id/...
usb_pixel_format: MJPG
usb_auto_exposure: false
usb_exposure: 20
usb_white_balance_auto: true
```

并对每个 `set` 检查返回值、立即 `get` 回读和打印。

### 10.2 海康后端

- 只枚举 USB 海康相机。
- 总是打开枚举列表第一个设备，不能按序列号选择。
- `vid_pid` 只用于 reset，名称容易让人误认为它用于选择设备。
- 帧率固定为 150，不能从 YAML 配置。
- 不显式设置连续采集/触发模式。
- 像素格式只支持四种 Bayer8，未做友好错误处理。
- 设置帧率的返回值没有检查。
- 时间戳是在 `GetImageBuffer` 返回后生成，不是相机硬件曝光时间。
- 掉线重启路径对“尚未成功创建 handle”的情况缺少严格状态保护。

建议后续增加：

```yaml
hik_serial_number: ...
hik_frame_rate: 150
hik_trigger_mode: false
hik_pixel_format: BayerRG8
```

并通过 SDK 读取实际节点值和硬件时间戳。

### 10.3 多相机共用问题

海康、迈德威视和 USB 后端都缺少统一的设备身份、能力查询和参数回读接口。新增相机时，不应只实现 `read()`；还应提供：

- 稳定设备标识；
- 实际分辨率和 FPS；
- 曝光、gain、gamma 的范围及当前值；
- 像素格式；
- 硬件或主机时间戳类型；
- 丢帧和重连计数。

## 11. 故障速查表

| 现象 | 最可能原因 | 第一检查项 |
|---|---|---|
| USB 画面全白 | 手动曝光过大 | `v4l2-ctl --get-ctrl=exposure_time_absolute` |
| USB 实际 FPS 约 20 | 曝光 500 约等于 50 ms | 降低 `usb_exposure` |
| USB 请求 120 只有 60 FPS | 当前格式不支持 120 | `--list-formats-ext` |
| USB 参数似乎不生效 | 驱动钳位或 sharpness 不是 2/3 | 运行后 `--get-ctrl` 回读 |
| USB 程序卡住 | 设备未产帧，线程等待空队列 | 先用 `ffplay` 验证节点 |
| USB 找到多个 video 节点 | 包含 metadata/IR 节点 | `v4l2-ctl --all` 看 capabilities |
| 海康提示 Not found camera | 权限、占用、SDK 或非 USB 型号 | `lsusb` 和 MVS 客户端 |
| 海康 OpenDevice 失败 | 被其他进程独占 | 关闭 MVS 和其他视觉进程 |
| 海康 GetImageBuffer 超时 | TriggerMode On、掉线或带宽问题 | MVS 中设 TriggerMode Off |
| 海康参数设置返回 `0x80000102` | 数值超出相机范围 | MVS Feature Tree 查询范围 |
| 海康取流线程异常退出 | 像素格式不在四种 Bayer8 中 | 检查 PixelFormat |
| 海康画面全白 | 曝光/gain 过大 | 降低 `exposure_ms` 和 gain |
| 海康帧率达不到 150 | 曝光过长或相机/USB 上限 | 保证曝光明显小于 6.67 ms |
| 画面正常但红蓝识别相反 | Bayer 转色/通道顺序或配置颜色 | 用已知红蓝目标核对 BGR 通道 |
| PnP 距离错误 | 使用了其他相机或分辨率的内参 | 重新标定并核对输入尺寸 |

相机问题应先在“设备能力和真实参数”层解决，再进入模型、PnP 和控制链路。YAML 只是请求值，设备读回值才是实际状态。
