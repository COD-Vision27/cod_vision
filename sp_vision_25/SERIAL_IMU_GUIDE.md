# UART 姿态接收与命令发送

协议标准为 [new_uart1.png](new_uart1.png) 和 [new_uart2.png](new_uart2.png)。旧图 `uart.png`、`uart2.png` 不再作为本次接口依据。接收固定 **27 字节**，发送固定 **13 字节**。项目流程、版本差异和运行入口见 [VERSION_COMPARISON_AND_USAGE.md](VERSION_COMPARISON_AND_USAGE.md)。

> 现场配置更新（2026-10-10）：当前控制板实际仍发送 `0D 00`。为解决 `cboard_test configs/sentry.yaml` 的有效帧超时，`sentry.yaml` 显式设置 `tail: [0x0d, 0x00]`，已用于实机只读验证。解析器默认值与 `serial_imu.yaml` 仍保持新图 `00 0D`，不会自动接受另一种尾。固件更新后，将 sentry 的 tail 改回 `[0x00, 0x0d]`。

## 1. 接收：控制板 → PC

`new_uart1.png` 在控制板侧先做了轴映射：

```cpp
Vision_Pitch_Angle = -INS_Info.Roll_Angle;
Vision_Yaw_Angle   =  INS_Info.Yaw_Angle;
Vision_Roll_Angle  = -INS_Info.Pitch_Angle;
```

| 字节偏移（从 0 开始） | 长度 | 图片定义 |
| --- | --- | --- |
| 0 | 2 | 帧头 `01 00` |
| 2 | 4 | Vision_Pitch，小端 float32 |
| 6 | 4 | Vision_Roll，小端 float32 |
| 10 | 4 | Vision_Yaw，小端 float32 |
| 14 | 11 | 图片中置零，不解析为模式或弹速 |
| 25 | 2 | 帧尾 `00 0D` |

配置项 `rpy_offsets` 的顺序是 **roll、pitch、yaw**，因此填写 `[6, 2, 10]`。固件已经对两轴取负，默认 `rpy_signs: [1, 1, 1]`，PC 不重复取负。保留区不参与姿态计算，也没有校验其必须为零。

接收器按 `angle_unit` 将角度转换为弧度，计算并归一化 `q = Rz(yaw) * Ry(pitch) * Rx(roll)`。随后把 q 按原 CBoard 的 `x/y/z/w`、大端 int16、缩放 10000 编码为进程内 `can_frame`，交给 `CBoard::callback()`。该帧不会发送到 CAN 总线；这一方式复用原姿态队列和插值接口，也保留约 `1e-4` 的分量量化精度。

图片没有标明角度单位。示例使用 `degrees`，若实际固件发送弧度，应设置 `radians`。`rpy_bias_deg`、`max_abs_angle_deg` 始终以度表示。`inverse: true` 对结果四元数取共轭，适用于明确的反向旋转约定，不能代替轴映射和实测标定。

## 2. 发送：PC → 控制板

`new_uart2.png` 的接收代码读取：

| 字节偏移 | 长度 | 图片读取字段 | PC 数据来源 |
| --- | --- | --- | --- |
| 0 | 1 | `0xFF` 帧头 | `tx.header`，标准值 `0xff` |
| 1 | 4 | yaw_test → Vx | `Command.yaw` |
| 5 | 4 | pitch_test → Vy | `Command.pitch` |
| 9 | 4 | distance_test → Vz | `Command.horizon_distance` |

总长 **13 字节**，三个字段为小端 float32，**没有 control、shoot 字节，也没有帧尾**。`Vx/Vy/Vz` 只是图片中的变量名，不能据此推断成速度指令；下位机后续如何使用这些值，需要核对固件调用链。

`Command.yaw/pitch` 在视觉算法内是弧度，编码时按 `serial_imu.tx.angle_unit` 转换。RX `angle_unit` 和 TX `tx.angle_unit` 独立配置。距离保留 `Command` 的数值，项目约定为米，固件必须采用相同约定。

示例：yaw = 30°，pitch = -15°，distance = 12.5，使用度发送时完整数据为：

```text
FF 00 00 F0 41 00 00 70 C1 00 00 48 41
```

编码直接使用原浮点命令，不经过 CAN int16 中间量。改变 `control/shoot` 不会改变数据包，因此当前协议无法传递“停止接管”或“允许开火”。`standard` 所用 `Aimer` 也没有填写 `horizon_distance`，默认发送 0；若固件需要距离，需另行定义并填充，不能认为当前已完成距离闭环。

不要添加旧协议的 `tx.tail`。非空 `tx.tail` 会被拒绝，`[]` 可接受但不会添加字节。USB CDC 回调不保证一次收到一个完整包，控制板必须累积并按 13 字节解析；图片未展示这一部分，尚需核对。

## 3. 配置与切换

独立标准通信配置见 [configs/serial_imu.yaml](configs/serial_imu.yaml)。[configs/sentry.yaml](configs/sentry.yaml) 包含视觉配置及当前设备的显式帧尾覆盖。完整视觉程序还需要相机、标定、模型和算法参数，不能仅使用通信配置启动。以下片段对应新图标准；当前固件使用 sentry 的 `tail: [0x0d, 0x00]`。

```yaml
use_can: false
quaternion_canid: 0x01
bullet_speed_canid: 0x110
send_canid: 0xff
imu_timeout_ms: 3000
imu_wait_ms: 100
imu_max_age_ms: 200
serial_imu:
  port: /dev/ttyACM0   # 实际示例使用 /dev/serial/by-id/... 稳定路径
  baudrate: 921600
  timeout_ms: 500
  reconnect_ms: 1000
  protocol: uart27
  header: [0x01, 0x00]
  tail: [0x00, 0x0d]
  rpy_offsets: [6, 2, 10]
  float_endian: little
  angle_unit: degrees
  rpy_signs: [1, 1, 1]
  rpy_bias_deg: [0, 0, 0]
  inverse: false
  max_abs_angle_deg: 36000
  tx:
    header: 0xff
    angle_unit: degrees
    write_timeout_ms: 20
```

`use_can` 缺省为 `true`。设为 `false` 时，`SocketCAN` 跳过 `can_interface` 检查、CAN socket 创建、接口索引查询和 bind，不要求 `can0` 存在。三个 ID 仍需填写且为互不相同的标准 CAN ID，以兼容 CBoard 配置和内部回调。

恢复原 CAN 时设 `use_can: true`，填 `can_interface: can0` 及原 ID，并按电控约定启用 CAN。开关只影响使用 `CBoard` 的程序，不影响 `Gimbal` 的 SP/CRC 协议。

可以配置 RX 头尾、偏移、大小端、单位、符号、偏置和取逆，便于明确的固件变更；**本次标准值仍以上述图片为准**。帧长固定 27，`protocol: auto` 或 21 字节布局会报错。

## 4. 鲁棒性与边界

- 任意半帧、粘包、噪声采用逐字节恢复同步；初次同步和失步后要求连续两帧有效，接收缓存最多两帧。
- 拒绝 NaN、Inf、超限角度；累积角度在三角函数计算前按 360° 取余。
- 接收断开或 `timeout_ms` 内没有有效帧时释放串口并重连，持续垃圾数据也会触发超时。
- raw 8N1、无回显、无软硬件流控；使用 `flock` 和 `TIOCEXCL` 独占设备，正常退出恢复原 termios。
- 完整命令发送和设备打开/关闭共享互斥锁；处理短写、EINTR、EAGAIN，等待受 `write_timeout_ms` 限制。
- 写入失败后阻止继续追加命令并重连，不缓存或重发旧命令。PC 重开串口不保证 MCU 清除残留半帧，MCU 也需要失步恢复。
- 检查有限 float32 表示范围及非负距离；姿态队列满时丢弃最旧数据。
- CBoard 启动、等待未来姿态和姿态新鲜度都有上限，失败抛出异常；通信线程可重连，但未捕获异常的主程序仍会退出。`CBoard::send()` 捕获发送错误并写日志，不提供 MCU 应答确认。

图片协议没有 CRC、序号和 MCU 采样时间戳。头尾与数值检查无法识别所有位错误，也无法精确恢复 MCU 采样时间。姿态使用 PC 回调接收时间与相机时间插值，USB 批量传输延迟仍需实测。下行没有应答，写入成功只表示数据交给操作系统。

## 5. 验证结果（2026-10-10）

从项目根目录运行：

```bash
cmake -S . -B build
cmake --build build --target serial_imu_test cboard_test standard capture -j2
ctest --test-dir build -R serial_imu_protocol --output-on-failure
./build/cboard_test configs/serial_imu.yaml
```

上面的通信命令用于新图标准固件。当前现场固件使用 `./build/cboard_test configs/sentry.yaml`。两份 YAML 分别严格匹配自己的帧尾，不自动混用。`cboard_test` 已捕获异常，发生启动或读取失败时输出错误并返回 1，不再因未捕获异常产生 `terminate`。

相关目标已构建通过，CTest `serial_imu_protocol` **1/1 通过**。测试覆盖图片对应的独立字节样例、轴顺序/符号、两份项目 YAML、度/弧度、噪声/坏帧/半帧/粘包、拒绝旧尾、PTY 断开重连、13 字节发送、写阻塞超时及 CBoard 启动超时/过期拒绝/恢复。

**真实设备尚未按新标准通过。** 此次只读 `/dev/ttyACM0` 约 2 秒得到 54027 字节、2001 个完整帧，帧尾全部为旧的 `0D 00`，新标准 `00 0D` 匹配数为 0。捕获临时文件为 `/tmp/zimiao_new_uart_capture.bin`，不属于仓库持久测试数据。前一帧如下：

```text
01 00 18 50 B1 40 C1 D8 2D C3 9E 43 B5 C2
00 00 00 00 00 00 00 00 00 00 00 0D 00
```

再次只读采样得到 54000 字节、2000 个完整帧，间隔均为 27 字节，仍全部为 `0D 00`。因此传输正常而有效帧超时的原因是配置帧尾不匹配。现场 sentry 配置已明确匹配该帧尾，接收可以继续使用；这是当前固件配置覆盖，不代表固件已符合新图标准。

修复后实机执行 `./build/cboard_test configs/sentry.yaml` 约 2 秒，输出 1823 条姿态，无 warning/error/timeout，以 SIGINT 停止后返回 0。另用不存在的串口验证启动失败，程序清楚报告错误并返回 1，没有 terminate。协议回归测试再次 1/1 通过。实机验证范围为当前固件的只读姿态接收，不包含下行控制和物理轴向正确性。

后续固件应更新为图片定义的 `00 0D`，并确认实际 `CDC_Transmit_HS(Buff, Len)` 的 `Buff` 指向已填充缓冲区、`Len` 为 27。图片无法证明这两个调用参数。默认标准解析器仍拒绝旧尾；固件升级时需同步改回 sentry 配置。

USB 信息只确认 STM32 Virtual COM（0483:5740），不能识别 IMU 芯片型号；原 README 中的 BMI088 是原项目硬件说明。实际三轴方向、角度单位和下行行为仍需更新固件后联调。本次未向真实控制板发送运动或开火命令。
