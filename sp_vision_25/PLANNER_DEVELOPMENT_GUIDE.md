# 新版自瞄与 Planner 开发指南

> 本文描述使用 io::Gimbal、auto_aim::Planner 和 TinyMPC 的新版自瞄路径。主要入口是 src/standard_mpc.cpp，调试入口是 src/auto_aim_debug_mpc.cpp。src/sentry.cpp 仍是旧版 CBoard + Aimer 路径，不能把两条链路混在一起。当前工作树还包含 `use_can: false` 的 CBoard/UART 适配，文中会单独标出它与原始 SP/CRC Gimbal 协议的差异。

## 1. 新旧路径对照

| 路径 | 主要入口 | 决策器 | 下位机接口 | 主要特点 |
|---|---|---|---|---|
| 旧版 | src/standard.cpp、src/sentry.cpp | Aimer、Shooter | CBoard::send(Command) | 直接计算目标角度，再判断开火 |
| 新版 | src/standard_mpc.cpp | Planner | Gimbal::send(angle, velocity, acceleration) | 先生成未来参考轨迹，再用 TinyMPC 平滑并约束角加速度 |

新版不是把 Aimer 改名为 Planner，而是改变了控制接口：视觉线程生成 Target，规划线程独立运行，把角度、角速度、角加速度一起发送给云台。

## 2. 新版完整数据流

~~~text
Gimbal 串口接收线程
  ├─ 当前模式
  ├─ 云台角度/角速度
  ├─ IMU 四元数
  └─ 弹速

相机取图像和时间戳 t
        ↓
Gimbal::q(t) 得到图像时刻对应的姿态
        ↓
Solver 设置云台到世界坐标旋转
        ↓
YOLO 检测装甲板
        ↓
Tracker 更新 Target 和 EKF
        ↓
只保留最新 Target，写入容量为 1 的队列
        ↓
Planner 线程取出 Target
        ↓
决策延迟补偿 + 子弹飞行时间补偿
        ↓
生成 yaw/pitch 参考轨迹
        ↓
yaw TinyMPC + pitch TinyMPC
        ↓
取预测窗口中点的角度、角速度、角加速度
        ↓
判断 fire
        ↓
Gimbal::send() 发送给下位机
~~~

主要源码：

- 主循环：[src/standard_mpc.cpp](src/standard_mpc.cpp)
- Planner：[tasks/auto_aim/planner/planner.cpp](tasks/auto_aim/planner/planner.cpp)
- 目标 EKF：[tasks/auto_aim/target.cpp](tasks/auto_aim/target.cpp)
- Tracker：[tasks/auto_aim/tracker.cpp](tasks/auto_aim/tracker.cpp)
- 新版云台协议：[io/gimbal/gimbal.cpp](io/gimbal/gimbal.cpp)
- TinyMPC：[tasks/auto_aim/planner/tinympc/admm.cpp](tasks/auto_aim/planner/tinympc/admm.cpp)

## 3. standard_mpc 的线程结构

### 3.1 视觉线程

主线程循环执行：

1. 读取 gimbal.mode；
2. 读取相机图像 img 和时间戳 t；
3. 调用 gimbal.q(t) 获得时间对齐的四元数；
4. 运行 YOLO；
5. 运行 tracker.track(armors, t)；
6. 将最新 Target 写入 target_queue；
7. 小符或大符模式进入能量机关分支。

### 3.2 Planner 线程

线程间队列定义为：

~~~cpp
tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
~~~

它表示：

- 元素是 `std::optional<Target>`，允许有目标和无目标；
- 容量为 1；
- 队列满时先丢弃旧元素，再放入新元素。

因此它不是逐帧排队，而是只保留最新目标。Planner 线程大约每 10 ms 运行一次：

~~~cpp
auto target = target_queue.front();
auto gs = gimbal.state();
auto plan = planner.plan(target, gs.bullet_speed);
gimbal.send(...);
sleep_for(10ms);
~~~

front() 返回当前目标副本但不弹出。视觉线程发布新目标后，旧目标会被替换。因此 Planner 可能重复规划同一个 Target，直到视觉线程产生新结果。

## 4. Target 的 EKF 状态

Target 内部状态顺序是：

~~~text
x vx y vy z vz a w r l h
~~~

数学向量为：

$$
\mathbf{x}=[x,v_x,y,v_y,z,v_z,a,w,r,l,h]^T
$$

| 变量 | 含义 | 单位 |
|---|---|---|
| x | 机器人旋转中心世界坐标 X | m |
| vx | X 方向速度 | m/s |
| y | 机器人旋转中心世界坐标 Y | m |
| vy | Y 方向速度 | m/s |
| z | 旋转中心高度 | m |
| vz | Z 方向速度 | m/s |
| a | 目标自身方位角 | rad |
| w | 目标自旋角速度 | rad/s |
| r | 普通装甲板到旋转中心的半径 | m |
| l | 长短轴半径差 | m |
| h | 上下层装甲板高度差 | m |

Target::armor_xyza_list() 根据状态生成所有装甲板的 x、y、z 和 yaw。第 i 个装甲板角度为：

$$
\alpha_i=a+i\frac{2\pi}{N_a}
$$

其中 Na 是装甲板数量。四装甲板目标的长装甲板使用 r+l 和 z+h，普通装甲板使用 r 和 z。

## 5. Planner 两个入口

### 5.1 optional Target 入口

standard_mpc 实际调用：

~~~cpp
planner.plan(target, gs.bullet_speed);
~~~

如果 target 为空，直接返回 Plan{false}。有目标时，根据目标角速度选择延迟：

$$
t_{delay}=
\begin{cases}
t_{high},&|w|>w_{decision}\\
t_{low},&|w|\le w_{decision}
\end{cases}
$$

其中 w 是 target.ekf_x()[7]，w_decision 是 decision_speed，t_high 和 t_low 是两个延迟配置。然后 Planner 将目标复制，并预测到 now + t_delay。

### 5.2 Target 入口

这个重载完成实际计算：

1. 检查弹速；
2. 选择水平距离最近的预测装甲板；
3. 计算弹道飞行时间；
4. 把目标预测到子弹到达时刻；
5. 生成 100 点参考轨迹；
6. 分别求解 yaw 和 pitch；
7. 取轨迹中点输出；
8. 用中点后约 20 ms 的误差判断开火。

## 6. 子弹速度与弹道

Planner 只接受 10 至 25 m/s 的弹速。超出范围时使用 22 m/s：

~~~cpp
if (bullet_speed < 10 || bullet_speed > 25)
  bullet_speed = 22;
~~~

对于每个预测装甲板，计算水平距离：

$$
d_i=\sqrt{x_i^2+y_i^2}
$$

取最小 d_i 的装甲板用于弹道计算。这里是几何距离选择，不是 Tracker 的目标优先级选择。

tools::Trajectory 忽略空气阻力，只考虑重力：

$$
h=d\tan\theta-\frac{gd^2}{2v_0^2\cos^2\theta}
$$

- d：水平距离；
- h：目标高度；
- v0：子弹初速度；
- θ：抬头角；
- g：代码中的 9.7833 m/s²。

代码计算低抛和高抛解，选择飞行时间更短的解。如果判别式小于 0，则弹道无实数解，Planner 返回 control=false。

## 7. aim：空间位置到云台角度

Planner::aim() 不是 MPC，只把一个预测装甲板转成理想角度。

### 7.1 yaw

$$
\psi=wrap(\operatorname{atan2}(y,x)+\psi_{offset})
$$

- atan2(y,x)：世界水平面方位角；
- yaw_offset：机械安装误差，配置为度，构造时转换为弧度；
- wrap：把角度限制到约 [-pi, pi]。

### 7.2 pitch

$$
\phi=-\theta_{ballistic}-\phi_{offset}
$$

负号来自项目坐标系约定，pitch_offset 也由配置中的度转换为弧度。

## 8. 生成 1 秒参考轨迹

Planner 常量：

~~~cpp
DT = 0.01
HALF_HORIZON = 50
HORIZON = 100
~~~

因此预测窗口约为 1 秒。参考矩阵为 4×100，每列是：

~~~text
[相对 yaw, yaw 速度, pitch, pitch 速度]
~~~

为了使用中心差分，get_trajectory() 先把目标临时预测到当前参考时刻前约 0.5 秒，再逐步向未来推进：

$$
\dot\psi_i\approx\frac{\psi_{i+1}-\psi_{i-1}}{2\Delta t}
$$

$$
\dot\phi_i\approx\frac{\phi_{i+1}-\phi_{i-1}}{2\Delta t}
$$

因此第 50 列对应当前参考时刻附近，而不是窗口起点。

Planner 先保存当前理想 yaw0，轨迹中的 yaw 使用相对量：

$$
\psi_{rel,i}=wrap(\psi_i-\psi_0)
$$

这样 yaw MPC 在较小相对角度附近优化，最后输出时再把 yaw0 加回去。

## 9. 两个 TinyMPC 问题

### 9.1 单轴模型

yaw 和 pitch 使用两个独立求解器。每个求解器的状态是角度和角速度，控制量是角加速度：

$$
x_k=\begin{bmatrix}\theta_k\\\dot\theta_k\end{bmatrix},\qquad
u_k=\ddot\theta_k
$$

离散模型：

$$
x_{k+1}=Ax_k+Bu_k
$$

$$
A=\begin{bmatrix}1&\Delta t\\0&1\end{bmatrix},\qquad
B=\begin{bmatrix}0\\\Delta t\end{bmatrix}
$$

源码中的 B 第一项是 0，控制量在一个步长内直接影响角速度，角度通过后续速度积分变化。它不是完整连续匀加速离散化中的 0.5*dt² 模型。

### 9.2 目标函数

可以把 TinyMPC 的目标理解为：

$$
J=\sum_{k=0}^{N-1}(x_k-x_{ref,k})^TQ(x_k-x_{ref,k})
+\sum_{k=0}^{N-2}u_k^TRu_k
+(x_{N-1}-x_{ref,N-1})^TP_f(x_{N-1}-x_{ref,N-1})
$$

- N：预测节点数；
- x_k：规划状态；
- x_ref,k：参考状态；
- u_k：角加速度；
- Q：状态误差权重；
- R：控制量权重；
- P_f：终端权重。

配置中的 Q_yaw、Q_pitch 会构造成角度和角速度的对角权重。当前示例将速度误差权重设为 0：

~~~yaml
Q_yaw: [9e6, 0]
R_yaw: [1]
Q_pitch: [9e6, 0]
R_pitch: [1]
~~~

### 9.3 加速度约束

$$
-u_{max}\le u_k\le u_{max}
$$

yaw 使用 max_yaw_acc，pitch 使用 max_pitch_acc。它们是规划器约束，不是自动测量得到的云台物理极限。

### 9.4 ADMM 求解

TinyMPC 使用 ADMM（交替方向乘子法）求解带约束的有限时域二次优化问题。每次迭代的大致过程是：

~~~text
Riccati backward pass
  → LQR forward rollout
  → 投影到状态/输入约束
  → 更新对偶变量
  → 更新线性代价项
  → 检查原始残差和对偶残差
~~~

Planner 的实际调用顺序：

~~~cpp
tiny_set_x0(solver, x0);
solver->work->Xref = reference;
tiny_solve(solver);
~~~

yaw 和 pitch 各求解一次。Planner 把两个求解器的最大迭代次数设置为 10；状态和输入 box constraint 开启，cone/linear constraint 未启用，自适应 rho 默认关闭。这样优先保证实时预算，不代表每次都完全收敛。

## 10. Planner 的初始状态与反馈边界

这是阅读源码时最容易误判的地方。MPC 初始状态来自参考轨迹第一列：

~~~cpp
x0 << traj(0, 0), traj(1, 0);
~~~

它没有把 gimbal.state().yaw 或 gimbal.state().pitch 作为 TinyMPC 的 x0。standard_mpc 里 gimbal.state() 主要只向 Planner 提供 bullet_speed，真实云台闭环执行在下位机。

当前架构更准确地说是：

~~~text
上位机 Planner：目标预测、参考轨迹生成、前馈控制量整形
下位机控制器：根据角度/速度/加速度命令进行真实闭环控制
~~~

若要把真实云台反馈纳入 MPC，必须重新定义状态时间戳、角度参考坐标系、初始速度、通信延迟和输出时刻，不能只把 gimbal.state() 简单传入。

## 11. Plan 输出字段

| 字段 | 含义 | 来源 |
|---|---|---|
| control | 是否允许视觉控制 | 有目标且规划成功时为 true |
| fire | 是否允许开火 | 参考与规划轨迹误差足够小时为 true |
| target_yaw | 参考轨迹中点 yaw | traj(0, 50) + yaw0 |
| target_pitch | 参考轨迹中点 pitch | traj(2, 50) |
| yaw | MPC 输出 yaw | yaw 求解器状态中点 + yaw0 |
| yaw_vel | MPC 输出 yaw 速度 | yaw 状态中点 |
| yaw_acc | MPC 输出 yaw 加速度 | yaw 输入中点 |
| pitch | MPC 输出 pitch | pitch 状态中点 |
| pitch_vel | MPC 输出 pitch 速度 | pitch 状态中点 |
| pitch_acc | MPC 输出 pitch 加速度 | pitch 输入中点 |

## 12. 开火判断

检查点为：

~~~text
HALF_HORIZON + shoot_offset = 50 + 2 = 52
~~~

因为 DT=0.01 s，检查点相对中点约为 20 ms。误差：

$$
e_{fire}=\sqrt{(\psi_{ref}-\psi_{plan})^2+(\phi_{ref}-\phi_{plan})^2}
$$

当 e_fire < fire_thresh 时置 fire=true。fire_thresh 的单位是弧度；0.003 rad 约为 0.172 度，0.0035 rad 约为 0.201 度。

这个判断检查的是参考轨迹与 MPC 轨迹之间的误差，不是实际云台反馈误差。真实云台是否跟上取决于下位机控制器和通信延迟。

## 13. Planner 输出与云台接口

Planner 输出进入：

~~~cpp
gimbal.send(
  plan.control, plan.fire,
  plan.yaw, plan.yaw_vel, plan.yaw_acc,
  plan.pitch, plan.pitch_vel, plan.pitch_acc);
~~~

### 13.1 原始 Gimbal/SP/CRC 模式

当配置未启用当前的 CBoard UART 适配（通常为 `use_can: true` 或未填写）时，`Gimbal::send()` 将上述字段写入 `VisionToGimbal`：

~~~text
head: SP
mode: 0 不控制，1 控制但不开火，2 控制且开火
yaw, yaw_vel, yaw_acc
pitch, pitch_vel, pitch_acc
crc16
~~~

下位机同时通过 `GimbalToVision` 返回四元数、模式、实际云台角/角速度、弹速和弹丸计数。相机图像的时间戳可以在相邻姿态之间插值，因此这是 Planner 原本设计的完整接口。

### 13.2 当前 CBoard/UART 兼容模式

当 YAML 设置 `use_can: false` 时，当前工作树中的 `Gimbal` 不发送 `SP/CRC` 帧，而是内部构造 `CBoard`，由 `SocketCAN` 的 UART 分支接收姿态并发送 `Command`。该链路使用 `serial_imu` 配置的串口和协议：

```text
接收：27 字节欧拉角帧 -> 四元数 -> CBoard 姿态队列
发送：13 字节 FF + yaw(float32) + pitch(float32) + distance(float32)
```

这条适配链路的限制必须明确：

- `Gimbal::q(t)` 仍能为视觉帧提供插值后的四元数；
- `Gimbal::state().bullet_speed` 使用 `uart_bullet_speed` 的固定值，不再来自下位机实时测速；
- `Gimbal::send()` 将 Planner 的输出压缩成 `Command{control, fire, yaw, pitch, 0}`，因此 yaw/pitch 速度和加速度不会进入 13 字节 UART 帧；
- 13 字节协议没有独立的 `control` 字段，`fire` 也无法按 SP 模式编码，下位机必须自行定义目标失效和开火语义；
- `com_port` 在这条模式下由 `serial_imu.port` 决定（若未配置则使用实现中的默认串口），波特率和角度单位见 `configs/standard3.yaml` 与 `SERIAL_IMU_GUIDE.md`。

因此，想验证完整的 TinyMPC 角速度/角加速度接口，应使用原始 SP/CRC 下位机；使用当前 27/13 字节 UART 时，只能验证视觉、预测、Planner 计算和最终 yaw/pitch 数值是否正确，不能据此证明下位机执行了 Planner 的速度/加速度前馈。不要把旧版 `CBoard::send(Command)` 的 CAN 命令格式、当前 CBoard/UART 13 字节格式和原始 `VisionToGimbal` 29 字节格式混用。

## 14. 配置项

| 字段 | 单位 | 作用 |
|---|---|---|
| yaw_offset | 配置为度，内部转弧度 | yaw 安装偏置 |
| pitch_offset | 配置为度，内部转弧度 | pitch 安装偏置 |
| decision_speed | rad/s | 高低速延迟分支 |
| high_speed_delay_time | s | 高速目标决策延迟 |
| low_speed_delay_time | s | 普通目标决策延迟 |
| max_yaw_acc | rad/s² | yaw 输入约束 |
| max_pitch_acc | rad/s² | pitch 输入约束 |
| Q_yaw、R_yaw | 权重 | yaw 跟踪与控制平滑 |
| Q_pitch、R_pitch | 权重 | pitch 跟踪与控制平滑 |
| fire_thresh | rad | 开火误差阈值 |

建议调参顺序：

1. 核对度与弧度单位；
2. 测量真实云台角加速度，再设置 max_*_acc；
3. 固定 R，逐步调 Q；
4. 调整高低速延迟；
5. 最后用命中数据调整 fire_thresh。

## 15. 测试顺序

### 15.1 离线 Planner

~~~bash
./build/planner_test_offline \
  --d=3.0 \
  --w=5.0 \
  configs/standard3.yaml
~~~

该程序构造 Target(d, w, 0.2, 0.1)，每 10 ms 调用 Planner，并通过 Plotter 输出 target_yaw、target_pitch、plan_yaw、plan_pitch、速度和加速度。

建议扫描距离 1、2、3、5 m，以及角速度 0、2、5、8、12 rad/s，观察轨迹连续性、约束触碰和 fire 状态。

### 15.2 云台接口测试

~~~bash
./build/planner_test configs/standard3.yaml
~~~

先使用架空或无负载云台，不接真实弹丸，核对角度方向、速度方向、加速度方向和 fire 信号。

### 15.3 真实视觉链路

~~~bash
./build/standard_mpc configs/standard3.yaml
~~~

真实链路为：

~~~text
相机 → YOLO → Solver → Tracker/EKF
     → target_queue → Planner → Gimbal
~~~

### 15.4 调试可视化

~~~bash
./build/auto_aim_debug_mpc configs/standard3.yaml
~~~

该程序会通过 Plotter 发送 target_yaw、plan_yaw、速度、加速度、fire 等 JSON 数据，并在图像上绘制预测装甲板和 Planner 选中的装甲板。

## 16. 推荐观察的曲线

~~~text
gimbal_yaw / gimbal_pitch
target_yaw / target_pitch
plan_yaw / plan_pitch
plan_yaw_vel / plan_pitch_vel
plan_yaw_acc / plan_pitch_acc
fire / fired
w
bullet_speed
~~~

| 现象 | 优先检查 |
|---|---|
| target 正常但 plan 滞后 | max_acc、Q、TinyMPC 迭代次数 |
| plan 抖动 | R、EKF 角速度、中心差分 |
| pitch 整体偏移 | 弹道、pitch_offset、外参 |
| fire 长期为假 | fire_thresh、弹道可解性、轨迹误差 |
| 角度正常但云台动作慢 | 下位机控制器和通信频率 |
| Planner 使用旧目标 | 容量为 1 的覆盖队列和目标时间戳 |
| 丢目标后仍控制 | optional<Target> 空值是否正确传递 |

## 17. 当前实现限制

1. Planner 没有显式析构 TinySolver 指针，TinyMPC 内存生命周期需要继续检查。
2. tiny_solve() 返回值没有在 Planner 中检查，未收敛时仍会读取工作区结果。
3. MPC 初始状态来自参考轨迹，不是真实云台角度。
4. yaw 和 pitch 独立求解，没有二维耦合约束。
5. Planner 只约束角加速度，没有角度和角速度上下界。
6. fire_thresh 检查的是参考轨迹与规划轨迹，不是实测云台误差。
7. Target 在 Planner 中被复制后多次 predict，必须区分原始时间和临时预测时间。
8. 最近装甲板选择使用水平距离，不使用 Tracker 优先级。
9. 弹道模型忽略空气阻力。
10. 视觉线程和 Planner 线程频率不同，目标会被覆盖而不是排队。

## 18. 修改 Planner 的方法

### 修改参考轨迹

只改 get_trajectory()，先保持 TinyMPC 模型不变。检查轨迹中心时刻、yaw 跨 pi、中心差分速度和飞行时间预测是否正确。

### 修改 MPC 模型

修改 A/B 矩阵时同步检查状态单位、x0 物理含义、Xref 行顺序、中点索引、输入约束和下位机对速度/加速度的解释。

### 修改开火判断

不要只调大 fire_thresh。应同时记录参考与规划误差、真实云台与 plan 误差、通信延迟、弹道飞行时间误差和实际命中窗口。

### 修改线程结构

若要使用真实云台反馈，必须明确反馈时间戳、目标预测时间、发送时间以及绝对角/相对角定义，不能只把 gimbal.state() 传入 Planner。

## 19. 推荐源码阅读顺序

~~~text
src/standard_mpc.cpp
  → io/gimbal/gimbal.hpp/.cpp
  → tasks/auto_aim/tracker.cpp
  → tasks/auto_aim/target.cpp
  → tasks/auto_aim/planner/planner.hpp/.cpp
  → tools/trajectory.cpp
  → tasks/auto_aim/planner/tinympc/tiny_api.cpp
  → tasks/auto_aim/planner/tinympc/admm.cpp
~~~

读完这条链路后，应该能回答：

- Target 从哪里来；
- EKF 的每个状态量是什么；
- 目标被预测了几次、分别预测到什么时刻；
- 参考轨迹中点对应哪个时刻；
- yaw/pitch 为什么使用两个求解器；
- Planner 输出怎样到达下位机；
- fire 判断的是哪个误差。
