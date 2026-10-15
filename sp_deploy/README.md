# 实机标定与一键部署

新增工具只读复用原 SP 的 YOLO、Solver、Tracker/EKF、Planner/TinyMPC 和相机/Gimbal 驱动，不改写 `sp_vision_25`。实机运行器在构建目录生成 IO 副本，仅增加线程安全的最新接收姿态读取接口，历史队列仍供图像时间戳配对使用；原源文件不变。当前已适配该工作树的海康/迈德威视相机与 UART44 电控；USB 通用相机和 CAN 不是当前运行器的接口。相机 SDK 的完整驱动、设备权限及真实采集仍须在接入相应硬件后验证。

本机只编辑和调度。构建、运行与实机采集应在设备所在电脑进行；借用目标机为 `wuxian@192.168.1.102`。目标机已经配置 Ubuntu 24.04 / OpenVINO 2024.6.0，实机运行器不需要 ROS。

## 从采集到部署

先复制并填写 `profiles/hikrobot-uart44.example.json`：串口使用实际 by-id 路径，确认相机 VID:PID、曝光、敌方颜色、实测弹速、IMU 坐标映射、标定板规格及枪口相对标定参考原点的位移 `muzzle_offset_gimbal_m`。该位移示例留空，不能把未测量的值自动当作零。示例使用 10×7 **圆心**、40 mm 间距的对称圆点板；棋盘格填写**内角点**数量，单位为米。

在目标机工作区执行：

```bash
# 环境检查；--hardware 还检查串口权限及唯一匹配的 USB 相机
python3 sp_deploy/deploy.py doctor --profile /path/to/robot.json --hardware
# 若系统开发依赖缺失，显式安装（sudo 在终端询问，不保存密码）
python3 sp_deploy/deploy.py deps
python3 sp_deploy/deploy.py build

# 两套独立数据：内参移动标定板，手眼固定标定板并改变云台姿态
python3 sp_deploy/deploy.py capture --profile /path/to/robot.json \
  --output /path/to/intrinsics-data --seconds 90 --period-ms 1000
python3 sp_deploy/deploy.py capture --profile /path/to/robot.json \
  --output /path/to/handeye-data --seconds 90 --period-ms 1000 --imu

# 一键：标定 -> 质量检查 -> 构建 -> 打包 -> 推理自检 -> 原子切换 current
# 只有确实共用固定旋转中心时才传 --fixed-center；否则提供每帧测量平移
python3 sp_deploy/deploy.py deploy --profile /path/to/robot.json \
  --intrinsics-data /path/to/intrinsics-data --handeye-data /path/to/handeye-data \
  --fixed-center --prefix /path/to/sp-runtime \
  --qualification sp_vision_ros/artifacts/accuracy_qualification.json

# 默认只观察；跟踪和开火由实际操作者明确选定
python3 sp_deploy/deploy.py launch --prefix /path/to/sp-runtime
python3 sp_deploy/deploy.py launch --prefix /path/to/sp-runtime --control
python3 sp_deploy/deploy.py launch --prefix /path/to/sp-runtime --control --fire

# 回滚到上一次成功切换的版本
python3 sp_deploy/deploy.py rollback --prefix /path/to/sp-runtime \
  --qualification sp_vision_ros/artifacts/accuracy_qualification.json
```

`deploy` 不启动机器人，不自动改变系统服务或电控固件。实际开火测试按 [实机实测文档](docs/实机实测流程.md)逐级进行。原 `standard_mpc` 直接使用 Planner 的 fire，不能靠 `auto_fire: false` 保证该程序不开火；本工具运行器以命令行权限决定输出控制/开火，并增加 250 ms 视觉状态期限、200 ms 最新 IMU 接收期限及基于独立最新姿态的反馈误差检查。

远程构建：`python3 tools/remote_run.py run -- python3 sp_deploy/deploy.py build`。单作业采集、部署或启动时同样通过 `run -- python3 sp_deploy/deploy.py ...` 逐个传参；这些命令只应在相机和电控实际连接的主机使用。长期实机启动会占用该调度工作区锁。

## 标定质量与坐标约定

内参至少 20 张成功识别图像，每 5 张保留 1 张进行验证；目标中心覆盖至少 25% 画面宽度和 20% 高度，板面有多种倾斜。训练 RMS ≤0.5 px、留出图像 RMS 的 p95 ≤0.8 px，板面法线变化须足够。

手眼至少 12 个成功识别且与姿态配对的视图，必须同时有 yaw 和 pitch 等不平行旋转。单轴旋转或重复姿态会被拒绝。软件使用 `A_i X B_i = T_board_world`：A 是云台→世界，B 是板→相机，输出 X 是**相机→云台**；平移始终为米。原始姿态保存为 wxyz，按与原 Solver 相同的 `Sᵀ R_imu S` 转换。

`--fixed-center` 代表云台坐标原点的世界平移固定为零。只有 yaw/pitch 轴确实相交或已定义为同一固定参考点时才适用。轴不共点、机器人底盘运动时，需要在 `dataset.json` 每帧提供 `gimbal_translation_m`，并省略该选项；只靠 IMU 朝向不能自动知道这些平移。

固定板恢复到世界后的平移残差 p95 ≤10 mm、旋转残差 p95 ≤1°，每帧 PnP RMS ≤1 px。错误四元数、非旋转矩阵、混合分辨率、板移动和退化运动不会生成可上线结果。对称图案可能出现 180°角点重排；保持板方向连续，核对左上角标记和世界残差，不能把低 PnP 误差直接当成手眼正确。

手眼报告的平移相对于所声明的云台参考原点。Planner 弹道计算需要相对于实际发射原点，因此运行配置使用 `t_camera2aim = t_camera2gimbal − muzzle_offset_gimbal_m`，原始标定报告保留不变；若标定参考原点已经定义在枪口，应明确填写 [0,0,0]。这不会自动测出枪管方向与编码器零位的安装偏差，还需独立轴向/静态瞄准检查。

这些阈值是当前工程验收门槛，不是 OpenCV 对所有镜头的通用保证。数学方向与姿态多样性依据 [OpenCV 手眼标定官方文档](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)。

## 数据与版本目录

采集目录包含 PNG 和 `dataset.json`，后者记录图像尺寸、图像 steady_clock 时间戳、wxyz 姿态和 `R_gimbal2imubody`。姿态匹配复用原 `Gimbal::q(image_timestamp)`；海康原驱动时间戳是获取图像缓冲区后的主机时间，不能称为硬件曝光时间。手眼每个姿态应停稳采样，以降低此差异的影响。

运行目录结构为 `releases/<id>/`、`current`、`previous`、`runtime/`、`logs/`。候选包包含原模型 XML/对应 BIN、相机 SDK 库、OpenVINO/TBB、校准报告、配置和可执行文件；外部系统 OpenCV/USB 等 ABI 依赖仍需要兼容的 Ubuntu 环境。包文件以 SHA-256 校验，任何校验失败都阻止切换/启动。

`prepare` 可仅构建候选包用于检查；`activate` 拒绝合成标定，并要求所选参数基础的 static/spin/move 每场景至少 100 发、保守命中率 ≥90%、Wilson 95% 下限 ≥85% 的完整仿真资格报告。合成测试不会生成 `current`。切换版本不强行结束已运行的进程，实际运行切换应先停止旧进程及其他控制同一电控端口的程序，再启动新版本。

当前工具测试证据与尚未进行的硬件项目见 [验证记录](docs/部署工具验证.md)。
