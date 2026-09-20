# 实机安全与验证边界

本仓库控制真实六轴机械臂与夹爪。源码通过构建、单元测试、离线规划或可视化，
都不意味着某台实物的方向、限位、负载、停机或抓取动作已经验收。
先核实 [hardware.md](hardware.md) 中的接口和装配约定，再按 [bringup.md](bringup.md) 分阶段启动。

2026-09-20 审计没有发送任何实体机械臂、夹爪或底盘命令，也没有运行硬件 launch。
所有需执行器动作的验证：`SKIPPED: requires physical robot actuation`。
本轮结果仅覆盖静态检查、离线构建、FakeSerial/PTY、mock GUI 与合成数值计算。

## 移动平台的新增边界

当前仓库没有底盘代码，尚无底盘 watchdog、里程计、停稳互锁或整机急停实现。
将臂安装到底盘后，必须重新核实支撑、重心、倾覆风险、载荷、底盘净空与臂展开工作区。
底盘停止命令不等于已停稳，机械臂 STOP 不会自动停止其它执行器；不能靠同名 `base_link`
或一条未测量的 static TF 构成移动操作集成。实现/验收顺序见 [mobile_base.md](mobile_base.md)。

## 启动默认值需要逐个入口确认

| 入口 | 当前默认 | 实际含义 |
| --- | --- | --- |
| `ros2 run arm_driver arm_driver_node` | `use_mock=true`，`allow_motion_commands=false` | 默认不打开串口，拒绝 START/MOVE/LIVE |
| `arm_console.launch.py` 及 Astra/D405 包装 | 两个 driver 启动开关 false；arm mock true、motion false；gripper hardware false | 即使只开启 start driver，也不会接通实体执行器 |
| `system_bringup.launch.py` | 两个 driver 启动开关 false；`arm_use_mock=true`、`arm_allow_motion_commands=false`、`gripper_allow_hardware_actuation=false` | 真机臂控制需同时显式关闭 mock、打开运动许可；夹爪单独 opt-in |
| `arm_feedback_safe.launch.py` / 旧 `astra_click_grasp` | mock 默认，固定 `arm_allow_motion_commands=false` | 选择 real 会打开串口，仍拒绝 START/MOVE；STOP/DISABLE 仍可能写串口 |
| 裸夹爪驱动 | `allow_hardware_actuation=false` | 不创建串口、不握手、不订阅 PWM；callback/send 双重拒绝，非 mock 成功 |

2026-09-21 第二阶段按明确授权修正默认值。原协议、映射、关节约束和 PWM 未改；
显式 opt-in 的迁移命令见 [bringup.md](bringup.md)。默认安全只约束这些入口自身，
不能关闭同域已授权的旧驱动，也不是安全认证或整机急停。

连接或重连机械臂时，驱动仅建立串口并读取位置，不自动发送 START 或回放旧 MOVE。
这不表示其他已运行的 ROS 节点没有发送命令。GUI 的 PREVIEW ONLY 仅约束 GUI 自己的输出，
不是整个 ROS 网络的运动权限开关。

需要读取实体六轴反馈而不允许该驱动 START/MOVE 时，使用明确参数。
下例只启动机械臂驱动，不启动机器人模型、相机、夹爪、Console 或抓取流程；
须先将 `ARM_PORT` 设为已确认的本机设备路径，且同一控制器只能有一个驱动所有者：

```bash
: "${ARM_PORT:?先将 ARM_PORT 设置为已确认的机械臂串口}"
ros2 launch dual_camera_foundation system_bringup.launch.py \
  start_robot_state:=false \
  start_arm_driver:=true arm_use_mock:=false \
  arm_allow_motion_commands:=false arm_serial_port:="$ARM_PORT" \
  start_gripper_driver:=false start_astra:=false start_d405:=false \
  start_perception:=false start_console:=false
```

`use_mock`、`allow_motion_commands` 和夹爪 `allow_hardware_actuation` 均在初始化时读取；
当前驱动没有将动态参数更新接入这些权限字段。
不要依赖 `ros2 param set` 在线切换它。需要更改权限时，在受控停机后重新启动并核对参数和日志。
这个开关也不是独立硬件急停。

## 运动前的现场检查

1. 机械臂固定牢靠，工作空间内无人、无松动物件，电缆和相机支架不会被关节或夹爪拉扯。
   确认独立停机手段可触及，并明确卸力后如何承载机械臂和负载。
2. 按实物逐轴核对 raw/ideal 方向、参考角和机械可用范围。不要从 URDF 的 `±3.14` 或
   J6 `continuous` 推断实际允许全行程、多圈或无限旋转。
3. 核对串口设备、固件、运行节点和命令所有者；禁止同一设备被两个驱动或直接串口工具同时使用。
   不要把测试节点、rosbag 回放或 mock `/joint_states` 与实机控制放在同一运行域。
4. 核对 Current/Target，特别是多圈 J6。Console 会在参考姿态附近对复制目标去除小幅噪声，
   GUI 控件也有显示范围；`TARGET = CURRENT` 不保证每个目标数值逐位等于测量值。
5. 视觉动作还需确认相机图像/深度新鲜、目标实际尺寸、安装外参、TF 时间和目标位置；
   旧 AprilTag 标定流程还需核对 Tag ID 与标签尺寸。
   RViz 中存在一条 TF 链不等于标定精度达标。
6. 使用现场已验证的有限目标和原有速度/PWM 配置。机械臂避障、夹爪受力与末端负载没有因为
   “规划成功”而自动得到物理保障。

## STOP、DISABLE 与失联

| 事件 | 主机端行为 | 仍不能保证的事情 |
| --- | --- | --- |
| STOP | 作废旧时间戳命令，退出 LIVE，发送 `!STOP` 并等待 `Stopped ok` | 不是安全认证急停；没有独立电气断能回路或停止距离认证 |
| DISABLE | 退出 LIVE；LIVE 时先 STOP，再请求 `!DISABLE`，等待 ACK | 卸力可能下坠或掉落物体；软件不提供支撑或机械抱闸认证 |
| 正常 MOVE 的 ACK 超时 | 关闭不同步的串口，不重放这条 MOVE | 已到达控制器的命令可能仍在执行；断开主机连接不等于实体停止 |
| 位置读取连续失败 | 默认 3 次失败后断开并退避重连；LIVE 在首次反馈失败时就尝试 STOP | 串口失联时 STOP 也可能无法送达；正常 MOVE 不具备 LIVE 的持续心跳保障 |
| LIVE 心跳超时/队列积压 | 撤销运动权限、发送 STOP；重新进入需要显式操作 | 看门狗是主机软件定时器，延迟取决于调度和串口 I/O，并非实时停止上界 |
| 驱动正常退出 | 若认为控制器已 START 或仍处于 LIVE，尽力发送 STOP，再关闭串口 | 进程被强杀、主机崩溃、掉电、USB 断线时不能承诺执行清理代码 |

STOP/DISABLE ACK 表示协议确认；超时意味着控制器状态未知。
`/arm/live_follow_state=false` 也可能伴随串口断开，不能独立作为“物理静止”证据。
正常 MOVE 后另一个更新命令可重新 START；STOP 不是永久的软件锁定。
实体有异常运动时，首先使用现场验证过的独立停机措施，避免反复重启节点或补发同一目标。

## LIVE FOLLOW 的实际约束

Console 默认 PREVIEW ONLY。显式启用 LIVE 需要操作确认、新鲜反馈、唯一命令路由和可用安全服务。
进入时执行 STOP → START → `CMDMODE 2` → 读取新鲜位置，不在启用事务中发送目标。

默认目标发送周期 0.1 s，每轴相对最新反馈最多前进 1°；同一反馈样本不连续推送多步。
目标有效入站年龄 0.25 s，反馈新鲜度 0.35 s，驱动心跳超时 0.30 s，GUI 心跳时限 0.25 s。
驱动要求 FIFO 至少 15 个空位，避免执行积压目标。
一次已接受的目标会在后续新鲜反馈和心跳支持下继续收敛，直到被替换或退出 LIVE。
这些保护不能替代碰撞检测、实机限位或硬件急停。

## 夹爪与视觉验证

当前 OPEN 为 600 μs，CLOSE/抓物预设为 1300 μs，软件允许范围为 600–1700 μs。
CAD 模型在 1700 μs 的零开口是建模假设；不要为了得到零间隙继续加力。
夹爪的 ROS 接口只发布命令，缺少位置/力反馈；机械臂反馈丢失后，GUI 仍允许独立夹爪操作。
夹爪显式 opt-in 后，ARM 的 STOP、motion 权限或 FEEDBACK LOST 都不自动阻止夹爪命令。

未知的 TCP、夹爪开口、CAD 安装关系或相机外参必须保持未验证状态。
`tabletop_grasp` 的独立验证字段不能因为离线单元测试或合成场景通过就改为真。
CAD 静态尺寸、近似运动包络、相机坐标方向箭头、MoveIt 路径、实际抓取成功分别是不同证据。

## 无硬件测试的隔离与停机顺序

纯 Python 几何、配置、串口 fake/PTY 和 CSV 检查不需要连接实体控制器。
涉及 ROS 的 mock、合成场景或预览测试使用专用 `ROS_DOMAIN_ID`，并确认该域没有实体驱动。
`ROS_LOCALHOST_ONLY=1` 可限制发现范围，但不提供对同机同域其他进程的权限隔离。
浏览器夹爪预览只离线计算，不应改接到 `/gripper/pulse_us`。

结束实体会话时，先停止发出目标的程序并退出 LIVE，确认机械臂停止和负载有支撑，
再根据现场要求选择保持或卸力，随后关闭驱动/相机。
关闭 GUI 只管理它拥有的可视化和 launch 子进程；不能假定外部启动的实体驱动也会退出。
未收到停机确认时，不应以终端消失或进程退出替代实物检查。
