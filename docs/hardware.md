# 硬件、坐标与串口契约

本项目面向一套 Dummy V2 六轴机械臂、独立舵机夹爪和 RGB-D 相机。
代码保存了该装配的控制约定；它们不是适用于任意同类机械臂的出厂参数。
安装与入口见 [bringup.md](bringup.md)，实机操作前阅读 [safety.md](safety.md)。

维护者仅声明转接件为本人原创，未声明机械臂本体和其他硬件模型为原创。转接件的具体文件与硬件许可证仍待确认；本轮所有模型均保留在本地、不随公共软件候选发布。

## 已有硬件接口与复现缺口

| 部件 | 当前软件支持 | 复现前仍需确认 |
| --- | --- | --- |
| 六轴机械臂 | `arm_driver` 通过 USB 串口读取六轴位置、发送位置命令、STOP 和 DISABLE | 实际控制板型号、固件版本与来源、各轴电机/减速比、接线、供电、机械限位和回零方法 |
| 夹爪 | `gripper_driver` 对接返回 `ESP32_PCA9685_GRIPPER_V1` 标识的控制器，默认 PCA9685 通道 0 | ESP32 固件及刷写方式、舵机型号/电源/PWM 频率、安装方向、正常工作行程、负载与夹持力 |
| 外置相机 | Astra Pro，ROS 包 `astra_camera` | 相机型号、驱动来源与版本、USB 连接和相机到基座的外参 |
| 腕部相机 | Intel RealSense D405，ROS 包 `realsense2_camera` | 实际设备序列号、驱动版本、支架安装、手眼外参及独立精度检验 |
| 机械描述 | Dummy V2 视频款 URDF 和派生网格；另保留 V1 参考描述 | CAD 再分发授权、原始模型获取渠道、实物与 CAD 的一致性、碰撞和惯量验证 |
| 停机保障 | 软件提供串口 STOP / DISABLE | 独立硬件急停、失联/掉电行为、承载姿态及防坠措施；仓库未提供完整电气安全设计 |
| 移动底盘 | 当前仓库无底盘驱动或反馈接口 | 型号、轮型、轮距、轮径、编码器、MCU、供电、通信协议与承载实测均 TBD；见 [底盘现状](mobile_base.md) |

ROS 驱动不是控制器固件替代品。当前发布材料不足以证明一块空白控制板可以仅靠本仓库完成刷写并复现。
不要从协议中的设备名推断未记录的接线、电压、电流额定值或固件兼容性。
夹爪握手只能支持 ESP32/PCA9685 协议判断，**ESP32-S3 具体芯片型号尚无本仓库固件/BOM 证据，记为 TBD**。
许可未确认的第三方 CAD、STL 和固件不随公开源码导出；本地保留的几何文件也不等于已获再分发授权。

## 机械臂 ROS 接口

以下名称以无额外 namespace/remapping 的默认运行方式为准。驱动同时使用相对和绝对 topic，
多机械臂或重命名部署需要逐项检查，不能只加一个 namespace 就假定整条链路已隔离。

| 方向 | 接口 | 类型 | 含义 |
| --- | --- | --- | --- |
| 发布 | `/joint_states` | `sensor_msgs/msg/JointState` | 经方向/零位映射后的六轴位置，单位 rad |
| 发布 | `/arm/raw_joint_states` | `sensor_msgs/msg/JointState` | 原始控制器角度转换成 rad；仍是原始坐标，不能替代模型关节位置 |
| 订阅 | `/arm/joint_command` | `sensor_msgs/msg/JointState` | 六轴模型坐标目标，位置单位 rad，必须有非零时间戳和唯一的 `joint1` 至 `joint6` |
| 订阅 | `/arm/live_joint_command` | `sensor_msgs/msg/JointState` | LIVE FOLLOW 的最新完整六轴目标；仅在显式启用后有效 |
| 订阅 | `/arm/live_follow_heartbeat` | `std_msgs/msg/Header` | LIVE FOLLOW 独立心跳；不创建运动目标 |
| 发布 | `/arm/live_follow_state` | `std_msgs/msg/Bool` | LIVE FOLLOW 的软件启用状态；可靠、Transient Local、深度 1 |
| 服务 | `/arm/stop` | `std_srvs/srv/Trigger` | 请求 STOP 并等待控制器确认 |
| 服务 | `/arm/disable` | `std_srvs/srv/Trigger` | 请求 DISABLE 并等待控制器确认 |
| 服务 | `/arm/live_follow` | `std_srvs/srv/SetBool` | 显式进入或退出 LIVE FOLLOW |

`arm_console` 只通过 ROS 通信，不直接打开串口。`/arm/preview_joint_states` 是独立的目标预览，
对应 `preview/` TF 树；它不能作为实际机器人反馈。驱动反馈不含速度、力矩或运动完成结果。
仓库没有为这一串口驱动提供 `FollowJointTrajectory` 硬件执行 action。

## 控制器角度与模型角度

映射源文件是 [`joint_mapping.py`](../src/arm_driver/arm_driver/joint_mapping.py)。
以下值是当前实现的原样记录，本次开源整理未更改：

| 轴 | raw 参考角 ° | ideal 参考角 ° | 方向系数 | 比例 | 零位补偿 ° |
| --- | ---: | ---: | ---: | ---: | ---: |
| J1 | 0 | 0 | -1 | 1 | 0 |
| J2 | -75 | 75 | -1 | 1 | 0 |
| J3 | 180 | -90 | -1 | 1 | 0 |
| J4 | 0 | 0 | 1 | 1 | 0 |
| J5 | 0 | 0 | 1 | 1 | 0 |
| J6 | 0 | 0 | 1 | 1 | 0 |

公式为 `ideal = ideal_reference + direction × (raw − raw_reference) × scale − zero_offset`。
因此当前 J1/J2 为 `−raw`，J3 为 `90° − raw`，J4/J5/J6 为 `raw`。
ROS 边界使用 rad，串口协议使用 °；GUI 输入显示 °。参考姿态不等于所有关节为 0，
也没有被定义为自动回零动作。

驱动和 Console 的 J1–J5 默认范围均为 `[-3.14, 3.14] rad`。
这些值与 URDF 的可视化限位保持一致，但尚不能作为实机机械限位认证。
J6 在 V2 URDF 中是 `continuous`；驱动接受任意有限 J6 位置。
连续关节的数学表示不保证线缆、夹爪和腕部相机能无限旋转。

**当前 GUI 限制：** Console 的六个编辑控件仍统一采用配置里的显示范围，默认包括 J6 的
`±3.14 rad`。多圈 J6 反馈在初始化或复制 Target 时可能被控件裁到该范围。
必须比较 Current 与 Target；不能把 GUI 的 `TARGET = CURRENT` 当作多圈 J6 的无误差复制。
此问题保留为独立硬件相关改进，未通过扩大范围来绕过。

## 六轴模型与工具坐标

V2 主链为：

```text
base_link
  └─ joint1 → link1_1_1
      └─ joint2 → link2_1_1
          └─ joint3 → link3_1_1
              └─ joint4 → link4_1_1
                  └─ joint5 → link5_1_1
                      └─ joint6 → link6_1_1
                          └─ tool0_fixed_joint → tool0
                              └─ tcp_extension_joint → tcp
```

关节轴依次为 `−Z, −X, −X, −Y, −X, +Y`，均相对于各自关节坐标。
`tool0` 相对末端 link 固定偏移为 `(0, −0.017, 0) m`。
`tcp_extension_joint` 是被动棱柱关节，沿 `tool0 −Y`，固定起点 100 mm，附加行程 0–31.5 mm。
它应参与 TF/FK，不能混入六轴电机命令或六轴 IK 的主动关节数组。
单独启动 `robot_state_publisher` 不会生成它的状态。

URDF 另有 `base_link → tag0_ref` 的现场固定参考坐标。
其位置属于原装配的标定布置，新装配必须独立核实；不能把该 frame 当作通用 AprilTag 原点。
模型没有可靠的惯量数据，采用详细网格碰撞体；它不是已验收的动力学仿真模型。
`effort=150`、`velocity=3.15` 等 URDF 字段也不构成额定力矩/速度实测。

## 机械臂串口协议

默认 `115200 baud, 8N1`。驱动在 Linux 上使用独占打开，优先解析 `/dev/serial/by-id/` 稳定别名。
不同设备须显式配置自己的端口，不要依赖 USB 插入顺序。

| 请求（以换行结束） | 当前解析的响应 | 软件用途 |
| --- | --- | --- |
| `#GETJPOS` | `ok a1 a2 a3 a4 a5 a6` | 六个 raw 角度，单位 ° |
| `!START` | `Started ok` | 首次正常 MOVE 前启动控制器；必须确认后才发送目标 |
| `&a1,a2,a3,a4,a5,a6,speed` | FIFO 空闲槽数 0–16 | 接收目标；255 是失败值，其他无效值也拒绝 |
| `!STOP` | `Stopped ok` | 停止命令/清理运动队列的协议屏障 |
| `!DISABLE` | `Disabled ok` | 卸力请求；可能使机械臂无法主动承载 |
| `#CMDMODE 2` | `Set command mode to [2]`，可带 `ok` 前缀 | LIVE FOLLOW 使用可中断模式 |

驱动处理分片响应、粘连的 `ok` 以及命令响应与位置响应交错。
**FIFO ACK 只表示入队；不表示到位、抓稳或完成整条轨迹。**
GUI 的 “Command published” 更早，只表示 ROS 发布成功。

正常速度参数 `normal_joint_speed=30.0`，LIVE FOLLOW `live_joint_speed=10.0`，
后者上限 20 且不得超过正常速度。协议 speed 是控制器接受的数值；在固件规格和实测缺失时，
不能将它等同于已经校准的末端速度。驱动不在主机端另行生成加速度曲线。
正常命令显式带 speed，避免沿用 LIVE 的低速状态。

## 夹爪协议、脉宽与估计值

夹爪是独立串口所有者，默认 `115200 baud, 8N1`，启动发送 `PING`，
必须收到 `PONG ESP32_PCA9685_GRIPPER_V1` 后才创建命令订阅。
它订阅 `/gripper/pulse_us`（`std_msgs/msg/Int32`），发送
`SET channel pulse_us move_duration_ms`，期待 `OK SET channel pulse_us move_duration_ms`。

| 配置 | 当前默认/边界 | 含义 |
| --- | --- | --- |
| `gripper_channel` | 0 | PCA9685 通道 |
| `open_pulse_us` | 600 μs | 当前 OPEN 预设 |
| `close_pulse_us` | 1300 μs | 当前抓物/CLOSE 预设 |
| 命令允许范围 | 600–1700 μs，含端点 | 软件允许区间；不是要求逐一执行所有脉宽 |
| `move_duration_ms` | 600 ms | 传给控制器的单次动作时间 |

1300 μs 与 1700 μs 不能互换：1300 是当前驱动、CLI 和 Console 的关闭预设；
1700 是软件允许上限，也是独立 CAD 近似模型采用的闭合假设端点。
后者不证明实机可以顶死到零间隙。夹爪无位置/力反馈 topic，也没有真实到位或抓稳反馈。
驱动日志中的 `OK SET` 不应当被当作夹持力证据。

[`gripper_geometry.py`](../src/gripper_driver/gripper_driver/gripper_geometry.py) 保存历史
`pulse → tool0-to-TCP` 表（600→100 mm，1300→126 mm，1700→131.5 mm，分段插值）。
它不直接测量当前脉宽、开口或 TCP；同一装配及同一参考定义需要复核。

`tabletop_grasp` 的 `gripper_motion` 及 CAD 预览是独立离线估算。
原生坐标为 `gripper_cad` / `gripper_cad_estimate`，没有已验证的机器人安装 TF。
运动模型、七组零件和包络均标记 `approximate_unverified`；不能据此改变实机 PWM、
把规划验证字段设为真或把近似包络当作碰撞验收。
原始 CAD 来源授权未定时，公开包可保留估算代码和空白实测模板，但不得捆绑未授权几何。

## 推荐记录的硬件信息

维护自己的本地配置和测量记录：控制板/固件版本、串口别名、机械版本、轴方向及参考姿态、
夹爪通道与已验证脉宽、相机设备选择、各次标定的安装状态和日期。
公开的问题报告应去掉个人路径、设备唯一编号和环境照片中的个人信息。
完整 BOM、接线图、固件获取与刷写说明、硬件急停设计仍是本项目的复现缺口。
