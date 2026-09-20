# 分层启动与退出

每个终端先 `source scripts/env.sh`，先完成 [安装](installation.md)。下面实机步骤仅供在现场逐层确认后执行，本次整理没有启动硬件。不要在同一串口/相机/TF 上启动多个 owner。

## 入口选择与终端数量

2026-09-20 审计保留已有 bringup，不为尚不存在的底盘创建空 package 或整机 launch。
纯数值 demo/arm mock 各需一个终端；已有合法模型的 synthetic launch 一个终端即可组合
测试场景、状态、规划和 RViz。真实开发通常按反馈、相机/感知、GUI/规划分成 2–4 个进程组，
实际数量取决于外部驱动和 TF owner，不能把所有入口叠加运行。

以下是全部 28 份 Python launch 的职责清单；本轮只构造 LaunchDescription，不执行 actions。
“无执行”只指该规划节点，不能限制同域其它控制源。

| 包 | launch 文件 | 当前用途 / 设备边界 |
| --- | --- | --- |
| arm_description | `robot_state`、`view_arm` | 模型 TF / RViz；无臂串口，仍需防止重复 TF 和模拟反馈混用 |
| arm_console | `arm_console`、`arm_console_astra`、`preview_rviz` | GUI、可选相机、目标预览；显式打开 driver 开关才启动执行器，arm 的 mock 不覆盖 gripper |
| camera_perception | `astra_click_target`、`camera_calibration`、`camera_runtime` | 旧 Astra 点击、标定与相机工具；可能启动相机，没有机械臂运动入口 |
| wrist_camera_bringup | `wrist_d405`、`wrist_d405_console` | 相机与可选 Console 组合；检查 start driver 参数，不能当纯离线 launch |
| dual_camera_foundation | `system_bringup` | 现有统一组合；默认只有机器人状态模型层，arm/gripper/camera/console 开关为 false，arm mock 默认 true、motion authority 默认 false；gripper 硬件许可默认 false |
| dual_camera_foundation | `system_bringup_fixed`、`dual_camera_foundation`、`dual_camera_system` | 包装/兼容入口；实际参数逐层传递到 canonical bringup，不视作新 subsystem |
| dual_camera_foundation | `arm_feedback_safe` | mock 默认入口，固定禁止 START/MOVE；real 时仍会 GETJPOS 和接受 STOP/DISABLE |
| dual_camera_foundation | `d405_apriltag_pose`、`d405_rgbd_perception`、`dual_perception`、`hand_eye_capture_canonical` | 目标、深度、双相机选择与标定采样；只读分析不等于可以自动执行整个 launch |
| dual_camera_foundation | `astra_click_grasp`、`d405_id3_grasp` | 前者新旧 graph 不兼容；后者已弃用并转到 tabletop。不要按旧名字恢复执行 |
| tabletop_grasp | `camera_observation` | 相机坐标观察，独立命名空间；会访问相机，无臂/夹爪驱动 |
| tabletop_grasp | `tabletop_grasp`、`moveit_planning`、`rviz` | 基座坐标感知、私有 MoveIt 规划与显示；不执行轨迹，但依赖模型/标定/状态 |
| tabletop_grasp | `synthetic_demo` | 域191、合成状态和几何规划；无硬件，缺模型时不可运行 |
| tabletop_grasp | `cad_preview`、`gripper_motion_preview` | 本地 CAD 显示与估算；不是实际夹爪动作或已验证几何 |

文件完整名均为 `<name>.launch.py`。没有 `mobile_base_driver`、base launch 或 ROS 导航入口，
参见 [Mobile Base](mobile_base.md)。底盘接入后须先确定停稳、TF 和动作权限合同。

## A. 无硬件

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=192 ROS_LOCALHOST_ONLY=1
python scripts/demo_geometry.py
ros2 run arm_driver arm_driver_node --ros-args \
  --params-file "$PWD/config/examples/arm.mock.yaml"
```

mock 只证明软件消息/协议路径，普通 MOVE 不模拟到位。需要模型的独立可视化与规划：

```bash
export ROS_DOMAIN_ID=191
export ROS_LOCALHOST_ONLY=1
ros2 launch arm_description view_arm.launch.py
# 或在独立终端、停止前一套重复TF发布者后：
ros2 launch tabletop_grasp synthetic_demo.launch.py rviz:=true
```

第二个命令自己发布测试关节与 robot_state；不要另开臂驱动或 joint_state_publisher。公开候选没有合法模型时，这两条命令不能运行；纯几何 demo 和三个基础包 CI 仍可尝试。

## B. 单独相机观察

实际相机型号、SDK 序列号、USB 3、视野经现场确认后：

```bash
ros2 launch tabletop_grasp camera_observation.launch.py \
  camera_serial:="${CAMERA_SERIAL:?先填写本机 SDK 序列号}" rviz:=true
```

仅相机、几何和 RViz，无臂串口/模型/基座外参。光学坐标 X 右 Y 下 Z 前。检查 `/tabletop_camera/diagnostics` 和 `/tabletop_camera/scene`；长短边箭头只是方向预览，不是有效机器人抓取位姿。

原厂驱动独立入口：

```bash
ros2 launch wrist_camera_bringup wrist_d405.launch.py \
  serial_no:="${CAMERA_SERIAL:?先填写本机 SDK 序列号}" \
  align_depth:=true pointcloud:=false publish_uncalibrated_mount_tf:=false
```

与相机观察入口二选一。已有驱动时使用 `camera_observation ... start_camera:=false`。RViz 点云 Reliability 选 Best Effort。

## C. 实体臂反馈，关闭运动许可

先核实串口和独立急停，移除其他命令发布者，再使用本机参数文件：

```bash
ros2 run arm_driver arm_driver_node --ros-args \
  --params-file "$PWD/config/local/arm.yaml" \
  -p use_mock:=false -p allow_motion_commands:=false
```

该文件应从 `arm.feedback.yaml` 复制，保留 `use_mock:false`、`allow_motion_commands:false`。如已有合法模型需要 TF，可改为唯一 canonical owner：

```bash
ros2 launch dual_camera_foundation system_bringup.launch.py \
  start_arm_driver:=true arm_use_mock:=false arm_allow_motion_commands:=false \
  arm_serial_port:="${ARM_SERIAL_PORT:?先填写已核实的稳定串口路径}" \
  start_gripper_driver:=false start_console:=false
```

这会访问串口并轮询反馈。`arm_feedback_safe.launch.py` 现固定禁止 START/MOVE；裸 arm_driver 现默认 mock 且运动许可 false。`allow_motion_commands:false` 的边界见 [safety.md](safety.md)。

```bash
ros2 topic echo /joint_states --once
ros2 topic info /arm/joint_command -v
ros2 topic info /arm/live_joint_command -v
ros2 run tf2_ros tf2_echo base_link tool0
```

## D. 相机与基座规划

必须先有可信六轴反馈、被动 `tcp_extension_joint` 状态、合法 URDF/SRDF、已验证 mount，以及实际夹爪尺寸配置。当前软件不能补造这些观测。

```bash
ros2 launch tabletop_grasp tabletop_grasp.launch.py \
  camera_serial:="${CAMERA_SERIAL:?先填写本机 SDK 序列号}" \
  config:="$PWD/config/local/tabletop.yaml" \
  calibration_file:="$PWD/config/local/d405_mount.yaml" \
  start_robot_state:=false rviz:=true
```

示例假设步骤 C 已拥有 robot_state。已有相机再加 `start_camera:=false`；已有 mount TF owner 再加 `publish_extrinsics:=false`。该入口仍无执行接口；验证门禁未满足时拒绝是正确结果。

## E. 手动控制台与夹爪

控制台先关闭驱动与运动许可检查界面：

```bash
ros2 launch arm_console arm_console.launch.py \
  start_driver:=false start_gripper_driver:=false arm_allow_motion_commands:=false
```

控制台对已存在驱动不能代替驱动端权限。实机 MOVE/LIVE 需要独立现场授权、映射/零位/工作区/速度核验。
**WARNING：下一条命令会允许实体动作请求。** 停止步骤 C 的驱动，确认串口唯一 owner 后，
显式启动原有手动控制路径；没有发送关节目标或启用 LIVE：

```bash
ros2 launch arm_console arm_console.launch.py \
  start_driver:=true use_mock:=false arm_allow_motion_commands:=true \
  serial_port:="${ARM_SERIAL_PORT:?填写已核实的串口}" start_gripper_driver:=false
```

需要 Console 自己管理夹爪时再显式传 `start_gripper_driver:=true`、
`gripper_allow_hardware_actuation:=true` 和 `gripper_serial_port:=...`。
Astra/D405 Console 包装入口转发同名夹爪许可；canonical `system_bringup` 亦如此。
`arm_feedback_safe` 和旧 `astra_click_grasp` 固定关闭臂运动许可，不作为动作入口。

夹爪默认 hardware disabled，不建串口或命令订阅，不提供模拟成功反馈。安全测试是 `scripts/test.sh functional --packages-select gripper_driver`；真实驱动启动必须确认 PING/PONG 固件和本机 YAML：

```bash
ros2 run gripper_driver gripper_driver_node --ros-args \
  --params-file "$PWD/config/local/gripper.yaml" -p allow_hardware_actuation:=true
```

默认开/合为 600/1300 µs、通道0、600 ms；允许上限1700不是建议每次闭到极限。ROS 命令 topic 是 `/gripper/pulse_us` (`std_msgs/Int32`)；只有明确受监督操作时才发送单个已核实目标，不做自动往复。驱动没有位置/力反馈，不能从串口 ACK 宣称抓紧。

## 退出

无硬件预演 Ctrl-C 退出，检查自己启动的 launch 子进程已停止。真实运行先按已验证 STOP/断能流程使机械臂安全，再关闭 GUI/驱动；不要把杀进程或拔串口当急停。必要的软件接口为 `/arm/stop`、`/arm/disable`（std_srvs/Trigger），必须同时观察实体及固件反馈。

旧 `run_astra_preview.sh` 和 Astra grasp validator 在新旧入口迁移中存在兼容债务，不作为当前推荐 demo。需要固定相机标定时读 [calibration.md](calibration.md)。
