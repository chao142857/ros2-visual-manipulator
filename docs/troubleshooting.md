# 故障排查

| 症状 | 先核对 | 正确处理 |
| --- | --- | --- |
| package not found | 新终端是否 source scripts/env.sh，build 是否完成 | 查看 `ros2 pkg prefix 包名`，不混用旧安装目录 |
| build 拒绝已有应用 overlay | AMENT_PREFIX_PATH 是否已有本项目 | 在干净终端构建，显式选择 vendor 和 venv |
| import open3d/serial/cv2 失败 | Python 路径和 requirements | 使用 ROS Python 3.10 的 repo-local venv，运行 pip check |
| ArucoDetector 不存在 | OpenCV 版本与 contrib 模块 | 旧标定需 >=4.7；使用 requirements-dev 约束，避免多个 cv2 wheel |
| rosdep 无 astra_camera | 可选 vendor 无标准 key | 新主链无需 Astra；旧入口先确认厂商版本再 source 外部 overlay |
| 缺 URDF/mesh/SRDF | 是否来自软件公开候选 | 见 models/README；未获授权不补假模型，不调整物理参数绕过 |
| 相机找不到 | SDK串号、USB 3、电源、是否另一个驱动占用 | 显式传相机身份，不能把 USB 描述符编号当 SDK 序列号 |
| RGB-D 无同步输出 | 图像尺寸/frame、CameraInfo、stamp、QoS | 检查注册深度；不要大幅放宽时间窗口掩盖陈旧图像 |
| 点云有但 base TF 缺失 | 真实 joint_states、mount、TF owner | 先用相机独立模式，禁止填假关节或 identity 外参 |
| geometry unverified | 开口/TCP/CAD安装/外参独立门禁 | 实测并记录；synthetic 参数只用于测试域 |
| scene invalid / 无平面 | ROI、有效深度、反光、物体/平面可见性 | 查看 input/filtered/table_cloud，保留拒绝原因 |
| 多目标/近正方形拒绝 | 是否符合 isolated 与长短边可观测假设 | 明确场景/选择策略，不能把不确定朝向当精确方向 |
| IK/path失败 | start collision、关节名、模型/夹爪几何 | 查 planner_status；不删桌面、不扩大 joint limits 来强行通过 |
| 机械臂 ACK 后未到位 | 真实反馈、FIFO、模式与故障 | ACK只表示接收，先按现场停机流程处理，不连发命令重试 |
| 串口断开/超时 | 设备身份、连接、电源、协议 | 掉线不代表停住；确认实体安全后再恢复 |
| 旧 Astra validator 超时 | 新 visual_grasp别名与旧topic不兼容 | 当前已列技术债，不能用旧 PASS 指标宣传新主链 |
| tool0 有 TF、tcp 没 TF / planner 缺 passive joint | 模型中的 tcp_extension_joint 为非固定关节，而正常 arm/GUI 状态只含六轴 | 当前没有常规真实状态来源；不要手工发假状态或改固定 TF 绕过，见 architecture 与 Known Issues H2 |
| 找不到底盘 package / cmd_vel / odom | 当前源码是否真的包含底盘工程 | 本次九包无这些实现；按 mobile_base.md 接入实际代码，不临时发布 odom/static TF 假装已集成 |
| pytest 被 ROS 插件干扰 | 环境、launch_testing及日志目录 | 使用 scripts/test.sh 的隔离配置，不删除未知系统包 |
| all测试失败但functional通过 | 失败是否旧 flake8/pep257/copyright | 两类结果分开报告，不能宣称全绿 |

诊断命令是只读的，但真实节点是否运行必须由现场确定：

```bash
ros2 node list
ros2 topic info /joint_states -v
ros2 topic echo /tabletop/planner_status --once
ros2 topic echo /tabletop_camera/diagnostics --once
ros2 run tf2_ros tf2_echo base_link tool0
```

停止预演用 Ctrl-C；若有子进程残留，只处理自己启动的进程。真实机器人先执行已验证的现场安全停止/断能流程，不能依赖结束 Python 进程。提交 issue 前移除私人路径、相机编号、IP、原始画面及凭证。
