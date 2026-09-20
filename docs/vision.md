# 视觉与 MoveIt 规划

当前推荐入口是 `tabletop_grasp`。AprilTag 保留用于旧标定工具，不参与新的正常桌面感知。算法为几何式点云处理，无训练权重下载步骤。

2026-09-20 审计依据为当前源码、launch、配置和离线检查。本轮未采集实时相机数据、启动实物驱动或执行抓取；历史演示与合成规划均不代表当前实物验收。

## Pipeline

1. 同步 RGB、注册深度和 CameraInfo，或接收带真实 header 的 PointCloud2。转换深度单位，拒绝尺寸/frame/编码不匹配及陈旧观测。
2. 基座模式在采集时间通过 tf2 转入 `base_link`；相机模式保持 optical frame，单独输出 `/tabletop_camera`。
3. ROI → voxel 降采样 → 统计离群过滤 → 可选半径过滤。
4. Open3D RANSAC 搜索支撑平面。基座模式限制高度与法向；相机模式只验证可见支撑面，不能确认重力或物理桌面身份。
5. 移除桌面、DBSCAN 分割。默认 `isolated` 要求一个可建模目标，不能建模的簇使观测无效。
6. 水平 PCA + 平面法向构造 OBB；X 长轴、Y 短轴、Z 法向。底面由物体接触桌面假设补齐。
7. 沿短边生成闭合方向，保留相差 π 的候选；近正方形的长短轴不可观测时拒绝抓取。
8. 独立检查夹爪开口、指部尺寸、TCP、CAD 安装和外参；未验证项阻止实物规划。
9. MoveIt FK、碰撞场景、KDL IK、OMPL 和 CartesianPath 规划预抓取、接近、空手撤退。scene 或关节失效/变化会清除结果。

`quality` 只是点数支持度启发值，不是成功概率。可见点云无法证明完整外形；相机专用顶面精化保留原深度包络，不能替换规划碰撞体。多帧一致也不能证明遮挡尺寸正确。

## MoveIt 范围

配置为 `src/tabletop_grasp/config/{dummy_v2.srdf,kinematics.yaml,ompl.yaml}`。私有服务在 `/tabletop`，包括 `compute_fk`、`compute_ik`、`check_state_validity`、`apply_planning_scene`、`plan_kinematic_path`、`compute_cartesian_path`。

arm 组是 `base_link → tool0` 六个主动关节。同一 RobotState 的 FK 求 `T_tcp_tool0`，转换 TCP 目标；被动 TCP 不成为第七个 IK 轴。

`allow_trajectory_execution=false`，不管理硬件 controllers，并禁用 MoveAction/ExecuteTrajectoryAction。planner 拒绝 `dry_run=false`。`READY_TO_EXECUTE` 只是历史状态名，不是执行许可。

撤退为空手路径，没有夹住物体后的 attachment、载荷和碰撞模型。不能将其描述成抬升成功。模型发布许可待确认，公开候选不能用其他机器人模型冒充。

## 验证与配置

参数通过 `config:=文件` 加载，多数仅启动时读取，修改后重启节点。合成入口要求域 191，使用测试 topics 和虚构夹爪尺寸，绝不能把 synthetic 的验证开关复制到实物配置。

相机观察无需臂反馈；基座规划需要真实六轴、被动 TCP 状态与外参。当前夹爪驱动不提供可信被动 TCP 输出。软件、相机数据接通和实物精度分别记录，见 [RELEASE_READINESS.md](RELEASE_READINESS.md)。

缺少 `tcp_extension_joint` 的反馈时，planner 会拒绝完整起始状态。不要将旧 PWM→TCP 估计或固定零值包装成实时测量来消除这个阻塞；接入可信状态来源及时间戳是后续控制集成任务。

当前 RGB-D 转换还提供显式 inverse-Brown 反投影选项，算法参考 librealsense v2.56.2；默认不启用。须核验本机 SDK 畸变模型后配置，不能凭 topic 名决定。公共测试使用合成内参，不分发现场 CameraInfo 或诊断 ROI。SDK 数学对照测试在没有 `pyrealsense2` 时明确跳过；它们即使通过也不证明真实相机精度。来源与许可见 [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)。

## 旧视觉入口与已知风险

`dual_camera_foundation` 的 `visual_grasp` executable 已重定向至新 tabletop planner；`d405_id3_grasp.launch.py` 也已明确改为新链的兼容名称。保留的 `visual_grasp_controller.py` 是退役实现，硬件执行锁保持关闭。

`astra_click_grasp.launch.py` 尚未完成相同迁移：它仍传入旧 controller 参数，提供 click 的 `PointStamped`，而新 planner 需要 `TabletopScene`、私有 MoveIt 服务和几何门禁；其 validator 仍等待旧 `/perception/dry_run/visual_grasp_solution`。因此这条组合的 IK 验证链当前不兼容，不应作为已工作的 Quick Start。不能通过重新启用旧执行器来绕过迁移。

旧 `rgbd_target_point` 节点仅按 K/P 内参做针孔反投影，未校验 pixel/depth/CameraInfo 的 frame、尺寸及畸变模型一致性。紧邻时间匹配不能代替几何配准；该路径应保留为实验工具。固定 Astra launch 使用最新 TF 的假设也需在移动底盘或外部固定相机接入时重新核查，见 [calibration.md](calibration.md)。
