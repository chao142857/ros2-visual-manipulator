# 标定与独立验收

不复制作者现场结果到新装置。公开示例刻意未标定；本地历史文件的拟合内残差不能作为新机器或独立测点精度。任何移动机械臂采样的步骤都需要现场监督与硬件授权，本次没有执行。

审计日期：2026-09-20。本轮仅检查源码与已存文件，未重新验证相机安装、内参、手眼精度、TCP 或实物接触。文件存在、可加载及 ROS TF 连通均不等于标定验收。

## 固定 Astra 外参

`camera_perception` 的 fixed-camera calibrator 用已知 `base_link → tag_ref` 与 PnP 的 `camera → tag` 关系求固定外参，再保存 YAML。该路径使用固定参考靶和样本均值，不是眼在手上的多姿态标定。参考 tag 的真实位置/方向/边长必须重新测量；URDF 里的历史 tag_ref 坐标不是通用值。

运行入口是 `camera_calibration.launch.py`，参数可用 `ros2 launch camera_perception camera_calibration.launch.py --show-args` 查阅。它需要相机、AprilTag 和真实参考坐标，不能无人值守启动后就认定已校准。`camera_extrinsics_publisher_node` 只有 `calibrated:true` 才发布固定 TF。

## D405 眼在手上

1. 固定支架，记录相机型号/SDK身份、分辨率、内参和实际 Tag ID/边长。相机独立观察入口不会自动产生标定结果。
2. 提供真实六轴反馈与合法机器人模型，让 `base_link → tool0` 在图像时间可查。不要使用 mock 或零关节。
3. 单独启动 AprilTag pose（`d405_apriltag_pose.launch.py`），明确 `target_tag_id`、`tag_size_m`、`target_frame`，核对与采样器一致。
4. 操作者在多个有充分姿态变化且完全静止的位置采样。采样器使用图像时间的 TF，拒绝陈旧/重复姿态。

```bash
mkdir -p config/local
ros2 launch dual_camera_foundation hand_eye_capture_canonical.launch.py \
  output_file:="$PWD/config/local/hand_eye_samples.yaml" sample_count:=20
# 在独立终端、每个已确认静止姿态下请求一次：
ros2 service call /hand_eye/capture std_srvs/srv/Trigger '{}'
```

这只是采样器，不会启动相机或移动机械臂。默认 `target_frame` 是 `wrist_calibration_target_6`，更换 Tag 时须同时更改检测器与采样器。源码历史 ID/尺寸表不是实际靶尺寸证明。

```bash
ros2 run dual_camera_foundation hand_eye_calibration \
  --samples config/local/hand_eye_samples.yaml \
  --output config/local/hand_eye_optical.yaml \
  --tool-frame tool0 --camera-frame wrist_d405_color_optical_frame \
  --minimum-samples 15
```

输出为 `T_tool0_optical`。RealSense 自己拥有 `T_mount_optical`，应用应发布的 mount 是：

`T_tool0_mount = T_tool0_optical × inverse(T_mount_optical)`。

从本机原厂 TF 记录内部变换为校准 schema 后，运行现成转换器：

```bash
ros2 run dual_camera_foundation mount_calibration \
  --optical-calibration config/local/hand_eye_optical.yaml \
  --mount-optical-calibration config/local/d405_internal_tf.yaml \
  --output config/local/d405_mount.yaml
```

不能把 optical 解直接发布成 `tool0 → wrist_d405_link`。旧 calibrated mount publisher 有样本数/RMS/outlier 门禁，但这是文件质量自报检查；新 tabletop extrinsics loader 主要校验格式/frame/矩阵，其实物外参精度由独立验证标志约束。二者都不是第三方独立标定证书。

当前两条加载路径的门禁不一致：旧 publisher 默认要求至少 15 个样本、平移 RMS ≤ 0.01 m、旋转 RMS ≤ 2°、无 outlier；tabletop loader 只要求至少 1 个样本及有限 RMS，不限制 RMS 大小或 outlier 数。新 planner 的 `camera_extrinsics_verified` 默认关闭，但基座感知/可视化仍可能使用已加载的暂定外参。此项作为已知风险保留，本轮不修改标定或降低门禁。

TF 必须只有一个安装变换发布者：机械臂模型提供 `base_link → tool0`，手眼 mount 文件提供 `tool0 → wrist_d405_link`，RealSense 提供相机内部 optical TF。不要同时启动 placeholder、旧 mount publisher 和 tabletop extrinsics，也不要将标定 optical 目标伪装成 mount link。固定 Astra 的 `use_latest_transform=true` 仅在观测期间相机相对目标基座不变时成立；接入移动底盘或世界固定相机后，必须重新定义安装关系并检查观测时刻 TF。

## 零位、TCP 与夹爪

J1–J5 映射、J6 continuous、关节限位和零位本轮保留，见 [hardware.md](hardware.md)。先在安全姿态核对单轴方向、模型零位和真实反馈，再做高位已知点 TCP 校验。

`gripper_geometry.py` 是既有 PWM→轴向 TCP 表，不能证明当前 PWM、开口或碰撞几何。新 CAD 运动估计仍为 approximate_unverified；静态 CAD 指间距不是极限开口。逐项量测可用开口、安装矩阵、TCP、指垫和静态配置，不能只设置一个全局 geometry_verified 绕过检查。

tabletop planner 还要求带可信时间戳的 `tcp_extension_joint` 状态；当前夹爪驱动未提供该状态，只有六轴反馈不足以完成规划起始状态检查。这个接口缺口与 TCP 几何标定是两件分别需要完成的工作。

## 独立验收记录

至少保存未用于拟合的多个已知基座点、不同腕姿的误差向量/最大值、标定文件版本、内参、机械零位、时间同步与原始采样。报告分别列拟合内 RMS、独立测点误差、重复性和真实接触结果；没有实测则明确 pending。私人原始数据放 `config/local` 或忽略的 artifacts，不自动公开。
