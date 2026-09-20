# 配置索引

保持现有 ROS 参数、launch arguments 与 YAML，不引入新的配置框架。新增 `config/examples` 只示范现有接口。硬件映射继续保留为明确的源码契约，避免误加载改变方向/零位。

| 对象 | 当前来源 | 修改方式 |
| --- | --- | --- |
| 臂 serial/baud/poll/mock/许可 | arm_driver 参数、system_bringup arguments | 本机 YAML + `--params-file`，或明确 launch arguments |
| raw/ideal 映射、方向、零位 | `arm_driver/joint_mapping.py` | 硬件契约，本轮不改，见 hardware.md |
| URDF joint axis/limit/TCP | arm_description 模型 | 需资源授权和物理验证；不能扩大限位让 IK 通过 |
| GUI 范围/超时、open/close | `arm_console/config/arm_console.yaml` | 当前硬件参考值，不保证其他装置适用 |
| 夹爪 serial/channel/PWM/duration/硬件许可 | gripper_driver 参数 | `gripper.example.yaml` 默认 disabled，非 mock；显式 `allow_hardware_actuation=true` 才接设备 |
| PWM→TCP | `gripper_driver/gripper_geometry.py` | 历史表，不是当前反馈或实测开口 |
| D405 串号/name/namespace/align/pointcloud | wrist_d405 launch arguments | 显式 SDK 串号；driver YAML 只是参考，并未自动加载 |
| D405 健康阈值 | wrist_d405_health.yaml | health 节点参数 |
| 固定相机外参 | camera_perception schema | 用户重新标定；公共示例 calibrated=false |
| optical/internal/mount 标定 | dual_camera_foundation schema | 分别采样/转换，存 `config/local/` |
| 新视觉 topics/frames/ROI/过滤/时序 | tabletop YAML、typed config、controller DEFAULTS | 复制并经 `config:=...` 显式加载 |
| RGB-D 反投影模型 | `rgbd_rectified`、`rgbd_inverse_brown_conrady` | 以本机 SDK 内参/畸变模型核验为依据；两者不能同时开启 |
| synthetic 几何 | synthetic.yaml | 域 191 专用，不能用于实物 |

```bash
mkdir -p config/local
cp config/examples/arm.feedback.yaml config/local/arm.yaml
```

本机参数、`.env`、bags 和运行产物被忽略；忽略规则不会删除已追踪历史，公开必须用发布导出目录。多数参数只在启动时读入，不能假设 `ros2 param set` 已实时更新算法。

Topic 可通过参数或 ROS remapping 改名；planner 的 `/tabletop` 服务名当前固定，多臂支持尚未完成。TF frame 无前导 `/`；修改 frame 需同时核对 URDF、SRDF、外参、图像 header 和 health。

默认 `rgbd_rectified=true`、`rgbd_inverse_brown_conrady=false`。`image_rect_raw` 名称本身不能证明没有畸变；RealSense ROS 的 `plumb_bob` 标签也不能区分 SDK 的 inverse-Brown 模型。只有核验实际 SDK profile 后，才在本机配置显式选择 `rgbd_rectified=false` 与 `rgbd_inverse_brown_conrady=true`。本轮保留默认行为，不复制现场诊断配置作为通用配置。

symlink-install 可能使安装目录标定文件指回源码；保存结果应显式使用本机新路径，避免覆盖历史原件。
