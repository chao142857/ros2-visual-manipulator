# 模型与发布许可

资源按 ROS 包 `package://` URI 安装，本目录不重复复制。授权未确认的模型、派生几何在公开候选排除，本地原件保留。原创 CAD 许可证也尚未选择，Apache-2.0 软件许可不覆盖它们。

维护者最终确认：硬件中仅转接件由本人设计，机械臂本体及其他硬件模型不属于这项原创声明。目前未建立转接件与具体文件的可核验对应关系，且未选择其硬件许可证；因此本轮连同转接件在内，所有硬件模型都不进入公开候选。

| 资源 | 开发版本预期位置 | 用途 |
| --- | --- | --- |
| 当前 URDF | `src/arm_description/urdf/dummy-v2.urdf` | joint1…joint6、tool0、被动 TCP |
| 旧 URDF | `src/arm_description/urdf/dummy-v1.urdf` | 历史版本，不能替换当前模型 |
| 当前 mesh | `src/arm_description/meshes/dummy_v2/*.stl` | 当前 URDF 引用 |
| 旧 mesh | `src/arm_description/meshes/*.stl` | 旧 URDF 仍引用，未当废弃品删除 |
| MoveIt SRDF | `src/tabletop_grasp/config/dummy_v2.srdf` | 需匹配当前 URDF |
| 夹爪几何 | `src/tabletop_grasp/meshes/`、`config/cad_*`、`config/gripper_motion*.json` | 未确认再分发权的静态和运动模型 |

不提供未经授权资产自动下载或同名替代。权利确认后需记录来源 URL、版本/hash、作者、许可证、生成步骤和修改，再更新发布清单。

缺资源时，可运行三个基础包 CI、纯数值几何、相机坐标观察。机器人 RViz、MoveIt、GUI 模型预览及部分测试暂不可完整复现。空目录 README 不是模型，安装成功不表示模型可用。
