# Release checklist — software-only candidate

复核日期：2026-09-21。候选目录：`arm_ws-publication-20260920`。
本轮仅完成 publication metadata；不改变机器人运行行为、不连接实体硬件、不改历史、不推送。
**STAGING GATE: GO — 仅限 private staging。**
**METADATA FINALIZATION: COMPLETE — 0 unresolved publication placeholders。**
旧 BLOCKED 门禁证据及上一候选完整保留；远程 CI 尚未执行，当前不是 GO FOR PUBLIC。

## Publication metadata

| Input | Current value | State |
| --- | --- | --- |
| `PUBLIC_REPOSITORY_URL` | `https://github.com/chao142857/ros2-visual-manipulator` | 用户明确确认，README 与全部 16 处 package/setup metadata 已填写 |
| `PUBLIC_MAINTAINER_EMAIL` | `chao070503@gmail.com` | 用户明确确认，已写入全部 16 处 package/setup metadata |

Repository URL / 邮箱清单（每个 package 都有 `package.xml`；仅以下七包另有 `setup.py`）：

| Package | package.xml | setup.py |
| --- | --- | --- |
| `arm_console` | [src/arm_console/package.xml](src/arm_console/package.xml) | [src/arm_console/setup.py](src/arm_console/setup.py) |
| `arm_description` | [src/arm_description/package.xml](src/arm_description/package.xml) | 不适用（ament_cmake） |
| `arm_driver` | [src/arm_driver/package.xml](src/arm_driver/package.xml) | [src/arm_driver/setup.py](src/arm_driver/setup.py) |
| `camera_perception` | [src/camera_perception/package.xml](src/camera_perception/package.xml) | [src/camera_perception/setup.py](src/camera_perception/setup.py) |
| `dual_camera_foundation` | [src/dual_camera_foundation/package.xml](src/dual_camera_foundation/package.xml) | [src/dual_camera_foundation/setup.py](src/dual_camera_foundation/setup.py) |
| `gripper_driver` | [src/gripper_driver/package.xml](src/gripper_driver/package.xml) | [src/gripper_driver/setup.py](src/gripper_driver/setup.py) |
| `tabletop_grasp` | [src/tabletop_grasp/package.xml](src/tabletop_grasp/package.xml) | [src/tabletop_grasp/setup.py](src/tabletop_grasp/setup.py) |
| `tabletop_grasp_interfaces` | [src/tabletop_grasp_interfaces/package.xml](src/tabletop_grasp_interfaces/package.xml) | 不适用（ament_cmake） |
| `wrist_camera_bringup` | [src/wrist_camera_bringup/package.xml](src/wrist_camera_bringup/package.xml) | [src/wrist_camera_bringup/setup.py](src/wrist_camera_bringup/setup.py) |

README 与安装文档包含真实 clone 命令；九个 package.xml 使用 repository URL 标签，
七个 setup.py 使用 url 字段，邮箱全部一致。没有猜测或新增 maintainer 名称；既有维护者显示名保留。
全部 publication metadata 已确认，不再需要额外 metadata 输入。
远程 GitHub Actions 和最终发布清单通过之前，**不得称为 GO FOR PUBLIC**。

## Release boundary

Internal CAD-derived geometry validation is excluded from the public software release.

- 从 allowlist 排除 `test_gripper_motion.py`、`test_cad_config.py`；私有原文件保留。
- 关联排除 `test_locked_j6_ik.py` 的来源未确认目标/关节快照；不改运行预设或合成替代数据。
- 三个保留测试遇到明确排除的 CAD/SRDF/mount 文件时，在 public candidate 中报告 SKIP。
- 三份 wrist lint tests 恢复 `ros2/ros2cli` 0.18.18 OSRF 署名；测试 AST 与原行为一致。
- `arm_description` 的 Apache-2.0 只覆盖导出的软件文件，不授权本地 CAD/模型。
- 原始 `.git`、models、现场标定、bags、artifacts、build/install/log、归档和截图均不导出。
- 运行代码、串口协议、映射、limits、PWM、TF 和 calibration 在此次清理中未修改。

公开软件包含九包源码、launch/config、tests、docs、许可证、notices 和发布工具；
不具备完整模型，不能宣称完整 MoveIt/RViz/机器人仿真复现。
样式债、实体性能、固件、电气急停和模型权利保持独立验收。

## Checklist

- [x] secrets checked — strict-public pattern scan；授权邮箱与上游 notice 联系方式分开核对。
- [x] license decided — 原创软件保留 Apache-2.0；未给未知资产重新授权。
- [x] third-party asset licenses checked — 12 个模板和 librealsense 改编及 notices 有准确出处。
- [x] clean build — 新临时 build/install/log，候选九包完成；无私有模型或应用 overlay。
- [x] tests passing — 442 功能用例通过、6 跳过、0 失败；publication 17/17；launch safety 17/17（也包含在功能用例中，不重复累加）。
- [x] safe-by-default — 已有默认 mock / hardware opt-in 保留，本轮未改运行代码。
- [x] no absolute local paths — 作者 home / 私有设备路径不进入候选。
- [x] README accurate — 软件子集与完整机器人、Working/Experimental/Planned 分开。
- [x] hardware requirements documented — BOM/固件/未实测项明确。
- [x] known issues documented — 模型缺失、测试覆盖缩减、实物未验收公开记录。
- [x] public history strategy decided — B：候选建立独立干净历史，原开发历史保留。
- [x] public maintainer email confirmed — 全部 9 package.xml / 7 setup.py 一致。
- [x] public repository URL assigned — 用户确认真实 URL，9 个 ROS manifest / 7 个 setup metadata 与 README 一致。
- [ ] GitHub Actions remote run passed — 本轮未创建 remote 或推送。
- [ ] final public checklist passed — 仅允许先进入 private staging。
- [ ] 全新 Ubuntu 22.04 冷安装 — 本机已有 ROS/vendor 依赖，不能冒充冷安装。

## Git boundary

没有初始化 Git、设置 remote、提交、改写历史或推送。
推荐继续采用独立候选的新历史；不从原开发 workspace 整体公开。
候选一旦具有 `.git`，导出器会拒绝 `--refresh` 覆盖；后续应导出到新临时审查目录。
硬件测试：`SKIPPED: requires physical robot actuation`。

## Offline validation

| Check | Result | Scope |
| --- | --- | --- |
| Clean colcon build | PASS: 9 packages | 当前候选，ROS Humble + 既有兼容 venv；显式 RealSense messages underlay，仅构建 |
| Functional pytest | PASS: 442 passed / 6 skipped | 3 个模型/现场标定缺资产；3 个可选 pyrealsense2 数学对照；不包含样式债 |
| Publication regression | PASS: 17 | allowlist、hash-pinned license override、联系人精确匹配与secret回归 |
| Offline launch safety | PASS: 17 | Node actions 不执行，覆盖默认参数和 opt-in 转发 |
| Structure / syntax | PASS within software scope | 167 Python、4 shell、13 YAML、9 XML；8 项已排除模型/标定依赖明确 pending |
| Metadata | PASS | 9 package.xml 标准解析、7 setup.py metadata、九包 Apache-2.0 |
| Payload / provenance / notices | Revalidated on regenerated payload | 每个文件 hash，继承仅限同 SHA 的旧逐文件结论；变更逐项复核 |
| Physical hardware | SKIPPED: requires physical robot actuation | 本轮未连接实体串口、相机或 actuator |

`arm_description` 在软件导出中没有 URDF，故 model XML lint 明确 SKIPPED；
不能以编译/功能通过替代 URDF/Xacro 或完整仿真验证。全仓历史 flake8/pep257 债不在本轮修复范围。
本轮证据留在原开发树的 publication gate 归档中，不进入 public payload。

## Metadata finalization — 2026-09-21

本次只增加真实 repository URL 并更新发布文档/导出清单；功能源码、测试源码、
运行配置和上游 notices/许可证保持 SHA 一致。包 metadata 重新做标准解析与 setup 字段校验；
导出器重新生成 payload hashes 和转换报告，再对整个 265 文件候选复核 provenance、
notices、secret patterns、publication placeholders 与 README 相对链接。
旧门禁证据保留；新证据独立存档。远程 GitHub Actions 尚未运行，当前不是 GO FOR PUBLIC。
