# GitHub Release Readiness Report

> 历史记录：下文保留 2026-09-08 的工作及验证，不代表最新快照。2026-09-20 的底盘范围、
> 新发现、实际改动和重新验证结果见 [当前审计报告](audit_2026-09-20.md)。

审查日期：2026-09-08。对象为当前九包 ROS 2 Humble 应用工作区与单独生成的公共软件候选。**本地完整硬件工作区不能直接公开；公开候选是排除了未知许可资产的部分软件项目，不是完整硬件复现套件。** 没有创建 GitHub 仓库、提交、推送或上传文件。

## 已完成

- 分工阅读现有源码、九包元数据、launch、URDF/MoveIt 关系、视觉/驱动/夹爪、测试、配置、文档及两次可用 Git 历史；模型做类型/来源/引用/散列审查。
- 建立根 README、架构、硬件、视觉、标定、安装、配置、bringup、安全、排障与贡献文档。
- 新增可移植 build/env/test/CI 脚本、纯几何演示、五份明确加载的配置示例和 GitHub issue/CI 模板。
- 修复缺失依赖、错误包描述、失效 console entry point、指向不存在 URDF 的检查和安装目录缓存排除；九包维护者改为已核实公开身份，私人邮箱改为明确 TODO。
- 新增根 Apache-2.0 文本，保留已有第三方版权头；模型许可独立 pending。
- 补齐 ignore、安全扫描（当前目录与全部可达 Git 对象）、逐文件发布清单、脱敏副本、清单 hash 和变更记录。
- 仅清除 47 个已证实 AppleDouble 元数据（192,512 bytes），有逐文件校验的外部备份；未删除功能源码、旧模型或现场证据。

## 架构与行为改动

没有重写核心算法或移动 package，没有改变方向、零位、电机映射、limit、PWM、速度/加速度、ACK、STOP/DISABLE 或 LIVE FOLLOW。新目录只形成公共操作和文档入口。已存在的驱动/GUI/URDF 大 diff 属于任务开始前工作，不能归因于本次整理。

删除 `joint_mapping:main` 失效 CLI 注册，保留 `joint_mapping.py` 数学实现。`arm_feedback_safe` 仅修正与实际权限相反的文案，不改权限 true。公开副本的相机串号和夹爪 USB 身份被替换为无法匹配现机的占位，必须显式配置设备；本地默认值保留。详见 [hardware_changes.md](hardware_changes.md) 和导出附带的 `PUBLICATION_TRANSFORMATIONS.json`。

另一个 RGB-D 重构任务在本次工作期间继续运行，本轮未覆盖其核心实现。整合已复核最新顶面支持计分与显式 inverse-Brown 反投影、节点参数传递及新增测试；这些算法改动属于并行任务。本轮为 SDK 派生算法补第三方声明，为公开测试替换现场内参并排除现场 profile 契约；仅改变公开副本的测试材料。测试/导出结果只适用于记录的文件快照，不能视为对之后改动的批准；最终整合以公开导出的 hash 为准。

与任务开始时的工作树进行 SHA-256 比较，37 个驱动、夹爪、GUI 和模型边界文件未变；比较基准不是旧 Git HEAD。完整样式检查涉及的 13 个实现文件也与起始快照字节一致。

## 验证结果

| 检查 | 结果与范围 |
| --- | --- |
| 全仓 colcon build | 本地原模型仍在场时，9 包独立构建通过；随后用全新 Python 环境及新的 build/install 再次通过 |
| 最新功能层 | 干净环境 490 个 colcon 汇总检查，0 errors / 0 failures / 3 skipped；跳过的是缺可选 pyrealsense2 的三个数学对照，非实机测试 |
| 新增视觉测试整合 | 最后定向 tabletop 复核 163 个汇总检查，0 errors / 0 failures / 同上3 skipped；223 个源码/配置文件前后 hash 不变 |
| 完整含样式层 | 较早快照493个汇总检查，5 failures、3 copyright skipped；失败为既有三包共118条flake8、6条pep257；13个报错实现文件与任务起始字节一致，未宣称全绿 |
| 无模型基础 CI | 源工作区中独立构建 arm_driver、gripper_driver、camera_perception，126 检查通过 |
| launch 静态构造 | 27 份 launch 构造通过，未执行节点；synthetic 在非191域正确拒绝 |
| Python/shell/YAML/XML/依赖与entry point | 本地182个Python、11个shell、20个YAML、12个XML/模型描述、9包元数据检查通过 |
| 安装规则 | 78 个 launch/config/URDF/mesh/RViz 安装文件可读 |
| 纯几何 demo | 合成物体约 81×36×41 mm，2 个候选，未创建 ROS 节点 |
| 秘密模式扫描 | 初扫当前文本及全部74历史blob，未检出模式内凭证；查到身份/路径/串号并处理公共候选。二进制/归档未逐项解码，直接排除 |
| 公开候选基础 CI | 直接在候选构建/测试三个基础包：126个汇总检查，0 errors / 0 failures / 0 skipped |
| 公开候选几何 | 单独构建两个视觉包通过；demo产生2候选、0拒绝；几何与公开反投影测试34 passed / 3 SDK-only skipped |
| 公开候选发布检查 | 253个源文件的明确清单，36条可追溯副本转换；严格模式扫描0 findings，未复制Git历史/模型/现场数据；14个发布工具回归通过 |
| 依赖一致性 | 独立公共依赖环境 pip check 通过；ArucoDetector、cv_bridge图像往返、Open3D构造通过。旧 venv 的 user-site OpenCV4.13/NumPy1.26 冲突未通过改系统包处理 |
| GitHub Actions | 已提供工作流，但未在GitHub运行；本地CI命令已验证的范围见上 |
| 真实硬件/动力学/抓取 | 本次未运行；无物理动作、无固件刷写、无电机/夹爪串口连接 |

本地原始日志保存在 `log-release/`，不进入公开候选。colcon 汇总含不同测试框架的结果，不等于独立 pytest 用例总数。早期共用结果目录的计数已被新的逐次独立结果目录取代。

干净环境：Python 3.10.12，NumPy 1.26.4，SciPy 1.14.1，Open3D CPU 0.19.0，opencv-contrib-python 4.11.0.86，pyserial 3.5，setuptools 69.5.1，pytest 8.4.2，PyYAML 6.0.3，cffi 1.17.1；禁用 user-site，使用 ROS Humble 系统 ABI。系统 ROS/MoveIt 和厂商 overlay 沿用现有安装，因此不是全新操作系统冷安装验证。

最新完整功能日志位于临时验证目录 `arm-fresh-log/test-functional-20260907T163758Z-97402`，定向整合日志为 `test-functional-20260907T163938Z-98555`。完整含样式日志为本地 `log-release/test-all-20260907T162833Z-90043`。这些日志保留在本机，不随候选分发。

候选日志为临时目录中的 `arm-candidate-ci-console.log`、`arm-candidate-geometry-console.log`、`arm-candidate-geometry-pytest.xml`；基础包结果为 `arm-candidate-log/test-functional-20260907T164556Z-101681`。测试直接读取公开副本，没有借用原工作区的应用 overlay。纯几何构建最初因未构建的集成包失败，已实证修正并在 README/安装文档中显式忽略 `arm_description arm_console wrist_camera_bringup`；没有更改 package 依赖或冒充完整 bringup 成功。

## 仍较重的技术债

1. 固件源码/版本、完整 BOM、接线、电源和独立急停规范缺失；无法从软件仓库完整复制硬件。
2. 模型/CAD/STL 来源与授权未闭合，原始 Git 含私人身份和旧资产；应从公开候选建立独立公共历史。
3. 旧 Astra grasp validator 与新 visual_grasp别名不兼容；旧 J6 与被动关节检查未适配。
4. 普通 MOVE 生命周期、故障停机证据以及夹爪反馈/重连/关闭仍需完善；GUI J6 多圈显示与后端连续关节不一致。
5. 当前夹爪开口、安装、TCP、外参和被动状态来源未完成实机验收；可见深度几何不是完整尺寸。
6. 旧文件格式/docstring 债和模块体积较大；依赖尚未形成跨机器、全哈希锁定的发布环境。

## 需要实机验证

逐轴方向/零位/限位、J6 线缆约束、手动 MOVE 到位、LIVE 心跳与失联、STOP/DISABLE、串口断开与重连、夹爪脉宽/受力/开口、独立 TCP 已知点、手眼留出姿态误差、视野/遮挡/同步、真正抓取/抬升/回程。新视觉链目前没有执行适配器，不能要求操作者解锁它完成验收。

## 需要维护者确认的信息

[PUBLICATION_AUDIT.md](../PUBLICATION_AUDIT.md) 按五类列出文件。维护者最后确认仅转接件为原创设计；本体及其他模型不在其原创声明内。重点待办是转接件对应文件和独立硬件许可证、arm_description 模型/URDF、tabletop 夹爪 mesh 与派生 JSON/SRDF 的来源/再分发权、旧现场照片/录像的公开许可。公开联系邮箱与 repository URL 已于 2026-09-21 确认，当前值见 [RELEASE_CHECKLIST.md](../RELEASE_CHECKLIST.md)。全部硬件模型此次排除，软件授权不覆盖这些未知权利。

维护者公开 GitHub 身份已核实为 `bbabbr123-sketch`（Chao142857）。历史上邮箱尚未确认；2026-09-21 用户已提供真实公开维护邮箱，当前值与仓库 URL 状态见 [RELEASE_CHECKLIST.md](../RELEASE_CHECKLIST.md)。

## 是否适合公开

**原始工作区：不适合直接公开。** 历史、现场数据和未授权模型仍保留在本地，`.gitignore` 不会清除已追踪内容。

**公共候选：已具备作为软件预览项目公开的准备条件。** 候选自身已通过上述扫描、基础 CI 与纯几何检查；完整模型、相机和实机复现尚未验收。应从这份排除未授权资产的候选创建公共历史，保留许可和限制说明；不能作为完整实机 Release 或成熟自动抓取系统宣传。模型权利、硬件资料和独立验收尚未齐备。本轮按维护者要求收尾，没有执行上传。

## 下一阶段最值得做的五件事

1. 完成来源/许可表、公开模型策略与硬件许可证选择；补齐固件版本、BOM、接线和急停资料。
2. 用干净 Ubuntu 22.04 主机或CI固定依赖复现，并为最终公共快照建立首个受审查提交。
3. 实测夹爪开口/指部、安装矩阵、动态TCP与手眼留出姿态，形成有版本的本机配置和数据格式。
4. 补齐可信关节/夹爪状态、运动生命周期与失联/STOP/DISABLE回归，再解决GUI连续J6和旧Astra入口。
5. 在上述证据满足后，单独设计可撤销、明确所有权、带故障处理的轨迹执行层；先空载受监督验收，再抓取。
