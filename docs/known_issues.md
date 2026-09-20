# Known Issues / Limitations

审计日期：2026-09-20。以下由当前源码、配置与离线验证支持；不是实物故障复现报告。
优先级按误操作后果和复现影响划分。没有更改映射、限位、TF 数值或标定；第二阶段经授权修正了执行权限默认值。
实际测试结果见 [审计报告](audit_2026-09-20.md)，修复顺序见 [Roadmap](roadmap.md)。

## Critical

### C1 — 裸驱动的真实硬件/运动许可默认值（已修复，2026-09-21）

原默认为 `use_mock=False`、`allow_motion_commands=True`，首次合格命令会 START/MOVE；
此前 `arm_feedback_safe` 也允许动作。现裸臂驱动/组合 launch 默认 mock 且权限 false，
feedback/旧 Astra preview 包装固定关闭运动许可；夹爪另设默认 false 的硬件许可。
构造级 fake 测试覆盖默认不打开串口、拒绝命令和显式 opt-in 的原协议字节。

真实反馈模式仍会 GETJPOS，STOP/DISABLE 仍可写入；显式启用后的硬件限制、
命令源仲裁和故障后实体状态仍需现场验收。这不是整个 ROS 网络的授权机制。
第一阶段发现及旧测试证据保留于 [审计报告](audit_2026-09-20.md)。

本轮模式扫描未检出规则内私钥/token/密码；没有因此认定仓库可以直接公开。历史个人身份、
设备标识及未解码二进制/归档的发布边界见 [本次审计](audit_2026-09-20.md)。

## High

| ID | 问题与源码证据 | 当前处理 / 验收缺口 |
| --- | --- | --- |
| H1 | 夹爪无位置/力反馈或与臂状态联锁；显式启用后同域合法 PWM 会产生 SET | 默认硬件许可已关闭；PING/ACK 不等于夹稳或 MCU 上电无 PWM，固件未收录 |
| H2 | V2 URDF 的 `tcp_extension_joint` 非固定；arm driver 和 GUI preview 只发布六轴；夹爪无关节状态 publisher | 标准真实反馈无法保证 `tool0→tcp` TF；新 planner 要求 passive joint 状态，合成发布者不能用于真实机器人。禁止把目标脉宽当实测 |
| H3 | J1–J5 默认 ±3.14 rad，J6 continuous；URDF 无惯量，驱动没有路径碰撞检查 | 这些是当前软件约定，未经逐轴机械限位/线缆/载荷认证；未发现足以改轴向或限位的证据 |
| H4 | `astra_click_grasp.launch.py` 启动的 `visual_grasp` 已指向 tabletop controller，输入却仍是旧 PointStamped，validator 仍等旧输出 | 旧 IK 组合链不兼容；不作为 Quick Start，不恢复已停用执行路径。新入口使用 TabletopScene + 私有 MoveIt |
| H5 | 本地 ignored URDF/SRDF/CAD JSON/tabletop YAML/标定存在，但软件导出不携带它们 | 九包本地 build 不能证明公开 clone 可做完整 MoveIt bringup；只承诺经过候选自身验证的软件子集 |
| H6 | 当前源码无底盘驱动、cmd_vel、Odometry、导航、停稳互锁或整机模型 | 整机移动操作集成阻塞；不推断底盘硬件不存在，也不编造底盘可运行。见 [mobile_base.md](mobile_base.md) |
| H7 | 原 Git 历史包含设备身份/个人邮箱和已有模型；模型权利尚未闭合 | `.gitignore` 不清理 tracking/history。保留私有源树与历史，使用新目录逐文件导出，不改 remote、不推送 |

## Medium

| ID | 问题与证据 | 后续方向 |
| --- | --- | --- |
| M1 | 普通 MOVE 时间戳校验拒绝零值/安全 barrier 之前消息，但没有最大年龄和未来时间上界；LIVE 路径更严格 | 单独定义普通命令 freshness、所有权及 action 生命周期，保留 ACK/STOP 协议 |
| M2 | START 等待、settle 与串口事务占用 executor/lock；软件 STOP 不保证立即抢占；DISABLE 可能失去承载力 | 独立硬件急停与可测最坏停机时延，禁止把杀进程当急停 |
| M3 | 夹爪缺独占打开/write timeout、完整重连及明确 close；读响应异常路径薄弱 | 增加纯 transport fault tests 后再独立修复，不承诺掉线恢复 |
| M4 | GUI 六个 target 编辑器统一范围，J6 多圈反馈可能被裁剪；旧 arm_monitor 只按六个位置顺序读，不支持 passive joint 合并 | 先明确连续角度显示/命令语义和命名状态合同；不能只扩大控件范围替代安全设计 |
| M5 | `tabletop_grasp/extrinsics.py` 只要求至少1样本和有限 RMS，比旧 mount validator 的样本数/RMS/离群门禁弱 | 基座可视化可使用较差外参；独立 camera_extrinsics_verified 仍阻断规划。统一质量 schema 前不修改现场文件 |
| M6 | 旧 `rgbd_target_point.py` 按针孔 K/P 投影，未建立尺寸/frame/畸变契约；旧 Astra target transform 采用 latest TF | 输入契约与采集时刻 TF 需专项修复，移动集成前明确相机安装与时序；新 tabletop 反投影更严格 |
| M7 | 现有机械臂根已经叫 `base_link`，ROI/SRDF/外参依赖它；未来底盘也用该名称会产生语义冲突 | 当前未证明有实际双发布冲突，不改 frame。先制定名称、安装变换和单发布者合同 |
| M8 | rosdep 的系统 OpenCV 不满足旧 AprilTag 的 ArucoDetector API；未形成全依赖锁定 | 使用 requirements 指定的 ROS Python 3.10 venv；冷安装/跨机 ABI 尚需验证，不能用当前电脑成功代替 |
| M9 | 部分本机路径、相机串号、夹爪 by-id 身份仍在本地源树/实验记录 | 多数设备入口已有 parameter/launch override；公开导出使用占位符，现场值留本机，不批量改设备默认 |
| M10 | 公开 `arm_description` 软件许可已明确，但本地模型、固件/BOM/接线/减速比/电气急停资料不完整 | 软件包 Apache-2.0 不解决排除模型与固件权属；补逐文件来源与硬件资料 |
| M11 | 无效 scene 停止目标 TF 更新，中止规划清 candidates/trajectory，但 tf2 缓存和下游已收 PoseStamped 不会被撤销 | 消费方须检查 scene.valid、时间戳与 planner_status；缓存中存在变换/阶段目标不代表当前仍有效 |

## Low

- 原有 flake8/pep257 样式债与若干 0.0.0 包版本仍在，不能把软件 CI 子集说成全仓 lint 通过。维护邮箱与 repository URL 均已确认；远程 CI 与最终公开发布验收仍待完成。
- 旧根 build/test wrappers 依赖历史 sibling workspace/overlay，旧 offline test 包列表不含新的 tabletop 两包；推荐 `scripts/` 入口。
- 许可证/ament test 模板和实验输出存在字节重复；不是有证据可以删除的重复业务模块。
- 第一阶段只修正 CLI 的 `Dsiable` 日志拼写；第二阶段只增加默认权限及其回归，未做样式重写。

## Experimental / WIP

当前感知只能从可见 RGB-D 支撑估计几何，遮挡完整尺寸、近方形 yaw、D405 深度质量和
标定精度不能由点云外观证明。新链为 perception → candidates → IK/collision → preview paths，
没有闭爪、带物抬升、附着物碰撞、回程、执行 action 或自动导航。
`READY_TO_EXECUTE` 是保留状态名；`dry_run=false` 被拒绝。

物理抓取/移动/刷写测试：**SKIPPED: requires physical robot actuation**。
相机实时采集、手眼采样和当前安装精度：**NOT TESTED**。

## Public software release boundary (2026-09-21)

Internal CAD-derived geometry validation is excluded from the public software release.

`test_gripper_motion.py`、`test_cad_config.py` 及含来源未确认目标/关节快照的
`test_locked_j6_ik.py` 不进入 allowlist；原开发树文件完整保留。
公开副本的 CAD preview、既有 mount calibration、SRDF contract 各有一个明确缺资产
SKIP；其余软件检查仍执行。缺资产在原开发树不会被自动忽略，public 判定要求导出 manifest。
`arm_description` 缺模型时跳过 model XML lint，保留两个 launch 的 lint；
构建成功仍不代表完整 RViz/MoveIt 模型可复现。
运行中的历史 TCP 插值和 RViz 预览姿态约定保持不变，不被宣称为新标定或通用硬件几何。
当前发布门禁与身份状态见 [RELEASE_CHECKLIST.md](../RELEASE_CHECKLIST.md)。
