# Roadmap

截至 2026-09-21 的工程待办；本页不创建远程 GitHub Issues。下列验收条件可直接拆成小 issue。
已实现内容见 [README](../README.md)，已知风险见 [Known Issues](known_issues.md)。

| 优先级 / 类别 | 下一项工作 | 完成标准 | 当前状态 |
| --- | --- | --- | --- |
| P0 Safety / Control | 明确 arm/gripper/base 的动作权限、所有权和停机合同 | 默认权限迁移有回归；STOP/失联/卸力边界可解释，实物时延由独立受监督验收提供 | 默认权限已修复并回归；动作所有权/实物停机/整机互锁仍 WIP |
| P0 Calibration / Manipulation | 建立真实 passive TCP 状态来源及夹爪开口/安装标定 | 区分命令、估计、实测；按 name/time 合并状态；RSP/planner 验证来源与新鲜度，独立已知点误差达标 | 缺状态来源，测量未闭合 |
| P0 Hardware / Release | 确认模型/转接件/固件来源与再分发权，补 BOM/接线/急停图 | 每个发布资产有来源、作者、许可证；不会用软件许可证代替硬件授权 | 源码许可已有，模型/固件 pending |
| P1 Mobile Base | 接入实际底盘源码、协议与独立 package | 可追溯版本、参数示例、mock parser/command/timeout tests；确认轮型/尺寸/反馈，见 [底盘合同](mobile_base.md) | 限定外部开发目录也未定位实体控制实现 |
| P1 Mobile Base / TF | 定义底盘根、机械臂根、odom 和相机关系 | frame 无重名/双 authority；测量安装变换；现有 ROI/手眼迁移可追溯，不静默改 base_link 语义 | Planned |
| P1 Control | 普通 MOVE freshness 与命令生命周期；夹爪 transport 恢复 | 迟到/未来命令、断连/超时/重连/取消有故障注入测试；ACK 和到位分别建模 | WIP |
| P1 GUI | 连续 J6 多圈显示/目标复制与状态来源 | 多圈 Current→Target 保真，显示范围不冒充实物线缆限制；无设备 UI 回归通过 | WIP |
| P1 Perception / Calibration | 统一外参质量 schema、旧 RGB-D 畸变与采集时间 TF | 无效/负数/过大误差 fail-closed；合成畸变/错帧/延迟 tests；独立留出姿态实测 | WIP |
| P1 Manipulation | 清理旧 Astra grasp 入口与 validator 迁移 | 声明停用或提供完整新 message graph 契约，不恢复隐式执行 | 旧链不兼容 |
| P1 Documentation / Reproducibility | 干净 Ubuntu/CI 复现与软件候选发布 | 依赖记录、候选 manifest/hash、有限 CI、真实已知问题；首个远程构建有证据 | 本轮只做本机离线验证，无 push |
| P2 Perception | 公开 RGB-D 失败案例、遮挡/深度稳定性与点云抓取评估 | 数据有许可，指标区分可见尺寸/全物体、方向可观测性和连续稳定时长；保留失败 | Experimental |
| P2 Simulation | 合法模型、惯量、接触/载荷、mock base/gripper | 数学演示、几何规划、动力学仿真分别有验收；不把可视化当仿真通过 | 仅数值/几何 demo 已有 |
| P2 Mobile Manipulation | 底盘停稳/臂姿态/稳定支撑互锁 | 反馈和 odometry 过期会阻断动作；导航与臂控制有唯一所有权；模拟异常与物理验收分离 | Planned，依赖前述 P0/P1 |
| P3 Navigation / Manipulation | 评估 Nav2 与受监督 pick-and-place 执行层 | 导航接口、带物碰撞、可取消 action、闭爪/抬升/回程/失败恢复各自通过验收 | Planned，当前无执行 adapter |

MoveIt Servo 不是当前已决定的依赖；只有明确需要连续笛卡尔控制并具备状态、限速和停机合同后
再评估。Nav2 同样不能仅靠加入 launch 或 map frame 宣称集成。

所有需要真实机械臂、夹爪或底盘动作的步骤均留待用户单独授权的现场任务：
`SKIPPED: requires physical robot actuation`。本轮不烧录、不发送控制命令、不创建远程 issues。
