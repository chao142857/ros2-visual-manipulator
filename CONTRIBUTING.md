# 贡献指南

本项目控制真实硬件。优先提交范围明确、行为可解释、能够复查的小改动。
请先阅读 [README](README.md)、[架构](docs/architecture.md)和
[安全边界](docs/safety.md)，区分软件测试、相机观察、规划与实体运动。

## 开始之前

1. 阅读目标 package 的代码、参数、launch、测试与文档，确认正在使用的 entry point。
2. 检查 `git status` 和已有改动，避免覆盖他人的视觉、CAD 或现场工作。
3. 说明要解决的问题、影响的接口和验证办法；跨包重构先记录必要性和兼容方案。
4. 使用 [安装文档](docs/installation.md) 的本地环境；不要求开发者采用某个用户名或
   固定工作区路径，不将本机 overlay 和标定复制为默认配置。

## 修改边界

关节方向、raw/ideal 映射、零位、限位、电机映射、速度/加速度、PWM 范围和停机行为
是硬件契约。不要为了格式统一、测试通过或扩大规划可达范围修改它们。
确需变更时，提交独立改动，并在 [hardware_changes.md](docs/hardware_changes.md)
写明旧行为、新行为、错误证据、影响和实体复验状态。

GUI 通过 ROS 接口工作；串口只由对应驱动持有。不要让视觉模块直接写串口，或为 dry-run
添加隐含执行端。不要用 synthetic 的验证字段证明实物 CAD、TCP、开口或外参已经验证。
新硬件功能需明确接收、执行、到位、超时、中止与失联语义。

配置优先使用现有 ROS 参数、remapping 和 YAML；无收益时不引入新配置框架。
变量、topic、frame 与单位应有明确名称。时间戳、SensorDataQoS、无效数据传播和 shutdown
需要结合实际运行路径审查，不能只检查 Python 能否导入。

## 验证

从未加载旧应用 overlay 的新终端构建。按改动选择测试，并记录命令、环境和结果：

```bash
export PYTHONNOUSERSITE=1
export ARM_PYTHON_ENV="$PWD/.venv"
bash scripts/ci.sh
```

这是公开候选的三个基础包功能测试。完整模型依赖已经满足的开发树可执行：

```bash
bash scripts/build.sh
source scripts/env.sh
bash scripts/test.sh functional
bash scripts/test.sh all
```

变更仅涉及部分包时，可向 build/test 脚本传入 `--packages-select`。
不要把现有 lint 失败隐藏为通过；分别报告新增问题和基线技术债。测试应针对真实行为与
失败边界，文档排版等低影响修改不必添加重复实现细节的测试。

涉及 ROS mock/合成输入时使用独立 domain 和 localhost，并确认其中没有实体驱动。
测试中不打开实体串口、不发送 PWM/MOVE、不默认访问相机。需要现场验证的功能单独
说明场景、设备、操作授权与停止方法，并将“软件通过”和“硬件尚未验证”分开记录。

## 文档与提交说明

一次改动应说明：具体问题、最终行为、验证结果、限制和硬件影响。
安装依赖、参数、topic/frame 或启动入口改变时，同步更新相关文档与示例。
过时示例应明确迁移路径，不能仅依靠旧 executable 名称暗示兼容。
不要在说明中把 ACK 写成抓取成功，或把规划轨迹写成实际执行录像。

建议 PR 包含以下并列信息：

- 问题与修复后的行为。
- 修改范围及兼容性，是否影响硬件契约。
- 实际执行的检查、测试与未完成的验证。
- 新依赖、资源来源及许可证（如有）。

## 数据、许可与发布

原创软件采用 [Apache-2.0](LICENSE)，保留已有第三方版权声明。
新增代码、CAD、mesh、图片、日志或数据集时，必须能够说明来源和再分发授权。
软件许可证不自动覆盖硬件设计与第三方 CAD，见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

本机配置放 `config/local/`，运行数据使用被忽略的目录；不提交 token、密钥、设备串号、
私人邮箱、现场标定、未审查相机画面或用户目录硬编码。`.gitignore` 不会清除既有 Git 历史。
公开候选使用逐文件 manifest，添加文件后需要重新审查清单与导出结果。

发布前按 [PUBLICATION_AUDIT.md](PUBLICATION_AUDIT.md) 重新导出和扫描，并基于最终文件
执行适用测试。未经审查的整个硬件工作区及其旧历史不能直接推送到公共远程。
发现安全问题时遵循 [SECURITY.md](SECURITY.md)，不要在公开 issue 中粘贴敏感材料。
