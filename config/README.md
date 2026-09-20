# 部署配置

公共示例在 `examples/`，本机副本在忽略的 `local/`。示例不会自动加载；通过 `--params-file` 或 launch 参数明确选择。

`arm.mock.yaml` 不打开串口。`arm.feedback.yaml` 会打开指定的真实串口，但禁用运动命令。`gripper.example.yaml` 不是 mock，当前夹爪驱动没有 mock 模式。占位设备路径必须人工填写。

外参示例没有有效标定，不能授权基座规划。索引见 [configuration.md](../docs/configuration.md)。
