# Microduck 语音交互

Microduck 的多轮语音对话已完成实机验收。2026-11-12，板载麦克风采集两轮语音，`qwen3.8-omni-flash-realtime` 保留了会话上下文，小鸭子的扬声器在第二轮答出“蓝莓”。短动作候选版支持在 SSH 终端输入 `确认` 并回车，板端联调状态见运行指南。 [短动作候选版](docs/多轮语音对话.md#短动作候选版)

板端已有百炼配置。Windows 通过 SSH 启动程序，麦克风、音频处理和扬声器都在小鸭子板上，Qwen 在百炼云端生成回复。2026-11-13 用户确认实机全双工插话正常，板端现已默认启用全双工。 [当前语音流程](docs/多轮语音对话.md#当前语音流程)

```powershell
ssh -t root@10.66.54.127 /root/microduck-voice/voice.sh
```

全双工在播报期间仍会收音，板端先消除回声，百炼检测到开口后板端清空旧播报。需要等播完再说时，给启动命令加 `half-duplex`。 [全双工运行与测试](docs/多轮语音对话.md#全双工)

Windows 可以双击 [启动语音起身前进.cmd](MicroDuck%20工作台前后端/启动语音起身前进.cmd)。起身和前进分别由语音触发，等 1 秒执行。前进最高 0.5 m/s，共 5 秒，最后 1 秒渐降到零并保持平衡。 [语音起身后前进](docs/多轮语音对话.md#语音起身后前进)

| 从哪里开始 | 内容 |
| --- | --- |
| [开发规范](AGENTS.md) | Windows 开发顺序、测试方法、Git 与实机控制边界 |
| [Windows 实机工作台](MicroDuck%20工作台前后端/使用说明.md#windows-实机工作台) | 打开实机控制、关节读数、起身、停步及卸力 |
| [多轮语音对话](docs/多轮语音对话.md) | 构建、运行、模拟测试和实录验收 |
| [语音交互执行方案](docs/语音交互执行方案.md) | 功能范围、接入架构和分阶段验收 |
| `docs/board/板端环境.md`（本机） | 板卡、连接、ALSA 和机器人运行时 |
| `docs/board/调试日志.md`（本机） | 本机、模拟和板端调试及实机验证记录 |
| [板端部署参考](skills/microduck-code-update/references/board-deployment.md) | 现有 Python 控制桥和 `robotd` 的部署及回滚 |
| [连接与恢复参考](skills/microduck-code-update/references/connection-and-recovery.md) | SSH、USB 控制台和 Mac 维护资料 |

- `src/` 是语音客户端，`tests/` 是本地 WebSocket 回归测试，`scripts/` 是板端配置和启动脚本。
- `build/` 存放构建产物。`.local/qwen-demo/` 存放可复用的 Windows 输入 WAV，`.local/validation/` 存放已归档的验收产物，`.local/diagnostics/` 和 `.local/references/` 存放本机调试资料。
