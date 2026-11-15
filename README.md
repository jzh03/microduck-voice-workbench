# MicroDuck 实机工作台与语音交互

本项目在 MicroDuck 双足机器人上实现了 Windows 实机工作台、板载多轮语音对话和语音动作控制。可以在浏览器里操作小鸭并查看关节反馈，也可以直接对着小鸭说话，让它回答问题、从地面起身或在站稳后向前走。

## 已实现的功能

- **Windows 实机控制。** 在浏览器中操作，工作台后端通过 SSH 连接开发板。支持地面起身、动态站立、站稳后手动前进和后退。松开前进／后退按钮或按空格可停步并保持平衡，结束时可执行停止并卸力。 [工作台操作](MicroDuck%20工作台前后端/使用说明.md#windows-实机工作台)
- **关节观测与调试。** 实时读取 15 路关节角度和卸力状态，在关节对照台查看实测与目标姿态，卸力时可读取静态 IMU。工作台支持导出本轮记录，控制器同时保存日志和遥测。 [关节对照台与记录](MicroDuck%20工作台前后端/使用说明.md#windows-实机工作台)
- **板载多轮语音对话。** Radxa ZERO 3W 上的 C++ 客户端通过 ALSA 收音和播放，通过 WebSocket 连接百炼 Qwen Realtime。同一个会话保留对话上下文，例如先说「我叫蓝莓」，再问「我刚才说我叫什么」，板载扬声器会答出「蓝莓」。 [语音链路](docs/多轮语音对话.md#当前语音流程)
- **全双工插话。** 播报时持续收音，板端使用 SpeexDSP 消除扬声器回声。识别到用户开口后，停掉当前播报并继续接收新一轮语音。 [全双工运行与测试](docs/多轮语音对话.md#全双工)
- **语音起身和前进。** 说「请从地面站起来」，板端进入工作台用过的起身流程。小鸭站稳后说「往前走」，控制器执行前进。两条口令均在接收后延迟 1 秒执行。前进目标速度最高为 0.5 m/s，最长 5 秒，末 1 秒平滑归零，随后继续保持平衡。 [语音动作操作](docs/多轮语音对话.md#语音起身后前进)

多轮对话、播报中插话、语音起身和前进均已在实机上验证。两轮语音前进记录到的目标速度峰值均为 0.5 m/s，完整的 5 秒前进与自动减速曲线、机体停稳时间待补测。带终端确认的短动作已在 Windows 和板端模拟环境验证。 [功能状态与验证记录](docs/语音交互执行方案.md)

## 启动入口

在已配置的 Windows 电脑上，可双击 [启动实机工作台.cmd](MicroDuck%20工作台前后端/启动实机工作台.cmd) 或 [启动语音起身前进.cmd](MicroDuck%20工作台前后端/启动语音起身前进.cmd)。板端配置好百炼后，可通过 PowerShell 启动日常语音对话。

```powershell
$boardIp = Read-Host '输入板端 IP'
ssh -t "root@$boardIp" /root/microduck-voice/voice.sh
```

启动命令默认启用全双工，附加 `half-duplex` 参数可在播报完成后再继续收音。 [板端配置与运行指南](docs/多轮语音对话.md)

## 项目文档

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
