---
name: microduck-code-update
description: 更新这台 Radxa Zero 3W + Dynamixel XL330 小鸭子 mumu-duck 的板载代码。用于连接主板、部署或回滚 robotd 和 Python 控制桥、修改原生 HOME、换卡恢复后的重新部署。仅拉取 Mac 仓库或编辑本机工作台时按对应本地任务处理。
---

# MicroDuck 板载代码更新

从 `/Users/quyi/NinotQuyi/microDuck` 开始。用户说“更新鸭子里的代码”时，以板端部署后的回读为交付点。用户只请求准备补丁、构建或检查时，按该范围交付。

1. 确定要更新的层。Mac 工作台位于 `walking-runtime/`，板端 Python 控制桥位于 `/root/microduck-imu-update/walking-runtime/`，原生程序由 `/opt/robot/daemon/current` 选择，策略由 `/opt/robot/policies/current` 选择。更新 HOME 用 `duck-control/src/model.rs` 的 `DEFAULT_POSITION`，输入归一化和输出目标共享这套基准。角度按舵机 ID 绑定，12 是右髋俯仰，22 是左髋俯仰。
2. 从 `walking-runtime/workbench-config.json` 读取板地址和本机部署配置。用 `walking-runtime/ssh_trial_transport.py` 的 `ssh_arguments(host)` 验证主机身份并回读实际版本。SSH 暂不可用时，继续本地准备和校验候选包，用参考文档处理连接。以主板上的软链接和哈希为准。
3. 准备更新并验证。板端部署、从已有版本恢复或调整 HOME 时，读取 [板端部署参考](references/board-deployment.md)。记录旧版本和候选哈希，使用与板端 Linux ARM64 兼容的构建，保留上一版发布目录及与之配套的配置。只变更与请求相关的文件。
4. 切换控制代码前确认控制会话已退出、舵机全部卸力，按需独占串口。部署后回读板端版本与配置，恢复原来的只读采集，复核 15 路力矩。实机运动属于单独的操作，按当次授权和现场状态决定。报告源码、构建、板端安装、实机表现各自已验证到哪一步。

遇到换卡或明确请求烧录时，读取 [连接与恢复参考](references/connection-and-recovery.md)。普通更新在可登录的系统上完成。烧卡或擦除仅在用户请求该操作且已确认目标设备时执行。
