# 连接与恢复参考

从 Mac 工作区 `/Users/quyi/NinotQuyi/microDuck` 执行。操作命令和恢复镜像拼装示例见该 Mac 工作区的 `我的小鸭子维护手册.md`。历史地址、端口编号和编译缓存只用于定位。

## 连接主板

- Mac 和小鸭子接入同一局域网。历史主机 `mumu-duck` 曾在热点 `quyinet` 上取得 `192.168.43.213`。先用 `workbench-config.json` 配置的地址尝试固定主机密钥的 SSH。旧地址超时时，让用户从热点设备列表或 USB 控制台取得当前 IP。更换 IP 时从可信控制台核对主机指纹，然后登记新地址并更新工作台配置。
- Mac 使用 `/dev/cu.usbmodem*` 枚举 USB ACM 控制台，已验证的登录速率为 115200。用数据线和板子的 USB-C OTG 口，枚举后可运行 `screen <实际端口> 115200`。设备只显示 Rockchip Maskrom/Loader 时，优先诊断引导和存储。区分 `8770` 的本机服务、板端 SSH 和舵机总线三层状态。

## 用户明确请求刷卡或换卡时

- 保留旧卡，按当前实物确认新卡、外置读卡器和实际容量。工作区中的 `outputs/system-hardening-20261026/RECOVERY_BACKUP.md` 是已留存备份入口。`python3 -B outputs/system-hardening-20261026/verify_backup.py` 会核对三件恢复数据的大小和 SHA-256，且只读取本机文件。
- `prepared-filesystem.img` 是 ext4 分区镜像。完整卡需要 `RECOVERY_BACKUP_MANIFEST.json` 中的引导前缀、分区镜像和 GPT 卡尾。已记录的原布局是 31,299,993,600 字节、512 字节扇区。只有匹配这一布局才使用维护手册的整盘拼装命令；其他容量的卡应单独迁移分区和重建 GPT，或装干净系统。写盘前以 `diskutil list external physical` 和 `diskutil info` 识别实际外置卡。
- 干净系统参考已验证的 Radxa Zero 3W + Armbian Debian 13 Minimal vendor 内核基线，按所选系统的官方首启流程配置。恢复这台机器所需的 UART 设备树、网络、SSH、软件零位、桥接、ONNX Runtime、策略及运行时绑定。10 月 26 日的恢复快照早于 11 月的 HOME、IMU 与工作台更新，开机后按板端部署参考回读并补齐所需版本。
- 交付区分镜像验证、实际开机、SSH/USB 可用、版本哈希及硬件卸力。恢复后先检查正常重启和文件持久化，再判断能否进入经授权的实机测试。
