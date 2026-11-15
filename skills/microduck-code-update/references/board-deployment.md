# 板端部署参考

下列 Mac 路径相对于 `/Users/quyi/NinotQuyi/microDuck`。板端目录和启动方式以回读及当前部署文件为准。

## 预检与备份

- 从 Mac 的 `walking-runtime/workbench-config.json` 取得 `host`、`certificate`、`reference`、`manual_reference` 和 `deployment_report`。`Backend(8769).request('/status')` 返回控制会话状态，读取所需字段即可，整份结果可能包含大量历史样本。Mac 网页地址是 `http://127.0.0.1:8770/`。
- 用固定主机密钥的 SSH 回读 `hostname`、boot ID、`uname -m`、`readlink -f /opt/robot/daemon/current`、`readlink -f /opt/robot/policies/current`、实际 `robotd` 和模型的 SHA-256，以及 `robotd.service` 的状态。用 `workbench-config.json` 定位部署记录，`outputs/current-native-home-build.json` 只是编译缓存指针，已安装版本可以回退到比缓存更早的构建。
- 保存此次更新涉及的板端文件、旧链接目标和 Mac 配置。板端的桥接清单是 `/root/microduck-imu-update/walking-runtime/bridge-deployment.json`，软件零位在 `/root/microduck-imu-update/software-zero-reference.json`。保留私有配置与访问材料在本机私有目录。
- 本地准备、读代码和 SSH 身份探测可先做。实际替换控制文件前，核对控制会话停止、当前 15 路力矩为零。新鲜采集来自 `outputs/live-joint-viewer-20261017/state.json`。若控制仍在运行，让现场先承托并结束会话，再核对卸力。过期数据或重连中状态只能记为未知。

## 构建或复用候选版本

- **Python 桥接更新**：准备改动文件及重新计算哈希的 `bridge-deployment.json`。保持本机和板端各自需要的调用接口一致。对于跨模块的改动，测试入口使用 `walking-runtime` 中的相关 `test_*.py`。将精确文件集传到私有暂存目录，读回哈希后部署。合同、零位、IMU 或参考姿态只有在本次变更涉及它们时才重新绑定。
- **原生程序或 HOME**：从相容的 Git 标签或提交开隔离工作树。已验证过的定制原生程序基于 `daemon-v0.17.9`。可复用 SHA-256 与既有记录完全匹配的 Linux aarch64 二进制和配套配置。需要重编译时，板端编译可复用已准备的 Rust 工具链及 Cargo 缓存。参考 `outputs/native-home-30p24-20261106-225013/build-detached.sh` 的环境配置，核对真实路径和缓存源码哈希。编译持续数分钟时使用可重新查询的后台任务和退出码记录，在控制停止的时段运行。

  ```sh
  cargo test --release -p duck-control --lib --locked --offline -j 2 &&
  cargo build --release -p robotd --bin robotd --locked --offline -j 2 &&
  sha256sum target/release/robotd
  ```

  `--offline` 依赖已准备好的 Cargo 缓存。切换编译源之前，分别核对板端编译缓存、本机候选源码和板上安装版本，三者可能来自不同版本。

## 板端切换与回滚

1. 从 `workbench-config.json` 定位现行 `deployment_report` 所在的产物目录。读取其中的 `installation.json`、`deploy-local.py`、`install-remote.py` 和 `install-payload.json`（有哪个读哪个）。缺少模板时，可查看 `outputs/native-home-restore30p24-20261106-230659/` 的已验证记录。**把这些脚本和载荷当模板。** 载荷里的旧链接、旧文件哈希、候选二进制路径、目标 HOME 和 release 名都依赖当次部署。新包须使用本次回读值和当前候选文件。
2. 只改 Python 时，按已有桥接文件清单暂存并验证目标文件，在同一维护窗口同步更新受影响的清单。改原生程序时，将候选 `robotd` 写入独立的 `/opt/robot/daemon/releases/<release>/` 发布目录，保留之前的发布目录。HOME 变更同时更新 `runtime-contract.json`（含原生二进制绑定）、部署清单、证书和站姿参考及 Mac 工作台配置。纯原生 HOME 调整保持桥接偏移为零。沿用 IMU 方向记录时要核对 IMU 安装和校准仍匹配，并在新记录里标明继承来源。
3. 用 `panel_recovery.control_lock(host)` 和 `ssh_trial_transport.pause_ssh_viewer(host)` 协调维护窗口。前者阻止并发控制，后者要求新鲜的卸力读数并释放采集串口。板端再次确认 `/dev/ttyS2` 无其他持有者，读取 15 路 Torque Enable，全零后进入安装。可复用历史安装器的 `WireBus(..., live=False)` 与零写入检查。存在进程或力矩状态未知时保留旧程序并核对状态。现有实机通过 Python 桥接应用软件零位和保护，维护时保留其 `robotd.service` 屏蔽状态和串口路径配置。
4. 核对候选哈希及链接库，在 `robotd --fake` 中读取候选 HOME，核对 `robotd` 未打开舵机串口。让新目录、板端文件和配置通过部署清单与证书校验，再原子切换 `current`，同步 Mac 配置。用 `run_teleop.verify_files(certificate, physical=True)` 复核板端绑定；这个函数只核对文件，名称中的 `physical=True` 启用实机文件校验条件。SSH 在切换中断开时，先回读实际链接和配置再判断是否完成或需要回滚。
5. 回滚时恢复同一批次的 `current`、受影响的桥接文件和清单、运行合同、证书和参考及 Mac 配置。若改过模型，同步恢复策略指针。板端回读通过后恢复 Mac 工作台和采集，必要时用 `python3 walking-runtime/launch_workbench.py --no-open`。若本机服务仍使用旧参数，核对进程身份与运行状态后重载受影响服务。

## 交付证据

保存板端版本及哈希、改动文件、已跑的测试、无驱动读回以及新鲜卸力结果。针对 HOME，可用以下本机检查：

```sh
cd /Users/quyi/NinotQuyi/microDuck/walking-runtime
python3 -m unittest test_native_home_binding test_stance_reference \
  test_policy_coordinates test_stance_kinematics test_stance_state test_workbench
```

如果板端尚不可达，记录已准备的候选包和具体连接错误。若配置已安装但本机采集尚未恢复，分别报告安装与力矩验证状态。报告实机动作结果需要独立的现场证据。

## 语音动作候选

`scripts/stage-voice-bridge.py` 读取板端 `walking-runtime` 和部署清单，核对 `run_teleop.py` 基线及清单中的每个文件，向新的候选目录写入语音控制补丁及候选清单。候选目录应位于 `/root/microduck-imu-update/` 下，与 `walking-runtime` 相邻，以复用同一份合同绑定的零位和坐标配置。

```sh
python3 -B /path/to/microduck/scripts/stage-voice-bridge.py \
  /root/microduck-imu-update/walking-runtime \
  /root/microduck-imu-update/walking-runtime-voice-candidate
```

先用候选 `run_teleop.py --mode fake` 和私有临时目录下的 `--voice-socket` 验证。使用独立 `--port`、`--token-file` 和 `--output-dir`，且确认没有打开舵机串口。`voice.sh control` 使用同目录 `control/microduck-voice` 的 ARM64 候选程序，C++ 端通过 `MICRODUCK_CONTROL_SOCKET` 对应候选 socket。候选验证通过后，按本节前述步骤处理现行控制桥的部署清单和切换。真实动作验证须由用户明确允许。
