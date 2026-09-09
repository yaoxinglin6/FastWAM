# 夹爪阻尼敏感性实验计划

## 状态与问题

2026-09-09：本阶段只准备代码入口、历史 seed 输入和运行配置，没有启动 GPU、重搜 seed 或获得实验结果。代码基线为 `c8ae740561cbd6215538e6edf70483100be1cf5c`；本轮新增能力是否已通过本地检查以 `ledger.md` 为准，真实 SAPIEN 响应须在服务器 smoke 阶段确认。

研究问题是：在固定控制器位置反馈增益和策略设置时，夹爪速度反馈增益变化是否影响闭合、交接、搬运保持和释放，并最终影响任务成功？这里改变的是夹爪关节驱动阻尼，既不是物体铰链阻尼，也不是碰撞接触阻尼。倍率增大不能直接解释为夹得更紧。刚度、速度目标、驱动力限制和接触共同决定动态响应；原始增益取决于运行时机器人资产配置，不能用代码兜底值代替实际值。

只新增 `gripper_damping_scale`，左右夹爪使用相同倍率。机器人双臂阻尼倍率保持默认 `1`；机械臂和夹爪全部刚度、夹爪驱动力限制、摩擦、恢复系数、物体质心、物体关节设置、动作节奏保持各任务的基线配置。策略输入中的夹爪观测和任务成功判据均不改变。不得同时启用其他物理覆盖。

后续配置更新：用户已指定本工作副本的场景反弹阈值统一为 0.5 m/s（见恢复系数实验账本）。使用当前工作副本运行本计划时，所有夹爪倍率必须固定同一阈值，并以本批倍率 `1` 为基线。此前导出的夹爪独立 patch 不包含这项后续修改。

## 阶段与固定面板

| 阶段 | 任务 | 每任务倍率 | 每倍率 seed | repeats | 试验槽位 | 进入条件 |
|---|---|---|---|---|---|---|
| smoke | handover_block | 1、0.5、2 | 1 个 clean | 1 | 3 | 依赖、资产、权重及 GPU 确认可用 |
| clean pilot | handover_block、stack_bowls_two、place_can_basket | 1、0.5、0.75、1.5、2 | 10 个 clean | 1 | 150 | smoke 的参数与记录验收通过 |
| random 扩展 | 同上 | 同上 | 10 个 random | 1 | 150 | clean 结果和诊断检查完成 |

作业文件按倍率 `1` 优先排队；多 GPU 并行时不保证所有基线完成后才启动其他倍率。smoke 额外占 3 个诊断槽位，不纳入 clean 的 150 个正式面板槽位。每个槽位先做专家前检；前检失败时不启动该槽位的策略 rollout，所以实际策略运行数可能少于槽位数。

`handover_block` 检查接收夹爪闭合与另一夹爪释放的配合；`stack_bowls_two` 检查抓取、搬运和放置释放；`place_can_basket` 补充抓住篮子并携罐抬升时的负载保持，其流程较长、机制归因弱于前两者。任务名称本身不能证明某一失败机制。

`successful-seeds/` 的三个 YAML 从父提交 `d00fff7d4c3c0000b6f719e4fc60496095ab3067` 原样恢复。来源路径、Git blob、字节数和 SHA256 见 `provenance.json`。每任务每 phase 恰有 10 个归档 seed，按文件顺序选择；smoke 使用 `handover_block` 的首个 clean seed `4300001`。固定策略 seed 为归档值 `42`。本轮没有重新验证历史的连续成功记录，归档中的 `consecutive_successes: 5` 也不意味着本阶段运行了 5 次。

同一 task/phase 的全部倍率严格使用同一 seed 面板，不根据新结果替换失败 seed。clean 对应 `demo_clean` 和 seen instruction；random 对应 `demo_randomized` 和 unseen instruction，两阶段同时改变了场景与指令分布，不能把二者差异直接归因为某一种随机化。跨 phase 即使整数 seed 相同也不是同一实验条件，不需要跨 phase 去重。

本次作业对旧版发布 checkpoint 显式固定 `sigma_shift=5.0`、`replan_steps=24`。历史 seed YAML 未记录 sigma shift，不能证明旧实验使用同一推理设置；历史成功率只作背景，本次所有效果比较以同批次倍率 `1` 为基线。如果服务器实际使用另一训练设置的权重，应在开始整个批次前统一调整所有作业并记录理由，不在不同倍率间混用。

这些 seed 经历史成功筛选，适合发现原可完成场景上的敏感性，不代表全部环境的无偏成功率。首轮 10 个 seed 的结果只支持诊断性结论，不能证明普遍鲁棒性。需要推广时另设未筛选且不因当前结果剔除的固定面板；重复同 seed 不等于增加独立环境样本。

## smoke 验收与记录

每个倍率确认：

1. 配置中的 `gripper_damping_scale` 到达环境与机器人初始化；左右夹爪实际驱动阻尼分别等于各自基线乘指定倍率，且倍率 `1` 复现基线。
2. 同时记录原始/有效阻尼和刚度，确认机械臂阻尼倍率 `1`、全部刚度及驱动力限制不因本扫描变化。基础左右参数可不同，不要求二者数值相同。
3. 真实夹爪位置/速度来自 articulation 的 `qpos/qvel`，目标位置来自 drive target，两类数据有独立字段。受阻闭合或运动中，两者可能不同；不能用目标位置冒充实测位置，也不能要求每帧必须不同。
4. 核对每槽位专家结果、策略结果或跳过原因、配置、退出状态及诊断记录。专家前检失败的槽位不应伪造策略诊断。本轮 seed runner 关闭视频录制，作业文件不会产生视频；若需要确认掉落/接触机制，另行安排指定 seed 的可视化复验并记录配置，不把诊断 CSV 当作完整接触证据。
5. 诊断记录覆盖策略执行的调用边界（包括最终状态，以实现为准），并关联对应槽位。此采样不是每个仿真子步，不能精确量出子步级接触峰值或闭合延迟。只可先看目标与实测误差、边界速度、阶段顺序及持续多个边界的滞后。

如果诊断读取报错、缺失或无效，相关机制结论暂停，即使成功率文件存在也不能据此解释夹爪响应。可先修复明确代码/配置错误并验证一次；资源、资产、权重或兼容性失败保留日志并停止相关任务，不盲目重跑。调度器本身不会在首个作业失败时停止全部队列；发现此类失败应人工中止本次调度，确认残留进程状态，再记录已完成与待恢复作业。不得重复提交已经成功的作业。

诊断 CSV 位于每 job 输出目录的 `diagnostics/<task_config>/seed_<seed>_repeat_<index>.csv`。每个 rollout JSON 的 `effective_drive_properties` 保存机器人当前配置，`gripper_diagnostics.applied_gripper_drives` 保存逐夹爪关节实际读取的刚度、阻尼、驱动力限制和驱动模式；`diagnostics_error` 必须为空。CSV 的 `qpos/qvel` 是实测关节状态，`drive_target/drive_velocity_target` 是目标，`take_action_cnt` 是策略动作计数而非仿真时间。每个 seed 的结果在 `results/<phase>/seed_<seed>.json`，未启用记录时保留原有输出。

clean 阶段先复核倍率 `1` 相对历史归档的行为；明显不一致时排查代码、资产和版本，不把差异直接归因为阻尼。合理范围下没有可重复变化时保留固定夹爪阻尼，并报告当前面板/采样下未发现敏感性；有证据再考虑 `0.25/4` 扩围或增加独立 seed，不在本轮默认加入。

## 统计口径

对每 task/phase/scale 分别报告计划槽位数、实际完成记录数、专家前检失败数、进入策略数、策略成功数与策略失败数。首轮每一配置的固定分母为 10；专家前检失败计为该槽位失败。未完成、超时或记录缺失必须显式列出，不能把部分 `summary.json` 的已记录分母当成完整面板分母，也不要把基础设施失败当成物理失败。

主指标为全流程成功数除以计划槽位数，同时单独报告前检通过条件下的策略成功率（策略成功数除以进入策略数，零分母标记无数据）。条件成功率会受随倍率变化的前检筛选影响，不能替代主指标。`summary.json/csv` 中保留的旧别名 `all_5_success_seed_count` 在本轮 `repeats=1` 下不代表五次成功，应使用实际 repeats 和通用字段。

以倍率 `1` 为参照逐 seed 配对，报告成功→失败、失败→成功、均成功、均失败四类计数，优先看跨倍率出现相同阶段行为的 seed。分别展示各任务，不只汇总平均成功率。若统计置信区间，独立分析单位为环境 seed；首轮小样本的不确定性必须保留，不从 10 个环境声称显著普遍提升。

## Linux 运行命令

CPU 回归检查可在不安装 CUDA/SAPIEN 的隔离环境运行：

```bash
uv --no-config run --no-project --with pyyaml --with numpy --default-index https://pypi.org/simple \
  --python 3.12 python -m unittest discover -s tests -v
```

本轮已通过 16 项检查。机器人驱动测试执行真实初始化/设置代码，但 articulation 为测试替身；其通过仅证明软件接口与参数隔离，正式仿真仍使用项目锁定的 Python 3.10 和 NumPy 1.26.4。

从已经确认归自己所有的私有 FastWAM 副本根目录运行。当前未连接服务器核实目录，不假定旧的 `/home/xlyao` 路径仍然适用。先将本轮修改及实验目录同步到该副本，并记录 `git status --short`、当前提交及本轮差异。本地 Windows 副本 `D:/code/FastWAM` 没有本次 GPU 执行结果。

下面要求用户先根据 `nvidia-smi` 设置 `GPU_IDS`，不假定任何 GPU 空闲。调度器没有跨进程 GPU 锁，只限制本进程的并发；同卡多个调度器仍会冲突。每 GPU 首轮只运行 1 个作业。可以放在已创建的 tmux 窗口中执行，但不要同时启动不同阶段。

```bash
set -euo pipefail
pwd
whoami
nvidia-smi
: "${GPU_IDS:?请先依据 nvidia-smi 设置 GPU_IDS，例如逗号分隔的已确认可用物理编号}"
FASTWAM_ROOT="$(pwd -P)"
case "$FASTWAM_ROOT" in
  /data/fastwam|/data/fastwam/*|/data/shared/FastWAM|/data/shared/FastWAM/*)
    printf '%s\n' '请切换到自己所有的私有 FastWAM 副本，不在共享代码目录运行修改' >&2; exit 1 ;;
esac
EXPERIMENT=2026-09-09-robotwin-gripper-damping-sweep
EXP_DIR="$FASTWAM_ROOT/docs/experiments/$EXPERIMENT"
OUTPUT_ROOT="${OUTPUT_ROOT:-$FASTWAM_ROOT/evaluate_results/robotwin/$EXPERIMENT}"
OUTPUT_ROOT="$(realpath -m -- "$OUTPUT_ROOT")"
case "$OUTPUT_ROOT" in
  /data/fastwam|/data/fastwam/*|/data/shared/FastWAM|/data/shared/FastWAM/*)
    printf '%s\n' 'OUTPUT_ROOT 必须是自己所有的输出目录，禁止写入共享资源' >&2; exit 1 ;;
esac
test -f "$FASTWAM_ROOT/experiments/robotwin/evaluate_robotwin_physics_sweep.py"
test -f "$EXP_DIR/smoke-jobs.json"
test -f "$FASTWAM_ROOT/checkpoints/fastwam_release/robotwin_uncond_3cam_384.pt"
git status --short
uv run --no-sync python experiments/robotwin/evaluate_robotwin_physics_sweep.py --help
```

应使用已验证的私有环境，或经确认可只读复用的环境；`uv run --no-sync` 不同步安装依赖。不要往共享 venv 安装包。manifest 的 checkpoint 是副本根目录下的相对路径；确认它指向已有权重即可，不复制大权重。正式运行前还需确认 dataset stats、RoboTwin 资产和当前实际使用的 RoboTwin 路径；此命令检查不代替这些条件。

先查询 `OUTPUT_ROOT` 下已有 scheduler 状态和结果，防止重复。以下 dry-run 会创建目录和 SQLite 队列，但不会启动模型，也不验证资产/GPU：

```bash
STAGE=smoke
uv run --no-sync python "$FASTWAM_ROOT/experiments/robotwin/schedule_robotwin_seed_search.py" \
  --experiment-name "$EXPERIMENT" \
  --jobs-file "$EXP_DIR/$STAGE-jobs.json" \
  --gpu-ids "$GPU_IDS" --max-tasks-per-gpu 1 \
  --task-timeout-seconds 36000 \
  --output-dir "$OUTPUT_ROOT/$STAGE-dry-run" --dry-run
```

确认 dry-run 的 `scheduler-run-config.json`、job 数量和 `argv_template` 无误后，正式启动同一阶段：

```bash
nvidia-smi
uv run --no-sync python "$FASTWAM_ROOT/experiments/robotwin/schedule_robotwin_seed_search.py" \
  --experiment-name "$EXPERIMENT" \
  --jobs-file "$EXP_DIR/$STAGE-jobs.json" \
  --gpu-ids "$GPU_IDS" --max-tasks-per-gpu 1 \
  --task-timeout-seconds 36000 \
  --output-dir "$OUTPUT_ROOT/$STAGE-run"
```

smoke 验收通过后显式设置 `STAGE=clean`，重复上述 dry-run 和正式启动；clean 诊断完成后才设置 `STAGE=random`。各阶段使用独立数据库，不传全局 `--database`。现有调度器拒绝已存在输出目录，不能把 dry-run 目录用作正式运行目录，也不能靠重复命令恢复中断。失败后先审计 SQLite、`launcher_logs/`、`scheduler.log` 和每个 `jobs/<job_name>/` 的结果，另建只含未完成作业的恢复配置并记录到同一 ledger；不要删除证据或换目录重跑整个面板。

作业最终完成需要队列退出状态与每 job 的 `summary.json`、`summary.csv`、配置、seed 记录和诊断产物一致。记录清点确认完整之前，不把调度器 completed 标签或汇总文件存在单独作为实验完成证据。实际输出根目录、GPU、命令、版本和结果进入同一 `ledger.md`；完成阶段后更新 `summary.md`。
