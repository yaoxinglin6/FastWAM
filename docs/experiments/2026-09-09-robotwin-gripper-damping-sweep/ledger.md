# 夹爪阻尼扫描实验账本

## 2026-09-09：准备阶段

- 目标：增加独立夹爪阻尼倍率和可选真实状态诊断，准备固定 seed 的分阶段实验；本轮不运行 GPU 实验。
- 基线：`robotwin-damping-restitution-center-of-mass`，`c8ae740561cbd6215538e6edf70483100be1cf5c`。
- 工作副本：`D:/code/FastWAM`；从本轮已核对的临时克隆建立，origin 指向 `https://github.com/yaoxinglin6/FastWAM.git`。
- 当前状态：实现中；开始前工作树干净。当前副本没有 scheduler SQLite、evaluate_results、assets、checkpoint 或已执行的本次实验结果；未访问服务器或提交任务。
- 关键决策：默认倍率 1；左右夹爪共同缩放；机械臂阻尼、全部刚度、策略输入与任务成功判据保持基线。只使用专用 physics sweep 接口。
- 输入归档：计划从基线父提交恢复三个任务的历史 successful-seed YAML 到本实验目录，并记录来源；这不是本阶段新搜索或验证结果。
- 验证：将用 CPU 单元测试检查参数隔离、参数传递、真实状态读取和实验配置；GPU/SAPIEN 行为仍待服务器 smoke 实验。
- 下一步：完成实现、实验计划、可运行作业文件、测试和交付。

### 参数接口已实现

- `Robot` 读取独立 `gripper_damping_scale`，默认 1，拒绝负数和非有限值；仅缩放左右夹爪配置阻尼，保留刚度和机械臂阻尼。
- physics sweep 新增倍率映射、`--phases` 和 `--record-gripper-state`。phase 默认仍为 manifest 中全部阶段，诊断默认关闭。
- 已核对 GitHub 分支仍为 `c8ae740`；仅本地修改，未提交或推送。
- CPU 测试初查发现可用 Python 缺少 PyYAML，将用 `uv run --no-project --with pyyaml --with numpy` 的隔离缓存环境验证，不同步 GPU 项目依赖。

### 诊断实现与验证环境

- 新增独立诊断模块，并在 seed runner 的可选路径接入。策略输入保持不变；仅在 setup 后和策略调用结束时记录真实关节位置、速度及驱动目标，非每个仿真子步。
- rollout 记录有效机器人配置、逐夹爪关节实际驱动参数和 CSV 路径；诊断故障单独标记，不将其计为策略失败。
- 默认清华镜像安装 CPU 验证依赖失败（TLS handshake EOF）；改用官方 PyPI 的 uv 隔离缓存环境，不修改项目锁文件或共享运行环境。

### CPU 验证与实验配置已完成

- uv 的已有配置仍优先使用失败镜像；通过 `uv --no-config` 明确使用官方 PyPI 后，隔离环境安装 PyYAML 6.0.3、NumPy 2.5.3 成功。测试使用 Python 3.12；这些是 CPU 合同测试环境，不是正式 Python 3.10 / NumPy 1.26.4 的仿真运行环境。
- 16 项 unittest 全部通过：默认倍率兼容、左右 base/mimic 驱动、arm/gripper 隔离、非法值、真实 kwargs 转发、phase 筛选、3/150/150 作业矩阵、实际 qpos/qvel 与目标的区别、异常隔离、资源关闭。
- 仿真依赖未导入，机器人测试执行实际 Robot 类初始化及驱动设置代码，使用 fake articulation。此证据不代替 SAPIEN smoke。
- 三份 seed YAML 已恢复至本实验目录，与父提交逐字节相同并记录 SHA256；作业文件经实际 scheduler parser/placeholder 渲染检查。
- 原 `.gitignore` 忽略全部 tests；仅为本次两个测试文件增加定向例外，确保交付包含验证代码。
- 完整诊断路径与字段已写入 plan；本轮不录制视频，不能声称已有视频或高频接触证据。

### 最终审查与交付

- 用户后续授权将本地更新提交并推送到现有 GitHub 物理实验分支；实现、测试、jobs 和 seed 归档一并交付。本轮完整修改总结按用户要求仅保留本地。以下“未提交/未推送”记录描述此前准备阶段，发布进度同时记录在恢复系数账本。

- 按用户要求新增跨项目更新总结 `docs/physics_update_summary_2026-09-09.md`，汇总本轮夹爪、诊断、实验矩阵、阈值及恢复系数修复；明确 23 项 CPU 测试的范围、未运行仿真状态，以及旧独立夹爪 patch 不包含后续修复。本次总结编写未修改代码或启动实验。

- 后续场景配置变更：用户指定反弹阈值为 0.5 m/s，现已在当前工作副本的 Base_Task 中设置；当前目录下运行的夹爪实验同样使用此阈值，所有倍率应保持一致。此前导出的独立夹爪 patch 保留原交付内容，不包含本次阈值变更。尚无 GPU 作业运行。

- 独立只读审查未发现本轮参数传递、arm/gripper 隔离、真实状态采样或作业启动接口的实质问题；`git diff --check` 通过。
- 本地验证命令：`uv --no-config run --no-project --offline --with pyyaml --with numpy --default-index https://pypi.org/simple --python 3.12 python -m unittest discover -s tests -v`，16 项通过。
- 当前状态：代码与实验输入准备完成；无正在运行的本次作业、无 GPU 实验结果。下一步仅在资源核实后运行 smoke，再决定进入 clean/random。
- 本轮修改保存在 `D:/code/FastWAM`，将另行导出包含新增文件的 patch；没有 Git commit 或 push。GitHub 上仍是原基线，服务器不会自动获得这些本地修改。
