# RoboTwin 物理实验代码实质修改说明

## 1. 文档范围与版本依据

- 修改前基线：`origin/find-successful-seeds`；
- 当前代码：`find-successful-seeds` 工作树上的未提交物理实验修改。
- 说明口径：不再把 seed 搜索、successful-seed validation、多 GPU 调度器算作本次修改，因为它们已经属于 `find-successful-seeds` 基线。

相对该基线，本次实质性代码范围是：

| 文件 | 类型 | 主要影响 |
| --- | --- | --- |
| `experiments/robotwin/search_robotwin_seeds.py` | 修改 | 在已有 fixed seed runner 中增加 `setup_overrides` 参数入口 |
| `experiments/robotwin/evaluate_robotwin_physics_sweep.py` | 新建 | 将物理参数映射成环境 kwargs，并在 fixed successful seeds 上评测 |
| `experiments/robotwin/check_restitution_material_response.py` | 新建 | 检查 `restitution` 材质设置是否真实影响碰撞回弹 |
| `third_party/RoboTwin/envs/_base_task.py` | 修改 | 接收并应用 object damping、restitution 开关和 center-of-mass offset |
| `third_party/RoboTwin/envs/utils/create_actor.py` | 修改 | 让主要 collision 创建路径显式使用默认物理材质 |
| `third_party/RoboTwin/envs/robot/robot.py` | 修改 | 支持机器人左右臂关节阻尼统一倍率 |
| `third_party/RoboTwin/envs/open_microwave.py` | 修改 | microwave articulation 支持 object damping 覆盖 |
| `third_party/RoboTwin/envs/put_object_cabinet.py` | 修改 | cabinet articulation 支持 object damping 覆盖 |
| `third_party/RoboTwin/envs/turn_switch.py` | 修改 | switch articulation 支持 object damping 覆盖 |

## 2. 实验参数如何进入环境

本次物理实验没有重新实现 seed 搜索逻辑，而是在 `find-successful-seeds` 已有的 fixed successful seed 评测流程上增加参数注入。

参数传递链路是：

```text
jobs.json / 命令行参数
  -> evaluate_robotwin_physics_sweep.py
  -> setup_overrides
  -> search_robotwin_seeds.py::_task_args()
  -> RoboTwin task setup_demo(**args)
  -> Base_Task / Robot / create_actor 在环境创建阶段应用物理修改
```

`search_robotwin_seeds.py` 的实质修改只有一个：`_task_args()` 会读取 config 中的 `setup_overrides`，并把非空键值加入 RoboTwin task args。

影响：

- 不传 `setup_overrides` 时，原来的 fixed seed 搜索和验证行为不变。
- 传入物理参数后，expert planning、稳定性检查和 policy rollout 都在同一物理设置下运行。
- 实验结果中的失败既可能来自环境初始化或 expert 前检，也可能来自 policy rollout，不能只解释成 policy 本身退化。

## 3. 物理 sweep runner

`evaluate_robotwin_physics_sweep.py` 是本次新增的物理扫描入口。它复用 `find-successful-seeds` 中已有的 manifest、policy 加载和 `_evaluate_candidate()`，但额外负责三件事：

1. 解析 task、物理参数名、参数值或 setting id。
2. 从 successful-seed manifest 中选取固定 seed panel。
3. 将物理参数转换成 `setup_overrides`，交给 RoboTwin 环境。

支持的物理参数映射如下：

| 参数 | 写入环境的 kwargs |
| --- | --- |
| `object_joint_damping` | `object_joint_damping`，并固定 `object_joint_stiffness=0.0` |
| `robot_joint_damping_scale` | `robot_joint_damping_scale` |
| `restitution` | `restitution` |
| `center_of_mass_x` | `center_of_mass_offset_x` |
| `center_of_mass_y` | `center_of_mass_offset_y` |
| `center_of_mass_z` | `center_of_mass_offset_z` |
| `center_of_mass_sphere` | 从 samples file 读取 `dx/dy/dz`，映射到三个 offset |

影响：

- 所有物理因素走同一评测协议：同一 policy、同一 successful seed 面板、clean/random 两个 phase 的固定分母。
- `run_config.json` 会记录实际使用的 `setup_overrides`，这是复现实验时确认参数是否传入的首要位置。
- `summary.csv` 保留逐 seed 的 expert 状态、rollout 成功数、错误信息和物理参数值。

使用注意：

- `center_of_mass_sphere` 必须额外提供 `--physics-samples-file`，并用 `--physics-setting-id` 指定样本。
- 默认 `--repeats` 是 5，但具体实验可能在 jobs 中改成 1；写实验文档时应以对应 run 的 `run_config.json` 为准。

## 4. 机器人关节阻尼修改

位置：`third_party/RoboTwin/envs/robot/robot.py`

新增 `robot_joint_damping_scale`，在机器人初始化时读取：

```python
joint_damping_scale = float(kwargs.get("robot_joint_damping_scale", 1.0))
```

左右臂关节阻尼由原来的 embodiment 配置值改为：

```python
joint_damping * robot_joint_damping_scale
```

影响：

- 该参数改变机器人左右臂的控制动态，expert planning 和 policy rollout 都会受影响。
- 左右臂使用同一个倍率，避免只改一侧引入额外变量。
- 不传该参数时默认 `1.0`，等价于 `find-successful-seeds` 基线行为。

使用注意：

- 该倍率只作用于 arm joint damping，不作用于 gripper damping。
- 大倍率或小倍率下的失败不能简单归因于物体物理变化，因为机器人自身响应也变了。

## 5. 物体 articulation 阻尼修改

位置：`third_party/RoboTwin/envs/_base_task.py`、`open_microwave.py`、`put_object_cabinet.py`、`turn_switch.py`

`Base_Task` 新增两个 kwargs：

```python
self.object_joint_damping = kwags.get("object_joint_damping")
self.object_joint_stiffness = kwags.get("object_joint_stiffness", 0.0)
```

任务加载 actors 之后会调用 `apply_task_physics_overrides()`。如果传入了 `object_joint_damping`，它会扫描 task 中保存的 `ArticulationActor`，并调用：

```python
actor.set_properties(float(object_joint_damping), float(object_joint_stiffness))
```

此外三个任务做了显式接入：

| 文件 | 修改影响 |
| --- | --- |
| `open_microwave.py` | microwave 门关节支持 damping 覆盖；未传参数时保留原来的 `set_properties(0.0, 0.0)` |
| `put_object_cabinet.py` | cabinet articulation 创建后立即应用 damping 覆盖 |
| `turn_switch.py` | switch articulation 创建后立即应用 damping 覆盖 |

影响：

- 微波炉门、柜门、开关等 articulated object 的动力学会改变。
- 该修改发生在 `load_actors()` 之后、稳定性检查之前，因此会影响稳定性检查、expert planning 和 policy rollout。
- sweep 中 `object_joint_stiffness` 固定为 `0.0`，这次没有扫描 stiffness。

使用注意：

- 不是所有任务都有 articulation，全任务平均会被大量不敏感任务稀释。
- 自动扫描 task 属性可能会改到 task 中多个 articulation，不是按模型名只改一个指定对象。
- 已显式接入的三个任务是物体关节阻尼分析时最需要关注的任务。

## 6. 恢复系数修改

位置：`third_party/RoboTwin/envs/_base_task.py`、`third_party/RoboTwin/envs/utils/create_actor.py`

`Base_Task` 中新增：

```python
self.use_default_collision_material = "restitution" in kwags
```

传入 `restitution` 时，RoboTwin 初始化流程会创建带恢复系数的默认物理材质；`create_actor.py` 新增 `_default_collision_material()`，在主要 collision 创建路径中把 `scene.default_physical_material` 显式传给 SAPIEN builder。

已覆盖的主要路径包括：

- `create_table()` 的 table leg collision。
- `create_obj()` 的 convex / nonconvex collision。
- `create_glb()` 的 convex / nonconvex collision。
- `create_actor()` 的 convex / nonconvex collision。

影响：

- 走这些 actor factory 的碰撞体可以响应传入的 `restitution`。
- 恢复系数在 collision shape 创建阶段生效，不是 rollout 中途修改。
- 不传 `restitution` 时，`use_default_collision_material=False`，新增分支会回到原来的创建方式。

使用注意：

- 该覆盖范围不是全局所有资产；URDF 内部自带材质、未走这些 factory 的 collision 不保证被统一覆盖。
- `create_sphere()` 当前创建了 `collision_material` 局部变量但没有实际使用；不过 sphere collision 本身仍使用 `scene.default_physical_material`。
- `create_entity_box()` 和 table top 在基线里已经使用默认物理材质，本次主要补的是之前没有显式传材质的 builder 路径。

## 7. 质心偏移修改

位置：`third_party/RoboTwin/envs/_base_task.py`

`Base_Task` 新增三个 offset 参数：

```python
center_of_mass_offset_x
center_of_mass_offset_y
center_of_mass_offset_z
```

环境初始化时会组装成三维 `center_of_mass_offset`。如果 offset 非零，则对 scene 中动态刚体组件和 task articulation links 调用：

```python
set_cmass_local_pose(original_pose + offset)
```

影响：

- 改变动态物体和 articulation link 的局部质心位置。
- 该修改发生在稳定性检查之前，因此较大 offset 可能直接导致初始化或 expert 阶段失败。
- `center_of_mass_sphere` 与单轴 offset 使用同一底层机制，只是 offset 来自采样文件中的 `dx/dy/dz`。

使用注意：

- 这是场景级动态物体和 task articulation link 的质心扰动，不是只改某一个目标物体。
- 静态对象不会作为 dynamic component 被修改。
- 单轴 `center_of_mass_x/y/z` 和球面采样的统计口径不同，合并写结论时要区分。

## 8. 恢复系数响应检查脚本

`check_restitution_material_response.py` 是本次新增的 sanity check，不参与正式 policy 成功率统计。它创建一个 SAPIEN scene，设置不同 `restitution`，让 box、bowl、can 从空中下落，并记录接触后的回弹高度和上行速度。

影响：

- 用来确认 `restitution` 不是“参数传了但物理完全不变”。
- 它只能证明代表性 actor 创建路径有物理响应，不能代表 RoboTwin 所有资产都被覆盖。

使用注意：

- 输出 CSV 是材质响应证据，不是任务成功率证据。
- 如果正式 sweep 对 `restitution` 不敏感，应优先结合任务集合、成功判据和覆盖路径解释，而不是直接认为参数无效。


## 9. 审计结论与已知限制

1. 相对 `origin/find-successful-seeds`，本次修改没有改变 FastWAM 模型权重、网络结构、成功判定或 seed 搜索策略。
2. 本次新增能力集中在物理参数注入、物理 sweep runner 和 RoboTwin 环境创建阶段的物理覆盖。
3. `setup_overrides` 是物理参数进入环境的公共入口；复现实验时应优先检查 `run_config.json` 中的该字段。
4. `restitution`、`object_joint_damping`、`center_of_mass_offset` 的覆盖范围都受 actor 创建路径和 task 对象结构限制，不是对所有对象的精确全局修改。
