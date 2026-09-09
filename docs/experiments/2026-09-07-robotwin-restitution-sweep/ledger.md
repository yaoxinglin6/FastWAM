# 恢复系数实验账本

## 2026-09-09：用户授权 GitHub 更新

- 用户要求将当前本地更新推送到 `yaoxinglin6/FastWAM`，明确排除本轮完整修改总结。
- 目标分支为现有 `robotwin-damping-restitution-center-of-mass`，远端起点核实为 c8ae740，与本地 HEAD 一致。
- 提交内容包括实现、测试、实验作业/种子输入及项目文档；`docs/physics_update_summary_2026-09-09.md` 仅保留本地，并加入本地 Git exclude，排除上传。
- 推送前验证依据为已通过的 23 项 CPU 测试、独立审查和暂存差异检查；此发布不表示真实仿真验证已完成。

## 2026-09-09：本轮完整更新总结

- 用户要求汇总本次所有变化，已新增 `docs/physics_update_summary_2026-09-09.md`，覆盖夹爪阻尼、真实状态诊断、实验输入与阶段设置、0.5 m/s 阈值、参数转发、摩擦保留、合法参数范围、测试和交付状态。
- 总结保留原材质基线与 e=0 扫描参考的区别、旧结果的解释限制，以及独立脚本接触测量仍未修复的事实。
- 本次仅编写总结及更新账本，未修改执行代码、未重复运行测试或实验、未提交或推送。
- 文档验证完成：7 个相对文件链接均可解析，代码围栏配对，差异格式检查通过；独立只读审查未发现实质遗漏或误述。

## 2026-09-09：修复转发并保持单因素

- 用户要求：修复恢复系数转发断点，扫描期间保持材质的其他物理属性不变；反弹阈值固定 0.5 m/s。
- 已核实：SAPIEN 3.0.0b1 的 actor builder 在没有显式 material 时调用 `sapien.physx.get_default_material()`；`physx_default.cpp` 中默认静摩擦/动摩擦/恢复系数为 0.3/0.3/0.1。任务显式创建的 scene.default_physical_material 为 0.5/0.5/0。因此此前强制网格使用 scene 材质会同时改变摩擦。
- 实施决定：仅向 setup_scene 转发 restitution；原来显式使用 scene 材质的形状保留其摩擦，原来使用 builder 默认材质的路径从实际默认材质读取摩擦，创建仅恢复系数不同的新材质。不修改引擎全局默认材质，不扩大到机器人/夹爪/URDF/地面等未覆盖路径。
- 验证安排：补 CPU 回归测试证明请求值到达创建材质处，并证明默认/自定义摩擦保留、未扫描路径兼容及跨场景无全局污染；尚未提交实验作业。
- 代码已更新：窄转发 restitution；构建器材质保留运行时默认摩擦；扫描入口、场景入口和独立脚本拒绝非有限或超出 [0,1] 的恢复系数，独立脚本默认值移除大于 1 的项。夹爪及其他物理覆盖入口保留。
- 验证完成：新增 `tests/test_restitution.py` 的 7 项 CPU 回归测试通过，执行真实 Base_Task 初始化/创建场景方法以及 OBJ/GLB/actor 的凸/非凸碰撞分支、桌面/桌腿创建函数，依赖使用 fake 对象。确认 e=0/0.2/0.8/1 到达材质；不扫描时保持原调用；保留 0.3/0.3 和自定义 0.7/0.2 摩擦；原共享材质不被修改；非法恢复系数在创建场景前拒绝。
- 旧缺陷复现：测试智能体仅在内存中读取 HEAD 原源码，证明请求 e=0.8 时材质创建仍得到 (0.5,0.5,0)，以及原默认摩擦 (0.7,0.2) 被旧 helper 替换成 (0.5,0.5)；没有回退或覆盖工作树。
- 全部检查：`uv --no-config run --no-project --offline --with pyyaml --with numpy --default-index https://pypi.org/simple --python 3.12 python -m unittest discover -s tests -v` 共 23 项通过，包含此前夹爪的 16 项；`git diff --check` 通过。测试输出中的诊断错误来自故障注入用例，不是实际磁盘或仿真故障。
- 独立只读审查：官方 SAPIEN 3.0.0b1 API 与当前 diff 一致，三个材质字段可读，create_physical_material 不修改全局默认；未扩大覆盖面。
- 当前完成边界：本地代码与 CPU 验证完成，未提交、未推送、未 GPU 仿真；真实材质读回和接触动力学仍待服务器验证。独立脚本的接触判定和 bounce_height 算法未在本次修复范围内，仍不能作为有效回弹测量。

## 2026-09-09：调整低速反弹阈值

- 目标：按用户要求将反弹阈值从 SAPIEN 默认的 2.0 m/s 调整为 0.5 m/s。
- 已修改：正式任务 `_base_task.py::setup_scene` 和独立验证脚本 `check_restitution_material_response.py::_measure`，均在创建场景前设置 `scene_config.bounce_threshold = 0.5`。
- 作用范围：本工作副本中经过上述入口创建的场景，包括其他物理因素实验；该设置不依赖 `restitution` 参数是否传入。
- 当前状态：代码已修改，两个文件的 Python AST/编译语法检查、阈值赋值检查及 `git diff --check` 均通过；未提交、未推送、未运行仿真或提交实验作业，服务器运行值未读取。
- 独立源码核查：SAPIEN 3.0.0b1 的 `python/pybind/physx.cpp` 将 `bounce_threshold` 绑定为可读写属性；`wrapper/engine.py::create_scene` 在创建场景之前应用配置；`src/physx/physx_system.cpp` 将该值传给 PhysX 的 `bounceThresholdVelocity`。因此两处赋值位置符合该版本 API。
- 历史结果：现有 summary 中的成功率来自此前实验，不是本次 0.5 m/s 设置下的结果。
- 本次阈值调整完成时的待处理项：当时正式任务仍调用无参数 `setup_scene()`；该断点已在后续单因素修复中解决（见本账本上节）。独立脚本的接触判定及回弹指标仍需修正。
- 下一步：检查 Python 语法及配置赋值位置，运行实验前确认实际场景参数；恢复系数正式扫描前处理上述已知问题并重新建立基线。
