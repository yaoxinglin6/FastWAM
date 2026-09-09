"""CPU regression tests of actual parameter/material code, not contact dynamics."""

from __future__ import annotations

import ast
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from experiments.robotwin import evaluate_robotwin_physics_sweep as sweep


ROOT = Path(__file__).resolve().parents[1]


def material(static, dynamic, restitution):
    return SimpleNamespace(static_friction=static, dynamic_friction=dynamic, restitution=restitution)


def values(value):
    return value.static_friction, value.dynamic_friction, value.restitution


class Scene:
    def __init__(self):
        self.materials = []
        self.builders = []

    def create_physical_material(self, *args):
        result = material(*args)
        self.materials.append(result)
        return result

    def create_actor_builder(self):
        builder = Mock()
        self.builders.append(builder)
        return builder

    def set_timestep(self, value):
        self.timestep = value

    def add_ground(self, value):
        self.ground_height = value

    def set_ambient_light(self, value):
        pass

    def add_directional_light(self, *args, **kwargs):
        pass

    def add_point_light(self, *args, **kwargs):
        pass


def pose(p, q=(1, 0, 0, 0)):
    return SimpleNamespace(p=p, q=q)


def execute_nodes(source, nodes, namespace):
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    module = ast.Module(body=[future, *nodes], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(source), "exec"), namespace)
    return namespace


def load_task():
    source = ROOT / "third_party/RoboTwin/envs/_base_task.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Base_Task")
    cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef)
                and node.name in {"__init__", "_init_task_env_", "setup_scene"}]
    scene = Scene()
    engine = Mock(create_scene=Mock(return_value=scene))
    namespace = {
        "gym": SimpleNamespace(Env=object), "math": math, "np": np,
        "torch": Mock(), "ta": Mock(),
        "sapien": SimpleNamespace(Engine=lambda: engine, SapienRenderer=Mock(),
                                  SceneConfig=SimpleNamespace, render=Mock()),
    }
    task = execute_nodes(source, [cls], namespace)["Base_Task"]()
    # Isolate the real initialization and scene setup from assets and robot control.
    for name in ("create_table_and_wall", "load_robot", "load_camera", "together_open_gripper",
                 "load_actors", "apply_task_physics_overrides"):
        setattr(task, name, Mock())
    task.robot = Mock()
    task.check_stable = Mock(return_value=(True, []))
    task.wall_texture = task.table_texture = None
    return task, scene, engine


def initialize_task(**overrides):
    task, scene, engine = load_task()
    render = ModuleType("sapien.render")
    render.set_global_config = Mock()
    with patch.dict(sys.modules, {"sapien.render": render}):
        task._init_task_env_(domain_randomization={}, render_freq=0, **overrides)
    return task, scene, engine


def load_builders(default):
    source = ROOT / "third_party/RoboTwin/envs/utils/create_actor.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    names = {"_default_collision_material", "preprocess", "create_obj", "create_glb",
             "create_actor", "create_table", "get_glb_or_obj_file"}
    nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    getter = Mock(return_value=default)
    namespace = {"sapien": SimpleNamespace(Scene=Scene, Pose=pose), "np": np,
                 "sapienp": SimpleNamespace(get_default_material=getter),
                 "Path": Path, "json": json, "Actor": lambda actor, metadata: actor}
    return execute_nodes(source, nodes, namespace), getter


class RestitutionForwardingTests(unittest.TestCase):
    def test_actual_initialization_forwards_each_sweep_value_to_scene_material(self):
        for restitution in (0.0, 0.2, 0.8, 1.0):
            with self.subTest(restitution=restitution):
                task, scene, engine = initialize_task(**sweep._setup_overrides("restitution", restitution))
                self.assertEqual(values(scene.default_physical_material), (0.5, 0.5, restitution))
                self.assertTrue(task.use_default_collision_material)
                self.assertEqual(engine.create_scene.call_args.args[0].bounce_threshold, 0.5)

    def test_no_override_keeps_default_and_unrelated_args_are_not_forwarded(self):
        task, scene, _ = initialize_task(static_friction=0.9, dynamic_friction=0.8, timestep=0.05)
        self.assertFalse(task.use_default_collision_material)
        self.assertEqual(values(scene.default_physical_material), (0.5, 0.5, 0.0))
        self.assertEqual(scene.timestep, 1 / 250)

    def test_invalid_restitution_is_rejected_by_sweep_and_before_scene_allocation(self):
        for restitution in (-0.1, 1.2, math.nan, math.inf, -math.inf):
            with self.subTest(restitution=restitution):
                with self.assertRaises(ValueError):
                    sweep._setup_overrides("restitution", restitution)
                task, scene, engine = load_task()
                task.random_light = False
                task.render_freq = 0
                render = ModuleType("sapien.render")
                render.set_global_config = Mock()
                with patch.dict(sys.modules, {"sapien.render": render}), self.assertRaises(ValueError):
                    task.setup_scene(restitution=restitution)
                engine.create_scene.assert_not_called()


class RestitutionMaterialTests(unittest.TestCase):
    def test_replaces_only_restitution_for_direct_scene_and_proxy_without_global_mutation(self):
        for baseline in ((0.3, 0.3, 0.1), (0.7, 0.2, 0.1)):
            default = material(*baseline)
            namespace, _ = load_builders(default)
            for restitution in (0.0, 0.8, 1.0):
                for proxy in (False, True):
                    with self.subTest(baseline=baseline, restitution=restitution, proxy=proxy):
                        scene = Scene()
                        scene.default_physical_material = material(0.5, 0.5, restitution)
                        context = SimpleNamespace(scene=scene, use_default_collision_material=True) if proxy else scene
                        context.use_default_collision_material = True
                        result = namespace["_default_collision_material"](context)
                        self.assertEqual(values(result), (*baseline[:2], restitution))
                        self.assertIsNot(result, default)
                        self.assertEqual(values(default), baseline)
                        self.assertEqual(values(scene.default_physical_material), (0.5, 0.5, restitution))

    def test_disabled_does_not_access_or_change_any_material(self):
        namespace, getter = load_builders(material(0.7, 0.2, 0.1))
        for context in (SimpleNamespace(), SimpleNamespace(use_default_collision_material=False), Scene()):
            self.assertIsNone(namespace["_default_collision_material"](context))
        getter.assert_not_called()

    def test_real_mesh_builders_preserve_friction_and_original_omitted_material_path(self):
        default = material(0.7, 0.2, 0.1)
        namespace, _ = load_builders(default)
        for name in ("create_obj", "create_glb", "create_actor"):
            for convex in (False, True):
                for enabled in (False, True):
                    with self.subTest(builder=name, convex=convex, enabled=enabled):
                        scene = Scene()
                        scene.default_physical_material = material(0.5, 0.5, 0.8)
                        context = SimpleNamespace(scene=scene, table_z_bias=0.0,
                                                  use_default_collision_material=enabled)
                        with patch.object(Path, "exists", return_value=True), patch("builtins.open", side_effect=FileNotFoundError):
                            namespace[name](context, pose([0, 0, 0]), "test_model", convex=convex)
                        method = "add_multiple_convex_collisions_from_file" if convex else "add_nonconvex_collision_from_file"
                        collision = getattr(scene.builders[-1], method)
                        collision.assert_called_once()
                        kwargs = collision.call_args.kwargs
                        if enabled:
                            self.assertEqual(values(kwargs["material"]), (0.7, 0.2, 0.8))
                        else:
                            self.assertNotIn("material", kwargs)
                        self.assertEqual(values(default), (0.7, 0.2, 0.1))

    def test_tabletop_and_legs_retain_their_distinct_original_friction(self):
        namespace, _ = load_builders(material(0.7, 0.2, 0.1))
        for enabled in (False, True):
            scene = Scene()
            scene.default_physical_material = material(0.5, 0.5, 0.8)
            context = SimpleNamespace(scene=scene, table_z_bias=0.0, use_default_collision_material=enabled)
            namespace["create_table"](context, pose([0, 0, 0]), 1.0, 1.0, 0.7)
            calls = scene.builders[-1].add_box_collision.call_args_list
            self.assertEqual(len(calls), 5)
            self.assertIs(calls[0].kwargs["material"], scene.default_physical_material)
            for call in calls[1:]:
                if enabled:
                    self.assertEqual(values(call.kwargs["material"]), (0.7, 0.2, 0.8))
                else:
                    self.assertNotIn("material", call.kwargs)


if __name__ == "__main__":
    unittest.main()
