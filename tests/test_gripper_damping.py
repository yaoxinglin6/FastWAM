"""CPU contract tests; fake articulation drives do not validate SAPIEN dynamics."""

from __future__ import annotations

import ast
import copy
import math
import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from experiments.robotwin import evaluate_robotwin_physics_sweep as sweep
from experiments.robotwin import search_robotwin_seeds as search
from experiments.robotwin import schedule_robotwin_seed_search as scheduler


ROOT = Path(__file__).resolve().parents[1]
EXPERIMENT = ROOT / "docs/experiments/2026-09-09-robotwin-gripper-damping-sweep"


def load_robot_class():
    # Execute the actual class without importing CUDA/planner dependencies.
    source = ROOT / "third_party/RoboTwin/envs/robot/robot.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Robot")
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), cls], type_ignores=[])
    namespace = {
        "math": math, "np": np, "os": os, "deepcopy": copy.deepcopy,
        "ta": SimpleNamespace(setup_logging=lambda *_: None),
        "sapien": SimpleNamespace(Pose=lambda p, q: SimpleNamespace(p=np.array(p, dtype=float), q=np.array(q))),
    }
    exec(compile(ast.fix_missing_locations(module), str(source), "exec"), namespace)
    return namespace["Robot"]


class Joint:
    def __init__(self, name):
        self.name = name
        self.child_link = SimpleNamespace(get_name=lambda: name)
        self.properties = None

    def set_drive_property(self, **kwargs):
        self.properties = kwargs


class Entity:
    def __init__(self):
        self.joints = {name: Joint(name) for name in ("arm", "finger", "mimic")}

    def get_active_joints(self):
        return list(self.joints.values())

    def find_joint_by_name(self, name):
        return self.joints[name]

    def find_link_by_name(self, name):
        return object()

    def set_root_pose(self, pose):
        pass


def embodiment(damping=20.0, stiffness=300.0):
    gripper = {"base": "finger", "mimic": [["mimic", -1.0, 0.0]]}
    return {
        "urdf_path": "robot.urdf", "move_group": ["left", "right"],
        "ee_joints": ["arm", "arm"], "arm_joints_name": [["arm"], ["arm"]],
        "gripper_name": [gripper, gripper], "gripper_bias": 0.0, "gripper_scale": [0.0, 1.0],
        "homestate": [[0.0], [0.0]], "joint_stiffness": 700.0, "joint_damping": 80.0,
        "gripper_stiffness": stiffness, "gripper_damping": damping,
    }


def robot_with_overrides(overrides, use_defaults=False):
    left, right = embodiment(), embodiment(damping=35.0, stiffness=450.0)
    if use_defaults:
        left.pop("gripper_damping")
        right.pop("gripper_damping")
    scene = SimpleNamespace(create_urdf_loader=lambda: SimpleNamespace(load=lambda _: Entity()))
    robot = load_robot_class()(scene, left_embodiment_config=left, right_embodiment_config=right,
                               left_robot_file="left", right_robot_file="right", dual_arm_embodied=False,
                               embodiment_dis=0.0, **overrides)
    robot.init_joints()
    return robot


class GripperDriveTests(unittest.TestCase):
    def test_scaling_reaches_both_base_and_mimic_drives_without_changing_stiffness_or_arms(self):
        for scale in (0.0, 0.5, 1.0, 2.0):
            with self.subTest(scale=scale):
                robot = robot_with_overrides(sweep._setup_overrides("gripper_damping_scale", scale))
                for side, damping, stiffness in (("left", 20.0, 300.0), ("right", 35.0, 450.0)):
                    entity = getattr(robot, f"{side}_entity")
                    self.assertEqual(entity.joints["arm"].properties, {"stiffness": 700.0, "damping": 80.0})
                    for name in ("finger", "mimic"):
                        self.assertEqual(entity.joints[name].properties, {"stiffness": stiffness, "damping": damping * scale})

    def test_default_matches_explicit_one_and_fallback_damping_is_scaled(self):
        default, explicit = robot_with_overrides({}), robot_with_overrides({"gripper_damping_scale": 1.0})
        for side in ("left", "right"):
            for name in ("arm", "finger", "mimic"):
                self.assertEqual(getattr(default, f"{side}_entity").joints[name].properties,
                                 getattr(explicit, f"{side}_entity").joints[name].properties)
        fallback = robot_with_overrides({"gripper_damping_scale": 0.5}, use_defaults=True)
        self.assertEqual(fallback.left_entity.joints["finger"].properties["damping"], 100.0)
        self.assertEqual(fallback.right_entity.joints["mimic"].properties["damping"], 100.0)

    def test_arm_and_gripper_multipliers_are_independent(self):
        robot = robot_with_overrides({"gripper_damping_scale": 0.5, "robot_joint_damping_scale": 3.0})
        self.assertEqual(robot.left_entity.joints["arm"].properties["damping"], 240.0)
        self.assertEqual(robot.left_entity.joints["finger"].properties["damping"], 10.0)

    def test_invalid_scales_rejected_at_both_entry_points(self):
        for value in (-1.0, math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "finite and non-negative"):
                    sweep._setup_overrides("gripper_damping_scale", value)
                with self.assertRaisesRegex(ValueError, "finite and non-negative"):
                    robot_with_overrides({"gripper_damping_scale": value})


class SweepContractTests(unittest.TestCase):
    def test_cli_phases_and_saved_diagnostics_configuration(self):
        original = yaml.safe_load((EXPERIMENT / "successful-seeds/handover_block.yaml").read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            checkpoint = folder / "model.pt"
            checkpoint.touch()
            original["checkpoint"] = str(checkpoint)
            manifest = folder / "seeds.yaml"
            manifest.write_text(yaml.safe_dump(original), encoding="utf-8")
            argv = ["--manifest", str(manifest), "--task-name", "handover_block", "--physics-parameter",
                    "gripper_damping_scale", "--physics-value", "1.5", "--repeats", "1"]
            with patch.object(search, "_resolve_dataset_stats", return_value=folder / "stats.json"), patch.object(search, "_git_revision", return_value="test"):
                base = sweep._make_config(sweep._build_parser().parse_args(argv))
                self.assertEqual(base["phases"], ["clean", "random"])
                self.assertFalse(base["record_gripper_state"])
                selected = sweep._make_config(sweep._build_parser().parse_args(argv + ["--phases", "clean", "--record-gripper-state"]))
                self.assertEqual(selected["phases"], ["clean"])
                self.assertTrue(selected["record_gripper_state"])
                self.assertEqual(selected["setup_overrides"], {"gripper_damping_scale": 1.5})
                self.assertEqual(selected["base_seed"], 42)
                self.assertEqual(selected["manifest"]["phases"]["clean"]["successful_seeds"], original["phases"]["clean"]["successful_seeds"][:3])
                for phases in ("", "clean,clean", "clean,", "unknown"):
                    with self.subTest(phases=phases), self.assertRaises(ValueError):
                        sweep._make_config(sweep._build_parser().parse_args(argv + ["--phases", phases]))

    def test_overrides_reach_environment_args(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            task_config = root / "task_config"
            task_config.mkdir()
            (task_config / "demo_clean.yml").write_text("embodiment: [test]\n", encoding="utf-8")
            (task_config / "_embodiment_config.yml").write_text("test:\n  file_path: test-assets\n", encoding="utf-8")
            official = SimpleNamespace(CONFIGS_PATH=str(task_config), get_embodiment_config=lambda _: embodiment())
            config = {"robotwin_root": str(root), "task_name": "handover_block", "ckpt": "test.pt",
                      "setup_overrides": sweep._setup_overrides("gripper_damping_scale", 2.0)}
            args = search._task_args({"official_eval": official, "yaml": yaml}, config, "clean")
            self.assertEqual(args["gripper_damping_scale"], 2.0)
            self.assertNotIn("robot_joint_damping_scale", args)
            self.assertEqual(args["left_embodiment_config"]["gripper_stiffness"], 300.0)

    def test_ready_job_files_are_parseable_and_have_fixed_panels(self):
        for phase, expected_jobs, seed_count in (("smoke", 3, 1), ("clean", 15, 10), ("random", 15, 10)):
            jobs = scheduler._load_command_jobs(str(EXPERIMENT / f"{phase}-jobs.json"))
            self.assertEqual(len(jobs), expected_jobs)
            for job in jobs:
                argv = scheduler._render_argv(job["argv"], job["name"], "0", Path("test-output"))
                index = next(i for i, item in enumerate(argv) if item.endswith("evaluate_robotwin_physics_sweep.py"))
                args = sweep._build_parser().parse_args(argv[index + 1:])
                task, parameter, tag = sweep._parse_job_name(args.job_name)
                self.assertEqual(parameter, "gripper_damping_scale")
                self.assertIn(sweep._value_tag_to_float(tag), (0.5, 0.75, 1.0, 1.5, 2.0))
                self.assertEqual(args.phases, "random" if phase == "random" else "clean")
                self.assertEqual(args.seed_limit_per_phase, seed_count)
                self.assertEqual(args.repeats, 1)
                self.assertEqual(args.sigma_shift, 5.0)
                self.assertTrue(args.record_gripper_state)
                manifest = yaml.safe_load((ROOT / args.manifest_root / f"{task}.yaml").read_text(encoding="utf-8"))
                self.assertEqual(manifest["policy_seed"], 42)
                seeds = manifest["phases"][args.phases]["successful_seeds"]
                self.assertEqual(len({record["environment_seed"] for record in seeds}), 10)


if __name__ == "__main__":
    unittest.main()
