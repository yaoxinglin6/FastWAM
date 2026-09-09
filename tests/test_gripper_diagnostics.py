import csv
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from experiments.robotwin.gripper_diagnostics import GripperDiagnostics
from experiments.robotwin import search_robotwin_seeds as search


class Joint:
    def __init__(self, name, dof=1):
        self.name, self.dof = name, dof

    def get_name(self):
        return self.name

    def get_dof(self):
        return self.dof

    def get_drive_target(self):
        return [0.8]

    def get_drive_velocity_target(self):
        return [0.05]

    def get_stiffness(self):
        return 1000

    def get_damping(self):
        return 400

    def get_force_limit(self):
        return 30

    def get_drive_mode(self):
        return "force"


class Entity:
    def __init__(self):
        # Non-gripper joint with two DOFs ensures enumeration index is not qpos index.
        self.joints = [Joint("arm", 2), Joint("finger"), Joint("other_arm"), Joint("mimic")]
        self.qpos = [9, 8, 0.2, 7, -0.3]
        self.qvel = [6, 5, 0.02, 4, -0.03]

    def get_active_joints(self):
        return self.joints

    def get_qpos(self):
        return self.qpos

    def get_qvel(self):
        return self.qvel


def make_robot():
    robot = SimpleNamespace()
    for side in ("left", "right"):
        entity = Entity()
        setattr(robot, f"{side}_entity", entity)
        setattr(robot, f"{side}_gripper", [(entity.joints[1], 1, 0), (entity.joints[3], -1, 0)])
        for key, value in {"joint_stiffness": 1200, "joint_damping": 200,
                           "gripper_stiffness": 1000, "gripper_damping": 400}.items():
            setattr(robot, f"{side}_{key}", value)
    return robot


class Env:
    step_lim = 2

    def __init__(self, robot=None):
        self._robot = robot
        self.closed = False
        self.take_action_cnt = 0
        self.eval_success = False

    @property
    def robot(self):
        if self._robot is None:
            raise AssertionError("Disabled diagnostics must not access robot")
        return self._robot

    def setup_demo(self, **kwargs):
        pass

    def set_instruction(self, **kwargs):
        pass

    def get_obs(self):
        return {"unchanged": True}

    def close_env(self, **kwargs):
        self.closed = True


class DiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = {"output_dir": self.temp.name, "skip_get_obs_within_replan": False,
                       "record_gripper_state": True}
        self.args = {"task_config": "demo_clean", "clear_cache_freq": 5}

    def run_rollout(self, env, callback=None):
        def evaluate(env, model, observation):
            self.assertEqual(observation, {"unchanged": True})
            env.take_action_cnt += 1
            env.eval_success = True
            if callback:
                callback(env)
        runtime = {"model": object(), "reset_func": lambda model: None,
                   "eval_func": evaluate, "rollout_count": 0}
        return search._run_rollout(runtime, self.config, env, self.args, 42, "move", 0)

    def test_trace_uses_real_states_offsets_all_gripper_joints_and_closes(self):
        env = Env(make_robot())
        diagnostic = GripperDiagnostics()
        diagnostic.start(env, self.config, self.args, 42, 0)
        env.take_action_cnt = 24
        env.robot.left_entity.qpos[2] = 0.4
        diagnostic.sample(env)
        diagnostic.close()
        self.assertTrue(diagnostic.file.closed)
        self.assertIsNone(diagnostic.error)
        self.assertEqual(diagnostic.metadata["samples"], 2)
        self.assertEqual(diagnostic.metadata["status"], "complete")
        with Path(diagnostic.metadata["csv_path"]).open(newline="") as file:
            rows = list(csv.DictReader(file))
        self.assertEqual(len(rows), 8)
        self.assertEqual({(r["side"], r["role"]) for r in rows},
                         {("left", "base"), ("left", "mimic"), ("right", "base"), ("right", "mimic")})
        self.assertEqual(float(rows[0]["qpos"]), 0.2)
        self.assertEqual(float(rows[0]["qvel"]), 0.02)
        self.assertEqual(float(rows[0]["drive_target"]), 0.8)
        self.assertEqual(float(rows[0]["drive_velocity_target"]), 0.05)
        self.assertEqual(float(rows[1]["qpos"]), -0.3)
        self.assertEqual(float(rows[4]["qpos"]), 0.4)
        self.assertEqual(rows[4]["take_action_cnt"], "24")
        self.assertEqual(diagnostic.effective_drive_properties["right_joint_stiffness"], 1200)
        self.assertEqual(diagnostic.metadata["applied_gripper_drives"][0]["damping"], 400)

    def test_default_disabled_never_accesses_robot_or_writes_trace(self):
        self.config.pop("record_gripper_state")
        result = self.run_rollout(Env())
        self.assertTrue(result["success"])
        self.assertNotIn("gripper_diagnostics", result)
        self.assertEqual(list(Path(self.temp.name).iterdir()), [])

    def test_enabled_rollout_records_setup_and_eval_boundaries(self):
        env = Env(make_robot())
        result = self.run_rollout(env)
        self.assertTrue(result["success"])
        self.assertIsNone(result["diagnostics_error"])
        self.assertEqual(result["gripper_diagnostics"]["samples"], 2)
        self.assertTrue(env.closed)

    def test_policy_exception_closes_trace_and_preserves_policy_error(self):
        diagnostic = GripperDiagnostics()
        def fail(env):
            raise RuntimeError("policy error")
        with patch.object(search, "GripperDiagnostics", return_value=diagnostic):
            result = self.run_rollout(Env(make_robot()), fail)
        self.assertFalse(result["success"])
        self.assertEqual(result["error"], "RuntimeError: policy error")
        self.assertIsNone(result["diagnostics_error"])
        self.assertTrue(diagnostic.file.closed)

    def test_sampling_failure_does_not_change_policy_success(self):
        env = Env(make_robot())
        result = self.run_rollout(env, lambda env: env.robot.left_entity.qpos.pop())
        self.assertTrue(result["success"])
        self.assertIsNone(result["error"])
        self.assertIn("state size", result["diagnostics_error"])
        self.assertEqual(result["gripper_diagnostics"]["status"], "error")
        self.assertEqual(result["gripper_diagnostics"]["samples"], 1)
        self.assertTrue(env.closed)

    def test_unsupported_gripper_dof_is_explicit_and_does_not_change_success(self):
        env = Env(make_robot())
        env.robot.left_entity.joints[1].dof = 2
        result = self.run_rollout(env)
        self.assertTrue(result["success"])
        self.assertIn("requires one DOF", result["diagnostics_error"])
        self.assertEqual(result["gripper_diagnostics"]["samples"], 0)

    def test_output_open_error_does_not_change_success(self):
        with patch("pathlib.Path.open", side_effect=OSError("disk full")):
            result = self.run_rollout(Env(make_robot()))
        self.assertTrue(result["success"])
        self.assertIn("disk full", result["diagnostics_error"])

    def test_close_error_does_not_change_success(self):
        real_open = Path.open
        class CloseError:
            def __init__(self, file):
                self.file = file
            def write(self, value):
                return self.file.write(value)
            def flush(self):
                return self.file.flush()
            def close(self):
                self.file.close()
                raise OSError("close failure")
        with patch("pathlib.Path.open", lambda path, *a, **kw: CloseError(real_open(path, *a, **kw))):
            result = self.run_rollout(Env(make_robot()))
        self.assertTrue(result["success"])
        self.assertIn("close failure", result["diagnostics_error"])

    def test_expert_failure_has_no_policy_trace(self):
        self.config.update(task_name="handover_block", base_seed=42, instruction_type=None)
        runtime = {"official_eval": SimpleNamespace(class_decorator=lambda task: object())}
        with patch.object(search, "_task_args", return_value=self.args), patch.object(
            search, "_run_expert", return_value=({"ok": False, "error": None}, None)
        ):
            result = search._evaluate_candidate(runtime, self.config, "clean", 42, "0")
        self.assertEqual(result["rollouts"], [])
        self.assertEqual(result["gripper_diagnostics"]["status"], "not_run_expert_failed")
        self.assertEqual(list(Path(self.temp.name).iterdir()), [])


if __name__ == "__main__":
    unittest.main()
