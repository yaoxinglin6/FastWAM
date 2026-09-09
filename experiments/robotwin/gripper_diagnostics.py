"""Optional, read-only gripper traces at policy-call boundaries (not physics substeps)."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any


def _scalar(value: Any) -> float:
    # SAPIEN state/target vectors may expose numpy scalars or one-element arrays.
    return float(value.item() if hasattr(value, "item") else value)


class GripperDiagnostics:
    """Keep diagnostic failures separate from policy success and always close the CSV."""

    def __init__(self) -> None:
        self.file = None
        self.writer = None
        self.sides = []
        self.error: str | None = None
        self.effective_drive_properties: dict[str, float] = {}
        self.metadata: dict[str, Any] = {
            "status": "not_started",
            "sampling": "after_setup_and_after_each_policy_eval_call_not_physics_substeps",
            "position_units": "native_joint_units_radians_or_meters",
            "velocity_units": "native_joint_units_per_second",
            "samples": 0,
            "csv_path": None,
            "applied_gripper_drives": [],
        }

    def _fail(self, stage: str, error: Exception) -> None:
        message = f"{stage}: {type(error).__name__}: {error}"
        self.error = f"{self.error}; {message}" if self.error else message
        self.metadata["status"] = "error"
        print(f"Gripper diagnostics error: {message}", flush=True)

    def start(self, env: Any, config: dict[str, Any], args: dict[str, Any], seed: int, repeat: int) -> None:
        try:
            robot = env.robot
            self.metadata["gripper_damping_scale"] = float(getattr(robot, "gripper_damping_scale", 1.0))
            for side in ("left", "right"):
                for kind in ("joint", "gripper"):
                    for gain in ("stiffness", "damping"):
                        key = f"{side}_{kind}_{gain}"
                        self.effective_drive_properties[key] = float(getattr(robot, key))
                entity = getattr(robot, f"{side}_entity")
                offsets = {}
                offset = 0
                for joint in entity.get_active_joints():
                    name = joint.get_name()
                    if name in offsets:
                        raise ValueError(f"Duplicate active joint name: {name}")
                    dof = joint.get_dof()
                    offsets[name] = (offset, dof)
                    offset += dof
                joints = []
                for index, (joint, _, _) in enumerate(getattr(robot, f"{side}_gripper")):
                    name = joint.get_name()
                    qindex, dof = offsets[name]
                    if dof != 1 or joint.get_dof() != 1:
                        raise ValueError(f"Gripper diagnostic requires one DOF: {side}/{name}, dof={dof}")
                    role = "base" if index == 0 else "mimic"
                    joints.append((joint, name, qindex, role))
                    self.metadata["applied_gripper_drives"].append({
                        "side": side, "joint_name": name, "role": role,
                        "stiffness": float(joint.get_stiffness()),
                        "damping": float(joint.get_damping()),
                        "force_limit": float(joint.get_force_limit()),
                        "drive_mode": str(joint.get_drive_mode()),
                    })
                if not joints:
                    raise ValueError(f"No {side} gripper joints found")
                self.sides.append((side, entity, offset, joints))
            path = Path(config["output_dir"]) / "diagnostics" / args["task_config"] / f"seed_{seed}_repeat_{repeat}.csv"
            path.parent.mkdir(parents=True, exist_ok=True)
            self.file = path.open("w", encoding="utf-8", newline="")
            self.metadata["csv_path"] = str(path)
            self.writer = csv.DictWriter(self.file, fieldnames=[
                "sample_index", "boundary", "take_action_cnt", "side", "joint_name", "role",
                "qpos", "qvel", "drive_target", "drive_velocity_target",
            ])
            self.writer.writeheader()
            self.metadata["status"] = "recording"
            self.sample(env, "after_setup")
        except Exception as error:
            self._fail("start", error)
            self.close()

    def sample(self, env: Any, boundary: str = "after_policy_eval") -> None:
        if self.error or self.writer is None:
            return
        try:
            rows = []
            for side, entity, dof, joints in self.sides:
                qpos, qvel = entity.get_qpos(), entity.get_qvel()
                if len(qpos) != dof or len(qvel) != dof:
                    raise ValueError(f"{side} state size does not match cumulative active-joint DOFs")
                for joint, name, offset, role in joints:
                    target = joint.get_drive_target()
                    velocity_target = joint.get_drive_velocity_target()
                    if len(target) != 1 or len(velocity_target) != 1:
                        raise ValueError(f"Expected one-dimensional drive targets: {side}/{name}")
                    rows.append({
                        "sample_index": self.metadata["samples"], "boundary": boundary,
                        "take_action_cnt": env.take_action_cnt, "side": side, "joint_name": name,
                        "role": role, "qpos": _scalar(qpos[offset]), "qvel": _scalar(qvel[offset]),
                        "drive_target": _scalar(target[0]), "drive_velocity_target": _scalar(velocity_target[0]),
                    })
            self.writer.writerows(rows)
            self.file.flush()
            self.metadata["samples"] += 1
        except Exception as error:
            self._fail("sample", error)
            self.close()

    def close(self) -> None:
        try:
            if self.file is not None:
                self.file.close()
            if self.metadata["status"] == "recording":
                self.metadata["status"] = "complete"
        except Exception as error:
            self._fail("close", error)
        finally:
            self.writer = None
