"""Measure whether RoboTwin collision creation paths respond to restitution."""

from __future__ import annotations

import argparse
import csv
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ROBOTWIN_ROOT = PROJECT_ROOT / "third_party" / "RoboTwin"


@dataclass
class SceneProxy:
    scene: Any
    table_z_bias: float = 0.0
    use_default_collision_material: bool = True


def _parse_values(raw: str) -> list[float]:
    return [float(item.strip()) for item in raw.split(",") if item.strip()]


def _measure(kind: str, restitution: float, steps: int) -> dict[str, Any]:
    os.chdir(ROBOTWIN_ROOT)
    if str(ROBOTWIN_ROOT) not in sys.path:
        sys.path.insert(0, str(ROBOTWIN_ROOT))

    import sapien.core as sapien
    from envs.utils import create_actor, create_box

    engine = sapien.Engine()
    scene = engine.create_scene(sapien.SceneConfig())
    scene.set_timestep(1 / 250)
    scene.default_physical_material = scene.create_physical_material(0.5, 0.5, restitution)
    scene.add_ground(0)
    proxy = SceneProxy(scene=scene)
    pose = sapien.Pose([0, 0, 0.5])

    if kind == "box":
        actor = create_box(proxy, pose=pose, half_size=[0.025, 0.025, 0.025], color=(1, 0, 0), name="test_box")
    elif kind == "bowl":
        actor = create_actor(proxy, pose=pose, modelname="002_bowl", model_id=3, convex=True)
    elif kind == "can":
        actor = create_actor(proxy, pose=pose, modelname="071_can", model_id=0, convex=True)
    else:
        raise ValueError(f"Unsupported kind: {kind}")

    start_z = float(actor.get_pose().p[2])
    min_z = start_z
    max_z_after_contact = 0.0
    max_upward_velocity_after_contact = 0.0
    contacted = False
    previous_z = start_z

    for _ in range(steps):
        scene.step()
        z = float(actor.get_pose().p[2])
        velocity_z = (z - previous_z) * 250
        previous_z = z
        min_z = min(min_z, z)
        if z < start_z - 0.1:
            contacted = True
        if contacted:
            max_z_after_contact = max(max_z_after_contact, z)
            max_upward_velocity_after_contact = max(max_upward_velocity_after_contact, velocity_z)

    return {
        "kind": kind,
        "restitution": restitution,
        "start_z": start_z,
        "min_z": min_z,
        "max_z_after_contact": max_z_after_contact,
        "bounce_height": max(0.0, max_z_after_contact - min_z),
        "max_upward_velocity_after_contact": max_upward_velocity_after_contact,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--values", default="0,0.2,0.5,0.8,1.0,1.2,1.5,2.0")
    parser.add_argument("--kinds", default="box,bowl,can")
    parser.add_argument("--steps", type=int, default=750)
    parser.add_argument("--output-dir", default=None)
    args = parser.parse_args()

    output_dir = Path(args.output_dir) if args.output_dir else (
        PROJECT_ROOT / "evaluate_results" / "robotwin" / "physics_sweep" / "restitution_material_response"
    )
    if not output_dir.is_absolute():
        output_dir = PROJECT_ROOT / output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = [
        _measure(kind, value, args.steps)
        for kind in [item.strip() for item in args.kinds.split(",") if item.strip()]
        for value in _parse_values(args.values)
    ]

    csv_path = output_dir / "restitution_material_response.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(csv_path)


if __name__ == "__main__":
    main()
