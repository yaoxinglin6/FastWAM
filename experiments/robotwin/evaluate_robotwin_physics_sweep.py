"""Evaluate fixed RoboTwin successful seeds under one physics override."""

from __future__ import annotations

import argparse
import csv
import json
import math
import multiprocessing as mp
import os
import queue
import traceback
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

try:
    from . import search_robotwin_seeds as search
    from . import validate_robotwin_successful_seeds as validate
except ImportError:
    import search_robotwin_seeds as search
    import validate_robotwin_successful_seeds as validate


DEFAULT_MANIFEST_ROOT = (
    search.PROJECT_ROOT
    / "docs"
    / "experiments"
    / "2026-08-27-robotwin-all-tasks-seed-search"
    / "successful-seeds"
)

SUPPORTED_PHYSICS_PARAMETERS = {
    "center_of_mass_sphere",
    "center_of_mass_x",
    "center_of_mass_y",
    "center_of_mass_z",
    "object_joint_damping",
    "restitution",
    "robot_joint_damping_scale",
    "gripper_damping_scale",
}


def _value_tag_to_float(raw: str) -> float:
    return float(raw.replace("p", "."))


def _parse_job_name(raw: str) -> tuple[str, str, str]:
    try:
        task_name, physics_parameter, value_tag = raw.rsplit("__", 2)
    except ValueError as error:
        raise ValueError(
            "--job-name must have the form <task>__<physics_parameter>__<value_tag>, "
            "for example open_microwave__object_joint_damping__0p5."
        ) from error
    return task_name, physics_parameter, value_tag


def _load_physics_samples(path: str | None) -> dict[str, Any]:
    if path is None:
        raise ValueError("--physics-samples-file is required for center_of_mass_sphere.")
    samples_path = search._resolve_path(path)
    payload = json.loads(samples_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("Physics samples file must contain a JSON mapping.")
    samples = payload.get("samples", payload)
    if not isinstance(samples, dict):
        raise ValueError("Physics samples file must contain a 'samples' mapping.")
    return samples


def _center_of_mass_sphere_sample(samples: dict[str, Any], setting_id: str) -> dict[str, Any]:
    raw = samples.get(setting_id)
    if not isinstance(raw, dict):
        raise ValueError(f"Physics samples file is missing setting {setting_id!r}.")
    sample = dict(raw)
    missing = [key for key in ("dx", "dy", "dz", "radius") if key not in sample]
    if missing:
        raise ValueError(f"Physics sample {setting_id!r} is missing keys: {missing}")
    for key in ("dx", "dy", "dz", "radius", "radius_min", "radius_max"):
        if key in sample:
            sample[key] = float(sample[key])
    return sample


def _load_seed_panel(path: str | None) -> dict[str, list[int]] | None:
    if path is None:
        return None
    panel_path = search._resolve_path(path)
    payload = json.loads(panel_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("Seed panel must be a JSON mapping from phase to environment_seed list.")
    panel: dict[str, list[int]] = {}
    for phase, seeds in payload.items():
        if not isinstance(phase, str) or not isinstance(seeds, list):
            raise ValueError("Seed panel must map phase names to lists.")
        if any(isinstance(seed, bool) or not isinstance(seed, int) for seed in seeds):
            raise ValueError(f"Seed panel phase {phase!r} contains a non-integer seed.")
        if len(seeds) != len(set(seeds)):
            raise ValueError(f"Seed panel phase {phase!r} contains duplicate seeds.")
        panel[phase] = seeds
    return panel


def _selected_manifest(
    manifest: dict[str, Any],
    seed_limit_per_phase: int,
    seed_panel: dict[str, list[int]] | None,
    unique_seeds_across_phases: bool,
) -> dict[str, Any]:
    if seed_limit_per_phase <= 0:
        raise ValueError("--seed-limit-per-phase must be positive.")
    selected = {**manifest, "phases": {}}
    used_environment_seeds: set[int] = set()
    for phase, phase_config in manifest["phases"].items():
        manifest_seeds = phase_config["successful_seeds"]
        if seed_panel is None:
            seeds = []
            for seed in manifest_seeds:
                environment_seed = int(seed["environment_seed"])
                if unique_seeds_across_phases and environment_seed in used_environment_seeds:
                    continue
                seeds.append(seed)
                if len(seeds) >= seed_limit_per_phase:
                    break
        else:
            selected_seed_values = seed_panel.get(phase)
            if not selected_seed_values:
                raise ValueError(f"Seed panel is missing phase {phase!r}.")
            by_seed = {seed["environment_seed"]: seed for seed in manifest_seeds}
            missing = sorted(set(selected_seed_values) - set(by_seed))
            if missing:
                raise ValueError(f"Seed panel phase {phase!r} contains seeds not present in manifest: {missing}")
            seeds = [by_seed[environment_seed] for environment_seed in selected_seed_values]
            if unique_seeds_across_phases:
                duplicate_seeds = sorted(set(selected_seed_values) & used_environment_seeds)
                if duplicate_seeds:
                    raise ValueError(
                        f"Seed panel phase {phase!r} reuses seeds from earlier phases: {duplicate_seeds}"
                    )
        if len(seeds) < seed_limit_per_phase:
            raise ValueError(
                f"Manifest phase {phase!r} has only {len(seeds)} seeds, "
                f"but {seed_limit_per_phase} were requested."
            )
        selected["phases"][phase] = {**phase_config, "successful_seeds": seeds}
        used_environment_seeds.update(int(seed["environment_seed"]) for seed in seeds)
    return selected


def _setup_overrides(
    physics_parameter: str,
    physics_value: float,
    physics_sample: dict[str, Any] | None = None,
) -> dict[str, float]:
    if physics_parameter == "object_joint_damping":
        return {
            "object_joint_damping": physics_value,
            "object_joint_stiffness": 0.0,
        }
    if physics_parameter == "robot_joint_damping_scale":
        return {"robot_joint_damping_scale": physics_value}
    if physics_parameter == "gripper_damping_scale":
        if not math.isfinite(physics_value) or physics_value < 0:
            raise ValueError("gripper_damping_scale must be finite and non-negative.")
        return {"gripper_damping_scale": physics_value}
    if physics_parameter == "restitution":
        if not math.isfinite(physics_value) or not 0 <= physics_value <= 1:
            raise ValueError("restitution must be finite and between 0 and 1.")
        return {"restitution": physics_value}
    if physics_parameter in {"center_of_mass_x", "center_of_mass_y", "center_of_mass_z"}:
        axis = physics_parameter.rsplit("_", 1)[-1]
        return {f"center_of_mass_offset_{axis}": physics_value}
    if physics_parameter == "center_of_mass_sphere":
        if physics_sample is None:
            raise ValueError("center_of_mass_sphere requires a physics sample.")
        return {
            "center_of_mass_offset_x": float(physics_sample["dx"]),
            "center_of_mass_offset_y": float(physics_sample["dy"]),
            "center_of_mass_offset_z": float(physics_sample["dz"]),
        }
    raise ValueError(f"Unsupported physics parameter: {physics_parameter}")


def _result_path(output_dir: Path, phase: str, environment_seed: int) -> Path:
    return output_dir / "results" / phase / f"seed_{environment_seed}.json"


def _write_summary(output_dir: Path, config: dict[str, Any], records: list[dict[str, Any]]) -> None:
    rows: list[dict[str, Any]] = []
    phases_summary: dict[str, dict[str, Any]] = {}
    overall_expected = 0
    overall_successes = 0
    overall_all_success = 0
    overall_expert_failures = 0
    overall_policy_failures = 0

    for phase in config["phases"]:
        phase_records = sorted(
            (record for record in records if record["phase"] == phase),
            key=lambda record: int(record["environment_seed"]),
        )
        expected_rollouts = len(phase_records) * config["repeats"]
        rollout_successes = sum(record["rollout_successes"] for record in phase_records)
        all_rollouts_success = sum(bool(record["all_rollouts_success"]) for record in phase_records)
        expert_failures = sum(not bool(record["expert"]["ok"]) for record in phase_records)
        policy_failures = sum(
            bool(record["expert"]["ok"]) and record["rollout_successes"] < config["repeats"]
            for record in phase_records
        )
        phases_summary[phase] = {
            "seed_count": len(phase_records),
            "repeats_per_seed": config["repeats"],
            "expected_rollouts": expected_rollouts,
            "successful_rollouts": rollout_successes,
            "rollout_success_rate": rollout_successes / expected_rollouts if expected_rollouts else None,
            "all_rollouts_success_seed_count": all_rollouts_success,
            "all_5_success_seed_count": all_rollouts_success,
            "expert_failures": expert_failures,
            "policy_failures": policy_failures,
        }

        overall_expected += expected_rollouts
        overall_successes += rollout_successes
        overall_all_success += all_rollouts_success
        overall_expert_failures += expert_failures
        overall_policy_failures += policy_failures

        for record in phase_records:
            rows.append(
                {
                    "task_name": record["task_name"],
                    "phase": phase,
                    "physics_parameter": config["physics_parameter"],
                    "physics_setting_id": config["physics_setting_id"],
                    "physics_value": config["physics_value"],
                    "center_of_mass_offset_x": config["setup_overrides"].get("center_of_mass_offset_x"),
                    "center_of_mass_offset_y": config["setup_overrides"].get("center_of_mass_offset_y"),
                    "center_of_mass_offset_z": config["setup_overrides"].get("center_of_mass_offset_z"),
                    "center_of_mass_radius": config["physics_sample"].get("radius") if config["physics_sample"] else None,
                    "radius_min": config["physics_sample"].get("radius_min") if config["physics_sample"] else None,
                    "radius_max": config["physics_sample"].get("radius_max") if config["physics_sample"] else None,
                    "environment_seed": record["environment_seed"],
                    "policy_seed": record["policy_seed"],
                    "expert_ok": record["expert"]["ok"],
                    "rollout_successes": record["rollout_successes"],
                    "expected_rollouts": config["repeats"],
                    "success_rate": record["success_rate"],
                    "all_rollouts_success": record["all_rollouts_success"],
                    "gpu_id": record["gpu_id"],
                    "error": record["error"],
                }
            )

    search._atomic_json(
        output_dir / "summary.json",
        {
            "updated_at": search._now(),
            "task_name": config["task_name"],
            "physics_parameter": config["physics_parameter"],
            "physics_setting_id": config["physics_setting_id"],
            "physics_value": config["physics_value"],
            "physics_sample": config["physics_sample"],
            "setup_overrides": config["setup_overrides"],
            "seed_limit_per_phase": config["seed_limit_per_phase"],
            "repeats_per_seed": config["repeats"],
            "policy_seed": config["base_seed"],
            "phases": phases_summary,
            "overall": {
                "seed_count": len(records),
                "expected_rollouts": overall_expected,
                "successful_rollouts": overall_successes,
                "rollout_success_rate": overall_successes / overall_expected if overall_expected else None,
                "all_rollouts_success_seed_count": overall_all_success,
                "all_rollouts_success_seed_count": overall_all_success,
                "all_5_success_seed_count": overall_all_success,
                "expert_failures": overall_expert_failures,
                "policy_failures": overall_policy_failures,
            },
        },
    )
    with (output_dir / "summary.csv").open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]) if rows else [])
        writer.writeheader()
        writer.writerows(rows)


def _evaluate(
    runtime: dict[str, Any],
    config: dict[str, Any],
    phase: str,
    environment_seed: int,
    gpu_id: str,
) -> dict[str, Any]:
    result = search._evaluate_candidate(runtime, config, phase, environment_seed, gpu_id)
    rollout_successes = sum(bool(rollout["success"]) for rollout in result["rollouts"])
    result["physics_parameter"] = config["physics_parameter"]
    result["physics_setting_id"] = config["physics_setting_id"]
    result["physics_value"] = config["physics_value"]
    result["physics_sample"] = config["physics_sample"]
    result["setup_overrides"] = config["setup_overrides"]
    result["rollout_count"] = len(result["rollouts"])
    result["rollout_successes"] = rollout_successes
    result["expected_rollouts"] = config["repeats"]
    result["success_rate"] = rollout_successes / config["repeats"]
    return result


def _worker(config: dict[str, Any], gpu_id: str, tasks: Any, messages: Any, log_path: str) -> None:
    os.environ["CUDA_VISIBLE_DEVICES"] = gpu_id
    os.environ["PYTHONUNBUFFERED"] = "1"
    worker_log = Path(log_path)
    worker_log.parent.mkdir(parents=True, exist_ok=True)
    with worker_log.open("w", encoding="utf-8") as log_file, redirect_stdout(log_file), redirect_stderr(log_file):
        try:
            runtime = search._prepare_runtime(config)
            print(f"[{search._now()}] model ready on physical GPU {gpu_id}", flush=True)
            messages.put({"kind": "ready", "gpu_id": gpu_id})
        except Exception:
            messages.put({"kind": "worker_error", "gpu_id": gpu_id, "error": traceback.format_exc()})
            return

        while (task := tasks.get()) is not None:
            phase, environment_seed = task
            try:
                messages.put({"kind": "result", "result": _evaluate(runtime, config, phase, environment_seed, gpu_id)})
            except Exception:
                messages.put({"kind": "worker_error", "gpu_id": gpu_id, "error": traceback.format_exc()})
                return
        messages.put({"kind": "done", "gpu_id": gpu_id})


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--job-name", default=None)
    parser.add_argument("--manifest", default=None)
    parser.add_argument("--manifest-root", default=str(DEFAULT_MANIFEST_ROOT))
    parser.add_argument("--phases", default=None, help="Comma-separated manifest phases, e.g. clean or clean,random; default: all.")
    parser.add_argument("--record-gripper-state", action="store_true", help="Record actual gripper states at policy-call boundaries without changing policy observations.")
    parser.add_argument("--task-name", default=None)
    parser.add_argument("--physics-parameter", choices=sorted(SUPPORTED_PHYSICS_PARAMETERS), default=None)
    parser.add_argument("--physics-value", type=float, default=None)
    parser.add_argument("--physics-setting-id", default=None)
    parser.add_argument("--physics-samples-file", default=None)
    parser.add_argument("--seed-limit-per-phase", type=int, default=3)
    parser.add_argument("--seed-panel-file", default=None)
    parser.add_argument("--unique-seeds-across-phases", action="store_true")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--gpu-ids", default="0")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--dataset-stats-path", default=None)
    parser.add_argument("--sim-task", default="robotwin_uncond_3cam_384_1e-4")
    parser.add_argument("--mixed-precision", choices=["no", "fp16", "bf16"], default="bf16")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--action-horizon", type=int, default=None)
    parser.add_argument("--replan-steps", type=int, default=24)
    parser.add_argument("--num-inference-steps", type=int, default=None)
    parser.add_argument("--sigma-shift", type=float, default=None)
    parser.add_argument("--text-cfg-scale", type=float, default=1.0)
    parser.add_argument("--negative-prompt", default="")
    parser.add_argument("--rand-device", default="cpu")
    parser.add_argument("--skip-get-obs-within-replan", action=argparse.BooleanOptionalAction, default=True)
    return parser


def _make_config(args: argparse.Namespace) -> dict[str, Any]:
    if args.job_name:
        task_name, physics_parameter, physics_token = _parse_job_name(args.job_name)
    else:
        if not args.task_name or not args.physics_parameter:
            raise ValueError("Pass either --job-name or both of --task-name and --physics-parameter.")
        task_name = args.task_name
        physics_parameter = args.physics_parameter
        if physics_parameter == "center_of_mass_sphere":
            if not args.physics_setting_id:
                raise ValueError("--physics-setting-id is required for center_of_mass_sphere.")
            physics_token = args.physics_setting_id
        else:
            if args.physics_value is None:
                raise ValueError("--physics-value is required for scalar physics parameters.")
            physics_token = str(args.physics_value)

    if physics_parameter not in SUPPORTED_PHYSICS_PARAMETERS:
        raise ValueError(f"Unsupported physics parameter: {physics_parameter}")
    if args.repeats <= 0:
        raise ValueError("--repeats must be positive.")

    physics_sample = None
    if physics_parameter == "center_of_mass_sphere":
        samples = _load_physics_samples(args.physics_samples_file)
        physics_setting_id = physics_token
        physics_sample = _center_of_mass_sphere_sample(samples, physics_setting_id)
        physics_value = float(physics_sample["radius"])
    else:
        physics_setting_id = str(physics_token)
        physics_value = _value_tag_to_float(str(physics_token))

    manifest_path = search._resolve_path(args.manifest) if args.manifest else search._resolve_path(
        Path(args.manifest_root) / f"{task_name}.yaml"
    )
    manifest = validate._load_manifest(manifest_path)
    if manifest["task_name"] != task_name:
        raise ValueError(f"Manifest task_name {manifest['task_name']!r} does not match {task_name!r}.")
    if args.phases is not None:
        phases = [phase.strip() for phase in args.phases.split(",")]
        if not all(phases) or len(phases) != len(set(phases)):
            raise ValueError("--phases must contain non-empty, distinct phase names.")
        missing = set(phases) - set(manifest["phases"])
        if missing:
            raise ValueError(f"Requested phases not present in manifest: {sorted(missing)}")
        manifest = {**manifest, "phases": {phase: manifest["phases"][phase] for phase in phases}}
    seed_panel = _load_seed_panel(args.seed_panel_file)
    manifest = _selected_manifest(manifest, args.seed_limit_per_phase, seed_panel, args.unique_seeds_across_phases)

    checkpoint = search._resolve_path(manifest["checkpoint"])
    if not checkpoint.is_file():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint}")
    robotwin_root = search.PROJECT_ROOT / "third_party" / "RoboTwin"
    if not robotwin_root.is_dir():
        raise FileNotFoundError(f"RoboTwin root not found: {robotwin_root}")

    if args.output_dir:
        output_dir = search._resolve_path(args.output_dir)
    else:
        value_tag = str(physics_value).replace(".", "p")
        output_dir = (
            search.PROJECT_ROOT
            / "evaluate_results"
            / "robotwin"
            / "physics_sweep"
            / f"{task_name}_{physics_parameter}_{value_tag}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )

    return {
        "task_name": task_name,
        "phases": list(manifest["phases"]),
        "gpu_ids": validate._parse_gpu_ids(args.gpu_ids),
        "base_seed": manifest["policy_seed"],
        "ckpt": str(checkpoint),
        "dataset_stats_path": str(search._resolve_dataset_stats(checkpoint, args.dataset_stats_path)),
        "output_dir": str(output_dir),
        "robotwin_root": str(robotwin_root.resolve()),
        "sim_cfg_path": str((search.PROJECT_ROOT / "configs" / "sim_robotwin.yaml").resolve()),
        "sim_task": args.sim_task,
        "instruction_type": None,
        "mixed_precision": args.mixed_precision,
        "device": args.device,
        "action_horizon": args.action_horizon,
        "replan_steps": args.replan_steps,
        "num_inference_steps": args.num_inference_steps,
        "sigma_shift": args.sigma_shift,
        "text_cfg_scale": args.text_cfg_scale,
        "negative_prompt": args.negative_prompt,
        "rand_device": args.rand_device,
        "skip_get_obs_within_replan": args.skip_get_obs_within_replan,
        "record_gripper_state": args.record_gripper_state,
        "manifest_path": str(manifest_path),
        "manifest": manifest,
        "seed_limit_per_phase": args.seed_limit_per_phase,
        "seed_panel_file": args.seed_panel_file,
        "unique_seeds_across_phases": args.unique_seeds_across_phases,
        "repeats": args.repeats,
        "physics_parameter": physics_parameter,
        "physics_setting_id": physics_setting_id,
        "physics_value": physics_value,
        "physics_samples_file": args.physics_samples_file,
        "physics_sample": physics_sample,
        "setup_overrides": _setup_overrides(physics_parameter, physics_value, physics_sample),
        "git_revision": search._git_revision(),
    }


def main() -> None:
    args = _build_parser().parse_args()
    config = _make_config(args)
    output_dir = Path(config["output_dir"])
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"Output directory already contains data: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    search._ensure_policy_symlink(Path(config["robotwin_root"]))
    search._atomic_json(output_dir / "run_config.json", {"created_at": search._now(), **config})

    tasks_to_run = [
        (phase, seed["environment_seed"])
        for phase, phase_config in config["manifest"]["phases"].items()
        for seed in phase_config["successful_seeds"]
    ]
    print(
        f"physics sweep task={config['task_name']} parameter={config['physics_parameter']} "
        f"value={config['physics_value']} seeds={len(tasks_to_run)} "
        f"repeats={config['repeats']} gpus={','.join(config['gpu_ids'])}",
        flush=True,
    )

    context = mp.get_context("spawn")
    tasks = context.Queue()
    messages = context.Queue()
    for task in tasks_to_run:
        tasks.put(task)
    for _ in config["gpu_ids"]:
        tasks.put(None)

    workers: list[mp.Process] = []
    worker_gpu_ids: dict[int, str] = {}
    for gpu_id in config["gpu_ids"]:
        worker = context.Process(
            target=_worker,
            args=(config, gpu_id, tasks, messages, str(output_dir / "logs" / f"worker_gpu{gpu_id}.log")),
        )
        worker.start()
        workers.append(worker)
        assert worker.pid is not None
        worker_gpu_ids[worker.pid] = gpu_id

    records: list[dict[str, Any]] = []
    finished_gpu_ids: set[str] = set()
    worker_error: str | None = None
    try:
        while len(finished_gpu_ids) < len(workers):
            try:
                message = messages.get(timeout=10)
            except queue.Empty:
                exited = [
                    worker
                    for worker in workers
                    if worker.exitcode is not None and worker_gpu_ids[worker.pid] not in finished_gpu_ids
                ]
                if exited:
                    worker_error = "worker exited before reporting completion: " + ", ".join(
                        f"pid={worker.pid},exitcode={worker.exitcode}" for worker in exited
                    )
                    break
                continue
            if message["kind"] == "ready":
                print(f"worker ready gpu={message['gpu_id']}", flush=True)
            elif message["kind"] == "result":
                result = message["result"]
                records.append(result)
                search._atomic_json(_result_path(output_dir, result["phase"], result["environment_seed"]), result)
                print(
                    f"phase={result['phase']} seed={result['environment_seed']} "
                    f"success_rate={result['success_rate']:.1%} "
                    f"({result['rollout_successes']}/{result['expected_rollouts']})",
                    flush=True,
                )
            elif message["kind"] == "worker_error":
                worker_error = f"worker GPU {message['gpu_id']} failed:\n{message['error']}"
                break
            elif message["kind"] == "done":
                finished_gpu_ids.add(message["gpu_id"])
    finally:
        if worker_error:
            for worker in workers:
                if worker.is_alive():
                    worker.terminate()
        for worker in workers:
            worker.join(timeout=30)
            if worker.is_alive():
                worker.kill()
                worker.join()

    _write_summary(output_dir, config, records)
    if worker_error:
        raise RuntimeError(worker_error)
    if len(records) != len(tasks_to_run):
        raise RuntimeError(f"Expected {len(tasks_to_run)} results, received {len(records)}.")
    print(f"finished summary={output_dir / 'summary.json'}", flush=True)


if __name__ == "__main__":
    main()
