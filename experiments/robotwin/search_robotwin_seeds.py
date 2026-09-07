"""Bounded, multi-GPU RoboTwin seed search for one FastWAM policy.

By default, ``--seed`` is the RoboTwin external/base seed and candidate
environment seeds begin at ``100000 * (1 + seed)``. An explicit
``--environment-seed-start`` changes only the candidate sequence; policy
sampling continues to use ``--seed``.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import multiprocessing as mp
import os
import queue
import subprocess
import sys
import time
import traceback
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CKPT = PROJECT_ROOT / "checkpoints" / "fastwam_release" / "robotwin_uncond_3cam_384.pt"
PHASE_TO_TASK_CONFIG = {"clean": "demo_clean", "random": "demo_randomized"}
PHASE_TO_INSTRUCTION = {"clean": "seen", "random": "unseen"}


def _now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _atomic_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def _atomic_yaml(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(yaml.safe_dump(payload, allow_unicode=True, sort_keys=False), encoding="utf-8")
    os.replace(temporary, path)


def _environment_seed_start(base_seed: int) -> int:
    return 100000 * (1 + base_seed)


def _parse_csv(raw: str) -> list[str]:
    values = [item.strip() for item in raw.split(",") if item.strip()]
    if not values:
        raise ValueError("Expected a non-empty comma-separated value list.")
    return values


def _resolve_path(raw: str | Path) -> Path:
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path.resolve()


def _git_revision() -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=PROJECT_ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _ensure_policy_symlink(robotwin_root: Path) -> None:
    source = (PROJECT_ROOT / "experiments" / "robotwin" / "fastwam_policy").resolve()
    target = robotwin_root / "policy" / "fastwam_policy"
    if not source.is_dir():
        raise FileNotFoundError(f"FastWAM RoboTwin policy source is missing: {source}")
    if target.is_symlink():
        if target.resolve() != source:
            raise RuntimeError(f"Policy symlink conflict: {target} -> {target.resolve()}, expected {source}")
        return
    if target.exists():
        raise RuntimeError(f"Policy path exists and is not the expected symlink: {target}")
    target.symlink_to(source, target_is_directory=True)


def _resolve_dataset_stats(ckpt: Path, explicit: str | None) -> Path:
    candidates: list[Path] = []
    if explicit:
        candidates.append(_resolve_path(explicit))
    candidates.extend(
        [
            ckpt.with_name(f"{ckpt.stem}_dataset_stats.json"),
            ckpt.parent / "dataset_stats.json",
        ]
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    raise FileNotFoundError(
        "Dataset statistics file not found. Pass --dataset-stats-path explicitly or place "
        f"{ckpt.stem}_dataset_stats.json next to the checkpoint."
    )


def _result_path(output_dir: Path, phase: str, environment_seed: int) -> Path:
    return output_dir / "results" / phase / f"seed_{environment_seed}.json"


def _all_result_records(output_dir: Path, phases: list[str]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for phase in phases:
        for path in sorted((output_dir / "results" / phase).glob("seed_*.json")):
            try:
                records.append(json.loads(path.read_text(encoding="utf-8")))
            except json.JSONDecodeError:
                continue
    return records


def _write_summary(output_dir: Path, config: dict[str, Any], selected: dict[str, list[int]]) -> None:
    records = _all_result_records(output_dir, config["phases"])
    rows: list[dict[str, Any]] = []
    phases_summary: dict[str, dict[str, Any]] = {}
    for phase in config["phases"]:
        phase_records = [record for record in records if record.get("phase") == phase]
        passed = [record for record in phase_records if record.get("all_rollouts_success")]
        phases_summary[phase] = {
            "candidate_limit": config["max_seed_attempts"],
            "attempted": len(phase_records),
            "passed_all_repeats": len(passed),
            "selected_environment_seeds": selected[phase],
            "target_good_seeds": config["target_good_seeds"],
            "target_reached": len(selected[phase]) >= config["target_good_seeds"],
        }
        for record in phase_records:
            rollouts = record.get("rollouts", [])
            rows.append(
                {
                    "phase": phase,
                    "environment_seed": record.get("environment_seed"),
                    "base_seed": record.get("base_seed"),
                    "policy_seed": record.get("policy_seed"),
                    "expert_ok": record.get("expert", {}).get("ok"),
                    "rollout_successes": sum(bool(item.get("success")) for item in rollouts),
                    "rollout_count": len(rollouts),
                    "all_rollouts_success": record.get("all_rollouts_success"),
                    "selected": record.get("selected", False),
                    "gpu_id": record.get("gpu_id"),
                    "error": record.get("error"),
                }
            )

    _atomic_json(
        output_dir / "summary.json",
        {
            "updated_at": _now(),
            "task_name": config["task_name"],
            "phases": phases_summary,
        },
    )
    with (output_dir / "summary.csv").open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "phase",
                "environment_seed",
                "base_seed",
                "policy_seed",
                "expert_ok",
                "rollout_successes",
                "rollout_count",
                "all_rollouts_success",
                "selected",
                "gpu_id",
                "error",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)


def _write_successful_seeds(output_dir: Path, config: dict[str, Any]) -> None:
    records = _all_result_records(output_dir, config["phases"])
    phases: dict[str, dict[str, Any]] = {}
    for phase in config["phases"]:
        selected_records = sorted(
            (
                record
                for record in records
                if record.get("phase") == phase
                and record.get("selected")
                and record.get("all_rollouts_success")
            ),
            key=lambda record: int(record["environment_seed"]),
        )
        example = selected_records[0] if selected_records else {}
        phases[phase] = {
            "task_config": example.get("task_config", PHASE_TO_TASK_CONFIG[phase]),
            "instruction_type": example.get(
                "instruction_type", config["instruction_type"] or PHASE_TO_INSTRUCTION[phase]
            ),
            "successful_seeds": [
                {
                    "environment_seed": record["environment_seed"],
                    "consecutive_successes": len(record["rollouts"]),
                }
                for record in selected_records
            ],
        }

    _atomic_yaml(
        output_dir / "successful-seeds.yaml",
        {
            "task_name": config["task_name"],
            "checkpoint": os.path.relpath(config["ckpt"], PROJECT_ROOT),
            "policy_seed": config["base_seed"],
            "phases": phases,
        },
    )


def _safe_close(env: Any, *, clear_cache: bool = False) -> None:
    try:
        env.close_env(clear_cache=clear_cache)
    except Exception:
        pass


def _prepare_runtime(config: dict[str, Any]) -> dict[str, Any]:
    robotwin_root = Path(config["robotwin_root"])
    os.chdir(robotwin_root)
    if str(robotwin_root) not in sys.path:
        sys.path.insert(0, str(robotwin_root))
    if str(PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(PROJECT_ROOT))

    from script import eval_policy as official_eval

    task_config_path = robotwin_root / "task_config" / PHASE_TO_TASK_CONFIG[config["phases"][0]]
    if not task_config_path.with_suffix(".yml").exists():
        raise FileNotFoundError(f"Task configuration not found: {task_config_path.with_suffix('.yml')}")

    policy_args = {
        "task_name": config["task_name"],
        "task_config": PHASE_TO_TASK_CONFIG[config["phases"][0]],
        "ckpt_setting": config["ckpt"],
        "seed": config["base_seed"],
        "policy_name": "fastwam_policy",
        "instruction_type": config["instruction_type"] or PHASE_TO_INSTRUCTION[config["phases"][0]],
        "sim_cfg_path": config["sim_cfg_path"],
        "sim_task": config["sim_task"],
        "mixed_precision": config["mixed_precision"],
        "device": config["device"],
        "dataset_stats_path": config["dataset_stats_path"],
        "action_horizon": config["action_horizon"],
        "replan_steps": config["replan_steps"],
        "num_inference_steps": config["num_inference_steps"],
        "sigma_shift": config["sigma_shift"],
        "text_cfg_scale": config["text_cfg_scale"],
        "negative_prompt": config["negative_prompt"],
        "rand_device": config["rand_device"],
        "tiled": False,
        "timing_enabled": False,
    }
    get_model = official_eval.eval_function_decorator("fastwam_policy", "get_model")
    return {
        "official_eval": official_eval,
        "yaml": yaml,
        "model": get_model(policy_args),
        "eval_func": official_eval.eval_function_decorator("fastwam_policy", "eval"),
        "reset_func": official_eval.eval_function_decorator("fastwam_policy", "reset_model"),
        "rollout_count": 0,
    }


def _task_args(runtime: dict[str, Any], config: dict[str, Any], phase: str) -> dict[str, Any]:
    official_eval = runtime["official_eval"]
    task_config = PHASE_TO_TASK_CONFIG[phase]
    with (Path(config["robotwin_root"]) / "task_config" / f"{task_config}.yml").open(
        "r", encoding="utf-8"
    ) as file:
        args = runtime["yaml"].safe_load(file)

    args["task_name"] = config["task_name"]
    args["task_config"] = task_config
    args["ckpt_setting"] = config["ckpt"]
    args["policy_name"] = "fastwam_policy"
    args["eval_mode"] = True
    args["render_freq"] = 0
    args["eval_video_log"] = False
    for key, value in config.get("setup_overrides", {}).items():
        if value is not None:
            args[key] = value

    embodiment_config_path = Path(official_eval.CONFIGS_PATH) / "_embodiment_config.yml"
    with embodiment_config_path.open("r", encoding="utf-8") as file:
        embodiments = runtime["yaml"].safe_load(file)
    embodiment = args["embodiment"]
    if len(embodiment) == 1:
        robot_file = embodiments[embodiment[0]]["file_path"]
        args["left_robot_file"] = robot_file
        args["right_robot_file"] = robot_file
        args["dual_arm_embodied"] = True
    elif len(embodiment) == 3:
        args["left_robot_file"] = embodiments[embodiment[0]]["file_path"]
        args["right_robot_file"] = embodiments[embodiment[1]]["file_path"]
        args["embodiment_dis"] = embodiment[2]
        args["dual_arm_embodied"] = False
    else:
        raise ValueError(f"Unsupported embodiment configuration: {embodiment}")
    args["left_embodiment_config"] = official_eval.get_embodiment_config(args["left_robot_file"])
    args["right_embodiment_config"] = official_eval.get_embodiment_config(args["right_robot_file"])
    return args


def _run_expert(env: Any, args: dict[str, Any], environment_seed: int) -> tuple[dict[str, Any], dict[str, Any] | None]:
    try:
        env.setup_demo(now_ep_num=0, seed=environment_seed, is_test=True, **args)
        episode_info = env.play_once()
        ok = bool(env.plan_success and env.check_success())
        return {"ok": ok, "error": None}, episode_info
    except Exception as error:
        return {"ok": False, "error": f"{type(error).__name__}: {error}"}, None
    finally:
        _safe_close(env)


def _pick_instruction(runtime: dict[str, Any], config: dict[str, Any], phase: str, episode_info: dict[str, Any]) -> str:
    official_eval = runtime["official_eval"]
    instruction_type = config["instruction_type"] or PHASE_TO_INSTRUCTION[phase]
    descriptions = official_eval.generate_episode_descriptions(
        config["task_name"], [episode_info["info"]], max_descriptions=1
    )
    choices = descriptions[0][instruction_type]
    if not choices:
        raise ValueError(f"No {instruction_type} instruction generated for {config['task_name']}")
    return str(official_eval.np.random.choice(choices))


def _run_rollout(
    runtime: dict[str, Any],
    config: dict[str, Any],
    env: Any,
    args: dict[str, Any],
    environment_seed: int,
    instruction: str,
    repeat_index: int,
) -> dict[str, Any]:
    success = False
    error_text: str | None = None
    try:
        env.setup_demo(now_ep_num=repeat_index, seed=environment_seed, is_test=True, **args)
        env.set_instruction(instruction=instruction)
        runtime["reset_func"](runtime["model"])
        while env.take_action_cnt < env.step_lim:
            observation = None
            if not config["skip_get_obs_within_replan"] or not hasattr(runtime["model"], "should_request_observation"):
                observation = env.get_obs()
            elif runtime["model"].should_request_observation():
                observation = env.get_obs()
            runtime["eval_func"](env, runtime["model"], observation)
            if env.eval_success:
                success = True
                break
    except Exception as error:
        error_text = f"{type(error).__name__}: {error}"
    finally:
        runtime["rollout_count"] += 1
        clear_cache = runtime["rollout_count"] % int(args["clear_cache_freq"]) == 0
        _safe_close(env, clear_cache=clear_cache)

    return {
        "repeat_index": repeat_index,
        "success": success,
        "steps": getattr(env, "take_action_cnt", None),
        "error": error_text,
    }


def _evaluate_candidate(runtime: dict[str, Any], config: dict[str, Any], phase: str, environment_seed: int, gpu_id: str) -> dict[str, Any]:
    started_at = _now()
    args = _task_args(runtime, config, phase)
    env = runtime["official_eval"].class_decorator(config["task_name"])
    expert, episode_info = _run_expert(env, args, environment_seed)
    result: dict[str, Any] = {
        "task_name": config["task_name"],
        "phase": phase,
        "task_config": PHASE_TO_TASK_CONFIG[phase],
        "instruction_type": config["instruction_type"] or PHASE_TO_INSTRUCTION[phase],
        "base_seed": config["base_seed"],
        "policy_seed": config["base_seed"],
        "environment_seed": environment_seed,
        "gpu_id": gpu_id,
        "started_at": started_at,
        "expert": expert,
        "instruction": None,
        "rollouts": [],
        "all_rollouts_success": False,
        "selected": False,
        "error": None,
    }
    if not expert["ok"] or episode_info is None:
        result["error"] = expert["error"] or "expert planning did not reach task success"
        result["finished_at"] = _now()
        return result

    try:
        instruction = _pick_instruction(runtime, config, phase, episode_info)
        result["instruction"] = instruction
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
        result["finished_at"] = _now()
        return result

    for repeat_index in range(config["repeats"]):
        result["rollouts"].append(
            _run_rollout(runtime, config, env, copy.deepcopy(args), environment_seed, instruction, repeat_index)
        )
    result["all_rollouts_success"] = all(item["success"] for item in result["rollouts"])
    result["finished_at"] = _now()
    return result


def _worker(
    config: dict[str, Any],
    gpu_id: str,
    tasks: Any,
    messages: Any,
    phase_stops: dict[str, Any],
    log_path: str,
) -> None:
    os.environ["CUDA_VISIBLE_DEVICES"] = gpu_id
    os.environ["PYTHONUNBUFFERED"] = "1"
    worker_log = Path(log_path)
    worker_log.parent.mkdir(parents=True, exist_ok=True)
    with worker_log.open("w", encoding="utf-8") as log_file, redirect_stdout(log_file), redirect_stderr(log_file):
        try:
            runtime = _prepare_runtime(config)
            print(f"[{_now()}] model ready on physical GPU {gpu_id}", flush=True)
            messages.put({"kind": "ready", "gpu_id": gpu_id})
        except Exception:
            messages.put({"kind": "worker_error", "gpu_id": gpu_id, "error": traceback.format_exc()})
            return

        while True:
            task = tasks.get()
            if task is None:
                break
            phase, environment_seed = task
            if phase_stops[phase].is_set():
                continue
            try:
                result = _evaluate_candidate(runtime, config, phase, environment_seed, gpu_id)
            except Exception:
                messages.put({"kind": "worker_error", "gpu_id": gpu_id, "error": traceback.format_exc()})
                return
            messages.put({"kind": "candidate", "result": result})
        messages.put({"kind": "done", "gpu_id": gpu_id})


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-name", default="click_bell")
    parser.add_argument("--phases", default="clean,random", help="Comma-separated subset of clean,random.")
    parser.add_argument("--gpu-ids", default="2,3,4,5", help="Comma-separated physical GPU ids.")
    parser.add_argument("--seed", type=int, default=42, help="FastWAM/RoboTwin external base seed.")
    parser.add_argument("--environment-seed-start", type=int, default=None)
    parser.add_argument("--max-seed-attempts", type=int, default=1000)
    parser.add_argument("--target-good-seeds", type=int, default=10)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--ckpt", default=str(DEFAULT_CKPT))
    parser.add_argument("--dataset-stats-path", default=None)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--instruction-type", choices=["seen", "unseen"], default=None)
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
    parser.add_argument(
        "--skip-get-obs-within-replan", action=argparse.BooleanOptionalAction, default=True
    )
    return parser


def _make_config(args: argparse.Namespace) -> dict[str, Any]:
    phases = _parse_csv(args.phases)
    invalid_phases = set(phases) - set(PHASE_TO_TASK_CONFIG)
    if invalid_phases:
        raise ValueError(f"Unsupported phases: {sorted(invalid_phases)}")
    gpu_ids = _parse_csv(args.gpu_ids)
    if args.max_seed_attempts <= 0 or args.target_good_seeds <= 0 or args.repeats <= 0:
        raise ValueError("--max-seed-attempts, --target-good-seeds, and --repeats must be positive.")

    ckpt = _resolve_path(args.ckpt)
    if not ckpt.exists():
        raise FileNotFoundError(f"Checkpoint not found: {ckpt}")
    robotwin_root = PROJECT_ROOT / "third_party" / "RoboTwin"
    if not robotwin_root.exists():
        raise FileNotFoundError(f"RoboTwin root not found: {robotwin_root}")
    stats = _resolve_dataset_stats(ckpt, args.dataset_stats_path)
    if args.output_dir:
        output_dir = _resolve_path(args.output_dir)
    else:
        output_dir = (
            PROJECT_ROOT
            / "evaluate_results"
            / "robotwin"
            / "seed_search"
            / f"{args.task_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
    environment_seed_start = args.environment_seed_start
    if environment_seed_start is None:
        environment_seed_start = _environment_seed_start(args.seed)
    return {
        "task_name": args.task_name,
        "phases": phases,
        "gpu_ids": gpu_ids,
        "base_seed": args.seed,
        "environment_seed_start": environment_seed_start,
        "max_seed_attempts": args.max_seed_attempts,
        "target_good_seeds": args.target_good_seeds,
        "repeats": args.repeats,
        "ckpt": str(ckpt),
        "dataset_stats_path": str(stats),
        "output_dir": str(output_dir),
        "robotwin_root": str(robotwin_root.resolve()),
        "sim_cfg_path": str((PROJECT_ROOT / "configs" / "sim_robotwin.yaml").resolve()),
        "sim_task": args.sim_task,
        "instruction_type": args.instruction_type,
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
        "git_revision": _git_revision(),
    }


def main() -> None:
    args = _build_parser().parse_args()
    config = _make_config(args)
    output_dir = Path(config["output_dir"])
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"Output directory already contains data: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    _ensure_policy_symlink(Path(config["robotwin_root"]))
    _atomic_json(output_dir / "run_config.json", {"created_at": _now(), **config})

    manager_log = output_dir / "manager.log"

    def log(message: str) -> None:
        line = f"[{_now()}] {message}"
        print(line, flush=True)
        with manager_log.open("a", encoding="utf-8") as file:
            file.write(line + "\n")

    start_seed = config["environment_seed_start"]
    log(
        f"start task={config['task_name']} phases={','.join(config['phases'])} "
        f"gpus={','.join(config['gpu_ids'])} base_seed={config['base_seed']} "
        f"environment_seeds={start_seed}..{start_seed + config['max_seed_attempts'] - 1} "
        f"repeats={config['repeats']} target_per_phase={config['target_good_seeds']}"
    )

    context = mp.get_context("spawn")
    tasks = context.Queue()
    messages = context.Queue()
    phase_stops = {phase: context.Event() for phase in config["phases"]}
    for environment_seed in range(start_seed, start_seed + config["max_seed_attempts"]):
        for phase in config["phases"]:
            tasks.put((phase, environment_seed))
    for _ in config["gpu_ids"]:
        tasks.put(None)

    workers: list[mp.Process] = []
    worker_gpu_ids: dict[int, str] = {}
    for gpu_id in config["gpu_ids"]:
        worker = context.Process(
            target=_worker,
            args=(
                config,
                gpu_id,
                tasks,
                messages,
                phase_stops,
                str(output_dir / "logs" / f"worker_gpu{gpu_id}.log"),
            ),
        )
        worker.start()
        workers.append(worker)
        assert worker.pid is not None
        worker_gpu_ids[worker.pid] = gpu_id

    selected = {phase: [] for phase in config["phases"]}
    finished_workers = 0
    finished_gpu_ids: set[str] = set()
    worker_error: str | None = None
    try:
        while finished_workers < len(workers):
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
                    log(worker_error)
                    break
                continue
            kind = message["kind"]
            if kind == "ready":
                log(f"worker ready gpu={message['gpu_id']}")
            elif kind == "candidate":
                result = message["result"]
                phase = result["phase"]
                if result["all_rollouts_success"] and len(selected[phase]) < config["target_good_seeds"]:
                    result["selected"] = True
                    selected[phase].append(result["environment_seed"])
                    if len(selected[phase]) == config["target_good_seeds"]:
                        phase_stops[phase].set()
                        log(f"phase target reached phase={phase} selected={selected[phase]}")
                _atomic_json(_result_path(output_dir, phase, result["environment_seed"]), result)
                log(
                    f"candidate phase={phase} environment_seed={result['environment_seed']} "
                    f"expert_ok={result['expert']['ok']} all_rollouts_success={result['all_rollouts_success']}"
                )
            elif kind == "worker_error":
                worker_error = f"worker GPU {message['gpu_id']} failed:\n{message['error']}"
                log(worker_error)
                break
            elif kind == "done":
                finished_workers += 1
                finished_gpu_ids.add(message["gpu_id"])
                log(f"worker done gpu={message['gpu_id']}")
    finally:
        if worker_error is not None:
            for worker in workers:
                if worker.is_alive():
                    worker.terminate()
        for worker in workers:
            worker.join(timeout=30)
            if worker.is_alive():
                worker.kill()
                worker.join()

    _write_summary(output_dir, config, selected)
    _write_successful_seeds(output_dir, config)
    if worker_error is not None:
        raise RuntimeError(worker_error)
    log(f"finished summary={output_dir / 'summary.json'}")


if __name__ == "__main__":
    main()
