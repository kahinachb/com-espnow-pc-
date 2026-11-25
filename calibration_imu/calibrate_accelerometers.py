#!/usr/bin/env python3
"""Calibration multi-position des accéléromètres selon Tedaldi et al. (2014)."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


PATCHES = {
    9: "Patch 1 — talon",
    11: "Patch 2",
    13: "Patch 3",
    15: "Patch 4 — orteils",
}
ACC_COLUMNS = ("accX (g)", "accY (g)", "accZ (g)")


def read_pose(path: Path) -> dict[int, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    result = {}
    for identifier in PATCHES:
        patch_rows = [row for row in rows if int(row["ID1"]) == identifier]
        result[identifier] = np.asarray(
            [[float(row[column]) for column in ACC_COLUMNS] for row in patch_rows],
            dtype=float,
        )
    return result


def vector_std(samples: np.ndarray) -> float:
    return float(np.sqrt(np.var(samples, axis=0).sum()))


def select_shared_static_window(
    samples_by_patch: dict[int, np.ndarray],
    window_samples: int,
    skip_samples: int,
    step_samples: int,
) -> tuple[int, int, float]:
    available = min(len(samples) for samples in samples_by_patch.values())
    if available < window_samples:
        raise ValueError(
            f"seulement {available} échantillons communs pour une fenêtre de {window_samples}"
        )
    last_start = available - window_samples
    first_start = min(skip_samples, last_start)
    candidates = range(first_start, last_start + 1, step_samples)
    best: tuple[float, int] | None = None
    for start in candidates:
        end = start + window_samples
        score = max(
            vector_std(samples[start:end])
            for samples in samples_by_patch.values()
        )
        if best is None or score < best[0]:
            best = score, start
    assert best is not None
    return best[1], best[1] + window_samples, best[0]


def matrices(parameters: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    alpha_yz, alpha_zy, alpha_zx = parameters[:3]
    scale = parameters[3:6]
    bias = parameters[6:9]
    misalignment = np.array(
        [
            [1.0, -alpha_yz, alpha_zy],
            [0.0, 1.0, -alpha_zx],
            [0.0, 0.0, 1.0],
        ]
    )
    scaling = np.diag(scale)
    return misalignment, scaling, bias


def apply_calibration(samples: np.ndarray, parameters: np.ndarray) -> np.ndarray:
    misalignment, scaling, bias = matrices(parameters)
    correction = misalignment @ scaling
    return (correction @ (samples + bias).T).T


def residuals(parameters: np.ndarray, pose_means: np.ndarray) -> np.ndarray:
    calibrated = apply_calibration(pose_means, parameters)
    # Équation 10 du papier, avec les accélérations exprimées en g.
    return 1.0 - np.einsum("ij,ij->i", calibrated, calibrated)


def numerical_jacobian(parameters: np.ndarray, pose_means: np.ndarray) -> np.ndarray:
    base = residuals(parameters, pose_means)
    jacobian = np.empty((len(base), len(parameters)))
    for index in range(len(parameters)):
        step = 1e-6 * max(1.0, abs(parameters[index]))
        shifted = parameters.copy()
        shifted[index] += step
        jacobian[:, index] = (residuals(shifted, pose_means) - base) / step
    return jacobian


def calibrate_lm(
    pose_means: np.ndarray,
    max_iterations: int = 500,
) -> tuple[np.ndarray, dict[str, float | int | bool]]:
    if len(pose_means) < 9:
        raise ValueError("au moins neuf poses sont nécessaires pour estimer neuf paramètres")

    parameters = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
    damping = 1e-3
    current_residuals = residuals(parameters, pose_means)
    cost = 0.5 * float(current_residuals @ current_residuals)
    converged = False

    for iteration in range(1, max_iterations + 1):
        jacobian = numerical_jacobian(parameters, pose_means)
        normal = jacobian.T @ jacobian
        diagonal = np.maximum(np.diag(normal), 1e-12)
        gradient = jacobian.T @ current_residuals
        if np.linalg.norm(gradient, ord=np.inf) < 1e-8:
            converged = True
            break
        try:
            update = np.linalg.solve(
                normal + damping * np.diag(diagonal),
                -gradient,
            )
        except np.linalg.LinAlgError:
            damping *= 10.0
            continue

        candidate = parameters + update
        # Garde-fous physiques, largement plus ouverts que les erreurs attendues.
        if (
            np.any(np.abs(candidate[:3]) > 0.3)
            or np.any(candidate[3:6] < 0.5)
            or np.any(candidate[3:6] > 1.5)
            or np.any(np.abs(candidate[6:9]) > 0.5)
        ):
            damping *= 10.0
            continue

        candidate_residuals = residuals(candidate, pose_means)
        candidate_cost = 0.5 * float(candidate_residuals @ candidate_residuals)
        if candidate_cost < cost:
            relative_improvement = (cost - candidate_cost) / max(cost, 1e-30)
            parameters = candidate
            current_residuals = candidate_residuals
            cost = candidate_cost
            damping = max(damping / 3.0, 1e-12)
            if np.linalg.norm(update) < 1e-11 or relative_improvement < 1e-12:
                converged = True
                break
        else:
            damping = min(damping * 10.0, 1e15)

    jacobian = numerical_jacobian(parameters, pose_means)
    condition = float(np.linalg.cond(jacobian.T @ jacobian))
    return parameters, {
        "converged": converged,
        "iterations": iteration,
        "cost": cost,
        "normal_matrix_condition": condition,
    }


def leave_one_out_errors(pose_means: np.ndarray) -> np.ndarray:
    errors = np.empty(len(pose_means))
    for held_out in range(len(pose_means)):
        training = np.delete(pose_means, held_out, axis=0)
        parameters, _ = calibrate_lm(training)
        corrected = apply_calibration(pose_means[[held_out]], parameters)[0]
        errors[held_out] = np.linalg.norm(corrected) - 1.0
    return errors


def rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def write_comparison_csv(
    output: Path,
    pose_names: list[str],
    means_by_patch: dict[int, np.ndarray],
    parameters_by_patch: dict[int, np.ndarray],
    loo_by_patch: dict[int, np.ndarray],
) -> None:
    fields = (
        "pose",
        "patch_number",
        "ID1",
        "raw_accX_g",
        "raw_accY_g",
        "raw_accZ_g",
        "raw_norm_g",
        "cal_accX_g",
        "cal_accY_g",
        "cal_accZ_g",
        "cal_norm_g",
        "loo_norm_error_g",
    )
    with output.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for patch_number, identifier in enumerate(PATCHES, start=1):
            raw = means_by_patch[identifier]
            calibrated = apply_calibration(raw, parameters_by_patch[identifier])
            for index, pose in enumerate(pose_names):
                writer.writerow(
                    {
                        "pose": pose,
                        "patch_number": patch_number,
                        "ID1": identifier,
                        "raw_accX_g": raw[index, 0],
                        "raw_accY_g": raw[index, 1],
                        "raw_accZ_g": raw[index, 2],
                        "raw_norm_g": np.linalg.norm(raw[index]),
                        "cal_accX_g": calibrated[index, 0],
                        "cal_accY_g": calibrated[index, 1],
                        "cal_accZ_g": calibrated[index, 2],
                        "cal_norm_g": np.linalg.norm(calibrated[index]),
                        "loo_norm_error_g": loo_by_patch[identifier][index],
                    }
                )


def plot_norm_comparison(
    output: Path,
    pose_names: list[str],
    means_by_patch: dict[int, np.ndarray],
    parameters_by_patch: dict[int, np.ndarray],
) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(15, 10), sharex=True, sharey=True)
    x = np.arange(1, len(pose_names) + 1)
    for axis, (identifier, patch_name) in zip(axes.flat, PATCHES.items()):
        raw = means_by_patch[identifier]
        calibrated = apply_calibration(raw, parameters_by_patch[identifier])
        axis.plot(x, np.linalg.norm(raw, axis=1), "o-", label="Non calibré")
        axis.plot(x, np.linalg.norm(calibrated, axis=1), "o-", label="Calibré")
        axis.axhline(1.0, color="black", linestyle=":", label="Gravité idéale")
        axis.set_title(f"{patch_name} (ID1={identifier})")
        axis.set_xlabel("Numéro de pose")
        axis.set_ylabel("Norme de l'accélération (g)")
        axis.grid(True, alpha=0.3)
        axis.legend()
    figure.suptitle("Accéléromètres — comparaison avant/après calibration", fontsize=16)
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(output, dpi=180)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Calibre les quatre accéléromètres gauches avec la méthode multi-position."
    )
    parser.add_argument("session_dir", type=Path)
    parser.add_argument(
        "--window-seconds",
        type=float,
        default=1.0,
        help="durée de la fenêtre statique sélectionnée par pose (défaut : 1 s)",
    )
    parser.add_argument("--frequency", type=float, default=200.0)
    parser.add_argument(
        "--skip-seconds",
        type=float,
        default=0.5,
        help="début ignoré avant la recherche de stabilité (défaut : 0,5 s)",
    )
    parser.add_argument(
        "--max-static-std",
        type=float,
        default=0.01,
        help="rejet si l'écart-type vectoriel dépasse cette valeur en g",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="défaut : <session>/calibration_acc",
    )
    args = parser.parse_args()
    if args.frequency <= 0 or args.window_seconds <= 0:
        parser.error("la fréquence et la durée de fenêtre doivent être positives")

    pose_files = sorted(args.session_dir.glob("pose_*.csv"))
    if len(pose_files) < 9:
        parser.error(f"seulement {len(pose_files)} poses trouvées ; minimum : 9")

    output_dir = args.output_dir or args.session_dir / "calibration_acc"
    output_dir.mkdir(parents=True, exist_ok=True)
    window_samples = max(2, round(args.window_seconds * args.frequency))
    skip_samples = max(0, round(args.skip_seconds * args.frequency))
    step_samples = max(1, round(0.1 * args.frequency))

    accepted_names: list[str] = []
    means_by_patch_lists: dict[int, list[np.ndarray]] = {
        identifier: [] for identifier in PATCHES
    }
    stability_rows = []
    for pose_file in pose_files:
        samples_by_patch = read_pose(pose_file)
        try:
            start, end, score = select_shared_static_window(
                samples_by_patch,
                window_samples,
                skip_samples,
                step_samples,
            )
        except ValueError as error:
            print(f"Rejet {pose_file.stem} : {error}")
            continue
        accepted = score <= args.max_static_std
        stability_rows.append(
            {
                "pose": pose_file.stem,
                "accepted": int(accepted),
                "window_start_sample": start,
                "window_end_sample": end,
                "max_vector_std_g": score,
            }
        )
        if not accepted:
            print(
                f"Rejet {pose_file.stem} : écart-type {score:.6f} g "
                f"> {args.max_static_std:.6f} g"
            )
            continue
        accepted_names.append(pose_file.stem)
        for identifier, samples in samples_by_patch.items():
            means_by_patch_lists[identifier].append(samples[start:end].mean(axis=0))

    if len(accepted_names) < 9:
        parser.error(f"seulement {len(accepted_names)} poses statiques acceptées ; minimum : 9")

    with (output_dir / "stability_windows.csv").open(
        "w", newline="", encoding="utf-8"
    ) as file:
        writer = csv.DictWriter(file, fieldnames=stability_rows[0].keys())
        writer.writeheader()
        writer.writerows(stability_rows)

    means_by_patch = {
        identifier: np.vstack(means)
        for identifier, means in means_by_patch_lists.items()
    }
    parameters_by_patch: dict[int, np.ndarray] = {}
    loo_by_patch: dict[int, np.ndarray] = {}
    calibration_json: dict[str, object] = {
        "model": "Tedaldi et al. 2014 accelerometer multi-position",
        "units": "g",
        "formula": "a_cal = misalignment_matrix @ scale_matrix @ (a_raw + additive_bias_g)",
        "accepted_poses": accepted_names,
        "window_seconds": args.window_seconds,
        "patches": {},
    }

    for patch_number, identifier in enumerate(PATCHES, start=1):
        raw = means_by_patch[identifier]
        parameters, diagnostics = calibrate_lm(raw)
        parameters_by_patch[identifier] = parameters
        loo_errors = leave_one_out_errors(raw)
        loo_by_patch[identifier] = loo_errors
        calibrated = apply_calibration(raw, parameters)
        raw_errors = np.linalg.norm(raw, axis=1) - 1.0
        calibrated_errors = np.linalg.norm(calibrated, axis=1) - 1.0
        misalignment, scaling, bias = matrices(parameters)
        correction = misalignment @ scaling
        calibration_json["patches"][str(patch_number)] = {
            "ID1": identifier,
            "misalignment_parameters_rad": {
                "alpha_yz": parameters[0],
                "alpha_zy": parameters[1],
                "alpha_zx": parameters[2],
            },
            "scale_factors": {
                "x": parameters[3],
                "y": parameters[4],
                "z": parameters[5],
            },
            "additive_bias_g": {
                "x": bias[0],
                "y": bias[1],
                "z": bias[2],
            },
            "subtractive_offset_g": {
                "x": -bias[0],
                "y": -bias[1],
                "z": -bias[2],
            },
            "misalignment_matrix": misalignment.tolist(),
            "scale_matrix": scaling.tolist(),
            "correction_matrix": correction.tolist(),
            "metrics": {
                "raw_norm_rmse_g": rmse(raw_errors),
                "calibrated_fit_norm_rmse_g": rmse(calibrated_errors),
                "leave_one_pose_out_norm_rmse_g": rmse(loo_errors),
                **diagnostics,
            },
        }

    (output_dir / "accelerometer_calibration.json").write_text(
        json.dumps(calibration_json, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    write_comparison_csv(
        output_dir / "pose_means_comparison.csv",
        accepted_names,
        means_by_patch,
        parameters_by_patch,
        loo_by_patch,
    )
    plot_norm_comparison(
        output_dir / "acc_norm_before_after.png",
        accepted_names,
        means_by_patch,
        parameters_by_patch,
    )

    print(f"Poses acceptées : {len(accepted_names)}/{len(pose_files)}")
    for patch_number in range(1, 5):
        metrics = calibration_json["patches"][str(patch_number)]["metrics"]
        print(
            f"Patch {patch_number}: RMSE norme brute={metrics['raw_norm_rmse_g']:.6f} g, "
            f"calibrée={metrics['calibrated_fit_norm_rmse_g']:.6f} g, "
            f"LOO={metrics['leave_one_pose_out_norm_rmse_g']:.6f} g"
        )
    print(f"Résultats : {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
