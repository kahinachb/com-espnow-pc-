#!/usr/bin/env python3
"""Calibration des gyroscopes selon Tedaldi et al. (ICRA 2014)."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
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
GYR_COLUMNS = ("gyX (°/s)", "gyY (°/s)", "gyZ (°/s)")


@dataclass
class Transition:
    name: str
    time_s: np.ndarray
    acceleration: dict[int, np.ndarray]
    gyroscope_dps: dict[int, np.ndarray]
    initial_slice: slice
    final_slice: slice


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def patch_array(
    rows: list[dict[str, str]],
    identifier: int,
    columns: tuple[str, str, str],
) -> np.ndarray:
    selected = [row for row in rows if int(row["ID1"]) == identifier]
    return np.asarray(
        [[float(row[column]) for column in columns] for row in selected],
        dtype=float,
    )


def unwrap_time(rows: list[dict[str, str]]) -> np.ndarray:
    toe_rows = [row for row in rows if int(row["ID1"]) == 15]
    raw = [int(row["Temps"]) for row in toe_rows]
    if len(raw) < 2:
        raise ValueError("timestamps du patch 4 absents")
    unwrapped = [raw[0]]
    for previous, current in zip(raw, raw[1:]):
        unwrapped.append(unwrapped[-1] + ((current - previous) & 0xFFFFFFFF))
    result = (np.asarray(unwrapped, dtype=np.float64) - unwrapped[0]) / 1e6
    if np.any(np.diff(result) <= 0):
        raise ValueError("timestamps non strictement croissants")
    return result


def vector_std(samples: np.ndarray) -> float:
    return float(np.sqrt(np.var(samples, axis=0).sum()))


def best_static_slice(
    time_s: np.ndarray,
    acceleration: dict[int, np.ndarray],
    interval_start: float,
    interval_end: float,
    window_seconds: float,
    step_seconds: float = 0.1,
) -> tuple[slice, float]:
    candidates: list[tuple[float, slice]] = []
    start = interval_start
    while start + window_seconds <= interval_end + 1e-9:
        begin = int(np.searchsorted(time_s, start, side="left"))
        end = int(np.searchsorted(time_s, start + window_seconds, side="right"))
        if end - begin >= 50:
            score = max(
                vector_std(samples[begin:end])
                for samples in acceleration.values()
            )
            candidates.append((score, slice(begin, end)))
        start += step_seconds
    if not candidates:
        raise ValueError("aucune fenêtre statique assez longue")
    return min(candidates, key=lambda item: item[0])[1], min(
        candidates, key=lambda item: item[0]
    )[0]


def load_accelerometer_calibration(path: Path) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    content = json.loads(path.read_text(encoding="utf-8"))
    result = {}
    for patch in content["patches"].values():
        identifier = int(patch["ID1"])
        correction = np.asarray(patch["correction_matrix"], dtype=float)
        additive_bias = np.asarray(
            [
                patch["additive_bias_g"]["x"],
                patch["additive_bias_g"]["y"],
                patch["additive_bias_g"]["z"],
            ],
            dtype=float,
        )
        result[identifier] = correction, additive_bias
    missing = set(PATCHES) - set(result)
    if missing:
        raise ValueError(f"calibration accéléromètre incomplète, ID manquants : {missing}")
    return result


def calibrate_acceleration(
    samples: np.ndarray,
    calibration: tuple[np.ndarray, np.ndarray],
) -> np.ndarray:
    correction, additive_bias = calibration
    return (correction @ (samples + additive_bias).T).T


def correction_matrix(parameters: np.ndarray) -> np.ndarray:
    """Construit Tg Kg avec l'ordre des paramètres de l'équation 12."""
    gamma_yz, gamma_zy, gamma_xz, gamma_zx, gamma_xy, gamma_yx = parameters[:6]
    sx, sy, sz = parameters[6:9]
    misalignment = np.array(
        [
            [1.0, -gamma_yz, gamma_zy],
            [gamma_xz, 1.0, -gamma_zx],
            [-gamma_xy, gamma_yx, 1.0],
        ]
    )
    return misalignment @ np.diag([sx, sy, sz])


def correction_parameters(matrix: np.ndarray) -> dict[str, object]:
    sx, sy, sz = matrix[0, 0], matrix[1, 1], matrix[2, 2]
    return {
        "scale_factors": {"x": sx, "y": sy, "z": sz},
        "misalignment_parameters_rad": {
            "gamma_yz": -matrix[0, 1] / sy,
            "gamma_zy": matrix[0, 2] / sz,
            "gamma_xz": matrix[1, 0] / sx,
            "gamma_zx": -matrix[1, 2] / sz,
            "gamma_xy": -matrix[2, 0] / sx,
            "gamma_yx": matrix[2, 1] / sy,
        },
    }


def integrate_gravity_rk4(
    initial_gravity: np.ndarray,
    raw_gyro_dps: np.ndarray,
    time_s: np.ndarray,
    correction_matrix: np.ndarray,
    bias_dps: np.ndarray,
) -> np.ndarray:
    """Intègre l'orientation par quaternion avec le RK4n du papier."""
    omega = ((correction_matrix @ (raw_gyro_dps - bias_dps).T).T) * (
        math.pi / 180.0
    )
    initial = initial_gravity / np.linalg.norm(initial_gravity)
    # Quaternion [w, x, y, z] donnant la rotation du repère initial vers le
    # repère courant. L'identité convient au début de chaque transition.
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    def derivative(q: np.ndarray, angular_rate: np.ndarray) -> np.ndarray:
        """Équations 15-16 : q_dot = 1/2 Omega(omega) q."""
        wx, wy, wz = angular_rate
        omega_matrix = np.array(
            [
                [0.0, -wx, -wy, -wz],
                [wx, 0.0, wz, -wy],
                [wy, -wz, 0.0, wx],
                [wz, wy, -wx, 0.0],
            ]
        )
        return 0.5 * (omega_matrix @ q)

    for index, dt in enumerate(np.diff(time_s)):
        if dt <= 0 or dt > 0.05:
            raise ValueError(f"pas temporel invalide pendant l'intégration : {dt}")
        omega_0 = omega[index]
        omega_1 = omega[index + 1]
        omega_mid = 0.5 * (omega_0 + omega_1)
        k1 = derivative(quaternion, omega_0)
        k2 = derivative(quaternion + 0.5 * dt * k1, omega_mid)
        k3 = derivative(quaternion + 0.5 * dt * k2, omega_mid)
        k4 = derivative(quaternion + dt * k3, omega_1)
        quaternion = quaternion + (dt / 6.0) * (
            k1 + 2.0 * k2 + 2.0 * k3 + k4
        )
        quaternion /= np.linalg.norm(quaternion)

    qw, qx, qy, qz = quaternion
    rotation = np.array(
        [
            [
                1 - 2 * (qy * qy + qz * qz),
                2 * (qx * qy - qw * qz),
                2 * (qx * qz + qw * qy),
            ],
            [
                2 * (qx * qy + qw * qz),
                1 - 2 * (qx * qx + qz * qz),
                2 * (qy * qz - qw * qx),
            ],
            [
                2 * (qx * qz - qw * qy),
                2 * (qy * qz + qw * qx),
                1 - 2 * (qx * qx + qy * qy),
            ],
        ]
    )
    # La gravité est fixe dans le monde : ses coordonnées dans le repère
    # courant sont données par la rotation inverse.
    return rotation.T @ initial


def transition_vectors(
    transition: Transition,
    identifier: int,
    acc_calibration: tuple[np.ndarray, np.ndarray],
    integration_stride: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    acceleration = calibrate_acceleration(
        transition.acceleration[identifier],
        acc_calibration,
    )
    initial = acceleration[transition.initial_slice].mean(axis=0)
    final = acceleration[transition.final_slice].mean(axis=0)
    initial /= np.linalg.norm(initial)
    final /= np.linalg.norm(final)
    start = (transition.initial_slice.start + transition.initial_slice.stop - 1) // 2
    end = (transition.final_slice.start + transition.final_slice.stop - 1) // 2
    gyro = transition.gyroscope_dps[identifier][start : end + 1]
    time_s = transition.time_s[start : end + 1]
    indexes = np.arange(0, len(time_s), integration_stride)
    if indexes[-1] != len(time_s) - 1:
        indexes = np.append(indexes, len(time_s) - 1)
    return initial, final, gyro[indexes], time_s[indexes]


def model_residuals(
    parameters: np.ndarray,
    prepared: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    bias_dps: np.ndarray,
) -> np.ndarray:
    matrix = correction_matrix(parameters)
    residuals = []
    for initial, final, gyro, time_s in prepared:
        prediction = integrate_gravity_rk4(
            initial,
            gyro,
            time_s,
            matrix,
            bias_dps,
        )
        residuals.extend(final - prediction)
    return np.asarray(residuals)


def numerical_jacobian(
    parameters: np.ndarray,
    prepared: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    bias_dps: np.ndarray,
) -> np.ndarray:
    base = model_residuals(parameters, prepared, bias_dps)
    jacobian = np.empty((len(base), len(parameters)))
    for index in range(len(parameters)):
        step = 1e-5 * max(1.0, abs(parameters[index]))
        shifted = parameters.copy()
        shifted[index] += step
        jacobian[:, index] = (
            model_residuals(shifted, prepared, bias_dps) - base
        ) / step
    return jacobian


def calibrate_lm(
    prepared: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    bias_dps: np.ndarray,
    max_iterations: int = 20,
) -> tuple[np.ndarray, dict[str, object]]:
    if len(prepared) < 4:
        raise ValueError("au moins quatre transitions sont nécessaires")
    # Eq. 12 : six défauts d'alignement nuls et trois échelles unitaires.
    parameters = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0])
    damping = 1e-3
    current = model_residuals(parameters, prepared, bias_dps)
    cost = 0.5 * float(current @ current)
    converged = False

    for iteration in range(1, max_iterations + 1):
        jacobian = numerical_jacobian(parameters, prepared, bias_dps)
        normal = jacobian.T @ jacobian
        gradient = jacobian.T @ current
        if np.linalg.norm(gradient, ord=np.inf) < 1e-8:
            converged = True
            break
        diagonal = np.maximum(np.diag(normal), 1e-12)
        try:
            update = np.linalg.solve(
                normal + damping * np.diag(diagonal),
                -gradient,
            )
        except np.linalg.LinAlgError:
            damping *= 10
            continue
        candidate = parameters + update
        if (
            np.any(np.abs(candidate[:6]) > 0.3)
            or np.any(candidate[6:9] < 0.5)
            or np.any(candidate[6:9] > 1.5)
        ):
            damping *= 10
            continue
        candidate_residuals = model_residuals(candidate, prepared, bias_dps)
        candidate_cost = 0.5 * float(candidate_residuals @ candidate_residuals)
        if candidate_cost < cost:
            relative = (cost - candidate_cost) / max(cost, 1e-30)
            parameters = candidate
            current = candidate_residuals
            cost = candidate_cost
            damping = max(damping / 3, 1e-12)
            if np.linalg.norm(update) < 1e-10 or relative < 1e-11:
                converged = True
                break
        else:
            damping = min(damping * 10, 1e15)

    jacobian = numerical_jacobian(parameters, prepared, bias_dps)
    return correction_matrix(parameters), {
        "converged": converged,
        "iterations": iteration,
        "cost": cost,
        "normal_matrix_condition": float(np.linalg.cond(jacobian.T @ jacobian)),
    }


def angular_errors_deg(
    prepared: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    bias_dps: np.ndarray,
    matrix: np.ndarray,
) -> np.ndarray:
    errors = []
    for initial, final, gyro, time_s in prepared:
        prediction = integrate_gravity_rk4(
            initial, gyro, time_s, matrix, bias_dps
        )
        errors.append(
            math.degrees(
                math.acos(float(np.clip(prediction @ final, -1.0, 1.0)))
            )
        )
    return np.asarray(errors)


def cross_validated_errors(
    prepared: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    bias_dps: np.ndarray,
    fold_count: int = 5,
) -> np.ndarray:
    errors = np.empty(len(prepared))
    fold_count = min(fold_count, len(prepared))
    for fold in range(fold_count):
        held_out = [index for index in range(len(prepared)) if index % fold_count == fold]
        training = [
            transition
            for index, transition in enumerate(prepared)
            if index not in held_out
        ]
        matrix, _ = calibrate_lm(training, bias_dps)
        held_out_errors = angular_errors_deg(
            [prepared[index] for index in held_out],
            bias_dps,
            matrix,
        )
        for index, error in zip(held_out, held_out_errors):
            errors[index] = error
    return errors


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Calibre les gyroscopes à partir des transitions statique-mouvement-statique."
    )
    parser.add_argument("session_dir", type=Path)
    parser.add_argument(
        "--acc-calibration",
        type=Path,
        required=True,
        help="accelerometer_calibration.json produit précédemment",
    )
    parser.add_argument(
        "--static-window",
        type=float,
        default=2.0,
        help="fenêtre statique twait du papier (défaut : 2 s)",
    )
    parser.add_argument("--max-static-std", type=float, default=0.01)
    parser.add_argument(
        "--max-axis-rate",
        type=float,
        default=1900.0,
        help="rejet si un axe approche la saturation (défaut : 1900 °/s)",
    )
    parser.add_argument(
        "--integration-hz",
        type=float,
        default=100.0,
        help="fréquence d'intégration après décimation (défaut : 100 Hz)",
    )
    parser.add_argument(
        "--cross-validation",
        action="store_true",
        help="effectuer en plus une validation croisée à cinq plis (calcul long)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="défaut : <session>/calibration_gyro",
    )
    args = parser.parse_args()
    metadata = json.loads(
        (args.session_dir / "metadata.json").read_text(encoding="utf-8")
    )
    pre_static = float(metadata["pre_static_s"])
    movement_end = pre_static + float(metadata["movement_s"])
    if args.integration_hz <= 0 or args.integration_hz > 200:
        parser.error("--integration-hz doit être dans ]0, 200]")
    integration_stride = max(1, round(200.0 / args.integration_hz))
    acc_calibrations = load_accelerometer_calibration(args.acc_calibration)

    initial_rows = read_rows(args.session_dir / "initial_static.csv")
    bias_by_patch = {
        identifier: patch_array(initial_rows, identifier, GYR_COLUMNS).mean(axis=0)
        for identifier in PATCHES
    }

    accepted: list[Transition] = []
    audit_rows = []
    for path in sorted(args.session_dir.glob("transition_*.csv")):
        reasons = []
        rows = read_rows(path)
        acceleration = {
            identifier: patch_array(rows, identifier, ACC_COLUMNS)
            for identifier in PATCHES
        }
        gyroscope = {
            identifier: patch_array(rows, identifier, GYR_COLUMNS)
            for identifier in PATCHES
        }
        lengths = {len(values) for values in acceleration.values()}
        if len(lengths) != 1 or not lengths or next(iter(lengths)) < 2:
            reasons.append("patches_non_alignes")
            audit_rows.append({"transition": path.stem, "accepted": 0, "reasons": ";".join(reasons)})
            continue
        time_s = unwrap_time(rows)
        gaps = np.diff(time_s)
        if np.any(gaps > 0.05):
            reasons.append("trou_temporel")
        try:
            initial_slice, initial_std = best_static_slice(
                time_s, acceleration, 0.0, pre_static, args.static_window
            )
            final_slice, final_std = best_static_slice(
                time_s,
                acceleration,
                movement_end,
                float(time_s[-1]),
                args.static_window,
            )
        except ValueError:
            initial_slice, final_slice = slice(0, 0), slice(0, 0)
            initial_std = final_std = float("inf")
            reasons.append("fenetre_statique_absente")
        if initial_std > args.max_static_std:
            reasons.append("debut_instable")
        if final_std > args.max_static_std:
            reasons.append("fin_instable")
        peak_axis = max(
            float(np.max(np.abs(values))) for values in gyroscope.values()
        )
        if peak_axis >= args.max_axis_rate:
            reasons.append("proche_saturation")
        transition = Transition(
            path.stem,
            time_s,
            acceleration,
            gyroscope,
            initial_slice,
            final_slice,
        )
        audit_rows.append(
            {
                "transition": path.stem,
                "accepted": int(not reasons),
                "reasons": ";".join(reasons),
                "initial_std_g": initial_std,
                "final_std_g": final_std,
                "peak_axis_dps": peak_axis,
                "initial_start_s": time_s[initial_slice.start] if initial_slice.stop else "",
                "final_start_s": time_s[final_slice.start] if final_slice.stop else "",
            }
        )
        if not reasons:
            accepted.append(transition)

    # Le papier impose au moins neuf attitudes, soit huit transitions entre
    # attitudes consécutives. Une trentaine de transitions reste préférable.
    if len(accepted) < 8:
        parser.error(
            f"seulement {len(accepted)} transitions acceptées ; minimum : 8"
        )
    output_dir = args.output_dir or args.session_dir / "calibration_gyro"
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "transition_audit.csv").open(
        "w", newline="", encoding="utf-8"
    ) as file:
        fields = sorted({key for row in audit_rows for key in row})
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(audit_rows)

    result: dict[str, object] = {
        "model": "Tedaldi et al. 2014 gyroscope multi-position",
        "formula": "omega_cal = correction_matrix @ (omega_raw - bias_dps)",
        "units": "degrees per second",
        "parameterization": (
            "theta_g = [gamma_yz, gamma_zy, gamma_xz, gamma_zx, "
            "gamma_xy, gamma_yx, sx, sy, sz]"
        ),
        "integration": (
            "normalized quaternion Runge-Kutta 4 (RK4n), "
            "Tedaldi equations 15-21"
        ),
        "accelerometer_calibration": str(args.acc_calibration),
        "accepted_transitions": [transition.name for transition in accepted],
        "patches": {},
    }
    comparison_rows = []
    figure, axes = plt.subplots(2, 2, figsize=(15, 10), sharex=True, sharey=True)
    for patch_number, (identifier, patch_name) in enumerate(PATCHES.items(), start=1):
        prepared = [
            transition_vectors(
                transition,
                identifier,
                acc_calibrations[identifier],
                integration_stride,
            )
            for transition in accepted
        ]
        bias = bias_by_patch[identifier]
        print(f"Optimisation du patch {patch_number}…", flush=True)
        matrix, diagnostics = calibrate_lm(prepared, bias)
        raw_errors = angular_errors_deg(prepared, bias, np.eye(3))
        calibrated_errors = angular_errors_deg(prepared, bias, matrix)
        cv_errors = (
            cross_validated_errors(prepared, bias)
            if args.cross_validation
            else np.full(len(prepared), np.nan)
        )
        parameters = correction_parameters(matrix)
        result["patches"][str(patch_number)] = {
            "ID1": identifier,
            "bias_dps": {"x": bias[0], "y": bias[1], "z": bias[2]},
            "correction_matrix": matrix.tolist(),
            **parameters,
            "metrics": {
                "raw_angle_rmse_deg": rms(raw_errors),
                "calibrated_fit_angle_rmse_deg": rms(calibrated_errors),
                "five_fold_cross_validation_angle_rmse_deg": (
                    rms(cv_errors) if args.cross_validation else None
                ),
                **diagnostics,
            },
        }
        for index, transition in enumerate(accepted):
            comparison_rows.append(
                {
                    "transition": transition.name,
                    "patch_number": patch_number,
                    "ID1": identifier,
                    "raw_angle_error_deg": raw_errors[index],
                    "calibrated_angle_error_deg": calibrated_errors[index],
                    "cross_validated_angle_error_deg": cv_errors[index],
                }
            )
        axis = axes.flat[patch_number - 1]
        x = np.arange(1, len(accepted) + 1)
        axis.plot(x, raw_errors, "o-", label="Non calibré")
        axis.plot(x, calibrated_errors, "o-", label="Calibré")
        if args.cross_validation:
            axis.plot(x, cv_errors, "o--", label="Validation croisée", alpha=0.8)
        axis.set_title(f"{patch_name} (ID1={identifier})")
        axis.set_xlabel("Transition acceptée")
        axis.set_ylabel("Erreur angulaire finale (°)")
        axis.grid(True, alpha=0.3)
        axis.legend()

    (output_dir / "gyroscope_calibration.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with (output_dir / "angle_error_comparison.csv").open(
        "w", newline="", encoding="utf-8"
    ) as file:
        writer = csv.DictWriter(file, fieldnames=comparison_rows[0].keys())
        writer.writeheader()
        writer.writerows(comparison_rows)
    figure.suptitle("Gyroscopes — erreur d'orientation avant/après calibration", fontsize=16)
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(output_dir / "gyro_angle_before_after.png", dpi=180)
    plt.close(figure)

    print(f"Transitions acceptées : {len(accepted)}/{len(audit_rows)}")
    for patch_number in range(1, 5):
        metrics = result["patches"][str(patch_number)]["metrics"]
        validation_text = (
            f", CV={metrics['five_fold_cross_validation_angle_rmse_deg']:.4f}°"
            if metrics["five_fold_cross_validation_angle_rmse_deg"] is not None
            else ""
        )
        print(
            f"Patch {patch_number}: brute={metrics['raw_angle_rmse_deg']:.4f}°, "
            f"calibrée={metrics['calibrated_fit_angle_rmse_deg']:.4f}°"
            f"{validation_text}"
        )
    print(f"Résultats : {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
