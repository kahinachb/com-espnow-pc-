#!/usr/bin/env python3
"""Calibre chaque magnétomètre d'une semelle par ajustement ellipsoïdal.

La correction calculée est :

    m_corrige = matrice_soft_iron @ (m_brut - biais_hard_iron)

La matrice est contrainte à avoir un déterminant égal à 1 afin de conserver
l'échelle physique nominale du MMC5983MA en µT.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import expm, logm
from scipy.optimize import least_squares


MAG_COLUMNS = ("magX (µT)", "magY (µT)", "magZ (µT)")
CAL_COLUMNS = ("magX_cal (µT)", "magY_cal (µT)", "magZ_cal (µT)")


def symmetric_trace_free(values):
    """Construit une matrice symétrique de trace nulle avec cinq paramètres."""
    s0, s1, s2, s3, s4 = values
    return np.array(
        [[s0, s1, s2], [s1, s3, s4], [s2, s4, -s0 - s3]],
        dtype=float,
    )


def matrix_to_parameters(matrix):
    """Convertit une matrice SPD de déterminant 1 vers cinq paramètres."""
    logarithm = np.real_if_close(logm(matrix)).astype(float)
    logarithm -= np.eye(3) * np.trace(logarithm) / 3.0
    return np.array(
        [logarithm[0, 0], logarithm[0, 1], logarithm[0, 2],
         logarithm[1, 1], logarithm[1, 2]],
        dtype=float,
    )


def algebraic_initialization(samples):
    """Produit une initialisation centre/matrice/rayon à partir d'un ellipsoïde."""
    x, y, z = samples.T
    design = np.column_stack(
        (x*x, y*y, z*z, 2*y*z, 2*x*z, 2*x*y, x, y, z)
    )
    parameters, *_ = np.linalg.lstsq(design, np.ones(len(samples)), rcond=None)
    quadratic = np.array(
        [
            [parameters[0], parameters[5], parameters[4]],
            [parameters[5], parameters[1], parameters[3]],
            [parameters[4], parameters[3], parameters[2]],
        ]
    )
    linear = parameters[6:9]
    center = -0.5 * np.linalg.solve(quadratic, linear)
    scale = 1.0 + center @ quadratic @ center
    normalized = quadratic / scale
    eigenvalues, eigenvectors = np.linalg.eigh(normalized)
    if np.any(eigenvalues <= 0):
        raise ValueError("la couverture ne produit pas un ellipsoïde positif")

    inverse_axes = eigenvectors @ np.diag(np.sqrt(eigenvalues)) @ eigenvectors.T
    radius = np.linalg.det(normalized) ** (-1.0 / 6.0)
    correction = radius * inverse_axes
    correction /= np.linalg.det(correction) ** (1.0 / 3.0)
    return center, correction, radius


def covariance_initialization(samples):
    """Initialisation SPD robuste lorsque l'ajustement algébrique est indéfini."""
    lower, upper = np.percentile(samples, (1.0, 99.0), axis=0)
    center = 0.5 * (lower + upper)
    covariance = np.cov(samples - center, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if np.any(eigenvalues <= 0):
        raise ValueError("la couverture 3D est singulière")

    correction = (
        eigenvectors
        @ np.diag(1.0 / np.sqrt(eigenvalues))
        @ eigenvectors.T
    )
    correction /= np.linalg.det(correction) ** (1.0 / 3.0)
    corrected = (samples - center) @ correction.T
    radius = float(np.median(np.linalg.norm(corrected, axis=1)))
    return center, correction, radius


def calibrate(samples, max_fit_samples=12000):
    """Ajuste biais, matrice soft-iron et norme du champ avec une perte robuste."""
    if len(samples) < 500:
        raise ValueError("au moins 500 mesures sont recommandées")

    if len(samples) > max_fit_samples:
        indices = np.linspace(0, len(samples) - 1, max_fit_samples, dtype=int)
        fit_samples = samples[indices]
    else:
        fit_samples = samples

    robust_initialization = covariance_initialization(fit_samples)
    try:
        initialization = algebraic_initialization(fit_samples)
        used_algebraic_initialization = True
    except (ValueError, np.linalg.LinAlgError):
        initialization = robust_initialization
        used_algebraic_initialization = False

    def pack_initial(parameters):
        center, correction, radius = parameters
        return np.concatenate(
            (center, matrix_to_parameters(correction), [math.log(radius)])
        )

    def residuals(parameters):
        center = parameters[:3]
        correction = expm(symmetric_trace_free(parameters[3:8]))
        radius = math.exp(parameters[8])
        corrected = (fit_samples - center) @ correction.T
        return np.linalg.norm(corrected, axis=1) / radius - 1.0

    def optimize(initial):
        return least_squares(
            residuals,
            initial,
            loss="soft_l1",
            f_scale=0.01,
            max_nfev=1000,
        )

    result = optimize(pack_initial(initialization))
    if not result.success and used_algebraic_initialization:
        result = optimize(pack_initial(robust_initialization))
    if not result.success:
        raise RuntimeError(result.message)

    bias = result.x[:3]
    matrix = expm(symmetric_trace_free(result.x[3:8]))
    reference_norm = math.exp(result.x[8])
    corrected = (samples - bias) @ matrix.T
    return bias, matrix, reference_norm, corrected


def stats(values):
    return {
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "coefficient_of_variation_percent": float(
            100.0 * np.std(values) / np.mean(values)
        ),
    }


def make_plot(raw, corrected, patch_label, destination):
    sample_step = max(1, len(raw) // 5000)
    raw_plot = raw[::sample_step]
    corrected_plot = corrected[::sample_step]
    pairs = ((0, 1, "X", "Y"), (0, 2, "X", "Z"), (1, 2, "Y", "Z"))

    figure, axes = plt.subplots(2, 2, figsize=(11, 9))
    for axis, (first, second, first_name, second_name) in zip(axes.flat[:3], pairs):
        axis.scatter(raw_plot[:, first], raw_plot[:, second], s=2, alpha=0.25,
                     label="brut")
        axis.scatter(corrected_plot[:, first], corrected_plot[:, second], s=2,
                     alpha=0.25, label="calibré")
        axis.set_xlabel(f"{first_name} (µT)")
        axis.set_ylabel(f"{second_name} (µT)")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.25)
        axis.legend()

    raw_norm = np.linalg.norm(raw, axis=1)
    corrected_norm = np.linalg.norm(corrected, axis=1)
    axes[1, 1].hist(raw_norm, bins=80, alpha=0.55, label="norme brute")
    axes[1, 1].hist(corrected_norm, bins=80, alpha=0.55, label="norme calibrée")
    axes[1, 1].set_xlabel("Norme (µT)")
    axes[1, 1].set_ylabel("Nombre de mesures")
    axes[1, 1].grid(True, alpha=0.25)
    axes[1, 1].legend()

    figure.suptitle(f"Calibration magnétomètre — patch {patch_label}")
    figure.tight_layout()
    figure.savefig(destination, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(
        description="Calibration hard-iron/soft-iron des magnétomètres par patch"
    )
    parser.add_argument("input_csv", type=Path, help="CSV produit par decodageToCSV.py")
    parser.add_argument(
        "--output-dir", type=Path, default=Path("magnetometer_calibration"),
        help="dossier des coefficients et graphes",
    )
    parser.add_argument(
        "--calibrated-csv", type=Path,
        help="CSV enrichi (défaut : <output-dir>/<nom>_calibrated.csv)",
    )
    args = parser.parse_args()

    with args.input_csv.open(newline="") as source:
        reader = csv.DictReader(source)
        rows = list(reader)
        original_fields = reader.fieldnames
    if not rows or not original_fields:
        raise SystemExit("CSV vide ou sans entête")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    calibrated_csv = args.calibrated_csv or (
        args.output_dir / f"{args.input_csv.stem}_calibrated.csv"
    )

    row_indices = {}
    samples_by_patch = {}
    for index, row in enumerate(rows):
        patch_id = int(row["ID1"])
        row_indices.setdefault(patch_id, []).append(index)
    for patch_id, indices in row_indices.items():
        samples_by_patch[patch_id] = np.array(
            [[float(rows[i][column]) for column in MAG_COLUMNS] for i in indices]
        )

    report = {
        "source": str(args.input_csv),
        "formula": "m_cal = soft_iron_matrix @ (m_raw - hard_iron_bias_uT)",
        "patches": {},
    }

    for patch_id in sorted(samples_by_patch):
        raw = samples_by_patch[patch_id]
        bias, matrix, reference_norm, corrected = calibrate(raw)
        raw_norm = np.linalg.norm(raw, axis=1)
        corrected_norm = np.linalg.norm(corrected, axis=1)
        centered = raw - np.mean(raw, axis=0)
        covariance_eigenvalues = np.linalg.eigvalsh(np.cov(centered, rowvar=False))
        id2 = int(rows[row_indices[patch_id][0]]["ID2"])
        label = f"{patch_id}-{id2}"

        report["patches"][label] = {
            "hard_iron_bias_uT": bias.tolist(),
            "soft_iron_matrix": matrix.tolist(),
            "reference_norm_uT": reference_norm,
            "matrix_determinant": float(np.linalg.det(matrix)),
            "sample_count": len(raw),
            "axis_span_uT": np.ptp(raw, axis=0).tolist(),
            "coverage_covariance_eigenvalue_ratio": float(
                covariance_eigenvalues[0] / covariance_eigenvalues[-1]
            ),
            "raw_norm": stats(raw_norm),
            "calibrated_norm": stats(corrected_norm),
        }

        for row_index, value in zip(row_indices[patch_id], corrected):
            rows[row_index][CAL_COLUMNS[0]] = value[0]
            rows[row_index][CAL_COLUMNS[1]] = value[1]
            rows[row_index][CAL_COLUMNS[2]] = value[2]
            rows[row_index]["Norme_cal (µT)"] = float(np.linalg.norm(value))

        make_plot(raw, corrected, label, args.output_dir / f"patch_{label}.png")

    coefficients_path = args.output_dir / "magnetometer_calibration.json"
    with coefficients_path.open("w") as destination:
        json.dump(report, destination, indent=2, ensure_ascii=False)

    calibrated_csv.parent.mkdir(parents=True, exist_ok=True)
    output_fields = list(original_fields) + list(CAL_COLUMNS) + ["Norme_cal (µT)"]
    with calibrated_csv.open("w", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=output_fields)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Coefficients : {coefficients_path}")
    print(f"CSV calibré  : {calibrated_csv}")
    for label, values in report["patches"].items():
        before = values["raw_norm"]["coefficient_of_variation_percent"]
        after = values["calibrated_norm"]["coefficient_of_variation_percent"]
        print(
            f"Patch {label}: norme de référence "
            f"{values['reference_norm_uT']:.3f} µT, CV {before:.2f}% -> {after:.2f}%"
        )


if __name__ == "__main__":
    main()
