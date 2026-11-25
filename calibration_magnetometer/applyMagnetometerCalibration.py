#!/usr/bin/env python3
"""Applique une calibration magnétomètre et trace brut/calibré par patch.

La calibration attendue dans le JSON suit la formule :

    m_cal = soft_iron_matrix @ (m_raw - hard_iron_bias_uT)

Les colonnes magnétiques brutes sont conservées dans le CSV de sortie.
"""

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


RAW_COLUMNS = ("magX (µT)", "magY (µT)", "magZ (µT)")
CAL_COLUMNS = ("magX_cal (µT)", "magY_cal (µT)", "magZ_cal (µT)")
CAL_NORM_COLUMN = "Norme_cal (µT)"
DEFAULT_COEFFICIENTS = Path(__file__).resolve().parent / "magnetometer_calibration.json"


def load_calibrations(path):
    with path.open() as source:
        document = json.load(source)

    calibrations = {}
    for patch_label, values in document.get("patches", {}).items():
        bias = np.asarray(values["hard_iron_bias_uT"], dtype=float)
        matrix = np.asarray(values["soft_iron_matrix"], dtype=float)
        if bias.shape != (3,) or matrix.shape != (3, 3):
            raise ValueError(f"coefficients invalides pour le patch {patch_label}")
        calibrations[patch_label] = (bias, matrix)

    if not calibrations:
        raise ValueError("aucun coefficient de patch trouvé dans le JSON")
    return calibrations


def patch_label(row):
    return f"{int(row['ID1'])}-{int(row['ID2'])}"


def plot_patch(raw, calibrated, label, frequency, destination, keep_open=False):
    time = np.arange(len(raw), dtype=float) / frequency
    raw_norm = np.linalg.norm(raw, axis=1)
    calibrated_norm = np.linalg.norm(calibrated, axis=1)

    figure, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    names = ("X", "Y", "Z")
    for index, name in enumerate(names):
        axes[index].plot(time, raw[:, index], linewidth=0.8, alpha=0.75,
                         label=f"mag{name} brut")
        axes[index].plot(time, calibrated[:, index], linewidth=0.8, alpha=0.85,
                         label=f"mag{name} calibré")
        axes[index].set_ylabel(f"{name} (µT)")
        axes[index].grid(True, alpha=0.25)
        axes[index].legend(loc="upper right")

    axes[3].plot(time, raw_norm, linewidth=0.8, alpha=0.75, label="norme brute")
    axes[3].plot(time, calibrated_norm, linewidth=0.8, alpha=0.85,
                 label="norme calibrée")
    axes[3].set_ylabel("Norme (µT)")
    axes[3].set_xlabel("Temps depuis le début du patch (s)")
    axes[3].grid(True, alpha=0.25)
    axes[3].legend(loc="upper right")

    figure.suptitle(f"Magnétomètre brut et calibré — patch {label}")
    figure.tight_layout()
    figure.savefig(destination, dpi=160)
    if not keep_open:
        plt.close(figure)
    return figure


def main():
    parser = argparse.ArgumentParser(
        description="Applique les coefficients hard-iron/soft-iron aux données CSV"
    )
    parser.add_argument("input_csv", type=Path, help="CSV contenant magX, magY et magZ")
    parser.add_argument(
        "--coefficients",
        type=Path,
        default=DEFAULT_COEFFICIENTS,
        help="fichier JSON produit par calibrateMagnetometer.py",
    )
    parser.add_argument(
        "--output-csv",
        type=Path,
        help="CSV de sortie (défaut : <nom>_calibrated.csv)",
    )
    parser.add_argument(
        "--plot-dir",
        type=Path,
        help="dossier des graphes (défaut : <nom>_calibration_plots)",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=200.0,
        help="fréquence par patch utilisée pour l'axe temporel (défaut : 200 Hz)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="sauvegarde les graphes sans ouvrir les fenêtres matplotlib",
    )
    args = parser.parse_args()

    if args.frequency <= 0:
        raise SystemExit("la fréquence doit être strictement positive")

    output_csv = args.output_csv or args.input_csv.with_name(
        f"{args.input_csv.stem}_calibrated.csv"
    )
    plot_dir = args.plot_dir or args.input_csv.with_name(
        f"{args.input_csv.stem}_calibration_plots"
    )
    calibrations = load_calibrations(args.coefficients)

    with args.input_csv.open(newline="") as source:
        reader = csv.DictReader(source)
        rows = list(reader)
        original_fields = list(reader.fieldnames or [])
    if not rows:
        raise SystemExit("le CSV d'entrée est vide")

    required = {"ID1", "ID2", *RAW_COLUMNS}
    missing = required.difference(original_fields)
    if missing:
        raise SystemExit(f"colonnes absentes : {', '.join(sorted(missing))}")

    indices_by_patch = {}
    raw_by_patch = {}
    for index, row in enumerate(rows):
        label = patch_label(row)
        indices_by_patch.setdefault(label, []).append(index)

    unknown = sorted(set(indices_by_patch).difference(calibrations))
    if unknown:
        raise SystemExit(
            "coefficients absents pour les patchs : " + ", ".join(unknown)
        )

    calibrated_by_patch = {}
    for label, indices in indices_by_patch.items():
        raw = np.asarray(
            [[float(rows[index][column]) for column in RAW_COLUMNS]
             for index in indices],
            dtype=float,
        )
        bias, matrix = calibrations[label]
        calibrated = (raw - bias) @ matrix.T
        raw_by_patch[label] = raw
        calibrated_by_patch[label] = calibrated

        for row_index, values in zip(indices, calibrated):
            rows[row_index][CAL_COLUMNS[0]] = f"{values[0]:.12g}"
            rows[row_index][CAL_COLUMNS[1]] = f"{values[1]:.12g}"
            rows[row_index][CAL_COLUMNS[2]] = f"{values[2]:.12g}"
            rows[row_index][CAL_NORM_COLUMN] = f"{np.linalg.norm(values):.12g}"

    added_fields = [*CAL_COLUMNS, CAL_NORM_COLUMN]
    output_fields = original_fields + [
        field for field in added_fields if field not in original_fields
    ]
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=output_fields)
        writer.writeheader()
        writer.writerows(rows)

    plot_dir.mkdir(parents=True, exist_ok=True)
    open_figures = []
    for label in sorted(raw_by_patch, key=lambda value: int(value.split("-")[0])):
        open_figures.append(
            plot_patch(
                raw_by_patch[label],
                calibrated_by_patch[label],
                label,
                args.frequency,
                plot_dir / f"patch_{label}_magnetometer_raw_vs_calibrated.png",
                keep_open=not args.no_show,
            )
        )

    print(f"CSV calibré : {output_csv}")
    print(f"Graphes     : {plot_dir}")
    for label in sorted(raw_by_patch, key=lambda value: int(value.split("-")[0])):
        raw_norm = np.linalg.norm(raw_by_patch[label], axis=1)
        calibrated_norm = np.linalg.norm(calibrated_by_patch[label], axis=1)
        print(
            f"Patch {label}: norme brute {raw_norm.mean():.3f} ± {raw_norm.std():.3f} µT; "
            f"calibrée {calibrated_norm.mean():.3f} ± {calibrated_norm.std():.3f} µT"
        )

    if not args.no_show:
        print("Fermer les fenêtres des graphes pour terminer le programme.")
        plt.show()


if __name__ == "__main__":
    main()
