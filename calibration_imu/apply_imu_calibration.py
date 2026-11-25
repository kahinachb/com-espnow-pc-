#!/usr/bin/env python3
"""Applique les calibrations accéléromètre et gyroscope à un CSV IMU.

Le CSV produit conserve toutes les colonnes d'origine et ajoute les colonnes
accX/accY/accZ_cal (g) et gyX/gyY/gyZ_cal (°/s). Les calibrations sont
appliquées dans le repère natif de chaque IMU, avant une éventuelle conversion
vers le repère commun de la semelle.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


PATCH_NAMES = {
    1: "Patch 1 — talon",
    3: "Patch 2",
    5: "Patch 3",
    7: "Patch 4 — orteils",
}
ACC_RAW_COLUMNS = ["accX (g)", "accY (g)", "accZ (g)"]
GYRO_RAW_COLUMNS = ["gyX (°/s)", "gyY (°/s)", "gyZ (°/s)"]
ACC_CAL_COLUMNS = ["accX_cal (g)", "accY_cal (g)", "accZ_cal (g)"]
GYRO_CAL_COLUMNS = ["gyX_cal (°/s)", "gyY_cal (°/s)", "gyZ_cal (°/s)"]

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_ACC_CALIBRATION = (
    SCRIPT_DIR
    / "data/session_20260728_155627/calibration_acc/accelerometer_calibration.json"
)
DEFAULT_GYRO_CALIBRATION = (
    SCRIPT_DIR
    / "data/session_gyro_20260803_124950/calibration_gyro_std015_cv/gyroscope_calibration.json"
)


def load_accelerometer_calibration(
    path: Path,
) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    result = {}
    for patch in document["patches"].values():
        identifier = int(patch["ID1"])
        matrix = np.asarray(patch["correction_matrix"], dtype=float)
        bias = np.asarray(
            [patch["additive_bias_g"][axis] for axis in "xyz"], dtype=float
        )
        result[identifier] = matrix, bias
    return result


def load_gyroscope_calibration(
    path: Path,
) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    result = {}
    for patch in document["patches"].values():
        identifier = int(patch["ID1"])
        matrix = np.asarray(patch["correction_matrix"], dtype=float)
        bias = np.asarray(
            [patch["bias_dps"][axis] for axis in "xyz"], dtype=float
        )
        result[identifier] = matrix, bias
    return result


def validate_calibrations(
    acc: dict[int, tuple[np.ndarray, np.ndarray]],
    gyro: dict[int, tuple[np.ndarray, np.ndarray]],
) -> None:
    expected = set(PATCH_NAMES)
    for name, calibration in (("accéléromètre", acc), ("gyroscope", gyro)):
        missing = expected - set(calibration)
        if missing:
            raise ValueError(f"calibration {name} incomplète, ID1 manquants : {sorted(missing)}")
        for identifier, (matrix, bias) in calibration.items():
            if matrix.shape != (3, 3) or bias.shape != (3,):
                raise ValueError(f"dimensions invalides pour {name}, ID1={identifier}")


def estimate_sample_period(data: pd.DataFrame) -> float:
    """Estime la période depuis les timestamps du patch ID1=7."""
    timestamps = data.loc[data["ID1"] == 7, "Temps"].to_numpy(dtype=np.uint64)
    if len(timestamps) < 2:
        raise ValueError("pas assez de timestamps pour estimer la fréquence")
    intervals = ((timestamps[1:] - timestamps[:-1]) & np.uint64(0xFFFFFFFF)) / 1e6
    valid = intervals[(intervals > 0.0) & (intervals < 0.05)]
    if len(valid) == 0:
        raise ValueError("impossible d'estimer la fréquence depuis la colonne Temps")
    return float(np.median(valid))


def calibrate(
    data: pd.DataFrame,
    acc_calibrations: dict[int, tuple[np.ndarray, np.ndarray]],
    gyro_calibrations: dict[int, tuple[np.ndarray, np.ndarray]],
) -> pd.DataFrame:
    result = data.copy()
    for column in ACC_CAL_COLUMNS + GYRO_CAL_COLUMNS:
        result[column] = np.nan

    for identifier in PATCH_NAMES:
        mask = result["ID1"] == identifier
        if not mask.any():
            continue

        raw_acc = result.loc[mask, ACC_RAW_COLUMNS].to_numpy(dtype=float)
        acc_matrix, additive_bias = acc_calibrations[identifier]
        calibrated_acc = (acc_matrix @ (raw_acc + additive_bias).T).T
        result.loc[mask, ACC_CAL_COLUMNS] = calibrated_acc

        raw_gyro = result.loc[mask, GYRO_RAW_COLUMNS].to_numpy(dtype=float)
        gyro_matrix, subtractive_bias = gyro_calibrations[identifier]
        calibrated_gyro = (gyro_matrix @ (raw_gyro - subtractive_bias).T).T
        result.loc[mask, GYRO_CAL_COLUMNS] = calibrated_gyro

    return result


def plot_acceleration_comparison(
    data: pd.DataFrame,
    output_dir: Path,
    sample_period: float,
    image_format: str,
    dpi: int,
    save_plots: bool,
    show_plots: bool,
) -> None:
    if save_plots:
        output_dir.mkdir(parents=True, exist_ok=True)
    for patch_number, (identifier, patch_name) in enumerate(PATCH_NAMES.items(), start=1):
        patch = data.loc[data["ID1"] == identifier]
        if patch.empty:
            print(f"Attention : aucune donnée pour ID1={identifier}, figure ignorée")
            continue
        time = np.arange(len(patch), dtype=float) * sample_period
        raw = patch[ACC_RAW_COLUMNS].to_numpy(dtype=float)
        calibrated = patch[ACC_CAL_COLUMNS].to_numpy(dtype=float)

        figure, axes = plt.subplots(
            3, 1, figsize=(16, 10), sharex=True, constrained_layout=True
        )
        for axis_index, (axis, axis_name) in enumerate(zip(axes, "XYZ")):
            axis.plot(time, raw[:, axis_index], label="Brute", linewidth=0.9, alpha=0.8)
            axis.plot(
                time,
                calibrated[:, axis_index],
                label="Calibrée",
                linewidth=0.9,
                alpha=0.9,
            )
            axis.set_ylabel(f"Acc {axis_name} (g)")
            axis.grid(True, alpha=0.3)
            axis.legend(loc="upper right")
        axes[-1].set_xlabel("Temps (s)")
        figure.suptitle(
            f"{patch_name} (ID1={identifier}) — accélération brute et calibrée"
        )
        if save_plots:
            figure.savefig(
                output_dir / f"patch_{patch_number}_acc_raw_vs_calibrated.{image_format}",
                dpi=dpi,
                bbox_inches="tight",
            )
        if not show_plots:
            plt.close(figure)

        raw_gyro = patch[GYRO_RAW_COLUMNS].to_numpy(dtype=float)
        calibrated_gyro = patch[GYRO_CAL_COLUMNS].to_numpy(dtype=float)
        gyro_figure, gyro_axes = plt.subplots(
            3, 1, figsize=(16, 10), sharex=True, constrained_layout=True
        )
        for axis_index, (axis, axis_name) in enumerate(zip(gyro_axes, "XYZ")):
            axis.plot(
                time,
                raw_gyro[:, axis_index],
                label="Brut",
                linewidth=0.9,
                alpha=0.8,
            )
            axis.plot(
                time,
                calibrated_gyro[:, axis_index],
                label="Calibré",
                linewidth=0.9,
                alpha=0.9,
            )
            axis.set_ylabel(f"Gyro {axis_name} (°/s)")
            axis.grid(True, alpha=0.3)
            axis.legend(loc="upper right")
        gyro_axes[-1].set_xlabel("Temps (s)")
        gyro_figure.suptitle(
            f"{patch_name} (ID1={identifier}) — gyroscope brut et calibré"
        )
        if save_plots:
            gyro_figure.savefig(
                output_dir / f"patch_{patch_number}_gyro_raw_vs_calibrated.{image_format}",
                dpi=dpi,
                bbox_inches="tight",
            )
        if not show_plots:
            plt.close(gyro_figure)

    if show_plots:
        plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Calibre les données acc/gyro d'un CSV et trace les deux capteurs."
    )
    parser.add_argument("input", type=Path, help="CSV contenant les données IMU brutes")
    parser.add_argument(
        "--acc-calibration", type=Path, default=DEFAULT_ACC_CALIBRATION
    )
    parser.add_argument(
        "--gyro-calibration", type=Path, default=DEFAULT_GYRO_CALIBRATION
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="CSV de sortie (défaut : <entrée>_calibrated.csv)",
    )
    parser.add_argument(
        "--plots-dir",
        type=Path,
        help="dossier des figures (défaut : <entrée>_calibration_plots)",
    )
    parser.add_argument("--format", choices=("png", "svg", "pdf"), default="png")
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument(
        "--show",
        action="store_true",
        help="affiche les figures dans des fenêtres interactives",
    )
    parser.add_argument(
        "--no-save-plots",
        action="store_true",
        help="n'enregistre pas les figures (à combiner avec --show)",
    )
    args = parser.parse_args()

    for path, label in (
        (args.input, "CSV d'entrée"),
        (args.acc_calibration, "calibration accéléromètre"),
        (args.gyro_calibration, "calibration gyroscope"),
    ):
        if not path.is_file():
            parser.error(f"{label} introuvable : {path}")

    data = pd.read_csv(args.input)
    required = {"ID1", "Temps", *ACC_RAW_COLUMNS, *GYRO_RAW_COLUMNS}
    missing = required - set(data.columns)
    if missing:
        parser.error(f"colonnes manquantes : {sorted(missing)}")
    if data.empty:
        parser.error("le CSV d'entrée est vide")
    unknown_ids = sorted(set(data["ID1"].dropna().astype(int)) - set(PATCH_NAMES))
    if unknown_ids:
        parser.error(f"ID1 non pris en charge : {unknown_ids}")

    acc_calibrations = load_accelerometer_calibration(args.acc_calibration)
    gyro_calibrations = load_gyroscope_calibration(args.gyro_calibration)
    validate_calibrations(acc_calibrations, gyro_calibrations)
    calibrated = calibrate(data, acc_calibrations, gyro_calibrations)

    output = args.output or args.input.with_name(f"{args.input.stem}_calibrated.csv")
    plots_dir = args.plots_dir or args.input.with_name(
        f"{args.input.stem}_calibration_plots"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    calibrated.to_csv(output, index=False)

    sample_period = estimate_sample_period(calibrated)
    plot_acceleration_comparison(
        calibrated,
        plots_dir,
        sample_period,
        args.format,
        args.dpi,
        save_plots=not args.no_save_plots,
        show_plots=args.show,
    )

    print(f"Lignes traitées : {len(calibrated)}")
    print(f"Fréquence estimée : {1.0 / sample_period:.3f} Hz")
    print(f"CSV calibré : {output}")
    if not args.no_save_plots:
        print(f"Figures : {plots_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
