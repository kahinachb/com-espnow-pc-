#!/usr/bin/env python3
"""Trace les acquisitions IMU avant toute calibration."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


ORIENTATION_ORDER = (
    "face_up",
    "face_down",
    "toes_up",
    "heel_up",
    "medial_up",
    "lateral_up",
)
PATCHES = {
    9: "Patch 1 — talon (ID1=1)",
    11: "Patch 2 (ID1=3)",
    13: "Patch 3 (ID1=5)",
    15: "Patch 4 — orteils (ID1=7)",
}
ACC_COLUMNS = ("accX (g)", "accY (g)", "accZ (g)")
GYR_COLUMNS = ("gyX (°/s)", "gyY (°/s)", "gyZ (°/s)")
COLORS = ("tab:red", "tab:green", "tab:blue")


def read_by_patch(csv_path: Path) -> dict[int, list[dict[str, str]]]:
    with csv_path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    return {
        identifier: [row for row in rows if int(row["ID1"]) == identifier]
        for identifier in PATCHES
    }


def plot_orientation(
    csv_path: Path,
    output_dir: Path,
    frequency_hz: float,
) -> tuple[Path, dict[int, int]]:
    by_patch = read_by_patch(csv_path)
    figure, axes = plt.subplots(4, 2, figsize=(15, 14), sharex=False)
    figure.suptitle(
        f"IMU gauche avant calibration — {csv_path.stem}",
        fontsize=16,
    )

    counts: dict[int, int] = {}
    for row_index, (identifier, patch_name) in enumerate(PATCHES.items()):
        rows = by_patch[identifier]
        counts[identifier] = len(rows)
        axis_acc, axis_gyr = axes[row_index]
        if not rows:
            axis_acc.text(0.5, 0.5, "Aucune donnée", ha="center", va="center")
            axis_gyr.text(0.5, 0.5, "Aucune donnée", ha="center", va="center")
            continue

        time_s = [int(row["sample_index"]) / frequency_hz for row in rows]
        for column, color in zip(ACC_COLUMNS, COLORS):
            axis_acc.plot(
                time_s,
                [float(row[column]) for row in rows],
                color=color,
                linewidth=0.75,
                label=column[:4],
            )
        for column, color in zip(GYR_COLUMNS, COLORS):
            axis_gyr.plot(
                time_s,
                [float(row[column]) for row in rows],
                color=color,
                linewidth=0.75,
                label=column[:3],
            )

        axis_acc.axhline(0, color="black", linewidth=0.6, alpha=0.5)
        axis_acc.axhline(1, color="black", linewidth=0.6, linestyle=":", alpha=0.5)
        axis_acc.axhline(-1, color="black", linewidth=0.6, linestyle=":", alpha=0.5)
        axis_gyr.axhline(0, color="black", linewidth=0.6, alpha=0.5)
        axis_acc.set_title(f"{patch_name} — accélération")
        axis_gyr.set_title(f"{patch_name} — gyroscope")
        axis_acc.set_ylabel("Accélération (g)")
        axis_gyr.set_ylabel("Vitesse angulaire (°/s)")
        for axis in (axis_acc, axis_gyr):
            axis.set_xlabel("Temps estimé (s)")
            axis.grid(True, alpha=0.25)
            axis.legend(loc="upper right", ncol=3, fontsize="small")

    figure.tight_layout(rect=(0, 0, 1, 0.97))
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / f"{csv_path.stem}_raw_imu.png"
    figure.savefig(output, dpi=180)
    plt.close(figure)
    return output, counts


def discover_csv_files(session_dir: Path) -> list[Path]:
    """Détecte d'abord les poses numérotées, sinon les six orientations."""
    pose_files = sorted(session_dir.glob("pose_*.csv"))
    if pose_files:
        return pose_files
    return [
        session_dir / f"{orientation}.csv"
        for orientation in ORIENTATION_ORDER
        if (session_dir / f"{orientation}.csv").exists()
    ]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Trace accéléromètre et gyroscope avant calibration, orientation par orientation."
    )
    parser.add_argument("session_dir", type=Path, help="dossier session_YYYYMMDD_HHMMSS")
    parser.add_argument(
        "--frequency",
        type=float,
        default=200.0,
        help="fréquence utilisée uniquement pour l'axe temporel (défaut : 200 Hz)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="dossier de sortie ; par défaut <session>/plots_raw",
    )
    args = parser.parse_args()
    if args.frequency <= 0:
        parser.error("--frequency doit être strictement positive")
    if not args.session_dir.is_dir():
        parser.error(f"session introuvable : {args.session_dir}")

    output_dir = args.output_dir or args.session_dir / "plots_raw"
    generated = 0
    csv_files = discover_csv_files(args.session_dir)
    for csv_path in csv_files:
        output, counts = plot_orientation(csv_path, output_dir, args.frequency)
        count_text = ", ".join(
            f"P{patch_number}={counts[identifier]}"
            for patch_number, identifier in enumerate(PATCHES, start=1)
        )
        print(f"{output.name} ({count_text})")
        generated += 1

    if generated == 0:
        parser.error(
            "aucun fichier pose_*.csv ni CSV des six orientations trouvé dans la session"
        )
    print(f"{generated} graphique(s) sauvegardé(s) dans {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
