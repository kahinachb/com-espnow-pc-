#!/usr/bin/env python3
"""Variance/déviation d'Allan des gyroscopes sur une acquisition statique."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


PATCHES = {
    1: "Patch 1 — talon",
    3: "Patch 2",
    5: "Patch 3",
    7: "Patch 4 — orteils",
}
GYR_COLUMNS = ("gyX (°/s)", "gyY (°/s)", "gyZ (°/s)")
AXES = ("X", "Y", "Z")


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def estimate_sample_period(rows: list[dict[str, str]]) -> float:
    toe_rows = [row for row in rows if int(row["ID1"]) == 7]
    raw = [int(row["Temps"]) for row in toe_rows]
    intervals = [
        ((end - start) & 0xFFFFFFFF) / 1e6
        for start, end in zip(raw, raw[1:])
    ]
    valid = [interval for interval in intervals if 0 < interval < 0.05]
    if not valid:
        raise ValueError("impossible d'estimer la période d'échantillonnage")
    # La moyenne restitue ici la durée réelle de l'enregistrement. La médiane
    # surestimait la durée lorsque les intervalles alternaient autour de 5 ms.
    return float(np.mean(valid))


def logarithmic_cluster_sizes(maximum: int, count: int = 45) -> np.ndarray:
    if maximum < 1:
        raise ValueError("acquisition trop courte")
    return np.unique(
        np.maximum(
            1,
            np.rint(np.logspace(0, math.log10(maximum), count)).astype(int),
        )
    )


def overlapping_allan_variance(
    signal: np.ndarray,
    cluster_size: int,
) -> tuple[float, int]:
    """Variance d'Allan chevauchante de données de vitesse."""
    kernel = np.ones(cluster_size, dtype=float) / cluster_size
    cluster_averages = np.convolve(signal, kernel, mode="valid")
    differences = (
        cluster_averages[cluster_size:]
        - cluster_averages[:-cluster_size]
    )
    if len(differences) < 2:
        raise ValueError("nombre de paires insuffisant")
    return 0.5 * float(np.mean(np.square(differences))), len(differences)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Calcule et trace la variance d'Allan des quatre gyroscopes gauches."
    )
    parser.add_argument(
        "input",
        type=Path,
        help="initial_static.csv ou dossier session_gyro_*",
    )
    parser.add_argument(
        "--max-tau",
        type=float,
        help="tau maximal ; par défaut durée/10",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="défaut : <session>/allan_gyro",
    )
    args = parser.parse_args()
    input_csv = (
        args.input / "initial_static.csv"
        if args.input.is_dir()
        else args.input
    )
    if not input_csv.exists():
        parser.error(f"fichier introuvable : {input_csv}")

    rows = read_csv(input_csv)
    sample_period = estimate_sample_period(rows)
    sample_frequency = 1.0 / sample_period
    counts = [
        sum(int(row["ID1"]) == identifier for row in rows)
        for identifier in PATCHES
    ]
    sample_count = min(counts)
    duration = (sample_count - 1) * sample_period
    max_tau = args.max_tau if args.max_tau is not None else duration / 10.0
    if max_tau <= 0 or max_tau > duration / 2:
        parser.error("--max-tau doit être positif et inférieur à durée/2")
    max_cluster = min(int(max_tau / sample_period), sample_count // 2 - 1)
    cluster_sizes = logarithmic_cluster_sizes(max_cluster)
    output_dir = args.output_dir or input_csv.parent / "allan_gyro"
    output_dir.mkdir(parents=True, exist_ok=True)

    results = []
    figure, axes = plt.subplots(2, 2, figsize=(14, 10), sharex=True, sharey=True)
    minima = {}
    for axis_plot, (identifier, patch_name) in zip(axes.flat, PATCHES.items()):
        patch_rows = [row for row in rows if int(row["ID1"]) == identifier]
        minima[identifier] = {}
        for column, axis_name in zip(GYR_COLUMNS, AXES):
            signal = np.asarray(
                [float(row[column]) for row in patch_rows[:sample_count]],
                dtype=float,
            )
            taus = []
            deviations = []
            for cluster_size in cluster_sizes:
                variance, pair_count = overlapping_allan_variance(
                    signal,
                    int(cluster_size),
                )
                tau = cluster_size * sample_period
                deviation = math.sqrt(variance)
                taus.append(tau)
                deviations.append(deviation)
                results.append(
                    {
                        "patch": patch_name,
                        "ID1": identifier,
                        "axis": axis_name,
                        "tau_s": tau,
                        "cluster_samples": int(cluster_size),
                        "pair_count": pair_count,
                        "allan_variance_dps2": variance,
                        "allan_deviation_dps": deviation,
                    }
                )
            minimum_index = int(np.argmin(deviations))
            minima[identifier][axis_name] = {
                "tau_s": taus[minimum_index],
                "allan_deviation_dps": deviations[minimum_index],
            }
            axis_plot.loglog(
                taus,
                deviations,
                marker="o",
                markersize=3,
                linewidth=1,
                label=f"Axe {axis_name}",
            )
            axis_plot.scatter(
                [taus[minimum_index]],
                [deviations[minimum_index]],
                marker="x",
                s=55,
            )
        axis_plot.set_title(f"{patch_name} (ID1={identifier})")
        axis_plot.set_xlabel(r"Temps d'intégration $\tau$ (s)")
        axis_plot.set_ylabel("Déviation d'Allan (°/s)")
        axis_plot.grid(True, which="both", alpha=0.3)
        axis_plot.legend()

    with (output_dir / "allan_variance.csv").open(
        "w", newline="", encoding="utf-8"
    ) as file:
        writer = csv.DictWriter(file, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)
    figure.suptitle(
        f"Déviation d'Allan des gyroscopes — {duration:.1f} s à {sample_frequency:.2f} Hz\n"
        f"Analyse préliminaire limitée à τ ≤ {max_tau:.2f} s",
        fontsize=15,
    )
    figure.tight_layout(rect=(0, 0, 1, 0.94))
    figure.savefig(output_dir / "allan_deviation.png", dpi=180)
    plt.close(figure)

    print(f"Durée analysée : {duration:.3f} s")
    print(f"Fréquence estimée : {sample_frequency:.3f} Hz")
    print(f"Tau analysé : {cluster_sizes[0] * sample_period:.6f} à {max_tau:.6f} s")
    for patch_number, identifier in enumerate(PATCHES, start=1):
        text = ", ".join(
            (
                f"{axis}: min={values['allan_deviation_dps']:.6f} °/s "
                f"à τ={values['tau_s']:.3f} s"
            )
            for axis, values in minima[identifier].items()
        )
        print(f"Patch {patch_number} — {text}")
    print(f"Résultats : {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
