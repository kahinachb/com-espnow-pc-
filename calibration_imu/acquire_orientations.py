#!/usr/bin/env python3
"""Acquisition interactive des six orientations pour calibrer les IMU gauches."""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import termios
import time
import tty
from datetime import datetime, timezone
from pathlib import Path

import serial
from serial.tools import list_ports


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DECODER_DIR = PROJECT_ROOT / "codePlusieursPatchs"
sys.path.insert(0, str(DECODER_DIR))

from process_data.decodageToCSV import get_converted_data  # noqa: E402


ORIENTATIONS = (
    ("face_up", "semelle à plat, face capteurs vers le haut"),
    ("face_down", "semelle retournée, face capteurs vers le bas"),
    ("toes_up", "orteils vers le haut"),
    ("heel_up", "talon vers le haut"),
    ("medial_up", "bord médial (intérieur du pied gauche) vers le haut"),
    ("lateral_up", "bord latéral (extérieur du pied gauche) vers le haut"),
)
LEFT_IDS = {9, 11, 13, 15} # left : 1 3 5 7 , right : 9 11 13 15
CSV_COLUMNS = (
    "orientation",
    "sample_index",
    "ID1",
    "ID2",
    "CPT",
    "accX (g)",
    "accY (g)",
    "accZ (g)",
    "gyX (°/s)",
    "gyY (°/s)",
    "gyZ (°/s)",
    "Temps",
)


def immediate_key() -> str:
    """Lit une touche sans demander Entrée (terminal Linux/macOS)."""
    if not sys.stdin.isatty():
        raise RuntimeError("l'entrée standard doit être un terminal interactif")
    descriptor = sys.stdin.fileno()
    previous = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        return os.read(descriptor, 1).decode(errors="ignore").lower()
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, previous)


def choose_port(requested_port: str | None) -> str:
    if requested_port:
        return requested_port
    available = [port.device for port in list_ports.comports()]
    if len(available) == 1:
        return available[0]
    if not available:
        raise RuntimeError("aucun port série détecté ; utilisez --port")
    raise RuntimeError(
        "plusieurs ports série détectés : "
        + ", ".join(available)
        + " ; sélectionnez-en un avec --port"
    )


def record_bytes(connection: serial.Serial, duration_s: float) -> tuple[bytes, str, str]:
    """Vide l'ancien flux et enregistre exactement à partir du déclenchement."""
    connection.reset_input_buffer()
    start_wall = datetime.now(timezone.utc)
    start = time.monotonic()
    deadline = start + duration_s
    chunks: list[bytes] = []

    while time.monotonic() < deadline:
        waiting = connection.in_waiting
        chunk = connection.read(waiting if waiting else 1)
        if chunk:
            chunks.append(chunk)

    end_wall = datetime.now(timezone.utc)
    return (
        b"".join(chunks),
        start_wall.isoformat(timespec="microseconds"),
        end_wall.isoformat(timespec="microseconds"),
    )


def write_left_imu_csv(raw_data: bytes, orientation: str, output: Path) -> int:
    decoded = get_converted_data(raw_data)
    header = decoded[0]
    column = {name: index for index, name in enumerate(header)}
    sample_indexes = {identifier: 0 for identifier in LEFT_IDS}
    written = 0

    with output.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for row in decoded[1:]:
            identifier = int(row[column["ID1"]])
            if identifier not in LEFT_IDS:
                continue
            output_row = {
                "orientation": orientation,
                "sample_index": sample_indexes[identifier],
            }
            for name in CSV_COLUMNS[2:]:
                output_row[name] = row[column[name]]
            writer.writerow(output_row)
            sample_indexes[identifier] += 1
            written += 1
    return written


def acquire_orientation(
    connection: serial.Serial,
    session_dir: Path,
    orientation: str,
    description: str,
    duration_s: float,
    metadata: dict[str, object],
) -> bool:
    while True:
        print(f"\nPose : {orientation}")
        print(f"  {description}")
        print("Immobilisez la semelle, puis appuyez sur [s] pour enregistrer ou [q] pour quitter.")
        key = immediate_key()
        print()
        if key == "q":
            return False
        if key != "s":
            print(f"Touche « {key} » ignorée.")
            continue

        print(f"Acquisition pendant {duration_s:g} s…", flush=True)
        raw_data, start_utc, end_utc = record_bytes(connection, duration_s)
        raw_path = session_dir / f"{orientation}.bin"
        csv_path = session_dir / f"{orientation}.csv"
        raw_path.write_bytes(raw_data)
        row_count = write_left_imu_csv(raw_data, orientation, csv_path)

        metadata["acquisitions"][orientation] = {
            "description": description,
            "start_utc": start_utc,
            "end_utc": end_utc,
            "raw_bytes": len(raw_data),
            "left_imu_rows": row_count,
            "raw_file": raw_path.name,
            "csv_file": csv_path.name,
        }
        print(f"Terminé : {row_count} lignes IMU gauches dans {csv_path.name}")
        if row_count == 0:
            print("Aucune trame gauche valide : vérifiez le port et la transmission.")
            print("Appuyez sur [r] pour recommencer ou sur une autre touche pour continuer.")
            if immediate_key() == "r":
                continue
            print()
        return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Enregistre un CSV par orientation pour la calibration accéléromètre/gyroscope."
    )
    parser.add_argument("--port", help="port série, par exemple /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="durée par pose en secondes (défaut : 5)",
    )
    parser.add_argument(
        "--pose-count",
        type=int,
        default=30,
        help="nombre de poses statiques arbitraires (défaut : 30)",
    )
    parser.add_argument(
        "--six-orientations",
        action="store_true",
        help="utiliser l'ancien protocole des six orientations nommées",
    )
    parser.add_argument("--output-dir", type=Path, default=Path("calibration_imu/data"))
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration doit être strictement positive")
    if args.pose_count <= 0:
        parser.error("--pose-count doit être strictement positif")

    try:
        port = choose_port(args.port)
    except RuntimeError as error:
        parser.error(str(error))

    session_name = datetime.now().strftime("session_%Y%m%d_%H%M%S")
    session_dir = args.output_dir / session_name
    session_dir.mkdir(parents=True, exist_ok=False)
    metadata: dict[str, object] = {
        "port": port,
        "baud": args.baud,
        "duration_s": args.duration,
        "foot": "left",
        "trigger_key": "s",
        "note": "Le buffer série est vidé au moment de la pression sur s.",
        "protocol": "six_orientations" if args.six_orientations else "static_poses",
        "requested_pose_count": 6 if args.six_orientations else args.pose_count,
        "acquisitions": {},
    }

    print(f"Ouverture de {port} à {args.baud} bauds")
    print(f"Résultats : {session_dir}")
    if args.six_orientations:
        acquisitions = ORIENTATIONS
    else:
        acquisitions = tuple(
            (
                f"pose_{index:02d}",
                (
                    f"pose statique arbitraire {index}/{args.pose_count} ; "
                    "elle doit être nettement différente des précédentes"
                ),
            )
            for index in range(1, args.pose_count + 1)
        )
        print(
            f"Protocole accéléromètre : {args.pose_count} poses statiques arbitraires.\n"
            "Répartissez les orientations dans toutes les directions 3D ; "
            "évitez de répéter des poses presque identiques."
        )
    try:
        with serial.Serial(port, args.baud, timeout=0.02) as connection:
            time.sleep(0.3)
            for orientation, description in acquisitions:
                if not acquire_orientation(
                    connection,
                    session_dir,
                    orientation,
                    description,
                    args.duration,
                    metadata,
                ):
                    break
    except serial.SerialException as error:
        print(f"Erreur série : {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nAcquisition interrompue.")
    finally:
        metadata_path = session_dir / "metadata.json"
        metadata_path.write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

    print(f"\nSession sauvegardée dans {session_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
