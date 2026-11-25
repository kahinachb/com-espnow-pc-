#!/usr/bin/env python3
"""Acquisition guidée pour la calibration multi-position des gyroscopes."""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

from .acquire_orientations import (
    choose_port,
    immediate_key,
    record_bytes,
    write_left_imu_csv,
)


def wait_for_start(title: str, instructions: str) -> bool:
    while True:
        print(f"\n{title}")
        print(instructions)
        print("Appuyez sur [s] pour démarrer ou [q] pour terminer la session.")
        key = immediate_key()
        print()
        if key == "s":
            return True
        if key == "q":
            return False
        print(f"Touche « {key} » ignorée.")


def record_transition(
    connection: serial.Serial,
    pre_static_s: float,
    movement_s: float,
    post_static_s: float,
) -> tuple[bytes, str, str]:
    connection.reset_input_buffer()
    start_wall = datetime.now(timezone.utc)
    start = time.monotonic()
    movement_start = start + pre_static_s
    post_start = movement_start + movement_s
    deadline = post_start + post_static_s
    movement_announced = False
    post_announced = False
    chunks: list[bytes] = []

    while time.monotonic() < deadline:
        now = time.monotonic()
        if not movement_announced and now >= movement_start:
            print(
                "\n\a>>> BOUGEZ : tournez la semelle vers une nouvelle orientation.",
                flush=True,
            )
            movement_announced = True
        if not post_announced and now >= post_start:
            print(
                "\n\a>>> IMMOBILISEZ : ne touchez plus la semelle.",
                flush=True,
            )
            post_announced = True
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


def save_acquisition(
    session_dir: Path,
    name: str,
    raw_data: bytes,
    start_utc: str,
    end_utc: str,
    metadata: dict[str, object],
    extra_metadata: dict[str, object],
) -> int:
    raw_path = session_dir / f"{name}.bin"
    csv_path = session_dir / f"{name}.csv"
    raw_path.write_bytes(raw_data)
    row_count = write_left_imu_csv(raw_data, name, csv_path)
    metadata["acquisitions"][name] = {
        "start_utc": start_utc,
        "end_utc": end_utc,
        "raw_bytes": len(raw_data),
        "left_imu_rows": row_count,
        "raw_file": raw_path.name,
        "csv_file": csv_path.name,
        **extra_metadata,
    }
    return row_count


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Acquisition statique initiale et transitions pour calibrer les gyroscopes."
    )
    parser.add_argument("--port", help="port série, par exemple /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument("--transitions", type=int, default=30)
    parser.add_argument("--initial-duration", type=float, default=50.0)
    parser.add_argument("--pre-static", type=float, default=2.0)
    parser.add_argument("--movement", type=float, default=3.0)
    parser.add_argument("--post-static", type=float, default=3.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("calibration_imu/data"),
    )
    args = parser.parse_args()
    durations = (
        args.initial_duration,
        args.pre_static,
        args.movement,
        args.post_static,
    )
    if any(duration <= 0 for duration in durations):
        parser.error("toutes les durées doivent être strictement positives")
    if args.transitions <= 0:
        parser.error("--transitions doit être strictement positif")
    try:
        port = choose_port(args.port)
    except RuntimeError as error:
        parser.error(str(error))

    session_name = datetime.now().strftime("session_gyro_%Y%m%d_%H%M%S")
    session_dir = args.output_dir / session_name
    session_dir.mkdir(parents=True, exist_ok=False)
    metadata: dict[str, object] = {
        "protocol": "gyroscope_tedaldi_2014",
        "port": port,
        "baud": args.baud,
        "foot": "left",
        "trigger_key": "s",
        "requested_transitions": args.transitions,
        "initial_duration_s": args.initial_duration,
        "pre_static_s": args.pre_static,
        "movement_s": args.movement,
        "post_static_s": args.post_static,
        "acquisitions": {},
    }

    print(f"Ouverture de {port} à {args.baud} bauds")
    print(f"Résultats : {session_dir}")
    print(
        "\nLe protocole commence par une phase immobile pour estimer le biais.\n"
        "Pendant les transitions, effectuez des rotations variées autour des trois axes,\n"
        "sans choc et sans dépasser la durée indiquée."
    )
    try:
        with serial.Serial(port, args.baud, timeout=0.02) as connection:
            time.sleep(0.3)
            if not wait_for_start(
                "Phase initiale — biais gyroscopique",
                (
                    f"Posez la semelle et ne la touchez plus pendant "
                    f"{args.initial_duration:g} secondes."
                ),
            ):
                return 0
            print(f"Acquisition immobile pendant {args.initial_duration:g} s…")
            raw, start_utc, end_utc = record_bytes(
                connection,
                args.initial_duration,
            )
            rows = save_acquisition(
                session_dir,
                "initial_static",
                raw,
                start_utc,
                end_utc,
                metadata,
                {"phase": "initial_static"},
            )
            print(f"\aPhase initiale terminée : {rows} lignes.")

            for index in range(1, args.transitions + 1):
                name = f"transition_{index:02d}"
                if not wait_for_start(
                    f"Transition {index}/{args.transitions}",
                    (
                        "Gardez d'abord la semelle immobile. Après le signal BOUGEZ, "
                        "tournez-la vers une nouvelle orientation. Au signal IMMOBILISEZ, "
                        "elle doit être posée et ne plus bouger."
                    ),
                ):
                    break
                total = args.pre_static + args.movement + args.post_static
                print(f"Enregistrement de {total:g} s — restez immobile…", flush=True)
                raw, start_utc, end_utc = record_transition(
                    connection,
                    args.pre_static,
                    args.movement,
                    args.post_static,
                )
                rows = save_acquisition(
                    session_dir,
                    name,
                    raw,
                    start_utc,
                    end_utc,
                    metadata,
                    {
                        "phase": "transition",
                        "pre_static_s": args.pre_static,
                        "movement_s": args.movement,
                        "post_static_s": args.post_static,
                    },
                )
                print(f"\aTransition terminée : {rows} lignes dans {name}.csv")
    except serial.SerialException as error:
        print(f"Erreur série : {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nAcquisition interrompue.")
    finally:
        (session_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

    print(f"\nSession gyroscope sauvegardée dans {session_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
