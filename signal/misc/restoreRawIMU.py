# Script made with AI to run once to revert calibration made in the device when the first exercise recording session was done.
# Rationale: evaluate the complete pipeline with python

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SIGNAL_UTILS_SRC = PROJECT_ROOT / "signal-utils" / "src"
if str(SIGNAL_UTILS_SRC) not in sys.path:
    sys.path.insert(0, str(SIGNAL_UTILS_SRC))

import numpy as np
from signal_utils import IMUSampleReader, IMUSampleWriter

GYRO_STATIONARY_BIAS = np.array(
    [-5.133702, 3.22657, -1.3970467],
    dtype=np.float32,
)

# From signal/1-calibration/3-calibration.ipynb, where a_calibrated = [a_raw, 1] @ x
ACCEL_AFFINE_X = np.array(
    [
        [0.99878374, -0.00933288, 0.02040961],
        [0.01210606, 0.99696514, -0.01862394],
        [-0.01313606, -0.00642989, 0.98043959],
        [-0.32646267, -0.16230203, -0.1406934],
    ],
    dtype=np.float32,
)
ACCEL_LINEAR = ACCEL_AFFINE_X[:3, :]
ACCEL_OFFSET = ACCEL_AFFINE_X[3, :]
ACCEL_LINEAR_INV = np.linalg.inv(ACCEL_LINEAR)

DEFAULT_CAPTURE_DIRECTORIES = [
    PROJECT_ROOT / "3-orientation" / "capture",
    PROJECT_ROOT / "exercise-reception" / "capture-apr-28-2026",
]
DEFAULT_OUTPUT_SUFFIX = "-raw-restored"
REVERT_ACCEL_AFFINE = True


def restore_raw_acceleration(calibrated_acceleration: np.ndarray) -> np.ndarray:
    return (calibrated_acceleration - ACCEL_OFFSET) @ ACCEL_LINEAR_INV


def iter_input_files(directory: Path, suffix: str) -> list[Path]:
    return sorted(
        path for path in directory.glob("*.csv") if not path.stem.endswith(suffix)
    )


def build_output_path(input_path: Path, suffix: str) -> Path:
    return input_path.with_name(f"{input_path.stem}{suffix}{input_path.suffix}")


def process_file(
    input_path: Path,
    output_path: Path,
    reader: IMUSampleReader,
    writer: IMUSampleWriter,
    revert_affine: bool,
) -> None:
    seq, acceleration, angular_velocity = reader.read(str(input_path))

    restored_acceleration = acceleration
    if revert_affine:
        restored_acceleration = restore_raw_acceleration(acceleration)

    restored_angular_velocity = angular_velocity + GYRO_STATIONARY_BIAS
    writer.write(
        str(output_path),
        seq,
        restored_acceleration.astype(np.float32, copy=False),
        restored_angular_velocity.astype(np.float32, copy=False),
    )


def main() -> int:
    reader = IMUSampleReader()
    writer = IMUSampleWriter()

    processed_files = 0
    for directory in DEFAULT_CAPTURE_DIRECTORIES:
        resolved_directory = directory.resolve()
        if not resolved_directory.exists():
            print(f"Skipping missing directory: {resolved_directory}")
            continue
        if not resolved_directory.is_dir():
            print(f"Skipping non-directory path: {resolved_directory}")
            continue

        input_files = iter_input_files(resolved_directory, DEFAULT_OUTPUT_SUFFIX)
        if not input_files:
            print(f"No input CSV files found in: {resolved_directory}")
            continue

        for input_path in input_files:
            output_path = build_output_path(input_path, DEFAULT_OUTPUT_SUFFIX)
            process_file(
                input_path,
                output_path,
                reader,
                writer,
                REVERT_ACCEL_AFFINE,
            )
            print(f"Saved {output_path}")
            processed_files += 1

    if processed_files == 0:
        print("No files were processed.")
        return 1

    print(
        "Restored raw gyroscope data"
        + (
            " and reverted accelerometer affine calibration."
            if REVERT_ACCEL_AFFINE
            else "."
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
