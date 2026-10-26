#!/usr/bin/env python3

import argparse
import csv
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


CPP_SOURCE = r"""
#include <iomanip>
#include <iostream>
#include <string>

#include <ruckig/input_parameter.hpp>
#include <ruckig/output_parameter.hpp>
#include <ruckig/ruckig.hpp>

int main(int argc, char** argv) {
  if (argc != 9) {
    std::cerr << "usage: sampler dt current_pos current_vel current_acc target_pos max_vel max_acc max_jerk\n";
    return 1;
  }

  const double dt = std::stod(argv[1]);
  const double current_pos = std::stod(argv[2]);
  const double current_vel = std::stod(argv[3]);
  const double current_acc = std::stod(argv[4]);
  const double target_pos = std::stod(argv[5]);
  const double max_vel = std::stod(argv[6]);
  const double max_acc = std::stod(argv[7]);
  const double max_jerk = std::stod(argv[8]);

  ruckig::Ruckig<1> otg(dt);
  ruckig::InputParameter<1> input;
  ruckig::OutputParameter<1> output;

  input.current_position[0] = current_pos;
  input.current_velocity[0] = current_vel;
  input.current_acceleration[0] = current_acc;
  input.target_position[0] = target_pos;
  input.target_velocity[0] = 0.0;
  input.target_acceleration[0] = 0.0;
  input.max_velocity[0] = max_vel;
  input.max_acceleration[0] = max_acc;
  input.max_jerk[0] = max_jerk;

  if (!otg.validate_input(input)) {
    std::cerr << "invalid input\n";
    return 2;
  }

  std::cout << "time,position,velocity,acceleration\n";
  std::cout << std::fixed << std::setprecision(9);

  while (true) {
    const auto result = otg.update(input, output);
    if (result < 0) {
      std::cerr << "ruckig update failed: " << static_cast<int>(result) << "\n";
      return 3;
    }

    std::cout << output.time << ","
              << output.new_position[0] << ","
              << output.new_velocity[0] << ","
              << output.new_acceleration[0] << "\n";

    output.pass_to_input(input);

    if (result == ruckig::Result::Finished) {
      break;
    }
  }

  return 0;
}
"""


def parse_args():
    parser = argparse.ArgumentParser(
        description="Sample and plot a 1-DoF Ruckig profile."
    )
    parser.add_argument("--current-pos", type=float, default=0.0)
    parser.add_argument("--current-vel", type=float, default=0.0)
    parser.add_argument("--current-acc", type=float, default=0.0)
    parser.add_argument("--target-pos", type=float, default=3.0)
    parser.add_argument("--max-vel", type=float, default=1.0)
    parser.add_argument("--max-acc", type=float, default=2.0)
    parser.add_argument("--max-jerk", type=float, default=10.0)
    parser.add_argument("--dt", type=float, default=0.01,
                        help="Sampling period in seconds.")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("/tmp/ruckig_profile.png"),
        help="Output PNG path.",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("/tmp/ruckig_profile.csv"),
        help="Output CSV path.",
    )
    return parser.parse_args()


def compile_sampler(binary_path: Path):
    with tempfile.NamedTemporaryFile("w", suffix=".cpp", delete=False) as cpp_file:
        cpp_file.write(CPP_SOURCE)
        cpp_path = Path(cpp_file.name)

    try:
        cmd = [
            "g++",
            "-std=c++17",
            "-O2",
            str(cpp_path),
            "-I/usr/local/include",
            "-L/usr/local/lib",
            "-Wl,-rpath,/usr/local/lib",
            "-lruckig",
            "-o",
            str(binary_path),
        ]
        subprocess.run(cmd, check=True)
    finally:
        cpp_path.unlink(missing_ok=True)


def run_sampler(binary_path: Path, args) -> np.ndarray:
    cmd = [
        str(binary_path),
        str(args.dt),
        str(args.current_pos),
        str(args.current_vel),
        str(args.current_acc),
        str(args.target_pos),
        str(args.max_vel),
        str(args.max_acc),
        str(args.max_jerk),
    ]
    result = subprocess.run(cmd, check=True, capture_output=True, text=True)
    rows = list(csv.DictReader(result.stdout.splitlines()))
    data = np.array(
        [
            [
                float(row["time"]),
                float(row["position"]),
                float(row["velocity"]),
                float(row["acceleration"]),
            ]
            for row in rows
        ]
    )
    return data


def save_csv(path: Path, data: np.ndarray):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "position", "velocity", "acceleration", "jerk"])
        jerk = np.zeros(len(data))
        if len(data) >= 2:
            jerk[1:] = np.diff(data[:, 3]) / np.diff(data[:, 0])
        for i, row in enumerate(data):
            writer.writerow([*row.tolist(), jerk[i]])


def plot(path: Path, data: np.ndarray, args):
    path.parent.mkdir(parents=True, exist_ok=True)
    jerk = np.zeros(len(data))
    if len(data) >= 2:
        jerk[1:] = np.diff(data[:, 3]) / np.diff(data[:, 0])

    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

    axes[0].plot(data[:, 0], data[:, 1], label="position")
    axes[0].axhline(args.target_pos, color="tab:red", linestyle="--", label="target")
    axes[0].set_ylabel("pos")
    axes[0].legend(loc="best")

    axes[1].plot(data[:, 0], data[:, 2], label="velocity", color="tab:orange")
    axes[1].axhline(args.max_vel, color="tab:red", linestyle="--", alpha=0.5)
    axes[1].axhline(-args.max_vel, color="tab:red", linestyle="--", alpha=0.5)
    axes[1].set_ylabel("vel")

    axes[2].plot(data[:, 0], data[:, 3], label="acceleration", color="tab:green")
    axes[2].axhline(args.max_acc, color="tab:red", linestyle="--", alpha=0.5)
    axes[2].axhline(-args.max_acc, color="tab:red", linestyle="--", alpha=0.5)
    axes[2].set_ylabel("acc")

    axes[3].plot(data[:, 0], jerk, label="jerk", color="tab:purple")
    axes[3].axhline(args.max_jerk, color="tab:red", linestyle="--", alpha=0.5)
    axes[3].axhline(-args.max_jerk, color="tab:red", linestyle="--", alpha=0.5)
    axes[3].set_ylabel("jerk")
    axes[3].set_xlabel("time [s]")

    title = (
        f"Ruckig 1-DoF profile: {args.current_pos:.3f} -> {args.target_pos:.3f}, "
        f"vmax={args.max_vel}, amax={args.max_acc}, jmax={args.max_jerk}, dt={args.dt}"
    )
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    args = parse_args()
    with tempfile.TemporaryDirectory(prefix="ruckig_plot_") as temp_dir:
        binary_path = Path(temp_dir) / "ruckig_sampler"
        compile_sampler(binary_path)
        data = run_sampler(binary_path, args)

    if data.size == 0:
        print("No samples generated.", file=sys.stderr)
        return 1

    save_csv(args.csv, data)
    plot(args.out, data, args)

    print(f"PNG: {args.out}")
    print(f"CSV: {args.csv}")
    print(f"Samples: {len(data)}")
    print(f"Duration: {data[-1,0]:.6f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
