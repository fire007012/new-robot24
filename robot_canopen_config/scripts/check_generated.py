#!/usr/bin/env python3
"""Check generated DCF against the archived bus and installed bin layout."""

import argparse
import configparser
from pathlib import Path


NODE_FILES = {
    1: "left_front_arm_joint.bin",
    2: "right_front_arm_joint.bin",
    3: "left_rear_arm_joint.bin",
    4: "right_rear_arm_joint.bin",
    5: "shoulder_pitch_joint.bin",
    6: "shoulder_yaw_joint.bin",
}


def load_dcf(path):
    parser = configparser.ConfigParser(interpolation=None)
    parser.optionxform = str
    with path.open(encoding="utf-8") as stream:
        parser.read_file(stream)
    return parser


def check(legacy_path, generated_path, bin_dir):
    legacy = load_dcf(legacy_path)
    generated = load_dcf(generated_path)
    ignored = {"OptionalObjects"}
    ignored.update(f"1F22sub{node_id:X}" for node_id in NODE_FILES)
    for section in legacy.sections():
        if section in ignored:
            continue
        if section not in generated or dict(legacy[section]) != dict(generated[section]):
            raise ValueError(f"legacy DCF section changed: {section}")

    for node_id, filename in NODE_FILES.items():
        section = f"1F22sub{node_id:X}"
        configured = generated[section]["UploadFile"]
        if configured != f"./{filename}":
            raise ValueError(f"{section} points outside the installed config directory: {configured}")
        if not (bin_dir / filename).is_file():
            raise ValueError(f"missing generated configuration: {filename}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("legacy", type=Path)
    parser.add_argument("generated", type=Path)
    parser.add_argument("bin_dir", type=Path)
    args = parser.parse_args()
    check(args.legacy, args.generated, args.bin_dir)


if __name__ == "__main__":
    main()
