#!/usr/bin/env python3
"""bag_to_csv — extract the policy `~/debug` topic from a rosbag into the SAME obs
CSV format as obs_csv_logger.py / the sims, so a RECORDED run opens directly in
scripts/plot_csv_gui.py next to the sim CSVs.

The recorded bag (robot_pvt_policy.launch.py record_bag:=true) contains the
controller's ~/debug (Float32MultiArray, 100 floats) plus the raw sensor topics.
This reads ~/debug and reconstructs the exact 65-column CSV (policy_tick, 52 obs,
12 tau). The format/tau logic is REUSED from obs_csv_logger.py — no drift.

Usage:
    ros2 run harambe_policy_legs_controller bag_to_csv.py <bag_dir>
    ros2 run harambe_policy_legs_controller bag_to_csv.py <bag_dir> -o run.csv

Raw inspection of the same bag:
    ros2 bag info <bag_dir>
    ros2 bag play <bag_dir>          # republishes the topics
"""
import argparse
import csv
import datetime
import os
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# Reuse the frozen CSV format + tau reconstruction from the live logger (same
# install dir → importable without packaging). Keeps the two in lockstep.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obs_csv_logger import (  # noqa: E402
    LEG_JOINTS, NJ, OBS_DIM, DEFAULT_KP, DEFAULT_KD, DEFAULT_EFF, csv_obs_header)


def main():
    ap = argparse.ArgumentParser(
        description="Extract the policy ~/debug from a rosbag to the sim obs CSV.")
    ap.add_argument("bag", help="rosbag directory (the folder with metadata.yaml)")
    ap.add_argument("-o", "--out", default="",
                    help="output CSV (default: <bag>/obs_from_bag_<timestamp>.csv)")
    ap.add_argument("--topic", default="/harambe_policy_legs_controller/debug",
                    help="debug topic name in the bag")
    ap.add_argument("--kp", type=float, nargs=NJ, default=DEFAULT_KP,
                    help="per-joint kp for tau reconstruction (match the config)")
    ap.add_argument("--kd", type=float, nargs=NJ, default=DEFAULT_KD)
    ap.add_argument("--eff", type=float, nargs=NJ, default=DEFAULT_EFF)
    args = ap.parse_args()

    # Accept either a bag DIRECTORY (with metadata.yaml) or a loose *.mcap/*.db3
    # file. For a loose file we must name the storage backend explicitly.
    if os.path.isfile(args.bag):
        ext = os.path.splitext(args.bag)[1].lower()
        storage_id = {".mcap": "mcap", ".db3": "sqlite3"}.get(ext, "")
        if not storage_id:
            print(f"[bag_to_csv] unknown bag file type: {args.bag}")
            return 1
    elif os.path.isdir(args.bag):
        storage_id = ""   # auto-detect from metadata.yaml
    else:
        print(f"[bag_to_csv] not a bag dir or file: {args.bag}")
        return 1

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in types:
        print(f"[bag_to_csv] topic '{args.topic}' not in bag.\n  topics: "
              + ", ".join(sorted(types)))
        return 1
    msgtype = get_message(types[args.topic])

    out = args.out
    if not out:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        base_dir = args.bag if os.path.isdir(args.bag) else os.path.dirname(args.bag)
        out = os.path.join(base_dir, f"obs_from_bag_{ts}.csv")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)

    fh = open(out, "w", newline="")
    writer = csv.writer(fh)
    writer.writerow(csv_obs_header())

    tick = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != args.topic:
            continue
        d = deserialize_message(data, msgtype).data
        if len(d) != 4 * NJ + OBS_DIM:
            continue
        targets, positions, velocities = d[0:NJ], d[NJ:2 * NJ], d[2 * NJ:3 * NJ]
        obs = d[4 * NJ:4 * NJ + OBS_DIM]
        tau = []
        for i in range(NJ):
            t = args.kp[i] * (targets[i] - positions[i]) - args.kd[i] * velocities[i]
            tau.append(max(-args.eff[i], min(args.eff[i], t)))
        writer.writerow([tick] + list(obs) + tau)
        tick += 1

    fh.close()
    print(f"[bag_to_csv] wrote {tick} rows -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
