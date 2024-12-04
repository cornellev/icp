import argparse
import os

import matplotlib
import numpy as np
import pandas as pd
from bagpy import bagreader
from matplotlib import pyplot as plt

parser = argparse.ArgumentParser(
    prog="rosbag_extract",
    description="Extract LiDAR data from ROS bags to use in ICP testing",
)
parser.add_argument(
    "-i",
    "--input",
    type=str,
    help="Input ROS bag file",
    required=True,
)
parser.add_argument(
    "-o",
    "--output_dir",
    type=str,
    help="Output directory to save the extracted scans",
    required=True,
)
parser.add_argument(
    "-t",
    "--topic",
    type=str,
    help="Topic name to extract",
    required=True,
)
args = parser.parse_args()

os.makedirs(args.output_dir, exist_ok=True)

b = bagreader(args.input)
LASER_MSG = b.message_by_topic(args.topic)
df = pd.read_csv(LASER_MSG)

current_i = 0
exported = 0


def on_press(event):
    global current_i
    if event.key == "l":
        current_i = min(current_i + 1, len(df) - 1)
    elif event.key == "h":
        current_i = max(current_i - 1, 0)
    elif event.key == "p":
        current_i = min(current_i + 10, len(df) - 1)
    elif event.key == "u":
        current_i = max(current_i - 10, 0)
    elif event.key == "e":
        save(current_i)
    draw(current_i)


def get_ranges(row):
    ranges = []
    i = 0
    while True:
        if f"ranges_{i}" not in row:
            break
        ranges.append(row[f"ranges_{i}"])
        i += 1
    return np.array(ranges)


def draw(index: int):
    row = df.iloc[index]
    ranges = get_ranges(row)

    angle_increment = row["angle_increment"]
    angle_min = row["angle_min"]
    angle_max = row["angle_max"]
    angles = np.arange(angle_min, angle_max + angle_increment, angle_increment)

    x = np.cos(angles) * ranges
    y = np.sin(angles) * ranges

    ax.clear()
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal")
    ax.set_xscale("linear")
    ax.set_yscale("linear")

    lidar_max = df.iloc[0]["range_max"]
    ax.set_xlim(-lidar_max, lidar_max)
    ax.set_ylim(-lidar_max, lidar_max)
    ax.plot(x, y, "o", linestyle="None")


def save(index: int):
    global exported
    row = df.iloc[index]
    angle_increment = row["angle_increment"]
    angle_min = row["angle_min"]
    angle_max = row["angle_max"]
    range_min = row["range_min"]
    range_max = row["range_max"]
    ranges = get_ranges(row)
    length = len(ranges)
    with open(f"{args.output_dir}/scan_{exported}.conf", "w") as f:
        f.write(f"angle_max = {angle_max}\n")
        f.write(f"angle_min = {angle_min}\n")
        f.write(f"angle_increment = {angle_increment}\n")
        f.write(f"length = {length}\n")
        f.write(f"range_max = {range_max}\n")
        f.write(f"range_min = {range_min}\n")
        for i, r in enumerate(ranges):
            f.write(f"{i} = {r}\n")
    print(f"Exported scan {exported}")
    exported += 1


matplotlib.use("qt5agg")
fig, ax = plt.subplots()
plt.ion()
fig.canvas.mpl_connect("key_press_event", on_press)
draw(current_i)
plt.show(block=True)
