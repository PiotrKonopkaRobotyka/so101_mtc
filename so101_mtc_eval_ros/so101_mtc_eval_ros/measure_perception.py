"""The camera's block pose against the true one over the block area (M3's exit, ADR 0001 criterion 3).

Needs the simulation (so101_mtc_bringup's sim.launch.py) and the detector (so101_mtc_perception_ros's block_detector,
or so101_mtc_task's pick_place.launch.py with camera:=true). For each block drawn from config/task.yaml's area with
the seed (so101_mtc_eval.draw, the blocks of the trials), the arm waits home, out of the camera's view of the area;
the block is put down and settles, and the first /block_pose stamped after that is compared with the simulator's pose.

Rows go to $DATA_DIR/so101_mtc/m3/<run>.csv, the summary to <run>.json.

    ros2 run so101_mtc_eval_ros measure_perception [--blocks 50] [--seed 0] [--dry-run]
"""
import argparse
import csv
import json
import math
import os
import threading
import time
from datetime import datetime
from pathlib import Path

from so101_mtc_eval.draw import Area, draw_trials
from so101_mtc_eval.perception import errors, summary, yaw_of
from so101_mtc_eval_ros.run_trials import load_task, named_state

SETTLE = 1.0  # s after the block is put down
WAIT = 5.0  # s for a pose stamped after the settling


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--blocks", type=int, default=50)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true", help="print the blocks only, without ROS")
    args, ros_args = parser.parse_known_args(argv)
    task = load_task()
    area = Area.from_task(task)
    edge = task["block"]["size"]
    blocks = draw_trials(area, edge, args.blocks, args.seed)
    if args.dry_run:
        for block in blocks:
            print(f"block {block.index}: ({block.block_x:.3f}, {block.block_y:.3f}) "
                  f"yaw {math.degrees(block.block_yaw):.0f} deg")
        return

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from so101_mtc_eval_ros.run_trials import Runner

    rclpy.init(args=ros_args)
    runner = Runner(need_task=False)
    latest = {}
    lock = threading.Lock()

    def on_pose(message):
        stamp = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec
        q = message.pose.orientation
        with lock:
            latest["pose"] = (stamp, (message.pose.position.x, message.pose.position.y), (q.x, q.y, q.z, q.w))

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    runner.node.create_subscription(PoseStamped, "/block_pose", on_pose, latched)
    out_dir = Path(os.environ.get("DATA_DIR", "~/data")).expanduser() / "so101_mtc" / "m3"
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"perception_seed{args.seed}_{datetime.now():%Y%m%d-%H%M%S}"
    rows = []
    try:
        runner.move("arm", named_state("arm", "home"))
        with open(out_dir / f"{stem}.csv", "w", newline="") as file:
            fields = ["block", "true_x", "true_y", "true_yaw_deg", "found", "found_x", "found_y", "found_yaw_deg",
                      "error_mm", "yaw_error_deg", "radius"]
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writeheader()
            for block in blocks:
                runner.put_block(block.block_x, block.block_y, block.block_yaw, edge / 2)
                time.sleep(SETTLE)
                settled = runner.block()[0]
                deadline = time.monotonic() + WAIT
                pose = None
                while time.monotonic() < deadline:
                    with lock:
                        candidate = latest.get("pose")
                    if candidate and candidate[0] > settled:
                        pose = candidate
                        break
                    time.sleep(0.05)
                true_time, true_position, true_quaternion = runner.block()
                true_yaw = yaw_of(true_quaternion)
                row = {"block": block.index, "true_x": float(true_position[0]), "true_y": float(true_position[1]),
                       "true_yaw_deg": math.degrees(true_yaw), "found": pose is not None,
                       "radius": area.distance(*true_position[:2])}
                if pose:
                    distance, yaw_error = errors(true_position[:2], true_yaw, pose[1], yaw_of(pose[2]))
                    row.update({"found_x": pose[1][0], "found_y": pose[1][1],
                                "found_yaw_deg": math.degrees(yaw_of(pose[2])), "error_mm": distance * 1000,
                                "yaw_error_deg": math.degrees(yaw_error)})
                rows.append(row)
                writer.writerow(row)
                file.flush()
                print(f"block {block.index}: " + (f"{row['error_mm']:.2f} mm, {row['yaw_error_deg']:.2f} deg"
                                                   if pose else "NOT FOUND"), flush=True)
        result = summary(rows)
        (out_dir / f"{stem}.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
    finally:
        runner.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
