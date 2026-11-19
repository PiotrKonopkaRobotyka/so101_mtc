"""Pick and place trials in MuJoCo with project 1's success check (ROB-53; ADR 0004 is the protocol).

Needs the simulation (so101_mtc_bringup's sim.launch.py) and the task node (so101_mtc_task's pick_place.launch.py with
run_at_start:=false). The trials come from config/task.yaml's area and the seed (so101_mtc_eval.draw). Per trial:

1. reset: the gripper opens and the arm goes home (move_group), then the block is put down where the trial says
   (set_free_joint_state) and settles;
2. the task node picks the block and places it on the target (/pick_place/pick_place);
3. one second of the block's pose, with the tcp's position from TF, decides the success (so101_mtc_eval.success).

Each trial appends a row to $DATA_DIR/so101_mtc/m2/<run>.csv as it ends; <run>.json gets the summary.

    ros2 run so101_mtc_eval_ros run_trials [--trials 20] [--seed 0] [--dry-run]
"""
import argparse
import json
import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from so101_mtc_eval.draw import Area, draw_trials
from so101_mtc_eval.records import (MOVED, Recorder, Row, failure_stage, grasp_of, misalignment_deg, summary)
from so101_mtc_eval.success import Criteria, Sample, check

OBSERVE = 1.0  # s of the block's pose after the task
SETTLE = 1.0  # s after the block is put down
TASK_TIMEOUT = 180.0  # s for one pick and place, planning and execution (ADR 0004)


def load_task():
    path = Path(get_package_share_directory("so101_mtc_description")) / "config" / "task.yaml"
    return yaml.safe_load(path.read_text())


def named_state(group, name):
    """A named state's joint values from so101_mtc_moveit_config's SRDF."""
    srdf = Path(get_package_share_directory("so101_mtc_moveit_config")) / "config" / "so101_mtc.srdf"
    for state in ET.parse(srdf).getroot().findall("group_state"):
        if state.get("group") == group and state.get("name") == name:
            return {joint.get("name"): float(joint.get("value")) for joint in state.findall("joint")}
    raise KeyError(f"no state {name} of {group} in {srdf}")


def quaternion_angle(q1, q2):
    """The rotation between two unit quaternions (x, y, z, w), rad."""
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(q1, q2)))))


def wait(future, timeout):
    """The result of an rclpy future that the executor's thread completes."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        raise TimeoutError("no answer")
    return future.result()


class Runner:
    """The ROS side: clients, the block's pose and the tcp's position; one call per step of a trial."""

    def __init__(self):
        import rclpy
        from mujoco_ros2_control_msgs.msg import FreeJointStateArray
        from mujoco_ros2_control_msgs.srv import SetFreeJointState
        from moveit_msgs.action import MoveGroup
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from so101_mtc_interfaces.srv import PickPlace
        from tf2_ros import Buffer, TransformListener

        self.node = rclpy.create_node("run_trials")
        self.blocks = deque(maxlen=500)  # (time, position, quaternion) of the block, 10 s at 50 Hz
        self.node.create_subscription(FreeJointStateArray, "/block/free_joint_states", self._on_block, 10)
        self.set_block = self.node.create_client(SetFreeJointState, "/mujoco_ros2_control_node/set_free_joint_state")
        self.pick_place = self.node.create_client(PickPlace, "/pick_place/pick_place")
        self.move_group = ActionClient(self.node, MoveGroup, "/move_action")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self.node)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        for client, name in ((self.set_block, "set_free_joint_state"), (self.pick_place, "pick_place")):
            if not client.wait_for_service(timeout_sec=60.0):
                raise RuntimeError(f"{name} is not available: start the simulation and the task node")
        if not self.move_group.wait_for_server(timeout_sec=60.0):
            raise RuntimeError("move_group's /move_action is not available")

    def _on_block(self, message):
        for body in message.free_joints:
            if body.name == "block":
                p, q = body.pose.pose.position, body.pose.pose.orientation
                stamp = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec
                self.blocks.append((stamp, np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])))

    def block(self):
        return self.blocks[-1] if self.blocks else None

    def tcp(self):
        from rclpy.time import Time
        transform = self.tf.lookup_transform("world", "tcp", Time())  # the latest
        t = transform.transform.translation
        return np.array([t.x, t.y, t.z])

    def move(self, group, joints):
        """Plan and execute a joint goal through move_group; True on success."""
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, JointConstraint
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = group
        request.num_planning_attempts, request.allowed_planning_time = 3, 5.0
        request.max_velocity_scaling_factor = request.max_acceleration_scaling_factor = 0.5
        request.start_state.is_diff = True
        request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=value, tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0)
            for name, value in joints.items()])]
        handle = wait(self.move_group.send_goal_async(goal), 10.0)
        result = wait(handle.get_result_async(), 60.0)
        return result.result.error_code.val == 1

    def put_block(self, x, y, yaw, half):
        from mujoco_ros2_control_msgs.msg import FreeJointState
        from mujoco_ros2_control_msgs.srv import SetFreeJointState
        request = SetFreeJointState.Request()
        body = FreeJointState(name="block")
        body.pose.pose.position.x, body.pose.pose.position.y, body.pose.pose.position.z = x, y, half + 0.001
        body.pose.pose.orientation.z, body.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        request.free_joints = [body]
        wait(self.set_block.call_async(request), 10.0)

    def run_task(self, x, y):
        from geometry_msgs.msg import Point
        from so101_mtc_interfaces.srv import PickPlace
        return wait(self.pick_place.call_async(PickPlace.Request(target=Point(x=x, y=y, z=0.0))), TASK_TIMEOUT)

    def samples(self, since):
        """The block's poses since `since` (simulation time) as samples, with speeds from consecutive poses."""
        tcp = self.tcp()
        poses = [entry for entry in list(self.blocks) if entry[0] >= since]
        samples = []
        for previous, current in zip(poses, poses[1:]):
            dt = current[0] - previous[0]
            if dt <= 0:
                continue
            linear = float(np.linalg.norm(current[1] - previous[1])) / dt
            angular = quaternion_angle(previous[2], current[2]) / dt
            samples.append(Sample(current[0], *current[1], linear, angular, float(np.linalg.norm(tcp - current[1]))))
        return samples

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()


def run(trials, seed, out_dir, runner):
    task = load_task()
    area = Area.from_task(task)
    size = task["block"]["size"]
    criteria = Criteria(block_size=size, disc_radius=task["target"]["radius"])
    open_gripper, home = named_state("gripper", "open"), named_state("arm", "home")
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"trials_seed{seed}_{datetime.now():%Y%m%d-%H%M%S}"
    recorder = Recorder(out_dir / f"{stem}.csv")
    print(f"{trials} trials, seed {seed}: {out_dir / stem}.csv", flush=True)
    for trial in draw_trials(area, size, trials, seed):
        # 1. reset: release whatever the hand holds, go home, put the block down
        reset = runner.move("gripper", open_gripper) and runner.move("arm", home)
        runner.put_block(trial.block_x, trial.block_y, trial.block_yaw, size / 2)
        time.sleep(SETTLE)
        start = runner.block()
        # 2. the task
        started = time.monotonic()
        try:
            response = runner.run_task(trial.target_x, trial.target_y)
            task_success, failed_stage = response.success, response.failed_stage.strip()
        except TimeoutError:
            response, task_success, failed_stage = None, False, f"no answer within {TASK_TIMEOUT} s"
        total = time.monotonic() - started
        # 3. the check, over the block's poses after the task
        ended = runner.block()[0]
        time.sleep(OBSERVE)
        success, failed = check(runner.samples(ended), (trial.target_x, trial.target_y), criteria)
        end = runner.block()[1]
        moved = float(np.linalg.norm(end[:2] - start[1][:2])) > MOVED
        tilt, jaw = grasp_of(response.grasp if response else "")
        row = Row(
            trial=trial.index, seed=seed, block_x=trial.block_x, block_y=trial.block_y,
            block_yaw_deg=math.degrees(trial.block_yaw), block_radius=area.distance(trial.block_x, trial.block_y),
            target_x=trial.target_x, target_y=trial.target_y,
            target_radius=area.distance(trial.target_x, trial.target_y), grasp_tilt_deg=tilt,
            grasp_jaw_angle_deg=jaw,
            grasp_misalignment_deg=(misalignment_deg(trial.block_x, trial.block_y, trial.block_yaw, *area.centre)
                                    if tilt > 0 else (0.0 if tilt == 0 else math.nan)),
            place=response.place if response else "", planning_s=response.planning_time if response else math.nan,
            execution_s=response.execution_time if response else math.nan, total_s=total,
            task_success=task_success, success=success,
            stage=failure_stage(task_success, failed_stage, moved, success),
            failed=("" if reset else "reset failed; ")
            + (failed_stage if not task_success else "; ".join(failed)).replace("\n", " | "),
            end_x=float(end[0]), end_y=float(end[1]), end_z=float(end[2]),
            end_offset=math.hypot(end[0] - trial.target_x, end[1] - trial.target_y))
        recorder.add(row)
        print(f"trial {trial.index}: {'success' if success else 'FAIL ' + row.stage} r {row.block_radius:.3f} "
              f"tilt {tilt} offset {row.end_offset * 1000:.1f} mm, task {total:.1f} s {row.failed[:120]}", flush=True)
    recorder.close()
    result = summary(recorder.rows)
    (out_dir / f"{stem}.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true", help="print the trials only, without ROS")
    args, ros_args = parser.parse_known_args(argv)
    out_dir = Path(os.environ.get("DATA_DIR", "~/data")).expanduser() / "so101_mtc" / "m2"
    if args.dry_run:
        task = load_task()
        area = Area.from_task(task)
        for trial in draw_trials(area, task["block"]["size"], args.trials, args.seed):
            print(f"trial {trial.index}: block ({trial.block_x:.3f}, {trial.block_y:.3f}) "
                  f"r {area.distance(trial.block_x, trial.block_y):.3f} yaw {math.degrees(trial.block_yaw):.0f} deg, "
                  f"target ({trial.target_x:.3f}, {trial.target_y:.3f})")
        return
    import rclpy
    rclpy.init(args=ros_args)
    runner = Runner()
    try:
        run(args.trials, args.seed, out_dir, runner)
    finally:
        runner.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
