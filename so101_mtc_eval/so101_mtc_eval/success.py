"""Project 1's success check, from what a ROS observer of the simulation sees.

so101-nexus 0.11.0's `MuJoCoPickAndPlace-v2` (project 1's ADR 0009) counts a success when, for 0.2 s without a break,
the block's footprint centre lies on the target disc (5 cm radius), the table carries at least 90 % of its weight,
the robot and anything else push on it with at most 1 mN, and it is still (at most 0.01 m/s and 0.5 rad/s); the
gripper must have let go. ROS gives the block's pose and velocity but no contact forces, so here:

- on the disc: the block's centre (its footprint's centre when it rests on a face) within `disc_radius` of the target;
- carried by the table: its centre within `height_tolerance` of half its edge above the table, which only a block
  resting on a face reaches (on an edge it stands 41 % higher), with nothing under it;
- untouched: the tcp at least `release_distance` from the block's centre, out of the jaws' reach;
- still: as project 1, from the block's twist.

What this misses: a block pressed down or held sideways by something other than the robot (nothing else is in the
scene), and forces too small to move the block. See ADR 0004.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Criteria:
    block_size: float  # m, the cube's edge
    disc_radius: float = 0.05  # m
    height_tolerance: float = 0.002  # m
    release_distance: float = 0.05  # m, tcp to the block's centre
    max_linear_speed: float = 0.01  # m/s
    max_angular_speed: float = 0.5  # rad/s
    dwell: float = 0.2  # s without a break


@dataclass(frozen=True)
class Sample:
    """The block as observed at `time` (s, simulation time)."""

    time: float
    x: float
    y: float
    z: float
    linear_speed: float
    angular_speed: float
    tcp_distance: float  # m, from the tcp to the block's centre


def conditions(sample, target, criteria):
    """Each condition of the check for one sample; `target` is the disc's centre (x, y)."""
    return {
        "on_disc": math.hypot(sample.x - target[0], sample.y - target[1]) <= criteria.disc_radius,
        "on_table": abs(sample.z - criteria.block_size / 2.0) <= criteria.height_tolerance,
        "untouched": sample.tcp_distance >= criteria.release_distance,
        "still": sample.linear_speed <= criteria.max_linear_speed
        and sample.angular_speed <= criteria.max_angular_speed,
    }


def check(samples, target, criteria):
    """Success at the end of the samples: every condition held over the last `dwell` seconds without a break.

    Returns (success, the conditions that failed in the last sample, or that the samples are too short)."""
    if not samples:
        return False, ["no samples"]
    if samples[-1].time - samples[0].time < criteria.dwell:
        return False, [f"samples cover less than {criteria.dwell} s"]
    last = conditions(samples[-1], target, criteria)
    failed = [name for name, ok in last.items() if not ok]
    if failed:
        return False, failed
    for sample in reversed(samples):
        if samples[-1].time - sample.time > criteria.dwell:
            return True, []
        failed = [name for name, ok in conditions(sample, target, criteria).items() if not ok]
        if failed:
            return False, [f"{name} within the last {criteria.dwell} s" for name in failed]
    return True, []
