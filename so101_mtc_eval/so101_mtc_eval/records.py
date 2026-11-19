"""The records of a run of trials: one CSV row per trial, written as it ends, and the run's summary.

A failure is sorted by the stage that lost the block (ADR 0001's stages, without perception in M2):
- planning: MTC found no solution (IK and planning);
- execution: move_group stopped while executing;
- grasp: the task ran, but the block is still where it stood;
- place: the task ran and moved the block, but it does not pass the success check.
"""
import csv
from dataclasses import asdict, dataclass, fields
import math
import re

MOVED = 0.01  # m: a block farther than this from its start was picked up


@dataclass
class Row:
    trial: int
    seed: int
    block_x: float
    block_y: float
    block_yaw_deg: float
    block_radius: float  # m from the shoulder_pan axis
    target_x: float
    target_y: float
    target_radius: float
    grasp_tilt_deg: float  # nan when no grasp ran
    grasp_jaw_angle_deg: float
    grasp_misalignment_deg: float  # tilted grasps: the jaws off the block's sides, as seen from above
    place: str
    planning_s: float
    execution_s: float
    total_s: float  # from the start of the trial (the block reset) to the end of the task (the arm home)
    task_success: bool  # MoveIt planned and executed
    success: bool  # the simulator's check
    stage: str  # where a failure lost the block, "" on success
    failed: str  # the failed conditions or MTC's failing stages
    end_x: float
    end_y: float
    end_z: float
    end_offset: float  # m from the target's centre


def failure_stage(task_success, failed_stage, moved, success):
    """The stage that lost the block, from the task's response and the block's end pose."""
    if success:
        return ""
    if not task_success:
        return "execution" if failed_stage.startswith("execution") else "planning"
    return "place" if moved else "grasp"


def grasp_of(comment):
    """(tilt, jaw angle) in degrees from the node's comment "tilt 15 deg, jaw angle 180.0 deg", or (nan, nan)."""
    match = re.match(r"tilt (-?[\d.]+) deg, jaw angle (-?[\d.]+) deg", comment or "")
    return (float(match.group(1)), float(match.group(2))) if match else (math.nan, math.nan)


def misalignment_deg(block_x, block_y, yaw, pan_x, pan_y):
    """A tilted grasp's jaws, horizontal and across the arm plane, off the block's sides as seen from above (deg).
    The arm plane points at the block from the pan axis to within a few degrees (ADR 0003's correction)."""
    jaws = math.atan2(block_y - pan_y, block_x - pan_x) + math.pi / 2.0
    return abs(math.degrees(math.remainder(jaws - yaw, math.pi / 2.0)))


class Recorder:
    """Appends rows to a CSV file, flushing each, so that a stopped run keeps its finished trials."""

    def __init__(self, path):
        self.path = path
        self.rows = []
        self._file = open(path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=[field.name for field in fields(Row)])
        self._writer.writeheader()
        self._file.flush()

    def add(self, row):
        self.rows.append(row)
        self._writer.writerow(asdict(row))
        self._file.flush()

    def close(self):
        self._file.close()


def wilson_interval(successes, trials, z=1.96):
    """The 95 % Wilson score interval of a success rate (better than the normal one for small n or rates near 0 or 1)."""
    if trials == 0:
        return 0.0, 1.0
    rate = successes / trials
    centre = (rate + z * z / (2 * trials)) / (1 + z * z / trials)
    half = z * math.sqrt(rate * (1 - rate) / trials + z * z / (4 * trials * trials)) / (1 + z * z / trials)
    return max(0.0, centre - half), min(1.0, centre + half)


def summary(rows):
    """The run's result: the success rate with its interval, failures by stage, the times."""
    n = len(rows)
    successes = sum(row.success for row in rows)
    low, high = wilson_interval(successes, n)
    stages = {}
    for row in rows:
        if row.stage:
            stages[row.stage] = stages.get(row.stage, 0) + 1
    by_tilt = {}
    for row in rows:
        if not math.isnan(row.grasp_tilt_deg):
            key = f"{row.grasp_tilt_deg:.0f}"
            done, total = by_tilt.get(key, (0, 0))
            by_tilt[key] = (done + row.success, total + 1)
    ran = [row for row in rows if row.task_success]

    def median(values):
        values = sorted(values)
        return values[len(values) // 2] if values else math.nan

    return {
        "trials": n,
        "successes": successes,
        "success_rate": successes / n if n else math.nan,
        "interval_95": [low, high],
        "failures_by_stage": stages,
        "successes_by_grasp_tilt_deg": {key: {"successes": done, "trials": total} for key, (done, total) in by_tilt.items()},
        "median_planning_s": median([row.planning_s for row in rows if row.planning_s > 0]),
        "median_total_s_of_executed": median([row.total_s for row in ran]),
    }
