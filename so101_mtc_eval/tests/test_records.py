import csv
import math
import os
from pathlib import Path

from so101_mtc_eval.records import (Recorder, Row, failure_stage, grasp_of, misalignment_deg, release_time, summary,
                                    wilson_interval)


def row(trial, success, stage="", tilt=0.0, task_success=True):
    return Row(trial=trial, seed=0, block_x=0.2, block_y=0.0, block_yaw_deg=0.0, block_radius=0.16, target_x=0.2,
               target_y=-0.1, target_radius=0.19, grasp_tilt_deg=tilt, grasp_jaw_angle_deg=0.0,
               grasp_misalignment_deg=0.0, place="", planning_s=1.0, execution_s=12.0, total_s=14.0,
               release_s=10.0,
               task_success=task_success, success=success, stage=stage, failed="", end_x=0.2, end_y=-0.1,
               end_z=0.0125, end_offset=0.003)


def test_failure_stages():
    assert failure_stage(True, "", moved=True, success=True) == ""
    assert failure_stage(False, "Failing stage(s):\napproach (0/4)", moved=False, success=False) == "planning"
    assert failure_stage(False, "execution: MoveIt error code 99999", moved=True, success=False) == "execution"
    assert failure_stage(True, "", moved=False, success=False) == "grasp"
    no_pose = "perception: no block pose younger than 1 s"
    assert failure_stage(False, no_pose, moved=False, success=False) == "perception"
    assert failure_stage(True, "", moved=True, success=False) == "place"


def test_grasp_comment():
    assert grasp_of("tilt 15 deg, jaw angle 180.0 deg") == (15.0, 180.0)
    assert grasp_of("tilt 0 deg, jaw angle -121.4 deg") == (0.0, -121.4)
    assert all(math.isnan(value) for value in grasp_of(""))


def test_misalignment_of_tilted_jaws():
    # block straight ahead of the pan axis: the jaws run along y, so a block with yaw 0 faces them
    assert misalignment_deg(0.3, 0.0, 0.0, 0.0388, 0.0) < 1e-9
    assert abs(misalignment_deg(0.3, 0.0, math.radians(30), 0.0388, 0.0) - 30) < 1e-9
    assert abs(misalignment_deg(0.3, 0.0, math.radians(60), 0.0388, 0.0) - 30) < 1e-9


def test_wilson_interval():
    low, high = wilson_interval(20, 20)
    assert high == 1.0 and abs(low - 0.839) < 1e-3
    low, high = wilson_interval(10, 20)
    assert abs(low - 0.299) < 1e-3 and abs(high - 0.701) < 1e-3
    assert wilson_interval(0, 0) == (0.0, 1.0)


def test_summary_and_the_csv_file():
    path = Path(os.environ["DATA_DIR"]) / "records" / "trials.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    recorder = Recorder(path)
    rows = [row(0, True), row(1, False, "grasp", tilt=15.0), row(2, True, tilt=15.0),
            row(3, False, "planning", tilt=math.nan, task_success=False)]
    for r in rows:
        recorder.add(r)
    recorder.close()
    with open(path) as file:
        written = list(csv.DictReader(file))
    assert len(written) == 4 and written[1]["stage"] == "grasp"
    result = summary(rows)
    assert result["trials"] == 4 and result["successes"] == 2 and result["success_rate"] == 0.5
    assert result["failures_by_stage"] == {"grasp": 1, "planning": 1}
    assert result["successes_by_grasp_tilt_deg"] == {"0": {"successes": 1, "trials": 1},
                                                    "15": {"successes": 1, "trials": 2}}


def test_medians_of_an_even_count_lie_between_the_middle_values():
    rows = [row(0, True), row(1, True)]
    rows[0].planning_s, rows[1].planning_s = 1.0, 2.0
    rows[0].total_s, rows[1].total_s = 10.0, 20.0
    result = summary(rows)
    assert result["median_planning_s"] == 1.5 and result["median_total_s_of_executed"] == 15.0
    assert result["median_release_s_of_successes"] == 10.0


def test_tests_write_under_the_temporary_data_dir():
    assert "tests_" in os.environ["DATA_DIR"]


def test_release_is_the_first_opening_after_the_grip():
    gripper = [(0.0, 1.2), (1.0, 0.07), (5.0, 0.07), (6.0, 0.5), (6.5, 0.8), (9.0, 1.2)]
    assert release_time(gripper, since=0.0) == 6.5
    assert release_time(gripper, since=7.0) is None  # no grip after it
    assert release_time([(0.0, 1.2), (1.0, 1.2)], since=0.0) is None  # never closed
