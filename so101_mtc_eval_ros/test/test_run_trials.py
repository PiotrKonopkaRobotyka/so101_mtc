"""The installed command, called as a person calls it, with the default arguments: the dry run draws the trials
without a simulator and writes nothing."""
import os
import shutil
import subprocess
import tempfile


def test_dry_run_with_default_arguments_lists_twenty_trials():
    environment = dict(os.environ, DATA_DIR=tempfile.mkdtemp(prefix="tests_"))
    result = subprocess.run([shutil.which("ros2"), "run", "so101_mtc_eval_ros", "run_trials", "--dry-run"],
                            capture_output=True, text=True, timeout=60, env=environment)
    assert result.returncode == 0, result.stderr
    lines = [line for line in result.stdout.splitlines() if line.startswith("trial ")]
    assert len(lines) == 20
    assert not os.listdir(environment["DATA_DIR"])


def test_run_writes_a_row_per_trial_and_the_summary(tmp_path, monkeypatch):
    """run() with a stand-in for the ROS side: the block lands on the target each time, the hand away."""
    import csv
    import json
    from types import SimpleNamespace

    import numpy as np

    from so101_mtc_eval.success import Sample
    from so101_mtc_eval_ros import run_trials

    class FakeRunner:
        def __init__(self):
            self.position, self.time = None, 0.0

        def move(self, group, joints):
            return True

        def put_block(self, x, y, yaw, half):
            self.position = np.array([x, y, half])

        def block(self):
            self.time += 0.02
            return self.time, self.position.copy(), np.array([0.0, 0.0, 0.0, 1.0])

        def run_task(self, x, y):
            self.position = np.array([x, y, self.position[2]])
            return SimpleNamespace(success=True, failed_stage="", grasp="tilt 15 deg, jaw angle 0.0 deg",
                                   place="tilt 0 deg, jaw angle 180.0 deg", planning_time=1.0, execution_time=12.0)

        def samples(self, since):
            return [Sample(since + 0.02 * i, *self.position, 0.0, 0.0, 0.3) for i in range(51)]

    monkeypatch.setattr(run_trials, "SETTLE", 0.0)
    monkeypatch.setattr(run_trials, "OBSERVE", 0.0)
    result = run_trials.run(3, 0, tmp_path, FakeRunner())
    assert result["trials"] == 3 and result["successes"] == 3
    (rows_file,) = tmp_path.glob("trials_seed0_*.csv")
    rows = list(csv.DictReader(rows_file.open()))
    assert [row["success"] for row in rows] == ["True"] * 3 and rows[0]["grasp_tilt_deg"] == "15.0"
    (summary_file,) = tmp_path.glob("trials_seed0_*.json")
    assert json.loads(summary_file.read_text())["success_rate"] == 1.0
