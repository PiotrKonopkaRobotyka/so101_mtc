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
