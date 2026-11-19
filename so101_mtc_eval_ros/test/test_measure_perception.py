"""The installed command, called as a person calls it, with the default arguments: the dry run lists the blocks
without a simulator and writes nothing."""
import os
import shutil
import subprocess


def test_dry_run_with_default_arguments_lists_fifty_blocks(tmp_path):
    environment = dict(os.environ, DATA_DIR=str(tmp_path))
    result = subprocess.run([shutil.which("ros2"), "run", "so101_mtc_eval_ros", "measure_perception", "--dry-run"],
                            capture_output=True, text=True, timeout=60, env=environment)
    assert result.returncode == 0, result.stderr
    assert len([line for line in result.stdout.splitlines() if line.startswith("block ")]) == 50
    assert not any(tmp_path.iterdir())
