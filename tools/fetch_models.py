"""Fetch the third-party model folders pinned in so101_mtc.repos into the workspace's src/ (ADR 0002).

A sparse, shallow fetch of the pinned commit downloads only the folders this project uses (~55 MB on disk)
instead of whole repositories (~0.8 GB). The layout is the one `vcs import src < so101_mtc.repos` gives.
Run again to move an existing fetch to the pinned commit.

Usage, from the workspace: python3 src/so101_mtc/tools/fetch_models.py
"""
import subprocess
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
SRC = REPO.parent
# the folders each pinned repository contributes
FOLDERS = {"SO-ARM100": ["Simulation/SO101"], "mujoco_menagerie": ["robotstudio_so101"]}


def git(target, *args):
    subprocess.run(["git", "-C", str(target), *args], check=True)


def main():
    repositories = yaml.safe_load((REPO / "so101_mtc.repos").read_text())["repositories"]
    for name, folders in FOLDERS.items():
        url, version = repositories[name]["url"], repositories[name]["version"]
        target = SRC / name
        if not (target / ".git").exists():
            subprocess.run(["git", "init", "--quiet", str(target)], check=True)
            git(target, "remote", "add", "origin", url)
        git(target, "sparse-checkout", "set", *folders)
        git(target, "fetch", "--quiet", "--depth", "1", "--filter=blob:none", "origin", version)
        git(target, "checkout", "--quiet", "--detach", version)
        print(f"{name} @ {version[:8]}: {', '.join(folders)} in {target}")


if __name__ == "__main__":
    main()
