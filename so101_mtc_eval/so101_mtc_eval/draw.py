"""The block's and the target's positions for a trial, drawn from the task area as project 1 draws them.

so101-nexus 0.11.0 (project 1's simulator, `MuJoCoPickAndPlace-v2`) draws the disc first, its distance from the
spawn centre and its azimuth uniformly, then the cube the same way until it lies at least
`min_object_target_separation` plus its bounding radius (half the diagonal of its footprint) from the disc's centre,
with 100 attempts and the last one taken after that. The cube's yaw is uniform. The area is so101_mtc_description's
config/task.yaml: a ring sector around the shoulder_pan axis.
"""
from dataclasses import dataclass
import math

import numpy as np

MIN_OBJECT_TARGET_SEPARATION = 0.0375  # m, so101-nexus 0.11.0's default
ATTEMPTS = 100


@dataclass(frozen=True)
class Area:
    """A ring sector around `centre` (x, y): distances `radius` (min, max) and azimuths `azimuth` (min, max), in
    degrees."""

    centre: tuple
    radius: tuple
    azimuth: tuple

    @classmethod
    def from_task(cls, task):
        """The area of a loaded config/task.yaml."""
        area = task["area"]
        return cls(tuple(area["centre"]), tuple(area["radius"]), tuple(area["azimuth"]))

    def point(self, rng):
        r = rng.uniform(*self.radius)
        azimuth = math.radians(rng.uniform(*self.azimuth))
        return self.centre[0] + r * math.cos(azimuth), self.centre[1] + r * math.sin(azimuth)

    def distance(self, x, y):
        """A point's distance from the area's centre (the shoulder_pan axis)."""
        return math.hypot(x - self.centre[0], y - self.centre[1])


@dataclass(frozen=True)
class Trial:
    index: int
    block_x: float
    block_y: float
    block_yaw: float  # rad
    target_x: float
    target_y: float


def draw_trials(area, block_size, count, seed):
    """`count` trials from one seed: the same seed gives the same trials."""
    rng = np.random.default_rng(seed)
    separation = MIN_OBJECT_TARGET_SEPARATION + block_size / 2.0 * math.sqrt(2.0)
    trials = []
    for index in range(count):
        target = area.point(rng)
        for _ in range(ATTEMPTS):
            block = area.point(rng)
            if math.hypot(block[0] - target[0], block[1] - target[1]) >= separation:
                break
        yaw = rng.uniform(-math.pi, math.pi)
        trials.append(Trial(index, block[0], block[1], yaw, target[0], target[1]))
    return trials
