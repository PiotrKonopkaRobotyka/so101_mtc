import math

from so101_mtc_eval.draw import MIN_OBJECT_TARGET_SEPARATION, Area, draw_trials

# config/task.yaml of so101_mtc_description (project 1's area)
AREA = Area(centre=(0.0388, 0.0), radius=(0.12, 0.34), azimuth=(-60.0, 60.0))
BLOCK = 0.025


def test_the_same_seed_gives_the_same_trials_and_another_seed_others():
    assert draw_trials(AREA, BLOCK, 20, seed=0) == draw_trials(AREA, BLOCK, 20, seed=0)
    assert draw_trials(AREA, BLOCK, 20, seed=0) != draw_trials(AREA, BLOCK, 20, seed=1)


def test_blocks_and_targets_lie_in_the_area_and_apart():
    separation = MIN_OBJECT_TARGET_SEPARATION + BLOCK / 2 * math.sqrt(2)
    for trial in draw_trials(AREA, BLOCK, 500, seed=3):
        for x, y in ((trial.block_x, trial.block_y), (trial.target_x, trial.target_y)):
            assert 0.12 - 1e-9 <= AREA.distance(x, y) <= 0.34 + 1e-9
            assert abs(math.degrees(math.atan2(y - AREA.centre[1], x - AREA.centre[0]))) <= 60 + 1e-9
        assert math.hypot(trial.block_x - trial.target_x, trial.block_y - trial.target_y) >= separation
        assert -math.pi <= trial.block_yaw <= math.pi


def test_area_from_task_yaml_dictionary():
    task = {"area": {"centre": [0.0388, 0.0], "radius": [0.12, 0.34], "azimuth": [-60.0, 60.0]}}
    assert Area.from_task(task) == AREA
