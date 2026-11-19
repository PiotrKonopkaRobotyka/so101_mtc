from so101_mtc_eval.success import Criteria, Sample, check, conditions

CRITERIA = Criteria(block_size=0.025)
TARGET = (0.25, -0.10)


def resting(time, x=0.25, y=-0.10, z=0.0125, linear=0.0, angular=0.0, tcp=0.20):
    return Sample(time, x, y, z, linear, angular, tcp)


def samples(**changes):
    """1 s at 50 Hz of a block resting on the target, with the arm away; `changes` apply to the last 0.1 s."""
    return [resting(i * 0.02, **(changes if i * 0.02 > 0.9 else {})) for i in range(51)]


def test_a_block_resting_on_the_disc_untouched_and_still_succeeds():
    assert check(samples(), TARGET, CRITERIA) == (True, [])


def test_the_disc_edge_counts_and_beyond_it_fails():
    assert check(samples(x=0.25 + 0.0499), TARGET, CRITERIA)[0]
    assert check(samples(x=0.25 + 0.0501), TARGET, CRITERIA) == (False, ["on_disc"])


def test_a_block_on_an_edge_or_lifted_is_not_carried_by_the_table():
    assert check(samples(z=0.0125 * 1.41), TARGET, CRITERIA) == (False, ["on_table"])
    assert check(samples(z=0.03), TARGET, CRITERIA) == (False, ["on_table"])


def test_a_block_in_the_hand_or_moving_fails():
    assert check(samples(tcp=0.01), TARGET, CRITERIA) == (False, ["untouched"])
    assert check(samples(linear=0.05), TARGET, CRITERIA) == (False, ["still"])
    assert check(samples(angular=1.0), TARGET, CRITERIA) == (False, ["still"])


def test_the_conditions_must_hold_for_the_whole_dwell():
    # still at the end, but it slid 0.1 s ago: within the last 0.2 s
    series = [resting(i * 0.02, linear=(0.05 if 0.85 < i * 0.02 < 0.9 else 0.0)) for i in range(51)]
    success, failed = check(series, TARGET, CRITERIA)
    assert not success and failed == ["still within the last 0.2 s"]
    # it slid 0.3 s before the end: settled long enough
    series = [resting(i * 0.02, linear=(0.05 if 0.6 < i * 0.02 < 0.7 else 0.0)) for i in range(51)]
    assert check(series, TARGET, CRITERIA) == (True, [])


def test_too_few_samples_fail():
    assert check([], TARGET, CRITERIA) == (False, ["no samples"])
    assert not check([resting(0.0), resting(0.1)], TARGET, CRITERIA)[0]


def test_conditions_name_each_check():
    assert set(conditions(resting(0.0), TARGET, CRITERIA)) == {"on_disc", "on_table", "untouched", "still"}
