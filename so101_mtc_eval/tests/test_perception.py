import math

from so101_mtc_eval.perception import errors, summary, yaw_of


def test_yaw_of_a_turn_about_the_vertical():
    angle = math.radians(30)
    assert abs(yaw_of((0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))) - angle) < 1e-12


def test_errors_count_the_yaw_modulo_a_quarter_turn():
    distance, yaw = errors((0.2, 0.0), math.radians(10), (0.203, 0.004), math.radians(10 + 90 + 2))
    assert abs(distance - 0.005) < 1e-12
    assert abs(math.degrees(yaw) - 2) < 1e-9


def test_summary_of_found_and_missing_blocks():
    rows = [{"found": True, "error_mm": e, "yaw_error_deg": 0.1 * e} for e in (1.0, 2.0, 3.0)]
    rows.append({"found": False})
    result = summary(rows)
    assert result["blocks"] == 4 and result["found"] == 3
    assert result["error_mm"]["median"] == 2.0 and result["error_mm"]["max"] == 3.0
    assert summary([{"found": False}]) == {"blocks": 1, "found": 0}
