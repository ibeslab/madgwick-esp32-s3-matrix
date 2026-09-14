import math
import numpy as np

from rpy_viewer import parse_rpy_line, rotation_matrix_rpy


def test_parse_rpy_line_accepts_three_csv_degrees():
    assert parse_rpy_line("12.5,-3.0,90.0") == (12.5, -3.0, 90.0)


def test_parse_rpy_line_rejects_non_data_lines():
    assert parse_rpy_line("QMI8658 OK") is None
    assert parse_rpy_line("1,2") is None


def test_rotation_matrix_identity_at_zero_rpy():
    np.testing.assert_allclose(rotation_matrix_rpy(0.0, 0.0, 0.0), np.eye(3), atol=1e-7)


def test_rotation_matrix_yaw_90_rotates_body_x_to_world_y():
    R = rotation_matrix_rpy(0.0, 0.0, 90.0)
    rotated_x = R @ np.array([1.0, 0.0, 0.0])
    np.testing.assert_allclose(rotated_x, np.array([0.0, 1.0, 0.0]), atol=1e-7)


def test_rotation_matrix_roll_90_rotates_body_y_to_world_z():
    R = rotation_matrix_rpy(90.0, 0.0, 0.0)
    rotated_y = R @ np.array([0.0, 1.0, 0.0])
    np.testing.assert_allclose(rotated_y, np.array([0.0, 0.0, 1.0]), atol=1e-7)
