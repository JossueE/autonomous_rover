import math

from odom_comparison.metrics import (
    PoseSample,
    comparison_metrics,
    normalize_angle,
    source_metrics,
    yaw_from_quaternion,
)


def test_yaw_from_quaternion_90_deg():
    yaw = math.pi / 2.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    assert math.isclose(yaw_from_quaternion(0.0, 0.0, qz, qw), yaw, abs_tol=1e-9)


def test_normalize_angle_wraps_to_pi_range():
    assert math.isclose(normalize_angle(3.0 * math.pi), math.pi, abs_tol=1e-9)
    assert math.isclose(normalize_angle(-3.0 * math.pi), -math.pi, abs_tol=1e-9)


def test_source_metrics_uses_relative_start_pose():
    samples = [
        PoseSample(t=10.0, x=5.0, y=5.0, yaw=math.pi / 2.0),
        PoseSample(t=11.0, x=5.0, y=7.0, yaw=math.pi / 2.0),
    ]
    metrics = source_metrics('wheel', samples, 2.0, 0.0, 0.0)
    assert math.isclose(metrics['final_x_m'], 2.0, abs_tol=1e-9)
    assert math.isclose(metrics['final_y_m'], 0.0, abs_tol=1e-9)
    assert math.isclose(metrics['error_distance_m'], 0.0, abs_tol=1e-9)


def test_comparison_metrics_pairs_nearest_samples():
    wheel = [
        PoseSample(t=0.00, x=0.0, y=0.0, yaw=0.0),
        PoseSample(t=1.00, x=1.0, y=0.0, yaw=0.0),
    ]
    rtabmap = [
        PoseSample(t=0.02, x=0.0, y=0.0, yaw=0.0),
        PoseSample(t=1.03, x=1.1, y=0.0, yaw=0.1),
    ]
    metrics = comparison_metrics(wheel, rtabmap, max_sync_dt_s=0.05)
    assert metrics['paired_samples'] == 2.0
    assert math.isclose(metrics['final_xy_disagreement_m'], 0.1, abs_tol=1e-9)
    assert math.isclose(metrics['final_yaw_disagreement_deg'], math.degrees(-0.1), abs_tol=1e-9)
