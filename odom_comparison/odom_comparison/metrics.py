from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import mean
from typing import Dict, Iterable, List, Sequence, Tuple


@dataclass(frozen=True)
class PoseSample:
    t: float
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    wz: float = 0.0


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion using the ROS ENU convention."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_samples(samples: Sequence[PoseSample]) -> List[PoseSample]:
    if not samples:
        return []

    origin = samples[0]
    c = math.cos(origin.yaw)
    s = math.sin(origin.yaw)
    out: List[PoseSample] = []
    for sample in samples:
        dx = sample.x - origin.x
        dy = sample.y - origin.y
        rel_x = c * dx + s * dy
        rel_y = -s * dx + c * dy
        out.append(PoseSample(
            t=sample.t - origin.t,
            x=rel_x,
            y=rel_y,
            yaw=normalize_angle(sample.yaw - origin.yaw),
            vx=sample.vx,
            wz=sample.wz,
        ))
    return out


def path_length(samples: Sequence[PoseSample]) -> float:
    if len(samples) < 2:
        return 0.0
    return sum(
        math.hypot(samples[i].x - samples[i - 1].x,
                   samples[i].y - samples[i - 1].y)
        for i in range(1, len(samples))
    )


def frequency_hz(samples: Sequence[PoseSample]) -> float:
    if len(samples) < 2:
        return 0.0
    duration = samples[-1].t - samples[0].t
    if duration <= 0.0:
        return 0.0
    return (len(samples) - 1) / duration


def nearest_pairs(
    left: Sequence[PoseSample],
    right: Sequence[PoseSample],
    max_dt_s: float,
) -> List[Tuple[PoseSample, PoseSample, float]]:
    if not left or not right:
        return []

    pairs: List[Tuple[PoseSample, PoseSample, float]] = []
    j = 0
    for sample in left:
        while j + 1 < len(right) and abs(right[j + 1].t - sample.t) <= abs(right[j].t - sample.t):
            j += 1
        dt = right[j].t - sample.t
        if abs(dt) <= max_dt_s:
            pairs.append((sample, right[j], dt))
    return pairs


def source_metrics(
    name: str,
    samples: Sequence[PoseSample],
    reference_x_m: float,
    reference_y_m: float,
    reference_yaw_deg: float,
) -> Dict[str, float | str]:
    rel = relative_samples(samples)
    ref_yaw = math.radians(reference_yaw_deg)
    if rel:
        final = rel[-1]
        err_x = final.x - reference_x_m
        err_y = final.y - reference_y_m
        err_yaw = normalize_angle(final.yaw - ref_yaw)
    else:
        final = PoseSample(0.0, 0.0, 0.0, 0.0)
        err_x = 0.0
        err_y = 0.0
        err_yaw = 0.0

    return {
        'source': name,
        'sample_count': float(len(samples)),
        'duration_s': rel[-1].t if rel else 0.0,
        'frequency_hz': frequency_hz(samples),
        'final_x_m': final.x,
        'final_y_m': final.y,
        'final_yaw_deg': math.degrees(final.yaw),
        'path_length_m': path_length(rel),
        'error_x_m': err_x,
        'error_y_m': err_y,
        'error_distance_m': math.hypot(err_x, err_y),
        'error_yaw_deg': math.degrees(err_yaw),
        'closure_error_m': math.hypot(final.x, final.y),
    }


def comparison_metrics(
    wheel_samples: Sequence[PoseSample],
    rtabmap_samples: Sequence[PoseSample],
    max_sync_dt_s: float,
) -> Dict[str, float]:
    wheel_rel = relative_samples(wheel_samples)
    rtab_rel = relative_samples(rtabmap_samples)
    pairs = nearest_pairs(wheel_rel, rtab_rel, max_sync_dt_s)
    if not pairs:
        return {
            'paired_samples': 0.0,
            'mean_sync_dt_s': 0.0,
            'mean_xy_disagreement_m': 0.0,
            'max_xy_disagreement_m': 0.0,
            'mean_yaw_disagreement_deg': 0.0,
            'final_xy_disagreement_m': 0.0,
            'final_yaw_disagreement_deg': 0.0,
        }

    xy = [math.hypot(a.x - b.x, a.y - b.y) for a, b, _ in pairs]
    yaw = [abs(normalize_angle(a.yaw - b.yaw)) for a, b, _ in pairs]
    final_wheel, final_rtab, _ = pairs[-1]
    return {
        'paired_samples': float(len(pairs)),
        'mean_sync_dt_s': mean(abs(dt) for _, _, dt in pairs),
        'mean_xy_disagreement_m': mean(xy),
        'max_xy_disagreement_m': max(xy),
        'mean_yaw_disagreement_deg': math.degrees(mean(yaw)),
        'final_xy_disagreement_m': math.hypot(
            final_wheel.x - final_rtab.x,
            final_wheel.y - final_rtab.y,
        ),
        'final_yaw_disagreement_deg': math.degrees(
            normalize_angle(final_wheel.yaw - final_rtab.yaw)
        ),
    }


def rtabmap_lost_percent(info_events: Iterable[Dict[str, object]]) -> float:
    events = list(info_events)
    if not events:
        return 0.0
    lost = sum(1 for event in events if bool(event.get('lost', False)))
    return 100.0 * lost / len(events)


def compute_summary(
    wheel_samples: Sequence[PoseSample],
    rtabmap_samples: Sequence[PoseSample],
    rtabmap_info_events: Iterable[Dict[str, object]],
    reference_x_m: float,
    reference_y_m: float,
    reference_yaw_deg: float,
    max_sync_dt_s: float,
) -> Dict[str, object]:
    wheel = source_metrics(
        'wheel', wheel_samples, reference_x_m, reference_y_m, reference_yaw_deg)
    rtabmap = source_metrics(
        'rtabmap', rtabmap_samples, reference_x_m, reference_y_m, reference_yaw_deg)
    comparison = comparison_metrics(wheel_samples, rtabmap_samples, max_sync_dt_s)
    info_events = list(rtabmap_info_events)
    health = {
        'rtabmap_info_samples': float(len(info_events)),
        'rtabmap_lost_percent': rtabmap_lost_percent(info_events),
    }
    return {
        'sources': [wheel, rtabmap],
        'comparison': comparison,
        'health': health,
    }


def metric_rows(summary: Dict[str, object]) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    for source in summary['sources']:
        assert isinstance(source, dict)
        name = str(source['source'])
        for key, value in source.items():
            if key == 'source':
                continue
            rows.append({
                'group': 'source',
                'source': name,
                'metric': key,
                'value': f'{float(value):.6g}',
            })

    for group_name in ('comparison', 'health'):
        group = summary[group_name]
        assert isinstance(group, dict)
        for key, value in group.items():
            rows.append({
                'group': group_name,
                'source': '',
                'metric': key,
                'value': f'{float(value):.6g}',
            })
    return rows
