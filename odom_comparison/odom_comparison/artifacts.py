from __future__ import annotations

import csv
import contextlib
import io
import math
from pathlib import Path
import struct
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
import zlib


from .metrics import (
    PoseSample,
    compute_summary,
    metric_rows,
    relative_samples,
)


_PYPLOT_ATTEMPTED = False
_PYPLOT_CACHE = None


_FONT = {
    'A': ['01110', '10001', '10001', '11111', '10001', '10001', '10001'],
    'B': ['11110', '10001', '10001', '11110', '10001', '10001', '11110'],
    'C': ['01111', '10000', '10000', '10000', '10000', '10000', '01111'],
    'D': ['11110', '10001', '10001', '10001', '10001', '10001', '11110'],
    'E': ['11111', '10000', '10000', '11110', '10000', '10000', '11111'],
    'F': ['11111', '10000', '10000', '11110', '10000', '10000', '10000'],
    'G': ['01111', '10000', '10000', '10111', '10001', '10001', '01111'],
    'H': ['10001', '10001', '10001', '11111', '10001', '10001', '10001'],
    'I': ['11111', '00100', '00100', '00100', '00100', '00100', '11111'],
    'J': ['00111', '00010', '00010', '00010', '10010', '10010', '01100'],
    'K': ['10001', '10010', '10100', '11000', '10100', '10010', '10001'],
    'L': ['10000', '10000', '10000', '10000', '10000', '10000', '11111'],
    'M': ['10001', '11011', '10101', '10101', '10001', '10001', '10001'],
    'N': ['10001', '11001', '10101', '10011', '10001', '10001', '10001'],
    'O': ['01110', '10001', '10001', '10001', '10001', '10001', '01110'],
    'P': ['11110', '10001', '10001', '11110', '10000', '10000', '10000'],
    'Q': ['01110', '10001', '10001', '10001', '10101', '10010', '01101'],
    'R': ['11110', '10001', '10001', '11110', '10100', '10010', '10001'],
    'S': ['01111', '10000', '10000', '01110', '00001', '00001', '11110'],
    'T': ['11111', '00100', '00100', '00100', '00100', '00100', '00100'],
    'U': ['10001', '10001', '10001', '10001', '10001', '10001', '01110'],
    'V': ['10001', '10001', '10001', '10001', '10001', '01010', '00100'],
    'W': ['10001', '10001', '10001', '10101', '10101', '10101', '01010'],
    'X': ['10001', '10001', '01010', '00100', '01010', '10001', '10001'],
    'Y': ['10001', '10001', '01010', '00100', '00100', '00100', '00100'],
    'Z': ['11111', '00001', '00010', '00100', '01000', '10000', '11111'],
    '0': ['01110', '10001', '10011', '10101', '11001', '10001', '01110'],
    '1': ['00100', '01100', '00100', '00100', '00100', '00100', '01110'],
    '2': ['01110', '10001', '00001', '00010', '00100', '01000', '11111'],
    '3': ['11110', '00001', '00001', '01110', '00001', '00001', '11110'],
    '4': ['00010', '00110', '01010', '10010', '11111', '00010', '00010'],
    '5': ['11111', '10000', '10000', '11110', '00001', '00001', '11110'],
    '6': ['01110', '10000', '10000', '11110', '10001', '10001', '01110'],
    '7': ['11111', '00001', '00010', '00100', '01000', '01000', '01000'],
    '8': ['01110', '10001', '10001', '01110', '10001', '10001', '01110'],
    '9': ['01110', '10001', '10001', '01111', '00001', '00001', '01110'],
    '.': ['00000', '00000', '00000', '00000', '00000', '01100', '01100'],
    '-': ['00000', '00000', '00000', '11111', '00000', '00000', '00000'],
    '_': ['00000', '00000', '00000', '00000', '00000', '00000', '11111'],
    '/': ['00001', '00010', '00010', '00100', '01000', '01000', '10000'],
    ':': ['00000', '01100', '01100', '00000', '01100', '01100', '00000'],
    '%': ['11001', '11010', '00010', '00100', '01000', '01011', '10011'],
    ' ': ['00000', '00000', '00000', '00000', '00000', '00000', '00000'],
}


def _load_pyplot():
    global _PYPLOT_ATTEMPTED, _PYPLOT_CACHE
    if _PYPLOT_ATTEMPTED:
        return _PYPLOT_CACHE
    _PYPLOT_ATTEMPTED = True
    try:
        with contextlib.redirect_stderr(io.StringIO()):
            import matplotlib

            matplotlib.use('Agg')
            import matplotlib.pyplot as plt  # noqa: WPS433
        _PYPLOT_CACHE = plt
        return _PYPLOT_CACHE
    except Exception:
        _PYPLOT_CACHE = None
        return None


def _png_chunk(kind: bytes, data: bytes) -> bytes:
    payload = kind + data
    return struct.pack('>I', len(data)) + payload + struct.pack('>I', zlib.crc32(payload) & 0xFFFFFFFF)


def _write_png(path: Path, width: int, height: int, pixels: List[List[Tuple[int, int, int]]]) -> None:
    raw = b''.join(b'\x00' + b''.join(bytes(pixel) for pixel in row) for row in pixels)
    data = b'\x89PNG\r\n\x1a\n'
    data += _png_chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
    data += _png_chunk(b'IDAT', zlib.compress(raw, level=9))
    data += _png_chunk(b'IEND', b'')
    path.write_bytes(data)


def _canvas(width: int = 900, height: int = 520) -> List[List[Tuple[int, int, int]]]:
    return [[(255, 255, 255) for _ in range(width)] for _ in range(height)]


def _set_pixel(pixels, x: int, y: int, color: Tuple[int, int, int]) -> None:
    if 0 <= y < len(pixels) and 0 <= x < len(pixels[0]):
        pixels[y][x] = color


def _draw_line(pixels, x0: int, y0: int, x1: int, y1: int, color: Tuple[int, int, int]) -> None:
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        _set_pixel(pixels, x0, y0, color)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def _draw_cross(pixels, x: int, y: int, color: Tuple[int, int, int]) -> None:
    for delta in range(-6, 7):
        _set_pixel(pixels, x + delta, y + delta, color)
        _set_pixel(pixels, x + delta, y - delta, color)


def _draw_text(pixels, x: int, y: int, text: str, color: Tuple[int, int, int], scale: int = 3) -> None:
    cursor = x
    for char in text.upper():
        glyph = _FONT.get(char, _FONT[' '])
        for row_index, row in enumerate(glyph):
            for col_index, bit in enumerate(row):
                if bit == '1':
                    for yy in range(scale):
                        for xx in range(scale):
                            _set_pixel(
                                pixels,
                                cursor + col_index * scale + xx,
                                y + row_index * scale + yy,
                                color,
                            )
        cursor += 6 * scale


def _write_text_png(path: Path, lines: Sequence[str]) -> None:
    pixels = _canvas()
    y = 80
    for line in lines:
        _draw_text(pixels, 60, y, line, (20, 20, 20), scale=3)
        y += 46
    _write_png(path, len(pixels[0]), len(pixels), pixels)


def _scale_points(
    points: Sequence[Tuple[float, float]],
    width: int,
    height: int,
    margin: int = 55,
) -> List[Tuple[int, int]]:
    if not points:
        return []
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    if math.isclose(min_x, max_x):
        min_x -= 0.5
        max_x += 0.5
    if math.isclose(min_y, max_y):
        min_y -= 0.5
        max_y += 0.5
    return [
        (
            int(margin + (x - min_x) / (max_x - min_x) * (width - 2 * margin)),
            int(height - margin - (y - min_y) / (max_y - min_y) * (height - 2 * margin)),
        )
        for x, y in points
    ]


def _draw_polyline(pixels, points: Sequence[Tuple[int, int]], color: Tuple[int, int, int]) -> None:
    for index in range(1, len(points)):
        _draw_line(pixels, points[index - 1][0], points[index - 1][1],
                   points[index][0], points[index][1], color)


def ensure_output_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def write_raw_samples(
    output_dir: Path,
    wheel_samples: Sequence[PoseSample],
    rtabmap_samples: Sequence[PoseSample],
) -> None:
    rows: List[Dict[str, object]] = []
    for source, samples in (('wheel', wheel_samples), ('rtabmap', rtabmap_samples)):
        rel = relative_samples(samples)
        for raw, rel_sample in zip(samples, rel):
            rows.append({
                'source': source,
                't': f'{raw.t:.9f}',
                'rel_t': f'{rel_sample.t:.9f}',
                'x': f'{raw.x:.9f}',
                'y': f'{raw.y:.9f}',
                'yaw_rad': f'{raw.yaw:.9f}',
                'yaw_deg': f'{raw.yaw * 57.29577951308232:.6f}',
                'rel_x': f'{rel_sample.x:.9f}',
                'rel_y': f'{rel_sample.y:.9f}',
                'rel_yaw_deg': f'{rel_sample.yaw * 57.29577951308232:.6f}',
                'vx': f'{raw.vx:.9f}',
                'wz': f'{raw.wz:.9f}',
            })

    rows.sort(key=lambda row: (float(row['t']), str(row['source'])))
    with (output_dir / 'raw_samples.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=[
            'source', 't', 'rel_t', 'x', 'y', 'yaw_rad', 'yaw_deg',
            'rel_x', 'rel_y', 'rel_yaw_deg', 'vx', 'wz',
        ])
        writer.writeheader()
        writer.writerows(rows)


def write_rtabmap_info(output_dir: Path, info_events: Iterable[Dict[str, object]]) -> None:
    rows = list(info_events)
    with (output_dir / 'rtabmap_info.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=[
            'topic', 't', 'lost', 'matches', 'inliers', 'features',
            'icp_inliers_ratio', 'local_map_size',
        ])
        writer.writeheader()
        for row in rows:
            writer.writerow({
                'topic': row.get('topic', ''),
                't': f"{float(row.get('t', 0.0)):.9f}",
                'lost': int(bool(row.get('lost', False))),
                'matches': int(row.get('matches', 0)),
                'inliers': int(row.get('inliers', 0)),
                'features': int(row.get('features', 0)),
                'icp_inliers_ratio': f"{float(row.get('icp_inliers_ratio', 0.0)):.6f}",
                'local_map_size': int(row.get('local_map_size', 0)),
            })


def write_summary_csv(output_dir: Path, summary: Dict[str, object]) -> None:
    with (output_dir / 'summary_metrics.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=['group', 'source', 'metric', 'value'])
        writer.writeheader()
        writer.writerows(metric_rows(summary))


def _source(summary: Dict[str, object], name: str) -> Dict[str, float]:
    for row in summary['sources']:
        if row['source'] == name:
            return row
    raise KeyError(name)


def write_metrics_table(output_dir: Path, summary: Dict[str, object]) -> None:
    plt = _load_pyplot()
    if plt is None:
        _write_text_png(output_dir / 'metrics_table.png', [
            'MATPLOTLIB FAILED',
            'SEE SUMMARY_METRICS.CSV',
            'AND README_RESULTS.MD',
        ])
        return

    wheel = _source(summary, 'wheel')
    rtabmap = _source(summary, 'rtabmap')
    health = summary['health']
    table_rows = [
        ['final x (m)', wheel['final_x_m'], rtabmap['final_x_m']],
        ['final y (m)', wheel['final_y_m'], rtabmap['final_y_m']],
        ['final yaw (deg)', wheel['final_yaw_deg'], rtabmap['final_yaw_deg']],
        ['distance error (m)', wheel['error_distance_m'], rtabmap['error_distance_m']],
        ['yaw error (deg)', wheel['error_yaw_deg'], rtabmap['error_yaw_deg']],
        ['path length (m)', wheel['path_length_m'], rtabmap['path_length_m']],
        ['frequency (Hz)', wheel['frequency_hz'], rtabmap['frequency_hz']],
        ['RTAB lost (%)', 0.0, health['rtabmap_lost_percent']],
    ]
    cell_text = [[label, f'{w:.4g}', f'{r:.4g}'] for label, w, r in table_rows]

    fig, ax = plt.subplots(figsize=(9, 3.8))
    ax.axis('off')
    table = ax.table(
        cellText=cell_text,
        colLabels=['Metric', 'Wheel odom', 'RTAB-Map odom'],
        loc='center',
        cellLoc='center',
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.25)
    fig.tight_layout()
    fig.savefig(output_dir / 'metrics_table.png', dpi=180)
    plt.close(fig)


def write_trajectory_plot(
    output_dir: Path,
    wheel_samples: Sequence[PoseSample],
    rtabmap_samples: Sequence[PoseSample],
    reference_x_m: float,
    reference_y_m: float,
) -> None:
    wheel = relative_samples(wheel_samples)
    rtabmap = relative_samples(rtabmap_samples)
    plt = _load_pyplot()
    if plt is None:
        pixels = _canvas()
        width = len(pixels[0])
        height = len(pixels)
        wheel_xy = [(p.x, p.y) for p in wheel]
        rtab_xy = [(p.x, p.y) for p in rtabmap]
        all_points = wheel_xy + rtab_xy + [(0.0, 0.0), (reference_x_m, reference_y_m)]
        scaled_all = _scale_points(all_points, width, height)
        wheel_scaled = scaled_all[:len(wheel_xy)]
        rtab_scaled = scaled_all[len(wheel_xy):len(wheel_xy) + len(rtab_xy)]
        ref_scaled = scaled_all[-1]
        start_scaled = scaled_all[-2]
        _draw_polyline(pixels, wheel_scaled, (30, 95, 180))
        _draw_polyline(pixels, rtab_scaled, (190, 55, 45))
        _draw_cross(pixels, start_scaled[0], start_scaled[1], (30, 30, 30))
        _draw_cross(pixels, ref_scaled[0], ref_scaled[1], (0, 0, 0))
        _draw_text(pixels, 55, 24, 'TRAJECTORY XY FALLBACK', (20, 20, 20), scale=2)
        _write_png(output_dir / 'trajectory_xy.png', width, height, pixels)
        return

    fig, ax = plt.subplots(figsize=(7, 6))
    if wheel:
        ax.plot([p.x for p in wheel], [p.y for p in wheel], label='wheel odom', linewidth=2)
    if rtabmap:
        ax.plot([p.x for p in rtabmap], [p.y for p in rtabmap], label='RTAB-Map odom', linewidth=2)
    ax.scatter([0.0], [0.0], marker='o', s=45, label='start')
    ax.scatter([reference_x_m], [reference_y_m], marker='x', s=70, label='physical reference')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Relative XY trajectory')
    ax.axis('equal')
    ax.grid(True, alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / 'trajectory_xy.png', dpi=180)
    plt.close(fig)


def write_yaw_plot(
    output_dir: Path,
    wheel_samples: Sequence[PoseSample],
    rtabmap_samples: Sequence[PoseSample],
    reference_yaw_deg: float,
) -> None:
    wheel = relative_samples(wheel_samples)
    rtabmap = relative_samples(rtabmap_samples)
    plt = _load_pyplot()
    if plt is None:
        pixels = _canvas()
        width = len(pixels[0])
        height = len(pixels)
        wheel_points = [(p.t, p.yaw * 57.29577951308232) for p in wheel]
        rtab_points = [(p.t, p.yaw * 57.29577951308232) for p in rtabmap]
        if wheel_points or rtab_points:
            max_t = max([0.0] + [p[0] for p in wheel_points + rtab_points])
            ref_points = [(0.0, reference_yaw_deg), (max_t, reference_yaw_deg)]
        else:
            ref_points = [(0.0, reference_yaw_deg), (1.0, reference_yaw_deg)]
        all_points = wheel_points + rtab_points + ref_points
        scaled_all = _scale_points(all_points, width, height)
        wheel_scaled = scaled_all[:len(wheel_points)]
        rtab_scaled = scaled_all[len(wheel_points):len(wheel_points) + len(rtab_points)]
        ref_scaled = scaled_all[-2:]
        _draw_polyline(pixels, wheel_scaled, (30, 95, 180))
        _draw_polyline(pixels, rtab_scaled, (190, 55, 45))
        _draw_polyline(pixels, ref_scaled, (20, 20, 20))
        _draw_text(pixels, 55, 24, 'YAW FALLBACK', (20, 20, 20), scale=2)
        _write_png(output_dir / 'yaw_error.png', width, height, pixels)
        return

    fig, ax = plt.subplots(figsize=(8, 4.5))
    if wheel:
        ax.plot([p.t for p in wheel], [p.yaw * 57.29577951308232 for p in wheel],
                label='wheel odom', linewidth=2)
    if rtabmap:
        ax.plot([p.t for p in rtabmap], [p.yaw * 57.29577951308232 for p in rtabmap],
                label='RTAB-Map odom', linewidth=2)
    ax.axhline(reference_yaw_deg, color='black', linestyle='--', linewidth=1.2,
               label='physical reference')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('relative yaw (deg)')
    ax.set_title('Yaw response')
    ax.grid(True, alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / 'yaw_error.png', dpi=180)
    plt.close(fig)


def write_results_readme(
    output_dir: Path,
    trial_name: str,
    summary: Dict[str, object],
    reference_x_m: float,
    reference_y_m: float,
    reference_yaw_deg: float,
    notes: str,
) -> None:
    wheel = _source(summary, 'wheel')
    rtabmap = _source(summary, 'rtabmap')
    comparison = summary['comparison']
    health = summary['health']
    content = f"""# Odom Comparison Results: {trial_name}

## Reference

- Physical reference x: {reference_x_m:.3f} m
- Physical reference y: {reference_y_m:.3f} m
- Physical reference yaw: {reference_yaw_deg:.2f} deg
- Notes: {notes or 'n/a'}

## Summary

| Source | Distance error (m) | Yaw error (deg) | Path length (m) | Frequency (Hz) |
| --- | ---: | ---: | ---: | ---: |
| Wheel odom | {wheel['error_distance_m']:.4f} | {wheel['error_yaw_deg']:.3f} | {wheel['path_length_m']:.4f} | {wheel['frequency_hz']:.2f} |
| RTAB-Map odom | {rtabmap['error_distance_m']:.4f} | {rtabmap['error_yaw_deg']:.3f} | {rtabmap['path_length_m']:.4f} | {rtabmap['frequency_hz']:.2f} |

## Cross-Odometry Agreement

- Paired samples: {comparison['paired_samples']:.0f}
- Mean XY disagreement: {comparison['mean_xy_disagreement_m']:.4f} m
- Max XY disagreement: {comparison['max_xy_disagreement_m']:.4f} m
- Final XY disagreement: {comparison['final_xy_disagreement_m']:.4f} m
- Final yaw disagreement: {comparison['final_yaw_disagreement_deg']:.3f} deg
- RTAB-Map lost percentage: {health['rtabmap_lost_percent']:.2f} %

## Interpretation Template

Wheel odometry is expected to be repeatable on short straight segments, but it
accumulates heading and closure error when wheel slip or radius mismatch appears.
RTAB-Map odometry can reduce accumulated drift when the RGB-D/ICP registration is
well constrained, but it can degrade under poor lighting, low texture, sparse
geometry, motion blur, or aggressive angular velocity. These results use physical
marks as final-state references, so they support final error and closure claims,
not continuous ground-truth trajectory claims.
"""
    (output_dir / 'README_results.md').write_text(content)


def write_all_artifacts(
    output_dir: Path,
    trial_name: str,
    wheel_samples: Sequence[PoseSample],
    rtabmap_samples: Sequence[PoseSample],
    rtabmap_info_events: Iterable[Dict[str, object]],
    reference_x_m: float,
    reference_y_m: float,
    reference_yaw_deg: float,
    max_sync_dt_s: float,
    notes: str,
) -> Dict[str, object]:
    output_dir = ensure_output_dir(output_dir)
    info_events = list(rtabmap_info_events)
    summary = compute_summary(
        wheel_samples,
        rtabmap_samples,
        info_events,
        reference_x_m,
        reference_y_m,
        reference_yaw_deg,
        max_sync_dt_s,
    )
    write_raw_samples(output_dir, wheel_samples, rtabmap_samples)
    write_rtabmap_info(output_dir, info_events)
    write_summary_csv(output_dir, summary)
    write_metrics_table(output_dir, summary)
    write_trajectory_plot(output_dir, wheel_samples, rtabmap_samples,
                          reference_x_m, reference_y_m)
    write_yaw_plot(output_dir, wheel_samples, rtabmap_samples, reference_yaw_deg)
    write_results_readme(output_dir, trial_name, summary, reference_x_m,
                         reference_y_m, reference_yaw_deg, notes)
    return summary
