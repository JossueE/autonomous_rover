from __future__ import annotations

import math
from typing import Any


def _point_xyz(point: Any) -> tuple[float, float, float] | None:
    names = getattr(getattr(point, "dtype", None), "names", None)
    if names and {"x", "y", "z"}.issubset(names):
        x = float(point["x"])
        y = float(point["y"])
        z = float(point["z"])
    elif isinstance(point, dict):
        x = float(point["x"])
        y = float(point["y"])
        z = float(point["z"])
    else:
        x, y, z = point[:3]

    if not all(math.isfinite(value) for value in (x, y, z)):
        return None
    return x, y, z


def pointcloud_to_points(msg: Any, *, max_points: int = 5000) -> dict[str, Any]:
    from sensor_msgs_py import point_cloud2

    total = int(getattr(msg, "width", 0)) * int(getattr(msg, "height", 0))
    step = max(1, total // max_points) if total > max_points else 1
    points: list[list[float]] = []
    skipped = 0

    cloud = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    if hasattr(cloud, "__len__") and total <= 0:
        total = len(cloud)
        step = max(1, total // max_points) if total > max_points else 1

    for index, point in enumerate(cloud):
        if index % step != 0:
            continue
        xyz = _point_xyz(point)
        if xyz is None:
            skipped += 1
            continue
        x, y, z = xyz
        points.append([float(x), float(y), float(z)])
        if len(points) >= max_points:
            break

    return {
        "frame_id": str(getattr(msg.header, "frame_id", "")),
        "total_points": total,
        "sent_points": len(points),
        "skipped_points": skipped,
        "points": points,
    }
