from __future__ import annotations

import base64
from typing import Any

from app.core.config import (
    IMAGE_MAX_HEIGHT,
    IMAGE_MAX_WIDTH,
    JPEG_QUALITY,
    MAP_IMAGE_MAX_HEIGHT,
    MAP_IMAGE_MAX_WIDTH,
)

_BRIDGE = None


def _mime_from_format(format_value: str) -> str:
    normalized = format_value.lower()
    if "png" in normalized:
        return "image/png"
    if "webp" in normalized:
        return "image/webp"
    return "image/jpeg"


def _get_image_modules():
    import cv2
    import numpy as np
    from cv_bridge import CvBridge

    global _BRIDGE
    if _BRIDGE is None:
        _BRIDGE = CvBridge()
    return cv2, np, _BRIDGE


def _resize_to_fit(image: Any, *, max_width: int, max_height: int) -> Any:
    cv2, _np, _bridge = _get_image_modules()
    height, width = image.shape[:2]
    scale = min(max_width / width, max_height / height, 1.0)
    if scale >= 1.0:
        return image
    size = (max(1, int(width * scale)), max(1, int(height * scale)))
    return cv2.resize(image, size, interpolation=cv2.INTER_AREA)


def image_to_jpeg_payload(msg: Any, *, depth: bool = False, quality: int = JPEG_QUALITY) -> dict[str, Any]:
    cv2, np, bridge = _get_image_modules()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    encoding = str(getattr(msg, "encoding", ""))
    source_width = int(getattr(msg, "width", 0))
    source_height = int(getattr(msg, "height", 0))

    if depth or "16UC1" in encoding or "32FC1" in encoding:
        image = image.astype("float32", copy=False)
        valid = np.isfinite(image)
        if valid.any():
            low = float(np.nanpercentile(image[valid], 2))
            high = float(np.nanpercentile(image[valid], 98))
            if high <= low:
                high = low + 1.0
            image = np.clip((image - low) * 255.0 / (high - low), 0, 255).astype("uint8")
        else:
            image = np.zeros_like(image, dtype="uint8")
    elif encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif encoding == "rgba8":
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    elif encoding == "bgra8":
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    image = _resize_to_fit(image, max_width=IMAGE_MAX_WIDTH, max_height=IMAGE_MAX_HEIGHT)

    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise ValueError("cv2 failed to encode image as JPEG")

    output_height, output_width = image.shape[:2]
    data = base64.b64encode(encoded.tobytes()).decode("ascii")
    return {
        "format": "jpeg",
        "encoding": encoding,
        "width": output_width,
        "height": output_height,
        "source_width": source_width,
        "source_height": source_height,
        "data": f"data:image/jpeg;base64,{data}",
    }


def compressed_image_to_payload(msg: Any) -> dict[str, Any]:
    format_value = str(getattr(msg, "format", "jpeg"))
    mime = _mime_from_format(format_value)
    data = bytes(getattr(msg, "data", b""))
    encoded = base64.b64encode(data).decode("ascii")
    return {
        "format": format_value,
        "encoding": "compressed",
        "width": None,
        "height": None,
        "source_width": None,
        "source_height": None,
        "data": f"data:{mime};base64,{encoded}",
        "bytes": len(data),
    }


def occupancy_grid_to_image_payload(msg: Any, *, quality: int = JPEG_QUALITY) -> dict[str, Any]:
    cv2, np, _bridge = _get_image_modules()
    width = int(msg.info.width)
    height = int(msg.info.height)
    if width <= 0 or height <= 0:
        raise ValueError("occupancy grid has invalid dimensions")

    grid = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
    image = np.full_like(grid, 127, dtype=np.uint8)
    image[grid == 0] = 255
    image[grid > 50] = 0
    image = cv2.flip(image, 0)
    image = _resize_to_fit(image, max_width=MAP_IMAGE_MAX_WIDTH, max_height=MAP_IMAGE_MAX_HEIGHT)

    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise ValueError("cv2 failed to encode occupancy grid")
    output_height, output_width = image.shape[:2]
    data = base64.b64encode(encoded.tobytes()).decode("ascii")
    return {
        "format": "jpeg",
        "width": output_width,
        "height": output_height,
        "source_width": width,
        "source_height": height,
        "data": f"data:image/jpeg;base64,{data}",
    }
