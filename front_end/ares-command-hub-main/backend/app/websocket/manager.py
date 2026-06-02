from __future__ import annotations

import asyncio
from collections.abc import Callable
from typing import Any

from fastapi import WebSocket, WebSocketDisconnect


class WebSocketStreamManager:
    async def stream(
        self,
        websocket: WebSocket,
        payload_factory: Callable[[], dict[str, Any]],
        *,
        interval: float = 0.5,
    ) -> None:
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(payload_factory())
                await asyncio.sleep(interval)
        except WebSocketDisconnect:
            return


websocket_manager = WebSocketStreamManager()
