"""Byte-for-byte compatible JSON response.

The legacy stdlib server serialized every payload with:
    json.dumps(obj, ensure_ascii=False, allow_nan=False, separators=(",", ":"))

We must replicate this exactly:
  - ensure_ascii=False       -> keep CJK characters raw (smaller, matches old bytes)
  - allow_nan=False          -> intentionally raise on NaN/Infinity (the Vue frontend's
                                JSON.parse cannot handle them; better to error loudly)
  - separators=(",", ":")    -> compact output, identical to the old wire format

Returning a StrictJSONResponse directly from handlers also bypasses FastAPI's
jsonable_encoder, which matters for the ~1.25 MB /api/state payload.
"""
from __future__ import annotations

import json
from typing import Any

from starlette.responses import Response


class StrictJSONResponse(Response):
    media_type = "application/json; charset=utf-8"

    def render(self, content: Any) -> bytes:
        return json.dumps(
            content,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")

    def init_headers(self, headers=None) -> None:  # type: ignore[override]
        super().init_headers(headers)
        # Legacy write_bytes() always sent Cache-Control: no-store.
        self.raw_headers.append((b"cache-control", b"no-store"))
