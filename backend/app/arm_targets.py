"""Read the active YOLO model catalog from the local 18081 service."""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from typing import Any, Dict, Iterable, List, Optional


class ArmTargetCatalog:
    def __init__(self, base_url: str, timeout_sec: float = 3.0) -> None:
        self.base_url = str(base_url or "http://127.0.0.1:18081").rstrip("/")
        self.timeout_sec = max(0.2, float(timeout_sec))

    def _get_json(self, path: str) -> Dict[str, Any]:
        request = urllib.request.Request(
            f"{self.base_url}{path}",
            headers={"Accept": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
            raw = response.read()
        payload = json.loads(raw.decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError(f"{path} returned a non-object JSON payload")
        return payload

    @staticmethod
    def _first_mapping(payload: Dict[str, Any], keys: Iterable[str]) -> Any:
        for key in keys:
            value = payload.get(key)
            if isinstance(value, (dict, list)):
                return value
        return {}

    @staticmethod
    def _find_info(catalog: Any, label: str, class_index: int) -> Dict[str, Any]:
        if isinstance(catalog, dict):
            direct = catalog.get(label)
            if not isinstance(direct, dict):
                direct = catalog.get(str(class_index))
            if isinstance(direct, dict):
                return direct
            values = catalog.values()
        elif isinstance(catalog, list):
            values = catalog
        else:
            return {}

        for item in values:
            if not isinstance(item, dict):
                continue
            candidates = {
                str(item.get("label") or ""),
                str(item.get("id") or ""),
                str(item.get("cigarette_id") or ""),
                str(item.get("class") if item.get("class") is not None else ""),
                str(item.get("class_id") if item.get("class_id") is not None else ""),
            }
            if label in candidates or str(class_index) in candidates:
                return item
        return {}

    @staticmethod
    def _finite_number(value: Any) -> Optional[float]:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if number == number and abs(number) != float("inf") else None

    def read(self) -> Dict[str, Any]:
        started = time.monotonic()
        warnings: List[str] = []
        try:
            model_payload = self._get_json("/config/yolo_model")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            return {
                "ok": False,
                "source_base_url": self.base_url,
                "model": None,
                "targets": [],
                "warnings": [],
                "error": f"cannot read YOLO model catalog: {exc}",
                "elapsed_ms": round((time.monotonic() - started) * 1000.0, 1),
            }

        labels = model_payload.get("yolo_class_names") or []
        if isinstance(labels, dict):
            labels = [
                labels[key]
                for key in sorted(
                    labels,
                    key=lambda item: (0, int(item)) if str(item).isdigit() else (1, str(item)),
                )
            ]
        if not isinstance(labels, list):
            labels = []

        try:
            size_payload = self._get_json("/config/object_sizes")
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            size_payload = {}
            warnings.append(f"cannot read object information: {exc}")

        info_catalog = self._first_mapping(
            size_payload,
            ("object_infos", "cigarette_infos", "object_sizes", "effective_defaults"),
        )
        targets = []
        for class_index, raw_label in enumerate(labels):
            label = str(raw_label or "").strip()
            if not label:
                continue
            info = self._find_info(info_catalog, label, class_index)
            cigarette_id = str(info.get("cigarette_id") or info.get("id") or label).strip()
            name = str(info.get("name") or info.get("display_name") or "").strip()
            display_name = name if name and name == label else (f"{name} ({label})" if name else label)
            targets.append(
                {
                    "class_index": class_index,
                    "label": label,
                    "target_object": label,
                    "cigarette_id": cigarette_id,
                    "name": name,
                    "display_name": display_name,
                    "length_mm": self._finite_number(info.get("length_mm")),
                    "width_mm": self._finite_number(info.get("width_mm")),
                    "height_mm": self._finite_number(info.get("height_mm")),
                }
            )

        return {
            "ok": bool(targets),
            "source_base_url": self.base_url,
            "model": model_payload.get("effective_model") or model_payload.get("model") or model_payload.get("yolo_model"),
            "targets": targets,
            "warnings": warnings,
            "error": "" if targets else "active YOLO model contains no class names",
            "elapsed_ms": round((time.monotonic() - started) * 1000.0, 1),
        }
