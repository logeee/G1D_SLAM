"""Named workflow (action-chain) library persisted to disk.

Mirrors SavedPointStore: a single JSON file under data/ that holds multiple
named action chains the dashboard can save and reload. Each chain stores the
raw frontend action list verbatim so loading restores it exactly.
"""
from __future__ import annotations

import json
import os
import threading
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional

from ..ros.helpers import now_iso

MAX_NAME_LEN = 80
MAX_ACTIONS = 500


class WorkflowChainStore:
    def __init__(self, path: str) -> None:
        self.path = Path(path)
        self.lock = threading.RLock()
        self.data: Dict[str, Any] = {"version": 1, "chains": []}
        self.load()

    def load(self) -> None:
        with self.lock:
            if not self.path.exists():
                self.data = {"version": 1, "chains": []}
                return
            try:
                loaded = json.loads(self.path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                loaded = None
            chains = loaded.get("chains") if isinstance(loaded, dict) else []
            if not isinstance(chains, list):
                chains = []
            clean: List[Dict[str, Any]] = []
            for item in chains:
                normalized = self._normalize(item)
                if normalized is not None:
                    clean.append(normalized)
            self.data = {"version": 1, "chains": clean}

    def list_payload(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "ok": True,
                "path": str(self.path),
                "count": len(self.data["chains"]),
                "chains": [dict(chain) for chain in self.data["chains"]],
            }

    def save(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if not isinstance(payload, dict):
            return {"ok": False, "error": "payload must be an object"}
        name = str(payload.get("name") or "").strip()
        if not name:
            return {"ok": False, "error": "chain name is required"}
        name = name[:MAX_NAME_LEN]
        actions = payload.get("actions")
        if not isinstance(actions, list):
            return {"ok": False, "error": "actions must be a JSON list"}
        if len(actions) > MAX_ACTIONS:
            return {"ok": False, "error": f"too many actions (> {MAX_ACTIONS})"}
        if any(not isinstance(action, dict) for action in actions):
            return {"ok": False, "error": "each action must be an object"}
        chain_id = str(payload.get("id") or "").strip()
        now = now_iso()
        with self.lock:
            existing = self._find_locked(chain_id) if chain_id else None
            # Re-saving with an existing name overwrites that chain (upsert by name).
            if existing is None:
                existing = self._find_by_name_locked(name)
            if existing is not None:
                existing["name"] = name
                existing["actions"] = actions
                existing["count"] = len(actions)
                existing["updated_at"] = now
                chain = existing
            else:
                chain = {
                    "id": self.new_id(),
                    "name": name,
                    "actions": actions,
                    "count": len(actions),
                    "created_at": now,
                    "updated_at": now,
                }
                self.data["chains"].append(chain)
            self.write_locked()
            return {"ok": True, "chain": dict(chain)}

    def delete(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        chain_id = str((payload or {}).get("id") or "").strip()
        if not chain_id:
            return {"ok": False, "error": "missing chain id"}
        with self.lock:
            before = len(self.data["chains"])
            self.data["chains"] = [c for c in self.data["chains"] if c.get("id") != chain_id]
            if len(self.data["chains"]) == before:
                return {"ok": False, "error": "chain not found", "id": chain_id}
            self.write_locked()
        return {"ok": True, "deleted_id": chain_id}

    def _normalize(self, item: Any) -> Optional[Dict[str, Any]]:
        if not isinstance(item, dict):
            return None
        actions = item.get("actions")
        if not isinstance(actions, list):
            return None
        actions = [a for a in actions if isinstance(a, dict)]
        name = (str(item.get("name") or "").strip() or "动作链")[:MAX_NAME_LEN]
        chain_id = str(item.get("id") or "").strip() or self.new_id()
        now = now_iso()
        return {
            "id": chain_id,
            "name": name,
            "actions": actions,
            "count": len(actions),
            "created_at": item.get("created_at") or now,
            "updated_at": item.get("updated_at") or now,
        }

    def _find_locked(self, chain_id: str) -> Optional[Dict[str, Any]]:
        for chain in self.data["chains"]:
            if chain.get("id") == chain_id:
                return chain
        return None

    def _find_by_name_locked(self, name: str) -> Optional[Dict[str, Any]]:
        for chain in self.data["chains"]:
            if chain.get("name") == name:
                return chain
        return None

    def write_locked(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.path.with_suffix(self.path.suffix + ".tmp")
        tmp_path.write_text(json.dumps(self.data, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(tmp_path, self.path)

    @staticmethod
    def new_id() -> str:
        return "wf_" + uuid.uuid4().hex[:12]
