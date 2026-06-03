from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List, TypedDict

import yaml

from config.settings import MODELS_PATH


class ModelSpec(TypedDict, total=False):
    name: str
    url: str


def _ament_share() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("rover_bt"))
    except Exception:
        return None


def _source_package_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _inside_install(path: Path) -> bool:
    return "install" in path.parts


def _candidate_catalogs() -> list[Path]:
    configured = Path(MODELS_PATH)
    candidates: list[Path] = []
    if configured.is_absolute():
        candidates.append(configured)

    root = _source_package_root()
    candidates.extend([
        root / configured,
        root / "config" / "models.yml",
        Path.cwd() / configured,
        Path.cwd() / "rover_bt" / configured,
        Path.cwd() / "rover_bt" / "config" / "models.yml",
    ])

    share = _ament_share()
    if share is not None:
        candidates.append(share / configured)
        candidates.append(share / "config" / "models.yml")
    return candidates


def load_yaml() -> Dict[str, Any]:
    for path in _candidate_catalogs():
        if path.exists():
            with path.open("r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
    checked = "\n".join(str(p) for p in _candidate_catalogs())
    raise FileNotFoundError(f"No se encontro models.yml. Rutas revisadas:\n{checked}")


def _candidate_asset_roots() -> list[Path]:
    roots: list[Path] = []
    env_assets = os.environ.get("ROVER_BT_VOICE_ASSETS")
    if env_assets:
        roots.append(Path(env_assets))

    root = _source_package_root()
    if not _inside_install(root):
        roots.append(root / "voice_assets")
    roots.extend([
        Path.cwd() / "voice_assets",
        Path.cwd() / "rover_bt" / "voice_assets",
    ])

    share = _ament_share()
    if share is not None:
        workspace = _workspace_root_from_share(share)
        if workspace is not None:
            roots.extend([
                workspace / "rover_bt" / "voice_assets",
                workspace / "src" / "rover_bt" / "voice_assets",
                workspace / "src" / "autonomous_rover" / "rover_bt" / "voice_assets",
            ])
    return roots


def _workspace_root_from_share(share: Path) -> Path | None:
    parts = share.parts
    if "install" not in parts:
        return None
    idx = parts.index("install")
    if idx == 0:
        return None
    return Path(*parts[:idx])


class LoadModel:
    def __init__(self):
        self.data = load_yaml()

    def extract_section_models(self, section: str) -> List[ModelSpec]:
        items = self.data.get(section, [])
        if not isinstance(items, list):
            raise ValueError(f"La seccion '{section}' no es una lista")

        out: List[ModelSpec] = []
        for item in items:
            if not isinstance(item, dict):
                continue
            out.append({
                "name": item.get("name", ""),
                "url": item.get("url", ""),
            })
        return out

    def ensure_model(self, section: str) -> List[Path]:
        models = []
        for spec in self.extract_section_models(section):
            name = spec.get("name", "")
            if not name:
                continue

            for root in _candidate_asset_roots():
                candidate = root / section / name
                if candidate.exists():
                    models.append(candidate)
                    break
            else:
                checked = "\n".join(
                    str(root / section / name) for root in _candidate_asset_roots()
                )
                raise FileNotFoundError(
                    f"[LLM_LOADER] Modelo '{section}/{name}' no existe.\n"
                    f"Rutas revisadas:\n{checked}"
                )
        return models


if __name__ == "__main__":
    loader = LoadModel()
    print(loader.ensure_model("stt")[0])
