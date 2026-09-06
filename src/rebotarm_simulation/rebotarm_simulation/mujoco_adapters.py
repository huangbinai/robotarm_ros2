"""Explicit adapters for consumers that require native MuJoCo objects."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any


class _NativeMujocoAdapter:
    def __init__(self, model: Any, data: Any, ensure_open: Callable[[], None]) -> None:
        self._model = model
        self._data = data
        self._ensure_open = ensure_open

    @property
    def model(self):
        self._ensure_open()
        return self._model

    @property
    def data(self):
        self._ensure_open()
        return self._data

    def handles(self):
        """Return native handles scoped to this explicit backend adapter."""
        return self.model, self.data


class MujocoRenderAdapter(_NativeMujocoAdapter):
    """Native handles for viewer/rendering integrations only."""


class MujocoKinematicsAdapter(_NativeMujocoAdapter):
    """Native handles for MuJoCo FK/Jacobian integrations only."""
