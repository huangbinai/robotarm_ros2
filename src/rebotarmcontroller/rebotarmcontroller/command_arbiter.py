from __future__ import annotations

from dataclasses import dataclass
import threading


@dataclass(frozen=True)
class CommandLease:
    resource: str
    owner: str
    generation: int


class CommandArbiter:
    """Own command resources and prevent concurrent writers.

    Arm and gripper are separate resources so a gripper command may run while
    the arm is moving.  Generation numbers make stale callbacks harmless after
    an explicit stop/preemption releases a previous owner.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._owners: dict[str, CommandLease] = {}
        self._generation = 0

    def acquire(self, resource: str, owner: str) -> CommandLease | None:
        with self._lock:
            current = self._owners.get(resource)
            if current is not None and current.owner == owner:
                return current
            if current is not None:
                return None
            self._generation += 1
            lease = CommandLease(resource, owner, self._generation)
            self._owners[resource] = lease
            return lease

    def release(self, lease: CommandLease) -> bool:
        with self._lock:
            if self._owners.get(lease.resource) != lease:
                return False
            del self._owners[lease.resource]
            return True

    def force_release(self, resource: str) -> CommandLease | None:
        with self._lock:
            return self._owners.pop(resource, None)

    def owner(self, resource: str) -> str | None:
        with self._lock:
            lease = self._owners.get(resource)
            return lease.owner if lease is not None else None

    def available(self, resource: str) -> bool:
        return self.owner(resource) is None

    def is_current(self, lease: CommandLease) -> bool:
        with self._lock:
            return self._owners.get(lease.resource) == lease
