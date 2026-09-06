from __future__ import annotations

from queue import Empty, SimpleQueue
import threading

from .viewer_state import ViewerControlState, reduce_key


def take_event_snapshot(events: SimpleQueue) -> tuple[object, ...]:
    """Consume only events present at entry so producers cannot starve a frame."""
    snapshot = []
    for _ in range(events.qsize()):
        try:
            snapshot.append(events.get_nowait())
        except Empty:
            break
    return tuple(snapshot)


def start_command_reader(command_stream, command_events: SimpleQueue) -> threading.Thread:
    def read_lines() -> None:
        for line in command_stream:
            command_events.put(str(line).strip())

    thread = threading.Thread(
        target=read_lines,
        name="mujoco-viewer-command-input",
        daemon=True,
    )
    thread.start()
    return thread


def decode_key(keycode: int) -> str:
    if keycode == 256:
        return "\x1b"
    if keycode == 333:  # GLFW_KEY_KP_SUBTRACT
        return "-"
    if keycode == 334:  # GLFW_KEY_KP_ADD
        return "+"
    special = {
        258: "tab",
        295: "f6",
        296: "f7",
        297: "f8",
        298: "f9",
    }
    if keycode in special:
        return special[keycode]
    try:
        return chr(keycode)
    except (TypeError, ValueError):
        return ""


def drain_key_events(events: SimpleQueue, state: ViewerControlState) -> ViewerControlState:
    """Reduce a finite FIFO snapshot, leaving new events for the next cycle."""
    for keycode in take_event_snapshot(events):
        state = reduce_key(state, decode_key(keycode))
    return state
