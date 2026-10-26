from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
import threading
import json

from bridge_protocol import now_ms


@dataclass
class DebugEvent:
    id: int
    timestamp_ms: int
    level: str
    category: str
    message: str
    data: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "timestamp_ms": self.timestamp_ms,
            "level": self.level,
            "category": self.category,
            "message": self.message,
            "data": self.data,
        }


class EventSink:
    def emit(self, category: str, message: str, level: str = "info",
             data: Optional[Dict[str, Any]] = None) -> None:
        raise NotImplementedError


class NullEventSink(EventSink):
    def emit(self, category: str, message: str, level: str = "info",
             data: Optional[Dict[str, Any]] = None) -> None:
        return


class RingBufferEventSink(EventSink):
    def __init__(self, capacity: int = 500) -> None:
        self.capacity = capacity
        self._events: List[DebugEvent] = []
        self._next_id = 1
        self._lock = threading.Lock()

    def emit(self, category: str, message: str, level: str = "info",
             data: Optional[Dict[str, Any]] = None) -> None:
        with self._lock:
            event = DebugEvent(
                id=self._next_id,
                timestamp_ms=now_ms(),
                level=level,
                category=category,
                message=message,
                data=data or {},
            )
            self._next_id += 1
            self._events.append(event)
            if len(self._events) > self.capacity:
                self._events = self._events[-self.capacity:]

    def recent(self, limit: int = 100) -> List[Dict[str, Any]]:
        limit = max(0, min(limit, self.capacity))
        with self._lock:
            return [event.to_dict() for event in self._events[-limit:]]


class CompositeEventSink(EventSink):
    def __init__(self, *sinks: EventSink) -> None:
        self._sinks = [sink for sink in sinks if sink is not None]

    def emit(self, category: str, message: str, level: str = "info",
             data: Optional[Dict[str, Any]] = None) -> None:
        for sink in self._sinks:
            sink.emit(category, message, level=level, data=data)


class ConsoleEventSink(EventSink):
    LEVEL_PRIORITY = {
        "debug": 10,
        "info": 20,
        "warning": 30,
        "error": 40,
    }

    def __init__(self, min_level: str = "warning") -> None:
        self.min_level = min_level if min_level in self.LEVEL_PRIORITY else "warning"

    def emit(self, category: str, message: str, level: str = "info",
             data: Optional[Dict[str, Any]] = None) -> None:
        if self.LEVEL_PRIORITY.get(level, 20) < self.LEVEL_PRIORITY[self.min_level]:
            return
        payload = ""
        if data:
            payload = f" data={json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(',', ':'))}"
        print(f"[bridge][{level}][{category}] {message}{payload}", flush=True)
