from typing import Any, Callable


class OAABroker:
    def __init__(self):
        self._services: dict[str, list[Callable]] = {}

    def register(self, intent: str, handler: Callable) -> None:
        self._services.setdefault(intent, []).append(handler)

    def dispatch(self, intent: str, *args, **kwargs) -> list[Any]:
        if intent not in self._services:
            raise ValueError(f"Intent '{intent}' not found")

        results = []

        for handler in self._services[intent]:
            result = handler(*args, **kwargs)
            results.append(result)

        return results
