from collections import defaultdict
from typing import Any, Callable


class PubSubBroker:
    def __init__(self) -> None:
        self.subscribers = defaultdict(list)

    def subscribe(self, topic: str, callback: Callable[[Any], None], agent_id: int):
        self.subscribers[topic].append((callback, agent_id))

    def unsubscribe(self, topic: str, callback: Callable[[Any], None]):
        self.subscribers[topic] = [
            (cb, aid) for cb, aid in self.subscribers[topic] if cb != callback
        ]

    def publish(self, topic: str, data: Any, sender_id: int):
        for callback, agent_id in self.subscribers[topic]:
            if agent_id != sender_id:
                callback(data)
