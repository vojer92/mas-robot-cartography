from collections import defaultdict
from typing import Any, Callable


# A simple Publish-Subscribe (Pub/Sub) broker class
class PubSubBroker:
    def __init__(self) -> None:
        self.subscribers = defaultdict(list)

    def subscribe(self, topic: str, callback: Callable[[Any], None], agent_id: int):
        """
        Subscribes an agent to a specific topic.

        :param topic: The topic to subscribe to
        :param callback: The function to be called when a message is published
        :param agent_id: The unique ID of the subscribing agent
        """
        self.subscribers[topic].append((callback, agent_id))

    def unsubscribe(self, topic: str, callback: Callable[[Any], None]):
        """
        Unsubscribes a callback from a specific topic.

        :param topic: The topic to unsubscribe from
        :param callback: The callback function to be removed
        """
        self.subscribers[topic] = [
            (cb, aid) for cb, aid in self.subscribers[topic] if cb != callback
        ]

    def publish(self, topic: str, data: Any, sender_id: int):
        """
        Publishes a message to a topic, notifying all subscribers
        except the sender itself.

        :param topic: The topic to publish to
        :param data: The data/message to be sent
        :param sender_id: The ID of the sender (to avoid self-notification)
        """
        for callback, agent_id in self.subscribers[topic]:
            if agent_id != sender_id:
                callback(data)
