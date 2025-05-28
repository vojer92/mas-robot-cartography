from abc import ABC, abstractmethod

class CommunicationMethod(ABC):
    @abstractmethod
    def receive(self,
        key: str,
        receiver_id: int | None = None,
        accessors=None,
        **kwargs,
    ):
        """
        Receive information from the communication structure.

        Communication structure must have the following structure:
            Global variables / objects:
                {key:value}
            Variables / objects for specific receivers:
                {key:{receiver_id:value}}
            Deeper nested structures:
                {key:{accessor_1:{accessor_2:{...:{accessor_n:value}}}}}

        :param key: Identifier for the data (e.g. "assigned_goals" or topic)
        :param receiver_id: (Optional) The agent's ID receiving the data.
                            If provided and accessors is None, used as subkey.
        :param accessors: (Optional) List of subkeys for nested structures beginning from key.
                            If given, receiver_id is ignored.
        :return: Value / Object
        """
        pass


    @abstractmethod
    def send(self,
        key: str,
        value,
        receiver_id: int | None = None,
        accessors=None,
        **kwargs
    ) -> bool:
        """
        Send information into the communication structure.

        Communication structure must have the following structure:
            Global variables / objects:
                {key:value}
            Variables / objects for specific receivers:
                {key:{receiver_id:value}}
            Deeper nested structures:
                {key:{accessor_1:{accessor_2:{...:{accessor_n:value}}}}}

        :param key: Identifier for the data (e.g. "assigned_goals" or topic)
        :param value: Value to send.
        :param receiver_id: (Optional) The agent's ID receiving the data.
                            If provided and accessors is None, used as subkey.
        :param accessors: (Optional) List of subkeys for nested structures beginning from key.
                            If given, receiver_id is ignored.
        :return: True if sent successfully.
        """
        pass


    @abstractmethod
    def delete(self,
        key: str,
        receiver_id: int | None = None,
        accessors=None,
        **kwargs
    ) -> bool:
        """
        Delete information from the communication structure.

         Communication structure must have the following structure:
            Global variables / objects:
                {key:value}
            Variables / objects for specific receivers:
                {key:{receiver_id:value}}
            Deeper nested structures:
                {key:{accessor_1:{accessor_2:{...:{accessor_n:value}}}}}

        :param key: Identifier for the data (e.g. "assigned_goals" or topic)
        :param receiver_id: (Optional) The agent's ID receiving the data.
                            If provided and accessors is None, used as subkey.
        :param accessors: (Optional) List of subkeys for nested structures beginning from key.
                            If given, receiver_id is ignored.
        :return: True if a value was found and deleted successfully. False otherwise.
        """
        pass