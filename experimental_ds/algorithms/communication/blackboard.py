from algorithms.communication.communication_method import CommunicationMethod

class Blackboard(CommunicationMethod):
    """
    Provide communication via a flexible global blackboard structure.

    Blackboard must have the following structure:
            Global variables / objects:
                {key:value}
            Variables / objects for specific receivers:
                {key:{receiver_id:value}}
            Deeper nested structures:
                {key:{accessor_1:{accessor_2:{...:{accessor_n:value}}}}}
    """

    def __init__(self,
        blackboard,
        *args, **kwargs
    ) -> None:
        self.blackboard = blackboard
        self.log_missing = True


    def _build_accessors(self,
        receiver_id: int | None,
        accessors,
    ):
        """Build accessors if not provided and receiver_id is provided."""
        if accessors is None:
            accessors = []
            if receiver_id is not None:
                accessors.append(receiver_id)
            if not accessors:
                accessors = None # For global values
        return accessors


    def receive(self,
        key: str,
        receiver_id: int | None = None,
        accessors = None,
        **kwargs,
    ):
        """
        Receive data from the blackboard structure.

        :param key: Identifier for the data (e.g. "assigned_goals" or topic)
        :param receiver_id: (Optional) The agent's ID receiving the data.
                            If provided and accessors is None, used as subkey.
        :param accessors: (Optional) List of subkeys for nested structures beginning from key.
                            If given, receiver_id is ignored.
        :return: Value / Object
        """
        # Check key
        current = self.blackboard.get(key, None)
        if current is None:
            if self.log_missing:
                print(f"Key '{key}' not found in blackboard.") # Normal case, that key not exists, e.g. initial communication

        # Build accessors
        accessors = self._build_accessors(receiver_id, accessors)

        # Get values layer by layer
        if accessors:
            for accessor in accessors:
                if isinstance(current, dict) and accessor in current:
                    current = current[accessor]
                else:
                    if self.log_missing:
                        print(f" Accessor '{accessor}' not found for key '{key}' in blackboard.")
                    return None
        # Else return global value / object
        return current



    def send(self,
         key: str,
         value,
         receiver_id: int | None = None,
         accessors=None,
         **kwargs,
     ) -> bool:
        """
        Send information to the blackboard.

        :param key: Identifier for the data (e.g. "assigned_goals" or topic)
        :param value: Value to send.
        :param receiver_id: (Optional) The agent's ID receiving the data.
                            If provided and accessors is None, used as subkey.
        :param accessors: (Optional) List of subkeys for nested structures beginning from key.
                            If given, receiver_id is ignored.
        :return: True if sent successfully.
        """
        # Search for key
        # Initialize empty dict if key not already exists
        current = self.blackboard.setdefault(key, {})

        # Build accessors
        accessors = self._build_accessors(receiver_id, accessors)

        # Search sub-dict for every accessor (except last one)
        # Initialize sub-dict if it not already exist
        if accessors:
            for accessor in accessors[:-1]:
                current = current.setdefault(accessor, {})
            current[accessors[-1]] = value
        else:
        # If global value / object save by key
            self.blackboard[key] = value
        return True



    def delete(self,
        key: str,
        receiver_id: int | None = None,
        accessors=None,
        **kwargs
    ) -> bool:
        """
        Delete information from the blackboard.
        Recursive clean up after deleting information.

        :param key: Identifier for the data (e.g. "assigned_goals" or topic)
        :param receiver_id: (Optional) The agent's ID receiving the data.
                            If provided and accessors is None, used as subkey.
        :param accessors: (Optional) List of subkeys for nested structures beginning from key.
                            If given, receiver_id is ignored.
        :return: True if a value was found and deleted successfully. False otherwise.
        """
        # Search for key
        current = self.blackboard.get(key, None)
        if current is None:
            return False

        # Build accessors
        accessors = self._build_accessors(receiver_id, accessors)

        # If no accessors delete whole entry for key
        if not accessors:
            return self.blackboard.pop(key, None) is not None

        # Go through nested structures and search for accessors
        # Keep track of path for recursive clean up
        parents = []
        parents.append((self.blackboard, key))
        for accessor in accessors[:-1]:
            if not isinstance(current, dict) or accessor not in current:
                return False
            parents.append((current, accessor))
            current = current[accessor]

        # Try to delete the target incl. the now unnecessary key
        deleted =  current.pop(accessors[-1], None) is not None

        # Recursive clean up empty dicts
        if deleted:
            for parent_dict, parent_key in reversed(parents):
                # Don't delete the top-level key from self.blackboard in this loop
                if parent_key == key and parent_dict is self.blackboard:
                    # If the dict under key is now empty, remove key from blackboard
                    if not self.blackboard[key]:
                        self.blackboard.pop(key)
                    break
                # If the dict under parent_key is now empty, remove it from its parent
                if not parent_dict[parent_key]:
                    parent_dict.pop(parent_key)
                else:
                    # Stop if the dict is not empty anymore
                    break
        return deleted


