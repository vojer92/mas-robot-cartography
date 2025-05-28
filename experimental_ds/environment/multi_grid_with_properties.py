from mesa.space import MultiGrid
import numpy as np

class MultiGridWithProperties(MultiGrid):
    """
        Representation of the environment. Local knowledge of the environment and the environment itself.
    """
    def __init__(self, width, height, torus):
        super().__init__(width, height, torus)
        self.properties = {
            "unknown": np.full((width, height), True),
        }

    def add_property(self, name: str, default_value=None):
        self.properties[name] = np.full((self.width, self.height), default_value)

    def set_property(self, pos: tuple[int, int], name: str, value):
        self.properties[name][pos[0], pos[1]] = value

    def get_property(self, pos: tuple[int, int], name: str):
        if name not in self.properties:
            raise KeyError(f"Property '{name}' does not exist.")
        return self.properties[name][pos[0], pos[1]]