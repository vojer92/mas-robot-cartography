def manhattan_distance(
        pos_a: tuple[int, int],
        pos_b: tuple[int, int]
    ) -> float:
    """
    Calculate the Manhattan distance between two positions.
    """
    return sum(abs(a - b) for a, b in zip(pos_a, pos_b))