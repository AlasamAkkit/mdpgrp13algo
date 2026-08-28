import constants as constants
from misc.positioning import Position


class GridCell:
    def __init__(self, position, occupied):
        self.position = position
        self.occupied = occupied

    def __str__(self):
        return f"Cell({self.position})"

    __repr__ = __str__

    def __eq__(self, other):
        return (
            self.position.x == other.position.x
            and self.position.y == other.position.y
            and self.position.direction == other.position.direction
        )

    def __hash__(self):
        return hash(self.position.xy_dir())

    def copy(self):
        """
        Return a copy of this grid cell.
        """
        return GridCell(
            Position(self.position.x, self.position.y, self.position.direction),
            self.occupied,
        )

    def draw_cell(self):
        pass

    def draw_boundary_of_cell(self):
        pass

    def draw_all(self):
        pass
