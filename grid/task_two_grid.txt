import math
from collections import deque
from typing import List

import pygame

import constants as constants
from grid.grid_cell import GridCell
from grid.obstacle import Obstacle
from misc.positioning import Position


class GridTwo:
    def __init__(self, obstacles: List[Obstacle]):
        self.obstacles = obstacles
        self.gridcells = self.generate_grid()

    def generate_grid(self):
        """
        Generate the grid cells that make up this grid.
        Cells are in 5's as we consider you are in the cell if you reach the middle of the cell
        All grids that fall within safety distance of obstacle will be marked as occupied
        """
        grid = deque()
        for i in range(constants.TASK2_LENGTH // constants.GRID_CELL_LENGTH):
            row = deque()
            # 20 cells per side of square with each being 10cm
            for j in range(constants.TASK2_WIDTH // constants.GRID_CELL_LENGTH):
                x = constants.GRID_CELL_LENGTH * j  # 0, 10, ... , 190
                y = constants.GRID_CELL_LENGTH * i  # 0, 10, ... , 190
                new_cell = GridCell(
                    Position(x, y), not self.check_valid_position(Position(x, y))
                )
                row.append(new_cell)
            # grid[0] will refer to last row, grid[19] refer to first row (from the bottom)
            grid.appendleft(row)
        return grid

    def get_grid_cell_corresponding_to_coordinate(self, x, y):
        """
        Get the GridCell that lies at the specified x, y coordinates.

        Note that the x-y coordinates are in terms of the grid, and must be scaled properly.(?)
        """
        col = math.floor(x / constants.GRID_CELL_LENGTH)
        row = (
            constants.TASK2_WIDTH // constants.GRID_CELL_LENGTH
            - math.floor(y / constants.GRID_CELL_LENGTH)
            - 1
        )
        try:
            return self.gridcells[row][col]
        except IndexError:
            return None

    def copy(self):
        """
        Return a copy of the grid.
        """
        cells = []
        for row in self.gridcells:
            new_row = []
            for col in row:
                new_row.append(col.copy())
            cells.append(new_row)
        new_grid = GridTwo(self.obstacles)
        new_grid.gridcells = cells
        return new_grid

    def check_valid_position(self, pos: Position):
        """
        Check if a current position can be here.
        """
        # Check if position is inside any obstacle.
        if any(obstacle.check_within_boundary(pos) for obstacle in self.obstacles):
            return False

        # Check if position too close to the border.
        # NOTE: We allow the robot to overextend the border a little!
        # We do this by setting the limit to be GRID_CELL_LENGTH rather than ROBOT_SAFETY_DISTANCE
        # if (pos.y < 0 or
        #     pos.y > constants.GRID_LENGTH) or \
        #         (pos.x < 0 or
        #          pos.x > constants.GRID_LENGTH):
        #     return False

        if (
            pos.y < constants.GRID_CELL_LENGTH
            or pos.y >= constants.GRID_LENGTH - constants.GRID_CELL_LENGTH
        ) or (
            pos.x < constants.GRID_CELL_LENGTH
            or pos.x >= constants.GRID_LENGTH - constants.GRID_CELL_LENGTH
        ):
            return False
        return True

    @classmethod
    def draw_arena_borders(cls, screen):
        # Draw thicker lines on multipliers of 2 (?)
        for i in range(1, constants.TASK2_LENGTH // constants.GRID_CELL_LENGTH):
            if i % 5 == 0:
                pygame.draw.line(
                    screen,
                    constants.DARK_GRAY,
                    (0, 0 + i * constants.GRID_CELL_LENGTH),
                    (constants.GRID_LENGTH, 0 + i * constants.GRID_CELL_LENGTH),
                )
                pygame.draw.line(
                    screen,
                    constants.DARK_GRAY,
                    (0 + i * constants.GRID_CELL_LENGTH, 0),
                    (0 + i * constants.GRID_CELL_LENGTH, constants.GRID_LENGTH),
                )
        """
        Draw the arena borders.
        """
        # Draw upper border
        pygame.draw.line(screen, constants.RED, (0, 0), (constants.GRID_LENGTH, 0))
        # Draw lower border
        pygame.draw.line(
            screen,
            constants.RED,
            (0, constants.GRID_LENGTH),
            (constants.GRID_LENGTH, constants.GRID_LENGTH),
        )
        # Draw left border
        pygame.draw.line(screen, constants.RED, (0, 0), (0, constants.GRID_LENGTH))
        # Draw right border
        pygame.draw.line(
            screen,
            constants.RED,
            (constants.GRID_LENGTH, 0),
            (constants.GRID_LENGTH, constants.GRID_LENGTH),
        )

        # Draw numbers on side of grid
        font = pygame.freetype.SysFont(None, 18)
        font.origin = True
        for i in range(constants.NO_OF_GRID_CELLS_PER_SIDE):
            font.render_to(
                screen,
                (i * constants.GRID_CELL_LENGTH + 8, constants.GRID_LENGTH + 25),
                f"{i}",
                pygame.Color("DarkBlue"),
            )
        for j in range(constants.NO_OF_GRID_CELLS_PER_SIDE):
            font.render_to(
                screen,
                (
                    constants.GRID_LENGTH + 10,
                    constants.GRID_LENGTH - j * constants.GRID_CELL_LENGTH - 8,
                ),
                f"{j}",
                pygame.Color("DarkBlue"),
            )

    def draw_obstacles(self, screen):
        for ob in self.obstacles:
            ob.draw(screen)

    def draw_nodes(self, screen):
        for row in self.gridcells:
            for col in row:
                col.draw(screen)

    def draw(self, screen):
        # Draw nodes
        self.draw_nodes(screen)

        # Draw arena borders
        self.draw_arena_borders(screen)
        # Draw obstacles
        self.draw_obstacles(screen)
