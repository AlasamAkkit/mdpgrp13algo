import pygame
import math
from typing import List
from collections import deque
from Map.obstacle import Obstacle
from Map.position import Position
from Map.node import Node
from Settings.attributes import *
from Settings.colors import *


class Grid:
    def __init__(self, obstacles: List[Obstacle]):
        self.obstacles = obstacles
        self.nodes = self.generate_nodes()

    def generate_nodes(self):
        """
        Generate the nodes for this grid.
        """
        nodes = deque()
        for i in range(GRID_NUM_GRIDS):
            row = deque()
            for j in range(GRID_NUM_GRIDS):
                x, y = (GRID_CELL_LENGTH / 2 + GRID_CELL_LENGTH * j), \
                       (GRID_CELL_LENGTH / 2 + GRID_CELL_LENGTH * i)
                new_node = Node(x, y, not self.check_valid_position(Position(x, y)))
                row.append(new_node)
            nodes.appendleft(row)
        return nodes

    def get_coordinate_node(self, x, y):
        col_num = math.floor(x / GRID_CELL_LENGTH)
        row_num = GRID_NUM_GRIDS - math.floor(y / GRID_CELL_LENGTH) - 1
        try:
            return self.nodes[row_num][col_num]
        except IndexError:
            return None

    def copy(self):
        """
        Return a copy of the grid.
        """
        nodes = []
        for row in self.nodes:
            new_row = []
            for col in row:
                new_row.append(col.copy())
            nodes.append(new_row)
        new_grid = Grid(self.obstacles)
        new_grid.nodes = nodes
        return new_grid

    def delete_obstacle(self):
        self.obstacles.pop(0)
        return self.obstacles

    def check_valid_position(self, pos: Position):
        # Check if position is inside any obstacle.
        if any(obstacle.check_within_boundary(*pos.xy()) for obstacle in self.obstacles):
            return False

        # Check if position too close to the border.
        # NOTE: We allow the robot to overextend the border a little!
        # We do this by setting the limit to be GRID_CELL_LENGTH rather than ROBOT_SAFETY_DISTANCE
        if (pos.y <= GRID_CELL_LENGTH / 2 or pos.y > GRID_LENGTH) or \
                (pos.x <= GRID_CELL_LENGTH / 2 or pos.x > GRID_LENGTH):
            return False
        return True

    @classmethod
    def draw_arena_borders(cls, screen):
        """
        Draw the arena borders.
        """
        # Draw upper border
        pygame.draw.line(screen, BLACK, (0, 0), (GRID_LENGTH, 0))
        # Draw lower border
        pygame.draw.line(screen, BLACK, (0, GRID_LENGTH), (GRID_LENGTH, GRID_LENGTH))
        # Draw left border
        pygame.draw.line(screen, BLACK, (0, 0), (0, GRID_LENGTH))
        # Draw right border
        pygame.draw.line(screen, BLACK, (GRID_LENGTH, 0), (GRID_LENGTH, GRID_LENGTH))

    def draw_obstacles(self, screen):
        for ob in self.obstacles:
            ob.draw(screen)

    def draw_nodes(self, screen):
        for row in self.nodes:
            for col in row:
                col.draw(screen)

    def draw(self, screen):
        # Draw nodes
        self.draw_nodes(screen)
        # Draw arena borders
        self.draw_arena_borders(screen)
        # Draw obstacles
        self.draw_obstacles(screen)