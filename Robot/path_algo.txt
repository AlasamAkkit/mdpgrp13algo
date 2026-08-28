import math
from queue import PriorityQueue
from typing import List, Tuple
from Settings.attributes import *
from Settings.config import *
from Map.position import RobotPosition
from Map.grid import Grid
from Map.node import Node
from Robot.commands import *


class ModifiedAStar:
    def __init__(self, grid, brain, start: RobotPosition, end: RobotPosition):
        # We use a copy of the grid rather than use a reference
        # to the exact grid.
        self.grid: Grid = grid.copy()
        self.brain = brain
        self.total_cost = 0

        self.start = start
        self.end = end

    def getTotalCost(self):
        return self.total_cost

    def get_neighbours(self, pos: RobotPosition) -> List[Tuple[Node, RobotPosition, int, Command]]:
        neighbours = []

        # Check travel straights.
        straight_dist = 10 * SCALING_FACTOR
        straight_commands = [
            StraightCommand(straight_dist),
            StraightCommand(-straight_dist)
        ]
        for c in straight_commands:
            # Check if doing this command does not bring us to any invalid position.
            after, p = self.check_valid_command(c, pos)
            if after:
                neighbours.append((after, p, straight_dist, c))

        # Check turns
        turn_penalty = PATH_TURN_COST
        turn_commands = [
            TurnCommand(90, False),  # Forward right turn
            TurnCommand(-90, False),  # Forward left turn
            TurnCommand(90, True),  # Reverse with wheels to right.
            TurnCommand(-90, True),  # Reverse with wheels to left.
        ]
        for c in turn_commands:
            # Check if doing this command does not bring us to any invalid position.
            after, p = self.check_valid_command(c, pos)
            if after:
                neighbours.append((after, p, turn_penalty, c))

        return neighbours

    def check_valid_command(self, command: Command, p: RobotPosition):
        p = p.copy()
        if isinstance(command, TurnCommand):
            p_c = p.copy()
            for tick in range(command.ticks // PATH_TURN_CHECK_GRANULARITY):
                tick_command = TurnCommand(command.angle / (command.ticks // PATH_TURN_CHECK_GRANULARITY),
                                           command.rev)
                tick_command.apply_on_pos(p_c)
                if not (self.grid.check_valid_position(p_c) and self.grid.get_coordinate_node(*p_c.xy())):
                    return None, None
        command.apply_on_pos(p)
        if self.grid.check_valid_position(p) and (after := self.grid.get_coordinate_node(*p.xy())):
            after.pos.direction = p.direction
            return after.copy(), p
        return None, None

    def heuristic(self, curr_pos: RobotPosition):
        dx = abs(curr_pos.x - self.end.x)
        dy = abs(curr_pos.y - self.end.y)
        return math.sqrt(dx ** 2 + dy ** 2)

    def start_astar(self):
        frontier = PriorityQueue()
        backtrack = dict()
        cost = dict()

        # We can check what the goal node is
        goal_node = self.grid.get_coordinate_node(*self.end.xy()).copy()  # Take note of copy!
        goal_node.pos.direction = self.end.direction  # Set the required direction at this node.
        # Add starting node set into the frontier.
        start_node: Node = self.grid.get_coordinate_node(*self.start.xy()).copy()  # Take note of copy!
        start_node.direction = self.start.direction  # Make the node know which direction the robot is facing.
        offset = 0  # Used to tie-break.
        frontier.put((0, offset, (start_node, self.start)))  # Extra time parameter to tie-break same priority.
        cost[start_node] = 0
        # Having None as the parent means this key is the starting node.
        backtrack[start_node] = (None, None)  # Parent, Command

        while not frontier.empty():  # While there are still nodes to process.
            # Get the highest priority node.
            priority, _, (current_node, current_position) = frontier.get()

            # If the current node is our goal.
            if current_node == goal_node:
                # Get the commands needed to get to destination.
                self.extract_commands(backtrack, goal_node)
                self.total_cost=cost[goal_node]
                return current_position

            for new_node, new_pos, weight, c in self.get_neighbours(current_position):

                new_cost = cost.get(current_node) + weight

                if new_node not in backtrack or new_cost < cost[new_node]:
                    offset += 1
                    priority = new_cost + self.heuristic(new_pos)

                    frontier.put((priority, offset, (new_node, new_pos)))
                    backtrack[new_node] = (current_node, c)
                    cost[new_node] = new_cost
        return None

    def extract_commands(self, backtrack, goal_node):
        """
        Extract required commands to get to destination.
        """
        commands = []
        curr = goal_node
        while curr:
            curr, c = backtrack.get(curr, (None, None))
            if c:
                commands.append(c)
        commands.reverse()
        self.brain.commands.extend(commands)