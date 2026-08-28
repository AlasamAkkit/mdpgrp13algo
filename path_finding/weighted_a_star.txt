from queue import PriorityQueue

from commands.command import Command
from commands.go_straight_command import StraightCommand
from commands.turn_command import TurnCommand
from misc.positioning import RobotPosition
from misc.type_of_turn import TypeOfTurn
from path_finding.modified_a_star import ModifiedAStar


class WeightedAStar(ModifiedAStar):
    def __init__(self, grid, brain, start: RobotPosition, end: RobotPosition, yolo):
        super().__init__(grid, brain, start, end, yolo)
        # Adjust the weights as needed
        self.weight_straight = 0
        self.weight_small_turn = 10
        self.weight_medium_turn = 20
        self.weight_large_turn = 30

    def distance_heuristic(self, curr_pos: RobotPosition):
        sx, sy = curr_pos.x, curr_pos.y
        ex, ey = self.end.x, self.end.y

        # Find the diagonal distance, then the straight distance
        diag = min(abs(sx - ex), abs(sy - ey))

        if sx < ex:
            sx += diag
        else:
            sx -= diag

        if sy < ey:
            sy += diag
        else:
            sy -= diag

        return (diag + abs(sx - ex) + abs(sy - ey)) // 10

    def get_weight(self, command: Command):
        if isinstance(command, StraightCommand):
            return self.weight_straight
        elif isinstance(command, TurnCommand):
            if command.get_type_of_turn() == TypeOfTurn.SMALL:
                return self.weight_small_turn
            elif command.get_type_of_turn() == TypeOfTurn.MEDIUM:
                return self.weight_medium_turn
            elif command.get_type_of_turn() == TypeOfTurn.LARGE:
                return self.weight_large_turn
        return 100  # Default weight for other commands

    def start_weighted_astar(self, flag):
        frontier = PriorityQueue()
        backtrack = dict()
        cost = dict()

        goal_node = self.end.xy()
        goal_node_with_dir = goal_node + (self.end.direction,)

        start_node = self.start.xy()
        start_node_with_dir = start_node + (self.start.direction,)

        offset = 0
        frontier.put((0, offset, (start_node_with_dir, self.start)))
        cost[start_node_with_dir] = 0
        backtrack[start_node_with_dir] = (None, None)

        while not frontier.empty():
            priority, _, (current_node, current_position) = frontier.get()

            if (
                current_node[0] == goal_node_with_dir[0]
                and current_node[1] == goal_node_with_dir[1]
                and current_node[2] == goal_node_with_dir[2]
            ):
                commands = self.extract_commands(backtrack, goal_node_with_dir, flag)
                return current_position, commands

            neighbors = self.get_neighbours(current_position)
            for new_node, new_pos, command_weight, c in neighbors:
                new_cost = cost.get(current_node) + command_weight
                # print(
                #     "Checking new node: ",
                #     new_node,
                #     " with cost: ",
                #     new_cost,
                #     " cost.get(new_node): ",
                #     cost.get(new_node, 100000),
                # )
                if new_cost < cost.get(new_node, 100000):
                    offset += 1
                    priority = (
                        new_cost
                        + self.distance_heuristic(new_pos)
                        + self.direction_heuristic(new_pos)
                        + self.get_weight(c)
                    )
                    # print(
                    #     "Priority breakdown: ",
                    #     new_cost,
                    #     " + ",
                    #     self.distance_heuristic(new_pos),
                    #     " + ",
                    #     self.direction_heuristic(new_pos),
                    #     " + ",
                    #     self.get_weight(c),
                    # )
                    # print("Adding new node: ", new_node, " with priority: ", priority)
                    frontier.put((priority, offset, (new_node, new_pos)))
                    backtrack[new_node] = (current_node, c)
                    cost[new_node] = new_cost

        return None, []

    def start_astar(self, flag):
        return self.start_weighted_astar(flag)
