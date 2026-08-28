import itertools
import math
from collections import deque
from typing import Tuple

import constants as constants
from commands.go_straight_command import StraightCommand
from commands.scan_obstacle_command import ScanCommand
from grid.grid import Grid
from grid.obstacle import Obstacle
from misc.direction import Direction
from path_finding.modified_a_star import ModifiedAStar
from path_finding.weighted_a_star import WeightedAStar


class Hamiltonian:
    def __init__(self, robot, grid: Grid):
        self.robot = robot
        self.grid = grid

        # Compute the simple Hamiltonian path for all obstacles
        self.simple_hamiltonian = tuple()

        # Create all the commands required to finish the course.
        self.commands = deque()

    def get_simple_hamiltonian(self):
        return self.simple_hamiltonian

    def compute_simple_hamiltonian_path(self) -> Tuple[Obstacle]:
        """
        Get the Hamiltonian Path to all points with the best possible effort.
        This is a simple calculation where we assume that we travel directly to the next obstacle.
        """
        # Generate all possible permutations for the image obstacles
        perms = list(itertools.permutations(self.grid.obstacles))

        # Get the path that has the least distance travelled.
        def calc_distance(path):
            def weight_factor(source_pos, dest_pos, is_first) -> int:
                # # Right Grid to Left Grid, Top of Grid to Bottom of Grid unlikely
                # # if same direction (robot and targeted position has same direction)
                # if source_pos.direction.value - dest_pos.direction.value == 0:
                #     weight = 1 if is_first else 5
                # # if opposite direction
                # elif abs(source_pos.direction.value - dest_pos.direction.value) == 180:
                #     weight = 3
                # # if turn right or left
                # else:
                #     weight = 1.5 if check_pos(source_pos, dest_pos) else 7

                # Test code to use the shortest grid distance add with the difference in direction to calculate weight
                sx, sy = source_pos.x, source_pos.y
                ex, ey = dest_pos.x, dest_pos.y

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

                weight = (diag + abs(sx - ex) + abs(sy - ey)) // 10
                weight += abs(source_pos.direction.value - dest_pos.direction.value) // 90 * 5

                return weight

            def check_pos(robot_pos, target_pos):
                # left turn
                if robot_pos.direction.value - target_pos.direction.value == -90:
                    if robot_pos.direction == Direction.TOP:
                        return True if target_pos.x < robot_pos.x else False
                    if robot_pos.direction == Direction.BOTTOM:
                        return True if target_pos.x > robot_pos.x else False
                    if robot_pos.direction == Direction.LEFT:
                        return True if target_pos.y < robot_pos.y else False
                    if robot_pos.direction == Direction.RIGHT:
                        return True if target_pos.y > robot_pos.y else False
                # right turn
                else:
                    if robot_pos.direction == Direction.TOP:
                        return True if target_pos.x > robot_pos.x else False
                    if robot_pos.direction == Direction.BOTTOM:
                        return True if target_pos.x < robot_pos.x else False
                    if robot_pos.direction == Direction.LEFT:
                        return True if target_pos.y > robot_pos.y else False
                    if robot_pos.direction == Direction.RIGHT:
                        return True if target_pos.y < robot_pos.y else False

            # Create all target points, including the start.
            targets = [self.robot.pos.xy()]
            # Try out all the different permutations
            for obstacle in path:
                targets.append(obstacle.target_position.xy())

            # print("Targets ", targets)
            # print("Path ", path)
            # print()

            dist = 0
            multiplier = 1
            for i in range(len(targets) - 1):
                # Weight factor
                if i == 0:
                    # From start to first obstacle
                    multiplier = weight_factor(
                        self.robot.pos, path[i].target_position, True
                    )
                else:
                    # From obstacle to another obstacle
                    multiplier = weight_factor(
                        path[i - 1].target_position, path[i].target_position, False
                    )

                dist += multiplier * (
                    math.sqrt(
                        ((targets[i][0] - targets[i + 1][0]) ** 2)
                        + ((targets[i][1] - targets[i + 1][1]) ** 2)
                    )
                )
            return dist

        print("Calculating Distance for all possible permutation\n")
        # Change to max to show paths change
        # simple now holds the permutation that gives the lowest distance
        simple = min(perms, key=calc_distance)
        print("\nFound a simple hamiltonian path: ")
        # print(simple)

        # print out every obstacle in order of visitation
        for ob in simple:
            print(f"\t{ob}")
        print()
        print("Found Shortest Hamiltonian Path")

        # calc_distance(simple)
        # returns order of visitation of obstacles (the lowest cost)
        return simple

    def compress_paths(self):
        """
        Compress similar commands into one command.
        Compressing many straight line commands into one.
        Helps to reduce the number of commands.
        """
        print("Compressing commands... ", end="")
        index = 0
        new_commands = deque()

        while index < len(self.commands):
            command = self.commands[index]

            if isinstance(command, StraightCommand):
                new_length = 0
                while index < len(self.commands) and isinstance(
                    self.commands[index], StraightCommand
                ):
                    new_length += self.commands[index].dist
                    index += 1
                command = StraightCommand(new_length)
                new_commands.append(command)
            else:
                new_commands.append(command)
                index += 1

        self.commands = new_commands
        print("Done!")

    def plan_path(self):
        print("-" * 40)
        print("STARTING PATH COMPUTATION...")
        self.simple_hamiltonian = self.compute_simple_hamiltonian_path()
        print()

        # we use a deep copy rather than a reference
        curr = self.robot.pos.copy()
        for obstacle in self.simple_hamiltonian:
            target = obstacle.get_robot_target_pos()
            rerun = 0
            print(f"Obstacle is at: {obstacle}")
            print(f"Planning from current {curr} to target {target}")

            mode = constants.WEIGHTED_A_STAR
            if mode == constants.MODIFIED_A_STAR:
                res, cmd = ModifiedAStar(
                    self.grid, self, curr, target, rerun
                ).start_astar(True)
            else:
                res, cmd = WeightedAStar(
                    self.grid, self, curr, target, rerun
                ).start_astar(True)

            while res is None and rerun != 2:
                print(f"No path found from {curr} to {obstacle}")
                print("Fuck it... YOLO", end=" ")
                rerun += 1
                res, cmd = ModifiedAStar(
                    self.grid, self, curr, target, rerun
                ).start_astar(True)
                if res:
                    break
            if res is None:
                print(f"No path found from {curr} to {obstacle}")
            else:
                print("Path found!")
                curr = res
                self.commands.append(
                    ScanCommand(constants.ROBOT_SCAN_TIME, obstacle.index)
                )

        self.compress_paths()

        print()
        print("-" * 60)
        for command in self.commands:
            print(f"{command}")
        print("-" * 60)
        print()
