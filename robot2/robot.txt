import datetime

import pygame

import constants as constants
import misc.timer as timer
from commands.command import Command
from commands.go_straight_command import StraightCommand
from commands.turn_command import TurnCommand
from misc.direction import Direction
from misc.positioning import RobotPosition
from path_finding.hamiltonian import Hamiltonian


class Robot:
    def __init__(self, grid):
        # Note that we assume the robot starts always facing the top.
        # This value will never change, but it will not affect us as the robot uses a more fine-tuned internal
        # angle tracker.
        self.pos = RobotPosition(
            constants.ROBOT_SAFETY_DISTANCE,
            constants.ROBOT_SAFETY_DISTANCE,
            Direction.TOP,
            90,
        )

        self._start_copy = self.pos.copy()

        self.hamiltonian = Hamiltonian(self, grid)

        # self.__image = pygame.transform.scale(pygame.image.load("entities/effects/car-top.png"),
        #                                       (constants.ROBOT_LENGTH / 2, constants.ROBOT_LENGTH))
        # (constants.ROBOT_LENGTH / 2, constants.ROBOT_LENGTH / 2))

        # Stores the history of the path taken by the robot.
        self.path_hist = []

        # Index of the current command being executed.
        self.__current_command = 0
        self.printed = False  # Never printed total time before.

    def get_current_pos(self):
        return self.pos

    def __str__(self):
        print(f"robot is at {self.pos}")

    def setCurrentPos(self, x, y, direction):
        self.pos.x = constants.GRID_LENGTH - constants.GRID_CELL_LENGTH - (x * 10)
        self.pos.y = y * 10
        self.pos.direction = direction

    def setCurrentPosTask2(self, x, y, direction):
        self.pos.x = constants.TASK2_LENGTH - constants.GRID_CELL_LENGTH - (x * 10)
        self.pos.y = y * 10
        self.pos.direction = direction

    def start_algo_from_position(self, grid):
        self.pos = self.get_current_pos()
        self._start_copy = self.pos.copy()
        self.hamiltonian = Hamiltonian(self, grid)
        # Stores the history of the path taken by the robot.
        self.path_hist = []

        # Index of the current command being executed.
        self.__current_command = 0
        self.printed = False  # Never printed total time before.

    def convert_all_commands(self):
        """
        Convert the list of command objects to corresponding list of messages.
        """
        print("Converting commands to string...", end="")
        string_commands = [
            command.convert_to_message() for command in self.hamiltonian.commands
        ]
        print("Done!")
        return string_commands

    def turn(self, type_of_command, left, right, rev):
        """ """
        TurnCommand(type_of_command, left, right, rev).apply_on_pos(self.pos)

    def straight(self, dist):
        """
        Make a robot go straight.

        A negative number indicates that the robot will move in reverse, and vice versa.
        """
        StraightCommand(dist).apply_on_pos(self.pos)

    def draw_simple_hamiltonian_path(self, screen):
        prev = self._start_copy.xy_pygame()
        for obs in self.hamiltonian.simple_hamiltonian:
            target = obs.get_robot_target_pos().xy_pygame()
            pygame.draw.line(screen, constants.DARK_GREEN, prev, target)
            prev = target

    def draw_self(self, screen):
        # The arrow to represent the direction of the robot.
        rot_image = pygame.transform.rotate(self.__image, -(90 - self.pos.angle))
        rect = rot_image.get_rect()
        rect.center = self.pos.xy_pygame()
        screen.blit(rot_image, rect)

    def draw_historic_path(self, screen):
        for dot in self.path_hist:
            pygame.draw.circle(screen, constants.BLACK, dot, 2)

    def draw(self, screen):
        # Draw the robot itself.
        self.draw_self(screen)
        # Draw the simple hamiltonian path found by the robot.
        self.draw_simple_hamiltonian_path(screen)
        # Draw the path sketched by the robot
        self.draw_historic_path(screen)

    def update(self):
        # Store historic path
        if len(self.path_hist) == 0 or self.pos.xy_pygame() != self.path_hist[-1]:
            # Only add a new point history if there is none, and it is different from previous history.
            self.path_hist.append(self.pos.xy_pygame())

        # If no more commands to execute, then return.
        if self.__current_command >= len(self.hamiltonian.commands):
            return

        # Check current command has non-null ticks.
        # Needed to check commands that have 0 tick execution time.
        if self.hamiltonian.commands[self.__current_command].total_ticks == 0:
            self.__current_command += 1
            if self.__current_command >= len(self.hamiltonian.commands):
                return

        # If not, the first command in the list is always the command to execute.
        command: Command = self.hamiltonian.commands[self.__current_command]
        command.process_one_tick(self)
        # If there are no more ticks to do, then we can assume that we have
        # successfully completed this command, and so we can remove it.
        # The next time this method is run, then we will process the next command in the list.
        if command.ticks <= 0:
            print(f"Finished processing {command}, {self.pos}")
            self.__current_command += 1
            if (
                self.__current_command == len(self.hamiltonian.commands)
                and not self.printed
            ):
                total_time = 0
                for command in self.hamiltonian.commands:
                    total_time += command.time
                    total_time = round(total_time)
                # Calculate time for all commands
                # Then print it out.
                print(f"All commands took {datetime.timedelta(seconds=total_time)}")
                self.printed = True
                timer.Timer.end_timer()
