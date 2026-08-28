import pygame
import datetime
from Map.position import RobotPosition
from Settings.attributes import *
from Settings.colors import *
from Robot.commands import *
from Robot.path_mgr import Brain


class Robot:
    def __init__(self, grid):
        self.pos = RobotPosition(ROBOT_START_X,
                                 ROBOT_START_Y,
                                 Direction.TOP,
                                 90)

        self._start_copy = self.pos.copy()

        self.brain = Brain(self, grid)

        self.__image = pygame.transform.scale(pygame.image.load("Assets/robot.png"),
                                              (100, 100))

        self.path_hist = []  # Stores the history of the path taken by the robot.

        self.__current_command = 0  # Index of the current command being executed.
        self.printed = False  # Never printed total time before.

    def get_current_pos(self):
        return self.pos

    def convert_all_commands(self):
        print("Converting commands to string...", end="")
        string_commands = [command.convert_to_message() for command in self.brain.commands]
        print("Done!")
        return string_commands

    def convert_commands(self):
        print("Converting commands to string...", end="")
        string_commands = [command.convert_to_message() for command in self.brain.commands]
        print("Done!")
        print("-" * 70)
        return string_commands


    def turn(self, d_angle, rev):
        TurnCommand(d_angle, rev).apply_on_pos(self.pos)

    def straight(self, dist):
        StraightCommand(dist).apply_on_pos(self.pos)

    def draw_simple_hamiltonian_path(self, screen):
        prev = self._start_copy.xy_pygame()
        for obs in self.brain.simple_hamiltonian:
            target = obs.get_robot_target_pos().xy_pygame()
            pygame.draw.line(screen, BLUE, prev, target)
            prev = target

    def draw_self(self, screen):
        rot_image = pygame.transform.rotate(self.__image, -(90 - self.pos.angle))
        rect = rot_image.get_rect()
        rect.center = self.pos.xy_pygame()
        screen.blit(rot_image, rect)

    def draw_historic_path(self, screen):
        for dot in self.path_hist:
            pygame.draw.circle(screen, BLACK, dot, 3)

    def draw(self, screen):
        # Draw the robot.
        self.draw_self(screen)
        # Draw the simple hamiltonian path found.
        self.draw_simple_hamiltonian_path(screen)
        # Draw the path sketched.
        self.draw_historic_path(screen)

    def update(self):
        # Store historic path
        if len(self.path_hist) == 0 or self.pos.xy_pygame() != self.path_hist[-1]:
            # Only add a new point history if there is none, and it is different from previous history.
            self.path_hist.append(self.pos.xy_pygame())

        # If no more commands to execute, then return.
        if self.__current_command >= len(self.brain.commands):
            return

        # Check current command has non-null ticks.
        # Needed to check commands that have 0 tick execution time.
        if self.brain.commands[self.__current_command].total_ticks == 0:
            self.__current_command += 1
            if self.__current_command >= len(self.brain.commands):
                return


        command: Command = self.brain.commands[self.__current_command]
        command.process_one_tick(self)

        if command.ticks <= 0:
            print(f"Finished processing {command}, {self.pos}")
            self.__current_command += 1
            if self.__current_command == len(self.brain.commands) and not self.printed:
                total_time = 0
                for command in self.brain.commands:
                    total_time += command.time
                    total_time = round(total_time)
                print(f"All commands took {datetime.timedelta(seconds=total_time)}")
                self.printed = True