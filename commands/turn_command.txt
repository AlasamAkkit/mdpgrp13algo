import math

import constants as constants
from commands.command import Command
from misc.direction import Direction
from misc.positioning import Position, RobotPosition
from misc.type_of_turn import TypeOfTurn


class TurnCommand(Command):
    def __init__(self, type_of_turn, left, right, reverse):
        """
        Angle to turn and whether the turn is done in reverse or not. Note that this is in degrees.

        Note that negative angles will always result in the robot being rotated clockwise.
        """
        time = 0
        if type_of_turn == TypeOfTurn.SMALL:
            time = 10  # SOME VALUE TO BE EMPIRICALLY DETERMINED
        elif type_of_turn == TypeOfTurn.MEDIUM:
            time = 20  # SOME VALUE TO BE EMPIRICALLY DETERMINED
        elif type_of_turn == TypeOfTurn.LARGE:
            time = 30  # SOME VALUE TO BE EMPIRICALLY DETERMINED

        if type_of_turn == TypeOfTurn.SMALL:
            time = 10  # SOME VALUE TO BE EMPIRICALLY DETERMINED
        elif type_of_turn == TypeOfTurn.MEDIUM:
            time = 20  # SOME VALUE TO BE EMPIRICALLY DETERMINED
        elif type_of_turn == TypeOfTurn.LARGE:
            time = 30  # SOME VALUE TO BE EMPIRICALLY DETERMINED

        super().__init__(time)
        self.type_of_turn = type_of_turn
        self.left = left
        self.right = right
        self.reverse = reverse

    def __str__(self):
        # return f"TurnCommand:{self.type_of_turn}, {self.total_ticks} ticks, rev={self.reverse}, left={self.left}, right={self.right}) "
        return f"TurnCommand:{self.type_of_turn}, rev={self.reverse}, left={self.left}, right={self.right}) "

    __repr__ = __str__

    def process_one_tick(self, robot):
        if self.total_ticks == 0:
            return

        self.tick()
        robot.turn(self.type_of_turn, self.left, self.right, self.reverse)

    def get_type_of_turn(self):
        return self.type_of_turn

    def apply_on_pos(self, curr_pos: Position):
        """
        changes the robot position according to what command it is and where the robot is currently at
        """
        assert isinstance(curr_pos, RobotPosition), print(
            "Cannot apply turn command on non-robot positions!"
        )

        # Get change in (x, y) coordinate.

        # turn left and forward
        if self.left and not self.right and not self.reverse:
            if self.type_of_turn == TypeOfTurn.SMALL:
                if curr_pos.direction == Direction.TOP:
                    curr_pos.x -= 10
                    curr_pos.y += 40
                elif curr_pos.direction == Direction.LEFT:
                    curr_pos.x -= 40
                    curr_pos.y -= 10
                elif curr_pos.direction == Direction.RIGHT:
                    curr_pos.x += 40
                    curr_pos.y += 10
                elif curr_pos.direction == Direction.BOTTOM:
                    curr_pos.x += 10
                    curr_pos.y -= 40
            elif self.type_of_turn == TypeOfTurn.MEDIUM:
                if curr_pos.direction == Direction.TOP:
                    curr_pos.x -= 30
                    curr_pos.y += 20
                    curr_pos.direction = Direction.LEFT
                elif curr_pos.direction == Direction.LEFT:
                    curr_pos.x -= 20
                    curr_pos.y -= 30
                    curr_pos.direction = Direction.BOTTOM
                elif curr_pos.direction == Direction.RIGHT:
                    curr_pos.x += 20
                    curr_pos.y += 30
                    curr_pos.direction = Direction.TOP
                elif curr_pos.direction == Direction.BOTTOM:
                    curr_pos.x += 30
                    curr_pos.y -= 20
                    curr_pos.direction = Direction.RIGHT

        # turn right and forward
        if self.right and not self.left and not self.reverse:
            if self.type_of_turn == TypeOfTurn.SMALL:
                if curr_pos.direction == Direction.TOP:
                    curr_pos.x += 10
                    curr_pos.y += 40
                elif curr_pos.direction == Direction.LEFT:
                    curr_pos.x -= 40
                    curr_pos.y += 10
                elif curr_pos.direction == Direction.RIGHT:
                    curr_pos.x += 40
                    curr_pos.y -= 10
                elif curr_pos.direction == Direction.BOTTOM:
                    curr_pos.x -= 10
                    curr_pos.y -= 40
            elif self.type_of_turn == TypeOfTurn.MEDIUM:
                if curr_pos.direction == Direction.TOP:
                    curr_pos.x += 30
                    curr_pos.y += 20
                    curr_pos.direction = Direction.RIGHT
                elif curr_pos.direction == Direction.LEFT:
                    curr_pos.x -= 20
                    curr_pos.y += 30
                    curr_pos.direction = Direction.TOP
                elif curr_pos.direction == Direction.RIGHT:
                    curr_pos.x += 20
                    curr_pos.y -= 30
                    curr_pos.direction = Direction.BOTTOM
                elif curr_pos.direction == Direction.BOTTOM:
                    curr_pos.x -= 30
                    curr_pos.y -= 20
                    curr_pos.direction = Direction.LEFT

        # turn front wheels left and reverse
        if self.left and not self.right and self.reverse:
            if self.type_of_turn == TypeOfTurn.SMALL:
                if curr_pos.direction == Direction.TOP:
                    curr_pos.x -= 10
                    curr_pos.y -= 40
                elif curr_pos.direction == Direction.LEFT:
                    curr_pos.x += 40
                    curr_pos.y -= 10
                elif curr_pos.direction == Direction.RIGHT:
                    curr_pos.x -= 40
                    curr_pos.y += 10
                elif curr_pos.direction == Direction.BOTTOM:
                    curr_pos.x += 10
                    curr_pos.y += 40
            elif self.type_of_turn == TypeOfTurn.MEDIUM:
                if curr_pos.direction == Direction.TOP:
                    curr_pos.x -= 20
                    curr_pos.y -= 30
                    curr_pos.direction = Direction.RIGHT
                elif curr_pos.direction == Direction.LEFT:
                    curr_pos.x += 30
                    curr_pos.y -= 20
                    curr_pos.direction = Direction.TOP
                elif curr_pos.direction == Direction.RIGHT:
                    curr_pos.x -= 30
                    curr_pos.y += 20
                    curr_pos.direction = Direction.BOTTOM
                elif curr_pos.direction == Direction.BOTTOM:
                    curr_pos.x += 20
                    curr_pos.y += 30
                    curr_pos.direction = Direction.LEFT

        # turn front wheels right and reverse
        if self.right and not self.left and self.reverse:
            if self.type_of_turn == TypeOfTurn.SMALL:
                if curr_pos.direction == Direction.TOP:
                    curr_pos.x += 10
                    curr_pos.y -= 40
                elif curr_pos.direction == Direction.LEFT:
                    curr_pos.x += 40
                    curr_pos.y += 10
                elif curr_pos.direction == Direction.RIGHT:
                    curr_pos.x -= 40
                    curr_pos.y -= 10
                elif curr_pos.direction == Direction.BOTTOM:
                    curr_pos.x -= 10
                    curr_pos.y += 40
            elif self.type_of_turn == TypeOfTurn.MEDIUM:
                if curr_pos.direction == Direction.TOP:
                    curr_pos.x += 20
                    curr_pos.y -= 30
                    curr_pos.direction = Direction.LEFT
                elif curr_pos.direction == Direction.LEFT:
                    curr_pos.x += 30
                    curr_pos.y += 20
                    curr_pos.direction = Direction.BOTTOM
                elif curr_pos.direction == Direction.RIGHT:
                    curr_pos.x -= 30
                    curr_pos.y -= 20
                    curr_pos.direction = Direction.TOP
                elif curr_pos.direction == Direction.BOTTOM:
                    curr_pos.x -= 20
                    curr_pos.y += 30
                    curr_pos.direction = Direction.RIGHT

        return self

    def convert_to_message(self):
        if self.left and not self.right and not self.reverse:
            # This is going forward left.
            if self.type_of_turn == TypeOfTurn.SMALL:
                return "KF001"  # turn left small forward!
            elif self.type_of_turn == TypeOfTurn.MEDIUM:
                return "LF090"  # turn left medium forward!
            elif self.type_of_turn == TypeOfTurn.LARGE:
                return "LF180"  # turn left large forward!
        elif self.left and not self.right and self.reverse:
            # This is going backward and front wheels are turned to left
            if self.type_of_turn == TypeOfTurn.SMALL:
                return "KB001"  # turn left small reverse!
            elif self.type_of_turn == TypeOfTurn.MEDIUM:
                return "LB090"  # turn left medium reverse!
            elif self.type_of_turn == TypeOfTurn.LARGE:
                return "LB180"  # turn left large reverse!
        elif self.right and not self.left and not self.reverse:
            # This is going forward right.
            if self.type_of_turn == TypeOfTurn.SMALL:
                return "JF001"  # turn right small forward!
            elif self.type_of_turn == TypeOfTurn.MEDIUM:
                return "RF090"  # turn right medium forward!
            elif self.type_of_turn == TypeOfTurn.LARGE:
                return "RF180"  # turn right large forward!
        else:
            # This is going backward and the front wheels turned to the right.
            if self.type_of_turn == TypeOfTurn.SMALL:
                return "JB001"  # turn right small reverse!
            elif self.type_of_turn == TypeOfTurn.MEDIUM:
                return "RB090"  # turn right medium reverse!
            elif self.type_of_turn == TypeOfTurn.LARGE:
                return "RB180"  # turn right large reverse!
