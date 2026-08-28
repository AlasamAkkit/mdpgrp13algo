import pygame

import constants
from misc.direction import Direction
from misc.positioning import Position, RobotPosition


class Obstacle:
    def __init__(self, position: Position, index):
        """
        x -> x-coordinate of the obstacle.
        y -> y-coordinate of the obstacle.
        direction -> Which direction the image is facing. If image is on the right side of the obstacle, RIGHT.
        """
        # Check if the coordinates are multiples of 10 with offset 5. If they are not, then they are invalid
        # obstacle coordinates.
        # This is from the assumption that all obstacles are placed centered in each grid.
        if position.x % 10 != 0 or position.y % 10 != 0:
            raise AssertionError("Obstacle center coordinates must be multiples of 10!")

        self.position = position

        # Target position for permutation
        self.target_position = self.get_robot_target_pos()

        # Arrow to draw at the target coordinate.
        # self.target_image = pygame.transform.scale(pygame.image.load("entities/effects/target-pointer.png"),
        #                                            (25, 25))

        self.index = index

    def __str__(self):
        return self.position.__str__()

    __repr__ = __str__

    def check_within_boundary(self, position, yolo):
        """
        Checks whether a given position is within the safety boundary of this obstacle.
        If yes, means it can potentially hit the obstacle. We should avoid being inside the boundary
        """
        # if (self.position.x - constants.OBSTACLE_SAFETY_WIDTH <= position.x < self.position.x + constants.OBSTACLE_SAFETY_WIDTH) and (self.position.y - constants.OBSTACLE_SAFETY_WIDTH <= position.y < self.position.y + constants.OBSTACLE_SAFETY_WIDTH):
        #     return True

        # Assuming center is middle right square
        x_range, y_range = [], []

        x_range = [
            position.x - constants.GRID_CELL_LENGTH,
            position.x,
            position.x + constants.GRID_CELL_LENGTH,
        ]
        y_range = [
            position.y - constants.GRID_CELL_LENGTH,
            position.y,
            position.y + constants.GRID_CELL_LENGTH,
        ]

        # if position.direction == Direction.TOP:
        #     x_range = [position.x - constants.GRID_CELL_LENGTH,
        #                position.x]
        #     y_range = [position.y - constants.GRID_CELL_LENGTH,
        #                position.y, position.y + constants.GRID_CELL_LENGTH]
        # elif position.direction == Direction.BOTTOM:
        #     x_range = [position.x, position.x + constants.GRID_CELL_LENGTH]
        #     y_range = [position.y - constants.GRID_CELL_LENGTH,
        #                position.y, position.y + constants.GRID_CELL_LENGTH]
        # elif position.direction == Direction.RIGHT:
        #     x_range = [position.x - constants.GRID_CELL_LENGTH,
        #                position.x, position.x + constants.GRID_CELL_LENGTH]
        #     y_range = [position.y, position.y + constants.GRID_CELL_LENGTH]
        # elif position.direction == Direction.LEFT:
        #     x_range = [position.x - constants.GRID_CELL_LENGTH,
        #                position.x, position.x + constants.GRID_CELL_LENGTH]
        #     y_range = [position.y - constants.GRID_CELL_LENGTH, position.y]

        # print(f"Checking {position.x},{position.y}:", x_range, y_range)
        for x in x_range:
            for y in y_range:
                # cross
                if yolo == 1 and not (position.x == x or position.y == y):
                    continue

                # 1x1
                if yolo == 2 and not (position.x == x and position.y == y):
                    continue

                diffX = abs(self.position.x - x)
                diffY = abs(self.position.y - y)

                # if (position.direction.value - self.position.direction.value) == 180 and (diffX == 1 or diffY == 1):
                #     continue

                if (diffY < constants.OBSTACLE_SAFETY_WIDTH + 1) and (
                    diffX < constants.OBSTACLE_SAFETY_WIDTH + 1
                ):
                    return True

        return False

    def get_boundary_points(self):
        """
        Get points at the corner of the virtual obstacle for this image.

        Useful for checking if a point is within the boundary of this obstacle.
        """
        upper = self.position.y + constants.OBSTACLE_SAFETY_WIDTH
        lower = self.position.y - constants.OBSTACLE_SAFETY_WIDTH
        left = self.position.x - constants.OBSTACLE_SAFETY_WIDTH
        right = self.position.x + constants.OBSTACLE_SAFETY_WIDTH

        return [
            # Note that in this case, the direction does not matter.
            Position(left, lower),  # Bottom left.
            Position(right, lower),  # Bottom right.
            Position(left, upper),  # Upper left.
            Position(right, upper),  # Upper right.
        ]

    def get_robot_target_pos(self):
        """
        Returns the point that the robot should target for, including the target orientation.

        Note that the target orientation is now with respect to the robot. If the robot needs to face right, then
        we use 0 degrees.

        We can store this information within a Position object.

        The object will also store the angle that the robot should face.
        """

        """Boundary checking
        Cases include Side boundaries and Corners
        Robot target position are adjusted accordingly

        """
        # bottom left corner edge case
        # we add 10 to prevent the robot from going too close to the boundary for image facing top and right!
        if self.position.y == 0 and self.position.x == 0:
            if self.position.direction == Direction.TOP:
                return RobotPosition(
                    self.position.x + 10,
                    self.position.y
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    Direction.BOTTOM,
                )
            elif self.position.direction == Direction.BOTTOM:
                return RobotPosition(
                    self.position.x,
                    self.position.y
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    Direction.TOP,
                )
            elif self.position.direction == Direction.LEFT:
                return RobotPosition(
                    self.position.x
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    self.position.y,
                    Direction.RIGHT,
                )
            else:
                return RobotPosition(
                    self.position.x
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    self.position.y + 10,
                    Direction.LEFT,
                )

        # top left corner edge case!
        elif self.position.y == 190 and self.position.x == 0:
            if self.position.direction == Direction.TOP:
                return RobotPosition(
                    self.position.x,
                    self.position.y
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    Direction.BOTTOM,
                )
            elif self.position.direction == Direction.BOTTOM:
                return RobotPosition(
                    self.position.x + 10,
                    self.position.y
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    Direction.TOP,
                )
            elif self.position.direction == Direction.LEFT:
                return RobotPosition(
                    self.position.x
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    self.position.y,
                    Direction.RIGHT,
                )
            else:
                return RobotPosition(
                    self.position.x
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    self.position.y - 10,
                    Direction.LEFT,
                )

        # top right edge case
        elif self.position.y == 190 and self.position.x == 190:
            if self.position.direction == Direction.TOP:
                return RobotPosition(
                    self.position.x,
                    self.position.y
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    Direction.BOTTOM,
                )
            elif self.position.direction == Direction.BOTTOM:
                return RobotPosition(
                    self.position.x - 10,
                    self.position.y
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    Direction.TOP,
                )
            elif self.position.direction == Direction.LEFT:
                return RobotPosition(
                    self.position.x
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    self.position.y - 10,
                    Direction.RIGHT,
                )
            else:
                return RobotPosition(
                    self.position.x
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    self.position.y,
                    Direction.LEFT,
                )

        # bottom right edge case!
        elif self.position.y == 0 and self.position.x == 190:
            if self.position.direction == Direction.TOP:
                return RobotPosition(
                    self.position.x - 10,
                    self.position.y
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    Direction.BOTTOM,
                )
            elif self.position.direction == Direction.BOTTOM:
                return RobotPosition(
                    self.position.x,
                    self.position.y
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    Direction.TOP,
                )
            elif self.position.direction == Direction.LEFT:
                return RobotPosition(
                    self.position.x
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    self.position.y + 10,
                    Direction.RIGHT,
                )
            else:
                return RobotPosition(
                    self.position.x
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    self.position.y,
                    Direction.LEFT,
                )
        # cases where the obstacle is placed at the bottom of the field but not at the bottom left or right edge
        elif self.position.y == 0:
            if self.position.direction == Direction.TOP:
                return RobotPosition(
                    self.position.x,
                    self.position.y
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    Direction.BOTTOM,
                )
            elif self.position.direction == Direction.BOTTOM:
                return RobotPosition(
                    self.position.x,
                    self.position.y
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    Direction.TOP,
                )
            elif self.position.direction == Direction.LEFT:
                return RobotPosition(
                    self.position.x
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    self.position.y + 10,
                    Direction.RIGHT,
                )
            else:
                return RobotPosition(
                    self.position.x
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    self.position.y + 10,
                    Direction.LEFT,
                )

        # cases where obstacle is placed at top of the field but not at the top left or right edges
        elif self.position.y == 190:
            if self.position.direction == Direction.TOP:
                return RobotPosition(
                    self.position.x,
                    self.position.y
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    Direction.BOTTOM,
                )
            elif self.position.direction == Direction.BOTTOM:
                return RobotPosition(
                    self.position.x,
                    self.position.y
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    Direction.TOP,
                )
            elif self.position.direction == Direction.LEFT:
                return RobotPosition(
                    self.position.x
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    self.position.y - 10,
                    Direction.RIGHT,
                )
            else:
                return RobotPosition(
                    self.position.x
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    self.position.y - 10,
                    Direction.LEFT,
                )

        # cases where obstacle is placed at left side of field but not at bottom or top left
        elif self.position.x == 0:
            if self.position.direction == Direction.TOP:
                return RobotPosition(
                    self.position.x + 10,
                    self.position.y
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    Direction.BOTTOM,
                )
            elif self.position.direction == Direction.BOTTOM:
                return RobotPosition(
                    self.position.x + 10,
                    self.position.y
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    Direction.TOP,
                )
            elif self.position.direction == Direction.LEFT:
                return RobotPosition(
                    self.position.x
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    self.position.y + 10,
                    Direction.RIGHT,
                )
            else:
                return RobotPosition(
                    self.position.x
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    self.position.y,
                    Direction.LEFT,
                )

        # cases where obstacle is placed at right side of field but not at bottom or top right
        elif self.position.x == 190:
            if self.position.direction == Direction.TOP:
                return RobotPosition(
                    self.position.x - 10,
                    self.position.y
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    Direction.BOTTOM,
                )  # weakness
            elif self.position.direction == Direction.BOTTOM:
                return RobotPosition(
                    self.position.x - 10,
                    self.position.y
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    Direction.TOP,
                )  # weakness
            elif self.position.direction == Direction.LEFT:
                return RobotPosition(
                    self.position.x
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    self.position.y,
                    Direction.RIGHT,
                )
            else:
                return RobotPosition(
                    self.position.x
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    self.position.y,
                    Direction.LEFT,
                )
        # all other locations can be freely accessed via the 4 sides, no need to account for being close to the edges
        else:
            if self.position.direction == Direction.TOP:
                return RobotPosition(
                    self.position.x,
                    self.position.y
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    Direction.BOTTOM,
                )
            elif self.position.direction == Direction.BOTTOM:
                return RobotPosition(
                    self.position.x,
                    self.position.y
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    Direction.TOP,
                )
            elif self.position.direction == Direction.LEFT:
                return RobotPosition(
                    self.position.x
                    - constants.OBSTACLE_SAFETY_OFFSET
                    - constants.OBSTACLE_LENGTH,
                    self.position.y,
                    Direction.RIGHT,
                )
            else:
                return RobotPosition(
                    self.position.x
                    + constants.OBSTACLE_SAFETY_OFFSET
                    + constants.OBSTACLE_LENGTH,
                    self.position.y,
                    Direction.LEFT,
                )

    def draw_obstacles(self, screen):
        # Draw the obstacle onto the grid.
        # We need to translate the obstacle's center into that with respect to PyGame
        # Get the coordinates of the grid's bottom left-hand corner.
        rect = pygame.Rect(0, 0, constants.OBSTACLE_LENGTH, constants.OBSTACLE_LENGTH)
        rect.center = self.position.xy_pygame()
        pygame.draw.rect(screen, constants.RED, rect)

        # Draw the direction of the picture
        rect.width = constants.OBSTACLE_LENGTH / 2
        rect.height = constants.OBSTACLE_LENGTH / 2
        rect.center = self.position.xy_pygame()

        if self.position.direction == Direction.TOP:
            rect.centery -= constants.OBSTACLE_LENGTH / 4
        elif self.position.direction == Direction.BOTTOM:
            rect.centery += constants.OBSTACLE_LENGTH / 4
        elif self.position.direction == Direction.LEFT:
            rect.centerx -= constants.OBSTACLE_LENGTH / 4
        else:
            rect.centerx += constants.OBSTACLE_LENGTH / 4

        # Draw the picture place
        pygame.draw.rect(screen, constants.DARK_BLUE, rect)

    def draw_virtual_boundary(self, screen):
        # Get the boundary points
        points = self.get_boundary_points()

        # Draw left border
        pygame.draw.line(
            screen, constants.RED, points[0].xy_pygame(), points[2].xy_pygame()
        )
        # Draw right border
        pygame.draw.line(
            screen, constants.RED, points[1].xy_pygame(), points[3].xy_pygame()
        )
        # Draw upper border
        pygame.draw.line(
            screen, constants.RED, points[2].xy_pygame(), points[3].xy_pygame()
        )
        # Draw lower border
        pygame.draw.line(
            screen, constants.RED, points[0].xy_pygame(), points[1].xy_pygame()
        )

    def draw_robot_target(self, screen):
        target = self.get_robot_target_pos()

        rot_image = self.target_image
        angle = 0
        if target.direction == Direction.BOTTOM:
            angle = 180
        elif target.direction == Direction.LEFT:
            angle = 90
        elif target.direction == Direction.RIGHT:
            angle = -90

        rot_image = pygame.transform.rotate(rot_image, angle)
        rect = rot_image.get_rect()
        rect.center = target.xy_pygame()
        screen.blit(rot_image, rect)

    def draw(self, screen):
        # Draw the obstacle itself.
        self.draw_self(screen)
        # Draw the obstacle's boundary.
        self.draw_virtual_boundary(screen)
        # Draw the target for this obstacle.
        self.draw_robot_target(screen)
