from enum import Enum


class TypeOfTurn(Enum):
    SMALL = "mini turn, but face same direction"
    MEDIUM = "90 degree turn"
    LARGE = "180 degree turn"
