"""Safe, optional hardware connection helpers for the Task 1 planner.

Importing this package never opens a socket or serial port.  Use
``python -m task1_simulator.connection.pc_client`` on the algorithm PC and
``python -m task1_simulator.connection.rpi_bridge`` on the Raspberry Pi.
"""

from .protocol import (
    JsonLineConnection,
    ProtocolError,
    build_plan_message,
    classify_command,
    validate_commands,
)

__all__ = [
    "JsonLineConnection",
    "ProtocolError",
    "build_plan_message",
    "classify_command",
    "validate_commands",
]
