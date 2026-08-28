"""Project-specific RPi hooks that your hardware/image teams must complete.

Every function marked ``EDIT ME`` is deliberately isolated here so that the
network framing and safety sequencing do not need to be rewritten.
"""

from __future__ import annotations


def translate_motion_command(planner_command: str) -> str:
    """Translate a planner command into the exact STM firmware vocabulary.

    EDIT ME if the STM does not directly accept ``SF030``, ``RF090``, etc.
    For example, some archived code mentions older forms such as ``STM|FC050``;
    do not implement that conversion until the STM team confirms its units.
    """

    return planner_command


def normalise_stm_response(response: str) -> str:
    """Normalise a decoded STM reply before comparing ACK/DONE/error tokens.

    EDIT ME if replies contain a prefix, sequence number, or command echo.
    The safe default only strips surrounding whitespace.
    """

    return response.strip()


def recognise_target(obstacle_id: int) -> int:
    """Capture and recognise the image for one obstacle on the Raspberry Pi.

    EDIT ME: call your camera/image-recognition module here and return the real
    positive Target ID.  ``Obstacle.target_id`` in the simulator is demo data
    and must not be returned as if it were a real recognition result.
    """

    raise NotImplementedError(
        "EDIT ME: connect recognise_target() to the RPi camera service"
    )


def publish_android_message(message: str) -> None:
    """Publish checklist messages to Android over your Bluetooth connection.

    EDIT ME: replace the print with your existing Bluetooth/RFCOMM send call.
    Expected messages include ``TARGET, obstacle, target``.  Robot-position
    updates can be added once real odometry or a trusted pose source exists.
    """

    print(f"[ANDROID EDIT ME] {message}")
