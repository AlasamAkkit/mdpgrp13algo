"""Validated newline-delimited JSON protocol for the PC/RPi connection."""

from __future__ import annotations

import json
import re
import socket
from typing import Any, Iterable, Mapping
from uuid import uuid4

from . import config


PROTOCOL_VERSION = 1
_MOTOR_PATTERN = re.compile(r"^(?:S[FB](\d{3})|[LR][FB]090)$")
_SCAN_PATTERN = re.compile(r"^P___([1-9]\d*)$")
_PLAN_ID_PATTERN = re.compile(r"^[A-Za-z0-9._:-]{1,80}$")


class ProtocolError(RuntimeError):
    """Raised when a network frame or command violates the allow-list."""


class ConnectionClosed(ProtocolError):
    """Raised when a peer disconnects before sending a complete frame."""


def classify_command(command: str, max_straight_cm: int | None = None) -> str:
    """Return ``motor`` or ``scan`` after strict command validation."""

    if not isinstance(command, str):
        raise ProtocolError("every command must be a string")
    if not command or command != command.strip():
        raise ProtocolError(f"invalid command {command!r}")
    if "\r" in command or "\n" in command:
        raise ProtocolError("commands cannot contain line breaks")

    maximum = config.MAX_STRAIGHT_CM if max_straight_cm is None else max_straight_cm
    motor_match = _MOTOR_PATTERN.fullmatch(command)
    if motor_match:
        distance_text = motor_match.group(1)
        if distance_text is not None:
            distance = int(distance_text)
            if distance == 0:
                raise ProtocolError("zero-distance straight commands are not allowed")
            if distance > maximum:
                raise ProtocolError(
                    f"straight distance {distance} cm exceeds the {maximum} cm limit"
                )
        return "motor"

    if _SCAN_PATTERN.fullmatch(command):
        return "scan"

    raise ProtocolError(f"unsupported command {command!r}")


def scan_obstacle_id(command: str) -> int:
    """Extract a scan obstacle ID after validating the command."""

    if classify_command(command) != "scan":
        raise ProtocolError(f"{command!r} is not a scan command")
    match = _SCAN_PATTERN.fullmatch(command)
    assert match is not None
    return int(match.group(1))


def validate_commands(
    commands: Iterable[str],
    *,
    max_commands: int | None = None,
    max_straight_cm: int | None = None,
) -> tuple[str, ...]:
    """Validate a complete plan before any physical movement starts."""

    if isinstance(commands, (str, bytes)):
        raise ProtocolError("commands must be a list, not one string")
    try:
        checked = tuple(commands)
    except TypeError as exc:
        raise ProtocolError("commands must be an iterable of strings") from exc

    maximum_count = config.MAX_PLAN_COMMANDS if max_commands is None else max_commands
    if not checked:
        raise ProtocolError("a plan must contain at least one command")
    if len(checked) > maximum_count:
        raise ProtocolError(
            f"plan has {len(checked)} commands; maximum is {maximum_count}"
        )

    scan_ids: set[int] = set()
    for index, command in enumerate(checked):
        try:
            kind = classify_command(command, max_straight_cm=max_straight_cm)
        except ProtocolError as exc:
            raise ProtocolError(f"command {index}: {exc}") from exc
        if kind == "scan":
            obstacle_id = scan_obstacle_id(command)
            if obstacle_id in scan_ids:
                raise ProtocolError(f"obstacle {obstacle_id} is scanned more than once")
            scan_ids.add(obstacle_id)

    return checked


def validate_plan_id(plan_id: object) -> str:
    if not isinstance(plan_id, str) or not _PLAN_ID_PATTERN.fullmatch(plan_id):
        raise ProtocolError("plan_id must be 1-80 safe ASCII characters")
    return plan_id


def build_plan_message(
    commands: Iterable[str],
    token: str,
    *,
    plan_id: str | None = None,
) -> dict[str, Any]:
    """Build the only request type that can ask the RPi to execute movement."""

    if not isinstance(token, str) or not token:
        raise ProtocolError("shared token must be a non-empty string")
    checked_id = validate_plan_id(plan_id or uuid4().hex)
    checked_commands = validate_commands(commands)
    return {
        "protocol_version": PROTOCOL_VERSION,
        "type": "execute_plan",
        "plan_id": checked_id,
        "token": token,
        "commands": list(checked_commands),
    }


def parse_plan_message(payload: Mapping[str, Any]) -> tuple[str, str, tuple[str, ...]]:
    """Validate an incoming execute-plan request without authenticating it."""

    if not isinstance(payload, Mapping):
        raise ProtocolError("request must be a JSON object")
    if payload.get("protocol_version") != PROTOCOL_VERSION:
        raise ProtocolError(
            f"unsupported protocol_version {payload.get('protocol_version')!r}"
        )
    if payload.get("type") != "execute_plan":
        raise ProtocolError(f"unsupported request type {payload.get('type')!r}")

    token = payload.get("token")
    if not isinstance(token, str) or not token:
        raise ProtocolError("request token must be a non-empty string")
    plan_id = validate_plan_id(payload.get("plan_id"))

    commands_value = payload.get("commands")
    if not isinstance(commands_value, list):
        raise ProtocolError("commands must be a JSON array")
    commands = validate_commands(commands_value)
    return plan_id, token, commands


class JsonLineConnection:
    """Incremental JSON-lines framing that handles split/coalesced TCP reads."""

    def __init__(self, sock: socket.socket, max_frame_bytes: int | None = None) -> None:
        self.socket = sock
        self.max_frame_bytes = (
            config.MAX_TCP_FRAME_BYTES if max_frame_bytes is None else max_frame_bytes
        )
        if self.max_frame_bytes <= 0:
            raise ValueError("max_frame_bytes must be positive")
        self._buffer = bytearray()

    def send(self, payload: Mapping[str, Any]) -> None:
        if not isinstance(payload, Mapping):
            raise ProtocolError("outgoing frame must be a JSON object")
        try:
            body = json.dumps(
                dict(payload), separators=(",", ":"), ensure_ascii=True
            ).encode("utf-8")
        except (TypeError, ValueError) as exc:
            raise ProtocolError(f"frame is not JSON serialisable: {exc}") from exc
        if len(body) > self.max_frame_bytes:
            raise ProtocolError("outgoing frame exceeds maximum size")
        self.socket.sendall(body + b"\n")

    def receive(self) -> dict[str, Any]:
        while True:
            newline = self._buffer.find(b"\n")
            if newline >= 0:
                if newline > self.max_frame_bytes:
                    raise ProtocolError("incoming frame exceeds maximum size")
                raw = bytes(self._buffer[:newline])
                del self._buffer[: newline + 1]
                if not raw:
                    raise ProtocolError("empty JSON frame")
                try:
                    value = json.loads(raw.decode("utf-8"))
                except UnicodeDecodeError as exc:
                    raise ProtocolError("incoming frame is not valid UTF-8") from exc
                except json.JSONDecodeError as exc:
                    raise ProtocolError(f"invalid JSON frame: {exc.msg}") from exc
                if not isinstance(value, dict):
                    raise ProtocolError("incoming frame must contain a JSON object")
                return value

            if len(self._buffer) > self.max_frame_bytes:
                raise ProtocolError("unterminated incoming frame exceeds maximum size")

            chunk = self.socket.recv(4096)
            if not chunk:
                raise ConnectionClosed("peer disconnected before a complete frame")
            self._buffer.extend(chunk)


def public_event(event_type: str, plan_id: str, **fields: Any) -> dict[str, Any]:
    """Create an RPi status frame without echoing the shared token."""

    return {
        "protocol_version": PROTOCOL_VERSION,
        "type": event_type,
        "plan_id": validate_plan_id(plan_id),
        **fields,
    }


__all__ = [
    "ConnectionClosed",
    "JsonLineConnection",
    "PROTOCOL_VERSION",
    "ProtocolError",
    "build_plan_message",
    "classify_command",
    "parse_plan_message",
    "public_event",
    "scan_obstacle_id",
    "validate_commands",
]
