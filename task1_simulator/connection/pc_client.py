"""Algorithm-PC client for previewing or sending a Task 1 plan to the RPi.

Examples, from the ``Algorithm`` directory::

    python -m task1_simulator.connection.pc_client \
        --obstacles task1_simulator/connection/example_obstacles.json

    python -m task1_simulator.connection.pc_client \
        --obstacles task1_simulator/connection/example_obstacles.json --send

Preview is the default.  No socket is opened unless ``--send`` is supplied.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import sys
from typing import Any, Iterable

from task1_simulator import GridPose, Heading, Obstacle, PlanResult, Task1Planner

from . import config
from .protocol import JsonLineConnection, ProtocolError, build_plan_message


TERMINAL_RESPONSE_TYPES = {"plan_complete", "error"}


def _read_json(path: Path) -> Any:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ValueError(f"cannot read {path}: {exc}") from exc

    # utf-8-sig accepts ordinary UTF-8 and a UTF-8 BOM.  utf-16 support makes
    # files created with redirection in older Windows PowerShell usable too.
    errors: list[str] = []
    for encoding in ("utf-8-sig", "utf-16", "utf-16-le"):
        try:
            text = raw.decode(encoding)
        except UnicodeDecodeError as exc:
            errors.append(f"{encoding}: {exc}")
            continue
        try:
            return json.loads(text)
        except json.JSONDecodeError as exc:
            errors.append(f"{encoding}: {exc}")
    raise ValueError(f"{path} is not valid JSON ({'; '.join(errors)})")


def plan_from_obstacle_file(path: Path) -> PlanResult:
    """Load an arena JSON file and calculate a complete physical route."""

    payload = _read_json(path)
    rows = payload.get("obstacles") if isinstance(payload, dict) else payload
    if not isinstance(rows, list):
        raise ValueError("obstacle JSON must be a list or an object with 'obstacles'")

    obstacles: list[Obstacle] = []
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f"obstacle {index} must be a JSON object")
        try:
            obstacle_id = row["id"]
            x = row["x"]
            y = row["y"]
            face = Heading.parse(row["face"])
            target_id = row.get("target_id")
            obstacles.append(Obstacle(obstacle_id, x, y, face, target_id))
        except KeyError as exc:
            raise ValueError(f"obstacle {index} is missing {exc.args[0]!r}") from exc
        except ValueError as exc:
            raise ValueError(f"obstacle {index}: {exc}") from exc

    result = Task1Planner(obstacles).plan(GridPose(1, 1, Heading.N))
    if not result.complete:
        raise ValueError(
            "refusing to send an incomplete plan; unreachable obstacles: "
            f"{list(result.unreachable_ids)}"
        )
    return result


def commands_from_plan_file(path: Path) -> tuple[str, ...]:
    """Read either a command array or the simulator's headless JSON output."""

    payload = _read_json(path)
    if isinstance(payload, dict):
        if payload.get("complete") is False:
            raise ValueError("refusing to send a plan marked incomplete")
        commands = payload.get("commands")
    else:
        commands = payload
    if not isinstance(commands, list):
        raise ValueError("plan JSON must contain a 'commands' array")
    # build_plan_message performs the strict command allow-list validation.
    return tuple(commands)


def message_from_result(
    result: PlanResult,
    token: str,
    *,
    plan_id: str | None = None,
) -> dict[str, Any]:
    """Convert a verified planner result into an authenticated request."""

    if not result.complete:
        raise ValueError(
            "refusing to send an incomplete plan; unreachable obstacles: "
            f"{list(result.unreachable_ids)}"
        )
    return build_plan_message(result.command_messages, token, plan_id=plan_id)


def send_plan(
    message: dict[str, Any],
    *,
    host: str,
    port: int,
    connect_timeout_s: float = config.SOCKET_CONNECT_TIMEOUT_S,
    read_timeout_s: float = config.SOCKET_READ_TIMEOUT_S,
) -> list[dict[str, Any]]:
    """Send one plan and collect status frames until completion or failure."""

    responses: list[dict[str, Any]] = []
    with socket.create_connection((host, port), timeout=connect_timeout_s) as sock:
        sock.settimeout(read_timeout_s)
        channel = JsonLineConnection(sock)
        channel.send(message)
        while True:
            response = channel.receive()
            responses.append(response)
            print(json.dumps(response, ensure_ascii=True))
            if response.get("type") in TERMINAL_RESPONSE_TYPES:
                return responses


def _safe_preview(message: dict[str, Any]) -> dict[str, Any]:
    preview = dict(message)
    preview["token"] = "*** hidden ***"
    return preview


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Preview or send a validated Task 1 command plan to the RPi"
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--obstacles",
        type=Path,
        help="JSON obstacle list; calculate a new route before sending",
    )
    source.add_argument(
        "--plan",
        type=Path,
        help="JSON command list or --headless-plan output",
    )
    parser.add_argument(
        "--send",
        action="store_true",
        help="actually connect to the RPi; without this flag only preview",
    )
    parser.add_argument(
        "--host",
        default=config.RPI_HOST,
        help="RPi address (EDIT ME in config.py or override here)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=config.RPI_PORT,
        help="RPi TCP port",
    )
    parser.add_argument(
        "--token",
        default=config.SHARED_TOKEN,
        help="shared token; editing config.py avoids exposing it in shell history",
    )
    parser.add_argument("--plan-id", help="optional safe identifier for this run")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        if args.obstacles is not None:
            result = plan_from_obstacle_file(args.obstacles)
            message = message_from_result(result, args.token, plan_id=args.plan_id)
            print(
                f"Complete route: {list(result.visit_order)}; "
                f"{len(result.command_messages)} commands; "
                f"estimated {result.estimated_seconds:.2f} s"
            )
        else:
            commands = commands_from_plan_file(args.plan)
            message = build_plan_message(commands, args.token, plan_id=args.plan_id)

        print(json.dumps(_safe_preview(message), indent=2))
        if not args.send:
            print("Preview only. Add --send after the RPi bridge is running.")
            return 0

        responses = send_plan(message, host=args.host, port=args.port)
        terminal = responses[-1]
        return 0 if terminal.get("type") == "plan_complete" else 2
    except (ValueError, ProtocolError, OSError, TimeoutError) as exc:
        print(f"Connection error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
