"""RPi TCP server and sequential USB-serial executor for the STM robot.

Dry-run is the default and never imports PySerial or opens the STM device.
After editing ``config.py`` and ``hardware_hooks.py``, use ``--live`` explicitly
to enable physical movement.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hmac
import math
import socket
import sys
import time
from typing import Any, Callable, Iterable, Protocol

from . import config, hardware_hooks
from .protocol import (
    JsonLineConnection,
    ProtocolError,
    classify_command,
    parse_plan_message,
    public_event,
    scan_obstacle_id,
    validate_commands,
)


EventSink = Callable[[str, dict[str, Any]], None]
ScanHandler = Callable[[int], int | None]


class STMExecutor(Protocol):
    def execute_and_wait(self, planner_command: str) -> None: ...

    def emergency_stop(self) -> None: ...

    def close(self) -> None: ...


@dataclass(frozen=True, slots=True)
class STMSettings:
    serial_port: str = config.STM_SERIAL_PORT
    baud_rate: int = config.STM_BAUD_RATE
    bytesize: int = config.STM_BYTESIZE
    parity: str = config.STM_PARITY
    stopbits: float = config.STM_STOPBITS
    xonxoff: bool = config.STM_XONXOFF
    rtscts: bool = config.STM_RTSCTS
    dsrdtr: bool = config.STM_DSRDTR
    write_timeout_s: float = config.STM_WRITE_TIMEOUT_S
    read_slice_s: float = config.STM_READ_SLICE_S
    ack_required: bool = config.STM_ACK_REQUIRED
    ack_timeout_s: float = config.STM_ACK_TIMEOUT_S
    done_timeout_s: float = config.STM_DONE_TIMEOUT_S
    command_prefix: str = config.STM_COMMAND_PREFIX
    command_terminator: str = config.STM_COMMAND_TERMINATOR
    reply_terminator: str = config.STM_REPLY_TERMINATOR
    encoding: str = config.STM_ENCODING
    max_response_bytes: int = config.STM_MAX_RESPONSE_BYTES
    ack_responses: tuple[str, ...] = config.STM_ACK_RESPONSES
    done_responses: tuple[str, ...] = config.STM_DONE_RESPONSES
    error_prefixes: tuple[str, ...] = config.STM_ERROR_PREFIXES
    stop_command: str = config.STM_STOP_COMMAND

    def validate_for_live_use(self, shared_token: str) -> None:
        problems: list[str] = []
        if (
            not isinstance(shared_token, str)
            or "EDIT_ME" in shared_token
            or len(shared_token.strip()) < 16
        ):
            problems.append("SHARED_TOKEN")
        if not self.serial_port.strip():
            problems.append("STM_SERIAL_PORT")
        if isinstance(self.baud_rate, bool) or self.baud_rate <= 0:
            problems.append("STM_BAUD_RATE")
        if self.bytesize not in (5, 6, 7, 8):
            problems.append("STM_BYTESIZE")
        if self.parity not in ("N", "E", "O", "M", "S"):
            problems.append("STM_PARITY")
        if self.stopbits not in (1, 1.5, 2):
            problems.append("STM_STOPBITS")
        if not self.command_terminator:
            problems.append("STM_COMMAND_TERMINATOR")
        if not self.reply_terminator:
            problems.append("STM_REPLY_TERMINATOR")
        if self.ack_required and not self.ack_responses:
            problems.append("STM_ACK_RESPONSES")
        if not self.done_responses:
            problems.append("STM_DONE_RESPONSES")
        if not self.stop_command.strip() or "EDIT_ME" in self.stop_command:
            problems.append("STM_STOP_COMMAND")
        if (
            not math.isfinite(self.ack_timeout_s)
            or not math.isfinite(self.done_timeout_s)
            or self.ack_timeout_s <= 0
            or self.done_timeout_s <= 0
        ):
            problems.append("STM timeout values")
        if (
            not math.isfinite(self.read_slice_s)
            or self.read_slice_s <= 0
            or self.max_response_bytes <= 0
        ):
            problems.append("STM serial read limits")
        if set(self.ack_responses) & set(self.done_responses):
            problems.append("distinct STM ACK/DONE responses")
        if any(not value for value in (*self.ack_responses, *self.done_responses)):
            problems.append("non-empty STM ACK/DONE responses")
        if any(not value for value in self.error_prefixes):
            problems.append("non-empty STM error prefixes")
        text_settings = (
            self.command_prefix,
            self.command_terminator,
            self.reply_terminator,
            self.stop_command,
            *self.ack_responses,
            *self.done_responses,
            *self.error_prefixes,
        )
        try:
            for value in text_settings:
                if not isinstance(value, str):
                    raise TypeError
                value.encode(self.encoding)
        except (LookupError, TypeError, UnicodeEncodeError):
            problems.append("STM encoding/text tokens")
        if problems:
            raise ValueError(
                "live mode refused; edit these settings in connection/config.py: "
                + ", ".join(problems)
            )


class DryRunSTM:
    """Software-only executor used unless the operator explicitly selects live."""

    def __init__(self, delay_s: float = config.DRY_RUN_COMMAND_DELAY_S) -> None:
        self.delay_s = delay_s
        self.commands: list[str] = []
        self.stop_count = 0

    def execute_and_wait(self, planner_command: str) -> None:
        self.commands.append(planner_command)
        print(f"[DRY RUN STM] {planner_command} -> ACK -> DONE")
        if self.delay_s > 0:
            time.sleep(self.delay_s)

    def emergency_stop(self) -> None:
        if self.stop_count:
            return
        self.stop_count += 1
        print("[DRY RUN STM] emergency stop requested")

    def close(self) -> None:
        return


class SerialSTM:
    """One-command-at-a-time PySerial adapter with ACK/DONE state checking."""

    def __init__(self, settings: STMSettings) -> None:
        self.settings = settings
        self._serial: Any = None
        self._stop_sent = False

    def open(self) -> None:
        # PySerial stays optional for the simulator and Windows test suite.
        try:
            import serial
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "RPi serial support is missing; run: "
                "python3 -m pip install -r "
                "task1_simulator/connection/requirements-rpi.txt"
            ) from exc

        self._serial = serial.Serial(
            port=self.settings.serial_port,
            baudrate=self.settings.baud_rate,
            bytesize=self.settings.bytesize,
            parity=self.settings.parity,
            stopbits=self.settings.stopbits,
            timeout=self.settings.read_slice_s,
            write_timeout=self.settings.write_timeout_s,
            xonxoff=self.settings.xonxoff,
            rtscts=self.settings.rtscts,
            dsrdtr=self.settings.dsrdtr,
        )
        print(
            f"Opened STM serial {self.settings.serial_port} at "
            f"{self.settings.baud_rate} baud"
        )

    def _require_open(self) -> Any:
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("STM serial port is not open")
        return self._serial

    def _frame(self, wire_command: str) -> bytes:
        if not isinstance(wire_command, str) or not wire_command:
            raise RuntimeError("STM command translation returned an empty value")
        if "\r" in wire_command or "\n" in wire_command:
            raise RuntimeError("STM command translation cannot inject line breaks")
        text = (
            self.settings.command_prefix
            + wire_command
            + self.settings.command_terminator
        )
        try:
            return text.encode(self.settings.encoding)
        except (LookupError, UnicodeEncodeError) as exc:
            raise RuntimeError(f"cannot encode STM command {wire_command!r}: {exc}") from exc

    def _read_response_until(self, deadline: float) -> str:
        serial_port = self._require_open()
        terminator = self.settings.reply_terminator.encode(self.settings.encoding)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("STM response timeout")
            serial_port.timeout = min(self.settings.read_slice_s, remaining)
            raw = serial_port.read_until(
                expected=terminator,
                size=self.settings.max_response_bytes + len(terminator),
            )
            if not raw:
                continue
            if len(raw) > self.settings.max_response_bytes and not raw.endswith(terminator):
                raise RuntimeError("STM response exceeded the configured size limit")
            try:
                response = raw.decode(self.settings.encoding)
            except UnicodeDecodeError as exc:
                raise RuntimeError("STM returned undecodable bytes") from exc
            response = hardware_hooks.normalise_stm_response(response)
            if response:
                print(f"[STM RX] {response}")
                return response

    def _is_error(self, response: str) -> bool:
        return any(response.startswith(prefix) for prefix in self.settings.error_prefixes)

    def execute_and_wait(self, planner_command: str) -> None:
        serial_port = self._require_open()
        translated = hardware_hooks.translate_motion_command(planner_command)
        frame = self._frame(translated)

        # Remove stale startup/noise lines before a new command.  Never clear
        # the input buffer after sending, because that could discard its ACK.
        serial_port.reset_input_buffer()
        written = serial_port.write(frame)
        if written != len(frame):
            raise RuntimeError(
                f"short STM serial write: sent {written} of {len(frame)} bytes"
            )
        serial_port.flush()
        print(f"[STM TX] {translated}")

        if self.settings.ack_required:
            ack_deadline = time.monotonic() + self.settings.ack_timeout_s
            while True:
                response = self._read_response_until(ack_deadline)
                if self._is_error(response):
                    raise RuntimeError(f"STM rejected {planner_command}: {response}")
                if response in self.settings.done_responses:
                    raise RuntimeError("STM sent DONE before the required ACK")
                if response in self.settings.ack_responses:
                    break
                print(f"[STM] ignored unrelated pre-ACK response: {response}")

        done_deadline = time.monotonic() + self.settings.done_timeout_s
        while True:
            response = self._read_response_until(done_deadline)
            if self._is_error(response):
                raise RuntimeError(f"STM failed while running {planner_command}: {response}")
            if response in self.settings.done_responses:
                return
            if response in self.settings.ack_responses:
                print(f"[STM] duplicate ACK while waiting for DONE: {response}")
            else:
                print(f"[STM] ignored unrelated response while waiting for DONE: {response}")

    def emergency_stop(self) -> None:
        if self._stop_sent:
            return
        self._stop_sent = True
        try:
            serial_port = self._require_open()
            frame = self._frame(self.settings.stop_command)
            serial_port.write(frame)
            serial_port.flush()
            print("[STM TX] emergency stop")
        except Exception as exc:
            print(f"WARNING: unable to send STM emergency stop: {exc}", file=sys.stderr)

    def close(self) -> None:
        if self._serial is not None and self._serial.is_open:
            self._serial.close()


def _validate_target_id(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise RuntimeError(
            f"camera returned invalid Target ID {value!r}; expected a positive integer"
        )
    return value


def execute_commands(
    commands: Iterable[str],
    stm: STMExecutor,
    scan_handler: ScanHandler,
    event_sink: EventSink,
) -> None:
    """Execute a prevalidated route, stopping once on any failure."""

    checked = validate_commands(commands)
    try:
        for index, command in enumerate(checked):
            event_sink(
                "command_started",
                {"index": index, "command": command},
            )
            if classify_command(command) == "motor":
                stm.execute_and_wait(command)
            else:
                obstacle_id = scan_obstacle_id(command)
                event_sink(
                    "scan_started",
                    {"index": index, "command": command, "obstacle_id": obstacle_id},
                )
                target_id = scan_handler(obstacle_id)
                if target_id is None:
                    event_sink(
                        "scan_simulated",
                        {"index": index, "obstacle_id": obstacle_id},
                    )
                else:
                    checked_target = _validate_target_id(target_id)
                    event_sink(
                        "target",
                        {
                            "index": index,
                            "obstacle_id": obstacle_id,
                            "target_id": checked_target,
                            "android_message": (
                                f"TARGET, {obstacle_id}, {checked_target}"
                            ),
                        },
                    )
            event_sink(
                "command_done",
                {"index": index, "command": command},
            )
    except Exception:
        stm.emergency_stop()
        raise


def _live_scan_handler(obstacle_id: int) -> int:
    target_id = _validate_target_id(hardware_hooks.recognise_target(obstacle_id))
    hardware_hooks.publish_android_message(
        f"TARGET, {obstacle_id}, {target_id}"
    )
    return target_id


def _dry_scan_handler(obstacle_id: int) -> None:
    print(f"[DRY RUN CAMERA] would recognise obstacle {obstacle_id}")
    return None


def _safe_error(channel: JsonLineConnection, plan_id: str, message: str) -> None:
    try:
        channel.send(public_event("error", plan_id, message=message))
    except Exception:
        pass


def handle_client(
    client: socket.socket,
    address: tuple[str, int],
    *,
    shared_token: str,
    stm: STMExecutor,
    scan_handler: ScanHandler,
    seen_plan_ids: set[str],
) -> bool:
    """Handle exactly one authenticated plan from one PC connection."""

    channel = JsonLineConnection(client)
    plan_id = "unknown"
    try:
        if config.ALLOWED_PC_IPS and address[0] not in config.ALLOWED_PC_IPS:
            raise ProtocolError(f"PC address {address[0]} is not allowed")

        request = channel.receive()
        plan_id, supplied_token, commands = parse_plan_message(request)
        if not hmac.compare_digest(supplied_token, shared_token):
            raise ProtocolError("authentication failed")
        if plan_id in seen_plan_ids:
            raise ProtocolError(f"plan_id {plan_id!r} has already been accepted")

        # Record before movement.  A disconnected/retried client must not cause
        # uncertain physical commands to run twice.
        seen_plan_ids.add(plan_id)
        channel.send(
            public_event("accepted", plan_id, command_count=len(commands))
        )

        def emit(event_type: str, fields: dict[str, Any]) -> None:
            channel.send(public_event(event_type, plan_id, **fields))

        execute_commands(commands, stm, scan_handler, emit)
        channel.send(public_event("plan_complete", plan_id))
        return True
    except Exception as exc:
        stm.emergency_stop()
        print(f"Plan {plan_id} failed: {exc}", file=sys.stderr)
        _safe_error(channel, plan_id, str(exc))
        return False


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="RPi TCP-to-STM Task 1 bridge (dry-run unless --live)"
    )
    parser.add_argument("--live", action="store_true", help="enable real STM movement")
    parser.add_argument(
        "--serve-forever",
        action="store_true",
        help="accept another plan after a successful one",
    )
    parser.add_argument("--listen-host", default=config.RPI_LISTEN_HOST)
    parser.add_argument("--port", type=int, default=config.RPI_PORT)
    parser.add_argument(
        "--token",
        default=config.SHARED_TOKEN,
        help="shared token; editing config.py avoids exposing it in shell history",
    )
    parser.add_argument("--serial-port", default=config.STM_SERIAL_PORT)
    parser.add_argument("--baud", type=int, default=config.STM_BAUD_RATE)
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if not 1 <= args.port <= 65535:
        print("Port must be in 1..65535", file=sys.stderr)
        return 2

    settings = STMSettings(serial_port=args.serial_port, baud_rate=args.baud)
    if args.live:
        try:
            settings.validate_for_live_use(args.token)
            stm: STMExecutor = SerialSTM(settings)
            stm.open()
        except (ValueError, RuntimeError, OSError) as exc:
            print(f"Live bridge refused to start: {exc}", file=sys.stderr)
            return 2
        scan_handler: ScanHandler = _live_scan_handler
        mode = "LIVE - PHYSICAL MOVEMENT ENABLED"
    else:
        stm = DryRunSTM()
        scan_handler = _dry_scan_handler
        mode = "DRY RUN - serial port will not be opened"

    print(mode)
    print(f"Listening for algorithm PC on {args.listen_host}:{args.port}")
    seen_plan_ids: set[str] = set()
    exit_code = 0
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.listen_host, args.port))
            server.listen(1)
            while True:
                client, address = server.accept()
                print(f"PC connected from {address[0]}:{address[1]}")
                with client:
                    client.settimeout(config.SOCKET_READ_TIMEOUT_S)
                    success = handle_client(
                        client,
                        address,
                        shared_token=args.token,
                        stm=stm,
                        scan_handler=scan_handler,
                        seen_plan_ids=seen_plan_ids,
                    )
                if not success:
                    exit_code = 2
                    # After a live failure, physical position is uncertain.
                    # Require a human to resynchronise and restart the bridge.
                    if args.live:
                        break
                if not args.serve_forever:
                    break
    except KeyboardInterrupt:
        print("Bridge interrupted; requesting emergency stop", file=sys.stderr)
        stm.emergency_stop()
        exit_code = 130
    except OSError as exc:
        print(f"Bridge network error: {exc}", file=sys.stderr)
        stm.emergency_stop()
        exit_code = 2
    finally:
        stm.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
