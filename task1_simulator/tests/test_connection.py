"""Hardware-free tests for TCP framing and sequential STM execution."""

from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


ALGORITHM_DIR = Path(__file__).resolve().parents[2]
if str(ALGORITHM_DIR) not in sys.path:
    sys.path.insert(0, str(ALGORITHM_DIR))

from task1_simulator.connection.pc_client import message_from_result  # noqa: E402
from task1_simulator.connection.protocol import (  # noqa: E402
    JsonLineConnection,
    ProtocolError,
    build_plan_message,
    classify_command,
    parse_plan_message,
    validate_commands,
)
from task1_simulator.connection.rpi_bridge import (  # noqa: E402
    DryRunSTM,
    STMSettings,
    SerialSTM,
    execute_commands,
)


class FakeSocket:
    def __init__(self, chunks=()):
        self.chunks = list(chunks)
        self.sent: list[bytes] = []

    def recv(self, _size):
        return self.chunks.pop(0) if self.chunks else b""

    def sendall(self, data):
        self.sent.append(data)


class FakeSerial:
    def __init__(self, responses):
        self.responses = list(responses)
        self.writes: list[bytes] = []
        self.timeout = None
        self.is_open = True
        self.reset_count = 0

    def reset_input_buffer(self):
        self.reset_count += 1

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def flush(self):
        return

    def read_until(self, expected, size):
        del expected, size
        return self.responses.pop(0) if self.responses else b""

    def close(self):
        self.is_open = False


class FakeSTM:
    def __init__(self, fail_on=None):
        self.commands: list[str] = []
        self.fail_on = fail_on
        self.stop_count = 0

    def execute_and_wait(self, command):
        self.commands.append(command)
        if command == self.fail_on:
            raise TimeoutError("simulated lost DONE")

    def emergency_stop(self):
        if not self.stop_count:
            self.stop_count += 1

    def close(self):
        return


class ProtocolTests(unittest.TestCase):
    def test_json_decoder_handles_fragmented_and_coalesced_frames(self):
        fake = FakeSocket(
            [b'{"type":"one"}\n{"ty', b'pe":"two","value":2}\n']
        )
        channel = JsonLineConnection(fake)
        self.assertEqual({"type": "one"}, channel.receive())
        self.assertEqual({"type": "two", "value": 2}, channel.receive())

    def test_json_send_uses_one_newline_terminated_sendall(self):
        fake = FakeSocket()
        JsonLineConnection(fake).send({"type": "test", "value": 1})
        self.assertEqual([b'{"type":"test","value":1}\n'], fake.sent)

    def test_command_allow_list(self):
        accepted = (
            "SF001",
            "SF200",
            "SB010",
            "LF090",
            "RF090",
            "LB090",
            "RB090",
            "P___1",
            "P___12",
        )
        self.assertEqual(accepted, validate_commands(accepted))
        self.assertEqual("motor", classify_command("SF010"))
        self.assertEqual("scan", classify_command("P___4"))

        rejected = (
            "sf010",
            "SF10",
            "SF000",
            "SF201",
            "SF010\nSTOP",
            "LF091",
            "P___0",
            "STOP",
            "",
        )
        for command in rejected:
            with self.subTest(command=command):
                with self.assertRaises(ProtocolError):
                    validate_commands([command])

        with self.assertRaisesRegex(ProtocolError, "scanned more than once"):
            validate_commands(["P___1", "SF010", "P___1"])

    def test_plan_message_round_trip_validation(self):
        message = build_plan_message(
            ["SF010", "P___1"], "secret", plan_id="run-1"
        )
        plan_id, token, commands = parse_plan_message(message)
        self.assertEqual("run-1", plan_id)
        self.assertEqual("secret", token)
        self.assertEqual(("SF010", "P___1"), commands)

    def test_pc_refuses_incomplete_plan(self):
        incomplete = SimpleNamespace(
            complete=False,
            unreachable_ids=(4,),
            command_messages=("SF010",),
        )
        with self.assertRaisesRegex(ValueError, "incomplete plan"):
            message_from_result(incomplete, "secret")


class SequentialExecutionTests(unittest.TestCase):
    def test_scan_routes_to_camera_and_never_to_stm(self):
        stm = FakeSTM()
        camera_calls: list[int] = []
        events: list[tuple[str, dict]] = []

        def scan(obstacle_id):
            camera_calls.append(obstacle_id)
            return 11

        execute_commands(
            ["SF010", "P___3", "RF090"],
            stm,
            scan,
            lambda event, fields: events.append((event, fields)),
        )

        self.assertEqual(["SF010", "RF090"], stm.commands)
        self.assertEqual([3], camera_calls)
        target_events = [fields for event, fields in events if event == "target"]
        self.assertEqual(
            [
                {
                    "index": 1,
                    "obstacle_id": 3,
                    "target_id": 11,
                    "android_message": "TARGET, 3, 11",
                }
            ],
            target_events,
        )

    def test_failure_stops_once_and_never_runs_later_command(self):
        stm = FakeSTM(fail_on="RF090")
        with self.assertRaisesRegex(TimeoutError, "lost DONE"):
            execute_commands(
                ["SF010", "RF090", "SB010"],
                stm,
                lambda _obstacle: None,
                lambda _event, _fields: None,
            )
        self.assertEqual(["SF010", "RF090"], stm.commands)
        self.assertEqual(1, stm.stop_count)

    def test_serial_sends_one_frame_and_waits_for_ack_then_done(self):
        settings = STMSettings(
            baud_rate=115200,
            stop_command="STOP",
            ack_timeout_s=1.0,
            done_timeout_s=1.0,
        )
        stm = SerialSTM(settings)
        fake = FakeSerial([b"ACK\n", b"DONE\n"])
        stm._serial = fake
        stm.execute_and_wait("SF030")
        self.assertEqual([b"SF030\n"], fake.writes)
        self.assertEqual(1, fake.reset_count)

    def test_done_before_ack_aborts_and_sends_stop_once(self):
        settings = STMSettings(
            baud_rate=115200,
            stop_command="STOP",
            ack_timeout_s=1.0,
            done_timeout_s=1.0,
        )
        stm = SerialSTM(settings)
        fake = FakeSerial([b"DONE\n"])
        stm._serial = fake

        with self.assertRaisesRegex(RuntimeError, "DONE before"):
            execute_commands(
                ["SF010", "SB010"],
                stm,
                lambda _obstacle: None,
                lambda _event, _fields: None,
            )
        stm.emergency_stop()
        self.assertEqual([b"SF010\n", b"STOP\n"], fake.writes)

    def test_dry_run_and_live_placeholder_guard(self):
        dry = DryRunSTM(delay_s=0)
        execute_commands(
            ["SF010", "P___1"],
            dry,
            lambda _obstacle: None,
            lambda _event, _fields: None,
        )
        self.assertEqual(["SF010"], dry.commands)

        with self.assertRaisesRegex(ValueError, "live mode refused"):
            STMSettings(baud_rate=0).validate_for_live_use(
                "EDIT_ME_CHANGE_THIS_SHARED_TOKEN"
            )


if __name__ == "__main__":
    unittest.main()
