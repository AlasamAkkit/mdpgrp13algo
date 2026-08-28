# Task 1 PC, Raspberry Pi, and STM connection

This directory connects the pure Task 1 planner to the robot without putting
network or serial side effects inside `planner.py` or the simulator GUI.

```text
Algorithm PC -- Wi-Fi/TCP --> Raspberry Pi -- USB serial --> STM32
                                      |
                                      +--> camera and Android Bluetooth hooks
```

The bridge is a safe template, not a claim about your STM firmware. It starts
in dry-run mode and live mode refuses to start until critical `EDIT ME` values
have been changed.

## Files

- `config.py`: all IP, port, serial, framing, ACK/DONE, timeout, and STOP
  settings. Search this file for `EDIT ME` first.
- `hardware_hooks.py`: the three project-specific integrations: STM command
  translation, camera recognition, and Android Bluetooth publishing.
- `protocol.py`: strict command validation and newline-delimited JSON framing.
- `pc_client.py`: plans from obstacle JSON or reads a saved plan, then sends it
  from the algorithm PC to the RPi.
- `rpi_bridge.py`: receives one plan and executes one STM command at a time.
- `example_obstacles.json`: software-only example input.
- `requirements-rpi.txt`: the optional PySerial dependency for live RPi use.

## 1. Configure both machines

Copy the `task1_simulator` directory to the RPi as well as keeping it on the
algorithm PC. Run commands below from the directory containing
`task1_simulator` (the `mdpgrp13algo` repository root).

Edit `config.py` on both copies:

```python
# PC uses the RPi's actual hotspot/Wi-Fi address.
RPI_HOST = "192.168.1.1"       # EDIT ME
RPI_PORT = 6000                # Must match on both machines
```

Set the same private token as an environment variable rather than committing it
to `config.py`:

```powershell
# Algorithm PC
$env:MDP_SHARED_TOKEN = "replace-with-a-long-random-value"
```

```bash
# Raspberry Pi
export MDP_SHARED_TOKEN="replace-with-the-same-long-random-value"
```

On the RPi, locate the STM serial device:

```bash
ls /dev/ttyACM* /dev/ttyUSB* /dev/serial*
```

Then edit the STM block in `config.py`. Obtain these values from the STM
firmware/team; do not guess them:

```python
STM_SERIAL_PORT = "/dev/ttyACM0"   # EDIT ME
STM_BAUD_RATE = 0                   # EDIT ME; zero deliberately blocks live mode
STM_COMMAND_TERMINATOR = "\n"       # EDIT ME
STM_REPLY_TERMINATOR = "\n"         # EDIT ME
STM_ACK_RESPONSES = ("ACK",)        # EDIT ME
STM_DONE_RESPONSES = ("DONE",)      # EDIT ME
STM_STOP_COMMAND = "..."            # EDIT ME; required for live mode
```

If the STM does not accept `SF030`, `RF090`, etc. directly, edit only
`translate_motion_command()` in `hardware_hooks.py`. Do not change the planner
or the command validator to accommodate an unverified firmware protocol.

## 2. Test PC-to-RPi communication without the STM

Start the bridge on the RPi. This is dry-run and does not import PySerial or
open the STM port:

```bash
python3 -m task1_simulator.connection.rpi_bridge
```

On the PC, preview a route without opening a socket:

```powershell
python -m task1_simulator.connection.pc_client `
  --obstacles task1_simulator/connection/example_obstacles.json
```

Then send it to the dry-run RPi bridge:

```powershell
python -m task1_simulator.connection.pc_client `
  --obstacles task1_simulator/connection/example_obstacles.json `
  --send
```

The RPi should print simulated `ACK` and `DONE` messages in exactly the planned
order. Scan commands should appear as camera dry-runs and never as STM writes.

You may also send JSON produced by the existing headless planner:

```powershell
python -m task1_simulator --headless-plan "Open arena - 4" > plan.json
python -m task1_simulator.connection.pc_client --plan plan.json --send
```

## 3. Install live serial support on the RPi

```bash
python3 -m pip install -r task1_simulator/connection/requirements-rpi.txt
```

If Linux denies access to the serial device, inspect its group with
`ls -l /dev/ttyACM0` and add the RPi user to that device's group according to
your lab setup. Log out and back in after changing group membership.

## 4. Complete the camera and Android hooks

Edit `hardware_hooks.py`:

- `recognise_target(obstacle_id)` must run the real RPi camera/inference code
  and return a positive Target ID.
- `publish_android_message(message)` must use your Bluetooth/RFCOMM sender.
- `normalise_stm_response(response)` should only be changed when the firmware
  replies contain prefixes, command echoes, or sequence numbers.

The bridge converts a real recognition result into:

```text
TARGET, <Obstacle Number>, <Target ID>
```

The simulator's `target_id` field is demo data and is deliberately not used by
the live camera hook.

## 5. Enable physical movement

Live mode is explicit. Set `MDP_SHARED_TOKEN` in the same terminal first:

```bash
python3 -m task1_simulator.connection.rpi_bridge --live
```

It will refuse to start if the shared token, baud rate, or STOP command is
still a placeholder. Once it is listening, run the PC command with `--send`.

For every motor instruction, the bridge:

1. validates the entire route before movement;
2. writes exactly one framed command;
3. waits for `ACK` when configured;
4. waits for `DONE`, which must mean movement physically finished;
5. sends the next command only after `DONE`;
6. aborts and attempts STOP on timeout, STM error, camera failure, or PC loss.

It never automatically retries a timed-out movement. A missing `DONE` leaves
the physical position uncertain, so repeating the command could double-move
the car. Resynchronise the robot manually and restart the bridge.

## Physical test order

Test with the wheels lifted first, then in a clear area:

```text
SF010
SB010
LF090
RF090
LB090
RB090
```

Confirm distance units, left/right direction, 90-degree heading, turn radius,
ACK versus DONE behavior, emergency stop, and the STM motor watchdog before
running a complete arena plan. The planner defaults to a 30 cm turn radius, so
update/calibrate the motion model if the measured car differs.
