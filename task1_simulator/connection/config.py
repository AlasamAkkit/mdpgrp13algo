"""Connection settings shared by the algorithm PC and Raspberry Pi.

Search this file for ``EDIT ME`` before using the real robot.  The defaults are
safe for software-only testing: the bridge starts in dry-run mode, and live
mode refuses to start while critical settings are still placeholders.
"""

import os

# ---------------------------------------------------------------- PC <-> RPi

# EDIT ME: Set this to the RPi Wi-Fi/hotspot address shown by ``hostname -I``.
# The submitted project used 192.168.1.1, so that remains the example default.
RPI_HOST = "192.168.1.1"

# EDIT ME only if your RPi server uses another port.  This value must be the
# same on the PC client and RPi bridge.
RPI_PORT = 6000

# The RPi normally listens on all of its own interfaces.  Do not use 0.0.0.0 as
# the address passed to the PC client; the PC must use RPI_HOST above.
RPI_LISTEN_HOST = "0.0.0.0"

# EDIT ME through the MDP_SHARED_TOKEN environment variable on both machines.
# Keeping the real token outside this tracked file prevents accidental GitHub
# publication. Live mode deliberately refuses the placeholder value.
SHARED_TOKEN = os.environ.get(
    "MDP_SHARED_TOKEN", "EDIT_ME_CHANGE_THIS_SHARED_TOKEN"
)

SOCKET_CONNECT_TIMEOUT_S = 5.0
SOCKET_READ_TIMEOUT_S = 30.0
MAX_TCP_FRAME_BYTES = 1_000_000
MAX_PLAN_COMMANDS = 256

# A compressed straight command may not exceed this many centimetres.  The
# 200 cm Task 1 arena normally keeps every value at or below this limit.
MAX_STRAIGHT_CM = 200

# Optional allow-list for PC addresses.  Leave empty while setting up, or EDIT
# ME to something like ("192.168.1.2",).  The shared token is still checked.
ALLOWED_PC_IPS: tuple[str, ...] = ()

# ----------------------------------------------------------------- RPi <-> STM

# EDIT ME after running:
#   ls /dev/ttyACM* /dev/ttyUSB* /dev/serial*
STM_SERIAL_PORT = "/dev/ttyACM0"

# EDIT ME: this is intentionally invalid.  Replace it with the baud rate from
# your STM firmware, for example 115200 only if the STM team confirms it.
STM_BAUD_RATE = 0

# EDIT ME if the firmware is not using the common 8-N-1 configuration.
STM_BYTESIZE = 8
STM_PARITY = "N"
STM_STOPBITS = 1
STM_XONXOFF = False
STM_RTSCTS = False
STM_DSRDTR = False

STM_WRITE_TIMEOUT_S = 1.0
STM_READ_SLICE_S = 0.20

# EDIT ME to match the firmware timing.  ACK means "received"; DONE must mean
# that physical movement has actually finished.
STM_ACK_REQUIRED = True
STM_ACK_TIMEOUT_S = 2.0
STM_DONE_TIMEOUT_S = 15.0

# EDIT ME: firmware framing.  These are text examples, not confirmed STM
# values.  Escape sequences such as "\n" and "\r\n" are allowed here.
STM_COMMAND_PREFIX = ""
STM_COMMAND_TERMINATOR = "\n"
STM_REPLY_TERMINATOR = "\n"
STM_ENCODING = "ascii"
STM_MAX_RESPONSE_BYTES = 256

# EDIT ME: exact response strings returned by the STM firmware.
STM_ACK_RESPONSES = ("ACK",)
STM_DONE_RESPONSES = ("DONE",)
STM_ERROR_PREFIXES = ("ERR", "ERROR", "NACK")

# EDIT ME: obtain the real emergency-stop command from the STM team.  Live mode
# refuses to start with this placeholder.  The STM firmware should also have a
# watchdog; a Python STOP message cannot be the only safety mechanism.
STM_STOP_COMMAND = "EDIT_ME_STM_STOP_COMMAND"

# Dry-run timing is deliberately short and does not open a serial device.
DRY_RUN_COMMAND_DELAY_S = 0.01
