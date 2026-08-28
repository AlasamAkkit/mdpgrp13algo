# MDP Task 1 Algorithm Simulator

This package is an offline, dependency-free simulator for demonstrating the
SC2079 MDP Task 1 algorithm. It provides an editable 20 by 20 arena, plans a
car-like route through the image-facing side of every reachable obstacle,
shows the command-derived motion step by step, and reports simulated image
recognition events.

This is a clean refactor of the submitted algorithm, not a claim that the
archived `.txt` files are being executed byte for byte. The extracted uppercase
stack (`Map`, `Robot`, `Settings`, and `Simulator`) contains the relevant Task 1
implementation, but its Python modules and image assets are stored as `.txt`,
while `main.txt` imports `.py` modules and opens an RPi socket. The new package
preserves its coordinate system, tuned motion values, and command vocabulary in
a runnable, testable, offline implementation. It improves route selection by
using car-state A* for individual legs and Held-Karp dynamic programming for the
visit order.

See [REQUIREMENTS.md](REQUIREMENTS.md) for source precedence, requirement
traceability, geometry decisions, and known boundaries.

## Run the simulator

Python 3.10 or newer is required. Tkinter is included with the normal Windows
Python distribution; some Linux distributions package it separately.

From this repository:

```powershell
cd mdpgrp13algo
python -m task1_simulator
```

Choose a preset or edit the arena directly:

1. Left-click an empty cell to add an obstacle. Task 1 accepts 4 to 8.
2. Click or drag an obstacle to select or move it.
3. Right-click it to rotate the image face; use the editor for its face and
   target ID. Delete or Backspace removes the selected obstacle.
4. Select **Calculate shortest-time route**.
5. Use Play, Step, Reset, and the playback-speed control to demonstrate the
   command stream. The Clearance and Recognition poses switches explain the
   planner geometry.

Space toggles playback and Ctrl+R recalculates the route.

## Headless planning

The same planner can produce deterministic JSON without opening a window:

```powershell
cd mdpgrp13algo
python -m task1_simulator --headless-plan "Checklist demo - 5"
```

The other bundled choices are `Open arena - 4` and `Task 1 stress - 8`.
The JSON reports completion, visit order, unreachable obstacles, estimated
seconds, distance, expanded A* states, and the converted command list.

## Run the tests

Only the Python standard library and `unittest` are used:

```powershell
cd mdpgrp13algo
python -m unittest discover -s task1_simulator/tests -v
```

The regression suite checks the planning geometry, exact protocol conversion,
a deterministic complete route, partial-route behavior, and input validation.

## Optional Raspberry Pi and STM connection

The simulator remains offline by default. A separate, fail-closed connection
template is available in [connection/README.md](connection/README.md). It adds a
validated PC-to-RPi JSON-lines client and a sequential RPi-to-STM USB-serial
bridge without importing PySerial during normal simulator use.

The bridge defaults to dry-run, sends only one motor command at a time, waits
for configurable `ACK` and `DONE` responses, routes `P___<id>` to an RPi camera
hook, and attempts the configured emergency-stop command on failure. Search
`connection/config.py` and `connection/hardware_hooks.py` for `EDIT ME` before
using `--live`.

## Requirement interpretation

The current AY2026/27 Task 1 briefing says the assessed arena contains a subset
of **4 to 8 images** and allows **6 minutes**. The Week 7 checklist and the older
algorithm briefing both use a five-obstacle demonstration. The simulator
therefore validates 4 to 8 obstacles and includes a five-obstacle preset for the
checklist demonstration; it does not incorrectly restrict the current Task 1
run to five.

The model uses these deliberate geometry choices:

| Item | Simulator model |
| --- | --- |
| Arena | 200 cm by 200 cm; 20 by 20 cells of 10 cm |
| Start zone | Bottom-left 40 cm by 40 cm; 4 by 4 cells |
| Physical drawing | 20 cm by 21 cm chassis with a front-center camera |
| Planning envelope | Conservative 30 cm by 30 cm square |
| Obstacle | 10 cm by 10 cm; one grid cell |
| Collision envelope | 40 cm by 40 cm configuration-space square per obstacle |
| Recognition pose | Robot center 40 cm from the obstacle center, facing its image |
| Turn radius | 30 cm default from the submitted uppercase settings |

The 40 cm center stand-off leaves the requested 20 cm camera gap in the
conservative planning model: 40 cm minus the robot envelope's 15 cm half-length
minus the obstacle's 5 cm half-width equals 20 cm. The UI draws the smaller
20 cm by 21 cm physical chassis inside that conservative envelope so the two
figures are not confused.

The algorithm briefing describes an approximately 25 cm physical turning
radius that grows with speed and must be measured. The submitted uppercase code
uses 30 cm, so the simulator defaults to the submitted value. Calibrate the
physical robot before treating simulator timing or clearance as deployment
values.

## Planner and timing model

Each A* state is `(x, y, heading)`. Available transitions are a 10 cm forward or
reverse step and a 90-degree forward/reverse left/right arc. Straight and arc
samples are collision-checked against the arena and inflated obstacles. A*
minimizes estimated movement time for every pair of recognition poses.

Held-Karp dynamic programming then selects the minimum-estimated-time visit
order. If no route can cover all inputs, it first maximizes the number of
visited images and then chooses the fastest deterministic route among those
ties. This makes unreachable geometry visible instead of crashing or pretending
the run is complete.

Timing follows the active uppercase settings:

- Movement speed: 30 cm/s for straight segments and arc length.
- Turn: a 90-degree, radius-30-cm arc, approximately 47.12 cm or 1.571 s.
- Scan: 0.25 s per obstacle.
- Total distance: sum of absolute forward, reverse, and arc distances.

These are planning estimates. Acceleration, braking, wheel slip, camera latency,
communications, and recognition retries are outside this simulator.

## Command vocabulary

The animation uses readable motion records while the plan exposes the submitted
uppercase command protocol:

| Motion | Command message |
| --- | --- |
| Forward | `SFddd`, distance in cm, zero-padded |
| Reverse | `SBddd`, distance in cm, zero-padded |
| Forward left/right | `LF090`, `RF090` |
| Reverse left/right | `LB090`, `RB090` |
| Simulated scan | `P___<obstacle_id>` |

The arena event panel also presents checklist-style `ROBOT, x, y, direction` updates and
`TARGET, obstacle, target` recognition results.

## Offline boundary and non-goals

Running `task1_simulator` does not import or start the archived RPi client, open
a socket, connect to Bluetooth/STM hardware, use a camera, or perform real image
recognition. A `SCAN` step is a deterministic demonstration event for an image
position that was supplied to the planner. The package also does not implement
the Task 2 fastest-car course, localization correction, bull's-eye recovery, or
the complete 68-family Reeds-Shepp solver. The Reeds-Shepp paper supports the
forward/reverse car model; this implementation uses a discrete, collision-aware
motion lattice suitable for the simulator.
