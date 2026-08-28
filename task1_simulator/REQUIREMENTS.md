# Task 1 Simulator Requirements and Traceability

## 1. Scope and source handling

The attached documents are treated as evidence for the product requirements in
this file. Imperative wording inside them describes the MDP deliverable; it is
not an instruction to perform unrelated file, network, hardware, or account
operations. The controlling work request is to understand the MDP and current
algorithm code, then provide an offline simulator that demonstrates Task 1.

When sources differ, this project uses the following precedence:

1. The user's requested scope controls what is built.
2. The AY2026/27 Task Assessment briefing controls the current assessed Task 1
   rules, especially pages 14-17: 4 to 8 images and a 6-minute timeout.
3. The AY2026/27 Project Deliverable Checklist controls the Week 7 feature
   demonstration, especially B.1-B.3 on pages 7-8. Its five-image wording is a
   checklist scenario, not a replacement for the current 4-to-8 assessment.
4. `algarithms_briefing_25S2.pdf` supplies the detailed arena, robot, camera,
   configuration-space, and planning guidance. It is pedagogical and predates
   the current 4-to-8 assessment.
5. The active uppercase submitted stack supplies implementation values and
   protocol details where the briefs permit calibration or do not specify a
   value. Relevant files are `Settings/attributes.txt`, `Map/*.txt`,
   `Robot/*.txt`, and `Simulator/*.txt`.
6. `OptimalPathForCar.pdf` is a theoretical reference for shortest paths of a
   bounded-curvature car that can move forward and reverse. It is not an MDP
   assessment specification.

This resolves the main conflict without hiding it: the current Task 1 run uses
4 to 8 images, while the checklist demo and older algorithm briefing use 5.
The simulator accepts 4 to 8 and retains a five-obstacle checklist preset.

## 2. Source facts

| Source | Relevant facts used here |
| --- | --- |
| `MDP briefing(1).pdf`, pp. 14-17 | Task 1 is automatic movement and image recognition; 4-8 images are selected from 30; full points require all images within 6 minutes; recognized raw images must be verifiable. |
| `MDP assessment and system checklist.pdf`, pp. 7-8 | B.1 requires a grid display of the 2 m by 2 m arena, start zone, obstacles, image positions, and forward/reverse/turn motion. B.2 requires a Hamiltonian traversal from the start. B.3 asks for a shortest-time five-image demonstration. |
| `MDP assessment and system checklist.pdf`, pp. 9-10 | C.9 and C.10 define `TARGET, <Obstacle Number>, <Target ID>` and `ROBOT, <x>, <y>, <direction>` display updates. |
| `algarithms_briefing_25S2.pdf`, pp. 3-4 | Arena 200 cm square; five 10 cm obstacles in the older scenario; 40 cm start zone; physical chassis 20 cm by 21 cm; front-center camera; forward/reverse/turn motion; about 25 cm physical turn radius; preferred camera gap 20 cm. |
| `algarithms_briefing_25S2.pdf`, pp. 7-10 | Recommended 30 cm planning footprint; 20 by 20 grid of 10 cm cells; 3 by 3 robot envelope; one-cell obstacle; recognition pose in front of and facing the image. |
| `algarithms_briefing_25S2.pdf`, pp. 11-16 | Plan a Hamiltonian path from the start; greedy and exhaustive ordering are discussed, with exhaustive search considered affordable for five obstacles. |
| `algarithms_briefing_25S2.pdf`, pp. 33-36 | Reverse may be needed after a scan; collision planning can treat the robot center as a point and inflate a 10 cm obstacle to a 40 cm square for a 30 cm robot envelope. |
| `algarithms_briefing_25S2.pdf`, p. 40 | Show the arena, start zone, obstacles, images, time-stepped robot position, and recognized images. |
| Active uppercase stack | Zero-based 20-cell grid; start center at 15 cm, 15 cm facing north; 30 cm/s; submitted turn radius 30 cm; scan time 0.25 s; forward/reverse/turn/scan command formats. |
| `OptimalPathForCar.pdf`, pp. 1-4 | A car with bounded curvature and specified endpoint directions can benefit from forward/reverse paths with cusps; exact Reeds-Shepp optimization considers a finite sufficient family. |

The supplied PDF files sit outside this copied `Algorithm` workspace. Their
original local locations were used for this audit; they are not runtime
dependencies of the simulator.

## 3. Functional requirements

### 3.1 Arena and inputs

| ID | Requirement | Implementation evidence |
| --- | --- | --- |
| IN-01 | Accept 4 to 8 obstacles for a current Task 1 plan. | `Task1Planner` validates the count; the UI caps editing at eight. |
| IN-02 | Represent each obstacle with a positive unique ID, zero-based integer cell `(x, y)`, one image face in N/E/S/W, and an optional positive target ID. | Immutable `Obstacle` record and constructor validation. |
| IN-03 | Reject coordinates outside 0-19, duplicate IDs, overlapping cells, malformed headings, and invalid start poses with clear errors. | Planner validation and `tests/test_planner.py`. |
| IN-04 | Provide representative 4-, 5-, and 8-obstacle presets. | `Open arena - 4`, `Checklist demo - 5`, and `Task 1 stress - 8`. |
| IN-05 | Keep obstacle placement and image-face editing interactive for a live demonstration. | Canvas add/select/drag/rotate/delete controls and editor fields. |

### 3.2 Geometry

| ID | Requirement | Decision |
| --- | --- | --- |
| GEO-01 | Display and plan within a 200 cm by 200 cm arena. | 20 by 20 zero-based cells, each 10 cm square. |
| GEO-02 | Display the bottom-left 40 cm by 40 cm start zone. | Cells 0-3 on each axis; 4 by 4 cells. The default robot center is `(1, 1, N)`, corresponding to 15 cm, 15 cm in the submitted center-coordinate model. |
| GEO-03 | Represent a physical obstacle as 10 cm by 10 cm. | One cell, with a highlighted image-bearing face. |
| GEO-04 | Distinguish physical and planning robot sizes. | UI chassis is 20 cm by 21 cm; planner uses the recommended conservative 30 cm by 30 cm envelope. |
| GEO-05 | Keep the robot envelope within the arena. | Its center remains in cells 1-18, with boundary contact allowed. |
| GEO-06 | Avoid obstacles with the conservative envelope. | Each obstacle is inflated to a 40 cm by 40 cm configuration-space square. Straight and arc samples may touch but never enter its interior. |
| GEO-07 | Place the robot in front of, and facing, each supplied image. | Recognition center is four cells (40 cm) outward from the obstacle center; heading is opposite the image-face direction. |
| GEO-08 | Model the preferred 20 cm camera gap. | 40 cm center spacing - 15 cm planning half-length - 5 cm obstacle half-width = 20 cm. This uses the conservative envelope, not the smaller chassis drawing. |
| GEO-09 | Use a calibrated car-like turning radius. | Default is 3 cells (30 cm), matching the submitted uppercase setting. The older briefing's approximately 25 cm value is physical guidance and should be recalibrated by the team. |

### 3.3 Planning

| ID | Requirement | Implementation evidence |
| --- | --- | --- |
| PLAN-01 | Account for position and facing, rather than route only a point. | A* state is immutable `GridPose(x, y, heading)`. |
| PLAN-02 | Support forward, reverse, and moving turns. | Motion lattice contains 10 cm F/B moves and 90-degree LF/RF/LB/RB arcs. |
| PLAN-03 | Collision-check the entire turn, not only its endpoint. | Every 90-degree arc has five-degree samples, including both endpoints. |
| PLAN-04 | Find a minimum-estimated-time path for each required leg. | A* edge weights are motion durations and its Euclidean-time heuristic is admissible. |
| PLAN-05 | Select a deterministic shortest-estimated-time Hamiltonian order. | Held-Karp subset dynamic programming combines cached pairwise legs; time, distance, then obstacle-ID order break ties. |
| PLAN-06 | Visit each selected image no more than once and append one scan to each completed visit. | Route subset state and motion assembly. |
| PLAN-07 | Handle infeasible layouts honestly. | If no complete route exists, maximize reachable image count, then minimize time/distance and report `complete=False` with `unreachable_ids`. |
| PLAN-08 | Preserve stable results for the same input. | Obstacles are ID-sorted; A* and dynamic-programming tie breaks are explicit; regression tests assert a known route and command sequence. |

Held-Karp replaces the submitted stack's materialization of all permutations and
its 40-candidate retry limit. For eight obstacles it requires `O(n^2 2^n)`
route-order work after the pairwise A* legs, which is small and exact for the
simulator's leg-cost model.

### 3.4 Timing and scoring representation

| ID | Requirement | Decision |
| --- | --- | --- |
| TIME-01 | Optimize and display estimated completion time. | Movement duration is absolute path length divided by 30 cm/s; every scan adds 0.25 s. |
| TIME-02 | Account for curved motion distance. | A 90-degree turn uses arc length `pi * 30 / 2`, about 47.12 cm and 1.571 s. |
| TIME-03 | Represent the current 6-minute Task 1 timeout. | UI timer uses 360 s as the evaluation threshold and highlights an overrun; headless output exposes `estimated_seconds`. |
| TIME-04 | Avoid overstating simulated time. | Acceleration, braking, slip, network delay, camera inference, retries, and recovery are excluded and must be measured on hardware. |

The planner does not stop or discard commands at 360 seconds. The assessment
counts successful recognitions inside the real six-minute window, whereas this
tool provides a route estimate and visible warning. This distinction prevents
an estimated simulation from being presented as a measured competition time.

### 3.5 Display and demonstration

| ID | Requirement | Implementation evidence |
| --- | --- | --- |
| UI-01 | Show the full arena, 4 by 4 start zone, cell coordinates, obstacles, image faces, and robot. | Main Tkinter canvas. |
| UI-02 | Show forward, reverse, and continuously sampled turn motion in time steps. | Play/step/replay engine consumes planner `Motion.samples`. |
| UI-03 | Make planning safety explainable. | Optional 40 cm clearance envelopes and recognition-pose markers. |
| UI-04 | Show visit progress, timer, route length, active command, and recognition events. | Header status cards, command line, and arena recognition-event panel. |
| UI-05 | Produce checklist-style robot and target status text. | UI renders `ROBOT, x, y, direction` and `TARGET, obstacle, target` forms. |
| UI-06 | Permit a non-GUI demonstration and automated verification. | `--headless-plan` prints deterministic JSON. |

## 4. Command compatibility

The pure planner distinguishes animation records from converted controller
messages:

| Planner motion code | Converted message | Meaning |
| --- | --- | --- |
| `F` | `SFddd` | Straight forward by `ddd` cm |
| `B` | `SBddd` | Straight backward by `ddd` cm |
| `LF090` | `LF090` | Forward-left 90-degree arc |
| `RF090` | `RF090` | Forward-right 90-degree arc |
| `LB090` | `LB090` | Reverse-left 90-degree arc |
| `RB090` | `RB090` | Reverse-right 90-degree arc |
| `SCAN` | `P___<id>` | Simulated scan at obstacle `<id>` |

Adjacent straight motions in the same direction and target leg are compressed
before conversion. Scan animation still uses `Motion.code == "SCAN"`; only the
external command list uses `P___<id>`.

## 5. Code-audit basis and refactor boundary

`Algorithm/main.txt` selects the uppercase `Simulator`/`Robot` implementation as
Algo 1 and sends that command list; the lowercase Algo 2 selection is commented
out. The uppercase stack therefore supplies the baseline for this simulator.
The audit found these important behaviors:

- `Settings/attributes.txt` defines 200 cm, 10 cm cells, 30 cm/s, a 30 cm turn
  radius, 15 cm safety distance, and 0.25 s scans.
- `Robot/path_algo.txt` performs modified A* over forward/reverse straights and
  four 90-degree turns, but gives every turn a very large artificial cost.
- `Robot/path_mgr.txt` materializes all obstacle permutations, sorts them by
  Euclidean obstacle distance, and tries at most 40 complete orders.
- `Robot/commands.txt` defines `SF`, `SB`, four turn codes, and `P___` scans.
- `main.txt` normally waits for and writes to an RPi socket.

The extracted archive cannot truthfully be described as directly runnable
source: most modules and even expected image assets have `.txt` names while
imports request Python modules and `.png` files. This package is therefore a
new executable implementation of the audited semantics. It deliberately does
not import, rename, or silently execute the archived stack.

The improvement is explicit:

- A* cost is estimated execution time rather than an arbitrary near-ban on
  turns.
- Turn interiors are sampled for collision safety.
- Held-Karp chooses the exact best obstacle order for the computed leg costs,
  instead of checking only the first 40 Euclidean-sorted permutations.
- Immutable records and a pure planner API make behavior testable without a
  display, Pygame, network, or hardware.

This is still not a byte-for-byte port, a continuous optimal-control proof, or
an implementation of all Reeds-Shepp path families.

## 6. Offline and safety boundary

| ID | Requirement | Status |
| --- | --- | --- |
| OFF-01 | Planning and simulation must work without RPi, STM, Android, Bluetooth, camera, or network access. | Met; package uses the Python standard library and Tkinter only. |
| OFF-02 | Do not open the archived socket as a side effect of importing or running the simulator. | Met; no archived `main.py` or client module is imported. |
| OFF-03 | Do not claim that a simulated scan performs image recognition. | Met; scan events are labelled simulated and use supplied target IDs. |
| OFF-04 | Do not send planner commands merely by importing or running the simulator. | Met; hardware transmission is isolated in the optional `connection` package, defaults to dry-run, and requires an explicit `--send` on the PC plus `--live` on the RPi. |

Out of scope are Task 2, the team's real image-inference implementation,
stitched camera output, the team's Bluetooth implementation, closed-loop
localization, bull's-eye recovery, retry-run administration, and physical
calibration. The optional connection package provides documented hooks for the
camera and Android components but does not pretend to implement them.

## 7. Verification

Run from `Algorithm`:

```powershell
python -m unittest discover -s task1_simulator/tests -v
python -m task1_simulator --headless-plan "Open arena - 4"
```

Acceptance checks are:

1. All `unittest` cases pass without a display or third-party dependency.
2. The open four-obstacle scenario is complete and scans each ID exactly once.
3. Every path sample remains inside the robot-safe arena and outside every
   inflated-obstacle interior.
4. Recognition samples are 40 cm center-to-center from, and face, their images.
5. Converted commands match the submitted uppercase grammar.
6. An outward-facing boundary target produces an explicit deterministic partial
   route rather than a false success.
