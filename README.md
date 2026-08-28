# MDP Group 13 Algorithm

Task 1 path planning and simulation for the NTU SC2079 Multi-disciplinary
Design Project. The runnable implementation provides a 20 by 20 arena editor,
collision-aware car-like A* planning, Hamiltonian visit-order optimisation,
command playback, and an optional Raspberry Pi/STM connection layer.

## Run the Task 1 simulator

Python 3.10 or newer is required. From this repository root:

```powershell
python -m task1_simulator
```

Run the complete test suite:

```powershell
python -m unittest discover -s task1_simulator/tests -v
```

The simulator and planner documentation is in
[`task1_simulator/README.md`](task1_simulator/README.md).

## Raspberry Pi and STM integration

The optional connection layer uses framed TCP between the algorithm PC and RPi,
then executes one validated STM serial command at a time. It defaults to dry-run
and cannot enable live movement while hardware settings remain placeholders.

See [`task1_simulator/connection/README.md`](task1_simulator/connection/README.md)
before connecting the physical car.

## Legacy project files

The uppercase/lowercase `.txt` modules and disguised image assets are preserved
as submitted reference material. The executable and tested implementation is
under `task1_simulator/`; see its requirements document for the audit boundary
and compatibility decisions.
