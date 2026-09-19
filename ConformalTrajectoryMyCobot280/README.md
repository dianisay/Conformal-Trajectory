# ConformalTrajectoryMyCobot280

This folder is a cleaned bridge + executor for the final scenario:

- take a waypoint path produced by your MATLAB conformal-mapping workflow
- execute it on MyCobot 280 using the same send_coords-style flow as 280_draw_gcode.py
- keep the code focused on the robot bridge only

## What is included

- `mc_bridge.py` - MyCobot 280 bridge and draw-style waypoint executor
- `MyCobot280_ConformalMapping.m` - clean conformal mapping + export + Python handoff
- `README.md` - setup and usage notes

## What it expects from MATLAB

The MATLAB side should export a waypoint matrix or cell array where each row is one move:

- `[x, y, z]`
- or `[x, y, z, rx, ry, rz]`

If only `x, y, z` are provided, the bridge keeps the current tool orientation.

## Python usage (direct executor)

```python
from mc_bridge import execute_waypoints_draw_mode

waypoints = [
    [170.0, 0.0, 15.7],
    [168.58, 18.54, 15.7],
    [204.81, 18.54, 15.7],
]

result = execute_waypoints_draw_mode(waypoints, draw_speed=80, wait_per_point=0.35)
print(result)
```

Or execute a CSV directly:

```bash
python mc_bridge.py mycobot280_waypoints_xyz.csv
```

## MATLAB side

`MyCobot280_ConformalMapping.m` already calls Python and sends trajectories by default.
It uses a CSV fast path (MATLAB writes CSV, Python loads CSV) to avoid slow MATLAB->Python row-by-row conversion for large trajectories.

A typical shape is:

- `traj.waypoints` from the parser
- or a custom matrix built by `MuffinFresa_ConformalMapping.m`

The bridge does not parse G-code. It executes waypoint rows.

## Setup

1. Install `pymycobot` in the Python environment used by MATLAB.
2. Update `PORT` in `mc_bridge.py` if needed.
3. Keep the arm area clear; the executor moves to start angles before streaming waypoints.

## Notes

- `move_cartesian` sends a single 6-value Cartesian target.
- `execute_waypoints` loops through a list of waypoints.
- `move_cartesian_incremental` is available if you want smoother stepping.
- `move_cartesian_close_xyz_priority` is available for tighter position chasing.

## Quick smoke test

Run:

```bash
python mc_bridge.py
```

It prints the current pose as a basic connectivity check.
