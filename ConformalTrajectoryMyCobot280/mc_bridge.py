# ============================================================
# mc_bridge.py - MyCobot 280 bridge for conformal trajectories
# - Single robot connection
# - Safe pose/telemetry helpers
# - Cartesian moves
# - Execute waypoint lists from MATLAB or Python
# ============================================================

import math
import time
import csv
import os
import sys

PORT = "COM3"
BAUD = 115200
SPEED_DEFAULT = 50
MOVE_MODE = 1
TIMEOUT = 20.0
DRAW_SPEED_DEFAULT = 80
WAIT_PER_POINT_DEFAULT = 0.35
START_ANGLES_DEFAULT = [0, -40, -130, 80, 0, 50]

_mc = None


def _get_mc():
    global _mc
    if _mc is None:
        try:
            from pymycobot.mycobot import MyCobot
            cls = MyCobot
        except Exception:
            from pymycobot import MyCobot280
            cls = MyCobot280
        _mc = cls(PORT, BAUD)
        time.sleep(0.8)
    return _mc


def _ensure_power(mc):
    try:
        if not mc.is_power_on():
            mc.power_on()
            time.sleep(0.8)
    except Exception:
        mc.power_on()
        time.sleep(0.8)


def set_move_mode(mode: int):
    global MOVE_MODE
    value = int(mode)
    if value not in (0, 1):
        raise ValueError("mode must be 0 or 1")
    MOVE_MODE = value
    return MOVE_MODE


def _to6_list(value):
    value = list(value) if isinstance(value, (list, tuple)) else []
    value = (value + [None] * 6)[:6]
    result = []
    for item in value:
        try:
            result.append(float(item) if item is not None else math.nan)
        except Exception:
            result.append(math.nan)
    return result


def _get_coords_safe(retries=5, delay=0.05):
    mc = _get_mc()
    for _ in range(max(1, int(retries))):
        try:
            coords = mc.get_coords()
            if coords and len(coords) >= 6:
                return _to6_list(coords)
        except Exception:
            pass
        time.sleep(max(0.0, float(delay)))
    try:
        return _to6_list(mc.get_coords() or [])
    except Exception:
        return [math.nan] * 6


def _get_angles_safe(retries=5, delay=0.05):
    mc = _get_mc()
    for _ in range(max(1, int(retries))):
        try:
            angles = mc.get_angles()
            if angles and len(angles) >= 6:
                return _to6_list(angles)
        except Exception:
            pass
        time.sleep(max(0.0, float(delay)))
    try:
        return _to6_list(mc.get_angles() or [])
    except Exception:
        return [math.nan] * 6


def _finite(value, default):
    try:
        numeric = float(value)
        return numeric if math.isfinite(numeric) else float(default)
    except Exception:
        return float(default)


def get_pose(retries: int = 5, delay: float = 0.05):
    mc = _get_mc()
    _ensure_power(mc)
    return {
        "coords": _get_coords_safe(retries, delay),
        "q_deg": _get_angles_safe(retries, delay),
    }


def move_joints(q1, q2, q3, q4, q5, q6, speed=SPEED_DEFAULT, wait=True, timeout_s=TIMEOUT):
    mc = _get_mc()
    _ensure_power(mc)
    target = [float(q1), float(q2), float(q3), float(q4), float(q5), float(q6)]
    mc.send_angles(target, int(speed))
    if not wait:
        return {"ok": True, "final_q": target, "reached": None, "message": "Enviado"}

    start = time.time()
    last = None
    while time.time() - start < float(timeout_s):
        last = _get_angles_safe(1, 0.02)
        if all(abs(a - b) <= 2.0 for a, b in zip(last, target) if math.isfinite(a)):
            return {"ok": True, "final_q": last, "reached": True, "message": "OK"}
        time.sleep(0.08)
    return {"ok": bool(last), "final_q": last, "reached": False, "message": "Timeout"}


def home(q_deg):
    if len(q_deg) != 6:
        raise ValueError("Expected 6 joint values")
    return move_joints(*q_deg, speed=SPEED_DEFAULT, wait=True, timeout_s=TIMEOUT)


def move_cartesian(x, y, z, rx, ry, rz, speed=SPEED_DEFAULT, wait=True, timeout_s=TIMEOUT):
    mc = _get_mc()
    _ensure_power(mc)
    target = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
    try:
        if hasattr(mc, "set_fresh_mode"):
            mc.set_fresh_mode(1)
    except Exception:
        pass
    mc.send_coords(target, int(speed), int(MOVE_MODE))
    if not wait:
        return {"ok": True, "final_coords": target, "reached": None, "message": "Enviado"}

    start = time.time()
    last = None
    while time.time() - start < float(timeout_s):
        last = _get_coords_safe(1, 0.02)
        pos_ok = all(abs(a - b) <= 3.0 for a, b in zip(last[:3], target[:3]) if math.isfinite(a))
        ang_ok = all(abs(a - b) <= 3.0 for a, b in zip(last[3:], target[3:]) if math.isfinite(a))
        if pos_ok and ang_ok:
            return {"ok": True, "final_coords": last, "reached": True, "message": "OK"}
        time.sleep(0.08)
    return {"ok": bool(last), "final_coords": last if last else target, "reached": False, "message": "Timeout"}


def _waypoint_rows(waypoints):
    if waypoints is None:
        return
    try:
        iterator = iter(waypoints)
    except TypeError:
        return
    for row in iterator:
        yield row


def _normalize_waypoint(row, fallback_pose):
    try:
        values = list(row)
    except TypeError:
        values = [row]

    values = values[:6]
    if len(values) < 3:
        raise ValueError("Each waypoint needs at least X, Y and Z")

    x = float(values[0])
    y = float(values[1])
    z = float(values[2])

    if len(values) >= 6:
        rx = float(values[3])
        ry = float(values[4])
        rz = float(values[5])
    else:
        rx = float(fallback_pose[3])
        ry = float(fallback_pose[4])
        rz = float(fallback_pose[5])

    return [x, y, z, rx, ry, rz]


def transform_waypoints_for_draw(rows, start_pose, coordinate_mode="local"):
        """Transform raw waypoint rows into safe robot targets for draw mode.

        local:
            X/Y are offsets from the measured start pose.
            Z is referenced so the highest input Z stays at the measured start Z,
            and lower values move downward from there.

        absolute:
            Input rows are treated as final robot coordinates.
        """
        mode = str(coordinate_mode).lower().strip()
        normalized = [_normalize_waypoint(row, start_pose) for row in rows]
        if mode == "absolute":
                return normalized

        z_top = max(r[2] for r in normalized)
        out = []
        for x, y, z, rx, ry, rz in normalized:
                out.append([
                        float(start_pose[0]) + float(x),
                        float(start_pose[1]) + float(y),
                        float(start_pose[2]) + float(z) - float(z_top),
                        float(start_pose[3]),
                        float(start_pose[4]),
                        float(start_pose[5]),
                ])
        return out


def execute_waypoints(waypoints, speed=SPEED_DEFAULT, wait=True, timeout_s=TIMEOUT, settle_s=0.0, start_pose=None):
    """Execute a sequence of waypoints generated in MATLAB or Python."""
    mc = _get_mc()
    _ensure_power(mc)

    current_pose = _to6_list(start_pose) if start_pose is not None else _get_coords_safe(3, 0.03)
    if len(current_pose) < 6:
        current_pose = (current_pose + [0.0] * 6)[:6]

    normalized = []
    for row in _waypoint_rows(waypoints):
        normalized.append(_normalize_waypoint(row, current_pose))

    if not normalized:
        return {"ok": False, "count": 0, "message": "No waypoints provided", "final_coords": current_pose}

    last_result = None
    for target in normalized:
        last_result = move_cartesian(*target, speed=speed, wait=wait, timeout_s=timeout_s)
        if settle_s and settle_s > 0:
            time.sleep(float(settle_s))

        if last_result and isinstance(last_result, dict) and last_result.get("final_coords"):
            current_pose = _to6_list(last_result["final_coords"])
        else:
            current_pose = _to6_list(target)

    return {
        "ok": bool(last_result) and bool(last_result.get("ok", False)),
        "count": len(normalized),
        "message": "OK" if last_result and last_result.get("ok", False) else "Completed with warnings",
        "final_coords": current_pose,
        "last_result": last_result,
    }


def init_draw_session(start_angles=None, start_speed=50, settle_s=3.0):
    """Initialize the robot the same way as the classic 280_draw_gcode executor."""
    mc = _get_mc()
    _ensure_power(mc)

    angles = list(start_angles) if start_angles is not None else list(START_ANGLES_DEFAULT)
    if len(angles) != 6:
        raise ValueError("start_angles must contain 6 values")

    try:
        if hasattr(mc, "set_fresh_mode"):
            mc.set_fresh_mode(0)
    except Exception:
        pass

    mc.send_angles([float(a) for a in angles], int(start_speed))
    time.sleep(float(settle_s))

    pose = _get_coords_safe(8, 0.08)
    return {
        "ok": True,
        "start_angles": angles,
        "start_pose": pose,
    }


def execute_waypoints_draw_mode(waypoints, draw_speed=DRAW_SPEED_DEFAULT, wait_per_point=WAIT_PER_POINT_DEFAULT, start_angles=None, start_speed=50, settle_s=3.0, z_min=None, z_max=None, coordinate_mode="local"):
    """Draw-style executor matching 280_draw_gcode.py behavior.

    - set_fresh_mode(0)
    - move to start joint pose
    - keep orientation fixed from start pose when rows are XYZ only
    - send each target via send_coords(..., mode=1) with constant dwell
    """
    session = init_draw_session(start_angles=start_angles, start_speed=start_speed, settle_s=settle_s)
    start_pose = _to6_list(session.get("start_pose", []))
    if len(start_pose) < 6:
        start_pose = [0.0, 0.0, 0.0, 180.0, 0.0, 0.0]

    rows = list(_waypoint_rows(waypoints))

    if not rows:
        return {"ok": False, "count": 0, "message": "No waypoints provided", "final_coords": start_pose}

    targets = transform_waypoints_for_draw(rows, start_pose, coordinate_mode=coordinate_mode)

    mc = _get_mc()
    sent = 0
    for target in targets:
        if z_min is not None:
            target[2] = max(float(z_min), float(target[2]))
        if z_max is not None:
            target[2] = min(float(z_max), float(target[2]))
        mc.send_coords(target, int(draw_speed), 1)
        sent += 1
        if wait_per_point and wait_per_point > 0:
            time.sleep(float(wait_per_point))

    final_pose = _get_coords_safe(8, 0.08)
    return {
        "ok": True,
        "count": sent,
        "message": "OK",
        "start_pose": start_pose,
        "final_coords": final_pose,
    }


def load_waypoints_csv(csv_path):
    """Load waypoint rows [x,y,z] or [x,y,z,rx,ry,rz] from CSV."""
    rows = []
    with open(csv_path, "r", newline="") as handle:
        reader = csv.reader(handle)
        for raw in reader:
            if not raw:
                continue
            try:
                row = [float(x) for x in raw if str(x).strip() != ""]
            except Exception:
                continue
            if len(row) >= 3:
                rows.append(row[:6])
    return rows


def summarize_waypoints(rows):
    """Return min/max/span summary for XYZ waypoint rows."""
    if not rows:
        return {"count": 0}

    xs = [float(r[0]) for r in rows]
    ys = [float(r[1]) for r in rows]
    zs = [float(r[2]) for r in rows]
    return {
        "count": len(rows),
        "x_min": min(xs), "x_max": max(xs), "x_span": max(xs) - min(xs), "x_mid": 0.5 * (min(xs) + max(xs)),
        "y_min": min(ys), "y_max": max(ys), "y_span": max(ys) - min(ys), "y_mid": 0.5 * (min(ys) + max(ys)),
        "z_min": min(zs), "z_max": max(zs), "z_span": max(zs) - min(zs), "z_mid": 0.5 * (min(zs) + max(zs)),
    }


def detect_suspicious_waypoints(summary, z_min=None, z_max=None):
    """Detect obviously unsafe/local-frame trajectories before robot motion."""
    reasons = []
    if not summary or summary.get("count", 0) == 0:
        reasons.append("No waypoint rows found")
        return reasons

    if summary["z_span"] < 1e-6:
        reasons.append(f"Z is flat at {summary['z_min']:.3f} mm")

    if z_min is not None and abs(summary["z_min"] - float(z_min)) < 1e-6 and abs(summary["z_max"] - float(z_min)) < 1e-6:
        reasons.append(f"All Z values are pinned at z_min={float(z_min):.3f} mm")

    if z_max is not None and abs(summary["z_min"] - float(z_max)) < 1e-6 and abs(summary["z_max"] - float(z_max)) < 1e-6:
        reasons.append(f"All Z values are pinned at z_max={float(z_max):.3f} mm")

    if abs(summary["x_mid"]) < 60.0 and abs(summary["y_mid"]) < 60.0:
        reasons.append(
            "XY is still centered near the robot origin; this looks like a local model frame, not a calibrated robot workspace frame"
        )

    return reasons


def execute_waypoints_draw_mode_csv(csv_path, draw_speed=DRAW_SPEED_DEFAULT, wait_per_point=WAIT_PER_POINT_DEFAULT, start_angles=None, start_speed=50, settle_s=3.0, z_min=None, z_max=None, allow_suspicious=False, coordinate_mode="local"):
    """Load waypoints from CSV and execute them in draw mode."""
    rows = load_waypoints_csv(csv_path)
    if not rows:
        return {"ok": False, "count": 0, "message": f"No valid waypoint rows in CSV: {csv_path}", "final_coords": _get_coords_safe(2, 0.05)}

    summary = summarize_waypoints(rows)
    reasons = detect_suspicious_waypoints(summary, z_min=z_min, z_max=z_max)
    if str(coordinate_mode).lower().strip() == "local":
        reasons = [r for r in reasons if "centered near the robot origin" not in r]
    if reasons and not allow_suspicious:
        return {
            "ok": False,
            "count": summary["count"],
            "message": "Blocked suspicious trajectory: " + " | ".join(reasons),
            "summary": summary,
            "final_coords": _get_coords_safe(2, 0.05),
        }

    return execute_waypoints_draw_mode(
        rows,
        draw_speed=draw_speed,
        wait_per_point=wait_per_point,
        start_angles=start_angles,
        start_speed=start_speed,
        settle_s=settle_s,
        z_min=z_min,
        z_max=z_max,
        coordinate_mode=coordinate_mode,
    )


def move_cartesian_incremental(x, y, z, rx, ry, rz, steps=60, speed=SPEED_DEFAULT, dt=0.03, z_min=None, z_max=None):
    mc = _get_mc()
    _ensure_power(mc)
    current = _get_coords_safe(3, 0.03)

    start_pose = [
        _finite(current[0], 0), _finite(current[1], 0), _finite(current[2], 0),
        _finite(current[3], 0), _finite(current[4], 0), _finite(current[5], 0),
    ]
    final_pose = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
    steps = max(1, int(steps))
    speed = int(speed)
    speed = speed if 1 <= speed <= 100 else SPEED_DEFAULT

    for i in range(1, steps + 1):
        alpha = i / float(steps)
        px = start_pose[0] + alpha * (final_pose[0] - start_pose[0])
        py = start_pose[1] + alpha * (final_pose[1] - start_pose[1])
        pz = start_pose[2] + alpha * (final_pose[2] - start_pose[2])

        if (z_min is not None and pz < float(z_min)) or (z_max is not None and pz > float(z_max)):
            return {"ok": False, "reached": False, "message": f"Abortado Z ({pz:.1f})", "final_coords": _get_coords_safe(2, 0.02)}

        mc.send_coords([px, py, pz, final_pose[3], final_pose[4], final_pose[5]], speed, int(MOVE_MODE))
        if dt and dt > 0:
            time.sleep(float(dt))

    time.sleep(0.12)
    return {"ok": True, "reached": None, "message": "Incremental enviado", "final_coords": _get_coords_safe(2, 0.02)}


def move_cartesian_close_xyz_priority(x, y, z, step_mm=1.0, speed=SPEED_DEFAULT, dt=0.02, tol_pos=0.8, max_iters=1000, try_flip_mode=True, z_min=None, z_max=None):
    mc = _get_mc()
    _ensure_power(mc)
    target_xyz = [float(x), float(y), float(z)]
    mode_used = int(MOVE_MODE)

    def clamp(delta, step_size):
        try:
            delta = float(delta)
            step_size = float(step_size)
        except Exception:
            return 0.0
        return step_size if delta > step_size else (-step_size if delta < -step_size else delta)

    current = _get_coords_safe(3, 0.03)
    if all(math.isnan(v) for v in current):
        mc.send_coords([target_xyz[0], target_xyz[1], target_xyz[2], 0, 0, 0], int(speed), int(MOVE_MODE))
        time.sleep(0.2)
        current = _get_coords_safe(3, 0.03)
        if all(math.isnan(v) for v in current):
            return {"ok": False, "reached": False, "iters": 0, "final_coords": current, "mode_used": mode_used, "message": "Sin telemetría"}

    stagnation = 0
    last_xyz = [_finite(current[0], 0), _finite(current[1], 0), _finite(current[2], 0)]

    for iteration in range(1, int(max_iters) + 1):
        current = _get_coords_safe(1, 0.01)
        cx = _finite(current[0], last_xyz[0])
        cy = _finite(current[1], last_xyz[1])
        cz = _finite(current[2], last_xyz[2])

        ex = target_xyz[0] - cx
        ey = target_xyz[1] - cy
        ez = target_xyz[2] - cz

        if abs(ex) <= tol_pos and abs(ey) <= tol_pos and abs(ez) <= tol_pos:
            return {"ok": True, "reached": True, "iters": iteration, "final_coords": [cx, cy, cz, current[3], current[4], current[5]], "mode_used": mode_used, "message": "OK (xyz-priority)"}

        nx = cx + clamp(ex, step_mm)
        ny = cy + clamp(ey, step_mm)
        nz = cz + clamp(ez, step_mm)

        if (z_min is not None and nz < float(z_min)) or (z_max is not None and nz > float(z_max)):
            return {"ok": False, "reached": False, "iters": iteration, "final_coords": [cx, cy, cz, current[3], current[4], current[5]], "mode_used": mode_used, "message": f"Límite Z (next={nz:.1f})"}

        rx = _finite(current[3], 0.0)
        ry = _finite(current[4], 0.0)
        rz = _finite(current[5], 0.0)

        speed_value = int(speed)
        speed_value = speed_value if 1 <= speed_value <= 100 else SPEED_DEFAULT
        mc.send_coords([nx, ny, nz, rx, ry, rz], speed_value, int(MOVE_MODE))
        time.sleep(float(dt))

        next_pose = _get_coords_safe(1, 0.01)
        dx = _finite(next_pose[0], nx) - last_xyz[0]
        dy = _finite(next_pose[1], ny) - last_xyz[1]
        dz = _finite(next_pose[2], nz) - last_xyz[2]
        progressed = math.sqrt(dx * dx + dy * dy + dz * dz)
        if progressed < 0.25:
            stagnation += 1
        else:
            stagnation = 0
            last_xyz = [_finite(next_pose[0], nx), _finite(next_pose[1], ny), _finite(next_pose[2], nz)]

        if try_flip_mode and stagnation >= 10:
            set_move_mode(1 - int(MOVE_MODE))
            mode_used = int(MOVE_MODE)
            stagnation = 0
            mc.send_coords([nx, ny, nz, rx, ry, rz], speed_value, int(MOVE_MODE))
            time.sleep(0.12)

    final_pose = _get_coords_safe(2, 0.02)
    return {"ok": bool(final_pose), "reached": False, "iters": int(max_iters), "final_coords": final_pose, "mode_used": mode_used, "message": "No cerró (xyz-priority)"}


if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("Usage: python mc_bridge.py path_to_waypoints.csv")
        print("No CSV provided; printing current pose only.")
        print(get_pose())
        sys.exit(0)

    csv_path = sys.argv[1]
    if not os.path.isfile(csv_path):
        print(f"CSV not found: {csv_path}")
        sys.exit(2)

    points = load_waypoints_csv(csv_path)
    if not points:
        print("No valid waypoint rows found in CSV.")
        sys.exit(3)

    print(f"Loaded {len(points)} waypoint rows from {csv_path}")
    summary = summarize_waypoints(points)
    print(summary)
    result = execute_waypoints_draw_mode_csv(csv_path)
    print(result)
