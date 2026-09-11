# ============================================================
# mc_bridge.py — puente robusto para MyCobot 280 M5 (pymycobot)
# - HOME por juntas
# - Movimiento cartesiano directo (IK interna)
# - Streaming incremental
# - Cierre XYZ priorizando posición (orientación libre)
# - Telemetría robusta (reintentos; NaN si falta dato)
# ============================================================

import time, math

# === AJUSTA ESTOS VALORES A TU EQUIPO ===
PORT = "COM6"        # <-- CAMBIA al puerto correcto
BAUD = 115200        # 280 M5: 115200 o 1000000
SPEED_DEFAULT = 50
MOVE_MODE = 1        # 1=lineal (cartesiano), 0=directo
TIMEOUT = 20.0

# ---- Conexión única ----
_mc = None
def _get_mc():
    global _mc
    if _mc is None:
        try:
            from pymycobot.mycobot import MyCobot
            cls = MyCobot
        except Exception:
            # fallback antiguo
            from pymycobot import MyCobot280
            cls = MyCobot280
        _mc = cls(PORT, BAUD)
        time.sleep(0.8)
    return _mc

def _ensure_power(mc):
    try:
        if not mc.is_power_on():
            mc.power_on(); time.sleep(0.8)
    except Exception:
        mc.power_on(); time.sleep(0.8)

def set_move_mode(mode:int):
    """Cambia el modo global 0/1."""
    global MOVE_MODE
    m = int(mode)
    if m not in (0,1): raise ValueError("mode debe ser 0 o 1")
    MOVE_MODE = m
    return MOVE_MODE

# ================= Telemetría robusta =================
def _to6_list(v):
    """Normaliza a 6 elementos, convierte a float; None -> NaN."""
    v = list(v) if isinstance(v, (list, tuple)) else []
    v = (v + [None]*6)[:6]
    out = []
    for x in v:
        try:
            out.append(float(x) if x is not None else math.nan)
        except Exception:
            out.append(math.nan)
    return out

def _get_coords_safe(retries=5, delay=0.05):
    mc = _get_mc()
    for _ in range(max(1,int(retries))):
        try:
            c = mc.get_coords()
            if c and len(c) >= 6:
                return _to6_list(c)
        except Exception:
            pass
        time.sleep(max(0.0, float(delay)))
    # último intento (aunque sea None)
    try:
        return _to6_list(mc.get_coords() or [])
    except Exception:
        return [math.nan]*6

def _get_angles_safe(retries=5, delay=0.05):
    mc = _get_mc()
    for _ in range(max(1,int(retries))):
        try:
            a = mc.get_angles()
            if a and len(a) >= 6:
                return _to6_list(a)
        except Exception:
            pass
        time.sleep(max(0.0, float(delay)))
    try:
        return _to6_list(mc.get_angles() or [])
    except Exception:
        return [math.nan]*6

def _finite(x, default):
    """Devuelve x si es número finito; si es NaN/None, devuelve default."""
    try:
        xf = float(x)
        return xf if math.isfinite(xf) else float(default)
    except Exception:
        return float(default)

# ---- API: Pose segura (siempre 6 floats, NaN donde falten) ----
def get_pose(retries: int = 5, delay: float = 0.05):
    """
    Devuelve dict con keys 'coords' y 'q_deg', cada uno de 6 floats.
    Si firmware devuelve None, rellena con NaN (no lanza excepciones).
    """
    mc = _get_mc()
    _ensure_power(mc)
    coords = _get_coords_safe(retries, delay)
    q      = _get_angles_safe(retries, delay)
    return {"coords": coords, "q_deg": q}

# ================= Movimientos =================
def move_joints(q1,q2,q3,q4,q5,q6, speed=SPEED_DEFAULT, wait=True, timeout_s=TIMEOUT):
    mc = _get_mc(); _ensure_power(mc)
    tgt = [float(q1),float(q2),float(q3),float(q4),float(q5),float(q6)]
    mc.send_angles(tgt,int(speed))
    if not wait:
        return {"ok":True,"final_q":tgt,"reached":None,"message":"Enviado"}
    t0=time.time(); last=None
    while time.time()-t0<float(timeout_s):
        last = _get_angles_safe(1,0.02)
        if all(abs(a-t)<=2.0 for a,t in zip(last, tgt) if math.isfinite(a)):
            return {"ok":True,"final_q":last,"reached":True,"message":"OK"}
        time.sleep(0.08)
    return {"ok":bool(last),"final_q":last,"reached":False,"message":"Timeout"}

def home(q_deg):
    if len(q_deg)!=6: raise ValueError("Se esperan 6 ángulos")
    return move_joints(*q_deg, speed=SPEED_DEFAULT, wait=True, timeout_s=TIMEOUT)

def move_cartesian(x,y,z,rx,ry,rz, speed=SPEED_DEFAULT, wait=True, timeout_s=TIMEOUT):
    mc=_get_mc(); _ensure_power(mc)
    target=[float(x),float(y),float(z),float(rx),float(ry),float(rz)]
    try:
        if hasattr(mc,"set_fresh_mode"): mc.set_fresh_mode(1)
    except Exception: pass
    mc.send_coords(target,int(speed),int(MOVE_MODE))
    if not wait:
        return {"ok":True,"final_coords":target,"reached":None,"message":"Enviado"}

    t0=time.time(); last=None
    while time.time()-t0<float(timeout_s):
        last = _get_coords_safe(1,0.02)
        pos_ok = all(abs(a-b)<=3.0 for a,b in zip(last[:3], target[:3]) if math.isfinite(a))
        ang_ok = all(abs(a-b)<=3.0 for a,b in zip(last[3:], target[3:]) if math.isfinite(a))
        if pos_ok and ang_ok:
            return {"ok":True,"final_coords":last,"reached":True,"message":"OK"}
        time.sleep(0.08)
    return {"ok":bool(last),"final_coords":last if last else target,"reached":False,"message":"Timeout"}

def move_cartesian_incremental(x,y,z,rx,ry,rz, steps=60, speed=SPEED_DEFAULT, dt=0.03, z_min=None, z_max=None):
    mc=_get_mc(); _ensure_power(mc)
    cur = _get_coords_safe(3,0.03)

    p0=[_finite(cur[0],0), _finite(cur[1],0), _finite(cur[2],0),
        _finite(cur[3],0), _finite(cur[4],0), _finite(cur[5],0)]
    pf=[float(x),float(y),float(z),float(rx),float(ry),float(rz)]
    steps=max(1,int(steps))
    spd=int(speed); spd=spd if 1<=spd<=100 else SPEED_DEFAULT

    for i in range(1,steps+1):
        a=i/float(steps)
        px=p0[0]+a*(pf[0]-p0[0]); py=p0[1]+a*(pf[1]-p0[1]); pz=p0[2]+a*(pf[2]-p0[2])

        if (z_min is not None and pz<float(z_min)) or (z_max is not None and pz>float(z_max)):
            return {"ok":False,"reached":False,"message":f"Abortado Z ({pz:.1f})",
                    "final_coords": _get_coords_safe(2,0.02)}

        mc.send_coords([px,py,pz,pf[3],pf[4],pf[5]],spd,int(MOVE_MODE))
        if dt and dt>0: time.sleep(float(dt))

    time.sleep(0.12)
    final = _get_coords_safe(2,0.02)
    return {"ok":True,"reached":None,"message":"Incremental enviado","final_coords":final}

# ================= Cierre XYZ priorizando posición =================
def move_cartesian_close_xyz_priority(x,y,z,
                                      step_mm=1.0,
                                      speed=SPEED_DEFAULT,
                                      dt=0.02,
                                      tol_pos=0.8,
                                      max_iters=1000,
                                      try_flip_mode=True,
                                      z_min=None, z_max=None):
    """
    Cierra a [x,y,z] priorizando la posición.
    Orientación: se usa SIEMPRE la *medida* en cada micro-paso.
    Robusto a lecturas None (usa NaN y _finite con fallback).
    """
    mc=_get_mc(); _ensure_power(mc)
    target_xyz=[float(x),float(y),float(z)]
    mode_used=int(MOVE_MODE)

    def clamp(d,s):
        try:
            d=float(d); s=float(s)
        except Exception:
            return 0.0
        return s if d>s else (-s if d<-s else d)

    cur = _get_coords_safe(3,0.03)            # [x y z rx ry rz] con NaN donde falte
    if all(math.isnan(v) for v in cur):
        # Sin telemetría: intenta un salto con orientación neutra
        mc.send_coords([target_xyz[0],target_xyz[1],target_xyz[2],0,0,0],int(speed),int(MOVE_MODE))
        time.sleep(0.2)
        cur = _get_coords_safe(3,0.03)
        if all(math.isnan(v) for v in cur):
            return {"ok":False,"reached":False,"iters":0,"final_coords":cur,
                    "mode_used":mode_used,"message":"Sin telemetría"}

    stagn=0
    last_xyz = [ _finite(cur[0],0), _finite(cur[1],0), _finite(cur[2],0) ]

    for it in range(1, int(max_iters)+1):
        cur = _get_coords_safe(1,0.01)
        cx = _finite(cur[0], last_xyz[0])
        cy = _finite(cur[1], last_xyz[1])
        cz = _finite(cur[2], last_xyz[2])

        ex = target_xyz[0] - cx
        ey = target_xyz[1] - cy
        ez = target_xyz[2] - cz

        if abs(ex) <= tol_pos and abs(ey) <= tol_pos and abs(ez) <= tol_pos:
            return {"ok":True,"reached":True,"iters":it,"final_coords":[cx,cy,cz,cur[3],cur[4],cur[5]],
                    "mode_used":mode_used,"message":"OK (xyz-priority)"}

        nx = cx + clamp(ex, step_mm)
        ny = cy + clamp(ey, step_mm)
        nz = cz + clamp(ez, step_mm)

        # límites Z seguros (usa _finite para evitar float(NoneType) )
        if (z_min is not None and nz < float(z_min)) or (z_max is not None and nz > float(z_max)):
            return {"ok":False,"reached":False,"iters":it,
                    "final_coords":[cx,cy,cz,cur[3],cur[4],cur[5]],
                    "mode_used":mode_used,"message":f"Límite Z (next={nz:.1f})"}

        # orientación medida, saneada
        rx = _finite(cur[3], 0.0)
        ry = _finite(cur[4], 0.0)
        rz = _finite(cur[5], 0.0)

        spd = int(speed); spd = spd if 1 <= spd <= 100 else SPEED_DEFAULT
        mc.send_coords([nx,ny,nz, rx,ry,rz], spd, int(MOVE_MODE))
        time.sleep(float(dt))

        cur2 = _get_coords_safe(1,0.01)
        dx = _finite(cur2[0], nx) - last_xyz[0]
        dy = _finite(cur2[1], ny) - last_xyz[1]
        dz = _finite(cur2[2], nz) - last_xyz[2]
        progressed = math.sqrt(dx*dx + dy*dy + dz*dz)
        if progressed < 0.25:
            stagn += 1
        else:
            stagn = 0
            last_xyz = [_finite(cur2[0], nx), _finite(cur2[1], ny), _finite(cur2[2], nz)]

        if try_flip_mode and stagn >= 10:
            set_move_mode(1 - int(MOVE_MODE))
            mode_used = int(MOVE_MODE)
            stagn = 0
            mc.send_coords([nx,ny,nz, rx,ry,rz], spd, int(MOVE_MODE))
            time.sleep(0.12)

    final = _get_coords_safe(2,0.02)
    return {"ok":bool(final),"reached":False,"iters":int(max_iters),
            "final_coords":final,"mode_used":mode_used,"message":"No cerró (xyz-priority)"}