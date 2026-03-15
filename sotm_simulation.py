#!/usr/bin/env python3
"""
SOTM (Shoot-on-the-Move) simulation — Team 7840 Parabolica 2026.

Mirrors ShootCommand.java + ShooterKinematics.java in Python to validate
the algorithm offline.  Three strategies are compared:
  1. Naive        — no SOTM, stationary table, turret aimed at current hub angle.
  2. SOTM angle   — 4-iter position prediction (angle + dFire), no RPM correction.
  3. SOTM full    — position prediction + radial/lateral RPM correction.

Usage:
    python3 sotm_simulation.py

Outputs:
    sotm_simulation.png  — 8-panel figure (4 scenarios × height error + lateral miss)
    Prints stationary validation table + radial-correction table to stdout.
"""

import math
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")          # headless — saves PNG without a display
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# =============================================================================
# Constants  (exactly matching Constants.java + ShooterKinematics.java)
# =============================================================================

# Physics
GRAVITY_MPS2            = 9.81
DRAG_COEFFICIENT        = 0.47
AIR_DENSITY_KGM3        = 1.225
FUEL_RADIUS_M           = 0.07506          # 5.91 in / 2
FUEL_MASS_KG            = 0.215
BALL_CROSS_AREA_M2      = math.pi * FUEL_RADIUS_M**2
DRAG_ACCEL_COEFF        = (0.5 * DRAG_COEFFICIENT * AIR_DENSITY_KGM3 * BALL_CROSS_AREA_M2) / FUEL_MASS_KG

# Hub geometry
HUB_CENTER_HEIGHT_M        = 1.4764        # 58.125 in
HUB_TOP_OPENING_HEIGHT_M   = 1.8288        # 72.0 in
HUB_TOP_OPENING_DIAMETER_M = 1.0592        # 41.7 in
HEX_RIM_CIRCUMRADIUS_M     = HUB_TOP_OPENING_DIAMETER_M / math.sqrt(3)

# Flywheel / launcher
FLYWHEEL_GEAR_RATIO        = 1.5
FLYWHEEL_WHEEL_RADIUS_M    = 0.0508        # 2 in radius
FLYWHEEL_EFFICIENCY        = 0.745
LAUNCH_HEIGHT_M            = 0.4191        # 16.5 in
RIM_SAFETY_MARGIN_M        = 0.05

# Hood angle range (mechanism angle, not exit angle)
HOOD_MIN_ANGLE_DEG         = 27.5
HOOD_MAX_ANGLE_DEG         = 42.5

# Shooting range
MIN_SHOOT_RANGE_M          = 1.5
MAX_SHOOT_RANGE_M          = 7.0

# Euler sim
SIM_DT_S                   = 0.010
MAX_SIM_STEPS              = 5000
MAX_LAUNCH_SPEED_MPS       = 30.0
BINARY_SEARCH_ITERS        = 35
ANGLE_SEARCH_ITERS         = 30
PRECOMPUTE_STEP_M          = 0.05

# SOTM
SOTM_DRAG_DECAY_FACTOR     = 1.0

# =============================================================================
# Unit conversions  (rpm_to_launch_speed / v0_to_rpm)
# =============================================================================

def rpm_to_launch_speed(rpm: float) -> float:
    """Motor RPM → ball launch speed (m/s). Mirrors ShooterKinematics.rpmToLaunchSpeed."""
    return (rpm / FLYWHEEL_GEAR_RATIO
            * 2.0 * math.pi * FLYWHEEL_WHEEL_RADIUS_M
            * FLYWHEEL_EFFICIENCY / 60.0)

def v0_to_rpm(v0: float) -> float:
    """Ball launch speed (m/s) → motor RPM. Mirrors ShooterKinematics.v0ToRPM."""
    return (v0 * 60.0 * FLYWHEEL_GEAR_RATIO
            / (2.0 * math.pi * FLYWHEEL_WHEEL_RADIUS_M * FLYWHEEL_EFFICIENCY))

def hood_to_exit_deg(hood_deg: float) -> float:
    """Hood mechanism angle → ball exit angle from horizontal.  exit = 90° − hood."""
    return 90.0 - hood_deg

# =============================================================================
# 2-D drag-aware Euler simulation (height-at-X and flight-time variants)
# These mirror the private simulate* methods in ShooterKinematics.java.
# =============================================================================

def _sim_height_at_x(vx: float, vy: float, target_x: float) -> float:
    """
    Simulate ball height when it reaches horizontal position target_x.
    (vx, vy) are initial velocity components in the radial-vertical plane.
    Returns -inf if the ball stalls or hits the ground first.
    Mirrors simulateHeightAtXComponents.
    """
    x, y = 0.0, LAUNCH_HEIGHT_M
    for _ in range(MAX_SIM_STEPS):
        if y < 0.0 or vx <= 0.0:
            return -math.inf
        x_prev, y_prev = x, y
        v  = math.sqrt(vx * vx + vy * vy)
        ax = -DRAG_ACCEL_COEFF * v * vx
        ay = -GRAVITY_MPS2 - DRAG_ACCEL_COEFF * v * vy
        x  += vx * SIM_DT_S;  y  += vy * SIM_DT_S
        vx += ax * SIM_DT_S;  vy += ay * SIM_DT_S
        if x >= target_x:
            frac = (target_x - x_prev) / (x - x_prev)
            return y_prev + frac * (y - y_prev)
    return y

def _sim_flight_time(vx: float, vy: float, target_x: float) -> float:
    """
    Simulate flight time to reach horizontal position target_x.
    Mirrors simulateFlightTimeComponents.
    """
    x, y, t = 0.0, LAUNCH_HEIGHT_M, 0.0
    for _ in range(MAX_SIM_STEPS):
        if y < 0.0 or vx <= 0.0:
            break
        x_prev = x
        v  = math.sqrt(vx * vx + vy * vy)
        ax = -DRAG_ACCEL_COEFF * v * vx
        ay = -GRAVITY_MPS2 - DRAG_ACCEL_COEFF * v * vy
        x += vx * SIM_DT_S;  y += vy * SIM_DT_S
        vx += ax * SIM_DT_S; vy += ay * SIM_DT_S
        t  += SIM_DT_S
        if x >= target_x:
            frac = (target_x - x_prev) / (x - x_prev)
            return (t - SIM_DT_S) + frac * SIM_DT_S
    return t

# =============================================================================
# 3-point precompute solver  (mirrors calculatePhysicsWithDrag)
# =============================================================================

def _vacuum_hood_angle(d_rim: float, h_rim: float, d_center: float, h_center: float) -> float:
    """Parabola through shooter-exit, rim, and hub center → initial hood angle guess."""
    h0   = LAUNCH_HEIGHT_M
    d1   = h_rim    - h0
    d2   = h_center - h0
    det  = d_rim * d_center * (d_rim - d_center)
    b    = (d_rim**2 * d2 - d_center**2 * d1) / det
    return 90.0 - math.degrees(math.atan(b))

def _min_launch_speed(theta_rad: float, target_x: float, target_h: float) -> float:
    """Binary search for minimum v0 that clears target_h at target_x."""
    vx_max = MAX_LAUNCH_SPEED_MPS * math.cos(theta_rad)
    vy_max = MAX_LAUNCH_SPEED_MPS * math.sin(theta_rad)
    if _sim_height_at_x(vx_max, vy_max, target_x) < target_h:
        return MAX_LAUNCH_SPEED_MPS
    lo, hi = 0.0, MAX_LAUNCH_SPEED_MPS
    for _ in range(BINARY_SEARCH_ITERS):
        mid = (lo + hi) / 2.0
        vxm = mid * math.cos(theta_rad)
        vym = mid * math.sin(theta_rad)
        if _sim_height_at_x(vxm, vym, target_x) >= target_h:
            hi = mid
        else:
            lo = mid
    return hi

def _height_at_center_for_hood(hood_deg: float, d_rim: float, h_clear: float,
                                d_center: float) -> float:
    """Given a hood angle, find rim-clearing v0, return simulated height at hub center."""
    theta_rad = math.radians(hood_to_exit_deg(hood_deg))
    v0 = _min_launch_speed(theta_rad, d_rim, h_clear)
    if v0 >= MAX_LAUNCH_SPEED_MPS:
        return -math.inf
    vx = v0 * math.cos(theta_rad)
    vy = v0 * math.sin(theta_rad)
    return _sim_height_at_x(vx, vy, d_center)

def _calc_physics(dist: float):
    """
    Full 3-point drag-aware solver.  Returns (rpm, hood_deg).
    Mirrors calculatePhysicsWithDrag (turret yaw tolerance omitted for simplicity).
    """
    d_rim   = max(0.1, dist - HEX_RIM_CIRCUMRADIUS_M)
    h_clear = HUB_TOP_OPENING_HEIGHT_M + FUEL_RADIUS_M + RIM_SAFETY_MARGIN_M
    h_center = HUB_CENTER_HEIGHT_M

    vacuum_hood = _vacuum_hood_angle(d_rim, h_clear, dist, h_center)
    a_lo = max(HOOD_MIN_ANGLE_DEG, vacuum_hood - 10.0)
    a_hi = min(HOOD_MAX_ANGLE_DEG, vacuum_hood + 10.0)

    hood_deg = vacuum_hood
    h_lo = _height_at_center_for_hood(a_lo, d_rim, h_clear, dist)
    h_hi = _height_at_center_for_hood(a_hi, d_rim, h_clear, dist)

    if h_lo <= h_center <= h_hi:
        for _ in range(ANGLE_SEARCH_ITERS):
            mid   = (a_lo + a_hi) / 2.0
            h_mid = _height_at_center_for_hood(mid, d_rim, h_clear, dist)
            if h_mid < h_center:
                a_lo = mid
            else:
                a_hi = mid
        hood_deg = (a_lo + a_hi) / 2.0

    hood_deg  = max(HOOD_MIN_ANGLE_DEG, min(HOOD_MAX_ANGLE_DEG, hood_deg))
    theta_rad = math.radians(hood_to_exit_deg(hood_deg))
    v0        = _min_launch_speed(theta_rad, d_rim, h_clear)

    if v0 >= MAX_LAUNCH_SPEED_MPS:
        return (0.0, HOOD_MIN_ANGLE_DEG)

    return (v0_to_rpm(v0), hood_deg)

# =============================================================================
# Build interpolation tables
# =============================================================================

print("Precomputing shooter table...", flush=True)
_dist_pts = np.arange(MIN_SHOOT_RANGE_M, MAX_SHOOT_RANGE_M + 1e-9, PRECOMPUTE_STEP_M)
_rpm_pts  = []
_hood_pts = []
for _d in _dist_pts:
    _rpm, _hood = _calc_physics(_d)
    _rpm_pts.append(_rpm)
    _hood_pts.append(_hood)

_rpm_interp  = interp1d(_dist_pts, _rpm_pts,  bounds_error=False,
                        fill_value=(_rpm_pts[0],  _rpm_pts[-1]))
_hood_interp = interp1d(_dist_pts, _hood_pts, bounds_error=False,
                        fill_value=(_hood_pts[0], _hood_pts[-1]))

def calculate(dist: float):
    """Table lookup + interpolation.  Mirrors ShooterKinematics.calculate(dist)."""
    dist = max(MIN_SHOOT_RANGE_M, min(MAX_SHOOT_RANGE_M, dist))
    rpm  = float(_rpm_interp(dist))
    hood = float(_hood_interp(dist))
    hood = max(HOOD_MIN_ANGLE_DEG, min(HOOD_MAX_ANGLE_DEG, hood))
    return (rpm, hood)

def get_flight_time(dist: float, setpoint) -> float:
    """Drag-aware flight time.  Mirrors ShooterKinematics.getFlightTimeSeconds."""
    rpm, hood = setpoint
    v0        = rpm_to_launch_speed(rpm)
    theta     = math.radians(hood_to_exit_deg(hood))
    return _sim_flight_time(v0 * math.cos(theta), v0 * math.sin(theta), dist)

print("Precompute done.\n", flush=True)

# =============================================================================
# calculateWithRadialVelocity  (mirrors ShooterKinematics.calculateWithRadialVelocity)
# =============================================================================

def calculate_with_radial_velocity(dist: float, v_radial: float, v_lateral: float):
    """
    Binary-search RPM correction for robot motion.
    vx_ground = sqrt(v0h² − vLateral²) + vRadial   (lead-angle geometry)
    Hood angle kept from table; only RPM is adjusted.
    Falls back to base setpoint if no valid solution exists.
    """
    base     = calculate(dist)
    if abs(v_radial) < 0.05 and abs(v_lateral) < 0.05:
        return base

    rpm, hood = base
    theta     = math.radians(hood_to_exit_deg(hood))
    h_target  = HUB_CENTER_HEIGHT_M
    vlat_sq   = v_lateral * v_lateral

    # Verify solution bracket at max speed
    v0h_max = MAX_LAUNCH_SPEED_MPS * math.cos(theta)
    if v0h_max**2 <= vlat_sq:
        return base
    vx_max = math.sqrt(v0h_max**2 - vlat_sq) + v_radial
    vy_max = MAX_LAUNCH_SPEED_MPS * math.sin(theta)
    if vx_max <= 0.0 or _sim_height_at_x(vx_max, vy_max, dist) < h_target:
        return base

    # Binary search on muzzle speed
    lo, hi = 0.0, MAX_LAUNCH_SPEED_MPS
    for _ in range(25):
        mid  = (lo + hi) / 2.0
        v0h  = mid * math.cos(theta)
        if v0h**2 <= vlat_sq:
            lo = mid; continue
        vx0  = math.sqrt(v0h**2 - vlat_sq) + v_radial
        vy0  = mid * math.sin(theta)
        if vx0 <= 0.0:
            lo = mid; continue
        if _sim_height_at_x(vx0, vy0, dist) < h_target:
            lo = mid
        else:
            hi = mid
    corrected_v0 = (lo + hi) / 2.0

    # Rim clearance check on corrected speed
    d_rim   = max(0.1, dist - HEX_RIM_CIRCUMRADIUS_M)
    h_clear = HUB_TOP_OPENING_HEIGHT_M + FUEL_RADIUS_M + RIM_SAFETY_MARGIN_M
    v0h_f   = corrected_v0 * math.cos(theta)
    if v0h_f**2 <= vlat_sq:
        return base
    vx_f = math.sqrt(v0h_f**2 - vlat_sq) + v_radial
    vy_f = corrected_v0 * math.sin(theta)
    if vx_f <= 0.0 or _sim_height_at_x(vx_f, vy_f, d_rim) < h_clear:
        return base

    return (v0_to_rpm(corrected_v0), hood)

# =============================================================================
# SOTM 4-iteration position prediction  (mirrors ShootCommand.execute SOTM block)
# =============================================================================

def sotm_4iter(hub_dist: float, alpha_now: float, vxT: float, vyT: float,
               ax: float = 0.0, ay: float = 0.0):
    """
    4-iteration SOTM position prediction.

    hub_dist   : current distance to hub (m)
    alpha_now  : current angle to hub in robot frame (rad); typically 0
    vxT, vyT   : turret pivot velocity (m/s); x = radial toward hub, y = lateral
    ax, ay     : pivot acceleration (m/s²), typically 0 in this sim

    Returns (d_fire, alpha_fire_rad, setpoint).
    """
    decay = SOTM_DRAG_DECAY_FACTOR
    hub_x = hub_dist * math.cos(alpha_now)
    hub_y = hub_dist * math.sin(alpha_now)

    # Seed tof from base setpoint at current distance
    sp  = calculate(hub_dist)
    tof = get_flight_time(hub_dist, sp)

    # Iteration 0
    fire_x = hub_x - (vxT * tof + 0.5 * ax * tof**2) * decay
    fire_y = hub_y - (vyT * tof + 0.5 * ay * tof**2) * decay
    d_raw  = math.hypot(fire_x, fire_y)
    d_fire = max(MIN_SHOOT_RANGE_M, min(MAX_SHOOT_RANGE_M, d_raw))
    alpha_fire = math.atan2(fire_y, fire_x)
    sp = calculate(d_fire)

    # Iterations 1–3: refine tof
    for _ in range(3):
        tof        = get_flight_time(d_fire, sp)
        fire_x     = hub_x - (vxT * tof + 0.5 * ax * tof**2) * decay
        fire_y     = hub_y - (vyT * tof + 0.5 * ay * tof**2) * decay
        d_raw      = math.hypot(fire_x, fire_y)
        d_fire     = max(MIN_SHOOT_RANGE_M, min(MAX_SHOOT_RANGE_M, d_raw))
        alpha_fire = math.atan2(fire_y, fire_x)
        sp = calculate(d_fire)

    return d_fire, alpha_fire, sp

# =============================================================================
# Full 3-D trajectory simulation (for evaluating actual shot outcome)
# =============================================================================

def _sim_3d(v0: float, theta: float, turret_angle: float, vxT: float, vyT: float):
    """
    Simulate 3-D ball trajectory in field frame.

    The robot is at the origin at t=0.  Ball initial velocity (field frame):
        vx = v0·cos(θ)·cos(α) + vxT        ← radial toward hub
        vy = v0·cos(θ)·sin(α) + vyT        ← lateral
        vz = v0·sin(θ)                      ← vertical

    Returns list of (x, y, z) positions at each SIM_DT_S step.
    """
    v0h = v0 * math.cos(theta)
    v0v = v0 * math.sin(theta)
    vx  = v0h * math.cos(turret_angle) + vxT
    vy  = v0h * math.sin(turret_angle) + vyT
    vz  = v0v

    x, y, z = 0.0, 0.0, LAUNCH_HEIGHT_M
    traj = [(x, y, z)]

    for _ in range(MAX_SIM_STEPS):
        if z < 0.0:
            break
        v_mag = math.sqrt(vx*vx + vy*vy + vz*vz)
        ax = -DRAG_ACCEL_COEFF * v_mag * vx
        ay = -DRAG_ACCEL_COEFF * v_mag * vy
        az = -GRAVITY_MPS2 - DRAG_ACCEL_COEFF * v_mag * vz
        x  += vx * SIM_DT_S;  y  += vy * SIM_DT_S;  z  += vz * SIM_DT_S
        vx += ax * SIM_DT_S;  vy += ay * SIM_DT_S;  vz += az * SIM_DT_S
        traj.append((x, y, z))

    return traj

def _at_hub(traj, hub_x: float):
    """
    Interpolate trajectory to find (z_height, y_lateral) when ball crosses x = hub_x.
    Returns (None, None) if ball never reaches hub_x.
    """
    for i in range(1, len(traj)):
        x0, y0, z0 = traj[i - 1]
        x1, y1, z1 = traj[i]
        if x1 >= hub_x:
            if (x1 - x0) < 1e-12:
                return z0, y0
            frac = (hub_x - x0) / (x1 - x0)
            return z0 + frac * (z1 - z0), y0 + frac * (y1 - y0)
    return None, None

# =============================================================================
# Strategy evaluation
# =============================================================================

def evaluate(hub_dist: float, vxT: float, vyT: float, strategy: str):
    """
    Compute height error and lateral miss for one (scenario, strategy) combination.

    hub_dist : current distance to hub (m); hub is directly ahead (alpha_now = 0)
    vxT      : radial velocity toward hub (m/s); positive = approaching
    vyT      : lateral velocity (m/s)
    strategy : 'naive' | 'sotm_angle'

    Returns (height_error_m, lateral_miss_m).
    """
    if strategy == 'naive':
        sp   = calculate(hub_dist)
        rpm, hood = sp
        v0   = rpm_to_launch_speed(rpm)
        theta = math.radians(hood_to_exit_deg(hood))
        turret_angle = 0.0          # aim straight at current hub direction

    elif strategy == 'sotm_angle':
        d_fire, alpha_fire, sp = sotm_4iter(hub_dist, 0.0, vxT, vyT)
        rpm, hood = sp
        v0   = rpm_to_launch_speed(rpm)
        theta = math.radians(hood_to_exit_deg(hood))
        turret_angle = alpha_fire   # lead angle from position prediction, no RPM fix

    elif strategy == 'sotm_full':
        d_fire, alpha_fire, sp = sotm_4iter(hub_dist, 0.0, vxT, vyT)
        v_rad = vxT * math.cos(alpha_fire) + vyT * math.sin(alpha_fire)
        v_lat = -vxT * math.sin(alpha_fire) + vyT * math.cos(alpha_fire)
        sp = calculate_with_radial_velocity(d_fire, v_rad, v_lat)
        rpm, hood = sp
        v0   = rpm_to_launch_speed(rpm)
        theta = math.radians(hood_to_exit_deg(hood))
        turret_angle = alpha_fire

    else:
        raise ValueError(f"Unknown strategy: {strategy}")

    traj = _sim_3d(v0, theta, turret_angle, vxT, vyT)
    z_hub, y_hub = _at_hub(traj, hub_dist)

    if z_hub is None:
        return float('nan'), float('nan')

    return z_hub - HUB_CENTER_HEIGHT_M, y_hub   # (height_error, lateral_miss)

# =============================================================================
# Scenarios
# =============================================================================

STRATEGIES = [
    ('naive',      'Naive (no SOTM)',        '#e74c3c'),
    ('sotm_angle', 'SOTM (current code)',    '#2ecc71'),
]

SCENARIOS = [
    {
        'title': 'Radial Approach\n(toward hub)',
        'vxT':   lambda v: +v,
        'vyT':   lambda v:  0.0,
    },
    {
        'title': 'Radial Retreat\n(away from hub)',
        'vxT':   lambda v: -v,
        'vyT':   lambda v:  0.0,
    },
    {
        'title': 'Lateral Strafe\n(perpendicular)',
        'vxT':   lambda v:  0.0,
        'vyT':   lambda v: +v,
    },
    {
        'title': 'Diagonal (45°)\nradial + lateral',
        'vxT':   lambda v:  v / math.sqrt(2),
        'vyT':   lambda v:  v / math.sqrt(2),
    },
]

HUB_DIST   = 4.0                           # typical competition distance
SPEEDS     = np.linspace(0.0, 3.5, 36)    # 0 → 3.5 m/s

# Height band in which a shot can score (ball must enter hub opening)
# The ball needs z ≈ HUB_CENTER_HEIGHT_M; tolerance is ±(opening_half_height - ball_r)
H_HALF_BAND = (HUB_TOP_OPENING_HEIGHT_M - HUB_CENTER_HEIGHT_M) - FUEL_RADIUS_M   # ~0.28 m
# Lateral band: hub opening diameter / 2 − ball radius
Y_HALF_BAND = HUB_TOP_OPENING_DIAMETER_M / 2.0 - FUEL_RADIUS_M                   # ~0.45 m

# =============================================================================
# Run scenarios and collect results
# =============================================================================

all_h_err    = {}   # [scenario][strategy] → array
all_lat_miss = {}

for sc in SCENARIOS:
    title = sc['title']
    print(f"Running: {title.replace(chr(10), ' ')}", flush=True)
    all_h_err[title]    = {}
    all_lat_miss[title] = {}
    for strat, _, _ in STRATEGIES:
        h_errs = []
        lats   = []
        for v in SPEEDS:
            h_err, lat = evaluate(HUB_DIST, sc['vxT'](v), sc['vyT'](v), strat)
            h_errs.append(h_err)
            lats.append(lat)
        all_h_err[title][strat]    = np.array(h_errs)
        all_lat_miss[title][strat] = np.array(lats)

# =============================================================================
# Figure
# =============================================================================

fig, axes = plt.subplots(2, 4, figsize=(20, 9))
fig.suptitle(
    f"SOTM Algorithm Validation  ·  Hub Distance = {HUB_DIST} m  ·  Team 7840 Parabolica 2026",
    fontsize=13, fontweight='bold'
)

for col, sc in enumerate(SCENARIOS):
    title = sc['title']

    # ── Row 0: height error ────────────────────────────────────────────────
    ax = axes[0][col]
    ax.axhspan(-H_HALF_BAND, H_HALF_BAND, alpha=0.10, color='#27ae60',
               label='Scoring zone (height)')
    ax.axhline(0, color='k', linewidth=0.8, linestyle='-')
    ax.axhline(-H_HALF_BAND, color='#27ae60', linewidth=1, linestyle='--', alpha=0.7)
    ax.axhline(+H_HALF_BAND, color='#27ae60', linewidth=1, linestyle='--', alpha=0.7)
    for strat, label, color in STRATEGIES:
        ax.plot(SPEEDS, all_h_err[title][strat],
                label=label, color=color, linewidth=2.0)
    ax.set_title(title, fontsize=10, pad=4)
    ax.set_ylim(-0.55, 0.55)
    ax.set_ylabel('Height error (m)' if col == 0 else '', fontsize=9)
    ax.set_xticks([0, 1, 2, 3])
    ax.grid(True, alpha=0.3)
    if col == 0:
        ax.legend(fontsize=7.5, loc='upper left')

    # ── Row 1: lateral miss ────────────────────────────────────────────────
    ax = axes[1][col]
    ax.axhspan(-Y_HALF_BAND, Y_HALF_BAND, alpha=0.10, color='#2980b9',
               label='Scoring zone (lateral)')
    ax.axhline(0, color='k', linewidth=0.8)
    ax.axhline(-Y_HALF_BAND, color='#2980b9', linewidth=1, linestyle='--', alpha=0.7)
    ax.axhline(+Y_HALF_BAND, color='#2980b9', linewidth=1, linestyle='--', alpha=0.7)
    for strat, label, color in STRATEGIES:
        ax.plot(SPEEDS, all_lat_miss[title][strat],
                label=label, color=color, linewidth=2.0)
    ax.set_ylim(-0.7, 0.7)
    ax.set_xlabel('Robot speed (m/s)', fontsize=9)
    ax.set_ylabel('Lateral miss (m)' if col == 0 else '', fontsize=9)
    ax.set_xticks([0, 1, 2, 3])
    ax.grid(True, alpha=0.3)
    if col == 0:
        ax.legend(fontsize=7.5, loc='upper left')

plt.tight_layout(rect=[0, 0, 1, 0.96])
out_png = 'sotm_simulation.png'
plt.savefig(out_png, dpi=150, bbox_inches='tight')
print(f"\nSaved: {out_png}")

# =============================================================================
# Stationary table validation  (should closely match Java precompute output)
# =============================================================================

print()
print("── Stationary table (0 m/s, validate Java precompute) ──────────────────────────")
print(f"{'Dist':>6} │ {'RPM':>8} │ {'Hood°':>7} │ {'Exit°':>7} │"
      f" {'v0 m/s':>8} │ {'tof s':>6} │ {'h@hub m':>8} │ {'err mm':>7}")
print("───────┼──────────┼─────────┼─────────┼──────────┼────────┼──────────┼─────────")
for d in np.arange(1.5, 7.05, 0.5):
    rpm, hood = calculate(d)
    v0        = rpm_to_launch_speed(rpm)
    theta     = math.radians(hood_to_exit_deg(hood))
    tof       = _sim_flight_time(v0 * math.cos(theta), v0 * math.sin(theta), d)
    h         = _sim_height_at_x(v0 * math.cos(theta), v0 * math.sin(theta), d)
    err_mm    = (h - HUB_CENTER_HEIGHT_M) * 1000.0
    print(f"{d:>6.1f} │ {rpm:>8.1f} │ {hood:>7.2f} │ {hood_to_exit_deg(hood):>7.2f} │"
          f" {v0:>8.3f} │ {tof:>6.3f} │ {h:>8.4f} │ {err_mm:>+7.1f}")

# =============================================================================
# Summary: how many strategies stay inside scoring zone across all scenarios?
# =============================================================================

print()
print("── Scoring-zone pass rate by strategy (speed 0–3.5 m/s, all 4 scenarios) ───────")
print(f"{'Strategy':<30} │ {'Height inside zone':>20} │ {'Lateral inside zone':>20}")
print("─" * 76)
total_pts = len(SPEEDS) * len(SCENARIOS)
for strat, label, _ in STRATEGIES:
    h_ok   = 0
    lat_ok = 0
    for sc in SCENARIOS:
        title = sc['title']
        h_ok   += int(np.sum(np.abs(all_h_err[title][strat])    <= H_HALF_BAND))
        lat_ok += int(np.sum(np.abs(all_lat_miss[title][strat]) <= Y_HALF_BAND))
    print(f"  {label:<28} │  {h_ok:>4}/{total_pts}  ({100*h_ok/total_pts:5.1f} %)     │"
          f"  {lat_ok:>4}/{total_pts}  ({100*lat_ok/total_pts:5.1f} %)")

print()
print(f"Scoring zone limits  →  height: ±{H_HALF_BAND*100:.1f} cm   lateral: ±{Y_HALF_BAND*100:.1f} cm")
print()
