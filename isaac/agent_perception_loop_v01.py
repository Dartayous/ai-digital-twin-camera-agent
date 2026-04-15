#!/usr/bin/env python3

import math
import time
from pxr import Usd, UsdGeom, Gf

# ============================================================
# CONFIG
# ============================================================

STAGE_PATH = "/mnt/macross/DATA_ENGINEERING/USD_PROJECTS/project_07_ai_driven_digital_twin_system/usd/runtime_v03.usda"

AGENT_ROOT_PATH = "/World/AgentRoot"
CAMERA_PATH     = "/World/AgentRoot/AgentCamera"

# Based on your CURRENT clean hierarchy:
TARGET_PATH     = "/World/EnvWrapper/Environment/Warning_Light"

# ---------- Perception ----------
VISIBLE_ALIGNMENT_THRESHOLD = 0.80    # detect target
LOCK_ALIGNMENT_THRESHOLD    = 0.985   # hold lock / stop rotation
LOSE_TARGET_THRESHOLD       = 0.70    # drop back to patrol

# ---------- Patrol ----------
SCAN_SPEED_DEG_PER_STEP = 1.5
SCAN_LIMIT_DEG          = 45.0

# ---------- Lock Control ----------
KP_YAW                  = 0.18        # proportional gain
MAX_YAW_STEP_DEG        = 0.5         # clamp per loop
CENTER_DEADBAND_DEG     = 1.25        # stop micro-hunting when nearly centered

# ---------- Loop ----------
SLEEP_SEC               = 0.05

PROBE_STEP_DEG = 0.5
FOCUS_KP = 0.10
FOCUS_MAX_STEP_DEG = 0.25
CENTER_DEADBAND_DEG = 2.0

FOCUS_TEST_STEP_DEG = 0.25
FOCUS_APPLY_STEP_DEG = 0.25
LOCK_ALIGNMENT_EPS = 0.0005

# ============================================================
# HELPERS
# ============================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def safe_normalize(v: Gf.Vec3d):
    length = v.GetLength()
    if length < 1e-8:
        return None
    return v / length

def get_stage(path):
    stage = Usd.Stage.Open(path)
    if not stage:
        raise RuntimeError(f"Failed to open stage: {path}")
    return stage

def get_prim(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Invalid prim: {path}")
    return prim

def get_world_transform(cache, prim):
    return cache.GetLocalToWorldTransform(prim)

def get_world_position(cache, prim):
    xf = get_world_transform(cache, prim)
    return xf.ExtractTranslation()

def get_camera_forward_world(cache, camera_prim):
    """
    USD camera forward is -Z in local camera space.
    Use full world matrix to transform the direction.
    """
    xf = get_world_transform(cache, camera_prim)
    fwd = xf.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
    return safe_normalize(fwd)

def flatten_y(v: Gf.Vec3d):
    return Gf.Vec3d(v[0], 0.0, v[2])

def signed_horizontal_error_deg(forward_world: Gf.Vec3d, to_target_world: Gf.Vec3d):
    f = safe_normalize(flatten_y(forward_world))
    t = safe_normalize(flatten_y(to_target_world))

    if f is None or t is None:
        return 0.0

    dot_val = clamp(Gf.Dot(f, t), -1.0, 1.0)
    cross_y = Gf.Cross(f, t)[1]
    err_rad = math.atan2(cross_y, dot_val)
    err_deg = math.degrees(err_rad)

    return -err_deg
    # If steering is backwards, change to:
    # return -err_deg

def full_view_alignment(forward_world: Gf.Vec3d, to_target_world: Gf.Vec3d):
    f = safe_normalize(forward_world)
    t = safe_normalize(to_target_world)
    if f is None or t is None:
        return -1.0
    return clamp(Gf.Dot(f, t), -1.0, 1.0)

def get_or_create_rotate_y_op(xformable: UsdGeom.Xformable):
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateY:
            return op

    return xformable.AddRotateYOp(UsdGeom.XformOp.PrecisionDouble)

def get_current_root_yaw_deg(agent_root_prim):
    xformable = UsdGeom.Xformable(agent_root_prim)
    ry_op = get_or_create_rotate_y_op(xformable)
    val = ry_op.Get()
    return float(val) if val is not None else 0.0

def set_root_yaw_deg(agent_root_prim, yaw_deg):
    xformable = UsdGeom.Xformable(agent_root_prim)
    ry_op = get_or_create_rotate_y_op(xformable)
    ry_op.Set(float(yaw_deg))


def get_target_in_root_local(cache, agent_root_prim, target_prim):
    root_world = get_world_transform(cache, agent_root_prim)
    root_world_inv = root_world.GetInverse()

    target_world_pos = get_world_position(cache, target_prim)
    target_local = root_world_inv.Transform(target_world_pos)
    return target_local


def measure_alignment(cache, camera_prim, target_prim):
    camera_pos = get_world_position(cache, camera_prim)
    target_pos = get_world_position(cache, target_prim)

    to_target = target_pos - camera_pos
    to_target_n = safe_normalize(to_target)
    camera_fwd = get_camera_forward_world(cache, camera_prim)

    if to_target_n is None or camera_fwd is None:
        return -1.0

    return full_view_alignment(camera_fwd, to_target_n)


def wrap_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def get_target_in_camera_local(cache, camera_prim, target_prim):
    cam_world = get_world_transform(cache, camera_prim)
    cam_world_inv = cam_world.GetInverse()

    target_world_pos = get_world_position(cache, target_prim)
    target_cam = cam_world_inv.Transform(target_world_pos)
    return target_cam


def compute_camera_local_yaw_error_deg(cache, camera_prim, target_prim):
    """
    Compute horizontal yaw error in CAMERA local space.

    Assumes camera forward is local -Z.
    Positive / negative sign may need one final flip depending on your stage.
    """
    t_cam = get_target_in_camera_local(cache, camera_prim, target_prim)

    x = float(t_cam[0])
    z = float(t_cam[2])

    err_rad = math.atan2(x, -z)
    err_deg = math.degrees(err_rad)

    return err_deg
    # If steering is reversed, use:
    # return -err_deg


def choose_focus_sign(stage, agent_root_prim, camera_prim, target_prim, current_root_yaw):
    # Measure baseline
    cache = UsdGeom.XformCache()
    base_alignment = measure_alignment(cache, camera_prim, target_prim)

    # Try positive probe
    set_root_yaw_deg(agent_root_prim, wrap_angle_deg(current_root_yaw + PROBE_STEP_DEG))
    stage.Save()
    cache = UsdGeom.XformCache()
    plus_alignment = measure_alignment(cache, camera_prim, target_prim)

    # Try negative probe
    set_root_yaw_deg(agent_root_prim, wrap_angle_deg(current_root_yaw - PROBE_STEP_DEG))
    stage.Save()
    cache = UsdGeom.XformCache()
    minus_alignment = measure_alignment(cache, camera_prim, target_prim)

    # Restore original yaw
    set_root_yaw_deg(agent_root_prim, current_root_yaw)
    stage.Save()

    if plus_alignment > minus_alignment:
        return +1.0
    else:
        return -1.0
    

def get_alignment_for_yaw(stage, agent_root_prim, camera_prim, target_prim, yaw_deg):
    set_root_yaw_deg(agent_root_prim, wrap_angle_deg(yaw_deg))
    stage.Save()

    cache = UsdGeom.XformCache()
    return measure_alignment(cache, camera_prim, target_prim)


# ============================================================
# MAIN
# ============================================================

def main():
    stage = get_stage(STAGE_PATH)

    agent_root_prim = get_prim(stage, AGENT_ROOT_PATH)
    camera_prim     = get_prim(stage, CAMERA_PATH)
    target_prim     = get_prim(stage, TARGET_PATH)

    initial_yaw_deg = get_current_root_yaw_deg(agent_root_prim)
    scan_dir = 1.0
    state = "patrol"

    print("========================================")
    print("Agent lock controller starting")
    print(f"Stage:  {STAGE_PATH}")
    print(f"Agent:  {AGENT_ROOT_PATH}")
    print(f"Camera: {CAMERA_PATH}")
    print(f"Target: {TARGET_PATH}")
    print(f"Initial root yaw: {initial_yaw_deg:.3f}")
    print("========================================")

    focus_sign = None
    prev_state = None

    while True:
        cache = UsdGeom.XformCache()

        camera_pos = get_world_position(cache, camera_prim)
        target_pos = get_world_position(cache, target_prim)

        to_target = target_pos - camera_pos
        to_target_n = safe_normalize(to_target)
        camera_fwd = get_camera_forward_world(cache, camera_prim)

        if to_target_n is None or camera_fwd is None:
            print("Bad vector state; skipping frame")
            time.sleep(SLEEP_SEC)
            continue

        alignment = full_view_alignment(camera_fwd, to_target_n)
        yaw_error_deg = compute_camera_local_yaw_error_deg(cache, camera_prim, target_prim)

        current_root_yaw = get_current_root_yaw_deg(agent_root_prim)

        if state != prev_state:
            if state == "focus_target":
                focus_sign = choose_focus_sign(
                    stage,
                    agent_root_prim,
                    camera_prim,
                    target_prim,
                    current_root_yaw
                )
            prev_state = state

        # ----------------------------------------------------
        # STATE MACHINE
        # ----------------------------------------------------
        if state == "patrol":
            if alignment >= VISIBLE_ALIGNMENT_THRESHOLD:
                state = "focus_target"
            else:
                next_yaw = wrap_angle_deg(current_root_yaw + scan_dir * SCAN_SPEED_DEG_PER_STEP)

                rel = next_yaw - initial_yaw_deg
                if rel > SCAN_LIMIT_DEG:
                    next_yaw = initial_yaw_deg + SCAN_LIMIT_DEG
                    scan_dir = -1.0
                elif rel < -SCAN_LIMIT_DEG:
                    next_yaw = initial_yaw_deg - SCAN_LIMIT_DEG
                    scan_dir = 1.0

                set_root_yaw_deg(agent_root_prim, next_yaw)
                stage.Save()


        elif state == "focus_target":
            if alignment < LOSE_TARGET_THRESHOLD:
                state = "patrol"
            else:
                current_yaw = current_root_yaw
                base_alignment = alignment

                left_yaw  = wrap_angle_deg(current_yaw - FOCUS_TEST_STEP_DEG)
                right_yaw = wrap_angle_deg(current_yaw + FOCUS_TEST_STEP_DEG)

                left_alignment  = get_alignment_for_yaw(stage, agent_root_prim, camera_prim, target_prim, left_yaw)
                right_alignment = get_alignment_for_yaw(stage, agent_root_prim, camera_prim, target_prim, right_yaw)

                # Restore current yaw before choosing
                set_root_yaw_deg(agent_root_prim, current_yaw)
                stage.Save()

                best_alignment = base_alignment
                next_yaw = current_yaw

                if left_alignment > best_alignment + LOCK_ALIGNMENT_EPS:
                    best_alignment = left_alignment
                    next_yaw = left_yaw

                if right_alignment > best_alignment + LOCK_ALIGNMENT_EPS:
                    best_alignment = right_alignment
                    next_yaw = right_yaw

                set_root_yaw_deg(agent_root_prim, next_yaw)
                stage.Save()

        # ----------------------------------------------------
        # DEBUG
        # ----------------------------------------------------
        print(
            f"state: {state:12s} | "
            f"alignment: {alignment: .3f} | "
            f"yaw_error_deg: {yaw_error_deg: .2f} | "
            f"root_yaw: {get_current_root_yaw_deg(agent_root_prim): .2f}"
        )

        time.sleep(SLEEP_SEC)

if __name__ == "__main__":
    main()