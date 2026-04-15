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
TARGET_PATH     = "/World/EnvWrapper/Environment/Warning_Light"

# ---------- Detection / state ----------
START_ROOT_YAW_DEG = 147.73
START_ROOT_TRANSLATE = Gf.Vec3d(0.0, 8.0, 20.0)

VISIBLE_ALIGNMENT_THRESHOLD = 0.80
LOSE_TARGET_THRESHOLD       = 0.70

# ---------- Patrol ----------
SCAN_SPEED_DEG_PER_STEP = 1.5
SCAN_LIMIT_DEG          = 45.0

# ---------- Lock controller (alignment ascent) ----------
FOCUS_TEST_STEP_DEG  = 0.25
LOCK_ALIGNMENT_EPS   = 0.0005

# ---------- Movement ----------
MOVE_ALIGNMENT_THRESHOLD = 0.95     # only move when target is strongly centered
MOVE_STEP_UNITS          = 0.05     # world-space forward step per loop
TARGET_STOP_DISTANCE     = 1.25     # stop advancing when close enough

# ---------- Loop ----------
SLEEP_SEC = 0.05


ARRIVAL_DISTANCE = 6.0
DISTANCE_EPS = 0.02

# ============================================================
# HELPERS
# ============================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def wrap_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle

def safe_normalize(v):
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
    xf = get_world_transform(cache, camera_prim)
    fwd = xf.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
    return safe_normalize(fwd)

def full_view_alignment(forward_world, to_target_world):
    f = safe_normalize(forward_world)
    t = safe_normalize(to_target_world)
    if f is None or t is None:
        return -1.0
    return clamp(Gf.Dot(f, t), -1.0, 1.0)

def flatten_y(v):
    return Gf.Vec3d(v[0], 0.0, v[2])

def signed_horizontal_error_deg(forward_world, to_target_world):
    # Debug only. Control no longer depends on this.
    f = safe_normalize(flatten_y(forward_world))
    t = safe_normalize(flatten_y(to_target_world))

    if f is None or t is None:
        return 0.0

    dot_val = clamp(Gf.Dot(f, t), -1.0, 1.0)
    cross_y = Gf.Cross(f, t)[1]
    err_rad = math.atan2(cross_y, dot_val)
    err_deg = math.degrees(err_rad)
    return err_deg

def measure_alignment(cache, camera_prim, target_prim):
    camera_pos = get_world_position(cache, camera_prim)
    target_pos = get_world_position(cache, target_prim)

    to_target = target_pos - camera_pos
    to_target_n = safe_normalize(to_target)
    camera_fwd = get_camera_forward_world(cache, camera_prim)

    if to_target_n is None or camera_fwd is None:
        return -1.0

    return full_view_alignment(camera_fwd, to_target_n)

def get_or_create_rotate_y_op(xformable):
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateY:
            return op
    return xformable.AddRotateYOp(UsdGeom.XformOp.PrecisionDouble)

def get_or_create_translate_op(xformable):
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op
    return xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)

def get_current_root_yaw_deg(agent_root_prim):
    xformable = UsdGeom.Xformable(agent_root_prim)
    ry_op = get_or_create_rotate_y_op(xformable)
    val = ry_op.Get()
    return float(val) if val is not None else 0.0

def set_root_yaw_deg(agent_root_prim, yaw_deg):
    xformable = UsdGeom.Xformable(agent_root_prim)
    ry_op = get_or_create_rotate_y_op(xformable)
    ry_op.Set(float(wrap_angle_deg(yaw_deg)))

def get_current_root_translate(agent_root_prim):
    xformable = UsdGeom.Xformable(agent_root_prim)
    t_op = get_or_create_translate_op(xformable)
    val = t_op.Get()
    if val is None:
        return Gf.Vec3d(0.0, 0.0, 0.0)
    return Gf.Vec3d(val[0], val[1], val[2])

def set_root_translate(agent_root_prim, t):
    xformable = UsdGeom.Xformable(agent_root_prim)
    t_op = get_or_create_translate_op(xformable)
    t_op.Set(Gf.Vec3d(t[0], t[1], t[2]))

def get_alignment_for_yaw(stage, agent_root_prim, camera_prim, target_prim, yaw_deg):
    set_root_yaw_deg(agent_root_prim, wrap_angle_deg(yaw_deg))
    stage.Save()
    cache = UsdGeom.XformCache()
    return measure_alignment(cache, camera_prim, target_prim)

def move_root_forward_small_step(agent_root_prim, yaw_deg, step_units):
    """
    Move AgentRoot forward in world XZ based on its local -Z forward.
    Y is preserved.
    """
    current_t = get_current_root_translate(agent_root_prim)

    yaw_rad = math.radians(yaw_deg)

    # Local -Z forward rotated by Y yaw into world XZ
    forward_x = -math.sin(yaw_rad)
    forward_z = -math.cos(yaw_rad)

    next_t = Gf.Vec3d(
        current_t[0] + forward_x * step_units,
        current_t[1],
        current_t[2] + forward_z * step_units
    )

    set_root_translate(agent_root_prim, next_t)


def reset_agent_root_pose(agent_root_prim, yaw_deg, translate_vec):
    set_root_yaw_deg(agent_root_prim, yaw_deg)
    set_root_translate(agent_root_prim, translate_vec)


# ============================================================
# MAIN
# ============================================================

def main():
    stage = get_stage(STAGE_PATH)

    agent_root_prim = get_prim(stage, AGENT_ROOT_PATH)
    camera_prim     = get_prim(stage, CAMERA_PATH)
    target_prim     = get_prim(stage, TARGET_PATH)

    # Reset to known startup pose every run
    reset_agent_root_pose(agent_root_prim, START_ROOT_YAW_DEG, START_ROOT_TRANSLATE)
    stage.Save()

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

    best_distance_to_target = None

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
        yaw_error_deg = signed_horizontal_error_deg(camera_fwd, to_target_n)  # debug only
        current_root_yaw = get_current_root_yaw_deg(agent_root_prim)
        distance_to_target = to_target.GetLength()

        # ----------------------------------------------------
        # STATE MACHINE
        # ----------------------------------------------------
        if state == "patrol":
            if alignment >= VISIBLE_ALIGNMENT_THRESHOLD:
                state = "focus_target"
                best_distance_to_target = distance_to_target
            else:
                next_yaw = wrap_angle_deg(current_root_yaw + scan_dir * SCAN_SPEED_DEG_PER_STEP)

                rel = wrap_angle_deg(next_yaw - initial_yaw_deg)

                if rel > SCAN_LIMIT_DEG:
                    next_yaw = wrap_angle_deg(initial_yaw_deg + SCAN_LIMIT_DEG)
                    scan_dir = -1.0
                elif rel < -SCAN_LIMIT_DEG:
                    next_yaw = wrap_angle_deg(initial_yaw_deg - SCAN_LIMIT_DEG)
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

                # Apply best yaw step
                set_root_yaw_deg(agent_root_prim, next_yaw)

             
                # Update best (closest) distance seen during this lock episode
                if best_distance_to_target is None or distance_to_target < best_distance_to_target:
                    best_distance_to_target = distance_to_target

                distance_getting_worse = distance_to_target > (best_distance_to_target + DISTANCE_EPS)
                close_enough = distance_to_target <= ARRIVAL_DISTANCE

                # Controlled forward movement only when:
                # 1) lock is strong
                # 2) not already close enough
                # 3) we are not moving away from the best approach distance
                if (
                    best_alignment >= MOVE_ALIGNMENT_THRESHOLD
                    and not close_enough
                    and not distance_getting_worse
                ):
                    move_root_forward_small_step(agent_root_prim, next_yaw, MOVE_STEP_UNITS)

                stage.Save()

        # ----------------------------------------------------
        # DEBUG
        # ----------------------------------------------------
        root_t = get_current_root_translate(agent_root_prim)
        print(
            f"state: {state:12s} | "
            f"alignment: {alignment: .3f} | "
            f"yaw_error_deg: {yaw_error_deg: .2f} | "
            f"root_yaw: {get_current_root_yaw_deg(agent_root_prim): .2f} | "
            f"root_t: ({root_t[0]: .2f}, {root_t[1]: .2f}, {root_t[2]: .2f}) | "
            f"dist: {distance_to_target: .2f}"
        )

        time.sleep(SLEEP_SEC)

if __name__ == "__main__":
    main()