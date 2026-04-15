#!/usr/bin/env python3

import math
import time
from pxr import Usd, UsdGeom, Gf, Sdf

import omni.replicator.core as rep

# ============================================================
# CONFIG
# ============================================================

STAGE_PATH = "/mnt/macross/DATA_ENGINEERING/USD_PROJECTS/project_07_ai_driven_digital_twin_system/usd/runtime_v03.usda"

AGENT_ROOT_PATH = "/World/AgentRoot"
CAMERA_PATH     = "/World/AgentRoot/AgentCamera"
TARGET_PATH     = "/World/EnvWrapper/Environment/Warning_Light"

# ---------- Semantic labeling ----------
TARGET_CLASS_NAME = "warning_light"

# ---------- Deterministic startup ----------
START_ROOT_YAW_DEG   = 147.73
START_ROOT_TRANSLATE = Gf.Vec3d(0.0, 8.0, 20.0)

# ---------- Detection / state ----------
VISIBLE_ALIGNMENT_THRESHOLD = 0.80   # keep for debug/reference if desired
LOSE_TARGET_THRESHOLD       = 0.70   # keep for debug/reference if desired

# ---------- Patrol ----------
SCAN_SPEED_DEG_PER_STEP = 1.5
SCAN_LIMIT_DEG          = 45.0

# ---------- Lock controller ----------
FOCUS_TEST_STEP_DEG = 1.0
LOCK_ALIGNMENT_EPS  = 0.0005

# ---------- Movement / arrival ----------
MOVE_ALIGNMENT_THRESHOLD = 0.95
MOVE_STEP_UNITS          = 0.25

ARRIVAL_DISTANCE         = 10.0
HOLD_EXIT_DISTANCE       = 11.0

# ---------- BBox visibility thresholds ----------
MIN_BBOX_WIDTH  = 8.0
MIN_BBOX_HEIGHT = 8.0
MIN_BBOX_AREA   = 120.0

# ---------- Loop ----------
SLEEP_SEC = 0.05

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

def horizontal_distance_xz(a, b):
    dx = float(a[0] - b[0])
    dz = float(a[2] - b[2])
    return math.sqrt(dx * dx + dz * dz)

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
    # Debug only
    f = safe_normalize(flatten_y(forward_world))
    t = safe_normalize(flatten_y(to_target_world))
    if f is None or t is None:
        return 0.0

    dot_val = clamp(Gf.Dot(f, t), -1.0, 1.0)
    cross_y = Gf.Cross(f, t)[1]
    err_rad = math.atan2(cross_y, dot_val)
    return math.degrees(err_rad)

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

def reset_agent_root_pose(agent_root_prim, yaw_deg, translate_vec):
    set_root_yaw_deg(agent_root_prim, yaw_deg)
    set_root_translate(agent_root_prim, translate_vec)

def get_alignment_for_yaw(agent_root_prim, camera_prim, target_prim, yaw_deg):
    set_root_yaw_deg(agent_root_prim, wrap_angle_deg(yaw_deg))
    cache = UsdGeom.XformCache()
    return measure_alignment(cache, camera_prim, target_prim)

def move_root_toward_target_xz(agent_root_prim, root_pos, target_pos, step_units):
    dx = float(target_pos[0] - root_pos[0])
    dz = float(target_pos[2] - root_pos[2])

    length = math.sqrt(dx * dx + dz * dz)
    if length < 1e-8:
        return

    dir_x = dx / length
    dir_z = dz / length

    current_t = get_current_root_translate(agent_root_prim)

    next_t = Gf.Vec3d(
        current_t[0] + dir_x * step_units,
        current_t[1],
        current_t[2] + dir_z * step_units
    )

    set_root_translate(agent_root_prim, next_t)

# ============================================================
# SEMANTIC LABELING
# ============================================================

def apply_semantic_label(stage, prim_path, class_name):
    """
    Minimal semantic labeling helper.
    Depending on your build, this may need adaptation.
    This authors a 'class' semantic label onto the target prim.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Cannot label invalid prim: {prim_path}")

    # Common Omniverse semantic authoring pattern:
    sem_attr = prim.CreateAttribute("semantic:class", Sdf.ValueTypeNames.String)
    sem_attr.Set(class_name)

# ============================================================
# REPLICATOR / ANNOTATION SETUP
# ============================================================

def setup_bbox_visibility_pipeline(camera_path):
    """
    Creates:
    - render product from AgentCamera
    - bbox annotator

    NOTE:
    The exact bbox annotator name may vary by build.
    Try:
      'bounding_box_2d_tight'
    If that fails in your build, use the available 2D bbox annotator name.
    """
    render_product = rep.create.render_product(camera_path, (1280, 720))

    bbox_annotator = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
    bbox_annotator.attach([render_product])

    return render_product, bbox_annotator

def parse_bbox_entries(bbox_data):
    """
    Normalize annotator output into iterable records.

    Different builds may return different shapes. Start simple and inspect output.
    """
    if bbox_data is None:
        return []

    # Common pattern: dict with 'data'
    if isinstance(bbox_data, dict) and "data" in bbox_data:
        return bbox_data["data"]

    # Sometimes annotator returns direct list-like data
    if isinstance(bbox_data, (list, tuple)):
        return bbox_data

    return []

def extract_bbox_info(entry):
    """
    Build-specific adapter.
    Update field names once you inspect your actual bbox output.

    Expected return:
      semantic_label, x_min, y_min, x_max, y_max
    """
    # ---------- PLACEHOLDER ----------
    # Replace these keys with your build's actual bbox fields after first print.
    semantic_label = entry.get("semanticLabel", None)
    x_min = float(entry.get("x_min", 0.0))
    y_min = float(entry.get("y_min", 0.0))
    x_max = float(entry.get("x_max", 0.0))
    y_max = float(entry.get("y_max", 0.0))
    return semantic_label, x_min, y_min, x_max, y_max

def bbox_visible_for_target(bbox_data, target_class_name):
    """
    Returns:
      target_visible, best_bbox_area, bbox_count
    """
    entries = parse_bbox_entries(bbox_data)

    best_area = 0.0
    match_count = 0

    for entry in entries:
        try:
            semantic_label, x_min, y_min, x_max, y_max = extract_bbox_info(entry)
        except Exception:
            continue

        if semantic_label != target_class_name:
            continue

        w = max(0.0, x_max - x_min)
        h = max(0.0, y_max - y_min)
        area = w * h

        if w >= MIN_BBOX_WIDTH and h >= MIN_BBOX_HEIGHT and area >= MIN_BBOX_AREA:
            match_count += 1
            best_area = max(best_area, area)

    return (match_count > 0), best_area, match_count

# ============================================================
# MAIN
# ============================================================

def main():
    stage = get_stage(STAGE_PATH)

    agent_root_prim = get_prim(stage, AGENT_ROOT_PATH)
    camera_prim     = get_prim(stage, CAMERA_PATH)
    target_prim     = get_prim(stage, TARGET_PATH)

    # Label target for annotation lookup
    apply_semantic_label(stage, TARGET_PATH, TARGET_CLASS_NAME)
    stage.Save()

    # Reset startup pose
    reset_agent_root_pose(agent_root_prim, START_ROOT_YAW_DEG, START_ROOT_TRANSLATE)
    stage.Save()

    # Set up Replicator bbox pipeline
    render_product, bbox_annotator = setup_bbox_visibility_pipeline(CAMERA_PATH)

    initial_yaw_deg = get_current_root_yaw_deg(agent_root_prim)
    scan_dir = 1.0
    state = "patrol"

    print("========================================")
    print("Agent occlusion-aware controller starting")
    print(f"Stage:  {STAGE_PATH}")
    print(f"Agent:  {AGENT_ROOT_PATH}")
    print(f"Camera: {CAMERA_PATH}")
    print(f"Target: {TARGET_PATH}")
    print(f"Target class: {TARGET_CLASS_NAME}")
    print(f"Initial root yaw: {initial_yaw_deg:.3f}")
    print("========================================")

    while True:
        cache = UsdGeom.XformCache()

        root_pos   = get_world_position(cache, agent_root_prim)
        camera_pos = get_world_position(cache, camera_prim)
        target_pos = get_world_position(cache, target_prim)

        to_target   = target_pos - camera_pos
        to_target_n = safe_normalize(to_target)
        camera_fwd  = get_camera_forward_world(cache, camera_prim)

        if to_target_n is None or camera_fwd is None:
            print("Bad vector state; skipping frame")
            time.sleep(SLEEP_SEC)
            continue

        alignment = full_view_alignment(camera_fwd, to_target_n)
        yaw_error_deg = signed_horizontal_error_deg(camera_fwd, to_target_n)  # debug only
        current_root_yaw = get_current_root_yaw_deg(agent_root_prim)
        root_dist = horizontal_distance_xz(root_pos, target_pos)
        cam_dist  = horizontal_distance_xz(camera_pos, target_pos)

        # ----------------------------------------------------
        # NEW: camera-truth visibility
        # ----------------------------------------------------
        bbox_data = bbox_annotator.get_data()

        # First run: inspect actual output structure in your build
        # Uncomment once, inspect, then comment it back out:
        print("RAW BBOX DATA:", bbox_data)

        target_visible, best_bbox_area, bbox_count = bbox_visible_for_target(
            bbox_data,
            TARGET_CLASS_NAME
        )

        # ----------------------------------------------------
        # STATE MACHINE
        # ----------------------------------------------------
        if state == "patrol":
            if target_visible:
                state = "focus_target"
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
            if not target_visible:
                state = "patrol"
            else:
                current_yaw = current_root_yaw
                base_alignment = alignment

                left_yaw  = wrap_angle_deg(current_yaw - FOCUS_TEST_STEP_DEG)
                right_yaw = wrap_angle_deg(current_yaw + FOCUS_TEST_STEP_DEG)

                left_alignment  = get_alignment_for_yaw(agent_root_prim, camera_prim, target_prim, left_yaw)
                right_alignment = get_alignment_for_yaw(agent_root_prim, camera_prim, target_prim, right_yaw)

                # Restore current yaw before choosing
                set_root_yaw_deg(agent_root_prim, current_yaw)

                best_alignment = base_alignment
                next_yaw = current_yaw

                if left_alignment > best_alignment + LOCK_ALIGNMENT_EPS:
                    best_alignment = left_alignment
                    next_yaw = left_yaw

                if right_alignment > best_alignment + LOCK_ALIGNMENT_EPS:
                    best_alignment = right_alignment
                    next_yaw = right_yaw

                set_root_yaw_deg(agent_root_prim, next_yaw)

                if root_dist <= ARRIVAL_DISTANCE:
                    state = "hold_at_target"
                elif best_alignment >= MOVE_ALIGNMENT_THRESHOLD:
                    move_root_toward_target_xz(agent_root_prim, root_pos, target_pos, MOVE_STEP_UNITS)

                stage.Save()

        elif state == "hold_at_target":
            if not target_visible:
                state = "patrol"
            else:
                current_yaw = current_root_yaw
                base_alignment = alignment

                left_yaw  = wrap_angle_deg(current_yaw - FOCUS_TEST_STEP_DEG)
                right_yaw = wrap_angle_deg(current_yaw + FOCUS_TEST_STEP_DEG)

                left_alignment  = get_alignment_for_yaw(agent_root_prim, camera_prim, target_prim, left_yaw)
                right_alignment = get_alignment_for_yaw(agent_root_prim, camera_prim, target_prim, right_yaw)

                set_root_yaw_deg(agent_root_prim, current_yaw)

                best_alignment = base_alignment
                next_yaw = current_yaw

                if left_alignment > best_alignment + LOCK_ALIGNMENT_EPS:
                    best_alignment = left_alignment
                    next_yaw = left_yaw

                if right_alignment > best_alignment + LOCK_ALIGNMENT_EPS:
                    best_alignment = right_alignment
                    next_yaw = right_yaw

                set_root_yaw_deg(agent_root_prim, next_yaw)

                if root_dist > HOLD_EXIT_DISTANCE:
                    state = "focus_target"

                stage.Save()

        # ----------------------------------------------------
        # DEBUG
        # ----------------------------------------------------
        root_t = get_current_root_translate(agent_root_prim)
        print(
            f"state: {state:14s} | "
            f"target_visible: {str(target_visible):5s} | "
            f"bbox_count: {bbox_count:2d} | "
            f"bbox_area: {best_bbox_area:8.1f} | "
            f"alignment: {alignment: .3f} | "
            f"yaw_error_deg: {yaw_error_deg: .2f} | "
            f"root_yaw: {get_current_root_yaw_deg(agent_root_prim): .2f} | "
            f"root_t: ({root_t[0]: .2f}, {root_t[1]: .2f}, {root_t[2]: .2f}) | "
            f"root_dist: {root_dist: .2f} | "
            f"cam_dist: {cam_dist: .2f}"
        )

        time.sleep(SLEEP_SEC)

if __name__ == "__main__":
    main()