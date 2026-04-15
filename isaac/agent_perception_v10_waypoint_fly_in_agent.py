from pxr import UsdGeom, Gf, Semantics
import omni
import omni.replicator.core as rep
import asyncio
import math

# ============================================================
# CONFIG
# ============================================================

AGENT_ROOT_PATH = "/World/AgentRoot"
CAMERA_PATH = "/World/AgentRoot/AgentCamera"

TARGET_CLASS_NAME = "warning_light"

TARGET_PRIMS = [
    "/World/EnvWrapper/Environment/Warning_Light/Xform/Warning_Light/Glass_Dome/Sphere",
    "/World/EnvWrapper/Environment/Warning_Light/Xform/Warning_Light/Base/Cylinder",
]

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
CENTER_X = IMAGE_WIDTH * 0.5

# ------------------------------------------------------------
# FIXED START POSE
# Based on the pose you showed from the far warehouse edge.
# If heading is flipped, ONLY change START_ROOT_YAW_DEG.
# ------------------------------------------------------------
START_ROOT_TRANSLATE = Gf.Vec3d(50.3, 2.2, 3.4)
START_ROOT_YAW_DEG = 86.0

# ------------------------------------------------------------
# Overview patrol waypoints
# These are wide-search / strategic positions.
# Waypoint 0 matches the fixed start pose.
# ------------------------------------------------------------
OVERVIEW_WAYPOINTS = [
    (50.3, 2.2, 3.4),   # fixed far-edge start
    (36.0, 2.2, 4.0),
    (22.0, 2.2, 0.0),
    (8.0,  2.2, 0.0),
    (-6.0, 2.2, 0.0),
]

# Motion
# MOVE_STEP = 0.20
YAW_STEP = 2.0
WAYPOINT_ARRIVAL_DIST = 1.5
WAYPOINT_FACE_TOLERANCE_DEG = 6.0

# Full scan
SCAN_STEP_DEG = 6.0
SCAN_TOTAL_DEG = 360.0

# Far-search candidate detection
MIN_SEARCH_BBOX_AREA = 60.0
MIN_SEARCH_VISIBILITY = 0.20
SEARCH_EDGE_MARGIN_PX = 10.0
SEARCH_CONFIRM_FRAMES = 2

# Near-lock detection
MIN_TRACK_AREA = 300.0
MIN_TRACK_VISIBILITY = 0.50
TRACK_EDGE_MARGIN_PX = 100.0

# Track / approach hysteresis
TRACK_TO_APPROACH_DEADBAND = 40.0
APPROACH_TO_TRACK_DEADBAND = 90.0

# Arrival / hold
ARRIVAL_AREA = 1200.0
HOLD_EXIT_AREA = 900.0

# Lost target handling
TRACK_LOST_MAX_FRAMES = 20

# Fly-in investigation
INVESTIGATE_FLY_DISTANCE = 10.0
INVESTIGATE_ARRIVAL_DIST = 1.0

# Lock stability
TRACK_CENTER_CONFIRM_FRAMES = 3
CX_SMOOTH_ALPHA = 0.25

# New latch / dwell control
MIN_APPROACH_FRAMES = 12

# Close-range stop / inspect
STOP_AT_INVESTIGATE_DIST = 1.25
SLOWDOWN_AT_INVESTIGATE_DIST = 3.0

MOVE_STEP_FAST = 0.20
MOVE_STEP_SLOW = 0.08

INSPECT_HOLD_FRAMES = 40
INSPECT_YAW_TOLERANCE = 20.0

# ============================================================
# GLOBALS
# ============================================================

stage = None
bbox_annotator = None
render_product = None
task = None

# ============================================================
# HELPERS
# ============================================================

def wrap_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle

def get_prim(path):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Invalid prim: {path}")
    return prim

def get_or_create_translate_op(xformable):
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op
    return xformable.AddTranslateOp()

def get_or_create_rotate_y_op(xformable):
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateY:
            return op
    return xformable.AddRotateYOp()

def get_position(prim):
    xformable = UsdGeom.Xformable(prim)
    t = get_or_create_translate_op(xformable).Get()
    return Gf.Vec3d(t[0], t[1], t[2]) if t else Gf.Vec3d(0.0, 0.0, 0.0)

def set_position(prim, pos):
    xformable = UsdGeom.Xformable(prim)
    get_or_create_translate_op(xformable).Set(Gf.Vec3d(pos[0], pos[1], pos[2]))

def get_yaw_deg(prim):
    xformable = UsdGeom.Xformable(prim)
    val = get_or_create_rotate_y_op(xformable).Get()
    return float(val) if val is not None else 0.0

def set_yaw_deg(prim, yaw_deg):
    xformable = UsdGeom.Xformable(prim)
    get_or_create_rotate_y_op(xformable).Set(float(wrap_angle_deg(yaw_deg)))

def distance_xz(a, b):
    dx = float(a[0] - b[0])
    dz = float(a[2] - b[2])
    return math.sqrt(dx * dx + dz * dz)

def desired_yaw_to_point(from_pos, to_pos):
    dx = float(to_pos[0] - from_pos[0])
    dz = float(to_pos[2] - from_pos[2])
    return math.degrees(math.atan2(dx, dz))

def move_forward(prim, yaw_deg, step):
    """
    Forward convention corrected for your rig.
    """
    pos = get_position(prim)
    rad = math.radians(yaw_deg)

    dx = math.sin(rad) * step
    dz = math.cos(rad) * step

    set_position(prim, Gf.Vec3d(pos[0] + dx, pos[1], pos[2] + dz))

def apply_semantics(prim, class_name):
    sem_api = Semantics.SemanticsAPI.Apply(prim, "Semantics")
    sem_api.CreateSemanticTypeAttr().Set("class")
    sem_api.CreateSemanticDataAttr().Set(class_name)

def label_target_prims():
    for path in TARGET_PRIMS:
        prim = get_prim(path)
        apply_semantics(prim, TARGET_CLASS_NAME)
        print("Labeled:", path, "->", TARGET_CLASS_NAME)

def setup_bbox_pipeline():
    global render_product, bbox_annotator

    # Best effort cleanup in case this file gets rerun in the same session
    try:
        if bbox_annotator is not None and render_product is not None:
            bbox_annotator.detach([render_product])
    except Exception as e:
        print("BBox detach cleanup skipped:", e)

    try:
        if render_product is not None:
            rep.destroy.render_product(render_product)
    except Exception as e:
        print("Render product cleanup skipped:", e)

    render_product = rep.create.render_product(CAMERA_PATH, (IMAGE_WIDTH, IMAGE_HEIGHT))
    bbox_annotator = rep.AnnotatorRegistry.get_annotator(
        "bounding_box_2d_tight",
        init_params={"semanticTypes": ["class"]}
    )
    bbox_annotator.attach([render_product])

    print("BBox pipeline ready:", IMAGE_WIDTH, "x", IMAGE_HEIGHT)

def parse_best_bbox(bbox_data):
    """
    Merge all target boxes into one union bbox.

    This is critical because the warning light is composed of multiple
    labeled mesh parts, and choosing a single 'best' box can cause the
    controller to flip between components frame to frame.
    """
    arr = bbox_data["data"]

    if len(arr) == 0:
        return None

    valid = []

    for entry in arr:
        semantic_id, x1, y1, x2, y2, occ = entry

        w = max(0.0, float(x2) - float(x1))
        h = max(0.0, float(y2) - float(y1))
        area = w * h
        vis = 1.0 - float(occ)

        if area < 1.0:
            continue

        valid.append({
            "x1": int(x1),
            "y1": int(y1),
            "x2": int(x2),
            "y2": int(y2),
            "area": float(area),
            "visibility": float(vis),
        })

    if not valid:
        return None

    # Union all boxes into one stable target bbox
    union_x1 = min(b["x1"] for b in valid)
    union_y1 = min(b["y1"] for b in valid)
    union_x2 = max(b["x2"] for b in valid)
    union_y2 = max(b["y2"] for b in valid)

    union_w = max(0.0, float(union_x2) - float(union_x1))
    union_h = max(0.0, float(union_y2) - float(union_y1))
    union_area = union_w * union_h
    union_cx = (float(union_x1) + float(union_x2)) * 0.5

    # Use the best visibility among visible parts as a practical proxy
    union_visibility = max(b["visibility"] for b in valid)

    return {
        "x1": int(union_x1),
        "y1": int(union_y1),
        "x2": int(union_x2),
        "y2": int(union_y2),
        "cx": union_cx,
        "area": float(union_area),
        "visibility": float(union_visibility),
    }

def is_search_candidate(bbox):
    if bbox is None:
        return False

    if bbox["area"] < MIN_SEARCH_BBOX_AREA:
        return False
    if bbox["visibility"] < MIN_SEARCH_VISIBILITY:
        return False
    if bbox["cx"] < SEARCH_EDGE_MARGIN_PX:
        return False
    if bbox["cx"] > (IMAGE_WIDTH - SEARCH_EDGE_MARGIN_PX):
        return False

    return True

def is_lock_candidate(bbox):
    if bbox is None:
        return False

    if bbox["area"] < MIN_TRACK_AREA:
        return False
    if bbox["visibility"] < MIN_TRACK_VISIBILITY:
        return False
    if bbox["cx"] < TRACK_EDGE_MARGIN_PX:
        return False
    if bbox["cx"] > (IMAGE_WIDTH - TRACK_EDGE_MARGIN_PX):
        return False

    return True

def make_point_ahead(current_pos, yaw_deg, distance):
    rad = math.radians(yaw_deg)
    dx = math.sin(rad) * distance
    dz = math.cos(rad) * distance
    return Gf.Vec3d(current_pos[0] + dx, current_pos[1], current_pos[2] + dz)


def fmt_vec3(v):
    return f"({v[0]:.2f}, {v[1]:.2f}, {v[2]:.2f})"


def yaw_error_to_point(from_pos, current_yaw_deg, to_pos):
    desired = desired_yaw_to_point(from_pos, to_pos)
    return wrap_angle_deg(desired - current_yaw_deg)

# ============================================================
# MAIN
# ============================================================

print("=== V10 STARTED ===")

print("Render resolution:", IMAGE_WIDTH, "x", IMAGE_HEIGHT)
print("Start pose:", START_ROOT_TRANSLATE, "| yaw:", START_ROOT_YAW_DEG)


async def main():
    global stage
    stage = omni.usd.get_context().get_stage()

    root = get_prim(AGENT_ROOT_PATH)

    label_target_prims()
    setup_bbox_pipeline()

    # Fixed start pose
    set_position(root, START_ROOT_TRANSLATE)
    set_yaw_deg(root, START_ROOT_YAW_DEG)

    overview_index = 0
    state = "scan_overview"

    scan_accum_deg = 0.0
    search_confirm_count = 0
    lost_target_frames = 0

    candidate_yaw = None
    investigate_point = None

    smoothed_cx = None
    centered_frames = 0

    approach_frames = 0

    inspect_hold_frames = 0

    target_locked_final = False

    print("=== V10 STARTED ===")
    print("Fixed start translate:", START_ROOT_TRANSLATE)
    print("Fixed start yaw:", START_ROOT_YAW_DEG)

    while True:
        await rep.orchestrator.step_async()

        bbox_raw = bbox_annotator.get_data()
        bbox = parse_best_bbox(bbox_raw)

        raw_visible = bbox is not None
        search_candidate = is_search_candidate(bbox)
        lock_candidate = is_lock_candidate(bbox)

        pos = get_position(root)
        yaw = get_yaw_deg(root)

        bbox_error = None
        bbox_area = 0.0

        if bbox:
            bbox_area = bbox["area"]

            if smoothed_cx is None:
                smoothed_cx = bbox["cx"]
            else:
                smoothed_cx = (1.0 - CX_SMOOTH_ALPHA) * smoothed_cx + CX_SMOOTH_ALPHA * bbox["cx"]

            bbox_error = smoothed_cx - CENTER_X
        else:
            smoothed_cx = None

        overview_wp = Gf.Vec3d(*OVERVIEW_WAYPOINTS[overview_index])
        overview_dist = distance_xz(pos, overview_wp)
        yaw_to_overview = wrap_angle_deg(desired_yaw_to_point(pos, overview_wp) - yaw)

        investigate_dist = None
        yaw_to_investigate = None

        if investigate_point is not None:
            investigate_dist = distance_xz(pos, investigate_point)
            yaw_to_investigate = wrap_angle_deg(
                desired_yaw_to_point(pos, investigate_point) - yaw
            )

        reached_investigate_point = (
            investigate_dist is not None and investigate_dist <= STOP_AT_INVESTIGATE_DIST
        )

        # ====================================================
        # TURN TO OVERVIEW WAYPOINT
        # ====================================================
        # ----------------------------------------------------
        # HARD ARRIVAL OVERRIDE:
        # if we've reached the investigate point, stop there,
        # even if the target flickers out on this frame.
        # ----------------------------------------------------
        if (
            state in ["move_to_investigate", "track_target", "approach_target"]
            and reached_investigate_point
        ):
            target_locked_final = True
            inspect_hold_frames = 0
            centered_frames = 0
            approach_frames = 0
            state = "inspect_hold"

        if state == "turn_to_overview":
            if abs(yaw_to_overview) <= WAYPOINT_FACE_TOLERANCE_DEG:
                state = "move_to_overview"
            else:
                if yaw_to_overview > 0:
                    set_yaw_deg(root, yaw + YAW_STEP)
                else:
                    set_yaw_deg(root, yaw - YAW_STEP)

        # ====================================================
        # MOVE TO OVERVIEW WAYPOINT
        # ignore detections while moving
        # ====================================================
        elif state == "move_to_overview":
            if overview_dist <= WAYPOINT_ARRIVAL_DIST:
                state = "scan_overview"
                scan_accum_deg = 0.0
                search_confirm_count = 0
            else:
                if abs(yaw_to_overview) > WAYPOINT_FACE_TOLERANCE_DEG:
                    state = "turn_to_overview"
                else:
                    move_forward(root, yaw, MOVE_STEP_FAST)

        # ====================================================
        # SCAN OVERVIEW
        # act on search candidate only here
        # ====================================================
        elif state == "scan_overview":
            if search_candidate:
                search_confirm_count += 1

                if search_confirm_count >= SEARCH_CONFIRM_FRAMES:
                    candidate_yaw = yaw
                    investigate_point = make_point_ahead(pos, candidate_yaw, INVESTIGATE_FLY_DISTANCE)
                    state = "turn_to_investigate"
                    search_confirm_count = 0
            else:
                search_confirm_count = 0

                if scan_accum_deg >= SCAN_TOTAL_DEG:
                    overview_index = (overview_index + 1) % len(OVERVIEW_WAYPOINTS)
                    state = "turn_to_overview"
                    scan_accum_deg = 0.0
                else:
                    set_yaw_deg(root, yaw + SCAN_STEP_DEG)
                    scan_accum_deg += abs(SCAN_STEP_DEG)

        # ====================================================
        # TURN TO INVESTIGATE POINT
        # ====================================================
        elif state == "turn_to_investigate":
            if investigate_point is None:
                state = "scan_overview"
            else:
                if abs(yaw_to_investigate) <= WAYPOINT_FACE_TOLERANCE_DEG:
                    state = "move_to_investigate"
                else:
                    if yaw_to_investigate > 0:
                        set_yaw_deg(root, yaw + YAW_STEP)
                    else:
                        set_yaw_deg(root, yaw - YAW_STEP)

        # ====================================================
        # MOVE TO INVESTIGATE POINT
        # if lock-quality detection appears, escalate immediately
        # ====================================================
        elif state == "move_to_investigate":
            if investigate_point is None:
                state = "scan_overview"

            elif reached_investigate_point:
                inspect_hold_frames = 0
                state = "inspect_hold"

            elif lock_candidate:
                state = "track_target"
                lost_target_frames = 0

            else:
                if abs(yaw_to_investigate) > WAYPOINT_FACE_TOLERANCE_DEG:
                    state = "turn_to_investigate"
                else:
                    if investigate_dist is not None and investigate_dist <= SLOWDOWN_AT_INVESTIGATE_DIST:
                        move_forward(root, yaw, MOVE_STEP_SLOW)
                    else:
                        move_forward(root, yaw, MOVE_STEP_FAST)

        # ====================================================
        # SCAN INVESTIGATE
        # tighter scan near candidate region
        # ====================================================
        elif state == "scan_investigate":
            if lock_candidate:
                state = "track_target"
                lost_target_frames = 0
            else:
                if scan_accum_deg >= SCAN_TOTAL_DEG:
                    investigate_point = None
                    candidate_yaw = None
                    state = "turn_to_overview"
                    scan_accum_deg = 0.0
                else:
                    set_yaw_deg(root, yaw + SCAN_STEP_DEG)
                    scan_accum_deg += abs(SCAN_STEP_DEG)


        elif state == "inspect_hold":
            inspect_hold_frames += 1

            if target_locked_final:

                # CASE 1: target visible → fine alignment
                if raw_visible and bbox_error is not None:
                    if bbox_error > INSPECT_YAW_TOLERANCE:
                        set_yaw_deg(root, yaw + 0.3 * YAW_STEP)
                    elif bbox_error < -INSPECT_YAW_TOLERANCE:
                        set_yaw_deg(root, yaw - 0.3 * YAW_STEP)

                # CASE 2: target NOT visible → ACTIVE RECOVERY SCAN
                else:
                    # Slow scan to reacquire
                    set_yaw_deg(root, yaw + 0.5 * YAW_STEP)

                continue


        # ====================================================
        # TRACK TARGET
        # rotate only until tightly centered
        # ====================================================
        elif state == "track_target":
            if not raw_visible:
                lost_target_frames += 1
                centered_frames = 0

                if lost_target_frames > TRACK_LOST_MAX_FRAMES:
                    lost_target_frames = 0
                    state = "scan_investigate"
            else:
                lost_target_frames = 0

                if bbox_error is not None:
                    abs_err = abs(bbox_error)

                    # Clearly off-center: rotate and reset confirmation
                    if abs_err >= APPROACH_TO_TRACK_DEADBAND:
                        if bbox_error > 0:
                            set_yaw_deg(root, yaw + YAW_STEP)
                        else:
                            set_yaw_deg(root, yaw - YAW_STEP)
                        centered_frames = 0

                    # Good enough to count toward approach
                    elif abs_err <= TRACK_TO_APPROACH_DEADBAND:
                        centered_frames += 1

                        if centered_frames >= TRACK_CENTER_CONFIRM_FRAMES:
                            centered_frames = 0
                            approach_frames = 0
                            state = "approach_target"

                    # Middle zone: make tiny correction, but DO NOT reset counter
                    else:
                        if bbox_error > 0:
                            set_yaw_deg(root, yaw + 0.5 * YAW_STEP)
                        else:
                            set_yaw_deg(root, yaw - 0.5 * YAW_STEP)

        # ====================================================
        # APPROACH TARGET
        # move only if target stays acceptably centered
        # ====================================================
        elif state == "approach_target":
            if not raw_visible:
                lost_target_frames += 1

                if lost_target_frames > TRACK_LOST_MAX_FRAMES:
                    lost_target_frames = 0
                    approach_frames = 0
                    centered_frames = 0
                    state = "scan_investigate"
            else:
                lost_target_frames = 0
                approach_frames += 1

                if bbox_error is not None:
                    abs_err = abs(bbox_error)

                    # PRIMARY STOP CONDITION:
                    # If we've reached the investigate point, stop and hold/inspect now.
                    if investigate_dist is not None and investigate_dist <= STOP_AT_INVESTIGATE_DIST:
                        approach_frames = 0
                        centered_frames = 0
                        state = "hold_target"

                    # Secondary visual stop condition
                    elif bbox_area >= ARRIVAL_AREA:
                        approach_frames = 0
                        centered_frames = 0
                        state = "hold_target"

                    else:
                        # During the initial commitment window, do not fall back.
                        if approach_frames < MIN_APPROACH_FRAMES:
                            if bbox_error > TRACK_TO_APPROACH_DEADBAND:
                                set_yaw_deg(root, yaw + 0.5 * YAW_STEP)
                            elif bbox_error < -TRACK_TO_APPROACH_DEADBAND:
                                set_yaw_deg(root, yaw - 0.5 * YAW_STEP)

                            if investigate_dist is not None and investigate_dist <= SLOWDOWN_AT_INVESTIGATE_DIST:
                                move_forward(root, yaw, MOVE_STEP_SLOW)
                            else:
                                move_forward(root, yaw, MOVE_STEP_FAST)

                        else:
                            # After the commitment window, only fall back if badly off-center
                            if abs_err >= APPROACH_TO_TRACK_DEADBAND:
                                approach_frames = 0
                                centered_frames = 0

                                if bbox_error > 0:
                                    set_yaw_deg(root, yaw + YAW_STEP)
                                else:
                                    set_yaw_deg(root, yaw - YAW_STEP)

                                state = "track_target"
                            else:
                                if bbox_error > TRACK_TO_APPROACH_DEADBAND:
                                    set_yaw_deg(root, yaw + 0.5 * YAW_STEP)
                                elif bbox_error < -TRACK_TO_APPROACH_DEADBAND:
                                    set_yaw_deg(root, yaw - 0.5 * YAW_STEP)

                                if investigate_dist is not None and investigate_dist <= SLOWDOWN_AT_INVESTIGATE_DIST:
                                    move_forward(root, yaw, MOVE_STEP_SLOW)
                                else:
                                    move_forward(root, yaw, MOVE_STEP_FAST)

        # ====================================================
        # HOLD TARGET
        # ====================================================
        elif state == "hold_target":
            if not raw_visible:
                lost_target_frames += 1
                if lost_target_frames > TRACK_LOST_MAX_FRAMES:
                    lost_target_frames = 0
                    state = "scan_investigate"
            else:
                lost_target_frames = 0

                # Tiny orientation correction only
                if bbox_error is not None:
                    if bbox_error > APPROACH_TO_TRACK_DEADBAND:
                        set_yaw_deg(root, yaw + 0.5 * YAW_STEP)
                    elif bbox_error < -APPROACH_TO_TRACK_DEADBAND:
                        set_yaw_deg(root, yaw - 0.5 * YAW_STEP)

                # Only leave hold if target becomes genuinely poor again
                if not target_locked_final:
                    if (bbox_area < HOLD_EXIT_AREA):
                        state = "track_target"

        current_step = None
        if state == "approach_target":
            if investigate_dist is not None and investigate_dist <= SLOWDOWN_AT_INVESTIGATE_DIST:
                current_step = MOVE_STEP_SLOW
            else:
                current_step = MOVE_STEP_FAST

        print(
            f"state: {state} | "
            f"overview_idx: {overview_index} | "
            f"raw_visible: {raw_visible} | "
            f"search_candidate: {search_candidate} | "
            f"lock_candidate: {lock_candidate} | "
            f"search_confirm: {search_confirm_count} | "
            f"lost_frames: {lost_target_frames} | "
            f"centered_frames: {centered_frames} | "
            f"approach_frames: {approach_frames} | "
            f"inspect_hold_frames: {inspect_hold_frames} | "
            f"reached_investigate: {reached_investigate_point} | "
            f"world_pos: {fmt_vec3(pos)} | "
            f"yaw: {yaw:.2f} | "
            f"smoothed_cx: {None if smoothed_cx is None else round(smoothed_cx, 2)} | "
            f"bbox_error: {None if bbox_error is None else round(bbox_error, 2)} | "
            f"overview_dist: {overview_dist:.2f} | "
            f"investigate_dist: {investigate_dist if investigate_dist is not None else 'None'} | "
            f"step: {current_step} | "
            f"bbox: {bbox}"
        )

        await asyncio.sleep(0.05)

# ============================================================
# ENTRY
# ============================================================

task = asyncio.ensure_future(main())