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

# Motion
MOVE_STEP = 0.20
YAW_STEP = 2.0
WAYPOINT_ARRIVAL_DIST = 1.25
WAYPOINT_FACE_TOLERANCE_DEG = 6.0

# Scan
SCAN_STEP_DEG = 6.0
SCAN_TOTAL_DEG = 360.0

# Detection quality
MIN_BBOX_WIDTH = 8.0
MIN_BBOX_HEIGHT = 8.0
MIN_BBOX_AREA = 120.0
MIN_TRACK_AREA = 300.0
EDGE_MARGIN_PX = 100.0
VISIBILITY_THRESHOLD = 0.50

# Tracking
TRACK_CENTER_DEADBAND = 25.0
APPROACH_CENTER_DEADBAND = 60.0
TRACK_LOST_MAX_FRAMES = 20

# Arrival by bbox size
ARRIVAL_AREA = 1800.0
HOLD_EXIT_AREA = 1400.0

# Waypoints
WAYPOINTS = [
    (0.0,   8.0,  20.0),
    (0.0,   8.0,   8.0),
    (12.0,  8.0,   8.0),
    (-12.0, 8.0,   8.0),
    (12.0,  8.0,  -6.0),
    (-12.0, 8.0,  -6.0),
    (0.0,   8.0, -10.0),
]

# ============================================================
# GLOBALS
# ============================================================

stage = None
bbox_annotator = None
render_product = None

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
    pos = get_position(prim)
    rad = math.radians(yaw_deg)

    # Corrected forward convention for your rig
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

    render_product = rep.create.render_product(CAMERA_PATH, (IMAGE_WIDTH, IMAGE_HEIGHT))
    bbox_annotator = rep.AnnotatorRegistry.get_annotator(
        "bounding_box_2d_tight",
        init_params={"semanticTypes": ["class"]}
    )
    bbox_annotator.attach([render_product])

def parse_best_bbox(bbox_data):
    arr = bbox_data["data"]

    if len(arr) == 0:
        return None

    best = None
    best_vis = -1.0

    for entry in arr:
        semantic_id, x1, y1, x2, y2, occ = entry

        w = max(0.0, float(x2) - float(x1))
        h = max(0.0, float(y2) - float(y1))
        area = w * h
        vis = 1.0 - float(occ)

        if w < MIN_BBOX_WIDTH:
            continue
        if h < MIN_BBOX_HEIGHT:
            continue
        if area < MIN_BBOX_AREA:
            continue

        if vis > best_vis:
            best_vis = vis
            best = (int(x1), int(y1), int(x2), int(y2), float(area), float(vis))

    return best

def is_usable_detection(bbox):
    if bbox is None:
        return False

    x1, y1, x2, y2, area, vis = bbox
    cx = (x1 + x2) * 0.5

    if area < MIN_TRACK_AREA:
        return False
    if cx < EDGE_MARGIN_PX:
        return False
    if cx > (IMAGE_WIDTH - EDGE_MARGIN_PX):
        return False
    if vis < VISIBILITY_THRESHOLD:
        return False

    return True

# ============================================================
# MAIN
# ============================================================

async def main():
    global stage
    stage = omni.usd.get_context().get_stage()

    root = get_prim(AGENT_ROOT_PATH)

    label_target_prims()
    setup_bbox_pipeline()

    waypoint_index = 0
    state = "turn_to_waypoint"

    scan_accum_deg = 0.0
    lost_target_frames = 0

    print("=== V09 STARTED ===")

    while True:
        await rep.orchestrator.step_async()

        bbox_raw = bbox_annotator.get_data()
        bbox = parse_best_bbox(bbox_raw)

        raw_visible = bbox is not None
        usable_visible = is_usable_detection(bbox)

        pos = get_position(root)
        yaw = get_yaw_deg(root)

        bbox_area = 0.0
        bbox_center_x = None
        bbox_error = None

        if bbox:
            x1, y1, x2, y2, area, vis = bbox
            bbox_area = area
            bbox_center_x = (x1 + x2) * 0.5
            bbox_error = bbox_center_x - CENTER_X

        wp = WAYPOINTS[waypoint_index]
        wp_pos = Gf.Vec3d(wp[0], wp[1], wp[2])
        wp_dist = distance_xz(pos, wp_pos)
        desired_yaw = desired_yaw_to_point(pos, wp_pos)
        yaw_to_wp_err = wrap_angle_deg(desired_yaw - yaw)

        # ====================================================
        # TURN TO WAYPOINT
        # ====================================================
        if state == "turn_to_waypoint":
            if abs(yaw_to_wp_err) <= WAYPOINT_FACE_TOLERANCE_DEG:
                state = "move_to_waypoint"
            else:
                if yaw_to_wp_err > 0:
                    set_yaw_deg(root, yaw + YAW_STEP)
                else:
                    set_yaw_deg(root, yaw - YAW_STEP)

        # ====================================================
        # MOVE TO WAYPOINT
        # IGNORE detections while moving
        # ====================================================
        elif state == "move_to_waypoint":
            if wp_dist <= WAYPOINT_ARRIVAL_DIST:
                state = "scan_at_waypoint"
                scan_accum_deg = 0.0
            else:
                if abs(yaw_to_wp_err) > WAYPOINT_FACE_TOLERANCE_DEG:
                    state = "turn_to_waypoint"
                else:
                    move_forward(root, yaw, MOVE_STEP)

        # ====================================================
        # SCAN AT WAYPOINT
        # ONLY here do we act on detections
        # ====================================================
        elif state == "scan_at_waypoint":
            if usable_visible:
                state = "track_target"
                lost_target_frames = 0
            else:
                if scan_accum_deg >= SCAN_TOTAL_DEG:
                    waypoint_index = (waypoint_index + 1) % len(WAYPOINTS)
                    state = "turn_to_waypoint"
                else:
                    set_yaw_deg(root, yaw + SCAN_STEP_DEG)
                    scan_accum_deg += abs(SCAN_STEP_DEG)

        # ====================================================
        # TRACK TARGET
        # rotate only until centered
        # ====================================================
        elif state == "track_target":
            if not raw_visible:
                lost_target_frames += 1
                if lost_target_frames > TRACK_LOST_MAX_FRAMES:
                    state = "scan_at_waypoint"
                    lost_target_frames = 0
            else:
                lost_target_frames = 0

                if bbox_error is not None:
                    if bbox_error > TRACK_TO_APPROACH_DEADBAND:
                        set_yaw_deg(root, yaw + YAW_STEP)
                    elif bbox_error < -TRACK_TO_APPROACH_DEADBAND:
                        set_yaw_deg(root, yaw - YAW_STEP)
                    else:
                        state = "approach_target"

        # ====================================================
        # APPROACH TARGET
        # ====================================================
        elif state == "approach_target":
            if not raw_visible:
                lost_target_frames += 1
                if lost_target_frames > TRACK_LOST_MAX_FRAMES:
                    state = "scan_at_waypoint"
                    lost_target_frames = 0
            else:
                lost_target_frames = 0

                if bbox_error is not None:
                    if bbox_error > APPROACH_TO_TRACK_DEADBAND:
                        set_yaw_deg(root, yaw + YAW_STEP)
                        state = "track_target"
                    elif bbox_error < -APPROACH_TO_TRACK_DEADBAND:
                        set_yaw_deg(root, yaw - YAW_STEP)
                        state = "track_target"
                    else:
                        if bbox_area >= ARRIVAL_AREA:
                            state = "hold_target"
                        else:
                            move_forward(root, yaw, MOVE_STEP)

        # ====================================================
        # HOLD TARGET
        # ====================================================
        elif state == "hold_target":
            if not raw_visible:
                lost_target_frames += 1
                if lost_target_frames > TRACK_LOST_MAX_FRAMES:
                    state = "scan_at_waypoint"
                    lost_target_frames = 0
            else:
                lost_target_frames = 0

                if bbox_error is not None:
                    if bbox_error > APPROACH_CENTER_DEADBAND:
                        set_yaw_deg(root, yaw + YAW_STEP)
                    elif bbox_error < -APPROACH_CENTER_DEADBAND:
                        set_yaw_deg(root, yaw - YAW_STEP)

                if bbox_area < HOLD_EXIT_AREA:
                    state = "track_target"

        print(
            f"state: {state} | "
            f"waypoint: {waypoint_index} | "
            f"raw_visible: {raw_visible} | "
            f"usable_visible: {usable_visible} | "
            f"lost_frames: {lost_target_frames} | "
            f"wp_dist: {wp_dist:.2f} | "
            f"yaw: {yaw:.2f} | "
            f"yaw_to_wp_err: {yaw_to_wp_err:.2f} | "
            f"bbox: {bbox}"
        )

        await asyncio.sleep(0.05)

# ============================================================
# ENTRY
# ============================================================

task = asyncio.ensure_future(main())