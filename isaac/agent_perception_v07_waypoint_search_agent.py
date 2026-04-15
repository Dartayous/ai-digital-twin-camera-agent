from pxr import UsdGeom, Gf, Semantics
import omni
import omni.replicator.core as rep
import asyncio
import math

# =========================
# CONFIG
# =========================

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

MOVE_STEP = 0.35
YAW_STEP = 1.5
CENTER_DEADBAND = 40.0

ARRIVAL_AREA = 2000
VISIBILITY_THRESHOLD = 0.5

# Visibility hysteresis
LOST_TARGET_GRACE_FRAMES = 12

# Image-center control
TRACK_CENTER_DEADBAND = 60.0
APPROACH_CENTER_DEADBAND = 25.0

# Reacquire turn memory
REACQUIRE_YAW_STEP = 2.0

# Detection quality gates
MIN_TRACK_AREA = 300.0
EDGE_MARGIN_PX = 100.0

# Reacquire tuning
LOST_TARGET_GRACE_FRAMES = 20
REACQUIRE_YAW_STEP = 3.0

INVESTIGATE_GRACE_FRAMES = 20


# 🔥 Waypoints (EDIT THESE LATER IF NEEDED)
WAYPOINTS = [
    (0, 8, 20),
    (15, 8, 5),
    (-15, 8, 5),
    (15, 8, -10),
    (-15, 8, -10),
]

# =========================
# GLOBALS
# =========================

stage = None
bbox_annotator = None
render_product = None

# =========================
# HELPERS
# =========================

def get_prim(path):
    p = stage.GetPrimAtPath(path)
    if not p or not p.IsValid():
        raise RuntimeError(f"Invalid prim: {path}")
    return p

def get_translate_op(x):
    for op in x.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op
    return x.AddTranslateOp()

def get_rotate_op(x):
    for op in x.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateY:
            return op
    return x.AddRotateYOp()

def get_position(prim):
    x = UsdGeom.Xformable(prim)
    t = get_translate_op(x).Get()
    return Gf.Vec3d(t[0], t[1], t[2]) if t else Gf.Vec3d(0,0,0)

def set_position(prim, pos):
    x = UsdGeom.Xformable(prim)
    get_translate_op(x).Set(pos)

def get_yaw(prim):
    x = UsdGeom.Xformable(prim)
    val = get_rotate_op(x).Get()
    return float(val) if val else 0.0

def set_yaw(prim, yaw):
    x = UsdGeom.Xformable(prim)
    get_rotate_op(x).Set(float(yaw))

def move_forward(prim, yaw, step):
    t = get_position(prim)
    rad = math.radians(yaw)
    dx = -math.sin(rad) * step
    dz = -math.cos(rad) * step
    set_position(prim, Gf.Vec3d(t[0]+dx, t[1], t[2]+dz))

def label_prims():
    for p in TARGET_PRIMS:
        prim = get_prim(p)
        s = Semantics.SemanticsAPI.Apply(prim, "Semantics")
        s.CreateSemanticTypeAttr().Set("class")
        s.CreateSemanticDataAttr().Set(TARGET_CLASS_NAME)

def setup_bbox():
    global render_product, bbox_annotator
    render_product = rep.create.render_product(CAMERA_PATH, (IMAGE_WIDTH, IMAGE_HEIGHT))
    bbox_annotator = rep.AnnotatorRegistry.get_annotator(
        "bounding_box_2d_tight",
        init_params={"semanticTypes": ["class"]}
    )
    bbox_annotator.attach([render_product])

def parse_bbox(data):
    arr = data["data"]
    if len(arr) == 0:
        return None

    best = None
    best_vis = 0

    for e in arr:
        _, x1,y1,x2,y2,occ = e
        area = (x2-x1)*(y2-y1)
        vis = 1.0 - occ

        if vis > best_vis and area > 100:
            best_vis = vis
            best = (x1,y1,x2,y2,area,vis)

    return best


def is_usable_detection(bbox):
    if bbox is None:
        return False

    x1, y1, x2, y2, area, vis = bbox
    cx = (x1 + x2) * 0.5

    # Reject tiny detections
    if area < MIN_TRACK_AREA:
        return False

    # Reject detections hugging the screen edges
    if cx < EDGE_MARGIN_PX:
        return False
    if cx > (IMAGE_WIDTH - EDGE_MARGIN_PX):
        return False

    return True


# =========================
# MAIN
# =========================

async def main():
    global stage
    stage = omni.usd.get_context().get_stage()

    root = get_prim(AGENT_ROOT_PATH)

    label_prims()
    setup_bbox()

    waypoint_index = 1
    state = "patrol"

    lost_target_frames = 0
    last_seen_dir = 0   # -1 = left, +1 = right, 0 = unknown
    investigate_frames = 0

    print("=== V07 STARTED ===")

    while True:
        await rep.orchestrator.step_async()

        bbox_raw = bbox_annotator.get_data()
        bbox = parse_bbox(bbox_raw)

        yaw = get_yaw(root)
        pos = get_position(root)

        raw_visible = bbox is not None
        target_visible = is_usable_detection(bbox)

        bbox_area = 0.0
        bbox_center_x = None
        bbox_error = None

        if bbox:
            x1, y1, x2, y2, area, vis = bbox
            bbox_area = area
            bbox_center_x = (x1 + x2) * 0.5
            bbox_error = bbox_center_x - CENTER_X

            # remember which side target was last seen on
            if bbox_error < 0:
                last_seen_dir = -1
            elif bbox_error > 0:
                last_seen_dir = +1

        # =====================
        # PATROL
        # =====================
        if state == "patrol":
            # interrupt patrol if target appears
            if target_visible:
                state = "track"
                lost_target_frames = 0
                investigate_frames = 0
            elif raw_visible:
                state = "investigate"
                lost_target_frames = 0
                investigate_frames = 0
            else:
                wp = WAYPOINTS[waypoint_index]

                dx = wp[0] - pos[0]
                dz = wp[2] - pos[2]
                dist = math.sqrt(dx * dx + dz * dz)

                if dist < 1.0:
                    waypoint_index = (waypoint_index + 1) % len(WAYPOINTS)
                    wp = WAYPOINTS[waypoint_index]
                    dx = wp[0] - pos[0]
                    dz = wp[2] - pos[2]

                desired_yaw = math.degrees(math.atan2(-dx, -dz))

                yaw_delta = wrap_angle_deg(desired_yaw - yaw)

                if abs(yaw_delta) > 5.0:
                    set_yaw(root, yaw + (YAW_STEP if yaw_delta > 0 else -YAW_STEP))
                else:
                    move_forward(root, yaw, MOVE_STEP)


        elif state == "investigate":
            if target_visible:
                state = "track"
                lost_target_frames = 0
                investigate_frames = 0

            elif raw_visible:
                investigate_frames = 0

                # Turn toward the weak detection to try to center it
                if bbox_error is not None:
                    if bbox_error > 0:
                        set_yaw(root, yaw - REACQUIRE_YAW_STEP)
                        last_seen_dir = 1
                    elif bbox_error < 0:
                        set_yaw(root, yaw + REACQUIRE_YAW_STEP)
                        last_seen_dir = -1

            else:
                investigate_frames += 1

                # Keep turning briefly in the last seen direction
                if last_seen_dir > 0:
                    set_yaw(root, yaw - REACQUIRE_YAW_STEP)
                elif last_seen_dir < 0:
                    set_yaw(root, yaw + REACQUIRE_YAW_STEP)

                if investigate_frames > INVESTIGATE_GRACE_FRAMES:
                    state = "patrol"


        # =====================
        # TRACK
        # Rotate only. NO forward motion here.
        # =====================
        elif state == "track":
            if not target_visible:
                lost_target_frames += 1

                if lost_target_frames <= LOST_TARGET_GRACE_FRAMES:
                    state = "reacquire"
                else:
                    state = "patrol"
            else:
                lost_target_frames = 0

                # Rotate until target is well centered
                if bbox_error > TRACK_CENTER_DEADBAND:
                    set_yaw(root, yaw - YAW_STEP)
                elif bbox_error < -TRACK_CENTER_DEADBAND:
                    set_yaw(root, yaw + YAW_STEP)
                else:
                    state = "approach"

        # =====================
        # REACQUIRE
        # Keep turning in the last known direction briefly
        # before giving up back to patrol
        # =====================
        elif state == "reacquire":
            if target_visible:
                state = "track"
                lost_target_frames = 0
            else:
                lost_target_frames += 1

                if last_seen_dir > 0:
                    set_yaw(root, yaw - REACQUIRE_YAW_STEP)
                elif last_seen_dir < 0:
                    set_yaw(root, yaw + REACQUIRE_YAW_STEP)

                if lost_target_frames > LOST_TARGET_GRACE_FRAMES:
                    state = "patrol"

        # =====================
        # APPROACH
        # Move only if target is centered enough
        # =====================
        elif state == "approach":
            if not target_visible:
                lost_target_frames += 1

                if lost_target_frames <= LOST_TARGET_GRACE_FRAMES:
                    state = "reacquire"
                else:
                    state = "patrol"
            else:
                lost_target_frames = 0

                # If target drifts too far off-center, stop moving
                # and go back to track mode.
                if bbox_error > APPROACH_CENTER_DEADBAND:
                    set_yaw(root, yaw - YAW_STEP)
                    state = "track"
                elif bbox_error < -APPROACH_CENTER_DEADBAND:
                    set_yaw(root, yaw + YAW_STEP)
                    state = "track"
                else:
                    if bbox_area > ARRIVAL_AREA:
                        state = "hold"
                    else:
                        move_forward(root, yaw, MOVE_STEP)

        # =====================
        # HOLD
        # =====================
        elif state == "hold":
            if not target_visible:
                lost_target_frames += 1

                if lost_target_frames <= LOST_TARGET_GRACE_FRAMES:
                    state = "reacquire"
                else:
                    state = "patrol"
            else:
                lost_target_frames = 0

                # Minor steering only, no translation
                if bbox_error is not None:
                    if bbox_error > APPROACH_CENTER_DEADBAND:
                        set_yaw(root, yaw - YAW_STEP)
                    elif bbox_error < -APPROACH_CENTER_DEADBAND:
                        set_yaw(root, yaw + YAW_STEP)

        raw_visible = bbox is not None
        print(
            f"state: {state} | "
            f"waypoint: {waypoint_index} | "
            f"raw_visible: {raw_visible} | "
            f"usable_visible: {target_visible} | "
            f"lost_frames: {lost_target_frames} | "
            f"investigate_frames: {investigate_frames} | "
            f"last_seen_dir: {last_seen_dir} | "
            f"bbox: {bbox}"
        )
        await asyncio.sleep(0.05)

# =========================
# ENTRY
# =========================

task = asyncio.ensure_future(main())