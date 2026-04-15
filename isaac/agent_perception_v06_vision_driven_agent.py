from pxr import UsdGeom, Gf, Semantics
import omni
import omni.replicator.core as rep
import asyncio
import math

# ============================================================
# CONFIG
# ============================================================

STAGE_PATH_NOTE = "Designed to run with runtime_v03.usda OPEN in Isaac Sim."

AGENT_ROOT_PATH = "/World/AgentRoot"
CAMERA_PATH = "/World/AgentRoot/AgentCamera"

TARGET_CLASS_NAME = "warning_light"

# Label the ACTUAL visible mesh prims, not parent Xforms
TARGET_PRIMS = [
    "/World/EnvWrapper/Environment/Warning_Light/Xform/Warning_Light/Glass_Dome/Sphere",
    "/World/EnvWrapper/Environment/Warning_Light/Xform/Warning_Light/Base/Cylinder",
]

# Deterministic startup pose
START_ROOT_YAW_DEG = 147.73
START_ROOT_TRANSLATE = Gf.Vec3d(0.0, 8.0, 20.0)

# Camera / bbox image size
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
IMAGE_CENTER_X = IMAGE_WIDTH * 0.5
IMAGE_CENTER_Y = IMAGE_HEIGHT * 0.5

# Search behavior
SCAN_SPEED_DEG_PER_STEP = 1.0
SCAN_LIMIT_DEG = 90.0

# Vision gating
MIN_BBOX_WIDTH = 8.0
MIN_BBOX_HEIGHT = 8.0
MIN_BBOX_AREA = 120.0
MIN_VISIBILITY = 0.50

# Steering from image center
CENTER_DEADBAND_PX = 30.0
YAW_STEP_DEG = 1.0

# Forward motion
MOVE_STEP_UNITS = 0.25

# Arrival based on bbox size
# Bigger bbox area == closer target
ARRIVAL_BBOX_AREA = 1800.0
HOLD_EXIT_BBOX_AREA = 1400.0

# Loop timing
STEP_FRAMES_PER_LOOP = 1

# ============================================================
# GLOBALS
# ============================================================

stage = None
bbox_annotator = None
render_product = None

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

def get_prim(path):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Invalid prim: {path}")
    return prim

def get_world_transform(prim):
    cache = UsdGeom.XformCache()
    return cache.GetLocalToWorldTransform(prim)

def get_world_position(prim):
    return get_world_transform(prim).ExtractTranslation()

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

def apply_semantics(prim, class_name):
    sem_api = Semantics.SemanticsAPI.Apply(prim, "Semantics")
    sem_api.CreateSemanticTypeAttr().Set("class")
    sem_api.CreateSemanticDataAttr().Set(class_name)

def move_root_forward_small_step(agent_root_prim, yaw_deg, step_units):
    """
    Move forward in XZ according to current root yaw.
    Local forward assumed to be -Z rotated by Y.
    """
    current_t = get_current_root_translate(agent_root_prim)

    yaw_rad = math.radians(yaw_deg)
    forward_x = -math.sin(yaw_rad)
    forward_z = -math.cos(yaw_rad)

    next_t = Gf.Vec3d(
        current_t[0] + forward_x * step_units,
        current_t[1],
        current_t[2] + forward_z * step_units
    )
    set_root_translate(agent_root_prim, next_t)

def parse_best_bbox(bbox_data):
    """
    Expected working format from your Isaac 4.2 build:
      data: array([
        (semanticId, x_min, y_min, x_max, y_max, occlusionRatio),
        ...
      ])
    """
    bbox_array = bbox_data["data"]

    if len(bbox_array) == 0:
        return {
            "target_visible": False,
            "semantic_id": None,
            "x_min": None,
            "y_min": None,
            "x_max": None,
            "y_max": None,
            "center_x": None,
            "center_y": None,
            "width": 0.0,
            "height": 0.0,
            "area": 0.0,
            "occlusion_ratio": None,
            "visibility": 0.0,
            "raw_count": 0,
        }

    best_box = None
    best_visibility = -1.0

    for entry in bbox_array:
        semantic_id, x_min, y_min, x_max, y_max, occ = entry

        width = max(0.0, float(x_max) - float(x_min))
        height = max(0.0, float(y_max) - float(y_min))
        area = width * height
        visibility = 1.0 - float(occ)

        if width < MIN_BBOX_WIDTH:
            continue
        if height < MIN_BBOX_HEIGHT:
            continue
        if area < MIN_BBOX_AREA:
            continue

        if visibility > best_visibility:
            best_visibility = visibility
            best_box = entry

    if best_box is None:
        return {
            "target_visible": False,
            "semantic_id": None,
            "x_min": None,
            "y_min": None,
            "x_max": None,
            "y_max": None,
            "center_x": None,
            "center_y": None,
            "width": 0.0,
            "height": 0.0,
            "area": 0.0,
            "occlusion_ratio": None,
            "visibility": 0.0,
            "raw_count": len(bbox_array),
        }

    semantic_id, x_min, y_min, x_max, y_max, occ = best_box
    width = max(0.0, float(x_max) - float(x_min))
    height = max(0.0, float(y_max) - float(y_min))
    area = width * height
    center_x = (float(x_min) + float(x_max)) * 0.5
    center_y = (float(y_min) + float(y_max)) * 0.5
    visibility = 1.0 - float(occ)

    return {
        "target_visible": visibility >= MIN_VISIBILITY,
        "semantic_id": int(semantic_id),
        "x_min": int(x_min),
        "y_min": int(y_min),
        "x_max": int(x_max),
        "y_max": int(y_max),
        "center_x": center_x,
        "center_y": center_y,
        "width": width,
        "height": height,
        "area": area,
        "occlusion_ratio": float(occ),
        "visibility": visibility,
        "raw_count": len(bbox_array),
    }

def setup_bbox_pipeline():
    global render_product, bbox_annotator

    render_product = rep.create.render_product(CAMERA_PATH, (IMAGE_WIDTH, IMAGE_HEIGHT))
    bbox_annotator = rep.AnnotatorRegistry.get_annotator(
        "bounding_box_2d_tight",
        init_params={"semanticTypes": ["class"]}
    )
    bbox_annotator.attach([render_product])

# ============================================================
# MAIN VISION-DRIVEN LOOP
# ============================================================

async def vision_driven_agent_main():
    global stage
    stage = omni.usd.get_context().get_stage()

    agent_root_prim = get_prim(AGENT_ROOT_PATH)
    camera_prim = get_prim(CAMERA_PATH)

    # --------------------------------------------------------
    # Apply semantics to real visible meshes
    # --------------------------------------------------------
    for path in TARGET_PRIMS:
        prim = get_prim(path)
        apply_semantics(prim, TARGET_CLASS_NAME)
        print("Labeled:", path, "->", TARGET_CLASS_NAME)

    # --------------------------------------------------------
    # Reset startup pose
    # --------------------------------------------------------
    reset_agent_root_pose(agent_root_prim, START_ROOT_YAW_DEG, START_ROOT_TRANSLATE)

    # --------------------------------------------------------
    # Set up render product + bbox annotator
    # --------------------------------------------------------
    setup_bbox_pipeline()

    initial_yaw_deg = get_current_root_yaw_deg(agent_root_prim)
    scan_dir = 1.0
    state = "search"

    print("======================================================")
    print("V06 vision-driven agent starting")
    print("Stage note:", STAGE_PATH_NOTE)
    print("Camera:", CAMERA_PATH)
    print("Target class:", TARGET_CLASS_NAME)
    print("Initial root yaw:", initial_yaw_deg)
    print("======================================================")

    while True:
        # Let Replicator update
        for _ in range(STEP_FRAMES_PER_LOOP):
            await rep.orchestrator.step_async()

        bbox_data = bbox_annotator.get_data()
        parsed = parse_best_bbox(bbox_data)

        current_root_yaw = get_current_root_yaw_deg(agent_root_prim)
        root_t = get_current_root_translate(agent_root_prim)

        target_visible = parsed["target_visible"]
        bbox_area = parsed["area"]
        bbox_center_x = parsed["center_x"]
        bbox_error_x = None if bbox_center_x is None else (bbox_center_x - IMAGE_CENTER_X)

        # ----------------------------------------------------
        # STATE MACHINE
        # ----------------------------------------------------
        if state == "search":
            if target_visible:
                state = "track"
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

        elif state == "track":
            if not target_visible:
                state = "search"
            else:
                # Steering from bbox horizontal center
                if bbox_error_x is not None:
                    if bbox_error_x > CENTER_DEADBAND_PX:
                        # target is to the right in image
                        set_root_yaw_deg(agent_root_prim, current_root_yaw - YAW_STEP_DEG)
                    elif bbox_error_x < -CENTER_DEADBAND_PX:
                        # target is to the left in image
                        set_root_yaw_deg(agent_root_prim, current_root_yaw + YAW_STEP_DEG)

                # Transition
                if bbox_area >= ARRIVAL_BBOX_AREA:
                    state = "hold"
                else:
                    state = "approach"

        elif state == "approach":
            if not target_visible:
                state = "search"
            else:
                # Keep target centered while moving
                if bbox_error_x is not None:
                    if bbox_error_x > CENTER_DEADBAND_PX:
                        set_root_yaw_deg(agent_root_prim, current_root_yaw - YAW_STEP_DEG)
                    elif bbox_error_x < -CENTER_DEADBAND_PX:
                        set_root_yaw_deg(agent_root_prim, current_root_yaw + YAW_STEP_DEG)

                # Re-read yaw after possible steering
                current_root_yaw = get_current_root_yaw_deg(agent_root_prim)

                if bbox_area >= ARRIVAL_BBOX_AREA:
                    state = "hold"
                else:
                    move_root_forward_small_step(agent_root_prim, current_root_yaw, MOVE_STEP_UNITS)

        elif state == "hold":
            if not target_visible:
                state = "search"
            else:
                # Small steering only, no translation
                if bbox_error_x is not None:
                    if bbox_error_x > CENTER_DEADBAND_PX:
                        set_root_yaw_deg(agent_root_prim, current_root_yaw - YAW_STEP_DEG)
                    elif bbox_error_x < -CENTER_DEADBAND_PX:
                        set_root_yaw_deg(agent_root_prim, current_root_yaw + YAW_STEP_DEG)

                if bbox_area < HOLD_EXIT_BBOX_AREA:
                    state = "track"

        # ----------------------------------------------------
        # DEBUG
        # ----------------------------------------------------
        root_t = get_current_root_translate(agent_root_prim)
        print(
            f"state: {state:10s} | "
            f"visible: {str(target_visible):5s} | "
            f"bbox_count: {parsed['raw_count']:2d} | "
            f"bbox: ({parsed['x_min']}, {parsed['y_min']}, {parsed['x_max']}, {parsed['y_max']}) | "
            f"center_x: {parsed['center_x']} | "
            f"bbox_area: {parsed['area']:.1f} | "
            f"visibility: {parsed['visibility']:.3f} | "
            f"occ: {parsed['occlusion_ratio']} | "
            f"root_yaw: {get_current_root_yaw_deg(agent_root_prim):.2f} | "
            f"root_t: ({root_t[0]:.2f}, {root_t[1]:.2f}, {root_t[2]:.2f})"
        )

        await asyncio.sleep(0.05)

# ============================================================
# ENTRY
# ============================================================

task = asyncio.ensure_future(vision_driven_agent_main())