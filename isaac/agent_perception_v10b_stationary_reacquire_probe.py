from pxr import UsdGeom, Gf, Semantics
import omni
import omni.replicator.core as rep
import asyncio

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

# Use the stopped pose you just reached successfully.
STOPPED_TRANSLATE = Gf.Vec3d(24.28, 2.20, 9.29)
STOPPED_YAW_DEG = 7.0

# Small stationary scan
SCAN_STEP_DEG = 2.0
SCAN_TOTAL_DEG = 120.0   # 60 left + 60 right total sweep
INSPECT_YAW_TOLERANCE = 20.0

# Detection thresholds
MIN_AREA = 1.0

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

def fmt_vec3(v):
    return f"({v[0]:.2f}, {v[1]:.2f}, {v[2]:.2f})"

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

def parse_union_bbox(bbox_data):
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

        if area < MIN_AREA:
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

    union_x1 = min(b["x1"] for b in valid)
    union_y1 = min(b["y1"] for b in valid)
    union_x2 = max(b["x2"] for b in valid)
    union_y2 = max(b["y2"] for b in valid)

    union_w = max(0.0, float(union_x2) - float(union_x1))
    union_h = max(0.0, float(union_y2) - float(union_y1))
    union_area = union_w * union_h
    union_cx = (float(union_x1) + float(union_x2)) * 0.5
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

# ============================================================
# MAIN
# ============================================================

async def main():
    global stage
    stage = omni.usd.get_context().get_stage()

    root = get_prim(AGENT_ROOT_PATH)

    label_target_prims()
    setup_bbox_pipeline()

    # Force agent to the proven stopped pose
    set_position(root, STOPPED_TRANSLATE)
    set_yaw_deg(root, STOPPED_YAW_DEG)

    start_yaw = STOPPED_YAW_DEG
    scan_dir = 1.0
    scanned_deg = 0.0

    print("=== STATIONARY REACQUIRE PROBE STARTED ===")
    print("Stopped pose:", fmt_vec3(STOPPED_TRANSLATE), "| yaw:", STOPPED_YAW_DEG)

    while True:
        await rep.orchestrator.step_async()

        bbox_raw = bbox_annotator.get_data()
        bbox = parse_union_bbox(bbox_raw)

        pos = get_position(root)
        yaw = get_yaw_deg(root)

        raw_visible = bbox is not None
        bbox_error = None
        bbox_area = None
        bbox_vis = None

        if bbox is not None:
            bbox_error = bbox["cx"] - CENTER_X
            bbox_area = bbox["area"]
            bbox_vis = bbox["visibility"]

        # If visible, do tiny centering corrections only
        if raw_visible and bbox_error is not None:
            if bbox_error > INSPECT_YAW_TOLERANCE:
                set_yaw_deg(root, yaw + 0.5 * SCAN_STEP_DEG)
            elif bbox_error < -INSPECT_YAW_TOLERANCE:
                set_yaw_deg(root, yaw - 0.5 * SCAN_STEP_DEG)

        # If not visible, continue stationary sweep
        else:
            next_yaw = yaw + scan_dir * SCAN_STEP_DEG
            relative = wrap_angle_deg(next_yaw - start_yaw)

            if abs(relative) > (SCAN_TOTAL_DEG * 0.5):
                scan_dir *= -1.0
                next_yaw = yaw + scan_dir * SCAN_STEP_DEG

            set_yaw_deg(root, next_yaw)
            scanned_deg += abs(SCAN_STEP_DEG)

        print(
            f"raw_visible: {raw_visible} | "
            f"world_pos: {fmt_vec3(pos)} | "
            f"yaw: {yaw:.2f} | "
            f"bbox_error: {None if bbox_error is None else round(bbox_error, 2)} | "
            f"bbox_area: {bbox_area} | "
            f"bbox_vis: {bbox_vis} | "
            f"bbox: {bbox}"
        )

        await asyncio.sleep(0.05)

# ============================================================
# ENTRY
# ============================================================

task = asyncio.ensure_future(main())