# This becomes your known-good perception probe.

# If anything breaks later, you can ALWAYS come back to:

# “Does bbox detection still work?”

from pxr import Usd, Semantics
import omni
import omni.replicator.core as rep
import asyncio

# ============================================================
# CONFIG
# ============================================================

stage = omni.usd.get_context().get_stage()

CAMERA_PATH = "/World/AgentRoot/AgentCamera"
TARGET_CLASS_NAME = "warning_light"

# Label the ACTUAL visible mesh prims, not the parent Xforms
TARGET_PRIMS = [
    "/World/EnvWrapper/Environment/Warning_Light/Xform/Warning_Light/Glass_Dome/Sphere",
    "/World/EnvWrapper/Environment/Warning_Light/Xform/Warning_Light/Base/Cylinder",
]

# Optional visibility threshold if you want a binary visible/not-visible flag
MIN_VISIBILITY = 0.50

# ============================================================
# HELPERS
# ============================================================

def apply_semantics(prim, class_name):
    """
    Apply Omniverse semantics schema in the form Replicator expects.
    """
    sem_api = Semantics.SemanticsAPI.Apply(prim, "Semantics")
    sem_api.CreateSemanticTypeAttr().Set("class")
    sem_api.CreateSemanticDataAttr().Set(class_name)

def parse_best_bbox(bbox_data):
    """
    bbox_data comes back in the format you already validated:
        data: [
            (semanticId, x_min, y_min, x_max, y_max, occlusionRatio),
            ...
        ]

    This function:
    - finds all boxes
    - computes visibility = 1 - occlusionRatio
    - picks the most visible box
    - returns structured info
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
            "occlusion_ratio": None,
            "visibility": 0.0,
            "raw_count": 0,
        }

    best_box = None
    best_visibility = -1.0

    for entry in bbox_array:
        semantic_id, x_min, y_min, x_max, y_max, occ = entry
        visibility = 1.0 - float(occ)

        if visibility > best_visibility:
            best_visibility = visibility
            best_box = entry

    semantic_id, x_min, y_min, x_max, y_max, occ = best_box
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
        "occlusion_ratio": float(occ),
        "visibility": visibility,
        "raw_count": len(bbox_array),
    }

# ============================================================
# MAIN PROBE
# ============================================================

async def test_warning_light_bbox_probe():
    # --------------------------------------------------------
    # Apply semantic labels to the visible warning light meshes
    # --------------------------------------------------------
    for path in TARGET_PRIMS:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            raise RuntimeError(f"Invalid target mesh prim: {path}")

        apply_semantics(prim, TARGET_CLASS_NAME)
        print("Labeled:", path, "->", TARGET_CLASS_NAME)

    print("\nUsing current AgentCamera pose exactly as-is.")
    print("Make sure the red warning light is visible in AgentCamera view before running.\n")

    # --------------------------------------------------------
    # Create render product from AgentCamera
    # --------------------------------------------------------
    render_product = rep.create.render_product(CAMERA_PATH, (1280, 720))
    print("Render product created:", render_product)

    # --------------------------------------------------------
    # Attach bbox annotator
    # semanticTypes=['class'] is important
    # --------------------------------------------------------
    bbox_annotator = rep.AnnotatorRegistry.get_annotator(
        "bounding_box_2d_tight",
        init_params={"semanticTypes": ["class"]}
    )
    bbox_annotator.attach([render_product])
    print("BBox annotator attached.\n")

    # --------------------------------------------------------
    # Step frames and print parsed results
    # --------------------------------------------------------
    for i in range(8):
        await rep.orchestrator.step_async()
        bbox_data = bbox_annotator.get_data()

        print(f"FRAME {i}")
        print("RAW BBOX DATA:")
        print(bbox_data)

        parsed = parse_best_bbox(bbox_data)

        print("PARSED RESULT:")
        print(parsed)

        if parsed["target_visible"]:
            print("DETECTED TARGET")
            print("object identity:", TARGET_CLASS_NAME)
            print(
                "screen position:",
                (parsed["x_min"], parsed["y_min"], parsed["x_max"], parsed["y_max"])
            )
            print("center:", (parsed["center_x"], parsed["center_y"]))
            print("occlusion_ratio:", parsed["occlusion_ratio"])
            print("visibility:", parsed["visibility"])
        else:
            print("NO TARGET DETECTED")

        print("-" * 80)

asyncio.ensure_future(test_warning_light_bbox_probe())