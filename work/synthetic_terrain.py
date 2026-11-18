"""Deterministic, explicitly synthetic greenhouse floor profiles for simulation.

These profiles are engineering stress tests, not measured greenhouse terrain.
The mesh used for rendering, ray queries, and PhysX collision is the same.
"""
import math
import numpy as np

PROFILES = {
    "rough_mild": {"peak_amplitude_m": 0.006, "wavelengths_m": [1.2, 2.4]},
    "rough_moderate": {"peak_amplitude_m": 0.012, "wavelengths_m": [1.2, 2.4]},
}


def height(x, y, profile, seed=0):
    spec = PROFILES[profile]
    amp = spec["peak_amplitude_m"]
    phase = (int(seed) % 997) * 0.017
    wave = (
        0.55 * math.sin(2 * math.pi * x / 1.2 + phase)
        + 0.25 * math.sin(2 * math.pi * x / 2.4 - 0.6 * phase)
        + 0.20 * math.cos(2 * math.pi * y / 2.4 + 0.4 * phase)
    )
    return amp * wave


def install(stage, profile, seed=0):
    """Replace the flat floor with a fixed triangle mesh for a short pilot."""
    if profile not in PROFILES:
        raise ValueError(f"Unsupported synthetic terrain profile: {profile}")
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, Vt
    import full_map_surveyed_scene as scene

    floor = stage.GetPrimAtPath("/World/Floor")
    soil = stage.GetPrimAtPath("/World/LabSoil")
    if not floor or not floor.IsValid() or not soil or not soil.IsValid():
        raise RuntimeError("Expected surveyed flat floor and visual soil")
    floor.SetActive(False)
    soil.SetActive(False)

    # 15 cm spacing resolves both smooth wavelengths with at least 8 samples
    # per shortest period while keeping the static collision mesh modest.
    xmin, xmax = -scene.XBOUND - 0.2, scene.XBOUND + 0.2
    ymin, ymax = scene.YMIN - 0.5, scene.YMAX + 0.5
    spacing = 0.15
    nx = int(math.ceil((xmax - xmin) / spacing))
    ny = int(math.ceil((ymax - ymin) / spacing))
    xs = np.linspace(xmin, xmax, nx + 1)
    ys = np.linspace(ymin, ymax, ny + 1)
    points = []
    colors = []
    for x in xs:
        for y in ys:
            z = height(float(x), float(y), profile, seed)
            points.append(Gf.Vec3f(float(x), float(y), z))
            shade = 0.9 + 0.1 * math.sin(float(x) * 3.7 + float(y) * 2.1) ** 2
            colors.append(Gf.Vec3f(0.145 * shade, 0.095 * shade, 0.057 * shade))

    counts = []
    indices = []
    row = ny + 1
    for i in range(nx):
        for j in range(ny):
            a = i * row + j
            # Consistent winding, two triangles per grid cell.
            counts.extend((3, 3))
            indices.extend((a, a + row, a + row + 1, a, a + row + 1, a + 1))

    mesh = UsdGeom.Mesh.Define(stage, "/World/SyntheticUnevenFloor")
    mesh.CreatePointsAttr(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(Vt.Vec3fArray(colors))
    material_prim = stage.GetPrimAtPath("/World/LabMaterials/Soil")
    if material_prim and material_prim.IsValid():
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(UsdShade.Material(material_prim))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    # Explicitly request a static triangle mesh. The default approximation can
    # convexify a greenhouse-sized surface and produce a false tilted support.
    mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    mesh_collision.CreateApproximationAttr().Set("none")
    physics_material_prim = stage.GetPrimAtPath("/World/GroundMaterial")
    if physics_material_prim and physics_material_prim.IsValid():
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
            UsdShade.Material(physics_material_prim), materialPurpose="physics"
        )

    values = np.asarray([p[2] for p in points])
    return {
        "profile": profile,
        "source": "SYNTHETIC_ENGINEERING_STRESS_TEST_NOT_FIELD_MEASUREMENT",
        "seed": int(seed),
        "peak_amplitude_m": PROFILES[profile]["peak_amplitude_m"],
        "configured_wavelengths_m": PROFILES[profile]["wavelengths_m"],
        "height_min_m": float(values.min()),
        "height_max_m": float(values.max()),
        "mesh_spacing_m": spacing,
        "mesh_vertices": len(points),
        "mesh_triangles": len(indices) // 3,
        "collision_approximation": "none (static triangle mesh)",
        "same_mesh_for_render_and_collision": True,
        "flat_floor_disabled": True,
    }
