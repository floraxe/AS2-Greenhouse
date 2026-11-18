"""Illustrative complete five-aisle scene; original measured scene preserved."""
from full_mission import route,START

def prepare(stage):
    from pxr import UsdGeom,Gf
    from lidar_bypass_scene import prepare as style
    style(stage)
    stage.GetPrimAtPath('/World/Structure').SetActive(False)
    curve=UsdGeom.BasisCurves(stage.GetPrimAtPath('/World/RouteGuide'))
    pts=[(*START,.012)]+[(g['x'],g['y'],.012) for g in route()]
    curve.GetCurveVertexCountsAttr().Set([len(pts)]);curve.GetPointsAttr().Set(pts)
    for g in route():
        if g['job'] is None:continue
        path=f"/World/JobPoints/P{g['job']:02}"
        disk=UsdGeom.Cylinder.Define(stage,path)
        disk.CreateRadiusAttr(.18);disk.CreateHeightAttr(.008)
        disk.AddTranslateOp().Set(Gf.Vec3d(g['x'],g['y'],.018))
        disk.CreateDisplayColorAttr([Gf.Vec3f(.12,.42,.95)])
    stage.GetPrimAtPath('/World').SetCustomDataByKey('mission_note','5 predefined aisles, 10 mocked jobs, outer return; truth pose; illustrative dimensions')
