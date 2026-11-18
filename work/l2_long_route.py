"""Illustrative 10 m straight route; keep original measured assets untouched."""
START_X=-6.5
END_X=3.5
ROUTE_Y=-2.15
ROUTE_LENGTH=END_X-START_X


def prepare(stage):
    from pxr import UsdGeom,Gf,Sdf
    # These are decorative greenhouse members, not the robot mounting supports.
    structure=stage.GetPrimAtPath('/World/Structure')
    if structure:
        structure.SetActive(False)
    floor=UsdGeom.Xformable(stage.GetPrimAtPath('/World/Floor'))
    for op in floor.GetOrderedXformOps():
        if op.GetOpType()==UsdGeom.XformOp.TypeScale:
            s=op.Get();op.Set(Gf.Vec3d(26,s[1],s[2]))
    for prim in stage.GetPrimAtPath('/World/Beds').GetChildren():
        ops=UsdGeom.Xformable(prim).GetOrderedXformOps()
        t=ops[0].Get();s=ops[1].Get();sign=-1 if t[0]<0 else 1
        ops[0].Set(Gf.Vec3d(sign*5.4,t[1],t[2]))
        ops[1].Set(Gf.Vec3d(9.,s[1],s[2]))
    for prim in stage.GetPrimAtPath('/World/Guide').GetChildren():
        if prim.GetName().startswith('Aisle'):
            ops=UsdGeom.Xformable(prim).GetOrderedXformOps();s=ops[1].Get()
            ops[1].Set(Gf.Vec3d(19.8,s[1],s[2]))
    # Duplicate the rooted plants along each extended bed section.
    for row in stage.GetPrimAtPath('/World/Crops').GetChildren():
        sign=-1 if row.GetName().endswith('A') else 1
        for i in range(5):
            source=str(row.GetPath())+f'/Plant{i}'
            dest=str(row.GetPath())+f'/Plant{i+5}'
            Sdf.CopySpec(stage.GetRootLayer(),source,stage.GetRootLayer(),dest)
            UsdGeom.Xformable(stage.GetPrimAtPath(dest)).AddTranslateOp().Set(Gf.Vec3d(sign*3.9,0,0))
    for prim in stage.GetPrimAtPath('/World/Irrigation').GetChildren():
        c=UsdGeom.Cylinder(prim);c.GetHeightAttr().Set(c.GetHeightAttr().Get()+5.)
        op=UsdGeom.Xformable(prim).GetOrderedXformOps()[0];t=op.Get()
        op.Set(Gf.Vec3d(t[0]+(-2.5 if t[0]<0 else 2.5),t[1],t[2]))
    obstacle=UsdGeom.Xformable(stage.GetPrimAtPath('/World/TestObstacle'))
    obstacle.AddTranslateOp(opSuffix='long_route').Set(Gf.Vec3d(-3.5,0,0))
    stage.GetPrimAtPath('/World').SetCustomDataByKey('long_lidar_route',
        '10 m illustrative straight route from x=-6.5 to 3.5; decorative frame removed')
    from lidar_bypass_scene import prepare as prepare_visuals_and_crop
    prepare_visuals_and_crop(stage)
