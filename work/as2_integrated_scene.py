import math,json
from pathlib import Path
ROOT=Path("E:/AS2_Greenhouse")
def setup(app,greenhouse=False,arm=False,long_demo=False,full_demo=False,scene_hook=None):
    import numpy as np
    import omni.usd
    from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,Sdf,UsdLux
    from isaacsim.core.simulation_manager import SimulationManager as SM
    from isaacsim.core.experimental.prims import Articulation
    ctx=omni.usd.get_context();ctx.new_stage();stage=ctx.get_stage()
    world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim());UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    scene=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene');scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
    ps=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim());ps.CreateTimeStepsPerSecondAttr(240);ps.CreateEnableGPUDynamicsAttr(False);ps.CreateBroadphaseTypeAttr('MBP');ps.CreateSolverTypeAttr('TGS')
    floor=UsdGeom.Cube.Define(stage,'/World/Ground');floor.CreateSizeAttr(1);floor.AddTranslateOp().Set(Gf.Vec3d(0,0,-.05));floor.AddScaleOp().Set(Gf.Vec3d(12,8,.1));floor.CreateDisplayColorAttr([Gf.Vec3f(.25,.3,.32)]);UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    mat=UsdPhysics.MaterialAPI.Apply(stage.DefinePrim('/World/GroundMaterial','Material'));mat.CreateStaticFrictionAttr(.8);mat.CreateDynamicFrictionAttr(.7);mat.CreateRestitutionAttr(0)
    from pxr import UsdShade
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(UsdShade.Material(mat.GetPrim()),materialPurpose='physics')
    light=UsdLux.DomeLight.Define(stage,'/World/Light');light.CreateIntensityAttr(800)
    robot=UsdGeom.Xform.Define(stage,'/World/AS2');robot.GetPrim().GetReferences().AddReference(str(ROOT/('assets/as2_so101_usd/as2_so101_fixed/as2_so101_fixed.usda' if arm else 'assets/as2_usd_display/as2/as2.usda')));robot.GetPrim().GetVariantSet('Physics').SetVariantSelection('physx')
    start_x=-3.
    if long_demo:
        from l2_long_route import START_X
        start_x=START_X
    if full_demo:start_x=-5.8
    robot.AddTranslateOp().Set(Gf.Vec3d(start_x,-2.15,.4) if greenhouse else Gf.Vec3d(0,0,.4))
    fix=stage.GetPrimAtPath('/World/AS2/fix_base_joint');fix.SetActive(False)
    base=stage.GetPrimAtPath('/World/AS2/Geometry/world/base_link');UsdPhysics.ArticulationRootAPI.Apply(base);pa=PhysxSchema.PhysxArticulationAPI.Apply(base);pa.CreateEnabledSelfCollisionsAttr(False);pa.CreateSolverPositionIterationCountAttr(32);pa.CreateSolverVelocityIterationCountAttr(8)
    for j in stage.Traverse():
        if j.IsA(UsdPhysics.RevoluteJoint):
            drive=UsdPhysics.DriveAPI.Apply(j,'angular');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(1);drive.CreateDampingAttr(.05);drive.CreateMaxForceAttr(75 if 'calf' in j.GetName() else 50)
    cam=UsdGeom.Camera.Define(stage,'/World/ViewCamera');cam.AddTranslateOp().Set(Gf.Vec3d(2.4,-2.4,1.6))
    eye=Gf.Vec3d(2.4,-2.4,1.6);m=Gf.Matrix4d().SetLookAt(eye,Gf.Vec3d(0,0,.25),Gf.Vec3d(0,0,1)).GetInverse();cam.AddOrientOp().Set(Gf.Quatf(m.ExtractRotationQuat()));cam.CreateClippingRangeAttr(Gf.Vec2f(.01,100));cam.CreateFocalLengthAttr(22)
    if arm:
        from greenhouse_crop_scene_v4 import attach_mount_support
        mount_info=attach_mount_support(stage)
        (ROOT/'outputs/mount_support_v4.json').write_text(json.dumps(mount_info,indent=2),encoding='utf-8')
    if greenhouse:
        floor.GetPrim().SetActive(False)
        from greenhouse_crop_scene_v4 import build
        layout=build(stage)
        (ROOT/'outputs/greenhouse_crop_layout_v4.json').write_text(json.dumps(layout,indent=2),encoding='utf-8')
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath('/World/Floor')).Bind(UsdShade.Material(mat.GetPrim()),materialPurpose='physics')
        if long_demo:
            from l2_long_route import prepare
            prepare(stage)
        if full_demo:
            from full_mission_scene import prepare
            prepare(stage)
        if scene_hook:scene_hook(stage)
    sensor_path='/World/AS2/Geometry/world/base_link/CameraCADFrame'
    reference=json.loads((ROOT/'assets/camera_manual_reference.json').read_text())
    cad=UsdGeom.Xform.Define(stage,sensor_path);cad.AddTranslateOp().Set(Gf.Vec3d(*reference['cad_frame_base_translation_m']))
    cad.AddOrientOp().Set(Gf.Quatf((Gf.Rotation(Gf.Vec3d(0,1,0),84)*Gf.Rotation(Gf.Vec3d(0,0,1),180)).GetQuat()))
    sensor=UsdGeom.Camera.Define(stage,sensor_path+'/NavigationTestCamera');sensor.AddTranslateOp().Set(Gf.Vec3d(0,0,-.08));sensor.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1),90).GetQuat()))
    sensor.CreateFocalLengthAttr(.015);sensor.CreateHorizontalApertureAttr(.0724264);sensor.CreateVerticalApertureAttr(.04073985);sensor.CreateClippingRangeAttr(Gf.Vec2f(.01,30))
    manual=UsdGeom.Camera.Define(stage,sensor_path+'/SimCamera');manual.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1),90).GetQuat()))
    for attr in ['focalLength','horizontalAperture','verticalAperture','clippingRange']:manual.GetPrim().GetAttribute(attr).Set(sensor.GetPrim().GetAttribute(attr).Get())
    sensor.GetPrim().SetCustomData({'temporary_mount_forward_shift_m':.08,'depth':'synthetic ideal depth; hardware not confirmed'})
    if not greenhouse:
        obstacle=UsdGeom.Cube.Define(stage,'/World/TestObstacle');obstacle.CreateSizeAttr(1);obstacle.AddTranslateOp().Set(Gf.Vec3d(1.8,0,.3));obstacle.AddScaleOp().Set(Gf.Vec3d(.3,.25,.6));obstacle.CreateDisplayColorAttr([Gf.Vec3f(.65,.35,.15)]);UsdPhysics.CollisionAPI.Apply(obstacle.GetPrim());UsdGeom.Imageable(obstacle).GetVisibilityAttr().Set('invisible');UsdPhysics.CollisionAPI(obstacle).GetCollisionEnabledAttr().Set(False)
    stage.GetRootLayer().Export(str(ROOT/('outputs/showcase_v2_scene.usda' if scene_hook else 'outputs/full_mission_v1_scene.usda' if full_demo else 'outputs/l2_long_route_v2.usda' if long_demo else 'outputs/greenhouse_physical_arm_v2.usda' if arm and greenhouse else 'outputs/greenhouse_physical_v2.usda' if greenhouse else 'outputs/as2_command_physical_v2.usda')))

    for _ in range(10):app.update()
    print('PHYSICAL_INITIALIZING',flush=True);SM.set_device('cpu');SM.set_physics_dt(1/240);SM.initialize_physics();print('PHYSICS_READY',flush=True)
    art=Articulation('/World/AS2/Geometry/world/base_link');names=list(art.dof_names);print('DOFS',names,flush=True)
    if len(names)!=12:raise RuntimeError('Expected 12 actuated joints: '+repr(names))
    q=np.array([.9 if 'thigh' in n else -1.8 if 'calf' in n else .1 if n.startswith(('FR','RR')) else -.1 for n in names],dtype=np.float32)
    k=np.array([60 if 'calf' in n else 40 for n in names],np.float32);d=np.array([3 if 'calf' in n else 2 for n in names],np.float32)
    art.set_dof_gains(stiffnesses=k*5,dampings=d*math.sqrt(5));art.set_dof_positions(q[None]);art.set_dof_velocities(np.zeros((1,12),np.float32));art.set_dof_position_targets(q[None])
    return art,names,q,stage

