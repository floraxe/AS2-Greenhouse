"""Rooted partial crop intrusion and lightweight procedural scene styling."""
import math


def prepare(stage):
    from pxr import UsdGeom,UsdPhysics,UsdLux,UsdShade,Sdf,Gf
    root='/World/PartialCrop'
    UsdGeom.Xform.Define(stage,root)

    def cylinder(name,a,b,r=.01):
        a,b=Gf.Vec3d(*a),Gf.Vec3d(*b);d=b-a
        p=UsdGeom.Cylinder.Define(stage,root+'/'+name)
        p.CreateRadiusAttr(r);p.CreateHeightAttr(d.GetLength())
        p.AddTranslateOp().Set((a+b)/2)
        p.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1),d.GetNormalized()).GetQuat()))
        p.CreateDisplayColorAttr([Gf.Vec3f(.09,.26,.055)])
        UsdPhysics.CollisionAPI.Apply(p.GetPrim())

    def ellipsoid(name,c,axes,color):
        pts=[];faces=[];n,m=12,8
        for i in range(m+1):
            lat=-math.pi/2+math.pi*i/m
            for j in range(n):
                ang=2*math.pi*j/n
                pts.append((c[0]+axes[0]*math.cos(lat)*math.cos(ang),c[1]+axes[1]*math.cos(lat)*math.sin(ang),c[2]+axes[2]*math.sin(lat)))
        for i in range(m):
            for j in range(n):
                faces.append([i*n+j,i*n+(j+1)%n,(i+1)*n+(j+1)%n,(i+1)*n+j])
        p=UsdGeom.Mesh.Define(stage,root+'/'+name)
        p.CreatePointsAttr(pts);p.CreateFaceVertexCountsAttr([4]*len(faces));p.CreateFaceVertexIndicesAttr([x for f in faces for x in f])
        p.CreateSubdivisionSchemeAttr('none');p.CreateDoubleSidedAttr(True)
        p.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(p.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(p.GetPrim()).CreateApproximationAttr('convexHull')

    cylinder('Stem',(-4.6,-1.15,.23),(-4.6,-1.15,1.05))
    cylinder('IntrudingBranch',(-4.6,-1.15,.68),(-4.6,-1.92,.47),.012)
    ellipsoid('LeafLow',(-4.61,-1.91,.46),(.27,.12,.026),(.10,.34,.07))
    ellipsoid('LeafUpper',(-4.45,-1.90,.58),(.23,.12,.030),(.12,.39,.08))
    ellipsoid('Fruit',(-4.62,-1.87,.415),(.04,.04,.04),(.70,.10,.025))
    UsdGeom.Imageable(stage.GetPrimAtPath(root)).CreateVisibilityAttr().Set('invisible')
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(root) and prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)

    def material(name,color,roughness=.8,metallic=0.):
        p='/World/Looks/'+name
        mat=UsdShade.Material.Define(stage,p);shader=UsdShade.Shader.Define(stage,p+'/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metallic)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        return mat

    floor=material('WarmStone',(.48,.46,.39),.96)
    earth=material('RichSoil',(.18,.115,.063),.95)
    path=material('SandyPath',(.63,.59,.48),.96)
    leaves=[material('Leaf'+str(i),c,.72) for i,c in enumerate([(.075,.27,.065),(.10,.34,.075),(.14,.38,.085)])]
    fruit=material('Tomato',(.66,.075,.025),.42)
    stem=material('Stem',(.12,.26,.045),.84)
    steel=material('Irrigation',(.045,.055,.055),.67)
    for prim in list(stage.Traverse()):
        p=str(prim.GetPath());name=prim.GetName();mat=None
        if p=='/World/Floor':mat=floor
        elif p.startswith('/World/Beds/'):mat=earth
        elif p.startswith('/World/Guide/'):mat=path
        elif p.startswith('/World/Irrigation/'):mat=steel
        elif any(p.startswith(x) for x in ['/World/Crops/','/World/TestObstacle/','/World/PartialCrop/']):
            if 'Leaf' in name:mat=leaves[sum(map(ord,name))%3]
            elif 'Tomato' in name or 'Fruit' in name:mat=fruit
            elif prim.IsA(UsdGeom.Cylinder):mat=stem
        if mat is not None and prim.IsA(UsdGeom.Gprim):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
    light=UsdLux.DomeLight(stage.GetPrimAtPath('/World/Light'))
    light.GetIntensityAttr().Set(650)
    light.CreateColorAttr(Gf.Vec3f(.90,.95,1.0))
    sun=UsdLux.DistantLight.Define(stage,'/World/SoftSun')
    sun.CreateIntensityAttr(1600);sun.CreateAngleAttr(3.)
    sun.CreateColorAttr(Gf.Vec3f(1.,.91,.76))
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-35,-30,-25))
    # A thin, non-colliding route guide; does not enter the raycast sensor.
    curve=UsdGeom.BasisCurves.Define(stage,'/World/RouteGuide')
    curve.CreateTypeAttr('linear');curve.CreateCurveVertexCountsAttr([2])
    curve.CreatePointsAttr([(-6.5,-2.15,.008),(3.5,-2.15,.008)])
    curve.CreateWidthsAttr([.012]);curve.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    curve.CreateDisplayColorAttr([Gf.Vec3f(.08,.48,.46)])
