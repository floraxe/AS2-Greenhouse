"""Illustrative greenhouse: widened aisles and collidable crop intrusion.

This layout is for demonstrations, not a reconstruction of measured aisle widths.
Leaves are rigid geometry; no plant flexibility or calibrated sensor response.
"""
import math

AISLE_Y = -2.15
AISLE_WIDTH = 1.35
CONNECTOR_WIDTH = 1.8
BED_LENGTH = 4.0
BED_HEIGHT = .23
BASE_PATH = '/World/AS2/Geometry/world/base_link'


def build(stage):
    from pxr import UsdGeom, UsdPhysics, UsdShade, Gf

    def box(path, pos, size, color, collision=True):
        p = UsdGeom.Cube.Define(stage, path)
        p.CreateSizeAttr(1)
        p.AddTranslateOp().Set(Gf.Vec3d(*pos))
        p.AddScaleOp().Set(Gf.Vec3d(*size))
        p.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if collision:
            UsdPhysics.CollisionAPI.Apply(p.GetPrim())
        return p

    def stem(path, a, b, radius=.009, color=(.20,.36,.08), collision=True):
        va, vb = Gf.Vec3d(*a), Gf.Vec3d(*b)
        d = vb-va
        p = UsdGeom.Cylinder.Define(stage, path)
        p.CreateRadiusAttr(radius)
        p.CreateHeightAttr(d.GetLength())
        p.AddTranslateOp().Set((va+vb)/2)
        p.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1),d.GetNormalized()).GetQuat()))
        p.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if collision:
            UsdPhysics.CollisionAPI.Apply(p.GetPrim())
        return p

    def leaf(path, origin, direction, length=.31, width=.15, color=(.12,.38,.07)):
        # A closed, curved pointed blade. Its visible mesh is also the collider;
        # the test obstacle has no hidden box in the aisle.
        dx,dy = direction
        norm = math.hypot(dx,dy)
        dx,dy = dx/norm,dy/norm
        pts=[]
        rows,cols=9,5
        for layer in (-1,1):
            for i in range(rows):
                t=i/(rows-1)
                half=max(.001,width*.5*math.sin(math.pi*t)**.8)
                for j in range(cols):
                    lateral=(j/(cols-1)*2-1)*half
                    sag=.055*math.sin(math.pi*t)-.065*t*t
                    pts.append((origin[0]+dx*t*length-dy*lateral,
                                origin[1]+dy*t*length+dx*lateral,
                                origin[2]+sag+.10*abs(lateral)+layer*.002))
        faces=[]
        stride=rows*cols
        for layer in range(2):
            for i in range(rows-1):
                for j in range(cols-1):
                    a=layer*stride+i*cols+j
                    q=[a,a+1,a+cols+1,a+cols]
                    faces.append(q if layer else q[::-1])
        boundary=list(range(cols)) + [i*cols+cols-1 for i in range(1,rows)]
        boundary += list(range((rows-1)*cols+cols-2,(rows-1)*cols-1,-1))
        boundary += [i*cols for i in range(rows-2,0,-1)]
        for a,b in zip(boundary,boundary[1:]+boundary[:1]):
            faces.append([a,b,b+stride,a+stride])
        m=UsdGeom.Mesh.Define(stage,path)
        m.CreatePointsAttr(pts)
        m.CreateFaceVertexCountsAttr([len(x) for x in faces])
        m.CreateFaceVertexIndicesAttr([v for x in faces for v in x])
        m.CreateSubdivisionSchemeAttr('none')
        m.CreateDoubleSidedAttr(True)
        m.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
        return m

    def fruit(path,pos,radius=.032):
        p=UsdGeom.Sphere.Define(stage,path)
        p.CreateRadiusAttr(radius)
        p.AddTranslateOp().Set(Gf.Vec3d(*pos))
        p.CreateDisplayColorAttr([Gf.Vec3f(.75,.12,.035)])
        UsdPhysics.CollisionAPI.Apply(p.GetPrim())

    def plant(path,x,y,index=0):
        UsdGeom.Xform.Define(stage,path)
        z=BED_HEIGHT
        stem(path+'/MainStem',(x,y,z),(x+.025,y,z+.87),.010)
        for k in range(6):
            ang=k*2.399+index*.41
            dx,dy=math.cos(ang),math.sin(ang)
            start=(x,y,z+.22+k*.10)
            tip=(x+dx*.13,y+dy*.13,start[2]+.025)
            stem(path+f'/Branch{k}',start,tip,.005)
            leaf(path+f'/Leaf{k}',tip,(dx,dy),.20,.115,
                 (.10+.018*(k%3),.31+.025*(k%4),.055))
            if k in (1,3,4):
                fruit(path+f'/Tomato{k}',(tip[0],tip[1],tip[2]-.055))

    box('/World/Floor',(0,1.1,-.05),(14,15,.1),(.38,.33,.25))
    box('/World/Guide/EndConnector',(0,1.1,.001),(CONNECTOR_WIDTH,12,.002),(.54,.48,.35),False)
    widths=[.8,.65,.65,.6,.6]
    centers=[]
    center=AISLE_Y-AISLE_WIDTH/2-widths[0]/2
    beds=[]
    for i,width in enumerate(widths):
        if i:
            center+=widths[i-1]/2+AISLE_WIDTH+width/2
        centers.append(center)
        for side,sign in [('A',-1),('B',1)]:
            x=sign*(CONNECTOR_WIDTH/2+BED_LENGTH/2)
            path=f'/World/Beds/Bed_{i+1}_{side}'
            box(path,(x,center,BED_HEIGHT/2),(BED_LENGTH,width,BED_HEIGHT),(.31,.20,.105))
            beds.append([x-BED_LENGTH/2,x+BED_LENGTH/2,center-width/2,center+width/2])
            for k in range(5):
                px=sign*(CONNECTOR_WIDTH/2+.36+k*.78)
                plant(f'/World/Crops/Row{i+1}{side}/Plant{k}',px,center,k+i)
            # Irrigation pipe along the bed; it remains a visible collision object.
            stem(f'/World/Irrigation/Row{i+1}{side}',
                 (sign*(CONNECTOR_WIDTH/2+.04),center+.12,BED_HEIGHT+.015),
                 (sign*(CONNECTOR_WIDTH/2+BED_LENGTH-.04),center+.12,BED_HEIGHT+.015),
                 .012,(.08,.085,.07))
        ay=center+width/2+AISLE_WIDTH/2
        box(f'/World/Guide/Aisle_{i+1}',(0,ay,.001),
            (2*BED_LENGTH+CONNECTOR_WIDTH,AISLE_WIDTH,.002),(.51,.43,.31),False)
    # Simple open greenhouse frame provides context without transparent roof cost.
    for i,x in enumerate((-5.2,0,5.2)):
        for j,y in enumerate((-4.1,7.1)):
            stem(f'/World/Structure/Post{i}{j}',(x,y,0),(x,y,2.65),.035,(.55,.59,.56))
        stem(f'/World/Structure/Cross{i}',(x,-4.1,2.65),(x,7.1,2.65),.028,(.55,.59,.56))
    for j,y in enumerate((-4.1,7.1)):
        stem(f'/World/Structure/Rail{j}',(-5.2,y,2.65),(5.2,y,2.65),.028,(.55,.59,.56))

    obstacle='/World/TestObstacle'
    plant(obstacle,-1.10,centers[1],13)
    # A tomato vine grows out from its bed; broad leaves enter the walking corridor.
    a=(-1.10,centers[1],.74)
    b=(-1.05,AISLE_Y-.12,.51)
    stem(obstacle+'/IntrudingVine',a,b,.012)
    for i,t in enumerate((.42,.65,.86,1.0)):
        p=tuple(a[k]+(b[k]-a[k])*t for k in range(3))
        leaf(obstacle+f'/IntrudingLeaf{i}',p,(-1 if i%2 else 1,-.3),.42,.24,(.12,.40,.065))
        fruit(obstacle+f'/IntrudingFruit{i}',(p[0]+.025,p[1],p[2]-.07),.039)
    obstacle_prim=stage.GetPrimAtPath(obstacle)
    obstacle_prim.SetCustomData({'obstacle_type':'rigid tomato vine and foliage protruding from a bed',
                                'no_hidden_box':True,'flexible_plant_physics':False})
    UsdGeom.Imageable(obstacle_prim).CreateVisibilityAttr().Set('invisible')
    for p in stage.Traverse():
        if p.GetPath().HasPrefix(obstacle_prim.GetPath()) and p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
    stage.GetPrimAtPath('/World').SetCustomDataByKey('scene_layout',
        'Illustrative widened greenhouse; 1.35 m aisles, 1.8 m connector; not measured dimensions')
    return {'aisle_width_m':AISLE_WIDTH,'connector_width_m':CONNECTOR_WIDTH,
            'route_y_m':AISLE_Y,'bed_rectangles_xy_m':beds,
            'layout_status':'illustrative, not measured','obstacle_type':'crop foliage'}


def attach_mount_support(stage):
    """Add a visible saddle and risers as part of the existing fixed mount body."""
    from pxr import UsdGeom, UsdPhysics, Gf
    plate=stage.GetPrimAtPath(BASE_PATH+'/so101_mount_plate')
    if not plate:
        raise RuntimeError('Fixed SO101 mount body missing')
    joint=UsdPhysics.FixedJoint(stage.GetPrimAtPath('/World/AS2/Physics/so101_mount_to_as2'))
    if not joint or not joint.GetBody0Rel().GetTargets() or not joint.GetBody1Rel().GetTargets():
        raise RuntimeError('SO101 mount must have an actual fixed joint')
    # Parent coordinates: plate is centered at base (-.04,0,.15).
    parts=[('Saddle',(0,0,-.092),(.19,.17,.020))]
    for i,x in enumerate((-.043,.043)):
        for j,y in enumerate((-.048,.048)):
            parts.append((f'Riser{i}{j}',(x,y,-.045),(.021,.021,.078)))
    for name,pos,size in parts:
        cube=UsdGeom.Cube.Define(stage,str(plate.GetPath())+'/MountSupport/'+name)
        cube.CreateSizeAttr(1)
        cube.AddTranslateOp().Set(Gf.Vec3d(*pos))
        cube.AddScaleOp().Set(Gf.Vec3d(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.20,.23,.25)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    plate.SetCustomDataByKey('mount_support','Saddle and four risers welded to existing fixed mount body; provisional .12 kg assembly')
    return {'fixed_joint':str(joint.GetPath()),'support_parts':len(parts),
            'support_base_z_range_m':[.048,.144],'plate_bottom_base_z_m':.1425,
            'mount_mass_kg':.12,'mass_and_installation_provisional':True}
