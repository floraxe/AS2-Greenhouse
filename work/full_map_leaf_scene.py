"""Full-map-only light-leaf-contact approximation; original scene is untouched.

Leaves remain visible to camera/physics raycasts. Robot feet, bed bodies and all
original stem geometry retain physical collision. Not a calibrated elastic model.
"""
import full_map_surveyed_scene as original
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, PhysxSchema

BASE, START, END = original.BASE, original.START, original.END

# Scene simplification: contract each leaf about its attachment, never the bed
# or stem. Camera, collision mesh and LiDAR all see the same smaller geometry.
LEAF_HORIZONTAL_SCALE = 0.80

def contract_leaf(points, veins, first_point, first_vein, origin):
    def contract(p):
        return (origin[0] + LEAF_HORIZONTAL_SCALE * (p[0] - origin[0]),
                origin[1] + LEAF_HORIZONTAL_SCALE * (p[1] - origin[1]), p[2])
    points[first_point:] = [contract(p) for p in points[first_point:]]
    veins[first_vein:] = [[contract(p) for p in line] for line in veins[first_vein:]]


def build(stage):
    registry = {}
    prototype_groups = {}
    plant_class, mesh_function = original.PlantMesh, original.mesh

    class SeparatedPlant(plant_class):
        def __init__(self, rng):
            super().__init__(rng)
            self.leaf_faces, self.stem_faces = [], []
            self.leaf_groups = []
            registry[id(self.p)] = self

        def leaf(self, origin, angle, length, width, rise, color, serrated=False):
            # Two staggered blades per original leaf; density is added inside
            # the planted bed, not through invisible navigation geometry.
            for rotation,scale,tint in ((0.,1.,1.),(.95,.86,.93)):
                first=len(self.f);first_point=len(self.p);first_vein=len(self.veins)
                super().leaf(origin,angle+rotation,length*scale,width*scale,
                             rise*scale,tuple(min(1.,c*tint) for c in color),serrated)
                contract_leaf(self.p,self.veins,first_point,first_vein,origin)
                self.leaf_faces.extend(self.f[first:])
                self.leaf_groups.append((list(range(first_point,len(self.p))),
                                         list(origin),first_vein,len(self.veins)))

        def stem(self, *args, **kwargs):
            first = len(self.f)
            super().stem(*args, **kwargs)
            self.stem_faces.extend(self.f[first:])

    def compact_mesh(scene, path, plant, faces, material):
        used = sorted({i for face in faces for i in face})
        remap = {index: i for i, index in enumerate(used)}
        return mesh_function(scene, path, [plant.p[i] for i in used],
                             [[remap[i] for i in face] for face in faces],
                             [plant.c[i] for i in used], material, True)

    def separated_mesh(scene, path, points, faces, colors, material, collision=False):
        plant = registry.get(id(points))
        if path.endswith('/Foliage') and plant:
            leaf = compact_mesh(scene, path, plant, plant.leaf_faces, material)
            used=sorted({i for face in plant.leaf_faces for i in face})
            remap={index:i for i,index in enumerate(used)}
            prototype_groups[path.rsplit('/',1)[0]]=[
                ([remap[i] for i in indices],anchor,v0,v1)
                for indices,anchor,v0,v1 in plant.leaf_groups]
            if plant.stem_faces:
                compact_mesh(scene, path.rsplit('/', 1)[0] + '/Stems', plant,
                             plant.stem_faces, material)
            return leaf
        return mesh_function(scene, path, points, faces, colors, material, collision)

    original.PlantMesh, original.mesh = SeparatedPlant, separated_mesh
    try:
        layout = original.build(stage)
    finally:
        original.PlantMesh, original.mesh = plant_class, mesh_function

    # Keep complete leaf shapes, contracting about their real attachment
    # only when needed to fit the measured bed and the 90cm pipe gap.
    # The edited meshes are shared by RGB, physical raycasts and collisions.
    plant_count=leaf_count=0;minimum_inset=float('inf')
    for row_prim in list(stage.GetPrimAtPath('/World/Crops').GetChildren()):
        row=int(row_prim.GetName().replace('Row',''))-1
        bed=layout['bed_rectangles_xy_m'][row]
        for plant_prim in list(row_prim.GetChildren()):
            refs=plant_prim.GetMetadata('references')
            proto=str(refs.GetAddedOrExplicitItems()[0].primPath)
            groups=prototype_groups[proto]
            plant_prim.SetInstanceable(False)
            matrix=np.asarray(UsdGeom.Xformable(plant_prim).ComputeLocalToWorldTransform(0))
            leaf_mesh=UsdGeom.Mesh(stage.GetPrimAtPath(str(plant_prim.GetPath())+'/Foliage'))
            points=np.array(leaf_mesh.GetPointsAttr().Get(),dtype=float)
            vein=UsdGeom.BasisCurves(stage.GetPrimAtPath(str(plant_prim.GetPath())+'/Veins'))
            veins=np.array(vein.GetPointsAttr().Get(),dtype=float) if vein else None
            px=matrix[3,0]
            left_section,right_section=layout['crop_sections_x_m']
            xlo,xhi=left_section if px<0 else right_section
            low=np.array([xlo+.025,bed[2]+.025]);high=np.array([xhi-.025,bed[3]-.025])
            for indices,anchor,v0,v1 in groups:
                anchor=np.asarray(anchor);world_anchor=anchor@matrix[:3,:3]+matrix[3,:3]
                if np.any(world_anchor[:2]<low) or np.any(world_anchor[:2]>high):
                    raise RuntimeError('Leaf attachment outside planted bed interior')
                world=points[indices]@matrix[:3,:3]+matrix[3,:3]
                delta=world[:,:2]-world_anchor[:2];factor=1.
                for axis in (0,1):
                    pos=delta[:,axis].max();neg=delta[:,axis].min()
                    if pos>0:factor=min(factor,(high[axis]-world_anchor[axis])/pos)
                    if neg<0:factor=min(factor,(low[axis]-world_anchor[axis])/neg)
                factor=float(np.clip(factor,0.,1.))
                points[indices,:2]=anchor[:2]+factor*(points[indices,:2]-anchor[:2])
                if veins is not None:
                    veins[2*v0:2*v1,:2]=anchor[:2]+factor*(veins[2*v0:2*v1,:2]-anchor[:2])
            leaf_mesh.GetPointsAttr().Set(points.tolist())
            if veins is not None:vein.GetPointsAttr().Set(veins.tolist())
            world=points@matrix[:3,:3]+matrix[3,:3]
            minimum_inset=min(minimum_inset,float((world[:,:2]-np.array([xlo,bed[2]])).min()),
                              float((np.array([xhi,bed[3]])-world[:,:2]).min()))
            plant_count+=1;leaf_count+=len(groups)
    layout['dense_canopy']={'plants':plant_count,'leaves':leaf_count,
                            'leaves_per_original':2,'minimum_bed_inset_m':minimum_inset,
                            'same_mesh_for_render_lidar_collision':True}

    leaves, body = [], []
    stems, feet = [], []
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            if path.startswith('/World/Crops/') and path.endswith('/Foliage'):
                leaves.append(prim.GetPath())
            elif path.startswith('/World/Crops/') and path.endswith('/Stems'):
                stems.append(prim.GetPath())
            elif path.startswith('/World/AS2/'):
                (feet if '_foot/' in path or path.endswith('_foot') else body).append(prim.GetPath())
        if path.startswith('/World/AS2/') and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            # Enable before the physics/tensor view is initialized.
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)

    if not leaves or not feet or not body or not stems:
        raise RuntimeError('Incomplete leaf/body/feet/stem collision partition')
    leaf_group = UsdPhysics.CollisionGroup.Define(stage, '/World/LightLeafContact/Leaves')
    body_group = UsdPhysics.CollisionGroup.Define(stage, '/World/LightLeafContact/Body')
    for group, paths in [(leaf_group, leaves), (body_group, body)]:
        collection = group.GetCollidersCollectionAPI()
        collection.CreateExpansionRuleAttr().Set(Usd.Tokens.explicitOnly)
        collection.CreateIncludesRel().SetTargets(paths)
    leaf_group.CreateFilteredGroupsRel().SetTargets([body_group.GetPath()])
    body_group.CreateFilteredGroupsRel().SetTargets([leaf_group.GetPath()])
    layout['crop_geometry'] = {
        'policy': 'dense layered leaves constrained inside measured beds and crop sections',
        'leaf_horizontal_scale_about_attachment': LEAF_HORIZONTAL_SCALE,
        'leaf_height_unchanged': True, 'stems_and_beds_unchanged': True,
        'same_geometry_for_camera_lidar_and_collision': True,
    }
    layout['leaf_contact_model'] = {
        'type': 'bounded light-contact approximation; no calibrated elasticity',
        'leaf_colliders': len(leaves), 'rigid_stem_colliders': len(stems),
        'foot_colliders': len(feet), 'soft_contact_body_colliders': len(body),
        'camera_and_lidar_leaves_retained': True,
        'foot_leaf_and_all_stem_bed_collisions_retained': True,
    }
    return layout
