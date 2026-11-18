"""Photo-guided small solar greenhouse, no downloaded assets or textures.
Measured bed widths/gaps; length/height/manifold placement provisional.
No transverse travel: irrigation manifolds are policy no-cross boundaries.
"""
import math,json
import numpy as np
from pathlib import Path
from pxr import Usd,UsdGeom,UsdShade,UsdLux,UsdPhysics,Sdf,Gf
from aisle_mission import TURN_CENTER,TURN_BAY_BOUNDS
ROOT=Path('E:/AS2_Greenhouse')
BASE='/World/AS2/Geometry/world/base_link'
START=(-3.0,-.825);END=TURN_CENTER
WIDTHS=[.8,.65,.65,.6,.6];GAPS=[.65,.55,.6,.65]
RIGHT_MARGIN=.30;LEFT_MARGIN=.80
CROP_SECTIONS=((-20.45,-.45),(.45,20.45))
CROP_MIN,CROP_MAX=CROP_SECTIONS[0][0],CROP_SECTIONS[1][1]
CENTERS=[3.6,2.225,1.025,-.2,-1.45]
# Practical presentation layout: internal surveyed geometry is preserved while
# both outer margins are compact. The overall width is no longer forced to 8.70 m.
YMIN=CENTERS[-1]-WIDTHS[-1]/2-RIGHT_MARGIN;YMAX=CENTERS[0]+WIDTHS[0]/2+LEFT_MARGIN
XBOUND=24.05

def material(stage,name,color,roughness=.8,metallic=0,vertex=False,opacity=1):
 path='/World/LabMaterials/'+name;m=UsdShade.Material.Define(stage,path);s=UsdShade.Shader.Define(stage,path+'/Surface');s.CreateIdAttr('UsdPreviewSurface')
 s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(roughness);s.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metallic)
 if vertex:
  r=UsdShade.Shader.Define(stage,path+'/Color');r.CreateIdAttr('UsdPrimvarReader_float3');r.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('displayColor');s.GetInput('diffuseColor').ConnectToSource(r.ConnectableAPI(),'result')
 if opacity<1:s.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(opacity)
 m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return m

def bind(prim,mat):UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
def mesh(stage,path,pts,faces,colors,mat,collision=False):
 m=UsdGeom.Mesh.Define(stage,path);m.CreatePointsAttr(pts);m.CreateFaceVertexCountsAttr([len(f) for f in faces]);m.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);m.CreateSubdivisionSchemeAttr('none');m.CreateDoubleSidedAttr(True)
 m.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(colors);bind(m.GetPrim(),mat)
 if collision:UsdPhysics.CollisionAPI.Apply(m.GetPrim())
 return m

def tube(stage,path,a,b,r,color,mat,collision=True):
 a,b=Gf.Vec3d(*a),Gf.Vec3d(*b);v=b-a;c=UsdGeom.Cylinder.Define(stage,path);c.CreateRadiusAttr(r);c.CreateHeightAttr(v.GetLength());c.AddTranslateOp().Set((a+b)/2);c.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1),v.GetNormalized()).GetQuat()));c.CreateDisplayColorAttr([Gf.Vec3f(*color)]);bind(c.GetPrim(),mat)
 if collision:UsdPhysics.CollisionAPI.Apply(c.GetPrim())
 return c

class PlantMesh:
 def __init__(self,rng):self.p=[];self.f=[];self.c=[];self.rng=rng;self.veins=[]
 def leaf(self,origin,angle,length,width,rise,color,serrated=False):
  off=len(self.p);nr,nc=9,7;d=np.array([math.cos(angle),math.sin(angle)]);per=np.array([-d[1],d[0]])
  for i in range(nr):
   t=i/(nr-1);half=max(.0008,width*.5*math.sin(math.pi*t)**.7)
   if serrated:half*=1+.10*math.cos(t*14*math.pi)
   for j in range(nc):
    v=2*j/(nc-1)-1;xy=np.array(origin[:2])+d*(t*length)+per*(v*half)
    z=origin[2]+rise*t-.07*t*t+.018*math.sin(math.pi*t)*(1-v*v)+.008*math.sin(t*8*math.pi+v*2)*abs(v)
    self.p.append((float(xy[0]),float(xy[1]),float(z)))
    vein=.16*math.exp(-abs(v)*18)+.04*math.cos(t*12*math.pi+abs(v)*5);shade=.84+.16*t+vein
    self.c.append(tuple(float(np.clip(x*shade,0,1)) for x in color))
   if i in (1,3,5,7):
    self.veins.append([self.p[off+i*nc+nc//2],self.p[off+i*nc+1]])
    self.veins.append([self.p[off+i*nc+nc//2],self.p[off+i*nc+nc-2]])
  for i in range(nr-1):
   for j in range(nc-1):a=off+i*nc+j;self.f.append([a,a+1,a+nc+1,a+nc])
 def stem(self,a,b,r=.003,color=(.17,.24,.045)):
  a=np.array(a);b=np.array(b);v=b-a;v=v/np.linalg.norm(v);side=np.cross(v,[0,0,1])
  if np.linalg.norm(side)<.1:side=np.array([1.,0,0])
  side/=np.linalg.norm(side);other=np.cross(v,side);off=len(self.p)
  for p in [a,b]:
   for k in range(5):
    p2=p+r*(math.cos(k*2*math.pi/5)*side+math.sin(k*2*math.pi/5)*other);self.p.append(tuple(p2));self.c.append(color)
  for k in range(5):self.f.append([off+k,off+(k+1)%5,off+5+(k+1)%5,off+5+k])

def build(stage):
 for p in ['/World/Beds','/World/Crops','/World/Guide','/World/Structure','/World/Irrigation','/World/TestObstacle','/World/PartialCrop','/World/Floor','/World/RouteGuide','/World/JobPoints']:
  if stage.GetPrimAtPath(p):stage.RemovePrim(p)
 rng=np.random.default_rng(2410)
 soil=material(stage,'Soil',(.24,.17,.105),.97,vertex=True)
 film=material(stage,'Mulch',(.045,.050,.047),.38,vertex=True)
 green=material(stage,'Leaves',(.16,.31,.075),.68,vertex=True)
 steel=material(stage,'GalvanizedSteel',(.38,.43,.43),.39,.72)
 hose=material(stage,'Hose',(.025,.033,.029),.78)
 bamboo=material(stage,'Bamboo',(.36,.27,.13),.9)
 twine=material(stage,'Twine',(.70,.69,.57),.91)
 black=material(stage,'Insulation',(.055,.075,.075),.91,vertex=True)
 shell=material(stage,'PolyFilm',(.66,.70,.67),.85,opacity=.12)
 # Flat collision surface + subtly irregular visible soil. No downloaded textures.
 floor=UsdGeom.Cube.Define(stage,'/World/Floor');floor.CreateSizeAttr(1);floor.AddTranslateOp().Set(Gf.Vec3d(0,(YMIN+YMAX)/2,-.05));floor.AddScaleOp().Set(Gf.Vec3d(2*(XBOUND+.2),YMAX-YMIN+1,.1));floor.CreateDisplayColorAttr([Gf.Vec3f(.23,.17,.11)]);UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
 pts=[];colors=[];faces=[];nx,ny=100,70
 for i in range(nx+1):
  x=-XBOUND+(2*XBOUND)*i/nx
  for j in range(ny+1):
   y=YMIN-.5+(YMAX-YMIN+1)*j/ny;v=float(rng.uniform(.83,1.15));z=.001+float(rng.uniform(0,.002));pts.append((x,y,z));colors.append((.145*v,.095*v,.057*v))
 for i in range(nx):
  for j in range(ny):a=i*(ny+1)+j;faces.append([a,a+ny+1,a+ny+2,a+1])
 mesh(stage,'/World/LabSoil',pts,faces,colors,soil)
 beds=[]
 for row,(cy,w) in enumerate(zip(CENTERS,WIDTHS)):
  pts=[];colors=[];faces=[];cross=[(-w*.5,.015),(-w*.45,.085),(-w*.32,.16),(0,.18),(w*.32,.16),(w*.45,.085),(w*.5,.015)]
  bed_x=np.concatenate((np.linspace(*CROP_SECTIONS[0],201),np.linspace(*CROP_SECTIONS[1],201)))
  for i,x in enumerate(bed_x):
   for j,(dy,z) in enumerate(cross):
    wrinkle=.006*math.sin(x*33+row+j*.8)+.003*math.sin(x*97+j)
    pts.append((x,cy+dy,z+wrinkle));v=float(rng.uniform(.8,1.2));colors.append((.045*v,.050*v,.047*v))
  for i in range(len(bed_x)-1):
   if bed_x[i]<0<bed_x[i+1]:continue
   for j in range(6):a=i*7+j;faces.append([a,a+7,a+8,a+1])
  mesh(stage,f'/World/Beds/Bed{row+1}',pts,faces,colors,film,True);beds.append([CROP_MIN,CROP_MAX,cy-w/2,cy+w/2])
  tube(stage,f'/World/Irrigation/Feed{row}',(CROP_MIN-.1,cy,.18),(CROP_MAX+.1,cy,.18),.013,(.025,.035,.03),hose)
  # 90cm pipe band only on bed footprints; all longitudinal aisles continue.
  for k,x in enumerate(np.arange(CROP_MIN+.3,CROP_MAX-.2,.45 if row<3 else .57)):
   if abs(x)<.72:continue
   for side in ([-1,1] if row<3 else [0]):
    variant=(k+row+int(side))%3;kind='Strawberry' if row==0 else 'Greens' if row<3 else 'Tomato';proto=f'/PlantLibrary/{kind}{variant}'
    if not stage.GetPrimAtPath(proto):
     prim=stage.CreateClassPrim(proto);prim.SetTypeName('Xform');r=np.random.default_rng(row*40+variant+2);pm=PlantMesh(r)
     if kind=='Greens':
      for n in range(13):
       a=n*2.399+float(r.uniform(-.16,.16));length=float(r.uniform(.17,.30));pm.leaf((0,0,.025),a,length,float(r.uniform(.07,.12)),float(r.uniform(.08,.19)),(.22+float(r.uniform(-.035,.035)),.39+float(r.uniform(-.04,.04)),.065),True)
     elif kind=='Strawberry':
      for n in range(5):
       a=n*2.399;end=(.10*math.cos(a),.10*math.sin(a),float(r.uniform(.10,.18)));pm.stem((0,0,.01),end,.003)
       for l in range(3):pm.leaf(end,a+(l-1)*.82,.15,.105,.012,(.10,.26,.065),True)
     else:
      height=float(r.uniform(1.05,1.34));pm.stem((0,0,0),(.025,0,height),.007)
      for n in range(9):
       a=n*2.399+variant;z=.18+n*.12;tip=(.13*math.cos(a),.13*math.sin(a),z+.03);pm.stem((.015,0,z),tip,.003)
       for l in range(3):pm.leaf(tip,a+(l-1)*.65,float(r.uniform(.13,.21)),.055,.02,(.09,.19,.038),True)
     mesh(stage,proto+'/Foliage',pm.p,pm.f,pm.c,green,True)
     if pm.veins:
      c=UsdGeom.BasisCurves.Define(stage,proto+'/Veins');c.CreateTypeAttr('linear');c.CreateCurveVertexCountsAttr([2]*len(pm.veins));c.CreatePointsAttr([p for line in pm.veins for p in line]);c.CreateWidthsAttr([.0009]);c.SetWidthsInterpolation(UsdGeom.Tokens.constant);c.CreateDisplayColorAttr([Gf.Vec3f(.28,.40,.12)])
    path=f'/World/Crops/Row{row+1}/Plant{k}_{side+1}';inst=UsdGeom.Xform.Define(stage,path);inst.GetPrim().GetReferences().AddInternalReference(proto);inst.GetPrim().SetInstanceable(True);inst.AddTranslateOp().Set(Gf.Vec3d(float(x+rng.uniform(-.045,.045)),float(cy+side*w*.19),.17));inst.AddRotateZOp().Set(float(rng.uniform(0,360)));sc=float(rng.uniform(.86,1.11));inst.AddScaleOp().Set(Gf.Vec3d(sc,sc,sc))
   if row>=3:
    tube(stage,f'/World/Trellis/Post{row}_{k}',(x,cy,.13),(x,cy,1.92),.009,(.38,.28,.12),bamboo)
    tube(stage,f'/World/Trellis/Tie{row}_{k}',(x+.05,cy-.06,.32),(x+.05,cy-.06,1.92),.0025,(.7,.69,.57),twine,False)
  if row>=3:
   for z in [.7,1.15,1.9]:tube(stage,f'/World/Trellis/Rail{row}_{int(z*100)}',(CROP_MIN-.1,cy,z),(CROP_MAX+.1,cy,z),.009,(.38,.28,.12),bamboo)
 # Measured 90cm longitudinal break between collinear crop sections.
 # Pipe material occupies bed footprints ONLY, not longitudinal walking aisles.
 for row,(cy,w) in enumerate(zip(CENTERS,WIDTHS)):
  for k,x in enumerate((-.35,0.,.35)):
   p=tube(stage,f'/World/Irrigation/MiddleBand{row}_{k}',(x,cy-w*.45,.045),(x,cy+w*.45,.045),.025,(.025,.035,.03),hose)
   p.GetPrim().SetCustomData({'navigation_policy':'NO_CROSS bed-only pipe zone; longitudinal aisles remain open'})
 # Photo placement is provisional: irrigation links stay over beds, not across the assigned turn bay.
 for side,x in [('Near',CROP_MIN-.04),('Far',CROP_MAX+.04)]:
  for row,(cy,w) in enumerate(zip(CENTERS,WIDTHS)):
   p=tube(stage,f'/World/Irrigation/{side}BedLink{row}',(x,cy-w*.4,.05),(x,cy+w*.4,.05),.025,(.025,.035,.03),hose)
   p.GetPrim().SetCustomData({'navigation_policy':'NO_CROSS crop beds and irrigation; same aisle identity retained','placement':'provisional; verify actual piping before hardware'})
 # The roof frame is intentionally omitted from the presentation scene. It
 # previously obscured the overview and is not part of the navigation test.
 span=YMAX-YMIN
 # Dark insulated side, textured with small deterministic geometric folds.
 pts=[];cols=[];faces=[]
 for i in range(81):
  x=-XBOUND+(2*XBOUND)*i/80
  for j in range(13):
   z=j*2.65/12;y=YMIN-.015-.012*math.sin(x*31+z*2);pts.append((x,y,z));v=.8+.18*math.sin(x*11+z*6)**2;cols.append((.065*v,.085*v,.085*v))
 for i in range(80):
  for j in range(12):a=i*13+j;faces.append([a,a+13,a+14,a+1])
 mesh(stage,'/World/Structure/InsulatedSide',pts,faces,cols,black,True)
 # Light roof film is a cutaway by default for inspection, not a solid cover over the camera.
 pts=[];faces=[]
 for x in [-XBOUND,XBOUND]:
  for j in range(19):t=j/18;pts.append((x,YMIN+span*t,1.08+2.25*math.sin(math.pi*t)**.78+.03))
 for j in range(18):faces.append([j,j+19,j+20,j+1])
 roof=mesh(stage,'/World/RoofFilm',pts,faces,[(.66,.70,.67)]*len(pts),shell);roof.GetVisibilityAttr().Set('invisible')
 # A neutral, non-intruding end wall gives visual landmarks; no stylized guide colors.
 rear=UsdGeom.Cube.Define(stage,'/World/Structure/EndWall');rear.CreateSizeAttr(1);rear.AddTranslateOp().Set(Gf.Vec3d(XBOUND,(YMIN+YMAX)/2,1.25));rear.AddScaleOp().Set(Gf.Vec3d(.05,span,2.5));rear.CreateDisplayColorAttr([Gf.Vec3f(.12,.14,.13)]);UsdPhysics.CollisionAPI.Apply(rear.GetPrim());bind(rear.GetPrim(),black)
 for lamp_index,x in enumerate(np.linspace(-19,19,9)):
  tube(stage,f'/World/Structure/LampCable{lamp_index}',(x,(YMIN+YMAX)/2,3.32),(x,(YMIN+YMAX)/2,2.9),.003,(.3,.3,.28),steel,False)
  p=UsdGeom.Sphere.Define(stage,f'/World/Structure/Lamp{lamp_index}');p.CreateRadiusAttr(.075);p.AddTranslateOp().Set(Gf.Vec3d(x,(YMIN+YMAX)/2,2.88));p.AddScaleOp().Set(Gf.Vec3d(1,1,.3));p.CreateDisplayColorAttr([Gf.Vec3f(.55,.55,.45)])
 # Soft daytime illumination, no cinematic overlays.
 UsdLux.DomeLight(stage.GetPrimAtPath('/World/Light')).GetIntensityAttr().Set(550)
 sun=UsdLux.DistantLight.Define(stage,'/World/LabSun');sun.CreateIntensityAttr(1200);sun.CreateAngleAttr(12);sun.AddRotateXYZOp().Set(Gf.Vec3f(18,-45,-25))
 from showcase_visuals import camera
 camera(stage,'/World/LabOverview',(-42.,13.,38.),(0.,1.4,.35),16)
 camera(stage,'/World/LabAisle',(CROP_MIN-1.,START[1],1.35),(CROP_MIN+1.,START[1],.25),10)
 camera(stage,'/World/LabInterior',(-27.,2.,2.1),(-18.,2.,.8),19)
 data={'measured_bed_widths_m':WIDTHS,'measured_internal_aisle_widths_m':GAPS,'bed_centers_y':CENTERS,'presentation_inner_width_m':YMAX-YMIN,'route_start':START,'route_end':END,'mode':'serpentine aisles 2 to 6; no A*','provisional':['overall greenhouse width relaxed for presentation','manifold diameter','plant heights/mount'],'irrigation_links_x':[CROP_MIN-.04,CROP_MAX+.04],'end_bays_x':[[-XBOUND,CROP_MIN],[CROP_MAX,XBOUND]],'turn_center':list(TURN_CENTER),'entry_view':'look along +X: strawberry left, tall tomatoes right','bed_rectangles_xy_m':beds,'assigned_turn_bay_xy':list(TURN_BAY_BOUNDS),'outer_bounds_xy':[-XBOUND,XBOUND,YMIN,YMAX],'photo_inference':['row1 strawberry','row2/3 leafy vegetables','row4/5 trellised tomato'],'roof_frame_visible':False}
 data['survey_revision']='user photo + clarification 2026-11-01'
 data['measured_pipe_band_width_m']=.90
 data['pipe_zones_xy_m']=[[-.45,.45,cy-w/2,cy+w/2] for cy,w in zip(CENTERS,WIDTHS)]
 data['crop_sections_x_m']=[list(v) for v in CROP_SECTIONS]
 data['pipe_band_crosses_longitudinal_aisles']=False
 data['measured_right_outer_aisle_width_m']=RIGHT_MARGIN
 data['outer_margins_m']={'right_assigned_from_description':RIGHT_MARGIN,'left_remainder':LEFT_MARGIN}
 data['assumed_pipe_band_center_x_m']=0.
 data['provisional']+=['pipe band longitudinal position centered at X=0','compact right/left outer margins 0.30m/0.80m selected for the navigation presentation']
 stage.GetPrimAtPath('/World').SetCustomDataByKey('lab_scene_note','Photo-guided crop types and structure. Bed/gap widths measured; other dimensions provisional. Pipes are forbidden crossings. No photogrammetric reconstruction.')
 (ROOT/'outputs/lab_scene_layout.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
 return data
