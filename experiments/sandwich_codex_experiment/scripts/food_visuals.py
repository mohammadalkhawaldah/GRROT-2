"""Original deterministic food meshes. Rendering only; no collision geometry.

Dimensions remain within the validated thick rigid grasp envelopes. COLLADA
materials and actual surface detail avoid texture/model download dependencies.
"""
from collections import defaultdict
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import Delaunay

ASSETS=Path(__file__).resolve().parents[1]/'models/food/meshes'

class Mesh:
    def __init__(self):self.faces=defaultdict(list);self.colors={}
    def material(self,name,color,shine=5):self.colors[name]=(color,shine);return name
    def tri(self,material,a,b,c):self.faces[material].append((a,b,c))
    def quad(self,material,a,b,c,d):self.tri(material,a,b,c);self.tri(material,a,c,d)
    def fan(self,material,outline,z,up=True):
        center=np.mean(outline,axis=0)
        for a,b in zip(outline,np.roll(outline,-1,axis=0)):
            pts=[(*center,z),(*a,z),(*b,z)]
            self.tri(material,*(pts if up else pts[::-1]))
    def band(self,material,lower,z1,upper,z2):
        for i in range(len(lower)):
            j=(i+1)%len(lower)
            self.quad(material,(*lower[i],z1),(*lower[j],z1),(*upper[j],z2),(*upper[i],z2))
    def save(self,name):
        root=ET.Element('COLLADA',xmlns='http://www.collada.org/2005/11/COLLADASchema',version='1.4.1')
        asset=ET.SubElement(root,'asset');ET.SubElement(asset,'created').text='2026-11-21T00:00:00Z'
        ET.SubElement(asset,'modified').text='2026-11-21T00:00:00Z'
        ET.SubElement(asset,'unit',name='meter',meter='1');ET.SubElement(asset,'up_axis').text='Z_UP'
        effects=ET.SubElement(root,'library_effects');materials=ET.SubElement(root,'library_materials')
        geos=ET.SubElement(root,'library_geometries')
        scene=ET.SubElement(ET.SubElement(root,'library_visual_scenes'),'visual_scene',id='Scene')
        for material,faces in self.faces.items():
            color,shine=self.colors[material];effect=ET.SubElement(effects,'effect',id=material+'-fx')
            phong=ET.SubElement(ET.SubElement(ET.SubElement(effect,'profile_COMMON'),'technique',sid='common'),'phong')
            for tag,value in [('ambient',[c*.65 for c in color[:3]]+[1]),('diffuse',color),('specular',[.12,.12,.12,1])]:
                ET.SubElement(ET.SubElement(phong,tag),'color').text=' '.join(map(str,value))
            ET.SubElement(ET.SubElement(phong,'shininess'),'float').text=str(shine)
            mat=ET.SubElement(materials,'material',id=material);ET.SubElement(mat,'instance_effect',url='#'+material+'-fx')
            geometry=ET.SubElement(geos,'geometry',id=material+'-geo');mesh=ET.SubElement(geometry,'mesh')
            xyz=np.array(faces,dtype=float).reshape(-1,3)
            normals=[]
            for f in np.array(faces):
                n=np.cross(f[1]-f[0],f[2]-f[0]);n/=max(np.linalg.norm(n),1e-15)
                normals.extend([n]*3)
            for label,data in [('position',xyz),('normal',np.array(normals))]:
                source=ET.SubElement(mesh,'source',id=material+'-'+label)
                ET.SubElement(source,'float_array',id=material+'-'+label+'-array',count=str(data.size)).text=' '.join(f'{v:.8g}' for v in data.flat)
                acc=ET.SubElement(ET.SubElement(source,'technique_common'),'accessor',source='#'+material+'-'+label+'-array',count=str(len(data)),stride='3')
                for axis in 'XYZ':ET.SubElement(acc,'param',name=axis,type='float')
            vertices=ET.SubElement(mesh,'vertices',id=material+'-vertices')
            ET.SubElement(vertices,'input',semantic='POSITION',source='#'+material+'-position')
            triangles=ET.SubElement(mesh,'triangles',count=str(len(faces)),material=material)
            ET.SubElement(triangles,'input',semantic='VERTEX',source='#'+material+'-vertices',offset='0')
            ET.SubElement(triangles,'input',semantic='NORMAL',source='#'+material+'-normal',offset='1')
            ET.SubElement(triangles,'p').text=' '.join(f'{i} {i}' for i in range(len(xyz)))
            node=ET.SubElement(scene,'node',id=material+'-node')
            inst=ET.SubElement(node,'instance_geometry',url='#'+material+'-geo')
            tc=ET.SubElement(ET.SubElement(inst,'bind_material'),'technique_common')
            ET.SubElement(tc,'instance_material',symbol=material,target='#'+material)
        ET.SubElement(ET.SubElement(root,'scene'),'instance_visual_scene',url='#Scene')
        path=ASSETS/(name+'.dae');ET.indent(root);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
        return path

def ellipse(cx,cy,rx,ry,n=48,angle=0):
    t=np.linspace(0,2*np.pi,n,endpoint=False);x=rx*np.cos(t);y=ry*np.sin(t)
    return np.column_stack([cx+x*np.cos(angle)-y*np.sin(angle),cy+x*np.sin(angle)+y*np.cos(angle)])

def inside(p,poly):
    x,y=p;odd=False
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:odd=not odd
    return odd

def perforated_face(mesh,outline,holes,z,materials,seed,up=True,through=False,depth=.001):
    rng=np.random.default_rng(seed);points=list(outline)
    rings=[]
    for x,y,rx,ry,angle in holes:
        ring=ellipse(x,y,rx,ry,12,angle);rings.append(ring);points.extend(ring)
    def in_hole(p):
        for x,y,rx,ry,a in holes:
            dx,dy=p[0]-x,p[1]-y;u=dx*math.cos(a)+dy*math.sin(a);v=-dx*math.sin(a)+dy*math.cos(a)
            if (u/rx)**2+(v/ry)**2<.95:return True
        return False
    lo=np.min(outline,axis=0);hi=np.max(outline,axis=0)
    for p in rng.uniform(lo,hi,(1300,2)):
        if inside(p,outline) and not in_hole(p):points.append(p)
    points=np.array(points)
    for ids in Delaunay(points).simplices:
        tri=points[ids];center=tri.mean(axis=0)
        if not inside(center,outline) or in_hole(center):continue
        a,b,c=[(*p,z) for p in tri]
        if np.cross(tri[1]-tri[0],tri[2]-tri[0])<0:b,c=c,b
        mesh.tri(materials[int(rng.integers(len(materials)))],*( [a,b,c] if up else [c,b,a]))
    for ring in rings:
        if depth==0 and not through:continue
        lower=z-depth if up else z+depth
        if through:lower=-z
        # Hole walls face into the cavity, rather than out of the food.
        for i in range(len(ring)):
            j=(i+1)%len(ring);a,b=ring[i],ring[j]
            mesh.quad('pore_wall',(*a,z),(*b,z),(*b,lower),(*a,lower))
        if not through:mesh.fan('pore_floor',ring*.8+np.mean(ring,axis=0)*.2,lower,up)

def bread():
    m=Mesh()
    m.material('crust',[.49,.20,.065,1]);m.material('crust_edge',[.69,.34,.12,1]);m.material('crust_bevel',[.78,.45,.20,1])
    shades=[m.material('crumb'+str(i),[.87+i*.012,.79+i*.012,.62+i*.016,1]) for i in range(7)]
    m.material('pore_wall',[.64,.49,.28,1]);m.material('pore_floor',[.41,.29,.14,1])
    # Classic loaf-slice outline: rounded base, shoulders, domed upper crust.
    outline=[[-.020,-.035],[.020,-.035],[.025,-.030],[.025,.004],[.025,.011]]
    for t in np.linspace(0,np.pi,49):outline.append([.025*np.cos(t),.014+.020*np.sin(t)+.001*np.sin(3*t)])
    outline += [[-.025,.011],[-.025,.004],[-.025,-.030]]
    outline=np.array(outline);inner=outline*np.array([.87,.90]);outer_bevel=outline*np.array([.98,.98])
    m.band('crust',outline,-.0175,outline,.0175)
    m.band('crust_edge',outer_bevel,-.02,outline,-.0175)
    m.band('crust_edge',outline,.0175,outer_bevel,.02)
    m.band('crust_bevel',outer_bevel,.02,inner,.0197)
    m.band('crust_bevel',inner,-.0197,outer_bevel,-.02)
    rng=np.random.default_rng(39);holes=[]
    for _ in range(260):
        x,y=rng.uniform([-.020,-.029],[.020,.028]);rx=rng.uniform(.00022,.0008);ry=rx*rng.uniform(.6,1.5)
        if not all(inside(p,inner) for p in ellipse(x,y,rx*1.5,ry*1.5,8)):continue
        if any(math.hypot(x-h[0],y-h[1])<1.3*(max(rx,ry)+max(h[2],h[3])) for h in holes):continue
        holes.append((x,y,rx,ry,rng.uniform(0,np.pi)))
    perforated_face(m,inner,holes,.0197,shades,55,depth=.0009)
    perforated_face(m,inner,holes,-.0197,shades,56,up=False,depth=.0009)
    return m.save('bread_slice')

def rounded_rectangle(rx,ry,r):
    points=[]
    for cx,cy,start in [(rx-r,ry-r,0),(-rx+r,ry-r,np.pi/2),(-rx+r,-ry+r,np.pi),(rx-r,-ry+r,3*np.pi/2)]:
        for a in np.linspace(start,start+np.pi/2,12,endpoint=False):points.append([cx+r*np.cos(a),cy+r*np.sin(a)])
    return np.array(points)

def cheese():
    m=Mesh();m.material('cheese_side',[.92,.62,.12,1],22);m.material('cheese_edge',[1,.81,.29,1],20)
    mats=[m.material('cheese_face'+str(i),[.98,.79+i*.006,.32+i*.008,1],18) for i in range(6)]
    m.material('pore_wall',[.83,.52,.10,1],16);m.material('pore_floor',[.68,.39,.07,1])
    outline=rounded_rectangle(.025,.03,.004);inner=outline*np.array([.97,.975])
    m.band('cheese_side',outline,-.016,outline,.016)
    m.band('cheese_edge',inner,-.0175,outline,-.016);m.band('cheese_edge',outline,.016,inner,.0175)
    holes=[(-.012,-.015,.004,.005,0),(.011,-.018,.003,.0035,.3),(.009,.009,.005,.006,-.4),(-.013,.019,.003,.0038,.4)]
    perforated_face(m,inner,holes,.0175,mats,73,through=True)
    perforated_face(m,inner,holes,-.0175,mats,74,up=False,through=False,depth=0)
    # An opening in the side also makes the material recognizable at arm view.
    # The through holes above remain rendering detail, not collision cavities.
    return m.save('cheese_slice')

def tomato():
    m=Mesh();m.material('skin',[.53,.018,.01,1],45);m.material('skin_edge',[.83,.035,.018,1],45)
    m.material('flesh',[.91,.105,.055,1],30);m.material('flesh_light',[.99,.24,.12,1],30)
    m.material('gel',[.59,.12,.035,1],50);m.material('gel_light',[.79,.24,.055,1],50)
    m.material('pith',[1,.47,.32,1],12);m.material('seed',[1,.83,.40,1],25)
    m.material('seed_edge',[.88,.57,.18,1],25)
    t=np.linspace(0,2*np.pi,128,endpoint=False);lobe=1+.012*np.cos(6*t)
    # Slightly flattened opposing sides align the visible grasp lands with the
    # existing envelope, while retaining an organic rounded slice outline.
    outline=np.column_stack([.02465*np.sign(np.cos(t))*np.abs(np.cos(t))**(2/3)*lobe,
                             .02955*np.sign(np.sin(t))*np.abs(np.sin(t))**(2/3)*lobe])
    inner=outline*.95
    m.band('skin',outline*.98,-.0155,outline,.0155)
    m.band('skin_edge',outline,.0155,inner,.0175)
    m.band('skin_edge',inner,-.0175,outline*.98,-.0155)
    for side in [-1,1]:
        z=side*.0170;m.fan('flesh',inner,z,up=side>0)
        rng=np.random.default_rng(103+side)
        core=ellipse(0,0,.0035,.0042,48);m.fan('pith',core,side*.01715,up=side>0)
        for a in np.linspace(0,2*np.pi,6,endpoint=False):
            c=np.array([.013*np.cos(a),.0155*np.sin(a)])
            pocket=ellipse(*c,.005,.0075,40,a)
            m.fan('flesh_light',pocket*1.13-c*.13,side*.01704,up=side>0)
            m.fan('gel',pocket,side*.01707,up=side>0)
            m.fan('gel_light',pocket*.78+c*.22,side*.01709,up=side>0)
            for u in [-.5,0,.5]:
                seed_center=c+np.array([-np.sin(a),np.cos(a)])*u*.007
                seed=ellipse(*seed_center,.00065,.0013,16,a+float(rng.uniform(-.4,.4)))
                m.fan('seed_edge',seed,side*.01712,up=side>0)
                m.fan('seed',seed*.75+seed_center*.25,side*.01717,up=side>0)
            b=a+np.pi/6;direction=np.array([np.cos(b),np.sin(b)])
            perpendicular=np.array([-np.sin(b),np.cos(b)])
            shape=[direction*.003-perpendicular*.0004,direction*.021-perpendicular*.0003,
                   direction*.021+perpendicular*.0003,direction*.003+perpendicular*.0004]
            m.fan('pith',np.array(shape)*np.array([1,1.1]),side*.01712,up=side>0)
    return m.save('tomato_slice')

def ensure_assets():
    ASSETS.mkdir(parents=True,exist_ok=True)
    files={'bread_bottom':'bread_slice','bread_top':'bread_slice','tomato':'tomato_slice','cheese':'cheese_slice'}
    for name,builder in [('bread_slice',bread),('tomato_slice',tomato),('cheese_slice',cheese)]:
        if not (ASSETS/(name+'.dae')).exists():builder()
    return {n:ASSETS/(f+'.dae') for n,f in files.items()}

if __name__=='__main__':
    ASSETS.mkdir(parents=True,exist_ok=True)
    for build in [bread,tomato,cheese]:print(build())
