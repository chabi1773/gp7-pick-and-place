import json,math,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

P=Path(__file__).resolve().parents[1]
source=json.loads((P/'reference/mass_properties_input.json').read_text())
alignment={d['urdf_link']:d for d in json.loads((P/'checks/mesh_alignment.json').read_text())}
tree=ET.parse(P/'urdf/gp7.urdf');root=tree.getroot();root.set('name','gp7')
links={l.get('name'):l for l in root.findall('link')}
fmt=lambda a:' '.join(f'{v:.12g}' for v in a)
def tensor(a):
 xx,xy,xz,yy,yz,zz=a
 return np.array([[xx,-xy,-xz],[-xy,yy,-yz],[-xz,-yz,zz]],dtype=float)
report={'status':'provisional CAD mass properties; all source densities approximately 1 g/cm^3','mass_scale':.001,'length_scale':.001,'inertia_scale':1e-9,'inertia_notation':'negate source off-diagonal positive products, then rotate COM tensor into URDF link axes','links':[]}
for d in source['links']:
 name=d['link'];a=alignment[name];R=np.array(a['rotation_matrix']);t=np.array(a['visual_origin_xyz_m']);m=d['mass']*.001
 c=np.array(d['com'])*.001;I=tensor(d['L_com'])*1e-9
 com=R@c+t;Il=R@I@R.T;eig=np.linalg.eigvalsh(Il)
 assert (eig>0).all() and eig[0]+eig[1]>eig[2]
 principal_error=max(abs(np.linalg.eigvalsh(I)-np.array(d['principal'])*1e-9))
 assert principal_error<2e-11
 # The source COM and mass are rounded. Check its origin tensor using a bound
 # derived from 0.005 mm / 0.005 g half-unit report precision, rather than an
 # arbitrary threshold. This independently checks the cross-product sign.
 origin_I=tensor(d['I_origin'])*1e-9
 predicted=I+m*((c@c)*np.eye(3)-np.outer(c,c))
 dc=math.sqrt(3)*.005*.001;dm=.005*.001
 bound=m*(2*np.linalg.norm(c)*dc+dc*dc)+dm*(np.linalg.norm(c)+dc)**2+3e-11
 err=np.max(abs(predicted-origin_I));assert err<bound,(name,err,bound)
 link=links[name]
 for old in link.findall('inertial'):link.remove(old)
 inertia=ET.Element('inertial');link.insert(0,inertia)
 ET.SubElement(inertia,'origin',xyz=fmt(com),rpy='0 0 0');ET.SubElement(inertia,'mass',value=f'{m:.12g}')
 ET.SubElement(inertia,'inertia',**{key:f'{Il[i,j]:.12g}' for key,i,j in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})
 report['links'].append({'link':name,'assembly':d['assembly'],'mass_kg':m,'density_g_cm3':d['mass']/d['volume_mm3']*1000,'com_xyz_m':com.tolist(),'inertia_kg_m2':Il.tolist(),'principal_moments_kg_m2':eig.tolist(),'principal_reconstruction_error_kg_m2':principal_error,'parallel_axis_max_error_kg_m2':err,'parallel_axis_rounding_bound_kg_m2':bound})
report['total_mass_kg']=sum(d['mass_kg'] for d in report['links'])
root.insert(0,ET.Comment('Provisional physics: source CAD density is approximately 1 g/cm^3 for every link. See reference/mass_properties_input.json and checks/inertia_conversion.json. Mesh alignment remains estimated.'))
ET.indent(root,space='  ');tree.write(P/'urdf/gp7.urdf',encoding='utf-8',xml_declaration=True)
(P/'checks/inertia_conversion.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
