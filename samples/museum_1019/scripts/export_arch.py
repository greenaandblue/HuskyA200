import pickle, trimesh, os
from trimesh.visual.material import PBRMaterial
from door_model import add_door
parts = pickle.load(open('/home/claude/work/arch_parts.pkl', 'rb'))
floor = parts['FL']
sc = trimesh.Scene()
sc.add_geometry(floor, node_name='floor', geom_name='floor')
sc.add_geometry(parts['W'], node_name='walls', geom_name='walls')
sc.add_geometry(parts['L'], node_name='lintels', geom_name='lintels')
sc.add_geometry(parts['O'], node_name='exhibits', geom_name='exhibits')
add_door(sc)
p = '/mnt/user-data/outputs/museum_merged/model/museum_architectural.glb'
sc.export(p, include_normals=True)
print(p, round(os.path.getsize(p) / 2**20, 2), 'MiB')
