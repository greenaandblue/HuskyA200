"""Decimate the scan mesh, add the parametric glass double door, export GLB levels of detail."""
import json, numpy as np, open3d as o3d, trimesh, fast_simplification
from scipy.spatial import cKDTree
from trimesh.visual.material import PBRMaterial

OUT = '/mnt/user-data/outputs/museum_merged/model'
import os; os.makedirs(OUT, exist_ok=True)
import sys
DEPTH = int(sys.argv[1]) if len(sys.argv) > 1 else 10
PREVIEW = '--preview' in sys.argv
src = o3d.io.read_triangle_mesh(f'/home/claude/work/scan_mesh_d{DEPTH}.ply')
src = src.filter_smooth_taubin(number_of_iterations=15)   # take residual VLP-16 range noise off flat walls
# floor comes from the flat slab: drop scan triangles lying on the floor
_V = np.asarray(src.vertices); _F = np.asarray(src.triangles)
src.remove_triangles_by_mask(_V[_F][:, :, 2].max(1) < 0.05); src.remove_unreferenced_vertices()
V0 = np.asarray(src.vertices); F0 = np.asarray(src.triangles); C0 = np.asarray(src.vertex_colors)
tree0 = cKDTree(V0)
print('source', len(F0), 'tris')


def shade(c):
    """Lidar intensity -> soft plaster/stone grey (keeps contrast, avoids black)."""
    g = np.clip(c[:, 0], 0, 1) ** 0.6
    base = np.array([0.80, 0.78, 0.74])
    return np.clip((0.45 + 0.55 * g)[:, None] * base, 0, 1)


def decimate(V, F, n_target):
    red = 1 - n_target / len(F)
    v, f = fast_simplification.simplify(V.astype(np.float32), F.astype(np.int32), target_reduction=max(red, 0.0))
    _, j = tree0.query(v, k=1)
    rgb = (shade(C0[j]) * 255).astype(np.uint8)
    m = trimesh.Trimesh(v, f, vertex_colors=np.hstack([rgb, np.full((len(v), 1), 255, np.uint8)]), process=False)
    return m


# ---------------- door ----------------
DOOR = json.load(open('/home/claude/work/door.json'))
H, L, T = DOOR['height'] - 0.01, DOOR['leaf_width'] - 0.01, DOOR['leaf_thickness']
STILE, TOP, BOT = 0.09, 0.12, 0.22
wood = PBRMaterial(name='door_wood', baseColorFactor=[0.42, 0.27, 0.15, 1.0], metallicFactor=0.0, roughnessFactor=0.65)
glass = PBRMaterial(name='door_glass', baseColorFactor=[0.78, 0.90, 0.95, 0.22], metallicFactor=0.0, roughnessFactor=0.05, alphaMode='BLEND', doubleSided=True)
brass = PBRMaterial(name='door_handle', baseColorFactor=[0.78, 0.62, 0.30, 1.0], metallicFactor=0.9, roughnessFactor=0.3)


def box(x0, x1, y0, y1, z0, z1, mat):
    b = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    b.visual = trimesh.visual.TextureVisuals(material=mat)
    return b


def leaf_parts(y_off):
    """Leaf in its hinge frame: width along +X from the hinge axis, thickness centred on y_off."""
    a, b = y_off - T / 2, y_off + T / 2
    frame = trimesh.util.concatenate([
        box(0, STILE, a, b, 0, H, wood), box(L - STILE, L, a, b, 0, H, wood),
        box(STILE, L - STILE, a, b, H - TOP, H, wood), box(STILE, L - STILE, a, b, 0, BOT, wood)])
    frame.visual = trimesh.visual.TextureVisuals(material=wood)
    pane = box(STILE, L - STILE, y_off - 0.006, y_off + 0.006, BOT, H - TOP, glass)
    handle = box(L - STILE - 0.05, L - STILE - 0.02, a - 0.06, b + 0.06, 0.95, 1.15, brass)
    return frame, pane, handle


def add_door(scene):
    for leaf in DOOR['leaves']:
        hx, hy = leaf['hinge']
        frame, pane, handle = leaf_parts(leaf['open_y'] - hy)
        M = np.eye(4); M[:3, 3] = [hx, hy, 0.0]          # hinge axis; scanned state = open 90 deg toward the hall
        pivot = leaf['name']
        scene.graph.update(frame_to=pivot, frame_from=scene.graph.base_frame, matrix=M)
        scene.add_geometry(frame, node_name=f'{pivot}_frame', geom_name=f'{pivot}_frame', parent_node_name=pivot)
        scene.add_geometry(pane, node_name=f'{pivot}_glass', geom_name=f'{pivot}_glass', parent_node_name=pivot)
        scene.add_geometry(handle, node_name=f'{pivot}_handle', geom_name=f'{pivot}_handle', parent_node_name=pivot)


def fix_scan_material(p):
    """glTF default material is fully metallic (renders black); give the scan a matte plaster material."""
    import pygltflib
    g = pygltflib.GLTF2().load(p)
    g.materials.append(pygltflib.Material(name='scan_surface', doubleSided=True,
        pbrMetallicRoughness=pygltflib.PbrMetallicRoughness(baseColorFactor=[1, 1, 1, 1], metallicFactor=0.0, roughnessFactor=0.92)))
    mi = len(g.materials) - 1
    for m in g.meshes:
        if m.name.startswith('museum_scan'):
            for pr in m.primitives: pr.material = mi
    g.save(p)


FLOOR = trimesh.load('/home/claude/work/floor_mesh.ply', process=False)
FLOOR.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(name='floor', baseColorFactor=[0.66, 0.62, 0.56, 1.0], metallicFactor=0.0, roughnessFactor=0.8, doubleSided=True))


def export(mesh, name, with_door=True, floor_sel=None):
    sc = trimesh.Scene()
    sc.add_geometry(mesh, node_name='museum_scan', geom_name='museum_scan')
    fl = FLOOR
    if floor_sel is not None:
        c = FLOOR.triangles_center[:, 0]; fl = FLOOR.submesh([np.where(floor_sel(c))[0]], append=True)
        fl.visual = trimesh.visual.TextureVisuals(material=FLOOR.visual.material)
    sc.add_geometry(fl, node_name='museum_floor', geom_name='museum_floor')
    if with_door:
        add_door(sc)
    p = f'{OUT}/{name}.glb'
    sc.export(p, include_normals=True)
    fix_scan_material(p)
    print(name, len(mesh.faces), 'tris', round(os.path.getsize(p) / 2**20, 1), 'MiB', flush=True)
    return p


web = decimate(V0, F0, 320_000)
export(web, 'museum_scan_model_web')
if PREVIEW:
    sys.exit()
full = decimate(V0, F0, len(F0))          # full depth-10 resolution (colour transfer only)
export(full, 'museum_scan_model')
