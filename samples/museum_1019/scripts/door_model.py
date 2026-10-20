"""Parametric hall<->room04 glass double door (wood frame + glass), shared by both models."""
import json, numpy as np, trimesh
from trimesh.visual.material import PBRMaterial

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


