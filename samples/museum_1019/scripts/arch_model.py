"""Clean architectural model: walls / exhibits extruded from the lidar floor plan, lintels, flat floor, glass door."""
import json, numpy as np, open3d as o3d, trimesh, cv2
from scipy import ndimage
from shapely.geometry import Polygon, box as sbox
from shapely.ops import unary_union
from trimesh.visual.material import PBRMaterial

R = 0.05
pc = o3d.io.read_point_cloud('/home/claude/work/mesh_input.ply')
P = np.asarray(pc.points); N = np.asarray(pc.normals)
mk = np.load('/home/claude/work/final_mask.npz'); M10, lo10 = mk['M'], mk['lo']
lo = P[:, :2].min(0) - 1.0
shape = tuple(np.ceil((P[:, :2].max(0) + 1.0 - lo) / R).astype(int))
ij = np.floor((P[:, :2] - lo) / R).astype(int)
vert = np.abs(N[:, 2]) < 0.5
z = P[:, 2]


def count(sel):
    g = np.zeros(shape, np.int32); np.add.at(g, (ij[sel, 0], ij[sel, 1]), 1); return g


# occupancy from all three recordings (denser; doubled wall faces only thicken walls slightly)
A_ = np.load('/home/claude/work/merged_xyzr.npy')
_keep = np.load('/home/claude/work/keep_mask.npy') if False else None
_ij = np.floor((A_[:, :2] - lo) / R).astype(int)
_ok = (_ij >= 0).all(1) & (_ij[:, 0] < shape[0]) & (_ij[:, 1] < shape[1]) & (A_[:, 2] > 0.3) & (A_[:, 2] < 2.0)
_D = json.load(open('/home/claude/work/door.json'))
for _lf in _D['leaves']:          # door leaves are modelled separately, keep them out of the walls
    _hx, _hy = _lf['hinge']
    _ok &= ~((A_[:, 0] > _hx - 0.02) & (A_[:, 0] < _hx + _D['leaf_width'] + 0.15) & (np.abs(A_[:, 1] - _lf['open_y']) < 0.13))
low = np.zeros(shape, np.int32); np.add.at(low, (_ij[_ok, 0], _ij[_ok, 1]), 1)
tall = count(vert & (z > 2.3))
upper = count(vert & (z > 2.2) & (z < 3.2))

# inside-building mask resampled to 5 cm
gx = ((np.arange(shape[0]) + .5) * R + lo[0] - lo10[0]) / 0.1
gy = ((np.arange(shape[1]) + .5) * R + lo[1] - lo10[1]) / 0.1
GX, GY = np.meshgrid(np.clip(gx.astype(int), 0, M10.shape[0] - 1), np.clip(gy.astype(int), 0, M10.shape[1] - 1), indexing='ij')
INSIDE = M10[GX, GY]

solid = ndimage.binary_closing(low >= 3, iterations=2) & INSIDE
solid = ndimage.binary_opening(solid, iterations=1)
lab, n = ndimage.label(solid)
sz = ndimage.sum(solid, lab, np.arange(1, n + 1))
solid = np.isin(lab, np.where(sz * R * R >= 0.03)[0] + 1)
lab, n = ndimage.label(solid)

# a component is a WALL if a good part of it continues above 2.3 m on vertical surfaces, else an EXHIBIT/OBJECT
tall_frac = ndimage.sum(ndimage.binary_dilation(tall >= 2, iterations=1), lab, np.arange(1, n + 1)) / np.maximum(ndimage.sum(solid, lab, np.arange(1, n + 1)), 1)
is_wall_comp = np.concatenate([[False], tall_frac > 0.35])
WALL = is_wall_comp[lab] & solid
OBJ = solid & ~WALL

# heights: walls per 1 m tile (98th pct of vertical-surface z), objects per component (95th pct)
TILE = 1.0
pv = P[vert & (z > 0.15)]; iv = np.floor((pv[:, :2] - lo) / R).astype(int)
in_wall = WALL[iv[:, 0], iv[:, 1]]
tile_of = lambda xy: tuple(np.floor(xy / TILE).astype(int))
wall_h = {}
tk = np.floor(pv[in_wall, :2] / TILE).astype(int)
order = np.lexsort((tk[:, 1], tk[:, 0])); tk = tk[order]; zz = pv[in_wall, 2][order]
keys, starts = np.unique(tk, axis=0, return_index=True)
for k, s, e in zip(keys, starts, list(starts[1:]) + [len(tk)]):
    wall_h[tuple(k)] = float(np.clip(np.percentile(zz[s:e], 98), 2.4, 9.0))
# ceiling height per 1 m tile: median of downward-facing points above 2.2 m within 2.5 m, gaps filled by nearest
from scipy.spatial import cKDTree
cz = P[(N[:, 2] < -0.7) & (z > 2.2)]
ctree = cKDTree(cz[:, :2])
tiles = np.array(list({tuple(np.floor(p / TILE).astype(int)) for p in pv[in_wall, :2][::5]} | set(wall_h.keys())))
cen = (tiles + 0.5) * TILE
ceil_h = {}
for t, c, nb in zip(map(tuple, tiles), cen, ctree.query_ball_point(cen, r=2.5)):
    if len(nb) >= 30:
        ceil_h[t] = float(np.clip(np.median(cz[nb, 2]), 2.6, 6.0))
known = np.array(list(ceil_h.keys())); kv = np.array(list(ceil_h.values()))
_, nn = cKDTree(known).query(tiles)
for t, j in zip(map(tuple, tiles), nn):
    ceil_h.setdefault(t, float(kv[j]))
wall_h = ceil_h          # walls run floor-to-ceiling
obj_h = {}
in_obj = OBJ[iv[:, 0], iv[:, 1]]
olab, on = ndimage.label(OBJ)
ol = olab[iv[in_obj, 0], iv[in_obj, 1]]
for c in range(1, on + 1):
    q = pv[in_obj, 2][ol == c]
    obj_h[c] = float(np.clip(np.percentile(q, 95), 0.3, 2.3)) if len(q) > 10 else 1.0

# lintels: wall-like surfaces between 2.2 and 3.2 m over openings, next to walls
near_wall = ndimage.binary_dilation(WALL, iterations=8)
LINT = ndimage.binary_closing(upper >= 3, iterations=2) & near_wall & ~solid & INSIDE
LINT = ndimage.binary_opening(LINT, iterations=1)
llab, ln = ndimage.label(LINT)
lint_bot = {}
lsel = (z > 1.8) & (z < 3.2)
li = llab[ij[lsel, 0], ij[lsel, 1]]
for c in range(1, ln + 1):
    q = z[lsel][li == c]
    lint_bot[c] = float(np.clip(np.percentile(q, 3), 1.9, 3.0)) if len(q) > 20 else None


def polys(mask):
    m = np.ascontiguousarray(mask.T.astype(np.uint8))   # rows = y, cols = x
    cs, hier = cv2.findContours(m, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    if hier is None:
        return out
    hier = hier[0]
    for i, c in enumerate(cs):
        if hier[i][3] != -1 or len(c) < 3:
            continue
        to = lambda cc: [(lo[0] + (p[0][0] + .5) * R, lo[1] + (p[0][1] + .5) * R) for p in cc]
        holes = [to(cs[j]) for j in range(len(cs)) if hier[j][3] == i and len(cs[j]) >= 3]
        pg = Polygon(to(c), holes).buffer(R / 2, join_style=2).simplify(0.06).buffer(0)
        if pg.area > 0.01:
            out.append(pg)
    return out


def extrude(pg, z0, z1):
    out = []
    for g in getattr(pg, 'geoms', [pg]):
        if g.area < 0.004:
            continue
        m = trimesh.creation.extrude_polygon(g, z1 - z0)
        m.apply_translation([0, 0, z0]); out.append(m)
    return out


walls = []
for pg in polys(WALL):
    x0, y0, x1, y1 = pg.bounds
    for tx in range(int(np.floor(x0 / TILE)), int(np.floor(x1 / TILE)) + 1):
        for ty in range(int(np.floor(y0 / TILE)), int(np.floor(y1 / TILE)) + 1):
            piece = pg.intersection(sbox(tx * TILE, ty * TILE, (tx + 1) * TILE, (ty + 1) * TILE))
            if piece.is_empty:
                continue
            h = wall_h.get((tx, ty)) or np.median(list(wall_h.values()))
            walls += extrude(piece, 0.0, h)
objs = []
for c in range(1, on + 1):
    for pg in polys(olab == c):
        objs += extrude(pg, 0.0, obj_h[c])
lints = []
for c in range(1, ln + 1):
    if lint_bot[c] is None:
        continue
    for pg in polys(llab == c):
        x, y = pg.centroid.x, pg.centroid.y
        top = wall_h.get(tile_of(np.array([x, y]))) or 3.0
        if top > lint_bot[c] + 0.1:
            lints += extrude(pg, lint_bot[c], top)

# the measured hall <-> room04 doorway: guarantee a lintel over it
D = json.load(open('/home/claude/work/door.json'))
top = wall_h.get(tile_of(np.array([0.9, -0.2]))) or 3.0
lints += extrude(sbox(D['x_room'], D['y0'], D['x_hall'], D['y1']), D['height'], max(top, D['height'] + 0.3))

FM = np.load('/home/claude/work/floor_mask.npy'); FM_lo = mk['lo']
_fx = ((np.arange(shape[0]) + .5) * R + lo[0] - FM_lo[0]) / 0.1; _fy = ((np.arange(shape[1]) + .5) * R + lo[1] - FM_lo[1]) / 0.1
_FX, _FY = np.meshgrid(np.clip(_fx.astype(int), 0, FM.shape[0] - 1), np.clip(_fy.astype(int), 0, FM.shape[1] - 1), indexing='ij')
FLOOR5 = FM[_FX, _FY] | WALL | OBJ
floor_parts = []
for pg in polys(ndimage.binary_closing(FLOOR5, iterations=2)):
    floor_parts += extrude(pg, -0.03, 0.0)
FL = trimesh.util.concatenate(floor_parts)

mat = lambda name, rgb, rough=0.85: PBRMaterial(name=name, baseColorFactor=list(rgb) + [1.0], metallicFactor=0.0, roughnessFactor=rough, doubleSided=True)
W = trimesh.util.concatenate(walls); W.visual = trimesh.visual.TextureVisuals(material=mat('wall_plaster', [0.91, 0.89, 0.85]))
FL.visual = trimesh.visual.TextureVisuals(material=mat('floor_stone', [0.70, 0.63, 0.54], 0.8))
O = trimesh.util.concatenate(objs); O.visual = trimesh.visual.TextureVisuals(material=mat('exhibits_and_furniture', [0.62, 0.66, 0.70]))
L = trimesh.util.concatenate(lints); L.visual = trimesh.visual.TextureVisuals(material=mat('wall_plaster', [0.91, 0.89, 0.85]))
for m in (W, O, L, FL):
    m.merge_vertices(); m.unmerge_vertices()      # flat shading: sharp wall corners
trimesh.Scene({'walls': W, 'exhibits': O, 'lintels': L}).export('/home/claude/work/arch_parts.glb')
print('walls', len(walls), 'pieces', len(W.faces), 'tris | objects', on, len(O.faces), 'tris | lintels', len(lints), len(L.faces), 'tris')
print('wall height range', round(min(wall_h.values()), 2), round(np.median(list(wall_h.values())), 2), round(max(wall_h.values()), 2))
import pickle; pickle.dump({'W': W, 'O': O, 'L': L, 'FL': FL}, open('/home/claude/work/arch_parts.pkl', 'wb'))
