"""Screened Poisson reconstruction + trimming to the scanned surface."""
import sys, time, numpy as np, open3d as o3d
from scipy.spatial import cKDTree
depth = int(sys.argv[1]) if len(sys.argv) > 1 else 10
pc = o3d.io.read_point_cloud('/home/claude/work/mesh_input.ply')
_P = np.asarray(pc.points); _N = np.asarray(pc.normals)
floor = (_P[:, 2] < 0.07) & (_N[:, 2] > 0.85)          # replaced by a flat floor slab
np.save('/home/claude/work/floor_pts.npy', _P[floor][:, :2])
pc = pc.select_by_index(np.where(~floor)[0])
t = time.time()
mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pc, depth=depth, scale=1.05, linear_fit=True, n_threads=2)
dens = np.asarray(dens)
print('poisson', depth, len(mesh.triangles), 'tris', round(time.time() - t), 's', flush=True)
# 1) drop low-support vertices
mesh.remove_vertices_by_mask(dens < np.quantile(dens, 0.04))
# 2) drop anything farther than 12 cm from a real point (Poisson bubbles, invented ceilings, glass)
P = np.asarray(pc.points); V = np.asarray(mesh.vertices)
d, j = cKDTree(P).query(V, k=1, distance_upper_bound=0.5)
mesh.remove_vertices_by_mask(d > 0.12)
# colour from nearest point intensity
V = np.asarray(mesh.vertices); _, j = cKDTree(P).query(V, k=1)
mesh.vertex_colors = o3d.utility.Vector3dVector(np.asarray(pc.colors)[j])
mesh.remove_degenerate_triangles(); mesh.remove_unreferenced_vertices()
# 3) remove small floating pieces
cl, n, _ = mesh.cluster_connected_triangles(); cl = np.asarray(cl); n = np.asarray(n)
mesh.remove_triangles_by_mask(n[cl] < 300); mesh.remove_unreferenced_vertices()
mesh.compute_vertex_normals()
print('trimmed', len(mesh.triangles), 'tris', len(mesh.vertices), 'verts', flush=True)
o3d.io.write_triangle_mesh(f'/home/claude/work/scan_mesh_d{depth}.ply', mesh)
