#!/usr/bin/env python3
"""Copy the exported STL meshes (visual, unchanged) and build simplified collision meshes.

The exported meshes are assemblies of many disconnected parts, so quadric decimation of the
raw mesh breaks them.  The collision mesh is therefore the convex hull of the link, simplified
to about <target_faces> faces and re-hulled (always closed, convex, conservative).

usage: decimate_meshes.py <export_meshes_dir> <package_meshes_dir> [target_faces]
The J4/J5 link names are swapped in the export, so the two wrist files are renamed.
"""
import shutil, sys, os
import numpy as np, trimesh, fast_simplification

RENAME = {"wrist_roll_link": "wrist_pitch_link", "wrist_pitch_link": "wrist_roll_link"}
src, dst = sys.argv[1], sys.argv[2]
target = int(sys.argv[3]) if len(sys.argv) > 3 else 200
for f in sorted(os.listdir(src)):
    name = f[:-4]
    new = RENAME.get(name, name)
    shutil.copyfile(os.path.join(src, f), os.path.join(dst, "visual", new + ".STL"))
    m = trimesh.load(os.path.join(src, f))
    h = m.convex_hull
    if len(h.faces) > target:
        v, fc = fast_simplification.simplify(np.asarray(h.vertices, dtype=np.float32), np.asarray(h.faces),
                                             target_reduction=1.0 - target / len(h.faces))
        h = trimesh.Trimesh(v, fc, process=True).convex_hull
    c = h
    c.export(os.path.join(dst, "collision", new + ".stl"))
    print(f"{name:18s} -> {new:18s} {len(m.faces):7d} -> {len(c.faces):5d} faces, bbox change {np.abs(c.bounds - m.bounds).max()*1000:.2f} mm")
