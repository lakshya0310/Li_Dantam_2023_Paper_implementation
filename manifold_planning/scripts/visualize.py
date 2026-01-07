import trimesh
import sys
import os

DATA_DIR = "../data"

def show_obj(filename):
    path = os.path.join(DATA_DIR, filename)
    mesh = trimesh.load(path)
    mesh.show()

def show_pointcloud(filename):
    path = os.path.join(DATA_DIR, filename)
    pts = []
    with open(path, "r") as f:
        for line in f:
            pts.append([float(x) for x in line.split()])
    pc = trimesh.points.PointCloud(pts)
    pc.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python visualize.py mesh proof_mesh.obj")
        print("  python visualize.py points manifold.xyz")
        sys.exit(0)

    mode = sys.argv[1]
    file = sys.argv[2]

    if mode == "mesh":
        show_obj(file)
    elif mode == "points":
        show_pointcloud(file)
