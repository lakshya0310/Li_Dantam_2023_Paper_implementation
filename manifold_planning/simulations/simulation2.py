import pybullet as p
import pybullet_data as pd
import numpy as np
import time
import csv
import pandas as pan
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'c_space'))
import c_space as csg

def writesdf(filename, theta1s, theta2s, sdfs):
    with open(filename, 'w', newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["theta1", "theta2", "value"])

        for t1, t2, val in zip(theta1s, theta2s, sdfs):
            writer.writerow([t1, t2, val])


def saveMesh(mesh, filename):
    """
    Save mesh vertices and faces to a CSV-like file:
      id,x,y
      0, ...
      1, ...
      ...
      FACES
      0,1
      1,2
      ...
    """
    with open(filename, "w") as f:
        # Write vertices header
        f.write("id,x,y\n")

        # Write vertices
        for i, v in enumerate(mesh.vertices):
            f.write(f"{i},{v[0]},{v[1]}\n")

        # Separator so we can detect faces section in Python
        f.write("FACES\n")

        # Write faces
        for face in mesh.faces:
            line = ",".join(str(idx) for idx in face)
            f.write(line + "\n")

    print(f"[OK] Mesh saved including faces to {filename}")


table_height = 0.65

physicsClientId = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pd.getDataPath())
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("table/table.urdf")
p.setAdditionalSearchPath("/home/aayush/Research/Redemption/robots")
frankaId = p.loadURDF("2DOFRoboticArm.urdf", [-0.4, 0, table_height-0.05], useFixedBase=True)

color = [0.824, 0.824, 0.824, 0.5]

shelf_front_extent = [0.025, 0.4, 1.0]
shelf_front_pos = [-0.6, 0, 1.55]
shelf_front_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=shelf_front_extent)
shelf_front_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=shelf_front_extent, rgbaColor = color)

shelf_left_extent = [0.4, 0.025, 1.0]
shelf_left_pos = [-0.3, 0.4, 1.55]
shelf_left_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=shelf_left_extent)
shelf_left_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=shelf_left_extent, rgbaColor = color)

shelf_right_extent = [0.4, 0.025, 1.0]
shelf_right_pos = [-0.3, -0.4, 1.55]
shelf_right_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=shelf_right_extent)
shelf_right_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=shelf_right_extent, rgbaColor = color)


shelf_back_extent = [0.025, 0.4, 1.0]
shelf_back_pos = [0.0, 0, 1.55]
shelf_back_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=shelf_back_extent)
shelf_back_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=shelf_back_extent, rgbaColor = color)

collision_shapes = [shelf_front_col, shelf_left_col, shelf_right_col, shelf_back_col]
visual_shapes = [shelf_front_vis, shelf_left_vis, shelf_right_vis, shelf_back_vis]
shape_positions = [shelf_front_pos, shelf_left_pos, shelf_right_pos, shelf_back_pos]


shelf_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,
    baseVisualShapeIndex=-1,
    linkMasses=[0] * len(collision_shapes),
    linkCollisionShapeIndices=collision_shapes,
    linkVisualShapeIndices=visual_shapes,
    linkPositions=shape_positions,
    linkOrientations=[[0, 0, 0, 1]] * len(collision_shapes),
    linkInertialFramePositions=[[0, 0, 0]] * len(collision_shapes),
    linkInertialFrameOrientations=[[0, 0, 0, 1]] * len(collision_shapes),
    linkParentIndices=[0] * len(collision_shapes),
    linkJointTypes=[p.JOINT_FIXED] * len(collision_shapes),
    linkJointAxis=[[0, 0, 0]] * len(collision_shapes)
)

print("Created composite Shelf with ID: ", shelf_id)

joint_limits = [
    (-3.14, 3.14),
    (-3.14, 3.14),
    (-3.14, 3.14)
]

center = [0.0, 0.0, 0.0, 0.0]

num_samples = 1000
N = 2
all_obstacles = [shelf_id]

def get_min_dist(robot_id, obstacles_ids, max_search_dist = 0.5):
    min_dist = float('inf')

    robot_aabb_min, robot_aabb_max = p.getAABB(robot_id)
    expanded_min = [robot_aabb_min[i] - max_search_dist for i in range(3)]
    expanded_max = [robot_aabb_max[i] + max_search_dist for i in range(3)]

    nearby = p.getOverlappingObjects(expanded_min, expanded_max)
    nearby_ids = set([obj[0] for obj in nearby]) if nearby else set()

    for obs_id in obstacles_ids:
        if obs_id not in nearby_ids:
            continue

        pts = p.getClosestPoints(robot_id, obs_id, max_search_dist)
        if not pts:
            continue

        for pt in pts:
            dist = pt[8]
            if dist < min_dist:
                min_dist = dist

    return min_dist

all_obstacles = [shelf_id]
N = p.getNumJoints(frankaId)

def sdf(v):
    for i in range(N):  
        p.resetJointState(frankaId, i, v[i])
    p.stepSimulation()
    dist = get_min_dist(frankaId, all_obstacles)
    return dist


theta1_range = np.linspace(-np.pi, np.pi, 240)
theta2_range = np.linspace(-np.pi/2, np.pi/2, 240)

theta1_list = []
theta2_list = []
sdf_list = []

# for t1 in theta1_range:
#     for t2 in theta2_range:
#         v = [t1, t2] + [0.0]*(N-2)
#         dist = sdf(v)
#         theta1_list.append(t1)
#         theta2_list.append(t2)
#         sdf_list.append(dist)

# writesdf("data/sdf_field2.csv", theta1_list, theta2_list, sdf_list)

# print("SDF field written to sdf_field.csv")

boundary = csg.Manifold(2)
start = time.time()
csg.initManifold(boundary, [0.0, 0.0], sdf, 500)
csg.refineManifold(boundary, sdf, 0.05)
end = time.time()
dur = end - start
print(f"Time required for sphere tracing {dur}")



saveMesh(boundary, "data/mesh2.csv")

while True:
    p.stepSimulation()
    time.sleep(1./240.)