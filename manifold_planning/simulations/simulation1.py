import pybullet as p
import pybullet_data as pd
import numpy as np
import time
import csv
import pandas as pan
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'bindings'))
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


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

table_height = 0.65

planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("table/table.urdf")

p.setAdditionalSearchPath("/home/aayush/Research/Redemption/robots")
frankaId = p.loadURDF("2DOFRoboticArm.urdf", [-0.4, 0, table_height-0.05], useFixedBase=True)


color = [0.824, 0.824, 0.824, 1]

shelf_base_extent = [0.3, 0.4, 0.025]
shelf_base_pos = [0.3, 0, table_height]
shelf_base_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=shelf_base_extent)
shelf_base_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=shelf_base_extent, rgbaColor = color)

shelf_mid_extent = [0.3, 0.4, 0.025]
shelf_mid_pos = [0.3, 0, 1.15]
shelf_mid_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=shelf_mid_extent)
shelf_mid_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=shelf_mid_extent, rgbaColor = color)

shelf_top_extent = [0.3, 0.4, 0.025]
shelf_top_pos = [0.3, 0, 1.65]
shelf_top_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=shelf_top_extent)
shelf_top_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=shelf_top_extent, rgbaColor = color)

shelf_left_extent = [0.3, 0.025, 0.5]
shelf_left_pos = [0.3, 0.4, 1.15]
shelf_left_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=shelf_left_extent)
shelf_left_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=shelf_left_extent, rgbaColor = color)

shelf_right_extent = [0.3, 0.025, 0.5]
shelf_right_pos = [0.3, -0.4, 1.15]
shelf_right_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=shelf_right_extent)
shelf_right_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=shelf_right_extent, rgbaColor = color)

shelf_back_extent = [0.025, 0.4, 0.5]
shelf_back_pos = [0.6, 0, 1.15]
shelf_back_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=shelf_back_extent)
shelf_back_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=shelf_back_extent, rgbaColor = color)

collision_shapes = [shelf_base_col, shelf_mid_col, shelf_top_col, shelf_left_col, shelf_right_col, shelf_back_col]
visual_shapes = [shelf_base_vis, shelf_mid_vis, shelf_top_vis, shelf_left_vis, shelf_right_vis, shelf_back_vis]
shape_positions = [shelf_base_pos, shelf_mid_pos, shelf_top_pos, shelf_left_pos, shelf_right_pos, shelf_back_pos]

cube_extent = [0.075, 0.075, 0.075]
cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents = cube_extent)
cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents = cube_extent, rgbaColor = [1, 0, 0, 1])

num_cubes = 12

cube_positions = [[0.10527608126362643, -0.1684955869015416,    table_height + shelf_mid_pos[2] - shelf_base_pos[2] + shelf_mid_extent[2] + cube_extent[2]],
                  [0.21496869176397293, 0.25240283875081576,    table_height + shelf_mid_pos[2] - shelf_base_pos[2] + shelf_mid_extent[2] + cube_extent[2]],
                  [0.4873996788325066,  0.1567560895221497,     table_height + shelf_mid_pos[2] - shelf_base_pos[2] + shelf_mid_extent[2] + cube_extent[2]],
                  [0.4410175472404896,  -0.05737857018511772,   table_height + shelf_mid_pos[2] - shelf_base_pos[2] + shelf_mid_extent[2] + cube_extent[2]],
                  [0.41001177805214567, -0.2544132587255353,    table_height + shelf_base_extent[2] + cube_extent[2]],
                  [0.3632328530089752,  0.11738222843414359,    table_height + shelf_base_extent[2] + cube_extent[2]],
                  [0.5027154766928059,  0.05701354830943928,    table_height + shelf_base_extent[2] + cube_extent[2]],
                  [0.18693899440298462, -0.08411841253554,      table_height + shelf_base_extent[2] + cube_extent[2]],
                  [0.10527608126362643, 0.1684955869015416, table_height + shelf_mid_pos[2] - shelf_base_pos[2] + shelf_mid_extent[2] + 3*cube_extent[2]],
                  [0.21496869176397293, -0.08240283875081576, table_height + shelf_mid_pos[2] - shelf_base_pos[2] + shelf_mid_extent[2] + 3*cube_extent[2]],
                  [0.3873996788325066, -0.0567560895221497, table_height + shelf_mid_pos[2] - shelf_base_pos[2] + shelf_mid_extent[2] + 3*cube_extent[2]],
                  [0.3410175472404896, 0.05737857018511772, table_height + shelf_mid_pos[2] - shelf_base_pos[2] + shelf_mid_extent[2] + 3*cube_extent[2]]]

cube_yaws = [-1.5560652770151433, -0.34789205201574536, 0.5328894979438021, 1.76025621596012,
             -1.6806747855705495, 0.04422255206971748, 0.9422162576100384, 0.8868436122603103,
             -1.5560652770151433, -0.34789205201574536, 0.5328894979438021, 1.76025621596012]

cube_ids = []

for i in range(num_cubes):
    quat = p.getQuaternionFromEuler([0, 0, cube_yaws[i]])
    id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=cube_col,
        baseVisualShapeIndex=cube_vis,
        basePosition=cube_positions[i],
        baseOrientation=quat
    )
    cube_ids.append(id)

target_cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents = cube_extent)
target_cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents = cube_extent, rgbaColor = [0,0,1,1])
target_cube_position = [0.31793110667851,    -0.2473036687273794,    1.15 + shelf_mid_extent[2] + cube_extent[2]]
target_cube_yaw = 2.038157909550379

target_cube_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=target_cube_col,
    baseVisualShapeIndex=target_cube_vis,
    basePosition=target_cube_position,
    baseOrientation=p.getQuaternionFromEuler([0, 0, target_cube_yaw])
)

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

all_obstacles = [shelf_id] + cube_ids
N = p.getNumJoints(frankaId)

def sdf(v):
    for i in range(N):  
        p.resetJointState(frankaId, i, v[i])
    p.stepSimulation()
    dist = get_min_dist(frankaId, all_obstacles)
    return dist


theta1_range = np.linspace(-np.pi, np.pi, 500)
theta2_range = np.linspace(-np.pi, np.pi, 500)

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

# writesdf("data/sdf_field1.csv", theta1_list, theta2_list, sdf_list)

# boundary = csg.Manifold(2)
# start = time.time()
# csg.initManifold(boundary, [0.0, 0.0], sdf, 500)
# csg.refineManifold(boundary, sdf, 0.05)
# end = time.time()
# dur = end - start
# print(f"Time required for sphere tracing {dur}")



# saveMesh(boundary, "data/mesh.csv")

p.resetJointState(frankaId, 0, 0.0)
p.resetJointState(frankaId, 1, 1.2)



while True:
    p.stepSimulation()
    time.sleep(1./240.)