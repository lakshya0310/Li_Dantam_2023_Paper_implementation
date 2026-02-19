import os
import numpy as np
import pybullet as p
import pybullet_data


# --------------------------
# Helper: compute min distance
# --------------------------
def get_min_dist(robot_id, obstacles_ids, max_search_dist=0.5):
    """
    Returns minimum signed distance (actually closest distance) between robot and obstacles.
    Uses AABB broadphase + getClosestPoints narrowphase.
    """
    min_dist = float("inf")

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
            dist = pt[8]  # distance
            if dist < min_dist:
                min_dist = dist

    return min_dist


# --------------------------
# Simulation wrapper class
# --------------------------
class Simulation1:
    """
    Embedded-safe PyBullet simulation used by C++.
    Nothing runs at import-time except class definition.
    """

    def __init__(self, use_gui: bool = False):
        self.use_gui = use_gui

        # Connect exactly once
        self.cid = p.connect(p.GUI if use_gui else p.DIRECT)
        if self.cid < 0:
            raise RuntimeError("PyBullet connection failed")

        # Absolute directory of this file
        self.THIS_DIR = os.path.dirname(os.path.abspath(__file__))

        # Reset scene
        p.resetSimulation()

        # Always add search paths AFTER resetSimulation()
        self.data_path = pybullet_data.getDataPath()
        p.setAdditionalSearchPath(self.data_path)
        p.setAdditionalSearchPath(self.THIS_DIR)

        # Gravity
        p.setGravity(0, 0, -9.81)

        # Scene constants
        self.table_height = 0.65

        # ---- Load plane and table using absolute paths ----
        plane_path = os.path.join(self.data_path, "plane.urdf")
        table_path = os.path.join(self.data_path, "table", "table.urdf")

        if not os.path.exists(plane_path):
            raise FileNotFoundError(f"plane.urdf not found at: {plane_path}")
        if not os.path.exists(table_path):
            raise FileNotFoundError(f"table.urdf not found at: {table_path}")

        self.planeId = p.loadURDF(plane_path)
        self.tableId = p.loadURDF(table_path)

        # ---- Load your robot URDF from simulations folder ----
        urdf_path = os.path.join(self.THIS_DIR, "4DOFRoboticArm.urdf")
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"Robot URDF not found at: {urdf_path}")

        self.robotId = p.loadURDF(
            urdf_path,
            [-0.4, 0, self.table_height - 0.05],
            useFixedBase=True
        )

        # ---- Obstacles ----
        color = [0.824, 0.824, 0.824, 1.0]

        # Shelf parts
        shelf_base_extent = [0.3, 0.4, 0.025]
        shelf_base_pos = [0.3, 0, self.table_height]

        shelf_mid_extent = [0.3, 0.4, 0.025]
        shelf_mid_pos = [0.3, 0, 1.15]

        shelf_top_extent = [0.3, 0.4, 0.025]
        shelf_top_pos = [0.3, 0, 1.65]

        shelf_left_extent = [0.3, 0.025, 0.5]
        shelf_left_pos = [0.3, 0.4, 1.15]

        shelf_right_extent = [0.3, 0.025, 0.5]
        shelf_right_pos = [0.3, -0.4, 1.15]

        shelf_back_extent = [0.025, 0.4, 0.5]
        shelf_back_pos = [0.6, 0, 1.15]

        collision_shapes = []
        visual_shapes = []
        shape_positions = []

        def add_box(extent, pos, rgba):
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=extent)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=extent, rgbaColor=rgba)
            collision_shapes.append(col)
            visual_shapes.append(vis)
            shape_positions.append(pos)

        add_box(shelf_base_extent, shelf_base_pos, color)
        add_box(shelf_mid_extent, shelf_mid_pos, color)
        add_box(shelf_top_extent, shelf_top_pos, color)
        add_box(shelf_left_extent, shelf_left_pos, color)
        add_box(shelf_right_extent, shelf_right_pos, color)
        add_box(shelf_back_extent, shelf_back_pos, color)

        self.shelf_id = p.createMultiBody(
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

        # ---- Cubes ----
        cube_extent = [0.075, 0.075, 0.075]
        cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=cube_extent)
        cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=cube_extent, rgbaColor=[1, 0, 0, 1])

        cube_positions = [
            [0.10527608126362643, -0.1684955869015416,
             self.table_height + shelf_mid_pos[2] - shelf_base_pos[2] + shelf_mid_extent[2] + cube_extent[2]],
            [0.21496869176397293, 0.25240283875081576,
             self.table_height + shelf_mid_pos[2] - shelf_base_pos[2] + shelf_mid_extent[2] + cube_extent[2]],
        ]
        cube_yaws = [-1.5560652770151433, -0.34789205201574536]

        self.cube_ids = []
        for i in range(len(cube_positions)):
            quat = p.getQuaternionFromEuler([0, 0, cube_yaws[i]])
            cid = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=cube_col,
                baseVisualShapeIndex=cube_vis,
                basePosition=cube_positions[i],
                baseOrientation=quat
            )
            self.cube_ids.append(cid)

        # All obstacles
        self.all_obstacles = [self.shelf_id] + self.cube_ids

        # DOF
        self.N = p.getNumJoints(self.robotId)

        # Optional: step a few frames to settle
        for _ in range(10):
            p.stepSimulation()
    
    def get_start_config(self):
        # A configuration in the bottom-left free region
        return [-2.5, -2.5 , 0.0 , 0.0]

    def get_goal_config(self):
        # A configuration in the top-right free region
        return [0, 1.5 , 0.0 , 0.0]

    # degrees of freedom
    def dof(self) -> int:
        return int(self.N)

    def inCollision(self, q) -> bool:
        return self.sdf(q) <= 0.0

    def sdf(self, q) -> float:
        """
        Signed distance field approximation:
        returns minimum distance between robot and obstacles.
        """
        # ensure q has correct length
        if len(q) < self.N:
            raise ValueError(f"Expected q of length >= {self.N}, got {len(q)}")

        for i in range(self.N):
            p.resetJointState(self.robotId, i, float(q[i]))

        p.stepSimulation()

        dist = get_min_dist(self.robotId, self.all_obstacles)
        return float(dist)
