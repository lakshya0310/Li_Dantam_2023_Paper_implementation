import os
import numpy as np
import pybullet as p
import pybullet_data
import json

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

        # ---- Load your robot URDF from simulations folder ----
        urdf_path = os.path.join(self.THIS_DIR, "SCARA_like_1.urdf")
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"Robot URDF not found at: {urdf_path}")

        self.robotId = p.loadURDF(
            urdf_path,
            # [-0.4, 0, self.table_height - 0.05],
            [0, 0, self.table_height - 0.1],
            useFixedBase=True
        )

        # ---- Obstacles ----
        color = [0.824, 0.824, 0.824, 1.0]

        collision_shapes = []
        visual_shapes = []
        shape_positions = []

        def add_box(extent, pos, rgba):
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=extent)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=extent, rgbaColor=rgba)
            collision_shapes.append(col)
            visual_shapes.append(vis)
            shape_positions.append(pos)

        box_left_side_extent = np.array([0.05, 0.4, 0.4])
        box_left_side_pos = [-0.5, -0.3, 0.7]
        add_box(box_left_side_extent / 2, box_left_side_pos, color)

        box_right_side_extent = np.array([0.05, 0.4, 0.4])
        box_right_side_pos = [-0.9, -0.3, 0.7]
        add_box(box_right_side_extent / 2, box_right_side_pos, color)

        box_upper_left_side_extent = np.array([0.125, 0.4, 0.05])
        box_upper_left_side_pos = [-0.85, -0.3, 0.9]
        add_box(box_upper_left_side_extent / 2, box_upper_left_side_pos, color)

        box_upper_right_side_extent = np.array([0.125, 0.4, 0.05])
        box_upper_right_side_pos = [-0.55, -0.3, 0.9]
        add_box(box_upper_right_side_extent / 2, box_upper_right_side_pos, color)

        box_back_side_extent = np.array([0.4, 0.05, 0.4])
        box_back_side_pos = [-0.7, -0.5, 0.7]
        add_box(box_back_side_extent / 2, box_back_side_pos, color)

        self.box_id = p.createMultiBody(
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

        collision_shapes = []
        visual_shapes = []
        shape_positions = []

        table_leg_1_extent = np.array([0.1, 0.1, 0.5])
        table_leg_1_pos = [1.2, 0.65, 0.25]
        add_box(table_leg_1_extent / 2, table_leg_1_pos, color)

        table_leg_2_extent = np.array([0.1, 0.1, 0.5])
        table_leg_2_pos = [-1.2, 0.65, 0.25]
        add_box(table_leg_2_extent / 2, table_leg_2_pos, color)

        table_leg_3_extent = np.array([0.1, 0.1, 0.5])
        table_leg_3_pos = [1.2, -0.65, 0.25]
        add_box(table_leg_3_extent / 2, table_leg_3_pos, color)

        table_leg_4_extent = np.array([0.1, 0.1, 0.5])
        table_leg_4_pos = [-1.2, -0.65, 0.25]
        add_box(table_leg_4_extent / 2, table_leg_4_pos, color)

        table_top_extent = np.array([2.5, 1.4, 0.1])
        table_top_pos = [0.0, 0.0, 0.5]
        add_box(table_top_extent / 2, table_top_pos, color)

        self.table_id = p.createMultiBody(
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

        ball_col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.2)
        ball_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.2, rgbaColor=[1,0,0,1])
        self.ball_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=ball_col,
            baseVisualShapeIndex=ball_vis,
            basePosition=[-0.7, 0.1, 0.75]
        )

        # DOF
        self.N = p.getNumJoints(self.robotId)

        self.all_obstacles = [self.box_id, self.ball_id]
        # self.all_obstacles = [self.box_id, self.table_id, self.ball_id]
        # Optional: step a few frames to settle
        for _ in range(10):
            p.stepSimulation()
    
    def get_start_config(self):
        # A configuration in the bottom-left free region
        return [0.0, 0.0 , 0.0 , 0.0]
        # return [0.0, 0.0, 0.0]

    def get_goal_config(self):
        # A configuration in the top-right free region
        return [1.8, -0.95, 0, -0.05]
        # return [1.8, -0.95, 0]

    # degrees of freedom
    def dof(self) -> int:
        return int(self.N) - 1

    def inCollision(self, q) -> bool:
        return self.sdf(q) <= 0.0

    def sdf(self, q) -> float:
        """
        Signed distance field approximation:
        returns minimum distance between robot and obstacles.
        """
        # ensure q has correct length
        if len(q) < self.dof():
            raise ValueError(f"Expected q of length >= {self.dof()}, got {len(q)}")

        for i in range(self.dof()):
            p.resetJointState(self.robotId, i, float(q[i]))

        p.stepSimulation()

        dist = get_min_dist(self.robotId, self.all_obstacles)
        return float(dist)
    
    def run(self):
        if self.use_gui:

            config = self.get_goal_config()
            for i in range(self.N-1):
                p.resetJointState(self.robotId, i, float(config[i]))
            print("Running simulation with GUI. Close the window to exit.")
            while p.isConnected():
                states = p.getJointStates(self.robotId, [0,1,2,3,4])
                config = []
                for state in states:
                    config.append(state[0])
                
                print(self.sdf(config))
                p.stepSimulation()
        else:
            print("Running headless simulation. Press Ctrl+C to exit.")
            try:
                while True:
                    config = p.getJointStates([0,1,2,3,4])
                    print(self.sdf(config))
                    p.stepSimulation()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    sim = Simulation1(use_gui=True)
    sim.run()