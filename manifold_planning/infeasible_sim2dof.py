import os
import numpy as np
import pybullet as p
import pybullet_data

def get_min_dist(robot_id, obstacles_ids, max_search_dist=0.5):
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
            dist = pt[8] 
            if dist < min_dist:
                min_dist = dist
    return min_dist

class Simulation1:
    def __init__(self, use_gui=False):
        self.use_gui = use_gui
        self.cid = p.connect(p.GUI if use_gui else p.DIRECT)
        if self.cid < 0: raise RuntimeError("PyBullet connection failed")
        
        self.THIS_DIR = os.path.dirname(os.path.abspath(__file__))
        p.resetSimulation()
        
        self.data_path = pybullet_data.getDataPath()
        p.setAdditionalSearchPath(self.data_path)
        p.setAdditionalSearchPath(self.THIS_DIR)
        p.setGravity(0, 0, -9.81)

        self.table_height = 0.65
        self.planeId = p.loadURDF(os.path.join(self.data_path, "plane.urdf"))
        self.tableId = p.loadURDF(os.path.join(self.data_path, "table", "table.urdf"))

        # Load 2-DOF Robot
        urdf_path = os.path.join(self.THIS_DIR, "2DOFRoboticArm.urdf")
        self.robot_base_pos = [-0.4, 0, self.table_height - 0.05]
        self.robotId = p.loadURDF(urdf_path, self.robot_base_pos, useFixedBase=True)

        # ---- THE IRON CURTAIN (Infeasible Obstacles) ----
        # Two giant walls splitting the workspace, with a 16cm gap for the base
        color = [0.8, 0.2, 0.2, 1.0] # Red walls
        gap_half_width = 0.08
        wall_len = 1.5
        wall_thickness = 0.05
        wall_height = 0.8
        
        # Right Wall (+x)
        w1_ext = [wall_len, wall_thickness, wall_height]
        w1_pos = [self.robot_base_pos[0] + gap_half_width + wall_len, 0, self.table_height + wall_height]
        
        # Left Wall (-x)
        w2_ext = [wall_len, wall_thickness, wall_height]
        w2_pos = [self.robot_base_pos[0] - gap_half_width - wall_len, 0, self.table_height + wall_height]

        col1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=w1_ext)
        vis1 = p.createVisualShape(p.GEOM_BOX, halfExtents=w1_ext, rgbaColor=color)
        self.wall1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col1, baseVisualShapeIndex=vis1, basePosition=w1_pos)

        col2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=w2_ext)
        vis2 = p.createVisualShape(p.GEOM_BOX, halfExtents=w2_ext, rgbaColor=color)
        self.wall2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col2, baseVisualShapeIndex=vis2, basePosition=w2_pos)

        self.all_obstacles = [self.wall1, self.wall2]
        self.N = p.getNumJoints(self.robotId)

        for _ in range(10): p.stepSimulation()
    
    def get_start_config(self):
        # 90 degrees (Pointing safely into the +Y space)
        return [1.57, 0.0]

    def get_goal_config(self):
        # -90 degrees (Pointing safely into the -Y space, impossible to reach)
        return [-1.57, 0.0]

    def dof(self) -> int: return int(self.N)
    def inCollision(self, q) -> bool: return self.sdf(q) <= 0.0

    def sdf(self, q) -> float:
        for i in range(self.N): p.resetJointState(self.robotId, i, float(q[i]))
        p.stepSimulation()
        return float(get_min_dist(self.robotId, self.all_obstacles))