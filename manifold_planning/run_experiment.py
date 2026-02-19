import sys
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from scipy.spatial import cKDTree
import time
import collections

# CHANGE THIS LINE to test different environments
# e.g., simulation3, simulation4, simulation5
import simulation1 as sim_env 

try:
    import infeasibility_proof
except ImportError:
    print("Error: Could not import 'infeasibility_proof'.")
    sys.exit(1)

def build_prm(sim, num_samples, start, goal):
    dof = sim.dof()
    bounds = [(-3.14, 3.14)] * dof
    
    valid_samples = [start, goal]
    
    # 1. Sample Free Space
    for _ in range(num_samples):
        q = np.random.uniform([b[0] for b in bounds], [b[1] for b in bounds])
        if not sim.inCollision(q):
            valid_samples.append(q)
            
    valid_samples = np.array(valid_samples)
    
    # 2. Build Graph
    G = nx.Graph()
    G.add_nodes_from(range(len(valid_samples)))
    
    tree = cKDTree(valid_samples)
    k = 10 + (dof * 2) # Increase k-nearest neighbors for higher dimensions
    dists, indices = tree.query(valid_samples, k=k+1)
    
    for i, neighbors in enumerate(indices):
        for j in neighbors:
            if i == j: continue
            midpoint = (valid_samples[i] + valid_samples[j]) / 2.0
            if not sim.inCollision(midpoint):
                G.add_edge(i, j)
                
    return valid_samples, G, 0, 1

def check_proof_in_python(facets, sim, epsilon=0.1):
    """
    Dimension-agnostic bisection check with Memoization (Caching).
    This is CRITICAL for 3, 4, and 5 DOF to prevent extreme slowdowns.
    """
    queue = collections.deque(facets)
    collision_cache = {}
    
    def is_collision(pt):
        # Round to 4 decimal places to create a stable hash key for identical spatial points
        key = tuple(np.round(pt, 4))
        if key not in collision_cache:
            collision_cache[key] = sim.inCollision(pt)
        return collision_cache[key]

    # Pre-check all original vertices to fail fast
    for facet in facets:
        for pt in facet:
            if not is_collision(pt):
                return False

    bisections = 0
    while queue:
        facet = queue.popleft()
        pts = np.array(facet)
        
        # 1. Find the longest edge
        max_edge = 0
        u, v = 0, 0
        n_pts = len(pts)
        for i in range(n_pts):
            for j in range(i+1, n_pts):
                d = np.linalg.norm(pts[i] - pts[j])
                if d > max_edge:
                    max_edge = d
                    u, v = i, j
                    
        # 2. Base Case: Max edge is smaller than epsilon
        if max_edge < epsilon:
            continue 
            
        # 3. Bisect the longest edge
        mid = (pts[u] + pts[v]) / 2.0
        
        # 4. Check the new midpoint (if it leaks into Free Space, proof is invalid)
        if not is_collision(mid):
            return False
            
        # 5. Create two new simplices spanning the split volume
        f1 = list(facet); f1[u] = mid
        f2 = list(facet); f2[v] = mid
        
        queue.append(f1)
        queue.append(f2)
        bisections += 1
        
    print(f"   [Check] Performed {bisections} bisections. Unique points evaluated: {len(collision_cache)}")
    return True

def run():
    print("------------------------------------------------")
    print("       N-DOF Infeasibility Proof Experiment")
    print("------------------------------------------------")
    
    sim = sim_env.Simulation1(use_gui=False)
    dof = sim.dof()
    print(f"Detected Robot DOF: {dof}")
    
    planner = infeasibility_proof.InfeasibilityPlanner(dof)
    
    start_q = np.array(sim.get_start_config())
    goal_q = np.array(sim.get_goal_config())
    
    # Scale samples linearly with DOF to combat curse of dimensionality
    num_samples = 500 * dof 
    
    print(f"\n[Step 1] Building PRM with {num_samples} samples...")
    t0 = time.time()
    samples, G, s_idx, g_idx = build_prm(sim, num_samples, start_q, goal_q)
    print(f"   [Timer] PRM Build: {time.time()-t0:.4f} s")
    
    if nx.has_path(G, s_idx, g_idx):
        print("Path FOUND! Problem is feasible. No proof needed.")
        return

    print("\n[Step 2] Analyzing Connected Components...")
    t0 = time.time()
    comp_start = nx.node_connected_component(G, s_idx)
    comp_goal = nx.node_connected_component(G, g_idx)
    
    for idx in comp_start:
        planner.add_sample(samples[idx], -1.0)
    for idx in comp_goal:
        planner.add_sample(samples[idx], 1.0)
    print(f"   [Timer] Component Analysis: {time.time()-t0:.4f} s (Samples: {len(comp_start)} Start vs {len(comp_goal)} Goal)")
    
    print("\n[Step 3] Training SVM...")
    planner.train_manifold()
    
    print("\n[Step 4] Finding Manifold Seeds...")
    t0 = time.time()
    seeds = []
    min_dist = float('inf')
    best_seed = None
    
    s_indices = list(comp_start)
    g_indices = list(comp_goal)
    
    for _ in range(1000): # Increased search iterations
        i = np.random.choice(s_indices)
        j = np.random.choice(g_indices)
        dist = np.linalg.norm(samples[i] - samples[j])
        if dist < min_dist:
            min_dist = dist
            best_seed = (samples[i] + samples[j]) / 2.0
            
    if best_seed is not None:
        seeds.append(best_seed)
    print(f"   [Timer] Seed Search: {time.time()-t0:.4f} s (Gap: {min_dist:.3f})")
    
    print("\n[Step 5] Adaptive Proof Construction...")
    # Initial scale parameters (can be adjusted for 4D/5D if memory runs out)
    scale = 0.5 
    valid_proof = False
    final_facets = []
    
    while scale > 0.05:
        facets = planner.construct_proof(seeds, scale)
        print(f"   [Info] Generated {len(facets)} facets at scale {scale:.3f}.")
        
        if len(facets) == 0:
            scale *= 0.8
            continue
            
        t_check = time.time()
        
        # Use epsilon=0.2 for 4D/5D to prevent endless recursion
        epsilon = 0.1 if dof <= 3 else 0.2 
        
        if check_proof_in_python(facets, sim, epsilon=epsilon):
            print(f"   [Timer] Verification: {time.time()-t_check:.4f} s")
            print(">>> PROOF VALID! Triangulation tightly separates Start and Goal.")
            valid_proof = True
            final_facets = facets
            break
        else:
            print(f"   [Timer] Verification: {time.time()-t_check:.4f} s (Failed)")
            print(f"   Proof Invalid at scale {scale:.3f} (Intersects Free Space). Reducing...")
            scale *= 0.8

    # 6. Visualization Router
    print("\n[Step 6] Visualizing Results...")
    if dof == 2:
        visualize_2d(samples, comp_start, comp_goal, final_facets, valid_proof)
    elif dof == 3:
        visualize_3d(samples, comp_start, comp_goal, final_facets, valid_proof)
    else:
        print(f"Visualization skipped. {dof}-DOF cannot be cleanly projected into a static plot.")

def visualize_2d(samples, comp_start, comp_goal, facets, is_valid):
    fig, ax = plt.subplots(figsize=(10, 8))
    s_pts = samples[list(comp_start)]
    ax.scatter(s_pts[:,0], s_pts[:,1], c='blue', alpha=0.3, label='Start Comp')
    g_pts = samples[list(comp_goal)]
    ax.scatter(g_pts[:,0], g_pts[:,1], c='green', alpha=0.3, label='Goal Comp')
    
    color = 'black' if is_valid else 'red'
    for facet in facets:
        f = np.array(facet)
        if len(f) == 2:
            ax.plot(f[:,0], f[:,1], c=color, linewidth=2)
            
    ax.set_title(f"2D Infeasibility Proof (Valid: {is_valid})")
    ax.legend()
    plt.show()

def visualize_3d(samples, comp_start, comp_goal, facets, is_valid):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    s_pts = samples[list(comp_start)]
    ax.scatter(s_pts[:,0], s_pts[:,1], s_pts[:,2], c='blue', alpha=0.3, label='Start Comp', s=10)
    
    g_pts = samples[list(comp_goal)]
    ax.scatter(g_pts[:,0], g_pts[:,1], g_pts[:,2], c='green', alpha=0.3, label='Goal Comp', s=10)
    
    color = 'black' if is_valid else 'red'
    for facet in facets:
        f = np.array(facet)
        # In 3D C-Space, the manifold boundary is a 2D surface made of Triangles (3 vertices)
        if len(f) == 3:
            # Close the triangle loop for line plotting
            f_loop = np.vstack([f, f[0]]) 
            ax.plot(f_loop[:,0], f_loop[:,1], f_loop[:,2], c=color, linewidth=1)
            
    ax.set_title(f"3D Infeasibility Proof (Valid: {is_valid})")
    ax.legend()
    plt.show()

if __name__ == "__main__":
    run()