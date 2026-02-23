import sys
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from scipy.spatial import cKDTree
from scipy.optimize import minimize
import miniball
import time
import collections

# CHANGE THIS to simulation3, simulation4, or simulation5
import simulation4 as sim_env 

try:
    import infeasibility_proof
except ImportError:
    print("Error: Could not import 'infeasibility_proof'.")
    sys.exit(1)

def build_prm(sim, num_samples, start, goal):
    dof = sim.dof()
    bounds = [(-3.14, 3.14)] * dof
    
    valid_samples = [start, goal]
    
    for _ in range(num_samples):
        q = np.random.uniform([b[0] for b in bounds], [b[1] for b in bounds])
        if not sim.inCollision(q):
            valid_samples.append(q)
            
    valid_samples = np.array(valid_samples)
    
    G = nx.Graph()
    G.add_nodes_from(range(len(valid_samples)))
    
    tree = cKDTree(valid_samples)
    k = 10 + (dof * 2) 
    dists, indices = tree.query(valid_samples, k=k+1)
    
    for i, neighbors in enumerate(indices):
        for j in neighbors:
            if i == j: continue
            midpoint = (valid_samples[i] + valid_samples[j]) / 2.0
            if not sim.inCollision(midpoint):
                G.add_edge(i, j)
                
    return valid_samples, G, 0, 1

def elastic_update(q, sim, planner, training_samples, training_labels, max_iter=3):
    """
    Algorithm 4 from paper: Locally fixes the triangulation by projecting 
    C_free points onto the manifold using non-linear optimization (SLSQP).
    """
    current_q = np.copy(q)
    
    for _ in range(max_iter):
        # 1. Project to the manifold (min |F(q)|)
        res = minimize(
            lambda x: abs(planner.manifold_function(x)), 
            x0=current_q, 
            method='SLSQP'
        )
        q_proj = res.x
        
        # 2. If the projection is in Obstacle space, the elastic update succeeded!
        if sim.inCollision(q_proj):
            return q_proj, True
            
        # 3. If still in Free space, add to samples, retrain, and loop
        # Find label of nearest neighbor to maintain classification bounds
        dists = np.linalg.norm(np.array(training_samples) - q_proj, axis=1)
        closest_idx = np.argmin(dists)
        label = training_labels[closest_idx]
        
        training_samples.append(q_proj)
        training_labels.append(label)
        planner.add_sample(q_proj, label)
        planner.train_manifold() # Retrain SVM
        
        current_q = q_proj
        
    return current_q, False

def check_proof_in_python(facets, sim, planner, training_samples, training_labels, epsilon=0.1):
    """
    Section V-C: Facet Bisection and Checking utilizing the Miniball algorithm
    and integrating the local Elastic Update.
    """
    queue = collections.deque(facets)
    collision_cache = {}
    
    def check_and_update(pt):
        # Round to create stable hash key for caching
        key = tuple(np.round(pt, 4))
        if key in collision_cache:
            return pt, collision_cache[key]
            
        if sim.inCollision(pt):
            collision_cache[key] = True
            return pt, True
        else:
            # ELASTIC UPDATE TRIGGERED
            new_pt, success = elastic_update(pt, sim, planner, training_samples, training_labels)
            if success:
                new_key = tuple(np.round(new_pt, 4))
                collision_cache[new_key] = True
                return new_pt, True
            else:
                collision_cache[key] = False
                return pt, False

    bisections = 0
    elastic_fixes = 0
    
    while queue:
        facet = queue.popleft()
        pts = np.array(facet)
        
        # --- MINIBALL ALGORITHM CHECK ---
        center, squared_radius = miniball.get_bounding_ball(pts)
        radius = np.sqrt(squared_radius)
        
        # Base Case: Enclosed in hyper-ball of radius < epsilon
        if radius < epsilon:
            valid_facet = True
            for i in range(len(pts)):
                updated_pt, in_obs = check_and_update(pts[i])
                if not in_obs:
                    return False # Elastic update failed, need smaller global scale
                if not np.array_equal(pts[i], updated_pt):
                    pts[i] = updated_pt
                    elastic_fixes += 1
            continue
            
        # Recursive Step: Bisect longest edge
        max_edge = 0
        u, v = 0, 0
        n_pts = len(pts)
        for i in range(n_pts):
            for j in range(i+1, n_pts):
                d = np.linalg.norm(pts[i] - pts[j])
                if d > max_edge:
                    max_edge = d
                    u, v = i, j
                    
        mid = (pts[u] + pts[v]) / 2.0
        
        # Check midpoint with Elastic Update logic
        updated_mid, in_obs = check_and_update(mid)
        if not in_obs:
            return False
        if not np.array_equal(mid, updated_mid):
            elastic_fixes += 1
            
        f1 = list(facet); f1[u] = updated_mid
        f2 = list(facet); f2[v] = updated_mid
        
        queue.append(f1)
        queue.append(f2)
        bisections += 1
        
    print(f"   [Check] Bisections: {bisections} | Elastic Fixes: {elastic_fixes}")
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
    
    # Store explicit python lists for Elastic Update local retraining
    training_samples = []
    training_labels = []
    
    for idx in comp_start:
        planner.add_sample(samples[idx], -1.0)
        training_samples.append(samples[idx])
        training_labels.append(-1.0)
        
    for idx in comp_goal:
        planner.add_sample(samples[idx], 1.0)
        training_samples.append(samples[idx])
        training_labels.append(1.0)
        
    print(f"   [Timer] Component Analysis: {time.time()-t0:.4f} s")
    
    print("\n[Step 3] Training SVM...")
    planner.train_manifold()
    
    print("\n[Step 4] Finding Manifold Seeds...")
    t0 = time.time()
    seeds = []
    min_dist = float('inf')
    best_seed = None
    
    s_indices = list(comp_start)
    g_indices = list(comp_goal)
    
    for _ in range(1000): 
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
    scale = 0.5 
    valid_proof = False
    final_facets = []
    
    # We need bounds accessible for the 2D visualization
    bounds = [(-3.14, 3.14)] * dof 
    
    while scale > 0.05:
        facets = planner.construct_proof(seeds, scale)
        print(f"   [Info] Generated {len(facets)} facets at scale {scale:.3f}.")
        
        if len(facets) == 0:
            scale *= 0.8
            continue
            
        # FIX 1: Always store the latest facets so we can visualize the failed attempts
        final_facets = facets 
            
        t_check = time.time()
        
        # Check proof using Miniball and Elastic updates
        if dof <= 3:
            epsilon = 0.1
        elif dof == 4:
            epsilon = 0.3  # Stops deep bisection in 4D
        else:
            epsilon = 0.5
        is_valid = check_proof_in_python(
            facets, sim, planner, 
            training_samples, training_labels, 
            epsilon=epsilon
        )
        
        if is_valid:
            print(f"   [Timer] Verification: {time.time()-t_check:.4f} s")
            print(">>> PROOF VALID! Triangulation tightly separates Start and Goal.")
            valid_proof = True
            break # Exits loop with final_facets holding the successful mesh
        else:
            print(f"   [Timer] Verification: {time.time()-t_check:.4f} s (Failed)")
            print(f"   Elastic Updates failed at scale {scale:.3f}. Reducing global scale...")
            scale *= 0.8

    print("\n[Step 6] Visualizing Results...")
    if dof == 2:
        # Pass planner and bounds to the 2D visualizer to draw the SVM
        visualize_2d(samples, comp_start, comp_goal, final_facets, valid_proof, planner, bounds)
    elif dof == 3:
        visualize_3d(samples, comp_start, comp_goal, final_facets, valid_proof)
    else:
        print(f"Visualization skipped. {dof}-DOF cannot be cleanly projected into a static plot.")

def visualize_2d(samples, comp_start, comp_goal, facets, is_valid, planner, bounds):
    fig, ax = plt.subplots(figsize=(10, 8))
    
    s_pts = samples[list(comp_start)]
    ax.scatter(s_pts[:,0], s_pts[:,1], c='blue', alpha=0.3, label='Start Comp')
    
    g_pts = samples[list(comp_goal)]
    ax.scatter(g_pts[:,0], g_pts[:,1], c='green', alpha=0.3, label='Goal Comp')
    
    # Draw Triangulation
    color = 'black' if is_valid else 'red'
    for i, facet in enumerate(facets):
        f = np.array(facet)
        if len(f) == 2:
            # Add label only once to prevent legend clutter
            ax.plot(f[:,0], f[:,1], c=color, linewidth=2, label='Triangulation' if i==0 else "")
            
    # FIX 2: Re-add the SVM Boundary plotting
    print("   [Info] Generating SVM contour for visualization...")
    x = np.linspace(bounds[0][0], bounds[0][1], 100)
    y = np.linspace(bounds[1][0], bounds[1][1], 100)
    X, Y = np.meshgrid(x, y)
    Z = np.zeros_like(X)
    
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            q = np.array([X[i,j], Y[i,j]])
            Z[i,j] = planner.manifold_function(q)
            
    ax.contour(X, Y, Z, levels=[0], colors='purple', linestyles='dashed', linewidths=2, label='SVM Boundary')

    ax.set_title(f"2D Infeasibility Proof (Valid: {is_valid})")
    # Put legend outside the plot so it doesn't cover data
    ax.legend(loc='upper right', bbox_to_anchor=(1.15, 1)) 
    plt.tight_layout()
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
        if len(f) == 3:
            f_loop = np.vstack([f, f[0]]) 
            ax.plot(f_loop[:,0], f_loop[:,1], f_loop[:,2], c=color, linewidth=1)
            
    ax.set_title(f"3D Infeasibility Proof (Valid: {is_valid})")
    ax.legend()
    plt.show()

if __name__ == "__main__":
    run()