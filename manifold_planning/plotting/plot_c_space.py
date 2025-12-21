import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Read the data
circle_segments = pd.read_csv('circle_segments.csv')
coxeter_edges = pd.read_csv('coxeter_edges.csv')
vertices = pd.read_csv('circle_vertices.csv')

# Create figure with two subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

# ========== Left plot: Circle approximation ==========
ax1.set_title('Circle Approximation (Intersection Points Connected)', fontsize=14, fontweight='bold')

# Plot circle segments (edges along the circle)
for _, row in circle_segments.iterrows():
    ax1.plot([row['x0'], row['x1']], 
            [row['y0'], row['y1']], 
            'b-', linewidth=2, alpha=0.8)

# Plot vertices
ax1.plot(vertices['x'], vertices['y'], 'ro', markersize=6, 
         label=f'Intersection points ({len(vertices)})', zorder=5)

# Add reference circle
theta = np.linspace(0, 2*np.pi, 1000)
radius = 3.0
circle_x = radius * np.cos(theta)
circle_y = radius * np.sin(theta)
ax1.plot(circle_x, circle_y, 'g--', linewidth=1.5, alpha=0.5, label='True circle')

ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3)
ax1.set_xlabel('X', fontsize=12)
ax1.set_ylabel('Y', fontsize=12)
ax1.legend(fontsize=10)

# ========== Right plot: Coxeter triangulation edges ==========
ax2.set_title('Coxeter Triangulation Edges Intersecting Circle', fontsize=14, fontweight='bold')

# Plot Coxeter triangulation edges
for _, row in coxeter_edges.iterrows():
    ax2.plot([row['x0'], row['x1']], 
            [row['y0'], row['y1']], 
            'b-', linewidth=1.5, alpha=0.7)

# Plot intersection points
ax2.plot(vertices['x'], vertices['y'], 'ro', markersize=6, 
         label=f'Intersection points ({len(vertices)})', zorder=5)

# Add reference circle
ax2.plot(circle_x, circle_y, 'g--', linewidth=1.5, alpha=0.5, label='True circle')

ax2.set_aspect('equal')
ax2.grid(True, alpha=0.3)
ax2.set_xlabel('X', fontsize=12)
ax2.set_ylabel('Y', fontsize=12)
ax2.legend(fontsize=10)

# Set same limits for both plots
margin = 0.5
x_min = min(vertices['x'].min(), coxeter_edges[['x0', 'x1']].min().min()) - margin
x_max = max(vertices['x'].max(), coxeter_edges[['x0', 'x1']].max().max()) + margin
y_min = min(vertices['y'].min(), coxeter_edges[['y0', 'y1']].min().min()) - margin
y_max = max(vertices['y'].max(), coxeter_edges[['y0', 'y1']].max().max()) + margin

ax1.set_xlim(x_min, x_max)
ax1.set_ylim(y_min, y_max)
ax2.set_xlim(x_min, x_max)
ax2.set_ylim(y_min, y_max)

plt.tight_layout()
plt.savefig('circle_triangulation.png', dpi=300, bbox_inches='tight')
plt.show()

print(f"Left plot: {len(circle_segments)} segments connecting intersection points")
print(f"Right plot: {len(coxeter_edges)} Coxeter triangulation edges")
print(f"Total intersection points: {len(vertices)}")
print("Saved as circle_triangulation.png")