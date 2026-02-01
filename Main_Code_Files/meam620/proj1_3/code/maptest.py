import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Define the map data
map_data = {

    "bounds": {
        "extents": [-3.0, 3.0, -2.5, 2.5, 0, 2.5]
    },
    "blocks": [
        # Ground-level blocks
        {"extents": [-2.5, -2.0, -2.0, -1.5, 0, 1.2]},
        {"extents": [-1.5, -1.0, 1.0, 1.5, 0, 1.8]},
        {"extents": [0.0, 0.5, -1.5, -0.5, 0, 1.5]},
        {"extents": [1.0, 1.5, 0.5, 1.5, 0, 1.3]},
        {"extents": [2.0, 2.5, -1.0, -0.5, 0, 1.6]},
        {"extents": [-1.0, 0.0, -2.0, -1.5, 0, 1.0]},
        {"extents": [0.5, 1.0, -2.0, -1.8, 0, 1.7]},
        {"extents": [-2.0, -1.5, 0.0, 0.5, 0, 1.4]},
        {"extents": [1.5, 2.0, -2.0, -1.5, 0, 1.9]},
        
        # Overhanging blocks (tunnel-like structures)
        {"extents": [-1.0, 0.0, -1.8, -1.2, 1.5, 2.0]},  # Elevated block creating a tunnel
        {"extents": [0.5, 1.5, -0.5, 0.5, 1.2, 2.0]},    # Overhanging block with space below
        {"extents": [-2.0, -1.5, 1.0, 1.5, 1.3, 2.2]},   # Floating block creating a gap
        {"extents": [1.0, 1.5, -2.0, -1.2, 1.6, 2.5]},   # Higher overhang
        {"extents": [-0.5, 0.5, -0.5, 0.5, 1.8, 2.3]}    # Central overhanging block
    ],
    "start": [-2.8, -2.3, 0.5],
    "goal": [2.8, 2.3, 0.5]

}



def draw_block(ax, extents, color='black', alpha=0.5):
    """Draws a rectangular block in 3D space"""
    x_min, x_max = extents[0], extents[1]
    y_min, y_max = extents[2], extents[3]
    z_min, z_max = extents[4], extents[5]

    # Define the 8 corners of the cuboid
    corners = np.array([
        [x_min, y_min, z_min], [x_max, y_min, z_min],
        [x_max, y_max, z_min], [x_min, y_max, z_min],
        [x_min, y_min, z_max], [x_max, y_min, z_max],
        [x_max, y_max, z_max], [x_min, y_max, z_max]
    ])

    # Define the faces using vertex indices
    faces = [
        [corners[j] for j in [0, 1, 2, 3]],  # Bottom face
        [corners[j] for j in [4, 5, 6, 7]],  # Top face
        [corners[j] for j in [0, 1, 5, 4]],  # Front face
        [corners[j] for j in [2, 3, 7, 6]],  # Back face
        [corners[j] for j in [1, 2, 6, 5]],  # Right face
        [corners[j] for j in [0, 3, 7, 4]]   # Left face
    ]

    ax.add_collection3d(Poly3DCollection(faces, color=color, alpha=alpha, edgecolor='black'))

def plot_world(map_data):
    """Plots the 3D environment with obstacles, start, and goal points"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Get world bounds
    bounds = map_data["bounds"]["extents"]
    ax.set_xlim(bounds[0], bounds[1])
    ax.set_ylim(bounds[2], bounds[3])
    ax.set_zlim(bounds[4], bounds[5])

    # Plot obstacles
    for block in map_data["blocks"]:
        draw_block(ax, block["extents"])

    # Plot start and goal
    start = np.array(map_data["start"])
    goal = np.array(map_data["goal"])

    ax.scatter(*start, color='green', s=100, label="Start", edgecolors='black')
    ax.scatter(*goal, color='red', s=100, label="Goal", edgecolors='black')

    # Labels and title
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.set_title("3D Map Visualization")
    ax.legend()

    plt.show()

# Call function to plot the world
plot_world(map_data)
