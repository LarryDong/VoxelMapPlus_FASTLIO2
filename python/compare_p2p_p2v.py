
# plot and compare p2p, p2v for each scan


import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

VSCODE_DEBUG = True

# Specify the base_folder containing the files
base_folder = '/home/larry/Desktop/p2v_debug/'


# Create a new 3D figure
fig = plt.figure()
ax = fig.add_subplot(211, projection='3d')
ax2 = fig.add_subplot(212, projection='3d')


alpha = 0.1



idx = 3
while True:
    
    ax.clear()
    ax2.clear()

    # Construct filenames with 3-digit zero padding
    p2v_filename = os.path.join(base_folder, f"{idx}_p2v.txt")
    p2p_filename = os.path.join(base_folder, f"{idx}_p2p.txt")
    
    # Check if files exist
    if not os.path.exists(p2v_filename) or not os.path.exists(p2p_filename):
        if idx == 3:
            raise FileNotFoundError("No files found starting with index 3")
        else:
            print(f"Reached last index: {idx-1}")
            break
    
    # Read p2v file
    p2v = np.loadtxt(p2v_filename, delimiter=',')
    # Read p2p file
    p2p = np.loadtxt(p2p_filename, delimiter=',')
    

    # Plot p2v vectors (green)
    if p2v.size == 0:
        print(f"--> Empty p2v file for index {idx}")
    else:
        ax2.quiver(
            np.zeros(p2v.shape[0]), np.zeros(p2v.shape[0]), np.zeros(p2v.shape[0]),
            p2v[:, 0], p2v[:, 1], p2v[:, 2],
            color='g', arrow_length_ratio=0.1, length=1.0, alpha=alpha,
            label='p2v'
        )
    
    # Plot p2p vectors (red)
    if p2p.size == 0:
        print(f"--> Empty p2p file for index {idx}")
    else:
        ax.quiver(
            np.zeros(p2p.shape[0]), np.zeros(p2p.shape[0]), np.zeros(p2p.shape[0]),
            p2p[:, 0], p2p[:, 1], p2p[:, 2],
            color='r', arrow_length_ratio=0.1, length=1.0, alpha=alpha,
            label='p2p'
        )
    idx += 1

    # Set equal aspect ratio
    ax.set_box_aspect([1, 1, 1])  # For equal aspect ratio in all dimensions
    ax.set_title(f'scan: {idx}')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.view_init(elev=90, azim=-270)
    ax.legend()

    # Set equal aspect ratio
    ax2.set_box_aspect([1, 1, 1])  # For equal aspect ratio in all dimensions
    ax2.set_title(f'scan: {idx}')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.view_init(elev=90, azim=-270)
    ax2.legend()


    plt.show(block=False)

    if VSCODE_DEBUG:
        pass



