#!/usr/bin/env python3
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# ----------------- Configuration -----------------
# Replace these with the paths to your full run CSV logs
FILE_REF = "ref_run3.csv"
FILE_P09 = "p09_run3.csv"

# Commanded square side length L (default from node is 1.0 m or whatever you passed)
L = 2.0
DIRECTION = "ccw"  # 'ccw' or 'cw'

# Ideal commanded square vertices
if DIRECTION == "ccw":
    ideal_x = [0.0, L, L, 0.0, 0.0]
    ideal_y = [0.0, 0.0, L, L, 0.0]
else:
    ideal_x = [0.0, L, L, 0.0, 0.0]
    ideal_y = [0.0, 0.0, -L, -L, 0.0]

# ----------------- Load Data -----------------
df_ref = pd.read_csv(FILE_REF)
df_p09 = pd.read_csv(FILE_P09)

# ----------------- Create Plots -----------------
fig, axes = plt.subplots(1, 2, figsize=(13, 6), sharey=True)

robots = [
    ("Reference Robot (REF)", df_ref, axes[0]),
    ("Assigned Robot (P09)", df_p09, axes[1])
]

for title, df, ax in robots:
    # 1. Commanded / Ideal Square
    ax.plot(ideal_x, ideal_y, 'k--', linewidth=2, label='Commanded Square')

    # 2. Odometry Path (/odom)
    ax.plot(df['odom_x'], df['odom_y'], color='tab:blue', linewidth=1.8, label='/odom Path')

    # 3. Ground Truth Path (/ground_truth/odom)
    ax.plot(df['gt_x'], df['gt_y'], color='tab:red', linewidth=1.8, linestyle='-.', label='True Path (GT)')

    # Start and End Markers
    ax.plot(df['gt_x'].iloc[0], df['gt_y'].iloc[0], 'go', markersize=8, label='Start')
    ax.plot(df['gt_x'].iloc[-1], df['gt_y'].iloc[-1], 'rx', markersize=9, markeredgewidth=2, label='True End')

    ax.set_title(title, fontsize=13, fontweight='bold')
    ax.set_xlabel('X (m)', fontsize=11)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.set_aspect('equal', adjustable='box')  # Crucial: equal axis scaling
    ax.legend(loc='best', framealpha=0.9)

axes[0].set_ylabel('Y (m)', fontsize=11)

plt.tight_layout()
plt.savefig('open_loop_square_comparison.png', dpi=300)
plt.show()