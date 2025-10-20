#!/usr/bin/env python3
"""
Visualize all 9 lanes (3 GGV profiles × 3 lanes) in 3D
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import re

# Configuration
module = os.path.dirname(os.path.abspath(__file__))
map_name = "f1tenth"

# Read v_max from racecar.ini
ini_path = os.path.join(module, "params", "racecar.ini")
v_max = 9.0  # Default value

try:
    with open(ini_path, 'r') as f:
        for line in f:
            if '"v_max":' in line:
                match = re.search(r'"v_max":\s*([0-9.]+)', line)
                if match:
                    v_max = float(match.group(1))
                    print(f"Read v_max = {v_max} m/s from racecar.ini")
                    break
except Exception as e:
    print(f"Warning: Could not read v_max from racecar.ini, using default {v_max} m/s")
    print(f"  Error: {e}")

# GGV profiles
ggv_profiles = ["conservative", "normal", "aggressive"]
lane_types = ["left", "optimal", "right"]

# Color scheme
# Left: Red, Optimal: Green, Right: Blue
# Conservative: Light, Normal: Medium, Aggressive: Dark
colors = {
    "conservative": {
        "left": "#FF9999",      # Light red
        "optimal": "#90EE90",   # Light green
        "right": "#ADD8E6"      # Light blue
    },
    "normal": {
        "left": "#FF0000",      # Red
        "optimal": "#00FF00",   # Lime green
        "right": "#0000FF"      # Blue
    },
    "aggressive": {
        "left": "#8B0000",      # Dark red
        "optimal": "#006400",   # Dark green
        "right": "#00008B"      # Dark blue
    }
}

# Line widths for distinction
linewidths = {
    "conservative": 1.5,
    "normal": 2.0,
    "aggressive": 2.5
}

# Load all lane data
lane_data = {}
print("Loading lane data...")

for ggv in ggv_profiles:
    lane_data[ggv] = {}
    subdir = f"{ggv}_{v_max}"

    for lane in lane_types:
        file_path = os.path.join(module, "..", "path", map_name, subdir, f"lane_{lane}.csv")

        if os.path.exists(file_path):
            data = []
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('#') or not line:
                        continue
                    parts = line.split(',')
                    if len(parts) >= 3:
                        x, y, v = float(parts[0]), float(parts[1]), float(parts[2])
                        data.append([x, y, v])

            if data:
                lane_data[ggv][lane] = np.array(data)
                print(f"  ✓ Loaded {ggv}_{lane}: {len(data)} points")
        else:
            print(f"  ✗ Missing {ggv}_{lane}")

# Create 3D plot
print("\nCreating 3D visualization...")
fig = plt.figure(figsize=(18, 12))
ax = fig.add_subplot(111, projection='3d')

# Plot all lanes
for ggv in ggv_profiles:
    for lane in lane_types:
        if lane in lane_data[ggv]:
            data = lane_data[ggv][lane]
            x = data[:, 0]
            y = data[:, 1]
            v = data[:, 2]

            color = colors[ggv][lane]
            linewidth = linewidths[ggv]
            label = f"{ggv.capitalize()} - {lane.capitalize()}"

            ax.plot(x, y, v, color=color, linewidth=linewidth, label=label, zorder=10-ggv_profiles.index(ggv))

# Configure plot
ax.set_xlabel('X [m]', fontsize=12, fontweight='bold')
ax.set_ylabel('Y [m]', fontsize=12, fontweight='bold')
ax.set_zlabel('Velocity [m/s]', fontsize=12, fontweight='bold')
ax.set_title(f'F1TENTH Trajectory Comparison\n9 Lanes (3 GGV profiles × 3 lanes)\nv_max = {v_max} m/s',
             fontsize=14, fontweight='bold', pad=20)

# Equal aspect ratio for x and y
if len(lane_data) > 0:
    all_x = []
    all_y = []
    for ggv in ggv_profiles:
        for lane in lane_types:
            if lane in lane_data[ggv]:
                all_x.extend(lane_data[ggv][lane][:, 0])
                all_y.extend(lane_data[ggv][lane][:, 1])

    if all_x and all_y:
        max_range = np.array([max(all_x) - min(all_x), max(all_y) - min(all_y)]).max() / 2.0
        mid_x = (max(all_x) + min(all_x)) * 0.5
        mid_y = (max(all_y) + min(all_y)) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)

# Legend
ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1), fontsize=9, framealpha=0.9)

# Grid
ax.grid(True, alpha=0.3)

# View angle
ax.view_init(elev=25, azim=45)

# Tight layout
plt.tight_layout()

# Save figure
output_path = os.path.join(module, "..", "path", map_name, "all_lanes_3d_visualization.png")
os.makedirs(os.path.dirname(output_path), exist_ok=True)
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"\n✓ Saved visualization: {output_path}")

# Show plot
plt.show()

print("\nVisualization complete!")
