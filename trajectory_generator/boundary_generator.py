#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Complete Boundary Generator with GUI
- Extract boundaries from map image
- Auto-simplification with Douglas-Peucker
- Interactive editing with GUI
- Save to CSV
"""

import cv2
import csv
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import argparse
import yaml
import os


# ============================================================
# Douglas-Peucker Algorithm
# ============================================================

def point_line_distance(point, line_start, line_end):
    """Calculate perpendicular distance from point to line segment"""
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end

    line_len_sq = (x2 - x1)**2 + (y2 - y1)**2

    if line_len_sq < 1e-10:
        return math.sqrt((px - x1)**2 + (py - y1)**2)

    t = max(0.0, min(1.0, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_len_sq))

    proj_x = x1 + t * (x2 - x1)
    proj_y = y1 + t * (y2 - y1)

    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)


def douglas_peucker(points, tolerance):
    """Douglas-Peucker polygon simplification algorithm"""
    if len(points) <= 2:
        return points

    max_dist = 0.0
    max_idx = 0

    for i in range(1, len(points) - 1):
        dist = point_line_distance(points[i], points[0], points[-1])
        if dist > max_dist:
            max_dist = dist
            max_idx = i

    if max_dist > tolerance:
        left = douglas_peucker(points[:max_idx + 1], tolerance)
        right = douglas_peucker(points[max_idx:], tolerance)
        return np.vstack([left[:-1], right])
    else:
        return np.array([points[0], points[-1]])


# ============================================================
# Lane Generator Helper Functions
# ============================================================

def reorder_vertex(image, lane):
    """Reorder vertices to form continuous path"""
    path_img = np.zeros_like(image)
    for idx in range(len(lane)):
        cv2.circle(path_img, tuple(lane[idx]), 1, (255, 255, 255), 1)

    curr_kernel = np.ones((3, 3), np.uint8)
    iter_cnt = 0
    while True:
        if iter_cnt > 10:
            print("⚠️  Unable to reorder vertex, using original order")
            return lane
        curr_contours, curr_hierarchy = cv2.findContours(path_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(curr_contours) == 2 and curr_hierarchy[0][-1][-1] == 0:
            break
        path_img = cv2.dilate(path_img, curr_kernel, iterations=1)
        iter_cnt += 1

    path_img = cv2.ximgproc.thinning(path_img)
    curr_contours, curr_hierarchy = cv2.findContours(path_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return np.squeeze(curr_contours[0])


def transform_coords(path, height, scale, tx, ty):
    """Transform from pixel coordinates to map coordinates"""
    new_path_x = path[:, 0] * scale + tx
    new_path_y = (height - path[:, 1]) * scale + ty
    return np.vstack((new_path_x, new_path_y)).T


def save_csv_simple(data, csv_path):
    """Save 2D array to CSV"""
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        for row in data:
            writer.writerow([f"{row[0]:.6f}", f"{row[1]:.6f}"])


# ============================================================
# Boundary Extractor (from lane_generator)
# ============================================================

def extract_boundaries_from_map(map_path, yaml_path, inner_safe_dist=0.3, outer_safe_dist=0.3):
    """
    Extract inner and outer boundaries from map image

    Parameters:
        map_path: Path to map image (pgm/png)
        yaml_path: Path to map yaml file
        inner_safe_dist: Safety distance from inner bound (m)
        outer_safe_dist: Safety distance from outer bound (m)

    Returns:
        (outer_bound, inner_bound): Tuple of nx2 numpy arrays in map coordinates
    """
    print("\n" + "="*60)
    print("🗺️  Extracting Boundaries from Map")
    print("="*60)

    # Read map yaml
    with open(yaml_path, 'r') as stream:
        yaml_data = yaml.safe_load(stream)

    scale = yaml_data["resolution"]
    offset_x = yaml_data["origin"][0]
    offset_y = yaml_data["origin"][1]

    print(f"📊 Map metadata:")
    print(f"   Resolution: {scale} m/pixel")
    print(f"   Origin: ({offset_x}, {offset_y})")

    # Read image
    input_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    h, w = input_img.shape[:2]
    print(f"   Image size: {w}x{h}")

    # Flip black and white
    output_img = ~input_img

    # Convert to binary
    ret, output_img = cv2.threshold(output_img, thresh=127, maxval=255, type=cv2.THRESH_BINARY)

    # Remove small contours
    contours, hierarchy = cv2.findContours(output_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) < 70:
            cv2.fillPoly(output_img, pts=[contour], color=(0, 0, 0))

    # Dilate & Thin
    kernel = np.ones((5, 5), np.uint8)
    output_img = cv2.dilate(output_img, kernel, iterations=1)
    output_img = cv2.ximgproc.thinning(output_img)

    # Find outer and inner boundaries
    print("🔍 Finding track boundaries...")
    contours, hierarchy = cv2.findContours(output_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    parents = hierarchy[0][:, 3]

    if np.max(parents) < 1:
        raise ValueError("❌ Invalid track structure! Need at least 3 hierarchy levels.")

    # Find tree structure
    node = np.argmax(parents)
    tree_indices = []
    while node != -1:
        tree_indices.append(node)
        node = parents[node]
    tree_indices.reverse()

    outer_bound_contour = contours[tree_indices[1]]
    inner_bound_contour = contours[tree_indices[2]]

    print(f"✅ Found boundaries:")
    print(f"   Outer: {len(outer_bound_contour)} pixels")
    print(f"   Inner: {len(inner_bound_contour)} pixels")

    # Extract boundary points with safety distance
    print("📏 Applying safety distances...")
    X, Y = np.meshgrid(np.arange(w), np.arange(h))
    X = X.flatten().tolist()
    Y = Y.flatten().tolist()

    outer_bound_pts = []
    inner_bound_pts = []

    for (x, y) in zip(X, Y):
        outer_dist = cv2.pointPolygonTest(outer_bound_contour, (x, y), True)
        inner_dist = cv2.pointPolygonTest(inner_bound_contour, (x, y), True)

        # Outer bound with safety distance
        if abs(outer_dist - outer_safe_dist / scale) < 2:
            outer_bound_pts.append([x, y])

        # Inner bound with safety distance
        if abs(inner_dist + inner_safe_dist / scale) < 2:
            inner_bound_pts.append([x, y])

    outer_bound_pts = np.array(outer_bound_pts)
    inner_bound_pts = np.array(inner_bound_pts)

    # Reorder vertices
    print("🔄 Reordering vertices...")
    outer_bound_pts = reorder_vertex(output_img, outer_bound_pts)
    inner_bound_pts = reorder_vertex(output_img, inner_bound_pts)

    # Transform to map coordinates
    print("🌍 Transforming to map coordinates...")
    outer_bound = transform_coords(outer_bound_pts, h, scale, offset_x, offset_y)
    inner_bound = transform_coords(inner_bound_pts, h, scale, offset_x, offset_y)

    print(f"✅ Extraction complete:")
    print(f"   Outer: {len(outer_bound)} points")
    print(f"   Inner: {len(inner_bound)} points")
    print("="*60 + "\n")

    return outer_bound, inner_bound


# ============================================================
# Interactive GUI
# ============================================================

class BoundaryGeneratorGUI:
    def __init__(self, outer_bound, inner_bound, output_dir, map_name, initial_tolerance=0.10):
        """
        Interactive boundary generator with simplification

        Parameters:
            outer_bound: nx2 numpy array (map coordinates)
            inner_bound: nx2 numpy array (map coordinates)
            output_dir: Output directory for CSV files
            map_name: Name for output files
            initial_tolerance: Initial Douglas-Peucker tolerance (m)
        """
        self.outer_original = outer_bound
        self.inner_original = inner_bound
        self.output_dir = output_dir
        self.map_name = map_name

        # Manual removal tracking
        self.outer_manually_removed = set()
        self.inner_manually_removed = set()

        # Simplification (각각 독립적인 tolerance)
        self.outer_tolerance = initial_tolerance
        self.inner_tolerance = initial_tolerance
        self.outer_simplified = None
        self.inner_simplified = None
        self.simplify()

        # Active boundary ('outer' or 'inner')
        self.active_boundary = 'outer'

        # Setup GUI
        self.fig = plt.figure(figsize=(16, 10))
        self.gs = self.fig.add_gridspec(5, 2, height_ratios=[15, 1, 1, 1, 1], hspace=0.3)

        self.ax_main = self.fig.add_subplot(self.gs[0, :])
        self.ax_slider_outer = self.fig.add_subplot(self.gs[1, :])
        self.ax_slider_inner = self.fig.add_subplot(self.gs[2, :])
        self.ax_radio = self.fig.add_subplot(self.gs[3, 0])
        self.ax_btn_save = self.fig.add_subplot(self.gs[3, 1])
        self.ax_btn_reset = self.fig.add_subplot(self.gs[4, :])

        self.setup_widgets()
        self.setup_plot()
        self.connect_events()

        self.print_instructions()

    def print_instructions(self):
        print("\n" + "="*60)
        print("🎨 Boundary Generator GUI")
        print("="*60)
        print(f"📂 Output: {self.output_dir}")
        print(f"📝 Map: {self.map_name}")
        print(f"📍 Outer: {len(self.outer_original)} → {len(self.outer_simplified)} pts (tol={self.outer_tolerance}m)")
        print(f"📍 Inner: {len(self.inner_original)} → {len(self.inner_simplified)} pts (tol={self.inner_tolerance}m)")
        print("\n🖱️  Controls:")
        print("  - OUTER SLIDER: Adjust outer tolerance (0.01 ~ 0.50m)")
        print("  - INNER SLIDER: Adjust inner tolerance (0.01 ~ 0.50m)")
        print("  - RADIO:        Select boundary (Outer/Inner)")
        print("  - LEFT CLICK:   Remove/restore point")
        print("  - RESET btn:    Reset to auto-simplified")
        print("  - SAVE btn:     Save and exit")
        print("  - 's' key:      Save and exit")
        print("  - 'q' key:      Quit without saving")
        print("="*60 + "\n")

    def simplify(self):
        """Apply Douglas-Peucker simplification"""
        self.outer_simplified = douglas_peucker(self.outer_original, self.outer_tolerance)
        self.inner_simplified = douglas_peucker(self.inner_original, self.inner_tolerance)
        self.outer_manually_removed.clear()
        self.inner_manually_removed.clear()

    def setup_widgets(self):
        """Setup GUI widgets"""
        # Outer tolerance slider
        self.slider_outer = Slider(
            ax=self.ax_slider_outer,
            label='Outer Tolerance (m)',
            valmin=0.01,
            valmax=0.50,
            valinit=self.outer_tolerance,
            valstep=0.01,
            color='blue'
        )
        self.slider_outer.on_changed(self.on_outer_slider_change)

        # Inner tolerance slider
        self.slider_inner = Slider(
            ax=self.ax_slider_inner,
            label='Inner Tolerance (m)',
            valmin=0.01,
            valmax=0.50,
            valinit=self.inner_tolerance,
            valstep=0.01,
            color='red'
        )
        self.slider_inner.on_changed(self.on_inner_slider_change)

        # Radio buttons for boundary selection
        self.radio = RadioButtons(
            ax=self.ax_radio,
            labels=['Outer Boundary', 'Inner Boundary'],
            active=0
        )
        self.radio.on_clicked(self.on_radio_change)

        # Save button
        self.btn_save = Button(self.ax_btn_save, 'SAVE & EXIT',
                               color='lightgreen', hovercolor='green')
        self.btn_save.on_clicked(self.on_save_click)

        # Reset button
        self.btn_reset = Button(self.ax_btn_reset, 'RESET TO AUTO-SIMPLIFIED',
                               color='lightcoral', hovercolor='red')
        self.btn_reset.on_clicked(self.on_reset_click)

    def setup_plot(self):
        """Setup matplotlib plot"""
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3, linestyle='--')
        self.ax_main.set_xlabel('X (m)', fontsize=12)
        self.ax_main.set_ylabel('Y (m)', fontsize=12)
        self.redraw()

    def get_active_outer(self):
        """Get active outer points"""
        indices = [i for i in range(len(self.outer_simplified))
                  if i not in self.outer_manually_removed]
        return self.outer_simplified[indices]

    def get_active_inner(self):
        """Get active inner points"""
        indices = [i for i in range(len(self.inner_simplified))
                  if i not in self.inner_manually_removed]
        return self.inner_simplified[indices]

    def update_title(self):
        """Update plot title"""
        outer_active = len(self.get_active_outer())
        inner_active = len(self.get_active_inner())
        outer_reduction = ((len(self.outer_original) - outer_active) / len(self.outer_original)) * 100
        inner_reduction = ((len(self.inner_original) - inner_active) / len(self.inner_original)) * 100
        outer_speedup = len(self.outer_original) / outer_active if outer_active > 0 else 0
        inner_speedup = len(self.inner_original) / inner_active if inner_active > 0 else 0

        title = f"Boundary Generator - {self.map_name}\n"
        title += f"🔵 Outer: {len(self.outer_original)} → {outer_active} pts "
        title += f"({outer_reduction:.1f}% ↓, {outer_speedup:.1f}x ⚡) | "
        title += f"🔴 Inner: {len(self.inner_original)} → {inner_active} pts "
        title += f"({inner_reduction:.1f}% ↓, {inner_speedup:.1f}x ⚡)"

        self.ax_main.set_title(title, fontsize=12, fontweight='bold', pad=20)

    def redraw(self):
        """Redraw entire plot"""
        self.ax_main.clear()
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3, linestyle='--')
        self.ax_main.set_xlabel('X (m)', fontsize=12)
        self.ax_main.set_ylabel('Y (m)', fontsize=12)
        self.update_title()

        # Original boundaries (faded background)
        self.ax_main.plot(self.outer_original[:, 0], self.outer_original[:, 1],
                         'o', color='lightblue', markersize=2, alpha=0.2, label='Outer (original)')
        self.ax_main.plot(self.inner_original[:, 0], self.inner_original[:, 1],
                         'o', color='lightcoral', markersize=2, alpha=0.2, label='Inner (original)')

        # Active boundaries (lines)
        outer_active = self.get_active_outer()
        inner_active = self.get_active_inner()

        if len(outer_active) > 0:
            loop = np.vstack([outer_active, outer_active[0]])
            self.ax_main.plot(loop[:, 0], loop[:, 1], 'b-', linewidth=2.5, alpha=0.8)

        if len(inner_active) > 0:
            loop = np.vstack([inner_active, inner_active[0]])
            self.ax_main.plot(loop[:, 0], loop[:, 1], 'r-', linewidth=2.5, alpha=0.8)

        # Simplified points
        for idx, pt in enumerate(self.outer_simplified):
            if idx in self.outer_manually_removed:
                self.ax_main.scatter(pt[0], pt[1], c='gray', s=40, alpha=0.4, marker='x')
            else:
                size = 120 if self.active_boundary == 'outer' else 60
                self.ax_main.scatter(pt[0], pt[1], c='blue', s=size, alpha=0.9,
                                   edgecolors='black', linewidths=1.5, zorder=4, picker=5)

        for idx, pt in enumerate(self.inner_simplified):
            if idx in self.inner_manually_removed:
                self.ax_main.scatter(pt[0], pt[1], c='gray', s=40, alpha=0.4, marker='x')
            else:
                size = 120 if self.active_boundary == 'inner' else 60
                self.ax_main.scatter(pt[0], pt[1], c='red', s=size, alpha=0.9,
                                   edgecolors='black', linewidths=1.5, zorder=4, picker=5)

        # Highlight active boundary
        if self.active_boundary == 'outer':
            self.ax_main.text(0.02, 0.98, '🔵 Editing: OUTER',
                            transform=self.ax_main.transAxes, fontsize=14, fontweight='bold',
                            verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        else:
            self.ax_main.text(0.02, 0.98, '🔴 Editing: INNER',
                            transform=self.ax_main.transAxes, fontsize=14, fontweight='bold',
                            verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))

        self.fig.canvas.draw_idle()

    def find_nearest_point(self, x, y):
        """Find nearest point in active boundary"""
        if self.active_boundary == 'outer':
            points = self.outer_simplified
        else:
            points = self.inner_simplified

        if len(points) == 0:
            return None

        distances = np.sqrt((points[:, 0] - x)**2 + (points[:, 1] - y)**2)
        nearest_idx = np.argmin(distances)

        if distances[nearest_idx] < 0.5:
            return nearest_idx
        return None

    def on_click(self, event):
        """Handle mouse click"""
        if event.inaxes != self.ax_main or event.button != 1:
            return

        idx = self.find_nearest_point(event.xdata, event.ydata)
        if idx is not None:
            if self.active_boundary == 'outer':
                if idx in self.outer_manually_removed:
                    self.outer_manually_removed.remove(idx)
                    print(f"✅ Restored outer point {idx}")
                else:
                    self.outer_manually_removed.add(idx)
                    print(f"❌ Removed outer point {idx}")
            else:
                if idx in self.inner_manually_removed:
                    self.inner_manually_removed.remove(idx)
                    print(f"✅ Restored inner point {idx}")
                else:
                    self.inner_manually_removed.add(idx)
                    print(f"❌ Removed inner point {idx}")

            self.redraw()

    def on_outer_slider_change(self, val):
        """Handle outer tolerance slider change"""
        self.outer_tolerance = val
        print(f"🔵 Outer Tolerance: {self.outer_tolerance:.2f}m")
        # Only re-simplify outer
        self.outer_simplified = douglas_peucker(self.outer_original, self.outer_tolerance)
        self.outer_manually_removed.clear()
        self.redraw()

    def on_inner_slider_change(self, val):
        """Handle inner tolerance slider change"""
        self.inner_tolerance = val
        print(f"🔴 Inner Tolerance: {self.inner_tolerance:.2f}m")
        # Only re-simplify inner
        self.inner_simplified = douglas_peucker(self.inner_original, self.inner_tolerance)
        self.inner_manually_removed.clear()
        self.redraw()

    def on_radio_change(self, label):
        """Handle radio button change"""
        if label == 'Outer Boundary':
            self.active_boundary = 'outer'
            print("🔵 Switched to OUTER boundary")
        else:
            self.active_boundary = 'inner'
            print("🔴 Switched to INNER boundary")
        self.redraw()

    def on_save_click(self, event):
        """Handle save button"""
        self.save_and_exit()

    def on_reset_click(self, event):
        """Handle reset button"""
        print("🔄 Reset to auto-simplified")
        self.outer_manually_removed.clear()
        self.inner_manually_removed.clear()
        self.redraw()

    def on_key(self, event):
        """Handle keyboard"""
        if event.key == 's':
            self.save_and_exit()
        elif event.key == 'q':
            print("\n⚠️  Exited without saving")
            plt.close(self.fig)

    def save_and_exit(self):
        """Save boundaries and exit"""
        outer_active = self.get_active_outer()
        inner_active = self.get_active_inner()

        if len(outer_active) < 3 or len(inner_active) < 3:
            print("\n⚠️  Too few points! Need at least 3 points each. Not saving.")
            return

        # Save
        os.makedirs(self.output_dir, exist_ok=True)

        outer_path = os.path.join(self.output_dir, "outer_bound.csv")
        inner_path = os.path.join(self.output_dir, "inner_bound.csv")

        save_csv_simple(outer_active, outer_path)
        save_csv_simple(inner_active, inner_path)

        print("\n" + "="*60)
        print("✅ Boundaries Saved!")
        print("="*60)
        print(f"📂 Directory: {self.output_dir}")
        print(f"📄 Files:")
        print(f"   - outer_bound.csv ({len(outer_active)} points)")
        print(f"   - inner_bound.csv ({len(inner_active)} points)")
        print("\n📊 Statistics:")
        outer_reduction = ((len(self.outer_original) - len(outer_active)) / len(self.outer_original)) * 100
        inner_reduction = ((len(self.inner_original) - len(inner_active)) / len(self.inner_original)) * 100
        print(f"   Outer: {len(self.outer_original)} → {len(outer_active)} ({outer_reduction:.1f}% reduction)")
        print(f"   Inner: {len(self.inner_original)} → {len(inner_active)} ({inner_reduction:.1f}% reduction)")
        print(f"   Total: {len(self.outer_original) + len(self.inner_original)} → {len(outer_active) + len(inner_active)} points")
        print(f"   Speedup: {(len(self.outer_original) + len(self.inner_original)) / (len(outer_active) + len(inner_active)):.1f}x ⚡")
        print("="*60 + "\n")

        plt.close(self.fig)

    def connect_events(self):
        """Connect matplotlib events"""
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    def run(self):
        """Start GUI"""
        plt.show()


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(description='Boundary Generator with GUI')
    parser.add_argument('--config', '-c', type=str, default=None,
                       help='Path to config yaml file (default: config/params.yaml)')
    parser.add_argument('--map-path', type=str, default=None,
                       help='Override map path from config')
    parser.add_argument('--yaml-path', type=str, default=None,
                       help='Override yaml path from config')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='Output directory (default: ../bound/map_name/inner_margin_outer_margin)')
    parser.add_argument('--tolerance', '-t', type=float, default=0.10,
                       help='Initial tolerance (m, default: 0.10)')
    parser.add_argument('--inner-safe', type=float, default=None,
                       help='Override inner_safe_dist from config')
    parser.add_argument('--outer-safe', type=float, default=None,
                       help='Override outer_safe_dist from config')

    args = parser.parse_args()

    # Load config file
    script_dir = os.path.dirname(os.path.abspath(__file__))

    if args.config is None:
        config_path = os.path.join(script_dir, "config", "params.yaml")
    else:
        config_path = args.config

    if not os.path.exists(config_path):
        print(f"❌ Config file not found: {config_path}")
        return

    print(f"📄 Loading config: {config_path}")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Get parameters from config
    map_name = config.get('map_name', 'unknown')
    map_img_ext = config.get('map_img_ext', '.pgm')
    inner_safe_dist = config.get('inner_safe_dist', 0.3)
    outer_safe_dist = config.get('outer_safe_dist', 0.3)

    # Override with command line arguments
    if args.inner_safe is not None:
        inner_safe_dist = args.inner_safe
    if args.outer_safe is not None:
        outer_safe_dist = args.outer_safe

    # Determine map paths
    if args.map_path is not None:
        map_path = args.map_path
        yaml_path = args.yaml_path if args.yaml_path else os.path.splitext(map_path)[0] + '.yaml'
    else:
        # Use config and relative paths
        map_dir = os.path.join(script_dir, "..", "map")
        map_path = os.path.join(map_dir, map_name + map_img_ext)
        yaml_path = os.path.join(map_dir, map_name + ".yaml")

    # Check files
    if not os.path.exists(map_path):
        print(f"❌ Map file not found: {map_path}")
        return
    if not os.path.exists(yaml_path):
        print(f"❌ YAML file not found: {yaml_path}")
        return

    print(f"📂 Map: {map_path}")
    print(f"📂 YAML: {yaml_path}")
    print(f"📏 Safety distances: inner={inner_safe_dist}m, outer={outer_safe_dist}m")

    # Determine output directory with margin info
    if args.output is None:
        # Format: /home/ojg/RACE/bound/map_name/inner_margin_outer_margin/
        margin_str = f"{inner_safe_dist:.1f}_{outer_safe_dist:.1f}"
        bound_dir = os.path.join(script_dir, "..", "bound", map_name, margin_str)
        args.output = bound_dir

    print(f"💾 Output directory: {args.output}")

    # Extract boundaries from map
    try:
        outer_bound, inner_bound = extract_boundaries_from_map(
            map_path, yaml_path, inner_safe_dist, outer_safe_dist
        )
    except Exception as e:
        print(f"\n❌ Error extracting boundaries: {e}")
        import traceback
        traceback.print_exc()
        return

    # Start GUI
    display_name = f"{map_name} (inner={inner_safe_dist}m, outer={outer_safe_dist}m)"
    gui = BoundaryGeneratorGUI(outer_bound, inner_bound, args.output, display_name, args.tolerance)
    gui.run()


if __name__ == '__main__':
    main()
