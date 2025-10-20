#!/usr/bin/env python3
"""
Generate goal line from two points with 0.05m spacing
Saves to /home/ojg/RACE/map/{map_name}.csv
"""

import numpy as np
import os
import yaml


def interpolate_line(p1, p2, spacing=0.05):
    """
    Interpolate points between p1 and p2 with given spacing

    Args:
        p1: First point [x, y]
        p2: Second point [x, y]
        spacing: Distance between interpolated points (m)

    Returns:
        Array of interpolated points
    """
    p1 = np.array(p1)
    p2 = np.array(p2)

    # Calculate distance between points
    distance = np.linalg.norm(p2 - p1)

    # Calculate number of points needed
    num_points = int(distance / spacing) + 1

    # Generate interpolated points
    points = []
    for i in range(num_points):
        t = i / (num_points - 1) if num_points > 1 else 0
        point = p1 + t * (p2 - p1)
        points.append(point)

    return np.array(points)


def save_goal_line(points, map_name, output_dir):
    """
    Save goal line points to CSV file

    Args:
        points: Array of [x, y] coordinates
        map_name: Name of the map
        output_dir: Output directory path
    """
    os.makedirs(output_dir, exist_ok=True)

    output_file = os.path.join(output_dir, f"{map_name}.csv")

    # Save to CSV
    np.savetxt(output_file, points, delimiter=',', fmt='%.6f',
               header='x_m,y_m', comments='')

    print(f"\n✓ Goal line saved to: {output_file}")
    print(f"  Number of points: {len(points)}")
    print(f"  First point: ({points[0, 0]:.3f}, {points[0, 1]:.3f})")
    print(f"  Last point:  ({points[-1, 0]:.3f}, {points[-1, 1]:.3f})")
    print(f"  Total length: {np.linalg.norm(points[-1] - points[0]):.3f} m")


def main():
    """Main function"""
    # Get trajectory_generator directory
    module = os.path.dirname(os.path.abspath(__file__))

    # Read map name from config
    config_file = os.path.join(module, "config", "params.yaml")
    with open(config_file, 'r') as stream:
        parsed_yaml = yaml.safe_load(stream)
    map_name = parsed_yaml["map_name"]

    print("="*60)
    print(f"Goal Line Generator - Map: {map_name}")
    print("="*60)

    # Get user input
    print("\nEnter two points to define the goal line:")
    print("(Points will be interpolated with 0.05m spacing)")

    try:
        # Point 1
        x1 = float(input("\nPoint 1 - X coordinate (m): "))
        y1 = float(input("Point 1 - Y coordinate (m): "))

        # Point 2
        x2 = float(input("\nPoint 2 - X coordinate (m): "))
        y2 = float(input("Point 2 - Y coordinate (m): "))

        p1 = [x1, y1]
        p2 = [x2, y2]

        print(f"\nGenerating goal line:")
        print(f"  From: ({x1:.3f}, {y1:.3f})")
        print(f"  To:   ({x2:.3f}, {y2:.3f})")

        # Interpolate points
        spacing = 0.05
        points = interpolate_line(p1, p2, spacing)

        # Output directory
        output_dir = os.path.join(module, "..", "map")

        # Save to CSV
        save_goal_line(points, map_name, output_dir)

        print("\n" + "="*60)
        print("✓ Goal line generated successfully!")
        print("="*60)

    except ValueError as e:
        print(f"\n✗ Error: Invalid input - {e}")
        print("Please enter numerical values for coordinates")
    except KeyboardInterrupt:
        print("\n\n✗ Cancelled by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")


if __name__ == "__main__":
    main()
