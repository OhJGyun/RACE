# Map, Global Path, and Bound Files

This directory contains all map-related files for F1TENTH simulation in `sim_ws`.

## Directory Structure

```
map_global_path_bound/
├── maps/           # Map files (.pgm, .png, .yaml) and pgm_to_bounds.py script
├── global_paths/   # Raceline/waypoint CSV files (x, y, speed)
└── bounds/         # Track bound files (inner/outer bound world coordinates)
```

## Usage

### 1. Maps
- Map image files (.pgm or .png) and corresponding .yaml files
- Use with `map_server` for localization
- `pgm_to_bounds.py`: Script to generate bound files from map images

### 2. Global Paths (Racelines)
- CSV format: `x, y, speed`
- Used by MAP controller and other path-following algorithms
- Generated from racing line optimization or centerline extraction

### 3. Bounds
- `outer_bound_world.csv`: Outer track boundary in world coordinates
- `inner_bound_world.csv`: Inner track boundary in world coordinates
- Used by `bound_obstacle_detector` for real-time obstacle detection
- Generate new bounds using `../maps/pgm_to_bounds.py`

## Example Files

Maps:
- `0927_1.pgm/yaml` - Custom track
- `Spielberg_map.png/yaml` - Spielberg circuit
- `thursday_1.pgm/yaml` - Thursday track

Global Paths:
- `0927_speed_4.csv` - Raceline for 0927 track at 4 m/s
- `jg_thursday.csv` - Thursday track raceline

Bounds:
- `outer_bound_world.csv` - Corresponding to map
- `inner_bound_world.csv` - Corresponding to map

## Integration with Packages

### MAP Controller
```bash
ros2 launch map_controller map_controller.launch.py \
  csv_file_path:=/home/joon/f1tenth_ws/sim_ws/map_global_path_bound/global_paths/0927_speed_4.csv
```

### Bound Obstacle Detector
```bash
ros2 launch bound_obstacle_detector bound_detector.launch.py \
  outer_csv:=/home/joon/f1tenth_ws/sim_ws/map_global_path_bound/bounds/outer_bound_world.csv \
  inner_csvs:=/home/joon/f1tenth_ws/sim_ws/map_global_path_bound/bounds/inner_bound_world.csv
```

### Combined (MAP + Bounds)
```bash
ros2 launch map_controller map_with_bound.launch.py \
  csv_file_path:=/home/joon/f1tenth_ws/sim_ws/map_global_path_bound/global_paths/0927_speed_4.csv \
  outer_csv:=/home/joon/f1tenth_ws/sim_ws/map_global_path_bound/bounds/outer_bound_world.csv \
  inner_csvs:=/home/joon/f1tenth_ws/sim_ws/map_global_path_bound/bounds/inner_bound_world.csv
```
