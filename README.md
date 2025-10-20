# RACE - F1TENTH Autonomous Racing System

An integrated system for F1TENTH autonomous racing, including optimal trajectory generation, simulation, and real vehicle control.

## 📋 Table of Contents

- [Project Structure](#project-structure)
- [Environment Setup](#environment-setup)
- [1. Path Generation](#1-path-generation)
- [2. Simulation](#2-simulation)
- [3. Real Vehicle](#3-real-vehicle)
- [Troubleshooting](#troubleshooting)

---

## Project Structure

```
RACE/
├── map/                        # Map files
│   ├── {map_name}.yaml         # Map metadata
│   └── {map_name}.pgm          # Map image
│
├── path/                       # Generated trajectories (shared)
│   ├── {map_name}/             # Standard trajectories
│   │   ├── conservative_9.0/
│   │   │   ├── lane_left.csv
│   │   │   ├── lane_optimal.csv
│   │   │   └── lane_right.csv
│   │   ├── normal_9.0/
│   │   └── aggressive_9.0/
│   └── {map_name}_for_mpcc/    # MPCC trajectories
│       ├── centerline_waypoints.csv
│       ├── center_spline_derivatives.csv
│       ├── left_waypoints.csv
│       ├── right_waypoints.csv
│       └── track_widths.csv
│
├── trajectory_generator/       # Trajectory generation tools
│   ├── config/
│   │   ├── params.yaml         # Standard trajectory config
│   │   └── params_for_mpcc.yaml # MPCC trajectory config
│   ├── params/
│   │   ├── racecar.ini         # Vehicle parameters (standard)
│   │   └── racecar_for_mpcc.ini # Vehicle parameters (MPCC)
│   ├── inputs/
│   │   └── veh_dyn_info/
│   │       ├── ggv_conservative.csv
│   │       ├── ggv_normal.csv
│   │       └── ggv_aggressive.csv
│   ├── lane_generator.py
│   ├── main_globaltraj.py
│   ├── lane_generator_for_mpcc.py
│   ├── main_globaltraj_for_mpcc.py
│   ├── generate_all_lanes.sh
│   └── visualize_all_lanes.py
│
├── sim_ws/                     # ROS2 simulation workspace
│   └── src/
│       ├── f1tenth_gym_ros/    # F1TENTH Gym simulator
│       └── mpcc/               # MPCC controller
│
└── real_ws/                    # ROS2 real vehicle workspace (TBD)
```

---

## Environment Setup

### 1. Conda Environment

```bash
# Clone penn environment (for trajectory_generator)
cd RACE/trajectory_generator
conda env create -f penn_environment.yml

# Or use minimal version
conda env create -f penn_environment_minimal.yml

# Activate environment
conda activate penn
```

### 2. ROS2 Installation (Ubuntu 22.04)

```bash
# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install dependencies
sudo apt install python3-colcon-common-extensions
```

---

## 1. Path Generation

### 1.1 Configuration

#### Map Configuration

`trajectory_generator/config/params.yaml`:
```yaml
map_name: '1017'              # Map name
map_img_ext: '.pgm'           # Map image extension
num_lanes: 3                  # Number of lanes to generate (left, optimal, right)
clockwise: False              # Driving direction
inner_safe_dist: 0.2          # Inner safety distance (m)
outer_safe_dist: 0.2          # Outer safety distance (m)
```

#### Vehicle Parameters

`trajectory_generator/params/racecar.ini`:
```ini
ggv_file = "ggv_aggressive.csv"  # GGV profile
"v_max": 9.0,                    # Maximum velocity (m/s)
"mass": 3.362,                   # Vehicle mass (kg)
"width": 0.3,                    # Vehicle width (m)
```

### 1.2 Standard Trajectory Generation

**Batch generation of 9 lanes** (3 GGV × 3 lanes):

```bash
cd trajectory_generator

# Set V_MAX and run
./generate_all_lanes.sh
```

This script will:
1. Generate trajectories for 3 GGV profiles (conservative, normal, aggressive)
2. Create 3 lanes each (left, optimal, right)
3. Automatically run 3D visualization
4. Output: `path/{map_name}/{ggv_profile}_{v_max}/lane_*.csv`

**Manual generation**:

```bash
# 1. Generate lanes (extract centerline from map)
python3 lane_generator.py

# 2. Generate optimal trajectory
python3 main_globaltraj.py

# 3. Visualize
python3 visualize_all_lanes.py
```

### 1.3 MPCC Trajectory Generation

```bash
cd trajectory_generator

# 1. Check configuration
# - Verify map_name in config/params_for_mpcc.yaml
# - Check vehicle parameters in params/racecar_for_mpcc.ini

# 2. Generate lanes
python3 lane_generator_for_mpcc.py

# 3. Generate MPCC format
python3 main_globaltraj_for_mpcc.py
```

**Output files** (`path/{map_name}_for_mpcc/`):
- `centerline_waypoints.csv` - Centerline coordinates (x, y)
- `center_spline_derivatives.csv` - Centerline derivatives (dx/ds, dy/ds)
- `left_waypoints.csv` - Left boundary (x, y)
- `right_waypoints.csv` - Right boundary (x, y)
- `track_widths.csv` - Track widths (left_width, right_width)

### 1.4 GGV Profiles

Three driving styles:

| Profile | Characteristics | Use Case |
|---------|----------------|----------|
| `conservative` | Low acceleration, stable | Safety priority, early learning |
| `normal` | Balanced performance | General racing |
| `aggressive` | High acceleration, aggressive | Achieving best lap times |

**Modify GGV**: `inputs/veh_dyn_info/ggv_*.csv`
```csv
# v_mps, ax_max_mps2, ay_max_mps2
0.0, 15.0, 16.0
5.0, 12.4, 13.2
9.0, 6.5, 3.8
10.0, 5.5, 3.8
```

---

## 2. Simulation

### 2.1 Build

```bash
cd sim_ws

# Build packages
colcon build --packages-select f1tenth_gym_ros mpcc

# Source workspace
source install/setup.bash
```

### 2.2 MPCC Controller Configuration

#### Map Configuration

`sim_ws/src/mpcc/params/mpcc_params.yaml`:
```yaml
path_folder_name: 1017_for_mpcc  # Map name (in path/ directory)
```

**Important**: `path_folder_name` must match the `{map_name}_for_mpcc` folder in the `path/` directory.

#### Vehicle Parameters

```yaml
vehicle_L: 0.33020       # Wheelbase (m)
max_speed: 9.0           # Maximum speed (m/s)
mpc_ref_vel: 5.5         # Reference velocity (m/s)
car_width: 0.31          # Vehicle width (m)
```

#### Control Parameters

```yaml
mpc_steps_N: 20          # Prediction horizon
dT: 0.05                 # Time step (s)
controller_freq: 15.0    # Control frequency (Hz)

# Weights
mpc_w_cte: 0.7          # Cross-track error
mpc_w_lag: 1.0          # Lag error
mpc_w_vel: 1.0          # Velocity
mpc_w_delta: 1.0        # Steering
```

### 2.3 Execution

#### Launch F1TENTH Gym Simulator

```bash
# Terminal 1: Simulator
source install/setup.bash
ros2 run f1tenth_gym_ros gym_bridge
```

#### Launch MPCC Controller

```bash
# Terminal 2: MPCC Controller
source install/setup.bash
ros2 run mpcc mpcc_node.py --ros-args --params-file src/mpcc/params/mpcc_params.yaml
```

### 2.4 Topics

**Subscribed**:
- `/ego_racecar/odom` - Vehicle position/velocity
- `/goal_pose` - Goal position

**Published**:
- `/drive` - Control commands (steering, speed)

---

## 3. Real Vehicle

### 3.1 Hardware Requirements

TBD

### 3.2 Configuration

TBD

### 3.3 Execution

TBD

---

## Troubleshooting

### Path Generation

**Q: `ModuleNotFoundError: No module named 'trajectory_planning_helpers'`**
```bash
conda activate penn
pip install trajectory-planning-helpers
```

**Q: Visualization is empty or map is not visible**
- Check `map_name` in `config/params.yaml`
- Verify map files exist in `map/` directory
- Confirm `trajectory_generator/outputs/{map_name}/centerline.csv` is generated

**Q: How to change `v_max`?**
```bash
# Edit generate_all_lanes.sh
V_MAX=7.0  # Change to desired value
```

### Simulation

**Q: `Package 'mpcc' not found`**
```bash
cd sim_ws
colcon build --packages-select mpcc
source install/setup.bash
```

**Q: `FileNotFoundError: centerline_waypoints.csv`**
1. Generate MPCC trajectory first: `python3 main_globaltraj_for_mpcc.py`
2. Verify `path_folder_name` in `mpcc_params.yaml` matches directory name in `path/`
3. Check path exists: `path/{path_folder_name}/centerline_waypoints.csv`

**Q: Build error in conda environment**
```bash
# Deactivate conda and build
conda deactivate
cd sim_ws
colcon build
```

---

## References

### Related Libraries
- [Trajectory Planning Helpers](https://github.com/TUMFTM/trajectory_planning_helpers)
- [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym)

### Papers
- TBD

---

## License

TBD

---

## Contributing

TBD

---

## Contact

TBD
