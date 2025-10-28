#!/bin/bash

# Script to generate 9 lanes (3 GGV profiles × 3 lanes each)
# Output: conservative, normal, aggressive with left/optimal/right lanes

# ============================================
# CONFIGURATION
# ============================================
V_MAX=6.0  # Set maximum velocity here (m/s)
# ============================================

echo "=========================================="
echo "Generating trajectories for all GGV profiles"
echo "v_max = ${V_MAX} m/s"
echo "=========================================="

# Array of GGV files
declare -a ggv_files=("ggv_conservative.csv" "ggv_normal.csv" "ggv_aggressive.csv")
declare -a ggv_names=("conservative" "normal" "aggressive")

# Backup original racecar.ini
cp params/racecar.ini params/racecar.ini.backup

# Set v_max in racecar.ini
echo "Setting v_max to ${V_MAX} m/s in racecar.ini..."
sed -i "s/\"v_max\": [0-9.]*,/\"v_max\": ${V_MAX},/" params/racecar.ini

# Loop through each GGV file
for i in "${!ggv_files[@]}"; do
    ggv_file="${ggv_files[$i]}"
    ggv_name="${ggv_names[$i]}"

    echo ""
    echo "=========================================="
    echo "Processing: $ggv_name (${ggv_file})"
    echo "=========================================="

    # Update racecar.ini with current GGV file
    sed -i "s/^ggv_file=.*/ggv_file=\"${ggv_file}\"/" params/racecar.ini

    # Run trajectory generation
    python3 main_globaltraj.py

    echo "✓ Completed: $ggv_name"
done

echo ""
echo "=========================================="
echo "All trajectories generated successfully!"
echo "=========================================="
echo "Generated 9 lanes total:"
for name in "${ggv_names[@]}"; do
    echo "  ${name}_${V_MAX}/"
    echo "    - lane_left.csv"
    echo "    - lane_optimal.csv"
    echo "    - lane_right.csv"
done

echo ""
echo "Configuration used:"
echo "  v_max = ${V_MAX} m/s"
echo ""
echo "=========================================="
echo "Starting 3D visualization..."
echo "=========================================="

# Run visualization (needs racecar.ini with correct v_max)
python3 visualize_all_lanes.py

echo ""
echo "Restoring original racecar.ini..."
# Restore original racecar.ini after visualization
mv params/racecar.ini.backup params/racecar.ini

echo ""
echo "=========================================="
echo "Complete! All done."
echo "=========================================="
