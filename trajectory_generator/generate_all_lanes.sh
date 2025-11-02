#!/bin/bash

# Script to generate lanes for multiple v_max values (5 v_max values × 3 lanes each)
# Output: v_max_6.0, v_max_6.5, v_max_7.0, v_max_7.5, v_max_8.0 with left/optimal/right lanes

# ============================================
# CONFIGURATION
# ============================================
GGV_FILE="ggv_test.csv"  # Set GGV file to use here
# ============================================

echo "=========================================="
echo "Generating trajectories for multiple v_max values"
echo "GGV file = ${GGV_FILE}"
echo "=========================================="

# Array of v_max values
declare -a v_max_values=(6.0 6.5 7.0 7.5 8.0)

# Backup original racecar.ini
cp params/racecar.ini params/racecar.ini.backup

# Set GGV file in racecar.ini (once, at the beginning)
echo "Setting GGV file to ${GGV_FILE} in racecar.ini..."
sed -i "s/^ggv_file=.*/ggv_file=\"${GGV_FILE}\"/" params/racecar.ini

# Loop through each v_max value
for v_max in "${v_max_values[@]}"; do
    echo ""
    echo "=========================================="
    echo "Processing: v_max = ${v_max} m/s"
    echo "=========================================="

    # Update racecar.ini with current v_max
    sed -i "s/\"v_max\": [0-9.]*,/\"v_max\": ${v_max},/" params/racecar.ini

    # Run trajectory generation
    python3 main_globaltraj.py

    echo "✓ Completed: v_max = ${v_max} m/s"
done

echo ""
echo "=========================================="
echo "All trajectories generated successfully!"
echo "=========================================="
echo "Generated 15 lanes total:"
for v_max in "${v_max_values[@]}"; do
    echo "  v_max_${v_max}/"
    echo "    - lane_left.csv"
    echo "    - lane_optimal.csv"
    echo "    - lane_right.csv"
done

echo ""
echo "Configuration used:"
echo "  GGV file = ${GGV_FILE}"
echo "  v_max values = ${v_max_values[@]}"
echo ""
echo "=========================================="
echo "Starting 3D visualization..."
echo "=========================================="

# Run visualization (needs racecar.ini with correct settings)
python3 visualize_all_lanes.py

echo ""
echo "Restoring original racecar.ini..."
# Restore original racecar.ini after visualization
mv params/racecar.ini.backup params/racecar.ini

echo ""
echo "=========================================="
echo "Complete! All done."
echo "=========================================="
