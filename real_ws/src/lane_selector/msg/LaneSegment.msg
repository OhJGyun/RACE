# LaneSegment.msg
# Local planner segment information for lane_selector
# Publishes the current lane ID and the index range for local planning

int32 lane_id      # Current selected lane number
int32 start_idx    # Start index of the local segment
int32 end_idx      # End index of the local segment (exclusive)
