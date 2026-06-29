# 1. Build the workspace packages
colcon build

# 2. Source the newly built workspace overlay
source install/setup.bash

# 3. Launch the navigation brain node
ros2 run decision nav_brain