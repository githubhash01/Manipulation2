source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cw2_team_13 run_solution.launch.py \
use_gazebo_gui:=true use_rviz:=true
