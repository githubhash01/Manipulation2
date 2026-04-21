source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /task cw2_world_spawner/srv/TaskSetup "{task_index: 3}"
