# Team 13 — COMP0250 Coursework 2


### Task 2 — Shape Detection

Task 2 presents three shapes: two known references and one mystery shape. The
goal is to determine which reference the mystery shape matches. Our solution
moves the robot's camera above each shape, captures point cloud data, and
classifies each shape by analysing its geometry.

**Classification approach:** We exploit the key structural difference between
noughts and crosses. A cross has material at its centre, while a nought has a hollow centre. After isolating the shape's points, we measure the density of points near the centre relative to the overall shape. A high centre-density ratio indicates a cross, a near-zero ratio indicates a nought.

**Ground removal:** Rather than filtering by height, we filter by colour. The ground
tiles are green, so we remove green-dominant points from the point cloud,
leaving only the shape.

**Pipeline for each shape:**
1. Move the arm above the shape's known position (from the service request)
2. Capture a fresh point cloud and transform it into the `panda_link0` frame
3. Crop the cloud in XY around the shape's position
4. Remove green ground-tile points using colour filtering
5. Classify by comparing inner vs outer point density

**Robustness:** The classification is rotation-invariant (a rotated cross
still has a filled centre) and height-invariant (colour filtering does not
depend on Z), so it handles the `T2_ANY_ORIENTATION` and
`T2_GROUND_PLANE_NOISE` variants.


## Key Source Files

- **`src/cw2_class.cpp`** — Contains all task logic:
  - `t2_callback()` — orchestrates the Task 2 pipeline
  - `move_arm_above()` — positions the camera above a target point
  - `get_transformed_cloud()` — captures and transforms point clouds
  - `crop_cloud_xy()` — spatially crops a cloud around a point
  - `filter_out_ground_color()` — removes green ground-tile points
  - `classify_shape()` — determines cross vs nought by centre density

- **`include/cw2_class.h`** — Declares the `cw2` class with all members

- **`src/cw2_node.cpp`** — Initialises the ROS 2 node with a multi-threaded
  executor to allow concurrent point cloud updates during service callbacks


## Building and Running

```bash
# From the workspace root
cd comp0250_s26_labs
colcon build
source install/setup.bash

# Launch the full system 
ros2 launch cw2_team_13 run_solution.launch.py \
  use_gazebo_gui:=true use_rviz:=false

# In a separate terminal, trigger Task 2
cd comp0250_s26_labs
source install/setup.bash
ros2 service call /task cw2_world_spawner/srv/TaskSetup "{task_index: 2}"
```