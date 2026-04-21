/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>

#include <functional>
#include <memory>
#include <utility>
#include <cmath>
#include <string>
#include <vector>
#include <numeric>
#include <mutex>
#include <algorithm>
#include <fstream>

#include <rmw/qos_profiles.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
///////////////////////////////////////////////////////////////////////////////


cw1::cw1(const rclcpp::Node::SharedPtr &node)
{
  /* class constructor */

  node_ = node;
  service_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sensor_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // advertise solutions for coursework tasks
  t1_service_ = node_->create_service<cw1_world_spawner::srv::Task1Service>(
    "/task1_start",
    std::bind(&cw1::t1_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);
  t2_service_ = node_->create_service<cw1_world_spawner::srv::Task2Service>(
    "/task2_start",
    std::bind(&cw1::t2_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);
  t3_service_ = node_->create_service<cw1_world_spawner::srv::Task3Service>(
    "/task3_start",
    std::bind(&cw1::t3_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  // Service and sensor callbacks use separate callback groups to align with the
  // current runtime architecture used in cw1_team_0.
  rclcpp::SubscriptionOptions joint_state_sub_options;
  joint_state_sub_options.callback_group = sensor_cb_group_;
  auto joint_state_qos = rclcpp::QoS(rclcpp::KeepLast(50));
  joint_state_qos.reliable();
  joint_state_qos.durability_volatile();
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", joint_state_qos,
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      const int64_t stamp_ns =
        static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(msg->header.stamp.nanosec);
      latest_joint_state_stamp_ns_.store(stamp_ns, std::memory_order_relaxed);
      joint_state_msg_count_.fetch_add(1, std::memory_order_relaxed);
    },
    joint_state_sub_options);

  // TF setup for point cloud transformations
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::SubscriptionOptions cloud_sub_options;
  cloud_sub_options.callback_group = sensor_cb_group_;
  auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cloud_qos.reliable();
  cloud_qos.durability_volatile();
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/r200/camera/depth_registered/points", cloud_qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      const int64_t stamp_ns =
        static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(msg->header.stamp.nanosec);

      latest_cloud_stamp_ns_.store(stamp_ns, std::memory_order_relaxed);
      cloud_msg_count_.fetch_add(1, std::memory_order_relaxed);

      {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
      }
    },
    cloud_sub_options);

  // Parameter declarations intentionally mirror cw1_team_0 for compatibility.
  const bool use_gazebo_gui = node_->declare_parameter<bool>("use_gazebo_gui", true);
  (void)use_gazebo_gui;
  enable_cloud_viewer_ = node_->declare_parameter<bool>("enable_cloud_viewer", false);
  move_home_on_start_ = node_->declare_parameter<bool>("move_home_on_start", false);
  use_path_constraints_ = node_->declare_parameter<bool>("use_path_constraints", false);
  use_cartesian_reach_ = node_->declare_parameter<bool>("use_cartesian_reach", false);
  allow_position_only_fallback_ = node_->declare_parameter<bool>(
    "allow_position_only_fallback", allow_position_only_fallback_);
  cartesian_eef_step_ = node_->declare_parameter<double>(
    "cartesian_eef_step", cartesian_eef_step_);
  cartesian_jump_threshold_ = node_->declare_parameter<double>(
    "cartesian_jump_threshold", cartesian_jump_threshold_);
  cartesian_min_fraction_ = node_->declare_parameter<double>(
    "cartesian_min_fraction", cartesian_min_fraction_);
  publish_programmatic_debug_ = node_->declare_parameter<bool>(
    "publish_programmatic_debug", publish_programmatic_debug_);
  enable_task1_snap_ = node_->declare_parameter<bool>("enable_task1_snap", false);
  return_home_between_pick_place_ = node_->declare_parameter<bool>(
    "return_home_between_pick_place", return_home_between_pick_place_);
  return_home_after_pick_place_ = node_->declare_parameter<bool>(
    "return_home_after_pick_place", return_home_after_pick_place_);
  pick_offset_z_ = node_->declare_parameter<double>("pick_offset_z", pick_offset_z_);
  task3_pick_offset_z_ = node_->declare_parameter<double>(
    "task3_pick_offset_z", task3_pick_offset_z_);
  task2_capture_enabled_ = node_->declare_parameter<bool>(
    "task2_capture_enabled", task2_capture_enabled_);
  place_offset_z_ = node_->declare_parameter<double>("place_offset_z", place_offset_z_);
  grasp_approach_offset_z_ = node_->declare_parameter<double>(
    "grasp_approach_offset_z", grasp_approach_offset_z_);
  post_grasp_lift_z_ = node_->declare_parameter<double>(
    "post_grasp_lift_z", post_grasp_lift_z_);
  gripper_grasp_width_ = node_->declare_parameter<double>(
    "gripper_grasp_width", gripper_grasp_width_);
  joint_state_wait_timeout_sec_ = node_->declare_parameter<double>(
    "joint_state_wait_timeout_sec", joint_state_wait_timeout_sec_);

  RCLCPP_INFO(node_->get_logger(), "cw1 template class initialised with compatibility scaffold");
}
///////////////////////////////////////////////////////////////////////////////
// Motion Helpers
///////////////////////////////////////////////////////////////////////////////

void cw1::initMoveit()
{
  if (moveit_initialized_) return;
  
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "panda_arm");
  hand_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "hand");
  arm_group_->setPlanningTime(10.0);
  arm_group_->setNumPlanningAttempts(5);
  
  moveit_initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "MoveIt initialized successfully");
}

bool cw1::moveToTarget(geometry_msgs::msg::Pose target)
{
  arm_group_->setPoseTarget(target);
  auto [success, plan] = [this]() {
    moveit::planning_interface::MoveGroupInterface::Plan p;
    bool ok = (arm_group_->plan(p) == moveit::core::MoveItErrorCode::SUCCESS);
    return std::make_pair(ok, p);
  }();
  if (success) {
    arm_group_->execute(plan);
    return true;
  }
  RCLCPP_WARN(node_->get_logger(), "Planning failed!");
  return false;
}

bool cw1::openGripper()
{
  hand_group_->setJointValueTarget(std::vector<double>{0.04, 0.04});
  hand_group_->move();
  return true;
}

bool cw1::closeGripper()
{
  hand_group_->setJointValueTarget(std::vector<double>{0.005, 0.005});
  hand_group_->move();
  return true;
}

bool cw1::moveCartesian(geometry_msgs::msg::Pose target)
{
  // Get current end-effector pose as start
  geometry_msgs::msg::Pose start = arm_group_->getCurrentPose().pose;

  // Generate dense waypoints along the straight line
  std::vector<geometry_msgs::msg::Pose> waypoints;
  int num_points = 50;  // many intermediate points forces a straight line
  // Convert start/target orientations to tf2 quaternions for slerp
  tf2::Quaternion q_start(
    start.orientation.x,
    start.orientation.y,
    start.orientation.z,
    start.orientation.w);

  tf2::Quaternion q_target(
    target.orientation.x,
    target.orientation.y,
    target.orientation.z,
    target.orientation.w);
    
  for (int i = 1; i <= num_points; i++)
  {
    double t = static_cast<double>(i) / static_cast<double>(num_points);
    geometry_msgs::msg::Pose interpolated;
    interpolated.position.x = start.position.x + t * (target.position.x - start.position.x);
    interpolated.position.y = start.position.y + t * (target.position.y - start.position.y);
    interpolated.position.z = start.position.z + t * (target.position.z - start.position.z);
    // Keep orientation constant throughout
    interpolated.orientation = start.orientation;

    // Slerp orientation from start to target
    tf2::Quaternion q_interp = q_start.slerp(q_target, t);
    q_interp.normalize();
    interpolated.orientation.x = q_interp.x();
    interpolated.orientation.y = q_interp.y();
    interpolated.orientation.z = q_interp.z();
    interpolated.orientation.w = q_interp.w();


    waypoints.push_back(interpolated);
  }
  // Ensure final waypoint has exact target position
  waypoints.back().position = target.position;

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = arm_group_->computeCartesianPath(
    waypoints,
    0.001,   // very small eef_step for fine resolution
    0.0,     // jump_threshold disabled
    trajectory
  );

  if (fraction > 0.95) {
    arm_group_->execute(trajectory);
    return true;
  }

  RCLCPP_WARN(node_->get_logger(),
    "Cartesian path only achieved %.1f%%, falling back to moveToTarget", fraction * 100.0);
  // Fallback to normal planning if Cartesian fails
  return moveToTarget(target);
}

geometry_msgs::msg::Pose createPose(
  float x = 0.0, 
  float y = 0.0, 
  float z = 0.0, 
  float qx= 0.9239,
  float qy= -0.3827, 
  float qz= 0.0, 
  float qw= 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;
  return pose;
}

void cw1::pickAndPlace(float box_x, float box_y, float box_z,
                       float basket_x, float basket_y, float basket_z)
{
  openGripper();

  geometry_msgs::msg::Pose target;
  target.orientation.x = 0.9239;
  target.orientation.y = -0.3827;
  target.orientation.z = 0.0;
  target.orientation.w = 0.0;

  // Move above box 
  target.position.x = box_x;
  target.position.y = box_y;
  target.position.z = box_z + 0.50;
  moveCartesian(target);

  // Descend to grasp 
  target.position.z = box_z + 0.115;  // slightly above the box for grasping
  moveCartesian(target);

  closeGripper();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Lift
  target.position.z = box_z + 0.50;
  moveCartesian(target);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Move above basket 
  target.position.x = basket_x;
  target.position.y = basket_y;
  target.position.z = basket_z + 0.45;
  moveCartesian(target);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Lower into basket
  target.position.z = box_z + 0.25;
  moveCartesian(target);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  openGripper();

  // immediately move up to avoid dragging box on the table if grasp failed
  target.position.z = box_z + 0.50;
  moveCartesian(target);
}

///////////////////////////////////////////////////////////////////////////////
// Point Cloud Helpers
///////////////////////////////////////////////////////////////////////////////


// Gets latest cloud message safely with mutex protection
sensor_msgs::msg::PointCloud2::SharedPtr cw1::getLatestCloud()
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  return latest_cloud_msg_;
}

void cw1::savePointCloudPCD(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
  const std::string &filename)
{
  if (!cloud || cloud->empty()) {
    RCLCPP_WARN(node_->get_logger(), "Empty cloud, skipping save");
    return;
  }
  pcl::io::savePCDFileBinary(filename, *cloud);
  RCLCPP_INFO(node_->get_logger(), "Saved %zu points to %s", cloud->size(), filename.c_str());
}

// Moves to target, captures the cloud in PCL format 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw1::capturePointCloudAtTarget(
  geometry_msgs::msg::Pose target)
{
  moveCartesian(target);
   // allow some time for the cloud to update after moving
  rclcpp::sleep_for(std::chrono::milliseconds(1000)); 

  auto cloud_msg = getLatestCloud();
  if (!cloud_msg) return nullptr;

  // Look up transform from camera frame to world frame
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
        "panda_link0",
        cloud_msg->header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(2.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "TF lookup failed: %s", ex.what());
    return nullptr;
  }

  // Transform the cloud to world frame
  sensor_msgs::msg::PointCloud2 cloud_world;
  tf2::doTransform(*cloud_msg, cloud_world, tf);

  // Convert to PCL
  auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::fromROSMsg(cloud_world, *pcl_cloud);
  return pcl_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw1::combinePointClouds(
  const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds)
{
  auto combined = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (const auto &cloud : clouds) {
    if (cloud) *combined += *cloud;
  }
  RCLCPP_INFO(node_->get_logger(), "Combined cloud: %zu points", combined->size());
  return combined;
}

// Scans the Table by Capturing 3 Clouds from Different Viewpoints and Combining Them
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw1::scanTable()
{
  auto scan_pose = createPose(0.45, 0.0, 0.7);

  auto center = capturePointCloudAtTarget(scan_pose);

  scan_pose.position.y = 0.3;
  auto left = capturePointCloudAtTarget(scan_pose);

  scan_pose.position.y = -0.3;
  auto right = capturePointCloudAtTarget(scan_pose);

  auto combined =  combinePointClouds({center, left, right});

  // save the combined cloud for debugging
  savePointCloudPCD(combined, "/tmp/scan_combined.pcd");
  return combined;
}

// Filters Input Point Cloud to Remove Table 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw1::filterTableFromPointCloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  if (!cloud || cloud->empty()) return nullptr;

  // First pass: find table surface Z from green points
  double green_z_sum = 0.0;
  int green_count = 0;

  for (const auto &pt : cloud->points) {
    if (!std::isfinite(pt.x)) continue;
    if (pt.g > 100 && pt.r < 100 && pt.b < 100) {
      green_z_sum += pt.z;
      green_count++;
    }
  }

  if (green_count == 0) {
    RCLCPP_WARN(node_->get_logger(), "No green table points found");
    return nullptr;
  }

  float table_z_mean = static_cast<float>(green_z_sum / green_count);
  // float z_threshold = table_z_mean + 0.04f;
  float z_threshold = table_z_mean + 0.02f;
  RCLCPP_INFO(node_->get_logger(), "Table Z: %.4f, threshold: %.4f, green pts: %d",
              table_z_mean, z_threshold, green_count);

  // Second pass: keep points above table and not green
  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  for (const auto &pt : cloud->points) {
    if (!std::isfinite(pt.x)) continue;
    if (pt.z <= z_threshold) continue;
    if (pt.g > 100 && pt.r < 100 && pt.b < 100) continue;
    filtered->push_back(pt);
  }

  RCLCPP_INFO(node_->get_logger(), "Z-filtered: %zu points", filtered->size());
  return filtered;
}

std::string cw1::classifyCentroid(
  float cx, float cy, float cz,
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  float radius_sq = 0.06f * 0.06f;
  int purple_count = 0, red_count = 0, blue_count = 0, coloured_count = 0;

  for (const auto &pt : cloud->points) {
    if (!std::isfinite(pt.x)) continue;

    float dx = pt.x - cx, dy = pt.y - cy, dz = pt.z - cz;
    if (dx * dx + dy * dy + dz * dz > radius_sq) continue;

    int r = pt.r, g = pt.g, b = pt.b;

    int maxc = std::max({r, g, b});
    int minc = std::min({r, g, b});
    if (maxc - minc < 40) continue;
    if (g > r + 20 && g > b + 20) continue;

    coloured_count++;

    if (r > 80 && b > 80 && g < r - 20 && g < b - 20) {
      purple_count++;
    } else if (r > g + 25 && r > b + 25) {
      red_count++;
    } else if (b > r + 25 && b > g + 25) {
      blue_count++;
    }
  }

  if (coloured_count < 10) return "none";

  if (purple_count > 10 && purple_count > red_count / 2 && purple_count > blue_count / 2)
    return "purple";
  if (red_count > blue_count && red_count > 10)
    return "red";
  if (blue_count > red_count && blue_count > 10)
    return "blue";

  return "none";
}

cw1::BoxesAndBaskets cw1::detectShapes(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &filtered_cloud,
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &full_cloud)
{
  cw1::BoxesAndBaskets result;
  if (!filtered_cloud || filtered_cloud->empty()) return result;

  // DBSCAN via EuclideanClusterExtraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(filtered_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.02);  // eps = 2cm
  ec.setMinClusterSize(20);
  ec.setMaxClusterSize(100000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(filtered_cloud);
  ec.extract(cluster_indices);

  RCLCPP_INFO(node_->get_logger(), "Found %zu clusters", cluster_indices.size());

  for (size_t i = 0; i < cluster_indices.size(); i++) {
    const auto &indices = cluster_indices[i];

    // Compute centroid and extent
    float x_min = 1e9, y_min = 1e9, z_min = 1e9;
    float x_max = -1e9, y_max = -1e9, z_max = -1e9;
    float x_sum = 0, y_sum = 0, z_sum = 0;

    for (int idx : indices.indices) {
      const auto &pt = filtered_cloud->points[idx];
      x_sum += pt.x; y_sum += pt.y; z_sum += pt.z;
      x_min = std::min(x_min, pt.x); x_max = std::max(x_max, pt.x);
      y_min = std::min(y_min, pt.y); y_max = std::max(y_max, pt.y);
      z_min = std::min(z_min, pt.z); z_max = std::max(z_max, pt.z);
    }

    int n = static_cast<int>(indices.indices.size());
    float cx = x_sum / n, cy = y_sum / n, cz = z_sum / n;
    float xy_extent = std::max(x_max - x_min, y_max - y_min);

    if (xy_extent < 0.02f) continue;  // noise

    std::string obj_type = (xy_extent > 0.06f) ? "basket" : "box";
    // std::string obj_type = (xy_extent > 0.09f) ? "basket" : "box";
    std::string color_name = classifyCentroid(cx, cy, cz, full_cloud);

    if (obj_type == "basket") {
          if (color_name == "none") continue;
          // Use bounding box center better for hollow baskets
          float bbox_cx = (x_min + x_max) / 2.0f;
          float bbox_cy = (y_min + y_max) / 2.0f;
          result.baskets.push_back({color_name, bbox_cx, bbox_cy, cz});
    } else {
      result.boxes.push_back({color_name, cx, cy, cz});
    }
  }

  return result;
}

cw1::BoxesAndBaskets cw1::vision_pipeline()
{
  auto combined = scanTable();
  auto filtered = filterTableFromPointCloud(combined);
  auto detected = detectShapes(filtered, combined);  

  // manually update the z coordinate of detected objects
  for (auto &box : detected.boxes) 
    box.z = 0.03f;  // known box height 
  for (auto &basket : detected.baskets)
    basket.z = 0.02f;  // known basket height 

  return detected;
}


///////////////////////////////////////////////////////////////////////////////
// Callbacks for Coursework Tasks
///////////////////////////////////////////////////////////////////////////////

void
cw1::t1_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task1Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task1Service::Response> response)
{
  initMoveit();

  /* function which should solve task 1 */
  auto object_pose = request->object_loc.pose;
  auto goal_point = request->goal_loc.point;

  float box_x, box_y, box_z;
  box_x = object_pose.position.x;
  box_y = object_pose.position.y;
  box_z = object_pose.position.z;

  float basket_x, basket_y, basket_z;
  basket_x = goal_point.x;
  basket_y = goal_point.y;
  basket_z = goal_point.z;

  pickAndPlace(box_x, box_y, box_z, basket_x, basket_y, basket_z);
  
  (void)response;

  RCLCPP_INFO_STREAM(
    node_->get_logger(),
    "Task 1 callback triggered joint_msgs=" <<
      joint_state_msg_count_.load(std::memory_order_relaxed) <<
      ", cloud_msgs=" << cloud_msg_count_.load(std::memory_order_relaxed));
}


///////////////////////////////////////////////////////////////////////////////
// Task 2 callback
///////////////////////////////////////////////////////////////////////////////
void cw1::t2_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task2Service::Response> response)
{
  initMoveit();

  auto basket_locs = request->basket_locs;

  for (size_t i = 0; i < basket_locs.size(); i++) {

    auto target = createPose(basket_locs[i].point.x, basket_locs[i].point.y,basket_locs[i].point.z + 0.4);
    auto cloud = capturePointCloudAtTarget(target);
    if (!cloud) {
      response->basket_colours.push_back("none");
      continue;
    }
    // Classify color from the captured cloud around the basket center
    std::string color = classifyCentroid(
        basket_locs[i].point.x, basket_locs[i].point.y, basket_locs[i].point.z,
        cloud);
    RCLCPP_INFO(node_->get_logger(), "Basket %zu: %s at (%.2f, %.2f, %.2f)",
                i, color.c_str(),
                basket_locs[i].point.x, basket_locs[i].point.y, basket_locs[i].point.z);
    response->basket_colours.push_back(color);
  }

  // print the detected colours for debugging
  for (size_t i = 0; i < response->basket_colours.size(); i++) {
    RCLCPP_INFO(node_->get_logger(), "DETECTED COLOUR FOR BASKET %zu: %s", i, response->basket_colours[i].c_str());
  }
}

///////////////////////////////////////////////////////////////////////////////
// Task 3 callback
///////////////////////////////////////////////////////////////////////////////

  void 
  cw1::t3_callback(
    const std::shared_ptr<cw1_world_spawner::srv::Task3Service::Request> request,
    std::shared_ptr<cw1_world_spawner::srv::Task3Service::Response> response)
  {
    initMoveit();

    // 1. Scan table and detect all objects
    auto detected = vision_pipeline();

    RCLCPP_INFO(node_->get_logger(), "Task 3: found %zu boxes and %zu baskets",
                detected.boxes.size(), detected.baskets.size());

    // 2. Build color -> basket lookup
    std::map<std::string, BoxesAndBaskets::ColoredLocation> basket_map;
    for (const auto &basket : detected.baskets) {
      basket_map[basket.color] = basket;
      RCLCPP_INFO(node_->get_logger(), "  Basket: %s at (%.3f, %.3f, %.3f)",
                  basket.color.c_str(), basket.x, basket.y, basket.z);
    }

    // 3. Pick each box and place in matching basket
    for (const auto &box : detected.boxes) {
      auto it = basket_map.find(box.color);
      if (it == basket_map.end()) {
        RCLCPP_WARN(node_->get_logger(), "No %s basket found for box at (%.3f, %.3f, %.3f)",
                    box.color.c_str(), box.x, box.y, box.z);
        continue;
      }

      const auto &basket = it->second;
      RCLCPP_INFO(node_->get_logger(), "Picking %s box (%.3f, %.3f) -> %s basket (%.3f, %.3f)",
                  box.color.c_str(), box.x, box.y,
                  basket.color.c_str(), basket.x, basket.y);

      pickAndPlace(box.x, box.y, box.z, basket.x, basket.y, basket.z);
    }

    // 4. Rescan to check for dropped or missed boxes
    RCLCPP_INFO(node_->get_logger(), "Rescanning table for remaining boxes...");
    auto recheck = vision_pipeline();

    if (!recheck.boxes.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Found %zu remaining boxes, reattempting",
                  recheck.boxes.size());

      for (const auto &box : recheck.boxes) {

        // make sure that the recheck box is not inside a basket from the initial detection
        bool already_placed = false;
        for (const auto &basket : detected.baskets) {
          float dx = box.x - basket.x;
          float dy = box.y - basket.y;
          if (std::sqrt(dx*dx + dy*dy) < 0.15f) {
            RCLCPP_INFO(node_->get_logger(), "Skipping %s box at (%.3f, %.3f) — inside basket",
                        box.color.c_str(), box.x, box.y);
            already_placed = true;
            break;
          }
        }
        if (already_placed) continue;

        // if the colour is none - skip it 
        if (box.color == "none") {
          RCLCPP_WARN(node_->get_logger(), "Skipping unclassified box at (%.3f, %.3f)",
                      box.x, box.y);
          continue;
        }
        auto it = basket_map.find(box.color);
        if (it == basket_map.end()) {
          RCLCPP_WARN(node_->get_logger(), "Still no %s basket for remaining box",
                      box.color.c_str());
          continue;
        }

        const auto &basket = it->second;
        RCLCPP_INFO(node_->get_logger(), "Retry: %s box (%.3f, %.3f) -> %s basket (%.3f, %.3f)",
                    box.color.c_str(), box.x, box.y,
                    basket.color.c_str(), basket.x, basket.y);

        pickAndPlace(box.x, box.y, box.z, basket.x, basket.y, basket.z);
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "All boxes cleared");
    }

    RCLCPP_INFO(node_->get_logger(), "Task 3 complete");

    (void)request;
    (void)response;
  }