/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire
solution is contained within the cw2_team_<your_team_number> package */

#include <cw2_class.h>
#include <utility>

cw2::cw2(const rclcpp::Node::SharedPtr &node)
: node_(node),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_),
  g_cloud_ptr(new PointC)
{
  t1_service_ = node_->create_service<cw2_world_spawner::srv::Task1Service>(
    "/task1_start",
    std::bind(&cw2::t1_callback, this, std::placeholders::_1, std::placeholders::_2));
  t2_service_ = node_->create_service<cw2_world_spawner::srv::Task2Service>(
    "/task2_start",
    std::bind(&cw2::t2_callback, this, std::placeholders::_1, std::placeholders::_2));
  t3_service_ = node_->create_service<cw2_world_spawner::srv::Task3Service>(
    "/task3_start",
    std::bind(&cw2::t3_callback, this, std::placeholders::_1, std::placeholders::_2));

  pointcloud_topic_ = node_->declare_parameter<std::string>(
    "pointcloud_topic", "/r200/camera/depth_registered/points");
  pointcloud_qos_reliable_ =
    node_->declare_parameter<bool>("pointcloud_qos_reliable", true);

  pointcloud_callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions pointcloud_sub_options;
  pointcloud_sub_options.callback_group = pointcloud_callback_group_;

  rclcpp::QoS pointcloud_qos = rclcpp::SensorDataQoS();
  if (pointcloud_qos_reliable_) {
    pointcloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  }

  color_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_,
    pointcloud_qos,
    std::bind(&cw2::cloud_callback, this, std::placeholders::_1),
    pointcloud_sub_options);

  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "panda_arm");
  hand_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "hand");

  RCLCPP_INFO(
    node_->get_logger(),
    "cw2_team_13 template initialised with pointcloud topic '%s' (%s QoS)",
    pointcloud_topic_.c_str(),
    pointcloud_qos_reliable_ ? "reliable" : "sensor-data");
}

void cw2::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*msg, pcl_cloud);

  PointCPtr latest_cloud(new PointC);
  pcl::fromPCLPointCloud2(pcl_cloud, *latest_cloud);

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  g_input_pc_frame_id_ = msg->header.frame_id;
  g_cloud_ptr = std::move(latest_cloud);
  ++g_cloud_sequence_;
}

/*
Helper Functions
*/

///////////////////////////////////////////////////////////////////////////////
// Motion Helpers
///////////////////////////////////////////////////////////////////////////////

void cw2::initMoveit()
{
  if (moveit_initialized_) return;
  
  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "panda_arm");
  hand_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "hand");
  arm_group_->setPlanningTime(10.0);
  arm_group_->setNumPlanningAttempts(5);

  arm_group_->setMaxVelocityScalingFactor(1.0);    // default is often 0.1
  arm_group_->setMaxAccelerationScalingFactor(1.0); // default is often 0.1
  
  moveit_initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "MoveIt initialized successfully");
}

bool cw2::moveToTarget(geometry_msgs::msg::Pose target)
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

bool cw2::openGripper()
{
  hand_group_->setJointValueTarget(std::vector<double>{0.04, 0.04});
  hand_group_->move();
  return true;
}

bool cw2::closeGripper()
{
  hand_group_->setJointValueTarget(std::vector<double>{0.005, 0.005});
  hand_group_->move();
  return true;
}

bool cw2::moveCartesian(geometry_msgs::msg::Pose target)
{
  // Get current end-effector pose as start
  geometry_msgs::msg::Pose start = arm_group_->getCurrentPose().pose;

  // Generate dense waypoints along the straight line
  std::vector<geometry_msgs::msg::Pose> waypoints;
  int num_points = 20;  // many intermediate points forces a straight line

  for (int i = 1; i <= num_points; i++)
  {
    double t = static_cast<double>(i) / static_cast<double>(num_points);
    geometry_msgs::msg::Pose interpolated;
    interpolated.position.x = start.position.x + t * (target.position.x - start.position.x);
    interpolated.position.y = start.position.y + t * (target.position.y - start.position.y);
    interpolated.position.z = start.position.z + t * (target.position.z - start.position.z);
    // Keep orientation constant throughout
    interpolated.orientation = start.orientation;
    waypoints.push_back(interpolated);
  }
  // Ensure final waypoint has exact target position
  waypoints.back().position = target.position;

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = arm_group_->computeCartesianPath(
    waypoints,
    // 0.001,   // very small eef_step for fine resolution
    0.01,    // eef_step of 1cm is usually sufficient for smooth paths
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

///////////////////////////////////////////////////////////////////////////////
// Point Cloud Helpers 
///////////////////////////////////////////////////////////////////////////////

void cw2::savePointCloudPCD(
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::capturePointCloudAtTarget(
  geometry_msgs::msg::Pose target)
{
  // moveCartesian(target);
  moveToTarget(target);
  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  // Grab cloud from the template's storage (g_cloud_ptr is PointXYZRGBA)
  PointCPtr cloud_rgba;
  std::string frame_id;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (!g_cloud_ptr || g_cloud_ptr->empty()) return nullptr;
    cloud_rgba = std::make_shared<PointC>(*g_cloud_ptr);
    frame_id = g_input_pc_frame_id_;
  }

  // Look up transform using the template's tf_buffer_ (value, not pointer)
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_.lookupTransform(
        "panda_link0", frame_id,
        tf2::TimePointZero, tf2::durationFromSec(2.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "TF lookup failed: %s", ex.what());
    return nullptr;
  }

  // Apply transform manually and convert RGBA -> RGB
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  auto &t = tf_stamped.transform.translation;
  auto &r = tf_stamped.transform.rotation;
  transform.translation() << t.x, t.y, t.z;
  Eigen::Quaternionf q(r.w, r.x, r.y, r.z);
  transform.rotate(q);

  auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl_cloud->reserve(cloud_rgba->size());

  for (const auto &pt : cloud_rgba->points) {
    if (!std::isfinite(pt.x)) continue;
    Eigen::Vector3f p = transform * Eigen::Vector3f(pt.x, pt.y, pt.z);
    pcl::PointXYZRGB rgb_pt;
    rgb_pt.x = p.x(); rgb_pt.y = p.y(); rgb_pt.z = p.z();
    rgb_pt.r = pt.r; rgb_pt.g = pt.g; rgb_pt.b = pt.b;
    pcl_cloud->push_back(rgb_pt);
  }

  RCLCPP_INFO(node_->get_logger(), "Captured %zu points in world frame", pcl_cloud->size());
  return pcl_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::combinePointClouds(
  const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds)
{
  auto combined = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (const auto &cloud : clouds) {
    if (cloud) *combined += *cloud;
  }
  RCLCPP_INFO(node_->get_logger(), "Combined cloud: %zu points", combined->size());
  return combined;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::scanTable()
{
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;

  /*
  
  Scanning pattern (8 views around the table, 45 degrees apart, at 0.7m height):
  
        Front
         ^
         |
   Back < > Front
         |
         v
        Back
  */
  const float displacement = 0.4; // how far left/right from the center

  auto pose = createPose(displacement, 0.0, 0.7);

  // Front middle 
  clouds.push_back(capturePointCloudAtTarget(pose));

  // Front right
  pose.position.y = displacement; 
  clouds.push_back(capturePointCloudAtTarget(pose));

  // Middle right
  pose.position.x = 0.0; 
  clouds.push_back(capturePointCloudAtTarget(pose));

  // Back right
  pose.position.x = -displacement;
  clouds.push_back(capturePointCloudAtTarget(pose));

  // Back middle
  pose.position.y = 0.0;
  clouds.push_back(capturePointCloudAtTarget(pose));

  // Back left
  pose.position.y = -displacement;
  clouds.push_back(capturePointCloudAtTarget(pose));

  // Middle left
  pose.position.x = 0.0;
  clouds.push_back(capturePointCloudAtTarget(pose));

  // Front left
  pose.position.x = displacement;
  clouds.push_back(capturePointCloudAtTarget(pose));

  return combinePointClouds(clouds);
}

// Filters Input Point Cloud to Remove Table 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::filterTableFromPointCloud(
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
  float z_threshold = table_z_mean + 0.04f;
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





///////////////////////////////////////////////////////////////////////////////
// Implementations
///////////////////////////////////////////////////////////////////////////////

// TODO - implement Task 1 later 
void cw2::t1_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task1Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task1Service::Response> response)
{
  // (void)request;
  // (void)response;

  // Step 1: Print out object position, goal position, shape type from request 

  const auto & object_point = request->object_point;
  const auto & goal_point = request->goal_point;
  const std::string & shape_type = request->shape_type;

  RCLCPP_INFO(
    node_->get_logger(),
    "Task 1 request received");

  RCLCPP_INFO(
    node_->get_logger(),
    "object_point: frame='%s', x=%.3f, y=%.3f, z=%.3f",
    object_point.header.frame_id.c_str(),
    object_point.point.x,
    object_point.point.y,
    object_point.point.z);

  RCLCPP_INFO(
    node_->get_logger(),
    "goal_point: frame='%s', x=%.3f, y=%.3f, z=%.3f",
    goal_point.header.frame_id.c_str(),
    goal_point.point.x,
    goal_point.point.y,
    goal_point.point.z);

  RCLCPP_INFO(
    node_->get_logger(),
    "shape_type: '%s'",
    shape_type.c_str());

  
  (void)response;

  std::string frame_id;
  std::size_t point_count = 0;
  std::uint64_t sequence = 0;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    frame_id = g_input_pc_frame_id_;
    point_count = g_cloud_ptr ? g_cloud_ptr->size() : 0;
    sequence = g_cloud_sequence_;
  }

  RCLCPP_WARN(
    node_->get_logger(),
    "Task 1 is not implemented in cw2_team_13. Latest cloud: seq=%llu frame='%s' points=%zu",
    static_cast<unsigned long long>(sequence),
    frame_id.c_str(),
    point_count);
}

// TODO - implement Task 2 later
void cw2::t2_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> response)
{
  (void)request;
  response->mystery_object_num = -1;

  std::string frame_id;
  std::size_t point_count = 0;
  std::uint64_t sequence = 0;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    frame_id = g_input_pc_frame_id_;
    point_count = g_cloud_ptr ? g_cloud_ptr->size() : 0;
    sequence = g_cloud_sequence_;
  }

  RCLCPP_WARN(
    node_->get_logger(),
    "Task 2 is not implemented in cw2_team_13. Latest cloud: seq=%llu frame='%s' points=%zu",
    static_cast<unsigned long long>(sequence),
    frame_id.c_str(),
    point_count);
}


void cw2::t3_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> response)
{
  initMoveit();

  // Scan the table
  auto combined = scanTable();
  savePointCloudPCD(combined, "/tmp/cw2_scan_combined.pcd");

  // Remove the green table
  auto filtered = filterTableFromPointCloud(combined);
  savePointCloudPCD(filtered, "/tmp/cw2_scan_filtered.pcd");

  RCLCPP_INFO(node_->get_logger(), "Saved combined (%zu pts) and filtered (%zu pts) clouds",
              combined ? combined->size() : 0,
              filtered ? filtered->size() : 0);

  // TODO: clustering and shape detection
  (void)request;
  response->total_num_shapes = 0;
  response->num_most_common_shape = 0;
  response->most_common_shape_vector.clear();
}