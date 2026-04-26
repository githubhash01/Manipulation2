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
  g_cloud_msg_ = msg;                  
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

void cw2::addSafetyFloor(double height)
{
  moveit_msgs::msg::CollisionObject floor;
  floor.header.frame_id = arm_group_->getPlanningFrame();
  floor.id = "safety_floor";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  // Wide thin slab. 3 m x 3 m covers the whole reachable workspace;
  // 2 mm thick is enough to be a hard barrier without eating into
  // the usable z-range above it.
  box.dimensions[0] = 3.0;
  box.dimensions[1] = 3.0;
  box.dimensions[2] = 0.002;

  geometry_msgs::msg::Pose pose;
  pose.position.z = height;
  pose.orientation.w = 1.0;

  floor.primitives.push_back(box);
  floor.primitive_poses.push_back(pose);
  floor.operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_interface_.applyCollisionObjects({floor});
}

void cw2::removeSafetyFloor()
{
  planning_scene_interface_.removeCollisionObjects({"safety_floor"});
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

// bool cw2::openGripper()
// {
//   hand_group_->setJointValueTarget(std::vector<double>{0.04, 0.04});
//   hand_group_->move();
//   return true;
// }

// bool cw2::closeGripper()
// {
//   hand_group_->setJointValueTarget(std::vector<double>{0.005, 0.005});
//   hand_group_->move();
//   return true;
// }

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

geometry_msgs::msg::PointStamped createPointStamped(
  float x, float y, float z, const std::string &frame_id)
{
  geometry_msgs::msg::PointStamped ps;
  ps.header.frame_id = frame_id;
  ps.point.x = x;
  ps.point.y = y;
  ps.point.z = z;
  return ps;
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

// Helper to save OpenCV image (BGR) to disk
void cw2::saveImage(
    const cv::Mat& image,
    const std::string& filename)
{
    if (image.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Empty image, skipping save");
        return;
    }

    if (!cv::imwrite(filename, image)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to save image to %s", filename.c_str());
        return;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "Saved image %dx%d to %s",
        image.cols,
        image.rows,
        filename.c_str());
}


// Capture a Point Cloud at a Given Pose 
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::capturePointCloudAtTarget(
  geometry_msgs::msg::Pose target)
{
  // moveToTarget(target);
  moveCartesian(target);
   // allow some time for the cloud to update after moving
  rclcpp::sleep_for(std::chrono::milliseconds(4000)); 

  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud_msg = g_cloud_msg_;
  }
  if (!cloud_msg) return nullptr;

  // Look up transform from camera frame to world frame
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(
        "world",
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

// Combines Multiple Point Cloud Scans into a Single Cloud
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

// Scans Table with 8 Viewpoints 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::scanTable()
{
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds; 
  const double z = 0.75;
  const std::vector<std::pair<double, double>> scan_grid = {
    { 0.4,  0.4}, { 0.4,  0.0}, { 0.4, -0.4},   
    { 0.0,  0.5},               { 0.0, -0.5},   
    {-0.4,  0.4}, {-0.4,  0.0}, {-0.4, -0.4},   
  };

  int idx = 0;
  for (const auto &[x, y] : scan_grid) {
    RCLCPP_INFO(node_->get_logger(),
      "Scanning cell %d/%zu: [x=%.2f, y=%.2f]",
      idx + 1, scan_grid.size(), x, y);

    auto pose = createPose(x, y, z, 0.9239, -0.3827, 0.0, 0.0);
    auto cloud = capturePointCloudAtTarget(pose);

    if (!cloud || cloud->empty()) {
      RCLCPP_WARN(node_->get_logger(),
        "Cell at [%.2f, %.2f] unreachable or empty. Moving to next.", x, y);
    } else {
      RCLCPP_INFO(node_->get_logger(),
        "Cell %d (x=%.2f, y=%.2f): %zu points", idx, x, y, cloud->size());
      savePointCloudPCD(cloud, "/tmp/cw2_scan_" + std::to_string(idx) + ".pcd");
      clouds.push_back(cloud);
    }
    ++idx;
  }

  if (clouds.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No viewpoints succeeded");
    return nullptr;
  }

  return combinePointClouds(clouds);
}

// Filters Input Point Cloud to Remove Table and Robot
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::filterTableFromPointCloud(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  if (!cloud || cloud->empty()) return nullptr;

  // --- Pass 1: estimate table Z from green points (unchanged) ---
  double green_z_sum = 0.0;
  int    green_count = 0;
  for (const auto &pt : cloud->points) {
    if (!std::isfinite(pt.x)) continue;
    if (pt.g > 100 && pt.r < 100 && pt.b < 100) {
      green_z_sum += pt.z;
      ++green_count;
    }
  }
  if (green_count == 0) {
    RCLCPP_WARN(node_->get_logger(), "No green table points found");
    return nullptr;
  }
  const float table_z_mean = static_cast<float>(green_z_sum / green_count);
  const float z_threshold  = table_z_mean + 0.04f;

  // --- Robot filter parameters ---
  // Anything within this XY radius of panda_link0 origin is the robot.
  // Chosen to swallow the Franka base (~0.2m) plus margin, while staying
  // well clear of the nearest tile where an object could sit.
  const float robot_xy_radius = 0.25f;
  const float robot_xy_radius_sq = robot_xy_radius * robot_xy_radius;

  // Permissive "whitish/grey" colour check for limbs poking outside the cylinder.
  // Franka links render around (200, 200, 200); gripper fingers slightly darker.
  // We require all three channels high AND roughly equal (low saturation).
  auto looks_white = [](const pcl::PointXYZRGB &pt) {
    const int minC = std::min({pt.r, pt.g, pt.b});
    const int maxC = std::max({pt.r, pt.g, pt.b});
    return minC > 150 && (maxC - minC) < 30;  // bright and nearly achromatic
  };

  // --- Pass 2: keep points above the table, not green, not in robot cylinder,
  //            and (as a fallback) not white/grey. ---
  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  filtered->reserve(cloud->size());

  int dropped_cylinder = 0, dropped_white = 0, dropped_floor = 0, dropped_green = 0;
  for (const auto &pt : cloud->points) {
    if (!std::isfinite(pt.x)) continue;

    if (pt.z <= z_threshold)                { ++dropped_floor;    continue; }

    const float r2 = pt.x * pt.x + pt.y * pt.y;
    if (r2 < robot_xy_radius_sq)            { ++dropped_cylinder; continue; }

    if (pt.g > 100 && pt.r < 100 && pt.b < 100) { ++dropped_green; continue; }
    if (looks_white(pt))                    { ++dropped_white;    continue; }

    filtered->push_back(pt);
  }

  RCLCPP_INFO(node_->get_logger(),
    "Filter: table_z=%.3f  kept %zu  | dropped floor=%d cyl=%d green=%d white=%d",
    table_z_mean, filtered->size(),
    dropped_floor, dropped_cylinder, dropped_green, dropped_white);

  return filtered;
}

// Converts PCD to top-down RGB image and height map
cw2::TopDownResult cw2::pcdToTopdownImageWithHeight(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    int gridSize)
{
    if (!cloud || cloud->empty())
    {
        throw std::runtime_error("Input cloud is null or empty.");
    }

    if (gridSize <= 0)
    {
        throw std::runtime_error("gridSize must be > 0.");
    }

    float minX = std::numeric_limits<float>::infinity();
    float minY = std::numeric_limits<float>::infinity();
    float maxX = -std::numeric_limits<float>::infinity();
    float maxY = -std::numeric_limits<float>::infinity();

    // Find XY bounds
    for (const auto& pt : cloud->points)
    {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;

        minX = std::min(minX, pt.x);
        minY = std::min(minY, pt.y);
        maxX = std::max(maxX, pt.x);
        maxY = std::max(maxY, pt.y);
    }

    if (!std::isfinite(minX) || !std::isfinite(minY) ||
        !std::isfinite(maxX) || !std::isfinite(maxY))
    {
        throw std::runtime_error("No valid finite points found in cloud.");
    }

    float scaleX = maxX - minX;
    float scaleY = maxY - minY;

    if (scaleX == 0.0f) scaleX = 1e-6f;
    if (scaleY == 0.0f) scaleY = 1e-6f;

    // OpenCV uses BGR by default
    cv::Mat image(gridSize, gridSize, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat heightMap(
        gridSize,
        gridSize,
        CV_32FC1,
        cv::Scalar(-std::numeric_limits<float>::infinity()));

    for (const auto& pt : cloud->points)
    {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;

        float normX = (pt.x - minX) / scaleX;
        float normY = (pt.y - minY) / scaleY;

        int px = static_cast<int>(normX * (gridSize - 1));
        int py = static_cast<int>(normY * (gridSize - 1));

        // Clamp just in case of floating-point edge cases
        px = std::max(0, std::min(px, gridSize - 1));
        py = std::max(0, std::min(py, gridSize - 1));

        float z = pt.z;
        float& currentHeight = heightMap.at<float>(py, px);

        if (z > currentHeight)
        {
            currentHeight = z;

            // PCL stores color as RGB, OpenCV expects BGR
            image.at<cv::Vec3b>(py, px) = cv::Vec3b(pt.b, pt.g, pt.r);
        }
    }

    return {image, heightMap, minX, minY, maxX, maxY, gridSize};
}

// Convert a (col, row) pixel coordinate back into world (x, y) in panda_link0 frame.
cv::Point2f pixelToWorld(const cv::Point2f &pixel, const cw2::TopDownResult &td)
{
  const float nx = pixel.x / static_cast<float>(td.gridSize - 1);
  const float ny = pixel.y / static_cast<float>(td.gridSize - 1);
  const float wx = td.minX + nx * (td.maxX - td.minX);
  const float wy = td.minY + ny * (td.maxY - td.minY);
  return cv::Point2f(wx, wy);
}

cv::Mat detectBasketRegion(const cv::Mat& imgFull, const cv::Mat& heightMap)
{
    if (imgFull.empty() || imgFull.type() != CV_8UC3)
        throw std::runtime_error("Input image must be CV_8UC3");
    if (heightMap.empty() || heightMap.type() != CV_32FC1)
        throw std::runtime_error("Input height map must be CV_32FC1");
    if (imgFull.size() != heightMap.size())
        throw std::runtime_error("Image and height map must have same size");

    // Convert BGR -> HSV
    cv::Mat hsv;
    cv::cvtColor(imgFull, hsv, cv::COLOR_BGR2HSV);

    // Split channels: H(0-180), S(0-255), V(0-255)
    std::vector<cv::Mat> ch;
    cv::split(hsv, ch);
    cv::Mat H = ch[0], S = ch[1], V = ch[2];

    // Basket = dark red (low V distinguishes it from bright red rings/crosses)
    //   reddish hue:   H < 15 OR H > 165  (red wraps around)
    //   saturated:     S > 80
    //   dark:          V < 160
    cv::Mat hueLow, hueHigh, reddish;
    cv::threshold(H, hueLow,  14, 255, cv::THRESH_BINARY_INV);   // H < 15
    cv::threshold(H, hueHigh, 165, 255, cv::THRESH_BINARY);      // H > 165
    reddish = hueLow | hueHigh;

    cv::Mat satMask, darkMask;
    cv::threshold(S, satMask,  80, 255, cv::THRESH_BINARY);       // S > 80
    cv::threshold(V, darkMask, 160, 255, cv::THRESH_BINARY_INV);  // V < 160

    cv::Mat basketMask = reddish & satMask & darkMask;

    // Morphology cleanup: open to kill specks, close to fill the basket
    cv::morphologyEx(basketMask, basketMask, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::morphologyEx(basketMask, basketMask, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    return basketMask;
}

void filterRobot(cv::Mat& img, cv::Mat& heightMap, int radius = 40)
{
    if (img.empty() || heightMap.empty())
        return;

    int h = img.rows;
    int w = img.cols;

    int cx = w / 2;
    int cy = h / 2;

    int x0 = std::max(0, cx - radius);
    int x1 = std::min(w, cx + radius);
    int y0 = std::max(0, cy - radius);
    int y1 = std::min(h, cy + radius);

    // Define ROI
    cv::Rect roi(x0, y0, x1 - x0, y1 - y0);

    // Set image to white (BGR = 255,255,255)
    img(roi).setTo(cv::Scalar(255, 255, 255));

    // Set height map to -inf
    heightMap(roi).setTo(-std::numeric_limits<float>::infinity());
}

void makeWhiteBackground(cv::Mat& img)
{
    std::vector<cv::Mat> channels;
    cv::split(img, channels);

    cv::Mat sum = channels[0] + channels[1] + channels[2];

    cv::Mat mask = (sum == 0);

    img.setTo(cv::Scalar(255, 255, 255), mask);
}

ClusterResult clusterImage(const cv::Mat& img, const cv::Mat& heightMap){
    CV_Assert(img.type() == CV_8UC3);
    CV_Assert(heightMap.type() == CV_32FC1);
    CV_Assert(img.size() == heightMap.size());

    // Foreground = any non-white pixel 
    cv::Mat gray, binary;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    binary = gray < 250;

    cv::Mat labels, stats, centroids;
    int numLabels = cv::connectedComponentsWithStats(
        binary, labels, stats, centroids, 8, CV_32S);

    // Accumulators per label
    std::vector<double> sumH(numLabels, 0.0);
    std::vector<int>    cntH(numLabels, 0);
    std::vector<cv::Vec3d> sumC(numLabels, cv::Vec3d(0, 0, 0));

    // Single pass over the image
    for (int y = 0; y < labels.rows; ++y)
    {
        const int*       lrow = labels.ptr<int>(y);
        const cv::Vec3b* irow = img.ptr<cv::Vec3b>(y);
        const float*     hrow = heightMap.ptr<float>(y);

        for (int x = 0; x < labels.cols; ++x)
        {
            int l = lrow[x];
            if (l == 0) continue;

            sumC[l][0] += irow[x][0];
            sumC[l][1] += irow[x][1];
            sumC[l][2] += irow[x][2];

            if (std::isfinite(hrow[x])) {
                sumH[l] += hrow[x];
                cntH[l] += 1;
            }
        }
    }

    std::vector<ClusterInfo> clusters;
    for (int l = 1; l < numLabels; ++l)
    {
        int size = stats.at<int>(l, cv::CC_STAT_AREA);
        if (size < 50) continue;

        ClusterInfo c;
        c.label    = l;
        c.size     = size;
        c.centroid = cv::Point2f(
            static_cast<float>(centroids.at<double>(l, 0)),
            static_cast<float>(centroids.at<double>(l, 1)));
        c.height   = (cntH[l] > 0) ? static_cast<float>(sumH[l] / cntH[l]) : 0.0f;
        c.color    = cv::Vec3f(
            static_cast<float>(sumC[l][0] / size),
            static_cast<float>(sumC[l][1] / size),
            static_cast<float>(sumC[l][2] / size));
        clusters.push_back(c);
    }

    return {clusters, labels};
}

std::vector<ClusterInfo> classifyClusters(
    const std::vector<ClusterInfo>& clusters,
    const cv::Mat& labels,
    const cv::Mat& basketMask)
{
    if (labels.empty() || basketMask.empty())
        throw std::runtime_error("labels or basketMask is empty");

    if (labels.type() != CV_32S)
        throw std::runtime_error("labels must be CV_32S");

    if (basketMask.type() != CV_8UC1)
        throw std::runtime_error("basketMask must be CV_8UC1");

    if (labels.size() != basketMask.size())
        throw std::runtime_error("labels and basketMask must have same size");

    // Make sure basket mask is binary 0/255
    cv::Mat basketBinary;
    cv::threshold(basketMask, basketBinary, 0, 255, cv::THRESH_BINARY);

    // Dilated basket mask gives some tolerance for slight projection / mask mismatch
    cv::Mat basketDilated;
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(7, 7)
    );
    cv::dilate(basketBinary, basketDilated, kernel);

    std::vector<ClusterInfo> classified;
    classified.reserve(clusters.size());

    for (const auto& c : clusters)
    {
        ClusterInfo out = c;
        int labelId = c.label;

        cv::Mat clusterMask = (labels == labelId); // CV_8U, 0/255

        int clusterArea = cv::countNonZero(clusterMask);
        if (clusterArea == 0)
        {
            out.type = "unknown";
            classified.push_back(out);
            continue;
        }

        cv::Mat overlapMask;
        cv::bitwise_and(clusterMask, basketBinary, overlapMask);
        int overlap = cv::countNonZero(overlapMask);

        cv::Mat overlapDilatedMask;
        cv::bitwise_and(clusterMask, basketDilated, overlapDilatedMask);
        int overlapDilated = cv::countNonZero(overlapDilatedMask);

        double basketRatio =
            static_cast<double>(overlap) / static_cast<double>(clusterArea);

        double basketRatioDilated =
            static_cast<double>(overlapDilated) / static_cast<double>(clusterArea);

        // Strong basket veto: anything sufficiently overlapping / near basket
        // must not be allowed to fall through into nought/cross logic.
        if (
            overlap > 20 ||
            overlapDilated > 30 ||
            basketRatio > 0.10 ||
            basketRatioDilated > 0.15
        )
        {
            out.type = "basket";
        }
        else
        {
            float meanColor = (c.color[0] + c.color[1] + c.color[2]) / 3.0f;

            if (meanColor < 51.0f)
            {
                out.type = "obstacle";
            }
            else
            {
                std::vector<cv::Point> points;
                cv::findNonZero(clusterMask, points);

                if (points.empty())
                {
                    out.type = "unknown";
                    classified.push_back(out);
                    continue;
                }

                cv::Rect bbox = cv::boundingRect(points);
                cv::Mat crop = clusterMask(bbox);

                int h = crop.rows;
                int w = crop.cols;

                int y0 = h / 3;
                int y1 = 2 * h / 3;
                int x0 = w / 3;
                int x1 = 2 * w / 3;

                int centerH = y1 - y0;
                int centerW = x1 - x0;

                if (centerH <= 0 || centerW <= 0)
                {
                    out.type = "unknown";
                    classified.push_back(out);
                    continue;
                }

                cv::Rect centerRect(x0, y0, centerW, centerH);
                cv::Mat center = crop(centerRect);

                double centerMean = cv::mean(center)[0] / 255.0;

                if (centerMean < 0.2)
                    out.type = "nought";
                else
                    out.type = "cross";
            }
        }

        classified.push_back(out);
    }

    return classified;
}

struct MissionPlanPixels {
  bool valid = false;
  cv::Point2f objectPixel;
  cv::Point2f goalPixel;
  std::string objectType;
};

MissionPlanPixels planMission(const std::vector<ClusterInfo>& clusters)
{
    std::vector<ClusterInfo> noughts;
    std::vector<ClusterInfo> crosses;
    std::vector<ClusterInfo> baskets;

    for (const auto& c : clusters)
    {
        if (c.type == "nought")
        {
            noughts.push_back(c);
        }
        else if (c.type == "cross")
        {
            crosses.push_back(c);
        }
        else if (c.type == "basket")
        {
            baskets.push_back(c);
        }
    }

    if (baskets.empty() || (noughts.empty() && crosses.empty()))
    {
        return {};
    }

    // Largest basket
    const ClusterInfo* basket = &baskets[0];
    for (const auto& b : baskets)
    {
        if (b.size > basket->size)
        {
            basket = &b;
        }
    }

    const float bx = basket->centroid.x;
    const float by = basket->centroid.y;

    // Tie goes to cross
    const std::vector<ClusterInfo>& candidates =
        (crosses.size() >= noughts.size()) ? crosses : noughts;

    // Find candidate closest to basket
    const ClusterInfo* chosen = &candidates[0];
    float bestDistSq =
        (chosen->centroid.x - bx) * (chosen->centroid.x - bx) +
        (chosen->centroid.y - by) * (chosen->centroid.y - by);

    for (const auto& c : candidates)
    {
        float dx = c.centroid.x - bx;
        float dy = c.centroid.y - by;
        float distSq = dx * dx + dy * dy;

        if (distSq < bestDistSq)
        {
            bestDistSq = distSq;
            chosen = &c;
        }
    }

    MissionPlanPixels plan;
    plan.valid = true;
    plan.objectPixel = chosen->centroid;
    plan.goalPixel   = basket->centroid;
    plan.objectType  = chosen->type;
    return plan;
}

MissionPlan cw2::perception_pipeline()
{
  initMoveit();
  auto combined = scanTable();
  savePointCloudPCD(combined, "/tmp/cw2_scan_combined.pcd");

  auto filtered = filterTableFromPointCloud(combined);
  savePointCloudPCD(filtered, "/tmp/cw2_scan_filtered.pcd");

  TopDownResult filtered_result = pcdToTopdownImageWithHeight(filtered, 256);
  saveImage(filtered_result.image, "/tmp/cw2_topdown.png");

  TopDownResult full_result = pcdToTopdownImageWithHeight(combined, 256);
  saveImage(full_result.image, "/tmp/cw2_full_topdown.png");

  cv::Mat basket_mask = detectBasketRegion(full_result.image, full_result.heightMap);
  saveImage(basket_mask, "/tmp/cw2_basket_mask.png");

  filterRobot(filtered_result.image, filtered_result.heightMap, 40);
  makeWhiteBackground(filtered_result.image);
  saveImage(filtered_result.image, "/tmp/cw2_processed.png");

  ClusterResult cluster_result = clusterImage(filtered_result.image, filtered_result.heightMap);

  for (const auto& c : cluster_result.clusters) {
    RCLCPP_INFO(node_->get_logger(),
      "Cluster %d: size=%d centroid=(%.1f, %.1f) height=%.3f color=(B=%.1f, G=%.1f, R=%.1f)",
      c.label, c.size, c.centroid.x, c.centroid.y, c.height,
      c.color[0], c.color[1], c.color[2]);
  }

  std::vector<ClusterInfo> classified =
    classifyClusters(cluster_result.clusters, cluster_result.labels, basket_mask);

  // Find tallest obstacle height (0 if none)
  float max_obstacle_height = 0.0f;
  for (const auto& c : classified) {
    if (c.type == "obstacle" && c.height > max_obstacle_height) {
      max_obstacle_height = c.height;
    }
    RCLCPP_INFO(node_->get_logger(),
      "Cluster %d classified as '%s': size=%d centroid=(%.1f, %.1f) height=%.3f",
      c.label, c.type.c_str(), c.size, c.centroid.x, c.centroid.y, c.height);
  }
  RCLCPP_INFO(node_->get_logger(),
    "Max obstacle height: %.3f", max_obstacle_height);

  MissionPlanPixels pm = planMission(classified);
  MissionPlan mission;
  if (!pm.valid) {
    RCLCPP_WARN(node_->get_logger(), "Mission plan is invalid");
    return mission;
  }
  mission.objectType         = pm.objectType;
  mission.maxObstacleHeight  = max_obstacle_height;

  cv::Point2f pick_w  = pixelToWorld(pm.objectPixel, filtered_result);
  cv::Point2f place_w = pixelToWorld(pm.goalPixel,   filtered_result);

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(
        "panda_link0", "world",
        tf2::TimePointZero, tf2::durationFromSec(2.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(),
      "perception_pipeline TF lookup failed: %s", ex.what());
    return mission;  // valid=false
  }

  // Build PointStamped in world frame, then transform to panda_link0
  geometry_msgs::msg::PointStamped pick_world =
    createPointStamped(pick_w.x, pick_w.y, 0.0f, "world");
  geometry_msgs::msg::PointStamped place_world =
    createPointStamped(place_w.x, place_w.y, 0.0f, "world");

  tf2::doTransform(pick_world,  mission.objectPoint, tf);
  tf2::doTransform(place_world, mission.goalPoint,   tf);

  mission.valid = true;
  return mission;
}

void cw2::jointStatesCallback(
  const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_states_mutex_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    if (msg->name[i] == "panda_finger_joint1") finger1_pos_ = msg->position[i];
    else if (msg->name[i] == "panda_finger_joint2") finger2_pos_ = msg->position[i];
  }
  finger_state_seen_ = true;
}

// Grasp layout
//
// Both shapes come in three arm-widths s = {20, 30, 40} mm and sit in a
// 5s x 5s bounding box, 40 mm tall.
//   cross:   grasp the +x arm in shape-local frame, at (+1.5s, 0)
//   nought:  grasp the +y wall at (0, +2s)
// In both cases the fingers close along the shape-local y axis, so the
// hand's finger-open axis must lie along world yaw + pi/2.
//
// Yaw is recovered from the 4th-order complex moment of the filtered
// cloud, M4 = sum (x + j y)^4. For a C4-symmetric mass distribution
// arg(M4) = 4 theta + phi_shape (mod 2 pi). The cross has mass along
// its arms so phi_shape = 0; the square nought is dominated by its four
// outer corners which pull M4 onto the negative real axis, giving
// phi_shape = pi.

namespace {

constexpr double SHAPE_THICKNESS      = 0.040;   // z extent of the shape
constexpr double EE_TO_FINGER         = 0.1122;  // panda_link8 -> fingertip
// EVIDENCE: read directly from
// panda_description/meshes/collision/finger.stl, which has Z bounds
// [0.0001, 0.0538] in finger-link frame. Combined with finger_joint1
// origin at hand_z=0.0584 in hand frame, the actual fingertip sits
// at hand_z = 0.0584 + 0.0538 = 0.1122 m from link8. The previous
// 0.1034 placed grasp+place poses 8.8 mm too LOW; visible as the
// shape clipping through the basket floor on release (the user's
// 'placed it so deep into the box it broke the physics' symptom).
constexpr double APPROACH_DIST        = 0.10;    // cartesian descent
constexpr double SAFE_ALTITUDE        = 0.70;    // transit height for panda_link8
constexpr double BASKET_FLOOR_OFFSET  = 0.015;   // clearance above goal.z
// EVIDENCE for raising from 0.005 to 0.015: the held shape can tilt
// a few degrees during grasp/transit due to slight off-centre
// finger contact. A 5 deg tilt drops a 30 mm nought's corner by
// 0.075 * sin(5 deg) = 6.5 mm. With 5 mm of nominal clearance the
// corner ends up below the basket interior floor (goal.z + 0.0045
// after settling) and the physics engine resolves the
// interpenetration by ejecting the shape downward through the
// floor - the 'shoves the object deep below the bottom of the box'
// symptom the user reported. 15 mm of nominal clearance covers
// up to ~11 deg of tilt while still being a soft 0.54 m/s drop.
constexpr double SHAPE_LINK_Z_OFFSET  = 0.020;   // link-frame offset in the SDF

// ~2 mm of squeeze on the arm, with a sensible lower bound so we never
// close so far we crush a contact-stop.
inline double graspCloseWidth(double s)
{
  return std::max(0.012, s - 0.004);
}

// panda_hand is bolted on panda_link8 with a fixed -pi/4 yaw offset, so
// the world angle of the finger-open axis is link8_yaw - pi/4. Invert
// that here.
inline double link8YawForHandY(double phi)
{
  return phi + M_PI / 4.0;
}

}  // namespace


bool cw2::detectShapePose(
  const geometry_msgs::msg::Point & obj_xy,
  const std::string & shape_type,
  double & out_yaw,
  double & out_size,
  double & out_cx,
  double & out_cy)
{
  // MULTI-FRAME ACCUMULATION: take N consecutive cloud snapshots from
  // the static observation pose and accumulate ROI-filtered points
  // from all of them into one combined xy_pts. Per-frame point
  // jitter and asymmetric sampling biases largely average out, and
  // the M4 yaw estimate computed over the union has lower variance
  // than any single-frame estimate.
  using namespace std::chrono;
  constexpr int kNumFrames = 5;
  const std::string planning_frame = arm_group_->getPlanningFrame();

  std::vector<Eigen::Vector2d> xy_pts;
  xy_pts.reserve(8192);

  std::uint64_t last_seq = 0;
  {
    std::lock_guard<std::mutex> lk(cloud_mutex_);
    last_seq = g_cloud_sequence_;
  }

  std::string cloud_frame;
  for (int frame_idx = 0; frame_idx < kNumFrames; ++frame_idx) {
    // Wait for a NEW frame (sequence advances).
    const auto deadline = steady_clock::now() + milliseconds(1000);
    PointCPtr cloud_in;
    while (steady_clock::now() < deadline) {
      std::this_thread::sleep_for(milliseconds(30));
      std::lock_guard<std::mutex> lk(cloud_mutex_);
      if (g_cloud_sequence_ > last_seq && !g_cloud_ptr->empty()) {
        cloud_in = g_cloud_ptr;
        cloud_frame = g_input_pc_frame_id_;
        last_seq = g_cloud_sequence_;
        break;
      }
    }
    if (!cloud_in || cloud_in->empty() || cloud_frame.empty()) {
      RCLCPP_WARN(node_->get_logger(),
        "detectShapePose: no fresh cloud for frame %d, skipping",
        frame_idx);
      continue;
    }

    geometry_msgs::msg::TransformStamped tfs;
    try {
      tfs = tf_buffer_.lookupTransform(
        planning_frame, cloud_frame, tf2::TimePointZero,
        tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(node_->get_logger(),
        "detectShapePose: tf %s -> %s failed: %s, skipping frame",
        cloud_frame.c_str(), planning_frame.c_str(), e.what());
      continue;
    }
    const Eigen::Quaterniond Rq(
      tfs.transform.rotation.w, tfs.transform.rotation.x,
      tfs.transform.rotation.y, tfs.transform.rotation.z);
    const Eigen::Vector3d Tt(
      tfs.transform.translation.x, tfs.transform.translation.y,
      tfs.transform.translation.z);

    const double z_lo = 0.028;
    const double z_hi = 0.110;
    const double roi_r2 = 0.15 * 0.15;
    for (const auto & p : *cloud_in) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      const Eigen::Vector3d pw =
        Rq * Eigen::Vector3d(p.x, p.y, p.z) + Tt;
      if (pw.z() < z_lo || pw.z() > z_hi) continue;
      const double dx = pw.x() - obj_xy.x;
      const double dy = pw.y() - obj_xy.y;
      if (dx * dx + dy * dy > roi_r2) continue;
      const int r = p.r, g = p.g, b = p.b;
      if (g > r + 25 && g > b + 25) continue;
      xy_pts.emplace_back(pw.x(), pw.y());
    }
  }
  if (xy_pts.size() < 30) {
    RCLCPP_ERROR(node_->get_logger(),
      "detectShapePose: only %zu points after %d-frame accumulation",
      xy_pts.size(), kNumFrames);
    return false;
  }

  // Cluster mean - kept for diagnostics only. The mean is biased
  // toward whichever side of the shape has more visible points (the
  // wrist camera is offset, so this can be 1-2 cm away from the
  // true centre). We do NOT use it as the M4 pivot anymore.
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto & q : xy_pts) centroid += q;
  centroid /= static_cast<double>(xy_pts.size());

  // M4 PIVOT: the spawner-reported obj_xy is the true geometric
  // centre of the shape (the SDF model origin coincides with the
  // shape centre after the link transform). Computing the 4th-order
  // moment around obj_xy instead of the biased cluster mean removes
  // the centroid-bias coupling from the yaw estimate and makes the
  // recovered yaw an unbiased measurement of orientation alone.
  // Earlier behaviour: 5 deg of yaw bias from cluster mean offset
  // produced ~5 mm of finger-position error at the wall - enough to
  // straddle one finger past the wall edge. With obj as pivot, only
  // the per-frame measurement noise remains.
  const Eigen::Vector2d pivot(obj_xy.x, obj_xy.y);

  // 4th-order complex moment. The nought's corners flip its sign, so
  // cancel that phase before dividing by 4.
  std::complex<double> M4(0.0, 0.0);
  for (const auto & q : xy_pts) {
    const std::complex<double> z(q.x() - pivot.x(), q.y() - pivot.y());
    const std::complex<double> z2 = z * z;
    M4 += z2 * z2;
  }
  const double phi_shape = (shape_type == "nought") ? M_PI : 0.0;
  double yaw = 0.25 * std::arg(M4 * std::polar(1.0, -phi_shape));
  while (yaw >  M_PI / 4.0) yaw -= M_PI / 2.0;
  while (yaw < -M_PI / 4.0) yaw += M_PI / 2.0;
  out_yaw = yaw;

  // Oriented bounding box in the (mean-centroid, detected-yaw) frame.
  // The mean centroid is biased toward the far-from-camera side because
  // the wrist camera sits +X +Y of link8 at the observation pose, and
  // the open gripper shadows the near-camera face of the shape, leaving
  // more cluster points on the far face. The OBB midpoint, by contrast,
  // is the average of the EXTREME points along each axis; provided the
  // far-edge points along all four sides are visible (which they are
  // for this top-down + small camera offset) the midpoint sits at the
  // true geometric centre and we can correct the bias one-shot.
  const double cs = std::cos(-yaw), sn = std::sin(-yaw);
  double x_lo =  std::numeric_limits<double>::infinity();
  double x_hi = -std::numeric_limits<double>::infinity();
  double y_lo =  std::numeric_limits<double>::infinity();
  double y_hi = -std::numeric_limits<double>::infinity();
  double max_half = 0.0;
  for (const auto & q : xy_pts) {
    const double dx = q.x() - centroid.x(), dy = q.y() - centroid.y();
    const double rx = cs * dx - sn * dy;
    const double ry = sn * dx + cs * dy;
    if (rx < x_lo) x_lo = rx;
    if (rx > x_hi) x_hi = rx;
    if (ry < y_lo) y_lo = ry;
    if (ry > y_hi) y_hi = ry;
    max_half = std::max(max_half,
                        std::max(std::fabs(rx), std::fabs(ry)));
  }

  // Span half-extent (for diagnostics / centroid only). We do NOT use
  // this for the size snap because the depth sensor strips ~10-15 mm
  // off each outer edge of the shape, so the OBB extent reads
  // systematically short (a 30 mm nought lands at obb_half ~ 0.060
  // when the true outer half is 0.075). max_half below is more
  // robust because it captures the far extreme from the biased mean,
  // which roughly cancels the edge stripping for our setup.
  const double obb_half = std::max(x_hi - x_lo, y_hi - y_lo) / 2.0;
  const double s_raw = max_half / 2.5;
  const std::array<double, 3> sizes = {0.020, 0.030, 0.040};
  double best_size = sizes[0];
  double best_err = std::fabs(s_raw - sizes[0]);
  for (size_t i = 1; i < sizes.size(); ++i) {
    const double e = std::fabs(s_raw - sizes[i]);
    if (e < best_err) { best_err = e; best_size = sizes[i]; }
  }
  if (s_raw > 0.050) best_size = 0.040;
  out_size = best_size;

  // OBB midpoint (in rotated frame) is the bias offset from mean
  // centroid to true centre. Rotate it back into world coords and
  // anchor on the mean centroid to recover the true world centre.
  const double mid_local_x = 0.5 * (x_lo + x_hi);
  const double mid_local_y = 0.5 * (y_lo + y_hi);
  const double cs_b = std::cos(yaw), sn_b = std::sin(yaw);
  out_cx = centroid.x() + cs_b * mid_local_x - sn_b * mid_local_y;
  out_cy = centroid.y() + sn_b * mid_local_x + cs_b * mid_local_y;

  RCLCPP_INFO(node_->get_logger(),
    "detectShapePose[%s]: pts=%zu mean=(%.3f,%.3f) yaw=%.3f "
    "obb_xlo/hi=(%.3f,%.3f) obb_ylo/hi=(%.3f,%.3f) obb_half=%.3f "
    "max_half=%.3f raw_s=%.3f -> size=%.3f corr=(%.3f,%.3f)",
    shape_type.c_str(), xy_pts.size(), centroid.x(), centroid.y(),
    out_yaw,
    x_lo, x_hi, y_lo, y_hi,
    obb_half, max_half, s_raw, out_size,
    out_cx, out_cy);

  return true;
}


bool cw2::t1_pickAndPlace(
  const geometry_msgs::msg::Point & obj,
  const geometry_msgs::msg::Point & goal,
  const std::string & shape_type)
{

  detachShape();
  removeShapeCollision();
  removeTileCollision();  // also clear any stale tile slab from a
                          // failed transit that didn't reach removeTileCollision
  removeBasketCollision();  // and any stale basket walls from a prior task
  addBasketCollision(goal);

  if (!openGripper()) return false;
  addShapeCollision(obj, shape_type);

  // Overhead observation pose directly above obj.
  if (!moveArmToPose(
      makeTopDownPose(obj.x, obj.y, SAFE_ALTITUDE, 0.0),
      "safe-above-observation")) {
    removeShapeCollision();
    return false;
  }

  double yaw = 0.0;
  double size_s = 0.040;
  double cx = obj.x, cy = obj.y;
  if (!detectShapePose(obj, shape_type, yaw, size_s, cx, cy)) {
    RCLCPP_WARN(node_->get_logger(),
      "perception failed, falling back to yaw=0, size=40mm and obj xy");
    yaw = 0.0;
    size_s = 0.040;
    cx = obj.x;
    cy = obj.y;
  }
  const bool is_cross = (shape_type == "cross");
  const double ox_local = is_cross ? (1.5 * size_s) : 0.0;
  const double oy_local = is_cross ? 0.0           : (2.0 * size_s);
  const double c_y = std::cos(yaw), s_y = std::sin(yaw);
  const double ox_world = c_y * ox_local - s_y * oy_local;
  const double oy_world = s_y * ox_local + c_y * oy_local;
  const double grasp_x = obj.x + ox_world;
  const double grasp_y = obj.y + oy_world;

  double link8_yaw = link8YawForHandY(yaw + M_PI / 2.0);
  while (link8_yaw >  M_PI) link8_yaw -= 2.0 * M_PI;
  while (link8_yaw < -M_PI) link8_yaw += 2.0 * M_PI;
  if (link8_yaw >  M_PI / 2.0) link8_yaw -= M_PI;
  if (link8_yaw < -M_PI / 2.0) link8_yaw += M_PI;

  const double transit_yaw = -M_PI / 4.0;
  const double sc_hx = -ox_local;
  const double sc_hy =  oy_local;
  const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
  const double sc_lx = (sc_hx + sc_hy) * inv_sqrt2;
  const double sc_ly = (-sc_hx + sc_hy) * inv_sqrt2;
  const double sc_px = sc_lx;
  const double sc_py = -sc_ly;
  const double cs_t = std::cos(transit_yaw);
  const double sn_t = std::sin(transit_yaw);
  const double sc_wx = sc_px * cs_t - sc_py * sn_t;
  const double sc_wy = sc_px * sn_t + sc_py * cs_t;
  const double place_x = goal.x - sc_wx;
  const double place_y = goal.y - sc_wy;

  const double grasp_ee_z  = obj.z + SHAPE_LINK_Z_OFFSET
                              + SHAPE_THICKNESS / 2.0 + EE_TO_FINGER;
  const double pre_grasp_z = grasp_ee_z + APPROACH_DIST;
  const double place_ee_z  = goal.z + BASKET_FLOOR_OFFSET
                              + SHAPE_THICKNESS / 2.0 + EE_TO_FINGER;
  const double pre_place_z = place_ee_z + APPROACH_DIST;
  const double safe_z      = std::max({SAFE_ALTITUDE, pre_grasp_z + 0.05,
                                       pre_place_z + 0.05});

  const double close_w = graspCloseWidth(size_s);
  const double hand_y_ang = link8_yaw - M_PI / 4.0;
  const double hyx = std::cos(hand_y_ang);
  const double hyy = std::sin(hand_y_ang);
  const double f1x = grasp_x + 0.035 * hyx;
  const double f1y = grasp_y + 0.035 * hyy;
  const double f2x = grasp_x - 0.035 * hyx;
  const double f2y = grasp_y - 0.035 * hyy;

  RCLCPP_INFO(node_->get_logger(),
    "T1[%s] s=%.3f yaw=%.3f obj=(%.3f,%.3f,%.3f) c=(%.3f,%.3f) "
    "off=(%.3f,%.3f) grasp=(%.3f,%.3f,%.3f) link8_yaw=%.3f "
    "open_f1=(%.3f,%.3f) open_f2=(%.3f,%.3f) place=(%.3f,%.3f,%.3f) close=%.3f",
    shape_type.c_str(), size_s, yaw,
    obj.x, obj.y, obj.z, cx, cy,
    ox_world, oy_world,
    grasp_x, grasp_y, grasp_ee_z, link8_yaw,
    f1x, f1y, f2x, f2y,
    place_x, place_y, place_ee_z, close_w);

  if (!moveArmCartesian(
      {makeTopDownPose(grasp_x, grasp_y, safe_z, link8_yaw)},
      0.01, 0.0, /*allow_fallback=*/false)) {
    removeShapeCollision();
    return false;
  }

  removeShapeCollision();

  if (!moveArmCartesian(
      {makeTopDownPose(grasp_x, grasp_y, pre_grasp_z, link8_yaw)},
      0.005, 0.0, /*allow_fallback=*/false)) return false;
  if (!moveArmCartesian(
      {makeTopDownPose(grasp_x, grasp_y, grasp_ee_z, link8_yaw)},
      0.005, 0.0, /*allow_fallback=*/false)) return false;

  if (!closeGripper(close_w)) return false;

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  {
    double f1 = 0.0, f2 = 0.0;
    bool seen = false;
    {
      std::lock_guard<std::mutex> lk(joint_states_mutex_);
      f1 = finger1_pos_;
      f2 = finger2_pos_;
      seen = finger_state_seen_;
    }
    const double total_width = f1 + f2;
    const double threshold = close_w + 0.002;
    RCLCPP_INFO(node_->get_logger(),
      "grasp verify: seen=%d f1=%.4f f2=%.4f width=%.4f close_w=%.4f thr=%.4f",
      seen, f1, f2, total_width, close_w, threshold);
    if (seen && total_width < threshold) {
      RCLCPP_ERROR(node_->get_logger(),
        "grasp verify FAILED: fingers fully closed (width=%.4f < %.4f) - "
        "no shape gripped, aborting before lift",
        total_width, threshold);
      openGripper();
      removeBasketCollision();
      return false;
    }
  }

  attachShape(size_s, ox_local, oy_local);

  // Straight up at the grasp yaw.
  if (!moveArmCartesian(
      {makeTopDownPose(grasp_x, grasp_y, safe_z, link8_yaw)},
      0.005, 0.0, /*allow_fallback=*/false)) return false;

  if (!moveArmCartesian(
      {makeTopDownPose(grasp_x, grasp_y, safe_z, transit_yaw)},
      0.05, 0.0, /*allow_fallback=*/true)) return false;

  // Long transit from grasp to over-basket via joint-space planning.
  addTileCollision();
  const bool transit_ok = moveArmToPose(
    makeTopDownPose(place_x, place_y, safe_z, transit_yaw),
    "grasp-to-basket-transit");
  removeTileCollision();
  if (!transit_ok) return false;

  // Descend at neutral yaw to drop the shape into the basket.
  if (!moveArmCartesian(
      {makeTopDownPose(place_x, place_y, place_ee_z, transit_yaw)},
      0.005, 0.0, /*allow_fallback=*/false)) return false;

  if (!openGripper()) return false;

  // Shape is now in the basket - detach it from the gripper so the
  detachShape();

  // Warn-only: if the retreat fails the shape is already dropped.
  if (!moveArmCartesian(
      {makeTopDownPose(place_x, place_y, safe_z, transit_yaw)},
      0.005, 0.0, /*allow_fallback=*/false)) {
    RCLCPP_WARN(node_->get_logger(), "retreat failed after place");
  }

  removeBasketCollision();
  return true;
}


bool cw2::openGripper()
{
  hand_group_->setNamedTarget("open");
  auto result = hand_group_->move();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(),
      "openGripper failed (code %d)", static_cast<int>(result.val));
    return false;
  }
  return true;
}

bool cw2::closeGripper(double width)
{
  const double finger_pos = width / 2.0;
  std::map<std::string, double> targets;
  targets["panda_finger_joint1"] = finger_pos;
  targets["panda_finger_joint2"] = finger_pos;
  hand_group_->setJointValueTarget(targets);

  auto result = hand_group_->move();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    // Contact stop is expected when the fingers hit the arm before
    // reaching the commanded width.
    RCLCPP_WARN(node_->get_logger(),
      "closeGripper: code %d (contact stop?), continuing",
      static_cast<int>(result.val));
  }
  return true;
}


bool cw2::moveArmToPose(
  const geometry_msgs::msg::Pose & target_pose,
  const std::string & description)
{
  arm_group_->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_rc = arm_group_->plan(plan);
  if (plan_rc != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(),
      "moveArmToPose [%s]: plan failed (%d)",
      description.c_str(), static_cast<int>(plan_rc.val));
    arm_group_->clearPoseTargets();
    return false;
  }

  auto exec_rc = arm_group_->execute(plan);
  arm_group_->clearPoseTargets();
  if (exec_rc != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(),
      "moveArmToPose [%s]: exec failed (%d)",
      description.c_str(), static_cast<int>(exec_rc.val));
    return false;
  }
  return true;
}

bool cw2::moveArmCartesian(
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  double eef_step,
  double jump_threshold,
  bool allow_fallback)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = arm_group_->computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

  if (fraction < 0.0) {
    RCLCPP_WARN(node_->get_logger(),
      "moveArmCartesian: compute error");
    if (allow_fallback) {
      return moveArmToPose(waypoints.back(), "cartesian-fallback");
    }
    return false;
  }
  if (fraction < 0.9) {
    RCLCPP_WARN(node_->get_logger(),
      "moveArmCartesian: only %.1f%%", fraction * 100.0);
    if (allow_fallback) {
      return moveArmToPose(waypoints.back(), "cartesian-fallback");
    }
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  auto exec_rc = arm_group_->execute(plan);
  if (exec_rc != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(),
      "moveArmCartesian: exec failed (%d)",
      static_cast<int>(exec_rc.val));
    return false;
  }
  return true;
}


geometry_msgs::msg::Pose cw2::makeTopDownPose(
  double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  // Roll = pi points panda_link8's z axis down; yaw rotates about world z.
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, yaw);
  q.normalize();
  pose.orientation = tf2::toMsg(q);
  return pose;
}


void cw2::addGroundCollision()
{
  moveit_msgs::msg::CollisionObject ground;
  ground.header.frame_id = arm_group_->getPlanningFrame();
  ground.id = "ground_plane";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  // Subfloor only (z = -0.020 to 0). Earlier we tried extending this
  // upward to z = +0.020 (covering the tile) so the joint-space
  // transit couldn't drag the held shape across the tile - but
  // EVIDENCE from /tmp/cw2.log timestamp 80255.802:
  //   'ground_plane' ... 'panda_rightfinger' ... constitutes a collision
  // shows the start state of closeGripper failed because MoveIt's
  // default robot+object collision padding (~10 mm each, total ~20
  // mm) brings the expanded finger volume (true finger min z =
  // link8.z - 0.1122 = 0.031 at grasp pose) into contact with the
  // expanded ground top. The held-shape-on-tile problem is now
  // handled separately by the path constraint applied during
  // transit, NOT by extending the ground.
  box.dimensions[0] = 3.0;
  box.dimensions[1] = 3.0;
  box.dimensions[2] = 0.020;

  geometry_msgs::msg::Pose pose;
  pose.position.z = -0.010;
  pose.orientation.w = 1.0;

  ground.primitives.push_back(box);
  ground.primitive_poses.push_back(pose);
  ground.operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_interface_.applyCollisionObjects({ground});
}


void cw2::addShapeCollision(
  const geometry_msgs::msg::Point & obj,
  const std::string & shape_type)
{
  // Bounding box that closely tracks the shape itself. Worst-case
  // footprint is 5s x 5s at s=0.040 -> 0.20 x 0.20 m. The shape sits
  // on the tile spanning z = [0.020, 0.060]; we wrap that with a 20
  // mm margin -> z = [0.000, 0.080], 80 mm tall.
  //
  // EVIDENCE for shrinking from 0.25 m to 0.08 m: in the previous
  // failure log the joint-space planner returned a path whose
  // panda_rightfinger collided with t1_shape_obstacle at trajectory
  // indices 25 and 38 of 49, after post-smoothing. That happened
  // because the column extended to z=0.25 m, which is well into the
  // natural elbow/finger sweep envelope at 0.10-0.30 m. Capping the
  // column at z=0.08 means it only blocks links that actually intrude
  // on the shape's z-range, so smoothed paths can pass freely above.
  (void)shape_type;  // worst-case sizing covers both shapes

  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.header.frame_id = arm_group_->getPlanningFrame();
  obstacle.id = "t1_shape_obstacle";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = 0.20;
  box.dimensions[1] = 0.20;
  box.dimensions[2] = 0.08;

  geometry_msgs::msg::Pose pose;
  pose.position.x = obj.x;
  pose.position.y = obj.y;
  // Centre at z=0.040 = mid-shape; column spans [0, 0.080] in world.
  pose.position.z = 0.040;
  pose.orientation.w = 1.0;

  obstacle.primitives.push_back(box);
  obstacle.primitive_poses.push_back(pose);
  obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_interface_.applyCollisionObjects({obstacle});
}


void cw2::removeShapeCollision()
{
  planning_scene_interface_.removeCollisionObjects({"t1_shape_obstacle"});
}


void cw2::attachShape(double size_s, double ox_local, double oy_local)
{
  // Held shape after closeGripper, modelled in panda_hand frame.
  // At grasp time the gripper is oriented so hand_Y aligns with
  // shape +Y (in world) and hand_X aligns with -shape +X (in world);
  // therefore in hand frame:
  //   shape_center = (ox_local, -oy_local, EE_TO_FINGER)
  // i.e. the SHAPE CENTER is offset from the gripper TCP by the
  // grasp offset, with hand_X having a sign flip relative to
  // shape_X. Modelling the box at this offset means the planning
  // scene sees the held shape where it actually is.
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = "panda_hand";
  aco.object.id = "held_shape";
  aco.object.header.frame_id = "panda_hand";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  // 5*size_s in each lateral axis (worst-case bounding for either
  // shape with a few mm of margin), 40 mm thick.
  box.dimensions[0] = 5.0 * size_s;
  box.dimensions[1] = 5.0 * size_s;
  box.dimensions[2] = SHAPE_THICKNESS;

  geometry_msgs::msg::Pose box_pose;
  // Shape center in hand frame: at link8_yaw = -pi/4 (the place yaw
  // we use), hand_X = +shape_X and hand_Y = -shape_Y. Shape center is
  // at shape-frame (-ox_local, -oy_local) relative to the TCP, which
  // maps to hand-frame (-ox_local, +oy_local, EE_TO_FINGER).
  box_pose.position.x = -ox_local;
  box_pose.position.y =  oy_local;
  box_pose.position.z = EE_TO_FINGER;
  box_pose.orientation.w = 1.0;
  aco.object.primitives.push_back(box);
  aco.object.primitive_poses.push_back(box_pose);
  aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  aco.touch_links = {
    "panda_hand", "panda_leftfinger", "panda_rightfinger"
  };

  planning_scene_interface_.applyAttachedCollisionObject(aco);
}


void cw2::detachShape()
{
  // Detach AND remove. Detach alone leaves the object floating at the
  // last hand pose as a non-attached collision object, which would
  // then block subsequent moves.
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = "panda_hand";
  aco.object.id = "held_shape";
  aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  planning_scene_interface_.applyAttachedCollisionObject(aco);
  planning_scene_interface_.removeCollisionObjects({"held_shape"});
}


void cw2::addTileCollision()
{
  moveit_msgs::msg::CollisionObject tile;
  tile.header.frame_id = arm_group_->getPlanningFrame();
  tile.id = "tile_top";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  // 3 m x 3 m thin slab, 2 mm thick. Span z = [0.018, 0.020] - just
  // the very top of the tile. With held_shape attached at hand_z =
  // [0.0922, 0.1322] and MoveIt's default ~10 mm robot+10 mm object
  // padding, this forces link8.z > ~0.16 m throughout the transit,
  // which is enough margin to keep the held_shape from dragging.
  box.dimensions[0] = 3.0;
  box.dimensions[1] = 3.0;
  box.dimensions[2] = 0.002;

  geometry_msgs::msg::Pose pose;
  pose.position.z = 0.019;
  pose.orientation.w = 1.0;

  tile.primitives.push_back(box);
  tile.primitive_poses.push_back(pose);
  tile.operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_interface_.applyCollisionObjects({tile});
}


void cw2::removeTileCollision()
{
  planning_scene_interface_.removeCollisionObjects({"tile_top"});
}


void cw2::addBasketCollision(const geometry_msgs::msg::Point & goal)
{
  // Basket geometry from cw2_world_spawner/models/basket/model.sdf:
  //   - base 350 x 350 x 9 mm at model origin
  //   - 4 walls 9 x 350 x 50 mm (or 350 x 9 x 50 mm) at +/- 0.17050 m
  //     in X or Y, centred at z = 0.025 in model frame (wall span
  //     z = [0, 0.050] in model frame)
  // The model is spawned at goal.xyz, so the basket walls in WORLD
  // span z = [goal.z, goal.z + 0.050]. We model the four walls as
  // collision boxes; we do NOT model the base because the place
  // descent has to land *on* the base, and the base is already
  // covered by ground_plane padding.
  const std::string frame = arm_group_->getPlanningFrame();
  const double inset = 0.17050;     // wall centre offset from basket centre
  const double wall_t = 0.009;      // wall thickness
  const double wall_h = 0.050;      // wall height
  const double wall_l = 0.350;      // wall length
  const double wall_z = goal.z + 0.025;  // wall centre z

  // EVIDENCE for using a different API: the prior run's log contained
  //   "Tried to remove world object 'basket_wall_xpos', but it does
  //    not exist in this scene"
  // at task end - meaning the previous applyCollisionObjects(vector)
  // call silently failed to add any walls. With no walls in the
  // scene the joint-space planner can swing through where the basket
  // physically sits, and on the third place attempt the user observed
  // the basket physically shifted. The fix is to add each wall with
  // its own applyCollisionObject call, then yield briefly to give
  // the planning scene monitor time to propagate the change before
  // the next motion-planning request.
  auto add_wall =
    [&](const std::string & id, double cx, double cy,
        double sx, double sy)
    {
      moveit_msgs::msg::CollisionObject w;
      w.header.frame_id = frame;
      w.id = id;
      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions.resize(3);
      box.dimensions[0] = sx;
      box.dimensions[1] = sy;
      box.dimensions[2] = wall_h;
      geometry_msgs::msg::Pose p;
      p.position.x = cx;
      p.position.y = cy;
      p.position.z = wall_z;
      p.orientation.w = 1.0;
      w.primitives.push_back(box);
      w.primitive_poses.push_back(p);
      w.operation = moveit_msgs::msg::CollisionObject::ADD;
      planning_scene_interface_.applyCollisionObject(w);
    };

  add_wall("basket_wall_xpos", goal.x + inset, goal.y, wall_t, wall_l);
  add_wall("basket_wall_xneg", goal.x - inset, goal.y, wall_t, wall_l);
  add_wall("basket_wall_ypos", goal.x, goal.y + inset, wall_l, wall_t);
  add_wall("basket_wall_yneg", goal.x, goal.y - inset, wall_l, wall_t);

  // Yield + verify. Earlier evidence: the prior code used
  // applyCollisionObjects(vector) and 100 ms wasn't enough - the
  // walls weren't actually in the scene by the time the next plan()
  // request ran (warning at task end:
  //   "Tried to remove world object 'basket_wall_xpos', but it does
  //    not exist in this scene")
  // So we wait longer and then poll getKnownObjectNames() until the
  // walls show up (or 1 s timeout).
  using namespace std::chrono;
  std::this_thread::sleep_for(milliseconds(200));
  const auto deadline = steady_clock::now() + milliseconds(1000);
  while (steady_clock::now() < deadline) {
    const auto known = planning_scene_interface_.getKnownObjectNames();
    int found = 0;
    for (const auto & n : known) {
      if (n == "basket_wall_xpos" || n == "basket_wall_xneg" ||
          n == "basket_wall_ypos" || n == "basket_wall_yneg") {
        ++found;
      }
    }
    if (found == 4) {
      RCLCPP_INFO(node_->get_logger(),
        "addBasketCollision: all 4 basket walls present in planning scene");
      return;
    }
    std::this_thread::sleep_for(milliseconds(50));
  }
  RCLCPP_WARN(node_->get_logger(),
    "addBasketCollision: basket walls did not appear within 1 s; "
    "subsequent plans may not avoid the basket");
}


void cw2::removeBasketCollision()
{
  planning_scene_interface_.removeCollisionObjects({
    "basket_wall_xpos", "basket_wall_xneg",
    "basket_wall_ypos", "basket_wall_yneg"
  });
}

///////////////////////////////////////////////////////////////////////////////
// Callbacks
///////////////////////////////////////////////////////////////////////////////

// TODO - implement Task 1 later 
// void cw2::t1_callback(
//   const std::shared_ptr<cw2_world_spawner::srv::Task1Service::Request> request,
//   std::shared_ptr<cw2_world_spawner::srv::Task1Service::Response> response)
// {
//   // (void)request;
//   // (void)response;
  
//   auto mission = perception_pipeline();

//   // Print out Mission Plan for debugging
//   RCLCPP_INFO(node_->get_logger(),
//     "Mission: type=%s pick(%.3f, %.3f, %.3f) place(%.3f, %.3f, %.3f) maxObsH=%.3f",
//     mission.objectType.c_str(),
//     mission.objectPoint.point.x, mission.objectPoint.point.y, mission.objectPoint.point.z,
//     mission.goalPoint.point.x,   mission.goalPoint.point.y,   mission.goalPoint.point.z,
//     mission.maxObstacleHeight);

//   // Print out Ground Truth Info for debugging
//   const auto & object_point = request->object_point;
//   const auto & goal_point = request->goal_point;
//   const std::string & shape_type = request->shape_type;

//   RCLCPP_INFO(
//     node_->get_logger(),
//     "Ground Truth: Object ='%s', x=%.3f, y=%.3f, z=%.3f",
//     object_point.header.frame_id.c_str(),
//     object_point.point.x,
//     object_point.point.y,
//     object_point.point.z);

//   RCLCPP_INFO(
//     node_->get_logger(),
//     "Ground Truth Goal ='%s', x=%.3f, y=%.3f, z=%.3f",
//     goal_point.header.frame_id.c_str(),
//     goal_point.point.x,
//     goal_point.point.y,
//     goal_point.point.z);

//   RCLCPP_INFO(
//     node_->get_logger(),
//     "shape_type: '%s'",
//     shape_type.c_str());
  
//   (void)response;

//   std::string frame_id;
//   std::size_t point_count = 0;
//   std::uint64_t sequence = 0;
//   {
//     std::lock_guard<std::mutex> lock(cloud_mutex_);
//     frame_id = g_input_pc_frame_id_;
//     point_count = g_cloud_ptr ? g_cloud_ptr->size() : 0;
//     sequence = g_cloud_sequence_;
//   }

//   RCLCPP_WARN(
//     node_->get_logger(),
//     "Task 1 is not implemented in cw2_team_13. Latest cloud: seq=%llu frame='%s' points=%zu",
//     static_cast<unsigned long long>(sequence),
//     frame_id.c_str(),
//     point_count);
// }

void cw2::t1_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task1Service::Request>  request,
  std::shared_ptr<cw2_world_spawner::srv::Task1Service::Response> response)
{
  (void)response;

  RCLCPP_INFO(node_->get_logger(),
    "Task 1 start: shape='%s' obj=(%.3f,%.3f,%.3f) goal=(%.3f,%.3f,%.3f)",
    request->shape_type.c_str(),
    request->object_point.point.x,
    request->object_point.point.y,
    request->object_point.point.z,
    request->goal_point.point.x,
    request->goal_point.point.y,
    request->goal_point.point.z);

  addGroundCollision();

  bool ok = t1_pickAndPlace(
    request->object_point.point,
    request->goal_point.point,
    request->shape_type);

  if (ok) {
    RCLCPP_INFO(node_->get_logger(), "Task 1 complete.");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Task 1 failed.");
  }
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
  auto mission = perception_pipeline();
  // Waiting for a Task 1 implementation to then do pick and place 
  auto pick = mission.objectPoint;
  auto place = mission.goalPoint;
  auto obj_type = mission.objectType;

  // Print out the Mission Plan for debugging
  RCLCPP_INFO(node_->get_logger(),
    "Mission: type=%s pick(%.3f, %.3f, %.3f) place(%.3f, %.3f, %.3f) maxObsH=%.3f",
    mission.objectType.c_str(),
    mission.objectPoint.point.x, mission.objectPoint.point.y, mission.objectPoint.point.z,
    mission.goalPoint.point.x,   mission.goalPoint.point.y,   mission.goalPoint.point.z,
    mission.maxObstacleHeight);

  // Call t1 pick and place
  bool ok = t1_pickAndPlace(pick.point, place.point, obj_type);
  if (!ok) {
    RCLCPP_WARN(node_->get_logger(), "t1_pickAndPlace failed for shape '%s'", obj_type.c_str());
  }

  (void)request;
  response->total_num_shapes = 0;
  response->num_most_common_shape = 0;
  response->most_common_shape_vector.clear(); 
}