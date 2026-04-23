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
  g_cloud_msg_ = msg;                  // <-- keep the raw msg too
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

// --- Replacement for capturePointCloudAtTarget ---
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::capturePointCloudAtTarget(
  geometry_msgs::msg::Pose target)
{
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

  // Orientation: roll=pi, pitch=0, yaw=-pi/4.
  // The roll flips the EEF so its +Z points world -Z (camera looks down).
  // The -pi/4 yaw cancels the camera's 45-degree mount offset on panda_link8
  // so the cloud's axes align with world x/y after transform.
  auto makeDownPose = [](double x, double y, double z) {
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, -M_PI / 4.0);
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = tf2::toMsg(q);
    return pose;
  };

  // 8-cell perimeter grid around the robot base (center omitted to save time;
  // the robot base would occlude that view anyway).
  // x: +forward, -backward. y: +left, -right (Franka convention).
  // Sides pushed further out (0.5) to better reach the table edges.
  const std::vector<std::pair<double, double>> scan_grid = {
    { 0.4,  0.4}, { 0.4,  0.0}, { 0.4, -0.4},   // front row
    { 0.0,  0.5},               { 0.0, -0.5},   // mid row (sides only)
    {-0.4,  0.4}, {-0.4,  0.0}, {-0.4, -0.4},   // back row
  };

  const double z = 0.75;

  int idx = 0;
  for (const auto &[x, y] : scan_grid) {
    RCLCPP_INFO(node_->get_logger(),
      "Scanning cell %d/%zu: [x=%.2f, y=%.2f]",
      idx + 1, scan_grid.size(), x, y);

    auto pose = makeDownPose(x, y, z);
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

    return {image, heightMap};
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

// NOTE: ClusterInfo and ClusterResult are defined in cw2_class.h

ClusterResult clusterImage(const cv::Mat& img, const cv::Mat& heightMap)
{
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
    {
        throw std::runtime_error("labels or basketMask is empty");
    }

    if (labels.type() != CV_32S)
    {
        throw std::runtime_error("labels must be CV_32S");
    }

    if (basketMask.type() != CV_8UC1)
    {
        throw std::runtime_error("basketMask must be CV_8UC1");
    }

    if (labels.size() != basketMask.size())
    {
        throw std::runtime_error("labels and basketMask must have same size");
    }

    std::vector<ClusterInfo> classified;
    classified.reserve(clusters.size());

    for (const auto& c : clusters)
    {
        ClusterInfo out = c;
        int labelId = c.label;

        cv::Mat clusterMask = (labels == labelId);   // 0 / 255, CV_8U

        // Basket override
        cv::Mat overlapMask;
        cv::bitwise_and(clusterMask, basketMask, overlapMask);
        int overlap = cv::countNonZero(overlapMask);

        if (overlap > 50)
        {
            out.type = "basket";
        }
        else
        {
            // OpenCV color is BGR, but mean(color) is unchanged by channel order
            float meanColor = (c.color[0] + c.color[1] + c.color[2]) / 3.0f;

            // Python used threshold 0.2 on float [0,1]
            // Equivalent in uint8 [0,255] is about 51
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
                {
                    out.type = "nought";
                }
                else
                {
                    out.type = "cross";
                }
            }
        }

        classified.push_back(out);
    }

    return classified;
}

// NOTE: MissionPlan is defined in cw2_class.h

MissionPlan planMission(const std::vector<ClusterInfo>& clusters)
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
        // RCLCPP_WARN(node_->get_logger(),
        //     "planMission: no basket or no nought/cross candidates found");
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

    MissionPlan plan;
    plan.valid = true;
    plan.objectPose = chosen->centroid;
    plan.goalPose = basket->centroid;
    return plan;
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

  // Step 1: Scan the table
  auto combined = scanTable();
  savePointCloudPCD(combined, "/tmp/cw2_scan_combined.pcd");

  // Step 2: Remove the green table
  auto filtered = filterTableFromPointCloud(combined);
  savePointCloudPCD(filtered, "/tmp/cw2_scan_filtered.pcd");

  // Step 3: Convert PCD to topdown RGB image both with filtered and full 
  TopDownResult filtered_result = pcdToTopdownImageWithHeight(filtered, 256);
  saveImage(filtered_result.image, "/tmp/cw2_topdown.png"); 

  TopDownResult full_result = pcdToTopdownImageWithHeight(combined, 256);
  saveImage(full_result.image, "/tmp/cw2_full_topdown.png");

  // Step 4: Detect basket region (for later classification)
  cv::Mat basket_mask = detectBasketRegion(full_result.image, full_result.heightMap);
  saveImage(basket_mask, "/tmp/cw2_basket_mask.png");

  // Step 5: White background + filter robot
  filterRobot(filtered_result.image, filtered_result.heightMap, 40);
  makeWhiteBackground(filtered_result.image);
  saveImage(filtered_result.image, "/tmp/cw2_processed.png");

  // Step 6: Cluster the image and print info about each cluster
  ClusterResult cluster_result = clusterImage(filtered_result.image, filtered_result.heightMap);
  // Print cluster info
  for (const auto& cluster : cluster_result.clusters)
  {    RCLCPP_INFO(node_->get_logger(),
      "Cluster %d: size=%d centroid=(%.1f, %.1f) height=%.3f color=(B=%.1f, G=%.1f, R=%.1f)",
      cluster.label,
      cluster.size,
      cluster.centroid.x,
      cluster.centroid.y,
      cluster.height,
      cluster.color[0],
      cluster.color[1],
      cluster.color[2]);
  }

  // Step 7: Classify clusters and print results
  std::vector<ClusterInfo> classified = classifyClusters(cluster_result.clusters, cluster_result.labels, basket_mask);
  for (const auto& cluster : classified)
  {
      RCLCPP_INFO(node_->get_logger(),
          "Cluster %d classified as '%s': size=%d centroid=(%.1f, %.1f) height=%.3f color=(B=%.1f, G=%.1f, R=%.1f)",
          cluster.label,
          cluster.type.c_str(),
          cluster.size,
          cluster.centroid.x,
          cluster.centroid.y,
          cluster.height,
          cluster.color[0],
          cluster.color[1],
          cluster.color[2]);
  }

  // Step 8: Plan mission 
  MissionPlan mission = planMission(classified);

  // print mission plan
  if (mission.valid)
  {    
    RCLCPP_INFO(node_->get_logger(),
        "Mission plan: pick at (%.1f, %.1f) and place at (%.1f, %.1f)",
        mission.objectPose.x,
        mission.objectPose.y,
        mission.goalPose.x,
        mission.goalPose.y);

  }
  else  {
      RCLCPP_WARN(node_->get_logger(), "Mission plan is invalid");
  } 


  (void)request;
  response->total_num_shapes = 0;
  response->num_most_common_shape = 0;
  response->most_common_shape_vector.clear(); 
}