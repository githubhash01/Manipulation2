/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire
solution is contained within the cw2_team_<your_team_number> package */

#ifndef CW2_CLASS_H_
#define CW2_CLASS_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>   // for doTransform on PointCloud2

#include <opencv2/opencv.hpp>

#include <Eigen/Geometry>

#include "cw2_world_spawner/srv/task1_service.hpp"
#include "cw2_world_spawner/srv/task2_service.hpp"
#include "cw2_world_spawner/srv/task3_service.hpp"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

///////////////////////////////////////////////////////////////////////////////
// Free helper types (used by free helper functions defined in cw2_class.cpp)
///////////////////////////////////////////////////////////////////////////////

struct ClusterInfo
{
  int label;
  int size;
  cv::Point2f centroid;
  float height;
  cv::Vec3f color;
  std::string type;
};

struct ClusterResult
{
  std::vector<ClusterInfo> clusters;
  cv::Mat labels;
};

struct MissionPlan
{
  bool valid = false;
  // cv::Point2f objectPose;
  // cv::Point2f goalPose;
  geometry_msgs::msg::Pose objectPose;
  geometry_msgs::msg::Pose goalPose;
  std::string objectType;
};

///////////////////////////////////////////////////////////////////////////////
// Free helper function declarations (defined in cw2_class.cpp)
///////////////////////////////////////////////////////////////////////////////

// Default argument values are specified in the definition in cw2_class.cpp.
geometry_msgs::msg::Pose createPose(
  float x,
  float y,
  float z,
  float qx,
  float qy,
  float qz,
  float qw);

cv::Mat detectBasketRegion(const cv::Mat &imgFull, const cv::Mat &heightMap);

// Default value for radius is specified in the definition in cw2_class.cpp.
void filterRobot(cv::Mat &img, cv::Mat &heightMap, int radius);

void makeWhiteBackground(cv::Mat &img);

ClusterResult clusterImage(const cv::Mat &img, const cv::Mat &heightMap);

std::vector<ClusterInfo> classifyClusters(
  const std::vector<ClusterInfo> &clusters,
  const cv::Mat &labels,
  const cv::Mat &basketMask);

// MissionPlan planMission(const std::vector<ClusterInfo> &clusters);

///////////////////////////////////////////////////////////////////////////////
// cw2 class
///////////////////////////////////////////////////////////////////////////////

class cw2
{
public:
  // Nested result type for pcdToTopdownImageWithHeight
  struct TopDownResult
  {
    cv::Mat image;
    cv::Mat heightMap;
    float minX;
    float minY;
    float maxX;
    float maxY;
    int   gridSize;
  };

  explicit cw2(const rclcpp::Node::SharedPtr &node);

  void t1_callback(
    const std::shared_ptr<cw2_world_spawner::srv::Task1Service::Request> request,
    std::shared_ptr<cw2_world_spawner::srv::Task1Service::Response> response);
  void t2_callback(
    const std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> request,
    std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> response);
  void t3_callback(
    const std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> request,
    std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> response);

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<cw2_world_spawner::srv::Task1Service>::SharedPtr t1_service_;
  rclcpp::Service<cw2_world_spawner::srv::Task2Service>::SharedPtr t2_service_;
  rclcpp::Service<cw2_world_spawner::srv::Task3Service>::SharedPtr t3_service_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr color_cloud_sub_;
  rclcpp::CallbackGroup::SharedPtr pointcloud_callback_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex cloud_mutex_;
  PointCPtr g_cloud_ptr;
  // New
  sensor_msgs::msg::PointCloud2::ConstSharedPtr g_cloud_msg_;
  std::uint64_t g_cloud_sequence_ = 0;
  std::string g_input_pc_frame_id_;

  std::string pointcloud_topic_;
  bool pointcloud_qos_reliable_ = false;

private:
  bool moveit_initialized_ = false;
  void initMoveit();

  // Motion helpers
  bool moveToTarget(geometry_msgs::msg::Pose target);
  bool openGripper();
  bool closeGripper();
  bool moveCartesian(geometry_msgs::msg::Pose target);

  // Point cloud / image I/O helpers
  void savePointCloudPCD(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const std::string &filename);
  void saveImage(
    const cv::Mat &image,
    const std::string &filename);

  // Point cloud capture / processing helpers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr capturePointCloudAtTarget(
    geometry_msgs::msg::Pose target);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combinePointClouds(
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scanTable();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterTableFromPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

  // Top-down projection helper 
  TopDownResult pcdToTopdownImageWithHeight(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    int gridSize);

  // Full pipeline 
  MissionPlan perception_pipeline(); 
};

#endif  // CW2_CLASS_H_