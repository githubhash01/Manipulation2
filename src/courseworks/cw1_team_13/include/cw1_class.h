/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_
#include <mutex>
#include <sensor_msgs/msg/point_cloud2.hpp>


// system includes
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "cw1_world_spawner/srv/task1_service.hpp"
#include "cw1_world_spawner/srv/task2_service.hpp"
#include "cw1_world_spawner/srv/task3_service.hpp"

//NOTE included
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class cw1
{
public:

  /* ----- class member functions ----- */

  // constructor
  explicit cw1(const rclcpp::Node::SharedPtr &node);

  // service callbacks for tasks 1, 2, and 3
  void t1_callback(
    const std::shared_ptr<cw1_world_spawner::srv::Task1Service::Request> request,
    std::shared_ptr<cw1_world_spawner::srv::Task1Service::Response> response);
  void t2_callback(
    const std::shared_ptr<cw1_world_spawner::srv::Task2Service::Request> request,
    std::shared_ptr<cw1_world_spawner::srv::Task2Service::Response> response);
  void t3_callback(
    const std::shared_ptr<cw1_world_spawner::srv::Task3Service::Request> request,
    std::shared_ptr<cw1_world_spawner::srv::Task3Service::Response> response);

  /* ----- class member variables ----- */

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<cw1_world_spawner::srv::Task1Service>::SharedPtr t1_service_;
  rclcpp::Service<cw1_world_spawner::srv::Task2Service>::SharedPtr t2_service_;
  rclcpp::Service<cw1_world_spawner::srv::Task3Service>::SharedPtr t3_service_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // Sensor callback state bookkeeping for template diagnostics.
  std::atomic<int64_t> latest_joint_state_stamp_ns_{0};
  std::atomic<uint64_t> joint_state_msg_count_{0};
  std::atomic<int64_t> latest_cloud_stamp_ns_{0};
  std::atomic<uint64_t> cloud_msg_count_{0};

  // Runtime parameters (compatibility scaffold with cw1_team_0).
  bool enable_cloud_viewer_ = false;
  bool move_home_on_start_ = false;
  bool use_path_constraints_ = false;
  bool use_cartesian_reach_ = false;
  bool allow_position_only_fallback_ = false;
  bool publish_programmatic_debug_ = false;
  bool enable_task1_snap_ = false;
  bool return_home_between_pick_place_ = false;
  bool return_home_after_pick_place_ = false;
  bool task2_capture_enabled_ = false;

  double cartesian_eef_step_ = 0.005;
  double cartesian_jump_threshold_ = 0.0;
  double cartesian_min_fraction_ = 0.98;
  double pick_offset_z_ = 0.12;
  double task3_pick_offset_z_ = 0.13;
  double place_offset_z_ = 0.35;
  double grasp_approach_offset_z_ = 0.015;
  double post_grasp_lift_z_ = 0.05;
  double gripper_grasp_width_ = 0.03;
  double joint_state_wait_timeout_sec_ = 2.0;

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_group_;
  bool moveit_initialized_ = false;
  void initMoveit();
  
  bool moveToTarget(geometry_msgs::msg::Pose target);
  bool openGripper();
  bool closeGripper();
  bool moveCartesian(geometry_msgs::msg::Pose target);

  void pickAndPlace(float box_x, float box_y, float box_z,
                       float basket_x, float basket_y, float basket_z);


  // Point Cloud helpers

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_msg_;
  std::mutex cloud_mutex_;

  struct BoxesAndBaskets {
    struct ColoredLocation {
      std::string color;
      float x, y, z;
    };
    std::vector<ColoredLocation> boxes;
    std::vector<ColoredLocation> baskets;
  };

  void moveHome(); 
  sensor_msgs::msg::PointCloud2::SharedPtr getLatestCloud();
  void savePointCloudPCD(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const std::string &filename);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr capturePointCloudAtTarget(
    geometry_msgs::msg::Pose target);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combinePointClouds(
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scanTable();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterTableFromPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
  std::string classifyCentroid(
    float cx, float cy, float cz,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
  BoxesAndBaskets detectShapes(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &filtered_cloud,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &full_cloud);
  BoxesAndBaskets vision_pipeline();


};

#endif // end of include guard for CW1_CLASS_H_
