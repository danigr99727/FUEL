#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <memory>
#include <random>
#include <unordered_map>

using std::shared_ptr;
using std::normal_distribution;
using std::default_random_engine;

namespace fast_planner {

class SDFMap;

class MapROS {
public:
  MapROS();
  ~MapROS();
  void setMap(SDFMap* map);
  void init();

private:
  //void findPeople();

  std::tuple<Eigen::Matrix<double, 3, 1>, Eigen::Quaterniond> ProcessPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
  std::tuple<Eigen::Matrix<double, 3, 1>, Eigen::Quaterniond> ProcessPose(const geometry_msgs::TransformStamped::ConstPtr &msg);
  std::tuple<Eigen::Matrix<double, 3, 1>, Eigen::Quaterniond> ProcessPose(const nav_msgs::Odometry::ConstPtr &msg);

  void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                         const geometry_msgs::PoseStampedConstPtr& pose);
  void cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                         const geometry_msgs::PoseStampedConstPtr& pose);
  void depthTransformCallback(const sensor_msgs::ImageConstPtr& img,
                         const geometry_msgs::TransformStampedConstPtr& pose);
    void processDepthMsg(const sensor_msgs::ImageConstPtr& img);

    void semanticsDepthTransformCallback(const sensor_msgs::ImageConstPtr& semanticsMsg,
                                  const sensor_msgs::ImageConstPtr& depthMsg,
                                  const geometry_msgs::TransformStampedConstPtr& poseMsg);
  void semanticsDepthOdomCallback(const sensor_msgs::ImageConstPtr& semanticsMsg,
                                  const sensor_msgs::ImageConstPtr& depthMsg,
                                  const nav_msgs::OdometryConstPtr& odomMsg);
    void depthOdomCallback(const sensor_msgs::ImageConstPtr& depthMsg,
                           const nav_msgs::OdometryConstPtr& odomMsg);
  void updateESDFCallback(const ros::TimerEvent& /*event*/);
  void visCallback(const ros::TimerEvent& /*event*/);

  void publishMapAll();
  void publishMapLocal();
  void publishESDF();
  void publishUpdateRange();
  void publishDepth();

  void processDepthImage();

  std::unordered_map<int, Eigen::Vector3i> get_color_map(int N);
    std::unordered_map<int, Eigen::Vector3i> color_map_;
    SDFMap* map_;
  // may use ExactTime?
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::TransformStamped>
            SyncPolicyImageTransform;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageTransform>> SynchronizerImageTransform;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::TransformStamped>
            SyncPolicySemanticsImageTransform;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicySemanticsImageTransform>> SynchronizerSemanticsImageTransform;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, nav_msgs::Odometry>
            SyncPolicySemanticsImageOdom;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicySemanticsImageOdom>> SynchronizerSemanticsImageOdom;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
            SyncPolicyImageOdom;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                          geometry_msgs::PoseStamped> SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> semantic_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::TransformStamped>> transform_sub_;

    SynchronizerImagePose sync_image_pose_;
    SynchronizerSemanticsImageOdom sync_semantics_image_odom_;
    SynchronizerImageOdom sync_image_odom_;
    SynchronizerCloudPose sync_cloud_pose_;
    SynchronizerImageTransform sync_image_transform_;
    SynchronizerSemanticsImageTransform sync_semantics_image_transform_;

  ros::Publisher map_local_pub_, map_local_inflate_pub_, esdf_pub_, map_all_pub_, unknown_pub_,
      update_range_pub_, depth_pub_, semantic_map_pub_, semantic_color_map_pub_, sent_pub_, received_pub_;
  ros::Timer esdf_timer_, vis_timer_;

  // params, depth projection
  double cx_, cy_, fx_, fy_;
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;
  string frame_id_;
  // msg publication
  double esdf_slice_height_;
  double visualization_truncate_height_, visualization_truncate_low_;
  bool show_esdf_time_, show_occ_time_;
  bool show_all_map_;
  bool do_semantics_;
  bool input_rdf_;
  std::string pose_type_;
  int image_rows_;
  int image_cols_;
  // data
  // flags of map state
  bool local_updated_, esdf_need_update_;
  // input
  Eigen::Vector3d camera_pos_;
  Eigen::Quaterniond camera_q_;
  unique_ptr<cv::Mat> depth_image_;
  unique_ptr<cv::Mat> semantic_image_;
  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int fuse_num_, esdf_num_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;
  pcl::PointCloud<pcl::PointXYZL> semantic_point_cloud_;

  normal_distribution<double> rand_noise_;
  default_random_engine eng_;

  ros::Time map_start_time_;

  friend SDFMap;
};
}

#endif