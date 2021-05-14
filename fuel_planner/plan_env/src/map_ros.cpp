#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <fstream>

namespace fast_planner {
MapROS::MapROS() {
}

MapROS::~MapROS() {
}

void MapROS::setMap(SDFMap* map) {
  this->map_ = map;
}

void MapROS::init() {
  node_.param("map_ros/fx", fx_, -1.0);
  node_.param("map_ros/fy", fy_, -1.0);
  node_.param("map_ros/cx", cx_, -1.0);
  node_.param("map_ros/cy", cy_, -1.0);
  node_.param("map_ros/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
  node_.param("map_ros/depth_filter_mindist", depth_filter_mindist_, -1.0);
  node_.param("map_ros/depth_filter_margin", depth_filter_margin_, -1);
  node_.param("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_, -1.0);
  node_.param("map_ros/skip_pixel", skip_pixel_, -1);

  node_.param("map_ros/esdf_slice_height", esdf_slice_height_, -0.1);
  node_.param("map_ros/visualization_truncate_height", visualization_truncate_height_, -0.1);
  node_.param("map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);
  node_.param("map_ros/show_occ_time", show_occ_time_, false);
  node_.param("map_ros/show_esdf_time", show_esdf_time_, false);
  node_.param("map_ros/show_all_map", show_all_map_, false);
  node_.param("map_ros/do_semantics", do_semantics_, true);
  node_.param("map_ros/pose_type", pose_type_, string("odometry"));
  node_.param("map_ros/input_rows", input_rows_, 480);
  node_.param("map_ros/input_cols", input_cols_, 640);
  node_.param("map_ros/image_rows", image_rows_, 480);
  node_.param("map_ros/image_cols", image_cols_, 640);

  node_.param("map_ros/frame_id", frame_id_, string("world"));

  color_map_ = get_color_map(256);

  proj_points_.resize(image_cols_ * image_rows_ / (skip_pixel_ * skip_pixel_));
  point_cloud_.points.resize(image_cols_ * image_rows_ / (skip_pixel_ * skip_pixel_));
  semantic_point_cloud_.points.resize(image_cols_ * image_rows_ / (skip_pixel_ * skip_pixel_));

  proj_points_cnt = 0;

  local_updated_ = false;
  esdf_need_update_ = false;
  fuse_time_ = 0.0;
  esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
  max_esdf_time_ = 0.0;
  fuse_num_ = 0;
  esdf_num_ = 0;
  depth_image_.reset(new cv::Mat);
  semantic_image_.reset(new cv::Mat);

  rand_noise_ = normal_distribution<double>(0, 0.1);
  random_device rd;
  eng_ = default_random_engine(rd());

  esdf_timer_ = node_.createTimer(ros::Duration(0.1), &MapROS::updateESDFCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.1), &MapROS::visCallback, this);

  map_all_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 10);
  map_local_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 10);
  map_local_inflate_pub_ =
      node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);
  depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);

  if(do_semantics_){
      semantic_map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/semantic_occupancy", 10);
      semantic_color_map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/semantic_color_occupancy", 10);
  }

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/map_ros/depth", 50));
  semantic_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/map_ros/semantics", 10));
    cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "/map_ros/cloud", 50));
  pose_sub_.reset(
      new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/map_ros/posenot", 25));
  transform_sub_.reset(
            new message_filters::Subscriber<geometry_msgs::TransformStamped>(node_, "/map_ros/pose", 25));
  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/odom_world", 25));

    if (do_semantics_ && pose_type_=="transform"){
      sync_semantics_image_transform_.reset(new message_filters::Synchronizer<MapROS::SyncPolicySemanticsImageTransform>(
              MapROS::SyncPolicySemanticsImageTransform(100), *semantic_sub_,*depth_sub_, *transform_sub_));
      sync_semantics_image_transform_->registerCallback(boost::bind(&MapROS::semanticsDepthTransformCallback, this, _1, _2, _3));
  }
  else if (!do_semantics_ && pose_type_=="transform"){
      sync_image_transform_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImageTransform>(
              MapROS::SyncPolicyImageTransform(100), *depth_sub_, *transform_sub_));
      sync_image_transform_->registerCallback(boost::bind(&MapROS::depthTransformCallback, this, _1, _2));
  }
  else if (!do_semantics_ && pose_type_=="pose"){
      sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
              MapROS::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
      sync_image_pose_->registerCallback(boost::bind(&MapROS::depthPoseCallback, this, _1, _2));
  }

  else if (do_semantics_ && pose_type_=="odometry"){
    sync_semantics_image_odom_.reset(new message_filters::Synchronizer<MapROS::SyncPolicySemanticsImageOdom>(
          MapROS::SyncPolicySemanticsImageOdom(100), *semantic_sub_,*depth_sub_, *odom_sub_));
    sync_semantics_image_odom_->registerCallback(boost::bind(&MapROS::semanticsDepthOdomCallback, this, _1, _2, _3));
  }

  sync_cloud_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(
      MapROS::SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
  sync_cloud_pose_->registerCallback(boost::bind(&MapROS::cloudPoseCallback, this, _1, _2));

  map_start_time_ = ros::Time::now();
}

std::map<int, Eigen::Vector3i> MapROS::get_color_map(int N=256) {
// Return Color Map in PASCAL VOC format
    auto bitget = [](uint32_t byte_val, uint8_t idx){return (byte_val & (1<<idx)) !=0;};
    uint8_t r, g, b;
    std::map<int, Eigen::Vector3i> color_map;
    for(int i=0 ; i<N; i++){
        r = 0; g = 0; b = 0;
        int c = i;
        for(uint8_t j = 0; j<8; j++){
            r = r | (bitget(c, 0) << 7 - j);
            g = g | (bitget(c, 1) << 7 - j);
            b = b | (bitget(c, 2) << 7 - j);
            c = c >> 3;
        }
        color_map.insert(std::make_pair(i, Eigen::Vector3i(r, g, b)));

    }
    return color_map;
}


void MapROS::visCallback(const ros::TimerEvent& e) {
  publishMapLocal();
    if (show_all_map_) {
    // Limit the frequency of all map
    static double tpass = 0.0;
    tpass += (e.current_real - e.last_real).toSec();
    if (tpass > 0.1) {
      publishMapAll();
      tpass = 0.0;
    }
  }
  // publishUnknown();
  // publishESDF();

  // publishUpdateRange();
  // publishDepth();
}

void MapROS::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  if (!esdf_need_update_) return;
  auto t1 = ros::Time::now();

  map_->updateESDF3d();
  esdf_need_update_ = false;

  auto t2 = ros::Time::now();
  esdf_time_ += (t2 - t1).toSec();
  max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());
  esdf_num_++;
  if (show_esdf_time_)
    ROS_WARN("ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), esdf_time_ / esdf_num_,
             max_esdf_time_);
}

std::tuple<Eigen::Matrix<double, 3, 1>, Eigen::Quaterniond> MapROS::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)  {
    Eigen::Matrix<double, 3, 1> pos;
    Eigen::Quaterniond q;
    pos = Eigen::Matrix<double, 3, 1>(msg->pose.position.x,
                          msg->pose.position.y,
                          msg->pose.position.z);
    q = Eigen::Quaterniond(msg->pose.orientation.w,
                           msg->pose.orientation.x,
                           msg->pose.orientation.y,
                           msg->pose.orientation.z);
    return (std::make_tuple(pos, q));
}

std::tuple<Eigen::Matrix<double, 3, 1>, Eigen::Quaterniond> MapROS::PoseCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)  {
    Eigen::Matrix<double, 3, 1> pos;
    Eigen::Quaterniond q;
    static int stat = 0;
    pos = Eigen::Matrix<double, 3, 1>(msg->transform.translation.x,
                          msg->transform.translation.y-1,
                          msg->transform.translation.z);
    q = Eigen::Quaterniond(msg->transform.rotation.w,
                           msg->transform.rotation.x,
                           msg->transform.rotation.y,
                           msg->transform.rotation.z);
    Eigen::Matrix3d R_rdf_to_flu;
    R_rdf_to_flu << 0.0f, 0.0f, 1.0f,
            -1.0f, 0.0f, 0.0f,
            0.0f, -1.0f, 0.0f;
    Eigen::Quaterniond q_rdf_to_flu(R_rdf_to_flu);
    q = q_rdf_to_flu * q;
    pos = q_rdf_to_flu * pos;
    return (std::make_tuple(pos, q));
}

std::tuple<Eigen::Matrix<double, 3, 1>, Eigen::Quaterniond> MapROS::PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)  {
    Eigen::Matrix<double, 3, 1> pos;
    Eigen::Quaterniond q;
    static int stat = 0;
    pos = Eigen::Vector3d(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
    q = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                           msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z);
    //NED: NORTH, EAST, DOWN (aka, forward, right, down?) ENU: EAST NORTH UP (aka, right, forward, up)
    Eigen::Matrix3d R_rdf_to_flu;
    R_rdf_to_flu << 0.0f, 0.0f, 1.0f,
            -1.0f, 0.0f, 0.0f,
            0.0f, -1.0f, 0.0f;
    q = R_rdf_to_flu * q;
    pos = R_rdf_to_flu * pos;
    return (std::make_tuple(pos, q));
}

void MapROS::depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  std::tie(camera_pos_, camera_q_) = PoseCallback(pose);
    processDepthMsg(img);
}



void MapROS::cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  std::tie(camera_pos_, camera_q_) = PoseCallback(pose);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  int num = cloud.points.size();

  map_->inputPointCloud(cloud, num, camera_pos_);

  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }
}

void MapROS::depthTransformCallback(const sensor_msgs::ImageConstPtr& img,
                               const geometry_msgs::TransformStampedConstPtr& pose) {
    std::tie(camera_pos_, camera_q_) = PoseCallback(pose);
    processDepthMsg(img);
}

void MapROS::processDepthMsg(const sensor_msgs::ImageConstPtr& img)
{
    if (!map_->isInMap(camera_pos_))  // exceed mapped region
        return;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
    //cv::resize(cv_ptr->image, *depth_image_, cv::Size(image_rows_, image_cols_));
    cv_ptr->image.copyTo(*depth_image_);

    auto t1 = ros::Time::now();

    // generate point cloud, update map
    processDepthImage();
    if(do_semantics_)
    {
        map_->inputSemanticCloud(semantic_point_cloud_, proj_points_cnt, camera_pos_);
    }
    else
    {
        map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
    }
    if (local_updated_) {

        map_->clearAndInflateLocalMap();
        esdf_need_update_ = true;
        local_updated_ = false;
    }

    auto t2 = ros::Time::now();
    fuse_time_ += (t2 - t1).toSec();
    max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
    fuse_num_ += 1;
    if (show_occ_time_)
        ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), fuse_time_ / fuse_num_,
                 max_fuse_time_);
}

void MapROS::semanticsDepthTransformCallback(const sensor_msgs::ImageConstPtr& semanticsMsg,
                                        const sensor_msgs::ImageConstPtr& depthMsg,
                                        const geometry_msgs::TransformStampedConstPtr& poseMsg){
    cv_bridge::CvImagePtr semantics_cv_ptr = cv_bridge::toCvCopy(semanticsMsg, semanticsMsg->encoding);
    semantics_cv_ptr->image.copyTo(*semantic_image_);
    depthTransformCallback(depthMsg, poseMsg);
}

void MapROS::semanticsDepthOdomCallback(const sensor_msgs::ImageConstPtr& semanticsMsg,
                                const sensor_msgs::ImageConstPtr& depthMsg,
                                const nav_msgs::OdometryConstPtr& odomMsg){
    std::tie(camera_pos_, camera_q_) = PoseCallback(odomMsg);
    cv_bridge::CvImagePtr semantics_cv_ptr = cv_bridge::toCvCopy(semanticsMsg, semanticsMsg->encoding);
    semantics_cv_ptr->image.copyTo(*semantic_image_);
    processDepthMsg(depthMsg);
}

void MapROS::processDepthImage() {
  proj_points_cnt = 0;

  uint16_t* row_ptr;
  uint8_t* semantic_row_ptr;
  int cols = depth_image_->cols;
  int rows = depth_image_->rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / k_depth_scaling_factor_;
  for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) {
    row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
    if(do_semantics_)
    {
        semantic_row_ptr = semantic_image_->ptr<uint8_t>(v);
    }
    for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) {

        depth = (*row_ptr) * inv_factor;
        row_ptr = row_ptr + skip_pixel_;

      // // filter depth
      // if (depth > 0.01)
      //   depth += rand_noise_(eng_);

      // TODO: simplify the logic here
      if (*row_ptr == 0 || depth > depth_filter_maxdist_)
        depth = depth_filter_maxdist_;
      else if (depth < depth_filter_mindist_)
        continue;

      pt_cur(0) = (u - cx_) * depth / fx_;
      pt_cur(1) = (v - cy_) * depth / fy_;
      pt_cur(2) = depth;
      pt_world = camera_r * pt_cur + camera_pos_;

      //uu = coord.x()*fx_/coord.z() + cx_;
      //vv = coord.y()*fy_/coord.z() + cy_;
      //const double depth_filter_tolerance = 0.1;

      if(!do_semantics_){
          auto& pt = point_cloud_.points[proj_points_cnt++];
          pt.x = static_cast<float>(pt_world[0]);
          pt.y = static_cast<float>(pt_world[1]);
          pt.z = static_cast<float>(pt_world[2]);
      }
      else{
          auto& pt = semantic_point_cloud_.points[proj_points_cnt++];
          pt.x = static_cast<float>(pt_world[0]);
          pt.y = static_cast<float>(pt_world[1]);
          pt.z = static_cast<float>(pt_world[2]);
          pt.label = static_cast<uint32_t>(*semantic_row_ptr);
          semantic_row_ptr = semantic_row_ptr + skip_pixel_;
      }
    }
  }

    publishDepth();
}

void MapROS::publishMapAll() {
  pcl::PointXYZRGBL RGBLpt;
  pcl::PointXYZRGB RGBpt;
  //pcl::PointXYZL Lpt;
  pcl::PointXYZ pt;
  Eigen::Vector3i rgb;
  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::PointCloud<pcl::PointXYZRGB> rgbSemanticCloud;
  std::set<int> seen_labels;
  //pcl::PointCloud<pcl::PointXYZL> semanticCloud;
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          //if (pos(2) > visualization_truncate_height_) continue;
          //if (pos(2) < visualization_truncate_low_) continue;
          RGBLpt.x = pos(0);
          RGBLpt.y = pos(1);
          RGBLpt.z = pos(2);
          if(do_semantics_){
              seen_labels.emplace(RGBLpt.label);
              RGBLpt.label = map_->md_->semantics_buffer_[map_->toAddress(x,y,z)];
              rgb = color_map_[static_cast<int>(RGBLpt.label)];
              RGBLpt.r = rgb(0);
              RGBLpt.g = rgb(1);
              RGBLpt.b = rgb(2);
              pcl::copyPoint(RGBLpt, RGBpt);
              rgbSemanticCloud.push_back(RGBpt);
              //pcl::copyPoint(RGBLpt, Lpt);
              //semanticCloud.push_back(Lpt);
          }
          pcl::copyPoint(RGBLpt, pt);
          cloud1.push_back(pt);
        }
      }
  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_all_pub_.publish(cloud_msg);

  if(do_semantics_) {
      //for(int seen_label : seen_labels){
      //    std::cout<<"LABEL: "<<seen_label<<" COLORS: "<<color_map_[seen_label](0)<<" "<<color_map_[seen_label](1)<<" "<<color_map_[seen_label](2)<<std::endl;
      //}
      rgbSemanticCloud.width = rgbSemanticCloud.points.size();
      rgbSemanticCloud.height = 1;
      rgbSemanticCloud.is_dense = true;
      rgbSemanticCloud.header.frame_id = frame_id_;
      pcl::toROSMsg(rgbSemanticCloud, cloud_msg);
      semantic_color_map_pub_.publish(cloud_msg);
  }
  // Output time and known volumn
  /*double time_now = (ros::Time::now() - map_start_time_).toSec();
  double known_volumn = 0;

  for (int x = map_->mp_->box_min_(0) ; x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1) ; y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2) ; z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->clamp_min_log_ - 1e-3)
          known_volumn += 0.1 * 0.1 * 0.1;
      }

  ofstream file("/home/boboyu/workspaces/plan_ws/src/fast_planner/exploration_manager/resource/"
                "curve1.txt",
                ios::app);
  file << "time:" << time_now << ",vol:" << known_volumn << std::endl; */
}

void MapROS::publishMapLocal() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  // for (int z = min_cut(2); z <= max_cut(2); ++z)
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          // Occupied cells
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          //if (pos(2) > visualization_truncate_height_) continue;
          //if (pos(2) < visualization_truncate_low_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  cloud2.width = cloud2.points.size();
  cloud2.height = 1;
  cloud2.is_dense = true;
  cloud2.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_local_pub_.publish(cloud_msg);
  pcl::toROSMsg(cloud2, cloud_msg);
  map_local_inflate_pub_.publish(cloud_msg);
}


void MapROS::publishDepth() {
  if(!do_semantics_){
      pcl::PointCloud<pcl::PointXYZ> cloud;
      for (int i = 0; i < proj_points_cnt; ++i) {
        cloud.push_back(point_cloud_.points[i]);
      }
      cloud.width = cloud.points.size();
      cloud.height = 1;
      cloud.is_dense = true;
      cloud.header.frame_id = frame_id_;
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(cloud, cloud_msg);
      depth_pub_.publish(cloud_msg);
  }
  else{
      pcl::PointXYZRGB pt;
      Eigen::Vector3i rgb;
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      for (int i = 0; i < proj_points_cnt; ++i) {
          rgb = color_map_[static_cast<int>(semantic_point_cloud_.points[i].label)];
          pcl::copyPoint(semantic_point_cloud_.points[i], pt);
          pt.r = rgb(0);
          pt.g = rgb(1);
          pt.b = rgb(2);
          cloud.push_back(pt);
      }
      cloud.width = cloud.points.size();
      cloud.height = 1;
      cloud.is_dense = true;
      cloud.header.frame_id = frame_id_;
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(cloud, cloud_msg);
      depth_pub_.publish(cloud_msg);
  }
}

void MapROS::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
  map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void MapROS::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_);
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_);
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      Eigen::Vector3d pos;
      map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = esdf_slice_height_;
      dist = map_->getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}
}