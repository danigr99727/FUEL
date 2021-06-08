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

class Person{
    //frontier
    //pixels
    //velocity vector
    //position vector
    //acceleration vector??
};

/*void MapROS::findPeople(){
    std::vector<Person> people;
    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {

            }
}*/

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
  node_.param("map_ros/do_semantics", do_semantics_, false);
    node_.param("map_ros/input_rdf", input_rdf_, true);
  node_.param("map_ros/pose_type", pose_type_, string("odometry"));
  node_.param("map_ros/image_rows", image_rows_, 240);
  node_.param("map_ros/image_cols", image_cols_, 320);

  node_.param("map_ros/frame_id", frame_id_, string("world"));
image_rows_=240;
image_cols_=320;
  color_map_ = get_color_map(256);

  proj_points_.resize(image_cols_ * image_rows_ / (skip_pixel_ * skip_pixel_));
  point_cloud_.points.resize(image_cols_ * image_rows_ / (skip_pixel_ * skip_pixel_));
  if(do_semantics_) {
      semantic_point_cloud_.points.resize(image_cols_ * image_rows_ / (skip_pixel_ * skip_pixel_));
  }
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

  map_all_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 100);
  map_local_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 100);
  map_local_inflate_pub_ =
      node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local_inflate", 100);
  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 100);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 100);
  update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 100);
  depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 100);

  received_pub_ = node_.advertise<std_msgs::Header>("/sdf_map/received", 500);
  sent_pub_ = node_.advertise<std_msgs::Header>("/sdf_map/sent", 500);


    if(do_semantics_){
      semantic_color_map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/semantic_color_occupancy", 150);
  }

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/map_ros/depth", 150));
  semantic_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/map_ros/semantics", 150));
    cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "/map_ros/cloud", 150));
  pose_sub_.reset(
      new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/map_ros/posenot", 150));
  transform_sub_.reset(
            new message_filters::Subscriber<geometry_msgs::TransformStamped>(node_, "/map_ros/pose", 150));
  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/odom_world", 150));

    if (do_semantics_ && pose_type_=="transform"){
      sync_semantics_image_transform_.reset(new message_filters::Synchronizer<MapROS::SyncPolicySemanticsImageTransform>(
              MapROS::SyncPolicySemanticsImageTransform(150), *semantic_sub_,*depth_sub_, *transform_sub_));
      sync_semantics_image_transform_->registerCallback(boost::bind(&MapROS::semanticsDepthTransformCallback, this, _1, _2, _3));
  }
  else if (!do_semantics_ && pose_type_=="transform"){
      sync_image_transform_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImageTransform>(
              MapROS::SyncPolicyImageTransform(150), *depth_sub_, *transform_sub_));
      sync_image_transform_->registerCallback(boost::bind(&MapROS::depthTransformCallback, this, _1, _2));
  }
  else if (!do_semantics_ && pose_type_=="pose"){
      sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
              MapROS::SyncPolicyImagePose(150), *depth_sub_, *pose_sub_));
      sync_image_pose_->registerCallback(boost::bind(&MapROS::depthPoseCallback, this, _1, _2));
  }

  else if (do_semantics_ && pose_type_=="odometry"){
    sync_semantics_image_odom_.reset(new message_filters::Synchronizer<MapROS::SyncPolicySemanticsImageOdom>(
          MapROS::SyncPolicySemanticsImageOdom(150), *semantic_sub_,*depth_sub_, *odom_sub_));
    sync_semantics_image_odom_->registerCallback(boost::bind(&MapROS::semanticsDepthOdomCallback, this, _1, _2, _3));
  }

  else if (!do_semantics_ && pose_type_=="odometry"){
        sync_image_odom_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImageOdom>(
                MapROS::SyncPolicyImageOdom(150), *depth_sub_, *odom_sub_));
        sync_image_odom_->registerCallback(boost::bind(&MapROS::depthOdomCallback, this, _1, _2));
    }

  sync_cloud_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(
      MapROS::SyncPolicyCloudPose(150), *cloud_sub_, *pose_sub_));
  sync_cloud_pose_->registerCallback(boost::bind(&MapROS::cloudPoseCallback, this, _1, _2));

  map_start_time_ = ros::Time::now();
}

std::unordered_map<int, Eigen::Vector3i> MapROS::get_color_map(int N=256) {
// Return Color Map in PASCAL VOC format
    auto bitget = [](uint32_t byte_val, uint8_t idx){return (byte_val & (1<<idx)) !=0;};
    uint8_t r, g, b;
    std::unordered_map<int, Eigen::Vector3i> color_map;
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
  publishESDF();

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

std::tuple<Eigen::Matrix<double, 3, 1>, Eigen::Quaterniond> MapROS::ProcessPose(const geometry_msgs::PoseStamped::ConstPtr &msg)  {
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

std::tuple<Eigen::Matrix<double, 3, 1>, Eigen::Quaterniond> MapROS::ProcessPose(const geometry_msgs::TransformStamped::ConstPtr &msg)  {
    Eigen::Matrix<double, 3, 1> pos;
    Eigen::Quaterniond q;
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

std::tuple<Eigen::Matrix<double, 3, 1>, Eigen::Quaterniond> MapROS::ProcessPose(const nav_msgs::Odometry::ConstPtr &msg)  {
    Eigen::Matrix<double, 3, 1> pos;
    Eigen::Quaterniond q;

    Eigen::Matrix4d Pose_receive = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d cam02body;
    Eigen::Matrix4d cam2world;
    cam02body <<
    1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, -1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0;
    //cam02body=cam02body*cam02body; //for when using ORB, assuming its output is RDF
    cam2world = Eigen::Matrix4d::Identity();

    Eigen::Quaterniond cam2world_quat;
    Eigen::Vector3d request_position;
    Eigen::Quaterniond request_pose;
    if(input_rdf_){
        request_position.x() = msg->pose.pose.position.x;
        request_position.y() = msg->pose.pose.position.z;
        request_position.z() = -msg->pose.pose.position.y;
        request_pose.x() = msg->pose.pose.orientation.x;
        request_pose.y() = msg->pose.pose.orientation.z;
        request_pose.z() = -msg->pose.pose.orientation.y;
        request_pose.w() = msg->pose.pose.orientation.w;
    }
    else{
        request_position.x() = msg->pose.pose.position.x;
        request_position.y() = msg->pose.pose.position.y;
        request_position.z() = msg->pose.pose.position.z;
        request_pose.x() = msg->pose.pose.orientation.x;
        request_pose.y() = msg->pose.pose.orientation.y;
        request_pose.z() = msg->pose.pose.orientation.z;
        request_pose.w() = msg->pose.pose.orientation.w;
    }
    Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
    Pose_receive(0, 3) = request_position(0);
    Pose_receive(1, 3) = request_position(1);
    Pose_receive(2, 3) = request_position(2);

    Eigen::Matrix4d body_pose = Pose_receive;
    // convert to cam pose
    cam2world = body_pose * cam02body;
    cam2world_quat = cam2world.block<3, 3>(0, 0);

    q = Eigen::Quaterniond(cam2world_quat.w(),
                           cam2world_quat.x(),
                           cam2world_quat.y(),
                           cam2world_quat.z());
    pos = Eigen::Vector3d(cam2world(0, 3),
                          cam2world(1, 3),
                          cam2world(2, 3));

    return (std::make_tuple(pos, q));
}

void MapROS::depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  std::tie(camera_pos_, camera_q_) = ProcessPose(pose);
    processDepthMsg(img);
}

void MapROS::cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  std::tie(camera_pos_, camera_q_) = ProcessPose(pose);
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
    std::tie(camera_pos_, camera_q_) = ProcessPose(pose);
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
    std_msgs::Header header_msg;
    header_msg.stamp = odomMsg->header.stamp;
    received_pub_.publish(header_msg);

    std::tie(camera_pos_, camera_q_) = ProcessPose(odomMsg);
    cv_bridge::CvImagePtr semantics_cv_ptr = cv_bridge::toCvCopy(semanticsMsg, semanticsMsg->encoding);
    if (semanticsMsg->encoding == sensor_msgs::image_encodings::BGR8)
        cv::cvtColor((semantics_cv_ptr->image), *semantic_image_, cv::COLOR_BGR2GRAY);
    else
        semantics_cv_ptr->image.copyTo(*semantic_image_);
    processDepthMsg(depthMsg);
    sent_pub_.publish(header_msg);
}

void MapROS::depthOdomCallback(const sensor_msgs::ImageConstPtr& depthMsg,
                               const nav_msgs::OdometryConstPtr& odomMsg){
    std_msgs::Header header_msg;
    header_msg.stamp = odomMsg->header.stamp;
    received_pub_.publish(header_msg);

    std::tie(camera_pos_, camera_q_) = ProcessPose(odomMsg);
    processDepthMsg(depthMsg);

    sent_pub_.publish(header_msg);
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
    if(!do_semantics_)
        publishDepth();
}

void MapROS::publishMapAll() {
  pcl::PointXYZRGBL RGBLpt;
  pcl::PointXYZRGB RGBpt;
  //pcl::PointXYZL Lpt;
  pcl::PointXYZ pt;
  Eigen::Vector3i rgb;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> rgbSemanticCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

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
              /*static std::vector<uint32_t> all_labels;
              bool in=true;
              std::cout<<"LABELS: ";
              for(unsigned int all_label : all_labels){
                  std::cout<<all_label<<" ";
                  if(all_label==RGBLpt.label)
                      in=false;
              }
              if(in){
                  all_labels.push_back(RGBLpt.label);
                  std::cout<<RGBLpt.label;
              }
              std::cout<<std::endl;*/

              seen_labels.emplace(RGBLpt.label);
              RGBLpt.label = map_->md_->semantics_buffer_[map_->toAddress(x,y,z)];
              rgb = color_map_[static_cast<int>(RGBLpt.label)];
              RGBLpt.r = rgb(0);
              RGBLpt.g = rgb(1);
              RGBLpt.b = rgb(2);
              pcl::copyPoint(RGBLpt, RGBpt);
              rgbSemanticCloud->push_back(RGBpt);
              //pcl::copyPoint(RGBLpt, Lpt);
              //semanticCloud.push_back(Lpt);
          }
          pcl::copyPoint(RGBLpt, pt);
          cloud1->push_back(pt);
        }
      }
  cloud1->width = cloud1->points.size();
  cloud1->height = 1;
  cloud1->is_dense = true;
  cloud1->header.frame_id = frame_id_;

    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud1, *cloud_msg);
  map_all_pub_.publish(cloud_msg);

  if(do_semantics_) {
      //for(int seen_label : seen_labels){
      //    std::cout<<"LABEL: "<<seen_label<<" COLORS: "<<color_map_[seen_label](0)<<" "<<color_map_[seen_label](1)<<" "<<color_map_[seen_label](2)<<std::endl;
      //}
      rgbSemanticCloud->width = rgbSemanticCloud->points.size();
      rgbSemanticCloud->height = 1;
      rgbSemanticCloud->is_dense = true;
      rgbSemanticCloud->header.frame_id = frame_id_;
      sensor_msgs::PointCloud2Ptr cloud_msg_semantic(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*rgbSemanticCloud, *cloud_msg_semantic);
      semantic_color_map_pub_.publish(cloud_msg_semantic);
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
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud2(new pcl::PointCloud<pcl::PointXYZ>);

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
          cloud->push_back(pt);
        }
      }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->header.frame_id = frame_id_;
  cloud2->width = cloud2->points.size();
  cloud2->height = 1;
  cloud2->is_dense = true;
  cloud2->header.frame_id = frame_id_;

  sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud, *cloud_msg);
  map_local_pub_.publish(cloud_msg);
  sensor_msgs::PointCloud2Ptr cloud_msg2(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud2, *cloud_msg2);
  map_local_inflate_pub_.publish(cloud_msg);
}


void MapROS::publishDepth() {
  if(!do_semantics_){
      std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
      for (int i = 0; i < proj_points_cnt; ++i) {
        cloud->push_back(point_cloud_.points[i]);
      }
      cloud->width = cloud->points.size();
      cloud->height = 1;
      cloud->is_dense = true;
      cloud->header.frame_id = frame_id_;
      sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*cloud, *cloud_msg);
      depth_pub_.publish(cloud_msg);
  }
  else{
      pcl::PointXYZRGB pt;
      Eigen::Vector3i rgb;
      std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (int i = 0; i < proj_points_cnt; ++i) {
          rgb = color_map_[static_cast<int>(semantic_point_cloud_.points[i].label)];
          pcl::copyPoint(semantic_point_cloud_.points[i], pt);
          pt.r = rgb(0);
          pt.g = rgb(1);
          pt.b = rgb(2);
          cloud->push_back(pt);
      }
      cloud->width = cloud->points.size();
      cloud->height = 1;
      cloud->is_dense = true;
      cloud->header.frame_id = frame_id_;
      sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*cloud, *cloud_msg);
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
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud(new pcl::PointCloud<pcl::PointXYZI>);
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
      pt.z = 0.7;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud->push_back(pt);
    }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->header.frame_id = frame_id_;
  sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud, *cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}
}