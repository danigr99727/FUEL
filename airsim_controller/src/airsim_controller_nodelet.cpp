#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <AirsimController.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <airsim_ros_pkgs/VelCmd.h>
#include <airsim_ros_pkgs/Takeoff.h>

class AirsimControllerNodelet : public nodelet::Nodelet {
public:
  AirsimControllerNodelet()
    : position_cmd_updated_(false)
    , position_cmd_init_(false)
    , des_yaw_(0)
    , des_yaw_dot_(0)
    , current_yaw_(0)
{  }

  void onInit(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void publishSO3Command(void);
  void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr& msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr& msg);
  void imu_callback(const sensor_msgs::Imu& imu);

  AirsimController controller_;
  ros::Publisher vel_cmd_pub;
  ros::Subscriber odom_sub_;
  ros::Subscriber position_cmd_sub_;
  ros::Subscriber imu_sub_;

  ros::ServiceClient takeoff_cli;
  airsim_ros_pkgs::Takeoff::Request takeoffRequest;
  airsim_ros_pkgs::Takeoff::Response takeoffResponse;

  bool position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
  double des_yaw_, des_yaw_dot_;
  double current_yaw_;
};

void AirsimControllerNodelet::publishSO3Command(void) {
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_, des_yaw_dot_, kx_, kv_);

  const Eigen::Vector3d& force = controller_.getComputedForce();
  const Eigen::Quaterniond& orientation = controller_.getComputedOrientation();
  const double yawdot = controller_.getComputedYawdot();

  airsim_ros_pkgs::VelCmd::Ptr velocityRequest(new airsim_ros_pkgs::VelCmd);  //! @note memory leak?
  velocityRequest->twist.linear.x=force(1);
  velocityRequest->twist.linear.y=force(0);
  velocityRequest->twist.linear.z=-force(2);
  velocityRequest->twist.angular.z=-yawdot;

  //velocityRequest->twist.angular.x=orientation.y();
  //velocityRequest->twist.angular.y=orientation.x();
  //velocityRequest->twist.angular.w=orientation.w();
  vel_cmd_pub.publish(velocityRequest);
}

void AirsimControllerNodelet::position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd) {
    //static double max_yaw_cmd = 0;
    //if (cmd->yaw > max_yaw_cmd)
    //    max_yaw_cmd=cmd->yaw;
    //std::cout<<"max_yaw_cmd: "<< max_yaw_cmd <<std::endl;
    //static double min_yaw_cmd = 0;
    //if (cmd->yaw  < min_yaw_cmd)
    //    min_yaw_cmd=cmd->yaw;
    //std::cout<<"min_yaw_cmd: "<< min_yaw_cmd <<std::endl;
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
  kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

  des_yaw_ = cmd->yaw - M_PI/2;
  //std::cout<<"yaw: "<<cmd->yaw<<std::endl;
  des_yaw_dot_ = cmd->yaw_dot;
  position_cmd_updated_ = true;
  position_cmd_init_ = true;

  publishSO3Command();
}

void AirsimControllerNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
  const Eigen::Vector3d position(odom->pose.pose.position.x, odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x, odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  controller_.setPosition(position);
  controller_.setVelocity(velocity);
  //static double max_yaw = 0;
  //if (current_yaw_ > max_yaw)
  //  max_yaw=current_yaw_;
  //std::cout<<"max_yaw: "<< max_yaw<<std::endl;
  //  static double min_yaw = 0;
  //if (current_yaw_ < min_yaw)
  //    min_yaw=current_yaw_ ;
  //std::cout<<"min_yaw: "<< min_yaw<<std::endl;
  controller_.setYaw(current_yaw_);
  controller_.setYawdot(odom->twist.twist.angular.z);

  if (position_cmd_init_) {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some
    // time
    if (!position_cmd_updated_) publishSO3Command();
    position_cmd_updated_ = false;
  }
}

void AirsimControllerNodelet::imu_callback(const sensor_msgs::Imu& imu) {
  const Eigen::Vector3d acc(imu.linear_acceleration.x, imu.linear_acceleration.y,
                            imu.linear_acceleration.z);
  controller_.setAcc(acc);
}

void AirsimControllerNodelet::onInit(void) {
  ros::NodeHandle n(getPrivateNodeHandle());
  ros::NodeHandle pn(getNodeHandle());

  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;

  double mass;
  n.param("mass", mass, 0.74);
  controller_.setMass(mass);


  //so3_command_pub_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);
  vel_cmd_pub = n.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/drone_1/vel_cmd_world_frame", 10);

  odom_sub_ = n.subscribe("odom", 10, &AirsimControllerNodelet::odom_callback, this,
                          ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ = n.subscribe("position_cmd", 10, &AirsimControllerNodelet::position_cmd_callback, this,
                                  ros::TransportHints().tcpNoDelay());
  takeoff_cli = n.serviceClient<airsim_ros_pkgs::Takeoff>("takeoff", true);
  imu_sub_ = n.subscribe("imu", 10, &AirsimControllerNodelet::imu_callback, this, ros::TransportHints().tcpNoDelay());
  ros::Duration(5).sleep();
    takeoff_cli.call(takeoffRequest, takeoffResponse);
  //while(!takeoff_cli.call(takeoffRequest, takeoffResponse)){
  //      std::cout<<"TAKING OFF"<<std::endl;
  //      ros::Duration(1.0).sleep();
// }
    ros::Duration(5).sleep();

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(AirsimControllerNodelet, nodelet::Nodelet);
