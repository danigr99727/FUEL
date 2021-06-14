#include <iostream>
#include <AirsimController.h>

double AngleDifference(const double x, const double y)
{
    //static int count = 0;
    auto theta = std::fmod(x-y, M_PI*2);
    //if (count--==0){
    //    std::cout<<"des_yaw: "<<x<<" yaw: "<<y<<" numbah: "<<numbah<<std::endl;
    //    count = 20;
    //}
    if (theta < M_PI)
        return theta;
    else
        return theta-2*M_PI;

}

AirsimController::AirsimController() : mass_(0.5), g_(9.81) {
  acc_.setZero();
}

void AirsimController::setMass(const double mass) {
  mass_ = mass;
}

void AirsimController::setGravity(const double g) {
  g_ = g;
}

void AirsimController::setPosition(const Eigen::Vector3d& position) {
  pos_ = position;
}

void AirsimController::setVelocity(const Eigen::Vector3d& velocity) {
  vel_ = velocity;
}

void AirsimController::setYaw(const double yaw) {
    yaw_ = yaw;
}

void AirsimController::setYawdot(const double yawdot) {
    yawdot_ = yawdot;
}

void AirsimController::calculateControl(const Eigen::Vector3d& des_pos, const Eigen::Vector3d& des_vel,
                                        const Eigen::Vector3d& des_acc, const double des_yaw,
                                        const double des_yaw_dot, const Eigen::Vector3d& kx,
                                        const Eigen::Vector3d& kv) {
  //  ROS_INFO("Error %lf %lf %lf", (des_pos - pos_).norm(),
  //           (des_vel - vel_).norm(), (des_acc - acc_).norm());

  Eigen::Vector3d totalError = (des_pos - pos_) + (des_vel - vel_) + (des_acc - acc_);

  Eigen::Vector3d ka(fabs(totalError[0]) > 3 ? 0 : (fabs(totalError[0]) * 0.2),
                     fabs(totalError[1]) > 3 ? 0 : (fabs(totalError[1]) * 0.2),
                     fabs(totalError[2]) > 3 ? 0 : (fabs(totalError[2]) * 0.2));

  velocity_.noalias() = 0.15*kx.asDiagonal() * (des_pos - pos_) + 0.15*kv.asDiagonal() * (des_vel - vel_) + des_vel + des_acc;
  double kyaw=1;
  double kyawdot=1;
  yawdotCommand_ = kyaw * AngleDifference(des_yaw,yaw_) + kyawdot * (des_yaw_dot - yawdot_) + des_yaw_dot;

}

const Eigen::Vector3d& AirsimController::getComputedVelocity() {
  return velocity_;
}

double AirsimController::getComputedYawdot() {
    return yawdotCommand_;
}

void AirsimController::setAcc(const Eigen::Vector3d& acc) {
  acc_ = acc;
}
