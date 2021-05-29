#include <iostream>
#include <AirsimController.h>

double AngleDifference(const double x, const double y)
{
    //static int count = 0;
    auto numbah = std::fmod(x-y, M_PI*2);
    //if (count--==0){
    //    std::cout<<"des_yaw: "<<x<<" yaw: "<<y<<" numbah: "<<numbah<<std::endl;
    //    count = 20;
    //}
    if (numbah < M_PI)
        return numbah;
    else
        return numbah-2*M_PI;

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

  force_.noalias() = 0.15*kx.asDiagonal() * (des_pos - pos_) + 0.15*kv.asDiagonal() * (des_vel - vel_) + des_vel + des_acc;
  double kyaw=1;
  double kyawdot=1;
  yawdotCommand_ = kyaw * AngleDifference(des_yaw,yaw_) + kyawdot * (des_yaw_dot - yawdot_) + des_yaw_dot;

      //mass_ * /*(Eigen::Vector3d(1, 1, 1) - ka).asDiagonal() **/ (des_acc) +
      //mass_ * ka.asDiagonal() * (des_acc - acc_) + mass_ * g_ * Eigen::Vector3d(0, 0, 1);

  // Limit control angle to 45 degree
  double theta = M_PI / 2;
  //double c = cos(theta);
 /* Eigen::Vector3d f;
  f.noalias() = kx.asDiagonal() * (des_pos - pos_) + kv.asDiagonal() * (des_vel - vel_) +  des_vel;

  if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c) {
    double nf = f.norm();
    double A = c * c * nf * nf - f(2) * f(2);
    double B = 2 * (c * c - 1) * f(2) * mass_ * g_;
    double C = (c * c - 1) * mass_ * mass_ * g_ * g_;
    double s = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
    force_.noalias() = s * f + mass_ * g_ * Eigen::Vector3d(0, 0, 1);
  }
  // Limit control angle to 45 degree*/

  Eigen::Vector3d b1c, b2c, b3c;
  Eigen::Vector3d b1d(cos(des_yaw), sin(des_yaw), 0);

  if (force_.norm() > 1e-6)
    b3c.noalias() = force_.normalized();
  else
    b3c.noalias() = Eigen::Vector3d(0, 0, 1);

  b2c.noalias() = b3c.cross(b1d).normalized();
  b1c.noalias() = b2c.cross(b3c).normalized();

  Eigen::Matrix3d R;
  R << b1c, b2c, b3c;

  orientation_ = Eigen::Quaterniond(R);
}

const Eigen::Vector3d& AirsimController::getComputedForce() {
  return force_;
}

const Eigen::Quaterniond& AirsimController::getComputedOrientation() {
  return orientation_;
}

double AirsimController::getComputedYawdot() {
    return yawdotCommand_;
}

void AirsimController::setAcc(const Eigen::Vector3d& acc) {
  acc_ = acc;
}
