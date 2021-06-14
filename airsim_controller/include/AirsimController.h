#include <Eigen/Geometry>

class AirsimController {
public:
  AirsimController();

  void setMass(double mass);
  void setGravity(double g);
  void setPosition(const Eigen::Vector3d& position);
  void setVelocity(const Eigen::Vector3d& velocity);
  void setAcc(const Eigen::Vector3d& acc);
  void setYaw(double yaw);
  void setYawdot(double yawdot);

  void calculateControl(const Eigen::Vector3d& des_pos, const Eigen::Vector3d& des_vel,
                        const Eigen::Vector3d& des_acc, double des_yaw, double des_yaw_dot,
                        const Eigen::Vector3d& kx, const Eigen::Vector3d& kv);

  const Eigen::Vector3d& getComputedVelocity(void);
  const Eigen::Quaterniond& getComputedOrientation(void);
  double getComputedYawdot(void);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // Inputs for the controller
  double mass_;
  double g_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;
  double yawdotCommand_;
  double yawdot_;
  double yaw_;

  // Outputs of the controller
  Eigen::Vector3d velocity_;
};

