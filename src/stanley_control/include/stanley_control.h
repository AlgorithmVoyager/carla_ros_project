#pragma once
#include <math.h>

#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include "Eigen/Core"
#include "common.h"


namespace shenlan {
namespace control {

using Matrix = Eigen::MatrixXd;

class StanleyController {
 public:
  StanleyController(){};
  ~StanleyController(){};

  void LoadControlConf();
  void ComputeControlCmd(const VehicleState &vehicle_state,
                         const TrajectoryData &planning_published_trajectory,
                         ControlCmd &cmd);
  void ComputeLateralErrors(const double x, const double y, const double theta,
                            double &e_y, double &e_theta);
  TrajectoryPoint QueryNearestPointByPosition(const double x, const double y);

  private:
  int getLastThetaSign() const {return last_sign_;};
  void setLastThetaSign(int last_sign) {last_sign_ = last_sign;};

  const double getLFromGravityCenterToFront() {return distance_from_cg_to_front_wheel_;}
  double getSteerToDeltaRatio() const;
  void calculateFrontWheelPosition(double& front_wheel_x, double& front_wheel_y, const double& x, const double& y,const double& theta) const;

 protected:
  std::vector<TrajectoryPoint> trajectory_points_;
  double k_y_ = 0.0;
  double u_min_ = 0.0;
  double u_max_ = 100.0;

  double theta_ref_;
  double theta_0_;
  size_t previous_matched_index_=0;

  double distance_from_cg_to_front_wheel_ = 1.4;

  // last e_theta is positive or negative
  int  last_sign_ = 0;
};

}  // namespace control
}  // namespace shenlan
