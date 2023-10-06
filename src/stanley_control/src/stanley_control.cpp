#include "stanley_control.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include <math.h>

using namespace std;

#define GC_To_Front_Transfer 0

namespace shenlan {
namespace control {

double atan2_to_PI(const double atan2) {
  return atan2 * M_PI / 180;
}

double PointDistanceSquare(const TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.x - x;
  const double dy = point.y - y;
  return dx * dx + dy * dy;
}

void StanleyController::LoadControlConf() {
  k_y_ = 0.5;
}

double StanleyController::getSteerToDeltaRatio() const
{
#if GC_To_Front_Transfer
  // if current position we get is center of gravity, we need to transfer it to front wheel ; current ratio set as 1
  return 1;
#else
  // if current position we get is  point of the front wheel ; current ratio set as 2
  return 2.0;
#endif
}

void StanleyController::calculateFrontWheelPosition(double& front_wheel_x, double& front_wheel_y, const double& x, const double& y,const double& theta) const
{
#if GC_To_Front_Transfer
  // if current position we get is center of gravity, we need to transfer it to front wheel ; current ratio set as 1
  front_wheel_x = x + 1.4*cos(theta); // get it from Tesla model 3's car size
  front_wheel_y = y + 1.4*sin(theta);
  return ;
#else
  // if current position we get is  point of the front wheel ; current ratio set as 2
  front_wheel_x = x;
  front_wheel_y = y ;
 return ;
#endif
}

// /** to-do **/ 计算需要的控制命令, 实现对应的stanley模型,并将获得的控制命令传递给汽车
// 提示，在该函数中你需要调用计算误差 ComputeLateralErrors
void StanleyController::ComputeControlCmd(
    const VehicleState &vehicle_state,
    const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) {

    trajectory_points_.insert(trajectory_points_.end(),planning_published_trajectory.trajectory_points.begin(),planning_published_trajectory.trajectory_points.end());
    
     //1.  calculate error
    double e_y =0.0;
    double e_theta = 0.0;
    // std::cout << "calculate lateral erros" << std::endl;
    ComputeLateralErrors(vehicle_state.x,vehicle_state.y,vehicle_state.heading,e_y,e_theta);
    std::cout << "[LOG_INFO] : " << "===compute lateral errors successfully!===="<<std::endl;
    std::cout <<  "[LOG_INFO] : " << "e_y: "<< e_y << ", e_theta: "<< e_theta <<std::endl;

    // 2. generate stanley model
    if(vehicle_state.velocity < 1e-5)
    {
      std::cout << "current velocity is too small!" << std::endl;
      return ;
    }
    // 2.1 stanley model
    // add a smaller num to avoid divide by zero error
    double delta = e_theta + std::atan(k_y_*e_y/(vehicle_state.velocity+1e-5));
    std::cout << "[LOG_INFO] : " << "===compute front wheel delta successfully!===="<<std::endl;
    std::cout <<  "[LOG_INFO] : " << "delta: "<< delta <<std::endl;

    // 2.2 compare with max arrivable delta
    delta = (delta >atan2_to_PI(30)) ? atan2_to_PI(30) : delta;
    delta = (delta < -atan2_to_PI(30)) ? -atan2_to_PI(30) : delta;
    cmd.steer_target =getSteerToDeltaRatio()* delta;
    std::cout <<  "[LOG_INFO] : " << "current steer_target is "<< cmd.steer_target << std::endl;
    return ;
}

// /** to-do **/ 计算需要的误差，包括横向误差，纵向误差
void StanleyController::ComputeLateralErrors(const double x, const double y,
                                             const double theta, double &e_y,
                                             double &e_theta) {
     // 1. transfer vehicle postion  to front wheel
    double  front_wheel_x =0.0;
    double  front_wheel_y =0.0;                                
    calculateFrontWheelPosition(front_wheel_x, front_wheel_y, x, y, theta);
    
    // 2. match nearest point in the reference line
    TrajectoryPoint matched_point = QueryNearestPointByPosition(front_wheel_x,front_wheel_y);

    // 3. check if matched point is valid 
    auto func = [](TrajectoryPoint point) -> bool {
      return std::isnan(point.x) && std::isnan(point.y) && std::isnan(point.heading);
    };

    if(func(matched_point))
    {
      std::cout << "no valid match_point!!!!!!" <<std::endl;
      return ;
    }

    // 4. calculate e_y
    // 4.1 calculate normal vector of ref_point
    double n_x = -std::sin(matched_point.heading);
    double n_y = std::cos(matched_point.heading);

    // 4.2 calculate distance vector between vehicle and matched_point
    double v_x = x-matched_point.x;
    double v_y = y-matched_point.y;

    // calculate distance in normal direction (from ref point to front wheel point)
    e_y = v_x*n_x + v_y*n_y;

    // 5. calculate e_theta
    /*
      here we need check if e_theta is in the normal range
      normally, theta_ref_ is in the range [-pi,pi];
      and e_theta should also be in the range [-pi,pi]; if it's out of the range, we should make it in range [-pi,pi],
      and we should also make sure its direction  is as same with last step.
    */
    // 5.1 check if there is prompt jump of ref_point_theta
      if(std::abs(theta_ref_) > 3.14 )
      {
        // if last sign is zero or same with current point, do nothing
        if(theta_ref_ * getLastThetaSign() <0)
        {
          theta_ref_ = getLastThetaSign() * (2*M_PI - std::abs(theta_ref_));
        }
      }
      
      // 5.2 update last sign
      if(theta_ref_ >=0){
        setLastThetaSign(1);
      }
      else{
      setLastThetaSign(-1);
      }

      //5.3 calculate e_theta
      e_theta = theta -theta_ref_;

      // 5.4 make e_theta is normal range
      e_theta = (e_theta > M_PI) ? e_theta -= 2*M_PI : e_theta;
      e_theta = (e_theta < -M_PI) ? e_theta += 2*M_PI : e_theta;
      // std::cout << "current ref theta: "<< theta_ref_;
      // std::cout << "current theta: "<< theta;
      return ;
}

TrajectoryPoint StanleyController::QueryNearestPointByPosition(const double x,
                                                               const double y) {                      
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = previous_matched_index_; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  // std:: cout << "previous_matched_index_: " << previous_matched_index_ <<"\n index_min" << index_min<<std::endl;

  /*
     this part of code will make matched point continuely. 
     if the control effect is not good enough and vehicle is far away from the reference line,
     it may  lead to a mismatch problem. for a better and stable performance, i add following code:
  */ 

 if(index_min - previous_matched_index_ > 20)
 {
  std::cout <<  "[LOG_ERROR] : matched point is not right one, set it as last one!!!" <<std::endl;
  index_min = previous_matched_index_;
 }
 previous_matched_index_ = index_min;
 theta_ref_ = trajectory_points_[index_min].heading;

  return trajectory_points_[index_min];
}

}  // namespace control
}  // namespace shenlan
