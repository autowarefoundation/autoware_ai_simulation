/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <wf_simulator/vehicle_model_ros.h>
#include <string>

static constexpr double LINEAR_VEL_MIN = 1e-3;

// clang-format on
VehicleModelROS::VehicleModelROS(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh), is_initialized_(false)
{
  /* simulator parameters */
  double loop_rate;
  pnh_.param<double>("loop_rate", loop_rate, 50.0);
  nh_.param<double>("/vehicle_info/wheel_base", wheelbase_, 2.7);
  pnh_.param<bool>("add_measurement_noise", add_measurement_noise_, true);

  /* set vehicle model parameters */
  double angvel_lim, vel_lim, steer_lim, accel_rate, angvel_rate, steer_rate_lim, vel_time_delay,
      vel_time_constant, steer_time_delay, steer_time_constant, angvel_time_delay, angvel_time_constant;
  pnh_.param<double>("angvel_lim", angvel_lim, 3.0);
  pnh_.param<double>("vel_lim", vel_lim, 10.0);
  pnh_.param<double>("steer_lim", steer_lim, 3.14 / 3.0);
  pnh_.param<double>("accel_rate", accel_rate, 1.0);
  pnh_.param<double>("angvel_rate", angvel_rate, 1.0);
  pnh_.param<double>("steer_rate_lim", steer_rate_lim, 0.3);
  pnh_.param<double>("vel_time_delay", vel_time_delay, 0.25);
  pnh_.param<double>("vel_time_constant", vel_time_constant, 0.6197);
  pnh_.param<double>("steer_time_delay", steer_time_delay, 0.24);
  pnh_.param<double>("steer_time_constant", steer_time_constant, 0.27);
  pnh_.param<double>("angvel_time_delay", angvel_time_delay, 0.2);
  pnh_.param<double>("angvel_time_constant", angvel_time_constant, 0.5);
  const double dt = 1.0 / loop_rate;

  /* set vehicle model type */
  std::string vehicle_model_type_str;
  pnh_.param<std::string>("vehicle_model_type", vehicle_model_type_str, "IDEAL_TWIST");
  ROS_INFO("vehicle_model_type = %s", vehicle_model_type_str.c_str());
  if (vehicle_model_type_str == "IDEAL_TWIST")
  {
    vehicle_model_type_ = VehicleModelType::IDEAL_TWIST;
    vehicle_model_ptr_ = std::make_shared<VehicleModelIdealTwist>();
  }
  else if (vehicle_model_type_str == "IDEAL_STEER")
  {
    vehicle_model_type_ = VehicleModelType::IDEAL_STEER;
    vehicle_model_ptr_ = std::make_shared<VehicleModelIdealSteer>(wheelbase_);
  }
  else if (vehicle_model_type_str == "DELAY_TWIST")
  {
    vehicle_model_type_ = VehicleModelType::DELAY_TWIST;
    vehicle_model_ptr_ =
        std::make_shared<VehicleModelTimeDelayTwist>(vel_lim, angvel_lim, accel_rate, angvel_rate, dt, vel_time_delay,
                                                   vel_time_constant, angvel_time_delay, angvel_time_constant);
  }
  else if (vehicle_model_type_str == "DELAY_STEER")
  {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER;
    vehicle_model_ptr_ = std::make_shared<VehicleModelTimeDelaySteer>(vel_lim, steer_lim, accel_rate, steer_rate_lim,
                                                                    wheelbase_, dt, vel_time_delay, vel_time_constant,
                                                                    steer_time_delay, steer_time_constant);
  }
  else if (vehicle_model_type_str == "CONST_ACCEL_TWIST")
  {
    vehicle_model_type_ = VehicleModelType::CONST_ACCEL_TWIST;
    vehicle_model_ptr_ = std::make_shared<VehicleModelConstantAccelTwist>(vel_lim, angvel_lim, accel_rate, angvel_rate);
  }
  else
  {
    ROS_ERROR("Invalid vehicle_model_type. Initialization failed.");
    ros::shutdown();
  }

  /* set normal distribution noises */
  int random_seed;
  pnh_.param<int>("random_seed", random_seed, -1);
  if (random_seed >= 0)
  {
    rand_engine_ptr_ = std::make_shared<std::mt19937>(random_seed);
  }
  else
  {
    std::random_device seed;
    rand_engine_ptr_ = std::make_shared<std::mt19937>(seed());
  }
  double pos_noise_stddev, vel_noise_stddev, rpy_noise_stddev, angvel_noise_stddev, steer_noise_stddev;
  pnh_.param<double>("pos_noise_stddev", pos_noise_stddev, 1e-2);
  pnh_.param<double>("vel_noise_stddev", vel_noise_stddev, 1e-2);
  pnh_.param<double>("rpy_noise_stddev", rpy_noise_stddev, 1e-4);
  pnh_.param<double>("angvel_noise_stddev", angvel_noise_stddev, 1e-3);
  pnh_.param<double>("steer_noise_stddev", steer_noise_stddev, 1e-4);
  pos_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, pos_noise_stddev);
  vel_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, vel_noise_stddev);
  rpy_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, rpy_noise_stddev);
  angvel_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, angvel_noise_stddev);
  steer_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, steer_noise_stddev);
}

void VehicleModelROS::updateStatus(const double dt, const double closest_pos_z)
{
  /* update vehicle dynamics */
  vehicle_model_ptr_->update(dt);

  /* save current vehicle pose & twist */
  current_pose_.position.x = vehicle_model_ptr_->getX();
  current_pose_.position.y = vehicle_model_ptr_->getY();
  current_pose_.position.z = closest_pos_z;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = vehicle_model_ptr_->getYaw();
  current_twist_.linear.x = vehicle_model_ptr_->getVx();
  current_twist_.angular.z = vehicle_model_ptr_->getWz();
  current_steering_angle_ = vehicle_model_ptr_->getSteer();

  if (add_measurement_noise_)
  {
    current_pose_.position.x += (*pos_norm_dist_ptr_)(*rand_engine_ptr_);
    current_pose_.position.y += (*pos_norm_dist_ptr_)(*rand_engine_ptr_);
    current_pose_.position.z += (*pos_norm_dist_ptr_)(*rand_engine_ptr_);
    roll += (*rpy_norm_dist_ptr_)(*rand_engine_ptr_);
    pitch += (*rpy_norm_dist_ptr_)(*rand_engine_ptr_);
    yaw += (*rpy_norm_dist_ptr_)(*rand_engine_ptr_);
    current_twist_.linear.x += (*vel_norm_dist_ptr_)(*rand_engine_ptr_);
    current_twist_.angular.z += (*angvel_norm_dist_ptr_)(*rand_engine_ptr_);
    current_steering_angle_ += (*steer_norm_dist_ptr_)(*rand_engine_ptr_);
  }
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  current_pose_.orientation = tf2::toMsg(quaternion);
}

void VehicleModelROS::setVehicleCmd(const autoware_msgs::VehicleCmdConstPtr& msg)
{
  if (vehicle_model_type_ == VehicleModelType::IDEAL_TWIST || vehicle_model_type_ == VehicleModelType::DELAY_TWIST ||
      vehicle_model_type_ == VehicleModelType::CONST_ACCEL_TWIST)
  {
    Eigen::VectorXd input(2);
    input << msg->twist_cmd.twist.linear.x, msg->twist_cmd.twist.angular.z;
    vehicle_model_ptr_->setInput(input);
  }
  else if (vehicle_model_type_ == VehicleModelType::IDEAL_STEER || vehicle_model_type_ == VehicleModelType::DELAY_STEER)
  {
    Eigen::VectorXd input(2);
    input << msg->ctrl_cmd.linear_velocity, msg->ctrl_cmd.steering_angle;
    vehicle_model_ptr_->setInput(input);
  }
  else
  {
    ROS_WARN("[%s] : invalid vehicle_model_type_  error.", __func__);
  }
}

void VehicleModelROS::setInitialState(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist)
{
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double yaw = tf2::getYaw(pose.orientation);
  const double vx = twist.linear.x;
  const double wz = twist.angular.z;
  const double steer = (fabs(vx) > LINEAR_VEL_MIN) ? atan(wheelbase_ * wz / vx) : 0.0;

  if (vehicle_model_type_ == VehicleModelType::IDEAL_TWIST || vehicle_model_type_ == VehicleModelType::IDEAL_STEER)
  {
    Eigen::VectorXd state(3);
    state << x, y, yaw;
    vehicle_model_ptr_->setState(state);
  }
  else if (vehicle_model_type_ == VehicleModelType::DELAY_TWIST ||
           vehicle_model_type_ == VehicleModelType::CONST_ACCEL_TWIST)
  {
    Eigen::VectorXd state(5);
    state << x, y, yaw, vx, wz;
    vehicle_model_ptr_->setState(state);
  }
  else if (vehicle_model_type_ == VehicleModelType::DELAY_STEER)
  {
    Eigen::VectorXd state(5);
    state << x, y, yaw, vx, steer;
    vehicle_model_ptr_->setState(state);
  }
  else
  {
    ROS_WARN("undesired vehicle model type! Initialization failed.");
    return;
  }

  is_initialized_ = true;
}

const bool VehicleModelROS::isInitialized() const
{
  return is_initialized_;
}

const geometry_msgs::Pose VehicleModelROS::getCurrentPose() const
{
  return current_pose_;
}

const geometry_msgs::Twist VehicleModelROS::getCurrentTwist() const
{
  return current_twist_;
}

const double VehicleModelROS::getCurrentSteeringAngle() const
{
  return current_steering_angle_;
}
