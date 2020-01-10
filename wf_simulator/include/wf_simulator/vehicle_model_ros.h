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

/**
 * @file vehicle_model_ros.h
 * @brief vehicle dynamics simulation library for autoware
 * @author Yuma Nihei
 * @date 2019.12.03
 */

#ifndef WF_SIMULATOR_VEHICLE_MODEL_ROS_H
#define WF_SIMULATOR_VEHICLE_MODEL_ROS_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <random>

#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleStatus.h>

#include "vehicle_sim_model/vehicle_model_interface.h"
#include "vehicle_sim_model/vehicle_model_ideal.h"
#include "vehicle_sim_model/vehicle_model_time_delay.h"
#include "vehicle_sim_model/vehicle_model_constant_acceleration.h"

class VehicleModelROS
{
public:
  /**
   * @brief constructor
   * @param [in] nh node handle to access global parameters
   * @param [in] pnh node handle to access private parameters
   */
  VehicleModelROS(ros::NodeHandle nh, ros::NodeHandle pnh);

  /**
   * @brief update status for simulation with delta time (sec)
   * @param [in] dt delta time for motion simulation
   * @param [in] closest_pos_z z position on closest waypoint
   */
  void updateStatus(const double dt, const double closest_pos_z);

  /**
   * @brief set current_vehicle_cmd_ptr_ with received message
   */
  void setVehicleCmd(const autoware_msgs::VehicleCmdConstPtr& msg);

  /**
   * @brief set initial state of simulated vehicle
   * @param [in] pose initial position and orientation
   * @param [in] twist initial velocity and angular velocity
   */
  void setInitialState(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist);

  /**
   * @brief return value of is_initialized_
   */
  const bool isInitialized() const;

  /**
   * @brief return value of current_pose_
   */
  const geometry_msgs::Pose getCurrentPose() const;

  /**
   * @brief return value of current_twist_
   */
  const geometry_msgs::Twist getCurrentTwist() const;

  /**
   * @brief return value of current_steering_angle_
   */
  const double getCurrentSteeringAngle() const;

private:
  /* ros system */
  ros::NodeHandle nh_;                    //!< @brief ros node handle
  ros::NodeHandle pnh_;                   //!< @brief private ros node handle

  /* output variables */
  geometry_msgs::Pose current_pose_;    //!< @brief current vehicle position ang angle with pose message class
  geometry_msgs::Twist current_twist_;  //!< @brief current vehicle velocity with twist message class
  double current_steering_angle_;               // !< @brief current steering angle with double

  /* vehicle parameters */
  double wheelbase_;     //!< @brief wheelbase length to convert angular-velocity & steering

  /* flags */
  bool is_initialized_;         //!< @brief flag to check the initial position is set
  bool add_measurement_noise_;  //!< @brief flag to add measurement noise

  /* vehicle model */
  enum class VehicleModelType
  {
    IDEAL_TWIST = 0,
    IDEAL_STEER = 1,
    DELAY_TWIST = 2,
    DELAY_STEER = 3,
    CONST_ACCEL_TWIST = 4,
    IDEAL_FORKLIFT_RLS = 5,
    DELAY_FORKLIFT_RLS = 6,
  }
  vehicle_model_type_;                                    //!< @brief vehicle model type to decide the model dynamics
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_;  //!< @brief vehicle model pointer

  /* to generate measurement noise */
  std::shared_ptr<std::mt19937> rand_engine_ptr_;                     //!< @brief random engine for measurement noise
  std::shared_ptr<std::normal_distribution<>> pos_norm_dist_ptr_;     //!< @brief Gaussian noise for position
  std::shared_ptr<std::normal_distribution<>> vel_norm_dist_ptr_;     //!< @brief Gaussian noise for velocity
  std::shared_ptr<std::normal_distribution<>> rpy_norm_dist_ptr_;     //!< @brief Gaussian noise for roll-pitch-yaw
  std::shared_ptr<std::normal_distribution<>> angvel_norm_dist_ptr_;  //!< @brief Gaussian noise for angular velocity
  std::shared_ptr<std::normal_distribution<>> steer_norm_dist_ptr_;   //!< @brief Gaussian noise for steering angle
};

#endif  // WF_SIMULATOR_VEHICLE_MODEL_ROS_H
