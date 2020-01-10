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
 * @file wf_simulator_core.h
 * @brief vehicle dynamics simulation for autoware
 * @author Takamasa Horibe
 * @date 2019.08.17
 */

#ifndef WF_SIMULATOR_WF_SIMULATOR_CORE_H
#define WF_SIMULATOR_WF_SIMULATOR_CORE_H

#include <string>
#include <autoware_msgs/Lane.h>
#include <wf_simulator/vehicle_model_ros.h>

class WFSimulatorCore
{
public:
  /**
   * @brief constructor
   */
  WFSimulatorCore();

private:
  /* ros system */
  ros::NodeHandle nh_;                    //!< @brief ros node handle
  ros::NodeHandle pnh_;                   //!< @brief private ros node handle
  ros::Publisher pub_pose_;               //!< @brief topic ros publisher for current pose
  ros::Publisher pub_lidar_pose_;         //!< @brief topic ros publisher for lodar pose
  ros::Publisher pub_twist_;              //!< @brief topic ros publisher for current twist
  ros::Publisher pub_vehicle_status_;     //!< @brief topic ros publisher for current vehicle status
  ros::Subscriber sub_vehicle_cmd_;       //!< @brief topic subscriber for vehicle_cmd
  ros::Subscriber sub_waypoints_;         //!< @brief topic subscriber for waypoints used for z ppsition
  ros::Subscriber sub_initialpose_;       //!< @brief topic subscriber for initialpose topic
  ros::Subscriber sub_closest_waypoint_;  //!< @brief topic subscriber for closest_waypoint id for z position
  ros::Timer timer_simulation_;           //!< @brief timer for simulation
  ros::Timer timer_tf_;                   //!< @brief timer to pubish tf

  /* core library */
  VehicleModelROS vehicle_sim_model_;

  /* received & published topics */
  geometry_msgs::Pose current_pose_;    //!< @brief current vehicle position ang angle with pose message class
  std::shared_ptr<autoware_msgs::Lane> current_waypoints_ptr_;          //!< @brief latest received waypoints
  double closest_pos_z_;                                                //!< @brief z position on closest waypoint

  /* tf */
  tf::TransformListener tf_listener_;        //!< @brief tf listener
  tf::TransformBroadcaster tf_broadcaster_;  //!< @brief tf broadcaster

  /* frame_id */
  std::string simulation_frame_id_;  //!< @brief vehicle frame id simulated by wf_simulator
  std::string map_frame_id_;         //!< @brief map frame_id
  std::string lidar_frame_id_;       //!< @brief lidar frame_id

  /* wf_simulator parameters */
  double lidar_height_;  //!< @brief lidar height [m] to calculate lidar tf

  /* saved values */
  std::shared_ptr<ros::Time> prev_update_time_ptr_;  //!< @brief previously updated time

  /**
   * @brief set current_vehicle_cmd_ptr_ with received message
   */
  void callbackVehicleCmd(const autoware_msgs::VehicleCmdConstPtr& msg);

  /**
   * @brief set current_waypoints_ptr_ with received message
   */
  void callbackWaypoints(const autoware_msgs::LaneConstPtr& msg);

  /**
   * @brief set current_closest_waypoint_ptr_ with received message
   */
  void callbackClosestWaypoint(const std_msgs::Int32ConstPtr& msg);

  /**
   * @brief set initial pose for simulation with received message
   */
  void callbackInitialPoseWithCov(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  /**
   * @brief set initial pose with received message
   */
  void callbackInitialPoseStamped(const geometry_msgs::PoseStampedConstPtr& msg);

  /**
   * @brief get transform from two frame_ids
   * @param [in] parent_frame parent frame id
   * @param [in] child frame id
   */
  tf::StampedTransform getTransformFromTF(const std::string parent_frame, const std::string child_frame);

  /**
   * @brief timer callback for simulation with loop_rate
   */
  void timerCallbackSimulation(const ros::TimerEvent& e);

  /**
   * @brief timer callback for tf publication
   */
  void timerCallbackPublishTF(const ros::TimerEvent& e);

  /**
   * @brief set initial state of simulated vehicle with pose transformation based on frame_id
   * @param [in] pose initial position and orientation with header
   * @param [in] twist initial velocity and angular velocity
   */
  void setInitialStateWithPoseTransform(const geometry_msgs::PoseStamped& pose, const geometry_msgs::Twist& twist);

  /**
   * @brief set initial state of simulated vehicle with pose transformation based on frame_id
   * @param [in] pose initial position and orientation with header
   * @param [in] twist initial velocity and angular velocity
   */
  void setInitialStateWithPoseTransform(const geometry_msgs::PoseWithCovarianceStamped& pose,
                                        const geometry_msgs::Twist& twist);

  /**
   * @brief publish pose and twist
   * @param [in] pose pose to be published
   * @param [in] twist twist to be published
   */
  void publishPoseTwist(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist);

  /**
   * @brief publish tf
   * @param [in] pose pose used for tf
   */
  void publishTF(const geometry_msgs::Pose& pose);
};

#endif  // WF_SIMULATOR_WF_SIMULATOR_CORE_H
