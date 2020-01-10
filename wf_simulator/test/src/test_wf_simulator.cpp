/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include <ros/ros.h>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include <boost/optional.hpp>
#include <amathutils_lib/amathutils.hpp>

#include "wf_simulator/wf_simulator_core.h"

class TestSuite : public ::testing::Test
{
public:
  TestSuite()
  : nh_(""), pnh_("~"), curr_pose_(boost::none)
  {
    pnh_.setParam("vehicle_model_type", "IDEAL_TWIST");
    pnh_.setParam("add_measurement_noise", false);
    sub_pose_ = nh_.subscribe("/current_pose", 1, &TestSuite::callbackPose, this);
    spin_duration_ = 0.05;
    spin_loopnum_ = 10;
  }
  ~TestSuite()
  {
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_pose_;
  ros::Publisher pub_initialpose_;
  std::shared_ptr<WFSimulatorCore> wf_simulator_;
  std::string source_str_;
  boost::optional<geometry_msgs::PoseStamped> curr_pose_;
  double spin_duration_;
  int spin_loopnum_;

  void callbackPose(const geometry_msgs::PoseStamped& pose)
  {
    curr_pose_ = pose;
  }

  void spinWhile(double time_sec)
  {
    curr_pose_ = boost::none;
    for (int i = 0; i < std::round(time_sec / spin_duration_); ++i)
    {
      ros::Duration(spin_duration_).sleep();
      ros::spinOnce();
      if (curr_pose_)
      {
        break;
      }
    }
  }
protected:
  virtual void SetUp()
  {
    pnh_.getParam("initialize_source", source_str_);
    wf_simulator_ = std::make_shared<WFSimulatorCore>();
    if (source_str_ == "RVIZ")
    {
      pub_initialpose_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    }
    else if (source_str_ == "NDT")
    {
      pub_initialpose_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1);
    }
  }
};

TEST_F(TestSuite, TestInitializeSource)
{
  geometry_msgs::Pose pose;
  pose.position.x = 3.5;
  pose.orientation.w = 1.0;
  /* == TestInitialPose PoseStamped == */
  if (source_str_ == "RVIZ")
  {
    geometry_msgs::PoseWithCovarianceStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose.pose = pose;
    pub_initialpose_.publish(p);
  }
  /* == TestInitialPose PoseStampedWthCovariance == */
  else if (source_str_ == "NDT")
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose = pose;
    pub_initialpose_.publish(p);
  }
  spinWhile(1.0);
  ASSERT_TRUE(!source_str_.empty());
  ASSERT_TRUE(curr_pose_) << "subscribe is failed";
  const double diff = curr_pose_.get().pose.position.x - pose.position.x;
  ASSERT_LT(std::fabs(diff), 0.1) << "initial pose x should be same";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode_WfSim");
  return RUN_ALL_TESTS();
}
