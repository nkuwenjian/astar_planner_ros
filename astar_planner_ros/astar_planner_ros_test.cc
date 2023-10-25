/******************************************************************************
 * Copyright (c) 2023, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Jian Wen (nkuwenjian@gmail.com)
 *****************************************************************************/

#include "astar_planner_ros/astar_planner_ros.h"

#include <memory>
#include <mutex>   // NOLINT
#include <thread>  // NOLINT

#include "costmap_2d/costmap_2d_ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "glog/logging.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace astar_planner_ros {

class AStarPlannerROSTest {
 public:
  explicit AStarPlannerROSTest(tf2_ros::Buffer& tf);  // NOLINT
  virtual ~AStarPlannerROSTest() = default;

  void Initialize();

 private:
  void SetStart(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
  void SetGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void MakePlan();
  void PublishTransform();
  void PublishLoop();

  ros::NodeHandle nh_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;

  geometry_msgs::PoseStamped start_;
  geometry_msgs::PoseStamped goal_;
  bool start_received_ = false;
  bool goal_received_ = false;
  tf2_ros::Buffer& tf_;
  std::mutex start_mutex_;
  std::mutex goal_mutex_;
  std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_ = nullptr;
  std::unique_ptr<std::thread> transform_thread_ = nullptr;

  std::unique_ptr<AStarPlannerROS> planner_ = nullptr;
  std::unique_ptr<costmap_2d::Costmap2DROS> costmap_ros_ = nullptr;
};

AStarPlannerROSTest::AStarPlannerROSTest(tf2_ros::Buffer& tf) : tf_(tf) {}

void AStarPlannerROSTest::Initialize() {
  start_sub_ =
      nh_.subscribe("initialpose", 1, &AStarPlannerROSTest::SetStart, this);
  goal_sub_ = nh_.subscribe("move_base_simple/goal", 1,
                            &AStarPlannerROSTest::SetGoal, this);

  tf_broadcaster_ = std::make_unique<tf::TransformBroadcaster>();

  // Create a thread to periodically publish the latest map->base_link
  // transform; it needs to go out regularly, uninterrupted by potentially
  // long periods of computation in our main loop.
  transform_thread_ = std::make_unique<std::thread>([this] { PublishLoop(); });

  costmap_ros_ =
      std::make_unique<costmap_2d::Costmap2DROS>("global_costmap", tf_);
  costmap_ros_->pause();
  planner_ = std::make_unique<AStarPlannerROS>();
  planner_->initialize("AStarPlannerROS", costmap_ros_.get());

  // Start actively updating costmaps based on sensor data.
  costmap_ros_->start();
}

void AStarPlannerROSTest::PublishTransform() {
  std::lock_guard<std::mutex> lock(start_mutex_);

  tf::Transform map_to_base_link;
  if (start_received_) {
    map_to_base_link = tf::Transform(
        tf::createQuaternionFromYaw(tf::getYaw(start_.pose.orientation)),
        tf::Point(start_.pose.position.x, start_.pose.position.y, 0.0));
  } else {
    map_to_base_link.setIdentity();
  }

  ros::Time tf_expiration = ros::Time::now();
  tf_broadcaster_->sendTransform(tf::StampedTransform(
      map_to_base_link, tf_expiration, "map", "base_link"));
}

void AStarPlannerROSTest::PublishLoop() {
  static constexpr double kTransformPublishPeriod = 0.05;
  ros::Rate r(1.0 / kTransformPublishPeriod);
  while (ros::ok()) {
    PublishTransform();
    r.sleep();
  }
}

void AStarPlannerROSTest::SetStart(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start) {
  start_mutex_.lock();

  LOG(INFO) << "A new start is received.";
  start_.header = start->header;
  start_.pose = start->pose.pose;
  start_received_ = true;

  start_mutex_.unlock();

  MakePlan();
}

void AStarPlannerROSTest::SetGoal(
    const geometry_msgs::PoseStamped::ConstPtr& goal) {
  goal_mutex_.lock();

  LOG(INFO) << "A new goal is received.";
  goal_ = *goal;
  goal_received_ = true;

  goal_mutex_.unlock();

  MakePlan();
}

void AStarPlannerROSTest::MakePlan() {
  std::lock_guard<std::mutex> start_lock(start_mutex_);
  std::lock_guard<std::mutex> goal_lock(goal_mutex_);
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> map_lock(
      *(costmap_ros_->getCostmap()->getMutex()));

  if (!start_received_ || !goal_received_) {
    return;
  }

  std::vector<geometry_msgs::PoseStamped> plan;
  if (planner_->makePlan(start_, goal_, plan)) {
    LOG(INFO) << "Successfully find a grid path via A* algorithm.";
  } else {
    LOG(ERROR) << "Failed to find a grid path via A* algorithm.";
  }
}

}  // namespace astar_planner_ros

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "astar_planner_ros_test");
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  astar_planner_ros::AStarPlannerROSTest astar_planner_ros_test(buffer);
  astar_planner_ros_test.Initialize();
  ros::spin();

  return 0;
}