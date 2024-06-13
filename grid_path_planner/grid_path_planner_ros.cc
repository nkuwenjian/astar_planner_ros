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

#include "grid_path_planner/grid_path_planner_ros.h"

#include "costmap_2d/cost_values.h"
#include "costmap_2d/inflation_layer.h"
#include "glog/logging.h"
#include "nav_msgs/Path.h"
#include "pluginlib/class_list_macros.hpp"
#include "tf/tf.h"

#include "grid_path_planner/common/utils.h"

PLUGINLIB_EXPORT_CLASS(grid_path_planner::GridPathPlannerROS,
                       nav_core::BaseGlobalPlanner)

namespace grid_path_planner {

void GridPathPlannerROS::initialize(std::string name,
                                    costmap_2d::Costmap2DROS* costmap_ros) {
  // Check if GridPathPlannerROS has been initialized.
  if (initialized_) {
    LOG(INFO) << "GridPathPlannerROS has been initialized.";
    return;
  }

  // Check and update costmap.
  if (!UpdateCostmap(costmap_ros)) {
    LOG(ERROR) << "Failed to update costmap.";
    return;
  }

  ros::NodeHandle private_nh("~/" + name);
  VLOG(4) << "Name is " << name;
  name_ = name;
  LoadRosParamFromNodeHandle(private_nh);

  circumscribed_cost_ = ComputeCircumscribedCost();
  if (circumscribed_cost_ == 0U) {
    LOG(ERROR) << std::fixed
               << "The costmap value at the robot's circumscribed radius ("
               << layered_costmap_->getCircumscribedRadius() << " m) is 0.";
    return;
  }

  grid_path_planner_ = std::make_unique<grid_search::GridSearch>();
  grid_path_planner_->Init(static_cast<int>(costmap_2d_->getSizeInCellsX()),
                           static_cast<int>(costmap_2d_->getSizeInCellsY()),
                           costmap_2d_->getResolution(), circumscribed_cost_,
                           grid_search::GridSearch::SearchType::kAStar);

  plan_pub_ = private_nh.advertise<nav_msgs::Path>("grid_path", 1);

  initialized_ = true;
  LOG(INFO) << "GridPathPlannerROS is initialized successfully.";
}

bool GridPathPlannerROS::UpdateCostmap(costmap_2d::Costmap2DROS* costmap_ros) {
  if (costmap_ros == nullptr) {
    LOG(ERROR) << "costmap_ros == nullptr";
    return false;
  }

  costmap_ros_ = costmap_ros;
  costmap_2d_ = costmap_ros->getCostmap();
  layered_costmap_ = costmap_ros->getLayeredCostmap();
  if (costmap_2d_ == nullptr || layered_costmap_ == nullptr) {
    LOG(ERROR) << "costmap_2d_ == nullptr || layered_costmap_ == nullptr";
    return false;
  }
  return true;
}

void GridPathPlannerROS::LoadRosParamFromNodeHandle(const ros::NodeHandle& nh) {
  // Sanity checks.
  CHECK_NOTNULL(costmap_2d_);

  nh.param("treat_unknown_as_free", treat_unknown_as_free_, true);
  VLOG(4) << std::boolalpha
          << "treat_unknown_as_free: " << treat_unknown_as_free_;
}

uint8_t GridPathPlannerROS::ComputeCircumscribedCost() const {
  // Sanity checks.
  CHECK_NOTNULL(layered_costmap_);
  CHECK_NOTNULL(costmap_2d_);

  std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins =
      layered_costmap_->getPlugins();
  if (plugins == nullptr) {
    LOG(ERROR) << "plugins == nullptr";
    return 0U;
  }

  // Check if the costmap has an inflation layer.
  for (auto& plugin : *plugins) {
    auto inflation_layer =
        boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(plugin);
    if (inflation_layer != nullptr) {
      return inflation_layer->computeCost(
          layered_costmap_->getCircumscribedRadius() /
          costmap_2d_->getResolution());
    }
  }
  return 0U;
}

void GridPathPlannerROS::GetStartAndEndConfigurations(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, double resolution, double origin_x,
    double origin_y, int* start_x, int* start_y, int* end_x, int* end_y) {
  // Sanity checks.
  CHECK_NOTNULL(start_x);
  CHECK_NOTNULL(start_y);
  CHECK_NOTNULL(end_x);
  CHECK_NOTNULL(end_y);

  VLOG(4) << std::fixed << "start point: " << start.pose.position.x << ","
          << start.pose.position.y;
  VLOG(4) << std::fixed << "end point: " << goal.pose.position.x << ","
          << goal.pose.position.y;

  // Start configuration.
  *start_x = common::ContXY2Disc(start.pose.position.x - origin_x, resolution);
  *start_y = common::ContXY2Disc(start.pose.position.y - origin_y, resolution);

  // End configuration.
  *end_x = common::ContXY2Disc(goal.pose.position.x - origin_x, resolution);
  *end_y = common::ContXY2Disc(goal.pose.position.y - origin_y, resolution);
}

std::vector<std::vector<uint8_t>> GridPathPlannerROS::GetGridMap(
    const uint8_t* char_map, unsigned int size_x, unsigned int size_y,
    bool treat_unknown_as_free) {
  if (char_map == nullptr) {
    LOG(ERROR) << "char_map == nullptr";
    return std::vector<std::vector<uint8_t>>();
  }
  std::vector<std::vector<uint8_t>> grid_map;
  grid_map.resize(size_x);
  for (unsigned int i = 0U; i < size_x; ++i) {
    grid_map[i].resize(size_y);
    for (unsigned int j = 0U; j < size_y; ++j) {
      grid_map[i][j] = char_map[i + j * size_x];
      if (treat_unknown_as_free &&
          grid_map[i][j] == costmap_2d::NO_INFORMATION) {
        grid_map[i][j] = costmap_2d::FREE_SPACE;
      }
    }
  }
  return grid_map;
}

bool GridPathPlannerROS::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
  // Check if GridPathPlannerROS has been initialized.
  if (!initialized_) {
    LOG(ERROR) << "GridPathPlannerROS has not been initialized.";
    return false;
  }

  // Get start and end configurations.
  int start_x = 0;
  int start_y = 0;
  int end_x = 0;
  int end_y = 0;
  GetStartAndEndConfigurations(
      start, goal, costmap_2d_->getResolution(), costmap_2d_->getOriginX(),
      costmap_2d_->getOriginY(), &start_x, &start_y, &end_x, &end_y);

  // Get map data from cost map.
  std::vector<std::vector<uint8_t>> grid_map =
      GetGridMap(costmap_2d_->getCharMap(), costmap_2d_->getSizeInCellsX(),
                 costmap_2d_->getSizeInCellsY(), treat_unknown_as_free_);
  if (grid_map.empty()) {
    LOG(ERROR) << "Failed to get grid map.";
    return false;
  }

  // Search path via A-star algorithm.
  grid_search::GridAStarResult result;
  if (!grid_path_planner_->GenerateGridPath(start_x, start_y, end_x, end_y,
                                            grid_map, &result)) {
    LOG(ERROR) << "Failed to find a grid path.";
    return false;
  }

  // Populate global path.
  PopulateGlobalPlan(result, start.header, costmap_2d_->getResolution(),
                     costmap_2d_->getOriginX(), costmap_2d_->getOriginY(),
                     &plan);

  // Publish Voronoi path.
  PublishGlobalPlan(plan, plan_pub_);

  return true;
}

void GridPathPlannerROS::PopulateGlobalPlan(
    const grid_search::GridAStarResult& result, const std_msgs::Header& header,
    double resolution, double origin_x, double origin_y,
    std::vector<geometry_msgs::PoseStamped>* plan) {
  // Sanity checks.
  CHECK_EQ(result.x.size(), result.y.size());
  CHECK_NOTNULL(plan);
  plan->clear();

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = header;
  const std::size_t N = result.x.size();
  for (size_t i = 0; i < N; i++) {
    pose_stamped.pose.position.x =
        common::DiscXY2Cont(result.x[i], resolution) + origin_x;
    pose_stamped.pose.position.y =
        common::DiscXY2Cont(result.y[i], resolution) + origin_y;
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    plan->push_back(pose_stamped);
  }
}

void GridPathPlannerROS::PublishGlobalPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const ros::Publisher& pub) {
  if (plan.empty()) {
    return;
  }

  nav_msgs::Path gui_path;
  gui_path.header = plan.front().header;
  gui_path.poses = plan;
  pub.publish(gui_path);
}

}  // namespace grid_path_planner
