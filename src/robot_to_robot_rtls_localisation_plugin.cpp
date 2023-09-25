// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// std
#include <functional>
#include <memory>
#include <string>
#include <utility>

// romea
#include "romea_robot_to_robot_localisation_rtls_plugin/robot_to_robot_rtls_localisation_plugin.hpp"
#include "romea_rtls_transceiver_utils/rtls_transceiver_data_conversions.hpp"
#include "romea_common_utils/params/algorithm_parameters.hpp"
#include "romea_common_utils/qos.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
R2RRTLSLocalisationPlugin::R2RRTLSLocalisationPlugin(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("robot_to_robot_rtls_localisation_plugin", options)),
  plugin_(nullptr),
  scheduler_(nullptr),
  range_observation_(),
  leader_pose_observation_(),
  rtls_communication_hub_(nullptr),
  range_pub_(nullptr),
  leader_pose_pub_(nullptr),
  diagnostic_pub_(nullptr),
  range_stamp_()
{
  declare_parameters_();
  init_plugin_();
  init_communication_hub_();
  init_range_publisher_();
  init_leader_pose_publisher_();
  init_leader_twist_publisher_();
  init_diagnostic_publisher_();
  init_scheduler_();
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
R2RRTLSLocalisationPlugin::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
void R2RRTLSLocalisationPlugin::declare_parameters_()
{
  declare_base_footprint_frame_id(node_);
  declare_poll_rate(node_);
  declare_range_std(node_);
  declare_minimal_range(node_);
  declare_maximal_range(node_);
  declare_initiators_ids(node_);
  declare_initiators_names(node_);
  declare_initiators_positions(node_);
  declare_responders_ids(node_);
  declare_responders_names(node_);
  declare_responders_positions(node_);
}

//-----------------------------------------------------------------------------
void R2RRTLSLocalisationPlugin::init_communication_hub_()
{
  using namespace std::placeholders;
  auto payload_cb = std::bind(&R2RRTLSLocalisationPlugin::process_payload_, this, _1);
  auto range_cb = std::bind(&R2RRTLSLocalisationPlugin::process_range_, this, _1, _2, _3);
  rtls_communication_hub_ = std::make_unique<RTLSCommunicationHub>(node_, range_cb, payload_cb);
}

//-----------------------------------------------------------------------------
void R2RRTLSLocalisationPlugin::init_range_publisher_()
{
  range_pub_ = node_->create_publisher<ObservationRangeStampedMsg>(
    "range", sensor_data_qos());
}

//-----------------------------------------------------------------------------
void R2RRTLSLocalisationPlugin::init_leader_pose_publisher_()
{
  leader_pose_pub_ = make_stamped_data_publisher<ObservationPose, ObservationPose2DStampedMsg>(
    node_, "leader_pose", get_base_footprint_frame_id(node_), sensor_data_qos(), true);
}

//-----------------------------------------------------------------------------
void R2RRTLSLocalisationPlugin::init_leader_twist_publisher_()
{
  leader_twist_pub_ = make_stamped_data_publisher<ObservationTwist, ObservationTwist2DStampedMsg>(
    node_, "leader_twist", "leader_base_link", sensor_data_qos(), true);
}

//-----------------------------------------------------------------------------
void R2RRTLSLocalisationPlugin::init_diagnostic_publisher_()
{
  diagnostic_pub_ = make_diagnostic_publisher<DiagnosticReport>(node_, node_->get_name(), 1.0);
}

//-----------------------------------------------------------------------------
void R2RRTLSLocalisationPlugin::init_scheduler_()
{
  using namespace std::placeholders;
  auto cb = std::bind(&R2RRTLSLocalisationPlugin::process_ranging_request_, this, _1, _2, _3);

  scheduler_ = std::make_unique<Scheduler>(
    get_poll_rate(node_),
    get_initiators_names(node_),
    get_responders_names(node_),
    cb);

  scheduler_->start();
}

//-----------------------------------------------------------------------------
void R2RRTLSLocalisationPlugin::init_plugin_()
{
  plugin_ = std::make_unique<Plugin>(
    get_range_std(node_),
    get_minimal_range(node_),
    get_maximal_range(node_),
    20,   // rxPowerRejectionThreshold
    get_initiators_positions(node_),
    get_responders_positions(node_));
}

//-----------------------------------------------------------------------------
void R2RRTLSLocalisationPlugin::process_ranging_request_(
  const size_t & initiator_index,
  const size_t & responder_index,
  const Duration & timeout)
{
  rtls_communication_hub_->send_ranging_request(
    initiator_index, responder_index, durationToSecond(timeout));

  if (initiator_index == 0 && responder_index == 0) {
    diagnostic_pub_->publish(node_->get_clock()->now(), scheduler_->getReport());
  }
}

//-----------------------------------------------------------------------------
void R2RRTLSLocalisationPlugin::process_range_(
  const size_t & initiator_index,
  const size_t & responder_index,
  const RangeMsg & range)
{
  range_stamp_ = range.stamp;
  RangingResult ranging_result = to_romea(range);

  if (plugin_->processRangingResult(
      initiator_index, responder_index,
      ranging_result, range_observation_))
  {
    publish_range_(range_stamp_, "toto");
  }

  if (plugin_->computeLeaderPose(leader_pose_observation_)) {
    leader_pose_pub_->publish(range_stamp_, leader_pose_observation_);
  }

  scheduler_->feedback(initiator_index, responder_index, ranging_result);
}

//-----------------------------------------------------------------------------
void R2RRTLSLocalisationPlugin::process_payload_(const PayloadMsg & payload)
{
  if (!payload.data.empty()) {
    auto leader_twist = deserializeTwist2D(payload.data);
    leader_twist_observation_.firstMoment(0) = leader_twist.linearSpeeds.x();
    leader_twist_observation_.firstMoment(1) = leader_twist.linearSpeeds.x();
    leader_twist_observation_.firstMoment(2) = leader_twist.angularSpeed;
    leader_twist_observation_.secondMoment = leader_twist.covariance;
    leader_twist_pub_->publish(range_stamp_, leader_twist_observation_);
  }
}

//-----------------------------------------------------------------------------
void R2RRTLSLocalisationPlugin::publish_range_(
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  auto range_msg = std::make_unique<ObservationRangeStampedMsg>();
  to_ros_msg(stamp, frame_id, range_observation_, *range_msg);
  range_pub_->publish(std::move(range_msg));
}

}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::R2RRTLSLocalisationPlugin)
