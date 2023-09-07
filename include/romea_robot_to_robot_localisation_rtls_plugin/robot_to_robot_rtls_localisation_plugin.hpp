// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_ROBOT_TO_WOLRD_RTLS_LOCALISATION_PLUGIN_HPP_
#define ROMEA_ROBOT_TO_WOLRD_RTLS_LOCALISATION_PLUGIN_HPP_


// std
#include <string>
#include <memory>

// romea
#include "romea_core_localisation_rtls/R2RLocalisationRTLSPlugin.hpp"
#include "romea_core_rtls/coordination/RTLSSimpleCoordinatorScheduler.hpp"
#include "romea_core_rtls/serialization/Twist2DSerialization.hpp"
#include "romea_common_utils/conversions/diagnostic_conversions.hpp"
#include "romea_common_utils/publishers/diagnostic_publisher.hpp"
#include "romea_common_utils/publishers/stamped_data_publisher.hpp"
#include "romea_localisation_utils/conversions/observation_range_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_pose_conversions.hpp"
#include "romea_localisation_utils/conversions/observation_twist_conversions.hpp"
#include "romea_rtls_utils/rtls_communication_hub.hpp"
#include "romea_rtls_utils/rtls_parameters.hpp"

// local
#include "romea_robot_to_robot_localisation_rtls_plugin/visibility_control.h"

namespace romea
{

class R2RRTLSLocalisationPlugin
{
public:
  using Plugin = R2RLocalisationRTLSPlugin;
  using Scheduler = RTLSSimpleCoordinatorScheduler;
  using RangingResult = RTLSTransceiverRangingResult;

  using RangeMsg = romea_rtls_transceiver_msgs::msg::Range;
  using PayloadMsg = romea_rtls_transceiver_msgs::msg::Payload;
  using ObservationRangeStampedMsg = romea_localisation_msgs::msg::ObservationRangeStamped;
  using ObservationPose2DStampedMsg = romea_localisation_msgs::msg::ObservationPose2DStamped;

public:
  ROMEA_ROBOT_TO_ROBOT_LOCALISATION_RTLS_PUBLIC
  explicit R2RRTLSLocalisationPlugin(const rclcpp::NodeOptions & options);

  ROMEA_ROBOT_TO_ROBOT_LOCALISATION_RTLS_PUBLIC
  virtual ~R2RRTLSLocalisationPlugin() = default;

  ROMEA_ROBOT_TO_ROBOT_LOCALISATION_RTLS_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

protected:
  void declare_parameters_();

  void init_plugin_();

  void init_scheduler_();

  void init_communication_hub_();

  void init_range_publisher_();

  void init_leader_pose_publisher_();

  void init_leader_twist_publisher_();

  void init_diagnostic_publisher_();

  void process_ranging_request_(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const Duration & timeout);

  void process_range_(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const RangeMsg & range);

  void process_payload_(const PayloadMsg & payload);

  void publish_range_(const rclcpp::Time & stamp, const std::string & frame_id);

protected:
  rclcpp::Node::SharedPtr node_;

  std::unique_ptr<Plugin> plugin_;
  std::unique_ptr<Scheduler> scheduler_;
  romea::ObservationRange range_observation_;
  romea::ObservationPose leader_pose_observation_;
  romea::ObservationTwist leader_twist_observation_;

  std::unique_ptr<RTLSCommunicationHub> rtls_communication_hub_;
  rclcpp::Publisher<ObservationRangeStampedMsg>::SharedPtr range_pub_;
  std::shared_ptr<StampedPublisherBase<ObservationPose>> leader_pose_pub_;
  std::shared_ptr<StampedPublisherBase<ObservationTwist>> leader_twist_pub_;
  std::shared_ptr<StampedPublisherBase<DiagnosticReport>> diagnostic_pub_;
  rclcpp::Time range_stamp_;
};

}  // namespace romea

#endif  // ROMEA_ROBOT_TO_WOLRD_RTLS_LOCALISATION_PLUGIN_HPP_
