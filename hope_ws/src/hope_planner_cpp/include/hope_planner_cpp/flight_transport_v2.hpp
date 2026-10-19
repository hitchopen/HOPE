#pragma once
#include "hope_msgs/msg/ball_observation_v2.hpp"
#include "hope_msgs/msg/flight_packet_v2.hpp"
#include "hope_msgs/msg/flight_prediction_v2.hpp"
#include "hope_planner_cpp/flight_state_v2.hpp"
namespace hope_planner_cpp {
const char *name(EffectiveSpinMode);
const char *name(EventStatus);
const char *name(ForecastStatus);
BallSample decode_observation(const hope_msgs::msg::BallObservationV2 &);
hope_msgs::msg::BallObservationV2 encode_observation(const BallSample &);
hope_msgs::msg::FlightPredictionV2 encode_prediction(const FlightPrediction &,
                                                     const ShotContext &);
std::string packet_v2_hash(hope_msgs::msg::FlightPacketV2);
bool validate_packet_v2(const hope_msgs::msg::FlightPacketV2 &,
                        std::string &reason);
} // namespace hope_planner_cpp
namespace hope_planner_cpp {
hope_msgs::msg::FlightPacketV2 encode_packet_v2(const FlightPipeline &,
                                                const FlightPrediction &,
                                                const std::string &session,
                                                const std::string &producer,
                                                const std::string &config);
void restore_packet_v2(FlightPipeline &, const hope_msgs::msg::FlightPacketV2 &,
                       const std::string &expected_config);
} // namespace hope_planner_cpp
