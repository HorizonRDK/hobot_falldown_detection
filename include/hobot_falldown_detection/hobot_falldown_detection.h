// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef INCLUDE_HOBOT_FALLDOWN_DETECTION_HOBOT_FALLDOWN_DETECTION_H_
#define INCLUDE_HOBOT_FALLDOWN_DETECTION_HOBOT_FALLDOWN_DETECTION_H_

#include <string>
#include <vector>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "ai_msgs/msg/target.hpp"
#include "ai_msgs/msg/point.hpp"


#define TargetTypePersion "person"
#define PointTypeBody_kps "body_kps"
#define BodyKpsSize (19)

typedef enum {
    ExLow = 0,
    Low,
    Middle,
    High
}Sensivity;


using rclcpp::NodeOptions;
using ai_msgs::msg::PerceptionTargets;

class hobot_falldown_detection: public rclcpp::Node
{
 public:
    explicit hobot_falldown_detection(
        const NodeOptions &options = NodeOptions(),
        std::string node_name = "body_kps_node");
    ~hobot_falldown_detection();

 private:
    void topic_callback(
        const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

    bool IsFallDown(
        const std::vector<geometry_msgs::msg::Point32> &body_kps);

    void PointDebugInfo(
        const std::vector<geometry_msgs::msg::Point32> &body_kps);

    void PublishFallDownEvent(
        const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg,
        ai_msgs::msg::PerceptionTargets::UniquePtr publish_data);

    float upper_body_low_ = 30.0f;
    float upper_body_high_ = 66.0f;
    float lower_body_low_ = 45.0f;
    float lower_body_high_ = 80.0f;
    float differ_ = 30.0f;

    const float PI = 3.14159265f;

    int paramSensivity = 3;
    std::string body_kps_topic_name = "hobot_mono2d_body_detection";

    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::ConstSharedPtr
                                        subscription_ = nullptr;

    std::string msg_falldown_topic_name = "falldown_event";
    rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr
    falldown_publisher_;
};

#endif  // INCLUDE_HOBOT_FALLDOWN_DETECTION_HOBOT_FALLDOWN_DETECTION_H_

