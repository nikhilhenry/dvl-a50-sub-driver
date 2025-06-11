#include <geometry_msgs/Vector3.h>
#include <mavros/mavros_plugin.h>
#include <mavros_dvl_extras/PositionDelta.h>

namespace mavros {
namespace extra_plugins {
class VisionPositionDeltaPlugin : public plugin::PluginBase {
public:
  VisionPositionDeltaPlugin() : PluginBase(), sp_nh("~vision_position") {}

  void initialize(UAS &uas_) override {
    PluginBase::initialize(uas_);

    vision_delta_sub = sp_nh.subscribe(
        "delta", 10, &VisionPositionDeltaPlugin::vector_cb, this);
  }

  Subscriptions get_subscriptions() override { return {/* Rx disabled */}; }

private:
  ros::NodeHandle sp_nh;

  ros::Subscriber
      vision_delta_sub;

  uint64_t last_sent_usec = 0;

  void send_vision_speed_estimate(const uint64_t c_usec, const uint64_t d_usec,
                                  const geometry_msgs::Vector3 pd,
                                  const geometry_msgs::Vector3 ad,
                                  const float conf) {
    mavlink::ardupilotmega::msg::VISION_POSITION_DELTA vd{};

    vd.time_usec = c_usec;
    vd.time_delta_usec = d_usec;

    vd.position_delta = {static_cast<float>(pd.x), static_cast<float>(pd.y),
                         static_cast<float>(pd.z)};
    vd.angle_delta = {static_cast<float>(ad.x), static_cast<float>(ad.y),
                      static_cast<float>(ad.z)};

    vd.confidence = conf;

    UAS_FCU(m_uas)->send_message_ignore_drop(vd);
  }

  /* -*- callbacks -*- */
  void vector_cb(const mavros_dvl_extras::PositionDelta::ConstPtr &req) {

    auto time_usec = req->header.stamp.toNSec() / 1000;
    uint64_t time_delta;
    if (this->last_sent_usec == 0)
      time_delta = 0.0;
    else
      time_delta = time_usec - this->last_sent_usec;
    this->last_sent_usec = time_usec;
    send_vision_speed_estimate(time_usec, time_delta, req->position_delta,
                               req->angle_delta, req->confidence);
  }
};
} // namespace extra_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VisionPositionDeltaPlugin,
                       mavros::plugin::PluginBase)
