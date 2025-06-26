#include "watchdog/watchdog.hpp"
#include <algorithm>
#include <rclcpp_components/register_node_macro.hpp>
namespace image_pipeline
{
WatchDog::WatchDog(const rclcpp::NodeOptions & options)
  : Node("watchdog", options)
{
  declare_parameter("timeout_milliseconds", 0);
  declare_parameter("watchdog_checking_rate", 0);
  declare_parameter("topics_to_monitor", std::vector<std::string>());
  declare_parameter("out_topic", "UNSET");
  mTimeoutMilliseconds_ = get_parameter("timeout_milliseconds").as_int();
  mTopicsToMonitor_ = get_parameter("topics_to_monitor").as_string_array();
  mOutTopic_ = get_parameter("out_topic").as_string();
  mWatchDogCheckingRate_ = get_parameter("watchdog_checking_rate").as_int();

  for (size_t index = 0; const auto &subTopic : mTopicsToMonitor_) {
    mLatestTimestamps_.push_back(std::chrono::system_clock::now());
    mSubscribers_.emplace_back(
      create_subscription<std_msgs::msg::Empty>(subTopic, 10, [index, this](std_msgs::msg::Empty::SharedPtr msg){
      (void) msg;
      mLatestTimestamps_[index] = std::chrono::system_clock::now();
    })
      );
    index++;
  }

  mWatchDogTimer_ = create_wall_timer(
    std::chrono::milliseconds(mWatchDogCheckingRate_),
    std::bind(&WatchDog::heartbeatPub, this));

  mHeartBeatPublisher_.reset(
    new realtime_tools::RealtimePublisher<std_msgs::msg::Bool>(
      create_publisher<std_msgs::msg::Bool>(
        mOutTopic_,
        1
        )
      )
    );
}

void WatchDog::heartbeatPub(){
  std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
  if (mHeartBeatPublisher_->trylock()) {
    mHeartBeatPublisher_->msg_.data = std::all_of(mLatestTimestamps_.begin(), mLatestTimestamps_.end(), [now, this](std::chrono::time_point<std::chrono::system_clock> timestamp){
      auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - timestamp);
      return milliseconds.count() < mTimeoutMilliseconds_;
    });
    mHeartBeatPublisher_->unlockAndPublish();
  }
}
}

RCLCPP_COMPONENTS_REGISTER_NODE(image_pipeline::WatchDog);
