#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <rclcpp/node_options.hpp>
#include <vector>
#include <string>
#include <map>
#include <array>
#include <limits>
#include <algorithm>
#include <jsoncpp/json/json.h>
#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <image_pipeline_msgs/msg/bounding_box2_d_array.hpp>
#include <image_pipeline_msgs/msg/colors.hpp>

using json = nlohmann::json;
namespace image_pipeline {
class BuoyColor : public rclcpp::Node {
public:

  // Constructor expects exactly three JSON file paths
  BuoyColor(const rclcpp::NodeOptions & options) : Node("color_detector", options) {
    declare_parameter("hist_paths", std::vector<std::string>());
    declare_parameter("json_file_path", "UNSET");
    declare_parameter("in_image_topic", "UNSET");
    declare_parameter("in_detection_topic", "UNSET");
    declare_parameter("out_topic", "UNSET");
    declare_parameter("hist_threshold", 0.0);
    declare_parameter("min_votes_required", 0);

    mInImageTopic_ = get_parameter("in_image_topic").as_string();
    mInDetectionTopic_ = get_parameter("in_detection_topic").as_string();
    mOutTopic_ = get_parameter("out_topic").as_string();
    hist_threshold_ = get_parameter("hist_threshold").as_double();
    min_votes_required_ = get_parameter("min_votes_required").as_int();

    const std::vector<std::string> hist_paths = get_parameter("hist_paths").as_string_array();
    const std::string json_file_path = get_parameter("json_file_path").as_string();

    // 2) Load all three JSON histogram files
    loadMultipleHistograms(hist_paths);

    // 3) Now that we know which colors exist, assign per-color channel weights.
    //    The weight order is: { w_HS2D, w_H1D, w_S1D, w_V1D, w_L, w_a, w_b }.

    std::ifstream json_file(json_file_path);
    if (!json_file.is_open()) {
      throw std::runtime_error("Unable to open JSON file: " + json_file_path);
    }
    json j;
    json_file >> j;

    if (!j.contains("color_weights") || !j["color_weights"].is_object()) {
      throw std::runtime_error("Invalid JSON format: Missing 'color_weights' object.");
    }

    for (const auto& [color, weights] : j["color_weights"].items()) {
      channel_weights_[color] = weights.get<std::vector<double>>();
    }

    printChannelWeights();

    // 4) Initialize subscribers / synchronizer / publisher
    initializeROSComponents();

    // 5) Create display windows
    cv::namedWindow("Detection Result", cv::WINDOW_NORMAL);
    // cv::namedWindow("Circle Mask",      cv::WINDOW_NORMAL);
  }

private:
  std::string mInImageTopic_;
  std::string mInDetectionTopic_;
  std::string mOutTopic_;
  void printChannelWeights() const {
    for (const auto& kv : channel_weights_) {
      std::cout << kv.first << ": [";
      for (const auto& weight : kv.second) {
        std::cout << weight << " ";
      }
      std::cout << "]\n";
    }
  }
  struct ColorHistograms {
    cv::Mat hs2d;      // 2D H×S reference histogram
    cv::Mat h1d;       // 1D H reference histogram
    cv::Mat s1d;       // 1D S reference histogram
    cv::Mat v1d;       // 1D V reference histogram
    cv::Mat L;         // 1D L* reference histogram (Lab)
    cv::Mat a;         // 1D a* reference histogram (Lab)
    cv::Mat b;         // 1D b* reference histogram (Lab)
  };

  struct HistogramData {
    std::array<int, 2> bins_2d;
    std::array<std::array<float, 2>, 2> ranges_2d;
    std::vector<int> bins_1d;
    std::vector<std::array<float, 2>> ranges_1d;
    std::array<int, 3> lab_bins;
    std::array<std::array<float, 2>, 3> lab_ranges;
    std::map<std::string, ColorHistograms> color_hists;
    bool initialized = false;
  } hist_data_;

  double hist_threshold_;
  int min_votes_required_;
  const float ROI_SCALE = 1.0f;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
  std::shared_ptr<message_filters::Subscriber<image_pipeline_msgs::msg::BoundingBox2DArray>> boxes_sub_;

  // Synchronized callback
  void detectionCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
                         const image_pipeline_msgs::msg::BoundingBox2DArray::ConstSharedPtr &boxes_msg);

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, image_pipeline_msgs::msg::BoundingBox2DArray>;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> _sync;

  rclcpp::Publisher<image_pipeline_msgs::msg::Colors>::SharedPtr colors_pub_;

  // Per-color channel weights { w_HS2D, w_H1D, w_S1D, w_V1D, w_L, w_a, w_b }
  std::map<std::string, std::vector<double>> channel_weights_;

  // ------------------------------------------------------------
  // 1) Initialize ROS subscribers, synchronizer, and publisher
  void initializeROSComponents();

  // ------------------------------------------------------------
  // 2) Load three JSON files into hist_data_.color_hists
  void loadMultipleHistograms(const std::vector<std::string>& paths);

  // Extract color name from filename "something/<color>_histograms.json"
  std::string extractColorName(const std::string& filepath);

  // Load a single JSON ("<color>_histograms.json") into hist_data_
  void loadHistogramsFromFile(const std::string& path);

  // Build a 2D H×S histogram from a query ROI (HSV) with a circular mask
  cv::Mat computeHS2DHist(const cv::Mat& hsv, const cv::Mat& mask);

  // Build a 1D histogram for one channel (0=H, 1=S, 2=V) with circular mask
  cv::Mat compute1DHist(const cv::Mat& hsv, int channel, int bins, const std::array<float,2>& range, const cv::Mat& mask);

  // Compute Lab channel histograms
  cv::Mat computeLabHist(const cv::Mat& lab, int channel, const cv::Mat& mask);

  // Compare two histograms by Bhattacharyya distance
  double compareHistogram(const cv::Mat& a, const cv::Mat& b);

  // Voting logic with tie-breaking by per-color weighted distance
  std::string classifyROI(const cv::Mat& roi_bgr);

  // Use the full bounding box (no scaling)
  cv::Rect getScaledROI(const cv::Rect& orig);

  // For each detection: classify, draw, and publish
  std::string processDetection(cv::Mat& frame_bgr, const image_pipeline_msgs::msg::BoundingBox2D &box, const bool enablePlot = false);

};
}  // namespace image_pipeline
