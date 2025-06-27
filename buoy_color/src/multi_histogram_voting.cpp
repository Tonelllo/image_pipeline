#include "buoy_color/multi_histogram_voting.hpp"
#include <message_filters/subscriber.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/detail/image__struct.hpp>

namespace image_pipeline {
void BuoyColor::initializeROSComponents() {
  // Create subscribers for image and detected boxes
  rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;
  image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, mInImageTopic_, custom_qos);
  boxes_sub_ = std::make_shared<message_filters::Subscriber<image_pipeline_msgs::msg::BoundingBox2DArray>>(
    this, mInDetectionTopic_);

  // Create synchronizer
  //sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *image_sub_, *boxes_sub_);
  _sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(400));
  _sync->connectInput(*image_sub_, *boxes_sub_);
  _sync->registerCallback(std::bind(&BuoyColor::detectionCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Create publisher for object colors
  colors_pub_ = this->create_publisher<image_pipeline_msgs::msg::Colors>(mOutTopic_, 10);
}


// ------------------------------------------------------------
// 2) Load three JSON files into hist_data_.color_hists
void BuoyColor::loadMultipleHistograms(const std::vector<std::string>& paths) {
  if (paths.size() != 3) {
    throw std::runtime_error("Exactly three JSON paths must be provided");
  }
  for (const auto& path : paths) {
    loadHistogramsFromFile(path);
  }
  if (hist_data_.color_hists.empty()) {
    throw std::runtime_error("No histograms were loaded from any JSON.");
  }
  if (mShowDebugPrints_) {
    std::cerr << "Total colors loaded: " << hist_data_.color_hists.size() << std::endl;
  }
}

// Extract color name from filename "something/<color>_histograms.json"
std::string BuoyColor::extractColorName(const std::string& filepath) {
  size_t slash_pos = filepath.find_last_of("/\\");
  std::string filename = (slash_pos == std::string::npos) ? filepath
                                                            : filepath.substr(slash_pos + 1);
  const std::string suffix = "_histograms.json";
  if (filename.size() > suffix.size()
      && filename.substr(filename.size() - suffix.size()) == suffix)
  {
    return filename.substr(0, filename.size() - suffix.size());
  }
  size_t dotpos = filename.rfind(".json");
  if (dotpos != std::string::npos) {
    return filename.substr(0, dotpos);
  }
  return filename;
}

// Load a single JSON ("<color>_histograms.json") into hist_data_
void BuoyColor::loadHistogramsFromFile(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Histogram file not found: " + path);
  }
  nlohmann::json root;

  if (file.is_open()) {
    file >> root;
    file.close();
  } else {
    std::cerr << "Unable to open file" << std::endl;
  }
  if (!hist_data_.initialized) {
    // HSV parameters
    hist_data_.bins_2d[0] = root["metadata"]["hist_2d_bins"][0].get<int>();
    hist_data_.bins_2d[1] = root["metadata"]["hist_2d_bins"][1].get<int>();
    hist_data_.ranges_2d[0][0] = root["metadata"]["ranges_2d"][0].get<float>();
    hist_data_.ranges_2d[0][1] = root["metadata"]["ranges_2d"][1].get<float>();
    hist_data_.ranges_2d[1][0] = root["metadata"]["ranges_2d"][2].get<float>();
    hist_data_.ranges_2d[1][1] = root["metadata"]["ranges_2d"][3].get<float>();

    hist_data_.bins_1d.push_back(root["metadata"]["hist_1d_bins"][0].get<int>());
    hist_data_.bins_1d.push_back(root["metadata"]["hist_1d_bins"][1].get<int>());
    hist_data_.bins_1d.push_back(root["metadata"]["hist_1d_bins"][2].get<int>());

    for (int i = 0; i < 3; ++i) {
      std::array<float, 2> r = {
        root["metadata"]["ranges_1d"][2 * i].get<float>(),
        root["metadata"]["ranges_1d"][2 * i + 1].get<float>()
      };
      hist_data_.ranges_1d.push_back(r);
    }

    // Lab parameters
    hist_data_.lab_bins[0] = root["metadata"]["lab_bins"][0].get<int>();
    hist_data_.lab_bins[1] = root["metadata"]["lab_bins"][1].get<int>();
    hist_data_.lab_bins[2] = root["metadata"]["lab_bins"][2].get<int>();

    for (int i = 0; i < 3; ++i) {
      hist_data_.lab_ranges[i][0] = root["metadata"]["lab_ranges"][2 * i].get<float>();
      hist_data_.lab_ranges[i][1] = root["metadata"]["lab_ranges"][2 * i + 1].get<float>();
    }

    hist_data_.initialized = true;
  } else {
    // Verify 2D bins match
    int b2_0 = root["metadata"]["hist_2d_bins"][0].get<int>();
    int b2_1 = root["metadata"]["hist_2d_bins"][1].get<int>();
    if (b2_0 != hist_data_.bins_2d[0] || b2_1 != hist_data_.bins_2d[1]) {
      throw std::runtime_error("Mismatched 2D bins in " + path);
    }

    // Verify 2D ranges match
    float r2d0_min = root["metadata"]["ranges_2d"][0].get<float>();
    float r2d0_max = root["metadata"]["ranges_2d"][1].get<float>();
    float r2d1_min = root["metadata"]["ranges_2d"][2].get<float>();
    float r2d1_max = root["metadata"]["ranges_2d"][3].get<float>();
    if (r2d0_min != hist_data_.ranges_2d[0][0] ||
        r2d0_max != hist_data_.ranges_2d[0][1] ||
        r2d1_min != hist_data_.ranges_2d[1][0] ||
        r2d1_max != hist_data_.ranges_2d[1][1])
    {
      throw std::runtime_error("Mismatched 2D ranges in " + path);
    }

    // Verify 1D bins and ranges match
    for (int i = 0; i < 3; ++i) {
      int b1 = root["metadata"]["hist_1d_bins"][i].get<int>();
      if (b1 != hist_data_.bins_1d[i]) {
        throw std::runtime_error("Mismatched 1D bins in " + path);
      }
      float r1_min = root["metadata"]["ranges_1d"][2 * i].get<float>();
      float r1_max = root["metadata"]["ranges_1d"][2 * i + 1].get<float>();
      if (r1_min != hist_data_.ranges_1d[i][0] ||
          r1_max != hist_data_.ranges_1d[i][1])
      {
        throw std::runtime_error("Mismatched 1D ranges in " + path);
      }
    }

    // Verify Lab parameters match
    for (int i = 0; i < 3; ++i) {
      int lab_b = root["metadata"]["lab_bins"][i].get<int>();
      if (lab_b != hist_data_.lab_bins[i]) {
        throw std::runtime_error("Mismatched Lab bins in " + path);
      }
      float lab_min = root["metadata"]["lab_ranges"][2 * i].get<float>();
      float lab_max = root["metadata"]["lab_ranges"][2 * i + 1].get<float>();
      if (lab_min != hist_data_.lab_ranges[i][0] ||
          lab_max != hist_data_.lab_ranges[i][1])
      {
        throw std::runtime_error("Mismatched Lab ranges in " + path);
      }
    }
  }

  // Load this JSON's color histograms
  std::string color_name = extractColorName(path);
  const auto& H = root["histograms"]["hs2d"];
  const auto& h = root["histograms"]["h1d"];
  const auto& s = root["histograms"]["s1d"];
  const auto& v = root["histograms"]["v1d"];
  const auto& L = root["histograms"]["L"];
  const auto& a = root["histograms"]["a"];
  const auto& b = root["histograms"]["b"];

  ColorHistograms ch;
  // Allocate & fill hs2d (size: bins_2d[0] × bins_2d[1])
  ch.hs2d = cv::Mat(hist_data_.bins_2d[0], hist_data_.bins_2d[1], CV_32F);
  for (int i = 0; i < hist_data_.bins_2d[0]; ++i) {
    for (int j = 0; j < hist_data_.bins_2d[1]; ++j) {
      ch.hs2d.at<float>(i, j) = H[i][j].get<float>();
    }
  }
  cv::normalize(ch.hs2d, ch.hs2d, 1.0, 0.0, cv::NORM_L1);

  // Allocate & fill h1d (size: bins_1d[0])
  ch.h1d = cv::Mat(hist_data_.bins_1d[0], 1, CV_32F);
  for (int i = 0; i < hist_data_.bins_1d[0]; ++i) {
    ch.h1d.at<float>(i) = h[i].get<float>();
  }
  cv::normalize(ch.h1d, ch.h1d, 1.0, 0.0, cv::NORM_L1);

  // Allocate & fill s1d (size: bins_1d[1])
  ch.s1d = cv::Mat(hist_data_.bins_1d[1], 1, CV_32F);
  for (int i = 0; i < hist_data_.bins_1d[1]; ++i) {
    ch.s1d.at<float>(i) = s[i].get<float>();
  }
  cv::normalize(ch.s1d, ch.s1d, 1.0, 0.0, cv::NORM_L1);

  // Allocate & fill v1d (size: bins_1d[2])
  ch.v1d = cv::Mat(hist_data_.bins_1d[2], 1, CV_32F);
  for (int i = 0; i < hist_data_.bins_1d[2]; ++i) {
    ch.v1d.at<float>(i) = v[i].get<float>();
  }
  cv::normalize(ch.v1d, ch.v1d, 1.0, 0.0, cv::NORM_L1);

  // Allocate & fill L (size: lab_bins[0])
  ch.L = cv::Mat(hist_data_.lab_bins[0], 1, CV_32F);
  for (int i = 0; i < hist_data_.lab_bins[0]; ++i) {
    ch.L.at<float>(i) = L[i].get<float>();
  }
  cv::normalize(ch.L, ch.L, 1.0, 0.0, cv::NORM_L1);

  // Allocate & fill a (size: lab_bins[1])
  ch.a = cv::Mat(hist_data_.lab_bins[1], 1, CV_32F);
  for (int i = 0; i < hist_data_.lab_bins[1]; ++i) {
    ch.a.at<float>(i) = a[i].get<float>();
  }
  cv::normalize(ch.a, ch.a, 1.0, 0.0, cv::NORM_L1);

  // Allocate & fill b (size: lab_bins[2])
  ch.b = cv::Mat(hist_data_.lab_bins[2], 1, CV_32F);
  for (int i = 0; i < hist_data_.lab_bins[2]; ++i) {
    ch.b.at<float>(i) = b[i].get<float>();
  }
  cv::normalize(ch.b, ch.b, 1.0, 0.0, cv::NORM_L1);

  // Store under a unique key if needed
  std::string uniqueName = color_name;
  hist_data_.color_hists[uniqueName] = ch;

  if (mShowDebugPrints_) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Loaded histograms for color \"%s\" (file %s)",
                uniqueName.c_str(), path.c_str());
  }
}

// Build a 2D H×S histogram from a query ROI (HSV) with a circular mask
cv::Mat BuoyColor::computeHS2DHist(const cv::Mat& hsv, const cv::Mat& mask) {
  std::vector<cv::Mat> hsv_ch;
  cv::split(hsv, hsv_ch);    // hsv_ch[0]=H, hsv_ch[1]=S, hsv_ch[2]=V

  cv::Mat hs_mat;
  const cv::Mat arrHS[2] = { hsv_ch[0], hsv_ch[1] };
  cv::merge(arrHS, 2, hs_mat);

  const int channels[2] = {0, 1};
  const int histSize[2] = { hist_data_.bins_2d[0], hist_data_.bins_2d[1] };
  const float h_ranges[2] = { hist_data_.ranges_2d[0][0], hist_data_.ranges_2d[0][1] };
  const float s_ranges[2] = { hist_data_.ranges_2d[1][0], hist_data_.ranges_2d[1][1] };
  const float* ranges[2]   = { h_ranges, s_ranges };

  cv::Mat hist2d;
  const cv::Mat images2D[] = { hs_mat };
  cv::calcHist(
    images2D, 1, channels,
    mask,
    hist2d,
    2, histSize, ranges,
    true, false
    );
  cv::normalize(hist2d, hist2d, 1.0, 0.0, cv::NORM_L1);
  return hist2d;
}

// Build a 1D histogram for one channel (0=H, 1=S, 2=V) with circular mask
cv::Mat BuoyColor::compute1DHist(const cv::Mat& hsv, int channel, int bins, const std::array<float,2>& range, const cv::Mat& mask) {
  std::vector<cv::Mat> hsv_ch;
  cv::split(hsv, hsv_ch);

  const cv::Mat single = hsv_ch[channel];
  const int ch[1]    = {0};
  const int histSize[1] = { bins };
  const float r[2]     = { range[0], range[1] };
  const float* ranges[1] = { r };

  cv::Mat hist1d;
  const cv::Mat images1D[] = { single };
  cv::calcHist(
    images1D, 1, ch,
    mask,
    hist1d,
    1, histSize, ranges,
    true, false
    );
  cv::normalize(hist1d, hist1d, 1.0, 0.0, cv::NORM_L1);
  return hist1d;
}

// Compute Lab channel histograms
cv::Mat BuoyColor::computeLabHist(const cv::Mat& lab, int channel, const cv::Mat& mask) {
  std::vector<cv::Mat> lab_ch;
  cv::split(lab, lab_ch);

  const cv::Mat single = lab_ch[channel];
  const int ch[1] = {0};
  const int histSize[1] = { hist_data_.lab_bins[channel] };
  const float r[2] = { hist_data_.lab_ranges[channel][0],
                       hist_data_.lab_ranges[channel][1] };
  const float* ranges[1] = { r };

  cv::Mat hist;
  const cv::Mat images[] = { single };
  cv::calcHist(
    images, 1, ch,
    mask,
    hist,
    1, histSize, ranges,
    true, false
    );
  cv::normalize(hist, hist, 1.0, 0.0, cv::NORM_L1);
  return hist;
}

// Compare two histograms by Bhattacharyya distance
double BuoyColor::compareHistogram(const cv::Mat& a, const cv::Mat& b) {
  return cv::compareHist(a, b, cv::HISTCMP_BHATTACHARYYA);
}

// Voting logic with tie-breaking by per-color weighted distance
std::string BuoyColor::classifyROI(const cv::Mat& roi_bgr) {
  // Convert ROI to HSV and Lab
  cv::Mat hsv;
  cv::cvtColor(roi_bgr, hsv, cv::COLOR_BGR2HSV);

  cv::Mat lab;
  cv::cvtColor(roi_bgr, lab, cv::COLOR_BGR2Lab);

  // Build circular mask inside ROI
  int roi_h = hsv.rows;
  int roi_w = hsv.cols;
  cv::Mat mask = cv::Mat::zeros(roi_h, roi_w, CV_8UC1);
  cv::Point center(roi_w / 2, roi_h / 2);
  int radius = std::min(center.x, center.y);
  cv::circle(mask, center, radius, cv::Scalar(255), cv::FILLED);

  // Show the circular mask
  // cv::imshow("Circle Mask", mask);
  // cv::waitKey(1);

  // Compute query histograms with the mask
  cv::Mat q_hs2d = computeHS2DHist(hsv, mask);
  cv::Mat q_h1d  = compute1DHist(hsv, 0, hist_data_.bins_1d[0], hist_data_.ranges_1d[0], mask);
  cv::Mat q_s1d  = compute1DHist(hsv, 1, hist_data_.bins_1d[1], hist_data_.ranges_1d[1], mask);
  cv::Mat q_v1d  = compute1DHist(hsv, 2, hist_data_.bins_1d[2], hist_data_.ranges_1d[2], mask);

  // Compute Lab histograms
  cv::Mat q_L = computeLabHist(lab, 0, mask);
  cv::Mat q_a = computeLabHist(lab, 1, mask);
  cv::Mat q_b = computeLabHist(lab, 2, mask);

  std::string best_color = "unknown";
  int best_votes = 0;
  double best_metric = std::numeric_limits<double>::max();
  double min_total_dist = std::numeric_limits<double>::max();
  std::string min_dist_color;

  if (mShowDebugPrints_) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---- Voting debug ----");
  }

  // First pass: Compute distances and track minimum total distance
  for (const auto& kv : hist_data_.color_hists) {
    const std::string& color_name = kv.first;
    const ColorHistograms& ref = kv.second;
    const auto& w = channel_weights_[color_name];

    double d_hs2d = compareHistogram(q_hs2d, ref.hs2d);
    double d_h1d  = compareHistogram(q_h1d,  ref.h1d);
    double d_s1d  = compareHistogram(q_s1d,  ref.s1d);
    double d_v1d  = compareHistogram(q_v1d,  ref.v1d);
    double d_L    = compareHistogram(q_L,    ref.L);
    double d_a    = compareHistogram(q_a,    ref.a);
    double d_b    = compareHistogram(q_b,    ref.b);
    double total_dist = d_hs2d * w[0] + d_h1d * w[1] + d_s1d * w[2] +
                        d_v1d * w[3] + d_L * w[4] + d_a * w[5] + d_b * w[6];

    // Track minimum total distance
    if (total_dist < min_total_dist) {
      min_total_dist = total_dist;
      min_dist_color = color_name;
    }
  }

  // Second pass: Apply voting with extra vote condition
  for (const auto& kv : hist_data_.color_hists) {
    const std::string& color_name = kv.first;
    const ColorHistograms& ref = kv.second;
    const auto& w = channel_weights_[color_name];

    double d_hs2d = compareHistogram(q_hs2d, ref.hs2d);
    double d_h1d  = compareHistogram(q_h1d,  ref.h1d);
    double d_s1d  = compareHistogram(q_s1d,  ref.s1d);
    double d_v1d  = compareHistogram(q_v1d,  ref.v1d);
    double d_L    = compareHistogram(q_L,    ref.L);
    double d_a    = compareHistogram(q_a,    ref.a);
    double d_b    = compareHistogram(q_b,    ref.b);

    int votes = 0;
    if (d_hs2d < hist_threshold_) ++votes;
    if (d_h1d  < hist_threshold_) ++votes;
    if (d_s1d  < hist_threshold_) ++votes;
    if (d_v1d  < hist_threshold_) ++votes;
    if (d_L    < hist_threshold_) ++votes;
    if (d_a    < hist_threshold_) ++votes;
    if (d_b    < hist_threshold_) ++votes;

    // EXTRA VOTE: Award to color with smallest total distance
    if (color_name == min_dist_color) {
      votes += 1;
      if (mShowDebugPrints_) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Extra vote given to " << color_name
                                                                                << " for smallest total distance: " << min_total_dist);
      }
    }

    if (mShowDebugPrints_) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Ref[" << color_name << "]: "
                                                              << "d_hs2d=" << d_hs2d << "  "
                                                              << "d_h1d="  << d_h1d  << "  "
                                                              << "d_s1d="  << d_s1d  << "  "
                                                              << "d_v1d="  << d_v1d  << "  "
                                                              << "d_L="    << d_L    << "  "
                                                              << "d_a="    << d_a    << "  "
                                                              << "d_b="    << d_b    << "  "
                                                              << "votes="  << votes  << "  "
                                                              << "total_dist=" << (d_hs2d + d_h1d + d_s1d + d_v1d + d_L + d_a + d_b));
    }

    if (votes >= min_votes_required_) {
      if (votes > best_votes) {
        best_votes  = votes;
        best_metric = d_hs2d + d_h1d + d_s1d + d_v1d + d_L + d_a + d_b;
        best_color  = color_name;
      } else if (votes == best_votes) {
        double weighted_dist =
          d_hs2d * w[0] +
          d_h1d  * w[1] +
          d_s1d  * w[2] +
          d_v1d  * w[3] +
          d_L    * w[4] +
          d_a    * w[5] +
          d_b    * w[6];

        if (mShowDebugPrints_) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Ref[" << color_name << "] weighted_dist=" << weighted_dist
                                                                  << " (using weights " << w[0] << "," << w[1] << ","
                                                                  << w[2] << "," << w[3] << "," << w[4] << ","
                                                                  << w[5] << "," << w[6] << ")");
        }

        if (weighted_dist < best_metric) {
          best_metric = weighted_dist;
          best_color  = color_name;
        }
      }
    }
  }
  if (mShowDebugPrints_) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Final chosen: " << best_color
                                                                      << " (votes=" << best_votes
                                                                      << ", metric=" << best_metric << ")");
  }

  return best_color;
}

// Use the full bounding box (no scaling)
cv::Rect BuoyColor::getScaledROI(const cv::Rect& orig) {
  return orig;
}

// For each detection: classify, draw, and publish
std::string BuoyColor::processDetection(cv::Mat& frame_bgr, const image_pipeline_msgs::msg::BoundingBox2D &bx, const bool enablePlot) {
  (void)enablePlot;
  std::string color_results;

  // Each box is 4 points in row-major order
  int w  = bx.size_x;
  int h  = bx.size_y;
  cv::Rect box(bx.center_x - bx.size_x/2.0, bx.center_y- bx.size_y/2.0, w, h);


  if (mShowDebugPrints_) {
    std::cerr << "bx.center_x = " << bx.center_x << ", bx.center_y = " << bx.center_y << std::endl;
  }

  // Clip to image bounds
  box &= cv::Rect(0, 0, frame_bgr.cols, frame_bgr.rows);
  if (box.area() < 25) return "BOH";

  cv::Rect roi_box = getScaledROI(box);
  if (roi_box.area() < 1) return "BOH";

  cv::Mat roi_bgr = frame_bgr(roi_box);
  std::string color = classifyROI(roi_bgr);

  // Draw bounding box and label
  if (true) {
    cv::rectangle(frame_bgr, box, cv::Scalar(0, 255, 0), 2);
    cv::putText(frame_bgr, color,
                box.tl() + cv::Point(0, -5),
                cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(0, 255, 0), 2);
  }
  if (mShowResults_) {
    cv::imshow("Detection Result", frame_bgr);
    cv::waitKey(1);
  }

  return color;
}

// Synchronized callback
// Synchronized callback
void BuoyColor::detectionCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
                                  const image_pipeline_msgs::msg::BoundingBox2DArray::ConstSharedPtr &boxes_msg) {

  if (mShowDebugPrints_) {
    std::cerr << "[BuoyColor::detectionCallback] Start..." << std::endl;
  }
  try {
    // Convert the ROS2 Image message to a cv::Mat
    cv::Mat frame_bgr = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    std::vector<std::string> colors;

    for (size_t i = 0; i < boxes_msg->boxes.size(); i++) {
      auto box = boxes_msg->boxes[i];
      // Process detections using the image and the constructed polygon
      auto clr = processDetection(frame_bgr, box);
      colors.emplace_back(clr);
      if (mShowDebugPrints_) {
        std::cerr << "[BuoyColor::detectionCallback] box #" << i << " --> color is " << clr << std::endl;
      }
    }

    image_pipeline_msgs::msg::Colors colorsMsg;
    colorsMsg.header = boxes_msg->header;
    colorsMsg.colors = colors;
    colors_pub_->publish(colorsMsg);


  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Processing error: %s", e.what());
    std::cerr << "[BuoyColor::detectionCallback] End with error!" << std::endl;
  }
  if (mShowDebugPrints_) {
    std::cerr << "[BuoyColor::detectionCallback] End!" << std::endl;
  }
}
}  // namespace image_pipeline
