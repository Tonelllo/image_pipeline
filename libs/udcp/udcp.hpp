#pragma once
/*
* copyright 2025
*/
#include <opencv2/core.hpp>

class UDCP {
public:
  explicit UDCP(bool, uint);
  cv::Mat enhance(cv::Mat&);
private:
  cv::Mat getDarkChannel(cv::Mat&);
  cv::Mat getDarkChannel2(cv::Mat&);
  cv::Mat getAtmosphere(cv::Mat&, cv::Mat);
  cv::Mat getNeg(cv::Mat, cv::Mat);
  cv::Mat finalPass(cv::Mat, cv::Mat, cv::Mat);
  enum channels{BLUE, GREEN, RED};
  bool mShowImage_;
  uint mWindowSize_;
  double r;
  double eps;
};
