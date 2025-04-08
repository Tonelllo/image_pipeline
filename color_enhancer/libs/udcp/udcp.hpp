#pragma once
/*
 * copyright 2025
 */
#include <vector>

#include <opencv2/core.hpp>

class UDCP
{
public:
  explicit UDCP(bool, uint, uint64_t, uint64_t);
  cv::Mat enhance(cv::Mat &);

private:
  cv::Mat getDarkChannel(cv::Mat &);
  cv::Mat getAtmosphere(cv::Mat &, cv::Mat &);
  cv::Mat transmissionEstimate(cv::Mat &, cv::Mat &);
  cv::Mat finalPass(cv::Mat &, cv::Mat &, cv::Mat &);
  enum channels {BLUE, GREEN, RED};
  cv::Mat mStructuringKernel_;
  std::vector<cv::Point> mImageSortBuf_;
  bool mShowImage_;
  uint mWindowSize_;
  double r;
  double eps;
};
