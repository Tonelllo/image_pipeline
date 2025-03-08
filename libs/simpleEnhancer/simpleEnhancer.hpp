#pragma once
/*
* Copyright 2025
*/

#include <opencv2/core/mat.hpp>
class SimpleEnhancer{
public:
  cv::Mat enhance(cv::Mat&);
private:
  enum channels{BLUE, GREEN, RED};
  enum hsvchan{HUE, SAT, VAL};
  cv::Mat compensateRedBlue(cv::Mat&);
  cv::Mat greyWorldAlg(cv::Mat&);
  cv::Mat sharpen(cv::Mat&);
  cv::Mat equalizeVal(cv::Mat&);
};
