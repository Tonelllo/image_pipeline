#pragma once
/*
 * Copyright 2025
 */

#include <opencv2/core/mat.hpp>

class SimpleEnhancer
{
public:
  enum fusionMode_ {AVG, PCA};
  explicit SimpleEnhancer(bool, fusionMode_);
  cv::Mat enhance(cv::Mat &);

private:
  bool mShowSteps_;
  fusionMode_ mFusionMode_;
  enum channels {BLUE, GREEN, RED};
  enum hsvchan {HUE, SAT, VAL};
  cv::Mat compensateRedBlue(cv::Mat &);
  cv::Mat greyWorldAlg(cv::Mat &);
  cv::Mat sharpen(cv::Mat &);
  cv::Mat equalizeVal(cv::Mat &);
  cv::Mat pcaFusion(cv::Mat &, cv::Mat &);
  cv::Mat avgFusion(cv::Mat &, cv::Mat &);
};
