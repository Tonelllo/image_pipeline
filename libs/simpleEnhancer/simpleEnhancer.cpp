/*
* Copyright 2025
*/
#include "simpleEnhancer.hpp"
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>


cv::Mat SimpleEnhancer::compensateRedBlue(cv::Mat& image){
  cv::Mat channels[3];
  cv::Mat normChs[3];
  cv::Mat compensated;
  double rMax, rMin, gMax, gMin, bMax, bMin, bMean, rMean, gMean;
  rMean = 0;
  gMean = 0;
  bMean = 0;

  cv::split(image, channels);

  cv::minMaxLoc(channels[RED], &rMin, &rMax);
  cv::minMaxLoc(channels[GREEN], &gMin, &gMax);
  cv::minMaxLoc(channels[BLUE], &bMin, &bMax);

  normChs[RED] = cv::Mat::zeros(channels[RED].size(), CV_64FC1);
  normChs[GREEN] = cv::Mat::zeros(channels[GREEN].size(), CV_64FC1);
  normChs[BLUE] = cv::Mat::zeros(channels[BLUE].size(), CV_64FC1);

  for(size_t y = 0; y < channels[RED].rows; y++){
    for(size_t x = 0; x < channels[RED].cols; x++){
      normChs[RED].at<double>(y, x) = (static_cast<double>(channels[RED].at<uint8_t>(y, x))
        - rMin) / (rMax - rMin);
      normChs[GREEN].at<double>(y, x) = (static_cast<double>(channels[GREEN].at<uint8_t>(y, x))
        - gMin) / (gMax - gMin);
      normChs[BLUE].at<double>(y, x) = (static_cast<double>(channels[BLUE].at<uint8_t>(y, x))
        - bMin) / (bMax - bMin);
    }
  }
  rMean = cv::mean(normChs[RED])[0];
  gMean = cv::mean(normChs[GREEN])[0];
  bMean = cv::mean(normChs[BLUE])[0];

  for(size_t y = 0; y < normChs[BLUE].rows; y++){
    for(size_t x = 0; x < normChs[BLUE].cols; x++){
      channels[RED].at<uint8_t>(y, x) = round((normChs[RED].at<double>(y, x) + (gMean - rMean) *
        (1-normChs[RED].at<double>(y, x)) * normChs[GREEN].at<double>(y, x)) * rMax);
      channels[BLUE].at<uint8_t>(y, x) = round((normChs[BLUE].at<double>(y, x) + (gMean - bMean) *
        (1-normChs[BLUE].at<double>(y, x)) * normChs[GREEN].at<double>(y, x)) * bMax);
    }
  }
  for(size_t y = 0; y < normChs[BLUE].rows; y++){
    for(size_t x = 0; x < normChs[BLUE].cols; x++){
      channels[GREEN].at<uint8_t>(y, x) = round(normChs[GREEN].at<double>(y, x) * gMax);
    }
  }

  cv::merge(channels, 3, compensated);

  return compensated;
}

cv::Mat SimpleEnhancer::greyWorldAlg(cv::Mat& image){
  cv::Mat channels[3];
  cv::Mat gray, out;
  double  bMean, rMean, gMean, bnMean;

  cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
  cv::split(image, channels);

  rMean = cv::mean(channels[RED])[0];
  gMean = cv::mean(channels[GREEN])[0];
  bMean = cv::mean(channels[BLUE])[0];
  bnMean = cv::mean(gray)[0];


  for(size_t y = 0; y < channels[RED].rows; y++){
    for(size_t x = 0; x < channels[RED].cols; x++){
      channels[RED].at<uint8_t>(y, x) = round(static_cast<double>(
        channels[RED].at<uint8_t>(y, x)) * bnMean / rMean);
      channels[GREEN].at<uint8_t>(y, x) = round(static_cast<double>(
        channels[GREEN].at<uint8_t>(y, x)) * bnMean / gMean);
      channels[BLUE].at<uint8_t>(y, x) = round(static_cast<double>(
        channels[BLUE].at<uint8_t>(y, x)) * bnMean / bMean);
    }
  }

  cv::merge(channels, 3, out);

  return out;
}

cv::Mat SimpleEnhancer::sharpen(cv::Mat& image){
  cv::Mat process;
  cv::GaussianBlur(image, process, cv::Size(3, 3), 0.5);
  // 1.5
  cv::addWeighted(image, 2.0, process, -0.5, 0, process);
  return process;
}

cv::Mat SimpleEnhancer::equalizeVal(cv::Mat& image){
  cv::Mat hsvImg;
  cv::Mat channels[3];
  cv::Mat ret;
  cv::cvtColor(image, hsvImg, cv::COLOR_RGB2HSV);

  cv::split(image, channels);

  cv::equalizeHist(channels[VAL], channels[VAL]);
  cv::merge(channels, 3, ret);

  cv::cvtColor(ret, ret, cv::COLOR_HSV2RGB);
  return ret;
}


cv::Mat SimpleEnhancer::enhance(cv::Mat& image){
  cv::Mat ret;
  cv::Mat process;
  /*process = compensateRedBlue(image);*/
  /*cv::imshow("compensatedRB", process);*/
  /*process = greyWorldAlg(process);*/
  /*cv::imshow("greyWorld", process);*/
  /*process = sharpen(process);*/
  /*cv::imshow("sharpen", process);*/
  cv::imshow("orig", image);
  process = equalizeVal(image);
  cv::imshow("equalized hist", process);
  cv::waitKey(0);

  return ret;
}
