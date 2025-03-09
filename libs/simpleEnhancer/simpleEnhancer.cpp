/*
* Copyright 2025
*/
#include "simpleEnhancer.hpp"

#include <iostream>
#include <algorithm>
#include <opencv2/core/hal/interface.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>


SimpleEnhancer::SimpleEnhancer(bool showSteps){
  mShowSteps_ = showSteps;
}
cv::Mat SimpleEnhancer::compensateRedBlue(cv::Mat& image){
  cv::Mat channels[3];
  cv::Mat normChs[3];
  cv::Mat compensated;
  double rMax, rMin, gMax, gMin, bMax, bMin, bMean, rMean, gMean;
  rMean = 0;
  gMean = 0;
  bMean = 0;

  cv::split(image, channels);

  cv::minMaxLoc(channels[BLUE], &bMin, &bMax);
  cv::minMaxLoc(channels[GREEN], &gMin, &gMax);
  cv::minMaxLoc(channels[RED], &rMin, &rMax);

  normChs[BLUE] = cv::Mat::zeros(channels[BLUE].size(), CV_64FC1);
  normChs[GREEN] = cv::Mat::zeros(channels[GREEN].size(), CV_64FC1);
  normChs[RED] = cv::Mat::zeros(channels[RED].size(), CV_64FC1);

  /*cv::normalize(normChs[BLUE], normChs[BLUE], 1, 0, cv::NORM_MINMAX);*/
  /*cv::normalize(normChs[GREEN], normChs[GREEN], 1, 0, cv::NORM_MINMAX);*/
  /*cv::normalize(normChs[RED], normChs[RED], 1, 0, cv::NORM_MINMAX);*/
  for(size_t y = 0; y < channels[RED].rows; y++){
    for(size_t x = 0; x < channels[RED].cols; x++){
      normChs[BLUE].at<double>(y, x) = (static_cast<double>(channels[BLUE].at<uint8_t>(y, x))
        - bMin) / (bMax - bMin);
      normChs[GREEN].at<double>(y, x) = (static_cast<double>(channels[GREEN].at<uint8_t>(y, x))
        - gMin) / (gMax - gMin);
      normChs[RED].at<double>(y, x) = (static_cast<double>(channels[RED].at<uint8_t>(y, x))
        - rMin) / (rMax - rMin);
    }
  }
  bMean = cv::mean(normChs[BLUE])[0];
  gMean = cv::mean(normChs[GREEN])[0];
  rMean = cv::mean(normChs[RED])[0];

  for(size_t y = 0; y < normChs[BLUE].rows; y++){
    for(size_t x = 0; x < normChs[BLUE].cols; x++){
      channels[BLUE].at<uint8_t>(y, x) = round((normChs[BLUE].at<double>(y, x) + (gMean - bMean) *
        (1-normChs[BLUE].at<double>(y, x)) * normChs[GREEN].at<double>(y, x)) * bMax);
      channels[RED].at<uint8_t>(y, x) = round((normChs[RED].at<double>(y, x) + (gMean - rMean) *
        (1-normChs[RED].at<double>(y, x)) * normChs[GREEN].at<double>(y, x)) * rMax);
    }
  }
  for(size_t y = 0; y < normChs[GREEN].rows; y++){
    for(size_t x = 0; x < normChs[GREEN].cols; x++){
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

  bMean = cv::mean(channels[BLUE])[0];
  gMean = cv::mean(channels[GREEN])[0];
  rMean = cv::mean(channels[RED])[0];
  bnMean = cv::mean(gray)[0];


  for(size_t y = 0; y < channels[RED].rows; y++){
    for(size_t x = 0; x < channels[RED].cols; x++){
      channels[BLUE].at<uint8_t>(y, x) = round(static_cast<double>(
        channels[BLUE].at<uint8_t>(y, x)) * bnMean / bMean);
      channels[GREEN].at<uint8_t>(y, x) = round(static_cast<double>(
        channels[GREEN].at<uint8_t>(y, x)) * bnMean / gMean);
      channels[RED].at<uint8_t>(y, x) = round(static_cast<double>(
        channels[RED].at<uint8_t>(y, x)) * bnMean / rMean);
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
  cv::cvtColor(image, hsvImg, cv::COLOR_BGR2HSV);

  cv::split(hsvImg, channels);

  cv::equalizeHist(channels[VAL], channels[VAL]);

  cv::merge(channels, 3, ret);

  cv::cvtColor(ret, ret, cv::COLOR_HSV2BGR);
  return ret;
}

cv::Mat SimpleEnhancer::pcaFusion(cv::Mat& image1, cv::Mat& image2){
  cv::Mat channels1[3];
  cv::Mat channels2[3];
  cv::Mat toMerge[3];
  cv::Mat fused(image1.size(), image1.type());

  cv::split(image1, channels1);
  cv::split(image2, channels2);

  cv::Mat channel1;
  cv::Mat channel2;
  cv::Mat combined;

  for (size_t i = 0; i < 3; i++) {
    channel1 = channels1[i].reshape(1, 1).clone();
    channel2 = channels2[i].reshape(1, 1).clone();

    combined = cv::Mat::zeros(2, channel1.cols, CV_64F);

    channel1.convertTo(combined.row(0), CV_64F);
    channel2.convertTo(combined.row(1), CV_64F);

    cv::normalize(combined.row(0), combined.row(0), 0, 1, cv::NORM_MINMAX, CV_64F);
    cv::normalize(combined.row(1), combined.row(1), 0, 1, cv::NORM_MINMAX, CV_64F);

    cv::PCA pca(combined, cv::Mat(), cv::PCA::DATA_AS_COL);

    cv::Mat weights = pca.eigenvectors.row(0) / cv::sum(pca.eigenvectors.row(0))[0];

    cv::Mat fusedChannels = weights.at<double>(0) * channels1[i] +
      weights.at<double>(1) * channels2[i];

    fusedChannels.convertTo(toMerge[i], CV_8U);
  }
  cv::merge(toMerge, 3, fused);

  return fused;
}

cv::Mat SimpleEnhancer::enhance(cv::Mat& image){
  cv::Mat ret;
  cv::Mat process;
  cv::Mat contrasted;
  cv::Mat sharpened;
  process = compensateRedBlue(image);
  if(mShowSteps_){
    cv::imshow("compensatedRB", process);
  }
  process = greyWorldAlg(process);
  if(mShowSteps_){
    cv::imshow("greyWorld", process);
  }
  sharpened = sharpen(process);
  if(mShowSteps_){
    cv::imshow("sharpen", sharpened);
  }
  contrasted = equalizeVal(process);
  if(mShowSteps_){
    cv::imshow("equalized hist", contrasted);
  }
  ret = pcaFusion(sharpened, contrasted);
  if(mShowSteps_){
    cv::imshow("pca fused", ret);
  }

  return ret;
}
