/*
 * Copyright 2025
 */
#include "simpleEnhancer.hpp"

#include <opencv2/core/hal/interface.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>


SimpleEnhancer::SimpleEnhancer(bool showSteps, fusionMode_ fm){
  mShowSteps_ = showSteps;
  mFusionMode_ = fm;
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

  channels[BLUE].convertTo(normChs[BLUE], CV_64FC1, 1.0 / 255.0);
  channels[GREEN].convertTo(normChs[GREEN], CV_64FC1, 1.0 / 255.0);
  channels[RED].convertTo(normChs[RED], CV_64FC1, 1.0 / 255.0);

  bMean = cv::mean(normChs[BLUE])[0];
  gMean = cv::mean(normChs[GREEN])[0];
  rMean = cv::mean(normChs[RED])[0];

  cv::Mat tmp(normChs[GREEN].size(), normChs[GREEN].type());
  cv::multiply(cv::Scalar(1) - normChs[BLUE], normChs[GREEN], tmp);
  cv::Mat a = (normChs[BLUE] + (gMean - bMean) * tmp) * bMax;
  a.convertTo(channels[BLUE], CV_8UC1);

  cv::multiply(cv::Scalar(1) - normChs[RED], normChs[GREEN], tmp);
  cv::Mat b = (normChs[RED] + (gMean - rMean) * tmp) * rMax;
  b.convertTo(channels[RED], CV_8UC1);


  cv::Mat c = normChs[GREEN] * gMax;
  c.convertTo(channels[GREEN], CV_8UC1);

  cv::merge(channels, 3, compensated);

  return compensated;
}

cv::Mat SimpleEnhancer::greyWorldAlg(cv::Mat& image){
  cv::Mat channels[3];
  cv::Mat gray, out;
  double bMean, rMean, gMean, bnMean;

  cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
  cv::split(image, channels);

  bMean = cv::mean(channels[BLUE])[0];
  gMean = cv::mean(channels[GREEN])[0];
  rMean = cv::mean(channels[RED])[0];
  bnMean = cv::mean(gray)[0];

  cv::Mat dchannels[3];
  channels[BLUE].convertTo(dchannels[BLUE], CV_64FC1);
  channels[GREEN].convertTo(dchannels[GREEN], CV_64FC1);
  channels[RED].convertTo(dchannels[RED], CV_64FC1);

  dchannels[BLUE] *= (bnMean / bMean);
  dchannels[GREEN] *= (bnMean / gMean);
  dchannels[RED] *= (bnMean / rMean);

  dchannels[BLUE].convertTo(channels[BLUE], CV_8UC1);
  dchannels[RED].convertTo(channels[RED], CV_8UC1);
  dchannels[GREEN].convertTo(channels[GREEN], CV_8UC1);

  cv::merge(channels, 3, out);

  return out;
}

cv::Mat SimpleEnhancer::sharpen(cv::Mat& image){
  cv::Mat process;
  cv::GaussianBlur(image, process, cv::Size(3, 3), 0.5);
  // 1.5
  cv::addWeighted(image, 1.5, process, -0.5, 0, process);
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

cv::Mat SimpleEnhancer::avgFusion(cv::Mat& image1, cv::Mat& image2){
  cv::Mat channels1[3];
  cv::Mat channels2[3];
  cv::Mat avgs[3];
  cv::Mat ret;

  cv::split(image1, channels1);
  cv::split(image2, channels2);

  cv::Mat acc(image1.size(), CV_64FC1, cv::Scalar(0));

  cv::accumulate(channels1[BLUE], acc);
  cv::accumulate(channels2[BLUE], acc);
  acc.convertTo(avgs[BLUE], CV_8UC1, 1.0/2);

  acc = cv::Mat(image1.size(), CV_64FC1, cv::Scalar(0));

  cv::accumulate(channels1[GREEN], acc);
  cv::accumulate(channels2[GREEN], acc);
  acc.convertTo(avgs[GREEN], CV_8UC1, 1.0/2);

  acc = cv::Mat(image1.size(), CV_64FC1, cv::Scalar(0));

  cv::accumulate(channels1[RED], acc);
  cv::accumulate(channels2[RED], acc);
  acc.convertTo(avgs[RED], CV_8UC1, 1.0/2);

  cv::merge(avgs, 3, ret);

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

    channel1.convertTo(combined.row(0), CV_64F, 1.0 / 255.0);
    channel2.convertTo(combined.row(1), CV_64F, 1.0 / 255.0);

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
  if(mShowSteps_) {
    cv::imshow("compensatedRB", process);
  }
  process = greyWorldAlg(process);
  if(mShowSteps_) {
    cv::imshow("greyWorld", process);
  }
  sharpened = sharpen(process);
  if(mShowSteps_) {
    cv::imshow("sharpen", sharpened);
  }
  contrasted = equalizeVal(process);
  if(mShowSteps_) {
    cv::imshow("equalized hist", contrasted);
  }
  if(mFusionMode_ == fusionMode_::PCA) {
    ret = pcaFusion(sharpened, contrasted);
    if(mShowSteps_) {
      cv::imshow("pca fused", ret);
    }
  }else if(mFusionMode_ == fusionMode_::AVG) {
    ret = avgFusion(sharpened, contrasted);
    if(mShowSteps_) {
      cv::imshow("avg fused", ret);
    }
  }else{
    throw "Unsupported fusion method";
  }

  return ret;
}
