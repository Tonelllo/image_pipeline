/*
* copyright 2025
*/
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <udcp/udcp.hpp>
#include <vector>

UDCP::UDCP(bool showImage, uint windowSize){
  mShowImage_ = showImage;
  mWindowSize_ = windowSize;
}

cv::Mat UDCP::getDarkChannel(cv::Mat& image){
  cv::Mat channels[3];
  cv::Mat dc, kernel, dark;
  cv::split(image, channels);
  dc = cv::min(cv::min(channels[RED], channels[GREEN]), channels[BLUE]);
  kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(mWindowSize_, mWindowSize_));
  cv::erode(dc, dark, kernel);
  return dark;
}
cv::Mat UDCP::getDarkChannel2(cv::Mat& image){
  cv::Mat channels[3];
  cv::Mat gPadded, bPadded;
  cv::Mat dark(image.size(), CV_8UC1);
  r = 4 * mWindowSize_;
  eps = 10e-6;

  cv::split(image, channels);

  cv::copyMakeBorder(
    channels[GREEN],
    gPadded,
    mWindowSize_/2,
    mWindowSize_/2,
    mWindowSize_/2,
    mWindowSize_/2,
    cv::BORDER_CONSTANT,
    cv::Scalar(255));

  cv::copyMakeBorder(
    channels[BLUE],
    bPadded,
    mWindowSize_/2,
    mWindowSize_/2,
    mWindowSize_/2,
    mWindowSize_/2,
    cv::BORDER_CONSTANT,
    cv::Scalar(255));

  uchar* darkPtr = nullptr;
  for(size_t y = 0; y < gPadded.rows - mWindowSize_; y++){
    for(size_t x = 0; x < gPadded.cols - mWindowSize_; x++){
      double gMin, bMin;

      cv::Mat gRoi(gPadded(cv::Rect(x, y, mWindowSize_, mWindowSize_)));
      cv::Mat bRoi(bPadded(cv::Rect(x, y, mWindowSize_, mWindowSize_)));
      cv::minMaxLoc(gRoi, &gMin, nullptr);
      cv::minMaxLoc(bRoi, &bMin, nullptr);

      darkPtr = dark.ptr<uchar>(y);
      darkPtr[x] = static_cast<uchar>(fmin(gMin, bMin));
    }
  }
  return dark;
}

cv::Mat UDCP::getAtmosphere(cv::Mat& orig, cv::Mat image){
  // TODO(tonello) parameterize
  uint selectPixNum = floor(image.rows * image.cols * 0.0001);
  double dMax;
  cv::Point dpMax;
  cv::Mat channels[3];
  double bMean, gMean, rMean;
  bMean = 0;
  gMean = 0;
  rMean = 0;

  cv::split(orig, channels);

  channels[BLUE].convertTo(channels[BLUE], CV_64FC1);
  channels[GREEN].convertTo(channels[GREEN], CV_64FC1);
  channels[RED].convertTo(channels[RED], CV_64FC1);

  // Maybe thresholding
  for(size_t i = 0; i < selectPixNum; i++){
    cv::minMaxLoc(image, nullptr, &dMax, nullptr, &dpMax);
    image.at<uchar>(dpMax) = 0;

    bMean += channels[BLUE].at<double>(dpMax.y, dpMax.x);
    gMean += channels[GREEN].at<double>(dpMax.y, dpMax.x);
    rMean += channels[RED].at<double>(dpMax.y, dpMax.x);
  }

  bMean /= selectPixNum;
  gMean /= selectPixNum;
  rMean /= selectPixNum;

  return cv::Mat(image.size(), CV_8UC3, cv::Scalar(bMean, gMean, rMean));
}

cv::Mat UDCP::getNeg(cv::Mat image, cv::Mat atm){
  cv::Mat res;
  cv::Mat dc;

  image.convertTo(image, CV_64FC3);
  atm.convertTo(atm, CV_64FC3);

  atm.convertTo(atm, CV_64FC3, 1.0 / 255.0);
  image.convertTo(image, CV_64FC3, 1.0 / 255.0);


  cv::divide(image, atm, res);

  res.convertTo(res, CV_8UC3, 255.0);


  dc = getDarkChannel(res);

  dc = 255 - dc;

  return dc;
}

cv::Mat UDCP::finalPass(cv::Mat image, cv::Mat atm, cv::Mat guided){
  cv::Mat guided3;
  cv::Mat tmp[3];
  cv::Mat res;

  tmp[0] = guided;
  tmp[1] = guided;
  tmp[2] = guided;

  cv::merge(tmp, 3, guided3);

  image.convertTo(image, CV_64FC3, 1.0 / 255.0);
  atm.convertTo(atm, CV_64FC3, 1.0 / 255.0);
  guided3.convertTo(guided3, CV_64FC3, 1.0 / 255.0);

  cv::divide(image - atm, guided3, res);
  res += atm;
  res.convertTo(res, CV_8UC3, 255.0);
  return res;
}

cv::Mat UDCP::enhance(cv::Mat& image){
  cv::Mat process;
  cv::Mat darkCh;
  cv::Mat darkCh2;
  cv::Mat atm;
  cv::Mat guided;
  cv::Mat bwImage;
  darkCh = getDarkChannel(image);
  atm = getAtmosphere(image, darkCh.clone());
  process = getNeg(image.clone(), atm.clone());
  image.convertTo(bwImage, CV_8UC1);
  cv::ximgproc::guidedFilter(bwImage, process, guided, r, eps);
  process = finalPass(image, atm, guided);
  return process;
}
