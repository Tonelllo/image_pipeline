/*
 * copyright 2025
 */
#include <opencv2/core/hal/interface.h>

#include <opencv2/core.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <udcp/udcp.hpp>

UDCP::UDCP(bool showImage, uint windowSize){
  mShowImage_ = showImage;
  mWindowSize_ = windowSize;
  r = 60;
  eps = 0.0001;
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

cv::Mat UDCP::getAtmosphere(cv::Mat& orig, cv::Mat& image){
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

  // Maybe thresholding
  for(size_t i = 0; i < selectPixNum; i++) {
    cv::minMaxLoc(image, nullptr, &dMax, nullptr, &dpMax);
    image.at<uchar>(dpMax) = 0;

    bMean += channels[BLUE].at<double>(dpMax.y, dpMax.x);
    gMean += channels[GREEN].at<double>(dpMax.y, dpMax.x);
    rMean += channels[RED].at<double>(dpMax.y, dpMax.x);
  }

  bMean /= selectPixNum;
  gMean /= selectPixNum;
  rMean /= selectPixNum;

  return cv::Mat(image.size(), CV_64FC3, cv::Scalar(bMean, gMean, rMean));
}

cv::Mat UDCP::transmissionEstimate(cv::Mat& image, cv::Mat& atm){
  cv::Mat res;
  cv::Mat dc;

  cv::divide(image, atm, res);

  dc = getDarkChannel(res);

  dc = 1 - dc;

  return dc;
}

cv::Mat UDCP::finalPass(cv::Mat &image, cv::Mat& atm, cv::Mat& guided){
  cv::Mat guided3;
  cv::Mat tmp[3];
  cv::Mat res;

  tmp[0] = guided;
  tmp[1] = guided;
  tmp[2] = guided;

  cv::merge(tmp, 3, guided3);

  guided3.convertTo(guided3, CV_64FC3);

  cv::divide(image - atm, guided3, res);
  res += atm;
  res.convertTo(res, CV_8UC3, 255.0);
  return res;
}

cv::Mat UDCP::enhance(cv::Mat& image){
  cv::Mat process;
  cv::Mat darkCh;
  cv::Mat atm;
  cv::Mat guided;
  cv::Mat bwImage;
  cv::Mat normImage;
  image.convertTo(normImage, CV_64FC3, 1.0 / 255.0);

  darkCh = getDarkChannel(normImage);
  atm = getAtmosphere(normImage, darkCh);
  process = transmissionEstimate(normImage, atm);
  image.convertTo(bwImage, CV_32F);
  process.convertTo(process, CV_32F);
  cv::ximgproc::guidedFilter(bwImage, process, guided, r, eps);
  process = finalPass(normImage, atm, guided);
  return process;
}
