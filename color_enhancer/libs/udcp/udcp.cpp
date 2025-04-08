/*
 * copyright 2025
 */
#include <opencv2/core/hal/interface.h>

#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <udcp/udcp.hpp>

UDCP::UDCP(bool showImage, uint windowSize, uint64_t width, uint64_t height)
{
  mShowImage_ = showImage;
  mWindowSize_ = windowSize;
  r = 60;
  eps = 0.0001;
  mImageSortBuf_.reserve(width * height);
  mStructuringKernel_ =
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(mWindowSize_, mWindowSize_));
}

cv::Mat UDCP::getDarkChannel(cv::Mat & image)
{
  cv::Mat channels[3];
  cv::Mat dc, kernel, dark;
  cv::split(image, channels);
  dc = cv::min(cv::min(channels[RED], channels[GREEN]), channels[BLUE]);
  cv::erode(dc, dark, mStructuringKernel_);
  return dark;
}

cv::Mat UDCP::getAtmosphere(cv::Mat & orig, cv::Mat & image)
{
  // TODO(tonello) parameterize
  uint selectPixNum = floor(image.rows * image.cols * 0.001);
  double dMax;
  cv::Point dpMax;
  cv::Mat channels[3];
  double bMean, gMean, rMean;
  bMean = 0;
  gMean = 0;
  rMean = 0;

  cv::split(orig, channels);

  for (int i = 0; i < image.rows; ++i) {
      for (int j = 0; j < image.cols; ++j) {
          mImageSortBuf_.push_back(cv::Point(j, i));
      }
  }

  std::partial_sort(mImageSortBuf_.begin(),
                    mImageSortBuf_.begin() + selectPixNum,
                    mImageSortBuf_.end(),
            [&image](const cv::Point& p1, const cv::Point& p2) {
                return image.at<float>(p1) > image.at<float>(p2);
            });

  // Maybe thresholding
  for (size_t i = 0; i < selectPixNum; i++) {
    cv::Point dpMax = mImageSortBuf_[i];
    bMean += channels[BLUE].at<float>(dpMax);
    gMean += channels[GREEN].at<float>(dpMax);
    rMean += channels[RED].at<float>(dpMax);
  }
  mImageSortBuf_.clear();

  bMean /= selectPixNum;
  gMean /= selectPixNum;
  rMean /= selectPixNum;

  return cv::Mat(image.size(), CV_32FC3, cv::Scalar(bMean, gMean, rMean));
}

cv::Mat UDCP::transmissionEstimate(cv::Mat & image, cv::Mat & atm)
{
  cv::Mat res;
  cv::Mat dc;

  cv::divide(image, atm, res);

  dc = getDarkChannel(res);

  dc = 1 - 0.95 * dc;

  return dc;
}

cv::Mat UDCP::finalPass(cv::Mat & image, cv::Mat & atm, cv::Mat & guided)
{
  cv::Mat guided3;
  cv::Mat res;

  cv::merge(std::array<cv::Mat, 3>{guided, guided, guided}, guided3);

  cv::add(atm, (image - atm) / guided3, res);
  return res;
}

cv::Mat UDCP::enhance(cv::Mat & image)
{
  cv::Mat process;
  cv::Mat darkCh;
  cv::Mat atm;
  cv::Mat guided;
  cv::Mat bwImage;
  cv::Mat normImage;
  image.convertTo(normImage, CV_32FC3, 1.0 / 255.0);

  darkCh = getDarkChannel(normImage);
  atm = getAtmosphere(normImage, darkCh);
  process = transmissionEstimate(normImage, atm);
  cv::cvtColor(image, bwImage, CV_BGR2GRAY);
  cv::ximgproc::guidedFilter(bwImage, process, guided, r, eps);
  process = finalPass(normImage, atm, guided);
  return process;
}
