// Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

#ifndef INFERENCE_H
#define INFERENCE_H

// Cpp native
#include <fstream>
#include <vector>
#include <string>
#include <random>

// OpenCV / DNN / Inference
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

struct Detection
{
  int class_id{0};
  std::string className{};
  float confidence{0.0};
  cv::Scalar color{};
  cv::Rect box{};
};

class Inference
{
public:
  Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape = {640, 640}, const std::vector<std::string> &classes = {}, const bool &runWithCuda = true);
  std::vector<Detection> runInference(const cv::Mat &input);

private:
  void loadClassesFromFile();
  void loadOnnxNetwork();
  cv::Mat formatToSquare(const cv::Mat &source, int *pad_x, int *pad_y, float *scale);

  std::string modelPath{};
  std::string classesPath{};
  bool cudaEnabled{};

  std::vector<std::string> mClasses_;

  cv::Size2f modelShape{};

  float modelConfidenceThreshold {0.25};
  float modelScoreThreshold      {0.45};
  float modelNMSThreshold        {0.50};

  bool letterBoxForSquare = true;

  cv::dnn::Net net;
};

#endif // INFERENCE_H
