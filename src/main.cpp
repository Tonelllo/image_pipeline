/*
 * Copyright 2025 
*/
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <simpleEnhancer/simpleEnhancer.hpp>

int main () {
  cv::VideoCapture video("../media/underwater.mp4");
  cv::Mat frame;
  SimpleEnhancer se;
  if(!video.isOpened()){
    std::cout << "Unable to open video" << std::endl;
    exit(1);
  }

  while(true){
    video >> frame;
    cv::resize(frame, frame, cv::Size(frame.cols/2, frame.rows/2));
    if(frame.empty()){
      break;
    }
    se.enhance(frame);
    /*cv::imshow("res", se.enhance(frame));*/
    /*cv::waitKey(10);*/
    break;
  }
  return 0;
}
