/*
 * Copyright 2025 
*/
#include <iostream>

#include "udcp/udcp.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <simpleEnhancer/simpleEnhancer.hpp>

int main () {
  cv::VideoCapture video("../media/underwater.mp4");
  /*cv::VideoWriter writer;*/
  /*int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');*/
  cv::Mat frame;
  SimpleEnhancer se(false, SimpleEnhancer::fusionMode_::AVG);
  UDCP udcp(false, 25);
  if(!video.isOpened()){
    std::cout << "Unable to open video" << std::endl;
    exit(1);
  }

  /*writer.open("../media/avg.mp4", codec, video.get(cv::CAP_PROP_FPS), cv::Size(1920, 1080));*/
  /*uint counter = 5;*/
  while(true){
    video >> frame;
    if(frame.empty()){
      break;
    }
    cv::resize(frame, frame, cv::Size(frame.cols/2, frame.rows/2));
    /*cv::imshow("orig", frame);*/
    cv::imshow("udcp", udcp.enhance(frame));
    /*cv::imshow("alg", se.enhance(frame));*/
    cv::waitKey(1);

    /*writer.write(se.enhance(frame));*/
    /*std::cout << counter++ << " / " << video.get(cv::CAP_PROP_FRAME_COUNT) << std::endl;*/
  }

  video.release();
  /*writer.release();*/
  return 0;
}
