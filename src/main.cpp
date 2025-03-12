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
  cv::Mat testImage1 = cv::imread("../media/red_buoy.png");
  cv::Mat testImage2 = cv::imread("../media/pipe.png");
  cv::Mat testImage3 = cv::imread("../media/number.png");
  cv::Mat frame;
  SimpleEnhancer se(false, SimpleEnhancer::fusionMode_::PCA);
  UDCP udcp(false, 25);
  if(!video.isOpened()){
    std::cout << "Unable to open video" << std::endl;
    exit(1);
  }

  /*writer.open("../media/udcp.mp4", codec, video.get(cv::CAP_PROP_FPS), cv::Size(1920, 1080));*/
  /*uint counter = 30;*/
  while(true){
    /*video >> frame;*/
    /*if(frame.empty()){*/
    /*  break;*/
    /*}*/
    /*cv::resize(frame, frame, cv::Size(frame.cols/2, frame.rows/2));*/
    /*cv::imshow("orig", frame);*/
    /*cv::imshow("udcp", udcp.enhance(frame));*/
    /*cv::imshow("alg", se.enhance(frame));*/
    /*cv::waitKey(1);*/
    cv::imshow("orig1", testImage1);
    cv::imshow("simple1", se.enhance(testImage1));
    cv::imshow("udcp1", udcp.enhance(testImage1));
    cv::waitKey(100);

    cv::imshow("orig2", testImage2);
    cv::imshow("simple2", se.enhance(testImage2));
    cv::imshow("udcp2", udcp.enhance(testImage2));
    cv::waitKey(100);

    cv::imshow("orig3", testImage3);
    cv::imshow("simple3", se.enhance(testImage3));
    cv::imshow("udcp3", udcp.enhance(testImage3));
    cv::waitKey(0);

    /*writer.write(udcp.enhance(frame));*/
    /*std::cout << counter++ << " / " << video.get(cv::CAP_PROP_FRAME_COUNT) << std::endl;*/
  }

  video.release();
  /*writer.release();*/
  return 0;
}
