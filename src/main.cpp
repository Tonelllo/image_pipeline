/*
 * Copyright 2025 
*/
#include <chrono>
#include <iostream>
#include <filesystem>

#include "udcp/udcp.hpp"
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <simpleEnhancer/simpleEnhancer.hpp>

std::vector<int> history;

int main (int argc, char *argv[]) {
  const std::string keys =
    "{help h | | print this message}"
    "{path p | | [REQUIRED] Path of an image, video or directory to take as input.\n"
    "Note that if directory is specified there should be no videos}"
    "{outpath o | | This is required if you don't show the image directly with the show flag."
    "Be careful in showing too many images}"
    "{show s | | if set show images instead of saving them}"
    "{algorithm a | | [REQUIRED] Choose the algorithm to use."
    " The options are udcp, se_avg, se_pca}";
  cv::CommandLineParser parser(argc, argv, keys);
  parser.about("Enhancer tool 0.1\n"
"Example of usage: ./enhancer -a=udcp -p=./img.png -s");
  if(parser.has("help")){
    parser.printMessage();
    return 0;
  }
  if(!parser.check()){
    parser.printErrors();
    exit(1);
  }

  if(!parser.has("path")){
    std::cerr << "Path is required\n\n"<< std::endl;
    parser.printMessage();
    exit(1);
  }
  if(!parser.has("algorithm")){
    std::cerr << "Algorithm is required\n\n"<< std::endl;
    parser.printMessage();
    exit(1);
  }
  if(parser.has("show") == parser.has("outpath")){
    std::cerr << "Either show or outpath should be specified"<< std::endl;
    parser.printMessage();
    exit(1);
  }

  SimpleEnhancer sePca(false, SimpleEnhancer::fusionMode_::PCA);
  SimpleEnhancer seAvg(false, SimpleEnhancer::fusionMode_::AVG);
  UDCP udcp(false, 25, 1920, 1080);
  cv::Mat inImg;
  cv::Mat outImg;

  std::string path = parser.get<std::string>("path");
  std::vector<std::string> videoExtensions = {".webm", ".mkv", ".mp4", ".avi"};
  uint64 counter = 0;
  if(std::filesystem::is_directory(path)){
    for(const auto & entry : std::filesystem::directory_iterator(path)){
      if(std::ranges::find(videoExtensions, entry.path().extension().string())
        != videoExtensions.end()) {
        std::cerr << "No videos must be present in the input path."
              "If you want to process a video give it directly" << std::endl;
        exit(1);
      }
      if(entry.is_directory()){
        continue;
      }
      std::string algorithm = parser.get<std::string>("algorithm");
      inImg = cv::imread(entry.path());
      if(algorithm == "udcp"){
        outImg = udcp.enhance(inImg);
      }else if (algorithm == "se_avg"){
        outImg = seAvg.enhance(inImg);
      }else if (algorithm == "se_pca"){
        outImg = sePca.enhance(inImg);
      }else{
        std::cerr << "Undefined algorithm\n\n";
        parser.printMessage();
        exit(1);
      }
      if(parser.has("show")){
        cv::imshow(std::to_string(counter++), outImg);
        cv::waitKey(100);
      }else{
        std::string outPath = parser.get<std::string>("outpath");
        std::cout << "Writing image " <<
          outPath / entry.path().filename() << std::endl;
        cv::imwrite(outPath / entry.path().filename(), outImg);
      }
    }
    if(parser.has("show")){
      cv::waitKey(0);
    }
  }else if(std::filesystem::is_regular_file(path)){
    if(std::ranges::find(videoExtensions, std::filesystem::path(path).extension().string())
      != videoExtensions.end()){
      cv::VideoCapture video(path);
      cv::Mat frame;
      if(!video.isOpened()){
        std::cout << "Unable to open video" << std::endl;
        exit(1);
      }
      if(parser.has("show")){
        int count = 100;
        while(true){
          video >> frame;
          if(frame.empty()){
            break;
          }
          std::string algorithm = parser.get<std::string>("algorithm");
          if(algorithm == "udcp"){
            auto start = std::chrono::high_resolution_clock::now();
          cv::resize(frame, frame, cv::Size(frame.cols/2, frame.rows/2));
            /*cv::resize(frame, frame, cv::Size(640, 384));*/
            outImg = udcp.enhance(frame);
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            history.push_back(duration.count());
            auto avg = std::accumulate(history.begin(), history.end(), 0.0);
            std::cout << avg / history.size() << std::endl;
          }else if (algorithm == "se_avg"){
            outImg = seAvg.enhance(frame);
          }else if (algorithm == "se_pca"){
            outImg = sePca.enhance(frame);
          }else{
            std::cerr << "Undefined algorithm\n\n";
            parser.printMessage();
            exit(1);
          }
          cv::imshow("result", outImg);
          cv::waitKey(1);
        }
      }else{
        std::string outPath = parser.get<std::string>("outpath");
        cv::VideoWriter writer;
        int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        double width = video.get(cv::CAP_PROP_FRAME_WIDTH);
        double height = video.get(cv::CAP_PROP_FRAME_HEIGHT);

        writer.open(std::filesystem::path(outPath) / std::filesystem::path(path).filename(),
                    codec, video.get(cv::CAP_PROP_FPS),
                    cv::Size(width, height));

        while(true){
          video >> frame;
          if(frame.empty()){
            break;
          }
          cv::resize(frame, frame, cv::Size(frame.cols/2, frame.rows/2));
          std::string algorithm = parser.get<std::string>("algorithm");
          if(algorithm == "udcp"){
            outImg = udcp.enhance(frame);
          }else if (algorithm == "se_avg"){
            outImg = seAvg.enhance(frame);
          }else if (algorithm == "se_pca"){
            outImg = sePca.enhance(frame);
          }else{
            std::cerr << "Undefined algorithm\n\n";
            parser.printMessage();
            exit(1);
          }
          writer.write(outImg);
          std::cout << counter++ << " / " << video.get(cv::CAP_PROP_FRAME_COUNT) << std::endl;
        }
        writer.release();
      }
      video.release();
    }else{
      if(parser.has("show")){
          std::string algorithm = parser.get<std::string>("algorithm");
          inImg = cv::imread(path);
          if(algorithm == "udcp"){
            outImg = udcp.enhance(inImg);
          }else if (algorithm == "se_avg"){
            outImg = seAvg.enhance(inImg);
          }else if (algorithm == "se_pca"){
            outImg = sePca.enhance(inImg);
          }else{
            std::cerr << "Undefined algorithm\n\n";
            parser.printMessage();
            exit(1);
          }
          cv::imshow("result", outImg);
          cv::waitKey(0);
      }else{
          std::string algorithm = parser.get<std::string>("algorithm");
          std::string outPath = parser.get<std::string>("outpath");
          inImg = cv::imread(path);
          if(algorithm == "udcp"){
            outImg = udcp.enhance(inImg);
          }else if (algorithm == "se_avg"){
            outImg = seAvg.enhance(inImg);
          }else if (algorithm == "se_pca"){
            outImg = sePca.enhance(inImg);
          }else{
            std::cerr << "Undefined algorithm\n\n";
            parser.printMessage();
            exit(1);
          }
        std::cout << "Writing image " <<
          std::filesystem::path(outPath) / std::filesystem::path(path).filename() << std::endl;
        cv::imwrite(std::filesystem::path(outPath) / std::filesystem::path(path).filename(),
                    outImg);
      }
    }
  }

  return 0;
}
