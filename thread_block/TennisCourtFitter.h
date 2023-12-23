//
// Created by Chlebus, Grzegorz on 28.08.17.
// Copyright (c) Chlebus, Grzegorz. All rights reserved.
//
#pragma once

#include "Line.h"
#include <opencv2/opencv.hpp>
#include "TennisCourtModel.h"

typedef struct arg_struct {
  int start;
  int end;
  int thread_id;
  cv::Mat binaryImage;
  cv::Mat rgbImage;
} arg_struct;

class TennisCourtFitter
{
public:
  struct Parameters
  {
      float weightConst;
      int finetune_iteration;
    Parameters();
  };

  TennisCourtFitter();

  TennisCourtFitter(Parameters p);

  TennisCourtModel run(const std::vector<Line>& lines, const cv::Mat& binaryImage, const cv::Mat& rgbImage, const int num_threads);

  static bool debug;
  static const std::string windowName;

private:
  void getHorizontalAndVerticalLines(const std::vector<Line>& lines, std::vector<Line>& hLines,
    std::vector<Line>& vLines, const cv::Mat& rgbImage);

  void sortHorizontalLines(std::vector<Line>& hLines, const cv::Mat& rgbImage);
  void sortHorizontalLines2(std::vector<Line>& hLines, const cv::Mat& rgbImage);

  void sortVerticalLines(std::vector<Line>& vLines, const cv::Mat& rgbImage);
  void sortVerticalLines2(std::vector<Line>& vLines, const cv::Mat& rgbImage);

  void findBestModelFit(const cv::Mat& binaryImage, const cv::Mat& rgbImage, const int num_threads);
  void localFindBest(arg_struct* args);
  //void findBestModelFit(const cv::Mat& binaryImage, const cv::Mat& rgbImage, const std::vector<Line>& lines);


  Parameters parameters;
  std::vector<LinePair> hLinePairs;
  std::vector<LinePair> vLinePairs;
  TennisCourtModel bestModel;
};

