//
// Created by Chlebus, Grzegorz on 28.08.17.
// Copyright (c) Chlebus, Grzegorz. All rights reserved.
//
#pragma once

#include "Line.h"

typedef std::pair<Line, Line> LinePair;

class TennisCourtModel
{
public:
  TennisCourtModel();

  TennisCourtModel(const TennisCourtModel& o);

  TennisCourtModel& operator=(const TennisCourtModel& o);

  float fit(const LinePair& hLinePair, const LinePair& vLinePair, const cv::Mat& binaryImage,
    const cv::Mat& rgbImage);

  static std::vector<LinePair> getPossibleLinePairs(std::vector<Line>& lines);

  

  void get3DTransformMatrix(std::vector<Line>& hlines, const cv::Mat& rgbImage);

  void drawModel(cv::Mat& image, cv::Scalar color=cv::Scalar(0, 255, 255));

  void writeToFile(const std::string& filename);

  std::vector<cv::Point2f> getCornerPoints();


  void finetune(const cv::Mat& binaryImage, const cv::Mat& rgbImage, const std::vector<Line>& Lines);
  void fixResized(int cols, int rows);
  cv::Mat masking(cv::Mat& image);

  float getBestScore() { return this->bestScore; };
  void setBestScore(float score) { this->bestScore = score; };
  
  static bool debug;
  static bool isfullCourt;

private:
  std::vector<cv::Point2f> getIntersectionPoints(const LinePair& hLinePair, const LinePair& vLinePair);

  float evaluateModel(const std::vector<cv::Point2f>& courtPoints, const cv::Mat& binaryImage);

  float computeScoreForLineSegment(const cv::Point2f& start, const cv::Point2f& end,
    const cv::Mat& binaryImage);

  bool isInsideTheImage(float x, float y, const cv::Mat& image);

  void drawModel(std::vector<cv::Point2f>& courtPoints, cv::Mat& image, cv::Scalar color=cv::Scalar(0, 255, 255));

  void findTipsPoints(std::vector<Line>& candidateLines, const cv::Mat& rgbImage, std::vector<cv::Point2f> transformedModelPoints, std::vector<cv::Point2f> polePoints);

  void createModelCourt();

  
  float bestScore;
  std::vector<Line> hLines;
  std::vector<Line> vLines;
  std::vector<LinePair> hLinePairs;
  std::vector<LinePair> vLinePairs;
  std::vector<cv::Point2f> courtPoints;
  cv::Mat transformationMatrix;


  std::vector<cv::Point2f> polePairs;
  cv::Mat rvec;
  cv::Mat tvec;
  cv::Mat pvec;
  cv::Mat transformationMatrix_3D;
  cv::Mat modelCourt;
};
