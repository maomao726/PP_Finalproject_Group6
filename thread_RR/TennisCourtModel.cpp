//
// Created by Chlebus, Grzegorz on 28.08.17.
// Copyright (c) Chlebus, Grzegorz. All rights reserved.
//

#include "TennisCourtModel.h"
#include "GlobalParameters.h"
#include "DebugHelpers.h"
#include "geometry.h"
#include "TimeMeasurement.h"
#include "opencv2/core/types_c.h"

using namespace cv;

const char* windowName = "TennisCourtModel";
bool TennisCourtModel::debug = false;
bool TennisCourtModel::isfullCourt = true;

TennisCourtModel::TennisCourtModel()
{
  Point2f hVector(1, 0);
  const Line upperBaseLine = Line(Point2f(0, 0), hVector);
  const Line upperLongLine = Line(Point2f(0, 760), hVector);
  const Line upperServiceLine = Line(Point2f(0, 4680), hVector);
  const Line netLine = Line(Point2f(0, 6700), hVector);
  const Line lowerServiceLine = Line(Point2f(0, 8720), hVector);
  const Line lowerLongLine = Line(Point2f(0, 12640), hVector);
  const Line lowerBaseLine = Line(Point2f(0, 13400), hVector);
  if (isfullCourt)
  {

      hLines = {
        upperBaseLine, upperLongLine, upperServiceLine, lowerServiceLine, lowerLongLine, lowerBaseLine
      };
  }
  else
  {

      hLines = {
        upperBaseLine, upperLongLine, upperServiceLine, netLine
      };
  }
  
  // Add more lines


  //hLines = {
  //  hLines[0], hLines[2], netLine, hLines[3], hLines[5]
  //};
  
   //for full court
   

  //for half court
  /*hLines = {
    hLines[0], hLines[1], hLines[2], netLine
  };*/

  Point2f vVector(0, 1);
  const Line leftSideLine = Line(Point2f(0, 0), vVector);
  const Line leftSinglesLine = Line(Point2f(460, 0), vVector);
  const Line centreServiceLine = Line(Point2f(3050, 0), vVector);
  const Line rightSinglesLine = Line(Point2f(5640, 0), vVector);
  const Line rightSideLine = Line(Point2f(6100, 0), vVector);
  vLines = {
    leftSideLine, leftSinglesLine, centreServiceLine, rightSinglesLine, rightSideLine
  };

  

  hLinePairs.push_back(std::make_pair(hLines[0], hLines[1]));
  hLinePairs.push_back(std::make_pair(hLines[1], hLines[2]));
  hLinePairs.push_back(std::make_pair(hLines[2], hLines[3]));
  if (isfullCourt)
  {
      hLinePairs.push_back(std::make_pair(hLines[3], hLines[4]));
      hLinePairs.push_back(std::make_pair(hLines[4], hLines[5]));
  }
  


  
  vLinePairs.push_back(std::make_pair(vLines[0], vLines[1]));
  vLinePairs.push_back(std::make_pair(vLines[1], vLines[2]));
  vLinePairs.push_back(std::make_pair(vLines[2], vLines[3]));
  vLinePairs.push_back(std::make_pair(vLines[1], vLines[3]));
  vLinePairs.push_back(std::make_pair(vLines[3], vLines[4]));



  Point2f point;
  if (isfullCourt)
  {
      if (hLines[0].computeIntersectionPoint(vLines[0], point))
      {
          courtPoints.push_back(point); // P0
      }
      if (hLines[5].computeIntersectionPoint(vLines[0], point))
      {
          courtPoints.push_back(point); // P1
      }
      if (hLines[5].computeIntersectionPoint(vLines[4], point))
      {
          courtPoints.push_back(point); // P2
      }
      if (hLines[0].computeIntersectionPoint(vLines[4], point))
      {
          courtPoints.push_back(point);  // P3
      }
      if (hLines[0].computeIntersectionPoint(vLines[1], point))
      {
          courtPoints.push_back(point);  // P4
      }
      if (hLines[5].computeIntersectionPoint(vLines[1], point))
      {
          courtPoints.push_back(point);  // P5
      }
      if (hLines[5].computeIntersectionPoint(vLines[3], point))
      {
          courtPoints.push_back(point);  // P6
      }
      if (hLines[0].computeIntersectionPoint(vLines[3], point))
      {
          courtPoints.push_back(point);  // P7
      }
      if (vLines[0].computeIntersectionPoint(hLines[1], point))
      {
          courtPoints.push_back(point);  // P8
      }
      if (vLines[4].computeIntersectionPoint(hLines[1], point))
      {
          courtPoints.push_back(point);  // P9
      }
      if (vLines[0].computeIntersectionPoint(hLines[4], point))
      {
          courtPoints.push_back(point);  // P10
      }
      if (vLines[4].computeIntersectionPoint(hLines[4], point))
      {
          courtPoints.push_back(point);  // P11
      }
      if (hLines[2].computeIntersectionPoint(vLines[0], point))
      {
          courtPoints.push_back(point);  // P12
      }
      if (hLines[2].computeIntersectionPoint(vLines[4], point))
      {
          courtPoints.push_back(point);  // P13
      }
      if (hLines[3].computeIntersectionPoint(vLines[0], point))
      {
          courtPoints.push_back(point);  // P14
      }
      if (hLines[3].computeIntersectionPoint(vLines[4], point))
      {
          courtPoints.push_back(point);  // P15
      }
      if (hLines[5].computeIntersectionPoint(vLines[2], point))
      {
          courtPoints.push_back(point);  // P16
      }
      if (hLines[3].computeIntersectionPoint(vLines[2], point))
      {
          courtPoints.push_back(point);  // P17
      }
      if (hLines[0].computeIntersectionPoint(vLines[2], point))
      {
          courtPoints.push_back(point);  // P18
      }
      if (hLines[2].computeIntersectionPoint(vLines[2], point))
      {
          courtPoints.push_back(point);  // P19
      }
      assert(courtPoints.size() == 20);
  }
  else
  {
      // for half court
      if (hLines[0].computeIntersectionPoint(vLines[0], point))
      {
          courtPoints.push_back(point); // P0
      }
      if (hLines[3].computeIntersectionPoint(vLines[0], point))
      {
          courtPoints.push_back(point); // P1
      }
      if (hLines[3].computeIntersectionPoint(vLines[4], point))
      {
          courtPoints.push_back(point); // P2
      }
      if (hLines[0].computeIntersectionPoint(vLines[4], point))
      {
          courtPoints.push_back(point);  // P3
      }
      if (hLines[0].computeIntersectionPoint(vLines[1], point))
      {
          courtPoints.push_back(point);  // P4
      }
      if (hLines[3].computeIntersectionPoint(vLines[1], point))
      {
          courtPoints.push_back(point);  // P5
      }
      if (hLines[3].computeIntersectionPoint(vLines[3], point))
      {
          courtPoints.push_back(point);  // P6
      }
      if (hLines[0].computeIntersectionPoint(vLines[3], point))
      {
          courtPoints.push_back(point);  // P7
      }
      if (vLines[0].computeIntersectionPoint(hLines[1], point))
      {
          courtPoints.push_back(point);  // P8
      }
      if (vLines[4].computeIntersectionPoint(hLines[1], point))
      {
          courtPoints.push_back(point);  // P9
      }
      if (hLines[2].computeIntersectionPoint(vLines[0], point))
      {
          courtPoints.push_back(point);  // P10
      }
      if (hLines[2].computeIntersectionPoint(vLines[4], point))
      {
          courtPoints.push_back(point);  // P11
      }
      if (hLines[0].computeIntersectionPoint(vLines[2], point))
      {
          courtPoints.push_back(point);  // P12
      }
      if (hLines[3].computeIntersectionPoint(vLines[2], point))
      {
          courtPoints.push_back(point);  // P13
      }
      assert(courtPoints.size() == 13);
  }
  this->bestScore = GlobalParameters().initialFitScore;
}

TennisCourtModel::TennisCourtModel(const TennisCourtModel& o)
  : transformationMatrix(o.transformationMatrix)
{
  courtPoints = o.courtPoints;
  hLinePairs = o.hLinePairs;
  vLinePairs = o.vLinePairs;
  hLines = o.hLines;
  vLines = o.vLines;
  bestScore = o.bestScore;
}

TennisCourtModel& TennisCourtModel::operator=(const TennisCourtModel& o)
{
  transformationMatrix = o.transformationMatrix;
  bestScore = o.bestScore;
  return *this;
}

float TennisCourtModel::fit(const LinePair& hLinePair, const LinePair& vLinePair,
  const cv::Mat& binaryImage, const cv::Mat& rgbImage)
{
  
 bestScore = GlobalParameters().initialFitScore;
  std::vector<Point2f> points = getIntersectionPoints(hLinePair, vLinePair);
  //TODO Check whether the intersection points make sense
  if (points.size() >= 4)
  {
      for (auto& modelHLinePair : hLinePairs)
      {
          for (auto& modelVLinePair : vLinePairs)
          {
              std::vector<Point2f> modelPoints = getIntersectionPoints(modelHLinePair, modelVLinePair);
              Mat matrix = getPerspectiveTransform(modelPoints, points);
              //std::vector<Point2f> transformedModelPoints(16);
              std::vector<Point2f> transformedModelPoints;
              if(isfullCourt)transformedModelPoints.resize(20);
              else transformedModelPoints.resize(13);
              perspectiveTransform(courtPoints, transformedModelPoints, matrix);
              float score = evaluateModel(transformedModelPoints, binaryImage);
              if (score > bestScore)
              {
                  bestScore = score;
                  transformationMatrix = matrix;
              }
              /*if (debug)
              {
                  std::vector<Point2f> debug1;
                  using namespace std;
                  perspectiveTransform(modelPoints, debug1, matrix);
                  cout << "Origin on image: " << endl;
                  cout << "[" << points[0] << " " << points[1] << " " << points[2] << " " << points[3] << "]" << endl;
                  cout << "=========================" << endl << "Origin on model : " << endl;
                  cout << "[" << modelPoints[0] << " " << modelPoints[1] << " " << modelPoints[2] << " " << modelPoints[3] << "]" << endl;
                  cout << "=========================" << endl << "model to image : " << endl;
                  cout << "[" << debug1[0] << " " << debug1[1] << " " << debug1[2] << " " << debug1[3] << "]" << endl;
                  cout << "=========================" << endl<<endl;
                  cv::waitKey(1);
                  
              }*/

          }
      }
  
  }
  return bestScore;
}


std::vector<cv::Point2f> TennisCourtModel::getIntersectionPoints(const LinePair& hLinePair,
  const LinePair& vLinePair)
{
  std::vector<Point2f> v;
  Point2f point;

  if (hLinePair.first.computeIntersectionPoint(vLinePair.first, point))
  {
    v.push_back(point);
  }
  if (hLinePair.first.computeIntersectionPoint(vLinePair.second, point))
  {
    v.push_back(point);
  }
  if (hLinePair.second.computeIntersectionPoint(vLinePair.first, point))
  {
    v.push_back(point);
  }
  if (hLinePair.second.computeIntersectionPoint(vLinePair.second, point))
  {
    v.push_back(point);
  }

  //assert(v.size() == 4);

  return v;
}

std::vector<LinePair> TennisCourtModel::getPossibleLinePairs(std::vector<Line>& lines)
{
  std::vector<LinePair> linePairs;
  for (size_t first = 0; first < lines.size(); ++first)
  {
    for (size_t second = first + 1; second < lines.size(); ++second)
    {
      linePairs.push_back(std::make_pair(lines[first], lines[second]));
    }
  }
  return linePairs;
}


void TennisCourtModel::drawModel(cv::Mat& image, Scalar color)
{
    std::vector<Point2f> transformedModelPoints;
  //std::vector<Point2f> transformedModelPoints(16);
  if(isfullCourt)transformedModelPoints.resize(20);
  else transformedModelPoints.resize(13);
  perspectiveTransform(courtPoints, transformedModelPoints, transformationMatrix);
  drawModel(transformedModelPoints, image, color);
}

void TennisCourtModel::drawModel(std::vector<Point2f>& courtPoints, Mat& image, Scalar color)
{
/*
  drawLine(courtPoints[0], courtPoints[1], image, color);
  drawLine(courtPoints[1], courtPoints[2], image, color);
  drawLine(courtPoints[2], courtPoints[3], image, color);
  drawLine(courtPoints[3], courtPoints[0], image, color);

  drawLine(courtPoints[4], courtPoints[5], image, color);
  drawLine(courtPoints[6], courtPoints[7], image, color);

  drawLine(courtPoints[8], courtPoints[9], image, color);
  drawLine(courtPoints[10], courtPoints[11], image, color);

  drawLine(courtPoints[12], courtPoints[13], image, color);
  drawLine(courtPoints[14], courtPoints[15], image, color);
  */
drawLine(courtPoints[0], courtPoints[1], image, color);
drawLine(courtPoints[1], courtPoints[2], image, color);
drawLine(courtPoints[2], courtPoints[3], image, color);
drawLine(courtPoints[3], courtPoints[0], image, color);
drawLine(courtPoints[4], courtPoints[5], image, color);
drawLine(courtPoints[6], courtPoints[7], image, color);
drawLine(courtPoints[8], courtPoints[9], image, color);
drawLine(courtPoints[10], courtPoints[11], image, color);
drawLine(courtPoints[12], courtPoints[13], image, color);
if (isfullCourt)
{
    drawLine(courtPoints[14], courtPoints[15], image, color);
    drawLine(courtPoints[16], courtPoints[17], image, color);
    drawLine(courtPoints[18], courtPoints[19], image, color);
}


}


float TennisCourtModel::evaluateModel(const std::vector<cv::Point2f>& courtPoints, const cv::Mat& binaryImage)
{
  float score = 0;

  // TODO: heuritic to see whether the model makes sense
  float d1 = distance(courtPoints[0], courtPoints[1]);
  float d2 = distance(courtPoints[1], courtPoints[2]);
  float d3 = distance(courtPoints[2], courtPoints[3]);
  float d4 = distance(courtPoints[3], courtPoints[0]);
  float t = 30;
  if (d1 < t || d2 < t || d3 < t || d4 < t)
  {
    return GlobalParameters().initialFitScore;
  }

  score += computeScoreForLineSegment(courtPoints[0], courtPoints[1], binaryImage);
  score += computeScoreForLineSegment(courtPoints[1], courtPoints[2], binaryImage);
  score += computeScoreForLineSegment(courtPoints[2], courtPoints[3], binaryImage);
  score += computeScoreForLineSegment(courtPoints[3], courtPoints[0], binaryImage);
  score += computeScoreForLineSegment(courtPoints[4], courtPoints[5], binaryImage);
  score += computeScoreForLineSegment(courtPoints[6], courtPoints[7], binaryImage);
  score += computeScoreForLineSegment(courtPoints[8], courtPoints[9], binaryImage);
  score += computeScoreForLineSegment(courtPoints[10], courtPoints[11], binaryImage);
  score += computeScoreForLineSegment(courtPoints[12], courtPoints[13], binaryImage);
  if (isfullCourt)
  {
      score += computeScoreForLineSegment(courtPoints[14], courtPoints[15], binaryImage);
      score += computeScoreForLineSegment(courtPoints[16], courtPoints[17], binaryImage);
      score += computeScoreForLineSegment(courtPoints[18], courtPoints[19], binaryImage);
  }
  

//  std::cout << "Score = " << score << std::endl;

  return score;
}



float TennisCourtModel::computeScoreForLineSegment(const cv::Point2f& start, const cv::Point2f& end,
  const cv::Mat& binaryImage)
{
  float score = 0;
  float fgScore = 1;
  float bgScore = -0.5;
  int length = round(distance(start, end));

  Point2f vec = normalize(end-start);

  for (int i = 0; i < length; i=i+1)
  {
    Point2f p = start + i*vec;
    int x = round(p.x);
    int y = round(p.y);
    if (isInsideTheImage(x, y, binaryImage))
    {
        uchar imageValue = binaryImage.at<uchar>(y, x);
        if (imageValue == GlobalParameters().fgValue)
        {
            score += fgScore;
        }
        else
        {
            score += bgScore;
        }
    }
    else score -= 0.05;
  }
  return score;
}


bool TennisCourtModel::isInsideTheImage(float x, float y, const cv::Mat& image)
{
  return (x >= 0 && x < image.cols) && (y >= 0 && y < image.rows);
}

void TennisCourtModel::writeToFile(const std::string& filename)
{
  //std::vector<Point2f> transformedModelPoints(16);
    std::vector<Point2f> Corners(courtPoints.begin(), courtPoints.begin() + 4);
    std::vector<Point2f> transformedModelPoints(4);
    
    perspectiveTransform(Corners, transformedModelPoints, transformationMatrix);

  std::ofstream outFile(filename);
  if (!outFile.is_open())
  {
    throw std::runtime_error("Unable to open file: " + filename);
  }
  for (auto& point: transformedModelPoints)
  {
    outFile << point.x << ";" << point.y << std::endl;
  }
}

void TennisCourtModel::get3DTransformMatrix(std::vector<Line>& hlines, const cv::Mat& rgbImage)
{

    std::vector<Point2f> polePoints;
    polePoints.push_back((courtPoints[0] + courtPoints[1]) / 2);
    polePoints.push_back((courtPoints[2] + courtPoints[3]) / 2);
    
    std::vector<Point2f> transformedModelPoints(20);
    perspectiveTransform(courtPoints, transformedModelPoints, transformationMatrix);

    findTipsPoints(hlines, rgbImage, transformedModelPoints, polePoints);

    std::vector<Point3f> objPoints;
    for (int i = 0; i < 4; i++)
    {
        objPoints.push_back(Point3f(courtPoints[i].x, courtPoints[i].y, 0));
    }
    objPoints.push_back(Point3f(polePoints[0].x, polePoints[0].y, -1.55));
    objPoints.push_back(Point3f(polePoints[1].x, polePoints[1].y, -1.55));
    transformedModelPoints.erase(transformedModelPoints.begin() + 4, transformedModelPoints.end());
    transformedModelPoints.push_back(polePairs[0]);
    transformedModelPoints.push_back(polePairs[1]);
    /*
    Mat camMatrix = Mat::eye(3, 3, CV_32F);
    Mat distCoeff = Mat::zeros(4, 1, CV_32F);

    solvePnP(objPoints, transformedModelPoints, camMatrix, distCoeff, rvec, tvec);

    //Rodrigues(rvec, rvec);
    //hconcat(rvec, tvec, transformationMatrix_3D);
    */
    Mat equation = Mat::zeros(12, 11, CV_32F);
    Mat equation_inv = Mat(11, 12, CV_32F);
    Mat eqResult = Mat::zeros(12, 1, CV_32F);
    transformationMatrix_3D = Mat(3, 4, CV_32F);

    for (int i = 0, j = 0; i < 6; i++, j += 2)
    {
        equation.at<float>(Point(0, j)) = objPoints[i].x;
        equation.at<float>(Point(1, j)) = objPoints[i].y;
        equation.at<float>(Point(2, j)) = objPoints[i].z;
        equation.at<float>(Point(3, j)) = 1;
        equation.at<float>(Point(8, j)) = -objPoints[i].x * transformedModelPoints[i].x;
        equation.at<float>(Point(9, j)) = -objPoints[i].y * transformedModelPoints[i].x;
        equation.at<float>(Point(10, j)) = -objPoints[i].z * transformedModelPoints[i].x;
        equation.at<float>(Point(4, j+1)) = objPoints[i].x;
        equation.at<float>(Point(5, j+1)) = objPoints[i].y;
        equation.at<float>(Point(6, j+1)) = objPoints[i].z;
        equation.at<float>(Point(7, j+1)) = 1;
        equation.at<float>(Point(8, j+1)) = -objPoints[i].x * transformedModelPoints[i].y;
        equation.at<float>(Point(9, j+1)) = -objPoints[i].y * transformedModelPoints[i].y;
        equation.at<float>(Point(10, j+1)) = -objPoints[i].z * transformedModelPoints[i].y;
        eqResult.at<float>(Point(0, j)) = transformedModelPoints[i].x;
        eqResult.at<float>(Point(0, j+1)) = transformedModelPoints[i].y;
    }
    std::cout<<equation<<std::endl;
    std::cout << eqResult << std::endl;

    invert(equation, equation_inv, DECOMP_SVD);
    std::cout << equation_inv << std::endl;
    Mat transVector = equation_inv * eqResult;

    for (int i = 0; i < 11; i++)
    {
        transformationMatrix_3D.at<float>(Point(i % 4, i / 4)) = transVector.at<float>(Point(0, i));
    }
    transformationMatrix_3D.at<float>(Point(3, 2)) = 1;
    
    decomposeProjectionMatrix(transformationMatrix_3D, pvec, rvec, tvec);
    
    
    if (debug)
    {
        std::vector<Point2f> transformedModelPoints(20);
        perspectiveTransform(courtPoints, transformedModelPoints, transformationMatrix);
        std::vector<Point3f> objPoints;

        for (int i = 0; i < 20; i++)
        {
            objPoints.push_back(Point3f(courtPoints[i].x, courtPoints[i].y, 0));
        }
        objPoints.push_back(Point3f(polePoints[0].x, polePoints[0].y, -1.55));
        objPoints.push_back(Point3f(polePoints[1].x, polePoints[1].y, -1.55));
        
        transformedModelPoints.push_back(polePairs[0]);
        transformedModelPoints.push_back(polePairs[1]);

        Mat test(4, objPoints.size(), CV_32F);
        for (int i = 0; i < objPoints.size(); i++)
        {
            test.at<float>(0, i) = objPoints[i].x;
            test.at<float>(1, i) = objPoints[i].y;
            test.at<float>(2, i) = objPoints[i].z;
            test.at<float>(3, i) = 1;
        }
        Mat test_trans;
        transpose(test, test_trans);
        
        std::cout << "Transformation Matrix (3D): " << std::endl << transformationMatrix_3D << std::endl;
        std::cout << "===============================================" << std::endl << std::endl;
        std::cout << "Court points: " << std::endl << test_trans << std::endl;
        std::cout << "===============================================" << std::endl << std::endl;
        std::cout << "ground truth: " << std::endl << transformedModelPoints << std::endl;
        std::cout << "===============================================" << std::endl << std::endl;
        std::cout << "pvec: " << std::endl << pvec << std::endl;
        std::cout << "===============================================" << std::endl << std::endl;
        std::cout << "rvec: " << std::endl << rvec << std::endl;
        std::cout << "===============================================" << std::endl << std::endl;


        tvec.at<float>(0, 0) = tvec.at<float>(0, 0) / tvec.at<float>(0, 3);
        tvec.at<float>(0, 1) = tvec.at<float>(0, 1) / tvec.at<float>(0, 3);
        tvec.at<float>(0, 2) = tvec.at<float>(0, 2) / tvec.at<float>(0, 3);
        tvec.at<float>(0, 3) = 1;
        std::cout << "tvec: " << std::endl << tvec << std::endl;
        std::cout << "===============================================" << std::endl << std::endl;
        //projectPoints(objPoints, rvec, tvec, camMatrix, distCoeff, test);
        Mat result = transformationMatrix_3D * test;
        for (int i = 0; i < objPoints.size(); i++)
        {
            result.at<float>(0, i) = result.at<float>(0, i) / result.at<float>(2, i);
            result.at<float>(1, i) = result.at<float>(1, i) / result.at<float>(2, i);
            result.at<float>(2, i) = 1;
        }
        transpose(result, result);
        std::cout << "Projection Result: " << std::endl << result << std::endl;
        
    }
    
}

void TennisCourtModel::findTipsPoints(std::vector<Line>& candidateLines, const cv::Mat& rgbImage, std::vector<Point2f> transformedModelPoints, std::vector<Point2f> polePoints)
{

    std::vector<Line> transformedCourtLines;

    transformedCourtLines.push_back(Line::fromTwoPoints(transformedModelPoints[0], transformedModelPoints[3]));
    transformedCourtLines.push_back(Line::fromTwoPoints(transformedModelPoints[1], transformedModelPoints[2]));
    transformedCourtLines.push_back(Line::fromTwoPoints(transformedModelPoints[4], transformedModelPoints[5]));
    transformedCourtLines.push_back(Line::fromTwoPoints(transformedModelPoints[6], transformedModelPoints[7]));
    transformedCourtLines.push_back(Line::fromTwoPoints(transformedModelPoints[8], transformedModelPoints[9]));
    transformedCourtLines.push_back(Line::fromTwoPoints(transformedModelPoints[10], transformedModelPoints[11]));

    // remove duplicate lines with court lines
    for (auto& transformedCourtLine : transformedCourtLines)
    {
        for (auto candidateline = candidateLines.begin(); candidateline != candidateLines.end(); candidateline++)
        {
            if (transformedCourtLine.isDuplicate(*candidateline))
            {
                candidateLines.erase(candidateline);
                candidateline--;
                continue;
            }
        }
    }
    if (debug)
    {
        Mat img = rgbImage.clone();
        std::cout << "Find Candidate Net-Line : " << candidateLines.size() << std::endl;
        drawLines(candidateLines, img, Scalar(255, 0, 0));
        displayImage(windowName, img);
    }
    // Constraints of search area
    float distanceThreshold = transformedCourtLines[0].getDistance(transformedModelPoints[2]) / 2;
    for (auto candidateline = candidateLines.begin(); candidateline != candidateLines.end(); candidateline++)
    {
        if ((*candidateline).getDistance(transformedModelPoints[3]) > distanceThreshold)
        {
            candidateLines.erase(candidateline);
            candidateline--;
            continue;
        }
    }
    if (debug)
    {
        Mat img = rgbImage.clone();
        std::cout << "Find Candidate Net-Line after search-area constaints: " << candidateLines.size() << std::endl;
        drawLines(candidateLines, img, Scalar(255, 0, 0));
        displayImage(windowName, img);
    }

    //Constraints of slope
    Point2f baseSlope = transformedCourtLines[0].getVector();
    float minslope = candidateLines[0].getVector().dot(baseSlope);
    for (auto candidateline = candidateLines.begin() + 1; candidateline != candidateLines.end(); candidateline++)
    {
        if ((*candidateline).getVector().dot(baseSlope) > minslope)
        {
            minslope = (*candidateline).getVector().dot(baseSlope);
            std::swap(*candidateline, candidateline[0]);

        }
        candidateLines.erase(candidateline);
        candidateline--;
        continue;
    }
    if (debug)
    {
        Mat img = rgbImage.clone();
        std::cout << "Find Candidate Net-Line after slope constaints: " << candidateLines.size() << std::endl;
        drawLines(candidateLines, img, Scalar(255, 0, 0));
        displayImage(windowName, img);
    }
    // find projected pole points 
    perspectiveTransform(polePoints, polePoints, transformationMatrix);
    polePairs.push_back(candidateLines[0].getPointOnLineClosestTo(polePoints[0]));
    polePairs.push_back(candidateLines[0].getPointOnLineClosestTo(polePoints[1]));
    std::cout << polePairs[0] << "\\" << polePairs[1] << std::endl;

    if (debug)
    {
        Mat img = rgbImage.clone();
        drawLine(polePairs[0], polePairs[1], img, Scalar(0, 255, 255));
        std::cout << "Find final net-line " << std::endl;
        displayImage(windowName, img);
    }
}

std::vector<Point2f> TennisCourtModel::getCornerPoints()
{
    std::vector<Point2f> Corners(courtPoints.begin(), courtPoints.begin() + 4);
    std::vector<Point2f> result(4);
    perspectiveTransform(Corners, result, transformationMatrix);
    
    std::cout<<"Court Coord:";
    for (auto c : Corners)
    {
        std::cout << c << " ";
    }
    std::cout << std::endl << "Image Coord:";
    for (auto r : result)
    {
        std::cout << r << " ";
    }

    return result;
}

void TennisCourtModel::createModelCourt()
{
    cv::Scalar color = cv::Scalar(255);
    modelCourt = cv::Mat::zeros(1340, 610, CV_8UC1);
    drawLine(courtPoints[0], courtPoints[1], modelCourt, color, 15);
    drawLine(courtPoints[1], courtPoints[2], modelCourt, color, 15);
    drawLine(courtPoints[2], courtPoints[3], modelCourt, color, 15);
    drawLine(courtPoints[3], courtPoints[0], modelCourt, color, 15);

    drawLine(courtPoints[4], courtPoints[5], modelCourt, color, 15);
    drawLine(courtPoints[6], courtPoints[7], modelCourt, color, 15);

    drawLine(courtPoints[8], courtPoints[9], modelCourt, color, 15);
    drawLine(courtPoints[10], courtPoints[11], modelCourt, color, 15);

    //drawLine((courtPoints[4] + courtPoints[7]) / 2, (courtPoints[5] + courtPoints[6]) / 2, image, color);

    drawLine(courtPoints[12], courtPoints[13], modelCourt, color, 15);
    drawLine(courtPoints[14], courtPoints[15], modelCourt, color, 15);
    drawLine(courtPoints[16], courtPoints[17], modelCourt, color, 15);
    drawLine(courtPoints[18], courtPoints[19], modelCourt, color, 15);
}

void TennisCourtModel::finetune(const Mat& binaryImage, const Mat& rgbImage, const std::vector<Line>& Lines)
{
    std::vector<Point2f> transformPoints;
    
    std::vector<Line> transformCourtLine;
    int vLine_count, hLine_count;

    if (isfullCourt)
    {
        transformPoints.resize(20);
        // vLine
        transformCourtLine.push_back(Line(transformPoints[0], transformPoints[0] - transformPoints[1]));
        transformCourtLine.push_back(Line(transformPoints[4], transformPoints[4] - transformPoints[5]));
        transformCourtLine.push_back(Line(transformPoints[16], transformPoints[17] - transformPoints[16]));
        transformCourtLine.push_back(Line(transformPoints[6], transformPoints[7] - transformPoints[6]));
        transformCourtLine.push_back(Line(transformPoints[2], transformPoints[3] - transformPoints[2]));
        //hLine
        transformCourtLine.push_back(Line(transformPoints[3], transformPoints[0] - transformPoints[3]));
        transformCourtLine.push_back(Line(transformPoints[8], transformPoints[8] - transformPoints[9]));
        transformCourtLine.push_back(Line(transformPoints[12], transformPoints[12] - transformPoints[13]));
        transformCourtLine.push_back(Line(transformPoints[14], transformPoints[14] - transformPoints[15]));
        transformCourtLine.push_back(Line(transformPoints[10], transformPoints[10] - transformPoints[11]));
        transformCourtLine.push_back(Line(transformPoints[1], transformPoints[1] - transformPoints[2]));
        
        vLine_count = 5;
        hLine_count = 6;
    }
    else
    {
        transformPoints.resize(13);
        // vLine
        transformCourtLine.push_back(Line(transformPoints[0], transformPoints[0] - transformPoints[1]));
        transformCourtLine.push_back(Line(transformPoints[4], transformPoints[4] - transformPoints[5]));
        transformCourtLine.push_back(Line(transformPoints[12], transformPoints[12] - transformPoints[13]));
        transformCourtLine.push_back(Line(transformPoints[7], transformPoints[7] - transformPoints[6]));
        transformCourtLine.push_back(Line(transformPoints[3], transformPoints[3] - transformPoints[2]));
        //hLine
        transformCourtLine.push_back(Line(transformPoints[0], transformPoints[0] - transformPoints[3]));
        transformCourtLine.push_back(Line(transformPoints[8], transformPoints[8] - transformPoints[9]));
        transformCourtLine.push_back(Line(transformPoints[10], transformPoints[10] - transformPoints[11]));
        transformCourtLine.push_back(Line(transformPoints[1], transformPoints[1] - transformPoints[2]));

        vLine_count = 5;
        hLine_count = 4;
    }
    
    
    perspectiveTransform(courtPoints, transformPoints, transformationMatrix);
    std::vector<Line> new_transformCourtLine(transformCourtLine);
    std::vector<Point2f> new_transformPoints(transformPoints);

    
    std::vector<std::vector<Line>> candidates(transformCourtLine.size());
    float hard_dist = 60;
    double hard_dot_threshold = 0.99;
    for (int i = 0; i < vLine_count; i++)
    {
        Point2f n1, n2, pren;
        float c1, c2, prec;
        float soft_dist = 200;
        double soft_dot_threshold = 0.9;
        transformCourtLine[i].toImplicit(n1, c1);

        for (auto line : Lines)
        {
            
            
            line.toImplicit(n2, c2);
            float dot = fabs(n1.dot(n2));
            float d = fabs(fabs(c1 + 500 * (n1.x + n1.y)) - fabs(c2 + 500 * (n2.x + n2.y)));
            /*if (i == 3)
            {
                Mat image = rgbImage.clone();
                std::cout << line.getVector() << std::endl;
                std::cout << d << "/" << dot << std::endl;
                std::cout << "===============" << std::endl;
                drawLine(line, image);
                drawLine(new_transformCourtLine[0], image, Scalar(255, 255, 0));
                drawLine(new_transformCourtLine[2], image, Scalar(255, 0, 255));
                drawLine(new_transformCourtLine[i - 1], image, Scalar(255, 0, 0));
                displayImage("Finetune: ", image);
            }*/
            if (d < soft_dist && dot > soft_dot_threshold)
            {
                //std::cout << "A" << std::endl;
                if (i != 0)
                {
                    float b1 = transformCourtLine[i - 1].getVector().x;
                    float b2 = line.getVector().x;
                    //std::cout << new_transformCourtLine[i - 1].getVector()<<" : " << line.getVector() << std::endl;

                    transformCourtLine[i - 1].toImplicit(pren, prec);
                    float dot_hard = fabs(n2.dot(pren));
                    float d_hard = fabs(fabs(c2 + 500 * (n2.x + n2.y)) - fabs(prec + 500 * (pren.x + pren.y)));
                    //std::cout << dot_hard << " " << d_hard << std::endl;
                    if (b1 >= b2)continue;
                    else if (dot_hard > hard_dot_threshold&& d_hard < hard_dist)continue;
                    else
                    {
                        
                        /*std::vector<Point2f> court;
                        std::vector<Point2f> imageP(4);
                        court.push_back(courtPoints[0]);
                        court.push_back(courtPoints[1]);
                        new_transformCourtLine[0].computeIntersectionPoint(new_transformCourtLine[5], imageP[0]);
                        new_transformCourtLine[0].computeIntersectionPoint(new_transformCourtLine[10], imageP[1]);
                        line.computeIntersectionPoint(new_transformCourtLine[5], imageP[2]);
                        line.computeIntersectionPoint(new_transformCourtLine[10], imageP[3]);
                        switch (i)
                        {
                        case 1:
                            court.push_back(courtPoints[4]);
                            court.push_back(courtPoints[5]);
                            break;
                        case 2:
                            court.push_back(courtPoints[18]);
                            court.push_back(courtPoints[16]);
                            break;
                        case 3:
                            court.push_back(courtPoints[7]);
                            court.push_back(courtPoints[6]);
                            break;
                        case4:
                            court.push_back(courtPoints[3]);
                            court.push_back(courtPoints[2]);
                            break;
                        }
                        try
                        {
                            Mat finetune_mat = getPerspectiveTransform(court, imageP);

                            perspectiveTransform(courtPoints, new_transformPoints, finetune_mat);

                            float score = evaluateModel(new_transformPoints, binaryImage);
                            if (debug)
                            {
                                std::cout << "Score: " << score << std::endl;
                                Mat image = rgbImage.clone();
                                
                                drawModel(new_transformPoints, image, cv::Scalar(128));
                                displayImage("Finetune: ", image);

                            }
                            if (score > bestScore)
                            {
                                std::cout << "Finetune" << std::endl;
                                transformationMatrix = finetune_mat;
                                bestScore = score;
                            }
                        }
                        catch(Exception)
                        {
                            continue;
                        }*/
                    }
                }
                /*std::cout << i << "!" << std::endl;
                soft_dot_threshold = dot;
                new_transformCourtLine[i] = line;*/
                candidates[i].push_back(line);
            }
            
        }
        //std::cout << "line" << i << ", final d: " << soft_dist << std::endl;
    }
    for (int i = vLine_count; i < hLine_count + vLine_count; i++)
    {
        Point2f n1, n2, pren;
        float c1, c2, prec;
        float soft_dist = 200;
        double soft_dot_threshold = 0.9;
        transformCourtLine[i].toImplicit(n1, c1);

        for (auto line : Lines)
        {


            line.toImplicit(n2, c2);
            float dot = fabs(n1.dot(n2));
            float d = fabs(fabs(c1 + 500 * (n1.x + n1.y)) - fabs(c2 + 500 * (n2.x + n2.y)));
            /*if (i == 3)
            {
                Mat image = rgbImage.clone();
                std::cout << line.getVector() << std::endl;
                std::cout << d << "/" << dot << std::endl;
                std::cout << "===============" << std::endl;
                drawLine(line, image);
                drawLine(new_transformCourtLine[0], image, Scalar(255, 255, 0));
                drawLine(new_transformCourtLine[2], image, Scalar(255, 0, 255));
                drawLine(new_transformCourtLine[i - 1], image, Scalar(255, 0, 0));
                displayImage("Finetune: ", image);
            }*/
            if (d < soft_dist && dot > soft_dot_threshold)
            {
                //std::cout << "A" << std::endl;
                if (i != 0)
                {
                    float b1 = fabs(transformCourtLine[i - 1].getVector().y);
                    float b2 = fabs(line.getVector().y);
                    //std::cout << transformCourtLine[i - 1].getVector() << " : " << line.getVector() << std::endl;

                    transformCourtLine[i - 1].toImplicit(pren, prec);
                    float dot_hard = fabs(n2.dot(pren));
                    float d_hard = fabs(fabs(c2 + 500 * (n2.x + n2.y)) - fabs(prec + 500 * (pren.x + pren.y)));
                    //std::cout << dot_hard << " " << d_hard << std::endl;
                    if (b1 >= b2)continue;
                    else if (dot_hard > hard_dot_threshold && d_hard < hard_dist)continue;
                    else
                    {

                        /*std::vector<Point2f> court;
                        std::vector<Point2f> imageP(4);
                        court.push_back(courtPoints[0]);
                        court.push_back(courtPoints[1]);
                        new_transformCourtLine[0].computeIntersectionPoint(new_transformCourtLine[5], imageP[0]);
                        new_transformCourtLine[0].computeIntersectionPoint(new_transformCourtLine[10], imageP[1]);
                        line.computeIntersectionPoint(new_transformCourtLine[5], imageP[2]);
                        line.computeIntersectionPoint(new_transformCourtLine[10], imageP[3]);
                        switch (i)
                        {
                        case 1:
                            court.push_back(courtPoints[4]);
                            court.push_back(courtPoints[5]);
                            break;
                        case 2:
                            court.push_back(courtPoints[18]);
                            court.push_back(courtPoints[16]);
                            break;
                        case 3:
                            court.push_back(courtPoints[7]);
                            court.push_back(courtPoints[6]);
                            break;
                        case4:
                            court.push_back(courtPoints[3]);
                            court.push_back(courtPoints[2]);
                            break;
                        }
                        try
                        {
                            Mat finetune_mat = getPerspectiveTransform(court, imageP);

                            perspectiveTransform(courtPoints, new_transformPoints, finetune_mat);

                            float score = evaluateModel(new_transformPoints, binaryImage);
                            if (debug)
                            {
                                std::cout << "Score: " << score << std::endl;
                                Mat image = rgbImage.clone();

                                drawModel(new_transformPoints, image, cv::Scalar(128));
                                displayImage("Finetune: ", image);

                            }
                            if (score > bestScore)
                            {
                                std::cout << "Finetune" << std::endl;
                                transformationMatrix = finetune_mat;
                                bestScore = score;
                            }
                        }
                        catch(Exception)
                        {
                            continue;
                        }*/
                    }
                }
                /*std::cout << i << "!" << std::endl;
                soft_dot_threshold = dot;
                new_transformCourtLine[i] = line;*/
                candidates[i].push_back(line);
            }

        }
        //std::cout << "line" << i << ", final d: " << soft_dist << std::endl;
    }
    for (int i = 0; i < vLine_count; i++)
    {
        for (int h = 0; h < candidates[i].size(); h++)
        {
            for (int j = vLine_count; j < hLine_count + vLine_count; j++)
            {
                int v1 = 0, v2, h1 , h2 = hLine_count + vLine_count - 1;
                if (i == 0)
                {
                    v2 = vLine_count-1;
                }
                else v2 = i;
                if (j == hLine_count + vLine_count - 1)
                {
                    h1 = hLine_count + vLine_count - 2;
                }
                else h1 = j;
                std::vector<Point2f> court(4);
                vLines[v1].computeIntersectionPoint(hLines[h1-5], court[0]);
                vLines[v1].computeIntersectionPoint(hLines[h2-5], court[1]);
                vLines[v2].computeIntersectionPoint(hLines[h2-5], court[2]);
                vLines[v2].computeIntersectionPoint(hLines[h1-5], court[3]);
                for (int k = 0; k < candidates[j].size(); k++)
                {
                    std::vector<Point2f> imgP(4);
                    if (i == 0 && j == hLine_count + vLine_count - 1)
                    {
                        //std::cout << v1 << " " << v2 << " " << h1 << " " << h2 << std::endl;
                        //std::cout << "------------" << std::endl << h << " " << k << std::endl;
                        candidates[v1][h].computeIntersectionPoint(transformCourtLine[h1], imgP[0]);
                        candidates[v1][h].computeIntersectionPoint(candidates[h2][k], imgP[1]);
                        transformCourtLine[v2].computeIntersectionPoint(candidates[h2][k], imgP[2]);
                        transformCourtLine[v2].computeIntersectionPoint(transformCourtLine[h1], imgP[3]);
                    }
                    else if (i == 0)
                    {
                        //std::cout << v1 << " " << v2 << " " << h1 << " " << h2 << std::endl;
                        //std::cout << "------------" << std::endl << h << " " << k << std::endl;
                        std::cout<<candidates[v1][h].computeIntersectionPoint(candidates[h1][k], imgP[0]);
                        std::cout << candidates[v1][h].computeIntersectionPoint(transformCourtLine[h2], imgP[1]);
                        std::cout << transformCourtLine[v2].computeIntersectionPoint(transformCourtLine[h2], imgP[2]);
                        std::cout << transformCourtLine[v2].computeIntersectionPoint(candidates[h1][k], imgP[3]);
                    }
                    else if (j == hLine_count + vLine_count - 1)
                    {
 /*                       std::cout << v1 << " " << v2 << " " << h1 << " " << h2 << std::endl;
                        std::cout << "------------" << std::endl << h << " " << k << std::endl;*/
                        transformCourtLine[v1].computeIntersectionPoint(transformCourtLine[h1], imgP[0]);
                        transformCourtLine[v1].computeIntersectionPoint(candidates[h2][k], imgP[1]);
                        candidates[v2][h].computeIntersectionPoint(candidates[h2][k], imgP[2]);
                        candidates[v2][h].computeIntersectionPoint(transformCourtLine[h1], imgP[3]);
                    }
                    else
                    {
                        //std::cout << v1 << " " << v2 << " " << h1 << " " << h2 << std::endl;
                        //std::cout << "------------" << std::endl << h << " " << k << std::endl;
                        transformCourtLine[v1].computeIntersectionPoint(candidates[h1][k], imgP[0]);
                        transformCourtLine[v1].computeIntersectionPoint(transformCourtLine[h2], imgP[1]);
                        candidates[v2][h].computeIntersectionPoint(transformCourtLine[h2], imgP[2]);
                        candidates[v2][h].computeIntersectionPoint(candidates[h1][k], imgP[3]);
                    }
                    try
                    {
                        Mat finetune_mat = getPerspectiveTransform(court, imgP);

                        perspectiveTransform(courtPoints, new_transformPoints, finetune_mat);

                        float score = evaluateModel(new_transformPoints, binaryImage);
                        
                        if (score > bestScore)
                        {
                            //std::cout << "Finetune" << std::endl;
                            transformationMatrix = finetune_mat;
                            bestScore = score;
                            if (debug)
                            {
                                std::cout << "Score: " << score << std::endl;
                                Mat image = rgbImage.clone();

                                drawModel(new_transformPoints, image, cv::Scalar(128));
                                displayImage("Finetune: ", image, 1000);

                            }
                        }
                    }
                    catch (Exception)
                    {
                        continue;
                    }
                }
                
                
            }
        }
        
    }

    if(debug)
    {
    
        for (int i = 0; i < new_transformPoints.size(); i++)
        {
            std::cerr << transformPoints[i] << new_transformPoints[i] << std::endl;
        }
    }


    
    //transformationMatrix = getPerspectiveTransform(courtPoints, new_transformPoints);
}

void TennisCourtModel::fixResized(int cols, int rows)
{
    std::vector<Point2f> Corners(courtPoints.begin(), courtPoints.begin() + 4);
    std::vector<Point2f> result(4);
    perspectiveTransform(Corners, result, transformationMatrix);
    if (debug)
    {
        for (int i = 0; i < result.size(); i++)
        {
            std::cout << result[i].x << " " << result[i].y << std::endl;
        }
    }
    for (int i = 0; i < result.size(); i++)
    {
        result[i].x = (int)(result[i].x / GlobalParameters().resize_size * cols);
        result[i].y = (int)(result[i].y / GlobalParameters().resize_size * rows);
    }
    if (debug)
    {
        for (int i = 0; i < result.size(); i++)
        {
            std::cout << result[i].x <<" "<< result[i].y << std::endl;
        }
    }
    this->transformationMatrix = getPerspectiveTransform(Corners, result);
}

Mat TennisCourtModel::masking(Mat& image)
{
    Mat mask(image.rows, image.cols, CV_8UC1, Scalar(0));
    Mat res;
    std::vector<Mat> channels(3);
    std::vector<Point2f> cornerP(courtPoints.begin(), courtPoints.begin() + 4);
    std::vector<Point2f> trans(4);
    perspectiveTransform(cornerP, trans, this->transformationMatrix);
    std::vector<Point> trans_int(trans.begin(), trans.end());
    fillPoly(mask, std::vector<std::vector<Point>>{trans_int}, Scalar(255));

    

    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask, mask, MORPH_ELLIPSE, element);
    bitwise_not(mask, mask);
    displayImage("A", mask);
    //bitwise_and(image, mask, res);
    image.copyTo(res, mask);

    if (debug)
    {
        displayImage("result", res);
    }
    return res;
}
