//
// Created by Chlebus, Grzegorz on 28.08.17.
// Copyright (c) Chlebus, Grzegorz. All rights reserved.
//
#include "TennisCourtFitter.h"
#include "GlobalParameters.h"
#include "TimeMeasurement.h"
#include "DebugHelpers.h"
#include "geometry.h"
#include <algorithm>
#include <thread>
#include <mutex>

struct edge
{
    float weight;
    int n1;
    int n2;
    edge(float w, int a, int b):weight(w), n1(a), n2(b){}
};
bool sortwithEdgeWeight(edge a, edge b)
{
    return a.weight > b.weight;
}
std::mutex mutex;
std::mutex output_mutex;

using namespace cv;

bool TennisCourtFitter::debug = false;
const std::string TennisCourtFitter::windowName = "TennisCourtFitter";

TennisCourtFitter::Parameters::Parameters()
{
    weightConst = 0.01;
    finetune_iteration = 10;
}

TennisCourtFitter::TennisCourtFitter()
  : TennisCourtFitter(Parameters())
{

}


TennisCourtFitter::TennisCourtFitter(TennisCourtFitter::Parameters p)
  : parameters(p)
{

}

TennisCourtModel TennisCourtFitter::run(const std::vector<Line>& lines, const Mat& binaryImage,
  const Mat& rgbImage, const int num_threads)
{
  TimeMeasurement::start("TennisCourtFitter::run");

  std::vector<Line> hLines, vLines;

  getHorizontalAndVerticalLines(lines, hLines, vLines, rgbImage);
  sortHorizontalLines(hLines, rgbImage);
  sortVerticalLines(vLines, rgbImage);

  if (debug)
  {
      Mat image = rgbImage.clone();
      if (false)            //show the sorted lines
      {
          for (int i = 0; i < hLines.size(); i++)
          {
              drawLine(hLines[i], image, Scalar(255, 0, 0));
              displayImage(windowName, image);
              cv::waitKey(0);
          }

          for (int i = 0; i < vLines.size(); i++)
          {
              drawLine(vLines[i], image, Scalar(0, 0, 255));
              displayImage(windowName, image);
              cv::waitKey(0);
          }
      }
      
  }

  hLinePairs = TennisCourtModel::getPossibleLinePairs(hLines);
  vLinePairs = TennisCourtModel::getPossibleLinePairs(vLines);

  if (debug)
  {
    std::cout << "Horizontal line pairs = " << hLinePairs.size() << std::endl;
    std::cout << "Vertical line pairs = " << vLinePairs.size() << std::endl;
  }

  // TODO
  if (hLinePairs.size() < 1 || vLinePairs.size() < 1)
  {
    throw std::runtime_error("Not enough line candidates were found.");
  }


  //TimeMeasurement::start("\tfindBestModelFit");
  findBestModelFit(binaryImage, rgbImage, num_threads);
  std::cerr <<"h lines finished. SWAP!"<<std::endl;
  hLines.swap(vLines);
  sortHorizontalLines2(hLines, rgbImage);
  sortVerticalLines2(vLines, rgbImage);
  hLinePairs = TennisCourtModel::getPossibleLinePairs(hLines);
  vLinePairs = TennisCourtModel::getPossibleLinePairs(vLines);
  findBestModelFit(binaryImage, rgbImage, num_threads);
  //TimeMeasurement::stop("\tfindBestModelFit");


    // for (int it = 0; it <= parameters.finetune_iteration; it++)
    // {
    //     std::cerr << "finetune..." << it << std::endl;
    //     bestModel.finetune(binaryImage, rgbImage, lines);
    // }

  
  //get 3D transform matrix
  //bestModel.get3DTransformMatrix(hLines, rgbImage);
    std::cout << "Best Score: " << bestModel.getBestScore() << std::endl;
  TimeMeasurement::stop("TennisCourtFitter::run");
  return bestModel;
}


void TennisCourtFitter::getHorizontalAndVerticalLines(const std::vector<Line>& lines,
  std::vector<Line>& hLines, std::vector<Line>& vLines, const cv::Mat& rgbImage)
{
    /*** Use maximum weight bipartite subgraph to seperate v / h lines ***/
    // Initialize adjacency matrix
    
    // �إ�adjacency matrix //
    using namespace std;
    int nodeCount = lines.size();
    vector<vector<float>> adjMat(nodeCount);
    for (int i = 0; i < nodeCount; i++)
    {
        adjMat[i].resize(nodeCount);
    }
    float maxWeight = 0; // Used for first step of bipartite
    int max_idxA, max_idxB;
    for (int idxA = 0; idxA < nodeCount; idxA++)
    {
        Point2f vectA = normalize(lines[idxA].getVector());
        for (int idxB = idxA+1; idxB < nodeCount; idxB++)
        {
            Point2f vectB = normalize(lines[idxB].getVector());
            float angle = acos(vectA.dot(vectB));
            if (fabs(acos(vectA.dot(-vectB))) < fabs(angle))
            {
                angle = acos(vectA.dot(-vectB));
            }
            float weight = powf(1 / (fabs(angle - CV_PI / 2) + parameters.weightConst), 2);
            if (isnan(weight))weight = 0;
            adjMat[idxA][idxB] = adjMat[idxB][idxA] = weight;
            if (maxWeight < weight)                                 // ������u���}�A��weight�̤j�� //
            {
                maxWeight = weight;
                max_idxA = idxA;
                max_idxB = idxB;
            }
        }
    }

    // visited�����U�u�����s���G�A��l�� //
    vector<int> visited(nodeCount);
    vector<edge> checkingEdge;
    for (int i = 0; i < visited.size(); i++)visited[i] = 0;

    // ����u��̱��񫫪��̥��������u //
    Point2f vecA = lines[max_idxA].getVector();
    Point2f vecB = lines[max_idxB].getVector();
    if (fabs(vecA.y / vecA.x) > fabs(vecB.y / vecB.x))
    {
        visited[max_idxA] = 1;
        visited[max_idxB] = -1;
    }
    else
    {
        visited[max_idxA] = -1;
        visited[max_idxB] = 1;
    }
    
    // �w���s���u�q�ƶq //
    int checked = 2;

    // Maximum weighted spanning tree�j�M�A�çQ�ά۾F�u�����s�M�v���M�w���s���G //
    adjMat[max_idxA][max_idxB] = adjMat[max_idxB][max_idxA] = INT_MIN;
    for (int i = 0; i < nodeCount; i++)
    {
        checkingEdge.push_back(edge(adjMat[max_idxA][i], max_idxA, i));
        checkingEdge.push_back(edge(adjMat[max_idxB][i], max_idxB, i));
    }
    while (checked < nodeCount)
    {
        sort(checkingEdge.begin(), checkingEdge.end(), sortwithEdgeWeight);
        while (checkingEdge.size() != 0)
        {
            if (visited[checkingEdge[0].n1] != 0 || visited[checkingEdge[0].n2] != 0) break;
            checkingEdge.erase(checkingEdge.begin());
        }
        if (visited[checkingEdge[0].n2] != 0)swap(checkingEdge[0].n1, checkingEdge[0].n2);


        double clusterWeight = 0;
        for (int i = 0; i < nodeCount; i++)
        {
            clusterWeight += adjMat[checkingEdge[0].n2][i] * visited[i];
        }
        if (clusterWeight > 0)visited[checkingEdge[0].n2] = -1;
        else visited[checkingEdge[0].n2] = 1;

        checkingEdge.erase(checkingEdge.begin());
        checked++;
    }


    // ���s�����A1���������u�A2���������u //
    for (int i = 0; i < nodeCount; i++)
    {
        if (visited[i] == 1)vLines.push_back(lines[i]);
        else if (visited[i] == -1)hLines.push_back(lines[i]);
        else
        {
            cout << i << " " << visited[i] << endl;
        }
    }
    
  /*
  for (auto& line: lines)
  {
    if (line.isVertical())
    {
      vLines.push_back(line);
    }
    else 
    {
      hLines.push_back(line);
    }
    
  }
  */
  

  if (debug)
  {
    std::cout << "Horizontal lines = " << hLines.size() << std::endl;
    std::cout << "Vertical lines = " << vLines.size() << std::endl;
    Mat image = rgbImage.clone();
    
    drawLines(hLines, image, Scalar(255, 0, 0));
    drawLines(vLines, image, Scalar(0, 0, 255));
    displayImage(windowName, image);
    cv::waitKey(0);
  }
}


void TennisCourtFitter::sortHorizontalLines(std::vector<Line>& hLines, const cv::Mat& rgbImage)
{
  float x = rgbImage.cols-1;
  sortLinesByDistanceToPoint(hLines, Point2f(rgbImage.cols / 2, 0));
  //std::sort(hLines.begin(), hLines.end(), sorthLine);

  if (false)
  {
    for (auto& line: hLines)
    {
        std::cout << line.getVector().y << std::endl;
      Mat image = rgbImage.clone();
      drawLine(line, image, Scalar(255, 0, 0));
      displayImage(windowName, image);
    }
  }
}

void TennisCourtFitter::sortHorizontalLines2(std::vector<Line>& hLines, const cv::Mat& rgbImage)
{
    float x = rgbImage.cols - 1;
    sortLinesByDistanceToPoint(hLines, Point2f(0, rgbImage.rows / 2));
    //std::sort(hLines.begin(), hLines.end(), sorthLine);

    if (false)
    {
        for (auto& line : hLines)
        {
            std::cout << line.getVector().y << std::endl;
            Mat image = rgbImage.clone();
            drawLine(line, image, Scalar(255, 0, 0));
            displayImage(windowName, image);
        }
    }
}

void TennisCourtFitter::sortVerticalLines(std::vector<Line>& vLines, const cv::Mat& rgbImage)
{
  sortLinesByDistanceToPoint(vLines, Point2f(0, rgbImage.rows / 2));

   // std::sort(vLines.begin(), vLines.end(), sortvLine);
  
  if (false)
  {
    for (auto& line: vLines)
    {
        std::cout << line.getVector().y << std::endl;
      Mat image = rgbImage.clone();
      drawLine(line, image, Scalar(0, 255, 0));
      displayImage(windowName, image);
    }
  }
}
void TennisCourtFitter::sortVerticalLines2(std::vector<Line>& vLines, const cv::Mat& rgbImage)
{
    sortLinesByDistanceToPoint(vLines, Point2f(rgbImage.cols / 2, rgbImage.rows));

    // std::sort(vLines.begin(), vLines.end(), sortvLine);

    if (false)
    {
        for (auto& line : vLines)
        {
            std::cout << line.getVector().y << std::endl;
            Mat image = rgbImage.clone();
            drawLine(line, image, Scalar(0, 255, 0));
            displayImage(windowName, image);
        }
    }
}

void TennisCourtFitter::localFindBest(arg_struct* arg)
{
  //std::cerr << arg->start<<arg->end<<std::endl;
  TimeMeasurement::start("singleThread" + std::to_string(arg->thread_id));
  for(int idx = arg->thread_id; idx < hLinePairs.size(); idx+=arg->num_threads)
  {
    TennisCourtModel model;
    for (auto& vLinePair : vLinePairs)
    {
      float score = model.fit(hLinePairs[idx], vLinePair, arg->binaryImage, arg->rgbImage);
      mutex.lock();
      if (score > bestModel.getBestScore())
      {
        bestModel = model;
        if (debug)
        {
            //std::cout << "Found better model " << i << " " << j << ", score: " << bestScore << std::endl;
            Mat image = arg->rgbImage.clone();
            bestModel.drawModel(image);
            drawLine(hLinePairs[idx].first, image, cv::Scalar(128, 128, 0));
            drawLine(hLinePairs[idx].second, image, cv::Scalar(128, 128, 0));
            drawLine(vLinePair.first, image, cv::Scalar(128, 0, 128));
            drawLine(vLinePair.second, image, cv::Scalar(128, 0, 128));
            displayImage(windowName, image, 10);
            std::cout << "Better Score: " << score << std::endl;
        }
        
      }
      mutex.unlock();
    }
  }
  double cur_time = TimeMeasurement::stop("singleThread" + std::to_string(arg->thread_id));
  output_mutex.lock();
  std::cerr << "id: "<<arg->thread_id << ", time: " << cur_time << std::endl;
  output_mutex.unlock();
}

void TennisCourtFitter::findBestModelFit(const cv::Mat& binaryImage, const cv::Mat& rgbImage, const int num_threads)
{
  std::cout<<"Number of threads: "<<num_threads<<std::endl;
  std::cout<< "total_hpairs: " << hLinePairs.size() << std::endl;
  std::vector<std::thread> threads(num_threads);
  int total_hpairs = hLinePairs.size();
  int each_hpairs = total_hpairs / num_threads;
  int offset = total_hpairs % num_threads;

  std::vector<arg_struct> args_list(num_threads);
  

  for(int i = 0; i < num_threads; i++)
  {
    args_list[i].num_threads = num_threads;
    args_list[i].binaryImage = binaryImage;
    args_list[i].rgbImage = rgbImage;
    args_list[i].thread_id = i;
		threads[i] = std::thread(&TennisCourtFitter::localFindBest, this, &args_list[i]);
  }

  // };
  int i = 0;
  std::cerr << "Iterative searching...";
  // std::vector<std::thread> thread_list;
  // for (auto& hLinePair : hLinePairs)
  // {
  //     thread_list.push_back(std::thread(parallel_fitting, hLinePair));
  //     std::cerr << i << " ";
  //     i++;
  // }
  for(int i = 0; i < num_threads; i++)
	{
		threads[i].join();
	}
  
    /*for (int it = 0; it <= parameters.finetune_iteration; it++)
    {
        std::cout << "finetune..." << it << std::endl;
        bestModel.finetune(binaryImage, rgbImage, Lines);
    }
    if (debug)
    {
        std::cout << "Best model score = " << bestScore << std::endl;
        Mat image = rgbImage.clone();
        bestModel.drawModel(image);
        displayImage(windowName, image);
    }*/

    //std::cout << "Fine tune..." << std::endl;
    

    
}
