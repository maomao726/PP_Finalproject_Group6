#include <opencv2/opencv.hpp>
#include "GlobalParameters.h"
#include "TimeMeasurement.h"
#include "Line.h"
#include "CourtLinePixelDetector.h"
#include "CourtLineCandidateDetector.h"
#include "TennisCourtFitter.h"
#include "TennisCourtModel.h"
#include "DebugHelpers.h"
#include <cstring>

using namespace cv;

int main(int argc, char** argv)
{
  TimeMeasurement::debug = false;
  CourtLinePixelDetector::debug = false;
  CourtLineCandidateDetector::debug = false;
  TennisCourtFitter::debug = false;
  TennisCourtModel::debug = false;
  

  if (argc < 7 || argc > 8)
  {
    std::cout << "Usage: ./detect input_format input_path [output_path]" << std::endl;
    std::cout << "       input_format: -i input an image file." << std::endl;
    std::cout << "                     -v input an video file." << std::endl;
    std::cout << "       input_path:   path to an input file." << std::endl;
    std::cout << "       court_type:   -f input full court image." << std::endl;
    std::cout << "                     -h input half court image." << std::endl;
    std::cout << "       num_threads:  -p num_of_threads" << std::endl;
    std::cout << "       color_format:   color of court line." << std::endl;
    std::cout << "       output_path:  path to an output file where the xy court point coordinates will be written." << std::endl;
    std::cout << "                     This argument is optional. If not present, then a window with the result will be opened." << std::endl;
    return -1;
  }
  std::string filename(argv[2]);
  Mat frame;
  if (strcmp(argv[1],"-v") == 0)
  {
      std::cout << "Reading video file " << filename << std::endl;
      VideoCapture vc(filename);
      if (!vc.isOpened())
      {
          std::cerr << "Cannot open file " << filename << std::endl;
          return 1;
      }
      printVideoInfo(vc);

      //int frameIndex = int(vc.get(CV_CAP_PROP_FRAME_COUNT)) / 2;
      //std::cout << frameIndex << std::endl;
      int frameIndex = 300;
      vc.set(CV_CAP_PROP_POS_FRAMES, frameIndex);
      if (!vc.read(frame))
      {
          std::cerr << "Failed to read frame with index " << frameIndex << std::endl;
          return 2;
      }
      std::cout << "Reading frame with index " << frameIndex << std::endl;
      vc.release();
  }
  else if (strcmp(argv[1], "-i") == 0)
  {
      std::cout << "Reading image file " << filename << std::endl;
      frame = imread(filename);
      if (frame.empty())
      {
          std::cerr << "Failed to read image " << filename << std::endl;
          return 2;
      }
  }
  
  TennisCourtModel::isfullCourt = (strcmp(argv[3], "-f") == 0);
  CourtLinePixelDetector courtLinePixelDetector;
  CourtLineCandidateDetector courtLineCandidateDetector;
  TennisCourtFitter tennisCourtFitter;

  std::cout << "Starting court line detection algorithm..." << std::endl;
  //displayImage("Original", frame);
  try
  {
    Mat frame_resized;
    resize(frame, frame_resized, Size(GlobalParameters().resize_size, GlobalParameters().resize_size), 0, 0, INTER_AREA);
    TimeMeasurement::start("LineDetection");
    Mat binaryImage = courtLinePixelDetector.run(frame_resized, argv[6]);
    double elapsed_seconds = TimeMeasurement::stop("LineDetection");
    std::cout << "LineDetection time: " << elapsed_seconds << "s." << std::endl;
    
    TimeMeasurement::start("LineCandidateDetection");
    std::vector<Line> candidateLines = courtLineCandidateDetector.run(binaryImage, frame_resized);
    elapsed_seconds = TimeMeasurement::stop("LineCandidateDetection");
    std::cout << "LineCandidateDetection time: " << elapsed_seconds << "s." << std::endl;
    
    TimeMeasurement::start("PP CourtFitter");
    TennisCourtModel model = tennisCourtFitter.run(candidateLines, binaryImage, frame_resized, std::atoi(argv[5]));
    elapsed_seconds = TimeMeasurement::stop("PP CourtFitter");
    std::cout << "PP CourtFitter time: " << elapsed_seconds << "s." << std::endl;
    model.fixResized(frame.cols, frame.rows);
    if (argc == 7)
    {
      model.drawModel(frame);
      displayImage("Result - press key to exit", frame);
      model.getCornerPoints();
      imwrite("a.png", frame);
    }
    else if (argc == 8)
    {
      std::string outFilename(argv[5]);
      model.drawModel(frame);
      displayImage("Result", frame);

      model.writeToFile(outFilename);
      std::cout << "Result written to " << outFilename << std::endl;
    }
    
  }
  catch (std::runtime_error& e)
  {
    std::cout << "Processing error: " << e.what() << std::endl;
    return 3;
  }

  
  return 0;
}
