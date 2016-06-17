/*
   This program is used to undistort an image. 

   Author Sourish Ghosh
*/

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
  if (argc != 3) {
  	cout << "Usage: ./undistort [distorted_img] [undistorted_img]" << endl; 

	return 0; 
  }

  cv::Mat K1, K2;
  cv::Vec4d D1, D2;

  //Distorted image from left -> img1 and right -> img2
  cv::Mat img1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  //cv::Mat img2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

  cv::Mat img1_resized, img2_resized; 

  //Resizes before undistorting. 
  resize(img1, img1_resized, Size(960, 600));
  //resize(img2, img2_resized, Size(960, 600));

  //Camera matrices (i.e. .yml files for left and right respectively)
  cv::FileStorage fs1("/home/rudy/Desktop/UMASS/CICS/data/undistort/cam_left.yml", cv::FileStorage::READ);
  //cv::FileStorage fs2("/home/rudy/Desktop/UMASS/CICS/data/undistort/cam_right.yml", cv::FileStorage::READ);
  fs1["K1"] >> K1;
  fs1["D1"] >> D1;
  //fs2["K1"] >> K2;
  //fs2["D1"] >> D2;
  
  cv::Mat left_u1, right_u1; 

  Matx33d K1new = Matx33d(K1); 
  //K1new(0, 0) = 1; K1new(1, 1) = 1; K1new(2, 2) = 1;
  //Matx33d K2new = Matx33d(K2); 
  //K2new(0, 0) = 1; K2new(1, 1) = 1; K2new(2, 2) = 1;

  //Undistorts images. 
  cv::fisheye::undistortImage(img1_resized, left_u1, Matx33d(K1), Mat(D1), K1new);
  //cv::fisheye::undistortImage(img2_resized, right_u1, Matx33d(K2), Mat(D2), K2new);

  //Resizes the images to their final resolution. 
  cv::Mat left_u2, right_u2;

  resize(left_u1, left_u2, Size(360, 224));
  //resize(right_u1, right_u2, Size(360, 224));

  cout << "writing to file..." << endl;
  imwrite(argv[2], left_u2);
  //imwrite(argv[4], right_u2);
  
  return 0;
}
