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
  if (argc != 7) {
  	cout << "Usage: ./undistort [left_distorted_img] [right_distorted_img] [left_camera_yaml] [right_camera_yaml] [left_undistorted] [right_undistorted]" << endl; 

	return 0; 
  }

  cv::Mat K1, K2;
  cv::Vec4d D1, D2;

  //Distorted image from left -> img1 and right -> img2
  cv::Mat img1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::Mat img2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

  //Camera matrices (i.e. .yaml files for left and right respectively)
  cv::FileStorage fs1(argv[3], cv::FileStorage::READ);
  cv::FileStorage fs2(argv[4], cv::FileStorage::READ);
  fs1["K1"] >> K1;
  fs1["D1"] >> D1;
  fs2["K1"] >> K2;
  fs2["D1"] >> D2;
  
  cv::Mat imgU1, imgU2;

  Matx33d K1new = Matx33d(K1); 
  //K1new(0, 0) = 1; K1new(1, 1) = 1; K1new(2, 2) = 1;
  Matx33d K2new = Matx33d(K2); 
  //K2new(0, 0) = 1; K2new(1, 1) = 1; K2new(2, 2) = 1;

  cv::fisheye::undistortImage(img1, imgU1, Matx33d(K1), Mat(D1), K1new);
  cv::fisheye::undistortImage(img2, imgU2, Matx33d(K2), Mat(D2), K2new);
  int k = waitKey(0);
  if (k > 0) {
		cout << "writing to file..." << endl;
    imwrite(argv[5], imgU1);
    imwrite(argv[6], imgU2);
  }
  return 0;
}
