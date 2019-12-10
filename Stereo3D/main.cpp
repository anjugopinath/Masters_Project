

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <string>
#include <utility>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include<opencv2/calib3d.hpp>
using namespace cv;
int main()
{cv::Mat leftimg =cv::imread("leftimage.jpg");
cv::Mat rightimg = cv::imread("rightimage.jpg");
cv::Size imagesize = leftimg.size();
cv::Mat disparity_left=cv::Mat(imagesize.height,imagesize.width,CV_16S);
cv::Mat disparity_right=cv::Mat(imagesize.height,imagesize.width,CV_16S);
cv::Mat g1,g2,disp,disp8;
cv::cvtColor(leftimg,g1,cv::COLOR_BGR2GRAY);
cv::cvtColor(rightimg,g2,cv::COLOR_BGR2GRAY);

cv::Ptr<cv::StereoBM> sbm = cv::createStereoBM(16,21);


sbm->setDisp12MaxDiff(1);
sbm->setSpeckleRange(8);
sbm->setSpeckleWindowSize(9);
sbm->setUniquenessRatio(0);
sbm->setTextureThreshold(507);
sbm->setMinDisparity(-39);
sbm->setPreFilterCap(61);
sbm->setPreFilterSize(5);
sbm->compute(g1,g2,disparity_left);
normalize(disparity_left, disp8, 0, 255, CV_MINMAX, CV_8U);

cv::imshow("left", leftimg);
cv::imshow("right", rightimg);
cv::imshow("disp", disp8);

cv::waitKey(0);
}


