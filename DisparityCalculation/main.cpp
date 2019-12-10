#include <stdio.h>
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc.hpp>
    
    using namespace cv;
    using namespace std;

    const char *windowDisparity = "Disparity";
    
    void readme();
    static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}
    int main( int argc, char** argv )
    {
    
      //-- 1. Read the images
      Mat imgLeft = imread(argv[1], IMREAD_GRAYSCALE );
      Mat imgRight = imread( argv[2], IMREAD_GRAYSCALE );
      //-- And create the image in which we will save our disparities
      Mat imgDisparity16S = Mat( imgLeft.rows, imgLeft.cols, CV_16S );
      Mat imgDisparity8U = Mat( imgLeft.rows, imgLeft.cols, CV_8UC1 );
    
      if( imgLeft.empty() || imgRight.empty() )
      { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
    
      //-- 2. Call the constructor for StereoBM
      int ndisparities = 16*14;   
      int SADWindowSize = 21; 
      Ptr<StereoBM> sbm = StereoBM::create( ndisparities, SADWindowSize );
    
      //-- 3. Calculate the disparity image
      sbm->compute( imgLeft, imgRight, imgDisparity16S );
    
      //-- Check its extreme values
      double minVal; double maxVal;
    
      minMaxLoc( imgDisparity16S, &minVal, &maxVal );
    
      printf("Min disp: %f Max value: %f \n", minVal, maxVal);
    
      //-- 4. Display it as a CV_8UC1 image
      imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal)); 
      namedWindow( windowDisparity, WINDOW_NORMAL );
      imshow( windowDisparity, imgDisparity8U );
    
      //-- 5. Save the image
      imwrite("SBM_sample.png", imgDisparity16S);


//computing the 3D points... 

     Size imageSize=imgDisparity16S.size();
     float h=(float)imageSize.height;
     float w=(float)imageSize.width;
     float f=(float)0.8*w;
     
     float data_f[16]={1, 0, 0, (float)-0.5*w,
                       0,-1, 0,  (float) 0.5*h,               
                       0, 0, 0,     -f,
                       0, 0, 1,     0 };
     Mat _Q=  Mat(4,4,CV_32F,data_f); 
     Mat xyz;
     reprojectImageTo3D(imgDisparity16S,xyz,  _Q,true,-1);
     saveXYZ("output.ply",xyz);
      
 



//    imshow("3d" , threeDimage);
     waitKey(0);
    
      return 0;
    }


    void readme()
    { std::cout << " Usage: ./SBMSample <imgLeft> <imgRight>" << std::endl; }

