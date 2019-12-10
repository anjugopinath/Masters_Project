/*
Author: Abhishek Kumar
./ZED_save_img ../im0.png ../im1.png  out.pfm 260


*/


#include <iostream>
 
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
 
 
 
#define DEPTH_WINDOW "Depth"
#define IMAGE_WINDOW "Image"
#define LEFT_WINDOW "Left"
#define RIGHT_WINDOW "Right"
#define DISPARITY_WINDOW "Disparity"
//ELAS
#include <iostream>
#include <time.h>
#include "elas.h"
#include "image.h"
#include <math.h>
 
//png2pgm
//#include "imageLib.h"

 
using namespace std;
using namespace cv;


//Camera poaramaeters
    double f= 349.421 ; 
    double cx=294.182;
    double cy=252.932;
    double cx_prime=326.96;
    double cy_prime=252.932;
    double B= 119.824;
    double doffs=32.778;  






// Input from keyboard
char keyboard = ' ';

 

// check whether machine is little endian
int littleendian2()
{
    int intval = 1;
    uchar *uval = (uchar *)&intval;
    return uval[0] == 1;
}

// write pfm image (added by DS 10/24/2013)
// 1-band PFM image, see http://netpbm.sourceforge.net/doc/pfm.html
cv::Mat WriteFilePFM(float *data, int width, int height,  float scalefactor=1/255.0)
{
     
    int n = width;
    cv::Mat matfile=Mat_<float>(height,width);
    // write rows -- pfm stores rows in inverse order!
    for (int y = height-1; y >= 0; y--) {
	float* ptr = data + y * width;
	// change invalid pixels (which seem to be represented as -10) to INF
	for (int x = 0; x < width; x++) {
	    if (ptr[x] < 0)
		ptr[x] = INFINITY;
           // printf("**.. %f\n",ptr[x]);
            matfile.at<float>(y,x)=ptr[x];
	} 
    }
      
   return matfile;
     
}



// compute disparities of pgm image input pair file_1, file_2
cv::Mat process (const char* file_1, const char* file_2,  int maxdisp, int no_interp,cv::Mat left) 
{ 
    cv::Mat matfile;
    clock_t c0 = clock();

    // load images
    image<uchar> *I1,*I2;
    I1 = loadPGM(file_1);
    I2 = loadPGM(file_2);
    
    // check for correct size
    if (I1->width()<=0 || I1->height() <=0 || I2->width()<=0 || I2->height() <=0 ||
	I1->width()!=I2->width() || I1->height()!=I2->height()) {
	cout << "ERROR: Images must be of same size, but" << endl;
	cout << "       I1: " << I1->width() <<  " x " << I1->height() << 
	    ", I2: " << I2->width() <<  " x " << I2->height() << endl;
	delete I1;
	delete I2;
	return matfile;    
    }

    // get image width and height
    int32_t width  = I1->width();
    int32_t height = I1->height();

    // allocate memory for disparity images
    const int32_t dims[3] = {width,height,width}; // bytes per line = width
    float* D1_data = (float*)malloc(width*height*sizeof(float));
    float* D2_data = (float*)malloc(width*height*sizeof(float));
  
    // process
    Elas::parameters param(Elas::MIDDLEBURY);
    if (no_interp) {
	//param = Elas::parameters(Elas::ROBOTICS);
	// don't use full 'robotics' setting, just the parameter to fill gaps
        param.ipol_gap_width = 3;
    }
    param.postprocess_only_left = false;
    param.disp_max = maxdisp;
    Elas elas(param);
    elas.process(I1->data,I2->data,D1_data,D2_data,dims,left);
    
    // added runtime output - DS 4/4/2013
    clock_t c1 = clock();
    double secs = (double)(c1 - c0) / CLOCKS_PER_SEC;
    printf("runtime: %.2fs  (%.2fs/MP)\n", secs, secs/(width*height/1000000.0));

    // save disparity image

      matfile=WriteFilePFM(D1_data, width, height, 1.0/maxdisp);
    cout<<"Disparity File saved.."<<endl;

    // free memory
    delete I1;
    delete I2;
    free(D1_data);
    free(D2_data);
    return matfile;
}
//draw a cirle on the image
static void drawCircle( cv::Mat img, cv::Point center )
{
 int thickness =  1;
 int lineType = 8;

 cv::circle( img, center, 3, Scalar( 0, 0, 255 ), thickness, lineType );
}
/// ZED & ELAS Integration code..
 

int main(int argc, char** argv)
{
	
    cv::namedWindow(IMAGE_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(DEPTH_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(LEFT_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(RIGHT_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(DISPARITY_WINDOW, cv::WINDOW_AUTOSIZE);

	//ELAS processing..
    const char *file1 = "im0.pgm";
    const char *file2 = "im1.pgm";
    const char *outfile = argv[3];
    int maxdisp = atoi(argv[4]);
    int no_interp = 0;
    if (argc > 5)
	no_interp = atoi(argv[5]);
          char command[70];


       //#### convert into pgm
         //LEFT Image
           sprintf(command,"../src/png2pgm %s %s", argv[1] ,file1);
           std::system(command); 
          //RIGHT Image
           sprintf(command,"../src/png2pgm %s %s",argv[2] ,file2);
           std::system(command);
           cv::Mat left=cv::imread(argv[1]);
           cv::Mat right=cv::imread(argv[2]);
           
     // apply ELAS
     cv::Mat tempMat =process(file1,file2,maxdisp,no_interp,left);
     
     //draw the support points.............here....
      /*

25 280 20
30 10 13
30 25 13
30 110 14
30 125 11
30 165 13
30 180 13
30 260 20
30 280 22
30 295 24
30 310 25
35 5 13
   */

     cv::Point p1=Point(15,5);
     cv::Point p2=Point(280,25);
     cv::Point p3=Point(10,30);
     cv::Point p4=Point(110,30);
     cv::Point p5=Point(295,30);
     cv::Point p6=Point(260,30);
     cv::Point p7=Point(15,5);
      
     drawCircle(left,p1);
     drawCircle(left,p2);
     drawCircle(left,p3);
     drawCircle(left,p4);
     drawCircle(left,p5);
     drawCircle(left,p6);
     drawCircle(left,p7);
    
    
     cv::imshow(LEFT_WINDOW,left);
     cv::imshow(DEPTH_WINDOW, tempMat/64.0	);
     cv::waitKey(0);
     printf("\n...Disparity created....\n"); 
	//ELAS ends

       //#D reconstruction of the disparity map
      



 




 
    return  0;
}
