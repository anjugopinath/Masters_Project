#include <iostream>
 
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
 
// ZED
#include <zed/Camera.hpp>
#define DEPTH_WINDOW "Depth"
#define DISPARITY_WINDOW "Disparity"
#define IMAGE_WINDOW "Image"
#define LEFT_WINDOW "Left"
#define RIGHT_WINDOW "Right"
#define DISPARITY_WINDOW "ST Aggreg Disparity"
 
 

using namespace sl::zed;
using namespace std;
using namespace cv;
// Input from keyboard
char keyboard = ' ';

Camera* zed; 
 
///ELAS


 
/// ZED & ELAS Integration code..
 

int main(int argc, char** argv)
{ 
	  char command[70];
 

// ZED camera 
 
    zed = new Camera(VGA);   // max_disp 170   // 672*376, supported framerates : 15, 30, 60, 100 fps, Jetson TK1 : 15, 30 fps 
   // zed = new Camera(HD720);   //260 // 1280*720, supported framerates : 15, 30, 60 fps, Jetson TK1 : 15 fps 

   // zed = new Camera(HD1080);  //300// 1920*1080, supported framerates : 15, 30 fps (unsupported by the Jetson TK1 at the moment) 

   // zed = new Camera(HD2K);   //340 // 2208*1242, supported framerate : 15 fps (unsupported by the Jetson TK1 at the moment) 

// INITIALIZE the parameters
    sl::zed::InitParams params;
    params.mode = PERFORMANCE;
    params.unit = METER;  // scale to fit openGL world
    params.coordinate = RIGHT_HANDED;		// openGL compatible
    params.verbose = true;
 
    ERRCODE err = zed->init(params);
    cout << errcode2str(err) << endl;
    if (err != SUCCESS) {
        delete zed;
        return 1;
    }


    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;

    cv::Mat left_image(height, width, CV_8UC4,1);
    cv::Mat right_image(height, width, CV_8UC4,1); 
    cv::Mat depth(height, width, CV_8UC4,1); 

    cv::namedWindow(IMAGE_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(DEPTH_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(LEFT_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(RIGHT_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(DISPARITY_WINDOW, cv::WINDOW_AUTOSIZE);


// Loop until 'q' is pressed
int count=0;
while (keyboard != 'q') {
 
	 // Grab frame and compute depth in FULL sensing mode
	 if (!zed->grab(sl::zed::SENSING_MODE::STANDARD ))
	 {
	 
		 // Retrieve left color image
		 sl::zed::Mat left =  zed->retrieveImage(sl::zed::SIDE::LEFT);
		 sl::zed::Mat right = zed->retrieveImage(sl::zed::SIDE::RIGHT);
                 sl::zed::Mat disparity = zed->retrieveMeasure(sl::zed::MEASURE::DISPARITY);
                 
                // cout<<"disparity measure..: rows->"<<disparity.getDataSize()<<endl;

		 //memcpy(disparity.data,depth.data,width*height*4*sizeof(uchar));
		    
                 //convert to CV Mat
                 slMat2cvMat(left).copyTo(left_image);
 		 slMat2cvMat(right).copyTo(right_image);

		// Retrieve depth map
		 sl::zed::Mat depthmap = zed->normalizeMeasure(sl::zed::MEASURE::DISPARITY); 
		 slMat2cvMat(depthmap).copyTo(depth); 
 
                 cv::imshow(DEPTH_WINDOW, depth);
		 // Display image in OpenCV window 
		 cv::imshow(LEFT_WINDOW, left_image);
		 cv::imshow(RIGHT_WINDOW, right_image);
	  
	 }
	
	keyboard = cv::waitKey(30);
        if(keyboard=='s'){
            count++;
           //save the image : PNG
           string leftImgNamePNG  =  std::to_string(count)+ "_left.png";
           string rightImgNamePNG =  std::to_string(count)+"_right.png";
 
             //: OutputDisparity file
           string dispOutPNG= std::to_string(count)+"_disp_out.png";
           

           cv::imwrite(leftImgNamePNG,left_image);
           cv::imwrite(rightImgNamePNG,right_image);
            

           
          
            
         //Compute Disparity Image . 
           sprintf(command,"../src/STCostAggre %s %s %s", leftImgNamePNG.c_str() ,rightImgNamePNG.c_str(),dispOutPNG.c_str() );
           std::system(command);  
           cout<<" Disparity Image saved..."<<count<<endl;


           // show the disparity image
            cv::Mat disparityImgPNG;
            disparityImgPNG = imread(dispOutPNG.c_str(),CV_LOAD_IMAGE_UNCHANGED);
             if (disparityImgPNG.empty()) //check whether the image is loaded or not
             {
		          cout << "Error : Image cannot be loaded..!!" <<dispOutPNG.c_str()<< endl;
		         //system("pause"); //wait for a key press
		  
            }else{ 
                imshow(DISPARITY_WINDOW, disparityImgPNG);
            }
            cv::waitKey(10);
	     
         }
          
}
    delete zed; 
    return  0;
}
