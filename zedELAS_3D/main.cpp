#include <iostream>
 
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
 
// ZED
#include <zed/Camera.hpp>
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

using namespace sl::zed;
using namespace std;
using namespace cv;
// Input from keyboard
char keyboard = ' ';

Camera* zed; 
 
///ELAS



// check whether machine is little endian
int littleendian2()
{
    int intval = 1;
    uchar *uval = (uchar *)&intval;
    return uval[0] == 1;
}

// write pfm image (added by DS 10/24/2013)
// 1-band PFM image, see http://netpbm.sourceforge.net/doc/pfm.html
void WriteFilePFM(float *data, int width, int height, const char* filename, float scalefactor=1/255.0)
{
    // Open the file
    FILE *stream = fopen(filename, "wb");
    if (stream == 0) {
        fprintf(stderr, "WriteFilePFM: could not open %s\n", filename);
	exit(1);
    }

    // sign of scalefact indicates endianness, see pfms specs
    if (littleendian2())
	scalefactor = -scalefactor;

    // write the header: 3 lines: Pf, dimensions, scale factor (negative val == little endian)
    fprintf(stream, "Pf\n%d %d\n%f\n", width, height, scalefactor);

    int n = width;
    // write rows -- pfm stores rows in inverse order!
    for (int y = height-1; y >= 0; y--) {
	float* ptr = data + y * width;
	// change invalid pixels (which seem to be represented as -10) to INF
	for (int x = 0; x < width; x++) {
	    if (ptr[x] < 0)
		ptr[x] = INFINITY;
	}
	if ((int)fwrite(ptr, sizeof(float), n, stream) != n) {
	    fprintf(stderr, "WriteFilePFM: problem writing data\n");
	    exit(1);
	}
    }
    
    // close file
    fclose(stream);
}



// compute disparities of pgm image input pair file_1, file_2
void process (const char* file_1, const char* file_2, const char* outfile, int maxdisp, int no_interp) 
{ 
	 
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
	return;    
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
    elas.process(I1->data,I2->data,D1_data,D2_data,dims);

    // added runtime output - DS 4/4/2013
    clock_t c1 = clock();
    double secs = (double)(c1 - c0) / CLOCKS_PER_SEC;
    printf("runtime: %.2fs  (%.2fs/MP)\n", secs, secs/(width*height/1000000.0));

    // save disparity image

    WriteFilePFM(D1_data, width, height, outfile, 1.0/maxdisp);
    cout<<"Disparity File saved.."<<endl;
    // free memory
    delete I1;
    delete I2;
    free(D1_data);
    free(D2_data);
}

/// ZED & ELAS Integration code..
 

int main(int argc, char** argv)
{
	
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
     // apply ELAS
    process(file1,file2,outfile,maxdisp,no_interp);
    printf("\n...Disparity created....\n"); 
	//ELAS ends


// ZED camera 



    zed = new Camera(VGA);   // max_disp 170   // 672*376, supported framerates : 15, 30, 60, 100 fps, Jetson TK1 : 15, 30 fps 
   // zed = new Camera(HD720);   //260 // 1280*720, supported framerates : 15, 30, 60 fps, Jetson TK1 : 15 fps 

   // zed = new Camera(HD1080);  //300// 1920*1080, supported framerates : 15, 30 fps (unsupported by the Jetson TK1 at the moment) 

   // zed = new Camera(HD2K);   //340 // 2208*1242, supported framerate : 15 fps (unsupported by the Jetson TK1 at the moment) 


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
             //PGM 
           string leftImgNamePGM  =  std::to_string(count)+ "_left.pgm";
           string rightImgNamePGM = std::to_string(count)+"_right.pgm";
             //PGM: OutputDisparity file
           string dispOutPFM= std::to_string(count)+"_disp_out.pfm";
           

           cv::imwrite(leftImgNamePNG,left_image);
           cv::imwrite(rightImgNamePNG,right_image);
            

           
          //convert to PGM
            
         //LEFT Image
           sprintf(command,"../src/png2pgm %s %s", leftImgNamePNG.c_str() ,leftImgNamePGM.c_str() );
           std::system(command); 
          //RIGHT Image
           sprintf(command,"../src/png2pgm %s %s",rightImgNamePNG.c_str() ,rightImgNamePGM.c_str());
           std::system(command); 
     
          //Apply ELAS ALgo
            process(leftImgNamePGM.c_str(),rightImgNamePGM.c_str(),dispOutPFM.c_str() , maxdisp, 0);
            cout<<"Image saved..."<<count;


           // show the disparity image
            cv::Mat disparityImgPFM;
            disparityImgPFM = imread(dispOutPFM.c_str(),CV_LOAD_IMAGE_UNCHANGED);
             if (disparityImgPFM.empty()) //check whether the image is loaded or not
             {
		  cout << "Error : Image cannot be loaded..!!" <<dispOutPFM.c_str()<< endl;
		  //system("pause"); //wait for a key press
		  
            }else{ 
                imshow(DISPARITY_WINDOW, disparityImgPFM);
            }
            cv::waitKey(10);
	     
         }
          
}
    delete zed; 
    return  0;
}
