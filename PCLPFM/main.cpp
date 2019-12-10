/*
Author: Abhishek Kumar

## to runt The Executatble file:

./ZED_save_img ../im0.png ../im1.png  out.pfm 260


*/


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
#include <time.h>
#include "elas.h"
#include "image.h"
#include <math.h>
 
//png2pgm
//#include "imageLib.h"

using namespace sl::zed;
using namespace std;
using namespace cv;


//Camera poaramaeters
    double f= 351.253 ; 
    double cx=348.179;
    double cy=187.267;
    double cx_prime=325.433;
    double cy_prime=186.932;
    double B= 119.824;
    double doffs=32.778;  
 

// Input from keyboard
char keyboard = ' ';

Camera* zed; 
 
///ELAS
//prefix lines for output format for .ply file to be viewd in meshlab
char ply_header[]="ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

//###################### save the 3d points with color information. in .ply format ############################ 
static void saveXYZ(const char* filename, const cv::Mat& mat,const cv:: Mat& colorImg)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt"); 
    char prefix [200]; 
    std::string content="";  
    int count=0; //to count the  toatl vertices in the ply file
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3b color = colorImg.at<Vec3b>(y,x); //save color info
            Vec3f point = mat.at<Vec3f>(y,x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z )
            {
                   continue;
            }
            count++; 
           string output = to_string( point[0] )+" "+to_string(point[1])+" "+to_string(point[2])+" "+to_string(color[0])+" "+to_string(color[1])+" "+to_string(color[2])+"\n";
         // fprintf(fp, "%f %f %f %d %d %d\n", point[0], point[1], point[2],color[0],color[1],color[2]) // // writng into file eachtime is not fast. so write all at the end. till then. store int string
           content+=output;
        }
    }

    //write the file
     sprintf(prefix,ply_header,count);
     fprintf(fp,"%s",prefix); //write the ply header
     fprintf(fp,"%s",content.c_str()); //write the 3d data
      
    cout<<"....Total 3D Points ***.....:  "<<count<<endl;
    fclose(fp);
} 

 //##########  local method to compute the depth
 static void saveXYZ2(const char* filename, const cv::Mat& mat,const cv::Mat& colorImg)
{
      f=  351.253 ; 
      cx=348.179;
      cy=187.267;
      cx_prime=325.433;
      cy_prime=186.932;
      B= 119.824;
      doffs=32.0;  
 
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt"); 
    char prefix [200]; 
    std::string content="";  
    int count=0; //to count the  toatl vertices in the ply file
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            float d=mat.at<float>(y,x); 																									;   
            float Z=f*B/(d+doffs); 
            float X=(x-cx)  ;
            float Y=(y-cy)  ; 
            //cout<<"X:"<<X<<endl;
            //cout<<"Y:"<<Y<<endl;
            //cout<<"Z:"<<Z<<endl;
             
            Vec3b color = colorImg.at<Vec3b>(y,x); //save color info
            Vec3f point = mat.at<Vec3f>(y,x); 
            if(fabs(Z - max_z) <  5 || fabs(Z) > max_z || Z!=Z)
            {
                  continue;  //don't include NAN or infinity depths
            }
           count++; 
           string output = to_string( X )+" "+to_string(-Y)+" "+to_string(-Z)+" "+to_string(color[0])+" "+to_string(color[1])+" "+to_string(color[2])+"\n";
           // sprintf(s, "%f %f %f %d %d %d\n",  X, -Y,  -Z,color[0],color[1],color[2]); // writng into file eachtime is not fast. so write all at the end. till then. store int string
           content+=output;
        }
    }

    //write the file
     sprintf(prefix,ply_header,count);
     fprintf(fp,"%s",prefix); //write the ply header
     fprintf(fp,"%s",content.c_str()); //write the 3d data
      
    cout<<"....**.....:  "<<count<<endl;
    fclose(fp);
} 

//############### MEthod to write the .pcd file by Anju G.
	static void saveXYZPCD(const char* filename, const cv::Mat& mat,const cv::Mat& colorImg){
			FILE* fp = fopen(filename, "wt");		
			char prefix [500];
			fprintf(fp,"VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"); 
			fprintf(fp,"WIDTH %d\n",mat.cols); 
			fprintf(fp,"HEIGHT %d\n",mat.rows); 
			fprintf(fp,"VIEWPOINT 0 0 0 1 0 0 0\n"); 
			fprintf(fp,"POINTS %d\n",mat.rows*mat.cols); 
			fprintf(fp,"DATA ascii\n"); 
			const double max_z = 1.0e4;
		
		
			for(int y = 0; y < mat.rows; y++)
	   		 {
				for(int x = 0; x < mat.cols; x++)
				{
				    Vec3b color = colorImg.at<Vec3b>(y,x); //save color info
				    Vec3f point = mat.at<Vec3f>(y,x);
				    if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
				    {
					  //continue;
				    }
		    			int rgb = ((int)color.val[0]) << 16 | ((int)color.val[1]) << 8 | ((int)color.val[2]);
					fprintf(fp, "%f %f %f %d\n", point[0], point[1], point[2],rgb);
				}
	    		}
	   	      fclose(fp);
	   }





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
cv::Mat process (const char* file_1, const char* file_2,  int maxdisp, int no_interp) 
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
    elas.process(I1->data,I2->data,D1_data,D2_data,dims);

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

//save point cloud from given color info & Disparity Image
void generatePointCloud1(cv::Mat left_image, cv::Mat matFileDisp, string cloudName){
            cv::Mat colors;
            cv::cvtColor(left_image,colors, CV_BGR2RGB);  
            //our method
	    saveXYZ2(cloudName.c_str(), matFileDisp,colors);
	   // ZED camera's default point cloud 
	    printf("XYZ2:Point cloud saved\n");     
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
     // apply ELAS
     cv::Mat tempMat =process(file1,file2,maxdisp,no_interp);
     cv::imshow(DEPTH_WINDOW, tempMat/64.0	);
     cv::waitKey(30);
     printf("\n...Disparity created....\n"); 
	//ELAS ends

       //#D reconstruction of the disparity map
      //
      cv::Mat tmpLeft_img=cv::imread(argv[1] , cv::IMREAD_COLOR);
      generatePointCloud1( tmpLeft_img, tempMat, "motor.ply" );











// ZED camera 
//


    zed = new Camera(VGA);   // max_disp 170   // 672*376, supported framerates : 15, 30, 60, 100 fps, Jetson TK1 : 15, 30 fps 
   //  zed = new Camera(HD720);   //260 // 1280*720, supported framerates : 15, 30, 60 fps, Jetson TK1 : 15 fps 

  //  zed = new Camera(HD1080);  //300// 1920*1080, supported framerates : 15, 30 fps (unsupported by the Jetson TK1 at the moment) 

   //zed = new Camera(HD2K);   //340 // 2208*1242, supported framerate : 15 fps (unsupported by the Jetson TK1 at the moment) 


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




// Loop until 'q' is pressed
int count=0;
while (keyboard != 'q') {
 
	 // Grab frame and compute depth in FULL sensing mode
	 if (!zed->grab(sl::zed::SENSING_MODE::STANDARD,1,1,1 ))
	 {
	 
		 // Retrieve left color image
		 sl::zed::Mat left =  zed->retrieveImage(sl::zed::SIDE::LEFT);
		 sl::zed::Mat right = zed->retrieveImage(sl::zed::SIDE::RIGHT);
                // sl::zed::Mat disparity = zed->retrieveMeasure(sl::zed::MEASURE::DISPARITY);
                 
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
  //save the depth if 's' key is pressed from the keyboard
   // if(keyboard=='s' || keyboard=='S'){
            count++;
           //save the image : PNG
           string leftImgNamePNG  =  std::to_string(count)+ "_left.png";
           string rightImgNamePNG =  std::to_string(count)+"_right.png";
             //PGM 
           string leftImgNamePGM  =  std::to_string(count)+ "_left.pgm";
           string rightImgNamePGM = std::to_string(count)+"_right.pgm";
 
          //save image to convert into pgm 
           cv::imwrite(leftImgNamePNG,left_image);
           cv::imwrite(rightImgNamePNG,right_image);
            

           
          //convert to PGM
            
         //LEFT Image
           sprintf(command,"../src/png2pgm %s %s", leftImgNamePNG.c_str() ,leftImgNamePGM.c_str() );
           std::system(command); 
          //RIGHT Image
           sprintf(command,"../src/png2pgm %s %s",rightImgNamePNG.c_str() ,rightImgNamePGM.c_str());
           std::system(command); 
     
          //Apply ELAS ALgo xyz
            cv::Mat  matFileDisp =process(leftImgNamePGM.c_str(),rightImgNamePGM.c_str()  , maxdisp, 0);
           


             cout<<"Image saved..."<<count; 
           
             if (matFileDisp.empty()) //check whether the image is loaded or not
             {
		  cout << "Error : Image cannot be loaded..!!" <<  endl;
		  //system("pause"); //wait for a key press 
            }else{ 

                //##### Apply various filters to smoothen the disparity image
                //cv::equalizeHist( matFileDisp, matFileDisp );
               // cv::medianBlur ( matFileDisp, matFileDisp, 5 ); 
               //cv::GaussianBlur( matFileDisp, matFileDisp, Size( 3, 3 ), 0, 0 ); 
                cv::Mat temp;
                cv::bilateralFilter( matFileDisp, temp,7, 2, 0);
                matFileDisp=temp ;
                imshow(DISPARITY_WINDOW, matFileDisp/32.0  );
            }
             cv::waitKey(10);
	     
            // Reproject image into 3d.
            cv::Mat disp8;
           // matFileDisp.convertTo(disp8, CV_8U, 255/(4*16.0));
            //cv::cvtColor(depth,disp8, CV_BGR2GRAY); 
 

	    f= 1351.421 ; 
	    cx=294.182;
	    cy=252.932;
	    cx_prime=326.96;
	    cy_prime=252.932;
	    B= 120.0;
	    doffs=32.778;   

	      f= 851.253 ; 
	     // cx=348.179;
	     // cy=187.267;
	      //cx_prime=325.433;
	      //cy_prime=186.932;
	      B= 120.0;
	      doffs=52.0;  

            cv::Mat Q = (Mat_<double>(4,4) <<  1,  0, 0,  -cx,
                                 	       0, -1, 0,   cy,     //# turn points 180 deg around x-axis,# so that y-axis looks up
                                 	       0,  0, 0,   -f,
                                 	       0,  0, 1/B,  -(cx-cx_prime)/B);

	    cv::Mat xyz ,colors;
            cv::cvtColor(left_image,colors, CV_BGR2RGB); 
            //reprojectImageTo3D(depth  , xyz, Q, true);  
	    reprojectImageTo3D(matFileDisp, xyz, Q, true); 

             //##for anju results
           // string pcdName = std::to_string(count)+"cloud.pcd";
           // saveXYZPCD(pcdName.c_str(), xyz,colors);
           //###end of anju results       

            string cloudNameCV= std::to_string(count)+"cloudCV.ply";
            string cloudName= std::to_string(count)+"cloud.ply";

            //call our method
	    generatePointCloud1(left_image,matFileDisp,cloudName);
            //opencv method to save 
            saveXYZ(cloudNameCV.c_str(), xyz,colors);
            //our method
	    saveXYZ2(cloudName.c_str(), matFileDisp,colors);
	   // ZED camera's default point cloud
            std::string plyname=std::to_string(count)+ "cloudZED";
            sl::writePointCloudAs(zed, sl::POINT_CLOUD_FORMAT::PLY, plyname,  true,  false  ) ;

	    printf("Point cloud saved\n");
	    
        
        // }
          
  }
    delete zed; 
    return  0;
}
