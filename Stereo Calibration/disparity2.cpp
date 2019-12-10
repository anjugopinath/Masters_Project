#include <stdio.h>
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
    
using namespace cv;
double _Q[] = { 1., 0., 0.,  -3.2554532241821289e+002,
                0., -1., 0., -2.4126087760925293e+002,
                0., 0., 0., 4.2440051858596559e+002, 
                0., 0., 1, 0. };

float a[480][640];
float b[480][640];
float c[480][640];

int main2(int argc,char** argv)
{
        IplImage* image1 = cvLoadImage(argv[1],CV_LOAD_IMAGE_GRAYSCALE);
        IplImage* image2 = cvLoadImage(argv[2],CV_LOAD_IMAGE_GRAYSCALE);
        IplImage* result = cvCreateImage(cvSize(image1->width,image1->height),image1->depth,1);
        //CvMat* disp = cvCreateMat(image1->height, image1->width, CV_16S);
        IplImage* disp= cvCreateImage( cvSize(image1->width, image1->height), IPL_DEPTH_16S, 1 );
        CvMat Q;
        cvInitMatHeader(&Q,4,4,CV_32FC1,_Q);
       
        //CvMat* reproj = cvCreateMat(image1->height, image1->width, CV_32FC3 );
        IplImage* depth = cvCreateImage( cvSize(image1->width, image1->height), IPL_DEPTH_32F, 3 );
        CvStereoBMState *BMState = cvCreateStereoBMState();
        assert(BMState != 0);
        BMState->preFilterSize = 41;
        BMState->preFilterCap = 31;
        BMState->SADWindowSize = 41;
        BMState->minDisparity = -64;
        BMState->numberOfDisparities = 128;
        BMState->textureThreshold = 10;
        BMState->uniquenessRatio = 15;
        cvFindStereoCorrespondenceBM(image1,image2,disp,BMState);
        IplImage* real_disparity= cvCreateImage( cvSize(image1->width, image1->height), IPL_DEPTH_8U, 1 );
        cvConvertScale( disp, real_disparity, 1.0/16, 0 );
        //cvWaitKey(-1);
        cvReprojectImageTo3D(real_disparity, depth, &Q );
        ////vector
        IplImage* Iz8U_ = cvCreateImage(cvSize(depth->width,depth->height),IPL_DEPTH_8U,1);

        printf("ok");
        int w = depth->width, h = depth->height;
        float Zmin= -3000.0;
        float Zmax= 0.0;
        float* fptr= (float*)(depth->imageData);
        float fv;
        double scaleit = 255.0 / (Zmax - Zmin);
        unsigned char *cptr = (unsigned char *)(Iz8U_->imageData);
        fptr = (float *)(depth->imageData);
        //LOOP
        for(int y=0; y<h; ++y)
        {
                for(int x=0; x<w; ++x)
                {
                        //*fptr++;
                        fv = *(fptr+2);  // read the depth
                        if((fv >= 0.0)||(fv > Zmax)) // too close
                                *cptr++ = 0;
                        else if(fv < Zmin)    // too far
                                *cptr++ = 255;
                        else {
                                *cptr++ = (unsigned char)( scaleit*( (double)fv - Zmin) );  
                                //printf(".....%uc.....",cptr);
                        }
                        fptr+= 3;

                        /*
                        a[y][x] = *fptr++;
            b[y][x] = *fptr++;
            c[y][x] = *fptr++;*/
// if(y == 478)
//    printf("%f...%f...%f\n",a[y][x],b[y][x],c[y][x]);
                }
        }
        //printf(".....%d.....",count);
        cvNamedWindow("HELLO",1);
        cvShowImage("HELLO",Iz8U_);
        cvWaitKey(-1);
        //cvReleaseImage(disp);
 
        return 0;
}


