#include <iostream>
#include <stdio.h>
#include <time.h>
#include "RoverSerial.h"
#include "ArduinoSerial.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>

#define ALTOIMG 480
#define ANCHOIMG 640
#define MAX_CORNERS 200
#define fpair CvPoint2D32f
#define newfpair cvPoint2D32f
#define finishedDrawing box.width!=0 && box.height!=0 && !drawing_box
    
using namespace std;

RoverSerial robot;
ArduinoSerial aruino;
int pan; //altura de la camara
int tilt; //direccion horizontal de la camara
bool drawing_box = false; //bandera para el rectangulo dibujado por el mouse
bool reset = false; //bandera que indica el reset de un cuadro del flow
CvRect box = cvRect(0,0,0,0); //cuadrado que cerca el objetivo del robot
CvPoint mouse = cvPoint(0,0); //ubicacion del mouse
float round(float d);
void mouseCallback( int event, int x, int y, int flags, void* param);
fpair mean(fpair *set, int size);
fpair variance(fpair *set, fpair mean, int size);
void difference(fpair *A, fpair *B, fpair *C, int size );
void resetFlow(fpair *corner, int *count, IplImage *imgA);
void opticalFlow(IplImage *prev, IplImage *post, IplImage *pyrprev, IplImage *pyrpost, 
                 fpair* cornersA, fpair* cornersB, int *count, 
                char *features_found, float *feature_errors);
void printPoints(fpair *points, int num);                
                

int main(int argc, char** argv)
{
        
        CvCapture *capture = NULL;
        IplImage  *imgARGV = NULL;
        IplImage *imgBRGV = NULL;
        IplImage  *imgA = NULL;
        IplImage *imgB = NULL;
        IplImage *imgC = NULL;
        IplImage *outA = NULL;
        IplImage *gray_outA = NULL;
        IplImage *outB = NULL;
        IplImage *gray_outB = NULL;
        IplImage *finalOut = NULL;

        int corner_count = MAX_CORNERS;
        fpair* cornersA = new fpair [ MAX_CORNERS ];
        fpair* cornersB = new fpair[ MAX_CORNERS ];
        fpair* cornersC = new fpair[ MAX_CORNERS ];

        fpair meanB; 
        fpair meanA;
        fpair meanABprev;
        fpair meanAB;
        fpair varAB;        
        
        int frames = 0;
        int key;

    /* initialize camera */
    capture = cvCaptureFromCAM( 0 );

    /* always check */
    assert(capture );

    /* create a window */
    cvNamedWindow( "OpticalFlow", 1 );
        
    imgARGV = cvQueryFrame( capture );
    imgA = cvCreateImage( cvSize(imgARGV->width, imgARGV->height), IPL_DEPTH_8U, 1 );
    cvCvtColor( imgARGV, imgA, CV_BGR2GRAY );
    char features_found[ MAX_CORNERS ];
    float feature_errors[ MAX_CORNERS ];

    finalOut = cvCreateImage(cvGetSize(imgARGV), 8, 3);

    /* Perform a Gaussian blur */
	outA = cvCreateImage( cvGetSize(imgARGV), IPL_DEPTH_8U, 3 );
	gray_outA = cvCreateImage( cvGetSize(imgARGV), IPL_DEPTH_8U, 1 );
	imgA = cvCreateImage( cvGetSize(imgARGV), IPL_DEPTH_8U, 1 );
	cvSmooth( imgARGV, outA, CV_GAUSSIAN, 11, 11 );
	cvCvtColor(outA , gray_outA, CV_RGB2GRAY);
    /* Perform Canny */
    cvCanny( gray_outA, imgA, 10, 20, 3 );




    /* setup mouse callback */
    cvSetMouseCallback("OpticalFlow", mouseCallback); 
   
    while( key != 'q' ) {
        /* get a frame */
        imgBRGV = cvQueryFrame( capture );
        /* get the image in black and white for the opt flow */
        imgB = cvCreateImage( cvSize(imgBRGV->width, imgBRGV->height), IPL_DEPTH_8U, 1 );
        cvCvtColor( imgBRGV, imgB, CV_BGR2GRAY );

        /* Perform a Gaussian blur */
        outB = cvCreateImage( cvGetSize(imgBRGV), IPL_DEPTH_8U, 3 );
        gray_outB = cvCreateImage( cvGetSize(imgBRGV), IPL_DEPTH_8U, 1 );
        imgB = cvCreateImage( cvGetSize(imgBRGV), IPL_DEPTH_8U, 1 );
        cvSmooth( imgBRGV, outB, CV_GAUSSIAN, 11, 11 );
        cvCvtColor(outB , gray_outB, CV_RGB2GRAY);
        /* Perform Canny */
        cvCanny( gray_outB, imgB, 10, 20, 3 );
 
       
        /* get optical flow if a box is drawn or reset is desired */
        frames++;
        if(frames%40==0 || corner_count<10)
            reset = true;

        if (finishedDrawing){        
        
            /* reset the points and put them inside the box */
            if (reset){
                cvSetImageROI(imgA,box);
                
                /* Reset corner_count */
                corner_count = MAX_CORNERS; 

                /* Get the features for tracking */
                CvSize img_sz = cvGetSize( imgA );
                IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
                IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );

                cvGoodFeaturesToTrack(imgA, eig_image, tmp_image, cornersA, &corner_count, 0.01, 0.01, NULL);
                cvGoodFeaturesToTrack( imgA, eig_image, tmp_image, cornersA, &corner_count,
                        0.05, 5.0, 0, 3, 0, 0.04 );
                cvResetImageROI(imgA);

                /* Get actual corner values*/
                int i;
                float xfloat = (float)box.x;
                float yfloat = (float)box.y;

                for (i=0; i<corner_count; i++) {
                    cornersA[i].x += xfloat;
                    cornersA[i].y += yfloat;   
                }
                
               
            }
            

            int win_size = 15;
            cvFindCornerSubPix( imgA, cornersA, corner_count, cvSize( win_size, win_size ),
                    cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

            /* Call Lucas Kanade algorithm */

            CvSize pyr_sz = cvSize( imgA->width+8, imgB->height/3 );

            IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
            IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
            cvCalcOpticalFlowPyrLK( imgA, imgB, pyrA, pyrB, cornersA, cornersB, corner_count, 
                    cvSize( win_size, win_size ), 5, features_found, feature_errors,
                     cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );

            /* Re-adjust rectangle size and position  */

            /* Calculate B's variance agaist A*/   
            meanA = newfpair(0,0);
            meanB = newfpair(0,0);
            meanAB = newfpair(0,0);
            varAB = newfpair(0,0);

            int j = 0;
            /* Calculate mean of A and B*/
            meanA = mean(cornersA, corner_count);
            meanB = mean(cornersB, corner_count);

            /* Calculate difference between A and B*/
            difference(cornersA, cornersB, cornersC, corner_count);
            
            /* Calculate mean of the difference*/
            meanAB = mean(cornersC, corner_count);
             
            /* Calculate variance */
            varAB = variance(cornersC, meanAB, corner_count);
            
            /* Change from Gray to RGV for output image*/ 
            imgC = cvCloneImage(imgB );
            cvCvtColor(imgC, finalOut, CV_GRAY2BGR); 
            
            if (reset){
                reset = false;
            }else{   
                int numElim = 0;
                /* Eliminate points that are too far away from the deviation */
                varAB= newfpair(sqrt(varAB.x)*(float)2, sqrt(varAB.y)*(float)2);
                for(j=0; j<corner_count; j++){
                        if (fabs(cornersC[j].x) > (varAB.x*4) || fabs(cornersC[j].y) > (varAB.y*4)) {
                        cvCircle(finalOut,cvPoint( cvRound( cornersB[j].x ), cvRound( cornersB[j].y )), 4, CV_RGB(255,255,0), -1);
                        numElim++;
                        corner_count--;
                        cornersA[j-numElim] = newfpair(cornersA[j].x, cornersA[j].y);              
                        cornersB[j-numElim] = newfpair(cornersB[j].x, cornersB[j].y);              
                        cornersC[j-numElim] = newfpair(cornersC[j].x, cornersC[j].y);              
                    
                    
                    }
                }    
                if (numElim != 0){
                    /* Recalculate means and variances */
                    meanA = mean(cornersA, corner_count);
                    meanB = mean(cornersB, corner_count);
                    meanAB = mean(cornersC, corner_count);
                    varAB = variance(cornersC, meanAB, corner_count);
                }
                 
                
                /* Re-adjust box-size */
                fpair cambio = newfpair(0,0);
                fpair differenceA = newfpair(0,0);
                fpair differenceB = newfpair(0,0);
                for(j=0;j<corner_count;j++){
                    differenceA.x += fabs(meanA.x-cornersA[j].x);  
                    differenceA.y += fabs(meanA.y-cornersA[j].y);
                    differenceB.x += fabs(meanB.x-cornersB[j].x);
                    differenceB.y += fabs(meanB.y-cornersB[j].y);
                }
                differenceA.x /= corner_count;
                differenceA.y /= corner_count;
                differenceB.x /= corner_count;
                differenceB.y /= corner_count;
                cambio = newfpair(differenceB.x/differenceA.x, differenceB.y/differenceA.y);
                box.x = round(meanB.x - (meanA.x-((float)box.x))*cambio.x);
                box.y = round(meanB.y - (meanA.y-((float)box.y))*cambio.y); 
                box.width = round(((float)box.width)*cambio.x);
                box.height = round(((float)box.height)*cambio.y); 
            }
        
            /* Draw flow points */
            int i;
            for( i=0; i<corner_count; i++ ){
                    CvPoint p0 = cvPoint( cvRound( cornersA[i].x ), cvRound( cornersA[i].y ) );
                    CvPoint p1 = cvPoint( cvRound( cornersB[i].x ), cvRound( cornersB[i].y ) );
                    cvLine( finalOut, p0, p1, CV_RGB(255,0,0), 2 );

                    /* Put new cornes in previous corners */
                    cornersA[i].x =  cornersB[i].x;
                    cornersA[i].y =  cornersB[i].y;
            }
            
            
        }
        else{
            imgC = cvCloneImage(imgB);
            cvCvtColor(imgC, finalOut, CV_GRAY2BGR); 
        }

        /* Draw clicked box, mouse lines */
        cvLine(finalOut, cvPoint(mouse.x,0), cvPoint(mouse.x,ALTOIMG), CV_RGB( 255, 0, 0 ),1,8);
        cvLine(finalOut, cvPoint(0,mouse.y), cvPoint(ANCHOIMG,mouse.y), CV_RGB( 255, 0, 0 ),1,8);
        if (drawing_box) {
            cvRectangle( finalOut,
            cvPoint( box.x, box.y ),
            cvPoint( box.x + box.width, box.y + box.height ),
            CV_RGB( 255, 0, 0 ), 1, 8, 0 ); 
        } else if (box.width !=0 && box.height !=0) {
            cvRectangle( finalOut,
            cvPoint( box.x, box.y ),
            cvPoint( box.x + box.width, box.y + box.height ),
            CV_RGB( 0, 255, 0 ), 1, 8, 0 );
        }
        
        /* show image and free variables */
        cvShowImage( "OpticalFlow", finalOut );
        cvReleaseImage(&imgC);  
        key = cvWaitKey( 10 );
        cvReleaseImage(&imgA);
        imgA = cvCloneImage(imgB);
        cvReleaseImage(&imgB);
        cvReleaseImage(&outB);
        cvReleaseImage(&gray_outB);
        meanABprev = meanAB;
}
        
    cvReleaseCapture( &capture );
//    cvReleaseImage( &imgARGV);
//    cvReleaseImage( &imgBRGV);
//    cvReleaseImage( &imgA);
//    cvReleaseImage( &imgB);
//    cvReleaseImage( &imgC);
//    cvReleaseImage( &outA);
//    cvReleaseImage( &gray_outA);
//    cvReleaseImage( &outB);
//    cvReleaseImage( &gray_outB);
//    cvReleaseImage( &finalOut);

    cvDestroyWindow( "OpticalFlow" );
    return(0);  

}

float round(float d)
{
  return floor(d + 0.5);
}

void mouseCallback( int event, int x, int y, int flags, void* param){   
        switch (event){    
            case CV_EVENT_MOUSEMOVE: 
                if (drawing_box){
                    box.width = x-box.x;
                    box.height = y-box.y;
                }  
                mouse = cvPoint(x,y);
                break;

            case CV_EVENT_LBUTTONDOWN:
                if (!drawing_box){
                    box.x = x;
                    box.y = y; 
                } else {
                    box.width = x-box.x;
                    box.height = y-box.y;

                    if (box.width < 0){
                        box.x += box.width;
                        box.width *= -1;
                    }
                    if (box.height < 0){
                        box.y += box.height;
                        box.height *= -1;
                    }  
                    reset = true;
                }
                drawing_box = !drawing_box;
                break;
        }
}

fpair mean(fpair *set, int size){
    fpair mean;
    mean.x = 0;
    mean.y = 0;
    int j = 0;
    for(j=0; j<size; j++){
        mean.x += set[j].x;
        mean.y += set[j].y;
    }
    mean.x /= (float)size;
    mean.y /= (float)size;
    return mean;
}

fpair variance(fpair *set, fpair mean, int size){
    fpair var;
    var.x = 0;
    var.y = 0;
    int j = 0;
    for(j=0; j<size; j++){
        var.x += pow( (set[j].x - mean.x), 2); 
        var.y += pow( (set[j].y - mean.y), 2);
    } 
    var.x /= (float)size;
    var.y /= (float)size;
    return var;
}

void difference(fpair *A, fpair *B, fpair *C, int size ){
    int j = 0;
     for(j=0; j<size; j++){
        C[j].x = B[j].x - A[j].x;
        C[j].y = B[j].y - A[j].y;
    }
}

void resetFlow(fpair *corner, int *count, IplImage *imgA){            
 
}

void opticalFlow(IplImage *prev, IplImage *post, IplImage *pyrprev, IplImage *pyrpost, 
     fpair* cornersA, fpair* cornersB, int *count, 
     char *features_found, float *feature_errors){
}

void printPoints(fpair *points, int num){
    int i;
    printf("[");
    for (i=0;i<num;i++)
        printf("(%f,%f) ", points[i].x, points[i].y);
    printf("]\n");
}
