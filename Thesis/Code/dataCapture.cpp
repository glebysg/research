#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include "RoverSerial.h"
#include "ArduinoSerial.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>

#define ALTOIMG 480
#define ANCHOIMG 640
#define MAX_CORNERS 30
#define fpair CvPoint2D32f
#define newfpair cvPoint2D32f
#define finishedDrawing box.width!=0 && box.height!=0 && !drawing_box
#define varRange 2.5
#define outOfVariance fabs(cornersC[j].x - meanAB.x)> (varAB.x*varRange) ||  fabs(cornersC[j].y-meanAB.y) > (varAB.y*varRange)

    using namespace std;

    /*** VARIABLES ***/
    bool drawing_box = false; //bandera para el rectangulo dibujado por el mouse
    bool reset = false; //bandera que indica el reset de un cuadro del flow
    CvRect box = cvRect(0,0,0,0); //cuadrado que cerca el objetivo del robot
    float boxP = 0; //box's proportion
    CvPoint mouse = cvPoint(0,0); //ubicacion del mouse

    /*** FUNCIONES ***/
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
    int scale(int x,int min, int max, int a, int b);

    int main(int argc, char** argv)
    {
            
        CvCapture *capture = NULL;
        IplImage  *imgARGV = NULL;
        IplImage *imgBRGV = NULL;
        IplImage  *imgA = NULL;
        IplImage *imgB = NULL;
        IplImage *imgC = NULL;
        IplImage *out = NULL;
        IplImage *gray_out = NULL;
        IplImage *finalOut = NULL;
        IplImage *imgAc = cvCreateImage(cvSize(45,95),8,1);
        IplImage *imgAG = cvCreateImage(cvSize(45,95),8,1);
               
        int corner_count = MAX_CORNERS;
        fpair* cornersA = new fpair [ MAX_CORNERS ];
        fpair* cornersB = new fpair[ MAX_CORNERS ];
        fpair* cornersC = new fpair[ MAX_CORNERS ];
        
        fpair meanB; 
        fpair meanA;
        fpair meanAB;
        fpair varAB;        
        
        int frames = 0;
        int key;

        CvRect punto = cvRect(0,0,0,0);

        /*for the photo names*/
        char directorio [80];
        char nombre [100];
        char *str;
        char comando [200];
        char* tok;
        char buffer[128];
        string result;
        int indice = 0;
        char nombreN [200];

        /*initialize the strings*/
        strcpy(directorio, argv[1]);
        strcpy(nombre, argv[2]);
        int lnombre = strlen(nombre);

        /* Read the directory */
        sprintf(comando, "ls %s | sort -k1.%dn", directorio,lnombre);
        FILE* pipe = popen(comando, "r");
        while(!feof(pipe)) {
            if(fgets(buffer, 128, pipe) != NULL)
                result += buffer;
            }
        pclose(pipe);
        //Si hay imagenes, comenzar desde la  ultima
        if ((int)result.length()>0){
            str = new char [result.size()+1];
            strcpy (str, result.c_str());
            tok=strtok (str,"\n");    
            tok[ (int)(strlen(tok)) - 4 ] = '\0';
            indice = atoi(tok+lnombre) + 1; 
        }

      
        /* for the interference */
        float proportion = 0;

        /* initialize camera */
        capture = cvCaptureFromCAM( 0 );

        /* always check */
        assert(capture);

        /* create a window */
        cvNamedWindow( "OpticalFlow", 1 );
    
    /* Initialize images */    
    imgARGV = cvQueryFrame( capture );
    imgA = cvCreateImage( cvSize(imgARGV->width, imgARGV->height), IPL_DEPTH_8U, 1 );
    imgAG = cvCreateImage( cvSize(imgARGV->width, imgARGV->height), IPL_DEPTH_8U, 1 );
    imgB = cvCreateImage( cvSize(imgARGV->width, imgARGV->height), IPL_DEPTH_8U, 1 );
    imgC = cvCreateImage( cvSize(imgARGV->width, imgARGV->height), IPL_DEPTH_8U, 1 );
    CvSize img_sz = cvGetSize( imgA );
    IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
    IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
    CvSize pyr_sz = cvSize( imgA->width+8, imgB->height/3 );
    IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
    IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
    cvCvtColor( imgARGV, imgA, CV_BGR2GRAY );
    cvCvtColor( imgARGV, imgAG, CV_BGR2GRAY );
    char features_found[ MAX_CORNERS ];
    float feature_errors[ MAX_CORNERS ];

    finalOut = cvCreateImage(cvGetSize(imgARGV), 8, 3);


    /* setup mouse callback */
    cvSetMouseCallback("OpticalFlow", mouseCallback); 
       
    while( key != 'q' ) {

        /* get a frame */
        imgBRGV = cvQueryFrame( capture );
        /* get the image in black and white for the opt flow */
        cvCvtColor( imgBRGV, imgB, CV_BGR2GRAY );
       
        /* get optical flow if a box is drawn or reset is desired */
        if(corner_count<3)
            reset = true;

        if (finishedDrawing){         
            /* Capture an object's image every 60 frames*/
               if (frames%60 == 0) {   
                   cvSetImageROI(imgAG,box);
                   cvResize(imgAG, imgAc);
                   sprintf(nombreN,"%s/%s%d.jpg",directorio,nombre,indice);
                   indice++;
                   cvSaveImage(nombreN, imgAc);
                   cvResetImageROI(imgAG);
               //cvReleaseImage(&imgAc);  
               } 
            /* reset the points and put them inside the box */
            if (reset){
                cvSetImageROI(imgA,box);
                cvSetImageROI(eig_image,box);
                cvSetImageROI(tmp_image,box);
                proportion = (float)box.width/(float)box.height;

                /* Reset corner_count */
                corner_count = MAX_CORNERS; 

                /* Get the features for tracking */

                cvGoodFeaturesToTrack(imgA, eig_image, tmp_image, cornersA, &corner_count, 0.01, 0.01, NULL);
                cvGoodFeaturesToTrack( imgA, eig_image, tmp_image, cornersA, &corner_count,
                        0.05, 5.0, 0, 3, 0, 0.04 );
                cvResetImageROI(imgA);
                cvResetImageROI(eig_image);
                cvResetImageROI(tmp_image);
                    
                /* Get actual corner values and make copies*/
                int i;
                float xfloat = (float)box.x;
                float yfloat = (float)box.y;

                for (i=0; i<corner_count; i++) {
                    cornersA[i].x += xfloat;
                    cornersA[i].y += yfloat;   
                   // cornersI[i].x = cornersA[i].x;
                   // cornersI[i].y = cornersA[i].y;
                }
            }
            

            int win_size = 15;
            cvFindCornerSubPix( imgA, cornersA, corner_count, cvSize( win_size, win_size ),
                    cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

            /* Call Lucas Kanade algorithm */

            cvCalcOpticalFlowPyrLK( imgA, imgB, pyrA, pyrB, cornersA, cornersB, corner_count, 
                    cvSize( win_size, win_size ), 5, features_found, feature_errors,
                     cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );

            /* Re-adjust rectangle size and position  */

            varAB = newfpair(0,0);
            int j = 0;

            /* Calculate difference between A and B*/
            difference(cornersA, cornersB, cornersC, corner_count);
            
            /* Calculate means of A and B and the difference*/
            meanA = mean(cornersA, corner_count);
            meanB = mean(cornersB, corner_count);
            meanAB = mean(cornersC, corner_count);
             
            /* Calculate variance */
            varAB = variance(cornersC, meanAB, corner_count);
            
            /* Change from Gray to RGV for output image*/ 
            cvCopy(imgB, imgC,NULL);
            cvCvtColor(imgC, finalOut, CV_GRAY2BGR); 
            if (reset){
                reset = false;
            }else{   
                int numElim = 0;
                /* Eliminate points that are too far away from the deviation */
                varAB= newfpair(sqrt(varAB.x)*(float)2, sqrt(varAB.y)*(float)2);
                int newcc = corner_count;
                for(j=0; j<newcc; j++){
                    if (outOfVariance) {
                        //printf(" (%f,%f) ", cornersC[j].x,cornersC[j].y);    
                        cvCircle(finalOut,cvPoint( cvRound( cornersB[j].x ), cvRound( cornersB[j].y )), 4, CV_RGB(255,255,0), -1);
                        numElim++;
                        if (j+numElim<corner_count){
                             cornersA[j] = newfpair(cornersA[j+numElim].x, cornersA[j+numElim].y);              
                             cornersB[j] = newfpair(cornersB[j+numElim].x, cornersB[j+numElim].y);              
                             cornersC[j] = newfpair(cornersC[j+numElim].x, cornersC[j+numElim].y);              
                            newcc--;
                            j--;    
                        }
                    } else if (numElim!=0){
                         if (j+numElim+1<corner_count){
                             cornersA[j+1] = newfpair(cornersA[j+numElim+1].x, cornersA[j+numElim+1].y);              
                             cornersB[j+1] = newfpair(cornersB[j+numElim+1].x, cornersB[j+numElim+1].y);              
                             cornersC[j+1] = newfpair(cornersC[j+numElim+1].x, cornersC[j+numElim+1].y);              
                        }
                    }
                }    
                corner_count = newcc;
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
                /*detect interference*/
                float newProportion = (float)box.width/(float)box.height;
                float interference = newProportion/proportion; 
                if ( 0.75>interference || interference>1.25 ){
                    box.width = 0;
                    box.height = 0;
                }
            }
       

            /* Draw flow points */
            int i;
			
            for( i=0; i<corner_count; i++ ){
                    CvPoint p0 = cvPoint( cvRound( cornersA[i].x ), cvRound( cornersA[i].y ) );
                    CvPoint p1 = cvPoint( cvRound( cornersB[i].x ), cvRound( cornersB[i].y ) );
                    cvLine( finalOut, p0, p1, CV_RGB(255,0,0), 2 );

                    /* Put new cornes in previous corners if box didnt lose proportion*/
                        cornersA[i].x =  cornersB[i].x;
                        cornersA[i].y =  cornersB[i].y;
            }
		}
        else{
            cvCopy(imgB,imgC,NULL);
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
        key = cvWaitKey( 10 );
        cvCopy(imgB,imgA,NULL);
        cvReleaseImage(&out);
        cvReleaseImage(&gray_out);
        cvCvtColor( imgBRGV, imgAG, CV_BGR2GRAY );
        frames++;  
}

       
    cvReleaseCapture( &capture );
//    cvReleaseImage( &imgARGV);
//    cvReleaseImage( &imgBRGV);
//    cvReleaseImage( &imgA);
//    cvReleaseImage( &imgB);
//    cvReleaseImage( &imgCn;
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
                    boxP = (float)box.width/(float)box.height;
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


int scale(int x, int min, int max, int a, int b){
      float numerador =(float)((b-a)*(x - min));
        float denominador = float(max -min);
          return round((numerador/denominador +(float)a));
}
/*****CODIGO PORSIA******/
//        /* Perform a Gaussian blur */
//        out = cvCreateImage( cvGetSize(imgBRGV), IPL_DEPTH_8U, 3 );
//        gray_out = cvCreateImage( cvGetSize(imgBRGV), IPL_DEPTH_8U, 1 );
//        imgB = cvCreateImage( cvGetSize(imgBRGV), IPL_DEPTH_8U, 1 );
//        cvSmooth( imgB, imgB, CV_GAUSSIAN, 11, 11 );
//        cvCvtColor(out , gray_out, CV_RGB2GRAY);
//        /* Perform Canny */
//        cvCanny( gray_out, imgB, 10, 20, 3 );

