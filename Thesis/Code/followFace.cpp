#include <cstdlib>
#include <iostream>
#include "RoverSerial.h"
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

//Velocidad maxima 10,minima=Const

CvHaarClassifierCascade *cascade;
CvMemStorage            *storage; 
RoverSerial robot;
CvRect detectFaces( IplImage *img );
// en 70 la cara esta chiquita


int main( int argc, char** argv )
{
    CvCapture *capture = NULL;
    IplImage  *frame = NULL;
    IplImage  *result = NULL;
    int       key;
    char      *filename = (char *)"/opt/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml";
    RoverSerial robot;
	CvRect punto = cvRect(0,0,0,0);
	int alto; //del centro de la imagen
	int ancho; //del centro de la imagen
	int altoPunto;
	int anchoPunto;
	int move = 127; //velocidad de avance
	int moveTurn; //velocidad del giro
	int pan = 127; //altura de la camara
	int tilt = 127; //direccion horizontal de la camara
	int vel = 2 ; //velocidad de la camara
	const int turn = 80; //constante de giro  
	int offset = 20;
	int camOffset = 20;
	int offsetReached = 0; //entero que comprueba si se centro la imagen
	int directionReached = 0;
	
	
    /* load the classifier
       note that I put the file in the same directory with
       this code */
    cascade = ( CvHaarClassifierCascade* )cvLoad( filename, 0, 0, 0 );

    /* setup memory buffer; needed by the face detector */
    storage = cvCreateMemStorage( 0 );

    /* initialize camera */
    capture = cvCaptureFromCAM( 0 );

    /* always check */
    assert( cascade && storage && capture );
 
    /* create a window */
    cvNamedWindow( "video", 1 );
	
	/* open robot port */ 
    robot.openPORT();
	robot.setDATA(127,127,127,127,127,127,false);
	robot.sendDATA();
	int i=0;
    while( key != 'q' ) {
		i++;
		
        /* get a frame */
        frame = cvQueryFrame( capture );

        /* always check */
        if( !frame ) break;

        /* clone and 'fix' frame */
		result = cvCloneImage(frame);
        cvFlip( result, result, 1 );
		
        /* detect faces and display video */
        punto = detectFaces( result );
        
        /* if faces were detected */
        if(punto.width!=0 && punto.height!=0)
        {
          // printf("tamano de la cara. Alto: %d, Ancho: %d\n", punto.width, punto.height);
		   /* move camera towards face*/
			alto = (result->height)/2;
			ancho = (result->width)/2;
			anchoPunto = punto.x + (punto.width/2);
			altoPunto = punto.y + (punto.height/2);
			moveTurn = 127;
		

			/*Si la cara esta a la izquerda del centro
			 cuidandose del overflow */
			if ((ancho - offset)  > anchoPunto){ 
				tilt += vel*(1 - (tilt/(254-vel)));
				offsetReached = 0;
				}	
			else if ((ancho + offset)  < anchoPunto){
				tilt -= vel*((253 - vel + tilt)/(254-vel));
				offsetReached = 0;
			    }
		    else
				offsetReached++;
		
			/*Si la cara esta arriba del centro
			 cuidandose del overflow*/
			if ((alto - offset)  > altoPunto)
				pan += vel*(1 - (pan/(254-vel)));
			else if ((alto + offset)  < altoPunto) 
				pan -= vel*((253 - vel + pan)/(254-vel));
		    
			/* Si ya centre la cara */    
			if (offsetReached > 4) {
					if (tilt < (127 - camOffset)){
						moveTurn -= turn;
						directionReached = 0;
					}
				    else if (tilt > (127 + camOffset)){
						moveTurn += turn;
						directionReached = 0;
					}
					else {
						moveTurn = 127;
						directionReached++;
					}
			}
			
			/* Si ya centre el robot */
		    if (directionReached > 4 && punto.width < 100)
				move = 150;
		    else
				move = 127;
			
			
		    robot.setDATA(move,moveTurn,127,127,pan,tilt,false);
			robot.sendDATA();	
		 
        } else {
	        robot.setDATA(127,127,127,127,pan,tilt,false);
			robot.sendDATA();
        }
	      
	       /* quit if user press 'q' */
	        key = cvWaitKey( 10 );
	}
	
    
	/* close port */
	robot.setDATA(127,127,127,127,127,127,false);
	robot.sendDATA();
	robot.closePORT();
	
    /* free memory */
    cvReleaseCapture( &capture );
    cvDestroyWindow( "video" );
    cvReleaseHaarClassifierCascade( &cascade );
    cvReleaseMemStorage( &storage );
     return 0;
  
}

CvRect detectFaces( IplImage *img )

{

    int i;
	CvRect datosCara = cvRect(0,0,0,0); 
	
    /* detect faces */
    CvSeq *faces = cvHaarDetectObjects(
            img,
            cascade,
            storage,
            1.1,
            3,
            0 ,
            cvSize( 40, 40 ) );

    /* for each face found, draw a red box */
    for( i = 0 ; i < ( faces ? faces->total : 0 ) ; i++ ) {
        CvRect *r = ( CvRect* )cvGetSeqElem( faces, i );
        cvRectangle( img,
                     cvPoint( r->x, r->y ),
                     cvPoint( r->x + r->width, r->y + r->height ),
                     CV_RGB( 255, 0, 0 ), 1, 8, 0 );
        
		datosCara = *r;	break;	
    }
 
    /* display video */
    cvShowImage( "video", img );
	return datosCara;
} 


