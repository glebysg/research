
/**
 * para compilar > g++ handgame.cpp -o p -m32 -F/Library/Frameworks/ -framework OpenCV -I"./sound/include/"  -L"/usr/lib" -lirrklang
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <string>
#include <list>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>



struct paso{
	int pos;
	int  tipo;
	int frame;
};

CvHaarClassifierCascade *cascade;
CvMemStorage            *storage;
bool redb;
bool blueb;
bool greenb;
bool redb2;
bool blueb2;
bool greenb2;
int contadorBlue;
int contadorGreen; 
int contadorRed; 
int contadorArriba;
int contadorMedio; 
int contadorAbajo;
int contadorBlue2;
int contadorGreen2; 
int contadorRed2;
int contadorArriba2;
int contadorMedio2; 
int contadorAbajo2;
int bienn, maln;
int pasos[200]; //cada paso con su pixel de posicion
clock_t initGame;
clock_t gameTime;
clock_t temp;
int score1, score2;
int frameN;
std::list<paso> pasos1;
std::list<paso> pasos2;


void detectHands( IplImage *img, IplImage *blue, IplImage *green, IplImage *red, bool caso);

void OverlayImage(IplImage *src, IplImage *overlay, CvPoint location, CvScalar S, CvScalar D);

void CheckAndDraw(IplImage *src,CvRect *rect, IplImage *blue, IplImage *green, IplImage *red, bool caso);

void ChekAndScore(IplImage *src, std::list<int>::iterator it1, std::list<int>::iterator it2);

std::string itoa(int value, unsigned int base);

void GenerateScoreMessage(IplImage *src, int s1, int s2);

void drawAndAdvance(IplImage *src,IplImage *good, IplImage *bad, IplImage *rstep1,  IplImage *rstep2);

void drawBien(IplImage *src, IplImage *bien, int tambor, bool caso); 

void drawMal(IplImage *src, IplImage *mal , int tambor, bool caso); 

int main( int argc, char** argv )
{
	

    contadorBlue = 0;
    contadorGreen = 0;
    contadorRed = 0;

    CvCapture *capture = NULL;
    IplImage  *frame = NULL;
    IplImage  *result = NULL;
    int       key;
    char      *filename = (char*)"aGest.xml";


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
    
    /* open and rezise images to be overlayed */
    IplImage *drumblue = cvLoadImage("./Drums/DrumBlue.png");
    IplImage *drumgreen = cvLoadImage("./Drums/DrumGreen.png");
    IplImage *drumred = cvLoadImage("./Drums/DrumRed.png");
    IplImage *lineblue = cvLoadImage("./Drums/BlueLine.png");
    IplImage *linegreen = cvLoadImage("./Drums/GreenLine.png");
    IplImage *linered = cvLoadImage("./Drums/RedLine.png");
    IplImage *step1 = cvLoadImage("./Drums/Step.png");
    IplImage *step2 = cvLoadImage("./Drums/Step2.png");
    IplImage *arrow1 = cvLoadImage("./Drums/Arrow1.png");
    IplImage *arrow2 = cvLoadImage("./Drums/Arrow2.png");
    IplImage *bien = cvLoadImage("./Drums/Bien.png");
    IplImage *buu = cvLoadImage("./Drums/Buu.png");


    IplImage *rdrumblue = cvCreateImage(cvSize(110,95),drumblue->depth, drumblue->nChannels);
    IplImage *rdrumgreen = cvCreateImage(cvSize(110,95),drumgreen->depth, drumgreen->nChannels);
    IplImage *rdrumred = cvCreateImage(cvSize(110,95),drumred->depth, drumred->nChannels);
    IplImage *rdrumblue2 = cvCreateImage(cvSize(110,95),drumblue->depth, drumblue->nChannels);
    IplImage *rdrumgreen2 = cvCreateImage(cvSize(110,95),drumgreen->depth, drumgreen->nChannels);
    IplImage *rdrumred2 = cvCreateImage(cvSize(110,95),drumred->depth, drumred->nChannels);
    IplImage *rlineblue = cvCreateImage(cvSize(230,80),lineblue->depth, lineblue->nChannels);
    IplImage *rlinegreen = cvCreateImage(cvSize(230,80),linegreen->depth, linegreen->nChannels);
    IplImage *rlinered = cvCreateImage(cvSize(230,80),linered->depth, linered->nChannels);
    IplImage *rlineblue2 = cvCreateImage(cvSize(230,80),lineblue->depth, lineblue->nChannels);
    IplImage *rlinegreen2 = cvCreateImage(cvSize(230,80),linegreen->depth, linegreen->nChannels);
    IplImage *rlinered2 = cvCreateImage(cvSize(230,80),linered->depth, linered->nChannels);
    IplImage *rstep1 = cvCreateImage(cvSize(100,100),step1->depth, step1->nChannels);
    IplImage *rstep2 = cvCreateImage(cvSize(100,100),step2->depth, step2->nChannels);
    IplImage *rarrow1 = cvCreateImage(cvSize(110,70),arrow1->depth, arrow1->nChannels);
    IplImage *rarrow2 = cvCreateImage(cvSize(110,70),arrow2->depth, arrow2->nChannels);
    IplImage *rbien = cvCreateImage(cvSize(60,25),bien->depth, bien->nChannels);
    IplImage *rbuu = cvCreateImage(cvSize(60,25),buu->depth, buu->nChannels);
    

    cvResize(drumblue, rdrumblue);
    cvResize(drumgreen, rdrumgreen);
    cvResize(drumred, rdrumred);
    cvResize(drumblue, rdrumblue2);
    cvResize(drumgreen, rdrumgreen2);
    cvResize(drumred, rdrumred2);
    cvResize(lineblue, rlineblue);
    cvResize(linegreen, rlinegreen);
    cvResize(linered, rlinered);
    cvResize(lineblue, rlineblue2);
    cvResize(linegreen, rlinegreen2);
    cvResize(linered, rlinered2);
    cvResize(step1, rstep1);
    cvResize(step2, rstep2);
    cvResize(arrow1, rarrow1);
    cvResize(arrow2, rarrow2);
    cvResize(bien, rbien);
    cvResize(buu, rbuu);

    cvFlip(rdrumblue2, rdrumblue2,1);
    cvFlip(rdrumgreen2, rdrumgreen2,1);
    cvFlip(rdrumred2, rdrumred2,1);
    cvFlip(rlineblue2, rlineblue2,1);
    cvFlip(rlinegreen2, rlinegreen2,1);
    cvFlip(rlinered2, rlinered2,1);

    /* release memory */
    cvReleaseImage( &drumblue);
    cvReleaseImage( &drumgreen);
    cvReleaseImage( &drumred);
    cvReleaseImage( &lineblue);
    cvReleaseImage( &linegreen);
    cvReleaseImage( &linered );
    cvReleaseImage( &step1 );
    cvReleaseImage( &step2 );
    cvReleaseImage( &arrow1 );
    cvReleaseImage( &arrow2 );
    cvReleaseImage( &bien);
    cvReleaseImage( &buu);

 
    /* create a window */
    cvNamedWindow( "video", 1 );
    
    /* set time and frame variables*/
    initGame = clock ();
    frameN = 0;

    /* set scores*/
    score1 = 0;
    score2 = 0;
    redb = false;
    greenb = false;
    blueb = false;
    redb2 = false;
    greenb2 = false;
    blueb2 = false;
    bienn =0;
    maln =0;

std::list<int> lista;
lista.push_front(1);
lista.push_front(2);
lista.push_front(3);
lista.push_front(4);
lista.push_front(5); 


    while( key != 'q' ) {

        /* get a frame */
        //frame: 640,480
        frame = cvQueryFrame( capture );

        /* always check */
        if( !frame ) break;

        /* clone and 'fix' frame */
        cvFlip( frame, frame, 1 );

	GenerateScoreMessage(frame,score1,score2);	
		
        /* detect Hands and draw boxes */
        detectHands( frame, rlineblue2, rlinegreen2, rlinered2, false );
	detectHands( frame, rlineblue, rlinegreen, rlinered, true);

  
	/* overlay the game play buttons */
	
	cvLine(frame, cvPoint(320,0), cvPoint(320,480), cvScalar(255,255,0), 2);
	
        OverlayImage(frame,rdrumblue,cvPoint(0,240),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
        OverlayImage(frame,rdrumgreen,cvPoint(0,315),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
        OverlayImage(frame,rdrumred,cvPoint(0,390),cvScalar(1,1,1,1),cvScalar(1,1,1,1));

        OverlayImage(frame,rdrumblue2,cvPoint(530, 15),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
        OverlayImage(frame,rdrumgreen2,cvPoint(530,90),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
        OverlayImage(frame,rdrumred2,cvPoint(530,165),cvScalar(1,1,1,1),cvScalar(1,1,1,1));

 	OverlayImage(frame,rarrow1,cvPoint(0, 23),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
	OverlayImage(frame,rarrow1,cvPoint(0,98),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
	OverlayImage(frame,rarrow1,cvPoint(0,173),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
	
	OverlayImage(frame,rarrow2,cvPoint(530,248),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
        OverlayImage(frame,rarrow2,cvPoint(530,323),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
        OverlayImage(frame,rarrow2,cvPoint(530,398),cvScalar(1,1,1,1),cvScalar(1,1,1,1));

	drawAndAdvance(frame,rbien, rbuu, rstep1, rstep2 );

//        OverlayImage(frame,rstep1,cvPoint(200,330),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
//        OverlayImage(frame,rstep2,cvPoint(400,330),cvScalar(1,1,1,1),cvScalar(1,1,1,1));

//        OverlayImage(frame,rbien,cvPoint(200,200),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
//        OverlayImage(frame,rbuu,cvPoint(400,200),cvScalar(1,1,1,1),cvScalar(1,1,1,1));

	
       /* display video */
        cvShowImage( "video", frame );

        /* quit if user press 'q' */
        key = cvWaitKey( 10 );
	
	frameN++;
    }
    
	
    /* free memory */
    cvReleaseCapture( &capture );
    cvDestroyWindow( "video" );
    cvReleaseHaarClassifierCascade( &cascade );
    cvReleaseMemStorage( &storage );
    return 0;

}

void detectHands( IplImage *img, IplImage *blue, IplImage *green, IplImage *red, bool caso)
{

    int i;

    /* sets the Region of Interest
   Note that the rectangle area has to be __INSIDE__ the image */
   if (caso) 
        cvSetImageROI(img, cvRect(0,0,(img->width)/3, (img->height)));
   else
        cvSetImageROI(img, cvRect(((img->width)*2)/3, 0 , (img->width), (img->height)));



    /* detect hands */
    CvSeq *hands = cvHaarDetectObjects(
            img,
            cascade,
            storage,
            1.1,
            3,
            0 /*CV_HAAR_DO_CANNY_PRUNNING*/
    );

    /* reset the Region of Interest */
    cvResetImageROI(img);


    /* for each hand found, draw a red box */
    for( i = 0 ; i < hands->total ; i++ ) {
        CvRect *r = ( CvRect* )cvGetSeqElem( hands, i );
        if (!caso){
		r->x += ((img->width)*2)/3;
        }
        cvRectangle( img,
                     cvPoint( r->x , r->y ),
                     cvPoint( r->x + r->width, r->y + r->height ),
                     CV_RGB( 255, 0, 0 ), 1, 8, 0 );
        
	CheckAndDraw(img,r,blue,green,red,caso);			
    }


} 

void OverlayImage(IplImage *src, IplImage *overlay, CvPoint location, CvScalar S, CvScalar D)
{
	for(int x=0; x<(overlay->width-10);x++)
    {
        if(x+location.x>=src->width) continue;
        for(int y=0;y<(overlay->height-10);y++)
        {
            if(y+location.y>=src->height) continue;
				CvScalar source = cvGet2D(src, y+location.y, x+location.x);
				CvScalar over = cvGet2D(overlay, y, x);
				CvScalar merged;
				    for(int i=0;i<4;i++)
				        merged.val[i] = (S.val[i]*source.val[i]+D.val[i]*over.val[i]);
					cvSet2D(src, y+location.y, x+location.x, merged);
		}
	}
}


void CheckAndDraw(IplImage *img, CvRect *rect, IplImage *blue, IplImage *green, IplImage *red, bool caso){
    
    //calcular punto medio de la mano
	int x = rect->x + (rect->width)/2;
        int y = rect->y + (rect->height)/2;
	bool check = true;
	bool check2 = true;
	paso step;
 
	if (caso){
	   //tocar azul
	    if (x>30 && x<65 && y>265 && y<295){
		contadorBlue = 0;
		contadorGreen++;
		contadorRed++;
		check = false;	
	    }	
	    //tocar verde
	    if (x>30 && x<65 && y>340 && y<370){
		contadorGreen = 0;
		contadorBlue++;
		contadorRed++;
		check = false;
	    }
	    //tocar rojo
	    if (x>30 && x<65 && y>415 && y< 445){
		contadorRed = 0;
		contadorBlue++;
		contadorGreen++;
		check = false;	
	    }
	    //no tocas nada
	    if (check) {  
		    contadorBlue++;
		    contadorGreen++;
		    contadorRed++;
	    }
	
	check = true;
	    //tocar arriba
	    if (x>30 && x<65 && y>40 && y<70){
		contadorArriba = 0;
		contadorMedio++;
		contadorAbajo++;	
		check = false;
	    }	
	    //tocar medio
	    if (x>30 && x<65 && y>115 && y<145){
		contadorMedio = 0;
		contadorArriba++;
		contadorAbajo++;
		check = false;
	    }
	    //tocar abajo
	    if (x>30 && x<65 && y>190 && y< 220){
		contadorAbajo = 0;
		contadorMedio++;
		contadorArriba++;
		check = false;	
	    }
	    //no tocas nada
	    if (check) {  
		    contadorArriba++;
		    contadorMedio++;
		    contadorAbajo++;
	    }	
	
	    if (contadorBlue<10){
	        OverlayImage(img,blue,cvPoint(100, 240),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
		blueb = true;
	    } else
		blueb = false;
	
	    if(contadorRed<10){
	        OverlayImage(img,red,cvPoint(100,395),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
		redb = true;
	    } else
		redb = false;
	
	    if(contadorGreen<10){
	        OverlayImage(img,green,cvPoint(100,320),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
		greenb = true;
	    } else
		greenb = false;
	
	
	    if (contadorArriba<10){
		step.pos = 100;
		step.tipo = 0;
		step.frame = frameN;
		if (!pasos1.empty()){
			if ( frameN - pasos1.front().frame   > 30)
			    pasos1.push_front(step);
		} else {
			pasos1.push_front(step);
		}
	    }
	    if(contadorMedio<10){
		step.pos = 100;
		step.tipo = 1;
		step.frame = frameN;
		if (!pasos1.empty()){
			if ( frameN - pasos1.front().frame   > 30)
			    pasos1.push_front(step);
		} else {
			pasos1.push_front(step);
		}
	    }
	    if(contadorAbajo<10){
		step.pos = 100;
		step.tipo = 2;
		step.frame = frameN;
		if (!pasos1.empty()){
			if ( frameN - pasos1.front().frame  > 30)
			    pasos1.push_front(step);
		} else {
			pasos1.push_front(step);
		}
	    }
	
	} 
	else{
	
	    //tocar azul
	    if (x>560 && x<595 && y>40 && y<70){
		contadorBlue2 = 0;
		contadorGreen2++;
		contadorRed2++;	
		check2 = false;
	    }	
	    //tocar verde
	    if (x>560 && x<595 && y>115 && y<145){
		contadorGreen2 = 0;
		contadorBlue2++;
		contadorRed2++;
		check2 = false;
	    }
	    //tocar rojo
	    if (x>560 && x<595 && y>190 && y< 220){
		contadorRed2 = 0;
		contadorBlue2++;
		contadorGreen2++;
		check2 = false;	
	    }
	    //no tocas nada
	    if (check2) {  
		    contadorBlue2++;
		    contadorGreen2++;
		    contadorRed2++;
	    }
	
	    check2 = true;
	    //tocar arriba
	    if (x>560 && x<595 &&  y>265 && y<295){
		contadorArriba2 = 0;
		contadorMedio2++;
		contadorAbajo2++;	
		check2 = false;
	    }	
	    //tocar medio
	    if (x>560 && x<595 && y>340 && y<370){
		contadorMedio2 = 0;
		contadorArriba2++;
		contadorAbajo2++;
		check2 = false;
	    }
	    //tocar abajo
	    if (x>560 && x<595 && y>415 && y< 445){
		contadorAbajo2 = 0;
		contadorMedio2++;
		contadorArriba2++;
		check2 = false;	
	    }
	    //no tocas nada
	    if (check2) {  
		    contadorArriba2++;
		    contadorMedio2++;
		    contadorAbajo2++;
	    }
	    if (contadorBlue2<10){
	        OverlayImage(img,blue,cvPoint(345, 20),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
		blueb2 = true;
	    } else
		blueb2 = false;
		
	    if(contadorRed2<10){
	        OverlayImage(img,red,cvPoint(345,170),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
		redb2 = true;
	    } else
		redb2 = false;
		
	    if(contadorGreen2<10){
	        OverlayImage(img,green,cvPoint(345,95),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
		greenb2 = true;
	    } else
		greenb2 = false;
		
	    if (contadorArriba2<10){
		step.pos = 500;
		step.tipo = 0;
		step.frame = frameN;
		if (!pasos2.empty()){
			if ( frameN - pasos2.front().frame   > 30)
			    pasos2.push_front(step);
		} else {
			pasos2.push_front(step);
		}
	    }
	    if(contadorMedio2<10){
		step.pos = 500;
		step.tipo = 1;
		step.frame = frameN;
		if (!pasos2.empty()){
			if ( frameN - pasos2.front().frame   > 30)
			    pasos2.push_front(step);
		} else {
			pasos2.push_front(step);
		}
	    }
	    if(contadorAbajo2<10){
		step.pos = 500;
		step.tipo = 2;
		step.frame = frameN;
		if (!pasos2.empty()){
			if ( frameN - pasos2.front().frame  > 30)
			    pasos2.push_front(step);
		} else {
			pasos2.push_front(step);
		}
	    }

	}


}

void ChekAndScore(IplImage *src, std::list<int>::iterator it1, std::list<int>::iterator it2){}

std::string itoa(int value, unsigned int base) {
	const char digitMap[] = "0123456789abcdef";
	std::string buf;
	if (base == 0 || base > 16) {
		// Error: may add more trace/log output here
		return buf;
	}
	// Take care of negative int:
	std::string sign;
	int _value = value;
	// Check for case when input is zero:
	if (_value == 0) return "0";
	if (value < 0) {
		_value = -value;
		sign = "-";
	}
	// Translating number to string with base:
	for (int i = 30; _value && i ; --i) {
		buf = digitMap[ _value % base ] + buf;
		_value /= base;
	}
	return sign.append(buf);	
}

void GenerateScoreMessage(IplImage *src,int s1, int s2){
	
   /* set fonts and variables for score dislay */
   CvFont font;
   double hScale=0.5;
   double vScale=0.5;
   int    lineWidth=2;
   cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth);
	
	std::string score;
	score += "Player 1: ";
	score += itoa(s1,10);
	score += "Pts";
	cvPutText (src,score.c_str(),cvPoint(185,15), &font, cvScalar(255,255,0));
	score = ("Player 2: ");
	score += itoa(s2,10);
	score += "Pts";
	cvPutText (src,score.c_str(),cvPoint(360,15), &font, cvScalar(255,255,0));
}

void drawAndAdvance(IplImage *src,IplImage *good, IplImage *bad, IplImage *rstep1,  IplImage *rstep2 ){
    for (std::list<paso>::iterator it=pasos1.begin(); it!=pasos1.end(); ++it){
	
	//dibujar
	if ((*it).tipo == 0)
	OverlayImage(src,rstep1,cvPoint((*it).pos, 20),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
	if ((*it).tipo == 1)
	OverlayImage(src,rstep1,cvPoint((*it).pos, 95),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
	if ((*it).tipo == 2)
	OverlayImage(src,rstep1,cvPoint((*it).pos, 170),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
	
        //si llego al otro lado
	if ((*it).tipo == 0 && (*it).pos>560 && (*it).pos<595 ){
		if (blueb2){
			score2 += 1;
			//drawBien(src,good,0,0);
			pasos1.erase(it);
		}
	} else if ((*it).tipo == 1 && (*it).pos>560 && (*it).pos<595 ){
		if (greenb2){
			score2 += 1;
			//drawBien(src,good,1,0);
			pasos1.erase(it);
		}
	} else if ((*it).tipo == 2 && (*it).pos>560 && (*it).pos<595 ){
		if (redb2){
			score2 += 1;
			//drawBien(src,good,2,0);
			pasos1.erase(it);
			
		}
        }
        // si no se acerto el tambor
        if ((*it).tipo == 0 && (*it).pos>600 ){
		score2 -= 1;
		//drawBien(src,bad,0,0);
		pasos1.erase(it);
        }
	else if ((*it).tipo == 1 && (*it).pos>600  ){
		score2 -= 1;
		//drawBien(src,bad,1,0);
		pasos1.erase(it);		
	}
	else if ((*it).tipo == 1 && (*it).pos>600){
		score2 -= 1;
		//drawBien(src,bad,2,0);
		pasos1.erase(it);		
	}
        
	(*it).pos +=4;
		
    }



for (std::list<paso>::iterator it=pasos2.begin(); it!=pasos2.end(); ++it){

	//dibujar
	if ((*it).tipo == 0)
	OverlayImage(src,rstep2,cvPoint((*it).pos, 245),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
	if ((*it).tipo == 1)
	OverlayImage(src,rstep2,cvPoint((*it).pos, 320),cvScalar(1,1,1,1),cvScalar(1,1,1,1));
	if ((*it).tipo == 2)
	OverlayImage(src,rstep2,cvPoint((*it).pos, 395),cvScalar(1,1,1,1),cvScalar(1,1,1,1));

        //si llego al otro lado
	if ((*it).tipo == 0 && (*it).pos>30 && (*it).pos<65 ){
		if (blueb){
			score1 += 1;
			//drawBien(src,good,0,0);
			pasos2.erase(it);
		}
	} else if ((*it).tipo == 1 && (*it).pos>30 && (*it).pos<65 ){
		if (greenb){
			score1 += 1;
			//drawBien(src,good,1,0);
			pasos2.erase(it);
		}
	} else if ((*it).tipo == 2 && (*it).pos>30 && (*it).pos<65 ){
		if (redb){
			score1 += 1;
			//drawBien(src,good,2,0);
			pasos2.erase(it);

		}
        }
        // si no se acerto el tambor
        if ((*it).tipo == 0 && (*it).pos<25 ){
		score1 -= 1;
		//drawBien(src,bad,0,0);
		pasos2.erase(it);
        }
	else if ((*it).tipo == 1 && (*it).pos<25 ){
		score1 -= 1;
		//drawBien(src,bad,1,0);
		pasos2.erase(it);		
	}
	else if ((*it).tipo == 1 && (*it).pos<25){
		score1 -= 1;
		//drawBien(src,bad,2,0);
		pasos2.erase(it);		
	}

	(*it).pos -=4;

    }
}

void drawBien(IplImage *src, IplImage *bien, int tambor, bool caso) {
	
}

void drawMal(IplImage *src, IplImage *mal, int tambor, bool caso) {
	
}