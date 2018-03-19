/*
#include "/usr/include/opencv2/objdetect/objdetect.hpp"
#include "/usr/include/opencv2/highgui/highgui.hpp"
#include "/usr/include/opencv2/imgproc/imgproc.hpp"
 //#include <cv.h>
 //#include <cvaux.h>
 //#include <highgui.h>
 //#include <yarp/os/Time.h>
 //#include <yarp/dev/ControlBoardInterfaces.h>
 //#include <yarp/dev/PolyDriver.h>
 //#include <yarp/sig/Vector.h>
 //using namespace std ;
 //#include <yarp/os/Network.h>
 */


#include </usr/include/opencv2/opencv.hpp>  
#include <iostream>  
#include <iterator> 
#include <stdio.h>  
#include <string>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

//void detectAndDraw(Mat& img, CascadeClassifier& cascade,
   //                double scale, bool tryflip);
Vec3i c =(0,0,0);
string cascadeName = "haarcascade_frontalface_alt2.xml";





//----------------------------------------------






//-------------------------------------


/*
void detectAndDraw(Mat& img, CascadeClassifier& cascade,
                   double scale, bool tryflip)
{
  
}

*/



void detectAndDraw(Mat& img, CascadeClassifier& cascade,
                   double scale, bool tryflip)
{
  int i = 0;
  double t = 0;
  vector<Rect> faces, faces2;
  const static Scalar colors[] = { CV_RGB(0,0,255),
                                   CV_RGB(0,128,255),
                                   CV_RGB(0,255,255),
                                   CV_RGB(0,255,0),
                                   CV_RGB(255,128,0),
                                   CV_RGB(255,255,0),
                                   CV_RGB(255,0,0),
                                   CV_RGB(255,0,255) };
  Mat gray, smallImg(cvRound(img.rows / scale), cvRound(img.cols / scale), CV_8UC1);
  
  cvtColor(img, gray, CV_BGR2GRAY);
  resize(gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR);
  equalizeHist(smallImg, smallImg);
  
  t = (double)cvGetTickCount();
  cascade.detectMultiScale(smallImg, faces,
                           1.1, 2, 0
                             //|CV_HAAR_FIND_BIGGEST_OBJECT  
                             | CV_HAAR_DO_ROUGH_SEARCH //Ч??????  
                             //|CV_HAAR_SCALE_IMAGE  
                             //|CV_HAAR_DO_CANNY_PRUNING  
                             ,
                             Size(30, 30));
  if (tryflip)
  {
    flip(smallImg, smallImg, 1); //??ת  
    cascade.detectMultiScale(smallImg, faces2,
                             1.1, 2, 0
                               //|CV_HAAR_FIND_BIGGEST_OBJECT  
                               | CV_HAAR_DO_ROUGH_SEARCH
                               //|CV_HAAR_SCALE_IMAGE  
                               //|CV_HAAR_DO_CANNY_PRUNING  
                               ,
                               Size(30, 30));
    for (vector<Rect>::const_iterator r = faces2.begin(); r != faces2.end(); r++)
    {
      faces.push_back(Rect(smallImg.cols - r->x - r->width, r->y, r->width, r->height));
    }
  }
  t = (double)cvGetTickCount() - t;
  printf("detection time = %g ms\n", t / ((double)cvGetTickFrequency()*1000.));
  for (vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++)
  {
    Mat smallImgROI;
    vector<Rect> nestedObjects;
    Point center;
    Scalar color = colors[i % 8];
    int radius;
    
    double aspect_ratio = (double)r->width / r->height;
    if (0.75 < aspect_ratio && aspect_ratio < 1.3)
    {
      center.x = cvRound((r->x + r->width*0.5)*scale);
      center.y = cvRound((r->y + r->height*0.5)*scale);
      radius = cvRound((r->width + r->height)*0.25*scale);
      circle(img, center, radius, color, 3, 8, 0);
    }
    else
      rectangle(img, cvPoint(cvRound(r->x*scale), cvRound(r->y*scale)),
                cvPoint(cvRound((r->x + r->width - 1)*scale), cvRound((r->y + r->height - 1)*scale)),
                color, 3, 8, 0);
    
    smallImgROI = smallImg(*r);
    
    for (vector<Rect>::const_iterator nr = nestedObjects.begin(); nr != nestedObjects.end(); nr++)
    {
      center.x = cvRound((r->x + nr->x + nr->width*0.5)*scale);
      center.y = cvRound((r->y + nr->y + nr->height*0.5)*scale);
      radius = cvRound((nr->width + nr->height)*0.25*scale);
      circle(img, center, radius, color, 3, 8, 0);
    }
  }
  // cv::imshow("result", img);
}


//-------------------------------






int main()
{
 
// MHD Rateb MHD Ziad Alissa
//set up yarp and prepare the ports
Network yarp; 
BufferedPort<ImageOf<PixelRgb> > imagePort; // make a port for reading images
imagePort.open("/tutorial/image/in"); // give the port a name
//Network::connect("/grabber","/tutorial/image/in");
Network::connect("/icubSim/cam/left","/tutorial/image/in");
//Network::connect("/test/video","/tutorial/image/in");

BufferedPort<ImageOf<PixelRgb> > linearFilter_imageOutput;          
linearFilter_imageOutput.open("/tutorial/image/out1");   
Network::connect("/tutorial/image/out1","/view/linearFilter"); 

BufferedPort<ImageOf<PixelRgb> > faceDetection_imageOutput;          
faceDetection_imageOutput.open("/tutorial/image/out2");   
Network::connect("/tutorial/image/out2","/view/faceDetection");

BufferedPort<ImageOf<PixelRgb> > objectDetection_imageOutput;          
objectDetection_imageOutput.open("/tutorial/image/out3");   
Network::connect("/tutorial/image/out3","/view/objectDetection");

// MHD Rateb MHD Ziad Alissa
//declare some varilbes for point 7
bool ReadDone=0;
bool Fullwave=0;
bool WriteDone=0;
char WaveCount=5;

//set up the device for point 7
Property options; // create device options
options.put( "device" , "remote_controlboard") ;
options.put( "local" , "/tutorial/client") ; // Local parameters
options.put( "remote" , "/icubSim/right_arm") ; //where we connect to remote parameters
PolyDriver robotDevice(options) ; // create driver

if(!robotDevice.isValid( ) ) { // check i f the d ri v e r i s a v ail abl e
  printf( "Device not avail able. Here are the known devices : \n" ) ;
  printf( "%s" , Drivers::factory( ).toString( ).c_str( ) ) ;
  return 1;
}

// point 6 prepartion
//-----------------------------------------
Property options2;
options2.put("device", "remote_controlboard");
options2.put("local", "/tutorial/motor/client");
options2.put("remote", "/icubSim/head");
PolyDriver robotHead(options2);
if (!robotHead.isValid()) {
  printf("Cannot connect to robot head\n");
  return 1;
}



IPositionControl *pos2;
IVelocityControl *vel2;
IEncoders *enc2;
IControlMode2 *ictrl2;
robotHead.view(pos2);
robotHead.view(vel2);
robotHead.view(enc2);
robotHead.view(ictrl2) ;
if (pos2==NULL || vel2==NULL || enc2==NULL || ictrl2==NULL) {
  printf("Cannot get interface to robot head\n");
  robotHead.close();
  return 1;
}
int jnts2 = 0;
pos2->getAxes(&jnts2);
yarp::sig::Vector setpoints2;
setpoints2.resize(jnts2);

//-----------------------------



while(1){
// MHD Rateb MHD Ziad Alissa  
//point 1 read an image
ImageOf<PixelRgb> *imge = imagePort.read();  
//printf("fffffffffff %g ",imge)  ;
	 
// MHD Rateb MHD Ziad Alissa 
//convert from yarp images to openCV images
printf("Copying YARP image to an OpenCV/IPL image\n");
IplImage *cvImage = cvCreateImage(cvSize(imge->width(),imge->height()),IPL_DEPTH_8U, 3 );
cvCvtColor((IplImage*)imge->getIplImage(), cvImage, CV_RGB2BGR);

//-------------------------------------------------------------
// MHD Rateb MHD Ziad Alissa	 
//point 3 aplly linear filter 

	Mat src, src_gray;
	Mat dst1;

	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	src = Mat(cvImage); // Load an image
	if( src.empty() )
	{ return -1; }
	GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );
	cvtColor( src, src_gray, COLOR_BGR2GRAY );
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );

	Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_x, abs_grad_x );
	convertScaleAbs( grad_y, abs_grad_y );
	addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst1 );
	//point 3 finished
	//------------------------------------------------------

	// MHD Rateb MHD Ziad Alissa
	//convert from oprnCV images to yarp images
	IplImage copy1 = dst1;
	IplImage* new_image1 = &copy1;

	printf("Taking image back into YARP...\n");
  ImageOf<PixelRgb> yarpReturnImage1;
  yarpReturnImage1.wrapIplImage(new_image1);

  // MHD Rateb MHD Ziad Alissa   
//write the results filtered images to the separt view (linear filter view).	   
linearFilter_imageOutput.prepare() = yarpReturnImage1;
linearFilter_imageOutput.write(); 



//------------------------------------------------------
// owner	 
//point 4 aplly face detection

//use Mat(cvImage) or cvImage as input 
 


// face detection algorithm
// we need face coordinates for point 6
// we need signal that you find face for point 7

  cv::CascadeClassifier cascade;
  double scale2 = 1;
  bool tryflip = true;
  cv::Mat img = Mat(cvImage);
  cv::Mat imgCopy = img.clone();
  cascade.load(cascadeName);
  //nestedCascade.load(nestedCascadeName);src3
 detectAndDraw(imgCopy, cascade, scale2, tryflip);
  //cv::waitKey(0);

  



//please give the image output name dst2


//point 4 finished
//------------------------------------------------------

// MHD Rateb MHD Ziad Alissa
//convert from oprnCV images to yarp images
IplImage copy2 = imgCopy;
IplImage* new_image2 = &copy2;

printf("Taking image back into YARP...\n");
ImageOf<PixelRgb> yarpReturnImage2;
yarpReturnImage2.wrapIplImage(new_image2);

// MHD Rateb MHD Ziad Alissa
//write the results filtered images to the separt view (linear filter view).	   
faceDetection_imageOutput.prepare() = yarpReturnImage2;
faceDetection_imageOutput.write(); 


//------------------------------------------------------
// owner	 
//point 5 aplly object detection

//use Mat(cvImage) or cvImage as input 



// object detection algorithm
 // we need object coordinates for point 6
 // we need signal that you find object for point 7
 


    // Loads an image
    Mat src4 = Mat(cvImage);
    // Check if image is loaded fine

    Mat gray4;
    cvtColor(src4, gray4, COLOR_BGR2GRAY);
    medianBlur(gray4, gray4, 5);
    vector<Vec3f> circles;
    HoughCircles(gray4, circles, CV_HOUGH_GRADIENT, 1,gray4.rows/16, 100, 30, 1, 30);
    //  HoughCircles( src_gray3, circles, CV_HOUGH_GRADIENT, 1, src_gray3.rows/8, 200, 100, 0, 0 );
    for( size_t i = 0; i < circles.size(); i++ )
    {
      
         c = circles[i];
      printf("ssssssssssssssssssssssssssssssssss   X= %g",c[0]);
        Point center = Point(c[0], c[1]);
        // circle center
        circle( src4, center, 1, Scalar(0,100,100), 3, 8, 0 );
        // circle outline
        int radius = c[2];
        circle( src4, center, radius, Scalar(255,0,255), 3, 8, 0 );
       
    }


//please give the output name dst3


//point 5 finished
//------------------------------------------------------

// MHD Rateb MHD Ziad Alissa
//convert from oprnCV images to yarp images
IplImage copy3 = src4;
IplImage* new_image3 = &copy3;

printf("Taking image back into YARP...\n");
ImageOf<PixelRgb> yarpReturnImage3;
yarpReturnImage3.wrapIplImage(new_image3);

// MHD Rateb MHD Ziad Alissa
//write the results filtered images to the separt view (linear filter view).	   
objectDetection_imageOutput.prepare() = yarpReturnImage3;
objectDetection_imageOutput.write();


printf("fffffffffffffffffffffffffffffffffffffffffffffff   X= %g ",c[0]);
printf("\n");

//------------------------------------------------------
// Sukrit	 
//point 6 aplly object detection


//finite-state machine (FSM) 

 
// yarp::sig::Vector target ; // read a target
// target.resize(3);
// target[0] = c[0];
//target[1] = c[1];
 //target[2] = 1;
// if (target!=NULL) { // check we actually got something
//printf("We got a vector containing");
//for (int i=0; i<target->size(); i++) {
//printf(" %g", (*target)[i]);
//}
  printf("x  %g", c[0],"\n"," y %g",c[1]);
printf("\n");
double x = c[0];
double y = c[1];
double conf = 1;
x -= 320/2;
y -= 240/2;
double vx = x*0.1;
double vy = -y*0.1;
/* prepare command */
for (int i=0; i<jnts2; i++) {
//  ictrl2->setControlMode(i,VOCAB_CM_VELOCITY);
  setpoints2[i] = 0;
}
if (conf>0.5) {
  setpoints2[3] = vy;
  setpoints2[4] = vx;
} else {
  setpoints2[3] = 0;
  setpoints2[4] = 0;
  
}
vel2->velocityMove(setpoints2.data());
//Vector& target = output.prepare();
//target.resize(1);
//target[0] = 5;
//output.write();
//}
 



//point 6 finished
//------------------------------------------------------
 



//check when it is seeing a particular circle or marker or face
if(circles.size()>0){
  //------------------------------------------------------
  // MHD Rateb MHD Ziad Alissa	 
  //point 7 aplly wave
  
  
  //wave 
  // create pointers
  IPositionControl *pos;
  IVelocityControl *vel;
  IEncoders *enc;
  IControlMode2 *ictrl;
  robotDevice.view(pos) ;
  robotDevice.view(vel) ;
  robotDevice.view(enc) ;
  robotDevice.view(ictrl) ;
  // check i f the poin ters are v alid
  if(pos==NULL || vel==NULL || enc==NULL || ictrl==NULL) {
    printf( " Error getting IPositionControl in terface . \n" ) ;
    robotDevice.close( ) ;
    return 1;
  }
  
  // ge t the amount o f j oi n t s o f the device , in t hi s case the arm
  int jnts = 0;
  pos->getAxes(&jnts) ;
  printf( "jnts = %d \n" ,jnts) ;
  yarp::sig::Vector setpoints;
  yarp::sig::Vector RefSpeed;
  yarp::sig::Vector Encoders;
  yarp::sig::Vector RefAcceleration;
  yarp::sig::Vector CurrentPos;
  setpoints.resize(jnts) ;
  RefSpeed.resize(jnts) ;
  RefAcceleration.resize(jnts) ;
  Encoders.resize(jnts) ;
  CurrentPos.resize(1) ;
  
  
  
  printf("\n");
  for(int i=0; i<jnts; i++) {
    ictrl->setControlMode(i,VOCAB_CM_POSITION) ;
    RefSpeed[i] = 25;
    RefAcceleration[i]= 30;
    setpoints[i] = 0;
  }
  pos->setRefSpeeds(RefSpeed.data( ) ) ;
  pos->setRefAccelerations(RefAcceleration.data( ) ) ;
  printf( "Waiting for encoders to become availabl e . . . " ) ;
  while ( !(enc->getEncoders(Encoders.data()))) {
    Time::delay(0.1) ;
    printf( "." ) ;
  }
  printf( " \n Encoders Found \n" ) ;
  for ( int i=0; i<jnts; i++) {
    printf( "Encoder value %d = %g \n" ,i,Encoders[i] ) ;
  }
  setpoints=Encoders;
  setpoints[0]= -90;
  setpoints[1] = 90;
  setpoints[2] = 0;
  setpoints[3] = 25;
  WriteDone=pos->positionMove(setpoints.data()) ;
  while (WaveCount>0) {
    if (ReadDone) {
      if (Encoders[3] >=85) {
        WriteDone=pos->positionMove(3,25) ;
        if (Fullwave) {
          WaveCount--;
          Fullwave=0;
        }
      }
      if (Encoders[3]<=30) {
        WriteDone=pos->positionMove(3,90) ;
        Fullwave=1;
      }
    }
    
    if (WriteDone) ReadDone=enc->getEncoders(Encoders.data( ) ) ;
  }
  printf( " i wove %d times , i am done fo r today goodby \n" ,WaveCount) ;
  
  
  //point 7 finished
  //------------------------------------------------------
  
 
}

}

	 	return 0;
}





