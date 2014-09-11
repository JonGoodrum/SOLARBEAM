#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include<string>

using namespace cv;

const int lowhue = 50;
const int lowsat = 120;
const int lowlum = 110;
const int highhue = 80;
const int highsat = 160;
const int highlum = 255;

cv::VideoCapture rovercam(1);

bool RoverTrackingX(){

  mtx_topxl.lock();
  mtx_topxr.lock();

  std::string str1;
  cv::Mat origimg;
  cv::Mat hslimg;
  cv::Mat threshimg;

  bool lastseenonleft;

  while(1){

    //break if frames not stored
    if(!rovercam.read(origimg)){break;}

    //BGR to HSL conversion
    cv::cvtColor(origimg, hslimg, CV_BGR2HSV);


    //threshold the image
    cv::inRange( hslimg,
             cv::Scalar(lowhue, lowsat, lowlum),
	     cv::Scalar(highhue, highsat, highlum),
	     threshimg);
    
    cv::erode( threshimg, threshimg,
    cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate( threshimg, threshimg, 
    cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)) );

    cv::dilate( threshimg, threshimg, 
    cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode( threshimg, threshimg,
    cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)) );

  //  cv::imshow("Specific Color", threshimg);
  //  cv::imshow("Original Feed", origimg);

//********** TODO: BEGIN TEST REGION



//this is for making two new Mats which point at the
//LH, RH sides of threshimg.  Magnitude = O(1)
    
    int lbd = 0;
    int m1bd = origimg.rows/2-17;
    int m2bd = origimg.rows/2+17;
    int rbd = origimg.rows;


    cv::Mat thrL(threshimg, cv::Range(lbd, m1bd), cv::Range::all());  //top of camera, right of gimbal
    cv::Mat thrM(threshimg, cv::Range(m1bd, m2bd), cv::Range::all()); //middle of both
    cv::Mat thrR(threshimg, cv::Range(m2bd, rbd), cv::Range::all());  //bottom of camera, left of gimbal
//    cv::imshow("TOP B&W", thrL);
//    cv::imshow("MID B&W", thrM);
//    cv::imshow("BTM B&W", thrR);

//    Mat orgL(origimg, Range(lbd, m1bd), Range::all());
//    Mat orgM(origimg, Range(m1bd, m2bd), Range::all());
//    Mat orgR(origimg, Range(m2bd, rbd), Range::all());
//    imshow("TOP ORIG", orgL); imshow("MID ORIG", orgM); imshow("BTM ORIG", orgR);


    double avgdist = (mean(thrL)[0] + mean(thrR)[0])/2;
    

    std::string prevstr = str1;
    if( (avgdist > 0.2) or (avgdist < -0.2) ){//if (there's a noticeable difference between left/right)
        if(mean(thrL)[0] > mean(thrR)[0]){//if (more white pixels on the left)
           //str1 = "move left";
           lastseenonleft = true;
	   mtx_topxl.unlock();
	   while(1){
             mtx_done.lock();
             if(bool_done){mtx_done.unlock(); break;}
             mtx_done.unlock();
	     if(!mtx_topxl.try_lock()){break;}
	     mtx_topxl.unlock();
           }
	   mtx_topxl.lock();
        }else{//else (more white pixels on the right)
           str1 = "move right";
	   lastseenonleft = false;
	   mtx_topxr.unlock();
	   while(1){
             mtx_done.lock();
             if(bool_done){mtx_done.unlock(); break;}
             mtx_done.unlock();
	     if(!mtx_topxr.try_lock()){break;}
	     mtx_topxr.unlock();
           }
	   mtx_topxr.lock();
       }
    }else{//else if(there's NOT a noticeable difference between left/right)
        if(mean(thrM)[0] < 0.2){//if(the image is NOT in the middle), we need to scan for it
            if(lastseenonleft){//if(the image was seen on left, but is now out of the frame, then it must've travelled so far left that we can't see it (so we need to pulse left))
              mtx_stepcount.lock();
              if(topx_stepcount >= topx_max_stepcount){lastseenonleft=!lastseenonleft;}//if (we've exceeded our stepcount) {scan the other direction}
              mtx_stepcount.unlock();

              mtx_topxl.unlock();
 	      while(1){
                mtx_done.lock();
                if(bool_done){mtx_done.unlock(); break;}
                mtx_done.unlock();
	        if(!mtx_topxl.try_lock()){break;}
	        mtx_topxl.unlock();
              }
	      mtx_topxl.lock();
	    }else{//else if(last seen on right) 
              mtx_stepcount.lock();
              if(topx_stepcount <= topx_min_stepcount){lastseenonleft=!lastseenonleft;}
              mtx_stepcount.unlock();
              mtx_topxr.unlock();
	      while(1){
                mtx_done.lock();
                if(bool_done){mtx_done.unlock(); break;}
                mtx_done.unlock();
	        if(!mtx_topxr.try_lock()){break;}
	        mtx_topxr.unlock();
              }
	      mtx_topxr.lock();
	    }
        }else{//end if(the image is NOT in the middle)
          str1 = "middle :]";
        }
    }
    if(prevstr.compare(str1) != 0){//this keeps us from printing a bajillion lines to the terminal, by only printing when the string changes
      std::cout << str1 << "   LEFT AVG: " << mean(thrL)[0]  << " RIGHT AVG: " << mean(thrR)[0] << std::endl;
    }


//********** TODO: END TEST REGION
    mtx_done.lock();
    if(bool_done){rovercam.release(); mtx_done.unlock(); break;}
    mtx_done.unlock();
  }//end While
  mtx_done.lock();
  bool_done = true;
  //std::cout << "CameraX set done=true\n";
  mtx_done.unlock();
  std::cout << "CameraX done\n";

}//end RoverTrackingX

































































/*



///////////////////////////////OLDCODEBELOW

int main(){

////INITIALIZE GIMBAL
	char 		lpDeviceString[PERFORMAX_MAX_DEVICE_STRLEN];
	AR_HANDLE	Handle; //usb handle
	char		out[64];
	char		in[64];
	AR_DWORD	num;


	if(!fnPerformaxComGetNumDevices(&num))
	{
		cout << "error in fnPerformaxComGetNumDevices\n";
		return 1;
	}
	if(num<1)
	{
		cout <<  "No motor found\n";
		return 1;
	}

	if( !fnPerformaxComGetProductString(0, lpDeviceString, PERFORMAX_RETURN_SERIAL_NUMBER) ||
		!fnPerformaxComGetProductString(0, lpDeviceString, PERFORMAX_RETURN_DESCRIPTION) )
	{
		cout << "error acquiring product string\n";
		return 1;
	}
	
	//printf("device description: %s\n", lpDeviceString);
	
	//setup the connection
	
	if(!fnPerformaxComOpen(0,&Handle))
	{
		cout <<  "Error opening device\n";
		return 1;
	}
	
	if(!fnPerformaxComSetTimeouts(5000,5000))
	{
		cout << "Error setting timeouts\n";
		return 1;
	}
	if(!fnPerformaxComFlush(Handle))
	{
		cout << "Error flushing the coms\n";
		return 1;
	}
	
	// setup the device
	
	strcpy(out, "LSPD=1000"); //set low speed
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		cout << "Could not send\n";
		return 1;
	}
	strcpy(out, "HSPD=4000"); //set high speed
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		cout << "Could not send\n";
		return 1;
	}
	
	strcpy(out, "POL=16"); //set polarity on the limit switch to be positive
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		cout << "Could not send\n";
		return 1;
	}

	strcpy(out, "ACC=300"); //set acceleration
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		cout << "Could not send\n";
		return 1;
	}
	
	strcpy(out, "EO1=1"); //enable device
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		cout << "Could not send\n";
		return 1;
	}

	strcpy(out, "INC"); //set incremental mode
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		cout << "Could not send\n";
		return 1;
	}
	
	strcpy(out, "ID"); //read current
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		cout << "Could not send\n";
		return 1;
	}
	
	cout << "Arcus Product: " << in << endl;

	strcpy(out, "DN"); //read current
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in))
	{
		cout << "Could not send\n";
		return 1;
	}
	
	cout << "Device Number: " << in << endl;

	cout << "Motor is active...\n";
	


////END GIMBAL INIT














  //first, activate the webcam
  VideoCapture cap(1);

  //terminate program if camera doesn't activate
  if(!cap.isOpened()){
    cout << "webcam not initializing..." << endl;
    return -1;
  }

  //create a video to show the activated webcam
  namedWindow("Color Detection", CV_WINDOW_AUTOSIZE);

  int lowhue = 50;
  int highhue = 90;

  int lowsat = 150;
  int highsat = 255;

  int lowlum = 100;
  int highlum = 255;

  //add HSL sliders in the "Object Detection" window
  createTrackbar("lowhue", "Color Detection", &lowhue, 100);
  createTrackbar("highhue", "Color Detection", &highhue, 100);
  createTrackbar("lowsat", "Color Detection", &lowsat, 255);
  createTrackbar("highsat", "Color Detection", &highsat, 255);
  createTrackbar("lowlum", "Color Detection", &lowlum, 255);
  createTrackbar("highlum", "Color Detection", &highlum, 255);

  String str1;
  while(1){
    //create a mat to store webcam frames
    Mat origimg;
    Mat hslimg;

    //break if frames not stored
    if(!cap.read(origimg)){break;}

    //BGR to HSL conversion
    cvtColor(origimg, hslimg, CV_BGR2HSV);

    Mat threshimg;

    //threshold the image
    inRange( hslimg,
             Scalar(lowhue, lowsat, lowlum),
	     Scalar(highhue, highsat, highlum),
	     threshimg);
    
    erode( threshimg, threshimg,
    getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( threshimg, threshimg, 
    getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    dilate( threshimg, threshimg, 
    getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode( threshimg, threshimg,
    getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    imshow("Specific Color", threshimg);
    imshow("Original Feed", origimg);

////********** TODO: BEGIN TEST REGION

//this prints out every pixel (LAAAAAME!!!)
//  cout << endl << endl << "Threshold readings:" << endl << endl << format(threshimg,"csv") << endl;




//this is for making two new Mats which point at the
//LH, RH sides of threshimg.  Magnitude = O(1)
    
    Mat tm1(threshimg, Range::all(), Range(0,threshimg.cols/2-50));
    Mat tm2(threshimg, Range::all(), Range(threshimg.cols/2+50, threshimg.cols));

    double avgdist = (mean(tm1)[0] + mean(tm2)[0])/2;
    

    String prevstr = str1;
    if( (avgdist > 1.0) or (avgdist < -1.0) ){
      if(mean(tm1)[0] > mean(tm2)[0]){
        str1 = "move left";
	strcpy(out, "X-3000"); //move the motor
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in)){
	  cout << "Could not send\n";
	  return 1;
	}
	strcpy(out, "MSTX"); //move the motor
	while( strcmp("0",in) != 0 ){
	  if(!fnPerformaxComSendRecv(Handle, out, 64,64, in)){
            cout << "Could not send\n";
	    return 1;
	  }
        }
      }else{
        str1 = "move right";
	strcpy(out, "X3000"); //move the motor
	if(!fnPerformaxComSendRecv(Handle, out, 64,64, in)){
	  cout << "Could not send\n";
	  return 1;
	}
	strcpy(out, "MSTX"); //move the motor
	while( strcmp("0",in) != 0 ){
	  if(!fnPerformaxComSendRecv(Handle, out, 64,64, in)){
            cout << "Could not send\n";
	    return 1;
	  }
        }
      }
    }else{ str1 = "middle :]";}
    if (prevstr.compare(str1) != 0){
      cout << endl << str1 << endl;
    }










////********** TODO: END TEST REGION

    if(waitKey(30) >= 0){break;}

  }


	if(!fnPerformaxComClose(Handle))
	{
		cout <<  "Error Closing\n";
		return 1;
	}
	
	cout << "Motor connection has been closed\n";


  return 69;

}

*/
