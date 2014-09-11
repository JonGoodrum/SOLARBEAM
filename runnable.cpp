
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <mutex>
#include <functional>
extern "C"{
#include "ArcusPerformaxDriver.h"
}

using namespace std;

const int topx_min_stepcount = -36500;  //right limit
const int topx_max_stepcount =  36500;  //left  limit
const int topy_min_stepcount =      0;  //upper limit
const int topy_max_stepcount =  18250;  //lower limit
const int btmx_min_stepcount = -36500;  //left  limit
const int btmx_max_stepcount =  36500;  //right limit
const int btmy_min_stepcount =      0;  //lower limit
const int btmy_max_stepcount =  36500;  //upper limit

std::mutex mtx_topxl;
std::mutex mtx_topxr;
std::mutex mtx_topyu;
std::mutex mtx_topyd;
std::mutex mtx_btmxl;
std::mutex mtx_btmxr;
std::mutex mtx_btmyu;
std::mutex mtx_btmyd;
std::mutex mtx_done;
std::mutex mtx_toppulse;
std::mutex mtx_btmpulse;
std::mutex mtx_stepcount;

bool bool_done = false;
bool suntracking_finished = false;

AR_HANDLE ARH_top;
AR_HANDLE ARH_btm;

int topx_stepcount = 0;
int topy_stepcount = 0;
int btmx_stepcount = 0;
int btmy_stepcount = 0;


#include "HomeGimbals.cpp"
#include "RoverTracker.cpp"
#include "PulseAxes.cpp"
#include "SunTracker.cpp"


/* things marked with TODO are done
 * 
 * (2) turn on top, bottom gimbals
 * (3) TODO: send top, bottom gimbals to home position (4 threads, join)
 * (4) create TWO suntracker threads
 * (5) join both suntracker threads
 * (1) TODO: turn on webcam
 * (6) create FOUR top threads (pulseX, pulseY, cameraX, cameraY)
 * (7) wait for them to join (global bool "done?")
 * (8) end program
 *
 */




int main(){

    char		      lpDeviceString0[PERFORMAX_MAX_DEVICE_STRLEN];
    char		      lpDeviceString1[PERFORMAX_MAX_DEVICE_STRLEN];
    char		      out[64];
    char		      in[64];
    AR_DWORD		      num;
//////////////////////////////////////////////////////////////////////////////////////////
/*
	if( !fnPerformaxComGetProductString(0, lpDeviceString0, PERFORMAX_RETURN_SERIAL_NUMBER) ||
		!fnPerformaxComGetProductString(0, lpDeviceString0, PERFORMAX_RETURN_DESCRIPTION) )
	{
		cout << "error acquiring product string\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	//printf("device description: %s\n", lpDeviceString);
	
	//setup the connection
	
	if(!fnPerformaxComOpen(0,&ARH_top))
	{
		cout <<  "Error opening device\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	if(!fnPerformaxComSetTimeouts(5000,5000))
	{
		cout << "Error setting timeouts\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	if(!fnPerformaxComFlush(ARH_top))
	{
		cout << "Error flushing the coms\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	// setup the device
	
	strcpy(out, "LSPD=1000"); //set low speed
	if(!fnPerformaxComSendRecv(ARH_top, out, 64,64, in))
	{
		cout << "LSPD Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	strcpy(out, "HSPD=4000"); //set high speed
	if(!fnPerformaxComSendRecv(ARH_top, out, 64,64, in))
	{
		cout << "HSPD Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	strcpy(out, "POL=16"); //set polarity on the limit switch to be positive
	if(!fnPerformaxComSendRecv(ARH_top, out, 64,64, in))
	{
		cout << "POL Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}

	strcpy(out, "ACC=300"); //set acceleration
	if(!fnPerformaxComSendRecv(ARH_top, out, 64,64, in))
	{
		cout << "ACC Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	strcpy(out, "EO1=1"); //enable device
	if(!fnPerformaxComSendRecv(ARH_top, out, 64,64, in))
	{
		cout << "EO1 Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	strcpy(out, "EO2=1"); //enable device
	if(!fnPerformaxComSendRecv(ARH_top, out, 64,64, in))
	{
		cout << "EO1 Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}

	strcpy(out, "INC"); //set absolute mode
	if(!fnPerformaxComSendRecv(ARH_top, out, 64,64, in))
	{
		cout << "INC Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	strcpy(out, "ID"); //read current
	if(!fnPerformaxComSendRecv(ARH_top, out, 64,64, in))
	{
		cout << "ID Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	cout << "Arcus Product: " << in << endl;

	strcpy(out, "DN"); //read current
	if(!fnPerformaxComSendRecv(ARH_top, out, 64,64, in))
	{
		cout << "DN Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	cout << "Device Number: " << in << endl;

*/
//////////////////////////////////////////////////////////////////////////////////////////
	if(!fnPerformaxComGetNumDevices(&num))
	{
		cout << "error in fnPerformaxComGetNumDevices\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	if(num<1)
	{
		cout <<  "No motor found\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}

	if( !fnPerformaxComGetProductString(0, lpDeviceString1, PERFORMAX_RETURN_SERIAL_NUMBER) ||
		!fnPerformaxComGetProductString(0, lpDeviceString1, PERFORMAX_RETURN_DESCRIPTION) )
	{
		cout << "error acquiring product string\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	//printf("device description: %s\n", lpDeviceString1);
	
	//setup the connection
	
	if(!fnPerformaxComOpen(0,&ARH_btm))
	{
		cout <<  "Error opening device\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	if(!fnPerformaxComSetTimeouts(5000,5000))
	{
		cout << "Error setting timeouts\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	if(!fnPerformaxComFlush(ARH_btm))
	{
		cout << "Error flushing the coms\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	// setup the device
	
	strcpy(out, "LSPD=1000"); //set low speed
	if(!fnPerformaxComSendRecv(ARH_btm, out, 64,64, in))
	{
		cout << "LSPD Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	strcpy(out, "HSPD=4000"); //set high speed
	if(!fnPerformaxComSendRecv(ARH_btm, out, 64,64, in))
	{
		cout << "HSPD Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	strcpy(out, "POL=16"); //set polarity on the limit switch to be positive
	if(!fnPerformaxComSendRecv(ARH_btm, out, 64,64, in))
	{
		cout << "POL Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}

	strcpy(out, "ACC=300"); //set acceleration
	if(!fnPerformaxComSendRecv(ARH_btm, out, 64,64, in))
	{
		cout << "ACC Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	strcpy(out, "EO1=1"); //enable device
	if(!fnPerformaxComSendRecv(ARH_btm, out, 64,64, in))
	{
		cout << "EO1 Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	strcpy(out, "EO2=1"); //enable device
	if(!fnPerformaxComSendRecv(ARH_btm, out, 64,64, in))
	{
		cout << "EO1 Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}

	strcpy(out, "INC"); //set absolute mode
	if(!fnPerformaxComSendRecv(ARH_btm, out, 64,64, in))
	{
		cout << "INC Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	strcpy(out, "ID"); //read current
	if(!fnPerformaxComSendRecv(ARH_btm, out, 64,64, in))
	{
		cout << "ID Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	cout << "Arcus Product: " << in << endl;

	strcpy(out, "DN"); //read current
	if(!fnPerformaxComSendRecv(ARH_btm, out, 64,64, in))
	{
		cout << "DN Could not send\n";
		mtx_done.lock();
		bool_done = true;
		mtx_done.unlock();
		return 1;
	}
	
	cout << "Device Number: " << in << endl;
//////////////////////////////////////////////////////////////////////////////////////////////





  //TODO: Home Top, Bottom Gimbals
    std::cout << "//~~~~~~~~~~HOMING AXES~~~~~~~~~~//\n";
    std::thread thr_HomeTopX(HomeTopX);
    std::thread thr_HomeTopY(HomeTopY);
    std::thread thr_HomeBottomX(HomeBtmX);
    std::thread thr_HomeBottomY(HomeBtmY);
    thr_HomeTopX.join();
    thr_HomeTopY.join();
    thr_HomeBottomX.join();
    thr_HomeBottomY.join();




  //TODO: Begin SunTracking, wait for completion
    std::cout << "//~~~~~~~~~~TRACKING SUN~~~~~~~~~~//\n";
    std::thread thr_SunTracker(SunTracker);
    usleep(1000000);
    std::thread thr_PulseBottomX(PulseAxis, 50, 3);
    std::thread thr_PulseBottomY(PulseAxis, 50, 4);
    thr_PulseBottomX.join();
    thr_PulseBottomY.join();
    thr_SunTracker.join();




  //TODO: Begin RoverTracking, wait for completion  
    std::cout << "//~~~~~~~~~~TRACKING ROVER~~~~~~~~~~//\n";
    std::thread thr_CameraX(RoverTrackingX);
    //std::thread thr_CameraY(RoverTrackingY); // TODO: create RoverTrackingY algorithm
    //usleep(1000000);
    std::thread thr_PulseTopX(std::bind(PulseAxis, 50, 1));
    //std::thread thr_PulseTopY(std::bind(PulseAxis, 50, 2));
   
    thr_CameraX.join();
    thr_PulseTopX.join();




    if(!fnPerformaxComClose(ARH_top)){
      std::cout <<  "Error Closing Top Gimbal\n";
      mtx_done.lock();
      bool_done = true;
      mtx_done.unlock();
      return 1;
    }
    std::cout << "Top motor connection has been closed\n";

    if(!fnPerformaxComClose(ARH_btm)){
      std::cout <<  "Error Closing Bottom Gimbal\n";
      mtx_done.lock();
      bool_done = true;
      mtx_done.unlock();
      return 1;
    }
    std::cout << "Bottom motor connection has been closed\n";


}//end main()















