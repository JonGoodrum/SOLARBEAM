#include <iostream>
#include <string.h>
extern "C"{
#include "ArcusPerformaxDriver.h"
}


int InitializeGimbals(int gimbalcontroller){

  char useless;
  char useless2;
  char lpDeviceString[PERFORMAX_MAX_DEVICE_STRLEN];
  char lpDeviceString0[PERFORMAX_MAX_DEVICE_STRLEN];
  char lpDeviceString1[PERFORMAX_MAX_DEVICE_STRLEN];
  char write[64];
  char read[64];
  AR_DWORD	num;
  AR_HANDLE* ARH_ptr;
  


  mtx_done.lock();
  if(bool_done){ mtx_done.unlock(); return 1;}
  mtx_done.unlock();


  if(gimbalcontroller==0){
    std::cout << "Make sure all gimbals are unplugged.  Press 'y' and hit ENTER when done\n";
    std::cin >> useless;
    std::cout << "Plug in ONLY the TOP gimbal.  Press 'y' and hit ENTER when done\n";
    std::cin >> useless2;
//    ARH_ptr = &ARH_top; //commented out to test the bottom individually
    ARH_ptr = &ARH_btm;
  }else if(gimbalcontroller==1){
    std::cout << "Now, plug in the BOTTOM gimbal.  Keep the TOP plugged in.\n  Press 'y' and hit ENTER when done\n";
    std::cin >> useless;
    ARH_ptr = &ARH_btm;
  }
  
 
  if(!fnPerformaxComGetNumDevices(&num)){std::cout << "couldn't get number of devices\n"; return 1;}

  std::cout << num << " gimbals plugged in...\n";
  
/*  
   
  if(gimbalcontroller==0){
    if(!fnPerformaxComGetProductString( 0, lpDeviceString0, PERFORMAX_RETURN_SERIAL_NUMBER)
    || !fnPerformaxComGetProductString( 0, lpDeviceString0, PERFORMAX_RETURN_DESCRIPTION) ){
      std::cout << "error acquiring product string\n";
      mtx_done.lock();
      bool_done = true;
      mtx_done.unlock();
      return 1;
    }
//    if(!fnPerformaxComOpen( 0 , &ARH_top)){
      if(!fnPerformaxComOpen( 0 , &ARH_btm)){  //inserted to test bottom individually
      std::cout <<  "Error opening device\n";
      mtx_done.lock();
      bool_done = true;
      mtx_done.unlock();
      return 1;
    }	
  }

   
  if(gimbalcontroller==1){
    if(!fnPerformaxComGetProductString( 0, lpDeviceString1, PERFORMAX_RETURN_SERIAL_NUMBER)
    || !fnPerformaxComGetProductString( 0, lpDeviceString1, PERFORMAX_RETURN_DESCRIPTION) ){
      std::cout << "error acquiring product string\n";
      mtx_done.lock();
      bool_done = true;
      mtx_done.unlock();
      return 1;
    }
    if(!fnPerformaxComOpen( 1 , &ARH_btm)){
      std::cout <<  "Error opening device\n";
      mtx_done.lock();
      bool_done = true;
      mtx_done.unlock();
      return 1;
    }	
  }
*/

    if(!fnPerformaxComGetProductString( 0, lpDeviceString, PERFORMAX_RETURN_SERIAL_NUMBER)
    || !fnPerformaxComGetProductString( 0, lpDeviceString, PERFORMAX_RETURN_DESCRIPTION) ){
      std::cout << "error acquiring product string\n";
      mtx_done.lock();
      bool_done = true;
      mtx_done.unlock();
      return 1;
    }
    if(!fnPerformaxComOpen( gimbalcontroller , &(*ARH_ptr) )){
      std::cout <<  "Error opening device\n";
      mtx_done.lock();
      bool_done = true;
      mtx_done.unlock();
      return 1;
    }	
  













  if(!fnPerformaxComSetTimeouts(5000,5000)){
    std::cout << "Error setting timeouts\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return 1;
  }
  if(!fnPerformaxComFlush((*ARH_ptr))){
    std::cout << "Error flushing the coms\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return 1;
  }	
  strcpy(write, "LSPD=1000"); //set low speed
  if(!fnPerformaxComSendRecv((*ARH_ptr), write, 64,64, read)){
    std::cout << "LSPD Could not send\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return 1;
  }
  strcpy(write, "HSPD=4000"); //set high speed
  if(!fnPerformaxComSendRecv((*ARH_ptr), write, 64,64, read)){
    std::cout << "HSPD Could not send\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return 1;
  }
  strcpy(write, "POL=16"); //set polarity on the limit switch to be positive
  if(!fnPerformaxComSendRecv((*ARH_ptr), write, 64,64, read)){
    std::cout << "POL Could not send\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return 1;
  }
  strcpy(write, "ACC=300"); //set acceleration
  if(!fnPerformaxComSendRecv((*ARH_ptr), write, 64,64, read)){
    std::cout << "ACC Could not send\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return 1;
  }
  strcpy(write, "EO1=1"); //enable X-Axis
  if(!fnPerformaxComSendRecv((*ARH_ptr), write, 64,64, read)){
    std::cout << "EO1 Could not send\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return 1;
  }
  strcpy(write, "EO2=1"); //enable Y-Axis
  if(!fnPerformaxComSendRecv((*ARH_ptr), write, 64,64, read)){
    std::cout << "EO1 Could not send\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return 1;
  }
  strcpy(write, "INC"); //set incremental mode
  if(!fnPerformaxComSendRecv((*ARH_ptr), write, 64,64, read)){
    std::cout << "INC Could not send\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return 1;
  }
  strcpy(write, "ID"); //read current
  if(!fnPerformaxComSendRecv((*ARH_ptr), write, 64,64, read)){
    std::cout << "ID Could not send\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return 1;
  }
  
  std::cout << "Arcus Product: " << read << std::endl;
  
  strcpy(write, "DN"); //read current
  if(!fnPerformaxComSendRecv((*ARH_ptr), write, 64,64, read)){
    std::cout << "DN Could not send\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return 1;
  }
	
  std::cout << "Device Number: " << read << std::endl;






  if(gimbalcontroller==0){std::cout << "TOP initialized\n";}
  else if(gimbalcontroller==1){std::cout << "BOTTOM initialized\n";}
}
