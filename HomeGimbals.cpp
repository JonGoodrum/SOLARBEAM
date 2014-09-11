#include<string.h>
#include<iostream>
#include<unistd.h>
extern"C"{
#include "ArcusPerformaxDriver.h"
}


bool HomeTopX(){

  char read[64];
  char write[64];

  strcpy(write, "X74000");
  mtx_toppulse.lock();
  if(!fnPerformaxComSendRecv(ARH_top, write, 64,64, read)){
    std::cout << "Could not pulse TopX (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return false;
  }	
  mtx_toppulse.unlock();
  while(1){
    usleep(500000);
    strcpy(write, "MSTX");
    mtx_toppulse.lock();
    if(!fnPerformaxComSendRecv(ARH_top, write, 64,64, read)){
      std::cout << "Could not check TopX motor status (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
      return false;
    }
    mtx_toppulse.unlock();
    if( strcmp(read,  "0") == 0 ){break;}
    if( strcmp(read, "16") == 0 ){break;}
    if( strcmp(read, "32") == 0 ){break;}
    if( strcmp(read, "64") == 0 ){break;}
    if( strcmp(read, "144") == 0 ){break;}
    if( strcmp(read, "288") == 0 ){break;}
  }

  strcpy(write, "X-36578");
  mtx_toppulse.lock();
  if(!fnPerformaxComSendRecv(ARH_top, write, 64,64, read)){
    std::cout << "Could not pulse TopX (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return false;
  }	
    mtx_toppulse.unlock();
  while(1){
    usleep(500000);
    strcpy(write, "MSTX");
    mtx_toppulse.lock();
    if(!fnPerformaxComSendRecv(ARH_top, write, 64,64, read)){
      std::cout << "Could not check TopX motor status (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
      return false;
    }
    mtx_toppulse.unlock();
    if( strcmp(read,  "0") == 0 ){break;}
    if( strcmp(read, "16") == 0 ){break;}
    if( strcmp(read, "32") == 0 ){break;}
    if( strcmp(read, "64") == 0 ){break;}
    if( strcmp(read, "144") == 0 ){break;}
    if( strcmp(read, "288") == 0 ){break;}
  }

  mtx_stepcount.lock();
  topx_stepcount = 0;
  mtx_stepcount.unlock();


}



bool HomeTopY(){

  char read[64];
  char write[64];

  strcpy(write, "Y-74000");
  mtx_toppulse.lock();
  if(!fnPerformaxComSendRecv(ARH_top, write, 64,64, read)){
    std::cout << "Could not pulse TopY (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return false;
  }	
  mtx_toppulse.unlock();
  while(1){
    usleep(500000);
    strcpy(write, "MSTY");
    mtx_toppulse.lock();
    if(!fnPerformaxComSendRecv(ARH_top, write, 64,64, read)){
      std::cout << "Could not check TopY motor status (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
      return false;
    }
    mtx_toppulse.unlock();
    if( strcmp(read,  "0") == 0 ){break;}
    if( strcmp(read, "16") == 0 ){break;}
    if( strcmp(read, "32") == 0 ){break;}
    if( strcmp(read, "64") == 0 ){break;}
    if( strcmp(read, "144") == 0 ){break;}
    if( strcmp(read, "288") == 0 ){break;}
  }

  //strcpy(write, "Y18292"); //this is where this program is supposed to Home Y to.  because there's not enough
                                          //of a viewing angle with this camera mount, we've decided to arbitrarily home
                                          //it to 12,000 (so it can see most of the FRL's floors, not the wall
  strcpy(write, "Y10000");  //this is a less than ideal starting location
  mtx_toppulse.lock();
  if(!fnPerformaxComSendRecv(ARH_top, write, 64,64, read)){
    std::cout << "Could not pulse TopY (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return false;
  }	
  mtx_toppulse.unlock();
  while(1){
    usleep(500000);
    strcpy(write, "MSTY");
    mtx_toppulse.lock();
    if(!fnPerformaxComSendRecv(ARH_top, write, 64,64, read)){
      std::cout << "Could not check TopY motor status (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
      return false;
    }
    mtx_toppulse.unlock();
    if( strcmp(read,  "0") == 0 ){break;}
    if( strcmp(read, "16") == 0 ){break;}
    if( strcmp(read, "32") == 0 ){break;}
    if( strcmp(read, "64") == 0 ){break;}
    if( strcmp(read, "144") == 0 ){break;}
    if( strcmp(read, "288") == 0 ){break;}
  }

  mtx_stepcount.lock();
  topy_stepcount = 0;
  mtx_stepcount.unlock();
}




bool HomeBtmX(){

  char read[64];
  char write[64];

  strcpy(write, "X74000");
  mtx_btmpulse.lock();
  if(!fnPerformaxComSendRecv(ARH_btm, write, 64,64, read)){
    std::cout << "Could not pulse BottomX (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return false;
  }	
  mtx_btmpulse.unlock();
  while(1){
    usleep(500000);
    strcpy(write, "MSTX");
    mtx_btmpulse.lock();
    if(!fnPerformaxComSendRecv(ARH_btm, write, 64,64, read)){
      std::cout << "Could not check BottomX motor status (homing)\n";
      mtx_done.lock();
      bool_done = true;
      mtx_done.unlock();
      return false;
    }
    mtx_btmpulse.unlock();
    if( strcmp(read,  "0") == 0 ){break;}
    if( strcmp(read, "16") == 0 ){break;}
    if( strcmp(read, "32") == 0 ){break;}
    if( strcmp(read, "64") == 0 ){break;}
    if( strcmp(read, "144") == 0 ){break;}
    if( strcmp(read, "288") == 0 ){break;}
  }

  strcpy(write, "X-36578");
  mtx_btmpulse.lock();
  if(!fnPerformaxComSendRecv(ARH_btm, write, 64,64, read)){
    std::cout << "Could not pulse BottomX (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return false;
  }	
  mtx_btmpulse.unlock();
  while(1){
    usleep(500000);
    strcpy(write, "MSTY");
    mtx_btmpulse.lock();
    if(!fnPerformaxComSendRecv(ARH_btm, write, 64,64, read)){
      std::cout << "Could not check BottomX motor status (homing)\n";
      mtx_done.lock();
      bool_done = true;
      mtx_done.unlock();
      return false;
    }
    mtx_btmpulse.unlock();
    if( strcmp(read,  "0") == 0 ){break;}
    if( strcmp(read, "16") == 0 ){break;}
    if( strcmp(read, "32") == 0 ){break;}
    if( strcmp(read, "64") == 0 ){break;}
    if( strcmp(read, "144") == 0 ){break;}
    if( strcmp(read, "288") == 0 ){break;}
  }

  mtx_stepcount.lock();
  btmx_stepcount = 0;
  mtx_stepcount.unlock();
}






bool HomeBtmY(){

  char read[64];
  char write[64];

  strcpy(write, "Y-74000");
  mtx_btmpulse.lock();
  if(!fnPerformaxComSendRecv(ARH_btm, write, 64,64, read)){
    std::cout << "Could not pulse BtmY (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return false;
  }	
  mtx_btmpulse.unlock();
  while(1){
    usleep(500000);
    strcpy(write, "MSTY");
    mtx_btmpulse.lock();
    if(!fnPerformaxComSendRecv(ARH_btm, write, 64,64, read)){
      std::cout << "Could not check BtmY motor status (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
      return false;
    }
    mtx_btmpulse.unlock();
    if( strcmp(read,  "0") == 0 ){break;}
    if( strcmp(read, "16") == 0 ){break;}
    if( strcmp(read, "32") == 0 ){break;}
    if( strcmp(read, "64") == 0 ){break;}
    if( strcmp(read, "144") == 0 ){break;}
    if( strcmp(read, "288") == 0 ){break;}
  }

  strcpy(write, "Y78");
  mtx_btmpulse.lock();
  if(!fnPerformaxComSendRecv(ARH_btm, write, 64,64, read)){
    std::cout << "Could not pulse BtmY (homing)\n";
    mtx_done.lock();
    bool_done = true;
    mtx_done.unlock();
    return false;
  }	
  mtx_btmpulse.unlock();
  while(1){
    usleep(500000);
    strcpy(write, "MSTY");
    mtx_btmpulse.lock();
    if(!fnPerformaxComSendRecv(ARH_btm, write, 64,64, read)){
      std::cout << "Could not check BtmY motor status (homing)\n";
      mtx_done.lock();
      bool_done = true;
      mtx_done.unlock();
      return false;
    }
    mtx_btmpulse.unlock();
    if( strcmp(read,  "0") == 0 ){break;}
    if( strcmp(read, "16") == 0 ){break;}
    if( strcmp(read, "32") == 0 ){break;}
    if( strcmp(read, "64") == 0 ){break;}
    if( strcmp(read, "144") == 0 ){break;}
    if( strcmp(read, "288") == 0 ){break;}
  }

  mtx_stepcount.lock();
  btmy_stepcount = 0;
  mtx_stepcount.unlock();
}






