#include <thread>
#include <mutex>
#include <unistd.h>
#include <iostream>
#include <string>
#include <string.h>
#include <stdio.h>

bool PulseAxis( int pulse_width,            //width of the steps the gimbal takes (any POSITIVE int value)
                int which_gimbal_which_axis ) //value: (1=TopX 2=TopY 3=BtmX 4=BtmY)
  {

  //used to read the gimbal's output string
  string strpos;
  string strneg;
  char read[64];
  char write[64];
  char status[64];
  char pulse_error[64];
  char read_error[64];
  char ch_pospulse[64];
  char ch_negpulse[64];
  int  poslimit; 
  int  neglimit;
  int* intptr_stepcount;

  std::mutex* mtx_posaxis;
  std::mutex* mtx_negaxis;
  std::mutex* mtx_gimbal;
  AR_HANDLE* ARH_gimbal;


  if(which_gimbal_which_axis==1){ //initialize things specific to the TopX axis
    strcpy(status, "MSTX");
    intptr_stepcount = &topx_stepcount;
    poslimit = topx_max_stepcount;
    neglimit = topx_min_stepcount;
    strcpy(pulse_error, "ERROR: cannot pulse TOPX\n");
    strcpy(read_error, "ERROR: cannot read TOPX\n");
    strpos = "X" + std::to_string(pulse_width);
    strneg = "X" + std::to_string(-pulse_width);
    strcpy( ch_pospulse, strpos.c_str());
    strcpy( ch_negpulse, strneg.c_str());
    mtx_posaxis = &mtx_topxl;
    mtx_negaxis = &mtx_topxr;
    mtx_gimbal = &mtx_toppulse;
    ARH_gimbal = &ARH_top;
  }
  if(which_gimbal_which_axis==2){ //initialize things specific to the TopY axis
    strcpy(status, "MSTY");
    intptr_stepcount = &topy_stepcount;
    poslimit = topy_max_stepcount;
    neglimit = topy_min_stepcount;
    strcpy(pulse_error, "ERROR: cannot pulse TOPY\n");
    strcpy(read_error, "ERROR: cannot read TOPY\n");
    strpos = "Y" + std::to_string(pulse_width);
    strneg = "Y" + std::to_string(-pulse_width);
    strcpy( ch_pospulse, strpos.c_str());
    strcpy( ch_negpulse, strneg.c_str());
    mtx_posaxis = &mtx_topyd;
    mtx_negaxis = &mtx_topyu;
    mtx_gimbal = &mtx_toppulse;
    ARH_gimbal = &ARH_top;
  }
  if(which_gimbal_which_axis==3){ //initialize things specific to the BottomX axis
    strcpy(status, "MSTX");
    intptr_stepcount = &btmx_stepcount;
    poslimit = btmx_max_stepcount;
    neglimit = btmx_min_stepcount;
    strcpy(pulse_error, "ERROR: cannot pulse BOTTOMX\n");
    strcpy(read_error, "ERROR: cannot read BOTTOMX\n");
    strpos = "X" + std::to_string(pulse_width);
    strneg = "X" + std::to_string(-pulse_width);
    strcpy( ch_pospulse, strpos.c_str());
    strcpy( ch_negpulse, strneg.c_str());
    mtx_posaxis = &mtx_btmxr;
    mtx_negaxis = &mtx_btmxl;
    mtx_gimbal = &mtx_btmpulse;
    ARH_gimbal = &ARH_btm;
  }
  if(which_gimbal_which_axis==4){ //initialize things specific to the BottomY axis
    strcpy(status, "MSTY");
    intptr_stepcount = &btmy_stepcount;
    poslimit = btmy_max_stepcount;
    neglimit = btmy_min_stepcount;
    strcpy(pulse_error, "ERROR: cannot pulse BOTTOMY\n");
    strcpy(read_error, "ERROR: cannot read BOTTOMY\n");
    strpos = "Y" + std::to_string(pulse_width);
    strneg = "Y" + std::to_string(-pulse_width);
    strcpy( ch_pospulse, strpos.c_str());
    strcpy( ch_negpulse, strneg.c_str());
    mtx_posaxis = &mtx_btmyu;
    mtx_negaxis = &mtx_btmyd;
    mtx_gimbal = &mtx_btmpulse;
    ARH_gimbal = &ARH_btm;
  }



  /*   Used to pulse the gimbal in the X direction.
   * This entire function is sent to a thread, and
   * the thread lives forever unless the main while(1)
   * is broken.  The global mutex (m_topx) determines
   * when this loop should run, and the global string
   * (ch_pulsetopx) decides which direction.  The string is
   * always updated before the mutex unlocks.
   *   The locking/unlocking outside the hash range
   * ensures that the ColorDetection functions don't
   * send too many pulse commands to the gimbal
   * (ex: if it's moving left, why send another move left command?)
   */

  while(1){//while(thread is active), wait for Camera/Sensor to tell us when to pulse.

        mtx_done.lock();
        if(bool_done){mtx_done.unlock(); break;}
        mtx_done.unlock();

      if(mtx_posaxis->try_lock()){//if(positive direction is free)
        //std::cout << "pulse acquired mtx_posaxis\n";
        mtx_done.lock();
        if(bool_done){mtx_done.unlock(); break;}
        mtx_done.unlock();
      //1) Pulse the gimbal (left or right)
      strcpy(write, ch_pospulse);
      mtx_gimbal->lock();
      //std::cout << "mtx_gimbal1 locked by posaxis\n";
      if(!fnPerformaxComSendRecv((*ARH_gimbal), write, 64, 64, read)){  
        std::cout << pulse_error;
        return false;
      }
      mtx_gimbal->unlock();
      //std::cout << "mtx_gimbal1 unlocked by posaxis\n";

      //update steps
      
      mtx_stepcount.lock();
      if( (*intptr_stepcount) < poslimit ){  (*intptr_stepcount)+= pulse_width;  }
      mtx_stepcount.unlock();




      //2) Wait for gimbal to finish moving
      while(1){//while(gimbal is moving), do 2 things:
        mtx_done.lock();
        if(bool_done){mtx_done.unlock(); break;}
        mtx_done.unlock();
        //2a) check motor status
        strcpy(write, status);
        mtx_gimbal->lock();
        //std::cout << "mtx_gimbal2 locked by posaxis\n";
        if(!fnPerformaxComSendRecv((*ARH_gimbal), write, 64, 64, read)){  
          std::cout << read_error;
          return false;
        }
        mtx_gimbal->unlock();
        //std::cout << "mtx_gimbal2 unlocked by posaxis\n";
        //2b) if motor is still, break.  Otherwise, repeat 3a.
        //std::cout << "M: " << read << "\n";
        if( strcmp(read,  "0") == 0 ){break;} // These are MST return values that
        if( strcmp(read, "16") == 0 ){break;} // indicate whether or not the motor is still.
        if( strcmp(read, "32") == 0 ){break;} // If the gimbal is still, break loop.
        if( strcmp(read, "64") == 0 ){break;}
        if( strcmp(read, "144") == 0 ){break;}
        if( strcmp(read, "288") == 0 ){break;}
        usleep(100);
      }//end while(gimbal is moving)

      //3) Wait for camera threads to regain control
      mtx_posaxis->unlock();//allow Camera X to grab our lock
      //std::cout << "mtx_posaxis->unlock(); UPPER\n";
      //usleep(10000);
      while(1){//wait for CameraX to grab our lock (so this doesn't accidentally pulse twice when it loops to the top)
        mtx_done.lock();
        if(bool_done){mtx_done.unlock(); break;}
        mtx_done.unlock();
        if(!mtx_posaxis->try_lock()){break;}//if you cannot grab the lock, CameraX must be ready.  Break and loop to top.
        mtx_posaxis->unlock();//if you can grab the lock, release and try again.  CameraX must not be ready.
        //std::cout << "mtx_posaxis->unlock() LOWER; \n";
      }
    }//end if(left is free)


    if(mtx_negaxis->try_lock()){//if(right is free)
        mtx_done.lock();
        if(bool_done){mtx_done.unlock(); break;}
        mtx_done.unlock();
      //1) Pulse the gimbal right
      strcpy(write, ch_negpulse);
      mtx_gimbal->lock();
      //std::cout << "mtx_gimbal1 locked by negaxis\n";
      if(!fnPerformaxComSendRecv((*ARH_gimbal), write, 64, 64, read)){  
        std::cout << pulse_error;
        return false;
      }
      mtx_gimbal->unlock();
      //std::cout << "mtx_gimbal1 unlocked by negaxis\n";

      //update steps
      
      mtx_stepcount.lock();
      if( (*intptr_stepcount) > neglimit){  (*intptr_stepcount)-=pulse_width;  }
      mtx_stepcount.unlock();




      //2) Wait for gimbal to finish moving
      while(1){//while(gimbal is moving), do 2 things:
        mtx_done.lock();
        if(bool_done){mtx_done.unlock(); break;}
        mtx_done.unlock();
        //2a) check motor status
        strcpy(write, status);
        mtx_gimbal->lock();
        //std::cout << "mtx_gimbal2 locked by negaxis\n";
        if(!fnPerformaxComSendRecv((*ARH_gimbal), write, 64, 64, read)){  
          std::cout << read_error;
          return false;
        }
        mtx_gimbal->unlock();
        //std::cout << "mtx_gimbal2 unlocked by negaxis\n";
        //2b) if motor is still, break.  Otherwise, repeat 3a.
        //std::cout << "M: " << read << "\n";
        if( strcmp(read,  "0") == 0 ){break;} // These are MST return values that
        if( strcmp(read, "16") == 0 ){break;} // indicate whether or not the motor is still.
        if( strcmp(read, "32") == 0 ){break;} // If the gimbal is still, break loop.
        if( strcmp(read, "64") == 0 ){break;}
        if( strcmp(read, "144") == 0 ){break;}
        if( strcmp(read, "288") == 0 ){break;}
        usleep(100);//sleep for a bit
      }//end while(gimbal is moving)

      //3) Wait for camera threads to regain control
      mtx_negaxis->unlock();//allow Camera X to grab our lock
      //std::cout << "mtx_negaxis->unlock(); UPPER\n";
      //usleep(10000);
      while(1){//wait for CameraX to grab our lock (so this doesn't accidentally pulse twice when it loops to the top)
        mtx_done.lock();
        if(bool_done){mtx_done.unlock(); break;}
        mtx_done.unlock();
        if(!mtx_negaxis->try_lock()){break;}//if you cannot grab the lock, CameraX must be ready.  Break and loop to top.
        mtx_negaxis->unlock();//if you can grab the lock, release and try again.  CameraX must not be ready.
        //std::cout << "mtx_posaxis->unlock() LOWER; \n";
      }
    }//end if(right is free)

    mtx_done.lock();
    if(bool_done){mtx_done.unlock(); break;}
    mtx_done.unlock();

    
    if((which_gimbal_which_axis==3) or (which_gimbal_which_axis==4)){
      mtx_done.lock();
      if(suntracking_finished){mtx_done.unlock(); break;}
      mtx_done.unlock();
    }
  }//end while(thread is active)

  /*
  mtx_done.lock();
  bool_done = true;
  mtx_done.unlock();
  */
  std::cout << "Pulse thread done\n";
}//end PulseAxis


