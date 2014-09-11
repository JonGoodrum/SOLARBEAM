extern "C"{
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
} 
#include <iostream>
/* My Arduino is on /dev/ttyACM0 */
char buf[1];
 
bool SunTracker(){

  mtx_btmxl.lock();
  mtx_btmxr.lock();
  mtx_btmyu.lock();
  mtx_btmyd.lock();



  int fd;
 
  /* Open the file descriptor in non-blocking mode */
  fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
  //fd = open(ArduinoPort, O_RDWR | O_NOCTTY);

 
  /* Set up the control structure */
  struct termios toptions;
 
  /* Get currently set options for the tty */
  tcgetattr(fd, &toptions);
 
  /* Set custom options */
 
  /* 9600 baud */
  cfsetispeed(&toptions, B9600);
  cfsetospeed(&toptions, B9600);
  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  /* no hardware flow control */
  toptions.c_cflag &= ~CRTSCTS;
  /* enable receiver, ignore status lines */
  toptions.c_cflag |= CREAD | CLOCAL;
  /* disable input/output flow control, disable restart chars */
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
  /* disable canonical input, disable echo,
  disable visually erase chars,
  disable terminal-generated signals */
  toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /* disable output processing */
  toptions.c_oflag &= ~OPOST;
 
  /* wait for XXX characters to come in before read returns */
  toptions.c_cc[VMIN] = 1;
  /* no minimum time to wait before read returns */
  toptions.c_cc[VTIME] = 0;
 
  /* commit the options */
  tcsetattr(fd, TCSANOW, &toptions);
 
  int tmp = 100;
  int bufarray[tmp]; //where buf values get stored.  Need one nonzero
  bufarray[0] = 69;
  int count = 0;  //used to determining which array element to erase
  int ibuf;  //buf, converted to int via atoi(buf)
  int sum = 0;
  bool fix_azimuth = false;

 
  while(1){
 
    //housekeeping stuff
    //count++;
    //if(count == tmp){count = 0; }
   
    /* Wait for the Arduino to reset */
    usleep(1000);
    /* read 1 char from the fd, store it in buf */
    int n = read(fd, buf, 1);
    //convert buf to an int
    ibuf = atoi(buf);
    //add ibuf to the array
    if(ibuf!=0){
      count++;
      if(count == tmp){count = 0; }
      bufarray[count] = ibuf;
    }
   
    //std::cout << "SUNTRACKER AT LRUD LOCKING PART\n"; 



    if(ibuf == 9){  std::cout << ibuf << "!\n"; /*DO NOTHING*/ }
    if(ibuf == 0){  std::cout << ibuf << "!\n"; /*DO NOTHING*/ }

   
    if(ibuf == 1){ //move left 
      std::cout << ibuf << "!\n";
     mtx_btmxl.unlock();
     //usleep(1000);
     while(1){
       mtx_done.lock();
       if(bool_done){mtx_done.unlock(); break;}
       mtx_done.unlock();
       if(!mtx_btmxl.try_lock()){break;}
       mtx_btmxl.unlock();
       //usleep(1000);
     }
     mtx_btmxl.lock();
     std::cout << ibuf << " done\n";
   }
   
   
   
   if(ibuf == 2){ //move up
     std::cout << ibuf << "!\n";
     mtx_btmyu.unlock();
     //usleep(1000);
     while(1){
       mtx_done.lock();
       if(bool_done){mtx_done.unlock(); break;}
       mtx_done.unlock();
       if(!mtx_btmyu.try_lock()){break;}
       mtx_btmyu.unlock();
       //usleep(1000);
     }
     mtx_btmyu.lock();
     std::cout << ibuf << " done\n";
   }

   
   if(ibuf == 3){ //move right
     std::cout << ibuf << "!\n";
     mtx_btmxr.unlock();
     //usleep(1000);
     while(1){
       mtx_done.lock();
       if(bool_done){mtx_done.unlock(); break;}
       mtx_done.unlock();
       if(!mtx_btmxr.try_lock()){break;}
       mtx_btmxr.unlock();
       //usleep(1000);
     }
     mtx_btmxr.lock();
     std::cout << ibuf << " done\n";
   }

   
   if(ibuf == 4){ //move down
     std::cout << ibuf << "!\n";
     mtx_btmyd.unlock();
     //usleep(1000);
     while(1){
       mtx_done.lock();
       if(bool_done){mtx_done.unlock(); break;}
       mtx_done.unlock();
       if(!mtx_btmyd.try_lock()){break;}
       mtx_btmyd.unlock();
       //usleep(1000);
     }
     mtx_btmyd.lock();
     std::cout << ibuf << " done\n";
   }
   
   
   
   if(ibuf == 5){ std::cout << "LOW sensor inactive\n"; break;}
   if(ibuf == 6){ std::cout << "FLOAT sensor inactive\n"; break;}
   if(ibuf == 7){ std::cout << "HIGH sensor inactive\n"; break;}
   //else { std::cout << "ERROR: no arduino connected (or wrong COM port)\n"; break;}
   
   if( (ibuf==9) or (ibuf==1) or (ibuf==2) or (ibuf==3) or (ibuf==4) ){
     //sum the array to see if all nines
     sum = 0;
     for(int i=0; i<tmp; ++i){
       sum+=bufarray[i];
     }
     if(sum==(9*tmp)){fix_azimuth = true; break;}
   } 
 }
 
  if(fix_azimuth){
    /* ADJUST PITCH OF MIRROR TO REFLECT UP */
    string tempstr;
    char write[64];
    char read[64];
    int tempint = (18250 - 3/2*btmy_stepcount);
    //now, add this to stepcount
    btmy_stepcount+=tempint;
    //parse a string with this
    tempstr = "Y" + std::to_string(tempint);
    //now, pulse the ARH_btm in this direction
    strcpy(write, tempstr.c_str());
    if(!fnPerformaxComSendRecv(ARH_btm, write, 64,64, read)){
      std::cout << "could not fix azimuth\n";
      mtx_done.lock();
      bool_done = true;
      mtx_done.unlock();
    }else{
      while(1){
        strcpy(write, "MSTY");
        if(!fnPerformaxComSendRecv(ARH_btm, write, 64,64, read)){
          std::cout << "could not read motor status after fixing azimuth\n";
          mtx_done.lock();
          bool_done = true;
          mtx_done.unlock();
        }
        if( strcmp(read,  "0") == 0 ){break;} // These are MST return values that
        if( strcmp(read, "16") == 0 ){break;} // indicate whether or not the motor is still.
        if( strcmp(read, "32") == 0 ){break;} // If the gimbal is still, break loop.
        if( strcmp(read, "64") == 0 ){break;}
        if( strcmp(read, "144") == 0 ){break;}
        if( strcmp(read, "288") == 0 ){break;}
        usleep(100);
      }    
      std::cout << "fixed azimuth\n";
    }
  }//end if(fix_azimuth)


}//end void SunTracker



int InitializeArduino(){
  mtx_done.lock();
  if(bool_done){mtx_done.unlock(); return 1;}
  mtx_done.unlock();
  char temp[64];
  std::cout << "Upload and run the 'activate_sunsensors.ino' Arduino sketch.\n"
    << "  When you do, look at the bottom left corner and read the\n"
    << "  serial port it's connected to (ex: /dev/ttyACM0).\n"
    << "      Type in the serial port now: ";
  std::cin >> temp;
  strcpy(ArduinoPort, temp);
  return 0;
}
