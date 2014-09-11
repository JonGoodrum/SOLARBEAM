
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

const int topx_min_stepcount = -36500;  //right limit
const int topx_max_stepcount =  36500;  //left  limit
const int topy_min_stepcount =      0;  //upper limit
const int topy_max_stepcount =  18250;  //lower limit
const int btmx_min_stepcount = -36500;  //left  limit
const int btmx_max_stepcount =  36500;  //right limit
const int btmy_min_stepcount =      0;  //lower limit
const int btmy_max_stepcount =  36500;  //upper limit
int topx_stepcount = 0;
int topy_stepcount = 0;
int btmx_stepcount = 0;
int btmy_stepcount = 0;

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

char* ArduinoPort;
