//================================================================
// Created by João Vasco on 26/11/2021.
//================================================================
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include <atomic>
#include <string>
#include "include/utils.hpp" //Contains the ctrl+c handler
#include <fstream>

/*
 * Zed Stuff
 */
#include <sl/Camera.hpp>

const std::string rgbFileZed = "../dataOut/zed/rgbData.bin";
const std::string depthFileZed = "../dataOut/zed/depthData.bin";
const std::string depthRawFileZed = "../dataOut/zed/depthRawData.bin";

// Data structure to store the info about the frames
// This data is stored at the start of the binary file and
// MUST be the same on the save and read code!
typedef struct
{
    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int pixelBytes = 0;
    unsigned int step = 0;
    unsigned int widthBytes = 0;
} frameInfoZed;


/*
 * Intel Stuff
 */
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

const std::string rgbFileIntel = "../dataOut/intel/rgbData.bin";
const std::string depthFileIntel = "../dataOut/intel/depthData.bin";
const std::string depthRGBFileIntel = "../dataOut/intel/depthRGBData.bin";


// Data structure to store the info about the frames
// This data is stored at the start of the binary file and
// MUST be the same on the save and read code!
typedef struct
{
    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int pixelBytes = 0;
    unsigned int strideBytes = 0;
    unsigned int dataSize = 0;
} frameInfoIntel;


/*
 * Motive Stuff
 */

const std::string motiveDataFile = "../dataOut/motive/motive.json";

/*
 * Timing const so we now when to capture each frame
 * If we want a final output with 30 fps we need to
 * capture 30 frames per second ie one each 1000/30 milliseconds
 */
const int fpsOutput = 30;
const int millisBetweenFrames = 1000 / fpsOutput;

std::atomic<long> zedFrameCount;
std::atomic<long> intelFrameCount;
std::atomic<long> motiveFrameCount;


typedef struct
{
    std::fstream rgb;
    std::fstream depthRGB;
    std::fstream depthRAW;
} files;


/* Aux functions declaration */
void threadTimer(std::function<void(std::fstream &, std::fstream &, std::fstream &)> inputFunctionZed,
                 std::function<void(std::fstream &, std::fstream &, std::fstream &)> inputFunctionIntel,
                 unsigned int interval,
                 std::fstream &rgbFileZed, std::fstream &depthFileZed, std::fstream &depthRGBFileZed,
                 std::fstream &rgbFileIntel, std::fstream &depthFileIntel, std::fstream &depthRGBFileIntel);

void intelFrameCapture(std::fstream &rgbFile, std::fstream &depthFile, std::fstream &depthRGBFile);
void zedFrameCapture(std::fstream &rgbFile, std::fstream &depthFile, std::fstream &depthRGBFile);
void motiveFrameCapture();


int main()
{
  frameInfoIntel intelInfo;
  frameInfoZed zedInfo;

  files intel;
  files zed;

  std::cout << "[INFO] - Final FPS: " << fpsOutput << std::endl;
  std::cout << "[INFO] - Time between frames: " << millisBetweenFrames << std::endl;



  /*
   * Intel start stuff
   * Create the files the cam writes to
   * Start te cam with the settings
   * - 720p
   * - 30 fps
   * - color map black to white
   * */


  intel.rgb.open(rgbFileIntel, std::ios::out | std::ios::binary);
  intel.depthRAW.open(depthFileIntel, std::ios::out | std::ios::binary);
  intel.depthRGB.open(depthRGBFileIntel, std::ios::out | std::ios::binary);





  system("pause"); //Wait for the user press enter to start
  auto programBegin = std::chrono::high_resolution_clock::now();
  //Capture Stuff
  //This program block is timed
  //Put the cameras capturing in the background
  threadTimer(zedFrameCapture, intelFrameCapture, millisBetweenFrames, zedCam, pipe, intelColorMap, rgb, depth,
              depthRGB);

  //End Capture Stuff
  SetCtrlHandler(); //Capture CTRL + C so we know when to exit
  while (!exit_app); //Wait unitl we want to leave the app
  auto programEnd = std::chrono::high_resolution_clock::now();

  //Calculate the time the program was capturing frames
  long long wallTime = std::chrono::duration_cast<std::chrono::milliseconds>(programEnd - programBegin).count();

  //======================
  //= App shutdown stuff =
  //======================

  //Wait for 2 seconds and let other threads to finish before closing
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  int zedFPS = static_cast<int>(zedFrameCount / wallTime);
  int intelFPS = static_cast<int>(intelFrameCount / wallTime);
  int motiveFPS = static_cast<int>(motiveFrameCount / wallTime);

  //Print some information and wait some time so we can read it
  std::cout << "===================================" << std::endl;
  std::cout << "[INFO ZED] - Frames Capturados: " << zedFrameCount << std::endl;
  std::cout << "[INFO ZED] - FPS: " << zedFPS << std::endl;
  std::cout << "[INFO INTEL] - Frames Capturados: " << intelFrameCount << std::endl;
  std::cout << "[INFO INTEL] - FPS: " << intelFPS << std::endl;
  std::cout << "[INFO MOTIVE] - Frames Capturados: " << motiveFrameCount << std::endl;
  std::cout << "[INFO MOTIVE] - FPS: " << motiveFPS << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(10000));

  return 0;
} //End main()
/*======================================================================================================================
 *====================================================================================================================*/
void threadTimer(std::function<void(sl::Camera & )> inputFunctionZed,
                 std::function<void(rs2::pipeline, rs2::colorizer)> inputFunctionIntel,
                 unsigned int interval, sl::Camera &zedObject, rs2::pipeline &inputPipe,
                 rs2::colorizer &inputColorizer)
{
  std::thread([inputFunctionZed, inputFunctionIntel, interval, &zedObject, &inputPipe, &inputColorizer]()
              {

              }).detach();
}
