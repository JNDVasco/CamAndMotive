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
const std::string depthRGBFileZed = "../dataOut/zed/depthRGBData.bin";

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
#include <vector>

#include "NatNetSDK/include/NatNetTypes.h"
#include "NatNetSDK/include/NatNetCAPI.h"
#include "NatNetSDK/include/NatNetClient.h"

void _WriteFrame(FILE *fp, sFrameOfMocapData *data);

const std::string motiveDataFile = "../dataOut/motive/motive.json";

void NATNET_CALLCONV DataHandler(sFrameOfMocapData *data, void *pUserData);    // receives data from the server
void resetClient();
int ConnectClient();


static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;
sNatNetClientConnectParams g_connectParams;
sServerDescription g_serverDescription;

NatNetClient *g_pClient = NULL;
FILE *g_outputFile;

// Data structure to store the info about the frames
// This data is stored at the start of the binary file and
// MUST be the same on the save and read code!
typedef struct
{
    unsigned int frame = 0;
    std::string bone;
    float xx = 0;
    float yy = 0;
    float zz = 0;
} frameInfoMotive;

frameInfoMotive infoMotive;

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
void
threadTimerIntel(const std::function<void(files &, const rs2::pipeline, const rs2::colorizer)> &, files &inputFiles,
                 rs2::pipeline inputPipe, rs2::colorizer inputColorizer, unsigned int interval);
void threadTimerZed(const std::function<void(files &, sl::Camera &zedObject, sl::Resolution)> &, files &inputFiles,
                    sl::Camera &zedObject, sl::Resolution, unsigned int interval);

void
threadTimerMotive(std::function<void(sFrameOfMocapData, void *)>, sFrameOfMocapData , void *, unsigned int interval);

void intelFrameCapture(files &, const rs2::pipeline &inputPipe, const rs2::colorizer &inputColorizer);
void zedFrameCapture(files &, sl::Camera &zedObject, sl::Resolution);
void motiveFrameCapture(sFrameOfMocapData, void *);


int main()
{
  /*
   * Structure to hold the information about the captured data
   */

  frameInfoIntel intelDepthRGBInfo;
  frameInfoIntel intelDepthRawInfo;
  frameInfoIntel intelRgbInfo;

  frameInfoZed zedDepthRGBInfo;
  frameInfoZed zedDepthRawInfo;
  frameInfoZed zedRgbInfo;

  /*
   * Open Files for both cameras
   */
  files intelFile;
  files zedFile;

  intelFile.rgb.open(rgbFileIntel, std::ios::out | std::ios::binary);
  intelFile.depthRAW.open(depthFileIntel, std::ios::out | std::ios::binary);
  intelFile.depthRGB.open(depthRGBFileIntel, std::ios::out | std::ios::binary);
  zedFile.rgb.open(rgbFileZed, std::ios::out | std::ios::binary);
  zedFile.depthRAW.open(depthFileZed, std::ios::out | std::ios::binary);
  zedFile.depthRGB.open(depthRGBFileZed, std::ios::out | std::ios::binary);

  if (!intelFile.rgb.is_open() || !intelFile.depthRGB.is_open() || !intelFile.depthRAW.is_open() ||
      !zedFile.rgb.is_open() || !zedFile.depthRGB.is_open() || !zedFile.depthRAW.is_open())
  {
    std::cout << "Failed while opening files" << std::endl;
    intelFile.rgb.close();
    intelFile.depthRAW.close();
    intelFile.depthRGB.close();
    zedFile.rgb.close();
    zedFile.depthRAW.close();
    zedFile.depthRGB.close();
    return -1;
  }

  /* Print some info */
  std::cout << "[INFO] - Final FPS: " << fpsOutput << std::endl;
  std::cout << "[INFO] - Time between frames: " << millisBetweenFrames << std::endl;

  /*
   * Intel start stuff
   * Create the files the cam writes to
   * Start te cam with the settings
   *  - RGB Color stream at 1280x720 @ 60fps
   *  - Depth Stream at 1280x720 @ 60fps
   *  and disable all others since we don't need them
   */
  std::cout << "[INFO] - Starting Intel" << std::endl;
  rs2::config config;
  config.disable_all_streams();
  config.enable_stream(rs2_stream::RS2_STREAM_COLOR, 1280, 720, rs2_format::RS2_FORMAT_RGB8, 30);
  config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 1280, 720, rs2_format::RS2_FORMAT_Z16, 30);

  rs2::colorizer color_map(3);
  std::cout << "[INFO] - Starting Intel 1" << std::endl;
  rs2::pipeline intelPipe;
  std::cout << "[INFO] - Starting Intel 2" << std::endl;
  intelPipe.start(config); //config
  std::cout << "[INFO] - Starting Intel 3" << std::endl;
  /*
   * This is, supposedly, to let the auto exposure settle a bit
   * Not sure if it really does something or not
   */

  for (auto i = 0; i < 30; ++i)
  {
    intelPipe.wait_for_frames();
  }

  /*
   * This is where we write the file start with info about the frame we will collect
   */

  for (auto &&dataFrame: intelPipe.wait_for_frames())
  {
    auto videoFrame = dataFrame.as<rs2::video_frame>();

    if (videoFrame.is<rs2::depth_frame>())
    {
      intelDepthRawInfo.width = videoFrame.get_width();
      intelDepthRawInfo.height = videoFrame.get_height();
      intelDepthRawInfo.strideBytes = videoFrame.get_stride_in_bytes();
      intelDepthRawInfo.pixelBytes = videoFrame.get_bytes_per_pixel();
      intelDepthRawInfo.dataSize = videoFrame.get_data_size();

      videoFrame = color_map.process(dataFrame);

      intelDepthRGBInfo.width = videoFrame.get_width();
      intelDepthRGBInfo.height = videoFrame.get_height();
      intelDepthRGBInfo.strideBytes = videoFrame.get_stride_in_bytes();
      intelDepthRGBInfo.pixelBytes = videoFrame.get_bytes_per_pixel();
      intelDepthRGBInfo.dataSize = videoFrame.get_data_size();
    }
    else
    {
      intelRgbInfo.width = videoFrame.get_width();
      intelRgbInfo.height = videoFrame.get_height();
      intelRgbInfo.strideBytes = videoFrame.get_stride_in_bytes();
      intelRgbInfo.pixelBytes = videoFrame.get_bytes_per_pixel();
      intelRgbInfo.dataSize = videoFrame.get_data_size();
    }


  }

  /*
   * Save the structures to the files
   */

  intelFile.depthRAW.write(reinterpret_cast<char *>(&intelDepthRawInfo), sizeof(intelDepthRawInfo));
  intelFile.depthRGB.write(reinterpret_cast<char *>(&intelDepthRGBInfo), sizeof(intelDepthRGBInfo));
  intelFile.rgb.write(reinterpret_cast<char *>(&intelRgbInfo), sizeof(intelRgbInfo));

  std::cout << "[INFO] - Started Intel" << std::endl;

  /*
   * End Intel start
   */

  /*
   * ZED start stuff
   * Create the files the cam writes to
   * Start te cam with the settings
   * - 720p
   * - 60 fps
   * - Quality Depth
   * */
  std::cout << "[INFO] - Starting Zed" << std::endl;

  sl::Camera zedCam;

  sl::InitParameters init_parameters;
  init_parameters.camera_resolution = sl::RESOLUTION::HD720;
  init_parameters.camera_fps = 30;
  init_parameters.depth_mode = sl::DEPTH_MODE::QUALITY;

  auto returned_state = zedCam.open(init_parameters);

  if (returned_state != sl::ERROR_CODE::SUCCESS)
  {
    std::cout << "Error " << returned_state << ", exit program." << std::endl;
    return EXIT_FAILURE;
  }


  /*
   * Get the resolution of the camera
   * Create a matrix that holds the RGB data and print some information about it
   * Create a matrix that holds the depth data and print some information about it
   * Create a matrix that holds the depth data and print some information about it
   */
  sl::Resolution imageSize = zedCam.getCameraInformation().camera_configuration.resolution;

  sl::Mat rgbData(imageSize, sl::MAT_TYPE::U8_C4);
  sl::Mat depthData(imageSize, sl::MAT_TYPE::U8_C4);
  sl::Mat depthRawData(imageSize, sl::MAT_TYPE::F32_C4);

  zedRgbInfo.width = rgbData.getWidth();
  zedRgbInfo.height = rgbData.getHeight();
  zedRgbInfo.pixelBytes = rgbData.getPixelBytes();
  zedRgbInfo.step = rgbData.getStepBytes();
  zedRgbInfo.widthBytes = rgbData.getWidthBytes();

  zedDepthRGBInfo.width = depthData.getWidth();
  zedDepthRGBInfo.height = depthData.getHeight();
  zedDepthRGBInfo.pixelBytes = depthData.getPixelBytes();
  zedDepthRGBInfo.step = depthData.getStepBytes();
  zedDepthRGBInfo.widthBytes = depthData.getWidthBytes();

  zedDepthRawInfo.width = depthRawData.getWidth();
  zedDepthRawInfo.height = depthRawData.getHeight();
  zedDepthRawInfo.pixelBytes = depthRawData.getPixelBytes();
  zedDepthRawInfo.step = depthRawData.getStepBytes();
  zedDepthRawInfo.widthBytes = depthRawData.getWidthBytes();

  /*
   * Save the information of the frames
   */
  zedFile.depthRGB.write(reinterpret_cast<char *>(&zedDepthRGBInfo), sizeof(zedDepthRGBInfo));
  zedFile.depthRAW.write(reinterpret_cast<char *>(&zedDepthRawInfo), sizeof(zedDepthRawInfo));
  zedFile.rgb.write(reinterpret_cast<char *>(&zedRgbInfo), sizeof(zedRgbInfo));

  std::cout << "[INFO] - Started Zed" << std::endl;

  /*
   * End Zed start
   */

  /*
  * Motive
  */

  std::cout << "[INFO] - Starting Motive" << std::endl;

  // create NatNet client
  g_pClient = new NatNetClient();
  // set the frame callback handler
  g_pClient->SetFrameReceivedCallback(DataHandler, g_pClient);  // this function will receive data from the server
  // print version info
  unsigned char ver[4];
  NatNet_GetVersion(ver);
  printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

  // Give server address
  g_connectParams.serverAddress = "10.11.65.208";   // IP do Motive
  g_connectParams.localAddress = "10.11.65.140";     // IP do Computador

  // Connect to Motive
  int iResult;
  iResult = ConnectClient();
  if (iResult != ErrorCode_OK)
  {
    printf("Error initializing client. See log for details. Exiting.\n");
    return 1;
  }
  else
  {
    printf("Client initialized and ready.\n");
  }


  // Retrieve Data Descriptions from Motive
  printf("\n\n[SampleClient] Requesting Data Descriptions...\n");
  sDataDescriptions *pDataDefs = NULL;
  iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
  if (iResult != ErrorCode_OK || pDataDefs == NULL)
  {
    printf("[SampleClient] Unable to retrieve Data Descriptions.\n");
  }
  else
  {
    for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
    {
      if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
      {
        // Skeleton
        sSkeletonDescription *pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
        printf("Skeleton Name : %s\n", pSK->szName);
        printf("Skeleton ID : %d\n", pSK->skeletonID);
        printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
        for (int j = 0; j < pSK->nRigidBodies; j++)
        {
          sRigidBodyDescription *pRB = &pSK->RigidBodies[j];
          printf("  RigidBody Name : %s\n", pRB->szName);
          printf("  RigidBody ID : %d\n", pRB->ID);
          printf("  RigidBody Parent ID : %d\n", pRB->parentID);
          printf("  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
        }
      }
      else
      {
        printf("Unknown data type.\n");
        // Unknown
      }
    }
  }

  std::cout << "[INFO] - Started Motive" << std::endl;

  //auto dataMotive = new sFrameOfMocapData;

  sFrameOfMocapData dataMotive;

  system("pause"); //Wait for the user press enter to start
  auto programBegin = std::chrono::high_resolution_clock::now();
  //Capture Stuff
  //This program block is timed
  //Put the cameras capturing in the background
  threadTimerIntel(intelFrameCapture, intelFile, intelPipe, color_map, millisBetweenFrames);
  threadTimerZed(zedFrameCapture, zedFile, zedCam, imageSize, millisBetweenFrames);

  for (int i = 0; i < 20; ++i)
  {
    DataHandler(&dataMotive, g_pClient);

    iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
    {
      printf("[SampleClient] Unable to retrieve Data Descriptions.\n");
    }
    else
    {
      for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
      {
        if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
        {
          // Skeleton
          sSkeletonDescription *pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
          printf("Skeleton Name : %s\n", pSK->szName);
          printf("Skeleton ID : %d\n", pSK->skeletonID);
          printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
          for (int j = 0; j < pSK->nRigidBodies; j++)
          {
            sRigidBodyDescription *pRB = &pSK->RigidBodies[j];
            printf("  RigidBody Name : %s\n", pRB->szName);
            printf("  RigidBody ID : %d\n", pRB->ID);
            printf("  RigidBody Parent ID : %d\n", pRB->parentID);
            printf("  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
          }
        }
        else
        {
          printf("Unknown data type.\n");
          // Unknown
        }
      }
    }
  }




  // threadTimerMotive(motiveFrameCapture, dataMotive, g_pClient, millisBetweenFrames);
  //threadTimerMotive();

  //End Capture Stuff
  SetCtrlHandler(); //Capture CTRL + C so we know when to exit
  long i = 0;
  while (!exit_app)
  {
    if (i >= 2000)
    {
      std::cout << "===================================" << std::endl;
      std::cout << "[INFO ZED] - Frames Capturados: " << zedFrameCount << std::endl;
      std::cout << "[INFO INTEL] - Frames Capturados: " << intelFrameCount << std::endl;
      std::cout << "[INFO MOTIVE] - Frames Capturados: " << motiveFrameCount << std::endl;
      i = 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    i++;
  }; //Wait unitl we want to leave the app
  auto programEnd = std::chrono::high_resolution_clock::now();

  //Calculate the time the program was capturing frames
  long long wallTime = std::chrono::duration_cast<std::chrono::milliseconds>(programEnd - programBegin).count();

  std::cout << "Wall: " << wallTime << std::endl;

  //======================
  //= App shutdown stuff =
  //======================

  //Wait for 2 seconds and let other threads to finish before closing
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  int zedFPS = static_cast<int>(zedFrameCount / (wallTime / 1000.0));
  int intelFPS = static_cast<int>(intelFrameCount / (wallTime / 1000.0));
  int motiveFPS = static_cast<int>(motiveFrameCount / (wallTime / 1000.0));

  //Print some information and wait some time so we can read it
  std::cout << "===================================" << std::endl;
  std::cout << "[INFO ZED] - Frames Capturados: " << zedFrameCount << std::endl;
  std::cout << "[INFO ZED] - FPS: " << zedFPS << std::endl;
  std::cout << "[INFO INTEL] - Frames Capturados: " << intelFrameCount << std::endl;
  std::cout << "[INFO INTEL] - FPS: " << intelFPS << std::endl;
  std::cout << "[INFO MOTIVE] - Frames Capturados: " << motiveFrameCount << std::endl;
  std::cout << "[INFO MOTIVE] - FPS: " << motiveFPS << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(10000));

  // Done - clean up Motive
  if (g_pClient)
  {
    g_pClient->Disconnect();
    delete g_pClient;
    g_pClient = NULL;
  }

  return 0;
} //End main()
/*======================================================================================================================
 *====================================================================================================================*/
void threadTimerIntel(const std::function<void(files &, const rs2::pipeline, const rs2::colorizer)> &inputFunction,
                      files &inputFiles, rs2::pipeline inputPipe, rs2::colorizer inputColorizer,
                      unsigned int interval)
{
  std::thread([inputFunction, interval, &inputPipe, &inputColorizer, &inputFiles]()
              {
                  while (!exit_app)
                  {
                    auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval);

                    inputFunction(inputFiles, inputPipe, inputColorizer);

                    std::this_thread::sleep_until(x);
                  }
              }).detach();
}

void intelFrameCapture(files &inputFiles, const rs2::pipeline &inputPipe, const rs2::colorizer &inputColorizer)
{
  for (auto &&frame: inputPipe.wait_for_frames())
  {
    if (auto vf = frame.as<rs2::video_frame>())
    {
      if (vf.is<rs2::depth_frame>()) //If it is a depth frame
      {
        auto pixels = (uint16_t *) vf.get_data(); //Grab the data pointer
        inputFiles.depthRAW.write(reinterpret_cast<char *>(pixels),
                                  vf.get_data_size()); //Write the raw data to a file

        vf = inputColorizer.process(frame);  //Apply a color map and "Make an image"
        pixels = (uint16_t *) vf.get_data();   //Grab the data pointer
        inputFiles.depthRGB.write(reinterpret_cast<char *>(pixels),
                                  vf.get_data_size()); //Write the RGB Image to the file
      }
      else
      {
        auto pixels = (uint16_t *) vf.get_data(); //Grab the data pointer
        inputFiles.rgb.write(reinterpret_cast<char *>(pixels), vf.get_data_size()); //Save to a file
      }
    }
  }
  intelFrameCount++;
}
/*======================================================================================================================
 *====================================================================================================================*/
void threadTimerZed(const std::function<void(files &, sl::Camera &zedObject, sl::Resolution)> &inputFunction,
                    files &inputFiles, sl::Camera &zedObject, sl::Resolution frameRes, unsigned int interval)
{
  std::thread([inputFunction, interval, &zedObject, frameRes, &inputFiles]()
              {
                  while (!exit_app)
                  {
                    auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval);

                    inputFunction(inputFiles, zedObject, frameRes);

                    std::this_thread::sleep_until(x);
                  }
              }).detach();
}

void zedFrameCapture(files &inoutFiles, sl::Camera &zedObject, sl::Resolution frameRes)
{
  sl::Mat rgbData(frameRes, sl::MAT_TYPE::U8_C4);
  sl::Mat depthData(frameRes, sl::MAT_TYPE::U8_C4);
  // sl::Mat depthRawData(frameRes, sl::MAT_TYPE::F32_C4);

  auto returned_state = zedObject.grab();
  // A new image is available if grab() returns ERROR_CODE::SUCCESS
  if (returned_state == sl::ERROR_CODE::SUCCESS)
  {
    /*
     * Grab a rgb image from the ZED
     */
    zedObject.retrieveImage(rgbData, sl::VIEW::LEFT);
    inoutFiles.rgb.write(reinterpret_cast<char *>(rgbData.getPtr<sl::uchar1>()),
                         (rgbData.getWidthBytes() * rgbData.getHeight())); //Save to a file


    /*
     * Grab a depth image from the ZED
     */
    zedObject.retrieveImage(depthData, sl::VIEW::DEPTH);
    inoutFiles.depthRGB.write(reinterpret_cast<char *>(depthData.getPtr<sl::uchar1>()),
                              (depthData.getWidthBytes() * depthData.getHeight())); //Save to a file


    /*
     * Grab a raw depth image from the ZED
     */

/*
    zedObject.retrieveMeasure(depthRawData, sl::MEASURE::XYZ);
    inoutFiles.depthRAW.write(reinterpret_cast<char *>(depthRawData.getPtr<sl::uchar1>()),
                              (depthRawData.getWidthBytes() * depthRawData.getHeight())); //Save to a file
*/

  }

  zedFrameCount++;
}
/*======================================================================================================================
 *====================================================================================================================
 * void
threadTimerMotive(std::function<void(sFrameOfMocapData *, void *)>, sFrameOfMocapData , void *, unsigned int interval);
 * */
void threadTimerMotive(const std::function<void(sFrameOfMocapData, void *)> &inputFunction, sFrameOfMocapData data,
                       void *pUserData, unsigned int interval)
{
  std::thread([inputFunction, interval, data, pUserData]()
              {
                  while (!exit_app)
                  {
                    auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval);

                    inputFunction(data, pUserData);

                    std::this_thread::sleep_until(x);
                  }
              }).detach();
}

void motiveFrameCapture(sFrameOfMocapData data, void *pUserData)
{
  DataHandler(&data, pUserData);
  motiveFrameCount++;
}

// MOTIVE FUNCTIONS
//
// Establish a NatNet Client connection
int ConnectClient()
{
  // Release previous server
  g_pClient->Disconnect();

  // Init Client and connect to NatNet server
  int retCode = g_pClient->Connect(g_connectParams);
  if (retCode != ErrorCode_OK)
  {
    printf("Unable to connect to server.  Error code: %d. Exiting.\n", retCode);
    return ErrorCode_Internal;
  }
  else
  {
    // connection succeeded

    void *pResult;
    int nBytes = 0;
    ErrorCode ret = ErrorCode_OK;

    // print server info
    memset(&g_serverDescription, 0, sizeof(g_serverDescription));
    ret = g_pClient->GetServerDescription(&g_serverDescription);
    if (ret != ErrorCode_OK || !g_serverDescription.HostPresent)
    {
      printf("Unable to connect to server. Host not present. Exiting.\n");
      return 1;
    }
    printf("\n[SampleClient] Server application info:\n");
    printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
           g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2],
           g_serverDescription.HostAppVersion[3]);
    printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
           g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
    printf("Client IP:%s\n", g_connectParams.localAddress);
    printf("Server IP:%s\n", g_connectParams.serverAddress);
    printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

    // get mocap frame rate
    ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
    if (ret == ErrorCode_OK)
    {
      float fRate = *((float *) pResult);
      printf("Mocap Framerate : %3.2f\n", fRate);
    }
    else
      printf("Error getting frame rate.\n");

    // get # of analog samples per mocap frame of data
    ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
    if (ret == ErrorCode_OK)
    {
      int g_analogSamplesPerMocapFrame = *((int *) pResult);
      printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
    }
    else
      printf("Error getting Analog frame rate.\n");
  }

  return ErrorCode_OK;
}

void resetClient()
{
  int iSuccess;

  printf("\n\nre-setting Client\n\n.");

  iSuccess = g_pClient->Disconnect();
  if (iSuccess != 0)
    printf("error un-initting Client\n");

  iSuccess = g_pClient->Connect(g_connectParams);
  if (iSuccess != 0)
    printf("error re-initting Client\n");
}

void NATNET_CALLCONV DataHandler(sFrameOfMocapData *data, void *pUserData)
{
  std::cout << "[INFO] - Data Handler" << std::endl;
  NatNetClient *pClient = (NatNetClient *) pUserData;

// Software latency here is defined as the span of time between:
//   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
// and
//   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
//
// This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
// because it additionally includes the time spent preparing to stream the data via NatNet.
  const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
  const double softwareLatencyMillisec =
    (softwareLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

// Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
// The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
  const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

  if (g_outputFile)
  {
    _WriteFrame(g_outputFile, data);
  }

  int i = 0;

  printf("FrameID : %d\n", data->iFrame);
  printf("Timestamp : %3.2lf\n", data->fTimestamp);
  printf("Software latency : %.2lf milliseconds\n", softwareLatencyMillisec);

// Only recent versions of the Motive software in combination with ethernet camera systems support system latency measurement.
// If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
  const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

  if (bSystemLatencyAvailable)
  {
// System latency here is defined as the span of time between:
//   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
// and
//   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
    const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
    const double systemLatencyMillisec =
      (systemLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

// Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
// This is the all-inclusive measurement (photons to client processing).
    const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp) * 1000.0;

// You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
//const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

    printf("System latency : %.2lf milliseconds\n", systemLatencyMillisec);
    printf("Total client latency : %.2lf milliseconds (transit time +%.2lf ms)\n", clientLatencyMillisec,
           transitLatencyMillisec);
  }
  else
  {
    printf("Transit latency : %.2lf milliseconds\n", transitLatencyMillisec);
  }

// FrameOfMocapData params
  bool bIsRecording = ((data->params & 0x01) != 0);
  bool bTrackedModelsChanged = ((data->params & 0x02) != 0);
  if (bIsRecording)
    printf("RECORDING\n");
  if (bTrackedModelsChanged)
    printf("Models Changed.\n");


// timecode - for systems with an eSync and SMPTE timecode generator - decode to values
  int hour, minute, second, frame, subframe;
  NatNet_DecodeTimecode(data
                          ->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
// decode to friendly string
  char szTimecode[128] = "";
  NatNet_TimecodeStringify(data
                             ->Timecode, data->TimecodeSubframe, szTimecode, 128);
  printf("Timecode : %s\n", szTimecode);


// Skeletons
  printf("Skeletons [Count=%d]\n", data->nSkeletons);
  for (
    i = 0;
    i < data->
      nSkeletons;
    i++)
  {
    sSkeletonData skData = data->Skeletons[i];
    printf("Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID, skData.nRigidBodies);
    for (
      int j = 0;
      j < skData.
        nRigidBodies;
      j++)
    {
      sRigidBodyData rbData = skData.RigidBodyData[j];
      printf("Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
             rbData.ID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy, rbData.qz, rbData.qw);

// guardar na struct os valores
      infoMotive.
        frame = frame;
      infoMotive.
        bone = rbData.ID;
      infoMotive.
        xx = rbData.x;
      infoMotive.
        yy = rbData.y;
      infoMotive.
        zz = rbData.z;
    }
  }
}

void _WriteFrame(FILE *fp, sFrameOfMocapData *data)
{
  fprintf(fp, "%d", data->iFrame);
  for (int i = 0; i < data->MocapData->nMarkers; i++)
  {
    fprintf(fp, "\t%.5f\t%.5f\t%.5f", data->MocapData->Markers[i][0], data->MocapData->Markers[i][1],
            data->MocapData->Markers[i][2]);
  }
  fprintf(fp, "\n");
}