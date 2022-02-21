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

/* ===============================================================================
* Zed Stuff
* =============================================================================== */
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


/* ===============================================================================
* Intel Stuff
* =============================================================================== */
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


/* ===============================================================================
* Motive Stuff
* =============================================================================== */

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32

#   include <conio.h>

#else
#   include <unistd.h>
#   include <termios.h>
#endif

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

void NATNET_CALLCONV DataHandler(sFrameOfMocapData *data, void *pUserData);    // receives data from the server
void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char *msg);      // receives NatNet error messages
void resetClient();

int ConnectClient();

int ConnectClient();

static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;
NatNetClient *g_pClient = nullptr;
sNatNetClientConnectParams g_connectParams;
char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
int g_analogSamplesPerMocapFrame = 0;
sServerDescription g_serverDescription;

//std::atomic<sFrameOfMocapData> globalFrameData;
sFrameOfMocapData *globalFrameData;

/* ===============================================================================
* Timing const so we now when to capture each frame
* If we want a final output with 30 fps we need to
* capture 30 frames per second ie one each 1000/30 milliseconds
* =============================================================================== */
const int fpsOutput = 20;
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
uint64_t millis(); //uint64t has 8 bytes
void threadTimerIntel(const std::function<void(files &, const rs2::pipeline, const rs2::colorizer)> &,
                      files &inputFiles, rs2::pipeline inputPipe, rs2::colorizer inputColorizer, unsigned int interval);

void threadTimerZed(const std::function<void(files &, sl::Camera &zedObject, sl::Resolution)> &, files &inputFiles,
                    sl::Camera &zedObject, sl::Resolution, unsigned int interval);

void intelFrameCapture(files &, const rs2::pipeline &inputPipe, const rs2::colorizer &inputColorizer);

void zedFrameCapture(files &, sl::Camera &zedObject, sl::Resolution);


void motiveFrameCapture();

void threadTimerMotive(std::function<void()>, unsigned int interval);

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
        } else
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
    // print version info
    unsigned char ver[4];
    NatNet_GetVersion(ver);
    printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

    // Install logging callback
    NatNet_SetLogCallback(MessageHandler);

    // create NatNet client
    g_pClient = new NatNetClient();

    // set the frame callback handler
    g_pClient->SetFrameReceivedCallback(DataHandler, g_pClient);  // this function will receive data from the server

    g_connectParams.connectionType = kDefaultConnectionType;
    g_connectParams.serverAddress = "127.0.0.1";
    g_connectParams.localAddress = "127.0.0.1";

    int iResult;

    // Connect to Motive
    iResult = ConnectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client. See log for details. Exiting.\n");
        return 1;
    } else
    {
        printf("Client initialized and ready.\n");
    }


    // Send/receive test request
    void *response;
    int nBytes;
    printf("[SampleClient] Sending Test Request\n");
    iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
    if (iResult == ErrorCode_OK)
    {
        printf("[SampleClient] Received: %s\n", (char *) response);
    }

    // Retrieve Data Descriptions from Motive
    printf("\n\n[SampleClient] Requesting Data Descriptions...\n");
    sDataDescriptions *pDataDefs = NULL;
    iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
    {
        printf("[SampleClient] Unable to retrieve Data Descriptions.\n");
    } else
    {
        printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
        for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
        {
            printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
            {
                // MarkerSet
                sMarkerSetDescription *pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                printf("MarkerSet Name : %s\n", pMS->szName);
                for (int i = 0; i < pMS->nMarkers; i++)
                    printf("%s\n", pMS->szMarkerNames[i]);

            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription *pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                printf("RigidBody Name : %s\n", pRB->szName);
                printf("RigidBody ID : %d\n", pRB->ID);
                printf("RigidBody Parent ID : %d\n", pRB->parentID);
                printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);

                if (pRB->MarkerPositions != NULL && pRB->MarkerRequiredLabels != NULL)
                {
                    for (int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx)
                    {
                        const MarkerData &markerPosition = pRB->MarkerPositions[markerIdx];
                        const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];

                        printf("\tMarker #%d:\n", markerIdx);
                        printf("\t\tPosition: %.2f, %.2f, %.2f\n", markerPosition[0], markerPosition[1],
                               markerPosition[2]);

                        if (markerRequiredLabel != 0)
                        {
                            printf("\t\tRequired active label: %d\n", markerRequiredLabel);
                        }
                    }
                }
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
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
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate)
            {
                // Force Plate
                sForcePlateDescription *pFP = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
                printf("Force Plate ID : %d\n", pFP->ID);
                printf("Force Plate Serial : %s\n", pFP->strSerialNo);
                printf("Force Plate Width : %3.2f\n", pFP->fWidth);
                printf("Force Plate Length : %3.2f\n", pFP->fLength);
                printf("Force Plate Electrical Center Offset (%3.3f, %3.3f, %3.3f)\n", pFP->fOriginX, pFP->fOriginY,
                       pFP->fOriginZ);
                for (int iCorner = 0; iCorner < 4; iCorner++)
                    printf("Force Plate Corner %d : (%3.4f, %3.4f, %3.4f)\n", iCorner, pFP->fCorners[iCorner][0],
                           pFP->fCorners[iCorner][1], pFP->fCorners[iCorner][2]);
                printf("Force Plate Type : %d\n", pFP->iPlateType);
                printf("Force Plate Data Type : %d\n", pFP->iChannelDataType);
                printf("Force Plate Channel Count : %d\n", pFP->nChannels);
                for (int iChannel = 0; iChannel < pFP->nChannels; iChannel++)
                    printf("\tChannel %d : %s\n", iChannel, pFP->szChannelNames[iChannel]);
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Device)
            {
                // Peripheral Device
                sDeviceDescription *pDevice = pDataDefs->arrDataDescriptions[i].Data.DeviceDescription;
                printf("Device Name : %s\n", pDevice->strName);
                printf("Device Serial : %s\n", pDevice->strSerialNo);
                printf("Device ID : %d\n", pDevice->ID);
                printf("Device Channel Count : %d\n", pDevice->nChannels);
                for (int iChannel = 0; iChannel < pDevice->nChannels; iChannel++)
                    printf("\tChannel %d : %s\n", iChannel, pDevice->szChannelNames[iChannel]);
            } else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Camera)
            {
                // Camera
                sCameraDescription *pCamera = pDataDefs->arrDataDescriptions[i].Data.CameraDescription;
                printf("Camera Name : %s\n", pCamera->strName);
                printf("Camera Position (%3.2f, %3.2f, %3.2f)\n", pCamera->x, pCamera->y, pCamera->z);
                printf("Camera Orientation (%3.2f, %3.2f, %3.2f, %3.2f)\n", pCamera->qx, pCamera->qy, pCamera->qz,
                       pCamera->qw);
            } else
            {
                printf("Unknown data type.\n");
                // Unknown
            }
        }
    }

    // Ready to receive marker stream!
    printf("\nClient is connected to server and listening for data...\n");


    /*
    * End Motive
    */

/* =====================================================================================================================
 * =====================================================================================================================
 * ================================================================================================================== */
    system("pause"); //Wait for the user press enter to start
    auto programBegin = std::chrono::high_resolution_clock::now();
    //Capture Stuff
    //This program block is timed
    //Put the cameras capturing in the background

    threadTimerIntel(intelFrameCapture, intelFile, intelPipe, color_map, millisBetweenFrames);
    threadTimerZed(zedFrameCapture, zedFile, zedCam, imageSize, millisBetweenFrames);
    threadTimerMotive(motiveFrameCapture, millisBetweenFrames);

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

    // Close the camera
    zedCam.close();

    // Done - clean up.
    if (g_pClient)
    {
        g_pClient->Disconnect();
        delete g_pClient;
        g_pClient = NULL;
    }

    return 0;
} //End main()

uint64_t millis()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
}


/*======================================================================================================================
 *= Intel Capture Stuff
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
    uint64_t timestamp = millis();
    for (auto &&frame: inputPipe.wait_for_frames())
    {
        if (auto vf = frame.as<rs2::video_frame>())
        {
            if (vf.is<rs2::depth_frame>()) //If it is a depth frame
            {
                auto pixels = (uint16_t *) vf.get_data(); //Grab the data pointer
                inputFiles.depthRAW.write(reinterpret_cast<char *>(timestamp), sizeof(timestamp));
                inputFiles.depthRAW.write(reinterpret_cast<char *>(pixels),
                                          vf.get_data_size()); //Write the raw data to a file

                vf = inputColorizer.process(frame);  //Apply a color map and "Make an image"
                pixels = (uint16_t *) vf.get_data();   //Grab the data pointer
                inputFiles.depthRGB.write(reinterpret_cast<char *>(timestamp), sizeof(timestamp));
                inputFiles.depthRGB.write(reinterpret_cast<char *>(pixels),
                                          vf.get_data_size()); //Write the RGB Image to the file
            } else
            {
                auto pixels = (uint16_t *) vf.get_data(); //Grab the data pointer
                inputFiles.rgb.write(reinterpret_cast<char *>(timestamp), sizeof(timestamp));
                inputFiles.rgb.write(reinterpret_cast<char *>(pixels), vf.get_data_size()); //Save to a file
            }
        }
    }
    intelFrameCount++;
}

/*======================================================================================================================
 *= Zed Capture Stuff
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

void zedFrameCapture(files &inputFiles, sl::Camera &zedObject, sl::Resolution frameRes)
{
    uint64_t timestamp = millis();

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
        inputFiles.rgb.write(reinterpret_cast<char *>(timestamp), sizeof(timestamp));
        inputFiles.rgb.write(reinterpret_cast<char *>(rgbData.getPtr<sl::uchar1>()),
                             (rgbData.getWidthBytes() * rgbData.getHeight())); //Save to a file


        /*
         * Grab a depth image from the ZED
         */
        zedObject.retrieveImage(depthData, sl::VIEW::DEPTH);
        inputFiles.depthRGB.write(reinterpret_cast<char *>(timestamp), sizeof(timestamp));
        inputFiles.depthRGB.write(reinterpret_cast<char *>(depthData.getPtr<sl::uchar1>()),
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
 *= Motive Capture Stuff
 *====================================================================================================================*/
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
    } else
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
        printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp,
               g_serverDescription.HostAppVersion[0],
               g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2],
               g_serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0],
               g_serverDescription.NatNetVersion[1],
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
        } else
            printf("Error getting frame rate.\n");

        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            g_analogSamplesPerMocapFrame = *((int *) pResult);
            printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
        } else
            printf("Error getting Analog frame rate.\n");
    }

    return ErrorCode_OK;
}


// DataHandler receives data from the server
// This function is called by NatNet when a frame of mocap data is available
void NATNET_CALLCONV DataHandler(sFrameOfMocapData *data, void *pUserData)
{
    NatNetClient *pClient = (NatNetClient *) pUserData;

    int i = 0;

    printf("FrameID : %d\n", data->iFrame);
    printf("Timestamp : %3.2lf\n", data->fTimestamp);

    // memcpy(globalFrameData, data, sizeof(data));


// Rigid Bodies
    printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
    for (i = 0; i < data->nRigidBodies; i++)
    {
// params
// 0x01 : bool, rigid body was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;

        printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError,
               bTrackingValid);
        printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
        printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
               data->RigidBodies[i].x,
               data->RigidBodies[i].y,
               data->RigidBodies[i].z,
               data->RigidBodies[i].qx,
               data->RigidBodies[i].qy,
               data->RigidBodies[i].qz,
               data->RigidBodies[i].qw);
    }

// Skeletons
    printf("Skeletons [Count=%d]\n", data->nSkeletons);
    for (i = 0; i < data->nSkeletons; i++)
    {
        sSkeletonData skData = data->Skeletons[i];
        printf("Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID, skData.nRigidBodies);
        for (int j = 0; j < skData.nRigidBodies; j++)
        {
            sRigidBodyData rbData = skData.RigidBodyData[j];
            printf("Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
                   rbData.ID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy, rbData.qz, rbData.qw);
        }
    }

// labeled markers - this includes all markers (Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
    bool bOccluded;     // marker was not visible (occluded) in this frame
    bool bPCSolved;     // reported position provided by point cloud solve
    bool bModelSolved;  // reported position provided by model solve
    bool bHasModel;     // marker has an associated asset in the data stream
    bool bUnlabeled;    // marker is 'unlabeled', but has a point cloud ID that matches Motive PointCloud ID (In Motive 3D View)
    bool bActiveMarker; // marker is an actively labeled LED marker

    printf("Markers [Count=%d]\n", data->nLabeledMarkers);
    for (i = 0; i < data->nLabeledMarkers; i++)
    {
        bOccluded = ((data->LabeledMarkers[i].params & 0x01) != 0);
        bPCSolved = ((data->LabeledMarkers[i].params & 0x02) != 0);
        bModelSolved = ((data->LabeledMarkers[i].params & 0x04) != 0);
        bHasModel = ((data->LabeledMarkers[i].params & 0x08) != 0);
        bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
        bActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);

        sMarker marker = data->LabeledMarkers[i];

// Marker ID Scheme:
// Active Markers:
//   ID = ActiveID, correlates to RB ActiveLabels list
// Passive Markers:
//   If Asset with Legacy Labels
//      AssetID 	(Hi Word)
//      MemberID	(Lo Word)
//   Else
//      PointCloud ID
        int modelID, markerID;
        NatNet_DecodeID(marker.ID, &modelID, &markerID);

        char szMarkerType[512];
        if (bActiveMarker)
        {
            strcpy(szMarkerType, "Active");
        } else if (bUnlabeled)
        {
            strcpy(szMarkerType, "Unlabeled");
        } else
        {
            strcpy(szMarkerType, "Labeled");
        }

        printf("%s Marker [ModelID=%d, MarkerID=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
               szMarkerType, modelID, markerID, marker.size, marker.x, marker.y, marker.z);
    }

// force plates
    printf("Force Plate [Count=%d]\n", data->nForcePlates);
    for (int iPlate = 0; iPlate < data->nForcePlates; iPlate++)
    {
        printf("Force Plate %d\n", data->ForcePlates[iPlate].ID);
        for (int iChannel = 0; iChannel < data->ForcePlates[iPlate].nChannels; iChannel++)
        {
            printf("\tChannel %d:\t", iChannel);
            if (data->ForcePlates[iPlate].ChannelData[iChannel].nFrames == 0)
            {
                printf("\tEmpty Frame\n");
            } else if (data->ForcePlates[iPlate].ChannelData[iChannel].nFrames != g_analogSamplesPerMocapFrame)
            {
                printf("\tPartial Frame [Expected:%d   Actual:%d]\n", g_analogSamplesPerMocapFrame,
                       data->ForcePlates[iPlate].ChannelData[iChannel].nFrames);
            }
            for (int iSample = 0; iSample < data->ForcePlates[iPlate].ChannelData[iChannel].nFrames; iSample++)
                printf("%3.2f\t", data->ForcePlates[iPlate].ChannelData[iChannel].Values[iSample]);
            printf("\n");
        }
    }
}


// MessageHandler receives NatNet error/debug messages
void NATNET_CALLCONV
MessageHandler(Verbosity msgType, const char *msg)
{
// Optional: Filter out debug messages
    if (msgType < Verbosity_Info)
    {
        return;
    }

    printf("\n[NatNetLib]");

    switch (msgType)
    {
        case Verbosity_Debug:
            printf(" [DEBUG]");
            break;
        case Verbosity_Info:
            printf("  [INFO]");
            break;
        case Verbosity_Warning:
            printf("  [WARN]");
            break;
        case Verbosity_Error:
            printf(" [ERROR]");
            break;
        default:
            printf(" [?????]");
            break;
    }

    printf(": %s\n", msg);
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


void threadTimerMotive(const std::function<void()> &inputFunction, unsigned int interval)
{
    std::thread([inputFunction, interval]()
                {
                    while (!exit_app)
                    {
                        auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval);

                        inputFunction();

                        std::this_thread::sleep_until(x);
                    }
                }).detach();
}

void motiveFrameCapture()
{
    motiveFrameCount++;
}

