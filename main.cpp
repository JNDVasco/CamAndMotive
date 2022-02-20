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


// void motiveFrameCapture(sFrameOfMocapData, void *);

//void
// threadTimerMotive(std::function<void(sFrameOfMocapData, void *)>, sFrameOfMocapData, void *, unsigned int interval);

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

    system("pause"); //Wait for the user press enter to start
    auto programBegin = std::chrono::high_resolution_clock::now();
    //Capture Stuff
    //This program block is timed
    //Put the cameras capturing in the background

    threadTimerIntel(intelFrameCapture, intelFile, intelPipe, color_map, millisBetweenFrames);
    threadTimerZed(zedFrameCapture, zedFile, zedCam, imageSize, millisBetweenFrames);

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

//    // Done - clean up Motive
//    if (g_pClient) {
//        g_pClient->Disconnect();
//        delete g_pClient;
//        g_pClient = NULL;
//    }

    return 0;
} //End main()

uint64_t millis()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
}


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
    uint64_t timestamp = millis();
    for (auto &&frame: inputPipe.wait_for_frames())
    {
        if (auto vf = frame.as<rs2::video_frame>())
        {
            if (vf.is<rs2::depth_frame>()) //If it is a depth frame
            {
                auto pixels = (uint16_t *) vf.get_data(); //Grab the data pointer
                inputFiles.depthRAW.write(reinterpret_cast<char *>(timestamp),sizeof(timestamp));
                inputFiles.depthRAW.write(reinterpret_cast<char *>(pixels),
                                          vf.get_data_size()); //Write the raw data to a file

                vf = inputColorizer.process(frame);  //Apply a color map and "Make an image"
                pixels = (uint16_t *) vf.get_data();   //Grab the data pointer
                inputFiles.depthRGB.write(reinterpret_cast<char *>(timestamp),sizeof(timestamp));
                inputFiles.depthRGB.write(reinterpret_cast<char *>(pixels),
                                          vf.get_data_size()); //Write the RGB Image to the file
            } else
            {
                auto pixels = (uint16_t *) vf.get_data(); //Grab the data pointer
                inputFiles.rgb.write(reinterpret_cast<char *>(timestamp),sizeof(timestamp));
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
        inputFiles.rgb.write(reinterpret_cast<char *>(timestamp),sizeof(timestamp));
        inputFiles.rgb.write(reinterpret_cast<char *>(rgbData.getPtr<sl::uchar1>()),
                             (rgbData.getWidthBytes() * rgbData.getHeight())); //Save to a file


        /*
         * Grab a depth image from the ZED
         */
        zedObject.retrieveImage(depthData, sl::VIEW::DEPTH);
        inputFiles.depthRGB.write(reinterpret_cast<char *>(timestamp),sizeof(timestamp));
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
*====================================================================================================================
* void
threadTimerMotive(std::function<void(sFrameOfMocapData *, void *)>, sFrameOfMocapData , void *, unsigned int interval);
* */
//void threadTimerMotive(const std::function<void(sFrameOfMocapData, void *)> &inputFunction, sFrameOfMocapData data,
//                       void *pUserData, unsigned int interval) {
//    std::thread([inputFunction, interval, data, pUserData]() {
//        while (!exit_app) {
//            auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval);
//
//            inputFunction(data, pUserData);
//
//            std::this_thread::sleep_until(x);
//        }
//    }).detach();
//}
//
//void motiveFrameCapture(sFrameOfMocapData data, void *pUserData) {
//    DataHandler(&data, pUserData);
//    motiveFrameCount++;
//}

