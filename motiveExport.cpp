//
// Created by JNDVasco on 21/02/2022.
//
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include <atomic>
#include <string>
#include <fstream>
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

#include <vector>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>


typedef struct
{
    uint64_t timestamp;
    int32_t nMarkers;
    sMarker markers[50];
    int32_t nSkeletons;
    sSkeletonData skeletons[50];
} motiveStruct;

motiveStruct motiveData;

const std::string motiveFilePath = "../dataOut/motive/data.bin";

int main()
{

    std::fstream motiveFile;

    motiveFile.open(motiveFilePath, std::ios::in | std::ios::binary);

    while (motiveFile.read(reinterpret_cast<char *>(&motiveData), sizeof(motiveData)))
    {
        int i = 0;
        printf("Skeletons [Count=%d]\n", motiveData.nSkeletons);
        for (i = 0; i < motiveData.nSkeletons; i++)
        {
            sSkeletonData skData = motiveData.skeletons[i];
            printf("Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID, skData.nRigidBodies);
            for (int j = 0; j < skData.nRigidBodies; j++)
            {
                sRigidBodyData rbData = skData.RigidBodyData[j];
                printf("Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
                       rbData.ID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy, rbData.qz, rbData.qw);
            }
        }

        printf("Markers [Count=%d]\n", motiveData.nMarkers);
        for (i = 0; i < motiveData.nMarkers; i++)
        {
            sMarker marker = motiveData.markers[i];
            
            int modelID, markerID;
            NatNet_DecodeID(marker.ID, &modelID, &markerID);

            printf("%s Marker [ModelID=%d, MarkerID=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
                   "szMarkerType", modelID, markerID, marker.size, marker.x, marker.y, marker.z);
        }
    }
}