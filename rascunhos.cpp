// Rigid Bodies
/*
printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
for (i = 0; i < data->nRigidBodies; i++)
{
    // params
    // 0x01 : bool, rigid body was successfully tracked in this frame
    bool bTrackingValid = data->RigidBodies[i].params & 0x01;

    printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
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
*/

// labeled markers - this includes all markers (Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
/*
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
        strcpy(szMarkerType, "Active");
    else if (bUnlabeled)
        strcpy(szMarkerType, "Unlabeled");
    else
        strcpy(szMarkerType, "Labeled");

    printf("%s Marker [ModelID=%d, MarkerID=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
        szMarkerType, modelID, markerID, marker.size, marker.x, marker.y, marker.z);
}
*/

// force plates
/*
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
        }
        else if (data->ForcePlates[iPlate].ChannelData[iChannel].nFrames != g_analogSamplesPerMocapFrame)
        {
            printf("\tPartial Frame [Expected:%d   Actual:%d]\n", g_analogSamplesPerMocapFrame, data->ForcePlates[iPlate].ChannelData[iChannel].nFrames);
        }
        for (int iSample = 0; iSample < data->ForcePlates[iPlate].ChannelData[iChannel].nFrames; iSample++)
            printf("%3.2f\t", data->ForcePlates[iPlate].ChannelData[iChannel].Values[iSample]);
        printf("\n");
    }
}
*/

// devices
/*printf("Device [Count=%d]\n", data->nDevices);
for (int iDevice = 0; iDevice < data->nDevices; iDevice++)
{
    printf("Device %d\n", data->Devices[iDevice].ID);
    for (int iChannel = 0; iChannel < data->Devices[iDevice].nChannels; iChannel++)
    {
        printf("\tChannel %d:\t", iChannel);
        if (data->Devices[iDevice].ChannelData[iChannel].nFrames == 0)
        {
            printf("\tEmpty Frame\n");
        }
        else if (data->Devices[iDevice].ChannelData[iChannel].nFrames != g_analogSamplesPerMocapFrame)
        {
            printf("\tPartial Frame [Expected:%d   Actual:%d]\n", g_analogSamplesPerMocapFrame, data->Devices[iDevice].ChannelData[iChannel].nFrames);
        }
        for (int iSample = 0; iSample < data->Devices[iDevice].ChannelData[iChannel].nFrames; iSample++)
            printf("%3.2f\t", data->Devices[iDevice].ChannelData[iChannel].Values[iSample]);
        printf("\n");
    }
}
*/