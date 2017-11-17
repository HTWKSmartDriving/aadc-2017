//
// Created by lina on 11.11.17.
//

#include "parkingOccupancy.h"

ADTF_FILTER_PLUGIN(FILTER_NAME, OID, parkingOccupancy)

parkingOccupancy::parkingOccupancy(const tChar *__info) : Leaf(__info) {
    SetPropertyFloat(LOWER_OCCUPANCY_RATIO_PROPERTY, LOWER_OCCUPANCY_RATIO_DEFAULT);
    SetPropertyStr(LOWER_OCCUPANCY_RATIO_PROPERTY NSSUBPROP_DESCRIPTION, LOWER_OCCUPANCY_RATIO_DESCRIPTION);
    SetPropertyFloat(LOWER_OCCUPANCY_RATIO_PROPERTY  NSSUBPROP_MIN, LOWER_OCCUPANCY_RATIO_MIN);
    SetPropertyFloat(LOWER_OCCUPANCY_RATIO_PROPERTY  NSSUBPROP_MAX, LOWER_OCCUPANCY_RATIO_MAX);
}

tResult parkingOccupancy::Init(tInitStage eStage, __exception) {
    RETURN_IF_FAILED(Leaf::Init(eStage, __exception_ptr));

    if (StageFirst == eStage) {
        RETURN_IF_FAILED(gridMapInput.Create("GridMap", IPin::PD_Input, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&gridMapInput));

        //create Pin for Map Visualization Video
        RETURN_IF_FAILED(
                debugOutputPin.Create("debug_video", IPin::PD_Output, static_cast<IPinEventSink *>(this)));
        RETURN_IF_FAILED(RegisterPin(&debugOutputPin));
    }


    if (StageNormal == eStage) {
        lowerOccRatio = (int) GetPropertyInt(LOWER_OCCUPANCY_RATIO_PROPERTY, LOWER_OCCUPANCY_RATIO_DEFAULT);
        /*for (const auto &lot: parkingLots) {
            if (lot.id == 6) {
                LOG_INFO(adtf_util::cString::Format("parkingLots status before check id: %d status: %d", lot.id,
                                                    static_cast<int>(lot.status)));
            }
        }*/


    }

    RETURN_NOERROR;
}

tResult parkingOccupancy::Shutdown(tInitStage eStage, __exception) {
    return Leaf::Shutdown(eStage, __exception_ptr);
}

tResult parkingOccupancy::OnTrigger() {
    tFloat32 headingInRad;
    //int targetParkingLotId;
    //ParkingLot targetParkingLot = ParkingLot();

    if (!IS_OK(worldService->Pull(WORLD_CURRENT_POSITION, position))) {
        //LOG_ERROR("could not pull position");
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    if (!IS_OK(worldService->Pull(WORLD_PARKING_LOTS, parkingLots))) {
        //LOG_ERROR("could not pull lots");
        TransmitStatus(BT::FAIL);
        RETURN_ERROR(ERR_FAILED);
    }

    cv::Mat flippedMat;
    cv::flip(gridMap, flippedMat, 0);
    gridMap = flippedMat;

    // Loop through list of parking lots to calculate occupancy of each parking lot
    for (auto &lot: parkingLots) {
        double parkingSpotX = lot.position.x();
        double parkingSpotY = lot.position.y();

        Orientation::MapTile parkingSpotDirection = lot.direction;

        HTWKPoint startPoint;
        HTWKPoint endPoint;

        // Calculate start and end point respective to the direction of the parking lot
        if (parkingSpotDirection == Orientation::MapTile::NORTH) {
            startPoint.setXval(parkingSpotX - 0.25);
            startPoint.setYval(parkingSpotY);
            endPoint.setXval(parkingSpotX + 0.5);
            endPoint.setYval(parkingSpotY - 1);
        } else if (parkingSpotDirection == Orientation::MapTile::EAST) {
            startPoint.setXval(parkingSpotX);
            startPoint.setYval(parkingSpotY - 0.25);
            endPoint.setXval(parkingSpotX - 1);
            endPoint.setYval(parkingSpotY + 0.5);
        } else if (parkingSpotDirection == Orientation::MapTile::SOUTH) {
            startPoint.setXval(parkingSpotX - 0.25);
            startPoint.setYval(parkingSpotY);
            endPoint.setXval(parkingSpotX + 0.5);
            endPoint.setYval(parkingSpotY + 1);
        } else if (parkingSpotDirection == Orientation::MapTile::WEST) {
            startPoint.setXval(parkingSpotX);
            startPoint.setYval(parkingSpotY - 0.25);
            endPoint.setXval(parkingSpotX + 1);
            endPoint.setYval(parkingSpotY + 0.5);
        }

        // Convert start and end point to points in the grid map
        startPoint.setXval(startPoint.x() * 10 + flippedMat.rows / 2.0);
        startPoint.setYval(startPoint.y() * 10 + flippedMat.cols / 2.0);
        endPoint.setXval(endPoint.x() * 10 + flippedMat.rows / 2.0);
        endPoint.setYval(endPoint.y() * 10 + flippedMat.cols / 2.0);
        if(debugOutputPin.IsConnected()) {
            cv::Mat debugMat = flippedMat.clone();
            cv::cvtColor(debugMat, debugMat, CV_GRAY2RGB);
            cv::Point2f tl, br;
            tl.x = startPoint.x();
            tl.y = startPoint.y();
            br.x = endPoint.x();
            br.y = endPoint.y();
            cv::rectangle(debugMat, tl, br, cv::Scalar(255, 0, 0));
            transmitVideo(debugMat, debugOutputPin, debugFormat);
        }

        int parkingOccRatio = 0;

        // Loop through all cells that are located in the lot and check occupancy of each grid cell
        for (int i = static_cast<int>(startPoint.x()); i >= endPoint.x(); i--) {
            for (int j = static_cast<int>(startPoint.y()); j <= endPoint.y(); j++) {
                //LOG_INFO(adtf_util::cString::Format("Value in Grid Map: %d", static_cast<int>(flippedMat.at<std::uint8_t>(i, j))));
                if (static_cast<int>(flippedMat.at<std::uint8_t>(i, j)) > 128) {
                    parkingOccRatio++;
                }
            }
        }

        //if (lot.id == 6) {
            //LOG_INFO(adtf_util::cString::Format("startPointX: %f startPointY: %f endPointX: %f endPointY: %f", startPoint.x(), startPoint.y(), endPoint.x(), endPoint.y()));
            //LOG_INFO(adtf_util::cString::Format("parkingOccRatio: %d", parkingOccRatio));
        //}

        // 50 cells in a lot 1/3 should be occupied
        if (parkingOccRatio >= lowerOccRatio) {
            lot.status = 1;
        } else {
            lot.status = 0;
        }
    }

    // Push parking lots back to world service
    worldService->Push(WORLD_PARKING_LOTS, parkingLots);

    /*for (const auto &lot: parkingLots) {
        if (lot.id == 6) {
            LOG_INFO(adtf_util::cString::Format("parkingLots status after check id: %d status: %d", lot.id,
                                                static_cast<int>(lot.status)));
        }
    }*/

    //LOG_INFO("success");
    TransmitStatus(BT::SUCCESS);
    RETURN_NOERROR;
}

tResult parkingOccupancy::OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample) {
    Leaf::OnPinEvent(pSource, nEventCode, nParam1, nParam2, pMediaSample);

    if (nEventCode != IPinEventSink::PE_MediaSampleReceived) {
        RETURN_NOERROR;
    }

    RETURN_IF_POINTER_NULL(pMediaSample);

    if (pSource == &gridMapInput) {
        if (gridMapBitmapFormat.nPixelFormat == IImage::PF_UNKNOWN) {
            RETURN_IF_FAILED(UpdateInputImageFormat(gridMapInput.GetFormat()));
        }
        RETURN_IF_POINTER_NULL(pMediaSample);
        const tVoid *l_pSrcBuffer;
        if (IS_OK(pMediaSample->Lock(&l_pSrcBuffer))) {
            if (tInt32(gridMap.total() * gridMap.elemSize()) == gridMapBitmapFormat.nSize) {
                memcpy(gridMap.data, l_pSrcBuffer, size_t(gridMapBitmapFormat.nSize));
                pMediaSample->Unlock(l_pSrcBuffer);
            }
        }
    }

    RETURN_NOERROR;
}

tResult parkingOccupancy::UpdateInputImageFormat(const tBitmapFormat *pFormat) {
    if (pFormat != NULL) {
        gridMapBitmapFormat = (*pFormat);
        RETURN_IF_FAILED(BmpFormat2Mat(gridMapBitmapFormat, gridMap));
    }
    RETURN_NOERROR;
}

tResult parkingOccupancy::transmitVideo(const cv::Mat &image, cVideoPin &pin, tBitmapFormat &bmp) {
    updateOutputImageFormat(image, bmp, pin);
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid **) &pMediaSample));
    RETURN_IF_FAILED(pMediaSample->AllocBuffer(bmp.nSize));
    RETURN_IF_FAILED(pMediaSample->Update(_clock->GetTime(), image.data, bmp.nSize, IMediaSample::MSF_None));
    RETURN_IF_FAILED(pin.Transmit(pMediaSample));
    RETURN_NOERROR;
}

