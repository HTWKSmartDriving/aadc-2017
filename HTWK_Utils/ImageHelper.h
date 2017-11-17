#ifndef AADC_USER_IMAGE_UPDATE_HELPER_H
#define AADC_USER_IMAGE_UPDATE_HELPER_H

#include "HTWK_OpenCV_helper.h"
#include "../HTWK_Types/CameraProperties.h"
//ADTF_OpenCV_helper.h has inital no headerguards!! -> ADD headerguards if not already done -> else modules doesnt init

inline static tResult updateInputImageFormat(const tBitmapFormat *pFormat, tBitmapFormat &inputBitmapFormat,
                                             cv::Mat &inputImage) {
    if (pFormat != NULL) {
        inputBitmapFormat = (*pFormat);
        RETURN_IF_FAILED(BmpFormat2Mat(inputBitmapFormat, inputImage));
    }
    RETURN_NOERROR;
}

inline static tResult bufferToMat(const tBitmapFormat *pFormat, IMediaSample *pMediaSample, cv::Mat &inputImage,
                                  tBitmapFormat &inputBitmapFormat) {
    if (inputBitmapFormat.nPixelFormat == IImage::PF_UNKNOWN) {
        RETURN_IF_FAILED(updateInputImageFormat(pFormat, inputBitmapFormat, inputImage));
    }
    const tVoid *l_pSrcBuffer;
    if (IS_OK(pMediaSample->Lock(&l_pSrcBuffer))) {
        if (tInt32(inputImage.total() * inputImage.elemSize()) == inputBitmapFormat.nSize) {
            memcpy(inputImage.data, l_pSrcBuffer, size_t(inputBitmapFormat.nSize));
        }
    }
    pMediaSample->Unlock(l_pSrcBuffer);
    RETURN_NOERROR;
}

inline static void updateOutputImageFormat(const cv::Mat &outputImage, tBitmapFormat &bitmap, cVideoPin &outputPin) {
    if (tInt32(outputImage.total() * outputImage.elemSize()) != bitmap.nSize) {
        Mat2BmpFormat(outputImage, bitmap);
        outputPin.SetFormat(&bitmap, NULL);
    }
}

namespace htwk {
    /**
     * calculates corresponding depth (z-Coordinate in Metern) via disparity
     */
    inline static float disparityToDepth(const float &disparity, const tIntrinsics &intrinsic, const float &baseline) {
        return disparity > 0 ? (baseline * intrinsic.focal.x) / disparity : (baseline * intrinsic.focal.x);
    }

    /**
     * calculates disparity from depth (z-Coordinate in Metern)
     */
    inline static float depthToDisparity(const float &depth, const tIntrinsics &intrinsic, const float &baseline) {
        return depth > 0 ? (baseline * intrinsic.focal.x) / depth : FLT_MAX;
    }

    /**
     * projects an 2D image pixel to an 3D world point (no distortion!)
     */
    inline static cv::Point3f
    pixelToPoint(const cv::Point2f &imgPoint, const float &depth, const tIntrinsics &intrinsic) {
        float x = (imgPoint.x - intrinsic.principal.x) / intrinsic.focal.x;
        float y = (imgPoint.y - intrinsic.principal.y) / intrinsic.focal.y;
        return cv::Point3f(depth * x, depth * y, depth);
    }

    /**
     * projects an 3D world point to an 2D image point (no distortion!)
     */
    inline static cv::Point2f pointToPixel(const cv::Point3f &point, const tIntrinsics &intrinsic) {
        const float u = (point.x / point.z) * intrinsic.focal.x + intrinsic.principal.x;
        const float v = (point.y / point.z) * intrinsic.focal.y + intrinsic.principal.y;
        return cv::Point2f(u, v);
    }

    /**
     * projects an 3D world point to another point of view (no distortion!)
     * rotation must be orthonormal (as in realsense extrensic)
     */
    inline static cv::Point3f transformPointToPoint(const cv::Point3f &point, const tExtrensics &extrensics) {
        const float x =
                extrensics.rotation[0] * point.x + extrensics.rotation[3] * point.y + extrensics.rotation[6] * point.z +
                extrensics.translation[0];
        const float y =
                extrensics.rotation[1] * point.x + extrensics.rotation[4] * point.y + extrensics.rotation[7] * point.z +
                extrensics.translation[1];
        const float z =
                extrensics.rotation[2] * point.x + extrensics.rotation[5] * point.y + extrensics.rotation[8] * point.z +
                extrensics.translation[2];
        return cv::Point3f(x, y, z);
    }

    /**
     * translates a point, then this point is rotated via grad, then this point is also translated
     */
    inline static cv::Point2f translateAndRotate2DPoint(const cv::Point2f &point, const float &grad,
                                                        const cv::Point2f &translBefore = cv::Point2f(0, 0),
                                                        const cv::Point2f &translAfter = cv::Point2f(0, 0)) {
        float x = cos(grad) * (point.x + translBefore.x) + -sin(grad) * (point.y + translBefore.y);
        float y = sin(grad) * (point.x + translBefore.x) + cos(grad) * (point.y + translBefore.y);
        return cv::Point2f(x + translAfter.x, y + translAfter.y);
    }

    namespace relationPoints {
        enum RelationOfPoints {
            NO_MATCH, OVERLAP, FIRST_CONTAINS_SECOND, SECOND_CONTAINS_FIRST, EQUAL
        };

        /**
         * Calcs relation of 2 points
         * @param p1 Point1
         * @param p2 Point2
         * @param r1 Radious of Point1
         * @param r2 Radius of Point2
         * @param distOut Returns distance
         * @return RelationOfPoints
         */
        inline static RelationOfPoints
        getRelationOf2Points(const cv::Point2f &p1, const cv::Point2f &p2, const float &r1, const float &r2,
                             float &distOut) {
            distOut = static_cast<float>(cv::norm(p1 - p2));
            if (distOut > r1 + r2) {
                return NO_MATCH;
            }
            const auto dR1 = distOut - r1;
            const auto dR2 = distOut - r2;

            if (dR1 >= 0 && dR2 >= 0) {
                return OVERLAP;
            } else if (dR1 >= 0 && dR2 <= 0) {
                return SECOND_CONTAINS_FIRST;
            } else if (dR1 <= 0 && dR2 >= 0) {
                return FIRST_CONTAINS_SECOND;
            }
            return EQUAL;
        }

        /**
         * Calcs relation of 2 points
         * @param p1 Point1
         * @param p2 Point2
         * @param r1 Radious of Point1
         * @param r2 Radius of Point2
         * @return RelationOfPoints
         */
        inline static RelationOfPoints
        getRelationOf2Points(const cv::Point2f &p1, const cv::Point2f &p2, const float &r1, const float &r2) {
            const auto dist = static_cast<float>(cv::norm(p1 - p2));
            if (dist > r1 + r2) {
                return NO_MATCH;
            }
            const auto dR1 = dist - r1;
            const auto dR2 = dist - r2;

            if (dR1 >= 0 && dR2 >= 0) {
                return OVERLAP;
            } else if (dR1 >= 0 && dR2 <= 0) {
                return SECOND_CONTAINS_FIRST;
            } else if (dR1 <= 0 && dR2 >= 0) {
                return FIRST_CONTAINS_SECOND;
            }
            return EQUAL;
        }
    };
};
#endif //AADC_USER_IMAGE_UPDATE_HELPER_H
