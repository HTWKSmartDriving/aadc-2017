#include "HTWKVisionUtils.h"

const int HTWKVisionUtils::Yen(const cv::Mat &image) {
    /// Establish the number of bins
    const int histSize = 256;
    /// Set the ranges ( for B,G,R) )
    const float range[] = {0, 256};
    const float *histRange = {range};

    cv::Mat data;
    cv::calcHist(&image, 1, 0, cv::Mat(), data, 1, &histSize, &histRange);
    //Ignore full Black and White
    data.at<float>(0) = 0;
    data.at<float>(255) = 0;

    float norm_histo[histSize]; /* normalized histogram */
    float P1[histSize]; /* cumulative normalized histogram */
    float P1_sq[histSize];
    float P2_sq[histSize];

    int total = 0;
    for (int ih = 0; ih < histSize; ih++) {
        total += data.at<float>(ih);
    }
    for (int ih = 0; ih < histSize; ih++) {
        norm_histo[ih] = data.at<float>(ih) / total;
    }

    P2_sq[histSize - 1] = 0.0;
    for (int ih = histSize - 2; ih >= 0; ih--) {
        P2_sq[ih] = P2_sq[ih + 1] + norm_histo[ih + 1] * norm_histo[ih + 1];
    }

    P1[0] = norm_histo[0];
    P1_sq[0] = norm_histo[0] * norm_histo[0];
    float maxCrit = calcCrit(P1[0], P1_sq[0], P2_sq[0]);
    int threshold = 0;
    float crit = 0;
    for (int ih = 1; ih < histSize; ih++) {
        P1[ih] = P1[ih - 1] + norm_histo[ih];
        P1_sq[ih] = P1_sq[ih - 1] + norm_histo[ih] * norm_histo[ih];
        /* Find the threshold that maximizes the criterion */
        crit = calcCrit(P1[ih], P1_sq[ih], P2_sq[ih]);
        if (crit > maxCrit) {
            maxCrit = crit;
            threshold = ih;
        }
    }
    return threshold;
}

inline const float HTWKVisionUtils::calcCrit(const float &P1, const float &P1_sq, const float &P2_sq) {
    return -1.0f * ((P1_sq * P2_sq) > 0.0f ? log(P1_sq * P2_sq) : 0.0f) +
           2 * ((P1 * (1.0f - P1)) > 0.0f ? log(P1 * (1.0f - P1)) : 0.0f);
}


const int HTWKVisionUtils::GetMatType(tInt16 bitsPerPixel)
{
    switch (bitsPerPixel)
    {
        case (24): // rgb image
            return CV_8UC3;
        case (16): // depth image
            return CV_16UC1;
        case (8): // grayscale image
            return CV_8UC1;
        default:
            return 0;
    }
}

const cv::Mat HTWKVisionUtils::ExtractImageFromMediaSample(IMediaSample *mediaSample, const tBitmapFormat &inputFormat)
{
    int depth = 0;
    int channel = 0;
    int matType = HTWKVisionUtils::GetMatType(inputFormat.nBitsPerPixel);

    switch (matType)
    {
        case (CV_8UC3):
            depth = IPL_DEPTH_8U;
            channel = 3;
            break;
        case (CV_16UC1):
            depth = IPL_DEPTH_16U;
            channel = 1;
            break;
        case (CV_8UC1):
            depth = IPL_DEPTH_8U;
            channel = 1;
            break;
        default:
            return cv::Mat();
    }

    const tVoid *buffer;

    if (IS_OK(mediaSample->Lock(&buffer)))
    {
        IplImage *imageBuffer = cvCreateImageHeader(cvSize(inputFormat.nWidth, inputFormat.nHeight), depth, channel);
        imageBuffer->imageData = (char *) buffer;
        cv::Mat image(cv::cvarrToMat(imageBuffer));
        cvReleaseImage(&imageBuffer);
        mediaSample->Unlock(buffer);

        return image;
    }

    return cv::Mat();
}

