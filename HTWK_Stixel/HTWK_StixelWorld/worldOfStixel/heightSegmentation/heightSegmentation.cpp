#include "heightSegmentation.h"


HeightSegmentation::HeightSegmentation(const tCameraProperties &cameraProperties,
                                       const HeightSegmentationParameter &segmentationParameter) :
        segParameter(segmentationParameter),
        zDepthFactor(cameraProperties.base * cameraProperties.inInfraredLeft.focal.x) {
}

void HeightSegmentation::compute(const Mat1f &disparityMapTransposed, const vector<int> &lowerPath) {
    Mat1f score = Mat1f(disparityMapTransposed.size(), 0);
    if (upperPath.empty()) {
        upperPath.resize(disparityMapTransposed.rows, 0);
    }
    calcScore(disparityMapTransposed, lowerPath, score);
    extractUpperPath(disparityMapTransposed, lowerPath, score);
}

void HeightSegmentation::calcScore(const Mat1f &disparityMapTransposed, const vector<int> &lowerPath, Mat1f &score) {
#pragma omp parallel for schedule(static, 8) num_threads(4)
    for (int u = 0; u < disparityMapTransposed.rows; u++) {
        const int vBottom = lowerPath[u];
        const float disparityBottom = disparityMapTransposed[u][vBottom];
        vector<float> membershipVector;
        float membershipOld = 0;
        float deltaDisparity = 0;
        if (disparityBottom > 0) {
            const float zBottom = zDepthFactor / disparityBottom;
            const float zBottomDeltaZ = zBottom + segParameter.deltaZ;
            const float disparityBottomDeltaDisparity = zDepthFactor / zBottomDeltaZ;
            deltaDisparity = disparityBottomDeltaDisparity - disparityBottom;
        }

        for (int v = 0; v < disparityMapTransposed.cols; v++) {
            const float disparity = disparityMapTransposed[u][v];

            float membership = 0;
            if (disparityBottom > 0 && disparity > 0) {
                const float vDeltaDisparity = (disparity - disparityBottom) / deltaDisparity;
                const float exponent = 1 - (vDeltaDisparity * vDeltaDisparity);
                membership = powf(2, exponent) - 1;
            }
            membershipOld = membershipOld + membership;
            membershipVector.push_back(membershipOld);
        }

        score[u][0] = membershipVector[vBottom - 1];
        for (int vHorizon = 1; vHorizon < vBottom; vHorizon++) {
            const float score1 = membershipVector[vHorizon - 1];
            const float score2 = membershipVector[vBottom - 1] - score1;
            score[u][vHorizon] = score1 - score2;
        }
    }
}

void
HeightSegmentation::extractUpperPath(const Mat1f &disparityMapTransposed, const vector<int> &lowerPath, Mat1f &score) {
    Mat1i table = Mat1i::zeros(score.size());
    // forward upperpath
    for (int u = 1; u < disparityMapTransposed.rows; u++) {
        const int vBottom = lowerPath[u];
        for (int v = 0; v < vBottom; v++) {
            const float disparity = disparityMapTransposed[u][v];
            const int vTop = max(v - segParameter.maxPixelJump, 0);
            const int vBottomUpdated = min(v + segParameter.maxPixelJump + 1, vBottom);
            float minScore = FLT_MAX;
            int minV = 0;

            for (int rangePos = vTop; rangePos < vBottomUpdated; rangePos++) {
                const float disparityDistance = disparityMapTransposed[u - 1][rangePos];

                float smoothnessCostsZ = 1; //S
                if (disparity > 0 && disparityDistance > 0) {
                    const float Z_u = zDepthFactor / disparity; //Zu
                    const float Z_u_next = zDepthFactor / disparityDistance; //Zu+1
                    smoothnessCostsZ = max(static_cast<float>(0), 1 - (fabsf(Z_u - Z_u_next) / segParameter.N_z));
                }

                const float penalty = segParameter.C_s * abs(v - rangePos) * smoothnessCostsZ;
                const float scoreWithPenalty = score[u - 1][rangePos] + penalty;
                if (scoreWithPenalty < minScore) {
                    minScore = scoreWithPenalty;
                    minV = rangePos;
                }
            }
            score[u][v] += minScore;
            table[u][v] = minV;
        }
    }

    // backward
    float minScore = FLT_MAX;
    int minV = 0;
    for (int v = 0; v < disparityMapTransposed.cols; v++) {
        if (score[disparityMapTransposed.rows - 1][v] < minScore) {
            minScore = score[disparityMapTransposed.rows - 1][v];
            minV = v;
        }
    }
    for (int u = disparityMapTransposed.rows - 1; u >= 0; u--) {
        upperPath[u] = minV;
        minV = table[u][minV];
    }
}

const vector<int> &HeightSegmentation::getComputedUpperPath() {
    return upperPath;
}
