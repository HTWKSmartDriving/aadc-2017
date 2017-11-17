#include "imageBasedFreeSpaceComputation.h"

FreeSpace::FreeSpace(const tCameraProperties &cameraProperties, const FreeSpaceParameter &freeSpaceParameter)
        : cProp(cameraProperties), freeSpaceParam(freeSpaceParameter) {
}

void FreeSpace::compute(const Mat1f &disparityMapTransposed) {
    if (lowerPath.empty()) {
        createRoadDisparityVector(disparityMapTransposed.cols);
        initVTop(disparityMapTransposed.cols); // init vTopology;
        lowerPath.resize(disparityMapTransposed.rows, 0);
    }
    Mat1f score;
    initScore(disparityMapTransposed, score); //Road Penality Score
    calcScore(disparityMapTransposed, score);
    extractLowerPath(disparityMapTransposed, score);
}

void FreeSpace::calcScore(const Mat1f &disparityMapTransposed, Mat1f &score) {
#pragma omp parallel for schedule(static, 8) num_threads(4)
    for (int u = 0; u < disparityMapTransposed.rows; u++) {
        vector<float> integralRoadDiff(disparityMapTransposed.cols, 0);
        float integralRoadDiffOld = 0;
        for (int v = vHorizon; v < disparityMapTransposed.cols; v++) {
            float roadDiff = disparityMapTransposed[u][v] > 0.f ? fabsf(disparityMapTransposed[u][v] - roadDisparity[v])
                                                                : SCORE_DEFAULT;
            integralRoadDiff[v] = integralRoadDiffOld + roadDiff;
            integralRoadDiffOld = integralRoadDiff[v];
        }

        for (int vBottom = vHorizon; vBottom < disparityMapTransposed.cols; vBottom++) {
            // calculate the object score
            float objectScore = 0;
            for (int v = vTopology[vBottom]; v < vBottom; ++v) {
                if (disparityMapTransposed[u][v] > 0.f) {
                    objectScore += fabsf(disparityMapTransposed[u][v] - roadDisparity[vBottom]);
                }
            }
            // calculate the road score
            const float roadScore = integralRoadDiff[disparityMapTransposed.cols - 1] - integralRoadDiff[vBottom - 1];

            score[u][vBottom] = PARAMETER_OBJECT * objectScore + PARAMETER_ROAD * roadScore;
        }
    }
}

void FreeSpace::extractLowerPath(const Mat1f &disparityMapTransposed, Mat1f &score) {
    Mat1i table = Mat1i::zeros(score.size());

    // forward path
    for (int u = 1; u < disparityMapTransposed.rows; u++) {
        for (int v = vHorizon; v < disparityMapTransposed.cols; v++) {
            const float disparity = disparityMapTransposed[u][v];
            const int vTop = max(v - freeSpaceParam.maxPixelJump, vHorizon);
            const int vBottom = min(v + freeSpaceParam.maxPixelJump + 1, disparityMapTransposed.cols);

            float minScore = FLT_MAX;
            int minV = 0;

            for (int rangePos = vTop; rangePos < vBottom; rangePos++) {
                const float disparityDistance = disparityMapTransposed[u - 1][rangePos];
                const float disparityJump = fabsf(disparityDistance - disparity);
                const float penalty = min(freeSpaceParam.parameterOne * disparityJump,
                                          static_cast<float>(freeSpaceParam.parameterOne *
                                                             freeSpaceParam.parameterTwo));
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

    // backward path
    float minScore = FLT_MAX;
    int minV = 0;
    for (int v = vHorizon; v < disparityMapTransposed.cols; v++) {
        if (score[disparityMapTransposed.rows - 1][v] < minScore) {
            minScore = score[disparityMapTransposed.rows - 1][v];
            minV = v;
        }
    }
    for (int u = disparityMapTransposed.rows - 1; u >= 0; u--) {
        lowerPath[u] = minV;
        minV = table[u][minV];
    }
}

void FreeSpace::initVTop(const int &vMax) {// compute search range
    vTopology.resize(vMax, 0);

    for (int vBottom = vHorizon; vBottom < vMax; vBottom++) {
        const float factor =
                (cProp.base / roadDisparity[vBottom]) * (cProp.inInfraredLeft.focal.x / cProp.inInfraredLeft.focal.y);
        const float yBottom = factor * (vBottom - cProp.inInfraredLeft.principal.y);
        const float zBottom = factor * cProp.inInfraredLeft.focal.y;
        const float yTop = yBottom - freeSpaceParam.objectHeight;
        const float vTop = cProp.inInfraredLeft.focal.y * (yTop / zBottom) + cProp.inInfraredLeft.principal.y;
        vTopology[vBottom] = max(cvRound(vTop), 0);
    }
}

void FreeSpace::initScore(const Mat1f &disparityMapTransposed, Mat1f &score) {
    score = Mat1f(disparityMapTransposed.size(), 0);
#pragma omp parallel for schedule(static, 8) num_threads(4)
    for (int vBottom = 0; vBottom < vHorizon; vBottom++) {
        score.col(vBottom) = SCORE_PENALITY_ROAD;
    }
}

void FreeSpace::createRoadDisparityVector(const int &vMax) {
    bool foundPositivDisparity = false;
    for (int v = 0; v < vMax; v++) {
        const float disparity = (cProp.base / cProp.height) * (v - cProp.inInfraredLeft.principal.y);
        roadDisparity.push_back(disparity);
        if (!foundPositivDisparity && disparity >= 0) {
            vHorizon = v;
            foundPositivDisparity = true;
        }
    }
}

const vector<int> &FreeSpace::getComputedLowerPath() {
    return lowerPath;
}
