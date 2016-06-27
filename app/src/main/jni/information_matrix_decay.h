#ifndef INFORMATION_MATRIX_DECAY_H
#define INFORMATION_MATRIX_DECAY_H
#include <opencv2/core/core.hpp>

void InformationMatrixDecay
(
    cv::Matx22f &infoMatrix,
    cv::Matx22f &decayQ
);

#endif // INFORMATION_MATRIX_DECAY_H

