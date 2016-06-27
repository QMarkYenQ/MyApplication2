#ifndef INFORMATION_MATRIX_FROM_LS_2_H
#define INFORMATION_MATRIX_FROM_LS_2_H
#include <opencv2/core/core.hpp>

void InformationMatrixFromLS2(
        std::vector<cv::Vec3f> &normalForm_Cont,
        std::vector<float> &length_Cont,
        std::vector<float> &arrowRate_Cont,
        std::vector<char> &valid_Cont,
        cv::Matx22f &infoMatrix,
        cv::Matx21f &supposeVector
        );


#endif // INFORMATION_MATRIX_FROM_LS_2_H

