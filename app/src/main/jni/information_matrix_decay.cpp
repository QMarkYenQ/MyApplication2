#include "information_matrix_decay.h"
void InformationMatrixDecay
(
    cv::Matx22f &infoMatrix,
    cv::Matx22f &decayQ
){
    cv::Matx22f O = infoMatrix;
    cv::Matx22f N = O + decayQ;
    cv::Matx22f Pt = N.solve( O,  cv::DECOMP_SVD);
    cv::Matx22f P = Pt.t();
    cv::Matx22f I(1,0,0,1);
    infoMatrix =(I-P)*O;
}
