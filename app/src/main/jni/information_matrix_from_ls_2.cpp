#include "information_matrix_from_ls_2.h"

void InformationMatrixFromLS2(
        std::vector<cv::Vec3f> &normalForm_Cont,
        std::vector<float> &length_Cont,
        std::vector<float> &arrowRate_Cont,
        std::vector<char> &valid_Cont,
        cv::Matx22f &infoMatrix,
        cv::Matx21f &supposeVector
){

    infoMatrix.zeros();
    supposeVector.zeros();
    for( size_t i = 0 ; i < normalForm_Cont.size(); ++i ){


        if( valid_Cont[i] ){
            float wt= length_Cont[i]*arrowRate_Cont[i];
            cv::Vec2f vec( normalForm_Cont[i](0), normalForm_Cont[i](1) );
            infoMatrix +=  wt * vec * vec.t() ;
            supposeVector -=  normalForm_Cont[i](2)*wt* vec;
        }
     }

}
