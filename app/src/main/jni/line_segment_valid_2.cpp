#include "line_segment_valid_2.h"

void LineSegmentValid2
(
    std::vector<float> &length_Cont,
    std::vector<float> &distance_Cont,
    std::vector<float> &arrowRate_Cont,
    //
    float distance_ub,
    float headRate_lb,
    float length_lb,
    //
    float &InlierRatio,
    float &InlierRatio_inner,
    std::vector<char> &valid_Cont
){


    float suppose_Best = 0;
    float suppose_Total = 0;
    float suppose_Temp = 0;
    float suppose_Temp_2 = 0;
    float suppose_Temp_3 = 0;


    valid_Cont.clear();
    for( size_t i = 0 ; i < length_Cont.size(); ++i ){
        float wt = length_Cont[i]*arrowRate_Cont[i];

        suppose_Total+= length_Cont[i];
        char act = distance_Cont[i] < distance_ub;
        if( act ){

            suppose_Temp+= wt;
            act = arrowRate_Cont[i] > headRate_lb;
        }

        if( act ){
            suppose_Temp_3+=length_Cont[i];


            suppose_Best += wt;
            suppose_Temp_2+= wt* abs( distance_Cont[i] );

        }

        valid_Cont.push_back( act );


    }

    InlierRatio =suppose_Temp_3/suppose_Total;
   // InlierRatio_inner = suppose_Best/suppose_Temp;



    InlierRatio_inner = suppose_Temp_2/suppose_Best;


}
