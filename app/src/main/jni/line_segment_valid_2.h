#ifndef LINE_SEGMENT_VALID_2
#define LINE_SEGMENT_VALID_2

#include <opencv2/core/core.hpp>


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
);

#endif // LINE_SEGMENT_VALID_2

