#ifndef LINE_SEGMENT_CRITICAL
#define LINE_SEGMENT_CRITICAL


#include <opencv2/core/core.hpp>

void LineSegmentCritical
(
    std::vector<cv::Matx23f> &line_Cont,
    std::vector<cv::Vec3f> &normalForm_Cont,
    std::vector<cv::Vec3f> &directForm_Cont,
    //
    cv::Vec3f &vanishingPoint,
    //
    std::vector<float> &length_Cont,
    std::vector<float> &distance_Cont,
    std::vector<float> &arrowRate_Cont
);






#endif // LINE_SEGMENT_CRITICAL

