#ifndef LINE_SEGMENT_NORMAL_FORM
#define LINE_SEGMENT_NORMAL_FORM

#include <opencv2/core/core.hpp>

float LineSegmentNormalForm(
    std::vector<cv::Matx23f> &line_Cont,
    std::vector<cv::Vec3f> &normalForm_Cont,
    std::vector<cv::Vec3f> &directForm_Cont,
    std::vector<float> &length_Cont,
    float  minLength
);
#endif // LINE_SEGMENT_NORMAL_FORM

