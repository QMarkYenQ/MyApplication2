#ifndef CATEGORIZATION_LINE_SEGMENT
#define CATEGORIZATION_LINE_SEGMENT

#include <opencv2/core/core.hpp>
void CategorizationLineSegment(
    std::vector< cv::Matx23f > &lineTwoPoints,
     cv::Vec3f offset,
        float tight,
    std::vector< char > &mask
);
#endif // CATEGORIZATION_LINE_SEGMENT

