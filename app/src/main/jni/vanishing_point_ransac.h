#ifndef VANISHING_POINT_RANSAC
#define VANISHING_POINT_RANSAC

#include <opencv2/core/core.hpp>

void VanishingPointRANSAC
(
        std::vector<cv::Vec3f> &normalForm_Cont,
        std::vector<float> &length_Cont,
        float distanceInlier,
        float angleCross,
        //
        char &is_valid,
        cv::Vec3f &vanishingPoint,
        float &ratioInlier,
        std::vector<char> &valid_Cont
);

#endif // VANISHING_POINT_RANSAC

