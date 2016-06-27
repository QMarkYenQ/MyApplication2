#ifndef VANISHING_POINT_MODULE_H
#define VANISHING_POINT_MODULE_H

#include <opencv2/core/core.hpp>
#include "transformation_coordinate.h"
#include "line_segment.h"
#include "categorization_line_segment.h"
#include "line_segment_normal_form.h"
#include "vanishing_point_ransac.h"
#include "line_segment_valid_2.h"
#include "line_segment_critical.h"
#include "information_matrix_from_ls_2.h"
#include "information_matrix_decay.h"

class vanishing_point_module
{

    std::vector<cv::Matx23f> line_segment_Cont;

    std::vector<cv::Vec3f> normalForm_Cont;
    std::vector<cv::Vec3f> directForm_Cont;
    std::vector<float> arrowRate_Cont;
    std::vector<float> distance_Cont;
    std::vector<float> length_Cont;

    void line_segment_selection( cv::Mat &frame );
//
    void detection();
    void capture();
    void confirm();
    void countiuns();


public:

    Camera camPart;
    Dsiplay dspPart;
    //

    cv::Vec3f     vp_Immedi;
    cv::Matx22f info_Immedi;
    float trace_Remain;
    cv::Matx22f info_Immedi_Decay;

    //float

    cv::Vec3f    vp_Complete;
    cv::Matx22f  info_Complete;

    float inlier;
    float nonConsist;


    cv::Vec3f     vp_Confirm;
    cv::Matx22f info_Confirm;



    cv::Vec3f     vp_Maintain;
    cv::Matx22f info_Maintain;
    std::vector<cv::Matx23f> line_segment_image_Cont;
    std::vector<char> valid_Cont;



    std::vector<cv::Matx23f> line_segment_source_Cont;


    char phase;

//
    void run( cv::Mat &frame );


    vanishing_point_module();
    ~vanishing_point_module();
};

#endif // VANISHING_POINT_MODULE_H
