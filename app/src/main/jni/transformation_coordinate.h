#ifndef TRANSFORMATION_COORDINATE
#define TRANSFORMATION_COORDINATE
#include <opencv2/core/core.hpp>


struct Camera{
    float cols_photo;
    float rows_photo;

    float width_photo;
    float height_photo;
    float point_principle_x_photo;
    float point_principle_y_photo;
    //
    float angle_swing;
    float angle_tilt;
    float angle_pan;
    //
    float offset_x;
    float offset_y;
    float offset_z;
    //
    float angle_pitch;
    float angle_yaw;
};
struct Dsiplay{

    //
    float shift_x;
    float shift_y;
    float shift_z;
    //
    float angle_theta;
    //
    float cols_image;
    float rows_image;

    float width_image;
    float height_image;
    float point_principle_x_image;
    float point_principle_y_image;
};

struct TransformationCoordinate{
    union{
        Camera camPart;
        struct{
            float cols_photo;
            float rows_photo;
            //
            float width_photo;
            float height_photo;
            float point_principle_x_photo;
            float point_principle_y_photo;
            //
            float angle_swing;
            float angle_tilt;
            float angle_pan;
            //
            float offset_x;
            float offset_y;
            float offset_z;
            //
            float angle_pitch;
            float angle_yaw;
        };
    };
    //
    union{
        Dsiplay dspPart;
        struct{

            //
            float shift_x;
            float shift_y;
            float shift_z;
            //
            float angle_theta;
            //
            float cols_image;
            float rows_image;
            //
            float width_image;
            float height_image;
            float point_principle_x_image;
            float point_principle_y_image;
        };
    };

    //
    cv::Matx33f returnMatxAffine_pho();
    //
    cv::Matx33f returnMatxSwing();
    cv::Matx33f returnMatxTilt();
    cv::Matx33f returnMatxPan();
    //
    cv::Matx33f returnMatxOffset();

    //
    cv::Matx33f returnMatxPitch();
    cv::Matx33f returnMatxYaw();
    //
    cv::Matx33f returnMatxShift();
    //
    cv::Matx33f returnMatxTheta();
    //
    cv::Matx33f returnMatxAffine_img();
    //
    cv::Matx33f returnMatx_Pho2Cam();
    cv::Matx33f returnMatx_Cam2Car();
    cv::Matx33f returnMatx_Car2Lane();
    cv::Matx33f returnMatx_Lane2View();
    cv::Matx33f returnMatx_View2Img();
    //
    cv::Matx33f returnMatx_Pho2Car();
    cv::Matx33f returnMatx_Pho2Lane();
    cv::Matx33f returnMatx_Pho2View();
    cv::Matx33f returnMatx_Pho2Img();




    //
    void transTowPoints_Pho2Car(
            std::vector<cv::Matx23f> &line_Pho, std::vector<cv::Matx23f> &line_Car );

    void transTowPoints_Pho2Lane(
            std::vector<cv::Matx23f> &line_Pho, std::vector<cv::Matx23f> &line_Lane );

    void transTowPoints_Pho2View(
            std::vector<cv::Matx23f> &line_Pho, std::vector<cv::Matx23f> &line_View );


    void transTowPoints_Pho2Img(
            std::vector<cv::Matx23f> &line_Pho, std::vector<cv::Matx23f> &line_Img );

};

#endif // TRANSFORMATION_COORDINATE


