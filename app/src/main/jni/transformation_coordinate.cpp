#include "transformation_coordinate.h"

cv::Matx33f TransformationCoordinate::returnMatxAffine_pho()
{
    float offset_x = -point_principle_x_photo;
    float offset_y = -point_principle_y_photo;
    float scale_x  = width_photo/cols_photo;
    float scale_y  = height_photo/rows_photo;

    cv::Matx33f matx_affine( scale_x, 0, offset_x,
          0, scale_y, offset_y,
          0, 0, 1
        );

    return matx_affine;
}
cv::Matx33f TransformationCoordinate::returnMatxSwing(){

    float cos_swing = cos( angle_swing );
    float sin_swing = sin( angle_swing );

    cv::Matx33f rotMatx_swing(
                cos_swing, -sin_swing, 0,
                sin_swing,  cos_swing, 0,
                0, 0, 1);
    return rotMatx_swing;
}
cv::Matx33f TransformationCoordinate::returnMatxTilt(){

    float cos_tilt = cos( angle_tilt );
    float sin_tilt = sin( angle_tilt );

    cv::Matx33f rotMatx_tilt(
                1, 0, 0,
                0, cos_tilt, -sin_tilt,
                0, sin_tilt,  cos_tilt );
    return rotMatx_tilt;
}



cv::Matx33f TransformationCoordinate::returnMatxOffset(){
    cv::Matx33f rotMatx_offset(
                1, 0, offset_x,
                0, 1, offset_y,
                0, 0, offset_z );
    return rotMatx_offset;
}


cv::Matx33f TransformationCoordinate::returnMatxPan(){
    float cos_pan = cos( angle_pan );
    float sin_pan = sin( angle_pan );

    cv::Matx33f rotMatx_pan(
                cos_pan, 0,  -sin_pan,
                0, 1, 0,
                sin_pan,0 ,  cos_pan );
    return rotMatx_pan;
}
cv::Matx33f TransformationCoordinate::returnMatxPitch(){
    float cos_pitch = cos( angle_pitch );
    float sin_pitch = sin( angle_pitch );

    cv::Matx33f rotMatx_pitch(
                1, 0, 0,
                0, cos_pitch, -sin_pitch,
                0, sin_pitch,  cos_pitch );

    return rotMatx_pitch;
}
cv::Matx33f TransformationCoordinate::returnMatxYaw(){
    float cos_yaw = cos( angle_yaw );
    float sin_yaw = sin( angle_yaw );

    cv::Matx33f rotMatx_yaw(
                cos_yaw, 0,  -sin_yaw,
                0, 1, 0,
                sin_yaw,0 ,  cos_yaw );

    return rotMatx_yaw;
}

cv::Matx33f TransformationCoordinate::returnMatxShift(){
    cv::Matx33f rotMatx_shift(
                1, 0, shift_x,
                0, 1, shift_y,
                0, 0, shift_z );
    return rotMatx_shift;
}

cv::Matx33f TransformationCoordinate::returnMatxTheta(){


    float cos_theta = cos( angle_theta );
    float sin_theta = sin( angle_theta );

    cv::Matx33f rotMatx_theta(
                1, 0, 0,
                0, cos_theta, -sin_theta,
                0, sin_theta,  cos_theta );

    return rotMatx_theta;
}
//
cv::Matx33f TransformationCoordinate::returnMatxAffine_img(){

    float offset_x = -point_principle_x_image;
    float offset_y = -point_principle_y_image;
    float scale_x  = width_image/cols_image;
    float scale_y  = height_image/rows_image;

    cv::Matx33f matx_affine( scale_x, 0, offset_x,
          0, scale_y, offset_y,
          0, 0, 1
        );

    return matx_affine;
}

//
cv::Matx33f TransformationCoordinate::returnMatx_Pho2Cam(){

    return returnMatxAffine_pho();
}
cv::Matx33f TransformationCoordinate::returnMatx_Cam2Car(){

    cv::Matx33f rotMatx_pan = returnMatxPan();
    cv::Matx33f rotMatx_tilt = returnMatxTilt();
    cv::Matx33f rotMatx_swing = returnMatxSwing();
    cv::Matx33f Matx_offset =returnMatxOffset();


    cv::Matx33f a( 1, 0, 0,
                   0, 0, -1,
                   0, 1, 0
                             );

    cv::Matx33f b( 1, 0, 0,
                   0, 0, 1,
                   0, -1, 0
                             );

    return rotMatx_pan * rotMatx_tilt * rotMatx_swing * b*Matx_offset*a;
}
cv::Matx33f TransformationCoordinate::returnMatx_Car2Lane(){

    cv::Matx33f rotMatx_yaw = returnMatxYaw();
    cv::Matx33f rotMatx_pitch = returnMatxPitch();
    cv::Matx33f Matx_shift =returnMatxShift();


    cv::Matx33f a( 1, 0, 0,
                   0, 0, -1,
                   0, 1, 0
                             );

    cv::Matx33f b( 1, 0, 0,
                   0, 0, 1,
                   0, -1, 0
                             );

    return rotMatx_yaw * rotMatx_pitch;
}
cv::Matx33f TransformationCoordinate::returnMatx_Lane2View(){

    cv::Matx33f a( 1, 0, 0,
                   0, 0, -1,
                   0, 1, 0
                             );

    cv::Matx33f b( 1, 0, 0,
                   0, 0, 1,
                   0, -1, 0
                             );
       cv::Matx33f Matx_shift =returnMatxShift();
    cv::Matx33f rotMatx_theta = returnMatxTheta();
    return rotMatx_theta*b*Matx_shift*a;
}

cv::Matx33f TransformationCoordinate::returnMatx_View2Img(){
    return returnMatxAffine_img().inv();
}




//
cv::Matx33f TransformationCoordinate::returnMatx_Pho2View(){

    cv::Matx33f affMatx_Pho2Cam = returnMatx_Pho2Cam();
    cv::Matx33f rotMatx_Cam2Car = returnMatx_Cam2Car();
    cv::Matx33f rotMatx_Car2Lane = returnMatx_Car2Lane();
    cv::Matx33f rotMatx_Lane2View = returnMatx_Lane2View();

    return  rotMatx_Lane2View * rotMatx_Car2Lane * rotMatx_Cam2Car * affMatx_Pho2Cam;

}
cv::Matx33f TransformationCoordinate::returnMatx_Pho2Car(){

     cv::Matx33f affMatx_Pho2Cam = returnMatx_Pho2Cam();
     cv::Matx33f rotMatx_Cam2Car = returnMatx_Cam2Car();
     return  rotMatx_Cam2Car * affMatx_Pho2Cam;
}
cv::Matx33f TransformationCoordinate::returnMatx_Pho2Lane(){

    cv::Matx33f affMatx_Pho2Cam = returnMatx_Pho2Cam();
    cv::Matx33f rotMatx_Cam2Car = returnMatx_Cam2Car();
    cv::Matx33f rotMatx_Car2Lane = returnMatx_Car2Lane();
    return  rotMatx_Car2Lane * rotMatx_Cam2Car * affMatx_Pho2Cam;
}
//
cv::Matx33f TransformationCoordinate::returnMatx_Pho2Img(){

   return returnMatx_View2Img()*returnMatx_Pho2View();
}


void TransformationCoordinate::transTowPoints_Pho2Car(
        std::vector<cv::Matx23f> &line_Pho, std::vector<cv::Matx23f> &line_Car ){
    line_Car.clear();
    cv::Matx23f zero( 0,0,0,0,0,0 );
    float s0 = 0, s1 = 0;
    cv::Matx33f matx = returnMatx_Pho2Car().t();

    cv::Matx23f line( 0,0,0,0,0,0 );

    for(size_t i = 0 ; i<line_Pho.size() ; ++i){
        line = line_Pho[i] * matx;
        s0 = line(0,2);
        s1 = line(1,2);
        if( s0>0 && s1>0 ){
            line(0,0) = line(0,0)/s0;
            line(0,1) = line(0,1)/s0;
            line(0,2) = 1;
            line(1,0) = line(1,0)/s1;
            line(1,1) = line(1,1)/s1;
            line(1,2) = 1;




        }else{
            line = zero;
        }
        line_Car.push_back(line);
    }

}


void TransformationCoordinate::transTowPoints_Pho2Lane(
        std::vector<cv::Matx23f> &line_Pho, std::vector<cv::Matx23f> &line_Lane ){
    line_Lane.clear();
    cv::Matx23f zero( 0,0,0,0,0,0 );
    float s0 = 0, s1 = 0;
    cv::Matx33f matx = returnMatx_Pho2Lane().t();

    cv::Matx23f line( 0,0,0,0,0,0 );

    for(size_t i = 0 ; i<line_Pho.size() ; ++i){
        line = line_Pho[i] * matx;
        s0 = line(0,2);
        s1 = line(1,2);
        if( s0>0 && s1>0 ){
            line(0,0) = line(0,0)/s0;
            line(0,1) = line(0,1)/s0;
            line(0,2) = 1;
            line(1,0) = line(1,0)/s1;
            line(1,1) = line(1,1)/s1;
            line(1,2) = 1;
        }else{
            line = zero;
        }
        line_Lane.push_back(line);
    }

}
void TransformationCoordinate::transTowPoints_Pho2View(
        std::vector<cv::Matx23f> &line_Pho, std::vector<cv::Matx23f> &line_View ){

    cv::Matx23f zero( 0,0,0,0,0,0 );
    float s0 = 0, s1 = 0;
    cv::Matx33f matx = returnMatx_Pho2View().t();

    cv::Matx23f line( 0,0,0,0,0,0 );

    for(size_t i = 0 ; i<line_Pho.size() ; ++i){
        line = line_Pho[i] * matx;
        s0 = line(0,2);
        s1 = line(1,2);
        if( s0>0 && s1>0 ){
            line(0,0) = line(0,0)/s0;
            line(0,1) = line(0,1)/s0;
            line(0,2) = 1;
            line(1,0) = line(1,0)/s1;
            line(1,1) = line(1,1)/s1;
            line(1,2) = 1;
        }else{
            line = zero;
        }
        line_View.push_back(line);
    }


}
void TransformationCoordinate::transTowPoints_Pho2Img(
        std::vector<cv::Matx23f> &line_Pho, std::vector<cv::Matx23f> &line_Img ){

    cv::Matx23f zero( 0,0,0,0,0,0 );
    float s0 = 0, s1 = 0;
    cv::Matx33f matx = returnMatx_Pho2Img().t();

    cv::Matx23f line( 0,0,0,0,0,0 );

    for(size_t i = 0 ; i<line_Pho.size() ; ++i){
        line = line_Pho[i] * matx;
        s0 = line(0,2);
        s1 = line(1,2);
        if( s0>0 && s1>0 ){
            line(0,0) = line(0,0)/s0;
            line(0,1) = line(0,1)/s0;
            line(0,2) = 1;
            line(1,0) = line(1,0)/s1;
            line(1,1) = line(1,1)/s1;
            line(1,2) = 1;
        }else{
            line = zero;
        }
        line_Img.push_back(line);
    }


}
