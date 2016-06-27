//
// Created by Mark on 2016/5/6.
//

#include "com_example_mark_myapplication_myNDK.h"
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "vanishing_point_module.h"
#include<android/log.h>

#define LOG    "ffmpegDemo-jni" // 这个是自定义的LOG的标识
#define LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG,__VA_ARGS__) // 定义LOGD类型
//#include "EDLineDetector.h"

using namespace cv;


static vanishing_point_module vp_module;


JNIEXPORT jstring JNICALL Java_com_example_mark_myapplication_myNDK_getMystring
        (JNIEnv *env, jobject)
{

    return (*env).NewStringUTF("MY !!  NDKString!!");


}

JNIEXPORT jfloat JNICALL Java_com_example_mark_myapplication_myNDK_geCount
        (JNIEnv *env, jobject)
{
   // return vp_module.phase;

   return 0.5F;
}
JNIEXPORT jint JNICALL Java_com_example_mark_myapplication_myNDK_setone
        (JNIEnv *env, jobject){
   // vp_module.phase =1;

    return 0;


}
JNIEXPORT jintArray JNICALL Java_com_example_mark_myapplication_myNDK_gray
        (JNIEnv *env, jobject obj, jintArray buf, int w, int h)
{
    jint *cbuf;
    cbuf = env->GetIntArrayElements( buf, JNI_FALSE );
    if (cbuf == NULL) {
        return 0;
    }
    Mat imgData(h, w, CV_8UC4, (unsigned char *) cbuf);
    Mat gray_mat, color_mat;
    cvtColor( imgData, gray_mat, CV_RGBA2GRAY );
    cvtColor( imgData, color_mat, CV_RGBA2BGR );


    vp_module.run( gray_mat );

    LOGD(LOG, "这是Debug的信息");


    for( int i = 0; i <  vp_module.line_segment_source_Cont.size(); i++ ){
        cv::Matx23f ln =  vp_module.line_segment_source_Cont[i];
        cv::line( color_mat,
                  cv::Point( ln(0,0), ln(0,1)  ),
                  cv::Point( ln(1,0), ln(1,1)), cv::Scalar(0,0,255), 5, CV_AA);
    }

    // trnasTemp
    TransformationCoordinate trnasTemp;
    trnasTemp.camPart = vp_module.camPart;
    trnasTemp.dspPart = vp_module.dspPart;

    //  if( ! (vp_module.phase == 3 ) ){
    //     trnasTemp.angle_yaw   = atan(vp_module.vp_Maintain(0));
    //    trnasTemp.angle_pitch = atan(vp_module.vp_Maintain(1));
    //}
    // if( vp_module.phase == 3  ){
    cv::Matx33f  maxtran = trnasTemp.returnMatx_Pho2Car().inv();
    cv::Vec3f vectes3;
    vectes3 = maxtran*vp_module.vp_Complete;
    vectes3 =vectes3/vectes3[2];
    cv::circle( color_mat, cv::Point( vectes3[0],  vectes3[1] ),20, cv::Scalar(0,255,255),3);


    //  }

    //  std::vector <cv::Matx23f> lineTwoPoints;
    // lineTwoPoints.clear();
/*

    EDLineDetector Ed;

    Ed.EDline( gray_mat );
    cv::Matx23f line;
    for (int i = 0; i < Ed.lineEndpoints_.size(); i++) {
        line = cv::Matx23f(
                Ed.lineEndpoints_[i][0], Ed.lineEndpoints_[i][1], 1.0, Ed.lineEndpoints_[i][2],
                Ed.lineEndpoints_[i][3], 1.0);
        lineTwoPoints.push_back(line);
    }

    for( int i = 0; i < lineTwoPoints.size(); i++ ){
        cv::Matx23f ln = lineTwoPoints[i];
        cv::line( color_mat,
                  cv::Point( ln(0,0), ln(0,1)  ),
                  cv::Point( ln(1,0), ln(1,1)), cv::Scalar(0,0,255), 5, CV_AA);
    }
*/

    cvtColor( color_mat,  imgData ,  CV_BGR2RGBA );


    int size = w * h;
    jintArray result = env->NewIntArray(size);
    env->SetIntArrayRegion(result, 0, size, cbuf);
    env->ReleaseIntArrayElements(buf, cbuf, 0);
    return result;
}
