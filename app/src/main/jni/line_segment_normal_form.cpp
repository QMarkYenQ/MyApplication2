#include "line_segment_normal_form.h"

float LineSegmentNormalForm(
    std::vector<cv::Matx23f> &line_Cont,
        std::vector<cv::Vec3f> &normalForm_Cont,
        std::vector<cv::Vec3f> &directForm_Cont,

    std::vector<float> &length_Cont,
    float  minLength
)
{
    float totalLength = 0;
    normalForm_Cont.clear();
    directForm_Cont.clear();
    length_Cont.clear();
    for( size_t i = 0; i < line_Cont.size(); ++i ){
        cv::Matx23f line = line_Cont[i];
        cv::Vec3f normalForm( line(0,1)-line(1,1), line(1,0)-line(0,0), 0 );
        cv::Vec3f directForm( line(1,0)-line(0,0), line(1,1)-line(1,2), 0 );

        float length = cv::norm( normalForm );
        if( length > minLength ){
            normalForm(2)= line(0,0) * line(1,1) - line(1,0) * line(0,1);
            normalForm = normalForm / length;

            directForm = directForm / length;

            if( normalForm(2) < 0 ) normalForm = ( -normalForm );

        }
        totalLength += length;
        length_Cont.push_back( length );
        normalForm_Cont.push_back( normalForm );
        directForm_Cont.push_back( directForm );
    }



    return totalLength;
}
