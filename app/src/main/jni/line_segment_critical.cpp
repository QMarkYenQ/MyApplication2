#include "line_segment_critical.h"

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
){

    length_Cont.clear();
    distance_Cont.clear();
    arrowRate_Cont.clear();

    for( size_t i = 0 ; i < directForm_Cont.size(); ++i ){
        cv::Vec3f ln_1( line_Cont[i](0,0) - vanishingPoint(0),
                        line_Cont[i](0,1) - vanishingPoint(1), 0 );
        float thd_1 = ln_1.dot(directForm_Cont[i]);
        cv::Vec3f ln_2( line_Cont[i](1,0) - vanishingPoint(0),
                        line_Cont[i](1,1) - vanishingPoint(1) , 0 );
        float thd_2 = ln_2.dot(directForm_Cont[i]);
        float ds = abs( vanishingPoint.dot( normalForm_Cont[i]));

        float lth = abs( thd_1 - thd_2 );
        float rat = 1;
        if( ds > 0.000001F){
            float td_1 = abs( thd_1/ds );
            float td_2 = abs( thd_2/ds );

            float d_1 =  td_1*atan(td_1) - 0.5F*log( 1 + td_1*td_1 );
            float d_2 =  td_2*atan(td_2) - 0.5F*log( 1 + td_2*td_2 );
            rat = abs( 2*ds*( d_1- d_2 )/( 3.1415926F*(thd_1-thd_2)) );

            /*
            if( ( thd_1 >0 && thd_2 >0 ) || ( thd_1 <0 && thd_2 <0 ) ){

            }else{
                if( abs( thd_1 )> abs( thd_2 )){
                    float d_1 =  td_1*atan(td_1) - 0.5F*log( 1 + td_1*td_1 );
                    rat = abs( 2*ds*( d_1)/( 3.1415926F*(thd_1)) );
                }else{
                    float d_2 =  td_1*atan(td_2) - 0.5F*log( 1 + td_2*td_2 );
                    rat = abs( 2*ds*( d_2)/( 3.1415926F*(thd_2)) );
                }
            }
            */
        }else{
            rat = abs( abs( thd_1 ) - abs( thd_2 ) / ( thd_1 - thd_2 ) );
        }
        length_Cont.push_back( lth );
        distance_Cont.push_back(ds);
        arrowRate_Cont.push_back(rat);
    }
}
