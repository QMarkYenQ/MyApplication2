#include "categorization_line_segment.h"

void CategorizationLineSegment(
    std::vector< cv::Matx23f > &line,
        cv::Vec3f offset,
           float tight,
    std::vector< char > &mask
){

    mask.clear();


    for( size_t i = 0; i < line.size(); ++i ){
        float a = line[i](0,1) - line[i](1,1);
        float b = line[i](1,0) - line[i](0,0);
        float c = line[i](0,0) * line[i](1,1) - line[i](1,0) * line[i](0,1);
        float d = line[i](1,1) -line[i](0,1);
        float e = 0;
        float offset2 = 0.1;
        if( d > 0  ){
            if(line[i](1,1)>offset2) e = (line[i](1,1)-offset2) /abs(d);
            else e = 0;
        }else{
            if(line[i](0,1)>offset2)e = (line[i](0,1)-offset2) /abs(d);
            else e = 0.;
        }
        cv::Vec2f ln( a, b );
        float lth = cv::norm( ln );
        cv::Vec3f formHessian = cv::Vec3f( a, b, c ) / lth;
        formHessian[2] = formHessian.dot( offset );
        if( formHessian[2] < 0 ) formHessian = ( - formHessian );

        char act = ( e > 0.5) && ( formHessian[1]< 0.98 )
             && ( formHessian[1] > formHessian[2]+ tight );

        mask.push_back( act );
    }
}
