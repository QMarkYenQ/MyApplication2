#include "line_segment.h"
#include "EDLineDetector.h"


void LineSegment(

    cv::Mat image,
    std::vector<cv::Matx23f> &lineTwoPoints
){
    lineTwoPoints.clear();


    EDLineDetector Ed;

    Ed.EDline(image);
    cv::Matx23f line;
    for (int i = 0; i < Ed.lineEndpoints_.size(); i++) {
        line = cv::Matx23f(
                Ed.lineEndpoints_[i][0], Ed.lineEndpoints_[i][1], 1.0, Ed.lineEndpoints_[i][2],
                Ed.lineEndpoints_[i][3], 1.0);
        lineTwoPoints.push_back(line);
    }






 /*
   EDLineDetector Ed;
 lineTwoPoints.clear();

   Ed.EDline( image );

   cv::Matx23f line;

    for( int i = 0; i <  Ed.lineEndpoints_.size(); i++ ){



        line = cv::Matx23f(
                    Ed.lineEndpoints_[i][0]
    ,  Ed.lineEndpoints_[i][1], 1.0, Ed.lineEndpoints_[i][2],  Ed.lineEndpoints_[i][3], 1.0 );

        lineTwoPoints.push_back( line );

    }
    qDebug()<<Ed.lineEndpoints_.size();
 */


/*
    int pNoLS = 0;

 cv::Size2i sz= image.size();
 LS *ls0 = DetectLinesByED( image.ptr<uchar>(0),sz.width, sz.height , &pNoLS );
 //
 lineTwoPoints.clear();
 cv::Matx23f line;
 for( int i = 0; i < pNoLS; i++ ){
     line = cv::Matx23f( ls0[i].ex, ls0[i].ey, 1.0, ls0[i].sx, ls0[i].sy, 1.0 );
     lineTwoPoints.push_back( line );
 }
 delete ls0;
*/
}

