#include "vanishing_point_module.h"

vanishing_point_module::vanishing_point_module()
    :vp_Confirm(0,0,1), info_Confirm(0,0,0,0)
    ,vp_Maintain(0,0,1), info_Maintain(0,0,0,0)
    ,vp_Immedi(0,0,1), info_Immedi(0,0,0,0),info_Immedi_Decay(0,0,0,0)
    ,vp_Complete(0,0,1), info_Complete(0,0,0,0)
    ,phase(0)
    ,inlier(0)
    ,nonConsist(0)
{

    line_segment_Cont.clear();


    camPart = {
        640,480,
        2.4, 1.8, 1.2, 0.9,
        0, 0., 0.0,
        0,0,1.2,
        0.,0.
    };

    dspPart = {
        0,0,1,
        (3.14159)/2,
        256,256 ,
        6, 3, 3, 3
    };
}

vanishing_point_module::~vanishing_point_module()
{

}


void vanishing_point_module::line_segment_selection( cv::Mat &frame )
{
    // step N. LineSegment
    LineSegment
    (
       frame
       ,line_segment_source_Cont
    );
    // step N. TransformationCoordinate
    TransformationCoordinate trnas;
    std::vector<cv::Matx23f> line_lane_Cont;
    {
        trnas.camPart = camPart;
        trnas.dspPart = dspPart;

        trnas.transTowPoints_Pho2Lane
        (
            line_segment_source_Cont
            ,line_lane_Cont
        );
    }
    // step N. CategorizationLineSegment
    std::vector<char> select_Cont;
    {
        cv::Vec3f offset =  cv::Vec3f(0,0.3F,1);
        float tight = -0.05;
        CategorizationLineSegment
        (
            line_lane_Cont
            ,offset
            ,tight
            ,select_Cont
        );
    }
    // step N. line_segment_image_Cont
    line_segment_image_Cont.clear();
    for( size_t i = 0; i < line_segment_source_Cont.size(); i++ ){
        if( select_Cont[i] )
            line_segment_image_Cont.push_back
                ( line_segment_source_Cont[i] );
    }
    // step N. line_segment_Cont
    line_segment_Cont.clear();
    trnas.transTowPoints_Pho2Car
    (
        line_segment_image_Cont
        ,line_segment_Cont
    );
    // step N. LineSegmentNormalForm
    LineSegmentNormalForm
    (
        line_segment_Cont
        ,normalForm_Cont, directForm_Cont, length_Cont,
        0.0005F
    );

}

void vanishing_point_module::detection()
{
    // step N. VanishingPointRANSAC
    char is_valid = 0;
    {
        std::vector<char> valid_Cont;
        float ratioInlier = 0;

        VanishingPointRANSAC
        (
            normalForm_Cont, length_Cont
            ,0.1F, 0.1F
            ,is_valid, vp_Immedi, ratioInlier, valid_Cont
        );
    }
    // step N.  camPart.angle
    if( is_valid )
    {
        camPart.angle_yaw = atan(vp_Immedi[0]);
        camPart.angle_pitch = atan(vp_Immedi[1]);
        phase = 1;
    }
}

void vanishing_point_module::capture()
{
    // step N. LineSegmentCritical
    LineSegmentCritical
    (
        line_segment_Cont, normalForm_Cont, directForm_Cont
        ,vp_Immedi
        ,length_Cont, distance_Cont, arrowRate_Cont
    );

    // step N. LineSegmentValid2
    float ratioInlier = 0;
    float ratioInlier2 = 0;
    LineSegmentValid2
    (
        length_Cont, distance_Cont, arrowRate_Cont
        ,0.125F, 0.8F, 0.0005F
        ,ratioInlier, ratioInlier2
        ,valid_Cont
    );
    // step N. InformationMatrixFromLS
    cv::Matx22f infoMatrix(0,0,0,0);
    cv::Matx21f supposeVector(0,0);
    InformationMatrixFromLS2
    (
        normalForm_Cont, length_Cont
        ,arrowRate_Cont, valid_Cont
        ,infoMatrix, supposeVector
    );
    info_Immedi = infoMatrix;
    // step N. vp and eigenValue
    cv::Matx21f vp(0,0);
    cv::Vec2f eigenValue;
    {
        vp = infoMatrix.solve( supposeVector, cv::DECOMP_SVD );
        vp_Immedi(0) = vp(0);
        vp_Immedi(1) = vp(1);
        vp_Immedi(2) = 1;
        cv::SVD svd( infoMatrix );
        eigenValue = svd.w;
    }

    if(
        eigenValue(0) > 1.6
     && eigenValue(1) > 0.5
     && ratioInlier > 0.8
     && ratioInlier2 < 0.05
    ){
        phase = 2;
        // Confirm
        vp_Confirm(0) = vp(0);
        vp_Confirm(1) = vp(1);
        vp_Confirm(2) = 1;
        info_Confirm = cv::Matx22f(0,0,0,0);
        // Complete
        vp_Complete(0) = vp(0);
        vp_Complete(1) = vp(1);
        vp_Complete(2) = 1;
        info_Complete = cv::Matx22f(0,0,0,0);

        camPart.angle_yaw = atan(vp(0));
        camPart.angle_pitch = atan(vp(1));
    }else{
        phase = 0;
        //
        camPart.angle_yaw = atan(vp_Maintain(0));
        camPart.angle_pitch = atan(vp_Maintain(1));
    }
}

void vanishing_point_module::confirm()
{
    // step N. LineSegmentCritical
    LineSegmentCritical
    (
        line_segment_Cont, normalForm_Cont, directForm_Cont
        ,vp_Complete
        ,length_Cont, distance_Cont, arrowRate_Cont
    );
    // step N. LineSegmentCritical

    float ratioInlier, ratioInlier2;
    LineSegmentValid2
    (
        length_Cont, distance_Cont, arrowRate_Cont
        ,0.125F, 0.8F, 0.0005F
        ,ratioInlier, ratioInlier2,
        valid_Cont
    );

    cv::Matx22f infoMatrix_Immediate(0,0,0,0);
    cv::Matx21f supposeVector_Immediate(0,0);
    InformationMatrixFromLS2
    (
        normalForm_Cont, length_Cont, arrowRate_Cont,
        valid_Cont,
        infoMatrix_Immediate, supposeVector_Immediate
    );
    //
    cv::Matx21f vp_Immediate = infoMatrix_Immediate.solve( supposeVector_Immediate, cv::DECOMP_SVD );
    vp_Immedi(0) = vp_Immediate(0);
    vp_Immedi(1) = vp_Immediate(1);
    vp_Immedi(2) = 1;
    info_Immedi = infoMatrix_Immediate;

    cv::SVD svd_Immediate( infoMatrix_Immediate );
    cv::Vec2f eigenValue_Immediate = svd_Immediate.w;
    //
    cv::Matx22f decayQ( 2,0,0,2 );
    cv::Matx22f infoMatrix_Immediate_decay = info_Immedi;

    InformationMatrixDecay
    (
        infoMatrix_Immediate_decay,
        decayQ
    );
    if( ratioInlier<0.75 &&  ratioInlier2 > 0.05 ){
        infoMatrix_Immediate_decay= cv::Matx22f(0,0,0,0);
    }

    svd_Immediate( infoMatrix_Immediate_decay );


   cv::Vec2f eigenValue_Immediate_decay= svd_Immediate.w;

    cv::Matx22f decayN = decayQ - infoMatrix_Immediate_decay;
    cv::Matx22f infoMatrix_Maintain_Decay = info_Confirm;



    InformationMatrixDecay
    (
        infoMatrix_Maintain_Decay,
        decayN
    );
    //
     cv::Matx21f eigenValue;

    {
        cv::Matx21f vansih_Maintain( vp_Confirm(0),vp_Confirm(1) );
        cv::Matx21f supposeVector_Maintain_Decay = infoMatrix_Maintain_Decay * vansih_Maintain;
        cv::Matx21f supposeVector_Immediate_decay = infoMatrix_Immediate_decay* vp_Immediate;
        cv::Matx21f supposeVector_Complement = supposeVector_Maintain_Decay + supposeVector_Immediate_decay;
        cv::Matx22f infoMatrix_Complement = infoMatrix_Maintain_Decay + infoMatrix_Immediate_decay  ;

        cv::Matx21f vp_Complement = infoMatrix_Complement.solve(supposeVector_Complement,cv::DECOMP_SVD);
        cv::SVD svd( infoMatrix_Complement );
        eigenValue = svd.w;
        vp_Complete(0) = vp_Complement(0);
        vp_Complete(1) = vp_Complement(1);
        vp_Complete(2) = 1;

    }
    //

    if( eigenValue(0)>0.6 &&eigenValue(1)>0.3)
    {
        camPart.angle_yaw   = atan(vp_Complete[0]);
        camPart.angle_pitch = atan(vp_Complete[1]);

        if( eigenValue_Immediate(0)>1 && eigenValue_Immediate(1)>0.3  ){

            cv::Matx21f supposeVector_Immediate_decay = infoMatrix_Immediate_decay * vp_Immediate;

            cv::Matx21f vansih_Maintain( vp_Confirm(0),vp_Confirm(1) );
            cv::Matx21f supposeVector_Maintain = info_Confirm * vansih_Maintain;

            cv::Matx21f supposeVector_Complement = supposeVector_Maintain + supposeVector_Immediate_decay;
            cv::Matx22f infoMatrix_Complement = info_Confirm + infoMatrix_Immediate_decay;
            cv::Matx21f vp_Complement  = infoMatrix_Complement.solve( supposeVector_Complement,cv::DECOMP_SVD);
            //
            info_Confirm = infoMatrix_Complement;
            vp_Confirm(0) = vp_Complement(0);
            vp_Confirm(1) = vp_Complement(1);
            vp_Confirm(2) = 1;
        }
    }
    else{
        camPart.angle_yaw = atan(vp_Maintain[0]);
        camPart.angle_pitch = atan(vp_Maintain[1]);
        phase = 0;
    }



    {
            cv::SVD svd( info_Confirm );
            cv::Vec2f eigenValue = svd.w;

            if( eigenValue(0)> 1. && eigenValue(1)>0.5 ){
                phase = 3;
                info_Maintain = info_Confirm;
                vp_Maintain = vp_Confirm;
            }
    }



}

void vanishing_point_module::countiuns(){
    // step N. LineSegmentCritical
    LineSegmentCritical
    (
        line_segment_Cont, normalForm_Cont, directForm_Cont,
        vp_Complete,        // *
        length_Cont, distance_Cont, arrowRate_Cont
    );
    // step N. LineSegmentValid2
    float ratioInlier, ratioInlier2;
    LineSegmentValid2
    (
        length_Cont, distance_Cont, arrowRate_Cont,
        0.125F, 0.8F, 0.0005F,     // *
        ratioInlier, ratioInlier2,
        valid_Cont
    );
    inlier = ratioInlier;
    nonConsist = ratioInlier2;
    //

    cv::Matx22f infoMatrix_Immediate(0,0,0,0);
    cv::Matx21f supposeVector_Immediate(0,0);
    InformationMatrixFromLS2
    (
        normalForm_Cont, length_Cont, arrowRate_Cont,
        valid_Cont,
        infoMatrix_Immediate, supposeVector_Immediate
    );


    cv::Matx21f temp = infoMatrix_Immediate*supposeVector_Immediate;
    float abc = temp.dot(supposeVector_Immediate);

    //cv::SVD svd_temp( infoMatrix_temp );
     //cv::Vec2f eigenValue_temp = svd_temp.w;

    //qDebug()<<"test"<<eigenValue_temp(0)<<eigenValue_temp(1);

    //qDebug()<<"test0"<<abc;
    //
    std::vector<float> OneRate_Cont;
    OneRate_Cont.clear();
    for( size_t i = 0 ; i < arrowRate_Cont.size(); ++i ){
        OneRate_Cont.push_back( 1- arrowRate_Cont[i] );
    }



          cv::Matx22f infoMatrix_temp(0,0,0,0);
          cv::Matx21f supposeVector_temp(0,0);
          InformationMatrixFromLS2
          (
              normalForm_Cont, length_Cont, OneRate_Cont,
              //
              valid_Cont,
              //
              infoMatrix_temp, supposeVector_temp
          );

            temp = infoMatrix_temp*supposeVector_temp;
          abc = temp.dot(supposeVector_temp);

         //cv::SVD svd_temp( infoMatrix_temp );
          //cv::Vec2f eigenValue_temp = svd_temp.w;

         //qDebug()<<"test"<<eigenValue_temp(0)<<eigenValue_temp(1);

//qDebug()<<"test"<<abc<<supposeVector_temp(0)<<supposeVector_temp(1);
    //
    cv::Matx21f vp_Immediate
        = infoMatrix_Immediate.solve( supposeVector_Immediate, cv::DECOMP_SVD );
    //
    info_Immedi = infoMatrix_Immediate;
    vp_Immedi(0) = vp_Immediate(0);
    vp_Immedi(1) = vp_Immediate(1);
    vp_Immedi(2) = 1;
    //
    cv::SVD svd_Immediate( infoMatrix_Immediate );
    cv::Vec2f eigenValue_Immediate = svd_Immediate.w;
    //
    cv::Matx22f decayQ( 2,0,0,2 );
    cv::Matx22f infoMatrix_Immediate_decay = infoMatrix_Immediate;
    //
    InformationMatrixDecay(
        infoMatrix_Immediate_decay,
        decayQ
    );


    if( ratioInlier<0.75 &&  ratioInlier2 > 0.05 ){
        infoMatrix_Immediate_decay= cv::Matx22f(0,0,0,0);
    }


    info_Immedi_Decay = infoMatrix_Immediate_decay;


     svd_Immediate( infoMatrix_Immediate_decay );


    cv::Vec2f eigenValue_Immediate_decay= svd_Immediate.w;

    //
    cv::Matx22f decayN = decayQ - infoMatrix_Immediate_decay;
    cv::Matx22f infoMatrix_Maintain_Decay = info_Maintain;
    InformationMatrixDecay
    (
        infoMatrix_Maintain_Decay,
        decayN
    );
    //
    cv::Matx21f eigenValue_Complement;
    {
        cv::Matx21f vansih_R( vp_Maintain(0),vp_Maintain(1) );
        cv::Matx21f supposeVector_RD = infoMatrix_Maintain_Decay * vansih_R;
        cv::Matx21f supposeVector_ID = infoMatrix_Immediate_decay* vp_Immediate;
        cv::Matx21f supposeVector_C = supposeVector_RD + supposeVector_ID;
        cv::Matx22f infoMatrix_C = infoMatrix_Maintain_Decay + infoMatrix_Immediate_decay  ;
        cv::Matx21f vansih_C = infoMatrix_C.solve(supposeVector_C,cv::DECOMP_SVD);
        cv::SVD svd( infoMatrix_C );
        eigenValue_Complement = svd.w;
        vp_Complete(0) = vansih_C(0);
        vp_Complete(1) = vansih_C(1);
        vp_Complete(2) = 1;
        info_Complete = infoMatrix_C;
    }
    //
    if( eigenValue_Complement(0)>0.6F &&eigenValue_Complement(1)>0.3F )
    {
        camPart.angle_yaw   = atan(vp_Complete[0]);
        camPart.angle_pitch = atan(vp_Complete[1]);

        if( eigenValue_Immediate(0)>1.0F && eigenValue_Immediate(1)>0.3F ){

            cv::Matx21f supposeVector_Immediate_decay = infoMatrix_Immediate_decay * vp_Immediate;

            cv::Matx21f vansih_Maintain( vp_Maintain(0),vp_Maintain(1) );
            cv::Matx21f supposeVector_Maintain = info_Maintain * vansih_Maintain;

            cv::Matx21f supposeVector_Complement = supposeVector_Maintain + supposeVector_Immediate_decay;
            cv::Matx22f infoMatrix_Complement = info_Maintain + infoMatrix_Immediate_decay;
            cv::Matx21f vp_Complement  = infoMatrix_Complement.solve( supposeVector_Complement,cv::DECOMP_SVD);
            //
            info_Maintain = infoMatrix_Complement;
            vp_Maintain(0) = vp_Complement(0);
            vp_Maintain(1) = vp_Complement(1);
            vp_Maintain(2) = 1;
        }
    }
    else{
        camPart.angle_yaw = atan(vp_Maintain[0]);
        camPart.angle_pitch = atan(vp_Maintain[1]);
        phase = 0;
    }


}
void vanishing_point_module:: run( cv::Mat &frame ){


    line_segment_selection( frame );

    switch ( phase ){

        case 0:
             detection();

    {
             cv::Matx21f vp(vp_Maintain(0),vp_Maintain(1));
             cv::Matx21f supposeVectorb = info_Maintain*vp;
             cv::Matx22f decayQ3(0.05,0,0,0.05);
             cv::Matx22f infoMatrixb = info_Maintain + decayQ3;
             cv::Matx21f vp3 = infoMatrixb.solve(supposeVectorb,cv::DECOMP_SVD);
             vp_Maintain(0) = vp3(0);
             vp_Maintain(1) = vp3(1);
             vp_Maintain(2) = 1;
    }

        break;

        case 1:
             capture();
    {
             cv::Matx21f vp(vp_Maintain(0),vp_Maintain(1));
             cv::Matx21f supposeVectorb = info_Maintain*vp;
             cv::Matx22f decayQ3(0.05,0,0,0.05);
             cv::Matx22f infoMatrixb = info_Maintain + decayQ3;
             cv::Matx21f vp3 = infoMatrixb.solve(supposeVectorb,cv::DECOMP_SVD);
             vp_Maintain(0) = vp3(0);
             vp_Maintain(1) = vp3(1);
             vp_Maintain(2) = 1;
    }

        break;

        case 2:
             confirm();
        break;

        case 3:
             countiuns();
        break;

    }


    cv::Matx22f decayQ2( 3,0,0,25 );
    InformationMatrixDecay
    (
        info_Maintain,
        decayQ2
    );

    if(phase ==2){


        cv::Matx22f decayQ3( 3,0,0,25 );
        InformationMatrixDecay
        (
            info_Confirm,
            decayQ3
        );


    }

}
