#include "vanishing_point_ransac.h"

void VanishingPointRANSAC(
    std::vector<cv::Vec3f> &normalForm_Cont,
    std::vector<float> &length_Cont,
    float distanceInlier,
    float angleCross,
    //
    char &is_valid,
    cv::Vec3f &vanishingPoint,
    float &ratioInlier,
    std::vector<char> &valid_Cont
) {

    vanishingPoint = cv::Vec3f( 0, 0, 1 );
    is_valid = false;
    valid_Cont.clear();
    valid_Cont.resize( normalForm_Cont.size() );
    //
    float suppose_Total = 0;
    for( size_t i = 0; i < length_Cont.size(); ++i ) suppose_Total += length_Cont[i];
    //
    float suppose_Best = 0;
    size_t atLeastIter = 1;
    float validRateIterNum  = 0;
    float validRateExist = 0.5;

    for(  size_t iterM = 0 ; iterM < atLeastIter; ++iterM ){
        //-------------------------------------------------
        //HypothesisGeneration
        cv::Vec3f vp_Hypo = cv::Vec3f( 0, 0, 1 );
        char valid_Hypo = false;
        {
            const size_t modelPoints = 2;
            const size_t maxAttempts = 100;
            size_t sz = normalForm_Cont.size();
            for( size_t iters = 0 ; iters < maxAttempts; ++iters )
            {
            //
                valid_Hypo = ( sz >= modelPoints );
                if( !valid_Hypo ) break;
            //
                cv::Vec3f temp_1, temp_2;
                {
                    cv::Mat_<float> idx1( 1, 1 );
                    cv::Mat_<float> idx2( 1, 1 );
                    cv::randu( idx1, cv::Scalar(0), cv::Scalar( sz ) );
                    cv::randu( idx2, cv::Scalar(0), cv::Scalar( sz - 1 ) );
                    if( idx2(0,0) >= idx1(0,0) ) ++idx2(0,0);
                    temp_1 = normalForm_Cont[idx1(0,0)];
                    temp_2 = normalForm_Cont[idx2(0,0)];
                }
            //
                if( !valid_Hypo ) continue;
            //
                {
                    vp_Hypo = temp_1.cross( temp_2 );
                    valid_Hypo = abs( vp_Hypo[2] ) > angleCross;
                    if( valid_Hypo ) vp_Hypo /= vp_Hypo[2];
                }
            //
                if( valid_Hypo ) break;
            //
            }
        }
        if( !valid_Hypo ) continue;
    //-------------------------------------------------
        std::vector< char > valid_Hypo_Cont;
        valid_Hypo_Cont.clear();
        float suppose_Hypo = 0;
        {
            for(  size_t i = 0 ; i < normalForm_Cont.size(); ++i )
            {
                float ds = abs( vp_Hypo.dot( normalForm_Cont[i] ) );
                char act = ds < distanceInlier;
                if( act ) suppose_Hypo += length_Cont[i];
                valid_Hypo_Cont.push_back( act );
            }
        }
   //-------------------------------------------------
        if( suppose_Hypo > suppose_Best ){
            is_valid = true;
            vanishingPoint = vp_Hypo;
            valid_Cont = valid_Hypo_Cont;
            suppose_Best = suppose_Hypo;
            validRateExist = suppose_Best/suppose_Total;
            ratioInlier = validRateExist;
        }
   //-------------------------------------------------
        if( validRateIterNum < validRateExist ){
        //
            const size_t modelPoints = 2;
            const float t0 = 0.01F;
            const float log_t0 = log( 0.01F );
            const float eps = 0.00001F;
            //
            validRateIterNum = validRateExist;
            float rate = validRateIterNum;
            if( rate < eps ) rate = eps;
            //
            float t1 = 1.0F - std::pow( rate, (int) modelPoints );
            if( t1 < t0 ) atLeastIter = 0;
            else atLeastIter = ( log_t0 / log( t1 ) );
        }
   //-------------------------------------------------
    }
}
