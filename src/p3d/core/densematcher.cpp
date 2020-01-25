#include "densematcher.h"

#include "p3d/console_logger.h"

DenseMatcher::DenseMatcher()
{

}

DenseMatcher::DenseMatcher(int method)
{
    _method = method;
}

DenseMatcher::~DenseMatcher()
{

}

void DenseMatcher::calculateDisparityMap()
{
    LOG_INFO("Disparity calculation started... ");
    if ( _lftIm == NULL || _rgtIm == NULL ){
        LOG_ERR("DenseMatcher module in not initialized: use denseMantcher->init command\n");
        return;
    }

    try{
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create( 16*_params.lowerBound,
                                                               16*_params.upperBound, //number of disparities
                                                                  _params.blockSize);
        sgbm->setMode(_method);

        int cn = _lftIm->channels();
        sgbm->setP1(8*cn*_params.blockSize*_params.blockSize);
        sgbm->setP2(32*cn*_params.blockSize*_params.blockSize);

        sgbm->compute( *_lftIm, *_rgtIm, _disp);

        cv::Mat_<float> temp = _disp;
        temp = temp / 16;

        cv::bilateralFilter(temp,_dispValues,21,180,180); //use bilateral filter ?
        //_dispValues = temp; // no filter

        LOG_OK("Done");
    }
    catch(...){
        LOG_ERR(0, "DenseMatcher::Unexpected error\n");
    }
}

void DenseMatcher::plotDisparityMap()
{
    if ( _disp.empty() ){
        LOG_ERR("Disparity was not calculated yet\n");
        return;
    }
    cv::Mat disp8;
    cv::normalize(_disp, disp8, 0, 255, CV_MINMAX, CV_8U);
    cv::imshow("Disparity", disp8);
}

void DenseMatcher::filterDisparity(int newVal, int maxSpeckleSize, int maxDiff)
{
    _paramsFilter.newVal = newVal;
    _paramsFilter.maxSpeckleSize = maxSpeckleSize;
    _paramsFilter.maxDiff = maxDiff;

    LOG_INFO("Disparity filtering started... ");

    if ( ! _disp.empty() ){
        _disp.copyTo(_dispFiltered);
        cv::filterSpeckles( _dispFiltered, newVal, maxSpeckleSize, maxDiff);

        cv::Mat_<float> temp = _dispFiltered;
        temp = temp / 16;

        _dispValues = temp;

    }
    else{
        LOG_INFO("Nothing to filter");
        return;
    }
    LOG_INFO("Done");
}

cv::Mat DenseMatcher::getDisparityToDisplay() const
{
//    if ( _disp.empty() )
//        LOG_INFO("Disparity was not calculated yet");

    cv::Mat disp8 = _disp;

    double minVal, maxVal;
    minMaxLoc(disp8, &minVal, &maxVal); //find minimum and maximum intensities
    cv::Mat draw;
    disp8.convertTo(draw, CV_8UC1, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
    //cv::imshow("image", draw);
    return draw;
}

void DenseMatcher::write(cv::FileStorage &fs) const
{
    fs << "DM_method"                  << _method;
    fs << "DM_parLowerBound"           << _params.lowerBound;
    fs << "DM_parUpperBound"           << _params.upperBound;
    fs << "DM_parBlockSize"            << _params.blockSize;
    fs << "DM_parFiltermaxDiff"        << _paramsFilter.maxDiff;
    fs << "DM_parFiltermaxSpeckleSize" << _paramsFilter.maxSpeckleSize;
    fs << "DM_parFilternewVal"         << _paramsFilter.newVal;
    fs << "DM_disparity"               << _disp;
    fs << "DM_disparityValues"         << _dispValues;
    fs << "DM_disparityFiltered"       << _dispFiltered;
}

void DenseMatcher::read(const cv::FileNode &node)
{
    node["DM_method"]                  >> _method;
    node["DM_parLowerBound"]           >> _params.lowerBound;
    node["DM_parUpperBound"]           >> _params.upperBound;
    node["DM_parBlockSize"]            >> _params.blockSize;
    node["DM_parFiltermaxDiff"]        >> _paramsFilter.maxDiff;
    node["DM_parFiltermaxSpeckleSize"] >> _paramsFilter.maxSpeckleSize;
    node["DM_parFilternewVal"]         >> _paramsFilter.newVal;
    node["DM_disparity"]               >> _disp;
    node["DM_disparityValues"]         >> _dispValues;
    node["DM_disparityFiltered"]       >> _dispFiltered;
}

void DenseMatcher::plotDisparityFiltered()
{
    if ( _dispFiltered.empty() ){
        LOG_INFO("Disparity was not filtered yet\n");
        return;
    }
    cv::Mat disp8;
    cv::normalize(_dispFiltered, disp8, 0, 255, CV_MINMAX, CV_8U);
    cv::imshow("Disparity", disp8);
}

