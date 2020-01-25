#include "matching.h"

#include "p3d/console_logger.h"

Matcher::Matcher()
{

}

Matcher::Matcher(int method)
{
    _method = method;
}

Matcher::~Matcher()
{

}

void Matcher::setFilterCoef(double filterCoef){
    _filterCoef = filterCoef;
}

void Matcher::setMethod(int method)
{
    _method = method;
}

void Matcher::init(cv::Mat & leftDesc, cv::Mat & rightDesc, std::vector<cv::KeyPoint> &keyptsLeft, std::vector<cv::KeyPoint> &keyptsRight)
{
    _descriptorsLeftImage = &leftDesc;
    _descriptorsRightImage = &rightDesc;

    _keypointsLeftImage = &keyptsLeft;
    _keypointsRightImage = &keyptsRight;
}

void Matcher::setDescriptors(cv::Mat & left, cv::Mat & right){
    _descriptorsLeftImage = &left;
    _descriptorsRightImage = &right;
}

bool Matcher::isInit()
{
    //if ((_descriptorsLeftImage != NULL) && (_descriptorsRightImage != NULL)) {
        if ((_keypointsLeftImage != NULL) && (_keypointsRightImage != NULL)) {
            //return  ( !_descriptorsLeftImage->empty()) && ( !_descriptorsRightImage->empty()) &&
            return ( !_keypointsLeftImage->empty()) && ( !_keypointsRightImage->empty());
        }
    //}
    return false;
}

void Matcher::calculateMatches()
{
    if (_descriptorsLeftImage->empty() || _descriptorsRightImage->empty()){
        LOG_INFO("Images have no descriptors");
        return;
    }

    _matches.clear();
    std::vector<std::vector<cv::DMatch>> poor_matches;

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(_methodsStrings[_method]);
    matcher->knnMatch(*_descriptorsLeftImage, *_descriptorsRightImage, poor_matches, 2); // 2  best matches

    for(int i = 0; i < cv::min(_descriptorsLeftImage->rows-1,(int) poor_matches.size()); i++){
        if((poor_matches[i][0].distance < _filterCoef*(poor_matches[i][1].distance)) && ((int) poor_matches[i].size()<=2 && (int) poor_matches[i].size()>0)){
            _matches.push_back(poor_matches[i][0]);
        }
    }

    _matchesTable = getMatchesAsIdxTable();
    _inliersLeft  = getInliersLeftImage();
    _inliersRight  = getInliersRightImage();
}

int Matcher::getMatchesNb()
{
    return (int)_matches.size();
}

std::vector<cv::DMatch> Matcher::getMatchesAsDMatchVector(){
    return _matches;
}

cv::Mat Matcher::getMatchesAsIdxColumn(){

    if( ! _matches.empty()){
        int maxQueryIdx = -1;
        for(int i = 0; i < _matches.size(); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
        {
            if ( _matches[i].queryIdx > maxQueryIdx) maxQueryIdx = _matches[i].queryIdx;
        }

        cv::Mat matchColumnMat = cv::Mat::zeros(1, maxQueryIdx + 1, CV_64F);

        int row;
        for (int i = 0; i < _matches.size(); i++){
            row = _matches[i].queryIdx;
            matchColumnMat.at<double>(row) = _matches[i].trainIdx;
        }

        return matchColumnMat;
    }
    return cv::Mat::zeros(1, 1, CV_64F);
}

cv::Mat Matcher::getMatchesAsMeasMatrix()
{
    cv::Mat W = cv::Mat::zeros(4,getMatchesNb(),cv::DataType<double>::type);
    for (int i = 0; i < getMatchesNb(); i++){
        W.at<double>(0,i) = _inliersLeft[i].x;
        W.at<double>(1,i) = _inliersLeft[i].y;
        //W.at<double>(2,i) = 1;

        W.at<double>(2,i) = _inliersRight[i].x;
        W.at<double>(3,i) = _inliersRight[i].y;
        //W.at<double>(5,i) = 1;
    }
    return W;
}

void Matcher::setMatchesTable(const cv::Mat &table){
    _matchesTable = table.clone();
    _inliersLeft  = getInliersLeftImage();
    _inliersRight  = getInliersRightImage();
}

void Matcher::write(cv::FileStorage &fs) const
{
    fs << "Matcher_method" << _method;
    fs << "Matcher_filterCoef" << _filterCoef;
    fs << "Matcher_matches" << _matches;
    fs << "Matcher_matchesTable" << _matchesTable;
    fs << "Matcher_inliersLeft" << _inliersLeft;
    fs << "Matcher_inliersRight" << _inliersRight;
}

void Matcher::read(const cv::FileNode &node)//Read serialization for this class
{
    node["Matcher_method"] >> _method;
    node["Matcher_filterCoef"] >> _filterCoef;
    node["Matcher_matches"] >> _matches;
    node["Matcher_matchesTable"] >> _matchesTable;
    node["Matcher_inliersLeft"] >> _inliersLeft;
    node["Matcher_inliersRight"] >> _inliersRight;
}

std::vector<cv::Point2d> Matcher::getInliers(int imageIdx)
{
    if ( _matchesTable.empty()){
        _matchesTable = getMatchesAsIdxTable();
    }

    std::vector<cv::Point2d> inliers;
    std::vector<cv::KeyPoint> keyPts;
    if ( imageIdx == 0 ) keyPts = *_keypointsLeftImage;
    if ( imageIdx == 1 ) keyPts = *_keypointsRightImage;

    for (int i = 0; i < _matchesTable.rows ; i++){
        inliers.push_back( keyPts[_matchesTable.at<double>(i,imageIdx)].pt );
    }

    return inliers;
}

cv::Mat Matcher::getMatchesAsIdxTable(){
//    cv::Mat matchingCols = getMatchesAsIdxColumn();

    cv::Mat table = cv::Mat::zeros( (int) _matches.size(), 2, CV_64F);
    for (int j = 0; j < (int)_matches.size(); j++){
        table.at<double>(j,0) = _matches[j].queryIdx;
        table.at<double>(j,1) = _matches[j].trainIdx;
    }

/*    for (int j = 0; j < matchingCols.cols; j++){
        cv::Mat pointCorrespondence = cv::Mat::zeros(1, 2, CV_64F);

        int correspNb = 1;
        int a = j; // cols iterator - graph iterator
        pointCorrespondence.at<double>(0) = j;

        if (a < matchingCols.cols) {
            int temp = a; // to put zero in a cell that was allready written
            a = matchingCols.at<double>(a);
            matchingCols.at<double>(temp) = 0;

            if (a !=0 ){
                pointCorrespondence.at<double>(0 + 1) = a;
                correspNb++;
            }
        }

        if (correspNb > 1) table.push_back(pointCorrespondence);
    }*/
    return table;
}
