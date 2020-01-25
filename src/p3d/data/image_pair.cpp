#include "image_pair.h"

ImagePair::ImagePair()
{
    _imL = NULL;
    _imR = NULL;

    _matcher = NULL;
    F = NULL;
    rectifier = NULL;
    denseMatcher = NULL;
    _properties.clear();
}

ImagePair::ImagePair(Image *imL, Image *imR) : _imL(imL), _imR(imR) {
    _matcher = new Matcher(Matcher::METHOD_BruteForceL1);
    F = new FundMat();
    rectifier = new Rectifier();
    denseMatcher = new DenseMatcher();

    _matcher->init(imL->_desc,imR->_desc, imL->_kpts, imR->_kpts);
    F->init(&_matcher->_inliersLeft,&_matcher->_inliersRight);
    rectifier->init(imL,imR,F,&_matcher->_inliersLeft,&_matcher->_inliersRight);
    denseMatcher->init( &rectifier->imLrect, &rectifier->imRrect);
}

ImagePair::~ImagePair()
{
//    if (_matcher) delete _matcher;
//    if (F) delete F;
//    if (rectifier) delete rectifier;
//    if (denseMatcher) delete denseMatcher;
}

cv::Mat ImagePair::getOneStereoImage(cv::Mat im1, cv::Mat im2)
{
    if (im1.type() == CV_8UC4){
        cv::cvtColor(im1, im1, CV_BGR2GRAY);
        cv::cvtColor(im2, im2, CV_BGR2GRAY);
    }

    //qDebug() << QString::number(im1.type());
    cv::Mat outImg(im1.rows, im1.cols*2, CV_8UC3);
    cv::Rect rect1(0,0, im1.cols, im1.rows);
    cv::Rect rect2(im2.cols, 0, im2.cols, im2.rows);

    if (im1.type() == CV_8U)
    {
        cv::cvtColor(im1, outImg(rect1), CV_GRAY2BGR);
        cv::cvtColor(im2, outImg(rect2), CV_GRAY2BGR);
    }
    else
    {
        im1.copyTo(outImg(rect1));
        im2.copyTo(outImg(rect2));
    }

    return outImg;
}

cv::Mat ImagePair::plotStereoWithMatches()
{
    cv::Mat image;
    cv::Mat imLeftConverted;
    cv::Mat imRightConverted;

    cv::cvtColor( _imL->get(), imLeftConverted, CV_BGR2GRAY);
    cv::cvtColor( _imR->get(), imRightConverted, CV_BGR2GRAY);

    /*
    DEFAULT = 0,
    DRAW_OVER_OUTIMG = 1,
    NOT_DRAW_SINGLE_POINTS = 2,
    DRAW_RICH_KEYPOINTS = 4
    */
    cv::drawMatches( imLeftConverted, _imL->getKeyPoints(), imRightConverted, _imR->getKeyPoints(), _matcher->getMatchesAsDMatchVector(), image,
                     cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector< char >(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    return image;
}

cv::Mat ImagePair::plotStereoWithEpilines()
{
    cv::Mat outImg;

    if ( this->F->isEmpty() ) {
        return cv::Mat();
    }

    cv::Mat epilines1 = this->F->getEpilinesLeft();
    cv::Mat epilines2 = this->F->getEpilinesRight();

    std::vector<cv::Point2d> pts1 = _matcher->_inliersLeft;
    std::vector<cv::Point2d> pts2 = _matcher->_inliersRight;

    cv::Mat img1;
    cv::Mat img2;

    cv::cvtColor(_imL->get(), img1, CV_BGR2GRAY);
    cv::cvtColor(_imR->get(), img2, CV_BGR2GRAY);

    outImg = getOneStereoImage(img1, img2);
    cv::Rect rect1(0,0, img1.cols, img1.rows);
    cv::Rect rect2(img1.cols, 0, img2.cols, img2.rows);

    if (img1.type() == CV_8U)
    {
        cv::cvtColor(img1, outImg(rect1), CV_GRAY2BGR);
        cv::cvtColor(img2, outImg(rect2), CV_GRAY2BGR);
    }
    else
    {
        img1.copyTo(outImg(rect1));
        img2.copyTo(outImg(rect2));
    }

    cv::RNG rng;

    for(int i=0; i<pts1.size(); i++)
    {
        //cv::Scalar color(42,255,0);
        cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));

        std::vector<cv::Point> intesecPoints;
        intesecPoints.push_back(cv::Point(0,-epilines2.at<double>(i,2)/epilines2.at<double>(i,1)));
        intesecPoints.push_back(cv::Point(img2.cols,-(epilines2.at<double>(i,2)+epilines2.at<double>(i,0)*img2.cols)/epilines2.at<double>(i,1)));
        intesecPoints.push_back(cv::Point(-epilines2.at<double>(i,2)/epilines2.at<double>(i,0),0));
        intesecPoints.push_back(cv::Point(-(epilines2.at<double>(i,2)+epilines2.at<double>(i,1)*img2.rows)/epilines2.at<double>(i,0),img2.rows));

        if ( intesecPoints[3].x < 0 || intesecPoints[3].x > img2.cols )
            intesecPoints.erase( intesecPoints.begin() + 3);
        if ( intesecPoints[2].x < 0 || intesecPoints[2].x > img2.cols )
            intesecPoints.erase( intesecPoints.begin() + 2);
        if ( intesecPoints[1].y < 0 || intesecPoints[1].y > img2.rows )
            intesecPoints.erase( intesecPoints.begin() + 1);
        if ( intesecPoints[0].y < 0 || intesecPoints[0].y > img2.rows )
            intesecPoints.erase( intesecPoints.begin() + 0);

        cv::line(outImg(rect2),
                    intesecPoints[0],
                    intesecPoints[1],
                    color, 1, CV_AA);
        cv::circle(outImg(rect2), pts2[i], 3, color, -1, CV_AA);

        intesecPoints.clear();

        intesecPoints.push_back(cv::Point(0,-epilines1.at<double>(i,2)/epilines1.at<double>(i,1)));
        intesecPoints.push_back(cv::Point(img1.cols,-(epilines1.at<double>(i,2)+epilines1.at<double>(i,0)*img1.cols)/epilines1.at<double>(i,1)));
        intesecPoints.push_back(cv::Point(-epilines1.at<double>(i,2)/epilines1.at<double>(i,0),0));
        intesecPoints.push_back(cv::Point(-(epilines1.at<double>(i,2)+epilines1.at<double>(i,1)*img1.rows)/epilines1.at<double>(i,0),img1.rows));

        if ( intesecPoints[3].x < 0 || intesecPoints[3].x > img1.cols )
            intesecPoints.erase( intesecPoints.begin() + 3);
        if ( intesecPoints[2].x < 0 || intesecPoints[2].x > img1.cols )
            intesecPoints.erase( intesecPoints.begin() + 2);
        if ( intesecPoints[1].y < 0 || intesecPoints[1].y > img1.rows )
            intesecPoints.erase( intesecPoints.begin() + 1);
        if ( intesecPoints[0].y < 0 || intesecPoints[0].y > img1.rows )
            intesecPoints.erase( intesecPoints.begin() + 0);

        cv::line(outImg(rect1),
                    intesecPoints[0],
                    intesecPoints[1],
                    color, 1, CV_AA);
        cv::circle(outImg(rect1), pts1[i], 3, color, -1, CV_AA);

        intesecPoints.clear();
    }

    return outImg;
}

cv::Mat ImagePair::plotStereoRectified()
{
    return getOneStereoImage(rectifier->imLrect,rectifier->imRrect);
}

/*
void ImagePair::updatessssProperties()
{

    _properties.clear();
    _properties.push_back(new CVMatProperty(&_matcher->_matchesTable, "Matches"));
    _properties.push_back(new FundMatProperty(F, "FundMat"));
    _properties.push_back(new FundMatErrorProperty(F));
    _properties.push_back(new CVMatProperty(&denseMatcher->_dispValues, "Disparity"));

}
*/
