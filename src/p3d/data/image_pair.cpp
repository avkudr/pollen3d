#include "image_pair.h"

int dummyImagePair_ = ImagePair::initMeta();

int ImagePair::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: ImagePair" << std::endl;
        entt::meta<ImagePair>()
                .type("ImagePair"_hs)
                .data<&ImagePair::setMatches,&ImagePair::getMatches>(P3D_ID_TYPE(p3dImagePair_matches))
                .data<&ImagePair::setLeftImage,&ImagePair::imL>(P3D_ID_TYPE(p3dImagePair_imL))
                .data<&ImagePair::setRightImage,&ImagePair::imR>(P3D_ID_TYPE(p3dImagePair_imR))
                .data<&ImagePair::setFundMat,&ImagePair::getFundMat>(P3D_ID_TYPE(p3dImagePair_fundMat));

        SERIALIZE_TYPE_VECS(ImagePair,"vector_ImagePair"_hs);

        firstCall = false;
    }
    return 0;
}

ImagePair::ImagePair(int imL, int imR) : _imL(imL), _imR(imR)
{
    F.setRandom(3,3);
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

cv::Mat ImagePair::plotStereoWithEpilines()
{
    return cv::Mat();
    /*
    cv::Mat outImg;

    //if ( this->F->isEmpty() ) {
    //}

//    cv::Mat epilines1 = this->F->getEpilinesLeft();
//    cv::Mat epilines2 = this->F->getEpilinesRight();

    std::vector<cv::Point2d> pts1; // = _matcher->_inliersLeft;
    std::vector<cv::Point2d> pts2; // = _matcher->_inliersRight;

    cv::Mat img1;
    cv::Mat img2;

//    cv::cvtColor(_imL->cvMat(), img1, CV_BGR2GRAY);
//    cv::cvtColor(_imR->cvMat(), img2, CV_BGR2GRAY);

//    outImg = getOneStereoImage(img1, img2);
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
    */
}

cv::Mat ImagePair::plotStereoRectified()
{
    return getOneStereoImage(rectifier->imLrect,rectifier->imRrect);
}

int Match::initMeta() {
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: Match" << std::endl;
        entt::meta<Match>()
                .type("Match"_hs)
                .data<&Match::iPtL>(P3D_ID_TYPE(p3dMatch_iPtL))
                .data<&Match::iPtR>(P3D_ID_TYPE(p3dMatch_iPtR))
                .data<&Match::distance>(P3D_ID_TYPE(p3dMatch_distance));

        firstCall = false;

        SERIALIZE_TYPE_VECS(Match,"vector_Match"_hs);
    }
    return 0;
}
int dummyMatch_ = Match::initMeta();
