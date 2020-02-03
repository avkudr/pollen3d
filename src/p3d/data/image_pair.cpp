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
                .data<&ImagePair::setFundMat,&ImagePair::getFundMat>(P3D_ID_TYPE(p3dImagePair_fundMat))
                .data<&ImagePair::setHasF,&ImagePair::hasF>(P3D_ID_TYPE(p3dImagePair_hasF))
                .data<&ImagePair::setRectifyingTransformL,&ImagePair::getRectifyingTransformL>(P3D_ID_TYPE(p3dImagePair_rectifyingTransformLeft))
                .data<&ImagePair::setRectifyingTransformR,&ImagePair::getRectifyingTransformR>(P3D_ID_TYPE(p3dImagePair_rectifyingTransformRight));

        SERIALIZE_TYPE_VECS(ImagePair,"vector_ImagePair"_hs);

        firstCall = false;
    }
    return 0;
}

ImagePair::ImagePair(int imL, int imR) : _imL(imL), _imR(imR)
{
    F.setIdentity(3,3);
    m_imLrectified = cv::Mat();
    m_imRrectified = cv::Mat();
    m_rectifyingTransformL.setIdentity(3,3);
    m_rectifyingTransformR.setIdentity(3,3);
}

ImagePair::~ImagePair()
{
    m_hasF = false;
}

void ImagePair::getMatchesAsMap(std::map<int, int> &map) const
{
    map.clear();
    for (auto i = 0; i < m_matches.size(); ++i){
        const auto idxL = m_matches[i].iPtL;
        const auto idxR = m_matches[i].iPtR;
        map.insert({idxL,idxR});
    }
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

cv::Mat ImagePair::plotStereoRectified()
{
    return getOneStereoImage(rectifier->imLrect,rectifier->imRrect);
}

void ImagePair::getEpilines(const std::vector<Vec2> &pts, std::vector<Vec3> &epilines, bool transpose) const
{
    if (pts.empty()) return;
    auto nbPts = pts.size();

    epilines.reserve(pts.size());
    for (auto i = 0; i < nbPts; i++){
        Vec3 pt(pts[i][0],pts[i][1],1.0);
        if (transpose) pt = F.transpose()*pt;
        else pt = F*pt;
        epilines.emplace_back(pt);
    }
}



bool ImagePair::hasF() const
{
    return m_hasF;
}

void ImagePair::setHasF(bool isFready)
{
    m_hasF = isFready;
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
