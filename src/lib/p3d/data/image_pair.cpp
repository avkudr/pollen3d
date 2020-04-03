#include "image_pair.h"

using namespace p3d;

int dummyImagePair_ = ImagePair::initMeta();

int ImagePair::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: ImagePair" << std::endl;
        entt::meta<ImagePair>()
                .type("ImagePair"_hs)
                .data<&ImagePair::denseMatching>(P3D_ID_TYPE(p3dImagePair_denseMatching))
                .data<&ImagePair::setMatches,&ImagePair::getMatches>(P3D_ID_TYPE(p3dImagePair_matches))
                .data<&ImagePair::setLeftImage,&ImagePair::imL>(P3D_ID_TYPE(p3dImagePair_imL))
                .data<&ImagePair::setRightImage,&ImagePair::imR>(P3D_ID_TYPE(p3dImagePair_imR))
                .data<&ImagePair::setFundMat,&ImagePair::getFundMat>(P3D_ID_TYPE(p3dImagePair_fundMat))
                .data<&ImagePair::setRectifyingTransformL,&ImagePair::getRectifyingTransformL>(P3D_ID_TYPE(p3dImagePair_rectifyingTransformLeft))
                .data<&ImagePair::setRectifyingTransformR,&ImagePair::getRectifyingTransformR>(P3D_ID_TYPE(p3dImagePair_rectifyingTransformRight))
                .data<&ImagePair::setTheta1,&ImagePair::getTheta1>(P3D_ID_TYPE(p3dImagePair_Theta1))
                .data<&ImagePair::setRho,   &ImagePair::getRho>   (P3D_ID_TYPE(p3dImagePair_Rho))
                .data<&ImagePair::setTheta2,&ImagePair::getTheta2>(P3D_ID_TYPE(p3dImagePair_Theta2));

        SERIALIZE_TYPE_VECS(ImagePair,"vector_ImagePair"_hs);

        firstCall = false;
    }
    return 0;
}

ImagePair::ImagePair(int imL, int imR) : _imL(imL), _imR(imR)
{
    F.setZero(3,3);
    m_imLrectified = cv::Mat();
    m_imRrectified = cv::Mat();
    m_disparityMap = cv::Mat();
    m_rectifyingTransformL.setIdentity(3,3);
    m_rectifyingTransformR.setIdentity(3,3);
}

ImagePair::~ImagePair()
{

}

bool ImagePair::operator==(const ImagePair &i) const{
    if (m_matches.size() != i.m_matches.size()) return false;
    if (!utils::floatEq(getTheta1(),i.getTheta1())) return false;
    if (!utils::floatEq(getRho()   ,i.getRho()   )) return false;
    if (!utils::floatEq(getTheta2(),i.getTheta2())) return false;
    return true;
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

cv::Mat ImagePair::getDisparityMapPlot() const {
    cv::Mat display;
    if (m_disparityMap.type() != CV_32F) {
        LOG_ERR("ImagePair::getDisparityMapPlot. Wrong type");
        return display;
    }
    float Amin = *std::min_element(m_disparityMap.begin<float>(), m_disparityMap.end<float>());
    float Amax = *std::max_element(m_disparityMap.begin<float>(), m_disparityMap.end<float>());
    cv::Mat A_scaled = (m_disparityMap - Amin)/(Amax - Amin);
    A_scaled.convertTo(display, CV_8UC1, 255.0, 0);
    applyColorMap(display, display, cv::COLORMAP_HOT);
    return display;
}

bool ImagePair::hasF() const
{
    return !F.isApprox(Mat3::Zero());
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

int DenseMatching::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: DenseMatching" << std::endl;
        entt::meta<DenseMatching>()
                .type("DenseMatching"_hs)
                .data<&DenseMatching::dispMethod>(P3D_ID_TYPE(p3dDense_DispMethod))
                .data<&DenseMatching::dispLowerBound>(P3D_ID_TYPE(p3dDense_DispLowerBound))
                .data<&DenseMatching::dispUpperBound>(P3D_ID_TYPE(p3dDense_DispUpperBound))
                .data<&DenseMatching::dispBlockSize>(P3D_ID_TYPE(p3dDense_DispBlockSize))
                .data<&DenseMatching::dispFilterNewValue>(P3D_ID_TYPE(p3dDense_DispFilterNewValue))
                .data<&DenseMatching::dispFilterMaxSpeckleSize>(P3D_ID_TYPE(p3dDense_DispFilterMaxSpeckleSize))
                .data<&DenseMatching::dispFilterMaxDiff>(P3D_ID_TYPE(p3dDense_DispFilterMaxDiff))
                .data<&DenseMatching::bilateralD>(P3D_ID_TYPE(p3dDense_BilateralD))
                .data<&DenseMatching::bilateralSigmaColor>(P3D_ID_TYPE(p3dDense_BilateralSigmaColor))
                .data<&DenseMatching::bilateralSigmaSpace>(P3D_ID_TYPE(p3dDense_BilateralSigmaSpace));

        SERIALIZED_ADD_READ_WRITE(DenseMatching,"DenseMatching"_hs);
        firstCall = false;
    }
    return 0;
}
int dummyDenseMatching_ = DenseMatching::initMeta();
