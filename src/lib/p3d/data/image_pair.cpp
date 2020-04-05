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
            .data<&ImagePair::setDenseMatchingPars,
                  &ImagePair::getDenseMatchingPars>(
                P3D_ID_TYPE(p3dImagePair_denseMatchingPars))
            .data<&ImagePair::setMatchingPars, &ImagePair::getMatchingPars>(
                P3D_ID_TYPE(p3dImagePair_matchingPars))
            .data<&ImagePair::setMatches, &ImagePair::getMatches>(
                P3D_ID_TYPE(p3dImagePair_matches))
            .data<&ImagePair::setLeftImage, &ImagePair::imL>(
                P3D_ID_TYPE(p3dImagePair_imL))
            .data<&ImagePair::setRightImage, &ImagePair::imR>(
                P3D_ID_TYPE(p3dImagePair_imR))
            .data<&ImagePair::setFundMat, &ImagePair::getFundMat>(
                P3D_ID_TYPE(p3dImagePair_fundMat))
            .data<&ImagePair::setRectifyingTransformL,
                  &ImagePair::getRectifyingTransformL>(
                P3D_ID_TYPE(p3dImagePair_rectifyingTransformLeft))
            .data<&ImagePair::setRectifyingTransformR,
                  &ImagePair::getRectifyingTransformR>(
                P3D_ID_TYPE(p3dImagePair_rectifyingTransformRight))
            .data<&ImagePair::setTheta1, &ImagePair::getTheta1>(
                P3D_ID_TYPE(p3dImagePair_Theta1))
            .data<&ImagePair::setRho, &ImagePair::getRho>(
                P3D_ID_TYPE(p3dImagePair_Rho))
            .data<&ImagePair::setTheta2, &ImagePair::getTheta2>(
                P3D_ID_TYPE(p3dImagePair_Theta2));

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

bool ImagePair::hasF() const { return !F.isApprox(Mat3::Zero()); }

DenseMatchingPars *ImagePair::denseMatchingPars()
{
    return &m_denseMatchingPars;
}

const DenseMatchingPars &ImagePair::getDenseMatchingPars() const
{
    return m_denseMatchingPars;
}

void ImagePair::setDenseMatchingPars(const DenseMatchingPars &d)
{
    m_denseMatchingPars = d;
}

MatchingPars *ImagePair::matchingPars() { return &m_matchingPars; }

const MatchingPars &ImagePair::getMatchingPars() const
{
    return m_matchingPars;
}

void ImagePair::setMatchingPars(const MatchingPars &d) { m_matchingPars = d; }
