#include "image_pair.h"

using namespace p3d;

int dummyImagePair_ = ImagePair::initMeta();

int ImagePair::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        LOG_DBG("Reflecting: ImagePair");
        entt::meta<ImagePair>()
            .alias("ImagePair"_hs)
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

void ImagePair::getMatchesAsMap(std::map<int, int> &map) const
{
    map.clear();
    for (auto i = 0; i < m_matches.size(); ++i){
        const auto idxL = m_matches[i].iPtL;
        const auto idxR = m_matches[i].iPtR;
        map.insert({idxL,idxR});
    }
}

void ImagePair::getEpilines(const std::vector<Vec2> &pts,
                            std::vector<Vec3> &epilines, bool transpose) const
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

const Mat3 ImagePair::getRrelative() const
{
    return utils::RfromEulerZYZt(m_theta1, m_rho, m_theta2);
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

void ImagePair::writeAdditional(cv::FileStorage &fs)
{
    fs << "im_p" + std::to_string(int(p3dImagePair_rectifiedImageLeft))
       << m_imLrectified;
    fs << "im_p" + std::to_string(int(p3dImagePair_rectifiedImageRight))
       << m_imRrectified;
    fs << "im_p" + std::to_string(int(p3dImagePair_disparityMap))
       << m_disparityMap;
}

void ImagePair::readAdditional(const cv::FileNode &node)
{
    LOG_DBG("Maybe there is no need to store rectified images");
    // we could just regenerate them on project loading ?
    node["im_p" + std::to_string(int(p3dImagePair_rectifiedImageLeft))] >>
        m_imLrectified;
    node["im_p" + std::to_string(int(p3dImagePair_rectifiedImageRight))] >>
        m_imRrectified;
    node["im_p" + std::to_string(int(p3dImagePair_disparityMap))] >>
        m_disparityMap;
}
