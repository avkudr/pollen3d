#include "project.h"

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Dense>

#include "p3d/core.h"
#include "p3d/stereo/fundmat.h"

using namespace p3d;

int dummyProject_ = Project::initMeta();

int Project::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        LOG_DBG("Reflecting: Project");
        entt::meta<Project>()
            .alias("Project"_hs)
            .data<&Project::setAutocalibPars, &Project::getAutocalibPars>(
                P3D_ID_TYPE(p3dProject_autocalibPars))
            .data<&Project::setPointCloudCtnr, &Project::getPointCloudCtnr>(
                P3D_ID_TYPE(p3dProject_pointCloudCtnr))
            .data<&Project::setMeasurementMatrix, &Project::getMeasurementMatrix>(
                P3D_ID_TYPE(p3dProject_measMat))
            .data<&Project::setImagePairs, &Project::getImagePairs>(
                P3D_ID_TYPE(p3dProject_imagePairs))
            .data<&Project::setImageList, &Project::getImageList>(
                P3D_ID_TYPE(p3dProject_images))
            .data<&Project::setProjectPath, &Project::getProjectPath>(
                P3D_ID_TYPE(p3dProject_projectPath))
            .data<&Project::setSettings, &Project::getSettings>(
                P3D_ID_TYPE(p3dProject_projectSettings));
        firstCall = false;
    }
    return 0;
}

Project::Project() : Serializable() { clear(); }

void Project::setImageList(const std::vector<Image> &imList)
{
    if (imList.empty()) return;
    m_images = std::vector<Image>(imList);

    m_imagesPairs.clear();
    for (auto i = 0; i < m_images.size()-1; ++i) {
        m_imagesPairs.emplace_back(ImagePair(i,i+1));
    }
}

void Project::setImagePairs(const std::vector<ImagePair> &imPairs)
{
    if (m_images.size() - 1 != imPairs.size()) {
        LOG_ERR("Can't load imagePairs: %i != %i", m_images.size() - 1,
                imPairs.size());
        return;
    }
    m_imagesPairs.clear();
    m_imagesPairs = imPairs;
}

void Project::clear()
{
    m_images.clear();
    m_imagesPairs.clear();
}

Image *Project::imagePairL(const std::size_t idx) { return imagePairImage(idx, true); }

Image *Project::imagePairR(const std::size_t idx) { return imagePairImage(idx, false); }

void Project::getPairwiseMatches(const std::size_t i, std::vector<Vec2> &ptsL,
                                 std::vector<Vec2> &ptsR)
{
    ptsL.clear();
    ptsR.clear();

    ImagePair *imPair = imagePair(i);
    if (!imPair) return;
    Image *iL = imagePairL(i);
    Image * iR = imagePairR(i);
    if (!iL || !iR) return;

    if (!iL->hasFeatures() || !iR->hasFeatures()) return;
    if (!imPair->hasMatches()) return;

    auto ptsLraw = iL->getKeyPoints();
    auto ptsRraw = iR->getKeyPoints();
    auto matches = imPair->getMatches();
    ptsL.reserve(matches.size());
    ptsR.reserve(matches.size());
    for (int i = 0; i < matches.size(); ++i) {
        const auto & idL = matches[i].iPtL;
        const auto & idR = matches[i].iPtR;

        if (idL >= ptsLraw.size()) continue;
        if (idR >= ptsRraw.size()) continue;

        ptsL.emplace_back(Vec2(ptsLraw[idL].pt.x,ptsLraw[idL].pt.y));
        ptsR.emplace_back(Vec2(ptsRraw[idR].pt.x,ptsRraw[idR].pt.y));
    }
}

void Project::getEpipolarErrorsResidual(const std::size_t idx, Vec &errorsSquared)
{
    if (idx >= m_imagesPairs.size()) return;
    std::vector<Vec2> ptsL, ptsR;
    getPairwiseMatches(idx, ptsL, ptsR);
    if (ptsL.empty() || ptsR.empty()) return;
    const auto & F = imagePair(idx)->getFundMat();

    errorsSquared.setZero(ptsL.size(),1);
    for (int i = 0; i < ptsL.size(); ++i) {
        double error = 0;
        Vec3 ptLh, ptRh;
        ptLh << ptsL[i][0],ptsL[i][1],1.0;
        ptRh << ptsR[i][0],ptsR[i][1],1.0;
        error = ptRh.transpose() * F * ptLh;
        errorsSquared[i] = error*error;
    }
}

/*
 * see Equation 2.33, p53
 */
void Project::getEpipolarErrorsDistance(const std::size_t idx, Mat2X &distances)
{
    if (idx >= m_imagesPairs.size()) return;
    std::vector<Vec2> ptsL, ptsR;
    getPairwiseMatches(idx, ptsL, ptsR);
    if (ptsL.empty() || ptsR.empty()) return;
    const auto & F = imagePair(idx)->getFundMat();

    distances.setZero(2,ptsL.size());
    for (int i = 0; i < ptsL.size(); ++i) {
        Vec2 errs = FundMatUtil::epiporalDistancesF(F,ptsL[i],ptsR[i]);
        distances.col(i) = errs;
    }
}

void Project::getCamerasIntrinsics(std::vector<Vec3> *cam) const
{
    if (nbImages() == 0) return;
    if (cam == nullptr) return;

    cam->resize(nbImages());
    for (auto i = 0; i < nbImages(); i++){
        const auto f = m_images[i].getCamera().getFocal();
        const auto a = m_images[i].getCamera().getAlpha();
        const auto s = m_images[i].getCamera().getSkew();
        (*cam)[i] = Vec3(a,s,f);
    }
}

void Project::setCamerasIntrinsics(std::vector<Vec3> &cam)
{
    if (nbImages() != cam.size()) {
        LOG_ERR("Wrong number of instrinsic matrices");
        return;
    }

    for (auto i = 0; i < nbImages(); i++){
        AffineCamera a;
        a.setAlpha(cam[i][0]);
        a.setSkew( cam[i][1]);
        a.setFocal(cam[i][2]);
        m_images[i].setCamera(a);
    }
}

void Project::getCamerasRotations(std::vector<Mat3> *R) const
{
    if (!R) return;
    if (nbImages() == 0) return;

    R->clear();
    R->resize(nbImages());
    (*R)[0] = Mat3::Identity();

    for (auto i = 0; i < nbImages()-1; i++) {
        double t1 = m_imagesPairs[i].getTheta1();
        double rho = m_imagesPairs[i].getRho();
        double t2 = m_imagesPairs[i].getTheta2();

        (*R)[i + 1] = utils::RfromEulerZYZt_inv(t1, rho, t2);
        if (i != 0) (*R)[i + 1] = (*R)[i + 1] * (*R)[i];
    }
}

std::vector<Vec3> Project::getCameraRotationsAbsolute() const
{
    std::vector<Mat3> R;
    getCamerasRotations(&R);
    std::vector<Vec3> vec;
    vec.resize(R.size());

    for (auto i = 0; i < static_cast<int>(R.size()); i++) {
        double a,b,c;
        utils::EulerZYXfromR(R[i], a, b, c);
        vec[i] << a, b, c;
    }
    return vec;
}

void Project::setCameraRotationsAbsolute(const std::vector<Vec3> &abs, int N)
{
    if (nbImages() != abs.size()) return;
    if (N == 0) N = abs.size();

    std::vector<Mat3> Rarray(N);
    for (auto i = 0; i < N; i++) {
        const auto &a = abs[i][0];
        const auto &b = abs[i][1];
        const auto &c = abs[i][2];
        Rarray[i] = utils::RfromEulerZYX(a,b,c);
    }

    for (auto i = 0; i < N - 1; i++){
        Mat3 dR = Rarray[i+1] * Rarray[i].transpose();

        double t1, rho, t2;
        utils::EulerZYZtfromRinv(dR,t1,rho,t2);
        utils::wrapHalfPI(t1);
        utils::wrapHalfPI(t2);
        m_imagesPairs[i].setTheta1(t1);
        m_imagesPairs[i].setRho(rho);
        m_imagesPairs[i].setTheta2(t2);
    }
}

void Project::getCamerasExtrinsics(std::vector<Mat3> *R, std::vector<Vec2> *t) const
{
    std::vector<Vec3> dummy;
    getCamerasRotations(R);
    getCamerasExtrinsics(&dummy,t);
}

void Project::getCamerasExtrinsics(std::vector<Vec3> *Rvec, std::vector<Vec2> *t) const
{
    if (nbImages() == 0) return;
    if (Rvec) *Rvec = getCameraRotationsAbsolute();
    if (t) *t = getCameraTranslations();
}

void Project::setCamerasExtrinsics(std::vector<Vec3> &Rvec, std::vector<Vec2> &t, int N)
{
    if (t.size() != Rvec.size()) return;
    if (nbImages() != Rvec.size()) return;

   if (N == 0) N = Rvec.size();

   setCameraRotationsAbsolute(Rvec,N);
   setCameraTranslations(t);
}

std::vector<Vec3> Project::getCameraRotationsRelative() const
{
    std::vector<Vec3> rots;
    rots.reserve(nbImages());
    rots.emplace_back(Vec3(0,0,0));
    for (int i = 0; i < nbImagePairs(); i++) {
        Vec3 zyzt;
        zyzt(0) = m_imagesPairs[i].getTheta1();
        zyzt(1) = m_imagesPairs[i].getRho();
        zyzt(2) = m_imagesPairs[i].getTheta2();
        rots.emplace_back(zyzt);
    }
    return rots;
}

MatX3 Project::getCameraRotationsRelativeMat() const
{
    std::vector<Vec3> vec = getCameraRotationsRelative();
    MatX3 zyzt;
    zyzt.setZero(vec.size(),3);
    for (int i = 0; i < vec.size(); i++)
        zyzt.row(i) = vec[i].transpose();
    return zyzt;
}

void Project::setCameraRotationsRelative(const MatX3 &zyzt)
{
    if (zyzt.rows() != nbImages()) {
        LOG_ERR("Wrong number of elements in setCameraRotationsRelative");
        return;
    }

    for (int i = 0; i < nbImagePairs(); i++) {
        m_imagesPairs[i].setTheta1(zyzt(i+1,0));
        m_imagesPairs[i].setRho(zyzt(i+1,1));
        m_imagesPairs[i].setTheta2(zyzt(i+1,2));
    }
}

std::vector<Vec2> Project::getCameraTranslations() const
{
    std::vector<Vec2> t;
    t.resize(nbImages());

    for (int i = 0; i < static_cast<int>(nbImages()); i++) {
        t[i] = m_images[i].getTranslation();
    }
    return t;
}

void Project::setCameraTranslations(const std::vector<Vec2> &t)
{
    if (t.size() != nbImages()) {
        LOG_ERR("Wrong number of elements in setTranslations");
        return;
    }

    for (int i = 0; i < static_cast<int>(nbImages()); i++) {
        m_images[i].setTranslation(t[i]);
    }
}

std::vector<Mat34> Project::getCameraMatrices() const
{
    if (nbImages() == 0) return {};

    std::vector<Mat34> Ps(nbImages());
    std::vector<Mat3> R(nbImages());

    getCamerasRotations(&R);

    for (auto i = 0; i < nbImages(); i++){
        Mat2 A = m_images[i].getCamera().getA();
        Mat23 M = A * R[i].topRows(2);

        Ps[i].setZero();
        Ps[i].topLeftCorner(2,3)  = M;
        Ps[i].topRightCorner(2,1) = m_images[i].getTranslation();
        Ps[i](2,3) = 1.0;
    }

    return Ps;
}

Mat Project::getCameraMatricesMat() const
{
    Mat Pm;
    std::vector<Mat34> Ps = getCameraMatrices();
    if (Ps.size() == 0) return Pm;

    Pm.setZero(3*nbImages(),4);
    for (auto i = 0; i < nbImages(); ++i) {
        Pm.block(3*i,0,3,4) = Ps[i];
    }
    return Pm;
}

const PointCloudContainer &Project::getPointCloudCtnr() const { return m_pointCloudCtnr; }

void Project::setPointCloudCtnr(const PointCloudContainer &pointCloudCtnr)
{
    m_pointCloudCtnr = pointCloudCtnr;
}

const ProjectSettings &Project::getSettings() const { return m_settings; }

void Project::setSettings(const ProjectSettings &settings) { m_settings = settings; }

const AutocalibPars &Project::getAutocalibPars() const { return m_autocalibPars; }

void Project::setAutocalibPars(const AutocalibPars &autocalibPars)
{
    m_autocalibPars = autocalibPars;
}
