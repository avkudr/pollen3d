#include "project_data.h"

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

int dummyProjectData_ = ProjectData::initMeta();

int ProjectData::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: ProjectData" << std::endl;
        entt::meta<ProjectData>()
            .type("ProjectData"_hs)
            .data<&ProjectData::setPts3DDenseColors,&ProjectData::getPts3DDenseColors>(P3D_ID_TYPE(p3dData_pts3DDenseColors))
            .data<&ProjectData::setPts3DDense,&ProjectData::getPts3DDense>(P3D_ID_TYPE(p3dData_pts3DDense))
            .data<&ProjectData::setPts3DSparse,&ProjectData::getPts3DSparse>(P3D_ID_TYPE(p3dData_pts3DSparse))
            .data<&ProjectData::setMeasurementMatrix,&ProjectData::getMeasurementMatrix>(P3D_ID_TYPE(p3dData_measMat))
            .data<&ProjectData::setMeasurementMatrixFull,&ProjectData::getMeasurementMatrixFull>(P3D_ID_TYPE(p3dData_measMatFull))
            .data<&ProjectData::setImagePairs,&ProjectData::getImagePairs>(P3D_ID_TYPE(p3dData_imagePairs))
            .data<&ProjectData::setImageList,&ProjectData::getImageList>(P3D_ID_TYPE(p3dData_images))
            .data<&ProjectData::setProjectPath,&ProjectData::getProjectPath>(P3D_ID_TYPE(p3dData_projectPath));
        firstCall = false;
    }
    return 0;
}


ProjectData::ProjectData() : Serializable(){
    clear();
}

void ProjectData::setImageList(const std::vector<Image> &imList){
    if (imList.empty()) return;
    m_images = std::vector<Image>(imList);

    m_imagesPairs.clear();
    for (auto i = 0; i < m_images.size()-1; ++i) {
        m_imagesPairs.emplace_back(ImagePair(i,i+1));
    }
}

void ProjectData::clear()
{
    m_images.clear();
    m_imagesPairs.clear();
}

void ProjectData::getPairwiseMatches(const std::size_t i, std::vector<Vec2> &ptsL, std::vector<Vec2> &ptsR)
{
    ptsL.clear();
    ptsR.clear();

    ImagePair * imPair = imagePair(i);
    if (!imPair) return;
    Image * iL = imagePairL(i);
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
        ptsL.emplace_back(Vec2(ptsLraw[idL].pt.x,ptsLraw[idL].pt.y));
        ptsR.emplace_back(Vec2(ptsRraw[idR].pt.x,ptsRraw[idR].pt.y));
    }
}

void ProjectData::getEpipolarErrorsResidual(const std::size_t idx, Vec &errorsSquared)
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
void ProjectData::getEpipolarErrorsDistance(const std::size_t idx, Mat2X &distances)
{
    if (idx >= m_imagesPairs.size()) return;
    std::vector<Vec2> ptsL, ptsR;
    getPairwiseMatches(idx, ptsL, ptsR);
    if (ptsL.empty() || ptsR.empty()) return;
    const auto & F = imagePair(idx)->getFundMat();

    distances.setZero(2,ptsL.size());
    for (int i = 0; i < ptsL.size(); ++i) {
        Vec2 errs = fundmat::epiporalDistancesF(F,ptsL[i],ptsR[i]);
        distances.col(i) = errs;
    }
}

void ProjectData::getCamerasIntrinsics(std::vector<Vec3> *cam) const
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

void ProjectData::setCamerasIntrinsics(std::vector<Vec3> &cam)
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

void ProjectData::getCamerasExtrinsics(std::vector<Vec3> *R, std::vector<Vec2> *t) const
{
    if (nbImages() == 0) return;

    if (R) {
        std::vector<Mat3> Rarray(nbImages());
        Rarray[0].setIdentity();

        for (auto i = 0; i < nbImagePairs(); i++){
            double t1  = m_imagesPairs[i].getTheta1();
            double rho = m_imagesPairs[i].getRho();
            double t2  = m_imagesPairs[i].getTheta2();

            Rarray[i+1] = utils::RfromEulerZYZt_inv(t1,rho,t2);
            if (i != 0) Rarray[i+1] = Rarray[i+1] * Rarray[i];
        }

        R->resize(nbImages());
        for (auto i = 0; i < nbImages(); i++){
            const Mat3 & m = Rarray[i];
            double a,b,c;
            utils::EulerZYZtfromR(m, a, b, c);
            (*R)[i] << a,b,c;
        }
    }

    if (t) {
        t->resize(nbImages());
        for (auto i = 0; i < nbImages(); i++)
            (*t)[i] = m_images[i].getTranslation();
    }
}

void ProjectData::setCamerasExtrinsics(std::vector<Vec3> &R, std::vector<Vec2> &t)
{
    if (t.size() != R.size()) return;
    if (nbImages() != R.size()) return;

    std::vector<Mat3> Rarray(nbImages());
    Rarray[0].setIdentity();
    for (auto i = 0; i < nbImagePairs(); i++){
        const auto & a  = R[i+1][0];
        const auto & b = R[i+1][1];
        const auto & c  = R[i+1][2];
        Rarray[i+1] = utils::RfromEulerZYZt(a,b,c);
    }

    std::vector<Mat3> R_(nbImagePairs());
    for (auto i = 0; i < nbImagePairs(); i++){
        Mat3 dR = Rarray[i].inverse() * Rarray[i+1];

        double t1, rho, t2;
        utils::EulerZYZtfromRinv(dR,t1,rho,t2);
        utils::wrapHalfPI(t1);
        utils::wrapHalfPI(t2);
        m_imagesPairs[i].setTheta1(t1);
        m_imagesPairs[i].setRho(rho);
        m_imagesPairs[i].setTheta2(t2);
    }

    for (auto i = 0; i < nbImages(); i++)
        m_images[i].setTranslation(t[i]);
}

std::vector<Mat34> ProjectData::getCameraMatrices()
{
    if (nbImages() == 0) return {};

    std::vector<Mat34> Ps(nbImages());
    std::vector<Mat3> Rarray(nbImages());

    Rarray[0].setIdentity();

    for (auto i = 0; i < nbImagePairs(); i++){
        double t1  = m_imagesPairs[i].getTheta1();
        double rho = m_imagesPairs[i].getRho();
        double t2  = m_imagesPairs[i].getTheta2();

        Rarray[i+1] = utils::RfromEulerZYZt_inv(t1,rho,t2);
        if (i != 0) Rarray[i+1] = Rarray[i+1] * Rarray[i];
    }

    for (auto i = 0; i < nbImages(); i++){
        Mat2 A = m_images[i].getCamera().getA();
        Mat23 M = A * Rarray[i].topRows(2);

        Ps[i].setZero();
        Ps[i].topLeftCorner(2,3)  = M;
        Ps[i].topRightCorner(2,1) = m_images[i].getTranslation();
        Ps[i](2,3) = 1.0;
    }

    return Ps;
}

Mat ProjectData::getCameraMatricesMat()
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
