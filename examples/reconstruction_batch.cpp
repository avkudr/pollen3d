#include <iostream>

#include "p3d/project.h"
#include "p3d/logger.h"
#include "p3d/tasks.h"

#include "p3d/multiview/autocalib.h"
#include "p3d/multiview/bundle_adjustment.h"

using namespace p3d;

int main()
{
    p3d::Project data;
    p3d::openProject(data, "C:/Users/Andrey/Desktop/Ostracode/temp.yml.gz");

    auto W = data.getMeasurementMatrixFull();

    // ***** reconstruction from first x images
    std::cout << "W:\n" << W.rows() << "x" << W.cols() << std::endl;

    std::cout << "Camera matrices [before]: \n" << data.getCameraMatricesMat().topRows(7*3) << std::endl;

    Mat4X X;
    X.setZero(4,W.cols());

    std::set<int> calibrated;
    auto batches = utils::generateBatches(data.nbImages(),4);
    const int nbCamsTotal = data.nbImages();
    for (const auto & batch : batches) {
        for (auto i : batch) {
            std::cout << i << ",";
            calibrated.insert(i);
        }
        std::cout << std::endl;

        const int nbCams = batch.size();

        // **** get full measurement matrix for selected batch

        std::vector<int> selectedPts;
        Mat Wfsub;
        utils::findFullSubMeasurementMatrix(W,Wfsub,batch,&selectedPts);

        // **** get slopes

        Mat2X slopes;
        slopes.setZero(2, nbCams);
        for (auto i = 0; i < nbCams - 1; ++i) {
            auto imPair = data.imagePair(batch[i]);
            if (imPair->imL() != batch[i]) {
                LOG_ERR("Pair doesn't correspond to the index");
                return -1;
            }
            if (imPair->imR() != batch[i+1]) {
                LOG_ERR("Pair doesn't correspond to the index");
                return -1;
            }

            slopes(0, i + 1) = data.imagePair(batch[i])->getTheta1();
            slopes(1, i + 1) = data.imagePair(batch[i])->getTheta2();
        }

        // **** autocalibration

        AutoCalibrator autocalib(nbCams);
        autocalib.setMaxTime(60);
        autocalib.setMeasurementMatrix(Wfsub);
        autocalib.setSlopeAngles(slopes);
        autocalib.run();

        auto tvec = autocalib.getTranslations();
        auto ti = data.getImage(batch[0]).getTranslation();
        bool correctForTranslation1stCam = (ti.norm() > 1e-2);
        if (correctForTranslation1stCam) {
            std::vector<Vec2> x;
            auto P = autocalib.getCameraMatrices();
            for (int i = 0; i < P.size(); i++) x.push_back(tvec[i]);
            Vec4 X = utils::triangulate(x,P);
            X /= X.w();
            X(0) -= tvec[0](0) - ti(0);
            X(1) -= tvec[0](1) - ti(1);
            for (int i = 0; i < P.size(); i++) {
                Vec3 q = P[i] * X;
                q = q / q(2);
                tvec[i](0) = q(0);
                tvec[i](1) = q(1);
            }
        }

        for (auto n = 0; n < nbCams; ++n) {
            const auto i = batch[n];
            AffineCamera c;
            data.image(i)->setCamera(c);
            data.image(i)->setTranslation(tvec[n]);
        }

        auto rotRad = autocalib.getRotationAngles();
        // first line of rotRad is [0,0,0]
        for (auto n = 0; n < nbCams - 1; ++n) {
            const auto i = batch[n];
            data.imagePair(i)->setTheta1(rotRad(n + 1, 0));
            data.imagePair(i)->setRho(rotRad(n + 1, 1));
            data.imagePair(i)->setTheta2(rotRad(n + 1, 2));
        }

        // ***** triangulate missing

        std::vector<int> calibVec;
        for (const auto & d : calibrated) calibVec.push_back(d);

        auto Ps = data.getCameraMatrices();
        for (auto pt = 0; pt < W.cols(); pt++) {
            if (X(3,pt) == 1) continue;
            std::vector<Vec2> xa;
            std::vector<Mat34> Pa;
            for (const auto c : calibVec) {
                if (W(3*c+2, pt) != 1) continue;
                Pa.push_back(Ps[c]);
                xa.push_back(W.block(3*c,pt,2,1));
            }
            if (xa.size() < 2) continue;
            Vec4 Xa = utils::triangulate(xa,Pa);
            Xa /= Xa.w();
            X.col(pt) = Xa;
        }
        std::cout << "nb triangulated: " << X.bottomRows(1).sum() << "/" << X.cols() << std::endl;

        // ***** bundle selected cams and selected points

        BundleData bundleData;
        bundleData.X = X;
        bundleData.W = W;

        data.getCamerasIntrinsics(&bundleData.cam);
        data.getCamerasExtrinsics(&bundleData.R, &bundleData.t);

        LOG_OK("Bundle adjustement: started...");
        {
            BundleParams params(nbCamsTotal);
            params.setUsedCams(calibVec);
            params.setConstAll();
            params.setVaryingPts();
            BundleAdjustment ba;
            ba.run(bundleData, params);
        }

        {
            BundleParams params(nbCamsTotal);
            params.setConstAll();
            params.setUsedCams(calibVec);
            params.setVaryingAllCams(p3dBundleParam_R);
            params.setVaryingAllCams(p3dBundleParam_t);
            params.setConstAllParams({0});
            BundleAdjustment ba;
            ba.run(bundleData, params);
        }

        {
            BundleParams params(nbCamsTotal);
            params.setConstAll();
            params.setUsedCams(calibVec);
            params.setVaryingPts();
            BundleAdjustment ba;
            ba.run(bundleData, params);
        }

        X = bundleData.X;
        data.setCamerasIntrinsics(bundleData.cam);
        data.setCamerasExtrinsics(bundleData.R, bundleData.t);
    }

//    batch = {0,1,2,3};
//    p3d::autocalibrate(p, batch);
//    std::cout << "Camera matrices [after]: \n" << p.getCameraMatricesMat().topRows(7*3) << std::endl;

//    batch = {3,4,5,6};
//    p3d::autocalibrate(p, batch);
//    std::cout << "Camera matrices [after]: \n" << p.getCameraMatricesMat().topRows(7*3) << std::endl;

    p3d::saveProject(data, "C:/Users/Andrey/Desktop/Ostracode/temp2.yml.gz");

    return 0;
}
