#include <iostream>

#include "p3d/project.h"
#include "p3d/logger.h"
#include "p3d/tasks.h"
#include "p3d/utils.h"

#include "p3d/multiview/autocalib.h"
#include "p3d/multiview/bundle_adjustment.h"

using namespace p3d;

static void initDiamondProject(Project & project) {
    project = Project();

    Mat vertices;
    vertices.setZero(22, 4);
    vertices << 0, -147.20, 75.20, 1.0,
        0, -16.00, 303.20, 1.0,
        -119.20, -100.80, 162.40, 1.0,
        118.40, -100.80, 162.40, 1.0,
        134.40, -146.40, 1.60, 1.0,
        263.20, -98.40, 0, 1.0,
        268.80, -16.00, 160.00, 1.0,
        142.40, -16.00, 269.60, 1.0,
        349.60, -16.00, 23.20, 1.0,
        268.80, 0, 160.00, 1.0,
        142.40, 0, 269.60, 1.0,
        349.60, 0, 23.20, 1.0,
        0, 0, 303.20, 1.0,
        0, 320.00, 0, 1.0,
        -349.60, 0, 23.20, 1.0,
        -142.40, 0, 269.60, 1.0,
        -268.80, 0, 160.00, 1.0,
        -349.60, -16.00, 23.20, 1.0,
        -142.40, -16.00, 269.60, 1.0,
        -268.80, -16.00, 160.00, 1.0,
        -263.20, -98.40, 0, 1.0,
        -134.40, -146.40, 1.60, 1.0;
    vertices.transposeInPlace();

    project.pointCloudCtnr()["sparse"].setVertices(vertices.topRows(3).cast<float>());

    AffineCamera c;
    Image i1;
    i1.setCamera(c);
    i1.setTranslation({500,250});

    int nbImages = 42;
    std::vector<Image> ims(nbImages,i1);
    project.setImageList(ims);

    for (int i = 0; i < nbImages - 1; i++) {
        project.imagePair(i)->setTheta1(utils::deg2rad(i + 3.0f));
        project.imagePair(i)->setRho(utils::deg2rad(3.0f + 0.2f * i));
        project.imagePair(i)->setTheta2(utils::deg2rad(2.0f*i - 10.0f));
    }

    auto P = project.getCameraMatricesMat();
    auto W = P * vertices;
    for (int c = 0; c < project.nbImages(); c++) {
        auto lastRow = W.row(3*c + 2).cwiseInverse();
        W.row(3*c + 0).cwiseProduct(lastRow);
        W.row(3*c + 1).cwiseProduct(lastRow);
        W.row(3*c + 2).cwiseProduct(lastRow);
    }

    project.setMeasurementMatrix(W);
    project.setMeasurementMatrixFull(W);
}

int main()
{
    p3d::Project data;
    p3d::openProject(data, "C:/Users/Andrey/Desktop/_datasets/Ostracode/temp.yml.gz");
    //initDiamondProject(data);

    auto W = data.getMeasurementMatrixFull();

    // ***** reconstruction from first x images
    std::cout << "W:\n" << W.rows() << "x" << W.cols() << std::endl;
    std::cout << "W:\n" << W.topLeftCorner(7*3,20) << std::endl;

    auto temp = data.getCameraRotationsRelativeMat();
    temp.col(1) *= 0;
    data.setCameraRotationsRelative(temp);
    std::cout << "data.getCameraRotationsRelativeMat():\n" << utils::rad2deg(data.getCameraRotationsRelativeMat()) << std::endl;

    Mat4X X;
    X.setZero(4,W.cols());

    std::set<int> calibrated;
    auto batches = utils::generateBatches(data.nbImages(),4);
    const int nbCamsTotal = data.nbImages();

    int batchIdx = 0;
    Mat2X errors;
    errors.setZero(2, batches.size());

    for (const auto & batch : batches) {
        std::cout << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << std::endl;
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
        std::cout << "slopes:\n" << utils::rad2deg(slopes) << std::endl;
        {
            LOG_INFO("Autocalibration...");
            utils::HideCout l;
            p3d::logger::off();
            autocalib.run();
            p3d::logger::on();

        }

        auto tvec = autocalib.getTranslations();

        Vec2 t0 = data.getCameraTranslations()[batch[0]];
        bool batchesNeedAlign = (t0.norm() > 1e-2);
        if (batchesNeedAlign) {
            auto P = autocalib.getCameraMatrices();
            Vec2 t0p = P[0].topRightCorner(2,1);
            auto rotRad = autocalib.getRotationAngles();
            Mat3 R = Mat3::Identity();
            for (auto i = 0; i < rotRad.rows(); ++i) {
                double t1 = rotRad(i, 0);
                double rho = rotRad(i, 1);
                double t2 = rotRad(i, 2);
                R = utils::RfromEulerZYZt_inv(t1, rho, t2) * R;
                tvec[i] = R.topLeftCorner(2,2) * (t0 - t0p) + R.topRightCorner(2,1)+ tvec[i];
            }
        }

        for (auto n = 0; n < nbCams; ++n) {
            const auto i = batch[n];
            AffineCamera c;
            data.image(i)->setCamera(c);
            data.image(i)->setTranslation(tvec[n]);
        }

        auto rotRad = autocalib.getRotationAngles();
        for (auto n = 0; n < nbCams - 1; ++n) {
            const auto i = batch[n];
            data.imagePair(i)->setTheta1(rotRad(n+1,0));
            data.imagePair(i)->setRho(rotRad(n+1,1));
            data.imagePair(i)->setTheta2(rotRad(n+1,2));
        }

        {
            auto rot = utils::rad2deg(rotRad);
            Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

            LOG_OK("Angles: [theta rho theta']");
            std::stringstream ss;
            ss << rot.format(CleanFmt);
            auto rows = utils::split(ss.str(), "\n");
            for (auto i = 0; i < rot.rows(); ++i) {
                LOG_OK("Pair %i: %s", i, rows[i].c_str());
            }
        }

        // ***** triangulate missing

        std::vector<int> calibVec;
        for (const auto & d : calibrated) calibVec.push_back(d);

        auto Ps = data.getCameraMatrices();
        for (auto pt = 0; pt < W.cols(); pt++) {
            //if (X(3,pt) == 1) continue;
            std::vector<Vec2> xa;
            std::vector<Mat34> Pa;
            for (const auto c : calibVec) {
                if (W(3*c+2, pt) != 1.0) continue;
                Pa.push_back(Ps[c]);
                xa.push_back(W.block(3*c,pt,2,1));
            }
            if (xa.size() < 2) continue;
            Vec4 Xa = utils::triangulate(xa,Pa);
            Xa /= Xa(3);
            X.col(pt) = Xa;
        }

        std::cout << "nb triangulated: " << X.bottomRows(1).sum() << "/" << X.cols() << std::endl;

        utils::reprojectionError(W,data.getCameraMatricesMat(),X,calibVec);

        // ***** bundle selected cams and selected points

#define WITH_BUNDLE 0
#if (WITH_BUNDLE == 1)
        BundleData bundleData;
        bundleData.X = X;
        bundleData.W = W;

        LOG_OK("Bundle adjustement: started...");
        data.getCamerasIntrinsics(&bundleData.cam);
        bundleData.R = data.getCameraRotationsAbsolute();
        bundleData.t = data.getCameraTranslations();


        {
            utils::HideCout h;
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
        }

        data.setCamerasIntrinsics(bundleData.cam);
        data.setCameraRotationsAbsolute(bundleData.R, calibVec.back() + 1);
        data.setCameraTranslations(bundleData.t);
        X = bundleData.X;
#endif
        // **** saving

        data.pointCloudCtnr()["sparse"].setVertices(X.topRows(3).cast<float>());
        batchIdx++;

        std::cout << "after bundle: " << std::endl;
        utils::reprojectionError(W,data.getCameraMatricesMat(),X,calibVec);

        //std::cout << utils::rad2deg(data.getCameraRotationsRelativeMat()) << std::endl;
    }

    p3d::saveProject(data, "C:/Users/Andrey/Desktop/_datasets/Ostracode/temp2.yml.gz");

    return 0;
}
