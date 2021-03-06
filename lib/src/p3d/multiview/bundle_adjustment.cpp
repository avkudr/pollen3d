#include "bundle_adjustment.h"

#include <fstream>

#include "ceres/ceres.h"

#include "p3d/logger.h"
#include "p3d/utils.h"

using namespace p3d;

struct BAFunctor_Affine
{
    BAFunctor_Affine(const Vec2 & x_)
    {
        x[0] = x_(0); x[1] = x_(1);
    }

    template <typename T>
    bool operator()(
        const T* const cam_K,
        const T* const cam_R,
        const T* const cam_t,
        const T* const pos_3dpoints,
        T* out_residuals) const
    {
        //--
        // Apply external parameters (Pose)
        //--

        const T * Q1 = &pos_3dpoints[0];

        Eigen::Matrix<T,3,4> P{Eigen::Matrix<T,3,4>::Zero()};
        Eigen::Matrix<T,4,1> X;
        Eigen::Matrix<T,3,1> q;

        Eigen::Matrix<T,2,2> A;

        const auto& a = T(cam_R[0]);
        const auto& b = T(cam_R[1]);
        const auto& c = T(cam_R[2]);

        Eigen::Matrix<T, 3, 3> R = utils::RfromEulerZYX(a, b, c);

        A << cam_K[0] * cam_K[2], cam_K[1], T(0.0), cam_K[2];

        P.topLeftCorner(2, 3) = A * R.topRows(2);
        P(0,3) = cam_t[0];
        P(1,3) = cam_t[1];
        P(2,3) = T(1.0);

        X << pos_3dpoints[0], pos_3dpoints[1], pos_3dpoints[2], T(1.0);

        q = P * X;
        q = q / q(2);

        // Compute and return the error is the difference between the predicted
        //  and observed position
        out_residuals[0] = q(0) - T(x[0]);
        out_residuals[1] = q(1) - T(x[1]);
        return true;
    }

    double x[2]; // The 2D observation
};

void BundleAdjustment::run(BundleData &data, BundleParams &params)
{
    if (!data.isValid()) {
        LOG_ERR("bundle: the problem is not valid");
        return;
    }

    const auto nbCams = params.nbCams();
    const auto X = data.X;
    const auto W = data.W;

    std::vector<std::vector<double>> points3d(X.cols(),std::vector<double>(3));
    for (auto p = 0; p < X.cols(); p++){
        Vec3 Q1 = X.col(p).topRows(3);
        points3d[p] = {Q1(0),Q1(1),Q1(2)};
    }

    ceres::Problem problem;
    auto lossFunction = new ceres::HuberLoss(8);

    for (auto c = 0; c < nbCams; c++ ){
        if (!params.isCamUsed(c)) continue;

        problem.AddParameterBlock(&data.cam[c][0],data.cam[c].size());
        problem.AddParameterBlock(&data.R[c][0],data.R[c].size());
        problem.AddParameterBlock(&data.t[c][0],data.t[c].size());

        if (params.isConst(p3dBundleParam_K,c)) problem.SetParameterBlockConstant(&data.cam[c][0]);
        else{
            std::vector<int> constParams;
            if (params.isConst(p3dBundleParam_Alpha,c)) constParams.push_back(0);
            if (params.isConst(p3dBundleParam_Skew,c))  constParams.push_back(1);
            if (params.isConst(p3dBundleParam_Focal,c)) constParams.push_back(2);

            if (!constParams.empty()){
                int nbIntrinsicPars = 3;
                auto subset_parameterization = new ceres::SubsetParameterization(nbIntrinsicPars, constParams);
                problem.SetParameterization(&data.cam[c][0], subset_parameterization);
            }
        }
        if (params.isConst(p3dBundleParam_R,c))
            problem.SetParameterBlockConstant(&data.R[c][0]);
        if (params.isConst(p3dBundleParam_t,c))
            problem.SetParameterBlockConstant(&data.t[c][0]);

        for (auto p = 0; p < points3d.size(); p++)
        {
            if (W(3*c+2,p) != 1) continue;
            if (X(3,p) != 1) continue;

            Vec2 x1 = W.block(3*c,p,2,1);
            ceres::AutoDiffCostFunction<BAFunctor_Affine, 2, 3, 3, 2, 3> * cost_function =
                    new ceres::AutoDiffCostFunction<BAFunctor_Affine, 2, 3, 3, 2, 3>(
                        new BAFunctor_Affine(x1));

            problem.AddParameterBlock(&points3d[p][0],points3d[p].size());
            if (params.isConstPts()) problem.SetParameterBlockConstant(&points3d[p][0]);

            problem.AddResidualBlock(cost_function, lossFunction,
                                     &data.cam[c][0],
                                     &data.R[c][0],
                                     &data.t[c][0],
                                     &points3d[p][0]);
        }
    }

    ceres::Solver::Options options;
    options.preconditioner_type = ceres::JACOBI;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
    options.minimizer_progress_to_stdout = true;
    options.use_explicit_schur_complement = true;
    options.num_threads = utils::nbAvailableThreads();

    std::stringstream buffer;
    std::streambuf * old = std::cout.rdbuf(buffer.rdbuf());

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::string text = buffer.str();
    std::cout.rdbuf(old);

    auto lines = utils::split(text,"\n");
    for (const auto & l : lines) LOG_INFO("%s",l.c_str());

    //std::string sReport = summary.BriefReport();
    //std::cout << sReport << std::endl;

    for (auto p = 0; p < data.X.cols(); ++p){
        if (data.X(3,p) == 1)
            data.X.col(p) << points3d[p][0], points3d[p][1], points3d[p][2], 1.0;
    }
}
