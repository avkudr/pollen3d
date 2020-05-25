#include "bundle_adjustment.h"

#include <fstream>

#include "ceres/ceres.h"

#include "p3d/logger.h"
#include "p3d/utils.h"

using namespace p3d;

template <typename T>
Eigen::Matrix<T, 3, 4> P_fromARt(const T* const cam_K, const T* const cam_R,
                                 const T* const cam_t)
{
    Eigen::Matrix<T, 3, 4> P{Eigen::Matrix<T, 3, 4>::Zero()};

    const auto& a = T(cam_R[0]);
    const auto& b = T(cam_R[1]);
    const auto& c = T(cam_R[2]);

    Eigen::Matrix<T, 3, 3> R = utils::RfromEulerZYX(a, b, c);

    Eigen::Matrix<T, 2, 2> A;
    A << cam_K[0] * cam_K[2], cam_K[1], T(0.0), cam_K[2];

    P.topLeftCorner(2, 3) = A * R.topRows(2);
    P(0, 3) = cam_t[0];
    P(1, 3) = cam_t[1];
    P(2, 3) = T(1.0);
    return P;
}

struct BAFunctor_Affine
{
    BAFunctor_Affine(const Vec2& x_) : x{x_} {}

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

        Eigen::Matrix<T, 3, 4> P{Eigen::Matrix<T, 3, 4>::Zero()};

        const auto& a = T(cam_R[0]);
        const auto& b = T(cam_R[1]);
        const auto& c = T(cam_R[2]);
        Eigen::Matrix<T, 3, 3> R = utils::RfromEulerZYX(a, b, c);

        Eigen::Matrix<T, 2, 2> A;
        A << cam_K[0] * cam_K[2], cam_K[1], T(0.0), cam_K[2];

        P.topLeftCorner(2, 3) = A * R.topRows(2);
        P(0,3) = cam_t[0];
        P(1,3) = cam_t[1];
        P(2, 3) = T(1.0);

        Eigen::Matrix<T, 4, 1> X;
        X << pos_3dpoints[0], pos_3dpoints[1], pos_3dpoints[2], T(1.0);

        Eigen::Matrix<T, 3, 1> q;
        q = P * X;
        q = q / q(2);

        // Compute and return the error is the difference between the predicted
        //  and observed position
        out_residuals[0] = q(0) - T(x[0]);
        out_residuals[1] = q(1) - T(x[1]);
        return true;
    }

    const Vec2& x;  // The 2D observation
};

struct BAFunctor_AffinePts {
    BAFunctor_AffinePts(const Vec2& x_, const std::vector<double>& P,
                        const float& confidence_ = 1.0f)
        : x{x_}, cam_P(P), confidence{confidence_}
    {
    }

    template <typename T>
    bool operator()(const T* const pos_3dpoints, T* out_residuals) const
    {
        //--
        // Apply external parameters (Pose)
        //--

        const T* Q = &pos_3dpoints[0];

        const T q0 =
            T(cam_P[0]) * Q[0] + T(cam_P[1]) * Q[1] + T(cam_P[2]) * Q[2] + T(cam_P[3]);
        const T q1 =
            T(cam_P[4]) * Q[0] + T(cam_P[5]) * Q[1] + T(cam_P[6]) * Q[2] + T(cam_P[7]);

        //        out_residuals[0] = T(confidence) * (q0 - T(x[0]));
        //        out_residuals[1] = T(confidence) * (q1 - T(x[1]));
        out_residuals[0] = q0 - T(x[0]);
        out_residuals[1] = q1 - T(x[1]);
        return true;
    }

    const Vec2& x;                     // The 2D observation
    const std::vector<double>& cam_P;  // first two rows of P [0 1 2 3; 4 5 6 7]
    const float& confidence = 1.0f;
};

void BundleAdjustment::run(BundleData &data, BundleParams &params)
{
    if (!data.isValid()) {
        LOG_ERR("bundle: the problem is not valid");
        return;
    }

    const auto nbCams = params.nbCams();
    auto& X = *data.X;
    const auto& matches = *data.matches;

    ceres::Problem problem;
    auto lossFunction = new ceres::HuberLoss(8);

    for (auto c = 0; c < nbCams; c++ ){
        if (!params.isCamUsed(c)) continue;

        problem.AddParameterBlock(&data.cam[c][0], data.cam[c].size());
        problem.AddParameterBlock(&data.R[c][0], data.R[c].size());
        problem.AddParameterBlock(&data.t[c][0], data.t[c].size());

        if (params.isConst(p3dBundleParam_K, c))
            problem.SetParameterBlockConstant(&data.cam[c][0]);
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
    }

    for (auto p = 0; p < matches.size(); p++) {
        if (X(3, p) != 1) continue;  // wasn't triangulated
        double* ptrX = (double*)&X(0, p);

        for (const auto& [c, obs] : matches[p]) {
            ceres::AutoDiffCostFunction<BAFunctor_Affine, 2, 3, 3, 2, 3>* cost_function =
                new ceres::AutoDiffCostFunction<BAFunctor_Affine, 2, 3, 3, 2, 3>(
                    new BAFunctor_Affine(obs.pt));

            problem.AddParameterBlock(ptrX, 3);
            if (params.isConstPts()) problem.SetParameterBlockConstant(ptrX);

            problem.AddResidualBlock(cost_function, lossFunction, &data.cam[c][0],
                                     &data.R[c][0], &data.t[c][0], ptrX);
        }

        if (p % 100000 == 0)
            std::cout << "progress: " << p / float(matches.size()) << std::endl;
    }

    std::cout << "progress: done " << std::endl;

    ceres::Solver::Options options;
    options.preconditioner_type = ceres::JACOBI;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
    options.minimizer_progress_to_stdout = true;
    options.use_explicit_schur_complement = true;
    options.num_threads = utils::nbAvailableThreads();

    std::cout << "- solving: start" << std::endl;

    std::stringstream buffer;
    std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::string text = buffer.str();
    std::cout.rdbuf(old);

    std::cout << "- solving: done" << std::endl;

    auto lines = utils::split(text,"\n");
    for (const auto & l : lines) LOG_INFO("%s",l.c_str());

    // std::string sReport = summary.BriefReport();
    // std::cout << sReport << std::endl;
}

void BundleAdjustment::runPtsOnly(BundleData& data)
{
    if (!data.isValid()) {
        LOG_ERR("bundle: the problem is not valid");
        return;
    }

    const int nbCams = static_cast<int>(data.R.size());
    auto& X = *data.X;
    const std::vector<std::map<int, Observation>>& matches = *data.matches;

    std::vector<std::vector<double>> P(nbCams, std::vector<double>(8, 0));

    for (auto c = 0; c < nbCams; c++) {
        Mat34 Pmat = P_fromARt(&data.cam[c][0], &data.R[c][0], &data.t[c][0]);
        P[c] = {Pmat(0, 0), Pmat(0, 1), Pmat(0, 2), Pmat(0, 3),
                Pmat(1, 0), Pmat(1, 1), Pmat(1, 2), Pmat(1, 3)};
    }

    int start = 0;
    int nbPtsSimultaneous = 50000;
    int nbPtsTotal = matches.size();

    while (start < nbPtsTotal) {
        int end = std::min(nbPtsTotal, start + nbPtsSimultaneous);

        ceres::Problem problem;
        auto lossFunction = new ceres::HuberLoss(8);

        for (auto p = start; p < end; ++p) {
            if (X(3, p) != 1) continue;  // wasn't triangulated
            double* ptrX = (double*)&X(0, p);

            for (const auto& [c, obs] : matches[p]) {
                ceres::AutoDiffCostFunction<BAFunctor_AffinePts, 2, 3>* cost_function =
                    new ceres::AutoDiffCostFunction<BAFunctor_AffinePts, 2, 3>(
                        new BAFunctor_AffinePts(obs.pt, P[c], obs.confidence));

                problem.AddParameterBlock(ptrX, 3);
                problem.AddResidualBlock(cost_function, lossFunction, ptrX);
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
        std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        std::string text = buffer.str();
        std::cout.rdbuf(old);

        start += nbPtsSimultaneous;
        std::cout << " - ba progress: " << start / float(nbPtsTotal) << std::endl;
    }
}
