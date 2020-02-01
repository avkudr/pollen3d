#include "fundmat.h"

#include <Eigen/Eigen>

#include "p3d/console_logger.h"

void normalizePoints(std::vector<cv::Point2d> &pts, cv::Mat &transformMatrix)
{
    cv::Mat center = cv::Mat::ones(3,1,cv::DataType<double>::type);

    int nbPts = (int)pts.size();
    cv::Mat ptsMatHomog = cv::Mat::ones(3,nbPts,cv::DataType<double>::type);
    for (int i = 0; i < pts.size(); i++){
        ptsMatHomog.at<double>(0,i) = pts[i].x;
        ptsMatHomog.at<double>(1,i) = pts[i].y;
    }

    double meanX = 0.0, meanY = 0.0;
    meanX = cv::mean(ptsMatHomog.row(0))[0];
    meanY = cv::mean(ptsMatHomog.row(1))[0];
    center.at<double>(0,0) = meanX;
    center.at<double>(1,0) = meanY;

    double factorX = 0, factorY = 0;

    cv::Mat temp = cv::abs(ptsMatHomog - center * cv::Mat::ones(1,nbPts,cv::DataType<double>::type));

    double minval;
    cv::minMaxLoc(temp.row(0),&minval,&factorX);
    cv::minMaxLoc(temp.row(1),&minval,&factorY);

    transformMatrix = cv::Mat::zeros(3,3,cv::DataType<double>::type);

    transformMatrix.at<double>(0,0) = factorX;
    transformMatrix.at<double>(1,1) = factorY;
    transformMatrix.at<double>(2,2) = 1.0;
    transformMatrix.at<double>(0,2) = meanX;
    transformMatrix.at<double>(1,2) = meanY;

    //transformMatrix = (cv::Mat)(cv::Mat_<double>(3,3) << (factorX, 0, meanX, 0, factorY, meanY, 0, 0, 1));
    transformMatrix = transformMatrix.inv();
    ptsMatHomog = transformMatrix * ptsMatHomog;
    pts.clear();

    for (int i = 0; i < nbPts; i++){
        pts.push_back( cv::Point2d(
            ptsMatHomog.at<double>(0,i),
            ptsMatHomog.at<double>(1,i)));
    }
}

Mat3 FundMatAlgorithms::findFundMatGS(const std::vector<Vec2> &ptsL, const std::vector<Vec2> &ptsR)
{
    Mat3 F;
    F.setZero(3,3);
    F(2,2) = 1;

    int nbPts = 0;
    nbPts = (int)ptsR.size();

    Mat W;
    W.setZero(4,nbPts);
    for (int i = 0; i < nbPts; i++){
        W(0,i) = ptsR[i][0];
        W(1,i) = ptsR[i][1];
        W(2,i) = ptsL[i][0];
        W(3,i) = ptsL[i][1];
    }

    Vec4 meanW;
    for (auto & i : {0,1,2,3}) {
        meanW[i] = W.row(i).mean();
        W.row(i).array() -= meanW[i];
    }

    Eigen::JacobiSVD<Mat> svd(W.transpose(), Eigen::ComputeFullV);

    Mat N =  svd.matrixV().transpose().rightCols(1);

    F(0,2) = N(0); // a
    F(1,2) = N(1); // b
    F(2,0) = N(2); // c
    F(2,1) = N(3); // d
    F(2,2) = Mat(-1*N.transpose()*meanW)(0,0);

    F = F / F.norm();
    return F;
}

#include "ceres/ceres.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

struct FundamentalMatrixResidual {
    FundamentalMatrixResidual(const Vec2 qL, const Vec2 & qR)
        : xl(qL[0]), yl(qL[1]), xr(qR[0]), yr(qR[1]) {}

    template <typename T>
    bool operator()(const T* const p,  T* residual) const {

        const T & a = p[0];
        const T & b = p[1];
        const T & c = p[2];
        const T & d = p[3];
        T err1, err2;
        FundMatAlgorithms::epiporalDistances(
                    a,b,c,d,T(1.0),
                    T(xl),T(yl),T(xr),T(yr),
                    &err1,&err2);
        residual[0] = err1;
        residual[1] = err2;
        return true;
    }

private:
    const double xl;
    const double yl;
    const double xr;
    const double yr;
};

Mat3 FundMatAlgorithms::findFundMatCeres(const std::vector<Vec2> &ptsL, const std::vector<Vec2> &ptsR)
{
    Mat3 Fgs = findFundMatGS(ptsL,ptsR);

    Problem problem;
    const int nbPts = static_cast<int>(ptsL.size());

    std::vector<double> params = {Fgs(0,2) / Fgs(2,2), Fgs(1,2) / Fgs(2,2), Fgs(2,0) / Fgs(2,2), Fgs(2,1) / Fgs(2,2)};
    for (int i = 0; i < nbPts; ++i) {
        CostFunction* cost_function =
           new AutoDiffCostFunction<FundamentalMatrixResidual, 2, 4>(
               new FundamentalMatrixResidual(ptsL[i], ptsR[i]));
        problem.AddResidualBlock(cost_function, new CauchyLoss(2.0), &params[0]);
    }

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    LOG_INFO("Ceres. Init: %.10e; final: %.10e",summary.initial_cost,summary.final_cost);
    Mat3 F;
    F.setZero(3,3);
    F(0,2) = params[0];
    F(1,2) = params[1];
    F(2,0) = params[2];
    F(2,1) = params[3];
    F(2,2) = 1.0;

    F = F / F.norm();
    return F;
}

Vec2 FundMatAlgorithms::epiporalDistancesF(const Mat3 &F, const Vec2 &qL, const Vec2 &qR)
{
    const auto & a = F(0,2);
    const auto & b = F(1,2);
    const auto & c = F(2,0);
    const auto & d = F(2,1);
    const auto & e = F(2,2);

    const auto & qxR = qR[0];
    const auto & qyR = qR[1];
    const auto & qxL = qL[0];
    const auto & qyL = qL[1];

    double errL,errR;
    FundMatAlgorithms::epiporalDistances(a,b,c,d,e,
                                         qxL,qyL,qxR,qyR,
                                         &errL,&errR);

    Vec2 errs;
    errs << errL, errR;
    return errs;
}

template<typename T>
void FundMatAlgorithms::epiporalDistances(const T a, const T b, const T c, const T d, const T e,
                                       const T qxL, const T qyL, const T qxR, const T qyR,
                                       T *errorL, T *errorR)
{
    if (!errorL || !errorR) return;
    T error = abs(a*qxR +b*qyR + c*qxL + d*qyL + e);
    *errorL = 1.0/sqrt( a*a + b*b ) * error;
    *errorR = 1.0/sqrt( c*c + d*d ) * error;
}

