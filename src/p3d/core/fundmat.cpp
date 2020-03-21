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

Mat3 fundmat::findAffineGS(const std::vector<Vec2> &ptsX, const std::vector<Vec2> &ptsXt)
{
    Mat3 F;
    F.setZero(3,3);
    F(2,2) = 1;

    int nbPts = 0;
    nbPts = (int)ptsXt.size();

    Mat W;
    W.setZero(4,nbPts);
    for (int i = 0; i < nbPts; i++){
        W(0,i) = ptsX[i][0];
        W(1,i) = ptsX[i][1];
        W(2,i) = ptsXt[i][0];
        W(3,i) = ptsXt[i][1];
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
    //if (F(2,2) < 0) F *= -1;
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
    FundamentalMatrixResidual(const Vec2 X, const Vec2 & Xt)
        : x(X[0]), y(X[1]), xt(Xt[0]), yt(Xt[1]) {}

    template <typename T>
    bool operator()(const T* const p,  T* residual) const {

        const T & a = p[0];
        const T & b = p[1];
        const T & c = p[2];
        const T & d = p[3];
        const T & e = p[4];
        T err1, err2;
        err1 = err2 = T(100000);
        fundmat::epiporalDistances(
                    a,b,c,d,e,
                    T(x),T(y),T(xt),T(yt),
                    &err1,&err2);
        residual[0] = err1;
        residual[1] = err2;
        return true;
    }

private:
    const double x;
    const double y;
    const double xt;
    const double yt;
};

Mat3 fundmat::findAffineCeres(const std::vector<Vec2> &ptsX, const std::vector<Vec2> &ptsXt)
{
    //Mat3 Fgs = findAffineGS(ptsX,ptsXt);
    //std::vector<double> params = {Fgs(0,2), Fgs(1,2), Fgs(2,0), Fgs(2,1), Fgs(2,2)};
    std::vector<double> params = {1.0,1.0,1.0,1.0,1.0};

    Problem problem;
    const int nbPts = static_cast<int>(ptsX.size());

    for (int i = 0; i < nbPts; ++i) {
        CostFunction* cost_function =
           new AutoDiffCostFunction<FundamentalMatrixResidual, 2, 5>(
               new FundamentalMatrixResidual(ptsX[i], ptsXt[i]));
        problem.AddResidualBlock(cost_function, new CauchyLoss(2.0), &params[0]);
    }

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    LOG_INFO("Ceres. Init: %.10e; final: %.10e",summary.initial_cost,summary.final_cost);
    //printf("Ceres. Init: %.10e; final: %.10e\n",summary.initial_cost,summary.final_cost);
    Mat3 F;
    F.setZero(3,3);
    F(0,2) = params[0];
    F(1,2) = params[1];
    F(2,0) = params[2];
    F(2,1) = params[3];
    F(2,2) = params[4];

    F = F / F.norm();
    return F;
}

Vec2 fundmat::epiporalDistancesF(const Mat3 &F, const Vec2 &q, const Vec2 &qT)
{
    const auto & a = F(0,2);
    const auto & b = F(1,2);
    const auto & c = F(2,0);
    const auto & d = F(2,1);
    const auto & e = F(2,2);

    const auto & qTx = qT[0];
    const auto & qTy = qT[1];
    const auto & qx = q[0];
    const auto & qy = q[1];

    double err,errT;
    fundmat::epiporalDistances(a,b,c,d,e,qx,qy,qTx,qTy,&err,&errT);

    Vec2 errs;
    errs << err, errT;
    return errs;
}

/*
 * so that x2' * F * x1 = 0
 *

Mat3 fundmat::affineFromP(const Mat34 &Pl, const Mat34 &Pr)
{

    // https://www.ijcai.org/Proceedings/97-2/Papers/102.pdf
    // A General Expression of the Fundamental Matrix for Both Perspective and Affine Cameras
    // Zhengyou Zhang, Gang Xu

    Vec3 p11 = Pr.block(0,0,1,3).transpose();
    Vec3 p12 = Pr.block(1,0,1,3).transpose();
    Vec3 p21 = Pl.block(0,0,1,3).transpose();
    Vec3 p22 = Pl.block(1,0,1,3).transpose();

    Vec3 p23 = p21.cross(p22);

    Vec3 Pp;
    Pp << p11.transpose()*p23,p12.transpose()*p23,0;

    Mat3 temp = Pr * (Pl.transpose() * (Pl * Pl.transpose()).inverse());
    Mat3 F = utils::skewSym(Pp) * temp;
    F = F / F.norm();
    if (F(2,2) < 0) F *= -1;
    return F;
}
 */

std::pair<double, double> fundmat::slopAngles(const Mat3 &F)
{
    std::pair<double, double> angles;
    const auto & a = F(0,2);
    const auto & b = F(1,2);
    const auto & c = F(2,0);
    const auto & d = F(2,1);
    angles.second  = std::atan(-a/b); // theta
    angles.first   = std::atan(-c/d); // theta'
    return angles;
}

template<typename T>
void fundmat::epiporalDistances(const T a, const T b, const T c, const T d, const T e,
                                const T qx, const T qy, const T qTx, const T qTy,
                                T *errorL, T *errorR)
{
    if (!errorL || !errorR) return;
    T error = abs(a*qTx +b*qTy + c*qx + d*qy + e);
    *errorL = 1.0/sqrt( a*a + b*b ) * error;
    *errorR = 1.0/sqrt( c*c + d*d ) * error;
}

