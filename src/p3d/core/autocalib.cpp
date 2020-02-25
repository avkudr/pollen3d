#include "autocalib.h"

#include <iostream>
#include <iomanip>
#include <sstream>

#include<Eigen/Core>
#include<Eigen/SVD>

template<typename MatType>
using PseudoInverseType = Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime, MatType::RowsAtCompileTime>;

template<typename MatType>
PseudoInverseType<MatType> pseudoInverse(const MatType &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    using WorkingMatType = Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,
                                                                             MatType::MaxRowsAtCompileTime, MatType::MaxColsAtCompileTime>;
    Eigen::BDCSVD<WorkingMatType> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(epsilon*std::max(a.cols(), a.rows()));
    Eigen::Index rank = svd.rank();
    Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, MatType::RowsAtCompileTime,
                                0, Eigen::BDCSVD<WorkingMatType>::MaxDiagSizeAtCompileTime, MatType::MaxRowsAtCompileTime>
        tmp = svd.matrixU().leftCols(rank).adjoint();
    tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
    return svd.matrixV().leftCols(rank) * tmp;
}

#include <nlopt.hpp>

#include "p3d/core/utils.h"
#include "p3d/console_logger.h"

AutoCalibrator::AutoCalibrator(const int nbIm) :
    m_pars(BundleParams(nbIm))
{
    nbCams = nbIm;
    nbParams = -1;


    //paramConst.setOnes(nbCams,nbParsPerCam); // 1 for varying parameter
    paramBounds.setOnes(nbCams,nbParsPerCam);
    paramBounds *= M_PI/2;
    paramOffset.setZero(nbCams,nbParsPerCam);
    paramInitial.setZero(nbCams,nbParsPerCam);

    // Define default constant parameters:
    m_pars.setVaryingAll();
    m_pars.setConstAllCams(BundleParam_K);
    m_pars.setConstAllCams(BundleParam_t);
    m_pars.setConst(BundleParam_R,{0});
//    paramConst.col(K)     *= 0;
//    paramConst.col(ALPHA) *= 0;
//    paramConst.col(S)     *= 0;
//    paramConst.block(0,RT1,1,3) *= 0;

    // Define default offset parameters:
    paramOffset.col(BundleIdx_Focal).array() += 1;
    paramOffset.col(BundleIdx_Alpha).array() += 1;

    // Define default bounds (considered symmetric):

    // Define initial values. Actual value of parameter: offset + initial
    paramInitial.block(1,BundleIdx_Rho,nbCams-1,1).array() = utils::deg2rad(5.0);

    std::cout << paramInitial << std::endl;
}

void AutoCalibrator::setMeasurementMatrix(const Mat & inW)
{
    Win = inW;

    tm.clear();

    Mat Wmean;
    computeWmean(Win, Wmean, tm);

    utils::makeNonHomogenious(Wmean);
    W = Wmean;

    nbCams = W.rows() / 2;
    nbPts  = W.cols() ;
}

void AutoCalibrator::init(){

}

AutoCalibrator::~AutoCalibrator()
{

}

void AutoCalibrator::setSlopeAngles(const Mat2X & slopes)
{
    if (slopes.cols() != nbCams) throw "Bad slope angles";
    for (int i = 0; i < nbCams; ++i) {
        paramOffset(i,BundleIdx_Theta1) = slopes(0,i);
        paramOffset(i,BundleIdx_Theta2) = slopes(1,i);
        paramBounds(i,BundleIdx_Theta1) = utils::deg2rad(5.0); // +- 5 degrees
        paramBounds(i,BundleIdx_Theta2) = utils::deg2rad(5.0); // +- 5 degrees
    }
}

void AutoCalibrator::run()
{
    LOG_INFO("Autocalibration...");

    _minf = INFINITY;
    _bestObjValue = INFINITY;

    std::vector<double> x0,lb,ub;
    this->getInitialConditionsAndBounds(x0, lb, ub);
    this->setObjectiveFunction(AutoCalibrator::OBJFUNC_Rectification);
    //this->setObjectiveFunction(Optimization::OBJFUNC_PseudoInverse);

    //nlopt::opt optimizer(nlopt::GN_CRS2_LM, this->getParametersNumber()); // 1
    nlopt::opt optimizer(nlopt::GN_CRS2_LM, this->getParametersNumber()); // 1
    optimizer.set_lower_bounds(lb);
    optimizer.set_upper_bounds(ub);
    optimizer.set_min_objective( *AutoCalibrator::wrap, this);
//    optimizer.set_ftol_rel(1e-8);
    optimizer.set_stopval(1e-10);
    optimizer.set_ftol_abs(1e-10);
    optimizer.set_xtol_abs(1e-10);
//    optimizer.set_xtol_rel(1e-8);
    optimizer.set_maxtime(_maxTimeStep1);

    optimizer.optimize(x0, _minf);

    LOG_INFO("Found minimum: %.3f", _minf);

    Mat paramX = 0 * paramOffset;
    auto paramVarIdx = m_pars.getVaryingPairsCamParam();
    utils::copyVectorToMatElements(x0, paramVarIdx, paramX);
    paramResult = paramOffset + paramX;
}

double AutoCalibrator::operator() (const std::vector<double> &x, std::vector<double> &grad)
{
    (void)(grad); // remove warning for unused variable
    switch (_objFuncMethod) {
        case OBJFUNC_DistanceSquared: return distanceSquared(x); break;
        case OBJFUNC_Distance       : return testFunction(x); break;
        case OBJFUNC_PseudoInverse  : return distancePseudoInverse(x); break;
        case OBJFUNC_Rectification  : return distanceWminusMMW(x); break;
        default: break;
    }
    return 0;
}

double AutoCalibrator::testFunction (const std::vector<double> &x){
    return (x[0]*x[0] + x[1]*x[1] + 20*sin(x[0]) + 20*sin(x[1]));
}

std::vector<Mat23> AutoCalibrator::getMfromParams(const std::vector<double> &x){
    std::vector<Mat23> M;
    M.resize(nbCams);
    for (int i = 0; i < nbCams; i++) M[i].setZero();

    Mat2 A;
    A.setIdentity();
    A(0,0) = x[0] * x[1]; // k*E
    A(0,1) = x[0] * x[2];    // k*s

    M[0] = A * Mat23::Identity();

    Mat angles;
    angles.setIdentity(3, nbCams-1);
    //angles = params(4 : 4 + 3*(nCams-1)-1);
    //angles = reshape(angles, [3 length(angles)/3]);
    for(int i = 0 ; i < angles.cols() ; i++){
        angles(0,i) = x[3*i+3];
        angles(1,i) = x[3*i+4];
        angles(2,i) = x[3*i+5];
    }
    //std::cout << angles << std::endl;

    for (int i = 1 ; i < nbCams ; i++){
        double p = angles(0,i-1);
        double t = angles(1,i-1);
        double k = angles(2,i-1);

        // R = Rz(k)*Ry(t)*Rx(p)
        Mat23 R;
        R << cos(k)*cos(t), cos(k)*sin(p)*sin(t) - cos(p)*sin(k), sin(k)*sin(p) + cos(k)*cos(p)*sin(t),
             cos(t)*sin(k), cos(k)*cos(p) + sin(k)*sin(p)*sin(t), cos(p)*sin(k)*sin(t) - cos(k)*sin(p);

        M[i] = A * R ;
    }
    return M;
}

Mat AutoCalibrator::getPoints3DfromParams(const std::vector<double> &x){
    Mat X;
    X.setZero(3, nbPts);
    for (int i = 0 ; i < nbPts ; i++){
        X(0,i) = x[3 + 3*(nbCams-1) + 3*i];
        X(1,i) = x[3 + 3*(nbCams-1) + 3*i + 1];
        X(2,i) = x[3 + 3*(nbCams-1) + 3*i + 2];
    }
    return X;
}

double AutoCalibrator::distanceSquared(const std::vector<double> &x)
{
    _iterCnt++;

    std::vector<Mat23> a = getMfromParams(x);
    Mat M;
    M.setZero(2*a.size(),3);
    for (int i = 0; i < (int)a.size(); i++){
        M.block(0,2*i, 2,3) = a[i];
    }

    Mat X = getPoints3DfromParams(x);

    Mat reprojErrorMat = W - M*X;
    double reprojError = reprojErrorMat.array().square().sum();
    double s = x[2];

    double objValue = std::abs( reprojError + s );

    if (_iterCnt % nbParams * 500 == 0){
        if ( objValue < _bestObjValue) _bestObjValue = objValue ;
        std::stringstream stream;
        std::cout << "  *  " << std::setw(15) << _iterCnt
                  << "  *  " << std::setw(12) <<  std::setprecision(5) << _bestObjValue
                  << "  *  " << std::setw(12) <<  std::setprecision(5) << objValue
                  << std::endl << std::flush;
    }

    return objValue;
}

double AutoCalibrator::distancePseudoInverse(const std::vector<double> &x)
{
    _iterCnt++;

    auto a = getMfromParams(x);
    Mat M;
    M.setZero(2*a.size(),3);
    for (int i = 0; i < (int)a.size(); i++){
        M.block(0,2*i, 2,3) = a[i];
    }

    Mat X = getPoints3DfromParams(x);

    Mat Minv = pseudoInverse(M);

    Mat reprojErrorMat = W - M*X;
    double reprojError = reprojErrorMat.array().square().sum();
    double s = x[2];

    double objValue = std::abs( reprojError + s );

    if (_iterCnt % nbParams * 500 == 0){
        if ( objValue < _bestObjValue) _bestObjValue = objValue ;
        std::stringstream stream;
        std::cout << "  :  " << std::setw(15) << _iterCnt
                  << "  :  " << std::setw(12) <<  std::setprecision(5) << _bestObjValue
                  << "  :  " << std::setw(12) <<  std::setprecision(5) << objValue
                  << std::endl << std::flush;
    }

    return objValue;
}

Mat2 AutoCalibrator::getCalibrationMatrixFromParamTable(const Mat &paramTable ) const{
    double k     = paramTable(0,BundleIdx_Focal);
    double alpha = paramTable(0,BundleIdx_Alpha);
    double s     = paramTable(0,BundleIdx_Skew);
    Mat2 A;
    A << alpha, s, 0, 1.;
    A *= k;
    return A;
}

double AutoCalibrator::distanceWminusMMW(const std::vector<double> &x)
{
    auto paramVarIdx = m_pars.getVaryingPairsCamParam();
    Mat paramX = 0 * paramOffset;
    utils::copyVectorToMatElements( x, paramVarIdx, paramX);
    Mat paramTable = paramOffset + paramX;

    Mat2 A = getCalibrationMatrixFromParamTable(paramTable);

    std::vector<Mat23> Marray(nbCams);
    std::vector<Mat3> Rarray(nbCams);

    for (auto i = 0; i < nbCams; i++){
        double t1  = paramTable(i,BundleIdx_Theta1);
        double rho = paramTable(i,BundleIdx_Rho);
        double t2  = paramTable(i,BundleIdx_Theta2);

        Rarray[i] = utils::RfromEulerZYZt(t1,rho,t2);
        if (i != 0) Rarray[i] = Rarray[i] * Rarray[i-1];

        Marray[i] = A * Rarray[i].topRows(2);
    }

    Mat M = utils::concatenateMat(Marray, utils::CONCAT_VERTICAL);
    Mat pinvM = pseudoInverse(M);

    Mat errorMat = W - M*pinvM*W;
    double objValue = errorMat.array().square().sum();

    _iterCnt++;
    if ( _iterCnt % 1000 == 0){
        if ( objValue < _bestObjValue) _bestObjValue = objValue ;
        std::stringstream stream;
        std::cout << "  :  " << std::setw(15) << _iterCnt
                  << "  :  " << std::setw(12) <<  std::setprecision(5) << _bestObjValue
                  << "  :  " << std::setw(12) <<  std::setprecision(5) << objValue
                  << std::endl << std::flush;
        LOG_INFO("Iter: %5i, %5e, %5e", _iterCnt, _bestObjValue, objValue);
    }

    return objValue;
}

int AutoCalibrator::getParametersNumber()
{
    return nbParams;
}

void AutoCalibrator::getInitialConditionsAndBounds(std::vector<double> &_x0, std::vector<double> &_lb, std::vector<double> &_ub)
{
    //paramVarIdx.clear();
    auto paramVarIdx = m_pars.getVaryingPairsCamParam();
    nbParams = (int) paramVarIdx.size();
    LOG_INFO("Number of parameters: %i", nbParams);

    _x0.clear();
    _lb.clear();
    _ub.clear();

    _x0 = std::vector<double>(nbParams);
    _lb = std::vector<double>(nbParams);
    _ub = std::vector<double>(nbParams);

    Mat lbMat = -paramBounds;
    Mat ubMat =  paramBounds;

    lbMat(1,BundleIdx_Rho) = 0;

    utils::copyMatElementsToVector( paramInitial, paramVarIdx, _x0);
    utils::copyMatElementsToVector(       lbMat , paramVarIdx, _lb);
    utils::copyMatElementsToVector(       ubMat , paramVarIdx, _ub);
}


std::vector<Mat34> AutoCalibrator::getCameraMatrices() const
{
    std::vector<Mat34> Parray(nbCams);

    Mat2 A = getCalibrationMatrixFromParamTable(paramResult);

    std::vector<Mat3> Rarray(nbCams);

    for (auto i = 0; i < nbCams; i++){
        double t1  = paramResult(i,BundleIdx_Theta1);
        double rho = paramResult(i,BundleIdx_Rho);
        double t2  = paramResult(i,BundleIdx_Theta2);

        Rarray[i] = utils::RfromEulerZYZt(t1,rho,t2);
        if (i != 0) Rarray[i] = Rarray[i] * Rarray[i-1];

        Mat M = A * Rarray[i].topRows(2);

        Mat34 P;
        P.setZero();

        P(0,0) = M(0,0);
        P(0,1) = M(0,1);
        P(0,2) = M(0,2);

        P(1,0) = M(1,0);
        P(1,1) = M(1,1);
        P(1,2) = M(1,2);

        P(0,3) = tm[i](0,0);
        P(1,3) = tm[i](1,0);
        P(2,3) = 1;

        Parray[i] = P;
    }

    return Parray;
}

Mat2 AutoCalibrator::getCalibrationMatrix() const
{
    return getCalibrationMatrixFromParamTable(paramResult);
}

Mat AutoCalibrator::getRotationAngles() const
{
    return paramResult.middleCols(BundleIdx_Theta1,3);
}

void AutoCalibrator::computeWmean(const Mat & W, Mat & Wmean, std::vector<Vec2> & t){

    Wmean = W;
    int nCams = W.rows() / 3;
    t.clear();

    for (int i = 0; i < nCams ; i++){
        Vec2 tC{0,0};
        tC(0,0) = W.row( 3*i     ).mean();
        tC(1,0) = W.row( 3*i + 1 ).mean();
        t.push_back(tC);
        for (int j = 0; j < W.cols() ; j++){
            Wmean(3*i    ,j) = Wmean(3*i    ,j) - tC(0,0);
            Wmean(3*i + 1,j) = Wmean(3*i + 1,j) - tC(1,0);
        }
    }

}
