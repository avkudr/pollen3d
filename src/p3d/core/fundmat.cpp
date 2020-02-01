#include "fundmat.h"

#include <Eigen/Eigen>

#include "p3d/console_logger.h"

FundMat::FundMat()
{
    _method = METHOD_GOLDSTANDARD;
}

FundMat::~FundMat()
{

}

void FundMat::init(std::vector<cv::Point2d> * pts1, std::vector<cv::Point2d> * pts2, int method)
{
//    if ( pts1->empty() || pts2->empty() ){
//        LOG_INFO("Cannot initialize the fundamental matrix");
//        return;
//    }
    _pts1 = pts1;
    _pts2 = pts2;
    _method = method;
}

void FundMat::operator=(const cv::MatExpr & M)
{
    cv::Mat A = M;
    cv::Matx33d::operator = (A);
}

void FundMat::operator=(const cv::Mat & A)
{
    cv::Matx33d::operator = (A);
}

//void FundMat::consolePrint(std::ostream &stream) const
//{
//    cv::Mat a = (cv::Mat)*this;
//    stream << a;
//}

cv::Mat FundMat::toMat() const
{
    return (cv::Mat)*this;
}

bool FundMat::isInit()
{
    if ( _pts1 != NULL && _pts2 != NULL) {
        return ( ! _pts1->empty()) && ( ! _pts2->empty());
    }
    return false;
}


void FundMat::setToEye(){
    *this = cv::Mat::zeros( 3, 3, cv::DataType<double>::type);
}

void FundMat::estimate(std::vector<cv::Point2d> * pts1, std::vector<cv::Point2d> * pts2, int method){
    init(pts1,pts2,method);
    estimate();
}

void FundMat::estimate()
{
    std::vector<cv::Point2d> pts1 = *_pts1;
    std::vector<cv::Point2d> pts2 = *_pts2;
    switch(_method){
        case METHOD_RANSAC :
            *this = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC);
            break;
        case METHOD_8POINT :
            *this = cv::findFundamentalMat(pts1, pts2, cv::FM_8POINT);
            break;
        case METHOD_LMEDS :
            *this = cv::findFundamentalMat(pts1, pts2, cv::FM_LMEDS);
            break;
        case METHOD_GOLDSTANDARD :
            *this = this->findFundGS(pts1, pts2);
            break;
        case METHOD_GOLDSTANDARD_RANSAC :
            *this = this->findFundMatGSransac(pts1, pts2);
            break;
        case METHOD_GOLDSTANDARD_LMEDS :
            *this = this->findFundMatGSlmeds(pts1, pts2);
            break;
    }
}

cv::Mat FundMat::findFundGS(std::vector<cv::Point2d> const& pts1, std::vector<cv::Point2d> const& pts2){

    int nbPts = 0;
    nbPts = (int)pts1.size();

    cv::Mat F = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
    cv::Mat W = cv::Mat::zeros(4, nbPts,  cv::DataType<double>::type);

    for (int i = 0; i < nbPts; i++){
        W.at<double>(0,i) = pts2[i].x;
        W.at<double>(1,i) = pts2[i].y;
        W.at<double>(2,i) = pts1[i].x;
        W.at<double>(3,i) = pts1[i].y;
    }

    cv::Mat meanW = cv::Mat::zeros(4, 1, cv::DataType<double>::type);

    meanW.at<double>(0) = cv::mean(W.row(0))[0];
    meanW.at<double>(1) = cv::mean(W.row(1))[0];
    meanW.at<double>(2) = cv::mean(W.row(2))[0];
    meanW.at<double>(3) = cv::mean(W.row(3))[0];

    for (int i = 0; i < nbPts; i++){
        for (int j = 0; j < 4; j++){
            W.at<double>(j,i) = W.at<double>(j,i) - meanW.at<double>(j);
        }
    }

    cv::Mat S, U, Vt;
    cv::SVD::compute(W.t(), S, U, Vt);
    cv::Mat V = Vt.t();
    cv::Mat N = V.col(V.cols - 1);

    F.at<double>(0,2) = N.at<double>(0);
    F.at<double>(1,2) = N.at<double>(1);
    F.at<double>(2,0) = N.at<double>(2);
    F.at<double>(2,1) = N.at<double>(3);
    F.at<double>(2,2) = cv::Mat(-1*N.t()*meanW).at<double>(0,0);

    F = F / F.at<double>(2,2);

    return F;
}

cv::Mat FundMat::findFundMatGSransac(const std::vector<cv::Point2d> &_pts1, const std::vector<cv::Point2d> &_pts2)
{
    cv::Mat F = cv::Mat::zeros(3, 3, cv::DataType<double>::type);

    FundMatEstimation * findFundMat = new FundMatEstimation();
    findFundMat->setData(_pts1,_pts2);
    // Create estimator: RANSAC
    RANSAC * rans = new RANSAC();
    rans->solve(findFundMat);
    LOG_INFO("RANSAC fraction: %.2f %%", rans->getInliersFraction()*100.0f);
    LOG_INFO("Residual error: %.3f (%.3f", getResidualError(rans->getInliersIndices()),getResidualError());
    return findFundMat->getResult();
}

cv::Mat FundMat::findFundMatGSlmeds(const std::vector<cv::Point2d> &_pts1, const std::vector<cv::Point2d> &_pts2)
{
    cv::Mat F = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
    FundMatEstimation * findFundMat = new FundMatEstimation();
    findFundMat->setData(_pts1,_pts2);
    // Create estimator: RANSAC
    LMedS * rans = new LMedS();
    rans->solve(findFundMat);
    LOG_INFO("LMedS fraction: %.2f %%", rans->getInliersFraction()*100.0);
    LOG_INFO("Residual error: %.3f (%.3f", getResidualError(rans->getInliersIndices()),getResidualError());
    return findFundMat->getResult();
}

double FundMat::getResidualError(const std::vector<int> inliersIdx){
    double error = 0;
    std::vector<cv::Point2d> pts1;
    std::vector<cv::Point2d> pts2;

    if ( inliersIdx.empty() ){
           pts1 = *_pts1;
           pts2 = *_pts2;
    } else {
        for ( auto i : inliersIdx) {
            pts1.push_back((*_pts1)[i]);
            pts2.push_back((*_pts2)[i]);
        }
    }

    cv::Mat F = (cv::Mat)*this;

    int nbPts = 0;
    nbPts = (int)pts1.size();

    /*
    double a, b, c, d;
    a = F.at<double>(0,2);
    b = F.at<double>(1,2);
    c = F.at<double>(2,0);
    d = F.at<double>(2,1);

    for (int i = 0; i < nbPts; i++){
        double x2 = pts2[i].x;
        double y2 = pts2[i].y;
        double x1 = pts1[i].x;
        double y1 = pts1[i].y;

        error += pow((a*x2 + b*y2 +c*x1 + d*y1 + 1) / sqrt(a*a + b*b + c*c + d*d),2); //H&Z, p.350
    }*/






    cv::Mat qa = cv::Mat::ones(3, nbPts, cv::DataType<double>::type);
    cv::Mat la = cv::Mat::ones(3, nbPts, cv::DataType<double>::type);
    cv::Mat qb = cv::Mat::ones(3, nbPts, cv::DataType<double>::type);
    cv::Mat lb = cv::Mat::ones(3, nbPts, cv::DataType<double>::type);

    for (int i = 0; i < pts1.size(); i++){
        qa.at<double>(0,i) = pts1[i].x;
        qa.at<double>(1,i) = pts1[i].y;
        qb.at<double>(0,i) = pts2[i].x;
        qb.at<double>(1,i) = pts2[i].y;
    }

    la = F.t()*qb;
    lb = F*qa;

    double A, B, C, Mx, My;

    for (int i = 0; i < pts1.size(); i++){
        A = la.at<double>(0,i);
        B = la.at<double>(1,i);
        C = la.at<double>(2,i);

        Mx = qa.at<double>(0,i);
        My = qa.at<double>(1,i);

        error += pow(abs(A*Mx + B*My + C)/sqrt(A*A + B*B),2); // distance from point to epipolar line

        A = lb.at<double>(0,i);
        B = lb.at<double>(1,i);
        C = lb.at<double>(2,i);

        Mx = qb.at<double>(0,i);
        My = qb.at<double>(1,i);

        error += pow(abs(A*Mx + B*My + C)/sqrt(A*A + B*B),2);
    }

    return error / nbPts;
}

cv::Mat FundMat::getEpilines(bool left)
{
    cv::Mat epilines;
    cv::Mat F = (cv::Mat)*this;
    std::vector<cv::Point2d> pts;
    if (left) {
        F = F.t();
        pts = *_pts2;
    }
    else{
        pts = *_pts1;
    }
    int nbPts = (int) pts.size();

    cv::Mat ptsHom = cv::Mat::ones(3, nbPts,  cv::DataType<double>::type);
    for (int i = 0; i < nbPts; i++){
        ptsHom.at<double>(0,i) = pts[i].x;
        ptsHom.at<double>(1,i) = pts[i].y;
    }

    epilines = F*ptsHom;

    return epilines.t();
}

cv::Mat FundMat::getEpilinesLeft(){
    return getEpilines(true);
}

cv::Mat FundMat::getEpilinesRight(){
    return getEpilines(false);
}


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

void FundMatEstimation::setData(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2){

    //normalizePoints(pts1,T1);
    //normalizePoints(pts2,T2);

    for (int i = 0; i < pts1.size(); i++){
        correspondences.push_back(Correspondence(pts1[i],pts2[i]));
    }
}

double FundMatEstimation::estimErrorForSample(int i)
{
    double x2 = correspondences[i].second.x;
    double y2 = correspondences[i].second.y;
    double x1 = correspondences[i].first.x;
    double y1 = correspondences[i].first.y;

    double error = (a*x2 + b*y2 +c*x1 + d*y1 + 1) / sqrt(a*a + b*b + c*c + d*d); //H&Z, p.350
    return error;
}

double FundMatEstimation::estimModelFromSamples(std::vector<int> samplesIdx)
{
    int nbPts = 0;
    nbPts = (int)samplesIdx.size();

    cv::Mat W = cv::Mat::zeros(4, nbPts,  cv::DataType<double>::type);

    for (int i = 0; i < nbPts; i++){
        W.at<double>(0,i) = correspondences[samplesIdx[i]].second.x;
        W.at<double>(1,i) = correspondences[samplesIdx[i]].second.y;
        W.at<double>(2,i) = correspondences[samplesIdx[i]].first.x;
        W.at<double>(3,i) = correspondences[samplesIdx[i]].first.y;
    }

    cv::Mat meanW = cv::Mat::zeros(4, 1, cv::DataType<double>::type);

    meanW.at<double>(0) = cv::mean(W.row(0))[0];
    meanW.at<double>(1) = cv::mean(W.row(1))[0];
    meanW.at<double>(2) = cv::mean(W.row(2))[0];
    meanW.at<double>(3) = cv::mean(W.row(3))[0];

    for (int i = 0; i < nbPts; i++){
        for (int j = 0; j < 4; j++){
            W.at<double>(j,i) = W.at<double>(j,i) - meanW.at<double>(j);
        }
    }

    cv::Mat S, U, Vt;
    cv::SVD::compute(W.t(), S, U, Vt);
    cv::Mat V = Vt.t();
    cv::Mat N = V.col(V.cols - 1);

    //std::cout << cv::Mat(-1*N.t()*meanW).at<double>(0,0);
    double e = cv::Mat(-1*N.t()*meanW).at<double>(0,0);
    a = N.at<double>(0) / e;
    b = N.at<double>(1) / e;
    c = N.at<double>(2) / e;
    d = N.at<double>(3) / e;

    return 0;
}

bool FundMatEstimation::isDegenerate(std::vector<int> samplesIdx)
{
    return false;
}



Mat3 FundMatAlgorithms::findFundMatGS(const std::vector<Vec2> &pts1, const std::vector<Vec2> &pts2)
{
    Mat3 F;
    F.setZero(3,3);
    F(2,2) = 1;

    int nbPts = 0;
    nbPts = (int)pts1.size();

    Mat W;
    W.setZero(4,nbPts);
    for (int i = 0; i < nbPts; i++){
        W(0,i) = pts2[i][0];
        W(1,i) = pts2[i][1];
        W(2,i) = pts1[i][0];
        W(3,i) = pts1[i][1];
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

//    cv::SVD::compute(W.transpose(), S, U, Vt);
//    cv::Mat V = Vt.t();
//    cv::Mat N = V.col(V.cols - 1);

//    F.at<double>(0,2) = N.at<double>(0);
//    F.at<double>(1,2) = N.at<double>(1);
//    F.at<double>(2,0) = N.at<double>(2);
//    F.at<double>(2,1) = N.at<double>(3);
//    F.at<double>(2,2) = cv::Mat(-1*N.t()*meanW).at<double>(0,0);

//    F = F / F.at<double>(2,2);

    return F;
}

Mat3 FundMatAlgorithms::findFundMatCeres(const std::vector<Vec2> &pts1, const std::vector<Vec2> &pts2)
{

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
