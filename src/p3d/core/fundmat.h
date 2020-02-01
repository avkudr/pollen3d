/*!
 *  \brief     Class realisation of Fundamental Matrix
 *  \details   This class is used to demonstrate a number of section commands.
 *  \author    Andrey Kudryavtsev
 *  \version   0.1
 *  \date      15/05/2016
 *  \pre       First initialize the system.
 *  \bug       Not all memory is freed when deleting an object of this class.
 *  \warning   Most of methods takes no parameters as the 2D points are the attributes of the class. They are initialized during the computation of the fundamental matrix
  */

#ifndef FUNDAMENTALMATRIX_H
#define FUNDAMENTALMATRIX_H

#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "p3d/core/core.h"
#include "p3d/core/robust_estim.hpp"

struct FundMatAlgorithms{
    static Mat3 findFundMatGS(const std::vector<Vec2> & pts1,const std::vector<Vec2> & pts2);
};

class FundMat : public cv::Matx33d
{
public:
    FundMat();
    ~FundMat();

    void init(std::vector<cv::Point2d> * pts1, std::vector<cv::Point2d> * pts2, int method = METHOD_GOLDSTANDARD);
    /*! \fn void estimate(std::vector<cv::Point2d> * pts1, std::vector<cv::Point2d> * pts2, int method)
     *  \brief General function: it starts whether OpenCV for RANSAC etc. or GoldStandard for affine.
     *  \param pts1 points extracted from the left image
     *  \param pts2 points extracted from the right image
     *  \param method is one of following possibilities:
     */
    void estimate();
    /*! \fn void estimate(std::vector<cv::Point2d> * pts1, std::vector<cv::Point2d> * pts2, int method)
     *  \brief General function: it starts whether OpenCV for RANSAC etc. or GoldStandard for affine.
     *  \param pts1 points extracted from the left image
     *  \param pts2 points extracted from the right image
     *  \param method is one of following possibilities:
     */
    void estimate(std::vector<cv::Point2d> * pts1, std::vector<cv::Point2d> * pts2, int method = METHOD_GOLDSTANDARD);

    /*! \fn double getResidualError()
     *  \brief Compute the residual error using the following formula: \f$ \varepsilon_r = \frac{1}{N} \sum_{i}^{N} \left [ d({q_{i}}', F q_{i})^2 + d(q_{i}, F^\top {q_{i}}')^2 \right ] \f$
     *  It represents mean square distance from a point to a corrresponding epipolar line
     *  \param inliersIdx vector or inlier's indices
     */
    double getResidualError(const std::vector<int> inliersIdx = std::vector<int>());

    /*! \fn cv::Mat computeEpilinesLeft()
     *  \brief Compute corresponding epipolar lines:
     *  l_left  = F.t() * pts2, l_left are the lines in the left image
     *  l_right = F     * pts1
     */
    cv::Mat getEpilines(bool left);
    cv::Mat getEpilinesLeft();
    cv::Mat getEpilinesRight();

    void setToEye();
    void setMethod(int m) {_method = m;}
    void operator=(const cv::MatExpr & A);
    void operator=(const cv::Mat & A);
    cv::Mat toMat() const;

    void write(cv::FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "FundMat_method" << _method;
        cv::Mat temp = this->toMat();
        fs << "FundMat_data" << temp;
    }


    void read(const cv::FileNode& node)                          //Read serialization for this class
    {
        node["FundMat_method"] >> _method;
        cv::Mat temp;
        node["FundMat_data"] >> temp;
        *this = temp;
    }


    bool isInit();
    bool isEmpty() { return ((cv::Mat)*this).at<double>(2,2) == 0;}
    bool empty() { return ((cv::Mat)*this).empty();}

    /*! Fundamental matrix estimation methods */
    enum estimationMethod {
        METHOD_GOLDSTANDARD,    ///< Perform parabolic interpolation on the table
        METHOD_GOLDSTANDARD_RANSAC,    ///< Perform parabolic interpolation on the table
        METHOD_GOLDSTANDARD_LMEDS,
        METHOD_8POINT,          ///< Perform parabolic interpolation on the table
        METHOD_RANSAC,          ///< Perform linear interpolation on the table
        METHOD_LMEDS,           ///< Perform parabolic interpolation on the table
    };

    int _method = METHOD_GOLDSTANDARD;
private:
    std::vector<cv::Point2d> * _pts1 = NULL;
    std::vector<cv::Point2d> * _pts2 = NULL;
    cv::Mat findFundGS(std::vector<cv::Point2d> const& _pts1, std::vector<cv::Point2d> const& _pts2);
    cv::Mat findFundMatGSransac(std::vector<cv::Point2d> const& _pts1, std::vector<cv::Point2d> const& _pts2);
    cv::Mat findFundMatGSlmeds(std::vector<cv::Point2d> const& _pts1, std::vector<cv::Point2d> const& _pts2);
};

void normalizePoints(std::vector<cv::Point2d> & pts, cv::Mat & transformMatrix);

// ----------------------------------------------------------------------------
// ------- Class defining the Problem for RobustEstimator (RANSAC, LMedS)
// ----------------------------------------------------------------------------
class FundMatEstimation : public EstimationProblem{
    typedef std::pair<cv::Point2d,cv::Point2d> Correspondence;
    typedef std::vector<Correspondence> CorrespondenceVector;
public:
    FundMatEstimation(){
        setNbParams(4);
        setNbMinSamples(4);
    }

    void setData(std::vector<cv::Point2d> pts1, std::vector<cv::Point2d> pts2);

    double estimErrorForSample(int i);
    double estimModelFromSamples(std::vector<int> samplesIdx);

    int getTotalNbSamples() const{
        return (int) correspondences.size();
    }

    long double a,b,c,d; // Params

    cv::Mat getResult(){
        //cv::Mat F = T2.t() * (cv::Mat_<double>(3,3) << 0,0,a,0,0,b,c,d,1) * T1;
        cv::Mat F = (cv::Mat_<double>(3,3) << 0,0,a,0,0,b,c,d,1);
        if ( (!T1.empty()) && (!T2.empty())){
            F = T2.t() * F * T1;
        }
        F = F / F.at<double>(2,2);
        return F;
    }
private:
    bool isDegenerate(std::vector<int> samplesIdx);
    cv::Mat T1, T2;
    CorrespondenceVector correspondences; // Data
};

#endif // FUNDAMENTALMATRIX_H
