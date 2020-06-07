#pragma once

#include <vector>

#include "p3d/core.h"
#include "p3d/multiview/bundle_params.h"
#include "p3d/serialization.h"

namespace p3d
{
typedef int p3dAutocalib;
enum P3D_API p3dAutocalib_ {
    p3dAutocalib_batchSize,
    p3dAutocalib_batchMinNbMatches,
    p3dAutocalib_withBA,
};

class P3D_API AutocalibPars : public Serializable<AutocalibPars>
{
public:
    static int initMeta();
    static const char *classNameStatic() { return "AutocalibPars"; }

    int batchSize{4};
    int batchMinNbMatches{20};
    bool withBA{true};
};

/**
 * @brief AutoCalibrator allows to perform autocalibration.
 *
 * autocalibration consists in finding camera parameters (intrinsic and
 * extrinsic) from images only without any prior information about the object
 * (scene). This implementation is adapted for affine cameras only. In order to
 * perform autocalibration a measurement matrix is needed where all features (!) are
 * seen in all images. User may also specify slope angles of epipolar lines that
 * will be used as a starting point of the algorithm.
 */
class P3D_API AutoCalibrator
{
public:
    /*! Available objective functions */
    enum ObjFuncMethod_ {
        ObjFuncMethod_WminusMMW  ///< f(x) = W - M * Minv * W, [1]
    };

    AutoCalibrator() = delete;
    /**
     * @brief Constructs an autocalibrator.
     * @param nbIm the number of viewpoints (normally corresponds to the number
     * of images).
     */
    AutoCalibrator(const int nbIm);

    ~AutoCalibrator();

    /**
     * @brief Checks if the autocalibrator is correctly initialized
     * @return true if initialized
     */
    bool isInit() const
    {
        return Win.rows() == m_pars.nbCams() * 3 && Win.cols() > 0;
    }

    // ***** setters and getters

    void setObjectiveFunction(ObjFuncMethod_ objFuncMethod)
    {
        _objFuncMethod = objFuncMethod;
    }

    /**
     * @brief Sets the initial slope angles
     *
     * in autocalibration the relative rotation between two images is
     * represented as R= Rz(t1)*Ry(rho)*Rz(t2)'. Slope angles t1 and t2
     * represent the slope angles of the corresponding epipolar lines
     *
     * @param slopes matrix of size [2,c] where c is the number of cameras. 
     * The rotation of the first camera is usually set to [0,0]'
     */
    void setSlopeAngles(const Mat2X &slopes);

    /**
     * @brief Sets the measurement matrix for autocalibration
     * @param inW measurement matrix. Make sure that the number of rows is equal
     * to 3*nbImages.
     */
    void setMeasurementMatrix(const Mat &inW);

    /**
     * @brief Sets the maximum time allowed for autocalibration
     * @param maxTime in seconds. Default: 15
     */
    void setMaxTime(int maxTime) { m_maxTime = maxTime; }

    /**
     * @brief Stop when an objective value of at least stopval is found: stop
     * minimizing when an objective value is <= stopval is found
     * @param stopval. Default: 1e-5
     */
    void setStopValue(double stopval) { m_stopVal = stopval; }

    /**
     * @brief Set relative tolerance on function value: stop when an
     * optimization step (or an estimate of the optimum) changes the objective
     * function value by less than tol multiplied by the absolute value of the
     * function value. (If there is any chance that your optimum function value
     * is close to zero, you might want to set an absolute tolerance with
     * nlopt_set_ftol_abs as well.)
     * @param tol. Default: 1e-5
     */
    void setFtolRel(double tol) { m_ftolRel = tol; }

    /**
     * @brief Set absolute tolerance on function value: stop when an
     * optimization step (or an estimate of the optimum) changes the function
     * value by less than tol. Criterion is disabled if tol is non-positive.
     * @param tol. Default: 1e-5
     */
    void setFtolAbs(double tol) { m_ftolAbs = tol; }

    /**
     * @brief Set relative tolerance on optimization parameters: stop when an
     * optimization step (or an estimate of the optimum) causes a relative
     * change the parameters x by less than tol. Criterion is disabled if tol is
     * non-positive.
     * @param tol. Default: 1e-5
     */
    void setXtolRel(double tol) { m_xtolRel = tol; }

    /**
     * @brief Get camera matrices after autocalibration
     * @return vector of camera matrices [3x4]
     */
    std::vector<Mat34> getCameraMatrices() const;
    std::vector<Vec2> getTranslations() const { return m_tm; }
    std::vector<Mat3> getRotations() const;

    Mat2 getCalibrationMatrix() const;

    // clang-format off
    /**
     * @brief Get the rotation after optimization
     *
     * @return matrix [c,3] of rotation angles in radians in the following
     * format:
     * [t1_1, rho_1, t2_1] // ->camera 1 usually [0,0,0]
     * [t1_2, rho_2, t2_2] // ->camera 2
     * [        ...      ]
     * [t1_c, rho_c, t2_c] // ->camera c
     */
    MatX3 getRotationAngles() const;

    // clang-format on

    /*! @brief Get the minimum found after the optimization */
    double getMinf() const { return m_minf; }

    /*! @brief magic happens here. */
    void run();

private:
    static double wrap(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        return (*reinterpret_cast<AutoCalibrator *>(data))(x, grad);
    }
    double operator()(const std::vector<double> &x, std::vector<double> &grad);
    double distanceWminusMMW(const std::vector<double> &x);

    void getInitialConditionsAndBounds(std::vector<double> &_x0, std::vector<double> &_lb, std::vector<double> &_ub);

    void computeWmean(const Mat &Win, Mat &Wmean, std::vector<Vec2> &m_tm);
    Mat2 getCalibrationMatrixFromParamTable(const Mat &paramTable) const;

    int _objFuncMethod = ObjFuncMethod_WminusMMW;
    double m_minf{HUGE_VAL};
    double m_bestObjValue{HUGE_VAL};
    int m_iterCnt{0};
    int m_maxTime{15};
    double m_stopVal{1e-5};
    double m_ftolRel{1e-5};
    double m_ftolAbs{1e-5};
    double m_xtolRel{1e-5};

    Mat W;
    Mat Win;

    /**
     * @brief matrix defining optimization bounds.
     *
     * size = number of images \f$\times\f$ number of parameters.
     *  lower bounds = -paramBounds;
     *  upper bounds = +paramBounds;
     */
    Mat paramBounds;

    /**
     * @brief offset for all parameters.
     *
     * size = number of images \f$\times\f$ number
     * of parameters. Actual value is
     */
    Mat paramOffset;

    /**
     * @brief initial values of parameters.
     *
     * size = number of images \f$\times\f$ number
     * of parameters.
     */
    Mat paramInitial;

    /**
     * @brief initial values of parameters.
     *
     * size = number of images \f$\times\f$ number
     * of parameters. Calculated as \link #paramOffset + optimized parameters
     */
    Mat paramResult;

    int m_nbParams{-1};

    BundleParams m_pars;
    std::vector<Vec2> m_tm;
};
}  // namespace p3d
