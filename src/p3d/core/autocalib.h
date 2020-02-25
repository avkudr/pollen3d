#pragma once

#include <vector>

#include "p3d/core/core.h"
#include "p3d/core/bundle_params.h"

class AutoCalibrator
{
public:
    AutoCalibrator() = delete;
    AutoCalibrator(const int nbIm);

    ~AutoCalibrator();

    static double wrap(const std::vector<double> &x, std::vector<double> &grad, void *data) {
        return (*reinterpret_cast<AutoCalibrator*>(data))(x, grad);
    }

    void setSlopeAngles(const Mat2X & slopes);
    void setMeasurementMatrix(const Mat & inW);
    void init();
    bool isInit() const { return W.rows() > 0 && W.cols() > 0; }

    int _objFuncMethod = OBJFUNC_Rectification;

    enum ObjFuncMethod{
        OBJFUNC_DistanceSquared,
        OBJFUNC_Distance,
        OBJFUNC_PseudoInverse,
        OBJFUNC_Rectification
    };

    double _minf;
    double getMinf() const{ return _minf;}

    int _maxTimeStep1 = 15;
    int _maxTimeStep2 = 60;

    void setMaxTimeStep1(int maxTime){ _maxTimeStep1 = maxTime; }
    void setMaxTimeStep2(int maxTime){ _maxTimeStep2 = maxTime; }

    void run();

    double operator() (const std::vector<double> &x, std::vector<double> &grad);
    double testFunction (const std::vector<double> &x);
    double distanceSquared       (const std::vector<double> &x);
    double distancePseudoInverse (const std::vector<double> &x);
    double distanceWminusMMW     (const std::vector<double> &x);

    int getParametersNumber();
    void getInitialConditionsAndBounds(std::vector<double> & _x0, std::vector<double> & _lb, std::vector<double> & _ub);
    void setObjectiveFunction(int objFuncMethod) { _objFuncMethod = objFuncMethod; _iterCnt = 0;}

    bool isFeaturerScaled = true;
    void useFeatureScaling(bool b) { isFeaturerScaled = b; }

    double _bestObjValue = HUGE_VAL;
    int _iterCnt = 0;

    int nbCams, nbPts;
    Mat W;
    Mat Win;
    /** matrix defining optimization bounds. Size = number of images \f$\times\f$ number of parameters.
    *  lower bounds = -paramBounds;
    *  upper bounds = +paramBounds;
    */
    Mat paramBounds;
    /** Offset for all parameters. Size = number of images \f$\times\f$ number of parameters.
     *  Actual value is calculated as offset + initial
     */
    Mat paramOffset;

    Mat paramInitial; ///< Initial values of parameters. Size = number of images \f$\times\f$ number of parameters
    Mat paramResult; ///< Resulting values of parameters. Offset + optimization results
    int nbParams;

    std::vector<Mat34> getCameraMatrices() const;
    Mat2 getCalibrationMatrix() const;
    Mat getRotationAngles() const;
    Mat getPoints3D(const std::vector<double> &x) { return getPoints3DfromParams(x);}

private:
    const int nbParsPerCam = BundleIdx_TOTAL;
    BundleParams m_pars;
    std::vector<Vec2> tm;
    void computeWmean(const Mat & Win, Mat & Wmean, std::vector<Vec2> & tm);
    std::vector<Mat23> getMfromParams(const std::vector<double> &x);
    Mat getPoints3DfromParams(const std::vector<double> &x);
    Mat2 getCalibrationMatrixFromParamTable(const Mat &paramTable) const;
};
