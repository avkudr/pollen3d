#pragma once

#include <vector>

#include "p3d/core/core.h"
#include "p3d/core/multiview/bundle_params.h"

class AutoCalibrator
{
public:
    enum ObjFuncMethod_{
        ObjFuncMethod_WminusMMW
    };

    AutoCalibrator() = delete;
    AutoCalibrator(const int nbIm);

    ~AutoCalibrator();

    bool isInit() const { return W.rows() > 0 && W.cols() > 0; }

    // ***** setters and getters

    void setObjectiveFunction(ObjFuncMethod_ objFuncMethod) { _objFuncMethod = objFuncMethod;}
    void setSlopeAngles(const Mat2X & slopes);
    void setMeasurementMatrix(const Mat & inW);
    void setMaxTime(int maxTime){ m_maxTime = maxTime; }

    std::vector<Mat34> getCameraMatrices() const;
    std::vector<Vec2>  getTranslations() const { return m_tm; }
    Mat2 getCalibrationMatrix() const;
    Mat  getRotationAngles() const;
    double getMinf() const{ return m_minf;}

    // ***** magic happens here

    void run();

private:

    static double wrap(const std::vector<double> &x, std::vector<double> &grad, void *data) {
        return (*reinterpret_cast<AutoCalibrator*>(data))(x, grad);
    }
    double operator() (const std::vector<double> &x, std::vector<double> &grad);
    double distanceWminusMMW (const std::vector<double> &x);

    void getInitialConditionsAndBounds(std::vector<double> & _x0, std::vector<double> & _lb, std::vector<double> & _ub);

    void computeWmean(const Mat & Win, Mat & Wmean, std::vector<Vec2> & m_tm);
    std::vector<Mat23> getMfromParams(const std::vector<double> &x);
    Mat2 getCalibrationMatrixFromParamTable(const Mat &paramTable) const;

    int _objFuncMethod = ObjFuncMethod_WminusMMW;
    double m_minf{HUGE_VAL};
    double m_bestObjValue{HUGE_VAL};
    int m_iterCnt{0};
    int m_maxTime{15};

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

    int m_nbParams{-1};

    BundleParams m_pars;
    std::vector<Vec2> m_tm;
};
