#pragma once

#include "p3d/core/core.h"
#include "p3d/core/serialization.h"

enum p3dAffineCamera_{
    p3dAffineCamera_Alpha  = 0, //
    p3dAffineCamera_Skew   = 1, //
    p3dAffineCamera_Focal  = 2, //

    p3dAffineCamera_Theta1 = 3, // theta_1
    p3dAffineCamera_Rho    = 4, // rho
    p3dAffineCamera_Theta2 = 5, // theta_2

    p3dAffineCamera_t      = 6,
};


class AffineCamera : public Serializable<AffineCamera>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int initMeta();

    AffineCamera(){ updateA(); updateR(); }
    ~AffineCamera() override;

    inline bool operator==(const AffineCamera& i) const;

    // **** intrinsic parameters

    void setFocal(double f) { m_focal = f; updateA(); }
    void setAlpha(double a) { m_alpha = a; updateA(); }
    void setSkew (double s) { m_skew  = s; updateA(); }

    double getFocal() const { return m_focal; }
    double getAlpha() const { return m_alpha; }
    double getSkew () const { return m_skew ; }

    Mat2 getA() const { return m_A; }
    Mat3 getK() const;

    // **** extrinsic parameters

    void setTheta1(double a) { m_theta1 = a; updateR(); }
    void setRho   (double a) { m_rho    = a; updateR(); }
    void setTheta2(double a) { m_theta2 = a; updateR(); }

    double getTheta1() const { return m_theta1; }
    double getRho   () const { return m_rho   ; }
    double getTheta2() const { return m_theta2; }

    void setTranslation(const Vec2 & t) { m_t = t; }
    Vec2 getTranslation() const { return m_t; }

    const Mat3 & getRrelative() const { return m_Rrelative; }

private:
    void updateA();
    void updateR();

    double m_focal{1.0};
    double m_alpha{1.0};
    double m_skew{0.0};

    double m_theta1{0.0};
    double m_rho{0.0};
    double m_theta2{0.0};

    Vec2 m_t{0,0};
    Mat3 m_Rrelative;
    Mat2 m_A;
};
