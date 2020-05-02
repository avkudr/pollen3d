#pragma once

#include "p3d/core.h"
#include "p3d/serialization.h"

namespace p3d
{
enum P3D_API p3dAffineCamera_ {
    p3dAffineCamera_Alpha = 0,  //< aspect ratio
    p3dAffineCamera_Skew = 1,   //< skew
    p3dAffineCamera_Focal = 2,  //< focal length
};

class P3D_API AffineCamera : public Serializable<AffineCamera>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int initMeta();
    static const char* classNameStatic() { return "AffineCamera"; }

    AffineCamera() { updateA(); }
    AffineCamera(const Mat2& A);
    ~AffineCamera() override;

    // **** intrinsic parameters

    void setFocal(double f)
    {
        m_focal = f;
        updateA();
    }
    void setAlpha(double a)
    {
        m_alpha = a;
        updateA();
    }
    void setSkew(double s)
    {
        m_skew = s;
        updateA();
    }

    double getFocal() const { return m_focal; }
    double getAlpha() const { return m_alpha; }
    double getSkew() const { return m_skew; }

    Mat2 getA() const { return m_A; }
    Mat3 getK() const;

private:
    void updateA();

    double m_focal{1.0};
    double m_alpha{1.0};
    double m_skew{0.0};

    Mat2 m_A;
};
}  // namespace p3d
