#pragma once

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#if !defined(_MATH_DEFINES_DEFINED)
#define _MATH_DEFINES_DEFINED

#define M_E 2.71828182845904523536         // - e
#define M_LOG2E 1.44269504088896340736     // - log2(e)
#define M_LOG10E 0.43429448190325182765    // - log10(e)
#define M_LN2 0.69314718055994530941       // - ln(2)
#define M_LN10 2.30258509299404568402      // - ln(10)
#define M_PI 3.14159265358979323846        // - pi
#define M_PI_2 1.57079632679489661923      // - pi/2
#define M_PI_4 0.78539816339744830961      // - pi/4
#define M_1_PI 0.31830988618379067153      // - 1/pi
#define M_2_PI 0.63661977236758134307      // - 2/pi
#define M_2_SQRTPI 1.12837916709551257390  // - 2/sqrt(pi)
#define M_SQRT2 1.41421356237309504880     // - sqrt(2)
#define M_SQRT1_2 0.70710678118654752440   // - 1/sqrt(2)

#endif /* _USE_MATH_DEFINES */

#define P3D_EXPORTS

#include "meta/factory.hpp"
#include "meta/meta.hpp"

#ifdef ENTT_ID_TYPE
#define P3D_ID_TYPE ENTT_ID_TYPE
#else
#define P3D_ID_TYPE std::size_t
#endif

#define P3D_PROJECT_EXTENSION ".yml.gz"

// Unfortunately, we have to do it because of the fact that
// operator== throws an exception on comparing two Eigen::Matrix's
#undef eigen_assert
#define eigen_assert(x) \
    if (!(x)) { throw std::runtime_error("Eigen assertion failed"); }

#include <Eigen/Core>

namespace p3d
{
// clang-format off
using  Vec2   = Eigen::Vector2d;
using  Vec2f  = Eigen::Vector2f;
using  Vec3   = Eigen::Vector3d;
using  Vec3f  = Eigen::Vector3f;
using  Vec4   = Eigen::Matrix<double,4,1>;
using  Vec4f  = Eigen::Matrix<float,4,1>;
using  Mat2   = Eigen::Matrix<double,2,2>;
using  Mat23  = Eigen::Matrix<double,2,3>;
using  Mat3   = Eigen::Matrix3d;
using  Mat34  = Eigen::Matrix<double,3,4>;
using  Mat4   = Eigen::Matrix<double,4,4>;
using  Mat4f  = Eigen::Matrix<float,4,4>;
using  Mat    = Eigen::MatrixXd;
using  Mati   = Eigen::Matrix<int,-1,-1>;
using  Mat2X  = Eigen::Matrix<double,2,-1>;
using  Mat3X  = Eigen::Matrix<double,3,-1>;
using  Mat3Xf = Eigen::Matrix<float,3,-1>;
using  Mat4X  = Eigen::Matrix<double,4,-1>;
using  Vec    = Eigen::VectorXd;
using  Veci   = Eigen::Matrix<int,-1, 1>;
// clang-format on

class Exception : public std::exception
{
public:
    /*!
     Default constructor
     */
    Exception();
    /*!
     Full constructor. Normally the constuctor is not called explicitly.
     Instead, the macros p3d_Error(), p3d_Error_() and p3d_Func() are used.
    */
    Exception(int _code, const std::string& _err, const std::string& _func, const std::string& _file, int _line);
    virtual ~Exception() noexcept;

    /*!
     \return the error description and the context as a text string.
    */
    virtual const char* what() const noexcept;
    void formatMessage();

    std::string msg;  ///< the formatted error message

    int code;          ///< error code @see ARStatus
    std::string err;   ///< error description
    std::string func;  ///< function name. Available only when the compiler supports getting it
    std::string file;  ///< source file name where the error has occured
    int line;          ///< line number in the source file where the error has occured

    enum { ERROR_CODE,
           ASSERTION_CODE };
};

void error(const Exception& exc);
std::string format(const char* fmt, ...);

#if defined __GNUC__
#define p3d_Func __func__
#elif defined _MSC_VER
#define p3d_Func __FUNCTION__
#else
#define p3d_Func ""
#endif

#define Pollen3D_StsAssert Exception::ASSERTION_CODE

#define p3d_Error(code, msg) error(Exception(code, msg, p3d_Func, __FILE__, __LINE__))
#define p3d_Error_(code, args) error(Exception(code, format args, p3d_Func, __FILE__, __LINE__))
#define p3d_Assert(expr) \
    if (!!(expr))        \
        ;                \
    else                 \
        error(Exception(Pollen3D_StsAssert, #expr, p3d_Func, __FILE__, __LINE__))

#ifdef _DEBUG
#define p3d_DbgAssert(expr) p3d_Assert(expr)
#else
#define p3d_DbgAssert(expr)
#endif

}  // namespace p3d
