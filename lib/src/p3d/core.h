#pragma once

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#ifndef M_PI  // - pi
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2  // - pi/2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef M_1_PI  // - 1/pi
#define M_1_PI 0.31830988618379067153
#endif

#ifndef P3D_EXPORT
#   if defined _WIN32 || defined __CYGWIN__ || defined _MSC_VER
#       define P3D_EXPORT __declspec(dllexport)
#       define P3D_IMPORT __declspec(dllimport)
#       define P3D_HIDDEN
#   elif defined __GNUC__ && __GNUC__ >= 4
#       define P3D_EXPORT __attribute__((visibility("default")))
#       define P3D_IMPORT __attribute__((visibility("default")))
#       define P3D_HIDDEN __attribute__((visibility("hidden")))
#   else /* Unsupported compiler */
#       define P3D_EXPORT
#       define P3D_IMPORT
#       define P3D_HIDDEN
#   endif
#endif


#ifndef P3D_API
#   if defined P3D_API_EXPORT
#       define P3D_API P3D_EXPORT
#   elif defined P3D_API_IMPORT
#       define P3D_API P3D_IMPORT
#   else /* No API */
#       define P3D_API
#   endif
#endif

#include "entt/entt.hpp"

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
using Vec2   = Eigen::Vector2d;
using Vec2f  = Eigen::Vector2f;
using Vec3   = Eigen::Vector3d;
using Vec3f  = Eigen::Vector3f;
using Vec4   = Eigen::Matrix<double,4,1>;
using Vec4f  = Eigen::Matrix<float,4,1>;
using Mat2   = Eigen::Matrix<double,2,2>;
using Mat23  = Eigen::Matrix<double,2,3>;
using Mat3   = Eigen::Matrix3d;
using Mat3f  = Eigen::Matrix3f;
using Mat34  = Eigen::Matrix<double,3,4>;
using Mat4   = Eigen::Matrix<double,4,4>;
using Mat4f  = Eigen::Matrix<float,4,4>;
using Mat    = Eigen::MatrixXd;
using Mati   = Eigen::Matrix<int,-1,-1>;
using Mat2X  = Eigen::Matrix<double,2,-1>;
using Mat3X  = Eigen::Matrix<double,3,-1>;
using Mat3Xf = Eigen::Matrix<float,3,-1>;
using Mat4X  = Eigen::Matrix<double,4,-1>;
using Vec    = Eigen::VectorXd;
using Veci   = Eigen::Matrix<int,-1, 1>;
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
