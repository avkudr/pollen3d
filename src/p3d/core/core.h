#pragma once

#include <iostream>
#include <vector>
#include <sstream>

#ifndef M_PI
#define M_PI       3.141592653589793238462643383279502884L
#endif

#include "meta/meta.hpp"
#include "meta/factory.hpp"

#ifdef ENTT_ID_TYPE
#define P3D_ID_TYPE ENTT_ID_TYPE
#else
#define P3D_ID_TYPE std::size_t
#endif

// Unfortunately, we have to do it because of the fact that
// operator== throws an exception on comparing two Eigen::Matrix's
#undef eigen_assert
#define eigen_assert(x) if (!(x)) { printf("Eigen assertion failed\n"); }

#include <Eigen/Core>

using Vec2  = Eigen::Vector2d;
using Vec3  = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Vec4  = Eigen::Matrix<double,4,1>;
using Mat2  = Eigen::Matrix<double,2,2>;
using Mat23 = Eigen::Matrix<double,2,3>;
using Mat3  = Eigen::Matrix3d;
using Mat34 = Eigen::Matrix<double,3,4>;
using Mat4  = Eigen::Matrix<double,4,4>;
using Mat   = Eigen::Matrix<double,-1,-1>;
using Mati  = Eigen::Matrix<int,-1,-1>;
using Mat2X = Eigen::Matrix<double,2,-1>;
using Vec   = Eigen::Matrix<double,-1, 1>;
using Veci  = Eigen::Matrix<int,-1, 1>;

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
    virtual const char *what() const noexcept;
    void formatMessage();

    std::string msg; ///< the formatted error message

    int code; ///< error code @see ARStatus
    std::string err; ///< error description
    std::string func; ///< function name. Available only when the compiler supports getting it
    std::string file; ///< source file name where the error has occured
    int line; ///< line number in the source file where the error has occured

    enum {ERROR_CODE,ASSERTION_CODE};
};

void error( const Exception& exc );
std::string format( const char* fmt, ... );

#if defined __GNUC__
    #define p3d_Func __func__
#elif defined _MSC_VER
    #define p3d_Func __FUNCTION__
#else
    #define p3d_Func ""
#endif

#define Pollen3D_StsAssert Exception::ASSERTION_CODE

#define p3d_Error( code, msg ) error( Exception(code, msg, p3d_Func, __FILE__, __LINE__) )
#define p3d_Error_( code, args ) error( Exception(code, format args, p3d_Func, __FILE__, __LINE__) )
#define p3d_Assert( expr ) if(!!(expr)) ; else error( Exception(Pollen3D_StsAssert, #expr, p3d_Func, __FILE__, __LINE__) )

#ifdef _DEBUG
#define p3d_DbgAssert(expr) p3d_Assert(expr)
#else
#define p3d_DbgAssert(expr)
#endif
