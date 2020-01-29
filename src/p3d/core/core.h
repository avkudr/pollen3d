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

typedef int p3dSetting;
enum p3dSetting_
{
    p3dSetting_projectSettings   = 1000,
    p3dSetting_matcherCurAlg     = 1001,
    p3dSetting_matcherFilterCoef = 1002,
};

typedef int p3dData;
enum p3dData_
{
    p3dData_projectData = 2000,
    p3dData_projectPath = 2001,
    p3dData_dummy       = 2002,
    p3dData_images      = 2003,
    p3dData_imagePairs  = 2004,
};
