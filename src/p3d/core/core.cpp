#include "core.h"

#include "p3d/console_logger.h"

using std::string;

Exception::Exception() {
    code = 0;
    line = 0;
}

Exception::Exception(int _code, const string& _err, const string& _func, const string& _file, int _line)
: code(_code), err(_err), func(_func), file(_file), line(_line)
{
    formatMessage();
}

Exception::~Exception() noexcept {}

const char* Exception::what() const noexcept { return msg.c_str(); }

void Exception::formatMessage()
{
    if( func.size() > 0 )
        msg = format("%s:%d: error: (%d) %s in function %s\n", file.c_str(), line, code, err.c_str(), func.c_str());
    else
        msg = format("%s:%d: error: (%d) %s\n", file.c_str(), line, code, err.c_str());
}

#include <stdarg.h>

string format( const char* fmt, ... )
{
    char buf[1 << 16];
    va_list args;
    va_start( args, fmt );
#ifdef WIN32
    vsprintf_s( buf, fmt, args );
#elif __linux__ || __APPLE__
    vsprintf( buf, fmt, args );
#endif
    return string(buf);
}

const char* arErrorStr( int status ){

    static char buf[256];

    switch (status) {
        case Exception::ERROR_CODE :      return "Application error";
        case Exception::ASSERTION_CODE :  return "Assertion failed";
    };

#ifdef WIN32
    sprintf_s(buf, "Unknown %s code %d", status >= 0 ? "status":"error", status);
#elif __linux__ || __APPLE__
    sprintf(buf, "Unknown %s code %d", status >= 0 ? "status":"error", status);
#endif
    return buf;
}

void error(const Exception &exc)
{
    const char* errorStr = arErrorStr(exc.code);
    char buf[1 << 16];

#ifdef WIN32
    sprintf_s( buf, "Pollen3D Error: %s (%s) \n    function: %s\n    file    : %s\n    line    : %d",
        errorStr, exc.err.c_str(), exc.func.size() > 0 ?
        exc.func.c_str() : "unknown function", exc.file.c_str(), exc.line );
#elif __linux__ || __APPLE__
    sprintf( buf, "Pollen3D Error: %s (%s) \n    function: %s\n    file    : %s\n    line    : %d",
        errorStr, exc.err.c_str(), exc.func.size() > 0 ?
        exc.func.c_str() : "unknown function", exc.file.c_str(), exc.line );
#endif

    LOG_ERR(buf);
    throw exc;
}
