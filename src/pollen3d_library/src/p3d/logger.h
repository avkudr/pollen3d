#pragma once

#include <stdarg.h>
#include <string.h>
#include <cstdio>
#include <deque>
#include <iostream>
#include <memory>
#include <string>

#define COLOR_INFO 4288256409  // ImVec4{0.6f,0.6f,0.6f,1.0f}
#define COLOR_WARN 4281591526  // ImVec4{0.9f,0.9f,0.2f,1.0f}
#define COLOR_ERR_ 4281545702  // ImVec4{0.9f,0.2f,0.2f,1.0f}
#define COLOR_OK 4283282176    // ImVec4{0.0f,0.7f,0.3f,1.0f}
#define COLOR_DBG 4288237055   // ImVec4{1.0f,0.3f,0.6f,1.0f}

namespace p3d
{
class Logger
{
public:
    Logger() {}
    virtual ~Logger() {}
    virtual void print(const char* format, ...) = 0;

    void setType(std::string type) { m_type = type; }
    void setColor(unsigned int color) { m_color = color; }

protected:
    std::string m_type{"[ OK ]"};
    unsigned int m_color{COLOR_OK};  // 32-bit unsigned integer - grey
};

extern std::shared_ptr<Logger> _logger;

class StdLogger : public Logger
{
public:
    StdLogger() : Logger() {}
    ~StdLogger() {}
    void print(const char* format, ...) override
    {
        printf("%s", m_type.c_str());
        va_list arglist;
        va_start(arglist, format);
        vprintf(format, arglist);
        va_end(arglist);
        printf("\n");
        fflush(stdout);
    }
};

static void initStdLoger() { _logger = std::make_shared<StdLogger>(); }
}  // namespace p3d

#define LOG_IMPL(type, color, ...)            \
    do {                                      \
        if (p3d::_logger) {                   \
            p3d::_logger->setType(type);      \
            p3d::_logger->setColor(color);    \
            p3d::_logger->print(__VA_ARGS__); \
        }                                     \
    } while (0)

#define LOG_OK(...) LOG_IMPL("[ OK ] ", COLOR_OK, __VA_ARGS__)
#define LOG_INFO(...) LOG_IMPL("[INFO] ", COLOR_INFO, __VA_ARGS__)
#define LOG_WARN(...) LOG_IMPL("[WARN] ", COLOR_WARN, __VA_ARGS__)
#define LOG_ERR(...) LOG_IMPL("[ERR ] ", COLOR_ERR_, __VA_ARGS__)

#ifdef POLLEN3D_DEBUG
#define LOG_DBG(...) LOG_IMPL("[DEBUG] ", COLOR_DBG, __VA_ARGS__)
#else
#define LOG_DBG(...)
#endif
