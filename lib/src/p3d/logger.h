#pragma once

#include <stdarg.h>
#include <string.h>
#include <cstdio>
#include <deque>
#include <iostream>
#include <memory>
#include <string>

#include "p3d/core.h"

#define COLOR_INFO 4288256409  // ImVec4{0.6f,0.6f,0.6f,1.0f}
#define COLOR_WARN 4281591526  // ImVec4{0.9f,0.9f,0.2f,1.0f}
#define COLOR_ERR_ 4281545702  // ImVec4{0.9f,0.2f,0.2f,1.0f}
#define COLOR_OK 4283282176    // ImVec4{0.0f,0.7f,0.3f,1.0f}
#define COLOR_DBG 4288237055   // ImVec4{1.0f,0.3f,0.6f,1.0f}

namespace p3d::logger
{
class P3D_API Logger
{
public:
    Logger() {}
    virtual ~Logger() {}
    virtual void print(const char* format, ...) = 0;

    void setType(std::string type) { m_type = type; }
    void setColor(unsigned int color) { m_color = color; }

    void setOn() { m_isOn = true; }
    void setOff() { m_isOn = false; }
    bool isOn() const { return m_isOn; }

protected:
    std::string m_type{"[ OK ]"};
    unsigned int m_color{COLOR_OK};  // 32-bit unsigned integer - grey
    bool m_isOn{true};
};

class P3D_API StdLogger : public Logger
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

std::shared_ptr<Logger> P3D_API get();
void P3D_API set(std::shared_ptr<Logger> l);
void P3D_API setStd();
void P3D_API on();
void P3D_API off();

}  // namespace p3d::logger

#define LOG_IMPL(type, color, ...)                    \
    do {                                              \
        if (p3d::logger::get()                     \
            && p3d::logger::get()->isOn())         \
        {                                             \
            p3d::logger::get()->setType(type);     \
            p3d::logger::get()->setColor(color);   \
            p3d::logger::get()->print(__VA_ARGS__);\
        }                                             \
    } while (0)

#define LOG_OK(...) LOG_IMPL("* ", COLOR_OK, __VA_ARGS__)
#define LOG_INFO(...) LOG_IMPL("* ", COLOR_INFO, __VA_ARGS__)
#define LOG_WARN(...) LOG_IMPL("* ", COLOR_WARN, __VA_ARGS__)
#define LOG_ERR(...) LOG_IMPL("* ", COLOR_ERR_, __VA_ARGS__)

#ifdef POLLEN3D_DEBUG
#define LOG_DBG(...) LOG_IMPL("* ", COLOR_DBG, __VA_ARGS__)
#define LOG_DBG_WARN(...) LOG_IMPL("* ", COLOR_WARN, __VA_ARGS__)
#else
#define LOG_DBG(...) do {} while (0)
#define LOG_DBG_WARN(...) \
    do {                  \
    } while (0)
#endif
