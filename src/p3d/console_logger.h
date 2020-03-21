#pragma once

#include <string>
#include <iostream>
#include <cstdio>
#include <string.h>
#include <deque>
#include <memory>


#if __has_include("imgui.h")
#include <imgui.h>
#define LOG_TO_GUI
#else
#define LOG_TO_STOUT
#endif

#ifdef LOG_TO_STOUT
    #define LOG_OK(...)   printf("[ OK ]"); printf(__VA_ARGS__); printf("\n"); fflush(stdout)
    #define LOG_INFO(...) printf("[INFO]"); printf(__VA_ARGS__); printf("\n"); fflush(stdout)
    #define LOG_WARN(...) printf("[WARN]"); printf(__VA_ARGS__); printf("\n"); fflush(stdout)
    #define LOG_ERR(...)  printf("[ERR ]"); printf(__VA_ARGS__); printf("\n"); fflush(stdout)
#else
#ifdef LOG_TO_GUI
    #define COLOR_INFO     ImVec4{0.6f,0.6f,0.6f,1.0f}
    #define COLOR_WARN     ImVec4{0.9f,0.9f,0.2f,1.0f}
    #define COLOR_ERR_     ImVec4{0.9f,0.2f,0.2f,1.0f}
    #define COLOR_GREEN    ImVec4{0.0f,0.7f,0.3f,1.0f}
    #define COLOR_PINK     ImVec4{1.0f,0.3f,0.6f,1.0f}

    struct LogEntry{
    public:
        LogEntry(char * text) : m_ret(text) {}
        LogEntry(const char * type, const ImVec4 color, const std::string fmt_str, ...) {
            int final_n, n = static_cast<int>(fmt_str.size()) * 2; /* Reserve two times as much as the length of the fmt_str */
            std::unique_ptr<char[]> formatted;
            va_list ap;
            while(1) {
                formatted.reset(new char[n]); /* Wrap the plain char array into the unique_ptr */
                strcpy(&formatted[0], fmt_str.c_str());
                va_start(ap, fmt_str);
                final_n = vsnprintf(&formatted[0], n, fmt_str.c_str(), ap);
                va_end(ap);
                if (final_n < 0 || final_n >= n)
                    n += abs(final_n - n + 1);
                else
                    break;
            }
            m_ret = std::string(formatted.get());
            m_color = color;
            m_type = type;
        }

        LogEntry(const char * type, char * text) : m_ret(text), m_type(type) {}

        ~LogEntry(){

        }
        const std::string & str() { return m_ret; }
        const char * type() { return m_type; }
        ImVec4 color() const { return m_color; }
    private:
        ImVec4 m_color{0.9f,0.9f,0.9f,1.0f};
        const char * m_type = "INFO";
        std::string m_ret;
    };

    class ConsoleLogger
    {
    public:
        static ConsoleLogger * get() {
            if (!m_instance) {
               m_instance = new ConsoleLogger();
            }
            return m_instance;
        }

        void render(ImGuiWindowFlags flags = 0);

        void setFont(ImFont * f) { font = f; }
        void addEntry(char * str) { addEntry(LogEntry(str)); }
        void addEntry(const LogEntry&& e)
        {
            Lines.push_back(e);
        }

    private:
        ConsoleLogger() {

        }

        void clear()
        {
            Buf.clear();
            Lines.clear();
            //Lines.push_back(LogEntry("Hello"));
        }

        ImFont * font = nullptr;
        ImGuiTextBuffer      Buf;
        ImGuiTextFilter      Filter;
        std::deque<LogEntry> Lines;        // Index to lines offset. We maintain this with AddLog() calls, allowing us to have a random access on lines
        bool            AutoScroll = true;     // Keep scrolling if already at the bottom

        static ConsoleLogger * m_instance;

        bool m_separateWindow = false;
    };

    #define LOG_OK(...)   ConsoleLogger::get()->addEntry(LogEntry(" OK ",COLOR_GREEN,__VA_ARGS__))
    #define LOG_INFO(...) ConsoleLogger::get()->addEntry(LogEntry("INFO",COLOR_INFO,__VA_ARGS__))
    #define LOG_WARN(...) ConsoleLogger::get()->addEntry(LogEntry("WARN",COLOR_WARN,__VA_ARGS__))
    #define LOG_ERR(...)  ConsoleLogger::get()->addEntry(LogEntry("ERR ",COLOR_ERR_,__VA_ARGS__))
#endif
#endif

#ifdef POLLEN3D_DEBUG
#define LOG_DBG(...) ConsoleLogger::get()->addEntry(LogEntry("DEBUG",COLOR_PINK,__VA_ARGS__))
#else
#define LOG_DBG(...)
#endif
