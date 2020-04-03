#pragma once

#include <imgui.h>

#include "p3d/core/logger.h"

struct LogEntry {
public:
    LogEntry(std::string type, const ImU32 color, const std::string& str)
    {
        m_str = str;
        m_color = color;
        m_type = type;
    }
    ~LogEntry() {}
    const std::string& str() { return m_str; }
    const std::string& type() { return m_type; }
    ImU32 color() const { return m_color; }

private:
    ImU32 m_color{0xFFFFFF};
    std::string m_type{};
    std::string m_str{};
};

class ConsoleLogger : public Logger
{
public:
    ConsoleLogger() : Logger() {}
    ~ConsoleLogger() {}
    void render(ImGuiWindowFlags flags = 0);

    void setFont(ImFont* f) { font = f; }
    void addEntry(const LogEntry&& e) { Lines.push_back(e); }

    void print(const char* __restrict format, ...) override
    {
        int final_n, n = static_cast<int>(strlen(format)) * 2;
        /* Reserve two times as much as the length of the fmt_str */
        std::unique_ptr<char[]> formatted;
        va_list ap;
        while (1) {
            formatted.reset(new char[n]); /* Wrap the plain char array into the
                                             unique_ptr */
            strcpy(&formatted[0], format);
            va_start(ap, format);
            final_n = vsnprintf(&formatted[0], n, format, ap);
            va_end(ap);
            if (final_n < 0 || final_n >= n)
                n += abs(final_n - n + 1);
            else
                break;
        }
        std::string ret = std::string(formatted.get());
        Lines.emplace_back(m_type.c_str(), m_color, ret);
    }

private:
    void clear()
    {
        Buf.clear();
        Lines.clear();
        // Lines.push_back(LogEntry("Hello"));
    }

    ImFont* font = nullptr;
    ImGuiTextBuffer Buf;
    ImGuiTextFilter Filter;

    // Index to lines offset. We maintain this with AddLog() calls,
    // allowing us to have a random access on lines
    std::deque<LogEntry> Lines;

    bool AutoScroll = true;  // Keep scrolling if already at the bottom
    bool m_separateWindow = false;
};
