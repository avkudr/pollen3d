#pragma once

#include <imgui.h>

#include <mutex>

#include "p3d/logger.h"
#include "p3d/utils.h"

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

class WidgetLogger : public p3d::logger::Logger
{
public:
    WidgetLogger() : p3d::logger::Logger() {}
    ~WidgetLogger() {}
    void render(ImGuiWindowFlags flags = 0);

    void setFont(ImFont* f) {
        const std::lock_guard<std::mutex> lock(m_mutex);
        font = f;
    }
    void addEntry(const LogEntry&& e) {
        const std::lock_guard<std::mutex> lock(m_mutex);
        Lines.push_back(e);
    }

    void print(const char* __restrict format, ...) override
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
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
        auto msgLines = p3d::utils::split(ret, "\n");
        for (const auto& msg : msgLines) Lines.emplace_back(m_type.c_str(), m_color, msg);
    }

private:
    void clear()
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
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

    std::mutex m_mutex;
};
