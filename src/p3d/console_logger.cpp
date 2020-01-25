#include "console_logger.h"

ConsoleLogger *ConsoleLogger::m_instance = nullptr;

void ConsoleLogger::render(ImGuiWindowFlags flags) {

    if (flags) ImGui::Begin("Console", nullptr, flags);
    else ImGui::BeginChild("Console");

    if (ImGui::BeginPopupContextItem("item context menu"))
    {
        ImGui::Checkbox("Auto-scroll", &AutoScroll);
        if(ImGui::Button("Clear")) {
            clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    ImGui::BeginChild("scrolling", ImVec2(0,0), false, ImGuiWindowFlags_HorizontalScrollbar); 

    if (font) ImGui::PushFont(font);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
    if (Filter.IsActive())
    {
        // In this example we don't use the clipper when Filter is enabled.
        // This is because we don't have a random access on the result on our filter.
        // A real application processing logs with ten of thousands of entries may want to store the result of search/filter.
        // especially if the filtering function is not trivial (e.g. reg-exp).
        for (int line_no = 0; line_no < Lines.size(); line_no++)
        {
//            const char* line_start = buf + Lines[line_no];
//            const char* line_end = (line_no + 1 < Lines.Size) ? (buf + Lines[line_no + 1] - 1) : buf_end;
//            if (Filter.PassFilter(line_start, line_end))
//                ImGui::TextUnformatted(line_start, line_end);
        }
    }
    else
    {
        // The simplest and easy way to display the entire buffer:
        //   ImGui::TextUnformatted(buf_begin, buf_end);
        // And it'll just work. TextUnformatted() has specialization for large blob of text and will fast-forward to skip non-visible lines.
        // Here we instead demonstrate using the clipper to only process lines that are within the visible area.
        // If you have tens of thousands of items and their processing cost is non-negligible, coarse clipping them on your side is recommended.
        // Using ImGuiListClipper requires A) random access into your data, and B) items all being the  same height,
        // both of which we can handle since we an array pointing to the beginning of each line of text.
        // When using the filter (in the block of code above) we don't have random access into the data to display anymore, which is why we don't use the clipper.
        // Storing or skimming through the search result would make it possible (and would be recommended if you want to search through tens of thousands of entries)

        while(Lines.size() > 150) Lines.pop_front();
        ImGuiListClipper clipper;
        clipper.Begin(Lines.size());
        while (clipper.Step())
        {
            for (int line_no = clipper.DisplayStart; line_no < clipper.DisplayEnd; line_no++)
            {
                if (line_no < Lines.size()) {
                    ImGui::PushStyleColor(ImGuiCol_Text, Lines[line_no].color());
                    ImGui::Text("[%s] %s", Lines[line_no].type(), Lines[line_no].str().c_str());
                    ImGui::PopStyleColor();
                }
            }
        }
        clipper.End();
    }
    ImGui::PopStyleVar();
    if (font) ImGui::PopFont();

    if (AutoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
        ImGui::SetScrollHereY(1.0f);

    ImGui::EndChild();
    ImGui::OpenPopupOnItemClick("item context menu", 1);

    if (flags) ImGui::End();
    else ImGui::EndChild();
}
