#include "heavy_task.h"

#include <future>

std::future<void> HeavyTask::impl::heavyAsyncTask;
bool              HeavyTask::impl::startedHeavyCalculus = false;

void HeavyTask::draw(ImFont *monofont)
{
    if (impl::startedHeavyCalculus)
    {
        static unsigned int             counter = 0;
        static std::vector<std::string> chars{"[    ]",
                                              "[    ]",
                                              "[=   ]",
                                              "[==  ]",
                                              "[=== ]",
                                              "[ ===]",
                                              "[  ==]",
                                              "[   =]",
                                              "[    ]",
                                              "[   =]",
                                              "[  ==]",
                                              "[ ===]",
                                              "[====]",
                                              "[=== ]",
                                              "[==  ]",
                                              "[=   ]"};

        if (impl::heavyAsyncTask.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        {
            counter++;
            ImGui::SetNextWindowSize(ImVec2(400, 200));
            ImGui::OpenPopup("pollen3d");
            if (ImGui::BeginPopupModal("pollen3d",
                                       nullptr,
                                       ImGuiWindowFlags_Modal | ImGuiWindowFlags_NoResize
                                           | ImGuiWindowFlags_NoSavedSettings))
            {
                ImGui::SetCursorPos(ImVec2(135, 90));
                ImGui::Text("Please wait");
                ImGui::SameLine();

                if (monofont) ImGui::PushFont(monofont);
                char temp = (counter / 4) & 0x0F;
                ImGui::Text("%s", chars[temp].c_str());
                if (monofont) ImGui::PopFont();

                ImGui::EndPopup();
            }
        } else
        {
            impl::heavyAsyncTask.get();
            impl::startedHeavyCalculus = false;
            ImGui::CloseCurrentPopup();
        }
    }
}
