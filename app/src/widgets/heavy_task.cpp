#include "heavy_task.h"

#include <future>
#include <vector>

#include "p3d/core.h"
#include "p3d/logger.h"
#include "p3d/tasks.h"

std::future<void> HeavyTask::impl::heavyAsyncTask;
bool              HeavyTask::impl::startedHeavyCalculus = false;

void HeavyTask::draw(ImFont *monofont)
{
    bool heavyCalcStarted = impl::startedHeavyCalculus;
    if (heavyCalcStarted)
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

        bool heavyCalcInProgress = false;
        try {
            heavyCalcInProgress = impl::heavyAsyncTask.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
        } catch (const std::future_error& e) {
            LOG_ERR("Caught a future_error with code %i: %s", e.code(), e.what());
        }

        if (heavyCalcInProgress)
        {
            counter++;
            ImGui::SetNextWindowSize(ImVec2(400, 200));
            ImGui::OpenPopup("Please wait...##popup");
            if (ImGui::BeginPopupModal("Please wait...##popup",
                                       nullptr,
                                       ImGuiWindowFlags_Modal | ImGuiWindowFlags_NoResize
                                           | ImGuiWindowFlags_NoSavedSettings))
            {

                ImGui::Dummy(ImVec2(20.0,20.0));

                auto taskName = p3d::task::name();
                if (taskName.empty()) {
                    auto textSize = ImGui::CalcTextSize("Please wait");
                    ImGui::SetCursorPosX((ImGui::GetWindowSize().x - textSize.x) * 0.5f);
                    ImGui::Text("Please wait");
                } else {
                    auto textSize = ImGui::CalcTextSize(taskName.c_str());
                    ImGui::SetCursorPosX((ImGui::GetWindowSize().x - textSize.x) * 0.5f);
                    ImGui::Text("%s", taskName.c_str());
                }

                ImGui::Dummy(ImVec2(20.0,10.0));

                if (monofont) ImGui::PushFont(monofont);
                char temp = (counter / 4) & 0x0F;
                auto textSize = ImGui::CalcTextSize("[====]");
                ImGui::SetCursorPosX((ImGui::GetWindowSize().x - textSize.x) * 0.5f);
                ImGui::Text("%s", chars[temp].c_str());
                if (monofont) ImGui::PopFont();

                bool isProgressTask = p3d::task::total() > 0.0;
                if (isProgressTask) {
                    ImGui::Dummy(ImVec2(20.0,20.0));
                    char buf[32];
                    sprintf(buf, "%i%%", int(100 * p3d::task::progressPercent()));
                    ImGui::SetCursorPosX((ImGui::GetWindowSize().x - 260.0f) * 0.5f);
                    ImGui::ProgressBar(p3d::task::progressPercent(), ImVec2(260.0f,0.0f), buf);
                }

                ImGui::EndPopup();
            }
        } else {
            try {
                impl::heavyAsyncTask.get();
            } catch (const p3d::Exception& e) {
                if (p3d::task::name().empty())
                    LOG_ERR("error: %s", e.what());
                else
                    LOG_ERR("%s failed: %s", p3d::task::name().c_str(), e.what());
            } catch (const cv::Exception& e) {
                LOG_ERR("%s failed: opencv ", p3d::task::name().c_str(), e.what());
            } catch (...) {
                LOG_ERR("%s failed", p3d::task::name().c_str());
            }
            impl::startedHeavyCalculus = false;
            p3d::task::reset();
            ImGui::CloseCurrentPopup();
        }
    }
}
