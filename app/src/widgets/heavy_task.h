#pragma once

#include <chrono>
#include <future>

#include "imgui.h"

namespace HeavyTask {

namespace impl {
extern std::future<void> heavyAsyncTask;
extern bool              startedHeavyCalculus;
} // namespace impl

template<typename Func, class... Args>
void run(Func f, Args... args)
{
    impl::startedHeavyCalculus = true;
    impl::heavyAsyncTask       = std::async(std::launch::async, f, args...);
}

void draw(ImFont *monofont);

} // namespace HeavyTask
