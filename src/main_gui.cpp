#include "app.h"
#include "gui/widget_console.h"

#ifdef POLLEN3D_OPENGL
#include "app_opengl.h"
#else

#endif

#ifdef _WIN32
#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")
#endif

int main()
{
#ifndef POLLEN3D_DEBUG
    // no output in standard console
    std::cout.setstate(std::ios_base::failbit);
    std::cerr.setstate(std::ios_base::failbit);
#endif

#ifdef POLLEN3D_OPENGL
    Application * app = new ApplicationOpenGL();
#endif

    app->init();
    app->run();
    delete app;

    return EXIT_SUCCESS;
}
