#include "app.h"
#include "widgets/widget_console.h"

#ifdef POLLEN3D_OPENGL
#include "app_opengl.h"
#endif

#ifdef _WIN32
#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")
#define IDI_ICON_1 102
#endif

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

#ifdef POLLEN3D_OPENGL
    Application* app = new ApplicationOpenGL();
#endif

    app->init();
    app->run();
    delete app;

    return EXIT_SUCCESS;
}
