#include "app.h"
#include "gui/widget_console.h"

#ifdef POLLEN3D_OPENGL
#include "app_opengl.h"
#else

#endif

int main()
{
#ifdef POLLEN3D_OPENGL
    Application * app = new ApplicationOpenGL();
#endif

    app->init();
    app->run();
    delete app;

    return EXIT_SUCCESS;
}
