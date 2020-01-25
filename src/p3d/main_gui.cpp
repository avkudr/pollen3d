#include "p3d/app.h"
#ifdef POLLEN3D_OPENGL
#include "p3d/app_opengl.h"
#else

#endif

int main(){

    impl::registerTypes();

#ifdef POLLEN3D_OPENGL
    Application * app = new ApplicationOpenGL();
#endif

    app->init();
    app->run();
    delete app;

    return 0;
}
