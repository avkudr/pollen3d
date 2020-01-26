#include "p3d/app.h"
#ifdef POLLEN3D_OPENGL
#include "p3d/app_opengl.h"
#else

#endif

int main(){

    meta::reflect<Image>(p3d_hashStr("Image"))
        .data<&Image::setPath,&Image::getPath>(p3d_hash(p3dImage_path));
    SERIALIZE_TYPE_VECS(Image,"vector_Image");

    impl::registerTypes();

#ifdef POLLEN3D_OPENGL
    Application * app = new ApplicationOpenGL();
#endif

    app->init();
    app->run();
    delete app;

    return 0;
}
