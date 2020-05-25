#include "app.h"
#include "version.h"
#include "widgets/widget_console.h"

#ifdef POLLEN3D_OPENGL
#include "app_opengl.h"
#endif

#ifdef _WIN32
#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")
#define IDI_ICON_1 102
#endif

int main(int argc, char** argv)
{
    cv::String keys =
        "{project p | | full path to a project}"
        "{help h    | | show help message}";  // optional, show help optional

    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help")) {
        std::cout
            << "\n pollen3d v" << POLLEN3D_VERSION_MAJOR << "." << POLLEN3D_VERSION_MINOR
            << "." << POLLEN3D_VERSION_PATCH
            << "\n\n"
               " pollen3d is a software used for 3D reconstruction from images captured "
               " with an affine camera like a Scanning Electron Microscope.\n"
               " Usage:\n"
               " ./pollen3d \n"
               "    runs the software with an empty project\n"
               "    [-p=<project_path> run the software and open an existing project]\n"
               "    [-help show this message]\n"
               "\n"
               " git repository: https://github.com/avkudr/pollen3d/"
            << "\n"
            << std::endl;
        return 0;
    }

#ifdef POLLEN3D_OPENGL
    Application* app = new ApplicationOpenGL();
#endif

    app->init();

    bool hasProject = parser.has("p");
    if (hasProject) {
        std::string projectPath = parser.get<cv::String>("p");
        app->openProject(projectPath);
    }

    app->run();
    delete app;

    return EXIT_SUCCESS;
}
