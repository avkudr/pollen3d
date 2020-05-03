#include "common.h"

#include "portable-file-dialogs/portable-file-dialogs.h"

#include "p3d/core.h"
#include "p3d/utils.h"

std::vector<std::string> loadImagesDialog()
{
    auto f = pfd::open_file("Choose images", "",
    { "Images (.jpg .jpeg .png .tif .tiff)",
      "*.jpg *.jpeg *.png *.tif *.tiff",
      "All Files", "*" }, true);
    return f.result();
}

std::string openProjectDialog()
{
    std::string ext = "*" + std::string(P3D_PROJECT_EXTENSION);
    std::string filter = "pollen3d project (" + std::string(P3D_PROJECT_EXTENSION) + ")";

    auto f = pfd::open_file("Open project", "",
    { filter, ext, "All Files", "*" },
                            false);
    if (f.result().empty()) return "";
    return f.result()[0];
}

std::string saveProjectDialog()
{
    std::string filter = "pollen3d project (" + std::string(P3D_PROJECT_EXTENSION) + ")";
    std::string ext = "*" + std::string(P3D_PROJECT_EXTENSION);

    auto f = pfd::save_file("Save project", "",
    { filter, ext},
                            true);

    std::string out(f.result());
    if (!p3d::utils::endsWith(out, P3D_PROJECT_EXTENSION))
        out += P3D_PROJECT_EXTENSION;
    return out;
}

std::string exportPointCloudDialog()
{
    std::string filter = "point cloud (.ply)";
    std::string ext = "*.ply";

    auto f = pfd::save_file("Save project", "",
    { filter, ext},
                            true);

    std::string out(f.result());
    if (!p3d::utils::endsWith(out, ".ply"))
        out += ".ply";
    return out;
}
