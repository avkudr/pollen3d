#include <iostream>

#include "p3d/project.h"
#include "p3d/tasks.h"

int main()
{
    p3d::Project p;

    std::vector<std::string> imgs{
        std::string(P3D_IMAGES_DIR) + "/brassica/Brassica01.jpg",
        std::string(P3D_IMAGES_DIR) + "/brassica/Brassica02.jpg",
    };

    p3d::loadImages(p, imgs);
    p3d::extractFeatures(p);
    p3d::matchFeatures(p);

    std::cout << "Nb feature, im0: " << p.getImage(0).nbFeatures() << std::endl;
    std::cout << "Nb feature, im0: " << p.getImage(1).nbFeatures() << std::endl;
    std::cout << "Nb matches, im0-im1: " << p.getImagePair(0).nbMatches() << std::endl;

    p3d::undo();

    std::cout << "Nb matches, im0-im1: " << p.getImagePair(0).nbMatches() << std::endl;

    return 0;
}
