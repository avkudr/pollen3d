#pragma once

#include "p3d/core.h"
#include "p3d/image/feature_extraction.h"
#include "p3d/serialization.h"
#include "p3d/utils.h"

namespace p3d
{

enum P3D_API p3dFeatExtraction_ {
    p3dFeatExtraction_method,
    p3dFeatExtraction_descType,
    p3dFeatExtraction_descSize,
    p3dFeatExtraction_descChannels,
    p3dFeatExtraction_descThreshold
};

class P3D_API FeatExtractionPars : public Serializable<FeatExtractionPars>
{
public:
    static int initMeta();
    static const char* classNameStatic() { return "FeatExtractionPars"; }

    int   method{0}; // unused for now (only AKAZE)
    int   descType{5};
    int   descSize{0};
    int   descChannels{3};
    float threshold{0.001f};

    void print() const
    {
        std::cout << "FeatExtractionPars" << std::endl;
        std::cout << " - " << p3dFeatExtraction_method << ": " << method << std::endl;
        std::cout << " - " << p3dFeatExtraction_descType << ": " << descType << std::endl;
        std::cout << " - " << p3dFeatExtraction_descSize << ": " << descSize << std::endl;
        std::cout << " - " << p3dFeatExtraction_descChannels << ": " << descChannels << std::endl;
        std::cout << " - " << p3dFeatExtraction_descThreshold << ": " << threshold << std::endl;
    }
};

struct P3D_API FeatExtractionUtil
{
    static void extract(const cv::Mat &            im,
                        const FeatExtractionPars & pars,
                        std::vector<cv::KeyPoint> &kpts,
                        cv::Mat &                  desc);
};
}  // namespace p3d
