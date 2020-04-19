#include "feature_extraction.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

using namespace p3d;

int dummyFeatExtractionPars_ = FeatExtractionPars::initMeta();

int FeatExtractionPars::initMeta()
{
    static bool firstCall = true;
    if (firstCall)
    {
        LOG_DBG("Reflecting: FeatExtractionPars");
        entt::meta<FeatExtractionPars>()
            .alias("FeatExtractionPars"_hs)
            .data<&FeatExtractionPars::method>(P3D_ID_TYPE(p3dFeatExtraction_method))
            .data<&FeatExtractionPars::descType>(P3D_ID_TYPE(p3dFeatExtraction_descType))
            .data<&FeatExtractionPars::descSize>(P3D_ID_TYPE(p3dFeatExtraction_descSize))
            .data<&FeatExtractionPars::descChannels>(P3D_ID_TYPE(p3dFeatExtraction_descChannels))
            .data<&FeatExtractionPars::threshold>(P3D_ID_TYPE(p3dFeatExtraction_descThreshold));

        SERIALIZED_ADD_READ_WRITE(FeatExtractionPars);
        firstCall = false;
    }
    return 0;
}

void FeatExtractionUtil::extract(const cv::Mat &            im,
                                 const FeatExtractionPars & pars,
                                 std::vector<cv::KeyPoint> &kpts,
                                 cv::Mat &                  desc)
{
    try
    {
        cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(pars.descType,
                                                     pars.descSize,
                                                     pars.descChannels,
                                                     pars.threshold);
        akaze->detectAndCompute(im, cv::noArray(), kpts, desc);
        akaze.release();

    } catch (...)
    {
        kpts.clear();
        desc = cv::Mat();
    }
}
