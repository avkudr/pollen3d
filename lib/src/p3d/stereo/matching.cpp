#include "matching.h"

#include <stack>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <openMVG/tracks/tracks.hpp>

using namespace p3d;

int dummyMatch_ = Match::initMeta();

int Match::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        LOG_DBG("Reflecting: Match");
        entt::meta<Match>()
            .alias(p3d::alias(classNameStatic()))
            .data<&Match::iPtL>(P3D_ID_TYPE(p3dMatch_iPtL))
            .data<&Match::iPtR>(P3D_ID_TYPE(p3dMatch_iPtR))
            .data<&Match::distance>(P3D_ID_TYPE(p3dMatch_distance));

        firstCall = false;

        SERIALIZE_TYPE_VECS(Match);
    }
    return 0;
}

int dummyMatching_ = MatchingPars::initMeta();

int MatchingPars::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        SERIALIZED_ADD_READ_WRITE(MatchingPars);

        entt::meta<MatchingPars>()
            .data<&MatchingPars::method>(P3D_ID_TYPE(p3dMatching_method))
            .data<&MatchingPars::filterCoeff>(P3D_ID_TYPE(p3dMatching_filterCoeff));

        firstCall = false;
    }
    return 0;
}

void MatchingUtil::match(const cv::Mat &descL, const cv::Mat &descR,
                         const MatchingPars &pars, std::vector<Match> &matches)
{
    matches.clear();

    std::vector<std::vector<cv::DMatch>> poor_matches;
    std::vector<cv::DMatch> dmatches;

    int method = cv::DescriptorMatcher::BRUTEFORCE_L1;

    switch (pars.method) {
    case MatchingMethod_FLANNBASED:
        method = cv::DescriptorMatcher::FLANNBASED;
        break;
    case MatchingMethod_BRUTEFORCE:
        method = cv::DescriptorMatcher::BRUTEFORCE;
        break;
    case MatchingMethod_BRUTEFORCE_L1:
        method = cv::DescriptorMatcher::BRUTEFORCE_L1;
        break;
    case MatchingMethod_BRUTEFORCE_HAMMING:
        method = cv::DescriptorMatcher::BRUTEFORCE_HAMMING;
        break;
    case MatchingMethod_BRUTEFORCE_HAMMINGLUT:
        method = cv::DescriptorMatcher::BRUTEFORCE_HAMMINGLUT;
        break;
    case MatchingMethod_BRUTEFORCE_SL2:
        method = cv::DescriptorMatcher::BRUTEFORCE_SL2;
        break;
    default: break;
    }

    try {
        cv::Ptr<cv::DescriptorMatcher> matcher =
            cv::DescriptorMatcher::create(method);
        matcher->knnMatch(descL, descR, poor_matches, 2);  // 2  best matches

        for (int im = 0; im < cv::min(descL.rows - 1, (int)poor_matches.size());
             im++) {
            if ((poor_matches[im][0].distance <
                 pars.filterCoeff * (poor_matches[im][1].distance)) &&
                ((int)poor_matches[im].size() <= 2 &&
                 (int)poor_matches[im].size() > 0)) {
                dmatches.push_back(poor_matches[im][0]);

                matches.emplace_back(Match(poor_matches[im][0].queryIdx,
                                           poor_matches[im][0].trainIdx,
                                           poor_matches[im][0].distance));
            }
        }
    } catch (...) {
        matches.clear();
    }
}

void MatchingUtil::matchesMapsToTable(PairWiseMatchesMap &&matchesMaps, Tracks &tracks)
{
    // Create some tracks for image (A,B,C)
    // {A,B,C} imageId will be {0,1,2}
    // For those image link some features id depicted below
    // A    B    C
    // 0 -> 0 -> 0
    // 1 -> 1 -> 6
    // 2 -> 3

    // Result:
    // 0, {(0,0) (1,0) (2,0)}
    // 1, {(0,1) (1,1) (2,6)}
    // 2, {(0,2) (1,3)}

    openMVG::tracks::TracksBuilder trackBuilder;
    trackBuilder.Build(matchesMaps);

    // map < TrackId, vector <Image,PointId> >
    using STLMAPTracks = std::map<uint32_t, std::map<uint32_t, uint32_t>>;

    trackBuilder.ExportToSTL(tracks);

    LOG_INFO("Nb tracks: %i", tracks.size());

    {  // stats
        std::map<int, int> stats;
        for (const auto &track : tracks) {
            const auto nb = track.second.size();
            if (stats.count(nb) == 0)
                stats[nb] = 1;
            else
                stats[nb]++;
        }
        LOG_INFO("Stats. Total of %i matches", tracks.size());
        for (const auto &kv : stats) LOG_INFO(" - in %i imgs: %i", kv.first, kv.second);
    }
}
