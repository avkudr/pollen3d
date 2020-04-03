#include "match.h"

using namespace p3d;

int Match::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: Match" << std::endl;
        entt::meta<Match>()
            .type("Match"_hs)
            .data<&Match::iPtL>(P3D_ID_TYPE(p3dMatch_iPtL))
            .data<&Match::iPtR>(P3D_ID_TYPE(p3dMatch_iPtR))
            .data<&Match::distance>(P3D_ID_TYPE(p3dMatch_distance));

        firstCall = false;

        SERIALIZE_TYPE_VECS(Match, "vector_Match"_hs);
    }
    return 0;
}
int dummyMatch_ = Match::initMeta();
