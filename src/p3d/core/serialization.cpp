#include "serialization.h"

namespace impl{

int registerTypes() {
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: primitives" << std::endl;
        SERIALIZE_TYPE(float,"float"_hs);
        SERIALIZE_TYPE(double,"double"_hs);
        SERIALIZE_TYPE(int,"int"_hs);
        SERIALIZE_TYPE(std::string,"std::string"_hs);

        entt::meta<bool>()
                .type("bool"_hs)
                .func<&impl::_readBool>("_read"_hs)
                .func<&impl::_write<bool>>("_write"_hs);

        SERIALIZE_TYPE_VEC(float,"vector_float"_hs);
        SERIALIZE_TYPE_VEC(double,"vector_double"_hs);
        SERIALIZE_TYPE_VEC(int,"vector_int"_hs);
        SERIALIZE_TYPE_VEC(std::string,"vector_std::string"_hs);

        SERIALIZE_TYPE_EIGEN(float, 3, 3, "Eigen__float33"_hs);
        SERIALIZE_TYPE_EIGEN(float, 3, 4, "Eigen__float34"_hs);
        SERIALIZE_TYPE_EIGEN(float,-1,-1, "Eigen__floatXX"_hs);

        SERIALIZE_TYPE_EIGEN(double,2,1, "Eigen__double21"_hs);
        SERIALIZE_TYPE_EIGEN(double,3,1, "Eigen__double31"_hs);
        SERIALIZE_TYPE_EIGEN(double,3,4, "Eigen__double34"_hs);
        SERIALIZE_TYPE_EIGEN(double,3,3, "Eigen__double33"_hs);
        SERIALIZE_TYPE_EIGEN(double,-1,-1, "Eigen__doubleXX"_hs);

        //        SERIALIZE_TYPE_EIGEN(double,3,4, "Eigen_double34");

        firstCall = false;
    }
    return firstCall;
}

int dummyPrimitives_ = registerTypes();

}
