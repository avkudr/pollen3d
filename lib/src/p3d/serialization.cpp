#include "serialization.h"

namespace p3d::impl{

int dummyPrimitives_ = registerTypes();

int registerTypes()
{
    LOG_DBG("Reflecting: primitives");

    SERIALIZE_TYPE(float,"float"_hs);
    SERIALIZE_TYPE(double,"double"_hs);
    SERIALIZE_TYPE(int,"int"_hs);
    SERIALIZE_TYPE(std::string,"std::string"_hs);

    entt::meta<bool>()
        .alias("bool"_hs)
        .func<&p3d::impl::_readBool>("_read"_hs)
        .func<&p3d::impl::_write<bool>>("_write"_hs);

    SERIALIZE_TYPE_VEC(float,"vector_float"_hs);
    SERIALIZE_TYPE_VEC(double,"vector_double"_hs);
    SERIALIZE_TYPE_VEC(int,"vector_int"_hs);
    SERIALIZE_TYPE_VEC(std::string,"vector_std::string"_hs);

    SERIALIZE_TYPE_EIGEN(float, 3, 3, "Eigen__float33"_hs);
    SERIALIZE_TYPE_EIGEN(float, 3, 4, "Eigen__float34"_hs);
    SERIALIZE_TYPE_EIGEN(float, 4, 1, "Eigen__float41"_hs);
    SERIALIZE_TYPE_EIGEN(float, 4, 4, "Eigen__float44"_hs);
    SERIALIZE_TYPE_EIGEN(float,-1,-1, "Eigen__floatXX"_hs);
    SERIALIZE_TYPE_EIGEN(float, 3,-1, "Eigen__float3X"_hs);
    SERIALIZE_TYPE_EIGEN(float, 6,-1, "Eigen__float6X"_hs);

    SERIALIZE_TYPE_EIGEN(double, 2, 1, "Eigen__double21"_hs);
    SERIALIZE_TYPE_EIGEN(double, 3, 1, "Eigen__double31"_hs);
    SERIALIZE_TYPE_EIGEN(double, 3, 4, "Eigen__double34"_hs);
    SERIALIZE_TYPE_EIGEN(double, 3, 3, "Eigen__double33"_hs);
    SERIALIZE_TYPE_EIGEN(double,-1,-1, "Eigen__doubleXX"_hs);
    SERIALIZE_TYPE_EIGEN(double, 2,-1, "Eigen__double2X"_hs);
    SERIALIZE_TYPE_EIGEN(double, 3,-1, "Eigen__double3X"_hs);
    SERIALIZE_TYPE_EIGEN(double,-1, 3, "Eigen__doubleX3"_hs);
    SERIALIZE_TYPE_EIGEN(double, 4,-1, "Eigen__double4X"_hs);

    return 0;
}
} // namespace p3d::impl
