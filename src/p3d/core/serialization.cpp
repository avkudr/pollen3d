#include "serialization.h"

namespace impl{

int registerTypes() {
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: primitives" << std::endl;
        SERIALIZE_TYPE(float,"float");
        SERIALIZE_TYPE(double,"double");
        SERIALIZE_TYPE(int,"int");
        SERIALIZE_TYPE(std::string,"std::string");

        meta::reflect<bool>(p3d_hashStr("bool"))
                .func<&impl::_readBool>(p3d_hashStr("_read"))
                .func<&impl::_write<bool>>(p3d_hashStr("_write"));

        /*
         * not working :'(
        meta::reflect<cv::Mat>(p3d_hashStr("bool"))
                .func<&impl::_readCvMat>(p3d_hashStr("_read"))
                .func<&impl::_write<cv::Mat>>(p3d_hashStr("_write"));
        */

        SERIALIZE_TYPE_VEC(float,"vector_float");
        SERIALIZE_TYPE_VEC(double,"vector_double");
        SERIALIZE_TYPE_VEC(int,"vector_int");
        SERIALIZE_TYPE_VEC(std::string,"vector_std::string");

        SERIALIZE_TYPE_EIGEN(  float, 3, 3, "Eigen__float33");
        SERIALIZE_TYPE_EIGEN(  float, 3, 4, "Eigen__float34");
        SERIALIZE_TYPE_EIGEN(  float,-1,-1, "Eigen__floatXX");

        SERIALIZE_TYPE_EIGEN( double,3,3, "Eigen_double33");
        //        SERIALIZE_TYPE_EIGEN(double,3,4, "Eigen_double34");

        firstCall = false;
    }
    return firstCall;
}

int dummyPrimitives_ = registerTypes();

}
