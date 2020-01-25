#pragma once

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>


#include "p3d/core/core.h"

namespace impl {

#define SERIALIZE_TYPE(x,name) \
    meta::reflect<x>(p3d_hashStr(name)) \
            .func<&impl::_read<x>>(p3d_hashStr("_read")) \
            .func<&impl::_write<x>>(p3d_hashStr("_write"))

#define SERIALIZE_TYPE_VEC(x,name) \
    meta::reflect<std::vector<x>>(p3d_hashStr(name)) \
            .func<&impl::_readVec<x>>(p3d_hashStr("_read")) \
            .func<&impl::_writeVec<x>>(p3d_hashStr("_write"))

#define SERIALIZE_TYPE_EIGEN(type,x,y,name) \
    meta::reflect<Eigen::Matrix<type,x,y>>(p3d_hashStr(name)) \
            .func<&impl::_readEigen<type,x,y>>(p3d_hashStr("_read")) \
            .func<&impl::_writeEigen<type,x,y>>(p3d_hashStr("_write"))

template <typename Type>
static void _write(Type & f, std::size_t & id, cv::FileStorage& fs){
    fs << "p" + std::to_string(id) << f;
}

template <typename Type>
static Type _read(std::size_t & id, cv::FileNode& node){
    std::string name = "p" + std::to_string(id);
    Type t;
    node[name] >> t;
    return t;
}

static bool _readBool(std::size_t & id, cv::FileNode& node){
    std::string name = "p" + std::to_string(id);
    return static_cast<int>(node[name]) != 0;
}

template <typename Type>
static void _writeVec(std::vector<Type> & f, std::size_t & id, cv::FileStorage& fs){
    fs << "p" + std::to_string(id) << "[";
    for (auto i = 0; i < f.size(); ++i) fs << f[i];
    fs << "]";
}

template <typename Type>
static std::vector<Type> _readVec(std::size_t & id, cv::FileNode& node){
    std::string name = "p" + std::to_string(id);
    std::vector<Type> out;
    cv::FileNode n = node[name];
    if (n.type() != cv::FileNode::SEQ)
    {
        std::cout << "not a sequence";
        return out;
    }

    for (auto it = n.begin(); it != n.end(); ++it) {
        Type t;
        *it >> t;
        out.emplace_back(t);
    }
    return out;
}

template<typename Type, int SizeX, int SizeY>
static void _writeEigen(const Eigen::Matrix<Type, SizeX, SizeY> & f, std::size_t & id, cv::FileStorage& fs){
    cv::Mat temp;
    cv::eigen2cv(f,temp);
    fs << "p" + std::to_string(id) << temp;
}

template<typename Type, int SizeX, int SizeY>
static Eigen::Matrix<Type, SizeX, SizeY> _readEigen(std::size_t & id, cv::FileNode& node){
    std::string name = "p" + std::to_string(id);
    cv::Mat temp;
    node[name] >> temp;
    Eigen::MatrixXf out;
    out.setZero(temp.rows,temp.cols);
    cv::cv2eigen(temp,out);
    return out;
}


[[maybe_unused]]
static void registerTypes() {
    static bool notRegisteredYet = 1;
    if (notRegisteredYet) {
        std::cout << "register MetaTypes: " << notRegisteredYet << std::endl;
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

        SERIALIZE_TYPE_EIGEN( float,3,3, "Eigen__float33");
        SERIALIZE_TYPE_EIGEN( float,3,4, "Eigen__float34");
        SERIALIZE_TYPE_EIGEN( float,-1,-1, "Eigen__floatXX");

//        SERIALIZE_TYPE_EIGEN(double,3,3, "Eigen_double33");
//        SERIALIZE_TYPE_EIGEN(double,3,4, "Eigen_double34");

        notRegisteredYet = false;
    }
}

}

template<typename T>
class Serializable{

public:
    Serializable(){
    }

    virtual ~Serializable(){}

    void write(cv::FileStorage& fs) {

        // **** first we write all registered properies
        meta::resolve<T>().data([&](meta::data data) {
            meta::func func = data.type().func(p3d_hashStr("_write"));
            if (func) {
                auto ptr = dynamic_cast<T*>(this);
                meta::any any = data.get(*ptr);
                func.invoke(any,any,data.id(),fs);
            } else {
                std::cout << "not registered for write: " << data.id() << std::endl;
            }
        });

        // **** additionally, we can write something that can't be in meta (such as cv::Mat)
        writeAdditional(fs);
    }

    void read(const cv::FileNode& node) {
        // **** first we read all registered properies
        meta::resolve<T>().data([&](meta::data data) {
            meta::func func = data.type().func(p3d_hashStr("_read"));
            if (func) {
                auto ptr = dynamic_cast<T*>(this);
                meta::any any = data.get(*ptr);
                auto v = func.invoke(any,data.id(),const_cast<cv::FileNode&>(node));
                data.set(*ptr,v);
            } else {
                std::cout << "not registered for read: " << data.id() << std::endl;
            }
        });

        // **** additionally, we can read something that can't be in meta (such as cv::Mat)
        readAdditional(node);
    }

    virtual void writeAdditional(cv::FileStorage& fs) {}
    virtual void readAdditional(const cv::FileNode& node) {}
};
