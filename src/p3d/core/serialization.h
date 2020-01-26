#pragma once

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>


#include "p3d/core/core.h"

namespace impl {

template <typename Type>
class Serializable;

#define SERIALIZE_TYPE(x,name) \
    meta::reflect<x>(p3d_hashStr(name)) \
            .func<&impl::_read<x>>(p3d_hashStr("_read")) \
            .func<&impl::_write<x>>(p3d_hashStr("_write"))

#define SERIALIZE_TYPE_VEC(x,name) \
    meta::reflect<std::vector<x>>(p3d_hashStr(name)) \
            .func<&impl::_readVec<x>>(p3d_hashStr("_read")) \
            .func<&impl::_writeVec<x>>(p3d_hashStr("_write"))

#define SERIALIZE_TYPE_VECS(x,name) \
    meta::reflect<std::vector<x>>(p3d_hashStr(name)) \
            .func<&impl::_readVecS<x>>(p3d_hashStr("_read")) \
            .func<&impl::_writeVecS<x>>(p3d_hashStr("_write"))

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

template <typename Type>
static void _writeVecS(std::vector<Type> & f, std::size_t & id, cv::FileStorage& fs){
    fs << "p" + std::to_string(id) << "[";
    for (auto i = 0; i < f.size(); ++i) {
        fs << "{";
        f[i].write(fs);
        fs << "}";
    }
    fs << "]";
}

template <typename Type>
static std::vector<Type> _readVecS(std::size_t & id, cv::FileNode& node){
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
        t.read(*it);
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
    Eigen::Matrix<Type, SizeX, SizeY> out;
    if (temp.empty()) return out;
    out.setZero(temp.rows,temp.cols);
    cv::cv2eigen(temp,out);
    return out;
}


static int registerTypes();

}

template<typename T>
class Serializable{

public:
    Serializable(){
    }

    virtual ~Serializable(){}

    void write(cv::FileStorage& fs) {

        // **** first we write all registered properies
        std::string nodeName = "class" + std::to_string(meta::resolve<T>().id());
        fs << nodeName << "{";
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
        fs << "}";
    }

    void read(const cv::FileNode& node) {
        // **** first we read all registered properies
        std::string nodeName = "class" + std::to_string(meta::resolve<T>().id());
        cv::FileNode nodeL = node[nodeName];

        meta::resolve<T>().data([&](meta::data data) {
            meta::func func = data.type().func(p3d_hashStr("_read"));
            if (func) {
                auto ptr = dynamic_cast<T*>(this);
                meta::any any = data.get(*ptr);
                auto v = func.invoke(any,data.id(),nodeL);
                data.set(*ptr,v);
            } else {
                std::cout << "not registered for read: " << data.id() << std::endl;
            }
        });

        // **** additionally, we can read something that can't be in meta (such as cv::Mat)
        readAdditional(nodeL);
    }

    virtual void writeAdditional(cv::FileStorage& fs) {}
    virtual void readAdditional(const cv::FileNode& node) {}
};
