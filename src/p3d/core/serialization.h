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
static void _read(Type & v, std::size_t & id, cv::FileNode& node){
    std::string name = "p" + std::to_string(id);
    Type t;
    if (node[name].empty()) return;
    node[name] >> t;
    v = t;
}

static void _readBool(bool & v, std::size_t & id, cv::FileNode& node){
    std::string name = "p" + std::to_string(id);
    if (node[name].empty()) return;
    bool res = static_cast<int>(node[name]) != 0;
    v = res;
}

template <typename Type>
static void _writeVec(std::vector<Type> & f, std::size_t & id, cv::FileStorage& fs){
    fs << "p" + std::to_string(id) << "[";
    for (auto i = 0; i < f.size(); ++i) fs << f[i];
    fs << "]";
}

template <typename Type>
static void _readVec(std::vector<Type> & out, std::size_t & id, cv::FileNode& node){
    std::string name = "p" + std::to_string(id);
    std::vector<Type> temp;
    cv::FileNode n = node[name];
    if (node[name].empty()) return;
    if (n.type() != cv::FileNode::SEQ)
    {
        std::cout << "not a sequence";
        return;
    }

    for (auto it = n.begin(); it != n.end(); ++it) {
        Type t;
        *it >> t;
        temp.emplace_back(t);
    }

    out = temp;
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
static void _readVecS(std::vector<Type> & out, std::size_t & id, cv::FileNode& node){
    std::string name = "p" + std::to_string(id);
    std::vector<Type> temp;
    cv::FileNode n = node[name];
    if (node[name].empty()) return;
    if (n.type() != cv::FileNode::SEQ)
    {
        std::cout << "not a sequence";
        return;
    }

    for (auto it = n.begin(); it != n.end(); ++it) {
        Type t;
        t.read(*it);
        temp.emplace_back(t);
    }
    out = temp;
}

template<typename Type, int SizeX, int SizeY>
static void _writeEigen(const Eigen::Matrix<Type, SizeX, SizeY> & f, std::size_t & id, cv::FileStorage& fs){
    cv::Mat temp;
    cv::eigen2cv(f,temp);
    fs << "p" + std::to_string(id) << temp;
}

template<typename Type, int SizeX, int SizeY>
static void _readEigen(Eigen::Matrix<Type, SizeX, SizeY> & mat, std::size_t & id, cv::FileNode& node){
    std::string name = "p" + std::to_string(id);
    cv::Mat temp;
    if (node[name].empty()) return;
    node[name] >> temp;
    Eigen::Matrix<Type, SizeX, SizeY> out;
    if (temp.empty()) return;
    out.setZero(temp.rows,temp.cols);
    cv::cv2eigen(temp,out);
    mat = out;
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
                meta::any old = data.get(*ptr);
                meta::any any = old;
                func.invoke(any,any,data.id(),nodeL);
                data.set(*ptr,any);
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
