#pragma once

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "p3d/core/core.h"

namespace impl {

template <typename Type>
class Serializable;

#define SERIALIZE_TYPE(x,name) \
    entt::meta<x>() \
            .type(name) \
            .func<&impl::_read<x>>("_read"_hs) \
            .func<&impl::_write<x>>("_write"_hs)

#define SERIALIZE_TYPE_VEC(x,name) \
    entt::meta<std::vector<x>>() \
            .type(name) \
            .func<&impl::_readVec<x>>("_read"_hs) \
            .func<&impl::_writeVec<x>>("_write"_hs)

#define SERIALIZE_TYPE_VECS(x,name) \
    entt::meta<std::vector<x>>() \
            .type(name) \
            .func<&impl::_readVecS<x>>("_read"_hs) \
            .func<&impl::_writeVecS<x>>("_write"_hs)

#define SERIALIZE_TYPE_EIGEN(Scalar,x,y,name) \
    entt::meta<Eigen::Matrix<Scalar,x,y>>() \
            .type(name) \
            .func<&impl::_readEigen<Scalar,x,y>>("_read"_hs) \
            .func<&impl::_writeEigen<Scalar,x,y>>("_write"_hs)

template <typename Type>
static void _write(cv::FileStorage& fs, P3D_ID_TYPE & id, Type & f){
    fs << "p" + std::to_string(id) << f;
}

template <typename Type>
static void _read(cv::FileNode& node, P3D_ID_TYPE & id, Type & v){
    std::string name = "p" + std::to_string(id);
    Type t;
    if (node[name].empty()) {
        return;
    }
    node[name] >> t;
    v = t;
}

static void _readBool(cv::FileNode& node, P3D_ID_TYPE & id, bool & v){
    std::string name = "p" + std::to_string(id);
    if (node[name].empty()) return;
    bool res = static_cast<int>(node[name]) != 0;
    v = res;
}

template <typename Type>
static void _writeVec(cv::FileStorage& fs, P3D_ID_TYPE & id, std::vector<Type> & f){
    fs << "p" + std::to_string(id) << "[";
    for (auto i = 0; i < f.size(); ++i) fs << f[i];
    fs << "]";
}

template <typename Type>
static void _readVec(cv::FileNode& node, P3D_ID_TYPE & id, std::vector<Type> & out){
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
static void _writeVecS(cv::FileStorage& fs, P3D_ID_TYPE & id, std::vector<Type> & f){
    fs << "p" + std::to_string(id) << "[";
    for (auto i = 0; i < f.size(); ++i) {
        fs << "{";
        f[i].write(fs);
        fs << "}";
    }
    fs << "]";
}

template <typename Type>
static void _readVecS(cv::FileNode& node, P3D_ID_TYPE & id, std::vector<Type> & out){
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
static void _writeEigen(cv::FileStorage& fs, P3D_ID_TYPE & id, const Eigen::Matrix<Type, SizeX, SizeY> & f){
    cv::Mat temp;
    cv::eigen2cv(f,temp);
    fs << "p" + std::to_string(id) << temp;
}

template<typename Type, int SizeX, int SizeY>
static void _readEigen(cv::FileNode& node, P3D_ID_TYPE & id, Eigen::Matrix<Type, SizeX, SizeY> & mat){
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

        std::string nodeName = "class" + std::to_string(entt::resolve<T>().identifier());
        fs << nodeName << "{";
        entt::resolve<T>().data([&](entt::meta_data data) {
            entt::meta_func func = data.type().func("_write"_hs);
            if (func) {
                auto ptr = dynamic_cast<T*>(this);
                entt::meta_any any = data.get(*ptr);
                func.invoke(any,std::ref(fs),data.identifier(),any);
            } else {
                std::cout << "not registered for write: " << data.identifier() << std::endl;
            }
        });

        // **** additionally, we can write something that can't be in meta (such as cv::Mat)
        writeAdditional(fs);
        fs << "}";
    }

    void read(const cv::FileNode& node) {
        // **** first we read all registered properies
        std::string nodeName = "class" + std::to_string(entt::resolve<T>().identifier());
        cv::FileNode nodeL = node[nodeName];

        entt::resolve<T>().data([&](entt::meta_data data) {
            entt::meta_func func = data.type().func("_read"_hs);
            if (func) {
                auto ptr = dynamic_cast<T*>(this);
                entt::meta_any old = data.get(*ptr);
                entt::meta_any any = *old;
                func.invoke(ptr,nodeL,data.identifier(),any);
                data.set(*ptr,any);
            } else {
                std::cout << "not registered for read: " << data.identifier() << std::endl;
            }
        });

        // **** additionally, we can read something that can't be in meta (such as cv::Mat)
        readAdditional(nodeL);
    }

    virtual void writeAdditional(cv::FileStorage& fs) {}
    virtual void readAdditional(const cv::FileNode& node) {}
};
