#pragma once

#include "p3d/core.h"
#include "p3d/logger.h"

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace p3d
{
namespace impl
{
#define SERIALIZE_TYPE(x, name)                 \
    entt::meta<x>()                             \
        .alias(name)                            \
        .func<&p3d::impl::_read<x>>("_read"_hs) \
        .func<&p3d::impl::_write<x>>("_write"_hs)

#define SERIALIZED_ADD_READ_WRITE(x)                  \
    entt::meta<x>()                                   \
        .alias(p3d::alias(x::classNameStatic()))      \
        .func<&p3d::impl::_readCustom<x>>("_read"_hs) \
        .func<&p3d::impl::_writeCustom<x>>("_write"_hs)

#define SERIALIZE_TYPE_VEC(x, name)                \
    entt::meta<std::vector<x>>()                   \
        .alias(name)                                \
        .func<&p3d::impl::_readVec<x>>("_read"_hs) \
        .func<&p3d::impl::_writeVec<x>>("_write"_hs)

#define SERIALIZE_TYPE_VECS(x, name)                \
    entt::meta<std::vector<x>>()                    \
        .alias(name)                                 \
        .func<&p3d::impl::_readVecS<x>>("_read"_hs) \
        .func<&p3d::impl::_writeVecS<x>>("_write"_hs)

#define SERIALIZE_TYPE_EIGEN(Scalar, x, y, name)                   \
    entt::meta<Eigen::Matrix<Scalar, x, y>>()                      \
        .alias(name)                                               \
        .func<&p3d::impl::_readEigen<Scalar, x, y>>("_read"_hs)    \
        .func<&p3d::impl::_writeEigen<Scalar, x, y>>("_write"_hs); \
    entt::meta<std::vector<Eigen::Matrix<Scalar, x, y>>>()         \
        .alias("vector_" name)                                     \
        .func<&p3d::impl::_readVecEigen<Scalar, x, y>>("_read"_hs) \
        .func<&p3d::impl::_writeVecEigen<Scalar, x, y>>("_write"_hs)

template <typename Type>
static void _write(cv::FileStorage& fs, P3D_ID_TYPE& id, Type& f)
{
    fs << "p" + std::to_string(id) << f;
}

template <typename Type>
static void _read(cv::FileNode& node, P3D_ID_TYPE& id, Type& v)
{
    std::string name = "p" + std::to_string(id);
    Type t;
    if (node[name].empty()) {
        return;
    }
    node[name] >> t;
    v = t;
}

template <typename Type>
static void _writeCustom(cv::FileStorage& fs, P3D_ID_TYPE& id, Type& f)
{
    f.write(fs);
}

template <typename Type>
static void _readCustom(cv::FileNode& node, P3D_ID_TYPE& id, Type& v)
{
    v.read(node);
}

static void _readBool(cv::FileNode& node, P3D_ID_TYPE& id, bool& v)
{
    std::string name = "p" + std::to_string(id);
    if (node[name].empty()) return;
    bool res = static_cast<int>(node[name]) != 0;
    v = res;
}

template <typename Type>
static void _writeVec(cv::FileStorage& fs, P3D_ID_TYPE& id, std::vector<Type>& f)
{
    fs << "p" + std::to_string(id) << "[";
    for (auto i = 0; i < f.size(); ++i) fs << f[i];
    fs << "]";
}

template <typename Type>
static void _readVec(cv::FileNode& node, P3D_ID_TYPE& id, std::vector<Type>& out)
{
    std::string name = "p" + std::to_string(id);
    std::vector<Type> temp;
    cv::FileNode n = node[name];
    if (node[name].empty()) return;
    if (n.type() != cv::FileNode::SEQ) {
        std::cout << "[static void _readVec]: not a sequence" << std::endl;
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
static void _writeVecS(cv::FileStorage& fs, P3D_ID_TYPE& id, std::vector<Type>& f)
{
    fs << "p" + std::to_string(id) << "[";
    for (auto i = 0; i < f.size(); ++i) {
        fs << "{";
        f[i].write(fs);
        fs << "}";
    }
    fs << "]";
}

template <typename Type>
static void _readVecS(cv::FileNode& node, P3D_ID_TYPE& id, std::vector<Type>& out)
{
    std::string name = "p" + std::to_string(id);
    std::vector<Type> temp;
    cv::FileNode n = node[name];
    if (node[name].empty()) return;
    if (n.type() != cv::FileNode::SEQ) {
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

template <typename Type, int SizeX, int SizeY>
static void _writeEigen(cv::FileStorage& fs, P3D_ID_TYPE& id, const Eigen::Matrix<Type, SizeX, SizeY>& f)
{
    cv::Mat temp;
    cv::eigen2cv(f, temp);
    fs << "p" + std::to_string(id) << temp;
}

template <typename Type, int SizeX, int SizeY>
static void _readEigen(cv::FileNode& node, P3D_ID_TYPE& id, Eigen::Matrix<Type, SizeX, SizeY>& mat)
{
    std::string name = "p" + std::to_string(id);
    cv::Mat temp;
    if (node[name].empty()) return;
    node[name] >> temp;
    Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> out;
    if (temp.empty()) return;
    out.setZero(temp.rows, temp.cols);

    cv::cv2eigen(temp, out);
    mat = out;
}

template <typename Type, int SizeX, int SizeY>
static void _writeVecEigen(cv::FileStorage& fs, P3D_ID_TYPE& id, std::vector<Eigen::Matrix<Type, SizeX, SizeY>>& f)
{
    fs << "p" + std::to_string(id) << "[";
    for (auto i = 0; i < f.size(); ++i) {
        fs << "{";
        cv::Mat temp;
        cv::eigen2cv(f[i], temp);
        fs << "M" << temp;
        fs << "}";
    }
    fs << "]";
}

template <typename Type, int SizeX, int SizeY>
static void _readVecEigen(cv::FileNode& node, P3D_ID_TYPE& id, std::vector<Eigen::Matrix<Type, SizeX, SizeY>>& out)
{
    std::string name = "p" + std::to_string(id);
    std::vector<Eigen::Matrix<Type, SizeX, SizeY>> temp;
    cv::FileNode n = node[name];
    if (node[name].empty()) return;
    if (n.type() != cv::FileNode::SEQ) {
        std::cout << "not a sequence";
        return;
    }

    for (auto it = n.begin(); it != n.end(); ++it) {
        cv::Mat t;
        (*it)["M"] >> t;
        Eigen::Matrix<Type, SizeX, SizeY> b;
        b.setZero();
        if (!t.empty()) {
            Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> a;
            cv::cv2eigen(t, a);
            b = a;
        }
        temp.emplace_back(b);
    }
    out = temp;
}

int P3D_API registerTypes();
}  // namespace impl

template <typename T>
class P3D_API Serializable : public PObject
{
public:
    Serializable() {}

    virtual ~Serializable() {}

    P3D_ID_TYPE getAlias() override { return p3d::alias(className()); }

    virtual const char* className() { return T::classNameStatic(); }

    auto resolve() { return entt::resolve<T>(); }

    void setData(const entt::meta_data& data, const entt::meta_any& value) override
    {
        auto ptr = dynamic_cast<T*>(this);
        data.set(*ptr, value);
    }

    entt::meta_any getData(const entt::meta_data& data) override
    {
        auto ptr = dynamic_cast<T*>(this);
        return data.get(*ptr);
    }

    virtual bool operator==(const T& i) const
    {
        T* lhs = dynamic_cast<T*>(const_cast<Serializable<T>*>(this));
        T* rhs = const_cast<T*>(&i);
        bool res = true;
        if (!entt::resolve<T>()) { LOG_DBG("%s: unknown type", className()); }

        entt::resolve<T>().data([&](auto data) {
            if (data.get(*lhs) != data.get(*rhs)) res = false;
        });
        return res && equalsAdditional();
    }

    virtual bool equalsAdditional() const { return true; }

    void write(cv::FileStorage& fs)
    {
        // **** first we write all registered properies

        std::string nodeName = "class" + std::to_string(entt::resolve<T>().id());
        fs << nodeName << "{";
        entt::resolve<T>().data([&](auto data) {
            auto func = data.type().func("_write"_hs);
            if (func) {
                auto ptr = dynamic_cast<T*>(this);
                entt::meta_any any = data.get(*ptr);
                func.invoke(any, std::ref(fs), P3D_ID_TYPE(data.alias()), any);
            } else {
                LOG_ERR("%s, not registered for write: %i", className(), data.alias());
            }
        });

        // additionally, we can write something that can't be in meta (such as
        // cv::Mat)
        writeAdditional(fs);
        fs << "}";
    }

    void read(const cv::FileNode& node)
    {
        // **** first we read all registered properies
        std::string nodeName = "class" + std::to_string(entt::resolve<T>().id());
        cv::FileNode nodeL = node[nodeName];

        entt::resolve<T>().data([&](auto data) {
            entt::meta_func func = data.type().func("_read"_hs);
            if (func) {
                auto ptr = dynamic_cast<T*>(this);
                entt::meta_any old = data.get(*ptr);
                entt::meta_any any(old);
                func.invoke(old, nodeL, P3D_ID_TYPE(data.alias()), *any);
                data.set(*ptr, any);
            } else {
                LOG_ERR("not registered for read: %i", data.alias());
            }
        });

        // additionally, we can read something that can't be in meta (such as
        // cv::Mat)
        readAdditional(nodeL);
    }

    virtual void writeAdditional(cv::FileStorage& fs) {}
    virtual void readAdditional(const cv::FileNode& node) {}
};
}  // namespace p3d
