#pragma once

#include "gtest/gtest.h"

#include "p3d/commands.h"
#include "p3d/core.h"
#include "p3d/project.h"
#include "p3d/serialization.h"
#include "p3d/utils.h"

#include "p3d/tasks.h"
#include "p3d/command_manager.h"

#ifndef P3D_PROJECT_EXTENSION
#define P3D_PROJECT_EXTENSION ".yml.gz"
#endif

#include "entt/entt.hpp"

using namespace p3d;

#define __ID_PROP_INT 110521
#define __ID_PROP_EIGEN 110581

class A : public Serializable<A>
{
public:
    A()
    {
        static bool firstCall = true;
        if (firstCall) {
            entt::meta<A>()
                .alias(getAlias())
                .data<&A::setValue, &A::getValue>(P3D_ID_TYPE(__ID_PROP_INT))
                .data<&A::setEigenMat, &A::getEigenMat>(P3D_ID_TYPE(__ID_PROP_EIGEN));

            SERIALIZE_TYPE_MAPS(A);
            firstCall = false;
        }
    }

    static const char *classNameStatic() { return "A"; }

    int getValue() const { return m_value; }
    void setValue(int v) { m_value = v; }
    const cv::Mat &getMatrixCV() const
    {
        return m_matrixCV;
    }
    void setMatrixCV(const cv::Mat &matrixCV)
    {
        m_matrixCV = matrixCV.clone();
    }

    const Eigen::Matrix<double, -1, -1> &getEigenMat() const
    {
        return m_eigMat;
    }
    void setEigenMat(const Eigen::Matrix<double, -1, -1> &m)
    {
        m_eigMat = m;
    }

private:
    int m_value = 5;
    cv::Mat m_matrixCV;

    Eigen::Matrix<double, -1, -1> m_eigMat;
};

struct ClassA : public p3d::Serializable<ClassA> {
public:
    ClassA()
    {
        static bool first = true;
        if (first) {
            entt::meta<ClassA>()
                .alias(getAlias())
                .data<&ClassA::float1>(P3D_ID_TYPE(1))
                .data<&ClassA::float2>(P3D_ID_TYPE(2))
                .data<&ClassA::integer>(P3D_ID_TYPE(10))
                .data<&ClassA::str>(P3D_ID_TYPE(15))
                .data<&ClassA::vectorDouble>(P3D_ID_TYPE(20))
                .data<&ClassA::eigenMatrix>(P3D_ID_TYPE(25))
                .data<&ClassA::eigenMatrixDyn>(P3D_ID_TYPE(26))
                .data<&ClassA::eigenVec>(P3D_ID_TYPE(27))
                .data<&ClassA::map>(P3D_ID_TYPE(28));
            first = false;
        }
        eigenMatrix.setIdentity();
        eigenMatrixDyn.setIdentity(15, 15);
    }
    ~ClassA()
    {
    }

    static const char *classNameStatic() { return "ClassA"; }

    bool operator==(const ClassA &i) const { return false; }

    float float1 = 0.5;
    float float2 = 0.5;
    int integer = 24;
    std::string str = "hello";
    std::vector<double> vectorDouble = {0.0, 0.2};
    Eigen::Matrix3f eigenMatrix;
    Eigen::MatrixXf eigenMatrixDyn;
    std::vector<p3d::Vec2> eigenVec;
    std::map<int, A> map;
};

#define EXPECT_TYPE_SERIALIZABLE(x)                   \
    EXPECT_TRUE(entt::resolve<x>().func("_read"_hs)); \
    EXPECT_TRUE(entt::resolve<x>().func("_write"_hs))

TEST(META, meta_serializedTypes)
{
    EXPECT_TYPE_SERIALIZABLE(float);
    EXPECT_TYPE_SERIALIZABLE(std::string);
    EXPECT_TYPE_SERIALIZABLE(int);
    EXPECT_TYPE_SERIALIZABLE(double);
    EXPECT_TYPE_SERIALIZABLE(bool);

    EXPECT_TYPE_SERIALIZABLE(std::vector<float>);
    EXPECT_TYPE_SERIALIZABLE(std::vector<std::string>);
    EXPECT_TYPE_SERIALIZABLE(std::vector<int>);
    EXPECT_TYPE_SERIALIZABLE(std::vector<double>);

    EXPECT_TYPE_SERIALIZABLE(Eigen::MatrixXf);
    EXPECT_TYPE_SERIALIZABLE(p3d::AffineCamera);
}

TEST(META, meta_nbReflectedDataMembers)
{
    p3d::Project data;
    int cnt = 0;
    entt::resolve<p3d::Project>().data([&](entt::meta_data data) {
        (void)data;
        cnt++;
    });
    EXPECT_TRUE(cnt > 0);
}

TEST(META, meta_serializeClass)
{
    ClassA a;
    ClassA b;
    a.float1 = 8.10f;
    a.float2 = 26.04f;
    a.integer = 5465465;
    a.str = "pollen3d";
    a.vectorDouble = {0.65, 5465};
    a.eigenMatrix.setOnes();
    a.eigenMatrixDyn.setOnes();
    a.eigenVec = {p3d::Vec2(0, 0), p3d::Vec2(1, 1), p3d::Vec2(4, 2)};
    a.map.insert({0, A()});
    a.map.insert({1, A()});

    {
        cv::FileStorage fs("test.yml", cv::FileStorage::WRITE);
        fs << "Node1"
           << "{";
        a.write(fs);
        fs << "}";
        fs.release();
    }
    {
        cv::FileStorage fs("test.yml", cv::FileStorage::READ);
        b.read(fs["Node1"]);
        fs.release();
    }

    EXPECT_EQ(a.float1, b.float1);
    EXPECT_EQ(a.float2, b.float2);
    EXPECT_EQ(a.str, b.str);
    EXPECT_EQ(a.integer, b.integer);
    EXPECT_EQ(a.vectorDouble, b.vectorDouble);
    EXPECT_EQ(a.eigenMatrix, b.eigenMatrix);
    EXPECT_EQ(a.eigenMatrixDyn, b.eigenMatrixDyn);
    EXPECT_EQ(a.eigenVec, b.eigenVec);
    EXPECT_EQ(a.map, b.map);
}

#define SERIALIZATION_TEST(Type, X, Y)                          \
    {                                                           \
        auto id = entt::resolve<Type>().id();                   \
        cv::FileStorage fs("test.xml", cv::FileStorage::WRITE); \
        fs << "node1"                                           \
           << "{";                                              \
        p3d::impl::_write<Type>(fs, id, X);                     \
        fs << "}";                                              \
        fs.release();                                           \
    }                                                           \
    {                                                           \
        cv::FileStorage fs("test.xml", cv::FileStorage::READ);  \
        cv::FileNode node = fs["node1"];                        \
        auto id = entt::resolve<Type>().id();                   \
        p3d::impl::_read<Type>(node, id, Y);                    \
        fs.release();                                           \
    }

TEST(META, meta_serializeFloat)
{
    float m1 = 25.0;
    float m2;
    SERIALIZATION_TEST(float, m1, m2)
    ASSERT_FLOAT_EQ(m1, m2);
}

TEST(META, meta_serializeDouble)
{
    double m1 = 25.0;
    double m2;
    SERIALIZATION_TEST(double, m1, m2)
    ASSERT_FLOAT_EQ(m1, m2);
}

TEST(META, meta_serializeInt)
{
    int m1 = 25;
    int m2;
    SERIALIZATION_TEST(int, m1, m2)
    ASSERT_FLOAT_EQ(m1, m2);
}

TEST(META, meta_serializeEigen)
{
    using Matrix = Eigen::Matrix<float, -1, -1>;
    Matrix m1;
    m1.setOnes(15, 15);
    Matrix m2;

    {
        auto id = entt::resolve<Matrix>().id();
        cv::FileStorage fs("test.xml", cv::FileStorage::WRITE);
        fs << "node1"
           << "{";
        impl::_writeEigen<float, -1, -1>(fs, id, m1);
        fs << "}";
        fs.release();
    }
    {
        cv::FileStorage fs("test.xml", cv::FileStorage::READ);
        cv::FileNode node = fs["node1"];
        auto id = entt::resolve<Matrix>().id();
        impl::_readEigen<float, -1, -1>(node, id, m2);
        fs.release();
    }
    EXPECT_EQ(m1.size(), m2.size());
    for (int i = 0; i < m1.rows(); ++i) {
        for (int j = 0; j < m1.cols(); ++j) {
            EXPECT_FLOAT_EQ(m1(i, j), m2(i, j));
        }
    }
}

TEST(META, meta_serializeAffineCamera)
{
    AffineCamera cSave;
    cSave.setAlpha(1.1);
    cSave.setFocal(2.0);
    cSave.setSkew(-0.005);

    {
        cv::FileStorage fs("test.yml", cv::FileStorage::WRITE);
        fs << "AffineCamera"
           << "{";
        cSave.write(fs);
        fs << "}";
        fs.release();
    }

    AffineCamera cRead;
    {
        cv::FileStorage fs("test.yml", cv::FileStorage::READ);
        cRead.read(fs["AffineCamera"]);
        fs.release();
    }
    EXPECT_EQ(cSave, cRead);
}

TEST(META, meta_serializeImage)
{
    AffineCamera c;
    c.setAlpha(1.1);
    c.setFocal(2.0);
    c.setSkew(-0.005);

    Image imSave;
    imSave.setCamera(c);
    imSave.setPath("dummyPath");

    {
        cv::FileStorage fs("testSaveImage.yml", cv::FileStorage::WRITE);
        fs << "Image"
           << "{";
        imSave.write(fs);
        fs << "}";
        fs.release();
    }

    Image imRead;
    {
        cv::FileStorage fs("testSaveImage.yml", cv::FileStorage::READ);
        imRead.read(fs["Image"]);
        fs.release();
    }
    EXPECT_EQ(imSave.getPath(), imRead.getPath());
    EXPECT_EQ(imSave.getCamera(), imRead.getCamera());
}

TEST(META, meta_serializeVecFloat)
{
    std::vector<float> m1;
    m1.emplace_back(0);
    m1.emplace_back(1);
    m1.emplace_back(2);

    {
        auto id = entt::resolve<decltype(m1)>().id();
        cv::FileStorage fs("test.xml", cv::FileStorage::WRITE);
        fs << "node1"
           << "{";
        impl::_writeVec<float>(fs, id, m1);
        fs << "}";
        fs.release();
    }

    decltype(m1) m2;

    {
        cv::FileStorage fs("test.xml", cv::FileStorage::READ);
        cv::FileNode node = fs["node1"];
        auto id = entt::resolve<decltype(m1)>().id();
        impl::_readVec<float>(node, id, m2);
        fs.release();
    }

    EXPECT_EQ(m1.size(), m2.size());
    for (int i = 0; i < m2.size(); i++)
        EXPECT_FLOAT_EQ(m1[i], m2[i]);
}

TEST(META, meta_serializeVecMatches)
{
    std::vector<Match> m1;
    m1.emplace_back(Match(0, 1, 0.5));
    m1.emplace_back(Match(1, 2));
    m1.emplace_back(Match(2, 3));

    {
        auto id = entt::resolve<std::vector<Match>>().id();
        cv::FileStorage fs("test.xml", cv::FileStorage::WRITE);
        fs << "node1"
           << "{";
        impl::_writeVecS<Match>(fs, id, m1);
        fs << "}";
        fs.release();
    }

    std::vector<Match> m2;

    {
        cv::FileStorage fs("test.xml", cv::FileStorage::READ);
        cv::FileNode node = fs["node1"];
        auto id = entt::resolve<std::vector<Match>>().id();
        impl::_readVecS<Match>(node, id, m2);
        fs.release();
    }

    EXPECT_EQ(m1.size(), m2.size());
    for (int i = 0; i < m2.size(); i++) {
        EXPECT_EQ(m1[i].iPtL, m2[i].iPtL);
        EXPECT_EQ(m1[i].iPtR, m2[i].iPtR);
        EXPECT_FLOAT_EQ(m1[i].distance, m2[i].distance);
    }
}

TEST(META, meta_serializeProject)
{
    Project data1;
    std::string path = "test" + std::string(P3D_PROJECT_EXTENSION);
    data1.setProjectPath(path);
    p3d::saveProject(data1, path);

    Project data2;
    p3d::openProject(data2, path);

    entt::resolve<Project>().data([&](entt::meta_data data) {
        EXPECT_EQ(data.get(data1).operator bool(), data.get(data2).operator bool());  // both exist
        EXPECT_EQ(data.get(data1).type(), data.get(data2).type());                    // type is the same
        EXPECT_EQ(data.get(data1), data.get(data2));                                  // type is the same
    });
}

TEST(META, meta_noDoubleReflect)
{
    Project data;
    Project data2;
    ASSERT_EQ(1, 1);
}

TEST(COMMANDS, command_setProperty)
{
    A a;
    a.setValue(15);
    EXPECT_EQ(a.getValue(), 15);
    p3d::cmder::executeCommand(new CommandSetProperty(&a, __ID_PROP_INT, 254));
    EXPECT_EQ(a.getValue(), 254);
    p3d::cmder::get()->undoCommand();
    EXPECT_EQ(a.getValue(), 15);
}

TEST(COMMANDS, command_setPropertyEigenDyn)
{
    Mat m;
    m.setIdentity(2, 2);
    Mat m1;
    m1.setZero(16, 4);

    A a;
    a.setEigenMat(m);
    EXPECT_EQ(a.getEigenMat(), m);
    p3d::cmder::executeCommand(new CommandSetProperty(&a, __ID_PROP_EIGEN, m1));
    EXPECT_EQ(a.getEigenMat(), m1);
    p3d::cmder::get()->undoCommand();
    EXPECT_EQ(a.getEigenMat(), m);
}

TEST(COMMANDS, command_setPropertyGroup)
{
    A a;
    a.setValue(15);
    EXPECT_EQ(a.getValue(), 15);
    CommandGroup *grp = new CommandGroup();
    grp->add(new CommandSetProperty(&a, __ID_PROP_INT, 20));
    grp->add(new CommandSetProperty(&a, __ID_PROP_INT, 21));
    grp->add(new CommandSetProperty(&a, __ID_PROP_INT, 22));
    grp->add(new CommandSetProperty(&a, __ID_PROP_INT, 23));
    p3d::cmder::executeCommand(grp);
    EXPECT_EQ(a.getValue(), 23);
    p3d::cmder::get()->undoCommand();
    EXPECT_EQ(a.getValue(), 15);
}

TEST(COMMANDS, command_setPropertyCV)
{
    using namespace std::placeholders;

    A a;
    cv::Mat B = (cv::Mat_<double>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    cv::Mat C = (cv::Mat_<double>(3, 3) << 15, 14, 13, 12, 11, 10, 9, 8, 7);
    a.setMatrixCV(B);

    EXPECT_TRUE(utils::equalsCvMat(a.getMatrixCV(), B));
    p3d::cmder::executeCommand(
        new CommandSetPropertyCV{&a, &A::setMatrixCV, &A::getMatrixCV, C});
    EXPECT_TRUE(utils::equalsCvMat(a.getMatrixCV(), C));
    p3d::cmder::get()->undoCommand();
    EXPECT_TRUE(utils::equalsCvMat(a.getMatrixCV(), B));
}

TEST(COMMANDS, command_mergeNextCmd)
{
    A a;
    a.setValue(15);
    a.setEigenMat(Mat(Mat3::Identity()));

    p3d::cmder::executeCommand(new CommandSetProperty(&a, __ID_PROP_INT, 254));
    p3d::cmder::get()->mergeNextCommand();
    p3d::cmder::executeCommand(
        new CommandSetProperty(&a, __ID_PROP_EIGEN, Mat(Mat4::Zero())));

    EXPECT_EQ(a.getValue(), 254);
    EXPECT_EQ(a.getEigenMat(), Mat(Mat4::Zero()));
    p3d::cmder::get()->undoCommand();
    EXPECT_EQ(a.getValue(), 15);
    EXPECT_EQ(a.getEigenMat(), Mat(Mat3::Identity()));
}

TEST(META, meta_setValue)
{
    P3D_ID_TYPE id = p3dAffineCamera_Alpha;
    AffineCamera s;
    auto data = entt::resolve<AffineCamera>().data(id);
    entt::meta_any v = data.get(s);  // default value is 0.3f
    data.set(s, 2.0);
    EXPECT_DOUBLE_EQ(data.get(s).cast<double>(), 2.0);  // ok
    data.set(s, 2.0 * 2.0);
    EXPECT_DOUBLE_EQ(data.get(s).cast<double>(), 4.0);  // ok

    p3d::cmder::executeCommand(new CommandSetProperty(&s, id, 0.001));
    EXPECT_DOUBLE_EQ(data.get(s).cast<double>(), 0.001);
    p3d::cmder::executeCommand(new CommandSetProperty(&s, id, 2.0));
    EXPECT_DOUBLE_EQ(data.get(s).cast<double>(), 2.0);
    p3d::cmder::executeCommand(new CommandSetProperty(&s, id, 2.0 * 2.0));
    EXPECT_DOUBLE_EQ(data.get(s).cast<double>(), 4.0);
}

struct B {
    B() { entt::meta<B>().alias("ClassB"_hs).data<&B::value>("value"_hs); }
    int value = 5;
};

struct Dummy {
    entt::meta_any value{};
};

TEST(META, meta_setValueClassAttribute)
{
    Dummy d;
    B b;
    auto data = entt::resolve<B>().data("value"_hs);
    d.value = data.get(b);

    EXPECT_EQ(d.value, b.value);
}

TEST(META, meta_compare)
{
    MatchingPars p1;
    MatchingPars p2;

    EXPECT_TRUE(p1 == p2);

    p2.filterCoeff *= 2;

    EXPECT_FALSE(p1 == p2);
}
