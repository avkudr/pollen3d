#pragma once

#include "gtest/gtest.h"

#include "p3d/core/core.h"
#include "p3d/core/utils.h"
#include "p3d/core/serialization.h"
#include "p3d/data/project_data.h"
#include "p3d/project_manager.h"
#include "p3d/commands.h"

#ifndef P3D_PROJECT_EXTENSION
#define P3D_PROJECT_EXTENSION ".yml.gz"
#endif

class A{
public:
    A() {
        static bool firstCall = true;
        if (firstCall) {
            entt::meta<A>()
                .type(P3D_ID_TYPE(110520))
                .data<&A::setValue,&A::getValue>(P3D_ID_TYPE(110521));

            firstCall = false;
        }
    }
    int getValue() const { return m_value;}
    void setValue(int v) { m_value = v;}
    const cv::Mat & getMatrixCV() const {
        return m_matrixCV;
    }
    void setMatrixCV(const cv::Mat &matrixCV) {
        m_matrixCV = matrixCV.clone();
    }

private:
    int m_value = 5;
    cv::Mat m_matrixCV;
};


struct ClassA : public Serializable<ClassA>{

public:
    ClassA() {
        static bool first = true;
        if (first) {
            entt::meta<ClassA>()
                .type("ClassAdebug"_hs)
                .data<&ClassA::float1>(P3D_ID_TYPE(1))
                .data<&ClassA::float2>(P3D_ID_TYPE(2))
                .data<&ClassA::integer>(P3D_ID_TYPE(10))
                .data<&ClassA::str>(P3D_ID_TYPE(15))
                .data<&ClassA::vectorDouble>(P3D_ID_TYPE(20))
                .data<&ClassA::eigenMatrix>(P3D_ID_TYPE(25))
                .data<&ClassA::eigenMatrixDyn>(P3D_ID_TYPE(26));
            first = false;
        }
        eigenMatrix.setIdentity();
        eigenMatrixDyn.setIdentity(15,15);
    }
    ~ClassA(){

    }

    float float1 = 0.5;
    float float2 = 0.5;
    int integer = 24;
    std::string str = "hello";
    std::vector<double> vectorDouble = {0.0,0.2};
    Eigen::Matrix3f eigenMatrix;
    Eigen::MatrixXf eigenMatrixDyn;
};

#define EXPECT_TYPE_SERIALIZABLE(x) \
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
    EXPECT_TYPE_SERIALIZABLE(AffineCamera);

}

TEST(META, meta_nbReflectedDataMembers)
{
    ProjectData data;
    int cnt = 0;
    entt::resolve<ProjectData>().data([&](entt::meta_data data) {
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
    a.str = "pollen3d";
    a.vectorDouble = {0.65,5465};
    a.eigenMatrix.setOnes();
    a.eigenMatrixDyn.setOnes();

    {
        cv::FileStorage fs("test.yml", cv::FileStorage::WRITE);
        fs << "Node1" << "{";
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

}

#define SERIALIZATION_TEST(Type, X, Y) \
    { \
        auto id = entt::resolve<Type>().identifier(); \
        cv::FileStorage fs("test.xml", cv::FileStorage::WRITE); \
        fs << "node1" << "{"; \
        impl::_write<Type>(fs,id,X); \
        fs << "}"; \
        fs.release(); \
    } \
    { \
        cv::FileStorage fs("test.xml", cv::FileStorage::READ); \
        cv::FileNode node = fs["node1"]; \
        auto id = entt::resolve<Type>().identifier(); \
        impl::_read<Type>(node, id, Y); \
        fs.release(); \
    }


TEST(META, meta_serializeFloat)
{
    float m1 = 25.0;
    float m2;
    SERIALIZATION_TEST(float,m1,m2)
    ASSERT_FLOAT_EQ(m1, m2);
}

TEST(META, meta_serializeDouble)
{
    double m1 = 25.0;
    double m2;
    SERIALIZATION_TEST(double,m1,m2)
    ASSERT_FLOAT_EQ(m1, m2);
}

TEST(META, meta_serializeInt)
{
    int m1 = 25;
    int m2;
    SERIALIZATION_TEST(int,m1,m2)
    ASSERT_FLOAT_EQ(m1, m2);
}

TEST(META, meta_serializeEigen)
{
    using Matrix = Eigen::Matrix<float,-1,-1>;
    Matrix m1;
    m1.setOnes(15,15);
    Matrix m2;

    {
        auto id = entt::resolve<Matrix>().identifier();
        cv::FileStorage fs("test.xml", cv::FileStorage::WRITE);
        fs << "node1" << "{";
        impl::_writeEigen<float,-1,-1>(fs, id, m1);
        fs << "}";
        fs.release();
    }
    {
        cv::FileStorage fs("test.xml", cv::FileStorage::READ);
        cv::FileNode node = fs["node1"];
        auto id = entt::resolve<Matrix>().identifier();
        impl::_readEigen<float,-1,-1>(node, id, m2);
        fs.release();
    }
    EXPECT_EQ(m1.size(), m2.size());
    for (int i = 0; i < m1.rows(); ++i) {
        for (int j = 0; j < m1.cols(); ++j) {
            EXPECT_FLOAT_EQ(m1(i,j),m2(i,j));
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
        fs << "AffineCamera" << "{";
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
        fs << "Image" << "{";
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
    std::cout << imRead.getCamera().getA() << std::endl;
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
        auto id = entt::resolve<decltype(m1)>().identifier();
        cv::FileStorage fs("test.xml", cv::FileStorage::WRITE);
        fs << "node1" << "{";
        impl::_writeVec<float>(fs,id,m1);
        fs << "}";
        fs.release();
    }

    decltype(m1) m2;

    {
        cv::FileStorage fs("test.xml", cv::FileStorage::READ);
        cv::FileNode node = fs["node1"];
        auto id = entt::resolve<decltype(m1)>().identifier();
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
    m1.emplace_back(Match(0,1,0.5));
    m1.emplace_back(Match(1,2));
    m1.emplace_back(Match(2,3));

    {
        auto id = entt::resolve<std::vector<Match>>().identifier();
        cv::FileStorage fs("test.xml", cv::FileStorage::WRITE);
        fs << "node1" << "{";
        impl::_writeVecS<Match>(fs,id,m1);
        fs << "}";
        fs.release();
    }

    std::vector<Match> m2;

    {
        cv::FileStorage fs("test.xml", cv::FileStorage::READ);
        cv::FileNode node = fs["node1"];
        auto id = entt::resolve<std::vector<Match>>().identifier();
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
    ProjectData data1;
    std::string path = "test" + std::string(P3D_PROJECT_EXTENSION);
    data1.setProjectPath(path);
    ProjectManager::get()->saveProject(&data1,path);

    ProjectData data2;
    ProjectManager::get()->openProject(&data2,path);

    entt::resolve<ProjectData>().data([&](entt::meta_data data){
        EXPECT_EQ(data.get(data1).operator bool(), data.get(data2).operator bool()); // both exist
        EXPECT_EQ(data.get(data1).type()         , data.get(data2).type()); // type is the same
        EXPECT_EQ(data.get(data1)                , data.get(data2)); // type is the same
    });
}

TEST(META, meta_noDoubleReflect)
{
    ProjectData data;
    ProjectData data2;
    ASSERT_EQ(1,1);
}

TEST(COMMANDS, command_setProperty)
{
    A a;
    a.setValue(15);
    EXPECT_EQ(a.getValue(),15);
    CommandManager::get()->executeCommand(new CommandSetProperty(&a,110521,254));
    EXPECT_EQ(a.getValue(),254);
    CommandManager::get()->undoCommand();
    EXPECT_EQ(a.getValue(),15);
}

TEST(COMMANDS, command_setPropertyGroup)
{
    A a;
    a.setValue(15);
    EXPECT_EQ(a.getValue(),15);
    CommandGroup * grp = new CommandGroup();
    grp->add(new CommandSetProperty(&a,110521,20));
    grp->add(new CommandSetProperty(&a,110521,21));
    grp->add(new CommandSetProperty(&a,110521,22));
    grp->add(new CommandSetProperty(&a,110521,23));
    CommandManager::get()->executeCommand(grp);
    EXPECT_EQ(a.getValue(),23);
    CommandManager::get()->undoCommand();
    EXPECT_EQ(a.getValue(),15);
}

TEST(COMMANDS, command_setPropertyCV)
{
    using namespace std::placeholders;

    A a;
    cv::Mat B = (cv::Mat_<double>(3,3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    cv::Mat C = (cv::Mat_<double>(3,3) << 15,14,13,12,11,10,9,8,7);
    a.setMatrixCV(B);

    EXPECT_TRUE(utils::equalsCvMat(a.getMatrixCV(),B));
    CommandManager::get()->executeCommand(new CommandSetPropertyCV(&a,&A::setMatrixCV,&A::getMatrixCV,C));
    EXPECT_TRUE(utils::equalsCvMat(a.getMatrixCV(),C));
    CommandManager::get()->undoCommand();
    EXPECT_TRUE(utils::equalsCvMat(a.getMatrixCV(),B));
}

TEST(META, meta_setSettings)
{
    float f  = ProjectManager::get()->getSetting(p3dSetting_matcherFilterCoef).cast<float>();
    ProjectManager::get()->setSetting(p3dSetting_matcherFilterCoef, 2.0f*f);
    float f2 = ProjectManager::get()->getSetting(p3dSetting_matcherFilterCoef).cast<float>();
    EXPECT_FLOAT_EQ(f2,2.0f*f);
}


TEST(META, meta_setValue)
{
    auto identifier = P3D_ID_TYPE(1002);
    ProjectSettings s;
    auto data = entt::resolve<ProjectSettings>().data(identifier);
    entt::meta_any v = data.get(s); // default value is 0.3f
    data.set(s,v);                  EXPECT_FLOAT_EQ(data.get(s).cast<float>(), 0.3f);    // ok
    data.set(s,2.0f);               EXPECT_FLOAT_EQ(data.get(s).cast<float>(), 2.0f);    // ok
    data.set(s,2.0f * 2.0f);        EXPECT_FLOAT_EQ(data.get(s).cast<float>(), 4.0f);    // ok

    CommandManager::get()->executeCommand(new CommandSetProperty(&s,identifier,v));
    EXPECT_FLOAT_EQ(data.get(s).cast<float>(), 0.3f);
    CommandManager::get()->executeCommand(new CommandSetProperty(&s,identifier,2.0f));
    EXPECT_FLOAT_EQ(data.get(s).cast<float>(), 2.0f);
    CommandManager::get()->executeCommand(new CommandSetProperty(&s,identifier,2.0f * 2.0f));
    EXPECT_FLOAT_EQ(data.get(s).cast<float>(), 4.0f);

//    foo(&s,identifier,2.0f);
//    foo(&s,identifier,2.0f * 1.0f);
}

struct B{
    B() {
        entt::meta<B>()
            .type("ClassB"_hs)
            .data<&B::value>("value"_hs);
    }
    int value = 5;
};

struct Dummy{
    entt::meta_any value{};
};

TEST(META, meta_setValueClassAttribute)
{
    Dummy d;
    B b;
    auto data = entt::resolve<B>().data("value"_hs);
    d.value = data.get(b);

    EXPECT_EQ(d.value,b.value);
}
