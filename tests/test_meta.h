#pragma once

#include "gtest/gtest.h"

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
        meta::reflect<A>(p3d_hash(110520))
            .data<&A::setValue,&A::getValue>(p3d_hash(110521));
    }
    int getValue() const { return m_value;}
    void setValue(int v) { m_value = v;}
private:
    int m_value = 5;
};


struct ClassA : public Serializable<ClassA>{

public:
    ClassA() {
        static bool first = true;
        if (first) {
            meta::reflect<ClassA>(p3d_hashStr("ClassAdebug"))
                .data<&ClassA::a>(std::size_t(1))
                .data<&ClassA::a2>(std::size_t(2))
                .data<&ClassA::b>(std::size_t(10))
                .data<&ClassA::s>(std::size_t(15))
                .data<&ClassA::vd>(std::size_t(20))
                .data<&ClassA::m>(std::size_t(25))
                .data<&ClassA::mx>(std::size_t(26));
            first = false;
        }
        m.setIdentity();
        mx.setIdentity(15,15);
    }
    ~ClassA(){

    }

    float a = 0.5;
    float a2 = 0.5;
    int b = 24;
    std::string s = "hello";
    std::vector<double> vd = {0.0,0.2};
    Eigen::Matrix3f m;
    Eigen::MatrixXf mx;
};

#define EXPECT_TYPE_SERIALIZABLE(x) \
    EXPECT_TRUE(meta::resolve<x>().func(p3d_hashStr("_read"))); \
    EXPECT_TRUE(meta::resolve<x>().func(p3d_hashStr("_write")))

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

}

TEST(META, meta_nbReflectedDataMembers)
{
    ProjectData data;
    int cnt = 0;
    meta::resolve<ProjectData>().data([&](meta::data data) {
        (void)data;
        cnt++;
    });
    EXPECT_TRUE(cnt > 0);
}


TEST(META, meta_serializeReadWrite)
{
    ClassA a;
    ClassA b;
    a.a = 8.10f;
    a.a2 = 26.04f;
    a.s = "pollen3d";
    a.vd = {0.65,5465};
    a.m.setOnes();
    a.mx.setOnes();

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

    EXPECT_EQ(a.a, b.a);
    EXPECT_EQ(a.a2, b.a2);
    EXPECT_EQ(a.s, b.s);
    EXPECT_EQ(a.b, b.b);
    EXPECT_EQ(a.vd, b.vd);
    EXPECT_EQ(a.m, b.m);
    EXPECT_EQ(a.mx, b.mx);

}

TEST(META, meta_serializeVecMatches)
{
    std::vector<Match> m1;
    m1.emplace_back(Match(0,1,0.5));
    m1.emplace_back(Match(1,2));
    m1.emplace_back(Match(2,3));

    {
        size_t id = meta::resolve<std::vector<Match>>().id();
        cv::FileStorage fs("test.xml", cv::FileStorage::WRITE);
        fs << "node1" << "{";
        impl::_writeVecS<Match>(m1,id,fs);
        fs << "}";
        fs.release();
    }

    std::vector<Match> m2;

    {
        cv::FileStorage fs("test.xml", cv::FileStorage::READ);
        cv::FileNode node = fs["node1"];
        size_t id = meta::resolve<std::vector<Match>>().id();
        m2 = impl::_readVecS<Match>(id, node);
        fs.release();
    }

    EXPECT_EQ(m1.size(), m2.size());
    for (int i = 0; i < m2.size(); i++)
        EXPECT_EQ(m1[i], m2[i]);
}


TEST(META, meta_serializeProject)
{
    ProjectData data1;
    std::string path = "test" + std::string(P3D_PROJECT_EXTENSION);
    data1.setProjectPath(path);
    ProjectManager::get()->saveProject(&data1,path);

    ProjectData data2;
    ProjectManager::get()->openProject(&data2,path);

    meta::resolve<ProjectData>().data([&](meta::data data){
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

TEST(META, meta_setterGetter)
{
    A a;
    a.setValue(15);
    ASSERT_EQ(a.getValue(),15);
    CommandManager::get()->executeCommand(new CommandSetProperty(&a,110521,254));
    ASSERT_EQ(a.getValue(),254);
    CommandManager::get()->undoCommand();
    ASSERT_EQ(a.getValue(),15);
}
