#include "app_opengl.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

#include "p3d/console_logger.h"

#ifdef __APPLE__
#define P3D_GLSL_VERSION "#version 120"
#else
#define P3D_GLSL_VERSION 
#endif

void ApplicationOpenGL::init() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    if(!glfwInit()){
        throw "Failed to initialize OpenGL loader!";
    }
    m_window = glfwCreateWindow(m_width, m_height, "Pollen3D", nullptr, nullptr);
    if (m_window == nullptr) return;
    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1); // Enable vsync

    auto loadIcon = [](GLFWimage * im, std::string path){
        cv::Mat icon1 = cv::imread(path,CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat icon2;
        cv::cvtColor(icon1,icon2,CV_BGR2RGBA);
        im->width  = icon2.cols;
        im->height = icon2.rows;
        im->pixels = icon2.clone().data;
    };

    GLFWimage images[1];
    loadIcon(&images[0],"../../assets/pollen3d_icon64.png");
    //loadIcon(&images[1],"../../assets/pollen3d_icon32.png");
    glfwSetWindowIcon(m_window, 1, images);

#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
    bool err = gl3wInit() != 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
    glewExperimental = GL_TRUE;
    bool err = glewInit() != GLEW_OK;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
    bool err = gladLoadGL() == 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING)
    bool err = false;
    glbinding::initialize([](const char* name) { return (glbinding::ProcAddress)glfwGetProcAddress(name); });
#else
    bool err = false; // If you use IMGUI_IMPL_OPENGL_LOADER_CUSTOM, your loader is likely to requires some form of initialization.
#endif

    if (err) throw "Failed to initialize OpenGL loader!";

    // Setup Dear ImGui context

    initImGui();

    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init(P3D_GLSL_VERSION);
}

void ApplicationOpenGL::setWindowTitleImpl(std::string str)
{
    glfwSetWindowTitle(m_window, str.c_str());
}

void ApplicationOpenGL::destroy() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(m_window);
    glfwTerminate();
}

void ApplicationOpenGL::preLoop(){
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    glfwGetWindowSize(m_window,&m_width,&m_height);
}

void ApplicationOpenGL::postLoop() {
    int display_w, display_h;
    glfwGetFramebufferSize(m_window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(m_window);
}

void ApplicationOpenGL::textureBind(const cv::Mat &im)
{
    m_textureWidth  = 0;
    m_textureHeight = 0;

    if(im.empty()){
        LOG_DBG("textureAlloc: image is empty");
        return;
    }

    cv::Mat mat;
    if (im.channels() == 3) {
        cv::cvtColor(im,mat,CV_BGR2RGBA);
    } else if (im.channels() == 1) {
        cv::cvtColor(im,mat,CV_GRAY2RGBA);
    }

    if (mat.empty()) {
        LOG_ERR("TextureDisplay: conversion failed");
    }

    if(m_textureId) {
        glDeleteTextures(1,&m_textureId);
        m_textureId = 0;
    }
    int error = glGetError();

    glGenTextures(1, &m_textureId);
    glBindTexture(GL_TEXTURE_2D, m_textureId);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    //glPixelStorei(GL_UNPACK_ALIGNMENT,1);

    // Create the texture
    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                 0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGBA,    // Internal colour format to convert to
                 mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
                 mat.rows,          // Image height i.e. 480 for Kinect in standard mode
                 0,                 // Border width in pixels (can either be 1 or 0)
                 GL_RGBA, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,  // Image data type
                 mat.ptr());        // The actual image data itself

    if (!glGetError()) {
        m_textureWidth  = mat.cols;
        m_textureHeight = mat.rows;
        LOG_DBG("Texture bind");
    } else {
        LOG_ERR("glError: %i", error);
    }

}

void ApplicationOpenGL::textureDisplay(const ImVec2 &size, ImVec2 uv0, ImVec2 uv1)
{
    if (m_textureId && isTextureReady()) {
        ImGui::Image(ImTextureID(m_textureId), size, uv0, uv1);
    }
}
