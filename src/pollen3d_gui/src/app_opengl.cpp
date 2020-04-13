#include "app_opengl.h"

#include <filesystem>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "p3d/logger.h"

using namespace p3d;

void error_callback([[maybe_unused]] int error, const char* description)
{
    LOG_ERR("GLFW: %i, %s", error, description);
}

void ApplicationOpenGL::init()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    if (!glfwInit()) throw "Failed to initialize OpenGL loader!";

    const char* version = NULL;

#ifdef __APPLE__
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    version = "#version 410";
#endif

    glfwSetErrorCallback(error_callback);

    m_window = glfwCreateWindow(m_width, m_height, "Pollen3D", nullptr, nullptr);
    if (m_window == nullptr) return;
    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);  // Enable vsync

    auto loadIcon = [](GLFWimage* im, std::string path) {
        cv::Mat icon1 = cv::imread(path, CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat icon2;
        cv::cvtColor(icon1, icon2, CV_BGR2RGBA);
        im->width = icon2.cols;
        im->height = icon2.rows;
        im->pixels = icon2.clone().data;
    };

    GLFWimage images[1];
    loadIcon(&images[0], m_execPath + "/assets/pollen3d_icon64.png");
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
    bool err = false;  // If you use IMGUI_IMPL_OPENGL_LOADER_CUSTOM, your loader is likely to requires some form of initialization.
#endif

    if (err) throw "Failed to initialize OpenGL loader!";

    GLint major, minor;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    LOG_INFO("OpenGL version: %i.%i", major, minor);

    // Setup Dear ImGui context

    initImGui();

    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init(version);

    m_textureId = new GLuint(0);
    m_viewer3D = std::make_unique<Viewer3DOpenGL>(version);
    m_viewer3D->init();
}

void ApplicationOpenGL::setWindowTitleImpl(std::string str)
{
    glfwSetWindowTitle(m_window, str.c_str());
}

void ApplicationOpenGL::destroy()
{
    if (m_textureId) delete (GLuint*)m_textureId;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(m_window);
    glfwTerminate();
}

void ApplicationOpenGL::preLoop()
{
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    glfwGetWindowSize(m_window, &m_width, &m_height);
}

void ApplicationOpenGL::postLoop()
{
    int display_w, display_h;
    glfwGetFramebufferSize(m_window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(m_window);
}

void ApplicationOpenGL::textureBind(const cv::Mat& im)
{
    m_textureWidth = 0;
    m_textureHeight = 0;

    GLuint* tId = (GLuint*)m_textureId;

    if (im.empty()) {
        LOG_DBG("textureAlloc: image is empty");
        return;
    }

    cv::Mat mat;
    if (im.channels() == 3) {
        cv::cvtColor(im, mat, CV_BGR2RGBA);
    } else if (im.channels() == 1) {
        cv::cvtColor(im, mat, CV_GRAY2RGBA);
    }

    if (mat.empty()) {
        LOG_ERR("TextureDisplay: conversion failed");
    }

    if (tId) {
        glDeleteTextures(1, tId);
        *tId = 0;
    }

    glGenTextures(1, tId);
    glBindTexture(GL_TEXTURE_2D, *tId);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    //glPixelStorei(GL_UNPACK_ALIGNMENT,1);

    // Create the texture
    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                 0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGBA,           // Internal colour format to convert to
                 mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
                 mat.rows,          // Image height i.e. 480 for Kinect in standard mode
                 0,                 // Border width in pixels (can either be 1 or 0)
                 GL_RGBA,           // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,  // Image data type
                 mat.ptr());        // The actual image data itself

    auto error = glGetError();
    if (!error) {
        m_textureWidth = mat.cols;
        m_textureHeight = mat.rows;
        LOG_DBG("Texture bind");
    } else {
        LOG_ERR("glError: %i", error);
    }
}

void ApplicationOpenGL::textureDisplay(const ImVec2& size, ImVec2 uv0, ImVec2 uv1)
{
    GLuint* tId = (GLuint*)m_textureId;
    if (*tId && isTextureReady()) {
        ImGui::Image(reinterpret_cast<ImTextureID>(*tId), size, uv0, uv1);
    }
}
