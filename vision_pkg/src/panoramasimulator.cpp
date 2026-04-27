#include "vision_pkg/panoramasimulator.h"
#include <ros/ros.h>
#include <stdexcept>
#include <cmath>

// GL function pointers (loaded via EGL/GL)
// On Ubuntu 20 with mesa, linking -lGL provides these directly.

PanoramaSimulator::PanoramaSimulator()
{
}

PanoramaSimulator::~PanoramaSimulator()
{
    if (textureID) glDeleteTextures(1, &textureID);
    if (vaoID) glDeleteVertexArrays(1, &vaoID);
    if (vboID) glDeleteBuffers(1, &vboID);
    if (iboID) glDeleteBuffers(1, &iboID);
    if (shaderProgram) glDeleteProgram(shaderProgram);

    if (fbo) glDeleteFramebuffers(1, &fbo);
    if (rboColor) glDeleteRenderbuffers(1, &rboColor);
    if (rboDepth) glDeleteRenderbuffers(1, &rboDepth);

    if (eglDisplay != EGL_NO_DISPLAY) {
        eglMakeCurrent(eglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (eglContext != EGL_NO_CONTEXT) eglDestroyContext(eglDisplay, eglContext);
        if (eglSurface != EGL_NO_SURFACE) eglDestroySurface(eglDisplay, eglSurface);
        eglTerminate(eglDisplay);
    }
}

void PanoramaSimulator::initialize()
{
    initializeEGL();
    initializeGL();
    ROS_INFO("PanoramaSimulator initialized with EGL headless context");
}

void PanoramaSimulator::initializeEGL()
{
    eglDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (eglDisplay == EGL_NO_DISPLAY)
        throw std::runtime_error("EGL: Failed to get display");

    EGLint major, minor;
    if (!eglInitialize(eglDisplay, &major, &minor))
        throw std::runtime_error("EGL: Failed to initialize");

    // Choose config with OpenGL support
    EGLint configAttribs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 8,
        EGL_DEPTH_SIZE, 24,
        EGL_NONE
    };
    EGLConfig config;
    EGLint numConfigs;
    if (!eglChooseConfig(eglDisplay, configAttribs, &config, 1, &numConfigs) || numConfigs == 0)
        throw std::runtime_error("EGL: Failed to choose config");

    // Create PBuffer surface (headless)
    EGLint pbufferAttribs[] = {
        EGL_WIDTH, windowWidth,
        EGL_HEIGHT, windowHeight,
        EGL_NONE
    };
    eglSurface = eglCreatePbufferSurface(eglDisplay, config, pbufferAttribs);
    if (eglSurface == EGL_NO_SURFACE)
        throw std::runtime_error("EGL: Failed to create PBuffer surface");

    // Bind OpenGL API and create context
    eglBindAPI(EGL_OPENGL_API);
    EGLint contextAttribs[] = {
        EGL_CONTEXT_MAJOR_VERSION, 3,
        EGL_CONTEXT_MINOR_VERSION, 1,
        EGL_NONE
    };
    eglContext = eglCreateContext(eglDisplay, config, EGL_NO_CONTEXT, contextAttribs);
    if (eglContext == EGL_NO_CONTEXT)
        throw std::runtime_error("EGL: Failed to create context");

    if (!eglMakeCurrent(eglDisplay, eglSurface, eglSurface, eglContext))
        throw std::runtime_error("EGL: Failed to make context current");

    // Create FBO for offscreen rendering
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glGenRenderbuffers(1, &rboColor);
    glBindRenderbuffer(GL_RENDERBUFFER, rboColor);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, windowWidth, windowHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rboColor);

    glGenRenderbuffers(1, &rboDepth);
    glBindRenderbuffer(GL_RENDERBUFFER, rboDepth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, windowWidth, windowHeight);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rboDepth);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        throw std::runtime_error("EGL: Framebuffer is not complete");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void PanoramaSimulator::initializeGL()
{
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    initShaders();
    initSphereGeometry();

    projectionMatrix = glm::perspective(
        glm::radians(fovy),
        static_cast<float>(windowWidth) / windowHeight,
        0.1f, 10.0f
    );
}

void PanoramaSimulator::initShaders()
{
    const char *vertexShaderSource = R"(
        #version 140
        in vec3 position;
        in vec2 texCoord;
        out vec2 vTexCoord;
        uniform mat4 projection;
        uniform mat4 view;
        uniform mat4 model;
        void main() {
            gl_Position = projection * view * model * vec4(position, 1.0);
            vTexCoord = texCoord;
        }
    )";

    const char *fragmentShaderSource = R"(
        #version 140
        in vec2 vTexCoord;
        out vec4 fragColor;
        uniform sampler2D textureSampler;
        void main() {
            fragColor = texture(textureSampler, vTexCoord);
        }
    )";

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);
    GLint success;
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);
        throw std::runtime_error(std::string("Vertex shader compile failed: ") + infoLog);
    }

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(fragmentShader, 512, nullptr, infoLog);
        throw std::runtime_error(std::string("Fragment shader compile failed: ") + infoLog);
    }

    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram, 512, nullptr, infoLog);
        throw std::runtime_error(std::string("Shader program link failed: ") + infoLog);
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void PanoramaSimulator::initSphereGeometry()
{
    std::vector<float> vertices;
    std::vector<unsigned int> indices;
    int stacks = 40, slices = 80;

    for (int i = 0; i <= stacks; ++i) {
        float v = (float)i / stacks;
        float phi = M_PI * (v - 0.5f);
        float z = sin(phi);
        float zr = cos(phi);
        for (int j = 0; j <= slices; ++j) {
            float u = (float)j / slices;
            float theta = 2 * M_PI * u;
            float x = cos(theta) * zr;
            float y = sin(theta) * zr;
            vertices.push_back(-x);
            vertices.push_back(-y);
            vertices.push_back(-z);
            vertices.push_back(u);
            vertices.push_back(v);
        }
    }

    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            int k1 = i * (slices + 1) + j;
            int k2 = k1 + slices + 1;
            indices.push_back(k1);
            indices.push_back(k2);
            indices.push_back(k1 + 1);
            indices.push_back(k1 + 1);
            indices.push_back(k2);
            indices.push_back(k2 + 1);
        }
    }

    glGenVertexArrays(1, &vaoID);
    glGenBuffers(1, &vboID);
    glGenBuffers(1, &iboID);

    glBindVertexArray(vaoID);

    glBindBuffer(GL_ARRAY_BUFFER, vboID);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, iboID);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    indexCount = indices.size();
}

void PanoramaSimulator::drawSphere()
{
    glUseProgram(shaderProgram);

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::rotate(model, glm::radians(sphereAngleX), glm::vec3(1.0f, 0.0f, 0.0f));
    model = glm::rotate(model, glm::radians(sphereAngleY), glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::rotate(model, glm::radians(sphereAngleZ), glm::vec3(0.0f, 0.0f, 1.0f));
    model = glm::scale(model, glm::vec3(1.0f));

    GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
    GLint viewLoc = glGetUniformLocation(shaderProgram, "view");
    GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, &projectionMatrix[0][0]);
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, &viewMatrix[0][0]);
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, &model[0][0]);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glUniform1i(glGetUniformLocation(shaderProgram, "textureSampler"), 0);

    glBindVertexArray(vaoID);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    glUseProgram(0);
}

void PanoramaSimulator::processFrames(const cv::Mat &frame1, const cv::Mat &frame2)
{
    if (frame1.empty() || frame2.empty()) {
        ROS_WARN("PanoramaSimulator: Input frames empty, skipping");
        return;
    }
    m_frame1 = frame1.clone();
    m_frame2 = frame2.clone();
    updateTexture();

    cv::Mat rendered = captureRenderedFrame();
    if (m_callback && !rendered.empty()) {
        m_callback(rendered);
    }
}

void PanoramaSimulator::updateTexture()
{
    if (!m_frame1.empty() && !m_frame2.empty()) {
        cv::Mat stitched = combineImages(m_frame1, m_frame2);
        if (!stitched.empty()) {
            loadTexture(stitched);
        } else {
            ROS_ERROR("PanoramaSimulator: Stitched panorama is empty");
        }
    }
}

void PanoramaSimulator::loadTexture(const cv::Mat &image)
{
    eglMakeCurrent(eglDisplay, eglSurface, eglSurface, eglContext);

    if (textureID) glDeleteTextures(1, &textureID);
    cv::Mat rgbImage, flipped;
    try {
        cv::flip(image, flipped, 0);
        cv::cvtColor(flipped, rgbImage, cv::COLOR_BGR2RGB);
    } catch (const cv::Exception &e) {
        ROS_ERROR("OpenCV color conversion failed: %s", e.what());
        return;
    }

    cv::Mat rgbaImage;
    cv::cvtColor(rgbImage, rgbaImage, cv::COLOR_RGB2RGBA);
    for (int y = 0; y < rgbaImage.rows; y++) {
        for (int x = 0; x < rgbaImage.cols; x++) {
            cv::Vec4b& pixel = rgbaImage.at<cv::Vec4b>(y, x);
            if (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0) pixel[3] = 0;
        }
    }

    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgbaImage.cols, rgbaImage.rows,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, rgbaImage.data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        ROS_ERROR("OpenGL error in loadTexture: %d", err);
    }
}

cv::Mat PanoramaSimulator::combineImages(const cv::Mat &A, const cv::Mat &B)
{
    if (A.empty() || B.empty()) {
        ROS_ERROR("combineImages: Input images empty");
        return cv::Mat();
    }
    cv::Mat B_resized;
    if (A.size() != B.size()) {
        cv::resize(B, B_resized, A.size());
    } else {
        B_resized = B;
    }
    int H = A.rows;
    int W = A.cols;
    cv::Mat B_rotated;
    cv::flip(B_resized, B_rotated, -1);

    cv::Mat C = cv::Mat::zeros(H, W, A.type());
    cv::Rect topHalf(0, 0, W, H / 2);
    cv::Rect bottomHalf(0, H / 2, W, H / 2);
    A(topHalf).copyTo(C(topHalf));
    B_rotated(bottomHalf).copyTo(C(bottomHalf));
    return C;
}

cv::Mat PanoramaSimulator::captureRenderedFrame()
{
    eglMakeCurrent(eglDisplay, eglSurface, eglSurface, eglContext);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0, 0, windowWidth, windowHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float radius = 5.0f;
    float lookDirX = radius * cos(cameraAngleX * M_PI / 180.0f) * sin(cameraAngleY * M_PI / 180.0f);
    float lookDirY = radius * sin(cameraAngleX * M_PI / 180.0f);
    float lookDirZ = radius * cos(cameraAngleX * M_PI / 180.0f) * cos(cameraAngleY * M_PI / 180.0f);

    glm::vec3 eye(0.0f, 0.0f, 0.0f);
    glm::vec3 center(lookDirX, lookDirY, lookDirZ);
    glm::vec3 up(0.0f, 1.0f, 0.0f);
    viewMatrix = glm::lookAt(eye, center, up);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);

    drawSphere();

    glEnable(GL_DEPTH_TEST);

    cv::Mat frame(windowHeight, windowWidth, CV_8UC3);
    glReadPixels(0, 0, windowWidth, windowHeight, GL_RGB, GL_UNSIGNED_BYTE, frame.data);
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    cv::flip(frame, frame, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return frame;
}

void PanoramaSimulator::setPerspective()
{
    cameraAngleY = 0.0f;
    cameraAngleX = 0.0f;
}

void PanoramaSimulator::setPerspective(float x, float y)
{
    float dx = (x - 0.5f) * 2.0f;
    float dy = (y - 0.5f) * 2.0f;

    cameraAngleY -= dx * yawSpeed;
    cameraAngleX += dy * pitchSpeed;

    if (cameraAngleX > 90.0f) cameraAngleX = 90.0f;
    if (cameraAngleX < -90.0f) cameraAngleX = -90.0f;
    if (cameraAngleY >= 360.0f) cameraAngleY -= 360.0f;
    if (cameraAngleY < 0.0f) cameraAngleY += 360.0f;
}
