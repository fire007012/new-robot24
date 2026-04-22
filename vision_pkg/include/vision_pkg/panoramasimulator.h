#ifndef VISION_PKG_PANORAMASIMULATOR_H
#define VISION_PKG_PANORAMASIMULATOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <functional>

#include <EGL/egl.h>
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "vision_pkg/fisheye.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 基于 EGL 离屏渲染的全景球面模拟器
 *
 * 将两路鱼眼全景图拼合后贴到 3D 球体内表面，通过 OpenGL 渲染实现
 * 交互式视角控制。使用 EGL PBuffer 实现无显示器（headless）离屏渲染，
 * 适合嵌入式/服务器环境。
 */
class PanoramaSimulator
{
public:
    using PanoramaReadyCallback = std::function<void(const cv::Mat&)>;

    PanoramaSimulator();
    ~PanoramaSimulator();

    /** @brief 初始化 EGL 上下文和 OpenGL 资源（着色器、球体几何、FBO） */
    void initialize();

    /**
     * @brief 通过手柄输入设置相机视角
     * @param x 归一化水平输入 [0,1]，0.5 为中心
     * @param y 归一化垂直输入 [0,1]，0.5 为中心
     */
    void setPerspective(float x, float y);

    /** @brief 重置相机视角到默认正前方 */
    void setPerspective();

    /**
     * @brief 处理两路全景帧：拼合纹理 -> 渲染球面 -> 输出渲染结果
     * @param frame1 前方全景图
     * @param frame2 后方全景图
     */
    void processFrames(const cv::Mat &frame1, const cv::Mat &frame2);

    /** @brief 从 FBO 读取渲染结果为 OpenCV Mat */
    cv::Mat captureRenderedFrame();

    void setPanoramaReadyCallback(PanoramaReadyCallback cb) { m_callback = std::move(cb); }

    float getCameraDistance() const { return cameraDistance; }
    float getCameraAngleX() const { return cameraAngleX; }
    float getCameraAngleY() const { return cameraAngleY; }

private:
    /** @brief 初始化 EGL 显示、上下文、PBuffer 表面和 FBO */
    void initializeEGL();
    /** @brief 初始化 OpenGL 状态、着色器和球体几何 */
    void initializeGL();
    /** @brief 绘制贴有全景纹理的球体 */
    void drawSphere();
    /** @brief 将 OpenCV 图像上传为 OpenGL 纹理 */
    void loadTexture(const cv::Mat &image);
    /**
     * @brief 拼合前后两路全景图：取 A 的上半部分和 B（旋转180°）的下半部分
     * @param A 前方全景图
     * @param B 后方全景图
     * @return 拼合后的完整全景图
     */
    cv::Mat combineImages(const cv::Mat &A, const cv::Mat &B);
    /** @brief 更新球体纹理（拼合 + 上传） */
    void updateTexture();

    /** @brief 编译并链接顶点/片段着色器（GLSL 1.40） */
    void initShaders();
    /** @brief 生成 UV 球体网格（40层 x 80片），创建 VAO/VBO/IBO */
    void initSphereGeometry();

    // === EGL 上下文资源 ===
    EGLDisplay eglDisplay = EGL_NO_DISPLAY;
    EGLContext eglContext = EGL_NO_CONTEXT;
    EGLSurface eglSurface = EGL_NO_SURFACE;
    GLuint fbo = 0;       // 离屏帧缓冲对象
    GLuint rboColor = 0;  // 颜色渲染缓冲
    GLuint rboDepth = 0;  // 深度渲染缓冲

    // === 相机参数 ===
    float cameraDistance = 0.01f;   // 相机到球心距离（近似在球心）
    float cameraAngleX = 0.0f;     // 俯仰角（度）
    float cameraAngleY = 0.0f;     // 偏航角（度）
    float fovy = 60.0f;            // 垂直视场角（度）
    float sphereAngleX = 0.0f;     // 球体绕 X 轴旋转角度
    float sphereAngleY = 90.0f;    // 球体绕 Y 轴旋转角度
    float sphereAngleZ = 90.0f;    // 球体绕 Z 轴旋转角度
    const float yawSpeed = 12.36f;   // 偏航旋转速度系数
    const float pitchSpeed = 12.36f; // 俯仰旋转速度系数
    GLuint textureID = 0;           // 全景纹理 ID

    // === OpenGL 渲染资源 ===
    GLuint vaoID = 0, vboID = 0, iboID = 0; // 球体 VAO/VBO/IBO
    GLuint shaderProgram = 0;                // 着色器程序
    GLsizei indexCount = 0;                  // 球体索引数量

    int windowWidth = 540;   // 渲染目标宽度
    int windowHeight = 270;  // 渲染目标高度

    cv::Mat m_frame1, m_frame2;      // 缓存的前后全景帧
    glm::mat4 projectionMatrix;      // 投影矩阵
    glm::mat4 viewMatrix;            // 视图矩阵

    PanoramaReadyCallback m_callback;
};

#endif // VISION_PKG_PANORAMASIMULATOR_H

