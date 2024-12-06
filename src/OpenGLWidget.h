#pragma  once
#include <GL/glut.h>
#include <QGLWidget>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLExtraFunctions>
#include <QOpenGLFunctions>
#include <QOpenGLFramebufferObject>
#include <QGLFramebufferObjectFormat>
#include <QGLShaderProgram>
#include <QOpenGLTexture>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"
#include "math_utils.h"

class OpenGLWidget : public QGLWidget//, protected QOpenGLFunctions
{
	Q_OBJECT
public:
	OpenGLWidget(QWidget *parent = 0);
	~OpenGLWidget();
	QSize sizeHint() const;

	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();
	void renderToImg(cv::Mat& colorImg);

	void Init(std::vector<float3>* points, std::vector<int3>* faces);
	void loadTexture(QImage& img);
	void setViewMatrix(Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& view, Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& view_origin);
	void setUniform(Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& view,float* intr);
	//void mousePressEvent(QMouseEvent *event);
	//void mouseMoveEvent(QMouseEvent *event);
	//void wheelEvent(QWheelEvent *event);

private:
	QMatrix4x4 projMatrix_;
	QGLShaderProgram shader_program_;
	QOpenGLBuffer VBO_;
	QOpenGLBuffer EBO_;
	QOpenGLVertexArrayObject VAO_;
	QOpenGLFramebufferObject* FBO_;
	QOpenGLTexture* texture_;
	int width_, height_;
	int num;
};
