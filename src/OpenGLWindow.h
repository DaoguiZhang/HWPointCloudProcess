#pragma once
#include <GL/glut.h>
#include <QGLWidget>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QGLShaderProgram>
#include <QOpenGLTexture>
#include <QMouseEvent>
#include <QToolBar>
#include <QFileDialog>
#include <QtXml/QDomDocument>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"
#include "math_utils.h"
#include "texturing.h"
#include "ArcBall.h"

class OpenGLWindow : public QGLWidget//, protected QOpenGLFunctions
{
	Q_OBJECT
public:
	OpenGLWindow(QWidget *parent = 0);
	~OpenGLWindow();


	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();
	void setAttributeToBuffer(tex::Model& model,std::string& out_prefix);
	//void addIndexToBuffer();

	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	void keyPressEvent(QKeyEvent *event);
	void loadCameraSlot();
	//void keyReleaseEvent(QKeyEvent *event);

	

private:
	QMatrix4x4 projMatrix_;
	QMatrix4x4 mvMatrix_;
	QMatrix4x4 mvMatrix_origin_;
	QGLShaderProgram shader_program_;
	std::vector<QOpenGLBuffer*> VBOs_;
	std::vector<QOpenGLVertexArrayObject*> VAOs_;
	
	std::vector<QOpenGLTexture*> textures_;
	std::vector<int> counts_;
	int w, h;
	float fov_;
	float xTrans_;
	float yTrans_;
	float zTrans_;
	float scale_;
	ArcBallT *globalArcBall_;
	int lastMouseButton;

	tex::Model *model_;
	std::string out_prefix_;
//public slots :
	
};
