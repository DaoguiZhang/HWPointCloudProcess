#include "OpenGLWidget.h"
#include <fstream>
#include <Eigen/Dense>
#define M_PI       3.14159265358979323846
OpenGLWidget::OpenGLWidget(QWidget * parent)
	: QGLWidget(parent)
	, VBO_(QOpenGLBuffer::VertexBuffer)
	, EBO_(QOpenGLBuffer::IndexBuffer)
	, texture_(nullptr)
{

}

OpenGLWidget::~OpenGLWidget()
{
}

QSize OpenGLWidget::sizeHint() const
{
	return QSize(1024, 1024);
}

void OpenGLWidget::initializeGL()
{
	//this->initializeOpenGLFunctions();
	/*cv::Mat pano = cv::imread("G:\\xht\\huawei\\2019-04-28_14.54.52\\pano\\00000-pano.jpg");
	int pano_width_ = pano.cols, pano_height_ = pano.rows;
	int cube_length = pano_height_ / 2;*/	
	glEnable(GL_DEPTH_TEST);
	//glDisable(GL_CULL_FACE);
	//qglClearColor(QColor(Qt::black));
}

void OpenGLWidget::resizeGL(int width, int height)
{
	if (height == 0) {
		height = 1;
	}
	projMatrix_.setToIdentity();
	projMatrix_.perspective(90.0, 1.0f, 0.001, 100);
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixf(projMatrix_.data());
}

void OpenGLWidget::paintGL()
{
	//FBO_->bind();
	//GLenum buffers[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	shader_program_.bind();
	shader_program_.setUniformValue("projection", projMatrix_);
	shader_program_.setUniformValue("sampler", 0);
	texture_->bind();
	VAO_.bind();
	//glDrawArrays(GL_TRIANGLES, 0, 36);
	glDrawElements(GL_TRIANGLES, num, GL_UNSIGNED_INT, 0);
	//glDrawElements(GL_TRIANGLES, num, GL_UNSIGNED_INT, 0);
	VAO_.release();
	texture_->release();
	shader_program_.release();
	/*cv::Mat colorImg(1024, 1024, CV_8UC3);
	glReadPixels(0, 0, 1024, 1024, GL_BGR, GL_UNSIGNED_BYTE, colorImg.data);
	cv::flip(colorImg, colorImg, 0);
	cv::imshow("color", colorImg);
	cv::waitKey(0);
	FBO_->release();*/
	swapBuffers();
}

void OpenGLWidget::Init(std::vector<float3>* points, std::vector<int3>* faces)
{
	GLfloat vertices[] = {
		-1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		-1.0f, 1.0f, -1.0f,
		1.0f, 1.0f, -1.0f,
		-1.0f, -1.0f, 1.0f,
		1.0f, -1.0f, 1.0f,
		-1.0f, 1.0f, 1.0f,
		1.0f, 1.0f, 1.0f,
	};
	GLuint indices[] = {
		2, 1, 0,
		1, 2, 3,
		4, 2, 0,
		2, 4, 6,
		1, 4, 0,
		4, 1, 5,
		6, 5, 7,
		5, 6, 4,
		3, 6, 7,
		6, 3, 2,
		5, 3, 7,
		3, 5, 1,
	};
	bool success = shader_program_.addShaderFromSourceFile(QGLShader::Vertex, "../../src/cube.vert");
	//bool success = shader_program_.addShaderFromSourceFile(QGLShader::Vertex, "../../src/fisheye.vert");
	if (!success) {
		qDebug() << "shaderProgram addShaderFromSourceFile failed!" << shader_program_.log();
		return;
	}
	success = shader_program_.addShaderFromSourceFile(QGLShader::Fragment, "../../src/cube.frag");
	//success = shader_program_.addShaderFromSourceFile(QGLShader::Fragment, "../../src/fisheye.frag");
	if (!success) {
		qDebug() << "shaderProgram addShaderFromSourceFile failed!" << shader_program_.log();
		return;
	}
	success = shader_program_.link();
	if (!success) {
		qDebug() << "shaderProgram link failed!" << shader_program_.log();
	}
	shader_program_.bind();
	VAO_.create();
	VAO_.bind();
	VBO_.create();
	VBO_.bind();
	VBO_.setUsagePattern(QOpenGLBuffer::StaticDraw);
	VBO_.allocate(vertices, sizeof(vertices));
	EBO_.create();
	EBO_.bind();
	EBO_.setUsagePattern(QOpenGLBuffer::StaticDraw);
	EBO_.allocate(indices, sizeof(indices));
	num = 36;
	/*VBO_.allocate(points->data(), sizeof(float) * 3 * points->size());
	EBO_.create();
	EBO_.bind();
	EBO_.setUsagePattern(QOpenGLBuffer::StaticDraw);
	EBO_.allocate(faces->data(), sizeof(int) * 3 * faces->size());
	num = 3 * faces->size();*/
	shader_program_.enableAttributeArray(0);
	shader_program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, 3 * sizeof(float));
	VBO_.release();
	VAO_.release();
	shader_program_.release();
}

void OpenGLWidget::loadTexture(QImage& img)
{
	makeCurrent();
	if (texture_!=nullptr) {
		texture_->destroy();
	}
	//QString img= QString::fromStdString(path);
	texture_ = new QOpenGLTexture(img);
	width_ = 1440;// texture_->width() / 8;
	height_ = 1440;// texture_->height() / 8;
	texture_->setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::Repeat);
	texture_->setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::Repeat);
																			 
	texture_->setMinificationFilter(QOpenGLTexture::Linear);
	texture_->setMagnificationFilter(QOpenGLTexture::Linear);

	QOpenGLFramebufferObjectFormat fboFormat;
	fboFormat.setMipmap(true);
	fboFormat.setSamples(0);
	fboFormat.setInternalTextureFormat(GL_RGB);
	FBO_ = new QOpenGLFramebufferObject(QSize(width_, height_), fboFormat);
	FBO_->release();
	//return cube_length_;
}

void OpenGLWidget::setViewMatrix(Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& view, Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& view_origin)
{
	makeCurrent();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf(view.data());
	shader_program_.bind();	
	shader_program_.setUniformValue("view", QMatrix4x4(view.data()));
	shader_program_.setUniformValue("view_origin", QMatrix4x4(view_origin.data()));
	shader_program_.release();
	//char camDir[256];
	//sprintf(camDir, "G:\\xht\\huawei\\scene\\%05d-cam%d.CAM", pano_idx, i);
	//std::ofstream outFile(camDir);
	//outFile << view(0, 3) << " " << view(1, 3) << " " << view(2, 3) << " ";
	//for (int row = 0; row < 3; row++) {
	//	for (int col = 0; col < 3; col++) {
	//		outFile << view(row, col) << " ";
	//	}
	//}
	//outFile << "\n" << 0.5f << "\n";
	////outFile << cube_length_ / 2  << " " << 0.0f << " " << 0.0f << " " << 1.0f << " " << cube_length_ / 2 << " " << cube_length_ / 2 << "\n";
	//outFile.close();
}

void OpenGLWidget::setUniform(Eigen::Matrix<float, 4, 4, Eigen::RowMajor>& view,float* intr)
{
	makeCurrent();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf(view.data());
	projMatrix_.setToIdentity();
	projMatrix_.perspective(90.0, (float)height_ / (float)width_, 0.001, 100);
	projMatrix_(0, 0) = 1.0f;
	projMatrix_(1, 1) = (float)width_ / (float)height_;
	/*std::cout << projMatrix_(0, 0) << "  " << projMatrix_(0, 1) << "  " << projMatrix_(0, 2) << "  " << projMatrix_(0, 3) << "  " << std::endl;
	std::cout << projMatrix_(1, 0) << "  " << projMatrix_(1, 1) << "  " << projMatrix_(1, 2) << "  " << projMatrix_(1, 3) << "  " << std::endl;
	std::cout << projMatrix_(2, 0) << "  " << projMatrix_(2, 1) << "  " << projMatrix_(2, 2) << "  " << projMatrix_(2, 3) << "  " << std::endl;
	std::cout << projMatrix_(3, 0) << "  " << projMatrix_(3, 1) << "  " << projMatrix_(3, 2) << "  " << projMatrix_(3, 3) << "  " << std::endl;
	system("pause");*/
	glViewport(0, 0, width_, height_);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixf(projMatrix_.data());
	shader_program_.bind();
	shader_program_.setUniformValue("view", QMatrix4x4(view.data()));
	shader_program_.setUniformValue("projection", projMatrix_);
	shader_program_.setUniformValueArray("intr", intr, 23, 1);
	shader_program_.setUniformValue("width", width_);
	shader_program_.setUniformValue("height", height_);
	shader_program_.release();
}

void OpenGLWidget::renderToImg(cv::Mat& colorImg)
{
	makeCurrent();
	//glDisable(GL_DEPTH_TEST);
	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0, 0, width_, height_);
	FBO_->bind();
	//glDepthFunc(GL_LEQUAL);
	paintGL();
	colorImg = cv::Mat(height_, width_, CV_8UC3);
	glReadPixels(0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, colorImg.data);
	FBO_->release();
	glPopAttrib();
	cv::flip(colorImg, colorImg, 0);
	/*std::vector<int> pngCompressionParams;
	pngCompressionParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
	pngCompressionParams.push_back(0);
	char imgDir[256];
	sprintf(imgDir, "G:\\xht\\huawei\\scene\\%05d-cam%d.PNG", pano_idx, i);
	cv::imwrite(imgDir, colorImg, pngCompressionParams);*/
}
