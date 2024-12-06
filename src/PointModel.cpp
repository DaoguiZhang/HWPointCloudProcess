#include "PointModel.h"

PointModel::PointModel(QGLWidget * _openglWindow)
	:BaseModel(_openglWindow)
{
	this->point_size_ = 10.0f;
}

PointModel::~PointModel()
{
}

void PointModel::SetPointSize(float pz)
{
	this->point_size_ = pz;
}

void PointModel::Draw()
{
	VAO_->bind();
	glPointSize(point_size_);
	glDrawArrays(GL_POINTS, 0, vertex_num_);
	glPointSize(1.0f);
	VAO_->release();
}
