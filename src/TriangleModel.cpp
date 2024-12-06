#include "TriangleModel.h"

TriangleModel::TriangleModel(QGLWidget *_openglWindow)
	:BaseModel(_openglWindow)
{
}

TriangleModel::~TriangleModel()
{
}

void TriangleModel::Draw()
{
	VAO_->bind();
	glDrawElements(GL_TRIANGLES, index_num_, GL_UNSIGNED_INT, 0);
	VAO_->release();
}
