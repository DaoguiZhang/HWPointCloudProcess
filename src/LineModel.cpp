#include "LineModel.h"

LineModel::LineModel(QGLWidget * _openglWindow)
	:BaseModel(_openglWindow)
{
}

LineModel::~LineModel()
{
}

void LineModel::Draw()
{
	VAO_->bind();
	glDrawElements(GL_LINES, index_num_, GL_UNSIGNED_INT, 0);
	VAO_->release();
}
