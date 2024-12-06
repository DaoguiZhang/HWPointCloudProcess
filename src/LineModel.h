#pragma once
#include "BaseModel.h"
class LineModel : public BaseModel
{
public:
	LineModel(QGLWidget *_openglWindow);
	~LineModel();
	void Draw() override;
};