#pragma once
#include "BaseModel.h"
class TriangleModel : public BaseModel
{
public:
	TriangleModel(QGLWidget *_openglWindow);
	~TriangleModel();
	void Draw() override;
};