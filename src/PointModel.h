#pragma once
#include "BaseModel.h"
class PointModel : public BaseModel
{
public:
	PointModel(QGLWidget *_openglWindow);
	~PointModel();
	void SetPointSize(float pz);
	void Draw() override;

private:
	float point_size_;	//点云的点的大小
};