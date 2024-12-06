#pragma once
#include <QGLWidget>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QGLShaderProgram>
#include "math_utils.h"

class BaseModel {
public:
	BaseModel(QGLWidget *_openglWindow);
	~BaseModel();
	void InitBuffer(QGLWidget *_openglWindow);
	void SetProgram(QGLShaderProgram* shader_program);
	void SetVertexPos(const std::vector<float>& pos_vec);
	void SetVertexPos(const std::vector<float3>& pos_vec);	
	void SetVertexColor(const std::vector<float>& color_vec);
	void SetVertexColor(const std::vector<float3>& color_vec);
	void SetIndex(const std::vector<int>& index_vec);
	void SetIndex(const std::vector<int3>& index_vec);
	void UpdateVertexPos(const std::vector<float>& pos_vec);
	void UpdateVertexPos(const std::vector<float3>& pos_vec);
	void UpdateVertexColor(const std::vector<float>& color_vec);
	void UpdateVertexColor(const std::vector<float3>& color_vec);
	void UpdataIndex(const std::vector<int>& index_vec);
	void UpdataIndex(const std::vector<int3>& index_vec);

	virtual void Draw() {};

protected:
	QOpenGLVertexArrayObject* VAO_;
	QOpenGLBuffer* vertex_pos_buffer_;
	QOpenGLBuffer* vertex_color_buffer_;
	QOpenGLBuffer* index_buffer_;
	QGLShaderProgram* shader_program_;
	int index_num_;
	int vertex_num_;
};