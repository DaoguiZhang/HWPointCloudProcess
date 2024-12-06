#include "BaseModel.h"
#include<iostream>

BaseModel::BaseModel(QGLWidget * _openglWindow)
{
	InitBuffer(_openglWindow);
}

BaseModel::~BaseModel()
{
	if(VAO_!=nullptr) VAO_->destroy();
	if (vertex_pos_buffer_ != nullptr) vertex_pos_buffer_->destroy();
	if (vertex_color_buffer_ != nullptr) vertex_color_buffer_->destroy();
	if (index_buffer_ != nullptr) index_buffer_->destroy();
}

void BaseModel::InitBuffer(QGLWidget * _openglWindow)
{
	VAO_ = new QOpenGLVertexArrayObject(_openglWindow);
	VAO_->create();
	vertex_pos_buffer_= new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
	vertex_pos_buffer_->create();
	vertex_color_buffer_= new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
	vertex_color_buffer_->create();
	index_buffer_= new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
	index_buffer_->create();
}

void BaseModel::SetProgram(QGLShaderProgram * shader_program)
{
	shader_program_ = shader_program;
}

void BaseModel::SetVertexPos(const std::vector<float>& pos_vec)
{
	VAO_->bind();
	vertex_pos_buffer_->bind();
	vertex_pos_buffer_->setUsagePattern(QOpenGLBuffer::StaticDraw);
	vertex_pos_buffer_->allocate(pos_vec.data(), pos_vec.size() * sizeof(float));
	shader_program_->enableAttributeArray(0);
	shader_program_->setAttributeBuffer(0, GL_FLOAT, 0, 3, 3 * sizeof(float));
	vertex_pos_buffer_->release();
	VAO_->release();
	vertex_num_ = pos_vec.size() / 3;
}
void BaseModel::SetVertexPos(const std::vector<float3>& pos_vec)
{
	VAO_->bind();
	vertex_pos_buffer_->bind();
	vertex_pos_buffer_->setUsagePattern(QOpenGLBuffer::StaticDraw);
	vertex_pos_buffer_->allocate(pos_vec.data(), pos_vec.size() * sizeof(float3));
	shader_program_->enableAttributeArray(0);
	shader_program_->setAttributeBuffer(0, GL_FLOAT, 0, 3, 3 * sizeof(float));
	vertex_pos_buffer_->release();
	VAO_->release();
	vertex_num_ = pos_vec.size();
}

void BaseModel::SetVertexColor(const std::vector<float>& color_vec)
{
	VAO_->bind();
	vertex_color_buffer_->bind();
	vertex_color_buffer_->setUsagePattern(QOpenGLBuffer::StaticDraw);
	vertex_color_buffer_->allocate(color_vec.data(), color_vec.size() * sizeof(float));
	shader_program_->enableAttributeArray(1);
	shader_program_->setAttributeBuffer(1, GL_FLOAT, 0, 3, 3 * sizeof(float));
	vertex_color_buffer_->release();
	VAO_->release();
}
void BaseModel::SetVertexColor(const std::vector<float3>& color_vec)
{
	VAO_->bind();
	vertex_color_buffer_->bind();
	vertex_color_buffer_->setUsagePattern(QOpenGLBuffer::StaticDraw);
	vertex_color_buffer_->allocate(color_vec.data(), color_vec.size() * sizeof(float3));
	shader_program_->enableAttributeArray(1);
	shader_program_->setAttributeBuffer(1, GL_FLOAT, 0, 3, 3 * sizeof(float));
	vertex_pos_buffer_->release();
	
}

void BaseModel::SetIndex(const std::vector<int>& index_vec)
{
	VAO_->bind();
	index_buffer_->bind();
	index_buffer_->setUsagePattern(QOpenGLBuffer::StaticDraw);
	index_buffer_->allocate(index_vec.data(), index_vec.size() * sizeof(int));
	VAO_->release();
	index_num_ = index_vec.size();
}
void BaseModel::SetIndex(const std::vector<int3>& index_vec)
{
	VAO_->bind();
	index_buffer_->bind();
	index_buffer_->setUsagePattern(QOpenGLBuffer::StaticDraw);
	index_buffer_->allocate(index_vec.data(), index_vec.size() * sizeof(int3));
	VAO_->release();
	index_num_ = index_vec.size() * 3;
}

void BaseModel::UpdateVertexPos(const std::vector<float>& pos_vec)
{
	vertex_pos_buffer_->bind();
	vertex_pos_buffer_->allocate(pos_vec.data(), pos_vec.size() * sizeof(float));
	vertex_pos_buffer_->release();
	vertex_num_ = pos_vec.size() / 3;
}

void BaseModel::UpdateVertexPos(const std::vector<float3>& pos_vec)
{
	vertex_pos_buffer_->bind();
	vertex_pos_buffer_->allocate(pos_vec.data(), pos_vec.size() * sizeof(float3));
	vertex_pos_buffer_->release();
	vertex_num_ = pos_vec.size();
}

void BaseModel::UpdateVertexColor(const std::vector<float>& color_vec)
{
	vertex_color_buffer_->bind();
	vertex_color_buffer_->allocate(color_vec.data(), color_vec.size() * sizeof(float));
	vertex_color_buffer_->release();
}

void BaseModel::UpdateVertexColor(const std::vector<float3>& color_vec)
{
	vertex_color_buffer_->bind();
	vertex_color_buffer_->allocate(color_vec.data(), color_vec.size() * sizeof(float3));
	vertex_color_buffer_->release();
}

void BaseModel::UpdataIndex(const std::vector<int>& index_vec)
{
	index_buffer_->bind();
	index_buffer_->allocate(index_vec.data(), index_vec.size() * sizeof(int));
	index_num_ = index_vec.size();
}

void BaseModel::UpdataIndex(const std::vector<int3>& index_vec)
{
	index_buffer_->bind();
	index_buffer_->allocate(index_vec.data(), index_vec.size() * sizeof(int3));
	index_num_ = index_vec.size() * 3;
}
