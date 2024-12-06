#pragma once
#ifndef HW_PLANE_VIEW_GL_H
#define HW_PLANE_VIEW_GL_H

#include "hw_plane_gl.h"
#include <QMainWindow>
#include <QMenuBar>
#include <QMenu>
#include <QToolBar>

namespace HW
{
	//class HWPlaneGL;
	
	class HWPlaneViewGL : public QMainWindow
	{
		Q_OBJECT

	public:
		HWPlaneViewGL();
		~HWPlaneViewGL();

		void CreateToolBar();
		void CreatePlaneGLWindow();
		void CreateConnect();

		void ReplacePattern(std::string str, const std::string& old_pattern, const std::string& new_pattern);

	private:
		HWPlaneGL* main_view_;

		QToolBar* top_tool_bar_;
		QAction* load_image_action_;
		QAction* set_add_point_state_;
		QAction* set_add_polygon_point_state_;
		QAction* set_remove_point_state_;
		QAction* move_point_action_;
		QAction* move_picture_action_;
		std::string image_path_name_;

		cv::Mat loaded_image_;

		PlygonEditSate current_state_;

	public slots:
		void LoadImageFromPathAction();
		void AddPntAction();
		void AddPolygonPntAction();
		void MovePictureAction();
		void MovePolygonPntAction();
		void RemovePolygonPntAction();
	};
}

#endif