#pragma once
#ifndef HW_PLANE_GL_H
#define HW_PLANE_GL_H

#include <QWidget>
#include <QImage>
#include <QLabel>
#include <QPaintEvent>
#include <QPen>
#include <QPainter>
#include <QpointF>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QScrollArea>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>
#include <Eigen/Eigen>
#include <QVector3D>
#include <QOpenGLWidget>
#include <QMouseEvent>

#include "hw_plane.h"

namespace HW
{
	enum PlygonEditSate
	{
		kPlaneNone = 0,
		kAddPnt,
		kDeletePnt,
		kMovePnt,
		kMoveImage,
		kAddPolygonPnt
	};

	class HWPlaneGL: public QLabel//, public QOpenGLWidget
	{
		Q_OBJECT

	public:
		HWPlaneGL();
		~HWPlaneGL();

		void SetImage(cv::Mat& img);
		void SendFixedImageToMainWidget();

		void SendImageToMainWidget(cv::Mat& image);

		//�༭polygon
		void SetEidtPntState(PlygonEditSate& state);

		bool HasPolygonPnts();
		bool HasPnts();

		int image_window_width_;
		int image_window_height_;
		float scale_;

		float scale_factor_ = 0.9;
		float whole_scale_factor_ = 1.0;

		QPointF scaled_width_height_;

	protected:
		//events handling, ��������¼�
		void mousePressEvent(QMouseEvent* event);
		void mouseMoveEvent(QMouseEvent* event);
		void mouseDoubleClickEvent(QMouseEvent* event);
		void mouseReleaseEvent(QMouseEvent* event);
		void wheelEvent(QWheelEvent* event);
		
		//����polygon
		void DrawPolygon(QPainter& painter);
		void DrawPnts(QPainter& painter);
		void paintEvent(QPaintEvent *event);
		void ComputeOverlapArea();
		QPointF MapScalePntToFixedPnt(QPointF pnt_pos);
		QPointF MapFixedPntToScalePnt(QPointF pnt_pos);
		void CalcBilinearCoefs(double &coef_00, double &coef_10, double &coef_01, double &coef_11,
			float x, float y, float x_0, float y_0, float x_1, float y_1);
		void AddPolygonPnt(QPointF& screen_pnt);

		//����polygon���㣬���polygonһ���ǰ���˳�������еģ������γ�polygon
		void LoadPolygonPnts(const std::vector<float2>& polygon_pnts);

		//�ж϶����Ƿ���polygon����
		bool CheckPntInPolygon(QPointF& pnt);

		bool CheckOnSegmentLine(QPointF& a, QPointF& b, QPointF& pnt);

	private:

		//��������֮��ľ���
		float Compute2PntsDist(QPointF a, QPointF b);
		float ComputePntToLineDist(QPointF a, QPointF b, QPointF c);

		//��ǰ�༭״̬
		PlygonEditSate current_eidt_state_;

		QPointF scaled_image_center_pos_;

		//scaleͼƬ��С��fixedͼƬ��С����£�ͼƬ��λ��
		QPointF fixed_moved_pos_;

		//polygon�����ʰȡ
		int picked_pnt_idx = -1;

		//����¼�
		bool mouse_moved_;
		bool mouse_button_pressed_;
		bool ignore_mouse_release_event_;
		bool first_load_;

		//! Last mouse position
		QPointF last_mouse_pos_;

		QWidget* main_widget_;
		QImage* image_;

		cv::Mat imported_image_;
		cv::Mat fixed_image_;

		std::vector<QPointF> pnts_;

		std::vector<QPointF> polygons_pnts_pos_;
		std::list<int> polygons_pnts_idx_;
	};
}

#endif