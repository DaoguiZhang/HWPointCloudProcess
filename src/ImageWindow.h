#pragma once
#include <QWidget>
#include <QImage>
#include <QLabel>
#include <QPaintEvent>
#include <QPen>
#include <QPainter>
#include <QpointF>
#include <QMouseEvent>
#include <QFileDialog>
#include <iostream>
#include <Eigen/Dense>
#include "math_utils.h"

class MyLabel :public QLabel {
	Q_OBJECT
public:
	MyLabel(QWidget *parent = 0);
	void paintEvent(QPaintEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	QPointF start;
	QPointF end;
	Eigen::Matrix4f camera_pose_;
	Eigen::Matrix3f intr_;
	float4 coeff_;
	Eigen::Vector3f e1, e2;
};
class ImageWindow :public QWidget {
	Q_OBJECT
public:
	ImageWindow(QWidget *parent = 0);
	~ImageWindow();
	MyLabel *imageLabel_;
	Eigen::Matrix4f ext_;
	Eigen::Matrix4f camera_pose_;
	Eigen::Matrix3f intr_;
	float flen_;
public slots:
	void loadImage();
};