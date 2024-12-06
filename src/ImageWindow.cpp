#include<fstream>
#include "ImageWindow.h"

MyLabel::MyLabel(QWidget *parent)
	:
	QLabel(parent)
{
	start = QPointF(-1, -1);
	end = QPointF(-1, -1);
}

void MyLabel::paintEvent(QPaintEvent * event)
{
	QLabel::paintEvent(event);
	QPainter painter(this);
	QPen pen;
	pen.setWidth(2);
	painter.setPen(pen);
	pen.setBrush(Qt::red);
	if (start.x() != -1) {
		
		painter.drawEllipse(start, 4, 4);
	}
	if (end.x() != -1) {
		//pen.setBrush(Qt::green);
		painter.drawEllipse(end, 4, 4);
	}
	update();
}

void MyLabel::mouseReleaseEvent(QMouseEvent * event)
{
	if (event->modifiers() != Qt::ControlModifier) {
		start = event->pos();
		std::cout << "start: " << start.x() << " " << start.y() << "\n";
		Eigen::Vector3f s(start.x(), start.y(), 1);
		Eigen::Vector3f s_c = intr_.inverse()*s;
		std::cout << "s_c: " << s_c.transpose() << "\n";
		e1 = camera_pose_.topLeftCorner(3, 3)*s_c;
		std::cout << "e1: " << e1.transpose() << "\n";
	}
	else {
		end = event->pos();
		Eigen::Vector3f e(end.x(), end.y(), 1);
		Eigen::Vector3f e_c = intr_.inverse()*e;
		std::cout << "e_c: " << e_c.transpose() << "\n";
		e2 = camera_pose_.topLeftCorner(3, 3)*e_c;
		std::cout << "e2: " << e2.transpose() << "\n";
	}
	if (start.x() != -1 && end.x() != -1) {
		Eigen::Vector3f normal = e1.cross(e2);
		normal.normalize();
		std::cout << "normal: " << normal.transpose() << "\n";
		coeff_.x = normal(0);
		coeff_.y = normal(1);
		coeff_.z = normal(2);
		Eigen::Vector3f pos = camera_pose_.topRightCorner(3, 1);
		coeff_.w = -normal.dot(pos);
	}
	update();
}

ImageWindow::ImageWindow(QWidget *parent)
	:QWidget(parent)
{
	imageLabel_=new MyLabel(this);
}

ImageWindow::~ImageWindow()
{
	if (imageLabel_ != nullptr) {
		delete imageLabel_;
		imageLabel_ = NULL;
	}
}

void ImageWindow::loadImage() {
	QString fileName = QFileDialog::getOpenFileName(this, tr("Import Image"), ".", "Image Files(*.jpg *.png *.bmp *.pgm *.pbm)", 0);
	std::string file = fileName.toStdString();
	std::string camera_head = file.substr(0, file.find_last_of("."));
	std::string camera_path = camera_head + ".CAM";
	std::ifstream outFile(camera_path);
	outFile >> ext_(0, 3) >> ext_(1, 3) >> ext_(2, 3);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			outFile >> ext_(i, j);
		}
	}
	outFile >> flen_;
	outFile.close();
	ext_.row(3) = Eigen::RowVector4f(0, 0, 0, 1);
	camera_pose_ = ext_.inverse();
	QImage image;
	if (fileName != "") {
		if (image.load(fileName)) {
			flen_ = flen_*std::max(image.height(), image.width());
			intr_.setIdentity();
			intr_(0, 0) = flen_;
			intr_(0, 2) = 0.5*image.width();
			intr_(1, 1) = flen_;
			intr_(1, 2) = 0.5*image.height();
			imageLabel_->camera_pose_ = camera_pose_;
			imageLabel_->intr_ = intr_;
			imageLabel_->resize(image.size());
			imageLabel_->setPixmap(QPixmap::fromImage(image));
		}
	}
}