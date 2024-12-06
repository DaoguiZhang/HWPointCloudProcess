#include "hw_plane_gl.h"

namespace HW
{
	HWPlaneGL::HWPlaneGL()
	{
		first_load_ = true;
		fixed_moved_pos_.setX(0.0);
		fixed_moved_pos_.setY(0.0);

		//scaled_width_height_.setX(0.0);
		//scaled_width_height_.setY(0.0);
		//this->setCentralWidget
		//main_widget_ = new QWidget();
		//main_widget_->resize(640, 480);
		//main_widget_->show();
	}

	HWPlaneGL::~HWPlaneGL()
	{
		if (main_widget_ != nullptr)
			delete main_widget_;
		if (image_ != nullptr)
			delete image_;
	}

	void HWPlaneGL::SetImage(cv::Mat& img)
	{
		imported_image_.create(img.rows, img.cols, img.type());
		imported_image_ = img;
		image_window_height_ = scale_ * imported_image_.rows;
		image_ = new QImage(image_window_width_, image_window_height_, QImage::Format_RGB888);
		this->resize(image_window_width_, image_window_height_);
		this->setMaximumSize(image_window_width_, image_window_height_);
		this->setMinimumSize(image_window_width_, image_window_height_);
		this->setScaledContents(true);
		fixed_image_.create(image_window_height_, image_window_width_, imported_image_.type());
		cv::resize(imported_image_, fixed_image_, fixed_image_.size());
		scaled_image_center_pos_.setX(fixed_image_.cols / 2 - 0.5);
		scaled_image_center_pos_.setY(fixed_image_.rows / 2 - 0.5);
		scaled_width_height_.setX(image_window_width_);
		scaled_width_height_.setY(image_window_height_);
	}

	void HWPlaneGL::SendFixedImageToMainWidget()
	{
		cv::Mat resized_image_mat = fixed_image_;
		cv::cvtColor(resized_image_mat, resized_image_mat, CV_RGB2BGR);
		memcpy(image_->bits(), resized_image_mat.data, image_window_width_ * image_window_height_ * 3);
		this->setPixmap(QPixmap::fromImage(image_->scaled(image_window_width_, image_window_height_)));
	}
	
	void HWPlaneGL::SendImageToMainWidget(cv::Mat& image)
	{
		cv::Mat resized_image_mat = image;
		//cv::cvtColor(resized_image_mat, resized_image_mat, CV_RGB2BGR);
		memcpy(image_->bits(), resized_image_mat.data, image_window_width_ * image_window_height_ * 3);
		this->setPixmap(QPixmap::fromImage(image_->scaled(image_window_width_, image_window_height_)));
	}

	void HWPlaneGL::SetEidtPntState(PlygonEditSate& state)
	{
		current_eidt_state_ = state;
	}

	bool HWPlaneGL::HasPolygonPnts()
	{
		return(polygons_pnts_pos_.size() > 0);
	}

	bool HWPlaneGL::HasPnts()
	{
		return (pnts_.size() > 0);
	}

	void HWPlaneGL::mousePressEvent(QMouseEvent* event)
	{
		last_mouse_pos_ = event->pos();
		switch (current_eidt_state_)
		{
		case(PlygonEditSate::kMovePnt):
		{
			std::cout << "start move pnt..." << std::endl;
			QPointF pnt = MapScalePntToFixedPnt(last_mouse_pos_) / scale_;

			std::list<int>::iterator iter = polygons_pnts_idx_.begin();
			float min_dist = 1e7;
			std::list<int>::iterator picked_iter = polygons_pnts_idx_.end();
			//寻找到polygon的顶点
			for (; iter != polygons_pnts_idx_.end(); ++iter)
			{
				QPointF src_pnt = polygons_pnts_pos_[*iter];
				float dist = Compute2PntsDist(src_pnt, pnt);
				if (min_dist > dist)
				{
					min_dist = dist;
					picked_iter = iter;
				}
			}

			if (min_dist < 2 && picked_iter != polygons_pnts_idx_.end())
			{
				picked_pnt_idx = *picked_iter;
			}
			std::cout << "the picked point idx is: " << picked_pnt_idx << std::endl;
		}
		case(PlygonEditSate::kDeletePnt):
		{
			std::cout << "start picked pnt..." << std::endl;
			QPointF pnt = MapScalePntToFixedPnt(last_mouse_pos_) / scale_;

			std::list<int>::iterator iter = polygons_pnts_idx_.begin();
			float min_dist = 1e7;
			std::list<int>::iterator picked_iter = polygons_pnts_idx_.end();
			//寻找到polygon的顶点
			for (; iter != polygons_pnts_idx_.end(); ++iter)
			{
				QPointF src_pnt = polygons_pnts_pos_[*iter];
				float dist = Compute2PntsDist(src_pnt, pnt);
				if (min_dist > dist)
				{
					min_dist = dist;
					picked_iter = iter;
				}
			}

			if (min_dist < 2 && picked_iter != polygons_pnts_idx_.end())
			{
				picked_pnt_idx = *picked_iter;
			}
			std::cout << "the picked point idx is: " << picked_pnt_idx << std::endl;
		}

		}

	}

	void HWPlaneGL::mouseMoveEvent(QMouseEvent* event)
	{
		//获取当前坐标
		if (event->buttons() & Qt::LeftButton)
		{
			QPointF pnt_pos = event->pos();
			switch (current_eidt_state_)
			{
			case(PlygonEditSate::kMoveImage):
			{
				std::cout << "move image..." << std::endl;
				QPointF image_offset = pnt_pos - last_mouse_pos_;
				fixed_moved_pos_ += (0.3*image_offset);
				ComputeOverlapArea();
				update();
				break;
			}
			case(PlygonEditSate::kMovePnt):
			{
				if (picked_pnt_idx >= 0 && picked_pnt_idx < polygons_pnts_pos_.size())
				{
					std::cout << "move pnt..." << std::endl;
					//获取screen屏幕上的顶点
					QPointF fixed_pnt = MapScalePntToFixedPnt(pnt_pos) / scale_;
					polygons_pnts_pos_[picked_pnt_idx] = fixed_pnt;
				}
				update();
			}
			default:
				break;
			}
		}
	}

	void HWPlaneGL::mouseDoubleClickEvent(QMouseEvent* event)
	{
		//在平面中增加顶点

	}
	
	void HWPlaneGL::mouseReleaseEvent(QMouseEvent* event)
	{
		//获取当前坐标
		QPointF pnt_pos = event->pos();
		last_mouse_pos_ = event->pos();

		switch (current_eidt_state_)
		{
		case(PlygonEditSate::kAddPnt):
		{
			std::cout << "add pnt..." << std::endl;
			QPointF add_pnt = MapScalePntToFixedPnt(pnt_pos) / scale_;
			pnts_.emplace_back(add_pnt);
			break;
		}
		case(PlygonEditSate::kDeletePnt):
		{
			std::cout << "delete pnt..." << std::endl;
			std::list<int>::iterator iter;
			iter = std::find(polygons_pnts_idx_.begin(), polygons_pnts_idx_.end(), picked_pnt_idx);
			if (iter != polygons_pnts_idx_.end())
				polygons_pnts_idx_.erase(iter);
			picked_pnt_idx = -1;
			break;
		}
		case(PlygonEditSate::kAddPolygonPnt):
		{
			std::cout << "add polygon pnt..." << std::endl;
			AddPolygonPnt(pnt_pos);
			break;
		}
		case(PlygonEditSate::kMovePnt):
		{
			std::cout << "add polygon pnt..." << std::endl;
			picked_pnt_idx = -1;
			//AddPolygonPnt(pnt_pos);
			break;
		}
		default:
			break;
		}

		update();
	}

	void HWPlaneGL::wheelEvent(QWheelEvent* event)
	{
		//在平面中增加顶点
		if (event->delta() > 0)
		{
			whole_scale_factor_ *= scale_factor_;

			float  modify_height = (float)image_window_height_ / whole_scale_factor_;
			float modify_width = (float)image_window_width_ / whole_scale_factor_;
			scaled_width_height_.setX(modify_width);
			scaled_width_height_.setY(modify_height);
			if (whole_scale_factor_ >= 0.2)
			{
				whole_scale_factor_ /= scale_factor_;
				scaled_width_height_ *= scale_factor_;
			}
		}
		else
		{
			whole_scale_factor_ /= scale_factor_;

			float  modify_height = (float)image_window_height_ / whole_scale_factor_;
			float modify_width = (float)image_window_width_ / whole_scale_factor_;
			scaled_width_height_.setX(modify_width);
			scaled_width_height_.setY(modify_height);
			if (whole_scale_factor_ <= 10)
			{
				whole_scale_factor_ *= scale_factor_;
				scaled_width_height_ /= scale_factor_;
			}
		}
		ComputeOverlapArea();
		update();
	}

	void HWPlaneGL::DrawPolygon(QPainter& painter)
	{
		QPen pen;
		pen.setBrush(Qt::green);
		pen.setWidth(5);
		painter.setPen(pen);
		std::list<int>::iterator iter = polygons_pnts_idx_.begin();
		for (; iter != polygons_pnts_idx_.end(); ++iter)
		{
			if (*iter < polygons_pnts_pos_.size() && *iter >= 0)
			{
				std::cout << "idx: " << *iter << std::endl;
				QPointF screen_pnt = MapFixedPntToScalePnt(polygons_pnts_pos_[*iter] * scale_);
				painter.drawPoint(screen_pnt);
			}
		}

		//画polygon的线
		if (polygons_pnts_idx_.size() < 2)
			return;
		painter.setOpacity(0.5);
		pen.setBrush(Qt::blue);
		pen.setWidth(3);
		painter.setPen(pen);
		std::list<int>::iterator line_iter_start = polygons_pnts_idx_.begin();
		for (; line_iter_start != polygons_pnts_idx_.end(); ++line_iter_start)
		{
			std::list<int>::iterator line_iter_end = ++line_iter_start;
			if (line_iter_end == polygons_pnts_idx_.end())
			{
				line_iter_end = polygons_pnts_idx_.begin();
			}
			--line_iter_start;
			QPointF pnt_start = MapFixedPntToScalePnt(polygons_pnts_pos_[*line_iter_start] * scale_);
			QPointF pnt_end = MapFixedPntToScalePnt(polygons_pnts_pos_[*line_iter_end] * scale_);
			painter.drawLine(pnt_start, pnt_end);
		}
	}

	void HWPlaneGL::DrawPnts(QPainter& painter)
	{
		QPen pen;
		painter.setOpacity(1.0);
		pen.setWidth(5);
		pen.setBrush(Qt::white);
		painter.setPen(pen);
		for (int i = 0; i < pnts_.size(); ++i)
		{
			QPointF screen_pnt = MapFixedPntToScalePnt(pnts_[i] * scale_);
			painter.drawPoint(screen_pnt);
		}
	}

	void HWPlaneGL::paintEvent(QPaintEvent *event)
	{
		//画背景图片
		//ComputeOverlapArea();

		//画polygon和顶点
		QLabel::paintEvent(event);
		QPainter painter(this);
		
		if (HasPolygonPnts())
		{
			DrawPolygon(painter);
		}

		if (HasPnts())
		{
			DrawPnts(painter);
		}
	}

	void HWPlaneGL::ComputeOverlapArea()
	{
		std::cout << "image_window_width_, image_window_height_: " << image_window_width_ << " " << image_window_height_ << std::endl;
		cv::Mat resized_widget_mat = cv::Mat::zeros(image_window_height_, image_window_width_, imported_image_.type());

		for (int i = 0; i < image_window_height_; ++i)
		{
			for (int j = 0; j < image_window_width_; ++j)
			{
				QPointF scaled_pnt_pos((float)j, (float)i);
				QPointF fixed_pnt_pos = MapScalePntToFixedPnt(scaled_pnt_pos);
				if (fixed_pnt_pos.x() >= 0 &&
					fixed_pnt_pos.y() >= 0 &&
					fixed_pnt_pos.x() < image_window_width_ - 1&&
					fixed_pnt_pos.y() < image_window_height_ - 1)
				{
					//图片的边界点不作插值
					if (fixed_pnt_pos.x() < 1e-6 || fixed_pnt_pos.y() < 1e-6)
					{
						//在边界就不双线性插值
						resized_widget_mat.at<cv::Vec3b>(i, j) = fixed_image_.at<cv::Vec3b>((int)(fixed_pnt_pos.y() + 0.05), (int)(fixed_pnt_pos.x() + 0.05));
					}
					else
					{
						cv::Vec3b temp_color;
						QPoint u((int)(fixed_pnt_pos.x()), (int)(fixed_pnt_pos.y()));
						//QPoint v(u.x() + 1, u.y() + 1);
						double coef_00 = 1.0;
						double coef_10 = 0.0;
						double coef_01 = 0.0;
						double coef_11 = 0.0;
						CalcBilinearCoefs(coef_00, coef_10, coef_01, coef_11, fixed_pnt_pos.x(), fixed_pnt_pos.y(), u.x(), u.y(), u.x() + 1, u.y() + 1);
						//
						temp_color = coef_00 * fixed_image_.at<cv::Vec3b>(u.y(), u.x()) + coef_10 * fixed_image_.at<cv::Vec3b>(u.y(), u.x() + 1)
							+ coef_01 * fixed_image_.at<cv::Vec3b>(u.y() + 1, u.x()) + coef_11 * fixed_image_.at<cv::Vec3b>(u.y() + 1, u.x() + 1);

						resized_widget_mat.at<cv::Vec3b>(i, j) = temp_color;
					}
				}
			}
		}

		SendImageToMainWidget(resized_widget_mat);
	}

	QPointF HWPlaneGL::MapScalePntToFixedPnt(QPointF pnt_pos)
	{
		float scale = scaled_width_height_.x() / image_window_width_;
		QPointF point = pnt_pos / scale;
		return (point - fixed_moved_pos_);
	}

	QPointF HWPlaneGL::MapFixedPntToScalePnt(QPointF pnt_pos)
	{
		float scale = scaled_width_height_.x() / image_window_width_;
		QPointF moved_pnt = pnt_pos + fixed_moved_pos_;
		return (moved_pnt * scale);
	}

	void HWPlaneGL::CalcBilinearCoefs(double &coef_00, double &coef_10, double &coef_01, double &coef_11,
		float x, float y, float x_0, float y_0, float x_1, float y_1)
	{
		float norm_coef_x = 1 / (x_1 - x_0);
		float norm_coef_y = 1 / (y_1 - y_0);
		float coef_x_0 = (x_1 - x) * norm_coef_x;
		float coef_y_0 = (y_1 - y) * norm_coef_y;
		coef_00 = coef_x_0 * coef_y_0;
		coef_10 = (1 - coef_x_0) * coef_y_0;
		coef_01 = coef_x_0 * (1 - coef_y_0);
		coef_11 = (1 - coef_x_0) * (1 - coef_y_0);

	}

	//手动polygon顶点增加
	void HWPlaneGL::AddPolygonPnt(QPointF& screen_pnt)
	{
		//----------------------------获取边，并且得到到函数，然后加入离最近的边--------------------------------//
		//转化为当前坐标系
		QPointF add_pnt = MapScalePntToFixedPnt(screen_pnt) / scale_;

		if (polygons_pnts_pos_.size() < 3)
		{
			polygons_pnts_pos_.emplace_back(add_pnt);

			int icount = polygons_pnts_pos_.size() - 1;
			polygons_pnts_idx_.emplace_back(icount);
			return;
		}

		float min_dist = 1e7;
		std::list<int>::iterator min_iter = polygons_pnts_idx_.begin();
		std::list<int>::iterator iter = polygons_pnts_idx_.begin();
		for (; iter != polygons_pnts_idx_.end(); ++iter)
		{
			std::list<int>::iterator next_iter = ++iter;
			if (next_iter == polygons_pnts_idx_.end())
			{
				next_iter = polygons_pnts_idx_.begin();
			}
			--iter;
			QPointF a = polygons_pnts_pos_[*iter];
			QPointF b = polygons_pnts_pos_[*next_iter];
		
			//计算add_pnt 到ab线段上的距离
			float dist = ComputePntToLineDist(a, b, add_pnt);
			if (dist < min_dist)
			{
				min_dist = dist;
				min_iter = iter;
			}
		}

		std::cout << "the min dist is: " << min_dist << std::endl;
		//std::cout << "asdfsdf" << std::endl;
		//system("pause");
		if (min_dist < 3)
		{
			polygons_pnts_pos_.emplace_back(add_pnt);
			//增加一个元素
			int add_idx = polygons_pnts_pos_.size() - 1;
			polygons_pnts_idx_.insert(++min_iter, add_idx);
			/*if (min_iter != polygons_pnts_idx_.begin())
				polygons_pnts_idx_.insert(++min_iter, add_idx);
			else
				polygons_pnts_idx_.emplace_back(add_idx);*/
		}
		//--------------------------------------------end---------------------------------------------------//
	}

	void HWPlaneGL::LoadPolygonPnts(const std::vector<float2>& polygon_pnts)
	{
		for (int i = 0; i < polygon_pnts.size(); ++i)
		{
			polygons_pnts_pos_.emplace_back(QPointF(polygon_pnts[i].x, polygon_pnts[i].y));
			polygons_pnts_idx_.emplace_back(i);
		}
	}

	bool HWPlaneGL::CheckPntInPolygon(QPointF& pnt)
	{
		//顶点
		return false;
	}

	bool HWPlaneGL::CheckOnSegmentLine(QPointF& a, QPointF& b, QPointF& pnt)
	{
		//线段和x轴平行
		if (std::abs(a.x() - b.x()) < 1e-4)
		{
			if (std::abs(a.x() - pnt.x()) < 1e-4 && 
				std::min(a.y(), b.y()) <= pnt.y() 
				&& std::max(a.y(), b.y()) >= pnt.y())
			{
				return true;
			}
		}
		else if(std::abs(a.y() - b.y()) < 1e-4)	//线段和y轴平行
		{
			if (std::abs(a.y() - pnt.y()) < 1e-4 &&
				std::min(a.x(), b.x()) <= pnt.x()
				&& std::max(a.x(), b.x()) >= pnt.x())
			{
				return true;
			}
		}
		else
		{
			//(pnt - a) 叉乘 (a - b) = 0
			//if()
			return false;
		}
		return false;
	}

	float HWPlaneGL::Compute2PntsDist(QPointF a, QPointF b)
	{
		float dist = std::sqrtf((a - b).x()*(a - b).x() + (a - b).y()*(a - b).y());
		return dist;
	}

	float HWPlaneGL::ComputePntToLineDist(QPointF a, QPointF b, QPointF c)
	{
		QPointF ab = b - a;
		QPointF ac = c - a;
		float f = QPointF::dotProduct(ab,ac);
		if (f<0) return Compute2PntsDist(c, a);//C1处的点
		float d = QPointF::dotProduct(ab, ab);
		if (f>d) return Compute2PntsDist(c, b);//C2处的点，d=f*cos（theta）
		f = f / d;
		QPointF D = a + f * ab; // c在ab线段上的投影点

		return Compute2PntsDist(c, D);
	}
}