#include "hw_plane_view_gl.h"
#include <QFileDialog>

namespace HW
{
	HWPlaneViewGL::HWPlaneViewGL()
	{
		this->resize(1200, 900);
		CreateToolBar();
		CreatePlaneGLWindow();
		CreateConnect();
		this->show();
	}

	HWPlaneViewGL::~HWPlaneViewGL()
	{
		if (main_view_ != NULL)
			delete main_view_;
	}

	void HWPlaneViewGL::CreateToolBar()
	{
		top_tool_bar_ = new QToolBar(tr("main tool bar"));
		this->addToolBar(Qt::TopToolBarArea, top_tool_bar_);

		QIcon load_img_icon(".\\..\\images\\ccimg.png");
		load_image_action_ = top_tool_bar_->addAction(load_img_icon, tr("load image"));

		QIcon set_add_pnt_icon(".\\..\\images\\ccSampleCloud.png");
		set_add_point_state_ = top_tool_bar_->addAction(set_add_pnt_icon, tr("add points"));
		set_add_point_state_->setCheckable(true);

		QIcon move_image_icon(".\\..\\images\\mapIcon.png");
		move_picture_action_ = top_tool_bar_->addAction(move_image_icon, tr("move picture"));
		move_picture_action_->setCheckable(true);

		QIcon set_add_polygon_pnt_icon(".\\..\\images\\orthoSections.png");
		set_add_polygon_point_state_ = top_tool_bar_->addAction(set_add_polygon_pnt_icon, tr("add polygon points"));
		set_add_polygon_point_state_->setCheckable(true);

		QIcon set_move_polygon_pnt_icon(".\\..\\images\\ccCloudCloudDistance.png");
		move_point_action_ = top_tool_bar_->addAction(set_move_polygon_pnt_icon, tr("move polygon points"));
		move_point_action_->setCheckable(true);

		QIcon set_remove_polygon_pnt_icon(".\\..\\images\\smallPolygonSelect.png");
		set_remove_point_state_ = top_tool_bar_->addAction(set_remove_polygon_pnt_icon, tr("move polygon points"));
		set_remove_point_state_->setCheckable(true);

		/*QIcon import_images_icon(".\\icons\\ccOpen.png");
		import_images_action_ = top_tool_bar_->addAction(import_images_icon, tr("import images"));

		QIcon find_matches_icon(".\\icons\\ccCloudCloudDistance.png");
		find_matches_action_ = top_tool_bar_->addAction(find_matches_icon, tr("find matches"));

		QIcon save_matches_icon(".\\icons\\ccSave.png");
		save_matches_action_ = top_tool_bar_->addAction(save_matches_icon, tr("save matches"));

		QIcon load_cams_icon(".\\icons\\ccCamera.png");
		load_cams_action_ = top_tool_bar_->addAction(load_cams_icon, tr("load cams"));

		QIcon clear_icon(".\\icons\\icons_clear.png");
		clear_action_ = top_tool_bar_->addAction(clear_icon, tr("clear images"));*/

		/*is_show_cemera_->setCheckable(true);
		is_show_cemera_->setChecked(true);*/

	}

	void HWPlaneViewGL::CreatePlaneGLWindow()
	{
		main_view_ = new HWPlaneGL();
		this->setCentralWidget(main_view_);
		main_view_->setFocusPolicy(Qt::FocusPolicy::ClickFocus);
		main_view_->setWindowTitle("ImageWindow");
		main_view_->resize(1000, 750);
		main_view_->image_window_width_ = main_view_->width();
		main_view_->image_window_height_ = main_view_->height();
		main_view_->show();
	}

	void HWPlaneViewGL::CreateConnect()
	{
		connect(load_image_action_, SIGNAL(triggered()), this, SLOT(LoadImageFromPathAction()));
		connect(set_add_point_state_, SIGNAL(triggered()), this, SLOT(AddPntAction()));
		connect(move_picture_action_, SIGNAL(triggered()), this, SLOT(MovePictureAction()));
		connect(set_add_polygon_point_state_, SIGNAL(triggered()), this, SLOT(AddPolygonPntAction()));
		connect(move_point_action_, SIGNAL(triggered()), this, SLOT(MovePolygonPntAction()));
		connect(set_remove_point_state_, SIGNAL(triggered()), this, SLOT(RemovePolygonPntAction()));
	}

	void HWPlaneViewGL::ReplacePattern(std::string str, const std::string& old_pattern, const std::string& new_pattern)
	{
		while (true)
		{
			std::string::size_type pos(0);
			if ((pos = str.find(old_pattern)) != std::string::npos)
			{
				str.replace(pos, old_pattern.length(), new_pattern);
			}
			else
			{
				break;
			}
		}
	}

	void HWPlaneViewGL::LoadImageFromPathAction()
	{
		std::cout << "select path..." << std::endl;
		QString image_path = QFileDialog::getOpenFileName(this, tr("select images"));
		image_path_name_ = image_path.toStdString();
		ReplacePattern(image_path_name_, "\\", "/");
		std::cout << "the select path is: " << image_path_name_ << std::endl;
		if (image_path_name_.find(".jpg") != std::string::npos
			|| image_path_name_.find(".png") != std::string::npos
			|| image_path_name_.find(".JPG") != std::string::npos
			|| image_path_name_.find(".exr") != std::string::npos)
		{
			cv::Mat image = cv::imread(image_path_name_);
			loaded_image_.create(image.rows, image.cols, image.type());
			loaded_image_ = image;
		}

		if (!loaded_image_.empty())
		{
			main_view_->scale_ = 1.0*main_view_->image_window_width_ / loaded_image_.cols;
			main_view_->SetImage(loaded_image_);
			main_view_->SendFixedImageToMainWidget();
		}
	}

	void HWPlaneViewGL::AddPntAction()
	{
		if (set_add_point_state_->isChecked())
		{
			std::cout << "11" << std::endl;
			current_state_ = PlygonEditSate::kAddPnt;
			main_view_->SetEidtPntState(current_state_);
			//set_add_point_state_->setChecked(false);
		}
		else //if(!set_add_point_state_->isChecked())
		{
			std::cout << "22" << std::endl;
			current_state_ = PlygonEditSate::kPlaneNone;
			main_view_->SetEidtPntState(current_state_);
			//set_add_point_state_->setChecked(true);
		}
	}

	void HWPlaneViewGL::AddPolygonPntAction()
	{
		if (set_add_polygon_point_state_->isChecked())
		{
			std::cout << "11" << std::endl;
			current_state_ = PlygonEditSate::kAddPolygonPnt;
			main_view_->SetEidtPntState(current_state_);
			//set_add_point_state_->setChecked(false);
		}
		else //if(!set_add_point_state_->isChecked())
		{
			std::cout << "22" << std::endl;
			current_state_ = PlygonEditSate::kPlaneNone;
			main_view_->SetEidtPntState(current_state_);
			//set_add_point_state_->setChecked(true);
		}
	}

	void HWPlaneViewGL::MovePictureAction()
	{
		if (move_picture_action_->isChecked())
		{
			std::cout << "11" << std::endl;
			current_state_ = PlygonEditSate::kMoveImage;
			main_view_->SetEidtPntState(current_state_);
		}
		else
		{
			std::cout << "22" << std::endl;
			current_state_ = PlygonEditSate::kPlaneNone;
			main_view_->SetEidtPntState(current_state_);
		}
	}

	void HWPlaneViewGL::MovePolygonPntAction()
	{
		if (move_point_action_->isChecked())
		{
			std::cout << "11" << std::endl;
			current_state_ = PlygonEditSate::kMovePnt;
			main_view_->SetEidtPntState(current_state_);
		}
		else
		{
			std::cout << "22" << std::endl;
			current_state_ = PlygonEditSate::kPlaneNone;
			main_view_->SetEidtPntState(current_state_);
		}
	}

	void HWPlaneViewGL::RemovePolygonPntAction()
	{
		if (set_remove_point_state_->isChecked())
		{
			std::cout << "11" << std::endl;
			current_state_ = PlygonEditSate::kDeletePnt;
			main_view_->SetEidtPntState(current_state_);
		}
		else
		{
			std::cout << "22" << std::endl;
			current_state_ = PlygonEditSate::kPlaneNone;
			main_view_->SetEidtPntState(current_state_);
		}
	}

}