#include"hw_bundle.h"

namespace HW {

	HWBundle::HWBundle()
	{
		has_aligned_ = false;
	}

	HWBundle::~HWBundle()
	{
		if (bundle_cams_)
		{
			delete bundle_cams_;
			bundle_cams_ = NULL;
		}
		
		if (cams_)
		{
			delete cams_;
			cams_ = NULL;
		}

		if (align_cams_)
		{
			delete align_cams_;
			align_cams_ = NULL;
		}
	}

	void HWBundle::SetDir(std::string& b_dir)
	{
		b_dir_ = b_dir;
	}

	void HWBundle::LoadDataFromRealityCaptureDataDir()
	{
		if (b_dir_.empty())
		{
			std::cerr << "b_dir_ empty..." << std::endl;
			return;
		}
		//std::cerr << "111111111111" << std::endl;
		LoadDataFromRealityCaptureData(b_dir_);
	}

	void HWBundle::LoadDataFromRealityCaptureData(const std::string& bundle_dir)
	{
		/*if (bundle_cams_ != NULL)
		{
			delete bundle_cams_;
			bundle_cams_ = NULL;
		}*/
		ModelCameras* bundle_cams = new ModelCameras();
		bundle_cams_ = bundle_cams;
		//读取图片
		std::string imgs_dir = bundle_dir;
		std::cerr << "LoadDataFromRealityCaptureData imgs_dir " << imgs_dir << std::endl;
		bundle_cams_->LoadImagesFromRealityCaputureImgsdir(imgs_dir);
		
		//读取bundle文件
		std::vector<std::string> c_files;
		bundle_cams_->GetCurrentDirFiles(bundle_dir, c_files);
		std::string bundle_path;
		std::string align_matrix_path;
		for (int i = 0; i < c_files.size(); ++i)
		{
			if (c_files[i].find(".out") != std::string::npos)
			{
				bundle_path = c_files[i];
			}
			if (c_files[i].find(".cam") != std::string::npos)
			{
				align_matrix_path = c_files[i];
			}
		}
		if (!bundle_path.empty())
		{
			ParseBundleFileFromRealityCapture(bundle_path);
		}
		if (!align_matrix_path.empty())
		{
			LoadAlignMatrixFromCloudCampair(align_matrix_path);
		}
	}

	void HWBundle::ParseBundleFileFromRealityCapture(const std::string& bundle_path)
	{
		//解析bundler文件
		std::cerr << "start to parse the bundle file... " << std::endl;
		std::ifstream bundlerFile(bundle_path);
		//std::cout << "bundlerFile: " << (img_dir + "\\proj.out").c_str() << std::endl;
		if (!bundlerFile.is_open())
		{
			std::cout << "can not open the bundler file!" << std::endl;
		}
		std::string line_buffer;
		//获得第一行
		std::getline(bundlerFile, line_buffer);
		if (!line_buffer.find("v0.3"))
		{
			std::cout << "the file format is wrong!" << std::endl;
		}
		//获得第二行
		std::getline(bundlerFile, line_buffer);
		int cams_num = 0;	//相机数量
		int featurePoints_num;	//特征点数量
		std::sscanf(line_buffer.c_str(), "%d %d", &cams_num, &featurePoints_num);
		int img_num = bundle_cams_->GetModelCamerasNum();
		std::cout << "cameram number: " << cams_num << "    " << "feature points number: " << featurePoints_num << std::endl;
		if (cams_num != img_num)
		{
			cams_num = std::min(img_num, cams_num);
			std::cerr << "error: broken bundle files..." << std::endl;
			return;
		}
		for (int cam_idx = 0; cam_idx < cams_num; ++cam_idx)
		{
			//test
			//std::cout << "cam_idx: " << cam_idx << std::endl;
			//end test
			//获取相机参数
			float f;
			float k[2];
			Eigen::Matrix3f R;
			Eigen::Vector3f t;
			int img_width = 0;
			int img_height = 0;
			//获得相机对应的图片
			if (bundle_cams_)
			{
				if (!bundle_cams_->GetAllCamsImgsWidthAndHeight().empty())
				{
					std::cout << "color_image_wh_vec[cam_idx][0]: " << bundle_cams_->GetAllCamsImgsWidthAndHeight()[cam_idx][0] << ", " << "color_image_wh_vec[cam_idx][1]: " <<
						bundle_cams_->GetAllCamsImgsWidthAndHeight()[cam_idx][1] << std::endl;
					img_width = bundle_cams_->GetAllCamsImgsWidthAndHeight()[cam_idx][0];
					img_height = bundle_cams_->GetAllCamsImgsWidthAndHeight()[cam_idx][1];
				}
			}

			//focal lens
			std::getline(bundlerFile, line_buffer);
			sscanf(line_buffer.c_str(), "%f %f %f\n", &f, &k[0], &k[1]);

			//rotation
			std::getline(bundlerFile, line_buffer);
			sscanf(line_buffer.c_str(), "%f %f %f", &R(0,0), &R(0, 1), &R(0, 2));
			std::getline(bundlerFile, line_buffer);
			sscanf(line_buffer.c_str(), "%f %f %f", &R(1, 0), &R(1, 1), &R(1, 2));
			std::getline(bundlerFile, line_buffer);
			sscanf(line_buffer.c_str(), "%f %f %f", &R(2, 0), &R(2, 1), &R(2, 2));
			std::getline(bundlerFile, line_buffer);
			sscanf(line_buffer.c_str(), "%f %f %f", &t[0], &t[1], &t[2]);
			/*float3 R_Matrix[3];
			std::getline(bundlerFile, line_buffer);
			sscanf(line_buffer.c_str(), "%f %f %f", &R_Matrix[0].x, &R_Matrix[0].y, &R_Matrix[0].z);
			std::getline(bundlerFile, line_buffer);
			sscanf(line_buffer.c_str(), "%f %f %f", &R_Matrix[1].x, &R_Matrix[1].y, &R_Matrix[1].z);
			std::getline(bundlerFile, line_buffer);
			sscanf(line_buffer.c_str(), "%f %f %f", &R_Matrix[2].x, &R_Matrix[2].y, &R_Matrix[2].z);
			R << R_Matrix[0].x, R_Matrix[0].y, R_Matrix[0].z, R_Matrix[1].x, R_Matrix[1].y, R_Matrix[1].z, R_Matrix[2].x, R_Matrix[2].y, R_Matrix[2].z;*/

			////translation
			//float3 T_Matrix;
			//std::getline(bundlerFile, line_buffer);
			//sscanf(line_buffer.c_str(), "%f %f %f", &T_Matrix.x, &T_Matrix.y, &T_Matrix.z);
			//t << T_Matrix.x, T_Matrix.y, T_Matrix.z;
			
			//需要写一个拷贝构造函数

			//外参
			CameraModel tempCam;
			tempCam.fx_ = f;
			tempCam.fy_ = f;
			tempCam.distortion_.emplace_back(k[0]);
			tempCam.distortion_.emplace_back(k[1]);
			/*tempCam.cx_ = (float)(img_width - 1) / 2.0;
			tempCam.cy_ = (float)(img_height - 1) / 2.0;*/
			tempCam.cx_ = (float)img_width / 2.0;
			tempCam.cy_ = (float)img_height / 2.0;
			Eigen::Matrix4f cam_extr;
			cam_extr.topLeftCorner(3, 3) = R;
			cam_extr.topRightCorner(3, 1) = t;
			cam_extr(3, 0) = 0.0f;
			cam_extr(3, 1) = 0.0f;
			cam_extr(3, 2) = 0.0f;
			cam_extr(3, 3) = 1.0f;
			//std::cerr << "cam_extr: \n" << cam_extr << std::endl;
			//convert bundle cam to tranditional cam
			tempCam.cam_pose_ = cam_extr.inverse();
			bundle_cams_->SetCamModel(cam_idx, tempCam);
		}

		for (int point_idx = 0; point_idx < featurePoints_num && !bundlerFile.eof(); ++point_idx)
		{
			BundlerPoint3d point3d_buffer;

			std::getline(bundlerFile, line_buffer);
			std::sscanf(line_buffer.c_str(), "%f %f %f", &point3d_buffer.pnt3d_pos[0], &point3d_buffer.pnt3d_pos[1], &point3d_buffer.pnt3d_pos[2]);
			std::getline(bundlerFile, line_buffer);
			std::sscanf(line_buffer.c_str(), "%d %d %d", &point3d_buffer.pnt3d_color[0], &point3d_buffer.pnt3d_color[1], &point3d_buffer.pnt3d_color[2]);
			std::getline(bundlerFile, line_buffer);
			int view_num = 0;
			//读取这一行数据，把这行数据按照空格分成几组。使用新的函数替代一下, new----
			//读取bundler文件的三维顶点
			//std::string filter_pattern = " ";
			std::vector<std::string> split_strs;
			SplitStr(split_strs, line_buffer);
			view_num = std::stoi(split_strs[0]);
			point3d_buffer.views_list_num = view_num;
			//std::cout << "view_num: " << view_num << std::endl;
			//test
			if (view_num > cams_num || view_num <= 0)
			{
				std::cout << "read bundler view_num wrong!" << std::endl;
			}
			//end test
			for (int i = 0; i < view_num && i < cams_num; ++i)
			{
				BundlerPoint2d point2d_buffer;
				point2d_buffer.cam_idx = std::stoi(split_strs[4 * i + 1]);

				//std::cout << "point2d_buffer.cam_idx: " << point2d_buffer.cam_idx << std::endl;
				////test
				//if (point2d_buffer.cam_idx > cams_num || point2d_buffer.cam_idx < 0)
				//{
				//	std::cout << "read wrong!" << std::endl;
				//	return false;
				//}
				////end test

				point2d_buffer.feature_idx = std::stoi(split_strs[4 * i + 2]);
				point2d_buffer.x = std::stof(split_strs[4 * i + 3]);
				point2d_buffer.y = std::stof(split_strs[4 * i + 4]);

				point3d_buffer.views_list.push_back(point2d_buffer);
			}
			bundle_pnts_3d_.push_back(point3d_buffer);
		}
		bundlerFile.close();
	}

	void HWBundle::LoadAlignMatrixFromCloudCampair(const std::string& cl_path)
	{
		std::cerr << "start to load align matrix... " << std::endl;
		std::ifstream fh(cl_path);
		if (fh.is_open())
		{
			Eigen::Matrix4f t;
			std::string linebuf;
			std::getline(fh, linebuf);
			std::sscanf(linebuf.c_str(), "%f %f %f %f", &t(0, 0), &t(0, 1), &t(0, 2), &t(0, 3));
			std::getline(fh, linebuf);
			std::sscanf(linebuf.c_str(), "%f %f %f %f", &t(1, 0), &t(1, 1), &t(1, 2), &t(1, 3));
			std::getline(fh, linebuf);
			std::sscanf(linebuf.c_str(), "%f %f %f %f", &t(2, 0), &t(2, 1), &t(2, 2), &t(2, 3));
			std::getline(fh, linebuf);
			std::sscanf(linebuf.c_str(), "%f %f %f %f", &t(3, 0), &t(3, 1), &t(3, 2), &t(3, 3));
			rigid_matrix_ = t;
			fh.close();
		}
		std::cerr << "loaded align matrix: \n" << rigid_matrix_ << std::endl;
	}

	void HWBundle::SetAlignMatrix(Eigen::Matrix4f& al_m)
	{
		rigid_matrix_ = al_m;
	}

	void HWBundle::ConvertPnts2AlignCamsAndAlignPnts()
	{
		//转换相机位姿到普通的相机位姿
		if (!cams_)
		{
			std::cerr << "ConvertBundleData2CamsAndPnts3d: load error..." << std::endl;
			return;
		}
		//处理相机位姿
		align_cams_ = new ModelCameras();
		*align_cams_ = *cams_;
		for (int i = 0; i < align_cams_->GetModelCamerasNum(); ++i)
		{
			CameraModel cam_model;
			align_cams_->GetPickedCamModel(i, cam_model);
			Eigen::Matrix4f cam_pose = cam_model.cam_pose_;
			/*Eigen::Matrix4f cam_extr = cam_pose.inverse();
			Eigen::Matrix4f cam_algined_extr = cam_extr * rigid_matrix_.inverse();
			Eigen::Matrix4f cam_align_pose = cam_algined_extr.inverse();*/
			Eigen::Matrix4f cam_align_pose = rigid_matrix_ * cam_pose;
			cam_model.cam_pose_ = cam_align_pose;
			align_cams_->SetCamModel(i, cam_model);
		}
		//处理align的点云
		align_pnts_3d_.clear();
		for (int i = 0; i < pnts_3d_.size(); ++i)
		{
			Eigen::Vector4f align_pnt_4d = Eigen::Vector4f(pnts_3d_[i][0], pnts_3d_[i][1], pnts_3d_[i][2], 1.0f);
			Eigen::Vector4f aligned_pnt = rigid_matrix_ * align_pnt_4d;
			align_pnts_3d_.emplace_back(Eigen::Vector3f(aligned_pnt[0], aligned_pnt[1], aligned_pnt[2]));
		}
		has_aligned_ = true;
	}

	void HWBundle::SaveAlignedCamsIntoLogFiles()
	{
		align_cams_->SaveCamsModelsIntoCamFiles();
	}

	void HWBundle::GetBundleAllCamsModels(std::vector<CameraModel>& cams_model)
	{
		bundle_cams_->GetModelCameras(cams_model);
	}

	void HWBundle::GetBundleAllCamsPoses(std::vector<Eigen::Matrix4f>& cams_poses)
	{
		//bundle_cams_->GetModelCameras(cams_model);
	}

	bool HWBundle::GetBundlePickedCamsModel(int idx, CameraModel& cam_model)
	{
		if (idx >= bundle_cams_->GetModelCamerasNum() || idx < 0)
			return false;
		bundle_cams_->GetPickedCamModel(idx, cam_model);
		return true;
	}

	bool HWBundle::GetBundlePickedCamsPos(int idx, Eigen::Matrix4f& cam_model)
	{
		if (idx >= bundle_cams_->GetModelCamerasNum() || idx < 0)
			return false;
		bundle_cams_->GetPickedCamPose(idx, cam_model);
		return true;
	}

	bool HWBundle::ConvertBundleData2CamsAndPnts3d()
	{
		//转换相机位姿到普通的相机位姿
		if (!bundle_cams_)
		{
			std::cerr << "ConvertBundleData2CamsAndPnts3d: load error..." << std::endl;
			return false;
		}
		cams_ = new ModelCameras();
		*cams_ = *bundle_cams_;
		for (int i = 0; i < bundle_cams_->GetModelCamerasNum(); ++i)
		{
			CameraModel tmp_bundle_model;
			bundle_cams_->GetPickedCamModel(i, tmp_bundle_model);
			Eigen::Matrix4f tmp_bundle_cam = tmp_bundle_model.cam_pose_;
			Eigen::Matrix4f tmp_bundle_extr = tmp_bundle_cam.inverse();
			Eigen::Vector3f b_t = tmp_bundle_extr.topRightCorner(3, 1);
			Eigen::Matrix3f b_r = tmp_bundle_extr.topLeftCorner(3, 3);
			Eigen::Matrix4f tmp_extr = tmp_bundle_extr;
			Eigen::Matrix3f tmp_extr_r;
			tmp_extr_r.col(0) = b_r.col(0);
			tmp_extr_r.col(1) = b_r.col(2);
			tmp_extr_r.col(2) = b_r.col(1);
			tmp_extr_r(0, 2) *= -1;
			tmp_extr_r(1, 0) *= -1;
			tmp_extr_r(2, 0) *= -1;
			tmp_extr_r(1, 1) *= -1;
			tmp_extr_r(2, 1) *= -1;
			Eigen::Vector3f tmp_extr_t = b_t;
			tmp_extr_t[1] *= -1;
			tmp_extr_t[2] *= -1;
			tmp_extr.topLeftCorner(3, 3) = tmp_extr_r;
			tmp_extr.topRightCorner(3, 1) = tmp_extr_t;
			CameraModel tmp_model = tmp_bundle_model;
			tmp_model.cam_pose_ = tmp_extr.inverse();
			cams_->SetCamModel(i, tmp_model);
		}

		//bundle点云转换为正常的点云
		pnts_3d_.clear();
		for (int i = 0; i < bundle_pnts_3d_.size(); ++i)
		{
			Eigen::Vector3f pnt = bundle_pnts_3d_[i].pnt3d_pos;
			pnts_3d_.emplace_back(Eigen::Vector3f(pnt[0], pnt[2], -pnt[1]));
		}
		return true;
	}

	void HWBundle::ProjectBundlePnts3d2PickedImg(cv::Mat& c_img, int idx)
	{
		//
		if (idx >= bundle_cams_->GetModelCamerasNum() || idx < 0)
			return;
		CameraModel cm;
		bundle_cams_->GetPickedCamModel(idx, cm);
		Eigen::Matrix4f cm_pose = cm.cam_pose_;
		std::vector<cv::Point2f> img_pnts;
		Eigen::Matrix4f cam_extri = cm_pose.inverse();
		std::cerr << "the cam_pose: \n" << cm_pose << std::endl;
		std::cerr << "the cam_extri: \n" << cam_extri << std::endl;
		std::cerr << "cm.cx_: " << cm.cx_ << std::endl;
		std::cerr << "cm.cy_: " << cm.cy_ << std::endl;
		std::cerr << "cm.fx_: " << cm.fx_ << std::endl;
		std::cerr << "cm.fy_: " << cm.fy_ << std::endl;

		for (int i = 0; i < bundle_pnts_3d_.size(); ++i)
		{
			Eigen::Vector3f w_pnt = bundle_pnts_3d_[i].pnt3d_pos;
			//std::cerr << "w_pnt: " << w_pnt.transpose() << std::endl;
			Eigen::Vector4f w_pnt_4d = Eigen::Vector4f(w_pnt[0], w_pnt[1], w_pnt[2], 1.0);
			Eigen::Vector4f cm_pnt = cam_extri * w_pnt_4d;
			float u = -cm_pnt[0] / cm_pnt[2];
			float v = -cm_pnt[1] / cm_pnt[2];
			float sq_pos = u*u + v*v;
			u = cm.cx_ *(1.0 + sq_pos*cm.distortion_[0] + sq_pos*sq_pos*cm.distortion_[1]) * u;
			v = cm.cy_ *(1.0 + sq_pos*cm.distortion_[0] + sq_pos*sq_pos*cm.distortion_[1]) * v;
			//坐标系转换
			u = u + cm.cx_;
			v = (- v) + cm.cy_;
			cv::Point2f i_p = cv::Point2f(u, v);
			//std::cerr << "u, v: " << u << ", " << v << std::endl;
			if (u < c_img.cols && u >= 0 && v < c_img.rows && v >= 0)
			{
				img_pnts.emplace_back(i_p);
			}	
		}
		std::cerr << "img_pnts num: " << img_pnts.size() << std::endl;
		for (int i = 0; i < img_pnts.size(); ++i)
		{
			cv::circle(c_img, img_pnts[i], 1, cv::Scalar(255, 0, 0));
			cv::circle(c_img, img_pnts[i], 2, cv::Scalar(0, 0, 255));
		}
	}

	void HWBundle::SaveBundlePnts3dAndAllImgs()
	{
		if (bundle_cams_->GetModelCamerasNum() < 0)
		{
			std::cerr << "none img..." << std::endl;
			return;
		}
		//std::string img_f_path = bundle_cams_->GetPickedCamImagePath(0);
		//std::string img_dir = img_f_path.substr(0, img_f_path.find_last_of("/"));
		for (int i = 0; i < bundle_cams_->GetModelCamerasNum(); ++i)
		{
			std::string img_path = bundle_cams_->GetPickedCamImagePath(i);
			cv::Mat img = cv::imread(img_path);
			ProjectBundlePnts3d2PickedImg(img, i);
			std::string img_o_path = img_path.substr(0, img_path.find_last_of("."));
			std::string img_path_new = img_o_path + "_proj.png";
			std::cerr << "SaveBundlePnts3dAndAllImgs: the path new " << img_path_new << std::endl;
			cv::imwrite(img_path_new, img);
		}
	}

	void HWBundle::ProjectPnts3d2PickedImg(cv::Mat& c_img, int idx)
	{
		if (idx >= cams_->GetModelCamerasNum() || idx < 0)
			return;
		CameraModel cm;
		cams_->GetPickedCamModel(idx, cm);
		Eigen::Matrix4f cm_pose = cm.cam_pose_;
		std::vector<cv::Point2f> img_pnts;
		Eigen::Matrix4f cam_extri = cm_pose.inverse();
		std::cerr << "the cam_pose: \n" << cm_pose << std::endl;
		std::cerr << "the cam_extri: \n" << cam_extri << std::endl;
		std::cerr << "cm.cx_: " << cm.cx_ << std::endl;
		std::cerr << "cm.cy_: " << cm.cy_ << std::endl;
		std::cerr << "cm.fx_: " << cm.fx_ << std::endl;
		std::cerr << "cm.fy_: " << cm.fy_ << std::endl;

		for (int i = 0; i < pnts_3d_.size(); ++i)
		{
			Eigen::Vector3f w_pnt = pnts_3d_[i];
			//std::cerr << "w_pnt: " << w_pnt.transpose() << std::endl;
			Eigen::Vector4f w_pnt_4d = Eigen::Vector4f(w_pnt[0], w_pnt[1], w_pnt[2], 1.0);
			Eigen::Vector4f cm_pnt = cam_extri * w_pnt_4d;
			float u = cm.fx_*cm_pnt[0] / cm_pnt[2] + cm.cx_;
			float v = cm.fy_*cm_pnt[1] / cm_pnt[2] + cm.cy_;
			cv::Point2f i_p = cv::Point2f(u, v);
			//std::cerr << "u, v: " << u << ", " << v << std::endl;
			if (u < c_img.cols && u >= 0 && v < c_img.rows && v >= 0)
			{
				img_pnts.emplace_back(i_p);
			}
		}
		std::cerr << "img_pnts num: " << img_pnts.size() << std::endl;
		for (int i = 0; i < img_pnts.size(); ++i)
		{
			cv::circle(c_img, img_pnts[i], 1, cv::Scalar(255, 0, 0));
			cv::circle(c_img, img_pnts[i], 2, cv::Scalar(0, 0, 255));
		}
	}

	void HWBundle::ProjectAlignPnts3d2PickedImg(cv::Mat& c_img, int idx)
	{
		if (idx >= align_cams_->GetModelCamerasNum() || idx < 0)
			return;
		CameraModel cm;
		align_cams_->GetPickedCamModel(idx, cm);
		Eigen::Matrix4f cm_pose = cm.cam_pose_;
		std::vector<cv::Point2f> img_pnts;
		Eigen::Matrix4f cam_extri = cm_pose.inverse();
		std::cerr << "the cam_pose: \n" << cm_pose << std::endl;
		std::cerr << "the cam_extri: \n" << cam_extri << std::endl;
		std::cerr << "cm.cx_: " << cm.cx_ << std::endl;
		std::cerr << "cm.cy_: " << cm.cy_ << std::endl;
		std::cerr << "cm.fx_: " << cm.fx_ << std::endl;
		std::cerr << "cm.fy_: " << cm.fy_ << std::endl;

		for (int i = 0; i < align_pnts_3d_.size(); ++i)
		{
			Eigen::Vector3f w_pnt = align_pnts_3d_[i];
			//std::cerr << "w_pnt: " << w_pnt.transpose() << std::endl;
			Eigen::Vector4f w_pnt_4d = Eigen::Vector4f(w_pnt[0], w_pnt[1], w_pnt[2], 1.0);
			Eigen::Vector4f cm_pnt = cam_extri * w_pnt_4d;
			float u = cm.fx_*cm_pnt[0] / cm_pnt[2] + cm.cx_;
			float v = cm.fy_*cm_pnt[1] / cm_pnt[2] + cm.cy_;
			cv::Point2f i_p = cv::Point2f(u, v);
			//std::cerr << "u, v: " << u << ", " << v << std::endl;
			if (u < c_img.cols && u >= 0 && v < c_img.rows && v >= 0)
			{
				img_pnts.emplace_back(i_p);
			}
		}
		std::cerr << "align img_pnts num: " << img_pnts.size() << std::endl;
		for (int i = 0; i < img_pnts.size(); ++i)
		{
			cv::circle(c_img, img_pnts[i], 1, cv::Scalar(255, 0, 0));
			cv::circle(c_img, img_pnts[i], 2, cv::Scalar(0, 0, 255));
		}
	}

	void HWBundle::SaveOriginPnts3dAndAllImgs()
	{
		if (cams_->GetModelCamerasNum() < 0)
		{
			std::cerr << "none img..." << std::endl;
			return;
		}
		//std::string img_f_path = bundle_cams_->GetPickedCamImagePath(0);
		//std::string img_dir = img_f_path.substr(0, img_f_path.find_last_of("/"));
		for (int i = 0; i < cams_->GetModelCamerasNum(); ++i)
		{
			std::string img_path = cams_->GetPickedCamImagePath(i);
			cv::Mat img = cv::imread(img_path);
			ProjectPnts3d2PickedImg(img, i);
			std::string img_o_path = img_path.substr(0, img_path.find_last_of("."));
			std::string img_path_new = img_o_path + "_origin_proj.png";
			std::cerr << "SavePnts3dAndAllImgs: the path new " << img_path_new << std::endl;
			cv::imwrite(img_path_new, img);
		}
	}

	void HWBundle::SaveAlignedPnts3dAndAllImgs()
	{
		if (!has_aligned_)
		{
			std::cerr << "no aligned existed..." << std::endl;
			return;
		}
		if (align_cams_->GetModelCamerasNum() < 0)
		{
			std::cerr << "none img..." << std::endl;
			return;
		}
		//std::string img_f_path = bundle_cams_->GetPickedCamImagePath(0);
		//std::string img_dir = img_f_path.substr(0, img_f_path.find_last_of("/"));
		for (int i = 0; i < align_cams_->GetModelCamerasNum(); ++i)
		{
			std::string img_path = align_cams_->GetPickedCamImagePath(i);
			cv::Mat img = cv::imread(img_path);
			ProjectAlignPnts3d2PickedImg(img, i);
			std::string img_o_path = img_path.substr(0, img_path.find_last_of("."));
			std::string img_path_new = img_o_path + "_aligned_proj.png";
			std::cerr << "SaveAlignedPnts3dAndAllImgs: the path new " << img_path_new << std::endl;
			cv::imwrite(img_path_new, img);
		}
	}

	void HWBundle::SaveOriginBundlePnts3dIntoObj()
	{
		std::string out_pth = b_dir_ + "/origin_bundle_pnts.obj";
		std::ofstream fh(out_pth);
		for(int i = 0; i < bundle_pnts_3d_.size(); ++i)
		{
			Eigen::Vector3f pnt3d = bundle_pnts_3d_[i].pnt3d_pos;
			fh << "v " << pnt3d[0] << " " << pnt3d[1] << " " << pnt3d[2] << std::endl;
		}
		fh.close();
	}

	void HWBundle::SaveOriginPnts3dIntoObj()
	{
		std::string out_pth = b_dir_ + "/origin_pnts.obj";
		std::ofstream fh(out_pth);
		for (int i = 0; i < pnts_3d_.size(); ++i)
		{
			Eigen::Vector3f pnt3d = pnts_3d_[i];
			fh << "v " << pnt3d[0] << " " << pnt3d[1] << " " << pnt3d[2] << std::endl;
		}
		fh.close();
	}

	void HWBundle::SaveAlignedPnts3dIntoObj()
	{
		std::string out_pth = b_dir_ + "/aligned_pnts.obj";
		std::ofstream fh(out_pth);
		for (int i = 0; i < align_pnts_3d_.size(); ++i)
		{
			Eigen::Vector3f pnt3d = align_pnts_3d_[i];
			fh << "v " << pnt3d[0] << " " << pnt3d[1] << " " << pnt3d[2] << std::endl;
		}
		fh.close();
	}

	void HWBundle::SaveAlignedCamsPoseIntoObj()
	{
		if (!has_aligned_)
		{
			std::cerr << "no aligned existed..." << std::endl;
			return;
		}
		std::string out_pth = b_dir_ + "/aligned_cams.obj";
		std::ofstream fh(out_pth);
		float step_len = 0.5;
		for (int i = 0; i < align_cams_->GetModelCamerasNum(); ++i)
		{
			Eigen::Matrix4f tmp_cam;
			align_cams_->GetPickedCamPose(i, tmp_cam);
			Eigen::Vector3f c_p = tmp_cam.topRightCorner(3, 1);
			fh << "v " << c_p[0] << " " << c_p[1] << " " << c_p[2] << std::endl;
			Eigen::Matrix3f r = tmp_cam.topLeftCorner(3, 3);
			Eigen::Vector3f c_x = r.col(0);
			Eigen::Vector3f c_x_p = c_p + c_x*step_len;
			fh << "v " << c_x_p[0] << " " << c_x_p[1] << " " << c_x_p[2] << " " 
				<< 255 << " " << 0 << " " << 0 << std::endl;
			Eigen::Vector3f c_y = r.col(1);
			Eigen::Vector3f c_y_p = c_p + c_y*step_len;
			fh << "v " << c_y_p[0] << " " << c_y_p[1] << " " << c_y_p[2] << " "
				<< 0 << " " << 255 << " " << 0 << std::endl;
			Eigen::Vector3f c_z = r.col(2);
			Eigen::Vector3f c_z_p = c_p + c_z*step_len;
			fh << "v " << c_z_p[0] << " " << c_z_p[1] << " " << c_z_p[2] << " "
				<< 0 << " " << 0 << " " << 255 << std::endl;
		}
		for (int i = 0; i < align_cams_->GetModelCamerasNum(); ++i)
		{
			fh << "l " << 4 * i + 1 << " " << 4 * i + 2 << std::endl;
			fh << "l " << 4 * i + 1 << " " << 4 * i + 3 << std::endl;
			fh << "l " << 4 * i + 1 << " " << 4 * i + 4 << std::endl;
		}
		fh.close();
	}

	void HWBundle::SplitStr(std::vector<std::string>& split_strs, std::string const& str, char delim, bool keep_empty)
	{
		std::size_t new_tok = 0;
		std::size_t cur_pos = 0;
		for (; cur_pos < str.size(); ++cur_pos)
		{
			if (str[cur_pos] == delim)
			{
				std::string token = str.substr(new_tok, cur_pos - new_tok);
				if (keep_empty || !token.empty())
					split_strs.push_back(token);
				new_tok = cur_pos + 1;
			}
		}
		if (keep_empty || new_tok < str.size())
			split_strs.push_back(str.substr(new_tok));
	}

}