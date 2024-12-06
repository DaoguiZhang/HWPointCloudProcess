#include "hw_cam_model.h"

namespace HW {

	HWCAMModel::HWCAMModel()
	{
		image_ratio_ = 1.0f;
		images_path_.clear();
		image_width_ = 5472;
		image_height_ = 3648;
		sample_ratio_ = 1;
	}

	HWCAMModel::~HWCAMModel()
	{
	}

	void HWCAMModel::ReadIntrinsic(std::string path)
	{
		
	}

	void HWCAMModel::ReadIntrinsicExtrinsic(std::string path)
	{
		TiXmlDocument doc(path.c_str());
		doc.LoadFile();
		TiXmlHandle hDoc(&doc);         // hDoc是&doc指向的对象
		TiXmlElement* pElem;            // 指向元素的指针
		pElem = hDoc.FirstChildElement().Element(); //指向根节点
		TiXmlHandle hRoot(pElem);       // hRoot是根节点
		TiXmlElement* nodeElem = hRoot.FirstChild("Block").FirstChild("Photogroups").FirstChild("Photogroup").Element();

		//先读相机的内参
		TiXmlElement* CamModel;
		//TiXmlElement* CamModel
		TiXmlElement *fx, *fy, *cx, *cy, *k1, *k2, *k3, *p1, *p2;
		TiXmlElement* image_width, *image_height;
		//std::vector<TiXmlElement*> distortion;
		Intrinsic intrinsic;
		if (nodeElem) {
			CamModel = nodeElem->FirstChildElement("ImageDimensions");
			image_width = CamModel->FirstChildElement("Width");
			image_width_ = std::stoi(image_width->GetText());
			image_height = image_width->NextSiblingElement("Height");
			image_height_ = std::stoi(image_height->GetText());
		}
	
		fx = CamModel->NextSiblingElement("FocalLength");
		intrinsic.fx = std::stof(fx->GetText());
		//fy = CamModel->NextSiblingElement("FocalLength");
		intrinsic.fy = std::stof(fx->GetText());
		TiXmlElement* principal;
		principal = CamModel->NextSiblingElement("PrincipalPoint");
		cx = principal->FirstChildElement("x");
		intrinsic.cx = std::stof(cx->GetText());
		cy = cx->NextSiblingElement("y");
		intrinsic.cy = std::stof(cy->GetText());
		TiXmlElement* distortion;
		distortion = CamModel->NextSiblingElement("Distortion");
		k1 = distortion->FirstChildElement("K1");
		intrinsic.distortion.emplace_back(std::stof(k1->GetText()));
		k2 = k1->NextSiblingElement("K2");
		intrinsic.distortion.emplace_back(std::stof(k2->GetText()));
		k3 = k2->NextSiblingElement("K3");
		intrinsic.distortion.emplace_back(std::stof(k3->GetText()));
		p1 = k3->NextSiblingElement("P1");
		intrinsic.distortion.emplace_back(std::stof(p1->GetText()));
		p2 = p1->NextSiblingElement("P2");
		intrinsic.distortion.emplace_back(std::stof(p2->GetText()));

		intrinsics_.emplace_back(intrinsic);
		//std::cout << "555555555555555555555555555" << std::endl;
		TiXmlElement* current_node;
		current_node = CamModel->NextSiblingElement("Photo");
		
		while (current_node)
		{
			Eigen::Matrix4f cam_extrinsic;
			//TiXmlElement* image_path_element;
			CamModel = current_node->FirstChildElement("ImagePath");
			std::string image_path(CamModel->GetText());
			std::string tmp_image_path = image_path;
			images_path_.push_back(tmp_image_path);
			//std::cout << "the image path is: " << image_path << std::endl;
			//return;
			TiXmlElement* pos_element;
			pos_element = CamModel->NextSiblingElement("Pose");
			TiXmlElement* r_element;
			r_element = pos_element->FirstChildElement("Rotation")->FirstChildElement("M_00");
			Eigen::Matrix3f cam_r;
			std::vector<float> r_vec;
			for (r_element; r_element; r_element = r_element->NextSiblingElement())
			{
				float r_v = std::stof(r_element->GetText());
				//std::cout << "the r v: " << r_v << std::endl;
				r_vec.emplace_back(r_v);
			}

			cam_r << r_vec[0], r_vec[1], r_vec[2],
				r_vec[3], r_vec[4], r_vec[5],
				r_vec[6], r_vec[7], r_vec[8];

			TiXmlElement* t_element;
			Eigen::Vector3f cam_t;
			std::vector<float> t_vec;
			t_element = pos_element->FirstChildElement("Center")->FirstChildElement();
			for (t_element; t_element; t_element = t_element->NextSiblingElement())
			{
				float t_v = std::stof(t_element->GetText());
				//std::cout << "the tv is: " << t_v << std::endl;
				t_vec.emplace_back(t_v);
				//cam_t << t_v;
			}
			cam_t << t_vec[0], t_vec[1], t_vec[2];

			cam_t.x() -= 368394;
			cam_t.y() -= 3459110;

			//.xml读入的是R_cw,需要transpose转换,得到R_wc
			cam_extrinsic.topLeftCorner(3, 3) = cam_r.transpose();

			//因为.xml读入的是相机位置,它就是相机位姿(T_wc)的转移向量,直接存入即可
			Eigen::Vector3f t_wc = cam_t;

			cam_extrinsic.topRightCorner(3, 1) = cam_t;
			cam_extrinsic(3, 0) = 0.0f;
			cam_extrinsic(3, 1) = 0.0f;
			cam_extrinsic(3, 2) = 0.0f;
			cam_extrinsic(3, 3) = 1.0f;
			//std::cout << "the cam matrix is: \n" << cam_extrinsic << std::endl;
			T_WC_.emplace_back(cam_extrinsic);

			current_node = current_node->NextSiblingElement("Photo");
		}
	}
	
	int HWCAMModel::GetCamsSize()
	{
		return T_WC_.size();
	}

	int HWCAMModel::GetImagesPthsSize()
	{
		return images_path_.size();
	}

	void HWCAMModel::ReadExtrinsic(std::string dir)
	{
		
	}

	void HWCAMModel::GetCurrentDirFiles(const std::string & path, std::vector<std::string> & files)
	{
		//文件句柄  
		long long hFile = 0;
		//文件信息，_finddata_t需要io.h头文件  
		struct _finddata_t fileinfo;
		std::string p;
		int i = 0;
		if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
		{
			do
			{
				if (!(fileinfo.attrib & _A_SUBDIR))
				{
					files.push_back(p.assign(path).append("/").append(fileinfo.name));	
				}
			} while (_findnext(hFile, &fileinfo) == 0);
			_findclose(hFile);
		}
	}

	void HWCAMModel::GetAllDirFiles(const std::string & path, std::vector<std::string> & files)
	{
		//文件句柄  
		long long hFile = 0;
		//文件信息，_finddata_t需要io.h头文件  
		struct _finddata_t fileinfo;
		std::string p;
		int i = 0;
		if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
		{
			do
			{
				//如果是目录,迭代之  
				//如果不是,加入列表  
				if ((fileinfo.attrib & _A_SUBDIR))
				{
					if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
						GetAllDirFiles(p.assign(path).append(fileinfo.name), files);
				}
				else
				{
					files.push_back(p.assign(path).append("/").append(fileinfo.name));
				}
			} while (_findnext(hFile, &fileinfo) == 0);
			_findclose(hFile);
		}
	}

	void HWCAMModel::ReadRealImagesPath(const std::vector<std::string>& real_path)
	{
		if (real_path.size() != images_path_.size())
		{
			printf("no compare image and camera...\n");
			if (images_path_.size() > real_path.size())
				return;
		}

		std::vector<std::string> surplus_vec;
		for (int i = 0; i < real_path.size(); ++i)
		{
			std::string real_img_name = real_path[i].substr(real_path[i].find_last_of("/") + 1,
				real_path[i].length() - real_path[i].find_last_of("/") - 1);
			//std::cout << "real_img_name: " << real_img_name << std::endl;
			int j = 0;
			for ( ; j < images_path_.size(); ++j)
			{
				std::string img_name = images_path_[j].substr(images_path_[j].find_last_of("/") + 1,
					images_path_[j].length() - images_path_[j].find_last_of("/") - 1);
				if (real_img_name == img_name)
				{
					images_path_[j] = real_path[i];
					//std::cout << "img_name: " << img_name << std::endl;
					break;
				}
			}
			if (j == images_path_.size())
				surplus_vec.emplace_back(real_path[i]);
		}
		//std::cout << "the suplus vec size is: " << surplus_vec.size() << std::endl;
		//SaveSurplusImagesIntoTXT(surplus_vec);
	}

	void HWCAMModel::SetImageRatio(float img_ratio)
	{
		image_ratio_ = img_ratio;
	}

	void HWCAMModel::SetSampleImageRatio(float sample_ratio)
	{
		sample_ratio_ = sample_ratio;
	}

	void HWCAMModel::SetImgOutDir(const std::string& dir)
	{
		out_dir_ = dir;
	}

	void HWCAMModel::SaveCamsIntoCam()
	{
		if (T_WC_.size() != images_path_.size())
		{
			std::cout << "no compare between cam and image... " << std::endl;
			return;
		}

		//std::vector<Eigen::Matrix4f> cams_ps;

		for (int i = 0; i < GetCamsSize(); ++i)
		{
			if (i % sample_ratio_)
				continue;

			std::string cam_path;
			int extension_len = images_path_[i].length() - images_path_[i].find_last_of(".");
			std::string cam_name = images_path_[i].substr(images_path_[i].find_last_of("/") + 1,
				images_path_[i].length() - images_path_[i].find_last_of("/") - extension_len - 1);
			cam_path = out_dir_ + cam_name + ".CAM";
			std::ofstream fh(cam_path);
#if 0
			Eigen::Vector3f t_wc = T_WC_[i].topRightCorner(3, 1);
			Eigen::Matrix3f r_wc = T_WC_[i].topLeftCorner(3, 3);

			//假设读入的相机为r_cw;
			Eigen::Matrix3f r_cw = r_wc.transpose();	//正交矩阵的逆（inverse）等于正交矩阵的转置（transpose）
			//std::cout << "the rcw0: \n" << r_cw << std::endl;
			Eigen::Vector3f t_cw = -r_wc.transpose() * t_wc;
			//std::cout << "the rcw1: \n" << r_cw << std::endl;
#else
			//相机外参
			Eigen::Matrix4f M_cw = T_WC_[i].inverse();

			//cams_ps.emplace_back(T_WC_[i]);

			Eigen::Matrix3f r_cw = M_cw.topLeftCorner(3, 3);
			Eigen::Vector3f t_cw = M_cw.topRightCorner(3, 1);
#endif

			//存储相机的外参，也就是T_cw，用于投影
			fh << t_cw(0) << " " << t_cw(1) << " " << t_cw(2)
				<< " " << r_cw(0, 0) << " " << r_cw(0, 1) << " " << r_cw(0, 2)
				<< " " << r_cw(1, 0) << " " << r_cw(1, 1) << " " << r_cw(1, 2)
				<< " " << r_cw(2, 0) << " " << r_cw(2, 1) << " " << r_cw(2, 2) << std::endl;

			//相机的内参
			if (!intrinsics_.empty())
			{
#if 1	// 1代表瀚彤的.cam文件格式， 0代表新定义的.cam文件格式
				fh << (intrinsics_[0].fx * image_width_ / 13.2) * image_ratio_ << " " << (intrinsics_[0].fx * image_width_ / 13.2) * image_ratio_ << " "
					<< intrinsics_[0].cx * image_ratio_ << " " << intrinsics_[0].cy * image_ratio_ << std::endl;
				//对于畸变的话，可能不知道要不要和image_ratio_相乘，后续查阅一下，更改
				for (int j = 0; j < intrinsics_[0].distortion.size(); ++j)
				{
					fh << intrinsics_[0].distortion[j] << " ";
				}
#else
				float flen_size = (intrinsics_[0].fx * image_width_ / 13.2) * image_ratio_;
				float flen_from_resized_image = flen_size / (image_width_ * image_ratio_);
				fh << flen_from_resized_image;
#endif	
			}
			fh.close();
		}

		//test
		//std::string cam_path = "D:/vc_project_new/Dongfangwanguo_UAV_LiDAR_Pano/UAV/cams_objs.obj";
		//SaveCamPoseIntoOBJ(cams_ps, cam_path);
		//SaveCamsPoseIntoObjFile(cams_ps, cam_path, 20.0f);
		//end test
	}

	void HWCAMModel::SaveCamPoseIntoOBJ(std::vector<Eigen::Matrix4f>& cams_pose, const std::string& path)
	{
		std::ofstream fh(path);
		int step_size = 1000;
		float step = 0.05;

		for (int i = 0; i < cams_pose.size(); ++i)
		{

			Eigen::Matrix3f R_wc = cams_pose[i].topLeftCorner(3, 3);
			Eigen::Vector3f T_wc = cams_pose[i].topRightCorner(3, 1);

			////--------------------三根轴------------------------//
			//x轴
			Eigen::Vector3f local_x = R_wc.col(0);
			//y轴
			Eigen::Vector3f local_y = R_wc.col(1);
			//z轴
			Eigen::Vector3f local_z = R_wc.col(2);

			std::vector<Eigen::Vector3f> cam_x_local_pnts;
			std::vector<Eigen::Vector3f> cam_y_local_pnts;
			std::vector<Eigen::Vector3f> cam_z_local_pnts;

			for (int i = 0; i < step_size; ++i)
			{
				//x轴
				float x_x = T_wc(0) + i * step * local_x(0);
				float x_y = T_wc(1) + i * step * local_x(1);
				float x_z = T_wc(2) + i * step * local_x(2);
				Eigen::Vector3f tmp_x(x_x, x_y, x_z);
				cam_x_local_pnts.emplace_back(tmp_x);

				//y轴
				float y_x = T_wc(0) + i * step * local_y(0);
				float y_y = T_wc(1) + i * step * local_y(1);
				float y_z = T_wc(2) + i * step * local_y(2);
				Eigen::Vector3f tmp_y(y_x, y_y, y_z);
				cam_y_local_pnts.emplace_back(tmp_y);
				//z轴
				float z_x = T_wc(0) + i * step * local_z(0);
				float z_y = T_wc(1) + i * step * local_z(1);
				float z_z = T_wc(2) + i * step * local_z(2);
				Eigen::Vector3f tmp_z(z_x, z_y, z_z);
				cam_z_local_pnts.emplace_back(tmp_z);
			}
			for (int i = 0; i < step_size; ++i)
			{
				fh << "v " << cam_x_local_pnts[i](0) << " " << cam_x_local_pnts[i](1) << " " << cam_x_local_pnts[i](2) <<
					" " << 255 << " " << 0 << " " << 0 << std::endl;
				fh << "v " << cam_y_local_pnts[i](0) << " " << cam_y_local_pnts[i](1) << " " << cam_y_local_pnts[i](2) <<
					" " << 0 << " " << 255 << " " << 0 << std::endl;
				fh << "v " << cam_z_local_pnts[i](0) << " " << cam_z_local_pnts[i](1) << " " << cam_z_local_pnts[i](2) <<
					" " << 0 << " " << 0 << " " << 255 << std::endl;
			}
		}
		fh.close();
	}

	void HWCAMModel::SaveCamsPoseIntoObjFile(std::vector<Eigen::Matrix4f>& cams_pose, const std::string& file_path, const float& aix_len)
	{
		std::ofstream fh(file_path);
		for (int i = 0; i < cams_pose.size(); ++i)
		{
			Eigen::Matrix3f R_wc = cams_pose[i].topLeftCorner(3, 3);
			Eigen::Vector3f T_wc = cams_pose[i].topRightCorner(3, 1);

			////--------------------三根轴------------------------//
			//x轴
			Eigen::Vector3f local_x = R_wc.col(0);
			//y轴
			Eigen::Vector3f local_y = R_wc.col(1);
			//z轴
			Eigen::Vector3f local_z = R_wc.col(2);

			//相机位置
			Eigen::Vector3f pnt_origin = T_wc;
			local_x.normalize();
			local_y.normalize();
			local_z.normalize();
			//获取其它三个方向的终止点
			Eigen::Vector3f end_x = pnt_origin + aix_len*local_x;
			Eigen::Vector3f end_y = pnt_origin + aix_len*local_y;
			Eigen::Vector3f end_z = pnt_origin + aix_len*local_z;

			fh << "v " << pnt_origin[0] << " " << pnt_origin[1] << " " << pnt_origin[2] <<
				" " << 255 << " " << 255 << " " << 255 << std::endl;
			fh << "v " << end_x[0] << " " << end_x[1] << " " << end_x[2] <<
				" " << 255 << " " << 0 << " " << 0 << std::endl;
			fh << "v " << end_y[0] << " " << end_y[1] << " " << end_y[2] <<
				" " << 0 << " " << 255 << " " << 0 << std::endl;
			fh << "v " << end_z[0] << " " << end_z[1] << " " << end_z[2] <<
				" " << 0 << " " << 0 << " " << 255 << std::endl;
		}
		//画出线，就是它们三根轴
		for (int i = 0; i < cams_pose.size(); ++i)
		{
			fh << "l " << 4 * i + 1 << " " << 4 * i + 2 << std::endl;
			fh << "l " << 4 * i + 1 << " " << 4 * i + 3 << std::endl;
			fh << "l " << 4 * i + 1 << " " << 4 * i + 4 << std::endl;
		}
		fh.close();
	}

	void HWCAMModel::DrawCamsPoseIntoOBJ(std::vector<int> selects_idx, const std::string& path)
	{
		std::ofstream fh(path);

		std::vector<int> real_idxs;
		for (int i = 0; i < selects_idx.size(); ++i)
		{
			if (selects_idx[i] < 0 || selects_idx[0] >= T_WC_.size())
				continue;
			real_idxs.emplace_back(selects_idx[i]);
		}

		int step_size = 1000;
		float step = 0.05;
		
		for (int j = 0; j < real_idxs.size(); ++j)
		{
			int select_idx = real_idxs[j];
			
			//相机的位姿，可以直接画出来
			Eigen::Vector3f t = T_WC_[select_idx].topRightCorner(3, 1); //相机坐标系到世界坐标系
			Eigen::Matrix3f r = T_WC_[select_idx].topLeftCorner(3, 3);
			
			//t.x() -= 368394;
			//t.y() -= 3459110;

			Eigen::Vector3f T_wc = t;
			Eigen::Matrix3f R_wc = r;

			////--------------------三根轴------------------------//
			//x轴
			Eigen::Vector3f local_x = R_wc.col(0);
			//y轴
			Eigen::Vector3f local_y = R_wc.col(1);
			//z轴
			Eigen::Vector3f local_z = R_wc.col(2);

			std::vector<Eigen::Vector3f> cam_x_local_pnts;
			std::vector<Eigen::Vector3f> cam_y_local_pnts;
			std::vector<Eigen::Vector3f> cam_z_local_pnts;

			for (int i = 0; i < step_size; ++i)
			{
				//x轴
				float x_x = T_wc(0) + i * step * local_x(0);
				float x_y = T_wc(1) + i * step * local_x(1);
				float x_z = T_wc(2) + i * step * local_x(2);
				Eigen::Vector3f tmp_x(x_x, x_y, x_z);
				cam_x_local_pnts.emplace_back(tmp_x);

				//y轴
				float y_x = T_wc.x() + i * step * local_y(0);
				float y_y = T_wc.y() + i * step * local_y(1);
				float y_z = T_wc.z() + i * step * local_y(2);
				Eigen::Vector3f tmp_y(y_x, y_y, y_z);
				cam_y_local_pnts.emplace_back(tmp_y);
				//z轴
				float z_x = T_wc.x() + i * step * local_z(0);
				float z_y = T_wc.y() + i * step * local_z(1);
				float z_z = T_wc.z() + i * step * local_z(2);
				Eigen::Vector3f tmp_z(z_x, z_y, z_z);
				cam_z_local_pnts.emplace_back(tmp_z);
			}
			for (int i = 0; i < step_size; ++i)
			{
				fh << "v " << cam_x_local_pnts[i](0) << " " << cam_x_local_pnts[i](1) << " " << cam_x_local_pnts[i](2) <<
					" " << 255 << " " << 0 << " " << 0 << std::endl;
				fh << "v " << cam_y_local_pnts[i](0) << " " << cam_y_local_pnts[i](1) << " " << cam_y_local_pnts[i](2) <<
					" " << 0 << " " << 255 << " " << 0 << std::endl;
				fh << "v " << cam_z_local_pnts[i](0) << " " << cam_z_local_pnts[i](1) << " " << cam_z_local_pnts[i](2) <<
					" " << 0 << " " << 0 << " " << 255 << std::endl;
			}
		}

		fh.close();
	}

	void HWCAMModel::SaveImagesIntoPNG()
	{
		for (int i = 0; i < images_path_.size(); ++i)
		{
			if (i % sample_ratio_)
				continue;

			cv::Mat mat = cv::imread(images_path_[i]);
			std::string image_name = images_path_[i].substr(images_path_[i].find_last_of("/") + 1,
				images_path_[i].length() - images_path_[i].find_last_of("/") - 1);
			std::string save_path = out_dir_ + image_name;
			std::cout << "the saved path is: " << save_path << std::endl;
			cv::Mat resized_mat(cv::Size(mat.cols*image_ratio_, mat.rows*image_ratio_), mat.type());
			cv::resize(mat, resized_mat, resized_mat.size());
			cv::imwrite(save_path, resized_mat);
		}
	}

	void HWCAMModel::SaveSurplusImagesIntoTXT(std::vector<std::string>& surplus_names)
	{
		std::string path = out_dir_ + "surplus_images.txt";
		std::cout << "the log txt is: " << path << std::endl;
		std::ofstream fh(path);
		for (int i = 0; i < surplus_names.size(); ++i)
			fh << surplus_names[i] << std::endl;
		fh.close();
	}
}