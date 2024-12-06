#include"scene_elements.h"
#include<opencv/cv.hpp>
#include <direct.h>
#include"hw_cmns.h"

#define USING_NETWORK_INPUT_LINES 0;

SceneElements::SceneElements()
{

}

SceneElements::~SceneElements()
{
	//
}

void SceneElements::SetDir(std::string dir)
{
	s_dir_ = dir;
}

void SceneElements::LoadSceneElements()
{
	model_cam_data_.LoadCamsFromCAMDIR(s_dir_);
	std::vector<std::string> files;
	model_cam_data_.GetCurrentDirFiles(s_dir_, files);
	std::vector<std::string> layout_paths;
	for (int i = 0; i < files.size(); ++i)
	{
		if (files[i].find(".log") != std::string::npos)
		{
			std::string file_left_slash_name = HW::GetLeftSlashPathName(files[i]);
			layout_paths.emplace_back(file_left_slash_name);
		}
	}

	////处理这个lines layout
	//std::vector<std::string> lyout_paths_threshold;
	//for (int i = 0; i < layout_paths.size(); ++i)
	//{
	//	if (layout_paths[i].find(line_score_threshold_) != std::string::npos)
	//	{
	//		lyout_paths_threshold.emplace_back(layout_paths[i]);
	//	}
	//}

	bool use_integer_flag = true;
	for (int i = 0; i < layout_paths.size(); ++i)
	{
		std::string file_basename = HW::GetBaseNameWithoutSuffix(layout_paths[i]);
		if (file_basename.empty())
		{
			continue;
		}
		if (!HW::IsStrFrontElementInteger(file_basename))
		{
			use_integer_flag = false;
			break;
		}
	}
	if (use_integer_flag)
	{
		std::vector<std::pair<int, std::string> > ly_files_idxs_str;
		for (int i = 0; i < layout_paths.size(); ++i)
		{
			//sort by file names
			std::string file_basename = HW::GetBaseNameWithoutSuffix(layout_paths[i]);
			int lyid = std::stoi(file_basename);
			std::pair<int, std::string> tmp_file_name;
			tmp_file_name.first = lyid;
			tmp_file_name.second = layout_paths[i];
			ly_files_idxs_str.emplace_back(tmp_file_name);
		}
		std::sort(ly_files_idxs_str.begin(), ly_files_idxs_str.end(), CompareHWStrPairByIdxSceneCmp);
		//暂且没有处理文件不匹配的问题，后续再处理
		for (int i = 0; i < ly_files_idxs_str.size(); ++i)
		{
			SceneLayout ly;
#if USING_NETWORK_INPUT_LINES
			ly.ReadLayout2DFromNetPath(ly_files_idxs_str[i].second);

			/*
			compare model_cam_data_ name with layout_paths i
			to do next ...
			*/
			int cam_idx = i;	//write a function to do it. to do next zdg
			model_cam_data_.SetPickedCamModelLyoutId(cam_idx, i);
#else

			ly.ReadLayout2D(ly_files_idxs_str[i].second);
			//set the model_cam_data_ id
			ly.SetLayoutName(ly_files_idxs_str[i].second);
			/*
			compare model_cam_data_ name with layout_paths i
			to do next ...
			*/
			int cam_idx = i;	//write a function to do it. to do next zdg
			model_cam_data_.SetPickedCamModelLyoutId(cam_idx, i);
#endif
			scene_cams_ly_data_.emplace_back(ly);
		}
	}
	else
	{
		//暂且没有处理文件不匹配的问题，后续再处理
		for (int i = 0; i < layout_paths.size(); ++i)
		{
			SceneLayout ly;
#if USING_NETWORK_INPUT_LINES
			ly.ReadLayout2DFromNetPath(layout_paths[i]);

			/*
			compare model_cam_data_ name with layout_paths i
			to do next ...
			*/
			int cam_idx = i;	//write a function to do it. to do next zdg
			model_cam_data_.SetPickedCamModelLyoutId(cam_idx, i);
#else

			ly.ReadLayout2D(layout_paths[i]);
			//set the model_cam_data_ id
			//std::cerr <<""
			ly.SetLayoutName(layout_paths[i]);
			/*
			compare model_cam_data_ name with layout_paths i
			to do next ...
			*/
			int cam_idx = i;	//write a function to do it. to do next zdg
			model_cam_data_.SetPickedCamModelLyoutId(cam_idx, i);

			//model_cam_data_.s
#endif
			scene_cams_ly_data_.emplace_back(ly);
		}
	}

}

void SceneElements::UpdateAllScenesLayoutRedauntLinesIdxs()
{
	for (int i = 0; i < scene_cams_ly_data_.size(); ++i)
	{
		scene_cams_ly_data_[i].UpdateRedauntLineSegments();
	}
}

const std::string& SceneElements::GetSceneElementsDir()
{
	return s_dir_;
}

void SceneElements::SamplePntsInLinesFromImgCoord()
{
	for (int i = 0; i < scene_cams_ly_data_.size(); ++i)
	{
		scene_cams_ly_data_[i].SamplePntsFromAllLineEndPnts();
	}
}

void SceneElements::SetLineScoreThreshold(std::string& sc)
{
	//
	line_score_threshold_ = sc;
}

std::string SceneElements::GetLineScoreThreshold()
{
	return line_score_threshold_;
}

void SceneElements::SetHWPlanesVec(std::vector<HW::HWPlane*>& plane_vec)
{
	associated_planes_polygons_ = plane_vec;
}

void SceneElements::SetWorldLyoutFromPolygonWindow(std::vector<std::vector<Eigen::Vector3f> >& lyout)
{
	//
	scene_world_ly_data_.SetAllPolygonsSceneLayout3D(lyout);
}

ModelCameras* SceneElements::GetCamsData()
{
	return &model_cam_data_;
}

std::vector<SceneLayout>& SceneElements::GetCamSceneLyData()
{
	return scene_cams_ly_data_;
}

bool SceneElements::GetImgLayoutLineSeg(int img_idx, int l_idx, Eigen::Vector3f& ls, Eigen::Vector3f& le)
{
	if (img_idx < 0 || img_idx >= scene_cams_ly_data_.size())
	{
		return false;
	}
	if (l_idx < 0 || l_idx >= scene_cams_ly_data_[img_idx].GetLayoutLine2PntIdx().size())
	{
		return false;
	}
	scene_cams_ly_data_[img_idx].GetPickedLineSeg(l_idx, ls, le);
	return true;
}

bool SceneElements::GetImgLayoutLineSegSamplePnts(int img_idx, int l_idx, std::vector<Eigen::Vector2i>& line_sample_pnts)
{
	if (img_idx < 0 || img_idx >= scene_cams_ly_data_.size())
	{
		return false;
	}
	if (l_idx < 0 || l_idx >= scene_cams_ly_data_[img_idx].GetLayoutLine2PntIdx().size())
	{
		return false;
	}
	scene_cams_ly_data_[img_idx].GetPickedLineSegSamplePnts(l_idx, line_sample_pnts);
	return true;
}

bool SceneElements::GetPickedImgLayoutLineSegs(int img_idx,
	std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& layout_pnts)
{
	if (img_idx < 0 || img_idx >= scene_cams_ly_data_.size())
	{
		return false;
	}
	for (int i = 0; i < scene_cams_ly_data_[img_idx].LinesSize(); ++i)
	{
		Eigen::Vector3f ls, le;
		scene_cams_ly_data_[img_idx].GetPickedLineSeg(i, ls, le);
		std::pair<Eigen::Vector3f, Eigen::Vector3f> line_pnt = std::make_pair(ls, le);
		layout_pnts.emplace_back(line_pnt);
	}
	return true;
}

void SceneElements::ShowPickedImgLines(int idx)
{
	if (idx < 0 || idx >= model_cam_data_.GetModelCamerasNum())
	{
		std::cout << "cam vector: idx out of range! " << std::endl;
		return;
	}
	//获取选中的图片
	std::string picked_name = model_cam_data_.GetPickedCamImagePath(idx);
	cv::Mat img = cv::imread(picked_name);
	//获取对应的线段
	for (int i = 0; i < scene_cams_ly_data_[idx].LinesSize(); ++i)
	{
		Eigen::Vector3f s, e;
		scene_cams_ly_data_[idx].GetPickedLineSeg(i, s, e);
		cv::Point2f cvs(s[0], s[1]);
		cv::Point2f cve(e[0], e[1]);
		cv::line(img, cvs, cve, cv::Scalar(255, 0, 0), 3);
		cv::circle(img, cvs, 3, cv::Scalar(0, 255, 0), 3);
	}

	/*for (int i = 0; i < scene_cams_ly_data_[idx].LinesSize(); ++i)
	{
		std::vector<Eigen::Vector2i> lsp;
		scene_cams_ly_data_[idx].GetPickedLineSegSamplePnts(i, lsp);
		for (int j = 0; j < lsp.size(); ++j)
		{
			cv::Point2f lp(lsp[j][0], lsp[j][1]);
			cv::circle(img, lp, 1, cv::Scalar(0, 255, 0), 1);
		}
	}*/

	//获取选中的图像
	cv::namedWindow("scene line", cv::WINDOW_AUTOSIZE);
	cv::imshow("scene line", img);
	//cv::waitKey(0);
}

void SceneElements::SaveAllImgsLines()
{
	//获取path的dir
	std::string imgdir = s_dir_;
	std::string imgdirlast = imgdir.substr(0, imgdir.find_last_of("/"));
	std::string imgnewdir = imgdirlast + "/all_img_line";
	if (_access(imgdirlast.c_str(), 0) == -1) {
		std::cout << "last dir does not exist~~~" << std::endl;
		return;
	}
	if (_access(imgnewdir.c_str(), 0) == -1) {
		mkdir(imgnewdir.c_str());
	}
	std::cerr << "imgnewdir: " << imgnewdir << std::endl;
	for (int idx = 0; idx < scene_cams_ly_data_.size(); ++idx)
	{
		if (idx < 0 || idx >= model_cam_data_.GetModelCamerasNum())
		{
			std::cout << "cam vector: idx out of range! " << std::endl;
			continue;
		}
		std::string picked_name = model_cam_data_.GetPickedCamImagePath(idx);
		cv::Mat img = cv::imread(picked_name);
		for (int i = 0; i < scene_cams_ly_data_[idx].LinesSize(); ++i)
		{
			Eigen::Vector3f s, e;
			scene_cams_ly_data_[idx].GetPickedLineSeg(i, s, e);
			cv::Point2f cvs(s[0], s[1]);
			cv::Point2f cve(e[0], e[1]);
			cv::line(img, cvs, cve, cv::Scalar(255, 0, 0), 2);
			cv::circle(img, cvs, 3, cv::Scalar(0, 255, 0), 2);
		}
		//save the img
		std::string filename = picked_name.substr(picked_name.find_last_of("/"), picked_name.length() - picked_name.find_last_of("/"));
		std::string file_path_new = imgnewdir + "/" + filename;
		cv::imwrite(file_path_new, img);
	}
}

Eigen::Vector2f SceneElements::WorldPointProjToPickedImg(Eigen::Vector3f& p3d, int img_idx)
{
	//
	if (img_idx < 0 || img_idx >= model_cam_data_.GetModelCamerasNum())
	{
		std::cout << "cam vector: idx out of range! " << std::endl;
		return Eigen::Vector2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	}
	CameraModel picked_cam_model;
	Eigen::Matrix4f picked_cam_pose;
	model_cam_data_.GetPickedCamModel(img_idx, picked_cam_model);
	picked_cam_pose = picked_cam_model.cam_pose_;
	//Eigen::Vector3f picked_cam_z = picked_cam_pose.block(0, 2, 3, 1);
	//std::cout << "the z is: \n" << picked_cam_z << std::endl;
	Eigen::Vector4f pnt = Eigen::Vector4f(p3d[0], p3d[1], p3d[2], 1.0);
	Eigen::Vector4f cam_p = picked_cam_pose.inverse() * pnt;	//相机坐标系下的顶点坐标
	float p_c_z = cam_p[2];
	
	float s_u = picked_cam_model.fx_ * cam_p[0] / p_c_z + picked_cam_model.cx_;
	float s_v = picked_cam_model.fy_ * cam_p[1] / p_c_z + picked_cam_model.cy_;
	Eigen::Vector2f proj_pnt = Eigen::Vector2f(s_u, s_v);
	
	return proj_pnt;
}

void SceneElements::ShowWorldLyProjectLine2PickedImg(int idx)
{
	//
	if (idx < 0 || idx >= model_cam_data_.GetModelCamerasNum())
	{
		std::cout << "cam vector: idx out of range! " << std::endl;
		return;
	}
	//获取选中的图片
	std::string picked_name = model_cam_data_.GetPickedCamImagePath(idx);
	cv::Mat img = cv::imread(picked_name);
	//获取对应的线段
	for (int i = 0; i < scene_cams_ly_data_[idx].LinesSize(); ++i)
	{
		Eigen::Vector3f s, e;
		scene_cams_ly_data_[idx].GetPickedLineSeg(i, s, e);
		cv::Point2f cvs(s[0], s[1]);
		cv::Point2f cve(e[0], e[1]);
		cv::line(img, cvs, cve, cv::Scalar(255, 0, 0), 3);
		cv::circle(img, cvs, 3, cv::Scalar(0, 255, 0), 3);
	}

	CameraModel picked_cam_model;
	Eigen::Matrix4f picked_cam_pose;
	model_cam_data_.GetPickedCamModel(idx, picked_cam_model);
	picked_cam_pose = picked_cam_model.cam_pose_;
	std::cout << "the picked cam pose is: \n" << picked_cam_pose << std::endl;
	Eigen::Vector3f picked_cam_z = picked_cam_pose.block(0, 2, 3, 1);
	std::cout << "the z is: \n" << picked_cam_z << std::endl;
	
	Eigen::Vector2f start_img_pos;
	Eigen::Vector2f end_img_pos;

	for (int i = 0; i < scene_world_ly_data_.LinesSize(); ++i)
	{
		Eigen::Vector3f s, e;
		scene_world_ly_data_.GetPickedLineSeg(i, s, e);

		//project to img
		Eigen::Vector4f s_pnt = Eigen::Vector4f(s[0], s[1], s[2], 1.0);
		Eigen::Vector4f e_pnt = Eigen::Vector4f(e[0], e[1], e[2], 1.0);
		Eigen::Vector4f cam_s_p = picked_cam_pose.inverse() * s_pnt;	//相机坐标系下的顶点坐标
		Eigen::Vector4f cam_e_p = picked_cam_pose.inverse() * e_pnt;	//相机坐标系下的顶点坐标
		
		float s_z = cam_s_p[2];
		float e_z = cam_e_p[2];

		float s_u = 0.5*img.cols * cam_s_p[0] / s_z + 0.5*img.cols;
		float s_v = 0.5*img.cols * cam_s_p[1] / s_z + 0.5*img.rows;

		float e_u = 0.5*img.cols * cam_e_p[0] / e_z + 0.5*img.cols;
		float e_v = 0.5*img.cols * cam_e_p[1] / e_z + 0.5*img.rows;

		cv::Point2f cvs(s_u, s_v);
		cv::Point2f cve(e_u, e_v);

		std::cout << "the s" << i << " " << s_u <<" " << s_v << std::endl;
		std::cout << "the e" << i << " " << e_u << " "<< e_v << std::endl;

		cv::line(img, cvs, cve, cv::Scalar(255, 255, 0), 3);
		cv::circle(img, cvs, 3, cv::Scalar(0, 255, 255), 3);
	}

	//获取选中的图像
	cv::namedWindow("scene line", cv::WINDOW_AUTOSIZE);
	cv::imshow("scene line", img);
	cv::waitKey(0);
}

void SceneElements::DrawAllWorldLyProjectLine2PickedImg(int idx, cv::Mat& img)
{
	//
	if (idx < 0 || idx >= model_cam_data_.GetModelCamerasNum())
	{
		std::cout << "cam vector: idx out of range! " << std::endl;
		return;
	}
	//获取对应的线段
	for (int i = 0; i < scene_cams_proj_ly_data_[idx].LinesSize(); ++i)
	{
		Eigen::Vector3f s, e;
		scene_cams_proj_ly_data_[idx].GetPickedLineSeg(i, s, e);
		cv::Point2f cvs(s[0], s[1]);
		cv::Point2f cve(e[0], e[1]);
		cv::line(img, cvs, cve, cv::Scalar(255, 255, 0), 3);
		cv::circle(img, cvs, 3, cv::Scalar(0, 255, 255), 3);
	}
}

void SceneElements::DrawOriginCamLineSegPickedImg(int idx, cv::Mat& img)
{
	if (idx < 0 || idx >= model_cam_data_.GetModelCamerasNum())
	{
		std::cout << "cam vector: idx out of range! " << std::endl;
		return;
	}
	//获取对应的线段
	for (int i = 0; i < scene_cams_ly_data_[idx].LinesSize(); ++i)
	{
		Eigen::Vector3f s, e;
		scene_cams_ly_data_[idx].GetPickedLineSeg(i, s, e);
		cv::Point2f cvs(s[0], s[1]);
		cv::Point2f cve(e[0], e[1]);
		cv::line(img, cvs, cve, cv::Scalar(255, 0, 0), 3);
		cv::circle(img, cvs, 3, cv::Scalar(0, 255, 0), 3);
	}
}

void SceneElements::ShowAllWorldLyProjLyAndOriginLinesPickedImg(int idx)
{
	if (idx < 0 || idx >= model_cam_data_.GetModelCamerasNum())
	{
		std::cout << "cam vector: idx out of range! " << std::endl;
		return;
	}
	//获取选中的图片
	std::string picked_name = model_cam_data_.GetPickedCamImagePath(idx);
	cv::Mat img = cv::imread(picked_name);
	DrawOriginCamLineSegPickedImg(idx, img);
	DrawAllWorldLyProjectLine2PickedImg(idx, img);
	
	//获取选中的图像
	cv::namedWindow("scene line", cv::WINDOW_AUTOSIZE);
	cv::imshow("scene line", img);
	cv::waitKey(0);
}

void SceneElements::RunWorldLy2AllCamsImgLy()
{
	//
	std::cout << "begin to run the project layout process..." << std::endl;

	scene3D2scene2D_.resize(model_cam_data_.GetModelCamerasNum());

	//std::string mypath = "D:\\vc_project_new\\huawei_data_indoor\\room\\test_room_cam\\lineselect.obj";
	//Eigen::Vector3f s_13, e_13;
	//scene_world_ly_data_.GetPickedLineSeg(13, s_13, e_13);
	//scene_world_ly_data_.WriteSelected3DLineIntoObj(s_13, e_13, mypath);

	for (int i = 0; i < model_cam_data_.GetModelCamerasNum(); ++i)
	{
		//std::cout << "the cam idx: " << i << std::endl;
		SceneLayout scene2d;
		Ly3DToLy2D tmp_w2c;
		ComputeWorldly2SelectedImgLy(i, scene2d, tmp_w2c);
		scene_cams_proj_ly_data_.emplace_back(scene2d);
		scene3D2scene2D_[i] = tmp_w2c;
	}
	std::cout << "end the project layout process" << std::endl;
}

void SceneElements::ComputeWorldly2SelectedImgLy(int cam_idx, SceneLayout& scene2d_ly, Ly3DToLy2D& wly2cly)
{
	if (cam_idx < 0 || cam_idx >= model_cam_data_.GetModelCamerasNum())
	{
		std::cout << "cam vector: idx out of range! " << std::endl;
		return;
	}
	//获取选中的图片
	std::string picked_name = model_cam_data_.GetPickedCamImagePath(cam_idx);
	cv::Mat img = cv::imread(picked_name);
	CameraModel picked_model;
	model_cam_data_.GetPickedCamModel(cam_idx, picked_model);
	//获取相机的渲染的depth
	cv::Mat depth_img(img.size(), CV_32F);
	//获取渲染的depth，这些需要后续在处理

	std::vector<Eigen::Vector2f> lyout_pnts2d;
	//std::cout << "the scene world layout size: " << scene_world_ly_data_.LinesSize() << std::endl;

	//获取世界坐标系投影到对应相机后的线段，13,14;找到离它最近的相机坐标系 scene_world_ly_data_.LinesSize()
	for (int i = 0; i < scene_world_ly_data_.LinesSize(); ++i)
	{
		Eigen::Vector3f s, e;
		scene_world_ly_data_.GetPickedLineSeg(i, s, e);
		/*std::cout << "the s, e: " << s[0] << " " << s[1] << " " <<s[2] 
			<< ", " << e[0] << " " << e[1] << " " << e[2] << std::endl;*/
		////获取线段
		//Eigen::Vector3f direct_3d = e - s;
		//Eigen::Vector3f s_new = s + direct_3d * 0.7;

		//获取图片的line上的点
		std::vector<Eigen::Vector2f> lseg_pnts;
		if (GetImglineCorrespondingWorldLine2D(s, e, img, depth_img, picked_model, lseg_pnts))
		{
			std::cout << "the project s, e pnts: " << lseg_pnts[0][0] << " " << lseg_pnts[0][1] << ", " 
				<< lseg_pnts[1][0] << " " << lseg_pnts[1][1] << std::endl;

			lyout_pnts2d.emplace_back(lseg_pnts[0]);
			lyout_pnts2d.emplace_back(lseg_pnts[1]);
			int w_idx = i;
			int c_idx = (lyout_pnts2d.size()/2) - 1;
			std::pair<int, int> w2c = std::make_pair(w_idx, c_idx);
			wly2cly.L3ToL2Vec.emplace_back(w2c);
		}	
	}
	std::cout << "the cam idx: " << cam_idx << " " <<
		"the lines in img number: " << lyout_pnts2d.size() / 2 << std::endl;
	scene2d_ly.SetSceneLayout2D(lyout_pnts2d);
}

bool SceneElements::GetImglineCorrespondingWorldLine2D(Eigen::Vector3f& s, Eigen::Vector3f& e,
	cv::Mat& img, cv::Mat& depth_img, CameraModel& cam,
	std::vector<Eigen::Vector2f>& lseg_pnts)
{
	//线段的可视性，需要后续补上
	bool visiability_flag = false;
#if 1
	//to do next...
	//通过depth img进行处理
	visiability_flag = true;
	if (visiability_flag)
	{
		Eigen::Matrix4f picked_cam_pose = cam.cam_pose_;
		Eigen::Vector4f s_p = Eigen::Vector4f(s[0], s[1], s[2], 1.0);
		//Eigen::Vector4f e_p = Eigen::Vector4f(e[0], e[1], e[2], 1.0);
		Eigen::Vector4f cam_s_p = picked_cam_pose.inverse() * s_p;	//相机坐标系下的顶点坐标
		float p_c_s_z = cam_s_p[2];

		Eigen::Vector4f e_p = Eigen::Vector4f(e[0], e[1], e[2], 1.0);
		Eigen::Vector4f cam_e_p = picked_cam_pose.inverse() * e_p;	//相机坐标系下的顶点坐标
		float p_c_e_z = cam_e_p[2];
		//std::cout << "the s p: \n" << s_p << std::endl;
		//std::cout << "the e p: \n" << e_p << std::endl;
		//std::cout << "the cam is: \n" << picked_cam_pose << std::endl;
		//std::cout << "the cam intrinsic: " << cam.fx_ << " " << cam.fy_ << " " << cam.cx_ << " " << cam.cy_ << std::endl;

		if (p_c_s_z < 1e-3 && p_c_e_z < 1e-3)
		{
			return false;
		}
		else if (p_c_s_z > 1e-3 && p_c_e_z < 1e-3)
		{
			//表示le顶点看不到，但是ls顶点能看到，所以部分线段能够看到
			float s_u = cam.fx_ * cam_s_p[0] / p_c_s_z + cam.cx_;
			float s_v = cam.fy_ * cam_s_p[1] / p_c_s_z + cam.cy_;

			float e_u = cam.fx_ * cam_e_p[0] / p_c_e_z + cam.cx_;
			float e_v = cam.fy_ * cam_e_p[1] / p_c_e_z + cam.cy_;
			/*std::cout << "the uv: " << s_u << " " <<s_v << ", "
			<< e_u << " " << e_v << std::endl;*/

			//处理e_p这个顶点，部分线段的表示正常
			Eigen::Vector4f direct_se = e_p - s_p;
			Eigen::Matrix4f cam_extrinsic = picked_cam_pose.inverse();
			Eigen::Vector4f z_vec = Eigen::Vector4f(cam_extrinsic(2,0), cam_extrinsic(2, 1), cam_extrinsic(2, 2), cam_extrinsic(2, 3));

			float variable_error = 0.1;
			float s_value = z_vec.dot(s_p) - variable_error;
			float d_value = direct_se.dot(z_vec);
			float k = -s_value / d_value;
			Eigen::Vector4f e_p_new = s_p + k * direct_se;
			
			Eigen::Vector4f cam_e_p_new = cam_extrinsic * e_p_new;	//相机坐标系下的顶点坐标
			float e_u_new = cam.fx_ * cam_e_p_new[0] / cam_e_p_new[2] + cam.cx_;
			float e_v_new = cam.fy_ * cam_e_p_new[1] / cam_e_p_new[2] + cam.cy_;

			Eigen::Vector2f proj_s_pnt = Eigen::Vector2f(s_u, s_v);
			Eigen::Vector2f proj_e_pnt = Eigen::Vector2f(e_u_new, e_v_new);

			if ((proj_s_pnt[0] < img.cols && proj_s_pnt[0] > 0 && proj_s_pnt[1] < img.rows && proj_s_pnt[1] > 0) ||
				(proj_e_pnt[0] < img.cols && proj_e_pnt[0] > 0 && proj_e_pnt[1] < img.rows && proj_e_pnt[1] > 0))
			{
				lseg_pnts.emplace_back(proj_s_pnt);
				lseg_pnts.emplace_back(proj_e_pnt);
				return true;
			}
		}
		else if (p_c_s_z < 1e-3 && p_c_e_z > 1e-3)
		{
			//表示ls顶点看不到，但是le顶点能看到，所以部分线段能够看到
			float s_u = cam.fx_ * cam_s_p[0] / p_c_s_z + cam.cx_;
			float s_v = cam.fy_ * cam_s_p[1] / p_c_s_z + cam.cy_;

			float e_u = cam.fx_ * cam_e_p[0] / p_c_e_z + cam.cx_;
			float e_v = cam.fy_ * cam_e_p[1] / p_c_e_z + cam.cy_;
			/*std::cout << "the uv: " << s_u << " " <<s_v << ", "
			<< e_u << " " << e_v << std::endl;*/

			//处理s_p这个顶点，部分线段的表示正常
			Eigen::Vector4f direct_se = e_p - s_p;
			Eigen::Matrix4f cam_extrinsic = picked_cam_pose.inverse();
			Eigen::Vector4f z_vec = Eigen::Vector4f(cam_extrinsic(2, 0), cam_extrinsic(2, 1), cam_extrinsic(2, 2), cam_extrinsic(2, 3));

			float variable_error = 0.1;
			float s_value = z_vec.dot(s_p) - variable_error;
			float d_value = direct_se.dot(z_vec);
			float k = -s_value / d_value;
			Eigen::Vector4f s_p_new = s_p + k * direct_se;

			Eigen::Vector4f cam_s_p_new = cam_extrinsic * s_p_new;	//相机坐标系下的顶点坐标
			float s_u_new = cam.fx_ * cam_s_p_new[0] / cam_s_p_new[2] + cam.cx_;
			float s_v_new = cam.fy_ * cam_s_p_new[1] / cam_s_p_new[2] + cam.cy_;

			Eigen::Vector2f proj_s_pnt = Eigen::Vector2f(s_u_new, s_v_new);
			Eigen::Vector2f proj_e_pnt = Eigen::Vector2f(e_u, e_v);

			if ((proj_s_pnt[0] < img.cols && proj_s_pnt[0] > 0 && proj_s_pnt[1] < img.rows && proj_s_pnt[1] > 0) ||
				(proj_e_pnt[0] < img.cols && proj_e_pnt[0] > 0 && proj_e_pnt[1] < img.rows && proj_e_pnt[1] > 0))
			{
				lseg_pnts.emplace_back(proj_s_pnt);
				lseg_pnts.emplace_back(proj_e_pnt);
				return true;
			}
		}
		else   //表示两个线段的端点都能看到,所以需要这样判断
		{
			float s_u = cam.fx_ * cam_s_p[0] / p_c_s_z + cam.cx_;
			float s_v = cam.fy_ * cam_s_p[1] / p_c_s_z + cam.cy_;

			float e_u = cam.fx_ * cam_e_p[0] / p_c_e_z + cam.cx_;
			float e_v = cam.fy_ * cam_e_p[1] / p_c_e_z + cam.cy_;
			/*std::cout << "the uv: " << s_u << " " <<s_v << ", "
			<< e_u << " " << e_v << std::endl;*/

			Eigen::Vector2f proj_s_pnt = Eigen::Vector2f(s_u, s_v);
			Eigen::Vector2f proj_e_pnt = Eigen::Vector2f(e_u, e_v);

			if ((proj_s_pnt[0] < img.cols && proj_s_pnt[0] > 0 && proj_s_pnt[1] < img.rows && proj_s_pnt[1] > 0) ||
				(proj_e_pnt[0] < img.cols && proj_e_pnt[0] > 0 && proj_e_pnt[1] < img.rows && proj_e_pnt[1] > 0))
			{
				lseg_pnts.emplace_back(proj_s_pnt);
				lseg_pnts.emplace_back(proj_e_pnt);
				return true;
			}
		}
		return false;
	}
	else
	{
		return false;
	}
#endif
	//return visiability_flag;
}

void SceneElements::LineSeg2dProj2PickedCam(int cam_idx, Eigen::Vector3f& ls3d, Eigen::Vector3f& le3d,
	Eigen::Vector2f& ls2d, Eigen::Vector2f& le2d)
{
	//
	if (cam_idx < 0 || cam_idx >= model_cam_data_.GetModelCamerasNum())
	{
		std::cout << "cam vector: idx out of range! " << std::endl;
	}
	////获取选中的图片
	//std::string picked_name = model_cam_data_.GetPickedCamImagePath(cam_idx);
	//cv::Mat img = cv::imread(picked_name);
	Eigen::Matrix4f picked_cam_pose;
	//model_cam_data_.GetPickedCamPose(cam_idx, picked_cam_pose);
	CameraModel picked_cam_model;
	model_cam_data_.GetPickedCamModel(cam_idx, picked_cam_model);
	picked_cam_pose = picked_cam_model.cam_pose_;
	std::cout << "the picked cam pose is: \n" << picked_cam_pose << std::endl;
	Eigen::Vector3f picked_cam_z = picked_cam_pose.block(0, 2, 3, 1);
	std::cout << "the z is: \n" << picked_cam_z << std::endl;

	Eigen::Vector2f start_img_pos;
	Eigen::Vector2f end_img_pos;

	//project to img
	Eigen::Vector4f s_pnt = Eigen::Vector4f(ls3d[0], ls3d[1], ls3d[2], 1.0);
	Eigen::Vector4f e_pnt = Eigen::Vector4f(le3d[0], le3d[1], le3d[2], 1.0);
	Eigen::Vector4f cam_s_p = picked_cam_pose.inverse() * s_pnt;	//相机坐标系下的顶点坐标
	Eigen::Vector4f cam_e_p = picked_cam_pose.inverse() * e_pnt;	//相机坐标系下的顶点坐标
	float s_z = cam_s_p[2];
	float e_z = cam_e_p[2];

	float s_u = picked_cam_model.fx_*cam_s_p[0] / s_z + picked_cam_model.cx_;
	float s_v = picked_cam_model.fy_*cam_s_p[1] / s_z + picked_cam_model.cy_;

	float e_u = picked_cam_model.fx_ * cam_e_p[0] / e_z + picked_cam_model.cx_;
	float e_v = picked_cam_model.fy_ * cam_e_p[1] / e_z + picked_cam_model.cy_;

	ls2d[0] = s_u;
	ls2d[1] = s_v;
	le2d[0] = e_u;
	le2d[1] = e_v;
}

std::vector<cv::Point2i> SceneElements::ExtractPntsFromLineSegInImgCoord(Eigen::Vector2f& ls, Eigen::Vector2f& le, cv::Mat& img)
{
	std::vector<cv::Point2i> lsegpnts_vec;
	std::vector<Eigen::Vector2f> rect_lsegs;

	//矩形的大小
	Eigen::Vector2f min_cor(0, 0);
	Eigen::Vector2f max_cor(img.cols, img.rows);
	
	//生成4个线段
	SplitRectInto4LineSeg2D(min_cor, max_cor, rect_lsegs);

	//转化类型
	Eigen::Vector2i lsi = ConvertPntf2Pnti(ls);
	Eigen::Vector2i lei = ConvertPntf2Pnti(le);
	Eigen::Vector2i min_cori = ConvertPntf2Pnti(min_cor);
	Eigen::Vector2i max_cori = ConvertPntf2Pnti(max_cor);

	//都在矩形框里面
	if (CheckPoint2dInRect(ls, min_cor, max_cor) && CheckPoint2dInRect(le, min_cor, max_cor))
	{
		ExtractPnts2DFromLineSegInImg(lsi, lei, min_cori, max_cori, lsegpnts_vec);
	}
	//只有初始点在矩形框里面
	else if (CheckPoint2dInRect(ls, min_cor, max_cor) && !CheckPoint2dInRect(le, min_cor, max_cor))
	{
		//获取和图像的交点
		Eigen::Vector2f cross_p2d;
		for (int i = 0; i < rect_lsegs.size() / 2; ++i)
		{
			Eigen::Vector2f rls = rect_lsegs[2 * i];
			Eigen::Vector2f rle = rect_lsegs[2 * i + 1];
			if (CheckTwoLinesSegIntersect(ls, le, rls, rle))
			{
				cross_p2d = ComputeTwoLinesCrossPnt(ls, le, rls, rle);
				break;
			}
		}
		//获取交点
		//Eigen::Vector2f added_le = cross_p2d;
		Eigen::Vector2i added_lei = ConvertPntf2Pnti(cross_p2d);
		ExtractPnts2DFromLineSegInImg(lsi, added_lei, min_cori, max_cori, lsegpnts_vec);
	}
	//只有结束点在矩形框里面
	else if (!CheckPoint2dInRect(ls, min_cor, max_cor) && CheckPoint2dInRect(le, min_cor, max_cor))
	{
		//获取和图像的交点
		Eigen::Vector2f cross_p2d;
		for (int i = 0; i < rect_lsegs.size() / 2; ++i)
		{
			Eigen::Vector2f rls = rect_lsegs[2 * i];
			Eigen::Vector2f rle = rect_lsegs[2 * i + 1];
			if (CheckTwoLinesSegIntersect(ls, le, rls, rle))
			{
				cross_p2d = ComputeTwoLinesCrossPnt(ls, le, rls, rle);
				break;
			}
		}
		//获取交点
		//Eigen::Vector2f added_ls = cross_p2d;
		Eigen::Vector2i added_lsi =ConvertPntf2Pnti(cross_p2d);
		ExtractPnts2DFromLineSegInImg(added_lsi, lei, min_cori, max_cori, lsegpnts_vec);
	}
	else//都在矩形外面，需要重新处理
	{
		//这个是另外处理一下
		//获取和图像的交点，如果和两个边相交，这样就可以处理
		std::vector<Eigen::Vector2f> cross_pnts2d;
		std::vector<int> pl;
		for (int i = 0; i < rect_lsegs.size() / 2; ++i)
		{
			Eigen::Vector2f rls = rect_lsegs[2 * i];
			Eigen::Vector2f rle = rect_lsegs[2 * i + 1];
			if (CheckTwoLinesSegIntersect(ls, le, rls, rle))
			{
				Eigen::Vector2f pnt2d = ComputeTwoLinesCrossPnt(ls, le, rls, rle);
				cross_pnts2d.emplace_back(pnt2d);
				pl.emplace_back(i);
			}
		}

		if (cross_pnts2d.size() == 2)
		{
			float ls2cs0 = Pnt2d2Pnt2dDist(ls, cross_pnts2d[0]);
			float le2cs1 = Pnt2d2Pnt2dDist(ls, cross_pnts2d[1]);

			Eigen::Vector2i cross_i0 = ConvertPntf2Pnti(cross_pnts2d[0]);
			Eigen::Vector2i cross_i1 = ConvertPntf2Pnti(cross_pnts2d[1]);

			if (ls2cs0 < le2cs1)
			{
				//起始顶点为cross_pnts2d[0],结束点为cross_pnts2d[1]
				ExtractPnts2DFromLineSegInImg(cross_i0, cross_i1, min_cori, max_cori, lsegpnts_vec);
			}
			else
			{
				//起始顶点为cross_pnts2d[1]，结束点为cross_pnts2d[0]
				ExtractPnts2DFromLineSegInImg(cross_i1, cross_i0, min_cori, max_cori, lsegpnts_vec);
			}
		}
	}//
	//获取在图像上的顶点坐标
	return lsegpnts_vec;
}

Eigen::Vector2i SceneElements::ConvertPntf2Pnti(Eigen::Vector2f& pnt)
{
	Eigen::Vector2i tmp;
	if (std::abs(pnt[0] - std::floor(pnt[0])) < 0.5)
		tmp[0] = std::floor(pnt[0]);
	else
		tmp[0] = std::ceil(pnt[0]);
	if (std::abs(pnt[1] - std::floor(pnt[1])) < 0.5)
		tmp[1] = std::floor(pnt[1]);
	else
		tmp[1] = std::ceil(pnt[1]);
	return tmp;
}

void SceneElements::ExtractPnts2DFromLineSegInImg(Eigen::Vector2i& ls, Eigen::Vector2i& le,
	Eigen::Vector2i& mincor, Eigen::Vector2i& maxcor, std::vector<cv::Point2i>& pnts2d)
{
	if (std::abs((le - ls).norm()) < 1e-4)
		return;
	
	int dx = int(le[0] - ls[0]);
	int dy = int(le[1] - ls[1]);

	int ux = ((dx > 0) << 1) - 1;	//x的增量方向，取或-1
	int uy = ((dy > 0) << 1) - 1;	//y的增量方向，取或-1

	int x = ls[0], y = ls[1], eps;
	eps = 0; dx = std::abs(dx); dy = std::abs(dy);
	if (dx > dy)
	{
		for (x = le[0]; x != le[x]; x += ux)
		{
			pnts2d.emplace_back(cv::Point2i(x,y));
			eps += dy;
			if ((eps << 1) >= dx)
			{
				y += uy;
				eps -= dx;
			}
		}
	}
	else
	{
		for (y = ls[1]; y != le[1]; y += uy)
		{
			pnts2d.emplace_back(cv::Point2i(x, y));
			eps += dx;
			if ((eps << 1) >= dy)
			{
				x += ux;
				eps -= dy;
			}
		}
	}
}

void SceneElements::SplitRectInto4LineSeg2D(Eigen::Vector2f& min_cor, Eigen::Vector2f& max_cor, std::vector<Eigen::Vector2f>& linesCors)
{
	//矩形的4个线段, 逆时针方向
	linesCors.emplace_back(min_cor);
	linesCors.emplace_back(Eigen::Vector2f(min_cor[0], max_cor[1]));
	
	linesCors.emplace_back(Eigen::Vector2f(min_cor[0], max_cor[1]));
	linesCors.emplace_back(max_cor);

	linesCors.emplace_back(max_cor);
	linesCors.emplace_back(Eigen::Vector2f(max_cor[0], min_cor[1]));

	linesCors.emplace_back(Eigen::Vector2f(max_cor[0], min_cor[1]));
	linesCors.emplace_back(min_cor);
}

void SceneElements::LineSeg2LineSegDist0(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e, Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e)
{
	//
	Eigen::Vector3f lt_f;
	//目标的线段组成的函数
	if (ComputeFunctionFromTwoPnts(lt_s, lt_e, lt_f))
	{
		//从source线段中找到每个顶点，从而获取它到线段的距离
		
	}
}

float SceneElements::LineSeg2LineSegDistImgCoord(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e,
	Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e, cv::Mat& img)
{
	float lambda0 = 2.0, lambda1 = 1.0;
	float value = lambda0 * LineSeg2LineSegDistPntsImgCoord(ls_s, ls_e, lt_s, lt_e, img)
		+ lambda1 * LineSeg2LineSegCrossValueImgCoord(ls_s, ls_e, lt_s, lt_e);
	return value;
}

float SceneElements::LineSeg2LineSegCrossValueImgCoord(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e,
	Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e)
{
	//计算两个线段的夹角
	Eigen::Vector2f lsv = ls_e - ls_s;
	Eigen::Vector2f ltv = lt_e - lt_s;

	if (std::abs(lsv.norm()*ltv.norm()) < 1e-4)
		return std::numeric_limits<float>::max();

	float crossv = std::abs(lsv.dot(ltv)) / (lsv.norm()*ltv.norm());
	//return crossv;
	return std::acos(crossv);
}

float SceneElements::LineSeg2LineSegDistPntsImgCoord(Eigen::Vector2f& ls_s, Eigen::Vector2f& ls_e,
	Eigen::Vector2f& lt_s, Eigen::Vector2f& lt_e, cv::Mat& img)
{
	//从(ls_s,ls_e)在图像上的顶点到(lt_s,lt_e)的距离，以及它们的数量
	std::vector<cv::Point2i> ls_img_vec = ExtractPntsFromLineSegInImgCoord(ls_s, ls_e, img);
	float sums2t = 0.0f;
	for (int i = 0; i < ls_img_vec.size(); ++i)
	{
		Eigen::Vector2f p(ls_img_vec[i].x, ls_img_vec[i].y);
		float dist = Pnt2dToLineSegDist(p, lt_s, lt_e);
		sums2t += dist;
	}
	float average_s2t = sums2t / ls_img_vec.size();

	float sumt2s = 0.0;
	//从(lt_s,lt_e)在图像上的顶点到(ls_s,ls_e)的距离，以及它们的数量
	std::vector<cv::Point2i> lt_img_vec = ExtractPntsFromLineSegInImgCoord(lt_s, lt_e, img);
	for (int i = 0; i < lt_img_vec.size(); ++i)
	{
		Eigen::Vector2f p(lt_img_vec[i].x, lt_img_vec[i].y);
		float dist = Pnt2dToLineSegDist(p, ls_s, ls_e);
		sumt2s += dist;
	}
	float average_t2s = sumt2s / lt_img_vec.size();

	return ((average_s2t + average_t2s) / 2);
}

//f:Ax+By+C = 0->A = y2-y1; B=x1-x2; C=x2*y1-x1*y2
bool SceneElements::ComputeFunctionFromTwoPnts(Eigen::Vector2f& l_s, Eigen::Vector2f& l_e, Eigen::Vector3f& f2d)
{
	if (std::abs((l_e - l_s).norm()) < 1e-4)
		return false;
	f2d[0] = l_e[1] - l_s[1];
	f2d[1] = l_s[0] - l_e[0];
	f2d[2] = l_e[0] * l_s[1] - l_s[0] * l_e[1];
	return true;

	//if (std::abs(l_e[1] - l_s[1]) < std::abs(l_e[0] - l_s[0]))
	//{
	//	//y = kx+b -> kx-y+b = 0;
	//	float k = (l_e[1] - l_s[1]) / (l_e[0] - l_s[0]);
	//	float b = l_e[1] - k * l_e[0];
	//	f2d[0] = k;
	//	f2d[1] = -1;
	//	f2d[2] = b;
	//}
	//else
	//{
	//	//x = ky+b -> ky-x+b = 0;
	//	float k = (l_e[0] - l_s[0]) / (l_e[1] - l_s[1]);
	//	float b = l_e[0] - k * l_e[1];
	//	f2d[0] = -1;
	//	f2d[1] = k;
	//	f2d[2] = b;
	//}
}

float SceneElements::Compute2Line2DScore(Eigen::Vector2f& ss, Eigen::Vector2f& se, Eigen::Vector2f& ts, Eigen::Vector2f& te)
{
	return 0.0;
}

float SceneElements::Pnt2dToLineDist(Eigen::Vector2f pnt2d, Eigen::Vector3f f_2d)
{
	Eigen::Vector3f pnt_affine = Eigen::Vector3f(pnt2d[0], pnt2d[1], 1.0);
	if (std::powf(f_2d[0], 2) + std::powf(f_2d[0], 2) < 1e-6)
		return std::numeric_limits<float>::max();
	float dist = std::abs(f_2d.dot(pnt_affine)) / std::sqrtf(std::powf(f_2d[0], 2) + std::powf(f_2d[0], 2));
	return dist;
}

float SceneElements::Pnt2dToLineSegDist(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le)
{
	if (ls == le)
		return (pnt2d - ls).norm();

	Eigen::Vector2f v1 = le - ls, v2 = pnt2d - ls, v3 = pnt2d - le;
	if (v1.dot(v2) < 1e-4)
		return v2.norm();
	else if (v1.dot(v3) > 1e-4)
		return v3.norm();
	else
	{
		//叉积
		float crossv = v1[0] * v2[1] - v2[0] * v1[1];
		return crossv / v1.norm();
	}
}

float SceneElements::Pnt2d2Pnt2dDist(Eigen::Vector2f& pnt0, Eigen::Vector2f& pnt1)
{
	return (pnt1 - pnt0).norm();
}

void SceneElements::Pnt2dProj2LineSeg(Eigen::Vector2f& pnt2d, Eigen::Vector2f& ls, Eigen::Vector2f& le,
	Eigen::Vector2f& proj_pnt2d)
{
	//
	Eigen::Vector2f lse = le - ls;	//P2-P1
	Eigen::Vector2f lsp = pnt2d - ls;	//P3-P1
	if (std::abs(lse.norm()) < 1e-4)
	{
		std::cout << "project to pnt because of line(pnt)" << std::endl;
		proj_pnt2d = ls;
	}
	else
	{
		float k = lsp.dot(lse) / (lse.norm()*lse.norm());
		proj_pnt2d = ls + k * lse;
	}
}

bool SceneElements::Point2dAtLineSegLeftRight(Eigen::Vector2f& pnt, Eigen::Vector2f& ls, Eigen::Vector2f& le)
{
	Eigen::Vector2f ps = ls - pnt;
	Eigen::Vector2f pe = le - pnt;
	float nRet = ps[0] * pe[1] - ps[1] * pe[0];
	if (std::abs(nRet) < 1e-4)
		return 0;
	else if (nRet > 1e-4)
		return 1;
	else
		return -1;
}

bool SceneElements::CheckTwoLinesSegIntersect(Eigen::Vector2f& ss, Eigen::Vector2f& se,
	Eigen::Vector2f& ts, Eigen::Vector2f& te)
{
	int a = Point2dAtLineSegLeftRight(ss, ts, te);
	int b = Point2dAtLineSegLeftRight(se, ts, te);
	if (a*b > 0)
		return false;
	a = Point2dAtLineSegLeftRight(ts, ss, se);
	b = Point2dAtLineSegLeftRight(te, ss, se);
	if (a*b > 0)
		return false;
	return true;
}

Eigen::Vector2f SceneElements::ComputeTwoLinesCrossPnt(Eigen::Vector2f& ss, Eigen::Vector2f& se,
	Eigen::Vector2f& ts, Eigen::Vector2f& te)
{
	if (CheckTwoLinesSegIntersect(ss, se, ts, te))
	{
		Eigen::Vector3f fs;
		Eigen::Vector3f fe;
		ComputeFunctionFromTwoPnts(ss, se, fs);
		ComputeFunctionFromTwoPnts(ss, se, fe);
		//两个线段，计算它们的交点
		Eigen::Matrix2f A;
		Eigen::Vector2f B;
		A(0, 0) = fs[0];
		A(0, 1) = fs[1];
		A(1, 0) = fe[0];
		A(1, 1) = fe[1];
		B[0] = -fs[2];
		B[1] = -fe[2];
		Eigen::Vector2f cross_p = A.inverse()*B;
		return cross_p;
	}
	//判断线段的端点是否在线段上
	/*if (Point2dAtLineSegLeftRight(ss, ts, te) == 0)
	{
		return ss;
	}
	if (Point2dAtLineSegLeftRight(se, ts, te) == 0)
	{
		return se;
	}
	if (Point2dAtLineSegLeftRight(ts, ss, se) == 0)
	{
		return ts;
	}
	if (Point2dAtLineSegLeftRight(te, ss, se) == 0)
	{
		return te;
	}*/
}

float SceneElements::FindNearestSegFromSelected2DLayout(int img_idx,
	Eigen::Vector2f& ss, Eigen::Vector2f& se, Eigen::Vector2i& cmpidx)
{
	//获取线段，然后匹配线段，找到ss se在 图像img_idx中的layout最近的线段，同时计算它们距离
	//获取img_idx的layout
	float min_dist = std::numeric_limits<float>::max();
	SceneLayout cam_lyout = scene_cams_ly_data_[img_idx];
	std::vector<LinePoint> cam_ly_pnts = cam_lyout.GetLayoutPnts();
	std::vector<Eigen::Vector2i> cam_ly_idxs =  cam_lyout.GetLayoutLine2PntIdx();
	for (int i = 0; i < cam_ly_idxs.size(); ++i)
	{
		//
		Eigen::Vector3f cls = cam_ly_pnts[cam_ly_idxs[i][0]].pnt_;
		Eigen::Vector3f cle = cam_ly_pnts[cam_ly_idxs[i][1]].pnt_;
		Eigen::Vector2f cls_2d = Eigen::Vector2f(cls[0], cls[1]);
		Eigen::Vector2f cle_2d = Eigen::Vector2f(cle[0], cle[1]);
		
		//计算两个线段的距离
		//std::cout << "the line2d src: " << cls_2d[0] << " " << cls_2d[1] << ",  " << cle_2d[0] << " " << cle_2d[1] << std::endl;
		//std::cout << "the line2d target: " << ss[0] << " " << ss[1] << ",  " << se[0] << " " << se[1] << std::endl;

		float crossvalue = LineSeg2LineSegCrossValueImgCoord(cls_2d, cle_2d, ss, se);
		float s2l = Pnt2dToLineSegDist(ss, cls_2d, cle_2d);
		float e2l = Pnt2dToLineSegDist(se, cls_2d, cle_2d);

		//std::cout << "the cross value: " << crossvalue << std::endl;
		//std::cout << "the s2l value: " << s2l << std::endl;
		//std::cout << "the e2l value: " << e2l << std::endl;
		//
		float sumv = std::abs(crossvalue) + std::abs(s2l)/100 + std::abs(e2l)/100;	//可以通过lambda
		if (min_dist > sumv)
		{
			cmpidx[0] = img_idx;
			cmpidx[1] = i;
			min_dist = sumv;
		}
	}

	return min_dist;
}

float SceneElements::FindPntNearestSegFromSelected2DLayout(int img_idx,
	Eigen::Vector2f& ss, Eigen::Vector2i& cmpidx)
{
	//对layout寻找离线段最近的
	//获取线段，然后匹配线段，找到ss se在 图像img_idx中的layout最近的线段，同时计算它们距离
	//获取img_idx的layout
	float min_dist = std::numeric_limits<float>::max();
	SceneLayout cam_lyout = scene_cams_ly_data_[img_idx];
	std::vector<LinePoint> cam_ly_pnts = cam_lyout.GetLayoutPnts();
	std::vector<Eigen::Vector2i> cam_ly_idxs = cam_lyout.GetLayoutLine2PntIdx();
	for (int i = 0; i < cam_ly_idxs.size(); ++i)
	{
		//
		Eigen::Vector3f cls = cam_ly_pnts[cam_ly_idxs[i][0]].pnt_;
		Eigen::Vector3f cle = cam_ly_pnts[cam_ly_idxs[i][1]].pnt_;
		Eigen::Vector2f cls_2d = Eigen::Vector2f(cls[0], cls[1]);
		Eigen::Vector2f cle_2d = Eigen::Vector2f(cle[0], cle[1]);

		//计算两个线段的距离
		//std::cout << "the line2d src: " << cls_2d[0] << " " << cls_2d[1] << ",  " << cle_2d[0] << " " << cle_2d[1] << std::endl;
		//std::cout << "the line2d target: " << ss[0] << " " << ss[1] << ",  " << se[0] << " " << se[1] << std::endl;

		//float crossvalue = LineSeg2LineSegCrossValueImgCoord(cls_2d, cle_2d, ss, se);
		float s2l = Pnt2dToLineSegDist(ss, cls_2d, cle_2d);

		//std::cout << "the cross value: " << crossvalue << std::endl;
		//std::cout << "the s2l value: " << s2l << std::endl;
		//std::cout << "the e2l value: " << e2l << std::endl;
		//
		float sumv = std::abs(s2l);	//可以通过lambda
		if (min_dist > sumv)
		{
			cmpidx[0] = img_idx;
			cmpidx[1] = i;
			min_dist = sumv;
		}
	}
	return min_dist;
}

void SceneElements::WriteWorldSceneLinesIntoObj(const std::string& path)
{
	//
	scene_world_ly_data_.Write3DLinesIntoObj(path);
}

bool SceneElements::CheckLineSegIntersectRect(Eigen::Vector2f& ls, Eigen::Vector2f& le, Eigen::Vector2f& topleft, Eigen::Vector2f& bottomright)
{
	//
	return false;
}

int SceneElements::LineSegIntersectRectPnt(Eigen::Vector2f& ls, Eigen::Vector2f& le,
	Eigen::Vector2f& topleft, Eigen::Vector2f& bottomright, Eigen::Vector2f& cross_pnt0, Eigen::Vector2f& cross_pnt1)
{
	//有4个边
	Eigen::Vector2f ts, te;
	ts = topleft;
	te = Eigen::Vector2f(bottomright[0], topleft[1]);


	ts = Eigen::Vector2f(bottomright[0], topleft[1]);
	te = bottomright;

	ts = bottomright;
	te = Eigen::Vector2f(topleft[0], bottomright[1]);

	ts = Eigen::Vector2f(topleft[0], bottomright[1]);
	te = topleft;
	return 0;
}

bool SceneElements::ComputeLineSegCrossLine2dNew(Eigen::Vector2f& ls, Eigen::Vector2f& le,
	Eigen::Vector2f& t_pnt, Eigen::Vector2f& t_dir, Eigen::Vector2f& cross_pnt)
{
	if (std::abs((ls - le).norm()) < 1e-4)
		return false;
	Eigen::Vector2f tmp_cross_pnt;
	//计算交点
	Eigen::Vector2f lsdir = le - ls;
	tmp_cross_pnt = ComputeTwoLinesCrossPnt2dNew(ls, lsdir, t_pnt, t_dir);

	Eigen::Vector2f l_sc = tmp_cross_pnt - ls;
	Eigen::Vector2f l_ec = le - ls;
	float kabs = l_sc.norm() / l_ec.norm();
	if (l_sc.dot(l_ec) > 1e-8 && kabs > -1e-8 && kabs < 1.0f)
	{
		cross_pnt = tmp_cross_pnt;
		return true;
	}
	//cross_pnt = tmp_cross_pnt;
	return false;
}

bool SceneElements::ComputeTwoLineSegsCrossPnt2d(Eigen::Vector2f& ls, Eigen::Vector2f& le,
	Eigen::Vector2f& ts, Eigen::Vector2f& te, Eigen::Vector2f& cross_pnt)
{
	//
	Eigen::Vector2f s_dir = le - ls;
	Eigen::Vector2f t_dir = te - ts;
	if (s_dir.norm() < 1e-6 || t_dir.norm() < 1e-6)
		return false;
	s_dir.normalize();
	t_dir.normalize();
	//先判断它们是否平行
	if (std::abs(s_dir[0] - t_dir[0]) < 1e-4
		&& std::abs(s_dir[1] - t_dir[1]) < 1e-4)
	{
		std::cerr << "s_dir: " << s_dir[0] << " " << s_dir[1] << " ";
		std::cerr << "t_dir: " << t_dir[0] << " " << t_dir[1] << " " << std::endl;
		std::cerr << "two lines parallel..." << std::endl;
		return false;
	}
	Eigen::Vector2f tmp_cross_pnt = ComputeTwoLinesCrossPnt2dNew(ls, s_dir, ts, t_dir);
	Eigen::Vector2f l_sc = tmp_cross_pnt - ls;
	Eigen::Vector2f l_ec = le - ls;
	float kabs = l_sc.norm() / l_ec.norm();
	Eigen::Vector2f t_sc = tmp_cross_pnt - ts;
	Eigen::Vector2f t_ec = te - ts;
	float kabt = t_sc.norm() / t_ec.norm();
	if (l_sc.dot(l_ec) > 1e-6 && kabs > -1e-6 && kabs < 1.0f
		&& t_sc.dot(t_ec) > 1e-6 && kabt > -1e-6 && kabt < 1.0f)
	{
		cross_pnt = tmp_cross_pnt;
		return true;
	}
	return false;
}

//sp+k2*sdir=tp+k1*tdir
Eigen::Vector2f SceneElements::ComputeTwoLinesCrossPnt2dNew(Eigen::Vector2f& sp, Eigen::Vector2f& sdir,
	Eigen::Vector2f& tp, Eigen::Vector2f& tdir)
{
	Eigen::Matrix2f A;
	A << tdir[0], -sdir[0], tdir[1], -sdir[1];
	Eigen::Vector2f B;
	B[0] = sp[0] - tp[0];
	B[1] = sp[1] - tp[1];
	Eigen::Vector2f kvalue = A.fullPivHouseholderQr().solve(B);
	//std::cout << "the kvalue: " << kvalue[0] << " " << kvalue[1] << std::endl;
	Eigen::Vector2f crosspnt = sp + kvalue[1] * sdir;
	return crosspnt;
}

bool SceneElements::CheckPoint2dInRect(Eigen::Vector2f& pnt, Eigen::Vector2f& mincor, Eigen::Vector2f& maxcor)
{
	//
	if (pnt[0] >= mincor[0] && pnt[0] <= maxcor[0]
		&& pnt[1] >= mincor[1] && pnt[1] <= maxcor[1])
		return true;
	else
		return false;
}

bool SceneElements::CompareHWStrPairByIdxSceneCmp(const std::pair<int, std::string>& a,
	const std::pair<int, std::string>& b)
{
	return a.first < b.first;
}
