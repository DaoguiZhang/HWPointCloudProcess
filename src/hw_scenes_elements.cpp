#include"hw_scenes_elements.h"
#include"hw_algorithms.h"
#include<boost/filesystem.hpp>
#include<random>    //set random value
#include<map>
#include "GCO/GCoptimization.h"

namespace HW
{
	//my compare
	struct PolygonIdxCampareZDG
	{
		bool operator()(const std::pair<int, int>& p0, const std::pair<int, int>& p1)
		{
			return p0.second < p1.second;
		}
	} MyPolygonIdxCampareZDG;

	bool MyPolygonIdxCmpZDG(const std::pair<int, int>& p0, const std::pair<int, int>& p1)
	{
		return p0.second > p1.second;
	}

	bool MyLine3dSegValueCmpZDG(const std::pair<int, float>& v0, const std::pair<int, float>& v1)
	{
		return v0.second < v1.second;
	}

    HWScenesElements::HWScenesElements()
    {
        epipolar_overlap_ = KL3D_EPIPOLAR_OVERLAP;
        r_max_threshold_ = 0.5;
		r_max_polygon_threshold_ = 0.02;
		r_max_angle_threshold_ = 10.0;
		r_max_line_dist_threshold_ = 0.5;
		r_max_pnts_dist_threshold_ = 0.3;
		image_features_ratio_ = 0.2;
		cam_to_polygon_inter_line_threhold_ = 2.0;
		lines_match_angle_threshold_ = 8.0;
		lines_match_dist2line_threshold_ = 10.0;
		pnts_match_pnt2pnt_threshold_ = 10.0;
		images_labels_loaded_ = false;
		filter_wrong_lines_match_ = false;
		filter_wrong_pnts_match_ = false;
		views_initial_updated_flag_ = false;
		trl2Interl_option_.line_angle_threhold_ = 10.0;
		trl2Interl_option_.line_dist_threshold_ = 0.3;
		import_data_from_scene_elements_state_ = false;	//check the data if it is imported
		import_data_from_pairs_lines_txt_state_ = false;
		import_data_from_python_numpy_state_ = false;
		use_images_lines_pairs_imported_ = false;
		use_images_pnts_pairs_imported_ = false;
		associsated_images_labels_ = NULL;
    }

    HWScenesElements::~HWScenesElements()
    {

    }

    void HWScenesElements::SetDir(std::string& dir)
    {
        scene_dir_ = dir;
    }

    void HWScenesElements::LoadAllElements()
    {
        if(boost::filesystem::is_directory(scene_dir_))
        {
            scenes_cams_ = std::unique_ptr<HWScenesCams>(new HWScenesCams());
            scenes_cams_->SetCamsDir(scene_dir_);
            scenes_cams_->SetImagesDir(scene_dir_);
			std::cerr << "scene_dir_: " << scene_dir_ << std::endl;
            scenes_cams_->LoadElements();

            scenes_layouts_elements_ = std::unique_ptr<HWSceneLayouts>(new HWSceneLayouts());
            scenes_layouts_elements_->SetSceneDir(scene_dir_);
            scenes_layouts_elements_->LoadSceneElements();
        }
        std::cerr << "cams num: " << scenes_cams_->GetCamerasNum() << std::endl;
        std::cerr << "images num: " << scenes_cams_->GetImagesNum() << std::endl;
        std::cerr <<"layout num: " << scenes_layouts_elements_->GetScenelayout2DNum() << std::endl;

        //HWUpdate();
        //HWUpdateId();
		HWUpdateAllIds();
    }

	void HWScenesElements::ImportPairsLinesFromTxt(const std::string& path)
	{
		std::cerr << "start to import lines pairs txt file..." << std::endl;
		if (boost::filesystem::is_regular_file(path))
		{
			//std::cerr << "to do next..." << std::endl;
			std::ifstream fh(path);
			std::string str;
			std::getline(fh, str);
			std::cerr << str << std::endl;
			std::getline(fh, str);
			std::cerr << str << std::endl;
			std::getline(fh, str);
			//std::cerr << str << std::endl;
			//get view num
			std::stringstream head_ss(str);
			std::string lh, rh;
			std::getline(head_ss, lh, ' ');
			std::getline(head_ss, lh, ' ');
			std::getline(head_ss, rh);
			//std::cerr << lh << std::endl;
			//std::cerr << rh << std::endl;
			int view_num = std::stoi(rh);
			std::cerr << "view num: " << view_num << std::endl;
			std::vector<std::string> all_paths;
			std::vector<std::string> all_base_paths;
			for (int i = 0; i < view_num; ++i)
			{
				std::string str_path;
				std::getline(fh, str_path);
				//std::cerr << "str_path: " << str_path << std::endl;
				std::string path_left_slash = HW::GetLeftSlashPathName(str_path);
				std::string path_base_name = HW::GetBaseName(path_left_slash);
				all_paths.emplace_back(path_left_slash);
				all_base_paths.emplace_back(path_base_name);
				std::cerr << "base path: " << path_base_name << std::endl;
			}
			std::vector<HWLineCorrespondLine2D> lines_mathches_pairs;
			while (std::getline(fh, str))
			{
				HW::FileLineBufferProcess(str);
				if (str.empty())
				{
					std::cerr << "skip..." << std::endl;
					continue;
				}
				std::stringstream ss(str);
				std::string my_item;
				std::pair<int, int> view_pair_id;
				std::getline(ss, my_item, ' ');
				view_pair_id.first = std::stoi(my_item);
				std::getline(ss, my_item, ' ');
				view_pair_id.second = std::stoi(my_item);
				/*std::cerr << "view pair: " << view_pair_id.first <<
				", " << view_pair_id.second << std::endl;*/
				HWLineCorrespondLine2D line_pair;
				std::getline(ss, my_item, ' ');
				line_pair.line1.first[0] = std::stof(my_item);
				//std::cerr << "line_pair.line1.first[0]: " << line_pair.line1.first[0] << std::endl;

				std::getline(ss, my_item, ' ');
				line_pair.line1.first[1] = std::stof(my_item);
				//std::cerr << "line_pair.line1.first[1]: " << line_pair.line1.first[1] << std::endl;

				std::getline(ss, my_item, ' ');
				line_pair.line1.second[0] = std::stof(my_item);
				//std::cerr << "line_pair.line1.second[0]: " << line_pair.line1.second[0] << std::endl;

				std::getline(ss, my_item, ' ');
				line_pair.line1.second[1] = std::stof(my_item);
				//std::cerr << "line_pair.line1.second[1]: " << line_pair.line1.second[1] << std::endl;

				std::getline(ss, my_item, ' ');
				line_pair.line2.first[0] = std::stof(my_item);
				//std::cerr << "line_pair.line2.first[0]: " << line_pair.line2.first[0] << std::endl;

				std::getline(ss, my_item, ' ');
				line_pair.line2.first[1] = std::stof(my_item);
				//std::cerr << "line_pair.line2.first[1]: " << line_pair.line2.first[1] << std::endl;

				std::getline(ss, my_item, ' ');
				line_pair.line2.second[0] = std::stof(my_item);
				//std::cerr << "line_pair.line2.second[0]: " << line_pair.line2.second[0] << std::endl;

				std::getline(ss, my_item);
				line_pair.line2.second[1] = std::stof(my_item);
				//std::cerr << "line_pair.line2.second[1]: " << line_pair.line2.second[1] << std::endl;
				//std::cerr << "to do next..." << std::endl;
				if (view_pair_id.first < all_base_paths.size() && view_pair_id.first >= 0 &&
					view_pair_id.second < all_base_paths.size() && view_pair_id.second >= 0)
				{
					std::string lf_name = all_base_paths[view_pair_id.first];
					std::string rf_name = all_base_paths[view_pair_id.second];
					line_pair.base_path_pair.first = lf_name;
					line_pair.base_path_pair.second = rf_name;
					line_pair.camid_to_camid_.first = -1;
					line_pair.camid_to_camid_.second = -1;
					images_lines_pairs_imported_.emplace_back(line_pair);
					//image_window_->images_lines_matches_pairs_.emplace_back(line_pair);
				}
			}
			fh.close();
		}
		std::cerr << "end import lines pair from txt file..." << std::endl;
	}

	void HWScenesElements::ImportLinesPntsPairsFromPythonDir(const std::string& dir)
	{
		std::cerr << "start to import lines pairs txt file..." << std::endl;
		if (boost::filesystem::is_directory(dir))
		{
			std::vector<CameraModel> camsmodels = scenes_cams_->GetCamerasModels();
			std::vector<std::string> cams_paths;
			for (int i = 0; i < camsmodels.size(); ++i)
			{
				cams_paths.emplace_back(camsmodels[i].path_);
			}
			std::vector<std::string> cams_base_names = GetAllBaseNamesFromPaths(cams_paths);
			//int cam1_idx = FindStrIdxFromVecStrsNew(cam1_base_name, cams_base_names);
			std::vector<std::string> cur_all_paths = HW::GetFilesListFromDir(dir);
			std::vector<std::string> cur_py_paths;
			const std::vector<HWImage> all_images_models = scenes_cams_->GetImagesModels();
			std::vector<std::string> all_images_paths;
			for (int i = 0; i < all_images_models.size(); ++i)
			{
				std::cerr << "image path: " << all_images_models[i].GetImagePath() << std::endl;
				all_images_paths.emplace_back(all_images_models[i].GetImagePath());
			}
			for (int i = 0; i < cur_all_paths.size(); ++i)
			{
				if (cur_all_paths[i].find(".npy") != std::string::npos
					|| cur_all_paths[i].find(".NPY") != std::string::npos
					|| cur_all_paths[i].find(".npz") != std::string::npos
					|| cur_all_paths[i].find(".NPZ") != std::string::npos)
				{
					std::string cur_path = ReplaceStrWithNewStr(cur_all_paths[i], "\\", "/");
					std::cerr << "cur_path: " << cur_path << std::endl;
					cur_py_paths.emplace_back(cur_path);
				}
			}
			scenes_lines_pnts_matches_.clear();
			for (int i = 0; i < cur_py_paths.size(); ++i)
			{
				std::cerr << "cur_py_paths[" << std::to_string(i) << "]: " << cur_py_paths[i] << std::endl;
				if (cur_py_paths[i].find(".npy") != std::string::npos)
				{
					//load it into a new array
					cnpy::NpyArray arr = cnpy::npy_load(cur_py_paths[i]);
					//cnpy::NpyArray arr_mv1 = arr["myVar1"];
					std::cerr << "to do next..." << std::endl;
					//std::complex<double>* loaded_data = arr.data<std::complex<double>>();
				}
				else
				{
					//name data
					std::string cur_pair_base_names = GetBaseNameWithoutSuffix(cur_py_paths[i]);
					std::string cur_left_base_name = cur_pair_base_names.substr(0,
						cur_pair_base_names.find_first_of("_"));
					std::string cur_right_base_name = cur_pair_base_names.substr(cur_pair_base_names.find_first_of("_") + 1,
						cur_pair_base_names.find_last_of("_") - cur_pair_base_names.find_first_of("_") - 1);;
					std::cerr << "cur_left_base_name, cur_right_base_name: " <<
						cur_left_base_name << ", " << cur_right_base_name << std::endl;
					//get image idx from image name
					int cam_left_idx = FindStrIdxFromVecStrsNew(cur_left_base_name, cams_base_names);
					int cam_right_idx = FindStrIdxFromVecStrsNew(cur_right_base_name, cams_base_names);
					if (cam_left_idx == -1 && cam_right_idx == -1)
					{
						continue;
					}
					int image_left_id = camsmodels[cam_left_idx].image_id_;
					int image_right_id = camsmodels[cam_right_idx].image_id_;
					if (image_left_id == -1 && image_right_id == -1)
					{
						continue;
					}
					std::cerr << "image_left_id, right_image_id: "
						<< image_left_id << ", " << image_right_id << std::endl;
					//load it into a new array
					cnpy::npz_t all_data = cnpy::npz_load(cur_py_paths[i]);
					cnpy::NpyArray npy_keypoints0 = all_data["keypoints0"];
					cnpy::NpyArray npy_keypoints1 = all_data["keypoints1"];
					cnpy::NpyArray npy_matches_kpts = all_data["matches_p"];
					//cnpy::NpyArray npy_matches_kpts_confidence = all_data["matching_scores_p"];
					std::cerr << "start print npy_keypoints0 info: " << std::endl;
					PrintPyNpyShape(npy_keypoints0);
					std::cerr << "end npy_keypoints0 info" << std::endl;
					std::cerr << "start print npy_keypoints1 info: " << std::endl;
					PrintPyNpyShape(npy_keypoints1);
					std::cerr << "end npy_keypoints1 info" << std::endl;
					std::cerr << "start print npy_matches_kpts info: " << std::endl;
					PrintPyNpyShape(npy_matches_kpts);
					std::cerr << "end npy_matches_kpts info" << std::endl;
					std::vector<Eigen::Vector2f> keypoints_p0, keypoints_p1;
					ConvertNumpyPntsToPnts(npy_keypoints0, keypoints_p0);
					ConvertNumpyPntsToPnts(npy_keypoints1, keypoints_p1);
					std::vector<Eigen::Vector2i> pnts_matches_idxs;
					ConvertNumpyPntsIdxToPntsIdx(npy_matches_kpts, pnts_matches_idxs);
					//copy keypoints_p0 to HWImage from image idx
					bool pnts_left_loaded = scenes_cams_.get()->GetImageNetworkPointsLoadedFromImgid(image_left_id);
					bool pnts_right_loaded = scenes_cams_.get()->GetImageNetworkPointsLoadedFromImgid(image_right_id);
					if (!pnts_left_loaded)
					{
						//if points is empty, then load the left points
						std::vector<cv::Point2f> keypoints_cvp0;
						for (int j = 0; j < keypoints_p0.size(); ++j)
						{
							cv::Point2f kpnt;
							kpnt.x = keypoints_p0[j][0];
							kpnt.y = keypoints_p0[j][1];
							keypoints_cvp0.emplace_back(kpnt);
						}
						////test
						//if (i == 0)
						//{
						//	//write the image pnts into image
						//	/*const HWImage tmp_img = scenes_cams_->GetImageFromImgId(image_left_id);
						//	std::cerr << "tmp_img id: " << tmp_img.GetImageId() << std::endl;*/
						//	const std::string tmp_image_path = all_images_paths[image_left_id];
						//	//const std::string tmp_image_path = GetImagePathFromImageid(image_left_id);
						//	std::cerr << "left tmp_image_path: " << tmp_image_path << std::endl;
						//	cv::Mat tmp_image = cv::imread(tmp_image_path);
						//	std::string image_pnts_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part5_test/vis/"
						//		+ std::to_string(image_left_id) + "_pnts.png";
						//	for (int j = 0; j < keypoints_cvp0.size(); ++j)
						//	{
						//		cv::Point2f p = keypoints_cvp0[j];
						//		cv::circle(tmp_image, p, 5, cv::Scalar(255, 255, 0), 2);
						//	}
						//	cv::imwrite(image_pnts_path, tmp_image);
						//}
						////end test
						scenes_cams_.get()->SetImageNetworkPointsFromImgid(image_left_id, keypoints_cvp0);
					}
					if (!pnts_right_loaded)
					{
						//if points is empty, then load the right points
						std::vector<cv::Point2f> keypoints_cvp1;
						for (int j = 0; j < keypoints_p1.size(); ++j)
						{
							cv::Point2f kpnt;
							kpnt.x = keypoints_p1[j][0];
							kpnt.y = keypoints_p1[j][1];
							keypoints_cvp1.emplace_back(kpnt);
						}
						////test
						//if (i == 0)
						//{
						//	//write the image pnts into image
						//	//const HWImage tmp_img = scenes_cams_->GetImageFromImgId(image_right_id);
						//	//std::cerr << "tmp_img id: " << tmp_img.GetImageId() << std::endl;
						//	const std::string tmp_image_path = all_images_paths[image_right_id];
						//	//const std::string tmp_image_path = GetImagePathFromImageid(image_right_id);
						//	std::cerr << "right tmp_image_path: " << tmp_image_path << std::endl;
						//	cv::Mat tmp_image = cv::imread(tmp_image_path);
						//	std::string image_pnts_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part5_test/vis/" 
						//		+ std::to_string(image_right_id) + "_pnts.png";
						//	for (int j = 0; j < keypoints_cvp1.size(); ++j)
						//	{
						//		cv::Point2f p = keypoints_cvp1[j];
						//		cv::circle(tmp_image, p, 5, cv::Scalar(255, 255, 0), 2);
						//	}
						//	cv::imwrite(image_pnts_path, tmp_image);
						//}
						////end test
						scenes_cams_.get()->SetImageNetworkPointsFromImgid(image_right_id, keypoints_cvp1);
					}
					//copy data to images_pnts_pairs_imported_
					for (int j = 0; j < pnts_matches_idxs.size(); ++j)
					{
						HWLineCorrespondPnt2D tmp_pair_pnt;
						tmp_pair_pnt.base_path_pair.first = cur_left_base_name;
						tmp_pair_pnt.base_path_pair.second = cur_right_base_name;
						tmp_pair_pnt.camid_to_camid_.first = -1;
						tmp_pair_pnt.camid_to_camid_.second = -1;
						tmp_pair_pnt.pnt1 = keypoints_p0[pnts_matches_idxs[j][0]];
						tmp_pair_pnt.pnt2 = keypoints_p0[pnts_matches_idxs[j][1]];
						images_pnts_pairs_imported_.emplace_back(tmp_pair_pnt);
					}
					//copy points match data to scenes_lines_pnts_matches_
					HWTwoViewsCamsMatching tmp_pair_lines_pnts_matches;
					tmp_pair_lines_pnts_matches.view_1_id = cam_left_idx;
					tmp_pair_lines_pnts_matches.view_2_id = cam_right_idx;
					for (int j = 0; j < pnts_matches_idxs.size(); ++j)
					{
						HWPntsMatch pair_pnt_idx;
						//HWCorrespondence2D2DPnt pair_pnt;
						pair_pnt_idx.pnts_matches_idx_.first = pnts_matches_idxs[j][0];
						pair_pnt_idx.pnts_matches_idx_.second = pnts_matches_idxs[j][1];
						pair_pnt_idx.corresponding_pnts_.p1 = keypoints_p0[pnts_matches_idxs[j][0]];
						pair_pnt_idx.corresponding_pnts_.p2 = keypoints_p1[pnts_matches_idxs[j][1]];
						pair_pnt_idx.adj_poly_idxs_ = Eigen::Vector2i(-1, -1);
						pair_pnt_idx.view12_tri_pnt3d_ = Eigen::Vector3f(0.0, 0.0, 0.0);
						pair_pnt_idx.valid_match_ = true;
						tmp_pair_lines_pnts_matches.points_matches.emplace_back(pair_pnt_idx);
					}

					//copy corresponding points idx to images_pnts_pairs_imported_
					/*
					std::cerr << "-------------------start points-----------------------" << std::endl;
					for (int j = 0; j < keypoints_p0.size(); ++j)
					{
						std::cerr << keypoints_p0[j].transpose() << std::endl;
					}
					for (int j = 0; j < keypoints_p1.size(); ++j)
					{
						std::cerr << keypoints_p1[j].transpose() << std::endl;
					}
					std::cerr << "-------------------end points-----------------------" << std::endl;
					*/
					cnpy::NpyArray npy_keylines0 = all_data["keylines0"];
					cnpy::NpyArray npy_keylines1 = all_data["keylines1"];
					cnpy::NpyArray npy_matches_klines = all_data["matches_l"];
					//cnpy::NpyArray npy_matches_klines_confidence = all_data["matching_scores_l"];
					std::cerr << "start print npy_keylines0 info: " << std::endl;
					PrintPyNpyShape(npy_keylines0);
					std::cerr << "end npy_keypoints0 info" << std::endl;
					std::cerr << "start print npy_keylines1 info: " << std::endl;
					PrintPyNpyShape(npy_keylines1);
					std::cerr << "end npy_keylines1 info" << std::endl;
					std::cerr << "start print npy_matches_klines info: " << std::endl;
					PrintPyNpyShape(npy_matches_klines);
					std::cerr << "end npy_matches_klines info" << std::endl;

					/*std::vector<float> keylines0 = npy_keylines0.as_vec<float>();
					std::vector<float> keylines1 = npy_keylines1.as_vec<float>();
					std::vector<float> matches_klines = npy_matches_klines.as_vec<float>();*/

					std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > keylines0, keylines1;
					ConvertNumpyLinesPntsToLinesPnts(npy_keylines0, keylines0);
					ConvertNumpyLinesPntsToLinesPnts(npy_keylines1, keylines1);
					//to lines
					int layout_left_id = camsmodels[cam_left_idx].layout_id_;
					int layout_right_id = camsmodels[cam_right_idx].layout_id_;
					std::cerr << "layout_left_id, layout_right_id: "
						<< layout_left_id << ", " << layout_right_id << std::endl;
					if (layout_left_id == -1 && layout_right_id == -1)
					{
						continue;
					}
					//copy keypoints_p0 to HWImage from image idx
					bool lines_left_loaded = scenes_layouts_elements_.get()->GetLayoutsLoadedStateFromLyId(layout_left_id);
					bool lines_right_loaded = scenes_layouts_elements_.get()->GetLayoutsLoadedStateFromLyId(layout_right_id);
					if (!lines_left_loaded)
					{
						//if points is empty, then load the left points
						scenes_layouts_elements_.get()->SetLayoutLinesSegsFromLyId(layout_left_id, keylines0);
						
						////test
						//if (i == 0)
						//{
						//	const std::string tmp_image_path = all_images_paths[image_left_id];
						//	cv::Mat tmp_image = cv::imread(tmp_image_path);
						//	std::string image_lines_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part5_test/vis/"
						//		+ std::to_string(image_left_id) + "_lines.png";
						//	for (int j = 0; j < keylines0.size(); ++j)
						//	{
						//		//std::cerr << "j " << j << std::endl;
						//		Eigen::Vector2f ls = keylines0[j].first;
						//		Eigen::Vector2f le = keylines0[j].second;
						//		cv::Point2f lscv(ls[0], ls[1]);
						//		cv::Point2f	lecv(le[0], le[1]);
						//		cv::circle(tmp_image, lscv, 4, cv::Scalar(255, 0, 0), 2);
						//		cv::circle(tmp_image, lecv, 4, cv::Scalar(255, 0, 0), 2);
						//		cv::line(tmp_image, lscv, lecv, cv::Scalar(0, 0, 255), 2);
						//	}
						//	cv::imwrite(image_lines_path, tmp_image);
						//}
						////end test
					}
					if (!lines_right_loaded)
					{
						//if points is empty, then load the right points
						scenes_layouts_elements_.get()->SetLayoutLinesSegsFromLyId(layout_right_id, keylines1);

						////test
						//if (i == 0)
						//{
						//	const std::string tmp_image_path = all_images_paths[image_right_id];
						//	cv::Mat tmp_image = cv::imread(tmp_image_path);
						//	std::string image_lines_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part5_test/vis/"
						//		+ std::to_string(image_right_id) + "_lines.png";
						//	for (int j = 0; j < keylines1.size(); ++j)
						//	{
						//		//std::cerr << "j: " << j << std::endl;
						//		Eigen::Vector2f ls = keylines1[j].first;
						//		Eigen::Vector2f le = keylines1[j].second;
						//		cv::Point2f lscv(ls[0], ls[1]);
						//		cv::Point2f	lecv(le[0], le[1]);
						//		cv::circle(tmp_image, lscv, 4, cv::Scalar(255, 0, 0), 2);
						//		cv::circle(tmp_image, lecv, 4, cv::Scalar(255, 0, 0), 2);
						//		cv::line(tmp_image, lscv, lecv, cv::Scalar(0, 0, 255), 2);
						//	}
						//	cv::imwrite(image_lines_path, tmp_image);
						//}
						////end test
					}
					std::vector<Eigen::Vector2i> lines_matches_idxs;
					ConvertNumpyLinesIdxToLinesIdx(npy_matches_klines, lines_matches_idxs);

					/*
					std::cerr << "-------------------start lines points-----------------------" << std::endl;
					for (int j = 0; j < keylines0.size(); ++j)
					{
						std::cerr << keylines0[j].first.transpose() << std::endl;
						std::cerr << keylines0[j].second.transpose() << std::endl;
					}
					for (int j = 0; j < keylines1.size(); ++j)
					{
						std::cerr << keylines1[j].first.transpose() << std::endl;
						std::cerr << keylines1[j].second.transpose() << std::endl;
					}
					std::cerr << "-------------------end lines points-----------------------" << std::endl;
					*/

					//copy data to images_lines_pairs_imported_
					for (int j = 0; j < lines_matches_idxs.size(); ++j)
					{
						HWLineCorrespondLine2D tmp_pair_line;
						tmp_pair_line.base_path_pair.first = cur_left_base_name;
						tmp_pair_line.base_path_pair.second = cur_right_base_name;
						tmp_pair_line.camid_to_camid_.first = -1;
						tmp_pair_line.camid_to_camid_.second = -1;
						tmp_pair_line.line1 = keylines0[lines_matches_idxs[j][0]];
						tmp_pair_line.line2 = keylines1[lines_matches_idxs[j][1]];
						images_lines_pairs_imported_.emplace_back(tmp_pair_line);
					}
					
					//copy lines matches to scenes_lines_pnts_matches_
					for (int j = 0; j < lines_matches_idxs.size(); ++j)
					{
						HWLinesPntsMatch pair_line_idx;
						pair_line_idx.line_matches_idx_.first = lines_matches_idxs[j][0];
						pair_line_idx.line_matches_idx_.second = lines_matches_idxs[j][1];
						pair_line_idx.image_p1_ = keylines0[lines_matches_idxs[j][0]].first;
						pair_line_idx.image_p2_ = keylines0[lines_matches_idxs[j][0]].second;
						pair_line_idx.image_q1_ = keylines1[lines_matches_idxs[j][1]].first;
						pair_line_idx.image_q2_ = keylines1[lines_matches_idxs[j][1]].second;
						pair_line_idx.valid_match_ = true;
						tmp_pair_lines_pnts_matches.lines_matches.emplace_back(pair_line_idx);
					}

					//std::cerr << "asdfasdfasdfasdfasdfasdfasdfsf1: " << i << std::endl;
					scenes_lines_pnts_matches_.emplace_back(tmp_pair_lines_pnts_matches);
					//std::cerr << "asdfasdfasdfasdfasdfasdfasdfsf2: " << i << std::endl;
					//std::cerr << "scenes_lines_pnts_matches_: " << scenes_lines_pnts_matches_.size() << std::endl;
#if 0
					//test
					if (i == 0)
					{
						const std::string tmp_left_image_path = all_images_paths[image_left_id];
						cv::Mat tmp_left_image = cv::imread(tmp_left_image_path);
						std::string image_left_lines_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part4_test1/vis/"
							+ std::to_string(image_left_id) + "_left_lines_matches.png";
						for (int j = 0; j < lines_matches_idxs.size(); ++j)
						{
							int lidx = lines_matches_idxs[j][0];
							std::cerr << "lidx " << lidx << std::endl;
							Eigen::Vector2f ls = keylines0[lidx].first;
							Eigen::Vector2f le = keylines0[lidx].second;
							cv::Point2f lscv(ls[0], ls[1]);
							cv::Point2f	lecv(le[0], le[1]);
							cv::circle(tmp_left_image, lscv, 4, cv::Scalar(255, 0, 0), 2);
							cv::circle(tmp_left_image, lecv, 4, cv::Scalar(255, 0, 0), 2);
							cv::line(tmp_left_image, lscv, lecv, cv::Scalar(0, 0, 255), 2);
						}
						cv::imwrite(image_left_lines_path, tmp_left_image);

						const std::string tmp_right_image_path = all_images_paths[image_right_id];
						cv::Mat tmp_right_image = cv::imread(tmp_right_image_path);
						std::string image_right_lines_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part4_test1/vis/"
							+ std::to_string(image_right_id) + "_right_lines.png";
						for (int j = 0; j < lines_matches_idxs.size(); ++j)
						{
							int ridx = lines_matches_idxs[j][1];
							std::cerr << "ridx " << ridx << std::endl;
							Eigen::Vector2f ls = keylines1[ridx].first;
							Eigen::Vector2f le = keylines1[ridx].second;
							cv::Point2f lscv(ls[0], ls[1]);
							cv::Point2f	lecv(le[0], le[1]);
							cv::circle(tmp_right_image, lscv, 4, cv::Scalar(255, 0, 0), 2);
							cv::circle(tmp_right_image, lecv, 4, cv::Scalar(255, 0, 0), 2);
							cv::line(tmp_right_image, lscv, lecv, cv::Scalar(0, 0, 255), 2);
						}
						cv::imwrite(image_right_lines_path, tmp_right_image);

						//test match lines
					}
					//end test
#endif
				}
			}
		}
	}

	void HWScenesElements::SetCamerasModels(ModelCameras* models)
	{
		//set the cameras models
		std::cerr << "test cameras models..." << std::endl;
		scenes_cams_ = std::unique_ptr<HWScenesCams>(new HWScenesCams());
		//copy ModelCameras data to HWScenesCams
		std::vector<CameraModel> cams_models;
		models->GetModelCameras(cams_models);
		const std::vector<std::string> images_paths = models->GetImagesPaths();
		std::vector<std::string> images_paths_new;
		for (std::size_t i = 0; i < images_paths.size(); ++i)
		{
			images_paths_new.emplace_back(images_paths[i]);
		}
		//std::cerr << "222222222222" << std::endl;
		scenes_cams_->SetCamerasModels(cams_models);
		scenes_cams_->SetHWImageModelsFromPaths(images_paths_new);
		scenes_cams_->UpdateCamsModelsId2ImagesModelsId();
		//copy dir to HWScenesCams class
		//important
		//std::string cams_dir = 
		std::cerr << "set cams num: " << scenes_cams_->GetCamerasNum() << std::endl;
		std::cerr << "set images num: " << scenes_cams_->GetImagesNum() << std::endl;
	}

	void HWScenesElements::SetLayoutsModels(SceneElements* lay_models)
	{
		//set the layouts models
		std::cerr << "test layouts models..." << std::endl;
		scenes_layouts_elements_ = std::unique_ptr<HWSceneLayouts>(new HWSceneLayouts());
		//copy SceneElements to HWSceneLayouts
		std::vector<SceneLayout> scenelays = lay_models->GetCamSceneLyData();
		for (std::size_t i = 0; i < scenelays.size(); ++i)
		{
			//get data
			HWSceneLayout2D sly;
			sly.SetLayoutId(i);
			//copy SceneLayout to HWSceneLayout2D
			sly.SetHWLayoutDimType(HW::HWSceneLayoutDimType::kScene2DType);
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > tmp_scene_pnt = scenelays[i].GetAllLineSegsPnts();
			int tmp_scene_pnt_num = static_cast<int>(tmp_scene_pnt.size());
			sly.SetSceneLayoutFromLines2D(tmp_scene_pnt);
			const std::string lyname = scenelays[i].GetLayoutPath();
			//std::cerr << "lyname: " << lyname << std::endl;
			sly.SetLayoutPath(lyname);
			//std::cerr << "sly lyname: " << sly.GetLayout2DPath() << std::endl;
			//sly.set
			//add HWSceneLayout2D to HWSceneLayouts
			scenes_layouts_elements_->AddSceneLayout2d(sly);

		}
		//Important to test...
		scene_dir_ = lay_models->GetSceneElementsDir();
		//std::cerr << "HWScenesElement scene_dir_: " << scene_dir_ << std::endl;
		//std::cerr << "set layout2d from SceneElements num: " << scenes_layouts_elements_->GetScenelayout2DNum() << std::endl;
		HWUpdate();
		HWUpdateId();
	}

	void HWScenesElements::SetHWPolygonsModels(std::vector<HWPlane*> hw_polygons)
	{
		associated_polygons_ = hw_polygons;
		//std::cerr << "the associated polygons number: " << associated_polygons_.size() << std::endl;
	}

	void HWScenesElements::SetHWImagesLabels(HWImageLabels* polygons_images_labels)
	{
		associsated_images_labels_ = polygons_images_labels;
		if (associsated_images_labels_)
		{
			images_labels_loaded_ = true;
		}
	}
	//associsated_images_labels_

	void HWScenesElements::SetPolygonsIntersectLines(const std::vector<HWPolygonInterLines>& polygons_lines)
	{
		polygons_intersection_lines_.resize(polygons_lines.size());
		for (int i = 0; i < polygons_lines.size(); ++i)
		{
			polygons_intersection_lines_[i].polygons_idxs_ = polygons_lines[i].polygons_idxs_;
			polygons_intersection_lines_[i].ls3d_ = polygons_lines[i].ls3d_;
			polygons_intersection_lines_[i].le3d_ = polygons_lines[i].le3d_;
			polygons_intersection_lines_[i].cams_visibility_idx_ = polygons_lines[i].cams_visibility_idx_;
		}
	}

	void HWScenesElements::SetRmaxThreshold(double rmax)
	{
		r_max_threshold_ = rmax;
	}

	void HWScenesElements::SetImportDataFromSceneElementsSate(bool data_state)
	{
		import_data_from_scene_elements_state_ = data_state;
	}

	void HWScenesElements::SetImportDataFromPairsLinesTxtState(bool pair_txt_state)
	{
		import_data_from_pairs_lines_txt_state_ = pair_txt_state;
	}

	void HWScenesElements::SetImportDataFromPythonNumpyState(bool nmpy_state)
	{
		import_data_from_python_numpy_state_ = nmpy_state;
	}

	void HWScenesElements::SetRMaxPolygonThresholdRayCast(double r_polygon_max)
	{
		r_max_polygon_threshold_ = r_polygon_max;
	}

	const std::vector<CameraModel>& HWScenesElements::GetCamerasModels()
	{
		return scenes_cams_.get()->GetCamerasModels();
	}

	const std::vector<HW::HWImage>& HWScenesElements::GetImagesModels()
	{
		return scenes_cams_.get()->GetImagesModels();
	}

	void HWScenesElements::UpdateImageModelsLoadImgs()
	{
		scenes_cams_.get()->UpdateImagesModelLoadImgs();
	}

	HWSceneLayouts* HWScenesElements::GetLayoutsModels()
	{
		return scenes_layouts_elements_.get();
	}

	const std::map<unsigned int, std::set<unsigned int> > HWScenesElements::GetImagesLinesMatchesCamsIds()
	{
		return matched_;
	}

	void HWScenesElements::GetHWPolygonModels(std::vector<HWPlane*>& hw_polygons)
	{
		hw_polygons.resize(associated_polygons_.size());
		for (int i = 0; i < associated_polygons_.size(); ++i)
		{
			hw_polygons[i] = associated_polygons_[i];
		}
	}

	HWSurveyPoint3d2Pnts2dList* HWScenesElements::GetSurveyPnts2PntsList()
	{
		return &scenes_pnt3ds_observations_;
	}

	HWLineSurveyPoint3d2Pnts2dList* HWScenesElements::GetSurveyLines3D2Pnts2dList()
	{
		return &scenes_lines_pnts3ds_observations_;
	}

	HWCamsViewportList* HWScenesElements::GetViewsPortList()
	{
		return &hw_cams_views_list_;
	}

	const HWPairwisesMatchingView& HWScenesElements::GetImagesPairsMatchesViews()
	{
		return scenes_lines_pnts_matches_;
	}

	std::vector<std::pair<std::string, std::string> > HWScenesElements::GetCamsPairsIdsNamesFromScenesLinesPntsRefined()
	{
		std::vector<PairSetValue> cams_pairs_ids;
		for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			//HWTwoViewsCamsMatching tmp_pairs = scenes_lines_pnts_matches_[i];
			int view1_id = scenes_lines_pnts_matches_[i].view_1_id;
			int view2_id = scenes_lines_pnts_matches_[i].view_2_id;
			PairSetValue tmp_view;
			tmp_view.value_.first = view1_id;
			tmp_view.value_.second = view2_id;
			bool view_in = CheckPairsInPairsSet(tmp_view, cams_pairs_ids);
			if (!view_in)
			{
				cams_pairs_ids.emplace_back(tmp_view);
			}
		}
		std::vector<std::pair<std::string, std::string> > images_pairs_names;
		const std::vector<std::string> images_names = scenes_cams_->GetImagesPaths();
		for (int i = 0; i < cams_pairs_ids.size(); ++i)
		{
			int view1_id = cams_pairs_ids[i].value_.first;
			int view2_id = cams_pairs_ids[i].value_.second;
			int image1_id = scenes_cams_->GetImageidFromCamId(view1_id);
			int image2_id = scenes_cams_->GetImageidFromCamId(view2_id);
			std::string image1_name = images_names[image1_id];
			std::string image2_name = images_names[image2_id];
			std::string image1_base_name = HW::GetBaseName(image1_name);
			std::string image2_base_name = HW::GetBaseName(image2_name);
			std::pair<std::string, std::string> image_pair_name;
			image_pair_name.first = image1_base_name;
			image_pair_name.second = image2_base_name;
			images_pairs_names.emplace_back(image_pair_name);
		}
		return images_pairs_names;
	}

	const std::vector<HWLineCorrespondLine2D>& HWScenesElements::GetImportedCorrespondLines2D()
	{
		return images_lines_pairs_imported_;
	}

	const std::vector<HWLineCorrespondPnt2D>& HWScenesElements::GetImportedCorrespondPnts2D()
	{
		return images_pnts_pairs_imported_;
	}

	bool HWScenesElements::GetImportDataFromSceneElementsSate()
	{
		return import_data_from_scene_elements_state_;
	}

	bool HWScenesElements::GetImportDataFromPairsLinesTxtState()
	{
		return import_data_from_pairs_lines_txt_state_;
	}

	bool HWScenesElements::GetImportDataFromPythonNumpyState()
	{
		return import_data_from_python_numpy_state_;
	}

	void HWScenesElements::BackUpHWCamViewportsList()
	{
		hw_cams_views_list_prev_.resize(hw_cams_views_list_.size());
		for (int i = 0; i < hw_cams_views_list_.size(); ++i)
		{
			hw_cams_views_list_prev_[i] = hw_cams_views_list_[i];
		}
	}

    void HWScenesElements::HWUpdateId()
    {
        const std::vector<HWImage> images_model = scenes_cams_->GetImagesModels();
        for(std::size_t i = 0; i < images_model.size(); ++i)
        {
            HWImage image_model = images_model[i];
            const std::string image_name = image_model.GetImagePath();
            std::string image_base_name = GetBaseNameWithoutSuffix(image_name);
            unsigned int image_id = image_model.GetImageId();
			const std::vector<std::unique_ptr<HWSceneLayout2D> >& scenes_lyouts = scenes_layouts_elements_->GetSceneLayouts2D();
            for(std::size_t j = 0; j < scenes_lyouts.size(); ++j)
            {
                //HWSceneLayout2D ly2d = scenes_layouts_elements_->GetPickedLayout2D(j);
                const unsigned int lyid = scenes_lyouts[j]->GetLayoutId();
                std::string layout_base_name = GetBaseNameWithoutSuffix(scenes_lyouts[j]->GetLayout2DPath());
				std::cerr << "image base name: " << image_base_name << std::endl;
				std::cerr << "layout basename: " << layout_base_name << std::endl;
				std::cerr << std::endl;
                if(image_base_name == layout_base_name)
                {
                    //set image id
                    std::cerr << "update: image id, lyid: " << image_id <<", " << lyid << std::endl;
                    scenes_layouts_elements_->SetPickedLayout2DImageId(j, image_id);
                    scenes_cams_->SetLayoutIdFromImageIdx(i, lyid);
                    scenes_imgid_to_layoutid_[image_id] = lyid;
                }
            }
        }
        data_updated_ = true;
        std::cerr <<"end update images to layout..." << std::endl;
    }

    void HWScenesElements::HWUpdate()
    {
        std::cerr <<"update hw scene element ..." << std::endl;
        //image idx to layout idx

        std::vector<std::string> images_paths = scenes_cams_->GetImagesPaths();
        for(std::size_t i = 0; i < images_paths.size(); ++i)
        {
            std::string image_name =  images_paths[i];
            std::string image_base_name = GetBaseNameWithoutSuffix(image_name);
			const std::vector<std::unique_ptr<HWSceneLayout2D> >& scenes_lyouts = scenes_layouts_elements_->GetSceneLayouts2D();
            for(std::size_t j = 0; j < scenes_lyouts.size(); ++j)
            {
                //HWSceneLayout2D ly2d = scenes_layouts_elements_->GetPickedLayout2D(j);
				//std::cerr << "ly2d pnts: " << ly2d.GetLayoutLinesNum() << std::endl;
                std::string layout_base_name = GetBaseNameWithoutSuffix(scenes_lyouts[j]->GetLayout2DPath());
				//std::cerr << "layout base name: " << layout_base_name << std::endl;
                if(image_base_name == layout_base_name)
                {
                    //std::cerr << "image_base_name: " << image_base_name << std::endl;
                    //std::cerr << "layout_base_name: " << layout_base_name << std::endl;
                    //std::cerr << "i, j: " << i <<", " << j << std::endl;
                    std::pair<int, int> imgidx2lyidx = std::make_pair(i, j);
					//int image_id = i;
					//scenes_layouts_elements_->SetPickedLayout2DImageId(j, image_id);
					//scenes_layouts_elements_->SetPickedLayout2DId(j);
                    scenes_imagesidx_layoutsidx_.emplace_back(imgidx2lyidx);
                }
            }
        }
        data_updated_ = true;
        //update images to layout idx 
        std::cerr <<"end hw scene element update ..." << std::endl;
    }

	void HWScenesElements::HWUpdateAllIds()
	{
		std::cerr << "start to update all the ids..." << std::endl;
		HWUpdateCamsLyoutsId();
		HWUpdateImagesLyoutsId();
		data_updated_ = true;
		std::cerr << "end update all the ids..." << std::endl;
	}

	void HWScenesElements::HWUpdateCamsLyoutsId()
	{
		std::cerr << "start update layouts id to HWCams and cams id to HWSceneLayout..." << std::endl;
		const std::vector<CameraModel> cams_models = scenes_cams_->GetCamerasModels();
		for (std::size_t i = 0; i < cams_models.size(); ++i)
		{
			CameraModel cam_model = cams_models[i];
			const std::string cam_name = cam_model.path_;
			std::string cam_base_name = GetBaseNameWithoutSuffix(cam_name);
			unsigned int cam_id = cam_model.cam_id_;
			const std::vector<std::unique_ptr<HWSceneLayout2D> >& scenes_lyouts = scenes_layouts_elements_->GetSceneLayouts2D();
			for (std::size_t j = 0; j < scenes_lyouts.size(); ++j)
			{
				//HWSceneLayout2D ly2d = scenes_layouts_elements_->GetPickedLayout2D(j);
				const unsigned int lyid = scenes_lyouts[j]->GetLayoutId();
				std::string layout_base_name = GetBaseNameWithoutSuffix(scenes_lyouts[j]->GetLayout2DPath());
				//std::cerr << "cam base name: " << cam_base_name << std::endl;
				//std::cerr << "layout basename: " << layout_base_name << std::endl;
				//std::cerr << std::endl;
				if (cam_base_name == layout_base_name)
				{
					//set cam id and layout id
					//std::cerr << "update: cam idx, ly idx: " << i << ", " << j << std::endl;
					//std::cerr << "update: cam id, lyid: " << cam_id << ", " << lyid << std::endl;
					scenes_layouts_elements_->SetCamIdByLayoutIdx(j, cam_id);
					scenes_cams_->SetLayoutIdByCamIdx(i, lyid);
				}
			}
		}
		//data_updated_ = true;
		std::cerr << "end update layouts id to HWCams and cams id to HWSceneLayout..." << std::endl;
	}

	void HWScenesElements::HWUpdateImagesLyoutsId()
	{
		std::cerr << "start update layouts id to HWImages and images id to HWSceneLayout..." << std::endl;
		const std::vector<HWImage> images_models = scenes_cams_->GetImagesModels();
		for (std::size_t i = 0; i < images_models.size(); ++i)
		{
			HWImage image_model = images_models[i];
			const std::string image_name = image_model.GetImagePath();
			std::string image_base_name = GetBaseNameWithoutSuffix(image_name);
			unsigned int image_id = image_model.GetImageId();
			const std::vector<std::unique_ptr<HWSceneLayout2D> >& scenes_lyouts = scenes_layouts_elements_->GetSceneLayouts2D();
			for (std::size_t j = 0; j < scenes_lyouts.size(); ++j)
			{
				//HWSceneLayout2D ly2d = scenes_layouts_elements_->GetPickedLayout2D(j);
				const unsigned int lyid = scenes_lyouts[j]->GetLayoutId();
				std::string layout_base_name = GetBaseNameWithoutSuffix(scenes_lyouts[j]->GetLayout2DPath());
				//std::cerr << "image base name: " << image_base_name << std::endl;
				//std::cerr << "layout basename: " << layout_base_name << std::endl;
				//std::cerr << std::endl;
				if (image_base_name == layout_base_name)
				{
					//set cam id and layout id
					//std::cerr << "update: image idx, ly idx: " << i << ", " << j << std::endl;
					//std::cerr << "update: lay id, lyid: " << image_id << ", " << lyid << std::endl;
					scenes_layouts_elements_->SetImageIdByLayoutIdx(j, image_id);
					scenes_cams_->SetLayoutIdByImageIdx(i, lyid);
				}
			}
		}
		std::cerr << "end update layouts id to HWImages and images id to HWSceneLayout..." << std::endl;
	}

    bool HWScenesElements::IsUpdate()
    {
        return data_updated_;
    }

    void HWScenesElements::HWUpdateCamsGroup()
    {
        scenes_cams_->GroupCamsIdBasedOnCamDirect();
        const std::map<unsigned int, HWCamsIdxGroup> groups = scenes_cams_->GetCamsGroups();
        std::map<unsigned int, HWCamsIdxGroup>::const_iterator it;
        for(it = groups.begin(); it != groups.end(); ++it)
        {
            unsigned int src_id = it->first;
            std::set<unsigned int> cams_ids;
            HWCamsIdxGroup neigbor_cams = it->second;
            for(std::size_t j = 0; j < neigbor_cams.cams_idxs_.size(); ++j)
            {
                //compute fundmental matrix
                unsigned int tgt_id = static_cast<unsigned int> (neigbor_cams.cams_idxs_[j]);
                cams_ids.insert(tgt_id);
            }
            matched_[src_id] = cams_ids;
        }
        ////test
        //std::cerr <<"show the match: " << std::endl;
        //std::map<unsigned int, std::set<unsigned int> >::iterator tit;
        //for(tit = matched_.begin(); tit != matched_.end(); ++tit)
        //{
        //    std::cerr << tit->first << ": ";
        //    std::set<unsigned int> n = tit->second;
        //    std::set<unsigned int>::iterator nit;
        //    for(nit = n.begin(); nit != n.end(); ++nit)
        //    {
        //        std::cerr << *nit << " ";
        //    }
        //    std::cerr << std::endl;
        //}
    }
    
	void HWScenesElements::HWUpdateScenceLayoutsFilterSameLine()
	{
		scenes_layouts_elements_.get()->UpdateFilterAllLayoutsSameLines();
	}

	void HWScenesElements::HWUpdateScenceLayoutsNearLines()
	{
		scenes_layouts_elements_.get()->UpdateAllLayoutsNearLines();
	}

	void HWScenesElements::HWUpdateRemoveSceneLayoutsNearLines()
	{
		scenes_layouts_elements_.get()->RemoveAllLayoutsNearLines();
	}

	void HWScenesElements::HWUpdateRemoveSceneLayoutShortLines()
	{
		scenes_layouts_elements_.get()->RemoveAllLayoutsShortLinesByLength();
	}

	void HWScenesElements::SetDataFromPolygonsIntersectionLines(const std::vector<HWSceneLine3DNode>& lines_node)
	{
		scene_line3d_node_ = lines_node;
	}

	void HWScenesElements::SetPolygonInterLinesFromHWPolygonsLines(const std::vector<PolygonInterLineScene>& polygons_lines)
	{
		polygons_intersection_scene_lines_ = polygons_lines;
	}

	void HWScenesElements::SetView2PolygonLineDataFromView2Lines(const std::vector<View2PolygonInterLinesData>& lines_data)
	{
		views_visiable_polygon_inter_lines_ = lines_data;
	}

	int HWScenesElements::FindLine3dIdxFromImageIdLineId(int image_id, int lid)
	{
		for (int i = 0; i < scene_line3d_node_.size(); ++i)
		{
			//to set its
			HWSceneLine3DNode line3d_node = scene_line3d_node_[i];
			std::vector<int> cams_idx = line3d_node.cams_visible_idxs_;
			std::vector<std::pair<HWSCENELINEIDXTYPE, float> > imageidxs2lines = line3d_node.Imgidx2Imgev_;
			//get images lidx
			for (int j = 0; j < imageidxs2lines.size(); ++i)
			{
				int tmp_image_id = imageidxs2lines[j].first[0];
				int tmp_line_id = imageidxs2lines[j].first[1];
				//set layout external state
				if (tmp_image_id == image_id && tmp_line_id == lid)
				{
					return i;
				}
			}
		}
		return -1;
	}

	int HWScenesElements::FindLine3dIdxFromImageIdLineIdPair(int src_img_id, int src_lid, int tgt_img_id, int tgt_lid)
	{
		//
		int find_idx = -1;
		for (int i = 0; i < scene_line3d_node_.size(); ++i)
		{
			bool src_flag = false;
			bool tgt_flag = false;
			std::vector<std::pair<HWSCENELINEIDXTYPE, float> > line_tracks = scene_line3d_node_[i].Imgidx2Imgev_;
			for (int j = 0; j < line_tracks.size(); ++j)
			{
				//
				if (src_img_id == line_tracks[j].first[0]
					&& src_lid == line_tracks[j].first[1])
				{
					src_flag = true;
				}
				if (tgt_img_id == line_tracks[j].first[0]
					&& tgt_lid == line_tracks[j].first[1])
				{
					tgt_flag = true;
				}
				if (src_flag && tgt_flag)
				{
					find_idx = i;
					return find_idx;
				}
			}
		}
		return find_idx;
	}


    void HWScenesElements::UpdateCamsFundmentalMatrixs()
    {
        //const std::map<unsigned int, HWCamsIdxGroup> groups = scenes_cams_->GetCamsGroups();
        //const std::vector<std::pair<int, HWCameraModel>> cams_model = scenes_cams_->GetCamerasModels();
        std::map<unsigned int, std::set<unsigned int>>::iterator it;
        for(it = matched_.begin(); it != matched_.end(); ++it)
        {
            unsigned int src_cam_id = it->first;
            std::set<unsigned int> neigbor_cams = it->second;
            std::set<unsigned int>::iterator jt;
            std::map<unsigned int, Eigen::Matrix3f> cams_fund;
            for(jt = neigbor_cams.begin(); jt != neigbor_cams.end(); ++jt)
            {
                //compute fundmental matrix
                unsigned int tgt_cam_id = *jt;
                //get cam model
                CameraModel src_model = scenes_cams_->GetCameraModelFromCamId(src_cam_id);
                CameraModel tgt_model = scenes_cams_->GetCameraModelFromCamId(tgt_cam_id);
                Eigen::Matrix3f F = GetFundamentalMatrix(src_model, tgt_model);
                cams_fund[tgt_cam_id] = F;
            }
            matched_fundamentals_[src_cam_id] = cams_fund;
        }

        ////check F
        //std::map<unsigned int, std::map<unsigned int, Eigen::Matrix3d> >::iterator fit;
        //for(fit = matched_fundamentals_.begin(); fit != matched_fundamentals_.end(); ++fit)
        //{
        //    std::cerr << fit->first <<": " << std::endl;
        //    HWCameraModel srcm = scenes_cams_->GetCameraModelFromCamId(fit->first);
        //    //
        //    std::string srcname = srcm.path_;
        //    std::cerr << srcname << std::endl;
        //    std::map<unsigned int, Eigen::Matrix3d> nf = fit->second;
        //    std::map<unsigned int, Eigen::Matrix3d>::iterator fit_it;
        //    for(fit_it = nf.begin(); fit_it != nf.end(); ++fit_it)
        //    {
        //        std::cerr << fit_it->first << std::endl;
        //        std::cerr << fit_it->second << std::endl;
        //    }
        //    std::cerr << std::endl << std::endl;
        //}
    }

	void HWScenesElements::UpdateLinesMatchesFromCamsGroupsLSD()
	{
		//std::cerr << "to do next..." << std::endl;
		std::map<unsigned int, std::set<unsigned int>>::iterator it;
		for (it = matched_.begin(); it != matched_.end(); ++it)
		{
			//get two IDX
			unsigned int src_cam_id = it->first;
			//if(src_cam_id == 0)
			//{
			//    continue;
			//}
			std::set<unsigned int> neigbor_cams = it->second;
			std::set<unsigned int>::iterator jt;
			//std::map<unsigned int, Eigen::Matrix3d> cams_fund;
			for (jt = neigbor_cams.begin(); jt != neigbor_cams.end(); ++jt)
			{
				unsigned int tgt_cam_id = *jt;
				//std::cerr << "src_cam_id, tgt_cam_id: " << src_cam_id << ", " << tgt_cam_id << std::endl;
				Eigen::Matrix3f F = matched_fundamentals_[src_cam_id][tgt_cam_id];
				MatchLinesFromTwoCamidsLSD(src_cam_id, tgt_cam_id, F);
				//return;
			}
		}

		std::cerr << "end matches lines..." << std::endl;
	}

    void HWScenesElements::UpdateLinesMatchesForCamsImgs()
    {
        std::map<unsigned int, std::set<unsigned int>>::iterator it;
        for(it = matched_.begin(); it != matched_.end(); ++it)
        {
            //get two IDX
            unsigned int src_cam_id = it->first;
            //if(src_cam_id == 0)
            //{
            //    continue;
            //}
            std::set<unsigned int> neigbor_cams = it->second;
            std::set<unsigned int>::iterator jt;
            //std::map<unsigned int, Eigen::Matrix3d> cams_fund;
            for(jt = neigbor_cams.begin(); jt != neigbor_cams.end(); ++jt)
            {
                unsigned int tgt_cam_id = *jt;
                //std::cerr << "src_cam_id, tgt_cam_id: " << src_cam_id << ", " << tgt_cam_id << std::endl;
                Eigen::Matrix3f F = matched_fundamentals_[src_cam_id][tgt_cam_id];
                MatchLinesFromTwoImgs(src_cam_id, tgt_cam_id, F);
                //return;
            }
        }

        std::cerr <<"end matches lines..." << std::endl;
    }

	void HWScenesElements::UpdateLinesMatchesFromCamsAndPolygons()
	{
		std::map<unsigned int, std::set<unsigned int>>::iterator it;
		for (it = matched_.begin(); it != matched_.end(); ++it)
		{
			//get two IDX
			unsigned int src_cam_id = it->first;
			//if(src_cam_id == 0)
			//{
			//    continue;
			//}
			std::set<unsigned int> neigbor_cams = it->second;
			std::set<unsigned int>::iterator jt;
			//std::map<unsigned int, Eigen::Matrix3d> cams_fund;
			for (jt = neigbor_cams.begin(); jt != neigbor_cams.end(); ++jt)
			{
				unsigned int tgt_cam_id = *jt;
				//std::cerr << "src_cam_id, tgt_cam_id: " << src_cam_id << ", " << tgt_cam_id << std::endl;
				Eigen::Matrix3f F = matched_fundamentals_[src_cam_id][tgt_cam_id];
				MatchLinesFromTwoImgs(src_cam_id, tgt_cam_id, F);
				//return;
			}
		}
		std::cerr << "end matches lines..." << std::endl;
	}

    //it is wrong
    double HWScenesElements::MutualOverlap(const std::vector<Eigen::Vector3f>& collinear_pnts)
    {
        double overlap = 0.0;
        if(collinear_pnts.size() != 4)
        {
            return overlap;
        }
        Eigen::Vector3f p1 = collinear_pnts[0];
        Eigen::Vector3f p2 = collinear_pnts[1];
        Eigen::Vector3f q1 = collinear_pnts[2];
        Eigen::Vector3f q2 = collinear_pnts[3];

        Eigen::Vector2f p1d = Eigen::Vector2f(p1[0], p1[1]);
        Eigen::Vector2f p2d = Eigen::Vector2f(p2[0], p2[1]);
        Eigen::Vector2f q1d = Eigen::Vector2f(q1[0], q1[1]);
        Eigen::Vector2f q2d = Eigen::Vector2f(q2[0], q2[1]);

        if(CheckPntOnLineSegOnCollinearPnts(p1d, q1d, q2d) 
            || CheckPntOnLineSegOnCollinearPnts(p2d, q1d, q2d)
            || CheckPntOnLineSegOnCollinearPnts(q1d, p1d, p2d)
            || CheckPntOnLineSegOnCollinearPnts(q2d, p1d, p2d))
        {
            // find outer distance and inner points
            float max_dist = 0.0f;
            size_t outer1 = 0;
            size_t inner1 = 1;
            size_t inner2 = 2;
            size_t outer2 = 3;
            //check four pnts inner or outer
            for(std::size_t i = 0; i < 3; ++i)
            {
                for(std::size_t j = i+1; j < 4; ++j)
                {
                    double dist = (collinear_pnts[i] - collinear_pnts[j]).norm();
                    if(dist > max_dist)
                    {
                        max_dist = dist;
                        outer1 = i;
                        outer2 = j;
                    }
                }
            }
            if(max_dist < 1.0)  //same points
            {
                return 1.0;
            }
            if(outer1 == 0)
            {
                if(outer2 == 1)
                {
                    inner1 = 2;
                    inner2 = 3;
                }
                else if(outer2 == 2)
                {
                    inner1 = 1;
                    inner2 = 3;
                }
                else
                {
                    inner1 = 1;
                    inner2 = 2;
                }
            }
            else if(outer1 == 1)
            {
                inner1 = 0;
                if(outer2 == 2)
                {
                    inner2 = 3;
                }
                else
                {
                    inner2 = 2;
                }
            }
            else
            {
                inner1 = 0;

                inner2 = 1;
            }
            overlap = (collinear_pnts[inner1]-collinear_pnts[inner2]).norm()/max_dist;
        }
        return overlap;
    }

    void HWScenesElements::MatchLinesFromTwoImgs(unsigned int src_id, 
            unsigned int tgt_id, Eigen::Matrix3f& F)
    {
        const std::vector<std::string> image_paths = scenes_cams_->GetImagesPaths();
        int srccamid = static_cast<int>(src_id);
        int tgtcamid = static_cast<int>(tgt_id);
        int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
        int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
        HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
        HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
        std::string src_path = srcimg.GetImagePath();
        std::string tgt_path = tgtimg.GetImagePath();
        std::cerr <<"src_id: " << src_id << std::endl;
        std::cerr <<"tgt_id: " << tgt_id << std::endl;
        std::cerr << src_path << std::endl;
        std::cerr << tgt_path << std::endl;
        cv::Mat v_src = cv::imread(src_path);
        //v_src.Read(src_path);
		cv::Mat v_tgt = cv::imread(tgt_path);
        //v_tgt.Read(tgt_path);
        //int vtwidth = v_tgt.Width();
        //int vtheight = v_tgt.Height();
        //get layout 2d (lines)
        unsigned int src_lid = srcimg.GetLayoutId();
        unsigned int tgt_lid = tgtimg.GetLayoutId();
        //std::cerr <<"src lid, tgt lid: " << src_lid << ", " << tgt_lid << std::endl;
        //return;

        HWSceneLayout2D srcly = scenes_layouts_elements_->GetLayout2DFromId(src_lid);
        HWSceneLayout2D tgtly = scenes_layouts_elements_->GetLayout2DFromId(tgt_lid);

        const std::vector<HWLinePoint2D> srcpnts = srcly.GetLinesPnts();
        const std::vector<Eigen::Vector2i> srclines = srcly.GetLinesIdxs();
        const std::vector<HWLinePoint2D> tgtpnts = tgtly.GetLinesPnts();
        const std::vector<Eigen::Vector2i> tgtlines = tgtly.GetLinesIdxs();
                
        //important
        matches_[src_id].resize(srclines.size());
        //return;

        unsigned int num_matches;
        //std::cerr <<"src lines num: " << srclines.size() << std::endl;
        for(std::size_t i = 0; i < srclines.size(); ++i)
        {
            //if(i != 7)
            //{
            //    continue;
            //}
            unsigned int match_new = 0;
            //src line
            Eigen::Vector3f p1 = srcpnts[srclines[i][0]].pnt_;
            Eigen::Vector3f p2 = srcpnts[srclines[i][1]].pnt_;
            p1[2] = 1.0; p2[2] = 1.0;
            //epipolar line
            Eigen::Vector3f epi_p1 = F * p1;
            Eigen::Vector3f epi_p2 = F * p2;
            
            // use priority queue when kNN > 0
            for(std::size_t j = 0; j < tgtlines.size(); ++j)
            {
                //if(j != 8)
                //{
                //    continue;
                //}
                //std::cerr <<"i, j " << i <<", " << j << std::endl;
                Eigen::Vector3f q1 = tgtpnts[tgtlines[j][0]].pnt_;
                Eigen::Vector3f q2 = tgtpnts[tgtlines[j][1]].pnt_;
                
                //
                q1[2] = 1.0; q2[2] = 1.0;
                Eigen::Vector3f l2 = q1.cross(q2);
                // intersect, how to do it
                Eigen::Vector3f p1_proj = l2.cross(epi_p1);
                Eigen::Vector3f p2_proj = l2.cross(epi_p2);

                ////draw epi lines into images
                //Eigen::Vector2d epi1s(0,0);
                //Eigen::Vector2d epi1e(0,0);
                //Eigen::Vector2d epi2s(0,0);
                //Eigen::Vector2d epi2e(0,0); 
                //Eigen::Vector2d cormin(0,0); 
                //Eigen::Vector2d cormax(vtwidth, vtheight); 
                //bool p1flag = ComputeRectCropLinePnts(epi_p1, cormin, cormax, epi1s, epi1e);
                //bool p2flag = ComputeRectCropLinePnts(epi_p2, cormin, cormax, epi2s, epi2e);
                //std::cerr << "epi_p1: " << epi_p1.transpose() << std::endl;
                //std::cerr << "epi_p2: " << epi_p2.transpose() << std::endl;
                //std::cerr <<"epi1s, epi1e: " << epi1s.transpose() << ", " << epi1e.transpose() << std::endl;
                //std::cerr <<"epi2s, epi2e: " << epi2s.transpose() << ", " << epi2e.transpose() << std::endl;
                ////use trandiciontal method to do its
                //Eigen::Vector2d q12d(q1[0], q1[1]);
                //Eigen::Vector2d q22d(q2[0], q2[1]);
                //Eigen::Vector3d fq = ComputeFunctionFromPnts(q12d,q22d);
                //Eigen::Vector3d fp1 = epi_p1;
                //Eigen::Vector3d fp2 = epi_p2;
                ////draw epi_p1(import)
                //Eigen::Vector2d p1proj2dnew, p2proj2dnew;
                //ComputeLine2LineIntersection(fp1, fq, p1proj2dnew);
                //ComputeLine2LineIntersection(fp2, fq, p2proj2dnew);

                if(std::abs(p1_proj[2]) > KLINE3D_EPS && std::abs(p2_proj[2]) > KLINE3D_EPS)
                {
                    // normalize
                    p1_proj /= p1_proj[2];
                    p2_proj /= p2_proj[2];

                    // check overlap
                    std::vector<Eigen::Vector3f> collinear_points(4);
                    collinear_points[0] = p1_proj;
                    collinear_points[1] = p2_proj;
                    collinear_points[2] = q1;
                    collinear_points[3] = q2;
                    double score = MutualOverlap(collinear_points);
                    //std::cerr <<"score " << score << std::endl;
                    //return;
                    if(score > epipolar_overlap_)   //less than 0.25
                    {
                        //KL3D_EPIPOLAR_OVERLAP
                        Eigen::Vector2f depths_src = TrainglulationDepthFromTwoViews(src_id, p1, p2,
                                                                                    tgt_id, q1, q2);
                        Eigen::Vector2f depths_tgt = TrainglulationDepthFromTwoViews(tgt_id, q1, q2,
                                                                                    src_id, p1, p2);
						std::pair<Eigen::Vector3f, Eigen::Vector3f> src_line_pnts;
						std::pair<Eigen::Vector3f, Eigen::Vector3f> tgt_line_pnts;
						bool src_flag = TrianglulationWorldPntsFromTwoViewsLines(src_id, p1, p2, tgt_id, q1, q2, src_line_pnts);
						bool tgt_flag = TrianglulationWorldPntsFromTwoViewsLines(src_id, q1, q2, tgt_id, p1, p2, tgt_line_pnts);
						
						//test the tri 
                        if(depths_src[0] > KLINE3D_EPS && depths_src[1] > KLINE3D_EPS &&
                        depths_tgt[0] > KLINE3D_EPS && depths_tgt[1] > KLINE3D_EPS)
                        {
                            HWMatch M;
                            M.src_camID_ = src_id;
                            M.src_segID_ = i;
                            M.tgt_camID_ = tgt_id;
                            M.tgt_segID_ = j;
                            M.overlap_score_ = score;
                            M.score3D_ = 0.0;
                            M.depth_p1_ = depths_src[0];
                            M.depth_p2_ = depths_src[1];
                            M.depth_q1_ = depths_tgt[0];
                            M.depth_q2_ = depths_tgt[1];
                            M.valid_match_ = true;
							M.adj_poly_idxs_[0] = -1;
							M.adj_poly_idxs_[1] = -1;
							M.is_polygon_interior_ = -1;
							M.world_p1_ = src_line_pnts.first;
							M.world_p2_ = src_line_pnts.second;
							M.world_q1_ = tgt_line_pnts.first;
							M.world_q2_ = tgt_line_pnts.second;
                            //std::cerr <<"i,j: " << i <<", " << j << std::endl;
                            if(false)
                            {
                                //ought to sort matches_ based on score
                            }
                            else
                            {
                                //std::cerr <<"src_id: " << src_id << std::endl;
                                //std::cerr << "src_path: " << src_path << std::endl;
                                matches_[src_id][i].emplace_back(M);
                                match_new++;
                            }
                        }

                    }
                }
            }
            num_matches += match_new;
        }
        num_matches_[src_id] = num_matches;
        //find layout idx from image idx, to do next... 
        //修改结构，cam 和 image都有 id, id指向path
    }


	void HWScenesElements::MatchLinesFromTwoCamidsLSD(unsigned int src_id,
		unsigned int tgt_id, Eigen::Matrix3f& F)
	{
		const std::vector<std::string> image_paths = scenes_cams_->GetImagesPaths();
		int srccamid = static_cast<int>(src_id);
		int tgtcamid = static_cast<int>(tgt_id);
		int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
		int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
		HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
		HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
		std::string src_path = srcimg.GetImagePath();
		std::string tgt_path = tgtimg.GetImagePath();
		std::cerr << "src_id: " << src_id << std::endl;
		std::cerr << "tgt_id: " << tgt_id << std::endl;
		cv::Mat src_cv_img = srcimg.GetImage().clone();
		cv::Mat tgt_cv_img = tgtimg.GetImage().clone();
		std::vector<cv::line_descriptor::KeyLine> src_lines_lsd = srcimg.GetLsdImageKeyLinesOpencv();
		std::vector<cv::line_descriptor::KeyLine> tgt_lines_lsd = tgtimg.GetLsdImageKeyLinesOpencv();

		//get the image line discripetor
		/* BinaryDescriptorMatcher二值描述子匹配创建 */
		cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher> bdm = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
		cv::Mat left_lsd = srcimg.GetLsdImageLinesDescriptors();
		cv::Mat right_lsd = tgtimg.GetLsdImageLinesDescriptors();

		/* match lines */
		std::vector<cv::DMatch> lines_matches;
		bdm->match(left_lsd, right_lsd, lines_matches);

		/* filter 高精度匹配点对 */
		std::vector<cv::DMatch> good_lines_matches;
		for (int i = 0; i < (int)lines_matches.size(); i++)
		{
			if (lines_matches[i].distance < MATCHES_LINE_DIST_THRESHOLD_ZDG)
				good_lines_matches.push_back(lines_matches[i]);
		}

		/* show the lines matches */
		cv::Mat lsd_outImg;
		cv::resize(src_cv_img, src_cv_img, cv::Size(src_cv_img.cols / 2, src_cv_img.rows / 2));
		cv::resize(tgt_cv_img, tgt_cv_img, cv::Size(tgt_cv_img.cols / 2, tgt_cv_img.rows / 2));
		std::vector<char> lsd_mask(lines_matches.size(), 1);
		cv::line_descriptor::drawLineMatches(src_cv_img, src_lines_lsd, tgt_cv_img, tgt_lines_lsd, good_lines_matches, lsd_outImg,
			cv::Scalar::all(-1), cv::Scalar::all(-1), lsd_mask, cv::line_descriptor::DrawLinesMatchesFlags::DEFAULT);

		cv::imshow("LSD matches", lsd_outImg);
		std::cout << "LSDescriptorMatcher is : " << good_lines_matches.size() << std::endl;

		////set all lines matches to layout2d
		//unsigned int src_lid = srcimg.GetLayoutId();
		//unsigned int tgt_lid = tgtimg.GetLayoutId();
		//HWSceneLayout2D srcly = scenes_layouts_elements_->GetLayout2DFromId(src_lid);
		//HWSceneLayout2D tgtly = scenes_layouts_elements_->GetLayout2DFromId(tgt_lid);

		//match the detected lines with layout(because layout is delete the small lines segment and line near each other)
		std::cerr << "to do next..." << std::endl;
	}

	void HWScenesElements::RunLayout2dProjectPolygon()
	{
		//
		const std::vector<std::unique_ptr<HWSceneLayout2D> >& all_layout2 = scenes_layouts_elements_->GetSceneLayouts2D();
		for (int i = 0; i < all_layout2.size(); ++i)
		{
			HWSceneLayout2D* lay2d = all_layout2[i].get();
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > lay2d_lines_pnts;
			lay2d->GetAllPairLinesPnts(lay2d_lines_pnts);
			const unsigned int imgid = lay2d->GetImageId();
			std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > lay3d_lines_pnts;
			std::vector<std::pair<bool, bool> > lay3d_valid;
			ImageLayoutRay2ScenePolygonPnt3d(imgid, lay3d_valid, lay3d_lines_pnts);
			HWSceneProjLayout3D tmp_lay3d;
			tmp_lay3d.SetLayout2dId(i);
			tmp_lay3d.SetSceneLayout3DValid(lay3d_valid);
			tmp_lay3d.SetSceneLayout3D(lay3d_lines_pnts);
			scenes_layout3ds_.emplace_back(tmp_lay3d);
		}
	}


    Eigen::Vector2f HWScenesElements::TrainglulationDepthFromTwoViews(const unsigned int src_camID, const Eigen::Vector3f& p1,
                const Eigen::Vector3f& p2, const unsigned int tgt_camID,
                const Eigen::Vector3f& line_q1, const Eigen::Vector3f& line_q2)
    {
        //get cam from id
        CameraModel srccam = scenes_cams_->GetCameraModelFromCamId(src_camID);
        CameraModel tgtcam = scenes_cams_->GetCameraModelFromCamId(tgt_camID);
        //rays through pnts
        Eigen::Vector3f c1 = srccam.GetCamC();
        Eigen::Vector3f ray_p1 = srccam.GetRayNormalized(p1);
        Eigen::Vector3f ray_p2 = srccam.GetRayNormalized(p2);

        //epi plane
        Eigen::Vector3f c2 = tgtcam.GetCamC();
        Eigen::Vector3f ray_q1 = tgtcam.GetRayNormalized(line_q1);
        Eigen::Vector3f ray_q2 = tgtcam.GetRayNormalized(line_q2);
        Eigen::Vector3f n = ray_q1.cross(ray_q2);
        n.normalize();
        if(std::abs(ray_p1.dot(n)) < KLINE3D_EPS || std::abs(ray_p2.dot(n)) < KLINE3D_EPS)
        {
            return Eigen::Vector2f(-1.0,-1.0);
        }
        double d1 = (c2.dot(n) - n.dot(c1)) / (n.dot(ray_p1));
        double d2 = (c2.dot(n) - n.dot(c1)) / (n.dot(ray_p2));
        return Eigen::Vector2f(d1, d2);
    }

	bool HWScenesElements::TrianglulationWorldPntsFromTwoViewsLines(const unsigned int src_camID, const Eigen::Vector3f& p1,
		const Eigen::Vector3f& p2, const unsigned int tgt_camID, const Eigen::Vector3f& line_q1,
		const Eigen::Vector3f& line_q2, std::pair<Eigen::Vector3f, Eigen::Vector3f>& p1p2_world_pnts)
	{
		//get cam from id
		CameraModel srccam = scenes_cams_->GetCameraModelFromCamId(src_camID);
		CameraModel tgtcam = scenes_cams_->GetCameraModelFromCamId(tgt_camID);
		//rays through pnts
		Eigen::Vector3f c1 = srccam.GetCamC();
		Eigen::Vector3f ray_p1 = srccam.GetRayNormalized(p1);
		Eigen::Vector3f ray_p2 = srccam.GetRayNormalized(p2);
		//epi plane
		Eigen::Vector3f c2 = tgtcam.GetCamC();
		Eigen::Vector3f ray_q1 = tgtcam.GetRayNormalized(line_q1);
		Eigen::Vector3f ray_q2 = tgtcam.GetRayNormalized(line_q2);
		Eigen::Vector3f n = ray_q1.cross(ray_q2);
		//plane: (c2, n)
		float d = -c2.dot(n);
		Eigen::Vector4f fn = Eigen::Vector4f(n[0], n[1], n[2], d);
		n.normalize();
		p1p2_world_pnts.first = Eigen::Vector3f(KMAX_FLOAT_LIMIT_VALUE, KMAX_FLOAT_LIMIT_VALUE, KMAX_FLOAT_LIMIT_VALUE);
		p1p2_world_pnts.second = Eigen::Vector3f(KMAX_FLOAT_LIMIT_VALUE, KMAX_FLOAT_LIMIT_VALUE, KMAX_FLOAT_LIMIT_VALUE);
		if (std::abs(ray_p1.dot(n)) < KLINE3D_EPS || std::abs(ray_p2.dot(n)) < KLINE3D_EPS)
		{
			return false;
		}
		//ray into plane
		//two ray line: (c1, ray_p1); (c1, ray_p2); plane: (c2, n)
		Eigen::Vector3f w_pnt1, w_pnt2;
		if (Ray2Plane3D(c1, ray_p1, fn, w_pnt1) && Ray2Plane3D(c1, ray_p2, fn, w_pnt2))
		{
			p1p2_world_pnts.first = w_pnt1;
			p1p2_world_pnts.second = w_pnt2;
			return true;
		}
		return false;
	}

    void HWScenesElements::DrawTwoImgsLinesMathed(unsigned int src_id, unsigned int tgt_id)
    {
        std::cerr <<"Draw Two Imgs Lines..." << std::endl;
        //get image
        //HWImages srcimg = scenes_cams_->GetImageFromImgId();
        //if matches is done, to visualize its
        //cam id and img id is wrong, to do next to visualize them
        //get layout idx from cam_id
        //HWCameraModel src_cam = 
        int srccamid = static_cast<int>(src_id);
        int tgtcamid = static_cast<int>(tgt_id);
        std::cerr <<"src_id, tgt_id: " << src_id <<", " << tgt_id << std::endl;
        std::cerr <<"srccamid, tgtcamid: " << srccamid <<", " << tgtcamid << std::endl;
        int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
        int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
        HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
        HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
        std::string src_path = srcimg.GetImagePath();
        std::string tgt_path = tgtimg.GetImagePath();
        unsigned int src_lyid = srcimg.GetLayoutId();
        unsigned int tgt_lyid = tgtimg.GetLayoutId();
        std::vector<std::list<HWMatch>> lines_match = matches_[srccamid];   //get from cams ids
        HWSceneLayout2D srclayout = scenes_layouts_elements_->GetLayout2DFromId(src_lyid);
        HWSceneLayout2D tgtlayout = scenes_layouts_elements_->GetLayout2DFromId(tgt_lyid);
        std::cerr << "srclayout line number: " << srclayout.GetLayoutLinesNum() << std::endl;
        std::cerr << "tgtlayout line number: " << tgtlayout.GetLayoutLinesNum() << std::endl;
		std::cerr << "src2tgt lines matches number: " << lines_match.size() << std::endl;

        //unsigned int srcimgid = srclayout.GetImageId();
        //unsigned int tgtimgid = tgtlayout.GetImageId();
        //HWImages srcimg = scenes_cams_->GetImageFromImgId(srcimgid);
        //HWImages tgtimg = scenes_cams_->GetImageFromImgId(tgtimgid);
        //std::string srcpath = srcimg.GetImagePath();
        //std::string tgtpath = tgtimg.GetImagePath();
		cv::Mat v_src = cv::imread(src_path);
		cv::Mat v_tgt = cv::imread(tgt_path);
        int img1_w = v_src.cols;
        int img1_h = v_src.rows;
        int img2_w = v_tgt.cols;
        int img2_h = v_tgt.rows;
        int out_w = img1_w + img2_w;
        int out_h = std::max(img1_h, img2_h);
        cv::Mat outImg; // = colmap::Bitmap(); 
		cv::resize(v_src, outImg, cv::Size(out_w, out_h));
        //colmap::BitmapColor<uint8_t> outc(0,0,0);
        //outImg.Allocate(out_w, out_h, true);

        //std::string test_path_two = "/home/vradmin/zdg/two_origin.png";
        //outImg.Write(test_path_two);
        //std::cerr <<"end: " << test_path_two << std::endl;
        //return;
        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > clines;
        //get colmap image
        //get src lines
        for(std::size_t i = 0; i < lines_match.size(); ++i)
        {
            //if(i != 7)
            //{
            //    continue;
            //}
            double score_max = std::numeric_limits<double>::lowest();
            std::list<HWMatch> c_match = lines_match[i];
            //std::cerr <<"i: " << i << ", " << "c_match: " << c_match.size() << std::endl;
            std::list<HWMatch>::iterator it;
            unsigned int src_lid = KMAXMUMLIMIT;
            unsigned int tgt_lid = KMAXMUMLIMIT;
            for(it = c_match.begin(); it != c_match.end(); ++it)
            {
                //std::cerr <<"tgt camID: " << it->tgt_camID_ << std::endl;
                if(!it->valid_match_)
                {
                    std::cerr <<"invalid match..........." << std::endl;
                    continue;   //error match
                }
                if(it->tgt_camID_ == tgt_id)
                {
                    //get line seg id
                    //std::cerr <<"it->overlap_score_: " << it->overlap_score_ << std::endl;

                    if(score_max < it->overlap_score_)
                    {
                        src_lid = it->src_segID_;
                        tgt_lid = it->tgt_segID_;
                        score_max = it->overlap_score_;
                    }
                    //score_max
                    //break;
                }
            }
            if(src_lid == KMAXMUMLIMIT || tgt_lid == KMAXMUMLIMIT)
            {
                continue;
            }
            std::cerr <<"max score: " << score_max << std::endl;
            std::cerr << "src lid, tgt_lid: " << src_lid <<", " << tgt_lid << std::endl;

            //get line pos
            //const std::pair<HWLinePoint2D, HWLinePoint2D> srcl = srclayout.GetPickedLine(src_lid);
            //const std::pair<HWLinePoint2D, HWLinePoint2D> tgtl; // = tgtlayout.GetPickedLine(tgt_lid);
            //
            Eigen::Vector3f p1(0,0,1); // = srcl.first.pnt_;
            Eigen::Vector3f p2(0,0,1); // = srcl.first.pnt_;
            Eigen::Vector3f q1(0,0,1); // = tgtl.first.pnt_;
            Eigen::Vector3f q2(0,0,1); // = tgtl.first.pnt_;

            bool srcf = srclayout.GetPickedLineFromIdx(src_lid, p1, p2);
            bool tgtf = tgtlayout.GetPickedLineFromIdx(tgt_lid, q1, q2);
            
            if(!tgtf || !srcf)
            {
                std::cerr << "out of range..." << std::endl;
                continue;
            }

            cv::Vec3i lc(255, 255, 0);
            std::cerr << "random color: " << int(lc[0]) << ", " << int(lc[1]) << ", " << int(lc[2]) << std::endl;
            //p1 p2; q1, q2
            Eigen::Vector2d p1s = Eigen::Vector2d(p1[0], p1[1]);
            Eigen::Vector2d p2s = Eigen::Vector2d(p2[0], p2[1]);
			cv::Point2f p1scv, p2scv;
			p1scv.x = p1s[0];
			p1scv.y = p1s[1];
			p2scv.x = p2s[0];
			p2scv.y = p2s[1];
            //std::cerr <<"p1s, p2s: " << p1s.transpose() <<", " << p2s.transpose() << std::endl;
			cv::line(v_src, p1scv, p2scv, cv::Scalar(255, 0 , 255));
			
			
			//DrawLineIntoBitmap(v_src, p1s, p2s,lc);
            //std::string out_srcpath = "/home/vradmin/zdg/linematch_src.png";
            //v_src.Write(out_srcpath);

            Eigen::Vector2d q1s = Eigen::Vector2d(q1[0], q1[1]);
            Eigen::Vector2d q2s = Eigen::Vector2d(q2[0], q2[1]);
			cv::Point2f q1scv, q2scv;
			q1scv.x = q1s[0];
			q1scv.y = q1s[1];
			q2scv.x = q2s[0];
			q2scv.y = q2s[1];
			//std::cerr <<"q1s, q2s: " << q1s.transpose() <<", " << q2s.transpose() << std::endl;
			cv::line(v_tgt, q1scv, q2scv, cv::Scalar(255, 0, 0));
			
            
            //DrawLineIntoBitmap(v_tgt, q1s, q2s,lc);
            //std::string out_tgtpath = "/home/vradmin/zdg/linematch_tgt.png";
            //v_tgt.Write(out_tgtpath);
			
            //match lines, match color
            Eigen::Vector2d src_mean_pnt;
            src_mean_pnt = (p1s + p2s) / 2;
            Eigen::Vector2d tgt_mean_pnt;
            tgt_mean_pnt = (q1s + q2s) / 2;
            tgt_mean_pnt[0] += img2_w;
            clines.emplace_back(std::make_pair(src_mean_pnt, tgt_mean_pnt));
            //break;
        }
		
		/*std::string out_srcpath = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/src_image_matches.png";
		cv::imwrite(out_srcpath, v_src);
		std::string out_tgtpath = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/tgt_image_matches.png";
		cv::imwrite(out_tgtpath, v_tgt);*/

		cv::Rect src_rect = cv::Rect(0, 0, v_src.cols, v_src.rows);
		v_src.copyTo(outImg(src_rect));
		cv::Rect tgt_rect = cv::Rect(v_src.cols, 0, v_tgt.cols, v_tgt.rows);
		v_tgt.copyTo(outImg(tgt_rect));

		for (int i = 0; i < clines.size(); i++)
		{
			//
			std::pair<Eigen::Vector2d, Eigen::Vector2d> lm_pnt = clines[i];
			cv::Point2f lm_pnt_s, lm_pnt_e;
			lm_pnt_s.x = lm_pnt.first[0];
			lm_pnt_s.y = lm_pnt.first[1];
			lm_pnt_e.x = lm_pnt.second[0];
			lm_pnt_e.y = lm_pnt.second[1];
			cv::line(outImg, lm_pnt_s, lm_pnt_e, cv::Scalar(0,0,255));
		}

		std::string out_fusion_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/line_scene_test/fusion_image_matches_new_02_new.png";
		cv::imwrite(out_fusion_path, outImg);

		//std::cerr << "to draw the matched lines in image..." << std::endl;
		//std::cerr << "to do next..." << std::endl;
		std::cerr << "end draw the matched lines in image..." << std::endl;

#if 0
        //src img to left; tgt img to right
        for(int r = 0; r < img1_h; ++r)
        {
            for(int c = 0; c < img1_w; ++c)
            {
                colmap::BitmapColor<uint8_t> rcp;
                v_src.GetPixel(c, r, &rcp);
                outImg.SetPixel(c,r,rcp);
            }
        }
        for(int r = 0; r < img2_h; ++r)
        {
            for(int c = 0; c < img2_w; ++c)
            {
                colmap::BitmapColor<uint8_t> rcp;
                v_tgt.GetPixel(c, r, &rcp);
                outImg.SetPixel(c + img2_w,r,rcp);
            }
        }

        colmap::BitmapColor<uint8_t> cc(255,0,0);
        for(std::size_t i = 0; i < clines.size(); ++i)
        {
            DrawLineIntoBitmap(outImg, clines[i].first, clines[i].second, cc);
        }

        //std::string test_path_two = "/home/vradmin/zdg/all_two_lines9.png";
        //outImg.Write(test_path_two);
        //std::cerr <<"end: " << test_path_two << std::endl;
#endif

    }

	void HWScenesElements::DrawTwoImgLinesMatchedIntoMat(unsigned int src_id, unsigned int tgt_id, cv::Mat& fusion_mat)
	{
		std::cerr << "Draw Two Imgs Lines..." << std::endl;
		int srccamid = static_cast<int>(src_id);
		int tgtcamid = static_cast<int>(tgt_id);
		std::cerr << "src_id, tgt_id: " << src_id << ", " << tgt_id << std::endl;
		std::cerr << "srccamid, tgtcamid: " << srccamid << ", " << tgtcamid << std::endl;
		int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
		int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
		HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
		HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
		std::string src_path = srcimg.GetImagePath();
		std::string tgt_path = tgtimg.GetImagePath();
		unsigned int src_lyid = srcimg.GetLayoutId();
		unsigned int tgt_lyid = tgtimg.GetLayoutId();
		std::vector<std::list<HWMatch>> lines_match = matches_[srccamid];   //get from cams ids
		HWSceneLayout2D srclayout = scenes_layouts_elements_->GetLayout2DFromId(src_lyid);
		HWSceneLayout2D tgtlayout = scenes_layouts_elements_->GetLayout2DFromId(tgt_lyid);
		std::cerr << "srclayout line number: " << srclayout.GetLayoutLinesNum() << std::endl;
		std::cerr << "tgtlayout line number: " << tgtlayout.GetLayoutLinesNum() << std::endl;
		std::cerr << "src2tgt lines matches number: " << lines_match.size() << std::endl;
		cv::Mat v_src = cv::imread(src_path);
		cv::Mat v_tgt = cv::imread(tgt_path);
		int img1_w = v_src.cols;
		int img1_h = v_src.rows;
		int img2_w = v_tgt.cols;
		int img2_h = v_tgt.rows;
		int out_w = img1_w + img2_w;
		int out_h = std::max(img1_h, img2_h);
		//cv::Mat outImg; // = colmap::Bitmap(); 
		cv::resize(v_src, fusion_mat, cv::Size(out_w, out_h));
		//colmap::BitmapColor<uint8_t> outc(0,0,0);
		//outImg.Allocate(out_w, out_h, true);

		//std::string test_path_two = "/home/vradmin/zdg/two_origin.png";
		//outImg.Write(test_path_two);
		//std::cerr <<"end: " << test_path_two << std::endl;
		//return;
		std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > clines;
		//get colmap image
		//get src lines
		for (std::size_t i = 0; i < lines_match.size(); ++i)
		{
			//if(i != 7)
			//{
			//    continue;
			//}
			double score_max = std::numeric_limits<double>::lowest();
			std::list<HWMatch> c_match = lines_match[i];
			//std::cerr <<"i: " << i << ", " << "c_match: " << c_match.size() << std::endl;
			std::list<HWMatch>::iterator it;
			unsigned int src_lid = KMAXMUMLIMIT;
			unsigned int tgt_lid = KMAXMUMLIMIT;
			for (it = c_match.begin(); it != c_match.end(); ++it)
			{
				//std::cerr <<"tgt camID: " << it->tgt_camID_ << std::endl;
				if (!it->valid_match_)
				{
					std::cerr << "invalid match..........." << std::endl;
					continue;   //error match
				}
				if (it->tgt_camID_ == tgt_id)
				{
					//get line seg id
					//std::cerr <<"it->overlap_score_: " << it->overlap_score_ << std::endl;

					if (score_max < it->overlap_score_)
					{
						src_lid = it->src_segID_;
						tgt_lid = it->tgt_segID_;
						score_max = it->overlap_score_;
					}
					//score_max
					//break;
				}
			}
			if (src_lid == KMAXMUMLIMIT || tgt_lid == KMAXMUMLIMIT)
			{
				continue;
			}
			std::cerr << "max score: " << score_max << std::endl;
			std::cerr << "src lid, tgt_lid: " << src_lid << ", " << tgt_lid << std::endl;

			//get line pos
			//const std::pair<HWLinePoint2D, HWLinePoint2D> srcl = srclayout.GetPickedLine(src_lid);
			//const std::pair<HWLinePoint2D, HWLinePoint2D> tgtl; // = tgtlayout.GetPickedLine(tgt_lid);
			//
			Eigen::Vector3f p1(0, 0, 1); // = srcl.first.pnt_;
			Eigen::Vector3f p2(0, 0, 1); // = srcl.first.pnt_;
			Eigen::Vector3f q1(0, 0, 1); // = tgtl.first.pnt_;
			Eigen::Vector3f q2(0, 0, 1); // = tgtl.first.pnt_;

			bool srcf = srclayout.GetPickedLineFromIdx(src_lid, p1, p2);
			bool tgtf = tgtlayout.GetPickedLineFromIdx(tgt_lid, q1, q2);

			if (!tgtf || !srcf)
			{
				std::cerr << "out of range..." << std::endl;
				continue;
			}

			cv::Vec3i lc(255, 255, 0);
			std::cerr << "random color: " << int(lc[0]) << ", " << int(lc[1]) << ", " << int(lc[2]) << std::endl;
			//p1 p2; q1, q2
			Eigen::Vector2d p1s = Eigen::Vector2d(p1[0], p1[1]);
			Eigen::Vector2d p2s = Eigen::Vector2d(p2[0], p2[1]);
			cv::Point2f p1scv, p2scv;
			p1scv.x = p1s[0];
			p1scv.y = p1s[1];
			p2scv.x = p2s[0];
			p2scv.y = p2s[1];
			//std::cerr <<"p1s, p2s: " << p1s.transpose() <<", " << p2s.transpose() << std::endl;
			cv::line(v_src, p1scv, p2scv, cv::Scalar(255, 0, 255));


			//DrawLineIntoBitmap(v_src, p1s, p2s,lc);
			//std::string out_srcpath = "/home/vradmin/zdg/linematch_src.png";
			//v_src.Write(out_srcpath);

			Eigen::Vector2d q1s = Eigen::Vector2d(q1[0], q1[1]);
			Eigen::Vector2d q2s = Eigen::Vector2d(q2[0], q2[1]);
			cv::Point2f q1scv, q2scv;
			q1scv.x = q1s[0];
			q1scv.y = q1s[1];
			q2scv.x = q2s[0];
			q2scv.y = q2s[1];
			//std::cerr <<"q1s, q2s: " << q1s.transpose() <<", " << q2s.transpose() << std::endl;
			cv::line(v_tgt, q1scv, q2scv, cv::Scalar(255, 0, 0));


			//DrawLineIntoBitmap(v_tgt, q1s, q2s,lc);
			//std::string out_tgtpath = "/home/vradmin/zdg/linematch_tgt.png";
			//v_tgt.Write(out_tgtpath);

			//match lines, match color
			Eigen::Vector2d src_mean_pnt;
			src_mean_pnt = (p1s + p2s) / 2;
			Eigen::Vector2d tgt_mean_pnt;
			tgt_mean_pnt = (q1s + q2s) / 2;
			tgt_mean_pnt[0] += img2_w;
			clines.emplace_back(std::make_pair(src_mean_pnt, tgt_mean_pnt));
			//break;
		}

		/*std::string out_srcpath = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/src_image_matches.png";
		cv::imwrite(out_srcpath, v_src);
		std::string out_tgtpath = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/tgt_image_matches.png";
		cv::imwrite(out_tgtpath, v_tgt);*/

		cv::Rect src_rect = cv::Rect(0, 0, v_src.cols, v_src.rows);
		v_src.copyTo(fusion_mat(src_rect));
		cv::Rect tgt_rect = cv::Rect(v_src.cols, 0, v_tgt.cols, v_tgt.rows);
		v_tgt.copyTo(fusion_mat(tgt_rect));

		for (int i = 0; i < clines.size(); i++)
		{
			//
			std::pair<Eigen::Vector2d, Eigen::Vector2d> lm_pnt = clines[i];
			cv::Point2f lm_pnt_s, lm_pnt_e;
			lm_pnt_s.x = lm_pnt.first[0];
			lm_pnt_s.y = lm_pnt.first[1];
			lm_pnt_e.x = lm_pnt.second[0];
			lm_pnt_e.y = lm_pnt.second[1];
			cv::line(fusion_mat, lm_pnt_s, lm_pnt_e, cv::Scalar(0, 0, 255));
		}

		/*std::string out_fusion_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/fusion_image_matches_new.png";
		cv::imwrite(out_fusion_path, fusion_mat);*/

		std::cerr << "end draw the matched lines in image..." << std::endl;
	}

	void HWScenesElements::DrawOnePairPntsMatchedIntoMat(unsigned int src_id, unsigned int tgt_id,
		const std::pair<Eigen::Vector2f, Eigen::Vector2f>& pair_pnts, cv::Mat& fusion_mat)
	{
		std::cerr << "start to draw the image pnts pair..." << std::endl;
		int srccamid = static_cast<int>(src_id);
		int tgtcamid = static_cast<int>(tgt_id);
		std::cerr << "srccamid, tgtcamid: " << srccamid << ", " << tgtcamid << std::endl;
		int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
		int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
		HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
		HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
		std::string src_path = srcimg.GetImagePath();
		std::string tgt_path = tgtimg.GetImagePath();
		cv::Mat v_src = cv::imread(src_path);
		cv::Mat v_tgt = cv::imread(tgt_path);
		int img1_w = v_src.cols;
		int img1_h = v_src.rows;
		int img2_w = v_tgt.cols;
		int img2_h = v_tgt.rows;
		int out_w = img1_w + img2_w;
		int out_h = std::max(img1_h, img2_h);
		//cv::Mat outImg; // = colmap::Bitmap(); 
		cv::resize(v_src, fusion_mat, cv::Size(out_w, out_h));
		cv::Rect src_rect = cv::Rect(0, 0, v_src.cols, v_src.rows);
		v_src.copyTo(fusion_mat(src_rect));
		cv::Rect tgt_rect = cv::Rect(v_src.cols, 0, v_tgt.cols, v_tgt.rows);
		v_tgt.copyTo(fusion_mat(tgt_rect));
		cv::Point2f src_pnt, tgt_pnt;
		src_pnt.x = pair_pnts.first[0];
		src_pnt.y = pair_pnts.first[1];
		tgt_pnt.x = pair_pnts.second[0] + img1_w;
		tgt_pnt.y = pair_pnts.second[1];
		cv::circle(fusion_mat, src_pnt, 2, cv::Scalar(255, 0, 0), 1);
		cv::circle(fusion_mat, tgt_pnt, 2, cv::Scalar(0, 255, 0), 1);
		cv::line(fusion_mat, src_pnt, tgt_pnt, cv::Scalar(0, 0, 255), 2);
	}

	//pnts from scenes_cams_(origin data...)
	void HWScenesElements::DrawTwoImgsPntsMatchedIntoMatFromPairMatchingViewOrigin(int idx, cv::Mat& fusion_mat)
	{
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			//get images pairs matches
			int srccamid = tmp_pair_views.view_1_id;
			int tgtcamid = tmp_pair_views.view_2_id;
			int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
			int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
			HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
			HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
			/*cv::Mat v_src = srcimg.GetImage();
			cv::Mat v_tgt = tgtimg.GetImage();*/
			std::string src_path = srcimg.GetImagePath();
			std::string tgt_path = tgtimg.GetImagePath();
			cv::Mat v_src = cv::imread(src_path);
			cv::Mat v_tgt = cv::imread(tgt_path);
			int img1_w = v_src.cols;
			int img1_h = v_src.rows;
			int img2_w = v_tgt.cols;
			int img2_h = v_tgt.rows;
			int out_w = img1_w + img2_w;
			int out_h = std::max(img1_h, img2_h);
			//cv::Mat outImg; // = colmap::Bitmap(); 
			cv::resize(v_src, fusion_mat, cv::Size(out_w, out_h));
			
			const std::vector<cv::Point2f> src_pnts = srcimg.GetImageNetWorkPnts();
			const std::vector<cv::Point2f> tgt_pnts = tgtimg.GetImageNetWorkPnts();
			std::vector<HWPntsMatch> pnts_matches = tmp_pair_views.points_matches;
			std::vector<std::pair<cv::Point2f, cv::Point2f> > pnts_matches_lines;
			for (int i = 0; i < pnts_matches.size(); ++i)
			{
				HWPntsMatch tmp_pnt_match = pnts_matches[i];
				std::pair<int, int> tmp_pnt_match_idxs = tmp_pnt_match.pnts_matches_idx_;
				//get the pnts matches
				int lpidx = tmp_pnt_match_idxs.first;
				int rpidx = tmp_pnt_match_idxs.second;
				cv::Point2f tmp_src_pnt = src_pnts[lpidx];
				cv::Point2f tmp_tgt_pnt = tgt_pnts[rpidx];
				tmp_tgt_pnt.x += img2_w;
				pnts_matches_lines.emplace_back(std::make_pair(tmp_src_pnt, tmp_tgt_pnt));
				cv::circle(v_src, tmp_src_pnt, 2, cv::Scalar(0, 0, 255));
				cv::circle(v_tgt, tmp_tgt_pnt, 2, cv::Scalar(255, 0, 0));
			}

			cv::Rect src_rect = cv::Rect(0, 0, v_src.cols, v_src.rows);
			v_src.copyTo(fusion_mat(src_rect));
			cv::Rect tgt_rect = cv::Rect(v_src.cols, 0, v_tgt.cols, v_tgt.rows);
			v_tgt.copyTo(fusion_mat(tgt_rect));

			for (int i = 0; i < pnts_matches_lines.size(); i++)
			{
				//
				std::pair<cv::Point2f, cv::Point2f> c_pnts = pnts_matches_lines[i];
				cv::line(fusion_mat, c_pnts.first, c_pnts.second, cv::Scalar(0, 255, 0));
			}
		}
	}

	//lines from scenes_cams_(origin data...)
	void HWScenesElements::DrawTwoImgsLinesMatchedIntoMatFromPairMatchingViewOrigin(int idx, cv::Mat& fusion_mat)
	{
		if(idx < scenes_lines_pnts_matches_.size() && idx >= 0)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			//get images pairs matches
			int srccamid = tmp_pair_views.view_1_id;
			int tgtcamid = tmp_pair_views.view_2_id;
			int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
			int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
			HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
			HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
			/*cv::Mat v_src = srcimg.GetImage();
			cv::Mat v_tgt = tgtimg.GetImage();*/
			std::string src_path = srcimg.GetImagePath();
			std::string tgt_path = tgtimg.GetImagePath();
			cv::Mat v_src = cv::imread(src_path);
			cv::Mat v_tgt = cv::imread(tgt_path);
			int img1_w = v_src.cols;
			int img1_h = v_src.rows;
			int img2_w = v_tgt.cols;
			int img2_h = v_tgt.rows;
			int out_w = img1_w + img2_w;
			int out_h = std::max(img1_h, img2_h);
			//cv::Mat outImg; // = colmap::Bitmap(); 
			cv::resize(v_src, fusion_mat, cv::Size(out_w, out_h));

			unsigned int src_lyid = srcimg.GetLayoutId();
			unsigned int tgt_lyid = tgtimg.GetLayoutId();
			HWSceneLayout2D srclayout = scenes_layouts_elements_->GetLayout2DFromId(src_lyid);
			HWSceneLayout2D tgtlayout = scenes_layouts_elements_->GetLayout2DFromId(tgt_lyid);
			std::cerr << "srclayout line number: " << srclayout.GetLayoutLinesNum() << std::endl;
			std::cerr << "tgtlayout line number: " << tgtlayout.GetLayoutLinesNum() << std::endl;


			const std::vector<cv::Point2f> src_pnts = srcimg.GetImageNetWorkPnts();
			const std::vector<cv::Point2f> tgt_pnts = tgtimg.GetImageNetWorkPnts();
			std::vector<HWPntsMatch> pnts_matches = tmp_pair_views.points_matches;
			std::vector<std::pair<cv::Point2f, cv::Point2f> > pnts_matches_lines;
			for (int i = 0; i < pnts_matches.size(); ++i)
			{
				HWPntsMatch tmp_pnt_match = pnts_matches[i];
				std::pair<int, int> tmp_pnt_match_idxs = tmp_pnt_match.pnts_matches_idx_;
				//get the pnts matches
				int lpidx = tmp_pnt_match_idxs.first;
				int rpidx = tmp_pnt_match_idxs.second;
				cv::Point2f tmp_src_pnt = src_pnts[lpidx];
				cv::Point2f tmp_tgt_pnt = tgt_pnts[rpidx];
				tmp_tgt_pnt.x += img2_w;
				pnts_matches_lines.emplace_back(std::make_pair(tmp_src_pnt, tmp_tgt_pnt));
				cv::circle(v_src, tmp_src_pnt, 2, cv::Scalar(0, 0, 255));
				cv::circle(v_tgt, tmp_tgt_pnt, 2, cv::Scalar(255, 0, 0));
			}

			cv::Rect src_rect = cv::Rect(0, 0, v_src.cols, v_src.rows);
			v_src.copyTo(fusion_mat(src_rect));
			cv::Rect tgt_rect = cv::Rect(v_src.cols, 0, v_tgt.cols, v_tgt.rows);
			v_tgt.copyTo(fusion_mat(tgt_rect));

			for (int i = 0; i < pnts_matches_lines.size(); i++)
			{
				//
				std::pair<cv::Point2f, cv::Point2f> c_pnts = pnts_matches_lines[i];
				cv::line(fusion_mat, c_pnts.first, c_pnts.second, cv::Scalar(0, 255, 0));
			}
		}
	}

	//lines and points from scenes_cams_(origin data...)
	void HWScenesElements::DrawTwoImgsPntsAndLinesMatchedIntoMatFromPairMatchingViewOrigin(int idx)
	{

	}

	//get the images pairs number
	int HWScenesElements::GetPairwisesMatchingViewsImagesPairsNum()
	{
		int images_num = static_cast<int>(scenes_lines_pnts_matches_.size());
		return images_num;
	}

	//points from hw_cams_views_list_(it is loaded from scenes_cams_ by InitialConvertCamsAndLayoutsAndPntsIntoViewsNetWork())
	void HWScenesElements::DrawTwoImgsPntsMatchedIntoMatFromPairMatchingView(int idx, cv::Mat& fusion_mat)
	{
		std::cerr << "scenes lines pnts match num: " << scenes_lines_pnts_matches_.size() << std::endl;
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			//get images pairs matches
			int srccamid = tmp_pair_views.view_1_id;
			int tgtcamid = tmp_pair_views.view_2_id;
			std::cerr << "DrawTwoImgsPntsMatchedIntoMatFromPairMatchingView->srccamid, tgt_hw_camview_idx: " << srccamid << ", " << tgtcamid << std::endl;
			int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
			int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
			HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
			HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
			std::string src_path = srcimg.GetImagePath();
			std::string tgt_path = tgtimg.GetImagePath();
			cv::Mat v_src = cv::imread(src_path);
			cv::Mat v_tgt = cv::imread(tgt_path);
			//cv::Mat v_src = srcimg.GetImage();
			//cv::Mat v_tgt = tgtimg.GetImage();
			int img1_w = v_src.cols;
			int img1_h = v_src.rows;
			int img2_w = v_tgt.cols;
			int img2_h = v_tgt.rows;
			int out_w = img1_w + img2_w;
			int out_h = std::max(img1_h, img2_h);
			//cv::Mat outImg; // = colmap::Bitmap(); 
			std::cerr << "out_w, out_h: " << out_w << ", " << out_h << std::endl;
			cv::resize(v_src, fusion_mat, cv::Size(out_w, out_h));
			int src_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(srccamid);
			int tgt_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(tgtcamid);
			std::cerr << "src_hw_camview_idx, tgt_hw_camview_idx: " << src_hw_camview_idx << ", " << tgt_hw_camview_idx << std::endl;
			std::vector<Eigen::Vector2f> src_pnts_e = hw_cams_views_list_[src_hw_camview_idx].features_poistion_;
			std::vector<Eigen::Vector2f> tgt_pnts_e = hw_cams_views_list_[tgt_hw_camview_idx].features_poistion_;
			std::vector<cv::Point2f> src_pnts, tgt_pnts;
			src_pnts.resize(src_pnts_e.size());
			tgt_pnts.resize(tgt_pnts_e.size());
			for (int i = 0; i < src_pnts_e.size(); ++i)
			{
				src_pnts[i].x = src_pnts_e[i][0];
				src_pnts[i].y = src_pnts_e[i][1];
			}
			for (int i = 0; i < tgt_pnts_e.size(); ++i)
			{
				tgt_pnts[i].x = tgt_pnts_e[i][0];
				tgt_pnts[i].y = tgt_pnts_e[i][1];
			}

			if (src_pnts.size() > 3 && tgt_pnts.size() > 3)
			{
				std::cerr << "cerr out first 3 elements..." << std::endl;
				for (int i = 0; i < 3; ++i)
				{
					std::cerr << i << ": " << src_pnts[i].x << " " << src_pnts[i].y << ", "
						<< tgt_pnts[i].x << " " << tgt_pnts[i].y << std::endl;
				}
				std::cerr << std::endl << std::endl;
			}

			std::vector<HWPntsMatch> pnts_matches = tmp_pair_views.points_matches;
			std::vector<std::pair<cv::Point2f, cv::Point2f> > pnts_matches_lines;
			for (int i = 0; i < pnts_matches.size(); ++i)
			{
				if (!pnts_matches[i].valid_match_)
				{
					continue;
				}
				HWPntsMatch tmp_pnt_match = pnts_matches[i];
				std::pair<int, int> tmp_pnt_match_idxs = tmp_pnt_match.pnts_matches_idx_;
				//get the pnts matches
				int lpidx = tmp_pnt_match_idxs.first;
				int rpidx = tmp_pnt_match_idxs.second;
				cv::Point2f tmp_src_pnt = src_pnts[lpidx];
				cv::Point2f tmp_tgt_pnt = tgt_pnts[rpidx];
				cv::circle(v_src, tmp_src_pnt, 5, cv::Scalar(0, 0, 255), 2);
				cv::circle(v_tgt, tmp_tgt_pnt, 5, cv::Scalar(255, 0, 0), 2);
				tmp_tgt_pnt.x += img2_w;
				pnts_matches_lines.emplace_back(std::make_pair(tmp_src_pnt, tmp_tgt_pnt));
			}
			cv::Rect src_rect = cv::Rect(0, 0, v_src.cols, v_src.rows);
			v_src.copyTo(fusion_mat(src_rect));
			cv::Rect tgt_rect = cv::Rect(v_src.cols, 0, v_tgt.cols, v_tgt.rows);
			v_tgt.copyTo(fusion_mat(tgt_rect));
			for (int i = 0; i < pnts_matches_lines.size(); i++)
			{
				std::pair<cv::Point2f, cv::Point2f> c_pnts = pnts_matches_lines[i];
				cv::line(fusion_mat, c_pnts.first, c_pnts.second, cv::Scalar(0, 255, 0), 2);
			}
		}
	}

	//lines from hw_cams_views_list_(it is loaded from scenes_cams_ by InitialConvertCamsAndLayoutsAndPntsIntoViewsNetWork())
	void HWScenesElements::DrawTwoImgsLinesMatchedIntoMatFromPairMatchingView(int idx, cv::Mat& fusion_mat)
	{
		std::cerr << "scenes lines pnts match num: " << scenes_lines_pnts_matches_.size() << std::endl;
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			//get images pairs matches
			int srccamid = tmp_pair_views.view_1_id;
			int tgtcamid = tmp_pair_views.view_2_id;
			std::cerr << "DrawTwoImgsLinesMatchedIntoMatFromPairMatchingView->srccamid, tgt_hw_camview_idx: " << srccamid << ", " << tgtcamid << std::endl;
			int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
			int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
			HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
			HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
			/*cv::Mat v_src = srcimg.GetImage();
			cv::Mat v_tgt = tgtimg.GetImage();*/
			std::string src_path = srcimg.GetImagePath();
			std::string tgt_path = tgtimg.GetImagePath();
			cv::Mat v_src = cv::imread(src_path);
			cv::Mat v_tgt = cv::imread(tgt_path);
			int img1_w = v_src.cols;
			int img1_h = v_src.rows;
			int img2_w = v_tgt.cols;
			int img2_h = v_tgt.rows;
			int out_w = img1_w + img2_w;
			int out_h = std::max(img1_h, img2_h);
			//cv::Mat outImg; // = colmap::Bitmap(); 
			cv::resize(v_src, fusion_mat, cv::Size(out_w, out_h));
			int src_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(srccamid);
			int tgt_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(tgtcamid);
			std::cerr << "src_hw_camview_idx, tgt_hw_camview_idx: " << src_hw_camview_idx << ", " << tgt_hw_camview_idx << std::endl;
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > src_lines_e
				= hw_cams_views_list_[src_hw_camview_idx].lines_segments_;
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > tgt_lines_e
				= hw_cams_views_list_[tgt_hw_camview_idx].lines_segments_;
			std::vector<std::pair<cv::Point2f, cv::Point2f> > src_lines, tgt_lines;
			src_lines.resize(src_lines_e.size());
			tgt_lines.resize(tgt_lines_e.size());
			for (int i = 0; i < src_lines_e.size(); ++i)
			{
				src_lines[i].first.x = src_lines_e[i].first[0];
				src_lines[i].first.y = src_lines_e[i].first[1];
				src_lines[i].second.x = src_lines_e[i].second[0];
				src_lines[i].second.y = src_lines_e[i].second[1];
			}
			for (int i = 0; i < tgt_lines_e.size(); ++i)
			{
				tgt_lines[i].first.x = tgt_lines_e[i].first[0];
				tgt_lines[i].first.y = tgt_lines_e[i].first[1];
				tgt_lines[i].second.x = tgt_lines_e[i].second[0];
				tgt_lines[i].second.y = tgt_lines_e[i].second[1];
			}
			std::vector<HWLinesPntsMatch> lines_matches = tmp_pair_views.lines_matches;
			std::vector<std::pair<cv::Point2f, cv::Point2f> > lines_mean_pnts_matches_lines;
			for (int i = 0; i < lines_matches.size(); ++i)
			{
				if (!lines_matches[i].valid_match_)
				{
					continue;
				}
				HWLinesPntsMatch tmp_line_match = lines_matches[i];
				std::pair<int, int> tmp_line_match_idxs = tmp_line_match.line_matches_idx_;
				//get the pnts matches
				int llidx = tmp_line_match_idxs.first;
				int rlidx = tmp_line_match_idxs.second;
				std::pair<cv::Point2f, cv::Point2f> tmp_src_line = src_lines[llidx];
				std::pair<cv::Point2f, cv::Point2f> tmp_tgt_line = tgt_lines[rlidx];
				cv::Point2f tmp_src_line_mean = (tmp_src_line.first + tmp_src_line.second) / 2.0;
				cv::Point2f tmp_tgt_line_mean = (tmp_tgt_line.first + tmp_tgt_line.second) / 2.0;
				tmp_tgt_line_mean.x += img2_w;
				lines_mean_pnts_matches_lines.emplace_back(std::make_pair(tmp_src_line_mean, tmp_tgt_line_mean));
				cv::line(v_src, tmp_src_line.first, tmp_src_line.second, cv::Scalar(0, 0, 255), 3);
				cv::line(v_tgt, tmp_tgt_line.first, tmp_tgt_line.second, cv::Scalar(255, 0, 0), 3);
			}
			cv::Rect src_rect = cv::Rect(0, 0, v_src.cols, v_src.rows);
			v_src.copyTo(fusion_mat(src_rect));
			cv::Rect tgt_rect = cv::Rect(v_src.cols, 0, v_tgt.cols, v_tgt.rows);
			v_tgt.copyTo(fusion_mat(tgt_rect));
			for (int i = 0; i < lines_mean_pnts_matches_lines.size(); i++)
			{
				std::pair<cv::Point2f, cv::Point2f> c_pnts = lines_mean_pnts_matches_lines[i];
				cv::line(fusion_mat, c_pnts.first, c_pnts.second, cv::Scalar(0, 255, 0), 2);
			}
		}
	}

	//lines and points from hw_cams_views_list_(it is loaded from scenes_cams_ by InitialConvertCamsAndLayoutsAndPntsIntoViewsNetWork())
	void HWScenesElements::DrawTwoImgsPntsAndLinesMatchedIntoMatFromPairMatchingView(int idx)
	{
		
	}

	int HWScenesElements::GetViewIdxFromHWCamsViewportListByCamId(int cam_id)
	{
		int idx = -1;
		for (int i = 0; i < hw_cams_views_list_.size(); ++i)
		{
			int tmp_cam_id = hw_cams_views_list_[i].hw_camera_id_;
			if (tmp_cam_id == cam_id)
			{
				idx = i;
				return idx;
			}
		}
		return idx;
	}

	void HWScenesElements::DrawNetworkLinesFromHWImageIntoMat(int camid, cv::Mat& line_mat)
	{
		std::cerr << "DrawNetworkLinesFromHWImageIntoMat->camid: " << camid << std::endl;
		int image_id = scenes_cams_->GetImageidFromCamId(camid);
		HWImage hw_img = scenes_cams_->GetImageFromImgId(image_id);
		std::string image_path = hw_img.GetImagePath();
		cv::Mat v_mat = cv::imread(image_path);
		//get image lines
		unsigned int hw_lyid = hw_img.GetLayoutId();
		HWSceneLayout2D hwlayout = scenes_layouts_elements_->GetLayout2DFromId(hw_lyid);
		//std::cerr << "hwlayout line number: " << hwlayout.GetLayoutLinesNum() << std::endl;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > all_image_lines_pnts = hwlayout.GetAllCurrentPairsLinesPnts();
		//hwlayout.GetAllPairLinesPnts(all_image_lines_pnts);
		std::cerr << "hwlayout line number: " << all_image_lines_pnts.size() << std::endl;
		line_mat = v_mat.clone();
		for (int i = 0; i < all_image_lines_pnts.size(); ++i)
		{
			cv::Point2f tmp_ls(all_image_lines_pnts[i].first[0], all_image_lines_pnts[i].first[1]);
			cv::Point2f tmp_le(all_image_lines_pnts[i].second[0], all_image_lines_pnts[i].second[1]);
			cv::circle(line_mat, tmp_ls, 3, cv::Scalar(255, 0, 0), 2);
			cv::circle(line_mat, tmp_le, 3, cv::Scalar(0, 255, 0), 2);
			cv::line(line_mat, tmp_ls, tmp_le, cv::Scalar(0, 0, 255), 2);
		}
	}

	void HWScenesElements::DrawNetworkGroupedLinesFromHWImageIntoMat(int camid, cv::Mat& line_mat)
	{
		std::cerr << "DrawNetworkGroupedLinesFromHWImageIntoMat->camid: " << camid << std::endl;
		int image_id = scenes_cams_->GetImageidFromCamId(camid);
		HWImage hw_img = scenes_cams_->GetImageFromImgId(image_id);
		std::string image_path = hw_img.GetImagePath();
		cv::Mat v_mat = cv::imread(image_path);
		//get image lines
		unsigned int hw_lyid = hw_img.GetLayoutId();
		HWSceneLayout2D hwlayout = scenes_layouts_elements_->GetLayout2DFromId(hw_lyid);
		//std::cerr << "hwlayout line number: " << hwlayout.GetLayoutLinesNum() << std::endl;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > all_image_lines_pnts = hwlayout.GetAllCurrentPairsLinesPnts();
		//hwlayout.GetAllPairLinesPnts(all_image_lines_pnts);
		std::cerr << "hwlayout line number: " << all_image_lines_pnts.size() << std::endl;
		line_mat = v_mat.clone();
		const std::vector<std::pair<int, int> > all_ids_to_group_ids = hwlayout.GetAllLines2dIdxsToAllGroupedLines2dIdxs();
		const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > all_grouped_lines_pnts = hwlayout.GetAllGroupedLines2dPnts();
		std::cerr << "hwlayout group line number: " << all_grouped_lines_pnts.size() << std::endl;
		for (int i = 0; i < all_grouped_lines_pnts.size(); ++i)
		{
			cv::Point2f tmp_ls(all_grouped_lines_pnts[i].first[0], all_grouped_lines_pnts[i].first[1]);
			cv::Point2f tmp_le(all_grouped_lines_pnts[i].second[0], all_grouped_lines_pnts[i].second[1]);
			cv::circle(line_mat, tmp_ls, 3, cv::Scalar(255, 0, 0), 2);
			cv::circle(line_mat, tmp_le, 3, cv::Scalar(0, 255, 0), 2);
			cv::line(line_mat, tmp_ls, tmp_le, cv::Scalar(0, 0, 255), 2);
		}
	}

	void HWScenesElements::DrawNetworkGroupedLinesFromHWViewIntoMat(int camid, cv::Mat& line_mat)
	{
		int hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(camid);
		std::cerr << "hw_camview_idx: " << hw_camview_idx << std::endl;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > all_grouped_lines_pnts
			= hw_cams_views_list_[hw_camview_idx].grouped_lines_segments_;
		int image_id = scenes_cams_->GetImageidFromCamId(camid);
		HWImage hw_img = scenes_cams_->GetImageFromImgId(image_id);
		std::string image_path = hw_img.GetImagePath();
		cv::Mat v_mat = cv::imread(image_path);
		std::cerr << "hwlayout all_grouped_lines_pnts number: " << all_grouped_lines_pnts.size() << std::endl;
		line_mat = v_mat.clone();
		for (int i = 0; i < all_grouped_lines_pnts.size(); ++i)
		{
			cv::Point2f tmp_ls(all_grouped_lines_pnts[i].first[0], all_grouped_lines_pnts[i].first[1]);
			cv::Point2f tmp_le(all_grouped_lines_pnts[i].second[0], all_grouped_lines_pnts[i].second[1]);
			cv::circle(line_mat, tmp_ls, 3, cv::Scalar(255, 0, 0), 2);
			cv::circle(line_mat, tmp_le, 3, cv::Scalar(0, 255, 0), 2);
			cv::line(line_mat, tmp_ls, tmp_le, cv::Scalar(0, 0, 255), 2);
		}
	}

	void HWScenesElements::DrawTwoImgsPntsMatchedIntoMatFromPairMatchingViewAndImageLabels(int idx, cv::Mat& fusion_mat)
	{
		std::cerr << "scenes lines pnts match num: " << scenes_lines_pnts_matches_.size() << std::endl;
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0
			&& associsated_images_labels_)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			//get images pairs matches
			int srccamid = tmp_pair_views.view_1_id;
			int tgtcamid = tmp_pair_views.view_2_id;
			std::cerr << "DrawTwoImgsPntsMatchedIntoMatFromPairMatchingViewAndImageLabels->srccamid, tgt_hw_camview_idx: "
				<< srccamid << ", " << tgtcamid << std::endl;
			int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
			int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
			HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
			HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
			std::string src_path = srcimg.GetImagePath();
			std::string tgt_path = tgtimg.GetImagePath();
			cv::Mat v_src = cv::imread(src_path);
			cv::Mat v_tgt = cv::imread(tgt_path);
			//cv::Mat v_src = srcimg.GetImage();
			//cv::Mat v_tgt = tgtimg.GetImage();
			int img1_w = v_src.cols;
			int img1_h = v_src.rows;
			int img2_w = v_tgt.cols;
			int img2_h = v_tgt.rows;
			int out_w = img1_w + img2_w;
			int out_h = std::max(img1_h, img2_h);
			//cv::Mat outImg; // = colmap::Bitmap(); 
			std::cerr << "out_w, out_h: " << out_w << ", " << out_h << std::endl;
			cv::resize(v_src, fusion_mat, cv::Size(out_w, out_h));
			int src_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(srccamid);
			int tgt_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(tgtcamid);
			std::cerr << "src_hw_camview_idx, tgt_hw_camview_idx: " << src_hw_camview_idx << ", " << tgt_hw_camview_idx << std::endl;
			std::vector<Eigen::Vector2f> src_pnts_e = hw_cams_views_list_[src_hw_camview_idx].features_poistion_;
			std::vector<Eigen::Vector2f> tgt_pnts_e = hw_cams_views_list_[tgt_hw_camview_idx].features_poistion_;
			std::vector<cv::Point2f> src_pnts, tgt_pnts;
			src_pnts.resize(src_pnts_e.size());
			tgt_pnts.resize(tgt_pnts_e.size());
			for (int i = 0; i < src_pnts_e.size(); ++i)
			{
				src_pnts[i].x = src_pnts_e[i][0];
				src_pnts[i].y = src_pnts_e[i][1];
			}
			for (int i = 0; i < tgt_pnts_e.size(); ++i)
			{
				tgt_pnts[i].x = tgt_pnts_e[i][0];
				tgt_pnts[i].y = tgt_pnts_e[i][1];
			}

			/*if (src_pnts.size() > 3 && tgt_pnts.size() > 3)
			{
				std::cerr << "cerr out first 3 elements..." << std::endl;
				for (int i = 0; i < 3; ++i)
				{
					std::cerr << i << ": " << src_pnts[i].x << " " << src_pnts[i].y << ", "
						<< tgt_pnts[i].x << " " << tgt_pnts[i].y << std::endl;
				}
				std::cerr << std::endl << std::endl;
			}*/

			std::cerr << "111111111111111111111111111111" << std::endl;
			//get image labels data from associsated_images_labels_
			const std::vector<std::string> images_labels_paths = associsated_images_labels_->GetImagesPaths();
			int src_label_idx = GetImageLabelIdxFromImagesLabelPthByHWImagePath(images_labels_paths, src_path);
			int tgt_label_idx = GetImageLabelIdxFromImagesLabelPthByHWImagePath(images_labels_paths, tgt_path);
			if (src_label_idx == -1 || tgt_label_idx == -1)
			{
				return;
			}
			//get label images
			cv::Mat src_label_image = cv::imread(images_labels_paths[src_label_idx]);
			cv::Mat tgt_label_image = cv::imread(images_labels_paths[tgt_label_idx]);
			cv::addWeighted(v_src, 0.5, src_label_image, 0.5, 0.0, v_src);
			cv::addWeighted(v_tgt, 0.5, tgt_label_image, 0.5, 0.0, v_tgt);	//blend the images
			std::cerr << "222222222222222222222222222222" << std::endl;

			std::vector<HWPntsMatch> pnts_matches = tmp_pair_views.points_matches;
			std::vector<std::pair<cv::Point2f, cv::Point2f> > pnts_matches_lines;
			for (int i = 0; i < pnts_matches.size(); ++i)
			{
				if (!pnts_matches[i].valid_match_)
				{
					continue;
				}
				HWPntsMatch tmp_pnt_match = pnts_matches[i];
				std::pair<int, int> tmp_pnt_match_idxs = tmp_pnt_match.pnts_matches_idx_;
				//get the pnts matches
				int lpidx = tmp_pnt_match_idxs.first;
				int rpidx = tmp_pnt_match_idxs.second;
				cv::Point2f tmp_src_pnt = src_pnts[lpidx];
				cv::Point2f tmp_tgt_pnt = tgt_pnts[rpidx];
				cv::circle(v_src, tmp_src_pnt, 5, cv::Scalar(0, 0, 255), 2);
				cv::circle(v_tgt, tmp_tgt_pnt, 5, cv::Scalar(255, 0, 0), 2);
				tmp_tgt_pnt.x += img2_w;
				pnts_matches_lines.emplace_back(std::make_pair(tmp_src_pnt, tmp_tgt_pnt));
			}
			cv::Rect src_rect = cv::Rect(0, 0, v_src.cols, v_src.rows);
			v_src.copyTo(fusion_mat(src_rect));
			cv::Rect tgt_rect = cv::Rect(v_src.cols, 0, v_tgt.cols, v_tgt.rows);
			v_tgt.copyTo(fusion_mat(tgt_rect));
			for (int i = 0; i < pnts_matches_lines.size(); i++)
			{
				std::pair<cv::Point2f, cv::Point2f> c_pnts = pnts_matches_lines[i];
				cv::line(fusion_mat, c_pnts.first, c_pnts.second, cv::Scalar(0, 255, 0), 2);
			}
		}
	}

	void HWScenesElements::DrawTwoImgsLinesMatchedIntoMatFromPairMatchingViewAndImageLabels(int idx, cv::Mat& fusion_mat)
	{
		std::cerr << "scenes lines pnts match num: " << scenes_lines_pnts_matches_.size() << std::endl;
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0
			&& associsated_images_labels_)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			//get images pairs matches
			int srccamid = tmp_pair_views.view_1_id;
			int tgtcamid = tmp_pair_views.view_2_id;
			int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
			int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
			HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
			HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
			/*cv::Mat v_src = srcimg.GetImage();
			cv::Mat v_tgt = tgtimg.GetImage();*/
			std::string src_path = srcimg.GetImagePath();
			std::string tgt_path = tgtimg.GetImagePath();
			cv::Mat v_src = cv::imread(src_path);
			cv::Mat v_tgt = cv::imread(tgt_path);
			int img1_w = v_src.cols;
			int img1_h = v_src.rows;
			int img2_w = v_tgt.cols;
			int img2_h = v_tgt.rows;
			int out_w = img1_w + img2_w;
			int out_h = std::max(img1_h, img2_h);
			//cv::Mat outImg; // = colmap::Bitmap(); 
			cv::resize(v_src, fusion_mat, cv::Size(out_w, out_h));
			int src_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(srccamid);
			int tgt_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(tgtcamid);
			std::cerr << "src_hw_camview_idx, tgt_hw_camview_idx: " << src_hw_camview_idx << ", " << tgt_hw_camview_idx << std::endl;
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > src_lines_e
				= hw_cams_views_list_[src_hw_camview_idx].lines_segments_;
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > tgt_lines_e
				= hw_cams_views_list_[tgt_hw_camview_idx].lines_segments_;
			std::vector<std::pair<cv::Point2f, cv::Point2f> > src_lines, tgt_lines;
			src_lines.resize(src_lines_e.size());
			tgt_lines.resize(tgt_lines_e.size());
			for (int i = 0; i < src_lines_e.size(); ++i)
			{
				src_lines[i].first.x = src_lines_e[i].first[0];
				src_lines[i].first.y = src_lines_e[i].first[1];
				src_lines[i].second.x = src_lines_e[i].second[0];
				src_lines[i].second.y = src_lines_e[i].second[1];
			}
			for (int i = 0; i < tgt_lines_e.size(); ++i)
			{
				tgt_lines[i].first.x = tgt_lines_e[i].first[0];
				tgt_lines[i].first.y = tgt_lines_e[i].first[1];
				tgt_lines[i].second.x = tgt_lines_e[i].second[0];
				tgt_lines[i].second.y = tgt_lines_e[i].second[1];
			}

			//get image labels data from associsated_images_labels_
			std::cerr << "lines: 111111111111111111111111111111" << std::endl;
			//get image labels data from associsated_images_labels_
			const std::vector<std::string> images_labels_paths = associsated_images_labels_->GetImagesPaths();
			int src_label_idx = GetImageLabelIdxFromImagesLabelPthByHWImagePath(images_labels_paths, src_path);
			int tgt_label_idx = GetImageLabelIdxFromImagesLabelPthByHWImagePath(images_labels_paths, tgt_path);
			if (src_label_idx == -1 || tgt_label_idx == -1)
			{
				return;
			}
			//get label images
			cv::Mat src_label_image = cv::imread(images_labels_paths[src_label_idx]);
			cv::Mat tgt_label_image = cv::imread(images_labels_paths[tgt_label_idx]);
			cv::addWeighted(v_src, 0.5, src_label_image, 0.5, 0.0, v_src);
			cv::addWeighted(v_tgt, 0.5, tgt_label_image, 0.5, 0.0, v_tgt);	//blend the images
			std::cerr << "lines: 222222222222222222222222222222" << std::endl;

			std::vector<HWLinesPntsMatch> lines_matches = tmp_pair_views.lines_matches;
			std::vector<std::pair<cv::Point2f, cv::Point2f> > lines_mean_pnts_matches_lines;
			for (int i = 0; i < lines_matches.size(); ++i)
			{
				if (!lines_matches[i].valid_match_)
				{
					continue;
				}
				HWLinesPntsMatch tmp_line_match = lines_matches[i];
				std::pair<int, int> tmp_line_match_idxs = tmp_line_match.line_matches_idx_;
				//get the pnts matches
				int llidx = tmp_line_match_idxs.first;
				int rlidx = tmp_line_match_idxs.second;
				std::pair<cv::Point2f, cv::Point2f> tmp_src_line = src_lines[llidx];
				std::pair<cv::Point2f, cv::Point2f> tmp_tgt_line = tgt_lines[rlidx];
				cv::Point2f tmp_src_line_mean = (tmp_src_line.first + tmp_src_line.second) / 2.0;
				cv::Point2f tmp_tgt_line_mean = (tmp_tgt_line.first + tmp_tgt_line.second) / 2.0;
				tmp_tgt_line_mean.x += img2_w;
				lines_mean_pnts_matches_lines.emplace_back(std::make_pair(tmp_src_line_mean, tmp_tgt_line_mean));
				cv::line(v_src, tmp_src_line.first, tmp_src_line.second, cv::Scalar(0, 0, 255), 3);
				cv::line(v_tgt, tmp_tgt_line.first, tmp_tgt_line.second, cv::Scalar(255, 0, 0), 3);
			}
			
			cv::Rect src_rect = cv::Rect(0, 0, v_src.cols, v_src.rows);
			v_src.copyTo(fusion_mat(src_rect));
			

			cv::Rect tgt_rect = cv::Rect(v_src.cols, 0, v_tgt.cols, v_tgt.rows);
			v_tgt.copyTo(fusion_mat(tgt_rect));
			for (int i = 0; i < lines_mean_pnts_matches_lines.size(); i++)
			{
				std::pair<cv::Point2f, cv::Point2f> c_pnts = lines_mean_pnts_matches_lines[i];
				cv::line(fusion_mat, c_pnts.first, c_pnts.second, cv::Scalar(0, 255, 0), 2);
			}
		}
	}

	void HWScenesElements::DrawTwoImgsPntsMatchedIntoMatFromPairMatchingViewAndImageLabelsAssigned(int idx, cv::Mat& fusion_mat)
	{
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0
			&& associsated_images_labels_)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			cv::Mat v_src, v_tgt;
			std::vector<Eigen::Vector2f> src_pnts_e, tgt_pnts_e;
			GetTwoImageAndPairPntsIdxFromPairMatchingView(idx, v_src, src_pnts_e, v_tgt, tgt_pnts_e);
			int img1_w = v_src.cols;
			int img1_h = v_src.rows;
			int img2_w = v_tgt.cols;
			int img2_h = v_tgt.rows;
			int out_w = img1_w + img2_w;
			int out_h = std::max(img1_h, img2_h);
			//cv::Mat outImg; // = colmap::Bitmap(); 
			std::cerr << "out_w, out_h: " << out_w << ", " << out_h << std::endl;
			cv::resize(v_src, fusion_mat, cv::Size(out_w, out_h));
			std::vector<cv::Point2f> src_pnts, tgt_pnts;
			src_pnts.resize(src_pnts_e.size());
			tgt_pnts.resize(tgt_pnts_e.size());
			for (int i = 0; i < src_pnts_e.size(); ++i)
			{
				src_pnts[i].x = src_pnts_e[i][0];
				src_pnts[i].y = src_pnts_e[i][1];
			}
			for (int i = 0; i < tgt_pnts_e.size(); ++i)
			{
				tgt_pnts[i].x = tgt_pnts_e[i][0];
				tgt_pnts[i].y = tgt_pnts_e[i][1];
			}
			//get label images
			cv::Mat src_label_image, tgt_label_image;
			GetTwoPairLabelsImagesFromCamsViewportByPairMatchingViewIdx(idx, src_label_image, tgt_label_image);
			cv::addWeighted(v_src, 0.5, src_label_image, 0.5, 0.0, v_src);
			cv::addWeighted(v_tgt, 0.5, tgt_label_image, 0.5, 0.0, v_tgt);	//blend the images
			
			std::vector<HWPntsMatch> pnts_matches = tmp_pair_views.points_matches;
			std::vector<std::pair<cv::Point2f, cv::Point2f> > pnts_matches_lines;
			for (int i = 0; i < pnts_matches.size(); ++i)
			{
				if (!pnts_matches[i].valid_match_)
				{
					continue;
				}
				HWPntsMatch tmp_pnt_match = pnts_matches[i];
				std::pair<int, int> tmp_pnt_match_idxs = tmp_pnt_match.pnts_matches_idx_;
				//get the pnts matches
				int lpidx = tmp_pnt_match_idxs.first;
				int rpidx = tmp_pnt_match_idxs.second;
				cv::Point2f tmp_src_pnt = src_pnts[lpidx];
				cv::Point2f tmp_tgt_pnt = tgt_pnts[rpidx];

				//set point circle from
				if (tmp_pnt_match.left_polygon_idx_[0] != -1)
				{
					Eigen::Vector3i tmp_src_pnt_color = associsated_images_labels_->GetRGBFromPlaneLabel(tmp_pnt_match.left_polygon_idx_[0]);
					cv::circle(v_src, tmp_src_pnt, 6, cv::Scalar(tmp_src_pnt_color[2], tmp_src_pnt_color[1], tmp_src_pnt_color[0]), 3);
				}
				else
				{
					cv::circle(v_src, tmp_src_pnt, 5, cv::Scalar(0, 0, 255), 2);
				}

				if (tmp_pnt_match.right_polygon_idx_[0] != -1)
				{
					Eigen::Vector3i tmp_tgt_pnt_color = associsated_images_labels_->GetRGBFromPlaneLabel(tmp_pnt_match.right_polygon_idx_[0]);
					cv::circle(v_tgt, tmp_tgt_pnt, 6, cv::Scalar(tmp_tgt_pnt_color[2], tmp_tgt_pnt_color[1], tmp_tgt_pnt_color[0]), 3);
				}
				else
				{
					cv::circle(v_tgt, tmp_tgt_pnt, 5, cv::Scalar(255, 0, 0), 2);
				}

				tmp_tgt_pnt.x += img2_w;
				pnts_matches_lines.emplace_back(std::make_pair(tmp_src_pnt, tmp_tgt_pnt));
			}
			cv::Rect src_rect = cv::Rect(0, 0, v_src.cols, v_src.rows);
			v_src.copyTo(fusion_mat(src_rect));
			cv::Rect tgt_rect = cv::Rect(v_src.cols, 0, v_tgt.cols, v_tgt.rows);
			v_tgt.copyTo(fusion_mat(tgt_rect));
			for (int i = 0; i < pnts_matches_lines.size(); i++)
			{
				std::pair<cv::Point2f, cv::Point2f> c_pnts = pnts_matches_lines[i];
				cv::line(fusion_mat, c_pnts.first, c_pnts.second, cv::Scalar(0, 255, 0), 3);
			}
		}
	}

	void HWScenesElements::DrawTwoImgsLinesMatchedIntoMatFromPairMatchingViewAndImageLabelsAssigned(int idx, cv::Mat& fusion_mat)
	{
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0
			&& associsated_images_labels_)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			cv::Mat v_src, v_tgt;
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > src_lines_e, tgt_lines_e;
			GetTwoImageAndPairLinesIdxFromPairMatchingView(idx, v_src, src_lines_e, v_tgt, tgt_lines_e);
			int img1_w = v_src.cols;
			int img1_h = v_src.rows;
			int img2_w = v_tgt.cols;
			int img2_h = v_tgt.rows;
			int out_w = img1_w + img2_w;
			int out_h = std::max(img1_h, img2_h);
			//cv::Mat outImg; // = colmap::Bitmap(); 
			cv::resize(v_src, fusion_mat, cv::Size(out_w, out_h));
			std::vector<std::pair<cv::Point2f, cv::Point2f> > src_lines, tgt_lines;
			src_lines.resize(src_lines_e.size());
			tgt_lines.resize(tgt_lines_e.size());
			for (int i = 0; i < src_lines_e.size(); ++i)
			{
				src_lines[i].first.x = src_lines_e[i].first[0];
				src_lines[i].first.y = src_lines_e[i].first[1];
				src_lines[i].second.x = src_lines_e[i].second[0];
				src_lines[i].second.y = src_lines_e[i].second[1];
			}
			for (int i = 0; i < tgt_lines_e.size(); ++i)
			{
				tgt_lines[i].first.x = tgt_lines_e[i].first[0];
				tgt_lines[i].first.y = tgt_lines_e[i].first[1];
				tgt_lines[i].second.x = tgt_lines_e[i].second[0];
				tgt_lines[i].second.y = tgt_lines_e[i].second[1];
			}
			//get label images
			cv::Mat src_label_image, tgt_label_image;
			GetTwoPairLabelsImagesFromCamsViewportByPairMatchingViewIdx(idx, src_label_image, tgt_label_image);
			cv::addWeighted(v_src, 0.5, src_label_image, 0.5, 0.0, v_src);
			cv::addWeighted(v_tgt, 0.5, tgt_label_image, 0.5, 0.0, v_tgt);	//blend the images

			std::vector<HWLinesPntsMatch> lines_matches = tmp_pair_views.lines_matches;
			std::vector<std::pair<cv::Point2f, cv::Point2f> > lines_mean_pnts_matches_lines;
			for (int i = 0; i < lines_matches.size(); ++i)
			{
				if (!lines_matches[i].valid_match_)
				{
					continue;
				}
				HWLinesPntsMatch tmp_line_match = lines_matches[i];
				std::pair<int, int> tmp_line_match_idxs = tmp_line_match.line_matches_idx_;
				//get the pnts matches
				int llidx = tmp_line_match_idxs.first;
				int rlidx = tmp_line_match_idxs.second;
				std::pair<cv::Point2f, cv::Point2f> tmp_src_line = src_lines[llidx];
				std::pair<cv::Point2f, cv::Point2f> tmp_tgt_line = tgt_lines[rlidx];
				cv::Point2f tmp_src_line_mean = (tmp_src_line.first + tmp_src_line.second) / 2.0;
				cv::Point2f tmp_tgt_line_mean = (tmp_tgt_line.first + tmp_tgt_line.second) / 2.0;
				tmp_tgt_line_mean.x += img2_w;
				lines_mean_pnts_matches_lines.emplace_back(std::make_pair(tmp_src_line_mean, tmp_tgt_line_mean));
				
				//set point circle from
				if (tmp_line_match.p_polygon_idx_[0] != -1)
				{
					Eigen::Vector3i tmp_src_line_color = associsated_images_labels_->GetRGBFromPlaneLabel(tmp_line_match.p_polygon_idx_[0]);
					cv::line(v_src, tmp_src_line.first, tmp_src_line.second, cv::Scalar(tmp_src_line_color[2], tmp_src_line_color[1], tmp_src_line_color[0]), 3);
				}
				else
				{
					cv::line(v_src, tmp_src_line.first, tmp_src_line.second, cv::Scalar(0, 0, 255), 3);
				}
				if (tmp_line_match.q_polygon_idx_[0] != -1)
				{
					Eigen::Vector3i tmp_tgt_line_color = associsated_images_labels_->GetRGBFromPlaneLabel(tmp_line_match.q_polygon_idx_[0]);
					cv::line(v_tgt, tmp_tgt_line.first, tmp_tgt_line.second, cv::Scalar(tmp_tgt_line_color[2], tmp_tgt_line_color[1], tmp_tgt_line_color[0]), 3);
				}
				else
				{
					cv::line(v_tgt, tmp_tgt_line.first, tmp_tgt_line.second, cv::Scalar(255, 0, 0), 3);
				}
			}

			cv::Rect src_rect = cv::Rect(0, 0, v_src.cols, v_src.rows);
			v_src.copyTo(fusion_mat(src_rect));
			cv::Rect tgt_rect = cv::Rect(v_src.cols, 0, v_tgt.cols, v_tgt.rows);
			v_tgt.copyTo(fusion_mat(tgt_rect));
			for (int i = 0; i < lines_mean_pnts_matches_lines.size(); i++)
			{
				std::pair<cv::Point2f, cv::Point2f> c_pnts = lines_mean_pnts_matches_lines[i];
				cv::line(fusion_mat, c_pnts.first, c_pnts.second, cv::Scalar(0, 255, 0), 2);
			}
		}
	}

	bool HWScenesElements::GetTwoImageAndPairPntsIdxFromPairMatchingView(int idx, cv::Mat& src_img, std::vector<Eigen::Vector2f>& src_pnts,
		cv::Mat& tgt_img, std::vector<Eigen::Vector2f>& tgt_pnts)
	{
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0)
		{
			GetTwoPairImagesFromCamsViewportByPairMatchingViewIdx(idx, src_img, tgt_img);
			GetTwoImagesPairPntsFromCamsViewportByPairMatchingViewIdx(idx, src_pnts, tgt_pnts);
			return true;
		}
		else
		{
			return false;
		}
	}

	bool HWScenesElements::GetTwoImageAndPairLinesIdxFromPairMatchingView(int idx, cv::Mat& src_img, std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& src_lines,
		cv::Mat& tgt_img, std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& tgt_lines)
	{
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0)
		{
			GetTwoPairImagesFromCamsViewportByPairMatchingViewIdx(idx, src_img, tgt_img);
			GetTwoImagesPairLinesFromCamsViewportByPairMatchingViewIdx(idx, src_lines, tgt_lines);
			return true;
		}
		else
		{
			return false;
		}
	}

	bool HWScenesElements::GetTwoImagesPairPntsFromCamsViewportByPairMatchingViewIdx(int idx,
		std::vector<Eigen::Vector2f>& src_pnts, std::vector<Eigen::Vector2f>& tgt_pnts)
	{
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			//get images pairs matches
			int srccamid = tmp_pair_views.view_1_id;
			int tgtcamid = tmp_pair_views.view_2_id;
			std::cerr << "GetTwoImagesPairPntsFromCamsViewportByPairMatchingViewIdx->srccamid, tgt_hw_camview_idx: "
				<< srccamid << ", " << tgtcamid << std::endl;
			//get src image pnts and tgt image pnts from (srccamid and tgtcamid)
			int src_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(srccamid);
			int tgt_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(tgtcamid);
			//std::cerr << "src_hw_camview_idx, tgt_hw_camview_idx: " << src_hw_camview_idx << ", " << tgt_hw_camview_idx << std::endl;
			src_pnts = hw_cams_views_list_[src_hw_camview_idx].features_poistion_;
			tgt_pnts = hw_cams_views_list_[tgt_hw_camview_idx].features_poistion_;
			return true;
		}
		else
		{
			return false;
		}
	}

	bool HWScenesElements::GetTwoImagesPairLinesFromCamsViewportByPairMatchingViewIdx(int idx, std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& src_lines,
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& tgt_lines)
	{
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			//get images pairs matches
			int srccamid = tmp_pair_views.view_1_id;
			int tgtcamid = tmp_pair_views.view_2_id;
			std::cerr << "GetTwoImagesPairPntsFromCamsViewportByPairMatchingViewIdx->srccamid, tgt_hw_camview_idx: "
				<< srccamid << ", " << tgtcamid << std::endl;
			//get src image pnts and tgt image pnts from (srccamid and tgtcamid)
			int src_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(srccamid);
			int tgt_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(tgtcamid);
			//std::cerr << "src_hw_camview_idx, tgt_hw_camview_idx: " << src_hw_camview_idx << ", " << tgt_hw_camview_idx << std::endl;
			src_lines = hw_cams_views_list_[src_hw_camview_idx].lines_segments_;
			tgt_lines = hw_cams_views_list_[tgt_hw_camview_idx].lines_segments_;
			return true;
		}
		else
		{
			return false;
		}
	}

	bool HWScenesElements::GetTwoPairImagesFromCamsViewportByPairMatchingViewIdx(int idx, cv::Mat& src_img, cv::Mat& tgt_img)
	{
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			//get images pairs matches
			int srccamid = tmp_pair_views.view_1_id;
			int tgtcamid = tmp_pair_views.view_2_id;
			std::cerr << "GetTwoImageAndPairPntsIdxFromPairMatchingView->srccamid, tgt_hw_camview_idx: "
				<< srccamid << ", " << tgtcamid << std::endl;
			int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
			int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
			HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
			HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
			std::string src_path = srcimg.GetImagePath();
			std::string tgt_path = tgtimg.GetImagePath();
			cv::Mat v_src = cv::imread(src_path);
			cv::Mat v_tgt = cv::imread(tgt_path);
			src_img = v_src;
			tgt_img = v_tgt;
			return true;
		}
		else
		{
			return false;
		}
	}

	bool HWScenesElements::GetTwoPairLabelsImagesFromCamsViewportByPairMatchingViewIdx(int idx, cv::Mat& src_label_img, cv::Mat& tgt_label_img)
	{
		if (idx < scenes_lines_pnts_matches_.size() && idx >= 0
			&& associsated_images_labels_)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			//get images pairs matches
			int srccamid = tmp_pair_views.view_1_id;
			int tgtcamid = tmp_pair_views.view_2_id;
			std::cerr << "GetTwoPairLabelsImagesFromCamsViewportByPairMatchingViewIdx->srccamid, tgt_hw_camview_idx: "
				<< srccamid << ", " << tgtcamid << std::endl;
			int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
			int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
			HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
			HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
			std::string src_path = srcimg.GetImagePath();
			std::string tgt_path = tgtimg.GetImagePath();
			//get image labels data from associsated_images_labels_
			const std::vector<std::string> images_labels_paths = associsated_images_labels_->GetImagesPaths();
			int src_label_idx = GetImageLabelIdxFromImagesLabelPthByHWImagePath(images_labels_paths, src_path);
			int tgt_label_idx = GetImageLabelIdxFromImagesLabelPthByHWImagePath(images_labels_paths, tgt_path);
			if (src_label_idx == -1 || tgt_label_idx == -1)
			{
				return false;
			}
			//get label images
			src_label_img = cv::imread(images_labels_paths[src_label_idx]);
			tgt_label_img = cv::imread(images_labels_paths[tgt_label_idx]);
			return true;
		}
		else
		{
			return false;
		}
	}

	void HWScenesElements::GetTwoImgsLinesMatched(unsigned int src_id, unsigned int tgt_id,
		std::vector<HWCorrespondence2D2DLines>& lines_pairs)
	{
		int srccamid = static_cast<int>(src_id);
		int tgtcamid = static_cast<int>(tgt_id);
		std::cerr << "src_id, tgt_id: " << src_id << ", " << tgt_id << std::endl;
		std::cerr << "srccamid, tgtcamid: " << srccamid << ", " << tgtcamid << std::endl;
		int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
		int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
		HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
		HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
		std::string src_path = srcimg.GetImagePath();
		std::string tgt_path = tgtimg.GetImagePath();
		unsigned int src_lyid = srcimg.GetLayoutId();
		unsigned int tgt_lyid = tgtimg.GetLayoutId();
		std::vector<std::list<HWMatch>> lines_match = matches_[srccamid];   //get from cams ids
		HWSceneLayout2D srclayout = scenes_layouts_elements_->GetLayout2DFromId(src_lyid);
		HWSceneLayout2D tgtlayout = scenes_layouts_elements_->GetLayout2DFromId(tgt_lyid);
		std::cerr << "srclayout line number: " << srclayout.GetLayoutLinesNum() << std::endl;
		std::cerr << "tgtlayout line number: " << tgtlayout.GetLayoutLinesNum() << std::endl;
		std::cerr << "src2tgt lines matches number: " << lines_match.size() << std::endl;

		//std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> > clines;
		//get colmap image
		//get src lines
		for (std::size_t i = 0; i < lines_match.size(); ++i)
		{
			double score_max = std::numeric_limits<double>::lowest();
			std::list<HWMatch> c_match = lines_match[i];
			//std::cerr <<"i: " << i << ", " << "c_match: " << c_match.size() << std::endl;
			std::list<HWMatch>::iterator it;
			unsigned int src_lid = KMAXMUMLIMIT;
			unsigned int tgt_lid = KMAXMUMLIMIT;
			for (it = c_match.begin(); it != c_match.end(); ++it)
			{
				//std::cerr <<"tgt camID: " << it->tgt_camID_ << std::endl;
				if (!it->valid_match_)
				{
					std::cerr << "invalid match..........." << std::endl;
					continue;   //error match
				}
				if (it->tgt_camID_ == tgt_id)
				{
					//get line seg id
					//std::cerr <<"it->overlap_score_: " << it->overlap_score_ << std::endl;

					if (score_max < it->overlap_score_)
					{
						src_lid = it->src_segID_;
						tgt_lid = it->tgt_segID_;
						score_max = it->overlap_score_;
					}
					//score_max
					//break;
				}
			}
			if (src_lid == KMAXMUMLIMIT || tgt_lid == KMAXMUMLIMIT)
			{
				continue;
			}
			std::cerr << "max score: " << score_max << std::endl;
			std::cerr << "src lid, tgt_lid: " << src_lid << ", " << tgt_lid << std::endl;

			//get line pos
			//const std::pair<HWLinePoint2D, HWLinePoint2D> srcl = srclayout.GetPickedLine(src_lid);
			//const std::pair<HWLinePoint2D, HWLinePoint2D> tgtl; // = tgtlayout.GetPickedLine(tgt_lid);
			//
			Eigen::Vector3f p1(0, 0, 1); // = srcl.first.pnt_;
			Eigen::Vector3f p2(0, 0, 1); // = srcl.first.pnt_;
			Eigen::Vector3f q1(0, 0, 1); // = tgtl.first.pnt_;
			Eigen::Vector3f q2(0, 0, 1); // = tgtl.first.pnt_;

			bool srcf = srclayout.GetPickedLineFromIdx(src_lid, p1, p2);
			bool tgtf = tgtlayout.GetPickedLineFromIdx(tgt_lid, q1, q2);

			if (!tgtf || !srcf)
			{
				std::cerr << "out of range..." << std::endl;
				continue;
			}
			//p1 p2; q1, q2
			Eigen::Vector2f p1s = Eigen::Vector2f(p1[0], p1[1]);
			Eigen::Vector2f p2s = Eigen::Vector2f(p2[0], p2[1]);
			Eigen::Vector2f q1s = Eigen::Vector2f(q1[0], q1[1]);
			Eigen::Vector2f q2s = Eigen::Vector2f(q2[0], q2[1]);
			HWCorrespondence2D2DLines tmp_correspond_line;
			tmp_correspond_line.line1.first = p1s;
			tmp_correspond_line.line1.second = p2s;
			tmp_correspond_line.line2.first = q1s;
			tmp_correspond_line.line2.second = q2s;
			lines_pairs.emplace_back(tmp_correspond_line);
		}
	}

	void HWScenesElements::GetImagesLinesPairFromImagesPair(unsigned int src_id, unsigned int tgt_id,
		std::vector<HWCorrespondence2D2DLines>& lines_pairs)
	{
		int srccamid = static_cast<int>(src_id);
		int tgtcamid = static_cast<int>(tgt_id);
		std::cerr << "src_id, tgt_id: " << src_id << ", " << tgt_id << std::endl;
		//std::cerr << "srccamid, tgtcamid: " << srccamid << ", " << tgtcamid << std::endl;
		int src_lyid = scenes_cams_->GetLayoutIdFromCamId(srccamid);
		int tgt_lyid = scenes_cams_->GetLayoutIdFromCamId(tgtcamid);
		std::vector<std::list<HWMatch>> lines_match = matches_[srccamid];   //get from cams ids
		HWSceneLayout2D srclayout = scenes_layouts_elements_->GetLayout2DFromId(src_lyid);
		HWSceneLayout2D tgtlayout = scenes_layouts_elements_->GetLayout2DFromId(tgt_lyid);
		for (std::size_t i = 0; i < lines_match.size(); ++i)
		{
			double score_max = std::numeric_limits<double>::lowest();
			std::list<HWMatch> c_match = lines_match[i];
			//std::cerr <<"i: " << i << ", " << "c_match: " << c_match.size() << std::endl;
			std::list<HWMatch>::iterator it;
			unsigned int src_lid = KMAXMUMLIMIT;
			unsigned int tgt_lid = KMAXMUMLIMIT;
			for (it = c_match.begin(); it != c_match.end(); ++it)
			{
				if (!it->valid_match_)
				{
					std::cerr << "invalid match..........." << std::endl;
					continue;   //error match
				}
				if (it->tgt_camID_ == tgt_id)
				{
					src_lid = it->src_segID_;
					tgt_lid = it->tgt_segID_;
				}

				if (src_lid == KMAXMUMLIMIT || tgt_lid == KMAXMUMLIMIT)
				{
					continue;
				}
				std::cerr << "max score: " << score_max << std::endl;
				std::cerr << "src lid, tgt_lid: " << src_lid << ", " << tgt_lid << std::endl;

				//get line pos
				//const std::pair<HWLinePoint2D, HWLinePoint2D> srcl = srclayout.GetPickedLine(src_lid);
				//const std::pair<HWLinePoint2D, HWLinePoint2D> tgtl; // = tgtlayout.GetPickedLine(tgt_lid);
				//
				Eigen::Vector3f p1(0, 0, 1); // = srcl.first.pnt_;
				Eigen::Vector3f p2(0, 0, 1); // = srcl.first.pnt_;
				Eigen::Vector3f q1(0, 0, 1); // = tgtl.first.pnt_;
				Eigen::Vector3f q2(0, 0, 1); // = tgtl.first.pnt_;

				bool srcf = srclayout.GetPickedLineFromIdx(src_lid, p1, p2);
				bool tgtf = tgtlayout.GetPickedLineFromIdx(tgt_lid, q1, q2);

				if (!tgtf || !srcf)
				{
					std::cerr << "out of range..." << std::endl;
					continue;
				}
				//p1 p2; q1, q2
				Eigen::Vector2f p1s = Eigen::Vector2f(p1[0], p1[1]);
				Eigen::Vector2f p2s = Eigen::Vector2f(p2[0], p2[1]);
				Eigen::Vector2f q1s = Eigen::Vector2f(q1[0], q1[1]);
				Eigen::Vector2f q2s = Eigen::Vector2f(q2[0], q2[1]);
				HWCorrespondence2D2DLines tmp_correspond_line;
				tmp_correspond_line.line1.first = p1s;
				tmp_correspond_line.line1.second = p2s;
				tmp_correspond_line.line2.first = q1s;
				tmp_correspond_line.line2.second = q2s;
				lines_pairs.emplace_back(tmp_correspond_line);
			}
		}
	}

	Eigen::Vector2i HWScenesElements::GetViewLineProjectToImageLineIdx(const HWLineFeatureReferenceId& view_line_pair)
	{
		int camid = view_line_pair.view_id_;
		int lineid = view_line_pair.line_id_;
		CameraModel cam = scenes_cams_.get()->GetCameraModelFromCamId(camid);
		Eigen::Vector3f cm_pos = cam.cam_pose_.topRightCorner(3, 1);
		View2PolygonInterLinesData tmp_view_visiable_lines = views_visiable_polygon_inter_lines_[camid];
		std::vector<int> polygon_visiable_lines_idxs = tmp_view_visiable_lines.view_visible_lines_idxs_;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > polygon_visiable_lines 
			= tmp_view_visiable_lines.view_visiable_lines_pnts_;

		int layout_id = cam.layout_id_;
		const HWSceneLayout2D& const picked_layout = scenes_layouts_elements_.get()->GetLayout2DFromId(layout_id);
		//get src line seg
		Eigen::Vector3f src_ls, src_le;
		picked_layout.GetPickedLineFromIdx(lineid, src_ls, src_le);
		std::pair<Eigen::Vector2f, Eigen::Vector2f> src_line_pos;
		src_line_pos.first = Eigen::Vector2f(src_ls[0], src_ls[1]);
		src_line_pos.second = Eigen::Vector2f(src_le[0], src_le[1]);
		float lines_seg_angle_threshold = 5.0;
		float lines_seg_dist_threshold = 5.0;
		LineDistMeasure dist_l2l_min;
		dist_l2l_min.angle_measure_ = 90.0;
		dist_l2l_min.dist_measure_ = 100000.0f;
		int dist_l2l_min_idx = -1;
		for (int k = 0; k < polygon_visiable_lines.size(); ++k)
		{
			//int polygon_line_idx = polygon_visiable_lines_idxs[k];
			Eigen::Vector3f ls = polygon_visiable_lines[k].first;
			Eigen::Vector3f le = polygon_visiable_lines[k].second;
			Eigen::Vector2f ls2f = cam.WorldPnt3d2ImgPnt(ls);
			Eigen::Vector2f le2f = cam.WorldPnt3d2ImgPnt(le);
			//std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_view = polygon_visiable_lines[k];
			std::pair<Eigen::Vector2f, Eigen::Vector2f> img_proj_line_seg;
			img_proj_line_seg.first = ls2f;
			img_proj_line_seg.second = le2f;
			LineDistMeasure line2line_dist = ComputeViewProjLineToImageLineSegDist(src_line_pos, img_proj_line_seg);
			if (line2line_dist.angle_measure_ < lines_seg_angle_threshold
				&& line2line_dist.dist_measure_ < lines_seg_dist_threshold)
			{
				if (line2line_dist < dist_l2l_min)
				{
					dist_l2l_min = line2line_dist;
					dist_l2l_min_idx = k;
				}
			}
		}
		return Eigen::Vector2i(camid, dist_l2l_min_idx);
	}

	std::pair<Eigen::Vector2i, LineDistMeasure> HWScenesElements::ComputeViewVisiableLine3dToImageLineDist(const HWLineFeatureReferenceId& view_line_pair)
	{
		int camid = view_line_pair.view_id_;
		int lineid = view_line_pair.line_id_;
		CameraModel cam = scenes_cams_.get()->GetCameraModelFromCamId(camid);
		Eigen::Vector3f cm_pos = cam.cam_pose_.topRightCorner(3, 1);
		View2PolygonInterLinesData tmp_view_visiable_lines = views_visiable_polygon_inter_lines_[camid];
		std::vector<int> polygon_visiable_lines_idxs = tmp_view_visiable_lines.view_visible_lines_idxs_;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > polygon_visiable_lines
			= tmp_view_visiable_lines.view_visiable_lines_pnts_;
		int layout_id = cam.layout_id_;
		const HWSceneLayout2D& const picked_layout = scenes_layouts_elements_.get()->GetLayout2DFromId(layout_id);
		//get src line seg
		Eigen::Vector3f src_ls, src_le;
		picked_layout.GetPickedLineFromIdx(lineid, src_ls, src_le);
		std::pair<Eigen::Vector2f, Eigen::Vector2f> src_line_pos;
		src_line_pos.first = Eigen::Vector2f(src_ls[0], src_ls[1]);
		src_line_pos.second = Eigen::Vector2f(src_le[0], src_le[1]);
		float lines_seg_angle_threshold = 5.0;
		float lines_seg_dist_threshold = 5.0;
		LineDistMeasure dist_l2l_min;
		dist_l2l_min.angle_measure_ = 90.0;
		dist_l2l_min.dist_measure_ = 100000.0f;
		int dist_l2l_min_idx = -1;
		for (int k = 0; k < polygon_visiable_lines.size(); ++k)
		{
			//int polygon_line_idx = polygon_visiable_lines_idxs[k];
			Eigen::Vector3f ls = polygon_visiable_lines[k].first;
			Eigen::Vector3f le = polygon_visiable_lines[k].second;
			Eigen::Vector2f ls2f = cam.WorldPnt3d2ImgPnt(ls);
			Eigen::Vector2f le2f = cam.WorldPnt3d2ImgPnt(le);
			//std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_view = polygon_visiable_lines[k];
			std::pair<Eigen::Vector2f, Eigen::Vector2f> img_proj_line_seg;
			img_proj_line_seg.first = ls2f;
			img_proj_line_seg.second = le2f;
			LineDistMeasure line2line_dist = ComputeViewProjLineToImageLineSegDist(src_line_pos, img_proj_line_seg);
			if (line2line_dist.angle_measure_ < lines_seg_angle_threshold
				&& line2line_dist.dist_measure_ < lines_seg_dist_threshold)
			{
				if (line2line_dist < dist_l2l_min)
				{
					dist_l2l_min = line2line_dist;
					dist_l2l_min_idx = k;
				}
			}
		}
		Eigen::Vector2i tmp_min_select = Eigen::Vector2i(camid, dist_l2l_min_idx);
		std::pair<Eigen::Vector2i, LineDistMeasure> select_dist
			= std::make_pair(tmp_min_select, dist_l2l_min);
		return select_dist;
	}

	LineDistMeasure HWScenesElements::ComputeLine3dToImageViewLineDist(const std::pair<Eigen::Vector3f, Eigen::Vector3f> line_3d,
		const HWLineFeatureReferenceId& view_line_pair)
	{
		//std::cerr << "to do next..." << std::endl;
		int camid = view_line_pair.view_id_;
		int lineid = view_line_pair.line_id_;
		CameraModel cam = scenes_cams_.get()->GetCameraModelFromCamId(camid);
		//get image line
		int layout_id = cam.layout_id_;
		const HWSceneLayout2D& const picked_layout = scenes_layouts_elements_.get()->GetLayout2DFromId(layout_id);
		//get src line seg
		Eigen::Vector3f src_ls, src_le;
		picked_layout.GetPickedLineFromIdx(lineid, src_ls, src_le);
		std::pair<Eigen::Vector2f, Eigen::Vector2f> src_line_pos;
		src_line_pos.first = Eigen::Vector2f(src_ls[0], src_ls[1]);
		src_line_pos.second = Eigen::Vector2f(src_le[0], src_le[1]);
		//get the line_3d to image line by cam
		Eigen::Vector3f ls = line_3d.first;
		Eigen::Vector3f le = line_3d.second;
		Eigen::Vector2f ls2f = cam.WorldPnt3d2ImgPnt(ls);
		Eigen::Vector2f le2f = cam.WorldPnt3d2ImgPnt(le);
		//std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_view = polygon_visiable_lines[k];
		std::pair<Eigen::Vector2f, Eigen::Vector2f> img_proj_line_seg;
		img_proj_line_seg.first = ls2f;
		img_proj_line_seg.second = le2f;
		LineDistMeasure line2line_dist = ComputeImageLineToImageLineDist(src_line_pos, img_proj_line_seg);
		return line2line_dist;
	}

	LineDistMeasure HWScenesElements::ComputeViewProjLineToImageLineSegDist(const std::pair<Eigen::Vector2f, Eigen::Vector2f> line_seg0,
		const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_seg1)
	{
		Eigen::Vector2f src_ls = line_seg0.first;
		Eigen::Vector2f src_le = line_seg0.second;
		Eigen::Vector2f src_dir = src_le - src_ls;
		float src_line_length = src_dir.norm();
		src_dir.normalize();
		Eigen::Vector2f tgt_ls = line_seg1.first;
		Eigen::Vector2f tgt_le = line_seg1.second;
		Eigen::Vector2f tgt_dir = tgt_le - tgt_ls;
		float tgt_line_length = tgt_dir.norm();
		tgt_dir.normalize();
		float st_angle = ComputeAngleFromTwoLinesVector2D(src_dir, tgt_dir);
		if (st_angle > 90.0)
			st_angle = 180.0 - st_angle;
		float sl_dist, el_dist;
		if (src_line_length < tgt_line_length)
		{
			sl_dist = PntDist2LineSegment2D(src_ls, tgt_ls, tgt_le);
			el_dist = PntDist2LineSegment2D(src_le, tgt_ls, tgt_le);
		}
		else
		{
			sl_dist = PntDist2LineSegment2D(tgt_ls, src_ls, src_le);
			el_dist = PntDist2LineSegment2D(tgt_le, src_ls, src_le);
		}
		float dist_min = sl_dist;
		if (el_dist < sl_dist)
		{
			dist_min = el_dist;
		}
		LineDistMeasure tmp_measure;
		tmp_measure.angle_measure_ = st_angle;
		tmp_measure.dist_measure_ = dist_min;
		return tmp_measure;
	}

	LineDistMeasure HWScenesElements::ComputeImageLineToImageLineDist(const std::pair<Eigen::Vector2f, Eigen::Vector2f> line_seg0,
		const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_seg1)
	{
		Eigen::Vector2f src_ls = line_seg0.first;
		Eigen::Vector2f src_le = line_seg0.second;
		Eigen::Vector2f src_dir = src_le - src_ls;
		float src_line_length = src_dir.norm();
		src_dir.normalize();
		Eigen::Vector2f tgt_ls = line_seg1.first;
		Eigen::Vector2f tgt_le = line_seg1.second;
		Eigen::Vector2f tgt_dir = tgt_le - tgt_ls;
		float tgt_line_length = tgt_dir.norm();
		tgt_dir.normalize();
		float st_angle = ComputeAngleFromTwoLinesVector2D(src_dir, tgt_dir);
		if (st_angle > 90.0)
			st_angle = 180.0 - st_angle;
		float sl_dist, el_dist;
		if (src_line_length < tgt_line_length)
		{
			sl_dist = PntDist2Line2D(src_ls, tgt_ls, tgt_le);
			el_dist = PntDist2Line2D(src_le, tgt_ls, tgt_le);
		}
		else
		{
			sl_dist = PntDist2Line2D(tgt_ls, src_ls, src_le);
			el_dist = PntDist2Line2D(tgt_le, src_ls, src_le);
		}
		float dist_min = sl_dist;
		if (el_dist < sl_dist)
		{
			dist_min = el_dist;
		}
		LineDistMeasure tmp_measure;
		tmp_measure.angle_measure_ = st_angle;
		tmp_measure.dist_measure_ = dist_min;
		return tmp_measure;
	}

    void HWScenesElements::SaveLayoutsintoImages()
    {
        std::string last_dir = scene_dir_.substr(0, scene_dir_.find_last_of("/"));
        std::cerr <<"last dir: " << last_dir << std::endl;
        std::string dir_new = last_dir + "/" + "scene_line_vis/";
        if(!boost::filesystem::exists(last_dir))
        {
            std::cerr <<"last dir does not exist..." << std::endl;
        }
        if(!boost::filesystem::exists(dir_new))
        {
            boost::filesystem::create_directory(dir_new);
        }
		const std::vector<CameraModel> cams = scenes_cams_->GetCamerasModels();
		for (int i = 0; i < cams.size(); ++i)
		{
			int imgid = cams[i].image_id_;
			HWImage scene_img = scenes_cams_->GetImageFromImgId(imgid);
			int layout_id = scene_img.GetLayoutId();
			std::string src_image_path = scene_img.GetImagePath();
			std::string image_base_name = GetBaseNameWithoutSuffix(src_image_path);
			std::string path_new = dir_new + image_base_name + ".png";
			std::cerr << "imgid, layout_id: " << imgid << "," << layout_id << std::endl;
			std::cerr << "save path: " << path_new << std::endl;
			WritePickeLayoutIntoPickedImage(imgid, layout_id, path_new);
		}
    }

	void HWScenesElements::SaveAllLayouts2dIntoDir(const std::string& outdir)
	{
		scenes_layouts_elements_.get()->SetSceneOutDir(outdir);
		scenes_layouts_elements_.get()->SaveAllLayout2dIntoLogs();
	}

#if 0

    void HWScenesElements::DrawPntsIntoBitmap(colmap::Bitmap& img, std::vector<Eigen::Vector2i>& pnts, 
            colmap::BitmapColor<uint8_t>& color)
    {
        int height = img.Height();
        int width = img.Width();
        Eigen::Vector2i mincor = Eigen::Vector2i(0,0);
        Eigen::Vector2i maxcor = Eigen::Vector2i(width, height);    //x: width y:height
        for(std::size_t i = 0; i < pnts.size(); ++i)
        {
            //to do next < width not <= width.
            if(CheckPoint2dInRecti(pnts[i], mincor, maxcor))
            {
                img.SetPixel(pnts[i][0], pnts[i][1], color);
            }
        }
    }

    void HWScenesElements::DrawLineIntoBitmap(colmap::Bitmap& img, const Eigen::Vector2d& s, const Eigen::Vector2d& e, 
        colmap::BitmapColor<uint8_t>& color)
    {
        int height = img.Height();
        int width = img.Width();
        double xwmin = 0; double ywmin = 0; double xwmax = width; double ywmax = height;
        Eigen::Vector2d sn(0,0);
        Eigen::Vector2d en(0,0);
        int v = LiangBarskyClipperAlgo(xwmin, ywmin, xwmax, ywmax, 
            s[0], s[1], e[0], e[1], sn[0], sn[1], en[0], en[1]);
        if(v)
        {
            std::vector<Eigen::Vector2i> pnts;
            SamplePntsFromLineEndPnts2D(sn, en, pnts);
            for(std::size_t i = 0; i < pnts.size(); ++i)
            {
                if(pnts[i][0] >= 0 && pnts[i][0] < width 
                    && pnts[i][1] >= 0 && pnts[i][1] < height)
                {
                    img.SetPixel(pnts[i][0], pnts[i][1], color);
                }
            }
        }
        else
        {
            std::cerr <<"reject from image..." << std::endl;
        }
    }

    void HWScenesElements::DrawEpiLineIntoBitmap(colmap::Bitmap& img, Eigen::Vector3d& epi)    
    {
        //
        int width = img.Width();
        int height = img.Height();
        Eigen::Vector3d p1(0,0,1);
        Eigen::Vector3d p2(width,0,1);
        Eigen::Vector3d p3(width, height, 1);
        Eigen::Vector3d p4(0,height, 1);
        Eigen::Vector3d borders[4];
        borders[0] = p1.cross(p2);  //pluck coord
        borders[1] = p2.cross(p3);
        borders[2] = p3.cross(p4);
        borders[3] = p4.cross(p1);
        std::vector<Eigen::Vector3d> intersections;
        for(std::size_t i = 0; i < 4; ++i)
        {
            Eigen::Vector3d I = borders[i].cross(borders[i]);
            if(std::abs(I[2]) > KLINE3D_EPS)
            {
                I /= I[2];
                I[2] = 1.0;
                //check if in img
                if(I[0] > -1.0 && I[0] < width + 1
                    &&I[1] > -1.0 && I[1] < height + 1)
                {
                    intersections.emplace_back(I);
                }
            }
        }
        if(intersections.size() < 2)
        {
            return;
        }
        //find intersections that are farthest
        double max_dist = 0.0;
        Eigen::Vector3d ep1(0.0,0.0,0.0);
        Eigen::Vector3d ep2(0.0,0.0,0.0);
        for(std::size_t i=0; i < intersections.size() - 1; ++i)
        {
            Eigen::Vector3d p = intersections[i];
            for(std::size_t j = i+1; j < intersections.size(); ++j)
            {
                Eigen::Vector3d q = intersections[j];
                double length = (p-q).norm();
                if(length>max_dist)
                {
                    max_dist = length;
                    ep1 = p;
                    ep2 = q;
                }
            }
        }
        Eigen::Vector2d pt1(ep1[0], ep1[1]);
        Eigen::Vector2d pt2(ep2[0], ep2[1]);
        //DrawLineIntoBitmap();
        //std::vector<Eigen::Vector2i> linepnts;
        colmap::BitmapColor<uint8_t> c(0, 255, 0);
        //SamplePntsFromLineEndPnts2D(pt1, pt1, linepnts);
        DrawLineIntoBitmap(img, pt1, pt2, c);
        //line path
        //std::string imgpath = "/home/vradmin/zdg/test1.png";
        //img.Write(imgpath);
    }
#endif

    void HWScenesElements::ImageLayoutRay2ScenePolygonPnt3d(unsigned int image_id, 
        std::vector<std::pair<bool, bool>>& vec_valid, 
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& pnts3d)
    {
        std::cerr <<"run layout mean pnt recasting..." << std::endl;
        //get layout id
        HW::HWImage img = scenes_cams_->GetImageFromImgId(image_id);
        unsigned int layoutid = img.GetLayoutId();
        std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > layout_pnts;
        HW::HWSceneLayout2D lyout2d = scenes_layouts_elements_->GetLayout2DFromId(layoutid);
        lyout2d.GetAllPairLinesPnts(layout_pnts);
        vec_valid.resize(layout_pnts.size());
        pnts3d.resize(layout_pnts.size());
		//float threshold_rmax = 0.02; //  scene_polygon_data_->GetRmax();
		float threshold_rmax = r_max_polygon_threshold_;
        //get corresponding cam params
        unsigned int camid = img.GetCamId();
        CameraModel cam = scenes_cams_->GetCameraModelFromCamId(camid);
        Eigen::Vector3f cmc = cam.GetCamC();
        for(std::size_t i = 0; i < layout_pnts.size(); ++i)
        {
            Eigen::Vector3f spnt(layout_pnts[i].first[0], layout_pnts[i].first[1], 1.0);
            Eigen::Vector3f sraydir = cam.GetRayNormalized(spnt);
            Eigen::Vector3f epnt(layout_pnts[i].second[0], layout_pnts[i].second[1], 1.0);
            Eigen::Vector3f eraydir = cam.GetRayNormalized(epnt);
            int spoly_id = -1;
            int epoly_id = -1;
            Eigen::Vector3f sinter_pnt = cmc;
            Eigen::Vector3f einter_pnt = cmc;
            bool srayf = LinePntRay2Polygons(cmc, sraydir, threshold_rmax, spoly_id, sinter_pnt);
            bool erayf = LinePntRay2Polygons(cmc, eraydir, threshold_rmax, epoly_id, einter_pnt);
            std::pair<bool, bool> tmpvalid = std::make_pair(false, false);
            //std::cerr <<"sinter_pnt: " << sinter_pnt.transpose() << std::endl;
            //std::cerr <<"einter_pnt: " << einter_pnt.transpose() << std::endl;
            
            if(srayf && erayf)
            {
                tmpvalid.first = true;
                tmpvalid.second = true;
                vec_valid[i] = tmpvalid;
                pnts3d[i] = std::make_pair(sinter_pnt, einter_pnt);
            }
            else if(srayf && !erayf)
            {
                tmpvalid.first = true;
                tmpvalid.second = false;    
                vec_valid[i] = tmpvalid;
                pnts3d[i].first = sinter_pnt;
                pnts3d[i].second = einter_pnt;
            }
            else if(!srayf && erayf)
            {
                tmpvalid.first = false;
                tmpvalid.second = true;    
                vec_valid[i] = tmpvalid;
                pnts3d[i].first = sinter_pnt;
                pnts3d[i].second = einter_pnt;
            }
            else
            {
                tmpvalid.first = false;
                tmpvalid.second = false;    
                vec_valid[i] = tmpvalid;
                pnts3d[i].first = sinter_pnt;
                pnts3d[i].second = einter_pnt;
            }
        }
        //std::vector<HW::HWLinePoint2D> lypnts = lyout2d.GetLinesPnts();
        //std::vector<Eigen::Vector2i> lyidxs = lyout2d.GetLinesIdxs();
        //for(std::size_t i = 0; i < lyidxs.size(); ++i)
        //{
        //    Eigen::Vector3d s = 
        //}
        //std::vector<>

    }

    void HWScenesElements::ImagePickedLineRay2ScenePolygonPnt3d(unsigned int image_id, unsigned int lid, 
        std::pair<bool, bool>& ps_valid, std::pair<Eigen::Vector3f, Eigen::Vector3f>& pnts3d)
    {
        //std::cerr <<"run layout mean pnt recasting..." << std::endl;
        //get layout id
        HW::HWImage img = scenes_cams_->GetImageFromImgId(image_id);
        unsigned int layoutid = img.GetLayoutId();
        HW::HWSceneLayout2D lyout2d = scenes_layouts_elements_->GetLayout2DFromId(layoutid);
        //get line pnts
        std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > layout_pnts;
        lyout2d.GetAllPairLinesPnts(layout_pnts);
        //ps_valid.first = false;
        //ps_valid.second = false;
        //float threshold_rmax = associated_polygons_->GetRmax();
		float threshold_rmax = r_max_polygon_threshold_;
        //get corresponding cam params
        unsigned int camid = img.GetCamId();
        CameraModel cam = scenes_cams_->GetCameraModelFromCamId(camid);
        Eigen::Vector3f cmc = cam.GetCamC();
        Eigen::Vector3f spnt(layout_pnts[lid].first[0], layout_pnts[lid].first[1], 1.0);
        Eigen::Vector3f sraydir = cam.GetRayNormalized(spnt);
        Eigen::Vector3f epnt(layout_pnts[lid].second[0], layout_pnts[lid].second[1], 1.0);
        Eigen::Vector3f eraydir = cam.GetRayNormalized(epnt);
        int spoly_id = -1;
        int epoly_id = -1;
        Eigen::Vector3f sinter_pnt = cmc;
        Eigen::Vector3f einter_pnt = cmc;
        bool srayf = LinePntRay2Polygons(cmc, sraydir, threshold_rmax, spoly_id, sinter_pnt);
        bool erayf = LinePntRay2Polygons(cmc, eraydir, threshold_rmax, epoly_id, einter_pnt);
        std::pair<bool, bool> tmpvalid = std::make_pair(false, false);
		//std::cerr << "spoly_id: " << spoly_id << std::endl;
		//std::cerr << "epoly_id: " << epoly_id << std::endl;
		//std::cerr << "cmc: " << cmc.transpose() << std::endl;
		//std::cerr <<"sinter_pnt: " << sinter_pnt.transpose() << std::endl;
        //std::cerr <<"einter_pnt: " << einter_pnt.transpose() << std::endl;
        
        if(srayf && erayf)
        {
            tmpvalid.first = true;
            tmpvalid.second = true;
            ps_valid = tmpvalid;
            pnts3d = std::make_pair(sinter_pnt, einter_pnt);
        }
        else if(srayf && !erayf)
        {
            tmpvalid.first = true;
            tmpvalid.second = false;    
            ps_valid = tmpvalid;
            pnts3d.first = sinter_pnt;
            pnts3d.second = einter_pnt;
        }
        else if(!srayf && erayf)
        {
            tmpvalid.first = false;
            tmpvalid.second = true;    
            ps_valid = tmpvalid;
            pnts3d.first = sinter_pnt;
            pnts3d.second = einter_pnt;
        }
        else
        {
            tmpvalid.first = false;
            tmpvalid.second = false;    
            ps_valid = tmpvalid;
            pnts3d.first = sinter_pnt;
            pnts3d.second = einter_pnt;
        }
    }

	bool HWScenesElements::ImagePnt2dProj2ScenePolygonPnt3d(CameraModel cam,
		const Eigen::Vector2f& pnt2d, Eigen::Vector3f& pnt3d)
	{
		//to do next...
		const Eigen::Vector3f cmc = cam.GetCamC();
		Eigen::Vector3f spnt(pnt2d[0], pnt2d[1], 1.0);
		Eigen::Vector3f sraydir = cam.GetRayNormalized(spnt);
		int spoly_id = -1;
		Eigen::Vector3f sinter_pnt = cmc;
		bool srayf = LinePntRay2Polygons(cmc, sraydir, r_max_polygon_threshold_, spoly_id, sinter_pnt);
		pnt3d = sinter_pnt;
		return srayf;
	}

	bool HWScenesElements::ImagePnt2dPosProj2ScenePolygonsPnt3dPos(const CameraModel& cam,
		const Eigen::Vector2f& pnt2d, Eigen::Vector3f& pnt3d)
	{
		const Eigen::Vector3f cmc = cam.GetCamC();
		Eigen::Vector3f spnt(pnt2d[0], pnt2d[1], 1.0);
		Eigen::Vector3f sraydir = cam.GetRayNormalized(spnt);
		int spoly_id = -1;
		Eigen::Vector3f sinter_pnt = cmc;
		bool srayf = LinePntRay2Polygons(cmc, sraydir, r_max_polygon_threshold_, spoly_id, sinter_pnt);
		pnt3d = sinter_pnt;
		return srayf;
	}

	int HWScenesElements::ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(const CameraModel& cam,
		const Eigen::Vector2f& pnt2d, Eigen::Vector3f& pnt3d)
	{
		const Eigen::Vector3f cmc = cam.GetCamC();
		Eigen::Vector3f spnt(pnt2d[0], pnt2d[1], 1.0);
		Eigen::Vector3f sraydir = cam.GetRayNormalized(spnt);
		int spoly_id = -1;
		Eigen::Vector3f sinter_pnt = cmc;
		bool srayf = LinePntRay2Polygons(cmc, sraydir, r_max_polygon_threshold_, spoly_id, sinter_pnt);
		pnt3d = sinter_pnt;
		return spoly_id;
	}

	bool HWScenesElements::ImagePnt2dPosProj2ScenePickedPolygonPnt3dPos(const CameraModel& cam,
		const Eigen::Vector2f& pnt2d, int polygon_idx, Eigen::Vector3f& pnt3d)
	{
		const Eigen::Vector3f cmc = cam.GetCamC();
		Eigen::Vector3f spnt(pnt2d[0], pnt2d[1], 1.0);
		Eigen::Vector3f sraydir = cam.GetRayNormalized(spnt);
		int spoly_id = polygon_idx;
		Eigen::Vector3f sinter_pnt = cmc;
		bool srayf = LinePntRay2PickedPolygon(cmc, sraydir, r_max_polygon_threshold_, spoly_id, sinter_pnt);
		pnt3d = sinter_pnt;
		return srayf;
	}

	bool HWScenesElements::ImagePnt2dPosProj2ScenePickedPolygonPlanePnt3dPos(const CameraModel& cam,
		const Eigen::Vector2f& pnt2d, int polygon_idx, Eigen::Vector3f& pnt3d)
	{
		const Eigen::Vector3f cmc = cam.GetCamC();
		Eigen::Vector3f spnt(pnt2d[0], pnt2d[1], 1.0);
		Eigen::Vector3f sraydir = cam.GetRayNormalized(spnt);
		int spoly_id = polygon_idx;
		Eigen::Vector3f sinter_pnt = cmc;
		bool srayf = LinePntRay2PickedPolygonPlane(cmc, sraydir, polygon_idx, sinter_pnt);
		pnt3d = sinter_pnt;
		return srayf;
	}

    bool HWScenesElements::LinePntRay2Polygons(const Eigen::Vector3f& raypnt,
        const Eigen::Vector3f& raydir, float threshold, int& poly_id, Eigen::Vector3f& pnt3d)
    {
        //get polygon
        double mindist = KMAX_DOUBLE_LIMIT_VALUE;
        int minid = -1;
        for(std::size_t i = 0; i < associated_polygons_.size(); ++i)
        {
            HW::HWPlane *poly = associated_polygons_[i];
            poly->SetMaxPolyDistThreshold(threshold);
            bool ray_f = false;
            Eigen::Vector3f tmppnt3d;
            ray_f = poly->RayLinePnt2PolygonPntThreshold(raypnt, raydir, tmppnt3d);
			bool same_direct = false;
			if (raydir.dot(tmppnt3d - raypnt) > 0.0)
			{
				same_direct = true;
			}
			//same_direct = if (raydir.dot(tmppnt3d - raypnt) > 0) ? true : false;;
            if(ray_f && same_direct)
            {
                double dist = (raypnt - tmppnt3d).norm();
                if(dist < mindist)
                {
                    mindist = dist;
                    minid = i;
                    pnt3d = tmppnt3d;
                }
            }
        }
		poly_id = minid;	//assign the idx to polygon id
        if(minid != -1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

	bool HWScenesElements::LinePntRay2PickedPolygon(const Eigen::Vector3f& raypnt,
		const Eigen::Vector3f& raydir, float threshold, int polyid, Eigen::Vector3f& pnt3d)
	{
		if (polyid < 0 || polyid >= associated_polygons_.size())
		{
			return false;
		}
		HW::HWPlane *poly = associated_polygons_[polyid];
		poly->SetMaxPolyDistThreshold(threshold);
		bool ray_f = false;
		Eigen::Vector3f tmppnt3d;
		ray_f = poly->RayLinePnt2PolygonPntThreshold(raypnt, raydir, tmppnt3d);
		pnt3d = tmppnt3d;
		return ray_f;
	}

	bool HWScenesElements::LinePntRay2PickedPolygonPlane(const Eigen::Vector3f& raypnt,
		const Eigen::Vector3f& raydir, int polyid, Eigen::Vector3f& pnt3d)
	{
		if (polyid < 0 || polyid >= associated_polygons_.size())
		{
			return false;
		}
		HW::HWPlane *poly = associated_polygons_[polyid];
		bool ray_f = false;
		Eigen::Vector3f tmppnt3d;
		ray_f = poly->RayToPolyPlaneIntersectionPntConst(raypnt, raydir, tmppnt3d);
		pnt3d = tmppnt3d;
		return ray_f;
	}

    void HWScenesElements::FilterErrorLineMatchesByPolygons()
    {
        if(associated_polygons_.empty())
        {
            std::cerr <<"no scene polygons existed..." << std::endl;
            return;
        }
        //bool mydebug = true;
		if (matches_.empty())
		{
			std::cerr << "matched line is empty, wrong existing..." << std::endl;
		}
        std::map<unsigned int, std::vector<std::list<HWMatch> > >::iterator it;
        for(it=matches_.begin(); it != matches_.end(); ++it)
        {
            //get two image
            unsigned int src_cam_id = it->first;
			//std::cerr << "the src image match lines num: " << num_matches_[src_cam_id] << std::endl;
            std::vector<std::list<HWMatch> > srclym = it->second;
            CameraModel src_cam = scenes_cams_->GetCameraModelFromCamId(src_cam_id);
			//save camera pose

			//test the camera pose
			//std::string campose_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/src_cam_pose.obj";
			//std::cerr << "campose path: " << campose_path << std::endl;
			//src_cam.SaveMeOnlyIntoObj(campose_path, 0.2);
			//return;

            Eigen::Vector3f src_cmc = src_cam.GetCamC();	//camera pose in world coordinate
            unsigned int src_image_id = src_cam.image_id_;
            HW::HWImage src_img = scenes_cams_->GetImageFromImgId(src_image_id);
            unsigned int src_layoutid = src_img.GetLayoutId();
            HW::HWSceneLayout2D src_lyout2d = scenes_layouts_elements_->GetLayout2DFromId(src_layoutid);
			
            for(std::size_t i = 0; i < srclym.size(); ++i)
            {
                //i line to other line
                //src_lyout2d.GetPickedLineFromIdx();
                //if(mydebug && inter_pnts_valid.first)
                //{
                //    std::string path = "/home/vradmin/zdg/lines_test.obj";
                //    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > tmp_lines;
                //    std::pair<Eigen::Vector3d, Eigen::Vector3d> tmp_line(src_cmc, inter_pnts.first);
                //    tmp_lines.emplace_back(tmp_line);
                //    WriteLine3DIntoObj(tmp_lines, path);
                //    mydebug = false;
                //}

                //other matched line idx
                std::list<HWMatch> l2hwmatch = srclym[i];
                std::list<HWMatch>::iterator jter;
                for(jter = l2hwmatch.begin(); jter != l2hwmatch.end(); ++jter)
                {
					//find the idx from scene_line3d_node_
					int src_scene_node_idx = FindLine3dIdxFromImageIdLineIdPair(jter->src_camID_, 
						jter->src_segID_, jter->tgt_camID_, jter->tgt_segID_);
					std::cerr << "src_camID_, src_segID_->tgt_camID_, tgt_segID_: " <<
						jter->src_camID_ << ", " << jter->src_segID_ << "->" <<
						jter->tgt_camID_ << ", " << jter->tgt_segID_ << std::endl;
					src_scene_node_idx = -1;
					if (src_scene_node_idx != -1)
					{
						//set data from scene_line3d_node_
						std::cerr << "src_scene_node_idx: " << src_scene_node_idx << std::endl;
						HWSceneLine3DNode scene_node = scene_line3d_node_[src_scene_node_idx];
						//convert HWSceneLine3DNode to HWMatch
						jter->is_polygon_interior_ = src_scene_node_idx;
						//compute again to solve the 
						jter->adj_poly_idxs_ = scene_node.adj_poly_idxs_;
						//test it use image
						//get data from
						//std::vector<>
						//jter->valid_match_ = true;
						//jter->
					}
					else
					{
						HWMatch M = *jter;
						//double src_p1_depth = M.depth_p1_;
						//double src_p2_depth = M.depth_p2_;
						unsigned int src_lid = M.src_segID_;
						std::pair<bool, bool> inter_pnts_valid;
						std::pair<Eigen::Vector3f, Eigen::Vector3f> inter_pnts;
						ImagePickedLineRay2ScenePolygonPnt3d(src_image_id, src_lid, inter_pnts_valid, inter_pnts);
						//CameraModel cam = scenes_cams_->GetCameraModelFromCamId(jter->src_camID_);
						//get 
						//std::cerr << "first inter_pnts: " << inter_pnts.first.transpose() << std::endl;
						//std::cerr << "second inter_pnts: " << inter_pnts.second.transpose() << std::endl;
						//-------------test------------
						//get HWSceneLayout2D line 
						//draw matched lines from HWMatch M

						//std::string src_match_iline_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/src_cam_proj_line.png";
						//std::string tgt_match_iline_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/tgt_cam_proj_line.png";
						int src_image_id = scenes_cams_->GetImageidFromCamId(jter->src_camID_);
						//std::cerr << "src_match_iline_path: " << src_match_iline_path << std::endl;
						//std::cerr << "tgt_match_iline_path: " << tgt_match_iline_path << std::endl;
						//int src_lyout_id = scenes_cams_->SetLayoutIdFromImageIdx(src_image_id);
						int tgt_cam_id = jter->tgt_camID_;
						std::cerr << "src_cam_id, tgt_cam_id: " << src_cam_id << ", " << tgt_cam_id << std::endl;
						CameraModel tgt_cam = scenes_cams_->GetCameraModelFromCamId(tgt_cam_id);
						Eigen::Vector3f tgt_cmc = tgt_cam.GetCamC();
						unsigned int tgt_image_id = tgt_cam.image_id_;
						HW::HWImage tgt_img = scenes_cams_->GetImageFromImgId(tgt_image_id);
						unsigned int tgt_layoutid = tgt_img.GetLayoutId();
						HW::HWSceneLayout2D tgt_lyout2d = scenes_layouts_elements_->GetLayout2DFromId(tgt_layoutid);
						
						std::pair<Eigen::Vector3f, Eigen::Vector3f> tgt_inter_pnts;
						std::pair<bool, bool> tgt_inter_pnts_valid;
						unsigned int tgt_lid = M.tgt_segID_;
						ImagePickedLineRay2ScenePolygonPnt3d(tgt_image_id, tgt_lid, tgt_inter_pnts_valid, tgt_inter_pnts);
						//std::cerr << "first inter_pnts: " << tgt_inter_pnts.first.transpose() << std::endl;
						//std::cerr << "second inter_pnts: " << tgt_inter_pnts.second.transpose() << std::endl;

						//check two lines
						if (CheckLineSeg3d2LineSeg3dInSameLine(inter_pnts, tgt_inter_pnts, r_max_angle_threshold_, r_max_line_dist_threshold_))
						{
							std::cerr <<"True matched src lid, tgt lid: " << M.src_segID_ << ", " << M.tgt_segID_ << std::endl;
							SetMatchesLinesValid(M.src_camID_, M.src_segID_, M.tgt_camID_, M.tgt_segID_, true);
							M.world_p1_ = inter_pnts.first;
							M.world_p2_ = inter_pnts.second;
							M.world_q1_ = tgt_inter_pnts.first;
							M.world_q2_ = tgt_inter_pnts.second;
						}
						else
						{
							std::cerr << "False src lid, tgt lid: " << M.src_segID_ << ", " << M.tgt_segID_ << std::endl;
							SetMatchesLinesValid(M.src_camID_, M.src_segID_, M.tgt_camID_, M.tgt_segID_, false);
							M.world_p1_ = inter_pnts.first;
							M.world_p2_ = inter_pnts.second;
							M.world_q1_ = tgt_inter_pnts.first;
							M.world_q2_ = tgt_inter_pnts.second;
						}

						//
						//std::string campose_tgt_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/tgt_cam_pose.obj";
						//std::cerr << "campose_tgt_path path: " << campose_tgt_path << std::endl;
						//tgt_cam.SaveMeOnlyIntoObj(campose_tgt_path, 0.2);
						//unsigned int tgt_layoutid = tgt_img.GetLayoutId();
						//DrawLayoutPickedLineIntoImage(src_match_iline_path, src_layoutid, jter->src_segID_);
						//DrawLayoutPickedLineIntoImage(tgt_match_iline_path, tgt_layoutid, jter->tgt_segID_);
						//return;

						//------------end test----------
						
						////get src layout line
						//Eigen::Vector3f src_sp, src_ep;
						//src_lyout2d.GetPickedLineFromIdx(src_lid, src_sp, src_ep);
						//src_sp[2] = src_p1_depth; src_ep[2] = src_p2_depth;
						//Eigen::Vector3f src_cam_sp = src_cam.DepthPnt2CamPnt(src_sp);
						//Eigen::Vector3f src_cam_ep = src_cam.DepthPnt2CamPnt(src_ep);
						//Eigen::Vector3f src_world_sp = src_cam.CamPnt2WorldPnt(src_cam_sp);
						//Eigen::Vector3f src_world_ep = src_cam.CamPnt2WorldPnt(src_cam_ep);

						//std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > src_lines_pnts;
						//src_lines_pnts.emplace_back(std::make_pair(src_cmc, src_world_sp));
						//src_lines_pnts.emplace_back(std::make_pair(src_world_sp, src_world_ep));
						//src_lines_pnts.emplace_back(std::make_pair(src_world_ep, src_cmc));
						//std::string cmc2lines_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/src_line_match_depth_line3d.obj";
						//WriteLine3DIntoObj(src_lines_pnts, cmc2lines_path);

						/*std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > src_lines_pnts;
						src_lines_pnts.emplace_back(std::make_pair(src_cmc, inter_pnts.first));
						src_lines_pnts.emplace_back(std::make_pair(inter_pnts.first, inter_pnts.second));
						src_lines_pnts.emplace_back(std::make_pair(inter_pnts.second, src_cmc));
						std::string cmc2lines_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/src_line_match_polygon_line3d.obj";
						WriteLine3DIntoObj(src_lines_pnts, cmc2lines_path);*/

						/*std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > tgt_lines_pnts;
						tgt_lines_pnts.emplace_back(std::make_pair(tgt_cmc, tgt_inter_pnts.first));
						tgt_lines_pnts.emplace_back(std::make_pair(tgt_inter_pnts.first, tgt_inter_pnts.second));
						tgt_lines_pnts.emplace_back(std::make_pair(tgt_inter_pnts.second, tgt_cmc));
						std::string tgt_cmc2lines_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/tgt_line_match_polygon_line3d.obj";
						WriteLine3DIntoObj(tgt_lines_pnts, tgt_cmc2lines_path);*/

						////get tgt layout line
						//Eigen::Vector3f tgt_sp, tgt_ep;
						//int tgt_lid = jter->tgt_segID_;
						//tgt_lyout2d.GetPickedLineFromIdx(tgt_lid, tgt_sp, tgt_ep);
						//double tgt_p1_depth = M.depth_q1_;
						//double tgt_p2_depth = M.depth_q2_;
						//tgt_sp[2] = tgt_p1_depth; tgt_ep[2] = tgt_p2_depth;
						//Eigen::Vector3f tgt_cam_sp = tgt_cam.DepthPnt2CamPnt(tgt_sp);
						//Eigen::Vector3f tgt_cam_ep = tgt_cam.DepthPnt2CamPnt(tgt_ep);
						//Eigen::Vector3f tgt_world_sp = tgt_cam.CamPnt2WorldPnt(tgt_cam_sp);
						//Eigen::Vector3f tgt_world_ep = tgt_cam.CamPnt2WorldPnt(tgt_cam_ep);

						/*std::pair<bool, bool> inter_tgt_pnts_valid;
						std::pair<Eigen::Vector3f, Eigen::Vector3f> inter_tgt_pnts;
						ImagePickedLineRay2ScenePolygonPnt3d(src_image_id, src_lid, inter_pnts_valid, inter_pnts);
						std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > tgt_lines_pnts;
						tgt_lines_pnts.emplace_back(std::make_pair(tgt_cmc, tgt_world_sp));
						tgt_lines_pnts.emplace_back(std::make_pair(tgt_world_sp, tgt_world_ep));
						tgt_lines_pnts.emplace_back(std::make_pair(tgt_world_ep, tgt_cmc));
						std::string tgt_cmc2lines_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/tgt_line_match_depth_line3d.obj";
						WriteLine3DIntoObj(tgt_lines_pnts, tgt_cmc2lines_path);*/

						//return;

						////u,v? 
						//////src_p1_depth,src_p2_depth: 
						//////need to translate from image coord to world coord or cam coord, then check its
						////to do next...
						//if(mydebug && (inter_pnts_valid.first || inter_pnts_valid.second))
						//{
						//    Eigen::Vector3d src_world_sp = src_cam.DepthPnt2WorldPnt(src_sp);
						//    Eigen::Vector3d src_world_ep = src_cam.DepthPnt2WorldPnt(src_ep);
						//    std::string path = "/home/vradmin/zdg/lines_match_test.obj";
						//    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > tmp_lines;
						//    std::pair<Eigen::Vector3d, Eigen::Vector3d> tmp_line(src_world_sp, src_world_ep);
						//    tmp_lines.emplace_back(tmp_line);
						//    WriteLine3DIntoObj(tmp_lines, path);
						//    std::cerr <<"end save the obj file..." << std::endl;
						//    mydebug = false;
						//}

						
					}
                }
            }
        }

		//run view matched 
		//RunLineMatchesIntoViews();

		filter_wrong_lines_match_ = true;
    }

	void HWScenesElements::FilterErrorLineMatchesWithPolygonsByReprojection()
	{
		if (associated_polygons_.empty())
		{
			std::cerr << "no scene polygons existed..." << std::endl;
			return;
		}
		//bool mydebug = true;
		if (matches_.empty())
		{
			std::cerr << "matched line is empty, wrong existing..." << std::endl;
		}

		std::map<unsigned int, std::vector<std::list<HWMatch> > >::iterator it;
		for (it = matches_.begin(); it != matches_.end(); ++it)
		{
			//get two image
			unsigned int src_cam_id = it->first;
			//std::cerr << "the src image match lines num: " << num_matches_[src_cam_id] << std::endl;
			std::vector<std::list<HWMatch> > srclym = it->second;
			CameraModel src_cam = scenes_cams_->GetCameraModelFromCamId(src_cam_id);
			//save camera pose

			Eigen::Vector3f src_cmc = src_cam.GetCamC();	//camera pose in world coordinate
			unsigned int src_image_id = src_cam.image_id_;
			HW::HWImage src_img = scenes_cams_->GetImageFromImgId(src_image_id);
			unsigned int src_layoutid = src_img.GetLayoutId();
			HW::HWSceneLayout2D src_lyout2d = scenes_layouts_elements_->GetLayout2DFromId(src_layoutid);

			for (std::size_t i = 0; i < srclym.size(); ++i)
			{
				//other matched line idx
				std::list<HWMatch> l2hwmatch = srclym[i];
				std::list<HWMatch>::iterator jter;
				for (jter = l2hwmatch.begin(); jter != l2hwmatch.end(); ++jter)
				{
					//find the idx from scene_line3d_node_
					int src_scene_node_idx = FindLine3dIdxFromImageIdLineIdPair(jter->src_camID_,
						jter->src_segID_, jter->tgt_camID_, jter->tgt_segID_);
					std::cerr << "src_camID_, src_segID_->tgt_camID_, tgt_segID_: " <<
						jter->src_camID_ << ", " << jter->src_segID_ << "->" <<
						jter->tgt_camID_ << ", " << jter->tgt_segID_ << std::endl;
					HWMatch M = *jter;
					std::cerr << "True matched src lid, tgt lid: " << M.src_segID_ << ", " << M.tgt_segID_ << std::endl;
					SetMatchesLinesValid(M.src_camID_, M.src_segID_, M.tgt_camID_, M.tgt_segID_, true);
				}
			}
		}

		filter_wrong_lines_match_ = true;
	}

	void HWScenesElements::FilterErrorLinesMatchesByPolygonsNetNumpy()
	{
		if (associated_polygons_.empty())
		{
			std::cerr << "no scene polygons existed..." << std::endl;
			return;
		}
		int scenes_lines_pnts_matches_num = static_cast<int>(scenes_lines_pnts_matches_.size());
		std::cerr << "scenes_lines_pnts_matches_num: " << scenes_lines_pnts_matches_num << std::endl;
		for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			std::cerr << "lines i: " << i << std::endl;
			int src_cam_id = scenes_lines_pnts_matches_[i].view_1_id;
			int tgt_cam_id = scenes_lines_pnts_matches_[i].view_2_id;
			
			//check the line if correspond...
			CameraModel src_cam_model = scenes_cams_->GetCameraModelFromCamId(src_cam_id);
			CameraModel tgt_cam_model = scenes_cams_->GetCameraModelFromCamId(tgt_cam_id);
			//Eigen::Matrix3f FMatrix = matched_fundamentals_[src_cam_id][tgt_cam_id];
			
			for (int j = 0; j < scenes_lines_pnts_matches_[i].lines_matches.size(); ++j)
			{
				std::pair<Eigen::Vector2f, Eigen::Vector2f> src_line_pnts;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> tgt_line_pnts;
				src_line_pnts.first = scenes_lines_pnts_matches_[i].lines_matches[j].image_p1_;
				src_line_pnts.second = scenes_lines_pnts_matches_[i].lines_matches[j].image_p2_;
				tgt_line_pnts.first = scenes_lines_pnts_matches_[i].lines_matches[j].image_q1_;
				tgt_line_pnts.second = scenes_lines_pnts_matches_[i].lines_matches[j].image_q2_;
				Eigen::Vector3f src_pnt0_3d, src_pnt1_3d;
				int src_polygon_idx0 = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(src_cam_model, src_line_pnts.first, src_pnt0_3d);
				int src_polygon_idx1 = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(src_cam_model, src_line_pnts.second, src_pnt1_3d);

				if (src_polygon_idx0 != -1 && src_polygon_idx1 != -1)
				{
					//to do next...
					Eigen::Vector2f tgt_prj_pnt0 = tgt_cam_model.WorldPnt3d2ImgPnt(src_pnt0_3d);
					Eigen::Vector2f tgt_prj_pnt1 = tgt_cam_model.WorldPnt3d2ImgPnt(src_pnt1_3d);
					//check the line 2d with original 2d
					std::pair<Eigen::Vector2f, Eigen::Vector2f> tgt_prj_line;
					tgt_prj_line.first = tgt_prj_pnt0;
					tgt_prj_line.second = tgt_prj_pnt1;
					float angle_threshold = lines_match_angle_threshold_;
					float dist2line_threshold = lines_match_dist2line_threshold_;
					bool corresponding_line_flag = CheckLineSeg2d2LineSeg2dWithThreshold(tgt_line_pnts,
						tgt_prj_line, angle_threshold, dist2line_threshold);
					if (corresponding_line_flag)
					{
						scenes_lines_pnts_matches_[i].lines_matches[j].valid_match_ = true;
					}
					else
					{
						scenes_lines_pnts_matches_[i].lines_matches[j].valid_match_ = false;
					}
				}
				else
				{
					scenes_lines_pnts_matches_[i].lines_matches[j].valid_match_ = false;
				}
			}
		}

		if (use_images_lines_pairs_imported_)
		{
			for (int i = 0; i < images_lines_pairs_imported_.size(); ++i)
			{
				std::cerr << "lines pairs i: " << i << std::endl;
				int src_cam_id = images_lines_pairs_imported_[i].camid_to_camid_.first;
				int tgt_cam_id = images_lines_pairs_imported_[i].camid_to_camid_.second;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> src_line_pnts
					= images_lines_pairs_imported_[i].line1;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> tgt_line_pnts
					= images_lines_pairs_imported_[i].line2;
				//check the line if correspond...
				CameraModel src_cam_model = scenes_cams_->GetCameraModelFromCamId(src_cam_id);
				CameraModel tgt_cam_model = scenes_cams_->GetCameraModelFromCamId(tgt_cam_id);
				//Eigen::Matrix3f FMatrix = matched_fundamentals_[src_cam_id][tgt_cam_id];
				Eigen::Vector3f src_pnt0_3d, src_pnt1_3d;
				int src_polygon_idx0 = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(src_cam_model, src_line_pnts.first, src_pnt0_3d);
				int src_polygon_idx1 = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(src_cam_model, src_line_pnts.second, src_pnt1_3d);
				/*Eigen::Vector3f tgt_pnt0_3d, tgt_pnt1_3d;
				int tgt_polygon_idx0 = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tgt_cam_model, tgt_line_pnts.first, tgt_pnt0_3d);
				int tgt_polygon_idx1 = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tgt_cam_model, tgt_line_pnts.second, tgt_pnt1_3d);*/
				if (src_polygon_idx0 != -1 && src_polygon_idx1 != -1)
				{
					//to do next...
					Eigen::Vector2f tgt_prj_pnt0 = tgt_cam_model.WorldPnt3d2ImgPnt(src_pnt0_3d);
					Eigen::Vector2f tgt_prj_pnt1 = tgt_cam_model.WorldPnt3d2ImgPnt(src_pnt1_3d);
					//check the line 2d with original 2d
					std::pair<Eigen::Vector2f, Eigen::Vector2f> tgt_prj_line;
					tgt_prj_line.first = tgt_prj_pnt0;
					tgt_prj_line.second = tgt_prj_pnt1;
					float angle_threshold = 8.0;
					float dist2line_threshold = 10.0;
					bool corresponding_line_flag = CheckLineSeg2d2LineSeg2dWithThreshold(tgt_line_pnts,
						tgt_prj_line, angle_threshold, dist2line_threshold);
					if (corresponding_line_flag)
					{
						images_lines_pairs_imported_[i].lines_valid_ = true;
					}
					else
					{
						images_lines_pairs_imported_[i].lines_valid_ = false;
					}
				}
				else
				{
					images_lines_pairs_imported_[i].lines_valid_ = false;
				}
			}
		}
	}

	void HWScenesElements::FilterErrorPointsMatchesByPolygonsNetNumpy()
	{
		if (associated_polygons_.empty())
		{
			std::cerr << "no scene polygons existed..." << std::endl;
			return;
		}

		for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			std::cerr << "pnts matches->i: " << i << std::endl;
			int src_cam_id = scenes_lines_pnts_matches_[i].view_1_id;
			int tgt_cam_id = scenes_lines_pnts_matches_[i].view_2_id;

			//check the line if correspond...
			CameraModel src_cam_model = scenes_cams_->GetCameraModelFromCamId(src_cam_id);
			CameraModel tgt_cam_model = scenes_cams_->GetCameraModelFromCamId(tgt_cam_id);
			//Eigen::Matrix3f FMatrix = matched_fundamentals_[src_cam_id][tgt_cam_id];

			for (int j = 0; j < scenes_lines_pnts_matches_[i].points_matches.size(); ++j)
			{
				Eigen::Vector2f src_pnt = scenes_lines_pnts_matches_[i].points_matches[j].corresponding_pnts_.p1;
				Eigen::Vector2f tgt_pnt = scenes_lines_pnts_matches_[i].points_matches[j].corresponding_pnts_.p2;
				Eigen::Vector3f src_pnt_3d;
				int src_polygon_idx0 = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(src_cam_model, src_pnt, src_pnt_3d);
				if (src_polygon_idx0 != -1)
				{
					Eigen::Vector2f tgt_prj_pnt = tgt_cam_model.WorldPnt3d2ImgPnt(src_pnt_3d);
					float pnt2pnt_threshold = pnts_match_pnt2pnt_threshold_;
					bool corresponding_pnt_flag = CheckTwoPnt2dTheshold(tgt_pnt, tgt_prj_pnt, pnt2pnt_threshold);
					if (corresponding_pnt_flag)
					{
						scenes_lines_pnts_matches_[i].points_matches[j].valid_match_ = true;
					}
					else
					{
						scenes_lines_pnts_matches_[i].points_matches[j].valid_match_ = false;
					}
				}
				else
				{
					scenes_lines_pnts_matches_[i].points_matches[j].valid_match_ = false;
				}
			}
		}

		if (use_images_pnts_pairs_imported_)
		{
			for (int i = 0; i < images_pnts_pairs_imported_.size(); ++i)
			{
				std::cerr << "pnts pairs->i: " << i << std::endl;
				int src_cam_id = images_pnts_pairs_imported_[i].camid_to_camid_.first;
				int tgt_cam_id = images_pnts_pairs_imported_[i].camid_to_camid_.second;
				Eigen::Vector2f src_pnt = images_pnts_pairs_imported_[i].pnt1;
				Eigen::Vector2f tgt_pnt = images_pnts_pairs_imported_[i].pnt2;
				//check the line if correspond...
				CameraModel src_cam_model = scenes_cams_->GetCameraModelFromCamId(src_cam_id);
				CameraModel tgt_cam_model = scenes_cams_->GetCameraModelFromCamId(tgt_cam_id);
				//Eigen::Matrix3f FMatrix = matched_fundamentals_[src_cam_id][tgt_cam_id];
				Eigen::Vector3f src_pnt_3d;
				int src_polygon_idx0 = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(src_cam_model, src_pnt, src_pnt_3d);
				if (src_polygon_idx0 != -1)
				{
					Eigen::Vector2f tgt_prj_pnt = tgt_cam_model.WorldPnt3d2ImgPnt(src_pnt_3d);
					float dist2line_threshold = 10.0;
					bool corresponding_pnt_flag = CheckTwoPnt2dTheshold(tgt_pnt, tgt_prj_pnt, dist2line_threshold);
					if (corresponding_pnt_flag)
					{
						images_pnts_pairs_imported_[i].points_valid_ = true;
					}
					else
					{
						images_pnts_pairs_imported_[i].points_valid_ = false;
					}
				}
				else
				{
					images_pnts_pairs_imported_[i].points_valid_ = false;
				}
			}
		}
	}

	void HWScenesElements::RunConvertNumpyPntsAndLinesIntoViews()
	{
		std::cerr << "to do next(start to convert pnts and lines pairs)..." << std::endl;
		std::map<unsigned int, std::set<unsigned int>>::iterator it;
		for (it = matched_.begin(); it != matched_.end(); ++it)
		{
			unsigned int src_cam_id = it->first;
			std::set<unsigned int> neigbor_cams = it->second;
			std::set<unsigned int>::iterator jt;
			//std::map<unsigned int, Eigen::Matrix3d> cams_fund;
			for (jt = neigbor_cams.begin(); jt != neigbor_cams.end(); ++jt)
			{
				unsigned int tgt_cam_id = *jt;
				std::cerr << "src cam id and tgt cam id: " << src_cam_id << ", " << tgt_cam_id << std::endl;
				if (tgt_cam_id == src_cam_id)
				{
					//same cam id
					std::cerr << "same src camid and tgt camid" << std::endl;
					continue;
				}
				//if the pair view has already in scenes_lines_pnts_matches_, skip!!
				if (FindViewPairCamsIdsFromViewsLinePntsMatches(src_cam_id, tgt_cam_id) != -1)
				{
					continue;
				}
				HWTwoViewsCamsMatching view2match;
				view2match.view_1_id = src_cam_id;
				view2match.view_2_id = tgt_cam_id;
				std::vector<HWLinesPntsMatch> lines_pnts_match;
				//matches_[src_cam_id];
				std::vector<HWMatch> src_tgt_match = GetHWMatchFromMatches(src_cam_id, tgt_cam_id);

				////test the line match
				//std::cerr << "---------------------------------" << std::endl;
				//for (int i = 0; i < src_tgt_match.size(); ++i)
				//{
				//	std::cerr << "src_id, src_pnt_idx: " << src_tgt_match[i].src_camID_ << ", " << src_tgt_match[i].src_segID_ << " --> " <<
				//		"tgt_id, tgt_pnt_idx: " << src_tgt_match[i].tgt_camID_ << ", " << src_tgt_match[i].tgt_segID_ << std::endl;
				//}
				//std::cerr << "---------------------------------" << std::endl;
				////end the line match

				for (int i = 0; i < src_tgt_match.size(); ++i)
				{
					HWLinesPntsMatch line_pnt_match;
					std::pair<Eigen::Vector2f, Eigen::Vector2f> src_line_pnt, tgt_line_pnt;
					src_line_pnt = GetLayoutLineFromCamIdAndLid(src_tgt_match[i].src_camID_, src_tgt_match[i].src_segID_);
					tgt_line_pnt = GetLayoutLineFromCamIdAndLid(src_tgt_match[i].tgt_camID_, src_tgt_match[i].tgt_segID_);
					line_pnt_match.line_matches_idx_.first = src_tgt_match[i].src_segID_;
					line_pnt_match.line_matches_idx_.second = src_tgt_match[i].tgt_segID_;
					line_pnt_match.adj_poly_idxs_ = src_tgt_match[i].adj_poly_idxs_;
					line_pnt_match.image_p1_ = src_line_pnt.first;
					line_pnt_match.image_p2_ = src_line_pnt.second;
					line_pnt_match.image_q1_ = tgt_line_pnt.first;
					line_pnt_match.image_q2_ = tgt_line_pnt.second;
					line_pnt_match.valid_match_ = true;
					line_pnt_match.world_p1_ = src_tgt_match[i].world_p1_;
					line_pnt_match.world_p2_ = src_tgt_match[i].world_p2_;
					line_pnt_match.world_q1_ = src_tgt_match[i].world_q1_;
					line_pnt_match.world_q2_ = src_tgt_match[i].world_q2_;
					lines_pnts_match.emplace_back(line_pnt_match);
				}
				view2match.lines_matches = lines_pnts_match;
				scenes_lines_pnts_matches_.emplace_back(view2match);
			}
		}
	}

	void HWScenesElements::AssignAllRenderPlanesLabelsToHWCamsViewsListLines2d()
	{
		if (associated_polygons_.empty())
		{
			std::cerr << "no scene polygons existed..." << std::endl;
			return;
		}
		if (!images_labels_loaded_)
		{
			std::cerr << "no images labels loaded..." << std::endl;
			return;
		}
		if (hw_cams_views_list_.empty())
		{
			std::cerr << "no images hw_cams_views_list_ loaded..." << std::endl;
			return;
		}
		//std::cerr << "to do next..." << std::endl;
		//how to do it?
		for (int i = 0; i < hw_cams_views_list_.size(); ++i)
		{
			int view_id = hw_cams_views_list_[i].hw_camera_id_;
			std::cerr << "to do next..." << std::endl;
		}

	}

	void HWScenesElements::AssignAllRenderPlanesLabelsToHWPairwisesLines2d()
	{
		if (associated_polygons_.empty())
		{
			std::cerr << "no scene polygons existed..." << std::endl;
			return;
		}
		if (!images_labels_loaded_)
		{
			std::cerr << "no images labels loaded..." << std::endl;
			return;
		}
		const std::vector<cv::Mat> images_labels = associsated_images_labels_->GetImagesLabels();
		const std::vector<cv::Mat> images_labels_intersection = 
			associsated_images_labels_->GetImagesLabelsIntersactionRect();
		for (int idx = 0; idx < scenes_lines_pnts_matches_.size(); ++idx)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			//get images pairs matches
			int srccamid = tmp_pair_views.view_1_id;
			int tgtcamid = tmp_pair_views.view_2_id;
			std::cerr << "AssignAllRenderPlanesLabelsToHWPairwisesLines2d->srccamid, tgt_hw_camview_idx: "
				<< srccamid << ", " << tgtcamid << std::endl;
			int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
			int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
			HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
			HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
			std::string src_path = srcimg.GetImagePath();
			std::string tgt_path = tgtimg.GetImagePath();

			/*int src_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(srccamid);
			int tgt_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(tgtcamid);
			std::cerr << "src_hw_camview_idx, tgt_hw_camview_idx: " << src_hw_camview_idx << ", " << tgt_hw_camview_idx << std::endl;
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > src_lines_seg
				= hw_cams_views_list_[src_hw_camview_idx].lines_segments_;
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > tgt_lines_seg
				= hw_cams_views_list_[tgt_hw_camview_idx].lines_segments_;*/

			//get image labels data from associsated_images_labels_
			const std::vector<std::string> images_labels_paths = associsated_images_labels_->GetImagesPaths();
			int src_label_idx = GetImageLabelIdxFromImagesLabelPthByHWImagePath(images_labels_paths, src_path);
			int tgt_label_idx = GetImageLabelIdxFromImagesLabelPthByHWImagePath(images_labels_paths, tgt_path);
			if (src_label_idx == -1 || tgt_label_idx == -1)
			{
				return;
			}
			//get label images
			//cv::Mat src_label_image = cv::imread(images_labels_paths[src_label_idx]);
			//cv::Mat tgt_label_image = cv::imread(images_labels_paths[tgt_label_idx]);
			cv::Mat src_label_image = images_labels[src_label_idx];
			cv::Mat src_label_image_intersaction = images_labels_intersection[src_label_idx];
			cv::Mat tgt_label_image = images_labels[tgt_label_idx];
			cv::Mat tgt_label_image_intersaction = images_labels_intersection[tgt_label_idx];

			//src lines vector
			for (int i = 0; i < tmp_pair_views.lines_matches.size(); ++i)
			{
				std::pair<Eigen::Vector2f, Eigen::Vector2f> src_tmp_line_seg;
				src_tmp_line_seg.first = tmp_pair_views.lines_matches[i].image_p1_;
				src_tmp_line_seg.second = tmp_pair_views.lines_matches[i].image_p2_;
				Eigen::Vector2i src_tmp_line_labels;
				AssignRenderPlanesLabelsToSingleLine2d(src_tmp_line_seg, src_label_image, src_label_image_intersaction, src_tmp_line_labels);
				scenes_lines_pnts_matches_[idx].lines_matches[i].p_polygon_idx_ = src_tmp_line_labels;

				std::pair<Eigen::Vector2f, Eigen::Vector2f> tgt_tmp_line_seg;
				tgt_tmp_line_seg.first = tmp_pair_views.lines_matches[i].image_p1_;
				tgt_tmp_line_seg.second = tmp_pair_views.lines_matches[i].image_p2_;
				Eigen::Vector2i tgt_tmp_line_labels;
				AssignRenderPlanesLabelsToSingleLine2d(tgt_tmp_line_seg, tgt_label_image, tgt_label_image_intersaction, tgt_tmp_line_labels);
				scenes_lines_pnts_matches_[idx].lines_matches[i].q_polygon_idx_ = tgt_tmp_line_labels;
			}
		}
		
	}

	void HWScenesElements::AssignAllRenderPlanesLabelsToHWPairwisesPnts2d()
	{
		if (associated_polygons_.empty())
		{
			std::cerr << "no scene polygons existed..." << std::endl;
			return;
		}
		if (!images_labels_loaded_)
		{
			std::cerr << "no images labels loaded..." << std::endl;
			return;
		}
		const std::vector<cv::Mat> images_labels = associsated_images_labels_->GetImagesLabels();
		const std::vector<cv::Mat> images_labels_intersection =
			associsated_images_labels_->GetImagesLabelsIntersactionRect();
		for (int idx = 0; idx < scenes_lines_pnts_matches_.size(); ++idx)
		{
			HWTwoViewsCamsMatching tmp_pair_views = scenes_lines_pnts_matches_[idx];
			//get images pairs matches
			int srccamid = tmp_pair_views.view_1_id;
			int tgtcamid = tmp_pair_views.view_2_id;
			std::cerr << "AssignAllRenderPlanesLabelsToHWPairwisesPnts2d->srccamid, tgt_hw_camview_idx: "
				<< srccamid << ", " << tgtcamid << std::endl;
			int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
			int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
			HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
			HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
			std::string src_path = srcimg.GetImagePath();
			std::string tgt_path = tgtimg.GetImagePath();

			/*int src_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(srccamid);
			int tgt_hw_camview_idx = GetViewIdxFromHWCamsViewportListByCamId(tgtcamid);
			std::cerr << "src_hw_camview_idx, tgt_hw_camview_idx: " << src_hw_camview_idx << ", " << tgt_hw_camview_idx << std::endl;
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > src_lines_seg
			= hw_cams_views_list_[src_hw_camview_idx].lines_segments_;
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > tgt_lines_seg
			= hw_cams_views_list_[tgt_hw_camview_idx].lines_segments_;*/

			//get image labels data from associsated_images_labels_
			const std::vector<std::string> images_labels_paths = associsated_images_labels_->GetImagesPaths();
			int src_label_idx = GetImageLabelIdxFromImagesLabelPthByHWImagePath(images_labels_paths, src_path);
			int tgt_label_idx = GetImageLabelIdxFromImagesLabelPthByHWImagePath(images_labels_paths, tgt_path);
			if (src_label_idx == -1 || tgt_label_idx == -1)
			{
				return;
			}

			//get label images
			cv::Mat src_label_image = images_labels[src_label_idx];
			cv::Mat src_label_image_intersaction = images_labels_intersection[src_label_idx];
			cv::Mat tgt_label_image = images_labels[tgt_label_idx];
			cv::Mat tgt_label_image_intersaction = images_labels_intersection[tgt_label_idx];

			for (int i = 0; i < tmp_pair_views.points_matches.size(); ++i)
			{
				Eigen::Vector2f src_tmp_pnt;
				src_tmp_pnt = tmp_pair_views.points_matches[i].corresponding_pnts_.p1;
				Eigen::Vector2i src_tmp_pnt_labels;
				AssignRenderPlanesLabelsToSinglePnt2d(src_tmp_pnt, src_label_image, src_label_image_intersaction, src_tmp_pnt_labels);
				scenes_lines_pnts_matches_[idx].points_matches[i].left_polygon_idx_ = src_tmp_pnt_labels;

				Eigen::Vector2f tgt_tmp_pnt;
				tgt_tmp_pnt = tmp_pair_views.points_matches[i].corresponding_pnts_.p2;
				Eigen::Vector2i tgt_tmp_pnt_labels;
				AssignRenderPlanesLabelsToSinglePnt2d(tgt_tmp_pnt, tgt_label_image, tgt_label_image_intersaction, tgt_tmp_pnt_labels);
				scenes_lines_pnts_matches_[idx].points_matches[i].right_polygon_idx_ = tgt_tmp_pnt_labels;
			}
		}
	}

	void HWScenesElements::AssignRenderPlanesLabelsToSingleLine2d(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_sg,
		const cv::Mat& image_labes, const cv::Mat& image_intersection_labels, Eigen::Vector2i& label_assigned)
	{
		int sample_num_from_line = 5;
		std::vector<Eigen::Vector2i> tmp_line_sample_pnts;
		std::vector<unsigned short int>  tmp_line_sample_pnts_labels;
		SampleNLinePntsFromLineSeg(line_sg, sample_num_from_line, tmp_line_sample_pnts);
		bool src_in_polygon_intersection;
		for (int i = 0; i < tmp_line_sample_pnts.size(); ++i)
		{
			Eigen::Vector2i cur_pnt = tmp_line_sample_pnts[i];
			unsigned short int cur_label = image_labes.at<unsigned short int>(cur_pnt[1], cur_pnt[0]);
			
			cv::Vec3b cur_pnt_intersaction = image_intersection_labels.at<cv::Vec3b>(cur_pnt[1], cur_pnt[0]);
			if (cur_pnt_intersaction == cv::Vec3b(255, 255, 255))
			{
				tmp_line_sample_pnts_labels.emplace_back(KMAX_USHORT_VALUE);
			}
			else
			{
				tmp_line_sample_pnts_labels.emplace_back(cur_label);
			}
		}
		//extract labels from tmp_line_sample_pnts_labels
		int line_extracted_label = ExtractPlaneLabelFromLabelsVec(tmp_line_sample_pnts_labels);
		if (label_assigned[0] == KMAX_USHORT_VALUE)
		{
			label_assigned[0] = -1;
		}
		else
		{
			label_assigned[0] = line_extracted_label;
		}
		label_assigned[1] = -1;
	}

	void HWScenesElements::AssignRenderPlanesLabelsToSinglePnt2d(const Eigen::Vector2f& single_point,
		const cv::Mat& image_labes, const cv::Mat& image_intersection_labels, Eigen::Vector2i& label_assigned)
	{
		Eigen::Vector2i cur_pnt = Eigen::Vector2i(single_point[0], single_point[1]);
		unsigned short int cur_label = image_labes.at<unsigned short int>(cur_pnt[1], cur_pnt[0]);
		cv::Vec3b cur_pnt_intersaction = image_intersection_labels.at<cv::Vec3b>(cur_pnt[1], cur_pnt[0]);
		if (cur_pnt_intersaction == cv::Vec3b(255, 255, 255))
		{
			//label_assigned[0] = -1;
			label_assigned[0] = static_cast<int>(cur_label);
			label_assigned[1] = -1;
		}
		else
		{
			label_assigned[0] = static_cast<int>(cur_label);
			label_assigned[1] = -1;
		}
	}

	void HWScenesElements::SampleNLinePntsFromLineSeg(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_seg,
		int sample_num, std::vector<Eigen::Vector2i>& line_pnts)
	{
		if (sample_num == 0 || sample_num == 1)
		{
			Eigen::Vector2f lmean_pnt = (line_seg.second + line_seg.first) / 2.0;
			line_pnts.emplace_back(Eigen::Vector2i(lmean_pnt[0], lmean_pnt[1]));
		}
		else
		{
			Eigen::Vector2f ld = (line_seg.second - line_seg.first).normalized();
			float line_length = (line_seg.second - line_seg.first).norm();
			float line_step = line_length / (sample_num - 1);
			for (int i = 0; i < sample_num; ++i)
			{
				Eigen::Vector2f line_pnt = line_seg.first + (i * line_step)*ld;
				line_pnts.emplace_back(Eigen::Vector2i(line_pnt[0], line_pnt[1]));
			}
		}
	}

	void HWScenesElements::SampleNLineFloatPntsFromLineSeg(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_seg,
		int sample_num, std::vector<Eigen::Vector2f>& line_pnts)
	{
		if (sample_num == 0 || sample_num == 1)
		{
			Eigen::Vector2f lmean_pnt = (line_seg.second + line_seg.first) / 2.0;
			line_pnts.emplace_back(lmean_pnt);
		}
		else
		{
			Eigen::Vector2f ld = (line_seg.second - line_seg.first).normalized();
			float line_length = (line_seg.second - line_seg.first).norm();
			float line_step = line_length / (sample_num - 1);
			for (int i = 0; i < sample_num; ++i)
			{
				Eigen::Vector2f line_pnt = line_seg.first + (i * line_step)*ld;
				line_pnts.emplace_back(line_pnt);
			}
		}
	}

	void HWScenesElements::ComputeImagesPntsRayToPolygonsIdxs(std::vector<Eigen::Vector2f>& image_pnts,
		const CameraModel& cam_model, std::vector<std::pair<int, int> >& polygons_idxs_counts)
	{
		polygons_idxs_counts.clear();
		std::map<int, int> polygonsidxs2counts;
		for (int i = 0; i < image_pnts.size(); ++i)
		{
			Eigen::Vector2f tmp_pnt = image_pnts[i];
			Eigen::Vector3f tmp_pnt3d;
			int idx_sray_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(cam_model, tmp_pnt, tmp_pnt3d);
			std::map<int, int>::iterator idx_iter = polygonsidxs2counts.find(idx_sray_intersect);
			if (idx_iter == polygonsidxs2counts.end())
			{
				polygonsidxs2counts[idx_sray_intersect] = 1;
			}
			else
			{
				idx_iter->second += 1;
			}
		}
		std::vector<std::pair<int, int> > polyidxs2countsvec(polygonsidxs2counts.begin(), polygonsidxs2counts.end());
		polygons_idxs_counts = polyidxs2countsvec;
	}

	void HWScenesElements::ExtractPolygonIdxFromPolygonsIdxsCounts(const std::vector<std::pair<int, int> >& polygons_idxs_counts,
		Eigen::Vector2i& polygon_idxs)
	{
		polygon_idxs[0] = -1;
		polygon_idxs[1] = -1;
		//get the polygon idxs
		std::vector<std::pair<int, int> > polyidxs2countsvec(polygons_idxs_counts.begin(), polygons_idxs_counts.end());
		std::sort(polyidxs2countsvec.begin(), polyidxs2countsvec.end(), MyPolygonIdxCmpZDG);
		//scenes_pnts3d_track_list_[i].pos_ = (fusion_pnt3d / view_features_num);
		std::vector<std::pair<int, int> >::iterator iter_poly = polyidxs2countsvec.begin();
		std::vector<int> polygons_idxs_sorted;
		for (; iter_poly != polyidxs2countsvec.end(); ++iter_poly)
		{
			polygons_idxs_sorted.emplace_back(iter_poly->first);
		}
		if (polygons_idxs_sorted.size() == 1)
		{
			polygon_idxs[0] = polygons_idxs_sorted[0];
		}
		else if (polygons_idxs_sorted.empty())
		{
			return;
		}
		else
		{
			polygon_idxs[0] = polygons_idxs_sorted[0];
			polygon_idxs[1] = polygons_idxs_sorted[1];
		}
	}

	int HWScenesElements::ExtractPlaneLabelFromLabelsVec(const std::vector<unsigned short int>& line_pnts_labels)
	{
		std::vector<unsigned short int> tmp_line_pnts_labels = line_pnts_labels;
		int line_pnts_num = static_cast<int>(tmp_line_pnts_labels.size());
		if (line_pnts_num == 0)
		{
			return -1;
		}
		else if (line_pnts_num <= 2 && line_pnts_num > 0)
		{
			int tmp_label = static_cast<int>(tmp_line_pnts_labels[0]);
			return tmp_label;
		}
		std::sort(tmp_line_pnts_labels.begin(), tmp_line_pnts_labels.end(), [](const unsigned short int &a, const unsigned short int &b) { return (a < b); });
		//get the labels
		std::vector<int> labels_level_idx;
		for (int i = 0; i < tmp_line_pnts_labels.size() - 1; ++i)
		{
			int inext = i + 1;
			if (tmp_line_pnts_labels[i] != tmp_line_pnts_labels[inext])
			{
				labels_level_idx.emplace_back(i);
			}
		}
		if (labels_level_idx.empty())
		{
			//all have same label value
			int tmp_label = static_cast<int>(tmp_line_pnts_labels[0]);
			return tmp_label;
		}
		else
		{
			labels_level_idx.emplace_back(tmp_line_pnts_labels.size() - 1);
			int line_pnts_max_num = labels_level_idx[0] + 1;
			int idx_max = labels_level_idx[0];
			for (int i = 0; i < labels_level_idx.size() - 1; ++i)
			{
				int inext = i + 1;
				int tmp_line_pnts_num = labels_level_idx[inext] - labels_level_idx[i];
				if (tmp_line_pnts_num > line_pnts_max_num)
				{
					line_pnts_max_num = tmp_line_pnts_num;
					idx_max = labels_level_idx[inext];
				}
			}
			int tmp_label = static_cast<int>(tmp_line_pnts_labels[idx_max]);
			return tmp_label;
		}
	}

	void HWScenesElements::RunLineMatchesIntoViews()
	{
		std::map<unsigned int, std::set<unsigned int>>::iterator it;
		for (it = matched_.begin(); it != matched_.end(); ++it)
		{
			unsigned int src_cam_id = it->first;
			std::set<unsigned int> neigbor_cams = it->second;
			std::set<unsigned int>::iterator jt;
			//std::map<unsigned int, Eigen::Matrix3d> cams_fund;
			for (jt = neigbor_cams.begin(); jt != neigbor_cams.end(); ++jt)
			{
				unsigned int tgt_cam_id = *jt;
				std::cerr << "src cam id and tgt cam id: " << src_cam_id << ", " << tgt_cam_id << std::endl;
				if (tgt_cam_id == src_cam_id)
				{
					//same cam id
					std::cerr << "same src camid and tgt camid" << std::endl;
					continue;
				}
				//if the pair view has already in scenes_lines_pnts_matches_, skip!!
				if (FindViewPairCamsIdsFromViewsLinePntsMatches(src_cam_id, tgt_cam_id) != -1)
				{
					continue;
				}
				HWTwoViewsCamsMatching view2match;
				view2match.view_1_id = src_cam_id;
				view2match.view_2_id = tgt_cam_id;
				std::vector<HWLinesPntsMatch> lines_pnts_match;
				//matches_[src_cam_id];
				std::vector<HWMatch> src_tgt_match = GetHWMatchFromMatches(src_cam_id, tgt_cam_id);
				
				////test the line match
				//std::cerr << "---------------------------------" << std::endl;
				//for (int i = 0; i < src_tgt_match.size(); ++i)
				//{
				//	std::cerr << "src_id, src_pnt_idx: " << src_tgt_match[i].src_camID_ << ", " << src_tgt_match[i].src_segID_ << " --> " <<
				//		"tgt_id, tgt_pnt_idx: " << src_tgt_match[i].tgt_camID_ << ", " << src_tgt_match[i].tgt_segID_ << std::endl;
				//}
				//std::cerr << "---------------------------------" << std::endl;
				////end the line match

				for (int i = 0; i < src_tgt_match.size(); ++i)
				{
					HWLinesPntsMatch line_pnt_match;
					std::pair<Eigen::Vector2f, Eigen::Vector2f> src_line_pnt, tgt_line_pnt;
					src_line_pnt = GetLayoutLineFromCamIdAndLid(src_tgt_match[i].src_camID_, src_tgt_match[i].src_segID_);
					tgt_line_pnt = GetLayoutLineFromCamIdAndLid(src_tgt_match[i].tgt_camID_, src_tgt_match[i].tgt_segID_);
					line_pnt_match.line_matches_idx_.first = src_tgt_match[i].src_segID_;
					line_pnt_match.line_matches_idx_.second = src_tgt_match[i].tgt_segID_;
					line_pnt_match.adj_poly_idxs_ = src_tgt_match[i].adj_poly_idxs_;
					line_pnt_match.image_p1_ = src_line_pnt.first;
					line_pnt_match.image_p2_ = src_line_pnt.second;
					line_pnt_match.image_q1_ = tgt_line_pnt.first;
					line_pnt_match.image_q2_ = tgt_line_pnt.second;
					line_pnt_match.valid_match_ = true;
					line_pnt_match.world_p1_ = src_tgt_match[i].world_p1_;
					line_pnt_match.world_p2_ = src_tgt_match[i].world_p2_;
					line_pnt_match.world_q1_ = src_tgt_match[i].world_q1_;
					line_pnt_match.world_q2_ = src_tgt_match[i].world_q2_;
					lines_pnts_match.emplace_back(line_pnt_match);
				}
				view2match.lines_matches = lines_pnts_match;
				scenes_lines_pnts_matches_.emplace_back(view2match);
			}
		}
	}

	void HWScenesElements::RunLineMatchesIntoViewsFilterWithLinesMatchNum(int line_match_threshold)
	{
		std::map<unsigned int, std::set<unsigned int>>::iterator it;
		for (it = matched_.begin(); it != matched_.end(); ++it)
		{
			unsigned int src_cam_id = it->first;
			std::set<unsigned int> neigbor_cams = it->second;
			std::set<unsigned int>::iterator jt;
			//std::map<unsigned int, Eigen::Matrix3d> cams_fund;
			for (jt = neigbor_cams.begin(); jt != neigbor_cams.end(); ++jt)
			{
				unsigned int tgt_cam_id = *jt;
				std::cerr << "src cam id and tgt cam id: " << src_cam_id << ", " << tgt_cam_id << std::endl;
				if (tgt_cam_id == src_cam_id)
				{
					//same cam id
					//std::cerr << "111111" << std::endl;
					continue;
				}
				//if the pair view has already in scenes_lines_pnts_matches_, skip!!
				if (FindViewPairCamsIdsFromViewsLinePntsMatches(src_cam_id, tgt_cam_id) != -1)
				{
					continue;
				}
				HWTwoViewsCamsMatching view2match;
				view2match.view_1_id = src_cam_id;
				view2match.view_2_id = tgt_cam_id;
				std::vector<HWLinesPntsMatch> lines_pnts_match;
				//matches_[src_cam_id];
				std::vector<HWMatch> src_tgt_match = GetHWMatchFromMatches(src_cam_id, tgt_cam_id);

				////test the line match
				//std::cerr << "---------------------------------" << std::endl;
				//for (int i = 0; i < src_tgt_match.size(); ++i)
				//{
				//	std::cerr << "src_id, src_pnt_idx: " << src_tgt_match[i].src_camID_ << ", " << src_tgt_match[i].src_segID_ << " --> " <<
				//		"tgt_id, tgt_pnt_idx: " << src_tgt_match[i].tgt_camID_ << ", " << src_tgt_match[i].tgt_segID_ << std::endl;
				//}
				//std::cerr << "---------------------------------" << std::endl;
				////end the line match

				int src_tgt_lines_num = static_cast<int>(src_tgt_match.size());
				if (src_tgt_lines_num < line_match_threshold)
				{
					continue;
				}

				for (int i = 0; i < src_tgt_match.size(); ++i)
				{
					HWLinesPntsMatch line_pnt_match;
					std::pair<Eigen::Vector2f, Eigen::Vector2f> src_line_pnt, tgt_line_pnt;
					src_line_pnt = GetLayoutLineFromCamIdAndLid(src_tgt_match[i].src_camID_, src_tgt_match[i].src_segID_);
					tgt_line_pnt = GetLayoutLineFromCamIdAndLid(src_tgt_match[i].tgt_camID_, src_tgt_match[i].tgt_segID_);
					line_pnt_match.line_matches_idx_.first = src_tgt_match[i].src_segID_;
					line_pnt_match.line_matches_idx_.second = src_tgt_match[i].tgt_segID_;
					line_pnt_match.adj_poly_idxs_ = src_tgt_match[i].adj_poly_idxs_;
					line_pnt_match.image_p1_ = src_line_pnt.first;
					line_pnt_match.image_p2_ = src_line_pnt.second;
					line_pnt_match.image_q1_ = tgt_line_pnt.first;
					line_pnt_match.image_q2_ = tgt_line_pnt.second;
					line_pnt_match.valid_match_ = true;
					line_pnt_match.world_p1_ = src_tgt_match[i].world_p1_;
					line_pnt_match.world_p2_ = src_tgt_match[i].world_p2_;
					line_pnt_match.world_q1_ = src_tgt_match[i].world_q1_;
					line_pnt_match.world_q2_ = src_tgt_match[i].world_q2_;
					lines_pnts_match.emplace_back(line_pnt_match);
				}
				view2match.lines_matches = lines_pnts_match;
				scenes_lines_pnts_matches_.emplace_back(view2match);
			}
		}
	}

	int HWScenesElements::FindViewPairCamsIdsFromViewsLinePntsMatches(int src_view_id, int tgt_view_id)
	{
		for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			int view1_id = scenes_lines_pnts_matches_[i].view_1_id;
			int view2_id = scenes_lines_pnts_matches_[i].view_2_id;
			if (view1_id == src_view_id && view2_id == tgt_view_id)
			{
				return i;
			}
			if (view1_id == tgt_view_id && view2_id == src_view_id)
			{
				return i;
			}
		}
		return -1;
	}

	void HWScenesElements::RunAllImgsSiftDetector()
	{
		const std::vector<CameraModel> cams = scenes_cams_->GetCamerasModels();
		scenes_sift_detectors_.resize(cams.size());
		scenes_sifts_positions_.resize(cams.size());
		for (int i = 0; i < cams.size(); ++i)
		{
			int imgid = cams[i].image_id_;
			scenes_cams_->UpdateImageLoadedFromImgid(imgid);
			HWImage scene_img = scenes_cams_->GetImageFromImgId(imgid);
			std::vector<cv::KeyPoint> scene_keypnts;
			const cv::Mat& const scene_cv_img = scene_img.GetImage();
			cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> scene_feature_detector = cv::xfeatures2d::SiftFeatureDetector::create();
			scene_feature_detector->detect(scene_cv_img, scene_keypnts);
			scenes_sift_detectors_[i] = scene_feature_detector;
			scenes_sifts_positions_[i] = scene_keypnts;
			scenes_cams_->SetImageSiftKeyPointsFromImgid(imgid, scene_keypnts);
			//scenes_sift_detectors_.emplace_back(scene_feature_detector);
			//cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> scene_feature_desc = cv::xfeatures2d::SiftDescriptorExtractor::create();
			//scenes_cams_->UpdateSiftImageFromImgid(imgid);
			//set the scene keypnt to scenes_cams_ (important, to do next...)
		}
	}

	void HWScenesElements::RunAllLinesSegmentsDetector()
	{
		const std::vector<CameraModel> cams = scenes_cams_->GetCamerasModels();
		for (int i = 0; i < cams.size(); ++i)
		{
			int imgid = cams[i].image_id_;
			//if()
			//scenes_cams_->UpdateImageLoadedFromImgid(imgid);
			HWImage scene_img = scenes_cams_->GetImageFromImgId(imgid);
			if (!scene_img.ImageLoaded())
			{
				scenes_cams_->UpdateImageLoadedFromImgid(imgid);
			}
			scenes_cams_->UpdateLinesSegsDetectedFromImgid(imgid);
		}
	}
	
	void HWScenesElements::RunConvertAllImagesLinesSegmentsToLayouts()
	{
		const std::vector<CameraModel> cams = scenes_cams_->GetCamerasModels();
		std::map<int, std::vector<std::vector<float> > > lines_segs;
		if (scenes_layouts_elements_.get())
		{
			for (int i = 0; i < cams.size(); ++i)
			{
				int imgid = cams[i].image_id_;
				//if()
				//scenes_cams_->UpdateImageLoadedFromImgid(imgid);
				HWImage scene_img = scenes_cams_->GetImageFromImgId(imgid);
				int layout_id = scene_img.GetLayoutId();
				std::vector<std::vector<float>> ly_lines;
				std::vector<cv::Vec4f> ls_segs = scene_img.GetLinesSegmentsOpencv();
				for (int j = 0; j < ls_segs.size(); ++j)
				{
					std::vector<float> ly_line;
					ly_line.emplace_back(ls_segs[j][0]);
					ly_line.emplace_back(ls_segs[j][1]);
					ly_line.emplace_back(ls_segs[j][2]);
					ly_line.emplace_back(ls_segs[j][3]);
					ly_lines.emplace_back(ly_line);
				}
				lines_segs[layout_id] = ly_lines;
				/*if (!scene_img.ImageLoaded())
				{
				scenes_cams_->UpdateImageLoadedFromImgid(imgid);
				}
				scenes_cams_->UpdateLinesSegsDetectedFromImgid(imgid);*/
			}
			scenes_layouts_elements_.get()->SetLayoutsFromImgsLinesSegsOpencv(lines_segs);
		}
		else
		{
			std::cerr << "to generate the empty scenes layouts for next operation..." << std::endl;
			std::cerr << "to do next..." << std::endl;
		}
	}

	void HWScenesElements::RunAllLayoutsSegmentsCombinations()
	{
		if (scenes_layouts_elements_.get())
		{
			scenes_layouts_elements_.get()->UpdateAllLayoutsLinesCombinationProcess();
		}
	}

	void HWScenesElements::RunImgsPntsMatchesFromLineImgsMatch()
	{
		if (!filter_wrong_lines_match_)
		{
			return;
		}
		for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			unsigned int src_cam_id = scenes_lines_pnts_matches_[i].view_1_id;
			unsigned int tgt_cam_id = scenes_lines_pnts_matches_[i].view_2_id;
			int src_image_id = scenes_cams_->GetImageidFromCamId(src_cam_id);
			int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgt_cam_id);
			const CameraModel src_cam = scenes_cams_->GetCameraModelFromCamId(src_cam_id);
			const CameraModel tgt_cam = scenes_cams_->GetCameraModelFromCamId(tgt_cam_id);
			//get image to do next...
			HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
			HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
			//std::cerr << "asdffasdfasf " << std::endl;
			//std::vector<cv::KeyPoint> src_keypnts = srcimg.GetSiftFeatruesPosition();
			//cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> src_feature_desc = srcimg.GetSiftDescriptorPtr();
			cv::Mat src_cv_img = srcimg.GetImage();
			//std::vector<cv::KeyPoint> tgt_keypnts = tgtimg.GetSiftFeatruesPosition();
			//cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> tgt_feature_desc = tgtimg.GetSiftDescriptorPtr();
			cv::Mat tgt_cv_img = tgtimg.GetImage();
			//std::cerr << "2222222222" << std::endl;

			/*std::string src_cv_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/111111.png";
			std::string tgt_cv_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/222222.png";
			cv::imwrite(src_cv_path, src_cv_img);
			cv::imwrite(tgt_cv_path, tgt_cv_img);*/

			//灰度图转换  
			//cv::Mat src_gray_image, tgt_gray_image;
			//cv::cvtColor(src_cv_img, src_gray_image, CV_RGB2GRAY);
			//cv::cvtColor(tgt_cv_img, tgt_gray_image, CV_RGB2GRAY);

			std::vector<cv::KeyPoint> src_keypnts;
			std::vector<cv::KeyPoint> tgt_keypnts;

			cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> src_feature_detector = cv::xfeatures2d::SiftFeatureDetector::create();
			cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> tgt_feature_detector = cv::xfeatures2d::SiftFeatureDetector::create();
			src_feature_detector->detect(src_cv_img, src_keypnts);
			tgt_feature_detector->detect(tgt_cv_img, tgt_keypnts);

			cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> src_feature_desc1 = cv::xfeatures2d::SiftDescriptorExtractor::create();
			cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> tgt_feature_desc1 = cv::xfeatures2d::SiftDescriptorExtractor::create();
			
			cv::Mat src_descriptor_image_, tgt_descriptor_image_;
			src_feature_desc1->compute(src_cv_img, src_keypnts, src_descriptor_image_);
			tgt_feature_desc1->compute(tgt_cv_img, tgt_keypnts, tgt_descriptor_image_);

			std::cerr << "the src_keypnts number is: " << src_keypnts.size() << std::endl;
			std::cerr << "the tgt_keypnts number is: " << tgt_keypnts.size() << std::endl;
			
			//std::vector<cv::DMatch> matches_cv;
			//使用参数来匹配
			//std::vector<cv::DMatch> GoodMatchePoints;
			//cv::Ptr<cv::DescriptorMatcher> matcher = cv::FlannBasedMatcher::create();
			//matcher->match(src_descriptor_image_, tgt_descriptor_image_, GoodMatchePoints);
			//std::cout << "total match points number: " << GoodMatchePoints.size() << endl;
			//std::cerr << "33333333333" << std::endl;

			//进行FlannBasedMatcher暴力匹配
			cv::Ptr<cv::FlannBasedMatcher> matcher = cv::FlannBasedMatcher::create();
			//定义匹配结果变量
			std::vector<cv::DMatch> matches;
			//实现描述符之间的匹配
			matcher->match(src_descriptor_image_, tgt_descriptor_image_, matches);
			//std::vector<cv::DMatch> GoodMatchePoints;
			//定义向量距离的最大值与最小值
			//double image_max_dist=0.0;
			double image_min_dist=100000.0;
			for (int j = 1; j < matches.size(); ++j)
			{
				double tmp_dist=matches[j].distance;
				//if (tmp_dist>image_max_dist)
				//	image_max_dist=tmp_dist;
				if (tmp_dist<image_min_dist)
					image_min_dist=tmp_dist;
			}
			std::cout << "min_dist=" << image_min_dist << std::endl;
			//std::cout << "max_dist=" << image_max_dist << std::endl;
			//匹配结果筛选    
			std::vector<cv::DMatch> GoodMatchePoints;
			for (int j= 0; j < matches.size(); ++j)
			{
				double dist = matches[j].distance;
				if (dist < 3 * image_min_dist)
					GoodMatchePoints.push_back(matches[j]);
			}
			std::cout << "goodMatches:" << (int)GoodMatchePoints.size() << std::endl;

			//std::cerr << "the good match:" << matches.size() << std::endl;
			/*cv::FlannBasedMatcher matcher;
			std::vector<std::vector<cv::DMatch> > matchePoints;
			std::vector<cv::DMatch> GoodMatchePoints;
			std::vector<cv::Mat> train_desc(1, src_descriptor_image_);
			matcher.add(train_desc);
			matcher.train();
			matcher.knnMatch(tgt_descriptor_image_, matchePoints, 2);
			std::cout << "total match points number: " << matchePoints.size() << endl;*/

			/*std::cerr << "33333333333" << std::endl;
			return;*/
			// Lowe's algorithm,获取优秀匹配点
			/*for (int i = 0; i < matchePoints.size(); i++)
			{
				if (matchePoints[i][0].distance < 0.6 * matchePoints[i][1].distance)
				{
					GoodMatchePoints.push_back(matchePoints[i][0]);
				}
			}
			std::cout << "total good match points number: " << GoodMatchePoints.size() << endl;*/

			//test
			/*cv::Mat out_matches_img;
			cv::drawMatches(src_cv_img, src_keypnts, tgt_cv_img, tgt_keypnts, GoodMatchePoints, out_matches_img);
			std::string out_matches_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/src2tgt_pnts_match_new.png";
			cv::imwrite(out_matches_path, out_matches_img);*/

			int num_matches = static_cast<int>(GoodMatchePoints.size());
			std::cerr << "the two images matches pnts num is: " << num_matches << std::endl;

			/* Build correspondences from feature matching result. */
			//HWTwoViewsCamsMatching match_views;
			//get line match from matches

			//
			std::vector<cv::DMatch> filter_matches;
			for (std::size_t j = 0; j < GoodMatchePoints.size(); ++j)
			{
				if (GoodMatchePoints[j].queryIdx < 0 || GoodMatchePoints[j].trainIdx < 0)
					continue;
				HWCorrespondence2D2DPnt match_pnt;
				match_pnt.p1[0] = src_keypnts[GoodMatchePoints[j].queryIdx].pt.x;
				match_pnt.p1[1] = src_keypnts[GoodMatchePoints[j].queryIdx].pt.y;
				match_pnt.p2[0] = tgt_keypnts[GoodMatchePoints[j].trainIdx].pt.x;
				match_pnt.p2[1] = tgt_keypnts[GoodMatchePoints[j].trainIdx].pt.y;
				//it is important... to do next...
				int src_pnt_idx = GoodMatchePoints[j].queryIdx;
				int tgt_pnt_idx = GoodMatchePoints[j].trainIdx;

				//check the pnt coorindate 
				if (filter_wrong_lines_match_)
				{
					//get polygon
					Eigen::Vector3f src_pnt3d, tgt_pnt3d;
					bool src_pnt3d_flag = ImagePnt2dProj2ScenePolygonPnt3d(src_cam, match_pnt.p1, src_pnt3d);
					bool tgt_pnt3d_flag = ImagePnt2dProj2ScenePolygonPnt3d(tgt_cam, match_pnt.p2, tgt_pnt3d);
					if (src_pnt3d_flag && tgt_pnt3d_flag)
					{
						//check pnt is close enough
						if (CheckTwoPnt3dTheshold(src_pnt3d, tgt_pnt3d, r_max_threshold_))
						{
							//test
							filter_matches.emplace_back(GoodMatchePoints[j]);
							//end test

							HWPntsMatch cor_pnt;
							cor_pnt.pnts_matches_idx_.first = src_pnt_idx;
							cor_pnt.pnts_matches_idx_.second = tgt_pnt_idx;
							Eigen::Vector3f tri_pnt = (src_pnt3d + tgt_pnt3d) / 2;
							cor_pnt.view12_tri_pnt3d_ = tri_pnt;
							cor_pnt.corresponding_pnts_ = match_pnt;
							scenes_lines_pnts_matches_[i].points_matches.emplace_back(cor_pnt);
						}
					}
				}
				//unfiltered_matches.push_back(match);
				//unfiltered_indices.push_back(std::make_pair(matches_cv[i].queryIdx, matches_cv[i].trainIdx));
				//matches->push_back(unfiltered_indices[i]);
			}

			/*if (src_cam_id == 1 && tgt_cam_id == 2)
			{
				std::cerr << "asdfasdfasdfasf " << std::endl;
				int num_filter_matches = static_cast<int>(filter_matches.size());
				std::cerr << "draw the two images matches pnts num is: " << num_filter_matches << std::endl;
				cv::Mat out_filter_matches_img;
				cv::drawMatches(src_cv_img, src_keypnts, tgt_cv_img, tgt_keypnts, filter_matches, out_filter_matches_img);
				std::string out_filter_matches_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/src2tgt_pnts_match_filter_12.png";
				cv::imwrite(out_filter_matches_path, out_filter_matches_img);
			}*/

			/*int num_filter_matches = static_cast<int>(filter_matches.size());
			std::cerr << "the two images matches pnts num is: " << num_filter_matches << std::endl;
			cv::Mat out_filter_matches_img;
			cv::drawMatches(src_cv_img, src_keypnts, tgt_cv_img, tgt_keypnts, filter_matches, out_filter_matches_img);
			std::string out_filter_matches_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/src2tgt_pnts_match_filter_12.png";
			cv::imwrite(out_filter_matches_path, out_filter_matches_img);*/
		}
		std::cerr << "end complete the filter image..." << std::endl;
		//return;
		//std::map<unsigned int, std::set<unsigned int>>::iterator it;
		//for (it = matched_.begin(); it != matched_.end(); ++it)
		//{
		//	//get two IDX
		//	unsigned int src_cam_id = it->first;
		//	std::set<unsigned int> neigbor_cams = it->second;
		//	std::set<unsigned int>::iterator jt;
		//	//std::map<unsigned int, Eigen::Matrix3d> cams_fund;
		//	for (jt = neigbor_cams.begin(); jt != neigbor_cams.end(); ++jt)
		//	{
		//		unsigned int tgt_cam_id = *jt;
		//		RunImgsPntsMatchesFromTwoImgsMatch(src_cam_id, tgt_cam_id);
		//	}
		//}
	}

	void HWScenesElements::RunImgsPntsMatchesFromLineImgsMatchNew()
	{
		if (!filter_wrong_lines_match_)
		{
			return;
		}
		for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			unsigned int src_cam_id = scenes_lines_pnts_matches_[i].view_1_id;
			unsigned int tgt_cam_id = scenes_lines_pnts_matches_[i].view_2_id;
			unsigned int src_cam_idx = scenes_cams_->GetCamidxFromCamId(src_cam_id);
			unsigned int tgt_cam_idx = scenes_cams_->GetCamidxFromCamId(tgt_cam_id);
			cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> src_detector = scenes_sift_detectors_[src_cam_idx];
			cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> tgt_detector = scenes_sift_detectors_[tgt_cam_idx];
			std::vector<cv::KeyPoint> src_keypnts = scenes_sifts_positions_[src_cam_idx];
			std::vector<cv::KeyPoint> tgt_keypnts = scenes_sifts_positions_[tgt_cam_idx];
			int src_image_id = scenes_cams_->GetImageidFromCamId(src_cam_id);
			int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgt_cam_id);
			const CameraModel src_cam = scenes_cams_->GetCameraModelFromCamId(src_cam_id);
			const CameraModel tgt_cam = scenes_cams_->GetCameraModelFromCamId(tgt_cam_id);
			//get image to do next...
			HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
			HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);
			cv::Mat src_cv_img = srcimg.GetImage();
			cv::Mat tgt_cv_img = tgtimg.GetImage();
			cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> src_feature_desc1 = cv::xfeatures2d::SiftDescriptorExtractor::create();
			cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> tgt_feature_desc1 = cv::xfeatures2d::SiftDescriptorExtractor::create();

			cv::Mat src_descriptor_image_, tgt_descriptor_image_;
			src_feature_desc1->compute(src_cv_img, src_keypnts, src_descriptor_image_);
			tgt_feature_desc1->compute(tgt_cv_img, tgt_keypnts, tgt_descriptor_image_);
			std::cerr << "the src_keypnts number is: " << src_keypnts.size() << std::endl;
			std::cerr << "the tgt_keypnts number is: " << tgt_keypnts.size() << std::endl;

			//进行FlannBasedMatcher暴力匹配
			cv::Ptr<cv::FlannBasedMatcher> matcher = cv::FlannBasedMatcher::create();
			//定义匹配结果变量
			std::vector<cv::DMatch> matches;
			//实现描述符之间的匹配
			matcher->match(src_descriptor_image_, tgt_descriptor_image_, matches);
			//std::vector<cv::DMatch> GoodMatchePoints;
			//定义向量距离的最大值与最小值
			//double image_max_dist=0.0;
			double image_min_dist = 100000.0;
			for (int j = 0; j < matches.size(); ++j)
			{
				double tmp_dist = matches[j].distance;
				//if (tmp_dist>image_max_dist)
				//	image_max_dist=tmp_dist;
				if (tmp_dist<image_min_dist)
					image_min_dist = tmp_dist;
			}
			std::cout << "min_dist=" << image_min_dist << std::endl;
			//std::cout << "max_dist=" << image_max_dist << std::endl;
			//匹配结果筛选    
			std::vector<cv::DMatch> GoodMatchePoints;
			for (int j = 0; j < matches.size(); ++j)
			{
				double dist = matches[j].distance;
				if (dist < 2.5 * image_min_dist)
				{
					int good_idx = FindCVDMatchIdx(GoodMatchePoints, matches[j]);
					if (good_idx == -1)
					{
						GoodMatchePoints.push_back(matches[j]);
					}
					else
					{
						if (GoodMatchePoints[good_idx].distance > dist)
						{
							GoodMatchePoints[good_idx] = matches[j];
						}
					}
				}
			}
			//std::cout << "goodMatches:" << (int)GoodMatchePoints.size() << std::endl;
			int num_matches = static_cast<int>(GoodMatchePoints.size());
			std::cerr << "the two images matches pnts num is: " << num_matches << std::endl;
			//
			std::vector<cv::DMatch> filter_matches;
			for (std::size_t j = 0; j < GoodMatchePoints.size(); ++j)
			{
				if (GoodMatchePoints[j].queryIdx < 0 || GoodMatchePoints[j].trainIdx < 0)
					continue;
				HWCorrespondence2D2DPnt match_pnt;
				match_pnt.p1[0] = src_keypnts[GoodMatchePoints[j].queryIdx].pt.x;
				match_pnt.p1[1] = src_keypnts[GoodMatchePoints[j].queryIdx].pt.y;
				match_pnt.p2[0] = tgt_keypnts[GoodMatchePoints[j].trainIdx].pt.x;
				match_pnt.p2[1] = tgt_keypnts[GoodMatchePoints[j].trainIdx].pt.y;
				//it is important... to do next...
				int src_pnt_idx = GoodMatchePoints[j].queryIdx;
				int tgt_pnt_idx = GoodMatchePoints[j].trainIdx;
				//check the pnt coorindate 
				if (filter_wrong_lines_match_)
				{
					//get polygon
					Eigen::Vector3f src_pnt3d, tgt_pnt3d;
					bool src_pnt3d_flag = ImagePnt2dProj2ScenePolygonPnt3d(src_cam, match_pnt.p1, src_pnt3d);
					bool tgt_pnt3d_flag = ImagePnt2dProj2ScenePolygonPnt3d(tgt_cam, match_pnt.p2, tgt_pnt3d);
					if (src_pnt3d_flag && tgt_pnt3d_flag)
					{
						//check pnt is close enough
						if (CheckTwoPnt3dTheshold(src_pnt3d, tgt_pnt3d, r_max_threshold_))
						{
							//test
							filter_matches.emplace_back(GoodMatchePoints[j]);
							/*if (src_cam_id == 0 && tgt_cam_id == 2)
							{
								std::cerr << "src_cam_id, src_pnt_idx: " << src_cam_id << ", " << src_pnt_idx << " --> " <<
									"tgt_cam_id, tgt_pnt_idx: " << tgt_cam_id << ", " << tgt_pnt_idx << std::endl;
							}*/
							//end test
							HWPntsMatch cor_pnt;
							cor_pnt.pnts_matches_idx_.first = src_pnt_idx;
							cor_pnt.pnts_matches_idx_.second = tgt_pnt_idx;
							Eigen::Vector3f tri_pnt = (src_pnt3d + tgt_pnt3d) / 2;
							cor_pnt.view12_tri_pnt3d_ = tri_pnt;
							cor_pnt.corresponding_pnts_ = match_pnt;
							cor_pnt.valid_match_ = true;
							scenes_lines_pnts_matches_[i].points_matches.emplace_back(cor_pnt);
						}
					}
				}
				//unfiltered_matches.push_back(match);
				//unfiltered_indices.push_back(std::make_pair(matches_cv[i].queryIdx, matches_cv[i].trainIdx));
				//matches->push_back(unfiltered_indices[i]);
			}

			/*if (src_cam_id == 0 && tgt_cam_id == 2)
			{
				int num_filter_matches = static_cast<int>(filter_matches.size());
				std::cerr << "the two images GOOD matches pnts num is: " << num_filter_matches << std::endl;
				cv::Mat out_filter_matches_img;
				cv::drawMatches(src_cv_img, src_keypnts, tgt_cv_img, tgt_keypnts, filter_matches, out_filter_matches_img);
				std::string out_filter_matches_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/src2tgt_pnts_match_filter_02_new_removed.png";
				cv::imwrite(out_filter_matches_path, out_filter_matches_img);
			}*/

		}

		/*for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			int src_view_id = scenes_lines_pnts_matches_[i].view_1_id;
			int tgt_view_id = scenes_lines_pnts_matches_[i].view_2_id;
			if (src_view_id == 0 && tgt_view_id == 2)
			{
				int src_view_idx = scenes_cams_->GetCamidxFromCamId(src_view_id);
				int tgt_view_idx = scenes_cams_->GetCamidxFromCamId(tgt_view_id);
				int src_image_id = scenes_cams_->GetImageidFromCamId(src_view_id);
				int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgt_view_id);
				cv::Mat src_image = scenes_cams_->GetImageFromImgId(src_image_id).GetImage();
				cv::Mat tgt_image = scenes_cams_->GetImageFromImgId(tgt_image_id).GetImage();
				std::vector<HWPntsMatch> cor_pnts = scenes_lines_pnts_matches_[i].points_matches;
				std::vector<cv::DMatch> src_tgt_match;
				for (int j = 0; j < cor_pnts.size(); ++j)
				{
					HWPntsMatch pnt_match = cor_pnts[j];
					int src_pnt2d_idx = pnt_match.pnts_matches_idx_.first;
					int tgt_pnt2d_idx = pnt_match.pnts_matches_idx_.second;
					cv::DMatch tmp_match;
					tmp_match.queryIdx = src_pnt2d_idx;
					tmp_match.trainIdx = tgt_pnt2d_idx;
					src_tgt_match.emplace_back(tmp_match);
					std::cerr << "src_view_idx, tgt_view_idx: " << src_view_idx << ", " << tgt_view_idx << std::endl;
					std::cerr << "src_pnt2d_idx, tgt_pnt2d_idx: " << src_pnt2d_idx << ", " << tgt_pnt2d_idx << std::endl;
					cv::KeyPoint p0_cv = scenes_sifts_positions_[src_view_idx][src_pnt2d_idx];
					cv::KeyPoint p1_cv = scenes_sifts_positions_[tgt_view_idx][tgt_pnt2d_idx];
					Eigen::Vector2f p0 = Eigen::Vector2f(p0_cv.pt.x, p0_cv.pt.y);
					Eigen::Vector2f p1 = Eigen::Vector2f(p1_cv.pt.x, p1_cv.pt.y);
					std::pair<Eigen::Vector2f, Eigen::Vector2f> line_p0p1 = std::make_pair(p0, p1);
					cv::Mat fusion0_mat;
					DrawOnePairPntsMatchedIntoMat(src_view_id, tgt_view_id, line_p0p1, fusion0_mat);
					std::string v0v2_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/v0v2_match02_0_new.png";
					cv::imwrite(v0v2_path, fusion0_mat);
					break;
				}
				cv::Mat out_filter_matches_img;
				cv::drawMatches(src_image, scenes_sifts_positions_[src_view_idx], tgt_image, scenes_sifts_positions_[tgt_view_idx], src_tgt_match, out_filter_matches_img);
				std::string test_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/v0v2_match_test_0_new.png";
				cv::imwrite(test_path, out_filter_matches_img);
			}
		}*/

		std::cerr << "end complete the filter image new..." << std::endl;
	}

	int HWScenesElements::FindCVDMatchIdx(const std::vector<cv::DMatch>& matchs_vec, const cv::DMatch& match_v)
	{
		for (int i = 0; i < matchs_vec.size(); ++i)
		{
			if (match_v.trainIdx == matchs_vec[i].trainIdx)
			{
				return i;
			}
		}
		return -1;
	}

	void HWScenesElements::RunTrackLines3dListFromImgsLinesMatch()
	{
		std::cerr << "do run track line 3d from lines pair match..." << std::endl;
		for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			//std::cerr << "scenes_lines_pnts_matches_ idx: " << i << std::endl;
			HWTwoViewsCamsMatching line_pnt_match = scenes_lines_pnts_matches_[i];
			int view_1_id = line_pnt_match.view_1_id;
			int view_2_id = line_pnt_match.view_2_id;
			
			//Viewport& viewport1 = viewports->at(tvm.view_1_id);
			//Viewport& viewport2 = viewports->at(tvm.view_2_id);
			HWCamViewport& const viewport1 = hw_cams_views_list_[view_1_id];
			HWCamViewport& const viewport2 = hw_cams_views_list_[view_2_id];

			/* Iterate over matches for a pair. */
			for (std::size_t j = 0; j < line_pnt_match.lines_matches.size(); ++j)
			{
				//std::cerr << "lines_matches idx: " << j << std::endl;
				//line_pnt_match.lines_matches[j].valid_match_; check them
				std::pair<int, int> idx = line_pnt_match.lines_matches[j].line_matches_idx_;
				int const view1_line_tid = viewport1.lines_track_ids_[idx.first];
				int const view2_line_tid = viewport2.lines_track_ids_[idx.second];
				/*std::cerr << "view_1_id, src_pnt_idx: " << view_1_id << ", " << idx.first << " --> " <<
					"view_2_id, tgt_pnt_idx: " << view_2_id << ", " << idx.second << std::endl;*/
				if (view1_line_tid == -1 && view2_line_tid == -1)
				{
					/* No track ID associated with the match. Create track. */
					viewport1.lines_track_ids_[idx.first] = scenes_line3d_track_list_.size();
					viewport2.lines_track_ids_[idx.second] = scenes_line3d_track_list_.size();
					scenes_line3d_track_list_.push_back(HWLineTrack3D());
					scenes_line3d_track_list_.back().lines_features_.push_back(
						HWLineFeatureReferenceId(view_1_id, idx.first));

					//lines2d_idxs_to_grouped_lines2d_idxs_
					////get line 3d pnt
					//Eigen::Vector3f v1_line_s_pnt = line_pnt_match.lines_matches[j].world_p1_;
					//Eigen::Vector3f v1_line_e_pnt = line_pnt_match.lines_matches[j].world_p2_;
					//scenes_line3d_track_list_.back().line_views_pnts_.emplace_back(std::make_pair(v1_line_s_pnt, v1_line_e_pnt));

					scenes_line3d_track_list_.back().lines_features_.push_back(
						HWLineFeatureReferenceId(view_2_id, idx.second));

					scenes_line3d_track_list_.back().is_valid_ = true;
					////get line 3d pnt
					//Eigen::Vector3f v2_line_s_pnt = line_pnt_match.lines_matches[j].world_q1_;
					//Eigen::Vector3f v2_line_e_pnt = line_pnt_match.lines_matches[j].world_q2_;
					//scenes_line3d_track_list_.back().line_views_pnts_.emplace_back(std::make_pair(v2_line_s_pnt, v2_line_e_pnt));
				}
				else if (view1_line_tid == -1 && view2_line_tid != -1)
				{
					/* Propagate track ID from first to second view. */
					viewport1.lines_track_ids_[idx.first] = view2_line_tid;
					//scenes_line3d_track_list_[]
					scenes_line3d_track_list_[view2_line_tid].lines_features_.push_back(
						HWLineFeatureReferenceId(view_1_id, idx.first));
					//scenes_line3d_track_list_.back().is_valid_ = true;
				}
				else if (view1_line_tid != -1 && view2_line_tid == -1)
				{
					/* Propagate track ID from second to first view. */
					viewport2.lines_track_ids_[idx.second] = view1_line_tid;
					scenes_line3d_track_list_[view1_line_tid].lines_features_.push_back(
						HWLineFeatureReferenceId(view_2_id, idx.second));
					//scenes_line3d_track_list_.back().is_valid_ = true;
				}
				else if (view1_line_tid == view2_line_tid)
				{
					/* Track ID already propagated. */
				}
				else
				{
					/*
					* A track ID is already associated with both ends of a match,
					* however, is not consistent. Unify tracks.
					*/
					//std::cerr << "to do next..." << std::endl;
					//unify_tracks(view1_tid, view2_tid, tracks, viewports);
					unify_new_lines_tracks(view1_line_tid, view2_line_tid);
				}
			}
		}

		std::cerr << "compute track line 3d point position..." << std::endl;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			//compute the line point 3d
			//std::cerr << "track line 3d idx: " << i << std::endl;	//
			HWLineTrack3D& const tmp_line3d_track = scenes_line3d_track_list_[i];
			//int view_lines_num = 0;
			//Eigen::Vector3f fusion_s_pnt3d = Eigen::Vector3f(0.0, 0.0, 0.0);
			//Eigen::Vector3f fusion_e_pnt3d = Eigen::Vector3f(0.0, 0.0, 0.0);
			//get line point 2d
			int lines_features_num = static_cast<int>(tmp_line3d_track.lines_features_.size());
			tmp_line3d_track.line_views_pnts_.resize(lines_features_num);
			//std::cerr << "i: " << i << std::endl;
			std::map<int, int> polyidxs2counts;
			for (int j = 0; j < tmp_line3d_track.lines_features_.size(); ++j)
			{
				//std::cerr << "line 2d idx: " << j << std::endl;
				int view_id = tmp_line3d_track.lines_features_[j].view_id_;
				int line_id = tmp_line3d_track.lines_features_[j].line_id_;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> view_line_pnts_pos = hw_cams_views_list_[view_id].lines_segments_[line_id];
				CameraModel tmp_cam = hw_cams_views_list_[view_id].view_pose_;
				Eigen::Vector3f tmp_s_pnt3d, tmp_e_pnt3d;
				//bool tmp_s_ray_flag = ImagePnt2dPosProj2ScenePolygonsPnt3dPos(tmp_cam, view_line_pnts_pos.first, tmp_s_pnt3d);
				//bool tmp_e_ray_flag = ImagePnt2dPosProj2ScenePolygonsPnt3dPos(tmp_cam, view_line_pnts_pos.second, tmp_e_pnt3d);
				
				//get the ray to polygon(it is wrong, when the camera is deviate from correct pose(ray to other polygon))
				int idx_sray_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tmp_cam, view_line_pnts_pos.first, tmp_s_pnt3d);
				int idx_eray_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tmp_cam, view_line_pnts_pos.second, tmp_e_pnt3d);

				if (idx_sray_intersect != -1)
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].first = tmp_s_pnt3d;
					//scenes_line3d_track_list_[i].polygon_idxs_ line_views_pnts_
					std::map<int, int>::iterator idx_iter = polyidxs2counts.find(idx_sray_intersect);
					if (idx_iter == polyidxs2counts.end())
					{
						polyidxs2counts[idx_sray_intersect] = 1;
					}
					else
					{
						idx_iter->second += 1;
					}
				}
				else
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].first = Eigen::Vector3f(
						std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
						std::numeric_limits<float>::quiet_NaN());
				}
				if (idx_eray_intersect != -1)
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].second = tmp_e_pnt3d;
					std::map<int, int>::iterator idx_iter = polyidxs2counts.find(idx_sray_intersect);
					if (idx_iter == polyidxs2counts.end())
					{
						polyidxs2counts[idx_sray_intersect] = 1;
					}
					else
					{
						idx_iter->second += 1;
					}
				}
				else
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].second = Eigen::Vector3f(
						std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
						std::numeric_limits<float>::quiet_NaN());
				}
			}

			//sort MyPolygonIdxCmpZDG
			//std::sort(polyidxs2counts.begin(), polyidxs2counts.end(), MyPolygonIdxCampareZDG);
			std::vector<std::pair<int, int> > polyidxs2countsvec(polyidxs2counts.begin(), polyidxs2counts.end());
			std::sort(polyidxs2countsvec.begin(), polyidxs2countsvec.end(), MyPolygonIdxCmpZDG);
			//scenes_pnts3d_track_list_[i].pos_ = (fusion_pnt3d / view_features_num);
			std::vector<std::pair<int, int> >::iterator iter_poly = polyidxs2countsvec.begin();
			for (; iter_poly != polyidxs2countsvec.end(); ++iter_poly)
			{
				scenes_line3d_track_list_[i].polygon_idxs_.emplace_back(iter_poly->first);
			}
		}

		////test
		//std::cerr << "do compute the line tracks pnts 3d " << std::endl;
		////std::vector<Eigen::Vector3f> scene_test_lines_pnts;
		//std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> scene_test_views_lines_pnts;
		//for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		//{
		//	for (int j = 0; j < scenes_line3d_track_list_[i].line_views_pnts_.size(); ++j)
		//	{
		//		std::pair<Eigen::Vector3f, Eigen::Vector3f> line_pnt = scenes_line3d_track_list_[i].line_views_pnts_[j];
		//		std::cerr << "line start pnt and end pnt: " << line_pnt.first.transpose() << ", " 
		//			<< line_pnt.second.transpose() << std::endl;
		//		if (line_pnt.first[0] == line_pnt.first[0]
		//			&& line_pnt.first[1] == line_pnt.first[1]
		//			&& line_pnt.first[2] == line_pnt.first[2]
		//			&& line_pnt.second[0] == line_pnt.second[0]
		//			&& line_pnt.second[1] == line_pnt.second[1]
		//			&& line_pnt.second[2] == line_pnt.second[2])
		//		{
		//			//std::cerr << "in" << std::endl;
		//			scene_test_views_lines_pnts.emplace_back(line_pnt);
		//		}
		//	}
		//}
		////std::string sc_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/scene_pnts_observation.obj";
		//std::string sc_view_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part5_test/line_test_obj/views/scene_views_lines_views_observation_before.obj";
		////int scene_lines_pnts_num = static_cast<int>(scene_test_lines_pnts.size());
		//int scene_views_lines_pnts_num = static_cast<int>(scene_test_views_lines_pnts.size());
		////std::cerr << "scene lines number: " << scene_lines_pnts_num << std::endl;
		////WritePntsIntoObj(scene_test_lines_pnts, sc_path);
		//std::cerr << "scene view lines number: " << scene_views_lines_pnts_num << std::endl;
		//WriteLine3DIntoObj(scene_test_views_lines_pnts, sc_view_path);
		////end test

		std::cerr << "end track line 3d from lines pair match..." << std::endl;
	}

	void HWScenesElements::RunTrackLines3dListFromImgsLinesMatchFilterByGroupLines()
	{
		std::cerr << "do run track line 3d from lines pair match..." << std::endl;
		for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			//std::cerr << "scenes_lines_pnts_matches_ idx: " << i << std::endl;
			HWTwoViewsCamsMatching line_pnt_match = scenes_lines_pnts_matches_[i];
			int view_1_id = line_pnt_match.view_1_id;
			int view_2_id = line_pnt_match.view_2_id;

			//Viewport& viewport1 = viewports->at(tvm.view_1_id);
			//Viewport& viewport2 = viewports->at(tvm.view_2_id);
			HWCamViewport& const viewport1 = hw_cams_views_list_[view_1_id];
			HWCamViewport& const viewport2 = hw_cams_views_list_[view_2_id];

			/* Iterate over matches for a pair. */
			for (std::size_t j = 0; j < line_pnt_match.lines_matches.size(); ++j)
			{
				if (!line_pnt_match.lines_matches[j].valid_match_)
				{
					continue;
				}
				//std::cerr << "lines_matches idx: " << j << std::endl;
				//line_pnt_match.lines_matches[j].valid_match_; check them
				std::pair<int, int> idx = line_pnt_match.lines_matches[j].line_matches_idx_;
				int const view1_line_tid = viewport1.lines_track_ids_[idx.first];
				int const view2_line_tid = viewport2.lines_track_ids_[idx.second];
				/*std::cerr << "view_1_id, src_pnt_idx: " << view_1_id << ", " << idx.first << " --> " <<
				"view_2_id, tgt_pnt_idx: " << view_2_id << ", " << idx.second << std::endl;*/
				if (view1_line_tid == -1 && view2_line_tid == -1)
				{
					/* No track ID associated with the match. Create track. */
					viewport1.lines_track_ids_[idx.first] = scenes_line3d_track_list_.size();
					viewport2.lines_track_ids_[idx.second] = scenes_line3d_track_list_.size();
					scenes_line3d_track_list_.push_back(HWLineTrack3D());
					scenes_line3d_track_list_.back().lines_features_.push_back(
						HWLineFeatureReferenceId(view_1_id, idx.first));
					//get group idx
					std::pair<int, int> view1_line_idx_to_group_line_idx = std::make_pair(-1, -1);
					GetPairFromVectorPairsByPairFirst(viewport1.lines2d_idxs_to_grouped_lines2d_idxs_,
						idx.first, view1_line_idx_to_group_line_idx);
					scenes_line3d_track_list_.back().lines_features_.back().line_group_id 
						= view1_line_idx_to_group_line_idx.second;

					////get line 3d pnt
					//Eigen::Vector3f v1_line_s_pnt = line_pnt_match.lines_matches[j].world_p1_;
					//Eigen::Vector3f v1_line_e_pnt = line_pnt_match.lines_matches[j].world_p2_;
					//scenes_line3d_track_list_.back().line_views_pnts_.emplace_back(std::make_pair(v1_line_s_pnt, v1_line_e_pnt));

					scenes_line3d_track_list_.back().lines_features_.push_back(
						HWLineFeatureReferenceId(view_2_id, idx.second));

					//get group idx
					std::pair<int, int> view2_line_idx_to_group_line_idx = std::make_pair(-1, -1);
					GetPairFromVectorPairsByPairFirst(viewport2.lines2d_idxs_to_grouped_lines2d_idxs_,
						idx.second, view2_line_idx_to_group_line_idx);
					scenes_line3d_track_list_.back().lines_features_.back().line_group_id
						= view2_line_idx_to_group_line_idx.second;
				
					//lines2d_idxs_to_grouped_lines2d_idxs_
					//viewport1.lines2d_idxs_to_grouped_lines2d_idxs_[]

					scenes_line3d_track_list_.back().is_valid_ = true;
					////get line 3d pnt
					//Eigen::Vector3f v2_line_s_pnt = line_pnt_match.lines_matches[j].world_q1_;
					//Eigen::Vector3f v2_line_e_pnt = line_pnt_match.lines_matches[j].world_q2_;
					//scenes_line3d_track_list_.back().line_views_pnts_.emplace_back(std::make_pair(v2_line_s_pnt, v2_line_e_pnt));
				}
				else if (view1_line_tid == -1 && view2_line_tid != -1)
				{
					/* Propagate track ID from first to second view. */
					viewport1.lines_track_ids_[idx.first] = view2_line_tid;
					//scenes_line3d_track_list_[]
					scenes_line3d_track_list_[view2_line_tid].lines_features_.push_back(
						HWLineFeatureReferenceId(view_1_id, idx.first));

					//get group idx
					std::pair<int, int> view1_line_idx_to_group_line_idx = std::make_pair(-1, -1);
					GetPairFromVectorPairsByPairFirst(viewport1.lines2d_idxs_to_grouped_lines2d_idxs_,
						idx.first, view1_line_idx_to_group_line_idx);
					scenes_line3d_track_list_[view2_line_tid].lines_features_.back().line_group_id
						= view1_line_idx_to_group_line_idx.second;

					//scenes_line3d_track_list_.back().is_valid_ = true;
				}
				else if (view1_line_tid != -1 && view2_line_tid == -1)
				{
					/* Propagate track ID from second to first view. */
					viewport2.lines_track_ids_[idx.second] = view1_line_tid;
					scenes_line3d_track_list_[view1_line_tid].lines_features_.push_back(
						HWLineFeatureReferenceId(view_2_id, idx.second));

					//get group idx
					std::pair<int, int> view2_line_idx_to_group_line_idx = std::make_pair(-1, -1);
					GetPairFromVectorPairsByPairFirst(viewport2.lines2d_idxs_to_grouped_lines2d_idxs_,
						idx.second, view2_line_idx_to_group_line_idx);
					scenes_line3d_track_list_[view1_line_tid].lines_features_.back().line_group_id
						= view2_line_idx_to_group_line_idx.second;

					//scenes_line3d_track_list_.back().is_valid_ = true;
				}
				else if (view1_line_tid == view2_line_tid)
				{
					/* Track ID already propagated. */
				}
				else
				{
					/*
					* A track ID is already associated with both ends of a match,
					* however, is not consistent. Unify tracks.
					*/
					//std::cerr << "to do next..." << std::endl;
					//unify_tracks(view1_tid, view2_tid, tracks, viewports);
					unify_new_lines_tracks(view1_line_tid, view2_line_tid);
				}
			}
		}

		std::cerr << "compute track line 3d point position..." << std::endl;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			//compute the line point 3d
			//std::cerr << "track line 3d idx: " << i << std::endl;	//
			HWLineTrack3D& const tmp_line3d_track = scenes_line3d_track_list_[i];
			//int view_lines_num = 0;
			//Eigen::Vector3f fusion_s_pnt3d = Eigen::Vector3f(0.0, 0.0, 0.0);
			//Eigen::Vector3f fusion_e_pnt3d = Eigen::Vector3f(0.0, 0.0, 0.0);
			//get line point 2d
			int lines_features_num = static_cast<int>(tmp_line3d_track.lines_features_.size());
			tmp_line3d_track.line_views_pnts_.resize(lines_features_num);
			scenes_line3d_track_list_[i].line_views_to_polygon_idxs_.resize(lines_features_num, -1);

			//std::cerr << "i: " << i << std::endl;
			std::map<int, int> polyidxs2counts;
			for (int j = 0; j < tmp_line3d_track.lines_features_.size(); ++j)
			{
				//std::cerr << "line 2d idx: " << j << std::endl;
				int view_id = tmp_line3d_track.lines_features_[j].view_id_;
				int line_id = tmp_line3d_track.lines_features_[j].line_id_;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> view_line_pnts_pos = hw_cams_views_list_[view_id].lines_segments_[line_id];
				CameraModel tmp_cam = hw_cams_views_list_[view_id].view_pose_;


#if 0
				//Eigen::Vector3f tmp_s_pnt3d, tmp_e_pnt3d;
				//bool tmp_s_ray_flag = ImagePnt2dPosProj2ScenePolygonsPnt3dPos(tmp_cam, view_line_pnts_pos.first, tmp_s_pnt3d);
				//bool tmp_e_ray_flag = ImagePnt2dPosProj2ScenePolygonsPnt3dPos(tmp_cam, view_line_pnts_pos.second, tmp_e_pnt3d);
				//get the ray to polygon(it is wrong, when the camera is deviate from correct pose(ray to other polygon))
				int idx_sray_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tmp_cam, view_line_pnts_pos.first, tmp_s_pnt3d);
				int idx_eray_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tmp_cam, view_line_pnts_pos.second, tmp_e_pnt3d);
				if (idx_sray_intersect != -1)
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].first = tmp_s_pnt3d;
					//scenes_line3d_track_list_[i].polygon_idxs_ line_views_pnts_
					std::map<int, int>::iterator idx_iter = polyidxs2counts.find(idx_sray_intersect);
					if (idx_iter == polyidxs2counts.end())
					{
						polyidxs2counts[idx_sray_intersect] = 1;
					}
					else
					{
						idx_iter->second += 1;
					}
				}
				else
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].first = Eigen::Vector3f(
						std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
						std::numeric_limits<float>::quiet_NaN());
				}
				if (idx_eray_intersect != -1)
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].second = tmp_e_pnt3d;
					std::map<int, int>::iterator idx_iter = polyidxs2counts.find(idx_sray_intersect);
					if (idx_iter == polyidxs2counts.end())
					{
						polyidxs2counts[idx_sray_intersect] = 1;
					}
					else
					{
						idx_iter->second += 1;
					}
				}
				else
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].second = Eigen::Vector3f(
						std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
						std::numeric_limits<float>::quiet_NaN());
				}
#else
				Eigen::Vector3f tmp_ray_pnt3d;
				//check the view line, if it assigned to polygon idx
				std::vector<Eigen::Vector2f> view_line_sample_pnts_pos;
				//check the pnts from line segment
				SampleNLineFloatPntsFromLineSeg(view_line_pnts_pos, 8, view_line_sample_pnts_pos);

				//polygonidxs_counts[i].first polygon idx; polygonidxs_counts[i].second polygon idx's number
				std::vector<std::pair<int, int> > polygonidxs_counts;
				ComputeImagesPntsRayToPolygonsIdxs(view_line_sample_pnts_pos, tmp_cam, polygonidxs_counts);
				Eigen::Vector2i tmp_pnts_polygon_idxs;
				ExtractPolygonIdxFromPolygonsIdxsCounts(polygonidxs_counts, tmp_pnts_polygon_idxs);
				//get single line to polygon idx(only one polygon, drop other's polygon idx)
				//only to single polygon idxs
				int picked_polygon_idx = tmp_pnts_polygon_idxs[0];
				Eigen::Vector3f tmp_s_pnt3d, tmp_e_pnt3d;
				if (picked_polygon_idx != -1)
				{
					bool is_ls_intersection = ImagePnt2dPosProj2ScenePickedPolygonPnt3dPos(tmp_cam, view_line_pnts_pos.first, picked_polygon_idx, tmp_s_pnt3d);
					bool is_le_intersection = ImagePnt2dPosProj2ScenePickedPolygonPnt3dPos(tmp_cam, view_line_pnts_pos.second, picked_polygon_idx, tmp_e_pnt3d);
					if (is_ls_intersection)
					{
						scenes_line3d_track_list_[i].line_views_pnts_[j].first = tmp_s_pnt3d;
						//scenes_line3d_track_list_[i].polygon_idxs_ line_views_pnts_
						std::map<int, int>::iterator idx_iter = polyidxs2counts.find(picked_polygon_idx);
						if (idx_iter == polyidxs2counts.end())
						{
							polyidxs2counts[picked_polygon_idx] = 1;
						}
						else
						{
							idx_iter->second += 1;
						}
					}
					else
					{
						scenes_line3d_track_list_[i].line_views_pnts_[j].first = Eigen::Vector3f(
							std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
							std::numeric_limits<float>::quiet_NaN());
					}

					if (is_le_intersection)
					{
						scenes_line3d_track_list_[i].line_views_pnts_[j].second = tmp_e_pnt3d;
						//scenes_line3d_track_list_[i].polygon_idxs_ line_views_pnts_
						std::map<int, int>::iterator idx_iter = polyidxs2counts.find(picked_polygon_idx);
						if (idx_iter == polyidxs2counts.end())
						{
							polyidxs2counts[picked_polygon_idx] = 1;
						}
						else
						{
							idx_iter->second += 1;
						}
					}
					else
					{
						scenes_line3d_track_list_[i].line_views_pnts_[j].second = Eigen::Vector3f(
							std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
							std::numeric_limits<float>::quiet_NaN());
					}
					scenes_line3d_track_list_[i].line_views_to_polygon_idxs_[j] = picked_polygon_idx;
				}
				else
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].first = Eigen::Vector3f(
						std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
						std::numeric_limits<float>::quiet_NaN());
					scenes_line3d_track_list_[i].line_views_pnts_[j].second = Eigen::Vector3f(
						std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
						std::numeric_limits<float>::quiet_NaN());
				}
#endif

			}

			//sort MyPolygonIdxCmpZDG
			//std::sort(polyidxs2counts.begin(), polyidxs2counts.end(), MyPolygonIdxCampareZDG);
			std::vector<std::pair<int, int> > polyidxs2countsvec(polyidxs2counts.begin(), polyidxs2counts.end());
			std::sort(polyidxs2countsvec.begin(), polyidxs2countsvec.end(), MyPolygonIdxCmpZDG);
			//scenes_pnts3d_track_list_[i].pos_ = (fusion_pnt3d / view_features_num);
			std::vector<std::pair<int, int> >::iterator iter_poly = polyidxs2countsvec.begin();
			for (; iter_poly != polyidxs2countsvec.end(); ++iter_poly)
			{
				scenes_line3d_track_list_[i].polygon_idxs_.emplace_back(iter_poly->first);
			}
		}

		////test
		//std::cerr << "do compute the line tracks pnts 3d " << std::endl;
		////std::vector<Eigen::Vector3f> scene_test_lines_pnts;
		//std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> scene_test_views_lines_pnts;
		//for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		//{
		//	for (int j = 0; j < scenes_line3d_track_list_[i].line_views_pnts_.size(); ++j)
		//	{
		//		std::pair<Eigen::Vector3f, Eigen::Vector3f> line_pnt = scenes_line3d_track_list_[i].line_views_pnts_[j];
		//		std::cerr << "line start pnt and end pnt: " << line_pnt.first.transpose() << ", " 
		//			<< line_pnt.second.transpose() << std::endl;
		//		if (line_pnt.first[0] == line_pnt.first[0]
		//			&& line_pnt.first[1] == line_pnt.first[1]
		//			&& line_pnt.first[2] == line_pnt.first[2]
		//			&& line_pnt.second[0] == line_pnt.second[0]
		//			&& line_pnt.second[1] == line_pnt.second[1]
		//			&& line_pnt.second[2] == line_pnt.second[2])
		//		{
		//			//std::cerr << "in" << std::endl;
		//			scene_test_views_lines_pnts.emplace_back(line_pnt);
		//		}
		//	}
		//}
		////std::string sc_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/scene_pnts_observation.obj";
		//std::string sc_view_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part5_test/line_test_obj/views/scene_views_lines_views_observation_before.obj";
		////int scene_lines_pnts_num = static_cast<int>(scene_test_lines_pnts.size());
		//int scene_views_lines_pnts_num = static_cast<int>(scene_test_views_lines_pnts.size());
		////std::cerr << "scene lines number: " << scene_lines_pnts_num << std::endl;
		////WritePntsIntoObj(scene_test_lines_pnts, sc_path);
		//std::cerr << "scene view lines number: " << scene_views_lines_pnts_num << std::endl;
		//WriteLine3DIntoObj(scene_test_views_lines_pnts, sc_view_path);
		////end test

		std::cerr << "end track line 3d from lines pair match..." << std::endl;
	}

	void HWScenesElements::FilterGroupedTrackLines3dListNotInPolygons()
	{
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			if (scenes_line3d_track_list_[i].polygon_idxs_.empty())
			{
				scenes_line3d_track_list_[i].is_valid_ = false;
			}
			else
			{
				if (scenes_line3d_track_list_[i].polygon_idxs_[0] == -1)
				{
					scenes_line3d_track_list_[i].is_valid_ = false;
				}
			}
		}
	}

	void HWScenesElements::RunTrackLines3dListFromImgsLinesMatchFilter()
	{
		std::cerr << "do run track line 3d from lines pair match..." << std::endl;
		for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			//std::cerr << "scenes_lines_pnts_matches_ idx: " << i << std::endl;
			HWTwoViewsCamsMatching line_pnt_match = scenes_lines_pnts_matches_[i];
			int view_1_id = line_pnt_match.view_1_id;
			int view_2_id = line_pnt_match.view_2_id;

			//Viewport& viewport1 = viewports->at(tvm.view_1_id);
			//Viewport& viewport2 = viewports->at(tvm.view_2_id);
			HWCamViewport& const viewport1 = hw_cams_views_list_[view_1_id];
			HWCamViewport& const viewport2 = hw_cams_views_list_[view_2_id];

			/* Iterate over matches for a pair. */
			for (std::size_t j = 0; j < line_pnt_match.lines_matches.size(); ++j)
			{
				//std::cerr << "lines_matches idx: " << j << std::endl;
				//line_pnt_match.lines_matches[j].valid_match_; check them
				if (!line_pnt_match.lines_matches[j].valid_match_)
				{
					continue;
				}
				std::pair<int, int> idx = line_pnt_match.lines_matches[j].line_matches_idx_;
				int const view1_line_tid = viewport1.lines_track_ids_[idx.first];
				int const view2_line_tid = viewport2.lines_track_ids_[idx.second];
				/*std::cerr << "view_1_id, src_pnt_idx: " << view_1_id << ", " << idx.first << " --> " <<
				"view_2_id, tgt_pnt_idx: " << view_2_id << ", " << idx.second << std::endl;*/
				if (view1_line_tid == -1 && view2_line_tid == -1)
				{
					/* No track ID associated with the match. Create track. */
					viewport1.lines_track_ids_[idx.first] = scenes_line3d_track_list_.size();
					viewport2.lines_track_ids_[idx.second] = scenes_line3d_track_list_.size();
					scenes_line3d_track_list_.push_back(HWLineTrack3D());
					scenes_line3d_track_list_.back().lines_features_.push_back(
						HWLineFeatureReferenceId(view_1_id, idx.first));
					////get line 3d pnt
					//Eigen::Vector3f v1_line_s_pnt = line_pnt_match.lines_matches[j].world_p1_;
					//Eigen::Vector3f v1_line_e_pnt = line_pnt_match.lines_matches[j].world_p2_;
					//scenes_line3d_track_list_.back().line_views_pnts_.emplace_back(std::make_pair(v1_line_s_pnt, v1_line_e_pnt));

					scenes_line3d_track_list_.back().lines_features_.push_back(
						HWLineFeatureReferenceId(view_2_id, idx.second));

					scenes_line3d_track_list_.back().is_valid_ = true;
					////get line 3d pnt
					//Eigen::Vector3f v2_line_s_pnt = line_pnt_match.lines_matches[j].world_q1_;
					//Eigen::Vector3f v2_line_e_pnt = line_pnt_match.lines_matches[j].world_q2_;
					//scenes_line3d_track_list_.back().line_views_pnts_.emplace_back(std::make_pair(v2_line_s_pnt, v2_line_e_pnt));
				}
				else if (view1_line_tid == -1 && view2_line_tid != -1)
				{
					/* Propagate track ID from first to second view. */
					viewport1.lines_track_ids_[idx.first] = view2_line_tid;
					//scenes_line3d_track_list_[]
					scenes_line3d_track_list_[view2_line_tid].lines_features_.push_back(
						HWLineFeatureReferenceId(view_1_id, idx.first));
					//scenes_line3d_track_list_.back().is_valid_ = true;
				}
				else if (view1_line_tid != -1 && view2_line_tid == -1)
				{
					/* Propagate track ID from second to first view. */
					viewport2.lines_track_ids_[idx.second] = view1_line_tid;
					scenes_line3d_track_list_[view1_line_tid].lines_features_.push_back(
						HWLineFeatureReferenceId(view_2_id, idx.second));
					//scenes_line3d_track_list_.back().is_valid_ = true;
				}
				else if (view1_line_tid == view2_line_tid)
				{
					/* Track ID already propagated. */
				}
				else
				{
					/*
					* A track ID is already associated with both ends of a match,
					* however, is not consistent. Unify tracks.
					*/
					//std::cerr << "to do next..." << std::endl;
					//unify_tracks(view1_tid, view2_tid, tracks, viewports);
					unify_new_lines_tracks(view1_line_tid, view2_line_tid);
				}
			}
		}

		std::cerr << "compute track line 3d point position..." << std::endl;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			//compute the line point 3d
			//std::cerr << "track line 3d idx: " << i << std::endl;	//
			HWLineTrack3D& const tmp_line3d_track = scenes_line3d_track_list_[i];
			scenes_line3d_track_list_[i].polygon_idxs_.clear();
			//int view_lines_num = 0;
			//Eigen::Vector3f fusion_s_pnt3d = Eigen::Vector3f(0.0, 0.0, 0.0);
			//Eigen::Vector3f fusion_e_pnt3d = Eigen::Vector3f(0.0, 0.0, 0.0);
			//get line point 2d
			int lines_features_num = static_cast<int>(tmp_line3d_track.lines_features_.size());
			scenes_line3d_track_list_[i].line_views_pnts_.resize(lines_features_num);
			//std::cerr << "i: " << i << std::endl;
			std::map<int, int> polyidxs2counts;
			for (int j = 0; j < tmp_line3d_track.lines_features_.size(); ++j)
			{
				//std::cerr << "line 2d idx: " << j << std::endl;
				int view_id = tmp_line3d_track.lines_features_[j].view_id_;
				int line_id = tmp_line3d_track.lines_features_[j].line_id_;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> view_line_pnts_pos = hw_cams_views_list_[view_id].lines_segments_[line_id];
				CameraModel tmp_cam = hw_cams_views_list_[view_id].view_pose_;
				

#if 0
				//Eigen::Vector3f tmp_s_pnt3d, tmp_e_pnt3d;
				//bool tmp_s_ray_flag = ImagePnt2dPosProj2ScenePolygonsPnt3dPos(tmp_cam, view_line_pnts_pos.first, tmp_s_pnt3d);
				//bool tmp_e_ray_flag = ImagePnt2dPosProj2ScenePolygonsPnt3dPos(tmp_cam, view_line_pnts_pos.second, tmp_e_pnt3d);
				//get the ray to polygon(it is wrong, when the camera is deviate from correct pose(ray to other polygon))
				int idx_sray_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tmp_cam, view_line_pnts_pos.first, tmp_s_pnt3d);
				int idx_eray_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tmp_cam, view_line_pnts_pos.second, tmp_e_pnt3d);
				if (idx_sray_intersect != -1)
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].first = tmp_s_pnt3d;
					//scenes_line3d_track_list_[i].polygon_idxs_ line_views_pnts_
					std::map<int, int>::iterator idx_iter = polyidxs2counts.find(idx_sray_intersect);
					if (idx_iter == polyidxs2counts.end())
					{
						polyidxs2counts[idx_sray_intersect] = 1;
					}
					else
					{
						idx_iter->second += 1;
					}
				}
				else
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].first = Eigen::Vector3f(
						std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
						std::numeric_limits<float>::quiet_NaN());
				}
				if (idx_eray_intersect != -1)
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].second = tmp_e_pnt3d;
					std::map<int, int>::iterator idx_iter = polyidxs2counts.find(idx_sray_intersect);
					if (idx_iter == polyidxs2counts.end())
					{
						polyidxs2counts[idx_sray_intersect] = 1;
					}
					else
					{
						idx_iter->second += 1;
					}
				}
				else
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].second = Eigen::Vector3f(
						std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
						std::numeric_limits<float>::quiet_NaN());
				}
#else
				Eigen::Vector3f tmp_ray_pnt3d;
				//check the view line, if it assigned to polygon idx
				std::vector<Eigen::Vector2f> view_line_sample_pnts_pos;
				//check the pnts from line segment
				SampleNLineFloatPntsFromLineSeg(view_line_pnts_pos, 5, view_line_sample_pnts_pos);
				
				//polygonidxs_counts[i].first polygon idx; polygonidxs_counts[i].second polygon idx's number
				std::vector<std::pair<int, int> > polygonidxs_counts;
				ComputeImagesPntsRayToPolygonsIdxs(view_line_sample_pnts_pos, tmp_cam, polygonidxs_counts);
				Eigen::Vector2i tmp_pnts_polygon_idxs;
				ExtractPolygonIdxFromPolygonsIdxsCounts(polygonidxs_counts, tmp_pnts_polygon_idxs);
				//get single line to polygon idx(only one polygon, drop other's polygon idx)
				//only to single polygon idxs
				int picked_polygon_idx = tmp_pnts_polygon_idxs[0];
				Eigen::Vector3f tmp_s_pnt3d, tmp_e_pnt3d;
				if (picked_polygon_idx != -1)
				{
					bool is_ls_intersection = ImagePnt2dPosProj2ScenePickedPolygonPnt3dPos(tmp_cam, view_line_pnts_pos.first, picked_polygon_idx, tmp_s_pnt3d);
					bool is_le_intersection = ImagePnt2dPosProj2ScenePickedPolygonPnt3dPos(tmp_cam, view_line_pnts_pos.second, picked_polygon_idx, tmp_e_pnt3d);
					if (is_ls_intersection)
					{
						scenes_line3d_track_list_[i].line_views_pnts_[j].first = tmp_s_pnt3d;
						//scenes_line3d_track_list_[i].polygon_idxs_ line_views_pnts_
						std::map<int, int>::iterator idx_iter = polyidxs2counts.find(picked_polygon_idx);
						if (idx_iter == polyidxs2counts.end())
						{
							polyidxs2counts[picked_polygon_idx] = 1;
						}
						else
						{
							idx_iter->second += 1;
						}
					}
					else
					{
						scenes_line3d_track_list_[i].line_views_pnts_[j].first = Eigen::Vector3f(
							std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
							std::numeric_limits<float>::quiet_NaN());
					}

					if (is_le_intersection)
					{
						scenes_line3d_track_list_[i].line_views_pnts_[j].second = tmp_e_pnt3d;
						//scenes_line3d_track_list_[i].polygon_idxs_ line_views_pnts_
						std::map<int, int>::iterator idx_iter = polyidxs2counts.find(picked_polygon_idx);
						if (idx_iter == polyidxs2counts.end())
						{
							polyidxs2counts[picked_polygon_idx] = 1;
						}
						else
						{
							idx_iter->second += 1;
						}
					}
					else
					{
						scenes_line3d_track_list_[i].line_views_pnts_[j].second = Eigen::Vector3f(
							std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
							std::numeric_limits<float>::quiet_NaN());
					}
				}
				else
				{
					scenes_line3d_track_list_[i].line_views_pnts_[j].first = Eigen::Vector3f(
						std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
						std::numeric_limits<float>::quiet_NaN());
					scenes_line3d_track_list_[i].line_views_pnts_[j].second = Eigen::Vector3f(
						std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
						std::numeric_limits<float>::quiet_NaN());
				}
				
				//ImagePnt2dProj2ScenePolygonPnt3d
#endif

			}

			//sort MyPolygonIdxCmpZDG
			//std::sort(polyidxs2counts.begin(), polyidxs2counts.end(), MyPolygonIdxCampareZDG);
			std::vector<std::pair<int, int> > polyidxs2countsvec(polyidxs2counts.begin(), polyidxs2counts.end());
			std::sort(polyidxs2countsvec.begin(), polyidxs2countsvec.end(), MyPolygonIdxCmpZDG);
			//scenes_pnts3d_track_list_[i].pos_ = (fusion_pnt3d / view_features_num);
			std::vector<std::pair<int, int> >::iterator iter_poly = polyidxs2countsvec.begin();
			for (; iter_poly != polyidxs2countsvec.end(); ++iter_poly)
			{
				scenes_line3d_track_list_[i].polygon_idxs_.emplace_back(iter_poly->first);
			}
		}

		////test
		//std::cerr << "do compute the line tracks pnts 3d " << std::endl;
		////std::vector<Eigen::Vector3f> scene_test_lines_pnts;
		////std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scene_test_lines_pnts;
		//std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scene_test_views_lines_pnts;
		//for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		//{
		//	for (int j = 0; j < scenes_line3d_track_list_[i].line_views_pnts_.size(); ++j)
		//	{
		//		std::pair<Eigen::Vector3f, Eigen::Vector3f> line_pnt = scenes_line3d_track_list_[i].line_views_pnts_[j];
		//		//std::cerr << "line start pnt and end pnt: " << line_pnt.first.transpose() << ", " 
		//		//	<< line_pnt.second.transpose() << std::endl;
		//		if (line_pnt.first[0] == line_pnt.first[0]
		//			&& line_pnt.first[1] == line_pnt.first[1]
		//			&& line_pnt.first[2] == line_pnt.first[2]
		//			&& line_pnt.second[0] == line_pnt.second[0]
		//			&& line_pnt.second[1] == line_pnt.second[1]
		//			&& line_pnt.second[2] == line_pnt.second[2])
		//		{
		//			//std::cerr << "in" << std::endl;
		//			scene_test_views_lines_pnts.emplace_back(line_pnt);
		//		}
		//	}
		//}
		////std::string sc_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/scene_pnts_observation.obj";
		//std::string sc_view_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part5_test/line_test_obj/scene_views_lines_views_observation_before.obj";
		////int scene_lines_pnts_num = static_cast<int>(scene_test_lines_pnts.size());
		//int scene_views_lines_pnts_num = static_cast<int>(scene_test_views_lines_pnts.size());
		////std::cerr << "scene lines number: " << scene_lines_pnts_num << std::endl;
		////WritePntsIntoObj(scene_test_lines_pnts, sc_path);
		//std::cerr << "scene view lines number: " << scene_views_lines_pnts_num << std::endl;
		//WriteLine3DIntoObj(scene_test_views_lines_pnts, sc_view_path);
		////end test

		std::cerr << "end track line 3d from lines pair match..." << std::endl;
	}

	//to do next...
	void HWScenesElements::RunTrackLines3dListFromImgsLinesMatchWithLabels()
	{
		std::cerr << "do run track line 3d from lines pair match with labels..." << std::endl;
		for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			//std::cerr << "scenes_lines_pnts_matches_ idx: " << i << std::endl;
			HWTwoViewsCamsMatching line_pnt_match = scenes_lines_pnts_matches_[i];
			int view_1_id = line_pnt_match.view_1_id;
			int view_2_id = line_pnt_match.view_2_id;

			//Viewport& viewport1 = viewports->at(tvm.view_1_id);
			//Viewport& viewport2 = viewports->at(tvm.view_2_id);
			HWCamViewport& const viewport1 = hw_cams_views_list_[view_1_id];
			HWCamViewport& const viewport2 = hw_cams_views_list_[view_2_id];

			/* Iterate over matches for a pair. */
			for (std::size_t j = 0; j < line_pnt_match.lines_matches.size(); ++j)
			{
				//std::cerr << "lines_matches idx: " << j << std::endl;
				if (!line_pnt_match.lines_matches[j].valid_match_) //check them
				{
					continue;
				}
				std::pair<int, int> idx = line_pnt_match.lines_matches[j].line_matches_idx_;
				int const view1_line_tid = viewport1.lines_track_ids_[idx.first];
				int const view2_line_tid = viewport2.lines_track_ids_[idx.second];
				/*std::cerr << "view_1_id, src_pnt_idx: " << view_1_id << ", " << idx.first << " --> " <<
				"view_2_id, tgt_pnt_idx: " << view_2_id << ", " << idx.second << std::endl;*/
				if (view1_line_tid == -1 && view2_line_tid == -1)
				{
					/* No track ID associated with the match. Create track. */
					viewport1.lines_track_ids_[idx.first] = scenes_line3d_track_list_.size();
					viewport2.lines_track_ids_[idx.second] = scenes_line3d_track_list_.size();
					scenes_line3d_track_list_.push_back(HWLineTrack3D());

					//view1_line_tid
					scenes_line3d_track_list_.back().lines_features_.push_back(
						HWLineFeatureReferenceId(view_1_id, idx.first));
					//get polygon idx from label idx
					int view1_line_polygon_idx = line_pnt_match.lines_matches[j].p_polygon_idx_[0];
					scenes_line3d_track_list_.back().polygon_idxs_.emplace_back(view1_line_polygon_idx);
					////get line 3d pnt
					//Eigen::Vector3f v1_line_s_pnt = line_pnt_match.lines_matches[j].world_p1_;
					//Eigen::Vector3f v1_line_e_pnt = line_pnt_match.lines_matches[j].world_p2_;
					//scenes_line3d_track_list_.back().line_views_pnts_.emplace_back(std::make_pair(v1_line_s_pnt, v1_line_e_pnt));
					
					scenes_line3d_track_list_.back().lines_features_.push_back(
						HWLineFeatureReferenceId(view_2_id, idx.second));
					//get polygon idx from label idx
					int view2_line_polygon_idx = line_pnt_match.lines_matches[j].q_polygon_idx_[0];
					scenes_line3d_track_list_.back().polygon_idxs_.emplace_back(view2_line_polygon_idx);

					scenes_line3d_track_list_.back().is_valid_ = true;
					////get line 3d pnt
					//Eigen::Vector3f v2_line_s_pnt = line_pnt_match.lines_matches[j].world_q1_;
					//Eigen::Vector3f v2_line_e_pnt = line_pnt_match.lines_matches[j].world_q2_;
					//scenes_line3d_track_list_.back().line_views_pnts_.emplace_back(std::make_pair(v2_line_s_pnt, v2_line_e_pnt));
				}
				else if (view1_line_tid == -1 && view2_line_tid != -1)
				{
					/* Propagate track ID from first to second view. */
					viewport1.lines_track_ids_[idx.first] = view2_line_tid;
					//scenes_line3d_track_list_[]
					scenes_line3d_track_list_[view2_line_tid].lines_features_.push_back(
						HWLineFeatureReferenceId(view_1_id, idx.first));
					//get polygon idx from label idx
					int view1_line_polygon_idx = line_pnt_match.lines_matches[j].p_polygon_idx_[0];
					scenes_line3d_track_list_[view2_line_tid].polygon_idxs_.emplace_back(view1_line_polygon_idx);

					//scenes_line3d_track_list_.back().is_valid_ = true;
				}
				else if (view1_line_tid != -1 && view2_line_tid == -1)
				{
					/* Propagate track ID from second to first view. */
					viewport2.lines_track_ids_[idx.second] = view1_line_tid;
					scenes_line3d_track_list_[view1_line_tid].lines_features_.push_back(
						HWLineFeatureReferenceId(view_2_id, idx.second));
					//get polygon idx from label idx
					int view2_line_polygon_idx = line_pnt_match.lines_matches[j].q_polygon_idx_[0];
					scenes_line3d_track_list_[view1_line_tid].polygon_idxs_.emplace_back(view2_line_polygon_idx);

					//scenes_line3d_track_list_.back().is_valid_ = true;
				}
				else if (view1_line_tid == view2_line_tid)
				{
					/* Track ID already propagated. */
				}
				else
				{
					/*
					* A track ID is already associated with both ends of a match,
					* however, is not consistent. Unify tracks.
					*/
					//std::cerr << "to do next..." << std::endl;
					//unify_tracks(view1_tid, view2_tid, tracks, viewports);
					unify_new_lines_tracks_labels(view1_line_tid, view2_line_tid);
				}
			}
		}
		std::cerr << "end run track line 3d from lines pair match with labels..." << std::endl;

		//compute the polygon idx from lines_features_ label
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			//compute the line point 3d
			//std::cerr << "track line 3d idx: " << i << std::endl;	//
			HWLineTrack3D& const tmp_line3d_track = scenes_line3d_track_list_[i];
			//get line point 2d
			int lines_features_num = static_cast<int>(tmp_line3d_track.lines_features_.size());
			tmp_line3d_track.line_views_pnts_.resize(lines_features_num);
			//std::vector<int> tmp_line3d_track_line_polygon_idxs;
			std::map<int, int> polyidxs2counts;
			//compute tmp_line3d_track to polygon idxs(important)
			for (int j = 0; j < tmp_line3d_track.lines_features_.size(); ++j)
			{
				int line_to_polygon_idx = tmp_line3d_track.polygon_idxs_[j];
				if (line_to_polygon_idx != -1)
				{
					//scenes_line3d_track_list_[i].polygon_idxs_ line_views_pnts_
					std::map<int, int>::iterator idx_iter = polyidxs2counts.find(line_to_polygon_idx);
					if (idx_iter == polyidxs2counts.end())
					{
						polyidxs2counts[line_to_polygon_idx] = 1;
					}
					else
					{
						idx_iter->second += 1;
					}
				}
			}

			//sort MyPolygonIdxCmpZDG
			//std::sort(polyidxs2counts.begin(), polyidxs2counts.end(), MyPolygonIdxCampareZDG);
			std::vector<std::pair<int, int> > polyidxs2countsvec(polyidxs2counts.begin(), polyidxs2counts.end());
			std::sort(polyidxs2countsvec.begin(), polyidxs2countsvec.end(), MyPolygonIdxCmpZDG);
			//scenes_pnts3d_track_list_[i].pos_ = (fusion_pnt3d / view_features_num);
			std::vector<std::pair<int, int> >::iterator iter_poly = polyidxs2countsvec.begin();
			for (; iter_poly != polyidxs2countsvec.end(); ++iter_poly)
			{
				scenes_line3d_track_list_[i].polygon_idxs_.emplace_back(iter_poly->first);
			}
		}

		/*
		int view_id = tmp_line3d_track.lines_features_[j].view_id_;
		int line_id = tmp_line3d_track.lines_features_[j].line_id_;
		std::pair<Eigen::Vector2f, Eigen::Vector2f> view_line_pnts_pos = hw_cams_views_list_[view_id].lines_segments_[line_id];
		CameraModel tmp_cam = hw_cams_views_list_[view_id].view_pose_;
		Eigen::Vector3f tmp_s_pnt3d, tmp_e_pnt3d;
		//std::pair<Eigen::Vector2f, Eigen::Vector2f> view_line_pnts_pos = hw_cams_views_list_[view_id].lines_segments_[line_id];
		//get polygon idx

		std::pair<Eigen::Vector2f, Eigen::Vector2f> view_line_pnts_pos = hw_cams_views_list_[view_id].lines_segments_[line_id];
		if (line_to_polygon_idx != -1)
		{
		bool tmp_s_intersection = ImagePnt2dPosProj2ScenePickedPolygonPnt3dPos(tmp_cam,
		view_line_pnts_pos.first, line_to_polygon_idx, tmp_s_pnt3d);
		bool tmp_e_intersection = ImagePnt2dPosProj2ScenePickedPolygonPnt3dPos(tmp_cam,
		view_line_pnts_pos.second, line_to_polygon_idx, tmp_e_pnt3d);
		//
		if (tmp_s_intersection)
		{
		scenes_line3d_track_list_[i].line_views_pnts_[j].first = tmp_s_pnt3d;
		//scenes_line3d_track_list_[i].polygon_idxs_ line_views_pnts_
		std::map<int, int>::iterator idx_iter = polyidxs2counts.find(line_to_polygon_idx);
		if (idx_iter == polyidxs2counts.end())
		{
		polyidxs2counts[line_to_polygon_idx] = 1;
		}
		else
		{
		idx_iter->second += 1;
		}
		}
		else
		{

		}
		if (tmp_e_intersection)
		{
		}
		else
		{
		}
		}
		else
		{
		//do not assign polygon idx, just assign the intersect point
		//get the ray to polygon(it is wrong, when the camera is deviate from correct pose(ray to other polygon))
		int idx_sray_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tmp_cam, view_line_pnts_pos.first, tmp_s_pnt3d);
		int idx_eray_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tmp_cam, view_line_pnts_pos.second, tmp_e_pnt3d);

		}

		for (int j = 0; j < tmp_line3d_track.lines_features_.size(); ++j)
		{
		//std::cerr << "line 2d idx: " << j << std::endl;
		int view_id = tmp_line3d_track.lines_features_[j].view_id_;
		int line_id = tmp_line3d_track.lines_features_[j].line_id_;
		std::pair<Eigen::Vector2f, Eigen::Vector2f> view_line_pnts_pos = hw_cams_views_list_[view_id].lines_segments_[line_id];
		CameraModel tmp_cam = hw_cams_views_list_[view_id].view_pose_;
		Eigen::Vector3f tmp_s_pnt3d, tmp_e_pnt3d;
		//bool tmp_s_ray_flag = ImagePnt2dPosProj2ScenePolygonsPnt3dPos(tmp_cam, view_line_pnts_pos.first, tmp_s_pnt3d);
		//bool tmp_e_ray_flag = ImagePnt2dPosProj2ScenePolygonsPnt3dPos(tmp_cam, view_line_pnts_pos.second, tmp_e_pnt3d);

		//get the ray to polygon(it is wrong, when the camera is deviate from correct pose(ray to other polygon))
		int idx_sray_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tmp_cam, view_line_pnts_pos.first, tmp_s_pnt3d);
		int idx_eray_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tmp_cam, view_line_pnts_pos.second, tmp_e_pnt3d);

		if (idx_sray_intersect != -1)
		{
		scenes_line3d_track_list_[i].line_views_pnts_[j].first = tmp_s_pnt3d;
		//scenes_line3d_track_list_[i].polygon_idxs_ line_views_pnts_
		std::map<int, int>::iterator idx_iter = polyidxs2counts.find(idx_sray_intersect);
		if (idx_iter == polyidxs2counts.end())
		{
		polyidxs2counts[idx_sray_intersect] = 1;
		}
		else
		{
		idx_iter->second += 1;
		}
		}
		else
		{
		scenes_line3d_track_list_[i].line_views_pnts_[j].first = Eigen::Vector3f(
		std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
		std::numeric_limits<float>::quiet_NaN());
		}
		if (idx_eray_intersect != -1)
		{
		scenes_line3d_track_list_[i].line_views_pnts_[j].second = tmp_e_pnt3d;
		std::map<int, int>::iterator idx_iter = polyidxs2counts.find(idx_sray_intersect);
		if (idx_iter == polyidxs2counts.end())
		{
		polyidxs2counts[idx_sray_intersect] = 1;
		}
		else
		{
		idx_iter->second += 1;
		}
		}
		else
		{
		scenes_line3d_track_list_[i].line_views_pnts_[j].second = Eigen::Vector3f(
		std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
		std::numeric_limits<float>::quiet_NaN());
		}
		}

		*/

		std::cerr << "compute track line 3d point position..." << std::endl;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			//line start and end pnts
			HWLineTrack3D& const tmp_line3d_track = scenes_line3d_track_list_[i];
			//get line point 2d
			int lines_features_num = static_cast<int>(tmp_line3d_track.lines_features_.size());
			tmp_line3d_track.line_views_pnts_.resize(lines_features_num);
			for (int j = 0; j < lines_features_num; ++j)
			{
			}
		}
		std::cerr << "end track line 3d point position..." << std::endl;

		////test
		//std::cerr << "do compute the line tracks pnts 3d " << std::endl;
		////std::vector<Eigen::Vector3f> scene_test_lines_pnts;
		//std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> scene_test_views_lines_pnts;
		//for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		//{
		//	for (int j = 0; j < scenes_line3d_track_list_[i].line_views_pnts_.size(); ++j)
		//	{
		//		std::pair<Eigen::Vector3f, Eigen::Vector3f> line_pnt = scenes_line3d_track_list_[i].line_views_pnts_[j];
		//		scene_test_views_lines_pnts.emplace_back(line_pnt);
		//	}
		//}
		////std::string sc_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/scene_pnts_observation.obj";
		//std::string sc_view_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/scene_lines_views_pnts_observation.obj";
		////int scene_lines_pnts_num = static_cast<int>(scene_test_lines_pnts.size());
		//int scene_views_lines_pnts_num = static_cast<int>(scene_test_views_lines_pnts.size());
		////std::cerr << "scene lines number: " << scene_lines_pnts_num << std::endl;
		////WritePntsIntoObj(scene_test_lines_pnts, sc_path);
		//std::cerr << "scene view lines number: " << scene_views_lines_pnts_num << std::endl;
		//WriteLine3DIntoObj(scene_test_views_lines_pnts, sc_view_path);
		////end test

		std::cerr << "end track line 3d from lines pair match with labels..." << std::endl;
	}

	void HWScenesElements::RunTrackPnts3dListFromImgsFeaturesMatch()
	{
		std::cerr << "do run track pnt 3d from pair points match..." << std::endl;
		//std::cerr << "111111111111111111" << std::endl;
		for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			HWTwoViewsCamsMatching line_pnt_match = scenes_lines_pnts_matches_[i];
			int view_1_id = line_pnt_match.view_1_id;
			int view_2_id = line_pnt_match.view_2_id;
			//Viewport& viewport1 = viewports->at(tvm.view_1_id);
			//Viewport& viewport2 = viewports->at(tvm.view_2_id);
			HWCamViewport& const viewport1 = hw_cams_views_list_[view_1_id];
			HWCamViewport& const viewport2 = hw_cams_views_list_[view_2_id];
			//std::cerr << "i: " << i << std::endl;
			/* Iterate over matches for a pair. */
			for (std::size_t j = 0; j < line_pnt_match.points_matches.size(); ++j)
			{
				//line_pnt_match.points_matches[j].valid_match_; check them
				std::pair<int, int> idx = line_pnt_match.points_matches[j].pnts_matches_idx_;
				/*if (view_1_id == 0 && idx.first == 178)
				{
					std::cerr << "0, 178->" << view_2_id << ", " << idx.second << std::endl;
				}
				if (view_1_id == 0 && idx.first == 168)
				{
					std::cerr << "0, 168->" << view_2_id << ", " << idx.second << std::endl;
				}
				if (view_1_id == 0 && idx.first == 181)
				{
					std::cerr << "0, 181->" << view_2_id << ", " << idx.second << std::endl;
				}*/
				int const view1_pnt_tid = viewport1.track_ids_[idx.first];
				int const view2_pnt_tid = viewport2.track_ids_[idx.second];
				if (view1_pnt_tid == -1 && view2_pnt_tid == -1)
				{
					/* No track ID associated with the match. Create track. */
					viewport1.track_ids_[idx.first] = scenes_pnts3d_track_list_.size();
					viewport2.track_ids_[idx.second] = scenes_pnts3d_track_list_.size();
					scenes_pnts3d_track_list_.push_back(HWTrack3D());
					scenes_pnts3d_track_list_.back().features_.push_back(
						HWFeatureReferenceId(view_1_id, idx.first));
					scenes_pnts3d_track_list_.back().features_.push_back(
						HWFeatureReferenceId(view_2_id, idx.second));
					////pnt 3d to 3d, to do next
					//scenes_pnts3d_track_list_.back().pos_;
				}
				else if (view1_pnt_tid == -1 && view2_pnt_tid != -1)
				{
					/* Propagate track ID from first to second view. */
					viewport1.track_ids_[idx.first] = view2_pnt_tid;
					//scenes_line3d_track_list_[]
					scenes_pnts3d_track_list_[view2_pnt_tid].features_.push_back(
						HWFeatureReferenceId(view_1_id, idx.first));
				}
				else if (view1_pnt_tid != -1 && view2_pnt_tid == -1)
				{
					/* Propagate track ID from second to first view. */
					viewport2.track_ids_[idx.second] = view1_pnt_tid;
					scenes_pnts3d_track_list_[view1_pnt_tid].features_.push_back(
						HWFeatureReferenceId(view_2_id, idx.second));
				}
				else if (view1_pnt_tid == view2_pnt_tid)
				{
					/* Track ID already propagated. */
				}
				else
				{
					/*
					* A track ID is already associated with both ends of a match,
					* however, is not consistent. Unify tracks.
					*/
					//std::cerr << "to do next..." << std::endl;
					unify_new_pnts_tracks(view1_pnt_tid, view2_pnt_tid);
				}
			}
		}
		
		////test the track pnt list
		//std::cerr << "start to test the tracks pnts 3d..." << std::endl;
		//for (int i = 0; i < scenes_pnts3d_track_list_.size(); ++i)
		//{
		//	std::vector<int> views_lists;
		//	std::vector<Eigen::Vector2f> view_pnts2d;
		//	for (int j = 0; j < scenes_pnts3d_track_list_[i].features_.size(); ++j)
		//	{
		//		int src_view0_id = scenes_pnts3d_track_list_[i].features_[j].view_id_;
		//		views_lists.emplace_back(src_view0_id);
		//		int src_view0_pnt_id = scenes_pnts3d_track_list_[i].features_[j].feature_id_;
		//		Eigen::Vector2f src_view0_pnt = hw_cams_views_list_[src_view0_id].features_poistion_[src_view0_pnt_id];
		//		view_pnts2d.emplace_back(src_view0_pnt);
		//		std::cerr << "src_view0_id, src_view0_pnt_id: " << src_view0_id << ", " << src_view0_pnt_id << std::endl;
		//	}
		//	int views_list_num = static_cast<int>(views_lists.size());
		//	std::cerr << "views_list_num: " << views_list_num << std::endl;
		//	if (views_list_num >= 3)
		//	{
		//		int v0 = views_lists[0];
		//		int v1 = views_lists[1];
		//		int v2 = views_lists[2];
		//		Eigen::Vector2f p0 = view_pnts2d[0];
		//		Eigen::Vector2f p1 = view_pnts2d[1];
		//		Eigen::Vector2f p2 = view_pnts2d[2];
		//		std::pair<Eigen::Vector2f, Eigen::Vector2f> pair0_pnts = std::make_pair(p0, p1);
		//		std::pair<Eigen::Vector2f, Eigen::Vector2f> pair1_pnts = std::make_pair(p0, p2);
		//		cv::Mat fusion0_mat;
		//		DrawOnePairPntsMatchedIntoMat(v0, v1, pair0_pnts, fusion0_mat);
		//		cv::Mat fusion1_mat;
		//		DrawOnePairPntsMatchedIntoMat(v0, v2, pair1_pnts, fusion1_mat);
		//		std::string v0v1_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/v0v1.png";
		//		std::string v0v2_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/v0v2.png";
		//		cv::imwrite(v0v1_path, fusion0_mat);
		//		cv::imwrite(v0v2_path, fusion1_mat);
		//		//return;
		//	}
		//}
		//std::cerr << "end test the tracks pnts 3d..." << std::endl;
		////end test the track pnt list

		std::cerr << "do compute the tracks pnts 3d " << std::endl;
		std::vector<Eigen::Vector3f> scene_pnts;
		std::vector<Eigen::Vector3f> scene_views_pnts;
		for (int i = 0; i < scenes_pnts3d_track_list_.size(); ++i)
		{
			std::cerr << "i: " << i << std::endl;
			//compute the point 3d
			HWTrack3D tmp_pnt3d_track = scenes_pnts3d_track_list_[i];
			int view_features_num = 0;
			Eigen::Vector3f fusion_pnt3d = Eigen::Vector3f(0.0, 0.0, 0.0);
			//get view point 2d
			//scenes_pnts3d_track_list_[i].features_.size();
			int pnts_features_num = static_cast<int>(tmp_pnt3d_track.features_.size());
			scenes_pnts3d_track_list_[i].views_features_pnts3d_.resize(pnts_features_num);

			std::map<int, int> polyidxs2counts;
			for (int j = 0; j < scenes_pnts3d_track_list_[i].features_.size(); ++j)
			{
				int view_id = scenes_pnts3d_track_list_[i].features_[j].view_id_;
				int pnt_id = scenes_pnts3d_track_list_[i].features_[j].feature_id_;
				Eigen::Vector2f view_pnt_pos = hw_cams_views_list_[view_id].features_poistion_[pnt_id];
				CameraModel tmp_cam = hw_cams_views_list_[view_id].view_pose_;
				Eigen::Vector3f tmp_pnt3d;
				int idx_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tmp_cam, view_pnt_pos, tmp_pnt3d);
				//if (ImagePnt2dPosProj2ScenePolygonsPnt3dPos(tmp_cam, view_pnt_pos, tmp_pnt3d))
				if(idx_intersect != -1)
				{
					fusion_pnt3d += tmp_pnt3d;
					++view_features_num;
					std::map<int, int>::iterator idx_iter = polyidxs2counts.find(idx_intersect);
					if (idx_iter == polyidxs2counts.end())
					{
						polyidxs2counts[idx_intersect] = 1;
					}
					else
					{
						idx_iter->second += 1;
					}
					//polyidxs2counts[idx_intersect] = 1;
					//polyidxs2counts[idx_intersect] = polyidxs2counts[idx_intersect] + 1;
					scene_views_pnts.emplace_back(tmp_pnt3d);	//test
					scenes_pnts3d_track_list_[i].views_features_pnts3d_[j] = tmp_pnt3d;	//get view pnts 3d
				}
				else
				{
					scenes_pnts3d_track_list_[i].views_features_pnts3d_[j] = Eigen::Vector3f(
						std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
						std::numeric_limits<float>::quiet_NaN());
				}
			}
			if (view_features_num == 0)
			{
				scenes_pnts3d_track_list_[i].pos_ = fusion_pnt3d;
			}
			else
			{
				//sort MyPolygonIdxCmpZDG
				//std::sort(polyidxs2counts.begin(), polyidxs2counts.end(), MyPolygonIdxCampareZDG);
				std::vector<std::pair<int, int> > polyidxs2countsvec(polyidxs2counts.begin(), polyidxs2counts.end());
				std::sort(polyidxs2countsvec.begin(), polyidxs2countsvec.end(), MyPolygonIdxCmpZDG);
				scenes_pnts3d_track_list_[i].pos_ = (fusion_pnt3d / view_features_num);
				std::vector<std::pair<int, int> >::iterator iter_poly = polyidxs2countsvec.begin();
				for (; iter_poly != polyidxs2countsvec.end(); ++iter_poly)
				{
					scenes_pnts3d_track_list_[i].polygon_idxs_.emplace_back(iter_poly->first);
				}
			}
			//test
			scene_pnts.emplace_back(scenes_pnts3d_track_list_[i].pos_);
			//end test
		}

		////test
		//std::string sc_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/scene_pnts_observation.obj";
		//std::string sc_view_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/scene_views_pnts_observation.obj";
		//int scene_pnts_num = static_cast<int>(scene_pnts.size());
		//int scene_views_pnts_num = static_cast<int>(scene_views_pnts.size());
		//std::cerr << "scene points number: " << scene_pnts_num << std::endl;
		//WritePntsIntoObj(scene_pnts, sc_path);
		//std::cerr << "scene view points number: " << scene_views_pnts_num << std::endl;
		//WritePntsIntoObj(scene_views_pnts, sc_view_path);
		////end test
		std::cerr << "end track pnt 3d from pair points match..." << std::endl;
	}

	void HWScenesElements::RunTrackPnts3dListFromImgsFeaturesMatchFilter()
	{
		std::cerr << "do run track pnt 3d from pair points match(some thing wrong)..." << std::endl;
		//std::cerr << "111111111111111111" << std::endl;
		for (int i = 0; i < scenes_lines_pnts_matches_.size(); ++i)
		{
			HWTwoViewsCamsMatching line_pnt_match = scenes_lines_pnts_matches_[i];
			int view_1_id = line_pnt_match.view_1_id;
			int view_2_id = line_pnt_match.view_2_id;
			//Viewport& viewport1 = viewports->at(tvm.view_1_id);
			//Viewport& viewport2 = viewports->at(tvm.view_2_id);
			HWCamViewport& const viewport1 = hw_cams_views_list_[view_1_id];
			HWCamViewport& const viewport2 = hw_cams_views_list_[view_2_id];
			//std::cerr << "i: " << i << std::endl;
			/* Iterate over matches for a pair. */
			for (std::size_t j = 0; j < line_pnt_match.points_matches.size(); ++j)
			{
				//line_pnt_match.points_matches[j].valid_match_; check them
				if (!line_pnt_match.points_matches[j].valid_match_)
				{
					continue;
				}
				std::pair<int, int> idx = line_pnt_match.points_matches[j].pnts_matches_idx_;
				/*if (view_1_id == 0 && idx.first == 178)
				{
				std::cerr << "0, 178->" << view_2_id << ", " << idx.second << std::endl;
				}
				if (view_1_id == 0 && idx.first == 168)
				{
				std::cerr << "0, 168->" << view_2_id << ", " << idx.second << std::endl;
				}
				if (view_1_id == 0 && idx.first == 181)
				{
				std::cerr << "0, 181->" << view_2_id << ", " << idx.second << std::endl;
				}*/
				int const view1_pnt_tid = viewport1.track_ids_[idx.first];
				int const view2_pnt_tid = viewport2.track_ids_[idx.second];
				if (view1_pnt_tid == -1 && view2_pnt_tid == -1)
				{
					/* No track ID associated with the match. Create track. */
					viewport1.track_ids_[idx.first] = scenes_pnts3d_track_list_.size();
					viewport2.track_ids_[idx.second] = scenes_pnts3d_track_list_.size();
					scenes_pnts3d_track_list_.push_back(HWTrack3D());
					scenes_pnts3d_track_list_.back().features_.push_back(
						HWFeatureReferenceId(view_1_id, idx.first));
					scenes_pnts3d_track_list_.back().features_.push_back(
						HWFeatureReferenceId(view_2_id, idx.second));
					////pnt 3d to 3d, to do next
					//scenes_pnts3d_track_list_.back().pos_;
				}
				else if (view1_pnt_tid == -1 && view2_pnt_tid != -1)
				{
					/* Propagate track ID from first to second view. */
					viewport1.track_ids_[idx.first] = view2_pnt_tid;
					//scenes_line3d_track_list_[]
					scenes_pnts3d_track_list_[view2_pnt_tid].features_.push_back(
						HWFeatureReferenceId(view_1_id, idx.first));
				}
				else if (view1_pnt_tid != -1 && view2_pnt_tid == -1)
				{
					/* Propagate track ID from second to first view. */
					viewport2.track_ids_[idx.second] = view1_pnt_tid;
					scenes_pnts3d_track_list_[view1_pnt_tid].features_.push_back(
						HWFeatureReferenceId(view_2_id, idx.second));
				}
				else if (view1_pnt_tid == view2_pnt_tid)
				{
					/* Track ID already propagated. */
				}
				else
				{
					/*
					* A track ID is already associated with both ends of a match,
					* however, is not consistent. Unify tracks.
					*/
					//std::cerr << "to do next..." << std::endl;
					unify_new_pnts_tracks(view1_pnt_tid, view2_pnt_tid);
				}
			}
		}

		////test the track pnt list
		//std::cerr << "start to test the tracks pnts 3d..." << std::endl;
		//for (int i = 0; i < scenes_pnts3d_track_list_.size(); ++i)
		//{
		//	std::vector<int> views_lists;
		//	std::vector<Eigen::Vector2f> view_pnts2d;
		//	for (int j = 0; j < scenes_pnts3d_track_list_[i].features_.size(); ++j)
		//	{
		//		int src_view0_id = scenes_pnts3d_track_list_[i].features_[j].view_id_;
		//		views_lists.emplace_back(src_view0_id);
		//		int src_view0_pnt_id = scenes_pnts3d_track_list_[i].features_[j].feature_id_;
		//		Eigen::Vector2f src_view0_pnt = hw_cams_views_list_[src_view0_id].features_poistion_[src_view0_pnt_id];
		//		view_pnts2d.emplace_back(src_view0_pnt);
		//		std::cerr << "src_view0_id, src_view0_pnt_id: " << src_view0_id << ", " << src_view0_pnt_id << std::endl;
		//	}
		//	int views_list_num = static_cast<int>(views_lists.size());
		//	std::cerr << "views_list_num: " << views_list_num << std::endl;
		//	if (views_list_num >= 3)
		//	{
		//		int v0 = views_lists[0];
		//		int v1 = views_lists[1];
		//		int v2 = views_lists[2];
		//		Eigen::Vector2f p0 = view_pnts2d[0];
		//		Eigen::Vector2f p1 = view_pnts2d[1];
		//		Eigen::Vector2f p2 = view_pnts2d[2];
		//		std::pair<Eigen::Vector2f, Eigen::Vector2f> pair0_pnts = std::make_pair(p0, p1);
		//		std::pair<Eigen::Vector2f, Eigen::Vector2f> pair1_pnts = std::make_pair(p0, p2);
		//		cv::Mat fusion0_mat;
		//		DrawOnePairPntsMatchedIntoMat(v0, v1, pair0_pnts, fusion0_mat);
		//		cv::Mat fusion1_mat;
		//		DrawOnePairPntsMatchedIntoMat(v0, v2, pair1_pnts, fusion1_mat);
		//		std::string v0v1_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/v0v1.png";
		//		std::string v0v2_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images/v0v2.png";
		//		cv::imwrite(v0v1_path, fusion0_mat);
		//		cv::imwrite(v0v2_path, fusion1_mat);
		//		//return;
		//	}
		//}
		//std::cerr << "end test the tracks pnts 3d..." << std::endl;
		////end test the track pnt list

		std::cerr << "do compute the tracks pnts 3d " << std::endl;
		std::vector<Eigen::Vector3f> scene_pnts;
		std::vector<Eigen::Vector3f> scene_views_pnts;
		for (int i = 0; i < scenes_pnts3d_track_list_.size(); ++i)
		{
			//compute the point 3d
			HWTrack3D tmp_pnt3d_track = scenes_pnts3d_track_list_[i];
			int view_features_num = 0;
			Eigen::Vector3f fusion_pnt3d = Eigen::Vector3f(0.0, 0.0, 0.0);
			//get view point 2d
			//scenes_pnts3d_track_list_[i].features_.size();
			int pnts_features_num = static_cast<int>(tmp_pnt3d_track.features_.size());
			scenes_pnts3d_track_list_[i].views_features_pnts3d_.resize(pnts_features_num);	//get view point 2d
			scenes_pnts3d_track_list_[i].pnts_views_to_polygon_idxs_.resize(pnts_features_num, -1); //initialize the view pnts -1
			//scenes_pnts3d_track_list_[i].features_.size();
			std::map<int, int> polyidxs2counts;
			for (int j = 0; j < scenes_pnts3d_track_list_[i].features_.size(); ++j)
			{
				int view_id = scenes_pnts3d_track_list_[i].features_[j].view_id_;
				int pnt_id = scenes_pnts3d_track_list_[i].features_[j].feature_id_;
				Eigen::Vector2f view_pnt_pos = hw_cams_views_list_[view_id].features_poistion_[pnt_id];
				CameraModel tmp_cam = hw_cams_views_list_[view_id].view_pose_;
				Eigen::Vector3f tmp_pnt3d;
				int idx_intersect = ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(tmp_cam, view_pnt_pos, tmp_pnt3d);
				//if (ImagePnt2dPosProj2ScenePolygonsPnt3dPos(tmp_cam, view_pnt_pos, tmp_pnt3d))
				if (idx_intersect != -1)
				{
					fusion_pnt3d += tmp_pnt3d;
					++view_features_num;
					std::map<int, int>::iterator idx_iter = polyidxs2counts.find(idx_intersect);
					if (idx_iter == polyidxs2counts.end())
					{
						polyidxs2counts[idx_intersect] = 1;
					}
					else
					{
						idx_iter->second += 1;
					}
					//polyidxs2counts[idx_intersect] = 1;
					//polyidxs2counts[idx_intersect] = polyidxs2counts[idx_intersect] + 1;
					scene_views_pnts.emplace_back(tmp_pnt3d);	//test
					scenes_pnts3d_track_list_[i].views_features_pnts3d_[j] = tmp_pnt3d;	//get view pnts 3d
					scenes_pnts3d_track_list_[i].pnts_views_to_polygon_idxs_[j] = idx_intersect;
				}
				else
				{
					scenes_pnts3d_track_list_[i].views_features_pnts3d_[j] = Eigen::Vector3f(
						std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
						std::numeric_limits<float>::quiet_NaN());
					scenes_pnts3d_track_list_[i].pnts_views_to_polygon_idxs_[j] = -1;
				}
			}
			if (view_features_num == 0)
			{
				scenes_pnts3d_track_list_[i].pos_ = fusion_pnt3d;
				scenes_pnts3d_track_list_[i].polygon_idxs_.clear();
				scenes_pnts3d_track_list_[i].is_valid_ = false;
			}
			else
			{
				//sort MyPolygonIdxCmpZDG
				//std::sort(polyidxs2counts.begin(), polyidxs2counts.end(), MyPolygonIdxCampareZDG);
				std::vector<std::pair<int, int> > polyidxs2countsvec(polyidxs2counts.begin(), polyidxs2counts.end());
				std::sort(polyidxs2countsvec.begin(), polyidxs2countsvec.end(), MyPolygonIdxCmpZDG);
				scenes_pnts3d_track_list_[i].pos_ = (fusion_pnt3d / view_features_num);
				std::vector<std::pair<int, int> >::iterator iter_poly = polyidxs2countsvec.begin();
				for (; iter_poly != polyidxs2countsvec.end(); ++iter_poly)
				{
					scenes_pnts3d_track_list_[i].polygon_idxs_.emplace_back(iter_poly->first);
				}
			}
			//test
			scene_pnts.emplace_back(scenes_pnts3d_track_list_[i].pos_);
			//end test
		}

		////test the scenes_pnts3d_tracklist info
		//std::cerr << "--------------test scenes_pnts3d_track_list_-------------------" << std::endl;
		//for (int i = 0; i < scenes_pnts3d_track_list_.size(); ++i)
		//{
		//	std::cerr << "---------------------------------" << std::endl;
		//	for (int j = 0; j < scenes_pnts3d_track_list_[i].features_.size(); ++j)
		//	{
		//		std::cerr << "(" << scenes_pnts3d_track_list_[i].features_[j].view_id_ << " "
		//			<< scenes_pnts3d_track_list_[i].features_[j].feature_id_ << ")  ";
		//	}
		//	std::cerr << std::endl;
		//	std::cerr << "---------------------------------" << std::endl;
		//}
		//std::cerr << "-----------------end scenes_pnts3d_track_list_----------------" << std::endl;
		//end test the scenes_pnts3d_tracklist info

		////test
		//std::string sc_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part5_test/line_test_obj/views/scene_pnts_observation_before.obj";
		//std::string sc_view_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part5_test/line_test_obj/views/scene_views_pnts_observation_before.obj";
		//int scene_pnts_num = static_cast<int>(scene_pnts.size());
		//int scene_views_pnts_num = static_cast<int>(scene_views_pnts.size());
		//std::cerr << "scene points number: " << scene_pnts_num << std::endl;
		//WritePntsIntoObj(scene_pnts, sc_path);
		//std::cerr << "scene view points number: " << scene_views_pnts_num << std::endl;
		//WritePntsIntoObj(scene_views_pnts, sc_view_path);
		////end test

		std::cerr << "end track pnt 3d from pair points match..." << std::endl;
	}

	void HWScenesElements::FilterGroupedTrackPnts3dListNotInPolygons()
	{
		for (int i = 0; i < scenes_pnts3d_track_list_.size(); ++i)
		{
			if (scenes_pnts3d_track_list_[i].polygon_idxs_.empty())
			{
				scenes_pnts3d_track_list_[i].is_valid_ = false;
			}
			else
			{
				if (scenes_pnts3d_track_list_[i].polygon_idxs_[0] == -1)
				{
					scenes_pnts3d_track_list_[i].is_valid_ = false;
				}
			}
		}
	}

	void HWScenesElements::FilterDuplicatedTrackPnts3dListFromDuplicatedViewsId()
	{
		for (int i = 0; i < scenes_pnts3d_track_list_.size(); ++i)
		{
			std::vector<int> pnt3d_track_list_views_idxs;
			for (int j = 0; j < scenes_pnts3d_track_list_[i].features_.size(); ++j)
			{
				pnt3d_track_list_views_idxs.emplace_back(scenes_pnts3d_track_list_[i].features_[j].view_id_);
			}
			if (ElementVectorContainsDuplicatedElement(pnt3d_track_list_views_idxs))
			{
				scenes_pnts3d_track_list_[i].is_valid_ = false;
			}
		}
	}

	void HWScenesElements::RunTrackPnts3dListFromImgsFeaturesMatchWithLabels()
	{
		std::cerr << "to do next..." << std::endl;
	}

	void HWScenesElements::RunTrackPnts3dToPnts2Pnts3dObservations()
	{
		std::cerr << "run track points 3d into Pnts3d Observations..." << std::endl;
		//convert data to HWSurveyPoint3d2Pnts2dList
		scenes_pnt3ds_observations_.clear();
		for (int i = 0; i < scenes_pnts3d_track_list_.size(); ++i)
		{
			if (!scenes_pnts3d_track_list_[i].is_valid_)
			{
				continue;
			}
			HWSurveyPoint3d2Pnts2d tmp_pnt3d_observation;
			Eigen::Vector3f tmp_pnt3d = scenes_pnts3d_track_list_[i].pos_;
			tmp_pnt3d_observation.pos_ = tmp_pnt3d;
			tmp_pnt3d_observation.plane_idxs_ = scenes_pnts3d_track_list_[i].polygon_idxs_;
			int pnt3d_oberservation_num = scenes_pnts3d_track_list_[i].features_.size();
			tmp_pnt3d_observation.observations_.resize(pnt3d_oberservation_num);
			for (int j = 0; j < scenes_pnts3d_track_list_[i].features_.size(); ++j)
			{
				int tmp_view_id = scenes_pnts3d_track_list_[i].features_[j].view_id_;
				tmp_pnt3d_observation.observations_[j].view_id_ = tmp_view_id;
				int tpm_pnt_id = scenes_pnts3d_track_list_[i].features_[j].feature_id_;
				tmp_pnt3d_observation.observations_[j].pos_ = hw_cams_views_list_[tmp_view_id].features_poistion_[tpm_pnt_id];
				tmp_pnt3d_observation.observations_[j].plane_idxs_ = scenes_pnts3d_track_list_[i].polygon_idxs_;
				//check it
				if (IsHWPoint3dValidZDG(scenes_pnts3d_track_list_[i].views_features_pnts3d_[j]))
				{
					tmp_pnt3d_observation.observations_[j].is_valid_ = true;
				}
				else
				{
					tmp_pnt3d_observation.observations_[j].is_valid_ = false;
				}
				tmp_pnt3d_observation.observations_[j].view_pos_3d_ = scenes_pnts3d_track_list_[i].views_features_pnts3d_[j];	//get initial view point 3d
			}
			scenes_pnt3ds_observations_.emplace_back(tmp_pnt3d_observation);
		}
		std::cerr << "end track points 3d into Pnts3d Observations..." << std::endl;
	}

	void HWScenesElements::RunTrackPnts3dToPnts2Pnts3dObservationsNoPolygonInterLine()
	{
		std::cerr << "run track points 3d into Pnts3d Observations NoPolygonInterLine..." << std::endl;
		//convert data to HWSurveyPoint3d2Pnts2dList
		scenes_pnt3ds_observations_.clear();
		for (int i = 0; i < scenes_pnts3d_track_list_.size(); ++i)
		{
			if (!scenes_pnts3d_track_list_[i].is_valid_)
			{
				continue;
			}
			HWSurveyPoint3d2Pnts2d tmp_pnt3d_observation;
			Eigen::Vector3f tmp_pnt3d = scenes_pnts3d_track_list_[i].pos_;
			tmp_pnt3d_observation.pos_ = tmp_pnt3d;
			tmp_pnt3d_observation.plane_idxs_ = scenes_pnts3d_track_list_[i].polygon_idxs_;
			int pnt3d_oberservation_num = scenes_pnts3d_track_list_[i].features_.size();
			tmp_pnt3d_observation.observations_.resize(pnt3d_oberservation_num);
			for (int j = 0; j < scenes_pnts3d_track_list_[i].features_.size(); ++j)
			{
				int tmp_view_id = scenes_pnts3d_track_list_[i].features_[j].view_id_;
				tmp_pnt3d_observation.observations_[j].view_id_ = tmp_view_id;
				int tpm_pnt_id = scenes_pnts3d_track_list_[i].features_[j].feature_id_;
				tmp_pnt3d_observation.observations_[j].pos_ = hw_cams_views_list_[tmp_view_id].features_poistion_[tpm_pnt_id];
				tmp_pnt3d_observation.observations_[j].plane_idxs_ = scenes_pnts3d_track_list_[i].polygon_idxs_;	//same plane idxs
				//check it
				if (IsHWPoint3dValidZDG(scenes_pnts3d_track_list_[i].views_features_pnts3d_[j]))
				{
					tmp_pnt3d_observation.observations_[j].is_valid_ = true;
				}
				else
				{
					tmp_pnt3d_observation.observations_[j].is_valid_ = false;
				}
				tmp_pnt3d_observation.observations_[j].view_pos_3d_ = scenes_pnts3d_track_list_[i].views_features_pnts3d_[j];	//get initial view point 3d
			}
			scenes_pnt3ds_observations_.emplace_back(tmp_pnt3d_observation);
		}

		////test the points 3d observation image pairs
		//std::cerr << "---------view the scenes_pnt3ds_observations_----------" << std::endl;
		//for (int i = 0; i < scenes_pnt3ds_observations_.size(); ++i)
		//{
		//	std::cerr << "------------------------------------------" << std::endl;
		//	for (int j = 0; j < scenes_pnt3ds_observations_[i].observations_.size(); ++j)
		//	{
		//		int src_camid = scenes_pnt3ds_observations_[i].observations_[j].view_id_;
		//		std::cerr << src_camid << " ";
		//	}
		//	std::cerr << std::endl;
		//	std::cerr << "-------------------------------------------" << std::endl;
		//}
		//std::cerr << "---------view the scenes_pnt3ds_observations_----------" << std::endl;
		////end test the points 3d observation image pairs

		std::cerr << "end track points 3d into Pnts3d Observations NoPolygonInterLine..." << std::endl;
	}

	void HWScenesElements::FilterDuplicatedTrackLines3dListFromDuplicatedViewsId()
	{
		//std::cerr << "to do next..." << std::endl;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			std::vector<int> line3d_track_list_views_idxs;
			for (int j = 0; j < scenes_line3d_track_list_[i].lines_features_.size(); ++j)
			{
				line3d_track_list_views_idxs.emplace_back(scenes_line3d_track_list_[i].lines_features_[j].view_id_);
			}
			if (ElementVectorContainsDuplicatedElement(line3d_track_list_views_idxs))
			{
				scenes_line3d_track_list_[i].is_valid_ = false;
			}
		}
	}

	void HWScenesElements::GroupTrackLines3dListBySameGroupsIds()
	{
		scenes_line3d_grouped_track_list_.clear();
		//std::map<int, std::vector<int> > 
		std::vector<bool> scenes_lines3d_visited;
		scenes_lines3d_visited.resize(scenes_line3d_track_list_.size(), false);
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			//std::cerr << "scenes_line3d_track_list_ i: " << i << std::endl;
			/*std::cerr << "scenes_line3d_track_list_ i: " << i << std::endl;
			if (scenes_line3d_track_list_[i].is_valid_)
			{
				std::cerr << "i: " << i <<" true" << std::endl;
			}*/
			if (!scenes_line3d_track_list_[i].is_valid_)
			{
				std::cerr << "invalid i: " << i << std::endl;
				scenes_lines3d_visited[i] = true;
				continue;
			}
			if (scenes_lines3d_visited[i])
			{
				continue;
			}
			HWLineTrack3D tmp_track_list = scenes_line3d_track_list_[i];
			//std::cerr << "find_idx before " << std::endl;
			int find_idx = CheckMyTrackLine3dInGroupedTrackLine3d(tmp_track_list);
			scenes_lines3d_visited[i] = true;
			//std::cerr << "find_idx: " << find_idx << std::endl;
			if (find_idx != -1)
			{
				/*std::cerr << "111111111" << std::endl;
				if (i == 2691)
				{
					std::cerr << "scenes_line3d_grouped_track_list_ num: " << scenes_line3d_grouped_track_list_.size() << std::endl;
					std::cerr << "find_idx: " << find_idx << std::endl;
					std::cerr << "33333333 find_idx: " << std::endl;
					if (tmp_track_list.polygon_idxs_.empty())
					{
						std::cerr << "tmp_track_list polygon_idxs_.empty " << std::endl;
					}
					else
					{
						std::cerr << "tmp_track_list polygon_idxs_: " << tmp_track_list.polygon_idxs_[0] << std::endl;
					}
					if (tmp_track_list.polygon_idxs_.empty())
					{
						std::cerr << "scenes_line3d_grouped_track_list_[find_idx].polygon_idxs_ empty " << std::endl;
					}
					else
					{
						std::cerr << "scenes_line3d_grouped_track_list_[find_idx].polygon_idxs_: " << scenes_line3d_grouped_track_list_[find_idx].polygon_idxs_[0] << std::endl;
					}
				}*/
				
				if (scenes_line3d_grouped_track_list_[find_idx].polygon_idxs_[0] ==
					tmp_track_list.polygon_idxs_[0])
				{
					/*if (i == 2691)
					{
						std::cerr << "33333333 num: " << std::endl;
					}*/
					//in same polygon, assign the HWLineTrack3D to HWGroupedLineTrack3D
					AddAllHWLine3dToHWGroupedLinePart(tmp_track_list, scenes_line3d_grouped_track_list_[find_idx]);
					scenes_line3d_grouped_track_list_[find_idx].id_to_hwline_track3d_idx_.emplace_back(i);
				}
				else
				{
					/*if (i == 2691)
					{
						std::cerr << "444444444 num: " << std::endl;
					}*/
					//not in same polygon, new one
					HWGroupedLineTrack3D tmp_grouped_line3d;
					ConvertAllHWLine3dToHWGroupedLinePart(tmp_track_list, tmp_grouped_line3d);
					tmp_grouped_line3d.id_to_hwline_track3d_idx_.emplace_back(i);
					scenes_line3d_grouped_track_list_.emplace_back(tmp_grouped_line3d);
				}
			}
			else
			{
				//std::cerr << "22222222222" << std::endl;
				//not find its, new one
				HWGroupedLineTrack3D tmp_grouped_line3d;
				ConvertAllHWLine3dToHWGroupedLinePart(tmp_track_list, tmp_grouped_line3d);
				tmp_grouped_line3d.id_to_hwline_track3d_idx_.emplace_back(i);
				scenes_line3d_grouped_track_list_.emplace_back(tmp_grouped_line3d);				
				/*if (num_grouped_track == 2191)
				{
					int num_grouped_track = static_cast<int>(scenes_line3d_grouped_track_list_.size());
					if (tmp_grouped_line3d.polygon_idxs_.empty())
					{
						std::cerr << "some thing empty..." << std::endl;
					}
					else
					{
						std::cerr << "tmp_grouped_line3d polygon_idxs_: " << tmp_grouped_line3d.polygon_idxs_[0] << std::endl;
					}
				}*/
			}
		}
	}

	int HWScenesElements::CheckMyTrackLine3dInGroupedTrackLine3d(const HWLineTrack3D& tmp_track_line)
	{
		int find_grouped_idx = -1;
		for (int i = 0; i < scenes_line3d_grouped_track_list_.size(); ++i)
		{
			if (IsHWLine3dInHWGroupedLine3d(tmp_track_line, scenes_line3d_grouped_track_list_[i]))
			{
				find_grouped_idx = i;
				break;
			}
		}
		return find_grouped_idx;
	}

	bool HWScenesElements::IsHWLine3dInHWGroupedLine3d(const HWLineTrack3D& tmp_track_line, const HWGroupedLineTrack3D& grouped_track_line)
	{
		for (int i = 0; i < tmp_track_line.lines_features_.size(); ++i)
		{
			HWLineFeatureReferenceId tmp_line_reference_id = tmp_track_line.lines_features_[i];
			for (int j = 0; j < grouped_track_line.lines_features_.size(); ++j)
			{
				HWLineFeatureReferenceId tmp_grouped_line_reference_id 
					= grouped_track_line.lines_features_[j];
				if (IsTwoViewLineRefereneIdsInSameViewGroupedLine(tmp_line_reference_id, tmp_grouped_line_reference_id))
				{
					return true;
				}
			}
		}
		return false;
	}

	bool HWScenesElements::IsTwoViewLineRefereneIdsInSameViewGroupedLine(const HWLineFeatureReferenceId& hw_line_ref0,
		const HWLineFeatureReferenceId& hw_line_ref1)
	{
		if (hw_line_ref0.view_id_ == hw_line_ref1.view_id_
			&&hw_line_ref0.line_group_id == hw_line_ref1.line_group_id)
		{
			if (hw_line_ref0.line_group_id != -1)
			{
				return true;
			}
		}
		return false;
	}

	void HWScenesElements::ConvertAllHWLine3dToHWGroupedLinePart(const HWLineTrack3D& tmp_track_line, HWGroupedLineTrack3D& grouped_track_line)
	{
		grouped_track_line.is_valid_ = tmp_track_line.is_valid_;
		grouped_track_line.color_ = tmp_track_line.color_;
		grouped_track_line.s_pos_ = tmp_track_line.s_pos_;	//line start pnt
		grouped_track_line.e_pos_ = tmp_track_line.e_pos_;	//line end pnt
		grouped_track_line.polygon_idxs_ = tmp_track_line.polygon_idxs_;
		grouped_track_line.polygon_intersect_line_idx_ = tmp_track_line.polygon_intersect_line_idx_;
		grouped_track_line.lines_features_ = tmp_track_line.lines_features_;	//
		//line track list (one view line 2d segment to one view line point),
		grouped_track_line.line_views_pnts_ = tmp_track_line.line_views_pnts_;
		grouped_track_line.line_views_to_polygon_idxs_ = tmp_track_line.line_views_to_polygon_idxs_;
		grouped_track_line.view_line_polygon_inter_line_idxs_ = tmp_track_line.view_line_polygon_inter_line_idxs_;
	}

	void HWScenesElements::AddAllHWLine3dToHWGroupedLinePart(const HWLineTrack3D& tmp_track_line, HWGroupedLineTrack3D& grouped_track_line)
	{
		for (int i = 0; i < tmp_track_line.polygon_idxs_.size(); ++i)
		{
			grouped_track_line.polygon_idxs_.emplace_back(tmp_track_line.polygon_idxs_[i]);
		}
		for (int i = 0; i < tmp_track_line.lines_features_.size(); ++i)
		{
			grouped_track_line.lines_features_.emplace_back(tmp_track_line.lines_features_[i]);
		}
		for (int i = 0; i < tmp_track_line.line_views_pnts_.size(); ++i)
		{
			grouped_track_line.line_views_pnts_.emplace_back(tmp_track_line.line_views_pnts_[i]);
		}
		for (int i = 0; i < tmp_track_line.line_views_to_polygon_idxs_.size(); ++i)
		{
			grouped_track_line.line_views_to_polygon_idxs_.emplace_back(tmp_track_line.line_views_to_polygon_idxs_[i]);
		}
		for (int i = 0; i < tmp_track_line.view_line_polygon_inter_line_idxs_.size(); ++i)
		{
			grouped_track_line.view_line_polygon_inter_line_idxs_.emplace_back(tmp_track_line.view_line_polygon_inter_line_idxs_[i]);
		}
	}

	void HWScenesElements::SeprateTrackLines3dListByDifferentPolygonsIds()
	{
		std::cerr << "to do next..." << std::endl;
	}

	void HWScenesElements::ReComputeSceneLinesGroupedTracks3dList3dLinePosByPolygonsIds()
	{
		if (associated_polygons_.empty())
		{
			std::cerr << "ReComputeSceneLinesGrouped: no scene polygons existed..." << std::endl;
			return;
		}
		std::cerr << "ReComputeSceneLinesGroupedTracks3dList3dLinePosByPolygonsIds to do next..." << std::endl;
		for (int i = 0; i < scenes_line3d_grouped_track_list_.size(); ++i)
		{
			if (!scenes_line3d_grouped_track_list_[i].is_valid_)
			{
				continue;
			}
			if (scenes_line3d_grouped_track_list_[i].polygon_idxs_.empty())
			{
				continue;
			}
			int line3d_track_polygon_idx = scenes_line3d_grouped_track_list_[i].polygon_idxs_[0];
			if (line3d_track_polygon_idx == -1)
			{
				continue;
			}
			for (int j = 0; j < scenes_line3d_grouped_track_list_[i].line_views_pnts_.size(); ++j)
			{
				if (scenes_line3d_grouped_track_list_[i].line_views_to_polygon_idxs_[j] != line3d_track_polygon_idx)
				{
					scenes_line3d_grouped_track_list_[i].line_views_to_polygon_idxs_[j] = line3d_track_polygon_idx;
					int view_id = scenes_line3d_grouped_track_list_[i].lines_features_[j].view_id_;
					int line_id = scenes_line3d_grouped_track_list_[i].lines_features_[j].line_id_;
					std::pair<Eigen::Vector2f, Eigen::Vector2f> view_line_pnts_pos = hw_cams_views_list_[view_id].lines_segments_[line_id];
					CameraModel tmp_cam = hw_cams_views_list_[view_id].view_pose_;
					Eigen::Vector3f tmp_ls_pnt3d, tmp_le_pnt3d;
					bool is_ls_plane_intersection = ImagePnt2dPosProj2ScenePickedPolygonPnt3dPos(tmp_cam, view_line_pnts_pos.first, line3d_track_polygon_idx, tmp_ls_pnt3d);
					if (is_ls_plane_intersection)
					{
						scenes_line3d_grouped_track_list_[i].line_views_pnts_[j].first = tmp_ls_pnt3d;
					}
					else
					{
						std::cerr << "33333333333333 " << std::endl;
						scenes_line3d_grouped_track_list_[i].line_views_pnts_[j].first = Eigen::Vector3f(
							std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
							std::numeric_limits<float>::quiet_NaN());
						//scenes_pnts3d_track_list_[i].features_[j]
					}
					bool is_le_plane_intersection = ImagePnt2dPosProj2ScenePickedPolygonPnt3dPos(tmp_cam, view_line_pnts_pos.second, line3d_track_polygon_idx, tmp_le_pnt3d);
					if (is_le_plane_intersection)
					{
						scenes_line3d_grouped_track_list_[i].line_views_pnts_[j].second = tmp_le_pnt3d;
					}
					else
					{
						std::cerr << "222222222222222222 " << std::endl;
						scenes_line3d_grouped_track_list_[i].line_views_pnts_[j].second = Eigen::Vector3f(
							std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
							std::numeric_limits<float>::quiet_NaN());
					}
				}
			}
		}
		std::cerr << "End ReComputeSceneLinesGroupedTracks3dList3dLinePosByPolygonsIds..." << std::endl;
	}

	void HWScenesElements::ReComputeScenePntsGroupedTracks3dList3dPntsPosByPolygonsIds()
	{
		if (associated_polygons_.empty())
		{
			std::cerr << "ReComputeScenePntsGrouped: no scene polygons existed..." << std::endl;
			return;
		}
		std::cerr << "start to ReComputeScenePntsGroupedTracks3dList3dPntsPosByPolygonsIds..." << std::endl;
		//ImagePnt2dPosProj2ScenePickedPolygonPnt3dPos
		for (int i = 0; i < scenes_pnts3d_track_list_.size(); ++i)
		{
			if (!scenes_pnts3d_track_list_[i].is_valid_)
			{
				continue;
			}
			if (scenes_pnts3d_track_list_[i].polygon_idxs_.empty())
			{
				continue;
			}
			int pnt3d_track_polygon_idx = scenes_pnts3d_track_list_[i].polygon_idxs_[0];
			if (pnt3d_track_polygon_idx == -1)
			{
				continue;
			}
			for (int j = 0; j < scenes_pnts3d_track_list_[i].pnts_views_to_polygon_idxs_.size(); ++j)
			{
				if (scenes_pnts3d_track_list_[i].pnts_views_to_polygon_idxs_[j] != pnt3d_track_polygon_idx)
				{
					scenes_pnts3d_track_list_[i].pnts_views_to_polygon_idxs_[j] = pnt3d_track_polygon_idx;	//important, view points should have same polygon idx
					int view_id = scenes_pnts3d_track_list_[i].features_[j].view_id_;
					int pnt_id = scenes_pnts3d_track_list_[i].features_[j].feature_id_;
					Eigen::Vector2f view_pnt_pos = hw_cams_views_list_[view_id].features_poistion_[pnt_id];
					CameraModel tmp_cam = hw_cams_views_list_[view_id].view_pose_;
					Eigen::Vector3f tmp_pnt3d;
					bool is_plane_intersection = ImagePnt2dPosProj2ScenePickedPolygonPnt3dPos(tmp_cam, view_pnt_pos, pnt3d_track_polygon_idx, tmp_pnt3d);
					if (is_plane_intersection)
					{
						scenes_pnts3d_track_list_[i].views_features_pnts3d_[j] = tmp_pnt3d;
					}
					else
					{
						std::cerr << "11111111122222222 " << std::endl;
						scenes_pnts3d_track_list_[i].views_features_pnts3d_[j] = Eigen::Vector3f(
							std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
							std::numeric_limits<float>::quiet_NaN());
					}
				}
			}
		}
		std::cerr << "end ReComputeScenePntsGroupedTracks3dList3dPntsPosByPolygonsIds..." << std::endl;
	}

	void HWScenesElements::RunTrackLinesPnts3dToLinesPnts2LinesPnts3dObservations()
	{
		std::cerr << "run track lines 3d into lines points 3d Observations..." << std::endl;
		//std::cerr << "111111111111111" << std::endl;
		scenes_lines_pnts3ds_observations_.clear();
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			//std::cerr << "i: " << i << std::endl;
			if (!scenes_line3d_track_list_[i].is_valid_)
			{
				continue;
			}
			//std::cerr << "valid->i: " << i << std::endl;
			HWLineSurveyPoint3d2Pnts2d tmp_line3d_observation;
			Eigen::Vector3f tmp_s_pnt3d = scenes_line3d_track_list_[i].s_pos_;
			Eigen::Vector3f tmp_e_pnt3d = scenes_line3d_track_list_[i].e_pos_;
			tmp_line3d_observation.pos_s_ = tmp_s_pnt3d;
			tmp_line3d_observation.pos_e_ = tmp_e_pnt3d;
			int best_inter_line_idx = scenes_line3d_track_list_[i].polygon_intersect_line_idx_[0];
			if (best_inter_line_idx != -1)
			{
				HWPolygonInterLines best_inter_line = polygons_intersection_lines_[best_inter_line_idx];
				int plane_idx0 = best_inter_line.polygons_idxs_[0];
				int plane_idx1 = best_inter_line.polygons_idxs_[1];
				if (plane_idx0 != -1 && plane_idx1 != -1)
				{
					//std::cerr << "i->" << i << ": " << plane_idx0 << ", " << plane_idx1 << std::endl;
					tmp_line3d_observation.plane_idxs_.emplace_back(plane_idx0);
					tmp_line3d_observation.plane_idxs_.emplace_back(plane_idx1);
					tmp_line3d_observation.is_plane_lines_ = true;
				}
				else
				{
					//to do next...no polygon...
					//tmp_line3d_observation.plane_idxs_ = scenes_line3d_track_list_[i].polygon_idxs_;
					tmp_line3d_observation.is_plane_lines_ = false;
				}
			}
			else
			{
				//to do next...no polygon...
				//tmp_line3d_observation.plane_idxs_ = scenes_line3d_track_list_[i].polygon_idxs_;
				tmp_line3d_observation.is_plane_lines_ = false;
			}

			int line3d_oberservation_num = scenes_line3d_track_list_[i].lines_features_.size();
			//std::cerr << "line3d_oberservation_num: " << line3d_oberservation_num << std::endl;
			tmp_line3d_observation.observations_.resize(line3d_oberservation_num);
			for (int j = 0; j < scenes_line3d_track_list_[i].lines_features_.size(); ++j)
			{
				//std::cerr << "j: " << j << std::endl;
				int tmp_view_id = scenes_line3d_track_list_[i].lines_features_[j].view_id_;
				tmp_line3d_observation.observations_[j].view_id_ = tmp_view_id;
				int tpm_line_id = scenes_line3d_track_list_[i].lines_features_[j].line_id_;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_line_pnts2d = hw_cams_views_list_[tmp_view_id].lines_segments_[tpm_line_id];
				//std::cerr << "j line get: " << j << std::endl;
				tmp_line3d_observation.observations_[j].pos_s_ = tmp_line_pnts2d.first;
				tmp_line3d_observation.observations_[j].pos_e_ = tmp_line_pnts2d.second;
				//to do next...
				tmp_line3d_observation.observations_[j].plane_idxs_ = scenes_line3d_track_list_[i].polygon_idxs_;
				//tmp_line3d_observation.observations_[j].view_id_ = true;	//TO DO NEXT
				tmp_line3d_observation.observations_[j].is_valid_ = true;
				//std::cerr << "tmp_line3d_observation.observations_[j].view_id_: " << tmp_line3d_observation.observations_[j].view_id_ << std::endl;
				//check if the observed line is far distance to original observed line (if to far, set it false) to do next
				tmp_line3d_observation.observations_[j].view_line_pos_s_3d_ = scenes_line3d_track_list_[i].line_views_pnts_[j].first;	//get initial view point 3d
				tmp_line3d_observation.observations_[j].view_line_pos_e_3d_ = scenes_line3d_track_list_[i].line_views_pnts_[j].second;	//get initial view point 3d
			}
			scenes_lines_pnts3ds_observations_.emplace_back(tmp_line3d_observation);
		}
		std::cerr << "end track lines 3d into lines points 3d Observations..." << std::endl;
	}

	void HWScenesElements::RunTrackLinesPnts3dToLinesPnts2LinesPnts3dObservationsNew()
	{
		std::cerr << "run track lines 3d into lines points 3d Observations..." << std::endl;
		//std::cerr << "111111111111111" << std::endl;
		scenes_lines_pnts3ds_observations_.clear();
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			//std::cerr << "i: " << i << std::endl;
			if (!scenes_line3d_track_list_[i].is_valid_)
			{
				continue;
			}
			//std::cerr << "valid->i: " << i << std::endl;
			HWLineSurveyPoint3d2Pnts2d tmp_line3d_observation;
			Eigen::Vector3f tmp_s_pnt3d = scenes_line3d_track_list_[i].s_pos_;
			Eigen::Vector3f tmp_e_pnt3d = scenes_line3d_track_list_[i].e_pos_;
			tmp_line3d_observation.pos_s_ = tmp_s_pnt3d;
			tmp_line3d_observation.pos_e_ = tmp_e_pnt3d;
			//int best_inter_line_idx = scenes_line3d_track_list_[i].polygon_intersect_line_idx_[0];
			if (scenes_line3d_track_list_[i].polygon_idxs_.size() == 1)
			{
				int pidx0 = scenes_line3d_track_list_[i].polygon_idxs_[0];
				std::cerr << "i-> " << i << ": " << pidx0 << std::endl;
				tmp_line3d_observation.plane_idxs_.emplace_back(pidx0);
				tmp_line3d_observation.is_plane_lines_ = false;
			}
			else if (scenes_line3d_track_list_[i].polygon_idxs_.size() == 2)
			{
				int pidx0 = scenes_line3d_track_list_[i].polygon_idxs_[0];
				int pidx1 = scenes_line3d_track_list_[i].polygon_idxs_[1];

				std::cerr << "i-> " << i << ": " << pidx0 << ", " << pidx1 << std::endl;
				tmp_line3d_observation.plane_idxs_.emplace_back(pidx0);
				tmp_line3d_observation.plane_idxs_.emplace_back(pidx1);
				tmp_line3d_observation.is_plane_lines_ = true;
			}
			else
			{
				//to do next...no polygon...
				//tmp_line3d_observation.plane_idxs_ = scenes_line3d_track_list_[i].polygon_idxs_;
				std::cerr << "no polygon idx..." << std::endl;
				tmp_line3d_observation.is_plane_lines_ = false;
			}

			int line3d_oberservation_num = scenes_line3d_track_list_[i].lines_features_.size();
			//std::cerr << "line3d_oberservation_num: " << line3d_oberservation_num << std::endl;
			tmp_line3d_observation.observations_.resize(line3d_oberservation_num);
			for (int j = 0; j < scenes_line3d_track_list_[i].lines_features_.size(); ++j)
			{
				//std::cerr << "j: " << j << std::endl;
				int tmp_view_id = scenes_line3d_track_list_[i].lines_features_[j].view_id_;
				tmp_line3d_observation.observations_[j].view_id_ = tmp_view_id;
				int tpm_line_id = scenes_line3d_track_list_[i].lines_features_[j].line_id_;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_line_pnts2d = hw_cams_views_list_[tmp_view_id].lines_segments_[tpm_line_id];
				//std::cerr << "j line get: " << j << std::endl;
				tmp_line3d_observation.observations_[j].pos_s_ = tmp_line_pnts2d.first;
				tmp_line3d_observation.observations_[j].pos_e_ = tmp_line_pnts2d.second;
				//to do next...
				tmp_line3d_observation.observations_[j].plane_idxs_ = scenes_line3d_track_list_[i].polygon_idxs_;
				//tmp_line3d_observation.observations_[j].view_id_ = true;	//TO DO NEXT
				tmp_line3d_observation.observations_[j].is_valid_ = true;
				//std::cerr << "tmp_line3d_observation.observations_[j].view_id_: " << tmp_line3d_observation.observations_[j].view_id_ << std::endl;
				//check if the observed line is far distance to original observed line (if to far, set it false) to do next
				tmp_line3d_observation.observations_[j].view_line_pos_s_3d_ = scenes_line3d_track_list_[i].line_views_pnts_[j].first;	//get initial view point 3d
				tmp_line3d_observation.observations_[j].view_line_pos_e_3d_ = scenes_line3d_track_list_[i].line_views_pnts_[j].second;	//get initial view point 3d
			}
			scenes_lines_pnts3ds_observations_.emplace_back(tmp_line3d_observation);
		}
		std::cerr << "end track lines 3d into lines points 3d Observations..." << std::endl;
	}

	void HWScenesElements::RunTrackLinesPnts3dToLinesPnts2LinesPnts3dObservationsNoPolygonInterLine()
	{
		std::cerr << "run track lines 3d into lines points 3d Observations(no polygon intersection line)..." << std::endl;
		//std::cerr << "111111111111111" << std::endl;
		scenes_lines_pnts3ds_observations_.clear();
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			//std::cerr << "i: " << i << std::endl;
			if (!scenes_line3d_track_list_[i].is_valid_)
			{
				continue;
			}
			//std::cerr << "valid->i: " << i << std::endl;
			HWLineSurveyPoint3d2Pnts2d tmp_line3d_observation;
			Eigen::Vector3f tmp_s_pnt3d = scenes_line3d_track_list_[i].s_pos_;
			Eigen::Vector3f tmp_e_pnt3d = scenes_line3d_track_list_[i].e_pos_;
			tmp_line3d_observation.pos_s_ = tmp_s_pnt3d;
			tmp_line3d_observation.pos_e_ = tmp_e_pnt3d;
			//int best_inter_line_idx = scenes_line3d_track_list_[i].polygon_intersect_line_idx_[0];
			if (scenes_line3d_track_list_[i].polygon_idxs_.size() >= 1)
			{
				int pidx0 = scenes_line3d_track_list_[i].polygon_idxs_[0];
				std::cerr << "i-> " << i << ": " << pidx0 << std::endl;
				tmp_line3d_observation.plane_idxs_.emplace_back(pidx0);
				tmp_line3d_observation.is_plane_lines_ = false;
			}
			else
			{
				//to do next...no polygon...
				//tmp_line3d_observation.plane_idxs_ = scenes_line3d_track_list_[i].polygon_idxs_;
				std::cerr << "no polygon idx..." << std::endl;
				tmp_line3d_observation.is_plane_lines_ = false;
			}

			int line3d_oberservation_num = scenes_line3d_track_list_[i].lines_features_.size();
			//std::cerr << "line3d_oberservation_num: " << line3d_oberservation_num << std::endl;
			tmp_line3d_observation.observations_.resize(line3d_oberservation_num);
			for (int j = 0; j < scenes_line3d_track_list_[i].lines_features_.size(); ++j)
			{
				//std::cerr << "j: " << j << std::endl;
				int tmp_view_id = scenes_line3d_track_list_[i].lines_features_[j].view_id_;
				tmp_line3d_observation.observations_[j].view_id_ = tmp_view_id;
				int tpm_line_id = scenes_line3d_track_list_[i].lines_features_[j].line_id_;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_line_pnts2d = hw_cams_views_list_[tmp_view_id].lines_segments_[tpm_line_id];
				//std::cerr << "j line get: " << j << std::endl;
				tmp_line3d_observation.observations_[j].pos_s_ = tmp_line_pnts2d.first;
				tmp_line3d_observation.observations_[j].pos_e_ = tmp_line_pnts2d.second;
				//to do next...
				tmp_line3d_observation.observations_[j].plane_idxs_ = scenes_line3d_track_list_[i].polygon_idxs_;
				//tmp_line3d_observation.observations_[j].view_id_ = true;	//TO DO NEXT
				//if (scenes_line3d_track_list_[i].line_views_pnts_[j]);
				if (IsHWLineSegmentValidZDG(scenes_line3d_track_list_[i].line_views_pnts_[j]))
				{
					tmp_line3d_observation.observations_[j].is_valid_ = true;
				}
				else
				{
					tmp_line3d_observation.observations_[j].is_valid_ = false;
				}
				tmp_line3d_observation.observations_[j].view_line_pos_s_3d_ = scenes_line3d_track_list_[i].line_views_pnts_[j].first;	//get initial view point 3d
				tmp_line3d_observation.observations_[j].view_line_pos_e_3d_ = scenes_line3d_track_list_[i].line_views_pnts_[j].second;	//get initial view point 3d
				//tmp_line3d_observation.observations_[j].is_valid_ = true;
				//tmp_line3d_observation.observations_[j].is_valid_ = scenes_line3d_track_list_[i].lines_features_[j];
				//std::cerr << "tmp_line3d_observation.observations_[j].view_id_: " << tmp_line3d_observation.observations_[j].view_id_ << std::endl;
				//check if the observed line is far distance to original observed line (if to far, set it false) to do next

			}
			scenes_lines_pnts3ds_observations_.emplace_back(tmp_line3d_observation);
		}
		std::cerr << "end track lines 3d into lines points 3d Observations(no polygon intersection line)..." << std::endl;
	}

	void HWScenesElements::RunGroupedTrackLinesPnts3dToLinesPnts2LinesPnts3dObservationsNoPolygonInterLine()
	{
		std::cerr << "run track lines 3d into lines points 3d Observations(no polygon intersection line)..." << std::endl;
		//std::cerr << "111111111111111" << std::endl;
		scenes_lines_pnts3ds_observations_.clear();
		for (int i = 0; i < scenes_line3d_grouped_track_list_.size(); ++i)
		{
			//std::cerr << "i: " << i << std::endl;
			if (!scenes_line3d_grouped_track_list_[i].is_valid_)
			{
				continue;
			}
			//std::cerr << "valid->i: " << i << std::endl;
			HWLineSurveyPoint3d2Pnts2d tmp_line3d_observation;
			Eigen::Vector3f tmp_s_pnt3d = scenes_line3d_grouped_track_list_[i].s_pos_;
			Eigen::Vector3f tmp_e_pnt3d = scenes_line3d_grouped_track_list_[i].e_pos_;
			tmp_line3d_observation.pos_s_ = tmp_s_pnt3d;
			tmp_line3d_observation.pos_e_ = tmp_e_pnt3d;
			//int best_inter_line_idx = scenes_line3d_track_list_[i].polygon_intersect_line_idx_[0];
			if (scenes_line3d_grouped_track_list_[i].polygon_idxs_.size() >= 1)
			{
				int pidx0 = scenes_line3d_grouped_track_list_[i].polygon_idxs_[0];
				std::cerr << "i-> " << i << ": " << pidx0 << std::endl;
				tmp_line3d_observation.plane_idxs_.emplace_back(pidx0);
				tmp_line3d_observation.is_plane_lines_ = false;
			}
			else
			{
				//to do next...no polygon...
				//tmp_line3d_observation.plane_idxs_ = scenes_line3d_track_list_[i].polygon_idxs_;
				std::cerr << "no polygon idx..." << std::endl;
				tmp_line3d_observation.is_plane_lines_ = false;
			}

			int line3d_oberservation_num = scenes_line3d_grouped_track_list_[i].lines_features_.size();
			//std::cerr << "line3d_oberservation_num: " << line3d_oberservation_num << std::endl;
			tmp_line3d_observation.observations_.resize(line3d_oberservation_num);
			for (int j = 0; j < scenes_line3d_grouped_track_list_[i].lines_features_.size(); ++j)
			{
				//std::cerr << "j: " << j << std::endl;
				int tmp_view_id = scenes_line3d_grouped_track_list_[i].lines_features_[j].view_id_;
				tmp_line3d_observation.observations_[j].view_id_ = tmp_view_id;
				int tpm_line_id = scenes_line3d_grouped_track_list_[i].lines_features_[j].line_id_;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_line_pnts2d = hw_cams_views_list_[tmp_view_id].lines_segments_[tpm_line_id];
				//std::cerr << "j line get: " << j << std::endl;
				tmp_line3d_observation.observations_[j].pos_s_ = tmp_line_pnts2d.first;
				tmp_line3d_observation.observations_[j].pos_e_ = tmp_line_pnts2d.second;
				//to do next...
				tmp_line3d_observation.observations_[j].plane_idxs_ = scenes_line3d_grouped_track_list_[i].polygon_idxs_;
				//tmp_line3d_observation.observations_[j].view_id_ = true;	//TO DO NEXT
				//if (scenes_line3d_track_list_[i].line_views_pnts_[j]);
				if (IsHWLineSegmentValidZDG(scenes_line3d_grouped_track_list_[i].line_views_pnts_[j]))
				{
					tmp_line3d_observation.observations_[j].is_valid_ = true;
				}
				else
				{
					tmp_line3d_observation.observations_[j].is_valid_ = false;
				}
				tmp_line3d_observation.observations_[j].view_line_pos_s_3d_ = scenes_line3d_grouped_track_list_[i].line_views_pnts_[j].first;	//get initial view point 3d
				tmp_line3d_observation.observations_[j].view_line_pos_e_3d_ = scenes_line3d_grouped_track_list_[i].line_views_pnts_[j].second;	//get initial view point 3d
																																		//tmp_line3d_observation.observations_[j].is_valid_ = true;
				//tmp_line3d_observation.observations_[j].is_valid_ = scenes_line3d_track_list_[i].lines_features_[j];
				//std::cerr << "tmp_line3d_observation.observations_[j].view_id_: " << tmp_line3d_observation.observations_[j].view_id_ << std::endl;
				//check if the observed line is far distance to original observed line (if to far, set it false) to do next

			}
			scenes_lines_pnts3ds_observations_.emplace_back(tmp_line3d_observation);
		}
		std::cerr << "end track lines 3d into lines points 3d Observations(no polygon intersection line)..." << std::endl;
	}

	void HWScenesElements::RunTrackLinesPnts3dToLinesPnts2LinesPnts3dObservationsPolygonInterLine()
	{
		//
		std::cerr << "to do next... need to group view lines(near each other)..." << std::endl;
		std::cerr << "run track lines 3d into lines points 3d Observations(need polygon intersection line)..." << std::endl;
		//std::cerr << "111111111111111" << std::endl;
		scenes_lines_pnts3ds_observations_.clear();
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			//std::cerr << "i: " << i << std::endl;
			if (!scenes_line3d_track_list_[i].is_valid_)
			{
				continue;
			}
			//std::cerr << "valid->i: " << i << std::endl;
			HWLineSurveyPoint3d2Pnts2d tmp_line3d_observation;
			Eigen::Vector3f tmp_s_pnt3d = scenes_line3d_track_list_[i].s_pos_;
			Eigen::Vector3f tmp_e_pnt3d = scenes_line3d_track_list_[i].e_pos_;
			tmp_line3d_observation.pos_s_ = tmp_s_pnt3d;
			tmp_line3d_observation.pos_e_ = tmp_e_pnt3d;
			//int best_inter_line_idx = scenes_line3d_track_list_[i].polygon_intersect_line_idx_[0];
			if (scenes_line3d_track_list_[i].polygon_idxs_.size() == 1)
			{
				int pidx0 = scenes_line3d_track_list_[i].polygon_idxs_[0];
				std::cerr << "i-> " << i << ": " << pidx0 << std::endl;
				tmp_line3d_observation.plane_idxs_.emplace_back(pidx0);
				tmp_line3d_observation.is_plane_lines_ = false;
			}
			else if (scenes_line3d_track_list_[i].polygon_idxs_.size() == 2)
			{
				int pidx0 = scenes_line3d_track_list_[i].polygon_idxs_[0];
				int pidx1 = scenes_line3d_track_list_[i].polygon_idxs_[1];
				//
				std::cerr << "i-> " << i << ": " << pidx0 << ", " << pidx1 << std::endl;
				tmp_line3d_observation.plane_idxs_.emplace_back(pidx0);
				tmp_line3d_observation.plane_idxs_.emplace_back(pidx1);
				tmp_line3d_observation.is_plane_lines_ = true;
			}
			else
			{
				//to do next...no polygon...
				//tmp_line3d_observation.plane_idxs_ = scenes_line3d_track_list_[i].polygon_idxs_;
				std::cerr << "no polygon idx..." << std::endl;
				tmp_line3d_observation.is_plane_lines_ = false;
			}

			int line3d_oberservation_num = scenes_line3d_track_list_[i].lines_features_.size();
			//std::cerr << "line3d_oberservation_num: " << line3d_oberservation_num << std::endl;
			tmp_line3d_observation.observations_.resize(line3d_oberservation_num);
			for (int j = 0; j < scenes_line3d_track_list_[i].lines_features_.size(); ++j)
			{
				//std::cerr << "j: " << j << std::endl;
				int tmp_view_id = scenes_line3d_track_list_[i].lines_features_[j].view_id_;
				tmp_line3d_observation.observations_[j].view_id_ = tmp_view_id;
				int tpm_line_id = scenes_line3d_track_list_[i].lines_features_[j].line_id_;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_line_pnts2d = hw_cams_views_list_[tmp_view_id].lines_segments_[tpm_line_id];
				//std::cerr << "j line get: " << j << std::endl;
				tmp_line3d_observation.observations_[j].pos_s_ = tmp_line_pnts2d.first;
				tmp_line3d_observation.observations_[j].pos_e_ = tmp_line_pnts2d.second;
				//to do next...
				tmp_line3d_observation.observations_[j].plane_idxs_ = scenes_line3d_track_list_[i].polygon_idxs_;
				//tmp_line3d_observation.observations_[j].view_id_ = true;	//TO DO NEXT
				tmp_line3d_observation.observations_[j].is_valid_ = true;
				//std::cerr << "tmp_line3d_observation.observations_[j].view_id_: " << tmp_line3d_observation.observations_[j].view_id_ << std::endl;
				//check if the observed line is far distance to original observed line (if to far, set it false) to do next
				tmp_line3d_observation.observations_[j].view_line_pos_s_3d_ = scenes_line3d_track_list_[i].line_views_pnts_[j].first;	//get initial view point 3d
				tmp_line3d_observation.observations_[j].view_line_pos_e_3d_ = scenes_line3d_track_list_[i].line_views_pnts_[j].second;	//get initial view point 3d
			}
			scenes_lines_pnts3ds_observations_.emplace_back(tmp_line3d_observation);
		}
		std::cerr << "end track lines 3d into lines points 3d Observations(need polygon intersection line)..." << std::endl;
	}

	void HWScenesElements::RunFittingLinesFromTrackLinePnt3dlist()
	{
		std::cerr << "start fit line from track list..." << std::endl;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_3d_test_lines;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			//std::cerr << "i: " << i << std::endl;
			HWLineTrack3D tmp_line3d_track = scenes_line3d_track_list_[i];
			std::vector<Eigen::Vector3f> view_line_pnts;
			for (int j = 0; j < tmp_line3d_track.line_views_pnts_.size(); ++j)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> line_pnt = tmp_line3d_track.line_views_pnts_[j];
				view_line_pnts.emplace_back(line_pnt.first);
				view_line_pnts.emplace_back(line_pnt.second);
			}
			Eigen::Vector3f tmp_ls, tmp_le;
			FittingLine3dFromPnts3d3f(view_line_pnts, tmp_ls, tmp_le);
			scenes_3d_test_lines.emplace_back(std::make_pair(tmp_ls, tmp_le));
			scenes_line3d_track_list_[i].s_pos_ = tmp_ls;
			scenes_line3d_track_list_[i].e_pos_ = tmp_le;
		}
		/*std::string sc_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images_1/scene_layout_observation.obj";
		WriteLine3DIntoObj(scenes_3d_test_lines, sc_path);
		std::cerr << "end fit line..." << std::endl;*/
	}

	void HWScenesElements::RunFittingLines3DFromTrackLinePnt3dlistFilter()
	{
		//std::cerr << "start fit line from track list..." << std::endl;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_3d_test_lines;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			//std::cerr << "i: " << i << std::endl;
			HWLineTrack3D tmp_line3d_track = scenes_line3d_track_list_[i];
			if (!tmp_line3d_track.is_valid_)
			{
				//std::cerr << "invalid i: " << i << std::endl;
				continue;
			}
			std::vector<Eigen::Vector3f> view_line_pnts;
			for (int j = 0; j < tmp_line3d_track.line_views_pnts_.size(); ++j)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> line_pnt = tmp_line3d_track.line_views_pnts_[j];
				if (line_pnt.first[0] == line_pnt.first[0]
					&& line_pnt.first[1] == line_pnt.first[1]
					&& line_pnt.first[2] == line_pnt.first[2]
					&& line_pnt.second[0] == line_pnt.second[0]
					&& line_pnt.second[1] == line_pnt.second[1]
					&& line_pnt.second[2] == line_pnt.second[2])
				{
					view_line_pnts.emplace_back(line_pnt.first);
					view_line_pnts.emplace_back(line_pnt.second);
				}
			}
			if (view_line_pnts.size() < 2)
			{
				scenes_line3d_track_list_[i].is_valid_ = false;
			}
			else
			{
				Eigen::Vector3f tmp_ls, tmp_le;
				FittingLine3dFromPnts3d3f(view_line_pnts, tmp_ls, tmp_le);
				scenes_3d_test_lines.emplace_back(std::make_pair(tmp_ls, tmp_le));
				scenes_line3d_track_list_[i].s_pos_ = tmp_ls;
				scenes_line3d_track_list_[i].e_pos_ = tmp_le;
			}
		}
		std::string sc_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part5_test/image_opti_reproject/scene_layout_observation_initial.obj";
		WriteLine3DIntoObj(scenes_3d_test_lines, sc_path);
		std::cerr << "end fit line..." << std::endl;
	}

	void HWScenesElements::RunFittingLines3DFromTrackGroupedLinePnt3dlistFilter()
	{
		std::cerr << "start fit line from grouped line track list..." << std::endl;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_3d_test_lines;
		for (int i = 0; i < scenes_line3d_grouped_track_list_.size(); ++i)
		{
			std::cerr << "scenes_line3d_grouped_track_list_ i: " << i << std::endl;
			HWGroupedLineTrack3D tmp_line3d_track = scenes_line3d_grouped_track_list_[i];
			if (!tmp_line3d_track.is_valid_)
			{
				//std::cerr << "invalid i: " << i << std::endl;
				continue;
			}
			std::vector<Eigen::Vector3f> view_line_pnts;
			for (int j = 0; j < tmp_line3d_track.line_views_pnts_.size(); ++j)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> line_pnt = tmp_line3d_track.line_views_pnts_[j];
				if (line_pnt.first[0] == line_pnt.first[0]
					&& line_pnt.first[1] == line_pnt.first[1]
					&& line_pnt.first[2] == line_pnt.first[2]
					&& line_pnt.second[0] == line_pnt.second[0]
					&& line_pnt.second[1] == line_pnt.second[1]
					&& line_pnt.second[2] == line_pnt.second[2])
				{
					view_line_pnts.emplace_back(line_pnt.first);
					view_line_pnts.emplace_back(line_pnt.second);
				}
			}
			if (view_line_pnts.size() < 2)
			{
				//std::cerr << "scenes_line3d_grouped_track_list_[" << i << "]" << "is not valid..." << std::endl;
				scenes_line3d_grouped_track_list_[i].is_valid_ = false;
			}
			else
			{
				Eigen::Vector3f tmp_ls, tmp_le;
				FittingLine3dFromPnts3d3f(view_line_pnts, tmp_ls, tmp_le);
				scenes_3d_test_lines.emplace_back(std::make_pair(tmp_ls, tmp_le));
				scenes_line3d_grouped_track_list_[i].s_pos_ = tmp_ls;
				scenes_line3d_grouped_track_list_[i].e_pos_ = tmp_le;
			}
		}
		std::string sc_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part5_test/line_test_obj/views/scene_layout_lines_track_fitting_initial.obj";
		WriteLine3DIntoObj(scenes_3d_test_lines, sc_path);
		//std::cerr << "end fit line..." << std::endl;
		std::cerr << "end fit line from grouped line track list..." << std::endl;
	}

	void HWScenesElements::SetAllTheViewTrackLine3dSamePolygonIds()
	{
		std::cerr << "start filter line from grouped line track list..." << std::endl;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_3d_test_lines;
		for (int i = 0; i < scenes_line3d_grouped_track_list_.size(); ++i)
		{
			std::cerr << "SetAllTheViewTrackLine3dSamePolygonIds i: " << i << std::endl;
			HWGroupedLineTrack3D tmp_line3d_track = scenes_line3d_grouped_track_list_[i];
			if (!tmp_line3d_track.is_valid_)
			{
				//std::cerr << "invalid i: " << i << std::endl;
				continue;
			}
			std::vector<Eigen::Vector3f> view_line_pnts;
			//for (int j = 0; j < tmp_line3d_track.id_to_hwline_track3d_idx_.size(); ++j)
			//{
			//}
			for (int j = 0; j < tmp_line3d_track.line_views_pnts_.size(); ++j)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> line_pnt = tmp_line3d_track.line_views_pnts_[j];
				if (line_pnt.first[0] == line_pnt.first[0]
					&& line_pnt.first[1] == line_pnt.first[1]
					&& line_pnt.first[2] == line_pnt.first[2]
					&& line_pnt.second[0] == line_pnt.second[0]
					&& line_pnt.second[1] == line_pnt.second[1]
					&& line_pnt.second[2] == line_pnt.second[2])
				{
					view_line_pnts.emplace_back(line_pnt.first);
					view_line_pnts.emplace_back(line_pnt.second);
				}
			}
			if (view_line_pnts.size() < 2)
			{
				//std::cerr << "scenes_line3d_grouped_track_list_[" << i << "]" << "is not valid..." << std::endl;
				scenes_line3d_grouped_track_list_[i].is_valid_ = false;
			}
			else
			{
				Eigen::Vector3f tmp_ls, tmp_le;
				FittingLine3dFromPnts3d3f(view_line_pnts, tmp_ls, tmp_le);
				scenes_3d_test_lines.emplace_back(std::make_pair(tmp_ls, tmp_le));
				scenes_line3d_grouped_track_list_[i].s_pos_ = tmp_ls;
				scenes_line3d_grouped_track_list_[i].e_pos_ = tmp_le;
			}
		}
		std::string sc_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/scannet_my_data/scene0062_01_part5_test/line_test_obj/views/scene_layout_lines_track_fitting_initial.obj";
		WriteLine3DIntoObj(scenes_3d_test_lines, sc_path);
		//std::cerr << "end fit line..." << std::endl;
		std::cerr << "end filter line from grouped line track list..." << std::endl;
	}

	void HWScenesElements::RunFittingLinesByPolygonNearCamsFromTrackLinePnt3dlist()
	{
		std::cerr << "start fit line by polygons near cams from track list..." << std::endl;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_3d_test_lines;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_visisable_lines_pnts;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_all_views_projects_pnts;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_all_views_pnts;

		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			HWLineTrack3D tmp_line3d_track = scenes_line3d_track_list_[i];
			//get view from HWLineTrack3D: line 3d to cam pos distance
			std::vector<std::pair<int, float> > lines_dists;
			
			for (int j = 0; j < tmp_line3d_track.lines_features_.size(); ++j)
			{
				HWLineFeatureReferenceId view_line_pair = tmp_line3d_track.lines_features_[j];
				//get image visiable lines and project to line
				
				std::pair<Eigen::Vector2i, LineDistMeasure> visible_line_idx_dist 
					= ComputeViewVisiableLine3dToImageLineDist(view_line_pair);

				Eigen::Vector2i visiable_line_idx = GetViewLineProjectToImageLineIdx(view_line_pair);
				//compute the line 3d dist to cam
				if (visiable_line_idx[1] != -1)
				{
					//get the cam idx, line idx
					int camidx = visiable_line_idx[0];
					int line_visiable_idx = visiable_line_idx[1];
					std::pair<Eigen::Vector3f, Eigen::Vector3f> linesg3d
						= views_visiable_polygon_inter_lines_[camidx].view_visiable_lines_pnts_[line_visiable_idx];
					int polygon_inter_line_idx = views_visiable_polygon_inter_lines_[camidx].view_visible_lines_idxs_[line_visiable_idx];
					CameraModel cam = scenes_cams_->GetCameraModelFromCamId(camidx);
					Eigen::Vector3f cm_o = cam.cam_pose_.topRightCorner(3, 1);
					Eigen::Vector3f line_seg_mean = (linesg3d.first + linesg3d.second) / 2;
					float cam2line = (line_seg_mean - cm_o).norm();
					std::pair<int, float> tmpdist;
					tmpdist.first = j;
					tmpdist.second = cam2line;
					lines_dists.emplace_back(tmpdist);
				}
			}
			std::sort(lines_dists.begin(), lines_dists.end(), MyLine3dSegValueCmpZDG);
			
			if (!lines_dists.empty())
			{
				//find best view_line_pair
				//use the minimum lines_features_->view_line_pair
				int line_feature_idx = lines_dists[0].first;
				HWLineFeatureReferenceId view_line_pair = tmp_line3d_track.lines_features_[line_feature_idx];
				//判断哪些帧比较好，用于后续的处理(查看)
				Eigen::Vector2i visiable_line_idx = GetViewLineProjectToImageLineIdx(view_line_pair);
				//get the cam idx, line idx
				int camidx = visiable_line_idx[0];
				int line_visiable_idx = visiable_line_idx[1];
				std::pair<Eigen::Vector3f, Eigen::Vector3f> linesg3d
					= views_visiable_polygon_inter_lines_[camidx].view_visiable_lines_pnts_[line_visiable_idx];
				scenes_visisable_lines_pnts.emplace_back(linesg3d);
				Eigen::Vector3f linesg_dir = linesg3d.second - linesg3d.first;
				linesg_dir.normalize();
				//get the 
				int polygon_inter_line_idx = views_visiable_polygon_inter_lines_[camidx].view_visible_lines_idxs_[line_visiable_idx];
				PolygonInterLineScene tmp_poly_inter_line = polygons_intersection_scene_lines_[polygon_inter_line_idx];
				//set the value
				scenes_line3d_track_list_[i].polygon_idxs_.clear();
				scenes_line3d_track_list_[i].polygon_idxs_.emplace_back(tmp_poly_inter_line.adj_poly_idxs_[0]);
				scenes_line3d_track_list_[i].polygon_idxs_.emplace_back(tmp_poly_inter_line.adj_poly_idxs_[1]);
				std::cerr << "the scenes_line3d_track_list_["<< i << "].polygon_idxs_ size: " << scenes_line3d_track_list_[i].polygon_idxs_.size() << std::endl;
				scenes_line3d_track_list_[i].polygon_intersect_line_idx_[0] = polygon_inter_line_idx;

#if 0	//use the idx to access vec element
				for (int j = 0; j < scenes_line3d_track_list_[i].line_views_pnts_.size(); ++j)
				{
					//all project to linesg3d
					int view_pair_id = scenes_line3d_track_list_[i].lines_features_[j].view_id_;
					Eigen::Vector3f vls = scenes_line3d_track_list_[i].line_views_pnts_[j].first;
					Eigen::Vector3f vle = scenes_line3d_track_list_[i].line_views_pnts_[j].second;
					scenes_all_views_pnts.emplace_back(scenes_line3d_track_list_[i].line_views_pnts_[j]);
					Eigen::Vector3f vpls, vple;
					/*ComputePntProj2Line3DF(vls, linesg3d.first, linesg3d.second, vpls);
					ComputePntProj2Line3DF(vle, linesg3d.first, linesg3d.second, vple);*/
					ComputePntProj2Line3DF(linesg3d.first, linesg_dir, vls, vpls);
					ComputePntProj2Line3DF(linesg3d.first, linesg_dir, vle, vple);

					////filter the view that whose projected line far dist to origin line
					//float projline_to_line_dist = (vpls - vls).norm() + (vple - vle).norm();
					//projline_to_line_dist /= 2.0;
					//if (projline_to_line_dist > r_max_polygon_threshold_*2)
					//{
					//	scenes_line3d_track_list_[i].remove_view(view_pair_id);
					//}

					scenes_line3d_track_list_[i].line_views_pnts_[j].first = vpls;
					scenes_line3d_track_list_[i].line_views_pnts_[j].second = vple;

					scenes_all_views_projects_pnts.emplace_back(scenes_line3d_track_list_[i].line_views_pnts_[j]);
				}
#endif

				//use the iterator for the earase the view that is wrong
				HWLineFeatureReferenceIdList::iterator line_feature_iter
					= scenes_line3d_track_list_[i].lines_features_.begin();
				std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >::iterator line_pos_iter
					= scenes_line3d_track_list_[i].line_views_pnts_.begin();
				while (line_feature_iter != scenes_line3d_track_list_[i].lines_features_.end()
					&& line_pos_iter != scenes_line3d_track_list_[i].line_views_pnts_.end())
				{
					//all project to linesg3d
					int view_pair_id = line_feature_iter->view_id_;
					Eigen::Vector3f vls = line_pos_iter->first;
					Eigen::Vector3f vle = line_pos_iter->second;
					Eigen::Vector3f vpls, vple;
					/*ComputePntProj2Line3DF(vls, linesg3d.first, linesg3d.second, vpls);
					ComputePntProj2Line3DF(vle, linesg3d.first, linesg3d.second, vple);*/
					ComputePntProj2Line3DF(linesg3d.first, linesg_dir, vls, vpls);
					ComputePntProj2Line3DF(linesg3d.first, linesg_dir, vle, vple);

					//filter the view that whose projected line far dist to origin line
					float projline_to_line_dist = (vpls - vls).norm() + (vple - vle).norm();
					projline_to_line_dist /= 2.0;
					float r_max_polygon_threshold = 0.08;
					if (projline_to_line_dist > r_max_polygon_threshold)
					{
						std::cerr << "delete wrong view line match..." << std::endl;
						line_feature_iter = scenes_line3d_track_list_[i].lines_features_.erase(line_feature_iter);
						line_pos_iter = scenes_line3d_track_list_[i].line_views_pnts_.erase(line_pos_iter);
					}
					else
					{
						scenes_all_views_pnts.emplace_back(*line_pos_iter);
						line_pos_iter->first = vpls;
						line_pos_iter->second = vple;
						scenes_all_views_projects_pnts.emplace_back(*line_pos_iter);
						line_feature_iter++;
						line_pos_iter++;
					}
				}
			}
			// line to polygon line
			//find the minimum lines_dists
			//int lines
			std::vector<Eigen::Vector3f> view_line_pnts;
			for (int j = 0; j < scenes_line3d_track_list_[i].line_views_pnts_.size(); ++j)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> line_pnt = scenes_line3d_track_list_[i].line_views_pnts_[j];
				view_line_pnts.emplace_back(line_pnt.first);
				view_line_pnts.emplace_back(line_pnt.second);
			}
			Eigen::Vector3f tmp_ls, tmp_le;
			FittingLine3dFromPnts3d3f(view_line_pnts, tmp_ls, tmp_le);
			scenes_3d_test_lines.emplace_back(std::make_pair(tmp_ls, tmp_le));
			scenes_line3d_track_list_[i].s_pos_ = tmp_ls;
			scenes_line3d_track_list_[i].e_pos_ = tmp_le;
		}

		std::string sc_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj_test8/lines_tracks/scene_layout_observation.obj";
		WriteLine3DIntoObj(scenes_3d_test_lines, sc_path);
		std::string sc_seg_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj_test8/lines_tracks/scene_layout_visiable.obj";
		WriteLine3DIntoObj(scenes_visisable_lines_pnts, sc_seg_path);
		std::string sc_allview_project_seg_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj_test8/lines_tracks/scene_layout_all_views_project_pnts.obj";
		WriteLine3DIntoObj(scenes_all_views_projects_pnts, sc_allview_project_seg_path);

		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_all_views_to_project_view_pnts;
		for (int i = 0; i < scenes_all_views_projects_pnts.size(); ++i)
		{
			std::pair<Eigen::Vector3f, Eigen::Vector3f> linesg1 = scenes_all_views_projects_pnts[i];
			std::pair<Eigen::Vector3f, Eigen::Vector3f> linesg2 = scenes_all_views_pnts[i];
			Eigen::Vector3f mean_linesg1 = (linesg1.first + linesg1.second) / 2;
			Eigen::Vector3f mean_linesg2 = (linesg2.first + linesg2.second) / 2;
			scenes_all_views_to_project_view_pnts.emplace_back(std::make_pair(mean_linesg1, mean_linesg2));
		}

		std::string sc_all_view_seg_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj_test8/lines_tracks/scene_layout_all_views_pnts.obj";
		WriteLine3DIntoObj(scenes_all_views_pnts, sc_all_view_seg_path);
		std::string sc_allview_line2line_seg_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj_test8/lines_tracks/scene_layout_all_views_line2line_pnts.obj";
		WriteLine3DIntoObj(scenes_all_views_to_project_view_pnts, sc_allview_line2line_seg_path);

		std::cerr << "end fit line..." << std::endl;
	}

	void HWScenesElements::RunFittingLinesByPolygonNearCorrectedCamsFromTrackLinePnt3dlist()
	{
		std::cerr << "start fit line by polygons near cams from track list..." << std::endl;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_3d_test_lines;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_visisable_lines_pnts;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_all_views_projects_pnts;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_all_views_pnts;

		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			HWLineTrack3D tmp_line3d_track = scenes_line3d_track_list_[i];
			//get view from HWLineTrack3D: line 3d to cam pos distance
			//first: line feature idx, second: distance between cam pos with corresponding polygon 3d intersection line
			std::vector<std::pair<int, float> > lines_to_cams_dists;
			//first: line feature idx, second: distance between image line projected by corresponding polygon 3d intersection line with
			//detect image line
			std::map<int, LineDistMeasure> lines_idx_to_dists;
			for (int j = 0; j < tmp_line3d_track.lines_features_.size(); ++j)
			{
				HWLineFeatureReferenceId view_line_pair = tmp_line3d_track.lines_features_[j];
				//get image visiable lines and project to line
				std::pair<Eigen::Vector2i, LineDistMeasure> visible_line_idx_dist
					= ComputeViewVisiableLine3dToImageLineDist(view_line_pair);
				//compute the line 3d dist to cam
				if (visible_line_idx_dist.first[1] != -1)
				{
					//get the cam idx, line idx
					int camidx = visible_line_idx_dist.first[0];
					int line_visiable_idx = visible_line_idx_dist.first[1];
					std::pair<Eigen::Vector3f, Eigen::Vector3f> linesg3d
						= views_visiable_polygon_inter_lines_[camidx].view_visiable_lines_pnts_[line_visiable_idx];
					int polygon_inter_line_idx = views_visiable_polygon_inter_lines_[camidx].view_visible_lines_idxs_[line_visiable_idx];
					CameraModel cam = scenes_cams_->GetCameraModelFromCamId(camidx);
					Eigen::Vector3f cm_o = cam.cam_pose_.topRightCorner(3, 1);
					Eigen::Vector3f line_seg_mean = (linesg3d.first + linesg3d.second) / 2;
					float cam2line = (line_seg_mean - cm_o).norm();
					std::pair<int, float> tmpdist;
					tmpdist.first = j;
					tmpdist.second = cam2line;
					lines_to_cams_dists.emplace_back(tmpdist);
					lines_idx_to_dists[j] = visible_line_idx_dist.second;
				}
			}
			std::sort(lines_to_cams_dists.begin(), lines_to_cams_dists.end(), MyLine3dSegValueCmpZDG);

			int best_idx_to_polygon_inter_line = -1;
			if (!lines_to_cams_dists.empty())
			{
				int line_feature_idx0 = lines_to_cams_dists[0].first;
				int line_feature_idx1 = -1;
				for (int j = 1; j < lines_to_cams_dists.size(); ++j)
				{
					if (line_feature_idx0 != lines_to_cams_dists[j].first
						&& lines_to_cams_dists[j].second < cam_to_polygon_inter_line_threhold_)
					{
						line_feature_idx1 = lines_to_cams_dists[j].first;
						break;
					}
				}
				if (line_feature_idx1 != -1)
				{
					//check its
					HWLineFeatureReferenceId view_line_pair0 = tmp_line3d_track.lines_features_[line_feature_idx0];
					Eigen::Vector2i view0_visiable_line_idx = GetViewLineProjectToImageLineIdx(view_line_pair0);
					int cam0_id = view0_visiable_line_idx[0];
					int cam0_line_idx = view0_visiable_line_idx[1];
					std::pair<Eigen::Vector3f, Eigen::Vector3f> linesg3d0
						= views_visiable_polygon_inter_lines_[cam0_id].view_visiable_lines_pnts_[cam0_line_idx];
					HWLineFeatureReferenceId view_line_pair1 = tmp_line3d_track.lines_features_[line_feature_idx1];
					Eigen::Vector2i view1_visiable_line_idx = GetViewLineProjectToImageLineIdx(view_line_pair1);
					int cam1_id = view1_visiable_line_idx[0];
					int cam1_line_idx = view1_visiable_line_idx[1];
					std::pair<Eigen::Vector3f, Eigen::Vector3f> linesg3d1
						= views_visiable_polygon_inter_lines_[cam1_id].view_visiable_lines_pnts_[cam1_line_idx];
					//project to image0 image1
					CameraModel cam0 = scenes_cams_->GetCameraModelFromCamId(cam0_id);
					CameraModel cam1 = scenes_cams_->GetCameraModelFromCamId(cam1_id);
					LineDistMeasure cam1_line_to_image_line = ComputeLine3dToImageViewLineDist(linesg3d1, view_line_pair0);
					LineDistMeasure cam0_line_to_image_line = ComputeLine3dToImageViewLineDist(linesg3d0, view_line_pair0);
					//compare them
					if (cam1_line_to_image_line.dist_measure_ < cam0_line_to_image_line.dist_measure_)
					{
						std::cerr << "the scenes_line3d_track_list_[" << i << "] line_feature_idx1:" << line_feature_idx1 << std::endl;
						best_idx_to_polygon_inter_line = line_feature_idx1;
					}
					else
					{
						std::cerr << "the scenes_line3d_track_list_[" << i << "] line_feature_idx0:" << line_feature_idx0 << std::endl;
						best_idx_to_polygon_inter_line = line_feature_idx0;
					}
				}
				else
				{
					std::cerr << "the scenes_line3d_track_list_[" << i << "] other line_feature_idx0:" << line_feature_idx0 << std::endl;
					best_idx_to_polygon_inter_line = line_feature_idx0;
				}
			}

			if (best_idx_to_polygon_inter_line != -1)
			{

				//use the minimum lines_features_->view_line_pair
				//int line_feature_idx = lines_dists[0].first;
				HWLineFeatureReferenceId view_line_pair = tmp_line3d_track.lines_features_[best_idx_to_polygon_inter_line];
				//判断哪些帧比较好，用于后续的处理(查看)
				Eigen::Vector2i visiable_line_idx = GetViewLineProjectToImageLineIdx(view_line_pair);
				//get the cam idx, line idx
				int camidx = visiable_line_idx[0];
				int line_visiable_idx = visiable_line_idx[1];
				std::pair<Eigen::Vector3f, Eigen::Vector3f> linesg3d
					= views_visiable_polygon_inter_lines_[camidx].view_visiable_lines_pnts_[line_visiable_idx];
				scenes_visisable_lines_pnts.emplace_back(linesg3d);
				Eigen::Vector3f linesg_dir = linesg3d.second - linesg3d.first;
				linesg_dir.normalize();
				//get the 
				int polygon_inter_line_idx = views_visiable_polygon_inter_lines_[camidx].view_visible_lines_idxs_[line_visiable_idx];
				PolygonInterLineScene tmp_poly_inter_line = polygons_intersection_scene_lines_[polygon_inter_line_idx];
				//set the value
				scenes_line3d_track_list_[i].polygon_idxs_.clear();
				scenes_line3d_track_list_[i].polygon_idxs_.emplace_back(tmp_poly_inter_line.adj_poly_idxs_[0]);
				scenes_line3d_track_list_[i].polygon_idxs_.emplace_back(tmp_poly_inter_line.adj_poly_idxs_[1]);
				std::cerr << "the scenes_line3d_track_list_[" << i << "].polygon_idxs_ size: " << scenes_line3d_track_list_[i].polygon_idxs_.size() << std::endl;
				scenes_line3d_track_list_[i].polygon_intersect_line_idx_[0] = polygon_inter_line_idx;

#if 0	//use the idx to access vec element
				for (int j = 0; j < scenes_line3d_track_list_[i].line_views_pnts_.size(); ++j)
				{
					//all project to linesg3d
					int view_pair_id = scenes_line3d_track_list_[i].lines_features_[j].view_id_;
					Eigen::Vector3f vls = scenes_line3d_track_list_[i].line_views_pnts_[j].first;
					Eigen::Vector3f vle = scenes_line3d_track_list_[i].line_views_pnts_[j].second;
					scenes_all_views_pnts.emplace_back(scenes_line3d_track_list_[i].line_views_pnts_[j]);
					Eigen::Vector3f vpls, vple;
					/*ComputePntProj2Line3DF(vls, linesg3d.first, linesg3d.second, vpls);
					ComputePntProj2Line3DF(vle, linesg3d.first, linesg3d.second, vple);*/
					ComputePntProj2Line3DF(linesg3d.first, linesg_dir, vls, vpls);
					ComputePntProj2Line3DF(linesg3d.first, linesg_dir, vle, vple);

					////filter the view that whose projected line far dist to origin line
					//float projline_to_line_dist = (vpls - vls).norm() + (vple - vle).norm();
					//projline_to_line_dist /= 2.0;
					//if (projline_to_line_dist > r_max_polygon_threshold_*2)
					//{
					//	scenes_line3d_track_list_[i].remove_view(view_pair_id);
					//}

					scenes_line3d_track_list_[i].line_views_pnts_[j].first = vpls;
					scenes_line3d_track_list_[i].line_views_pnts_[j].second = vple;

					scenes_all_views_projects_pnts.emplace_back(scenes_line3d_track_list_[i].line_views_pnts_[j]);
				}
#endif

				//use the iterator for the earase the view that is wrong
				HWLineFeatureReferenceIdList::iterator line_feature_iter
					= scenes_line3d_track_list_[i].lines_features_.begin();
				std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >::iterator line_pos_iter
					= scenes_line3d_track_list_[i].line_views_pnts_.begin();
				while (line_feature_iter != scenes_line3d_track_list_[i].lines_features_.end()
					&& line_pos_iter != scenes_line3d_track_list_[i].line_views_pnts_.end())
				{
					//all project to linesg3d
					int view_pair_id = line_feature_iter->view_id_;
					Eigen::Vector3f vls = line_pos_iter->first;
					Eigen::Vector3f vle = line_pos_iter->second;
					Eigen::Vector3f vpls, vple;
					/*ComputePntProj2Line3DF(vls, linesg3d.first, linesg3d.second, vpls);
					ComputePntProj2Line3DF(vle, linesg3d.first, linesg3d.second, vple);*/
					ComputePntProj2Line3DF(linesg3d.first, linesg_dir, vls, vpls);
					ComputePntProj2Line3DF(linesg3d.first, linesg_dir, vle, vple);

					//filter the view that whose projected line far dist to origin line
					float projline_to_line_dist = (vpls - vls).norm() + (vple - vle).norm();
					projline_to_line_dist /= 2.0;
					float r_max_polygon_threshold = 0.1;
					if (projline_to_line_dist > r_max_polygon_threshold)
					{
						std::cerr << "delete wrong view line match..." << std::endl;
						line_feature_iter = scenes_line3d_track_list_[i].lines_features_.erase(line_feature_iter);
						line_pos_iter = scenes_line3d_track_list_[i].line_views_pnts_.erase(line_pos_iter);
					}
					else
					{
						scenes_all_views_pnts.emplace_back(*line_pos_iter);
						line_pos_iter->first = vpls;
						line_pos_iter->second = vple;
						scenes_all_views_projects_pnts.emplace_back(*line_pos_iter);
						line_feature_iter++;
						line_pos_iter++;
					}
				}
			}
			// line to polygon line
			//find the minimum lines_dists
			//int lines
			std::vector<Eigen::Vector3f> view_line_pnts;
			for (int j = 0; j < scenes_line3d_track_list_[i].line_views_pnts_.size(); ++j)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> line_pnt = scenes_line3d_track_list_[i].line_views_pnts_[j];
				view_line_pnts.emplace_back(line_pnt.first);
				view_line_pnts.emplace_back(line_pnt.second);
			}
			Eigen::Vector3f tmp_ls, tmp_le;
			FittingLine3dFromPnts3d3f(view_line_pnts, tmp_ls, tmp_le);
			scenes_3d_test_lines.emplace_back(std::make_pair(tmp_ls, tmp_le));
			scenes_line3d_track_list_[i].s_pos_ = tmp_ls;
			scenes_line3d_track_list_[i].e_pos_ = tmp_le;
		}

		std::string sc_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj_test8/lines_tracks/scene_layout_observation.obj";
		WriteLine3DIntoObj(scenes_3d_test_lines, sc_path);
		std::string sc_seg_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj_test8/lines_tracks/scene_layout_visiable.obj";
		WriteLine3DIntoObj(scenes_visisable_lines_pnts, sc_seg_path);
		std::string sc_allview_project_seg_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj_test8/lines_tracks/scene_layout_all_views_project_pnts.obj";
		WriteLine3DIntoObj(scenes_all_views_projects_pnts, sc_allview_project_seg_path);

		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > scenes_all_views_to_project_view_pnts;
		for (int i = 0; i < scenes_all_views_projects_pnts.size(); ++i)
		{
			std::pair<Eigen::Vector3f, Eigen::Vector3f> linesg1 = scenes_all_views_projects_pnts[i];
			std::pair<Eigen::Vector3f, Eigen::Vector3f> linesg2 = scenes_all_views_pnts[i];
			Eigen::Vector3f mean_linesg1 = (linesg1.first + linesg1.second) / 2;
			Eigen::Vector3f mean_linesg2 = (linesg2.first + linesg2.second) / 2;
			scenes_all_views_to_project_view_pnts.emplace_back(std::make_pair(mean_linesg1, mean_linesg2));
		}

		std::string sc_all_view_seg_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj_test8/lines_tracks/scene_layout_all_views_pnts.obj";
		WriteLine3DIntoObj(scenes_all_views_pnts, sc_all_view_seg_path);
		std::string sc_allview_line2line_seg_path = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj_test8/lines_tracks/scene_layout_all_views_line2line_pnts.obj";
		WriteLine3DIntoObj(scenes_all_views_to_project_view_pnts, sc_allview_line2line_seg_path);

		std::cerr << "end fit line..." << std::endl;
	}

	void HWScenesElements::RunPolygonsInterLinesAdjacent()
	{
		std::cerr << "start to run polygon line adjacent..." << std::endl;
		for (int i = 0; i < polygons_intersection_lines_.size(); ++i)
		{
			std::vector<std::pair<int, float> > line2vs;
			Eigen::Vector3f sls = polygons_intersection_lines_[i].ls3d_;
			Eigen::Vector3f sle = polygons_intersection_lines_[i].le3d_;
			for (int j = 0; j < polygons_intersection_lines_.size(); ++j)
			{
				if (i == j)
				{
					continue;
				}
				Eigen::Vector3f tls = polygons_intersection_lines_[j].ls3d_;
				Eigen::Vector3f tle = polygons_intersection_lines_[j].le3d_;
				float angle = ComputeAngleBetweenTwoLineSegs(sls, sle, tls, tle);
				float dist1 = Point3D2LineSegMinDistance(sls, tls, tle);
				float dist2 = Point3D2LineSegMinDistance(sle, tls, tle);
				float min_dist = std::min(dist1, dist2);
				if (angle < trl2Interl_option_.line_angle_threhold_
					&& min_dist < trl2Interl_option_.line_dist_threshold_)
				{
					std::pair<Eigen::Vector3f, Eigen::Vector3f> sline = std::make_pair(sls, sle);
					std::pair<Eigen::Vector3f, Eigen::Vector3f> tline = std::make_pair(tls, tle);
				 	float value = ComputeTwoLineSegsEnergyValue(sline, tline);
					std::pair<int, float> idx2v = std::make_pair(j, value);
					line2vs.emplace_back(idx2v);
				}
			}
			std::sort(line2vs.begin(), line2vs.end(), MyLine3dSegValueCmpZDG);
			//check it 
			for (int j = 0; j < line2vs.size(); ++j)
			{
				int idx = line2vs[j].first;
				bool find_idx = false;
				for (int k = 0; k < polygons_intersection_lines_[i].adjacent_lines_idxs_.size(); ++k)
				{
					if (polygons_intersection_lines_[i].adjacent_lines_idxs_[k] == idx)
					{
						find_idx = true;
						break;
					}
				}
				if (!find_idx)
				{
					polygons_intersection_lines_[i].adjacent_lines_idxs_.emplace_back(idx);
				}
			}
		}
		std::cerr << "end run polygon line adjacent..." << std::endl;
	}

	void HWScenesElements::RunTrackLinesMatchPolygonsInterLines()
	{
		std::cerr << "---run track lines match to Polygons intersection lines---" << std::endl;
		std::cerr << "the polygon intersection lines num: " << polygons_intersection_lines_.size() << std::endl;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			const HWLineTrack3D& const track_line = scenes_line3d_track_list_[i];
			std::vector<std::pair<int, float> > track2inter_vs;
			std::pair<Eigen::Vector3f, Eigen::Vector3f> tl;
			tl.first = track_line.s_pos_;
			tl.second = track_line.e_pos_;
			for (int j = 0; j < polygons_intersection_lines_.size(); ++j)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> il;
				il.first = polygons_intersection_lines_[j].ls3d_;
				il.second = polygons_intersection_lines_[j].le3d_;
				//check visibility for the polygon intersection lines
				float ev = ComputeTwoLineSegsEnergyValue(tl, il);
				track2inter_vs.emplace_back(std::make_pair(j, ev));
			}
			std::sort(track2inter_vs.begin(), track2inter_vs.end(), MyLine3dSegValueCmpZDG);
			//get first two track2inter_vs lines
			int track2inter_vs_num = track2inter_vs.size();
			if (track2inter_vs_num >= 2)
			{
				scenes_line3d_track_list_[i].polygon_intersect_line_idx_[0] = track2inter_vs[0].first;
				scenes_line3d_track_list_[i].polygon_intersect_line_idx_[1] = track2inter_vs[1].first;
			}
		}
		std::cerr << "---end track lines match to Polygons intersection lines---" << std::endl;

#if 0
		//test the match
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > lines3d_first;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			const HWLineTrack3D& const track_line = scenes_line3d_track_list_[i];
			//std::vector<std::pair<int, float> > track2inter_vs;
			std::pair<Eigen::Vector3f, Eigen::Vector3f> tl;
			tl.first = track_line.s_pos_;
			tl.second = track_line.e_pos_;
			Eigen::Vector3f t_m = (tl.first + tl.second) / 2.0;
			int inter_idx0 = scenes_line3d_track_list_[i].polygon_intersect_line_idx_[0];
			if (inter_idx0 != -1)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> il;
				il.first = polygons_intersection_lines_[inter_idx0].ls3d_;
				il.second = polygons_intersection_lines_[inter_idx0].le3d_;
				Eigen::Vector3f i_m = (il.first + il.second) / 2.0;
				std::pair<Eigen::Vector3f, Eigen::Vector3f> tim = std::make_pair(t_m, i_m);
				lines3d_first.emplace_back(tl);
				lines3d_first.emplace_back(il);
				lines3d_first.emplace_back(tim);
			}
		}
		std::string path_first = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images_1/trackline3d2InterLineFirst.obj";
		WriteLine3DIntoObj(lines3d_first, path_first);

		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > lines3d_second;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			const HWLineTrack3D& const track_line = scenes_line3d_track_list_[i];
			//std::vector<std::pair<int, float> > track2inter_vs;
			std::pair<Eigen::Vector3f, Eigen::Vector3f> tl;
			tl.first = track_line.s_pos_;
			tl.second = track_line.e_pos_;
			Eigen::Vector3f t_m = (tl.first + tl.second) / 2.0;
			int inter_idx1 = scenes_line3d_track_list_[i].polygon_intersect_line_idx_[1];
			if (inter_idx1 != -1)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> il;
				il.first = polygons_intersection_lines_[inter_idx1].ls3d_;
				il.second = polygons_intersection_lines_[inter_idx1].le3d_;
				Eigen::Vector3f i_m = (il.first + il.second) / 2.0;
				std::pair<Eigen::Vector3f, Eigen::Vector3f> tim = std::make_pair(t_m, i_m);
				lines3d_second.emplace_back(tl);
				lines3d_second.emplace_back(il);
				lines3d_second.emplace_back(tim);
			}
		}
		std::string path_second = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images_1/trackline3d2InterLinesecond.obj";
		WriteLine3DIntoObj(lines3d_second, path_second);

		//end test the match
#endif

	}

	void HWScenesElements::FilterErrorTrackLinesMatchInterLines()
	{
		std::cerr << "---run filter error track lines match to Polygons intersection lines---" << std::endl;
		std::cerr << "the polygon intersection lines num: " << polygons_intersection_lines_.size() << std::endl;
		
		//to do next...
		//将所有的track list 3d 匹配的 intersection line，判断可视性，且反投到图像上，发现它们都离附近的线段都很近
		//表示这些匹配有错,去掉。因为在图像上它们出现问题。
		//今天晚上完成这个功能。
		//for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		//{
		//	const HWLineTrack3D& const track_line = scenes_line3d_track_list_[i];
		//	if (track_line.polygon_intersect_line_idx_[0] != -1)
		//	{
		//		//Get all the intersection line
		//		HWPolygonInterLines inter_line = polygons_intersection_lines_[track_line.polygon_intersect_line_idx_[0]];
		//		Eigen::Vector3f inter_line_s = inter_line.ls3d_;
		//		Eigen::Vector3f inter_line_e = inter_line.le3d_;
		//		//判断可视性。
		//	}
		//}
		
		std::vector<float> evs;	//get the average energy value
		float sum_value = 0.0;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			const HWLineTrack3D& const track_line = scenes_line3d_track_list_[i];
			if (track_line.polygon_intersect_line_idx_[0] != -1)
			{
				//Get all the intersection line
				HWPolygonInterLines inter_line = polygons_intersection_lines_[track_line.polygon_intersect_line_idx_[0]];
				Eigen::Vector3f inter_line_s = inter_line.ls3d_;
				Eigen::Vector3f inter_line_e = inter_line.le3d_;
				std::pair<Eigen::Vector3f, Eigen::Vector3f> interline;
				interline.first = inter_line_s;
				interline.second = inter_line_e;
				//get every view line
				for (int j = 0; j < track_line.line_views_pnts_.size(); ++j)
				{
					std::pair<Eigen::Vector3f, Eigen::Vector3f> trackline;
					trackline.first = track_line.line_views_pnts_[j].first;
					trackline.second = track_line.line_views_pnts_[j].second;
					float ev = ComputeTwoLineSegsEnergyValue(interline, trackline);
					sum_value += ev;
					evs.emplace_back(ev);
				}
			}
		}
		//
		float evs_average_value = 0.0;
		int evs_num = static_cast<int> (evs.size());
		if (evs_num > 0)
		{
			evs_average_value = sum_value / evs_num;
		}
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			const HWLineTrack3D& const track_line = scenes_line3d_track_list_[i];
			if (track_line.polygon_intersect_line_idx_[0] != -1)
			{
				//Get all the intersection line
				HWPolygonInterLines inter_line = polygons_intersection_lines_[track_line.polygon_intersect_line_idx_[0]];
				Eigen::Vector3f inter_line_s = inter_line.ls3d_;
				Eigen::Vector3f inter_line_e = inter_line.le3d_;
				std::pair<Eigen::Vector3f, Eigen::Vector3f> interline;
				interline.first = inter_line_s;
				interline.second = inter_line_e;
				//get every view line
				for (int j = 0; j < track_line.line_views_pnts_.size(); ++j)
				{
					std::pair<Eigen::Vector3f, Eigen::Vector3f> trackline;
					trackline.first = track_line.line_views_pnts_[j].first;
					trackline.second = track_line.line_views_pnts_[j].second;
					float ev = ComputeTwoLineSegsEnergyValue(trackline, interline);
					if (ev > evs_average_value)
					{
						scenes_line3d_track_list_[i].polygon_intersect_line_idx_[0] = -1;
						scenes_line3d_track_list_[i].polygon_intersect_line_idx_[1] = -1;
					}
				}
			}
		}
		
		//test 
		//std::string path_first = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images_1/FilterTrackline3d2InterLine.obj";
		//WriteHWPolygonInterLineToLine3d(path_first);
		//end test

#if 0
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			const HWLineTrack3D& const track_line = scenes_line3d_track_list_[i];
			if (track_line.polygon_intersect_line_idx_[0] != -1)
			{
				//Get all the intersection line
				HWPolygonInterLines inter_line = polygons_intersection_lines_[track_line.polygon_intersect_line_idx_[0]];
				Eigen::Vector3f line_s = inter_line.ls3d_;
				Eigen::Vector3f line_e = inter_line.le3d_;
				Eigen::Vector3f pnt_mean = (line_s + line_e) / 2;
				//get every view line
				for (int j = 0; j < track_line.lines_features_.size(); ++j)
				{
					int camid = track_line.lines_features_[j].view_id_;
					//get camera pose
					CameraModel view_model = scenes_cams_->GetCameraModelFromCamId(camid);
					Eigen::Vector3f cmc = view_model.GetCamC();
					float cmc2line = (cmc - pnt_mean).norm();
					// to do next...
				}
			}
		}
#endif	
		//test
		//end test
		std::cerr << "---end filter error line match to polygons intersection lines---" << std::endl;
	}

	void HWScenesElements::FilterErrorTrackLines2RedauntImagesLines()
	{
		std::cerr << "---run filter error track lines match to multiply images lines---" << std::endl;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			HWLineFeatureReferenceIdList line_views_idxs = scenes_line3d_track_list_[i].lines_features_;
			//check if the 
			std::cerr << "FilterErrorTrackLines2RedauntImagesLines i: " << i << std::endl;
			bool multiply_lines_flag = false;
			std::vector<int> view_ids;
			for (int j = 0; j < line_views_idxs.size(); ++j)
			{
				view_ids.emplace_back(line_views_idxs[j].view_id_);
			}
			int view_ids_num = static_cast<int>(view_ids.size());
			if (view_ids_num < 2)
			{
				continue;
			}
			std::sort(view_ids.begin(), view_ids.end());
			for (int j = 0; j < view_ids.size() - 1; ++j)
			{
				if (view_ids[j] == view_ids[j + 1])
				{
					multiply_lines_flag = true;
					break;
				}
			}
			if (multiply_lines_flag)
			{
				scenes_line3d_track_list_[i].is_valid_ = false;
			}
		}
		std::cerr << "---end filter error track lines match to multiply images lines---" << std::endl;
	}

	void HWScenesElements::FilterInvalidTrackPointsAndViewsPointsObservation()
	{
		//std::cerr << "to do next..." << std::endl;
		//back up them
		scenes_pnt3ds_observations_pre_ = scenes_pnt3ds_observations_;
		HWSurveyPoint3d2Pnts2dList scenes_pnt3ds_observation_filter;
		for (int i = 0; i < scenes_pnt3ds_observations_.size(); ++i)
		{
			HWSurveyPoint3d2Pnts2d tmp_scenes_pnt3d_observations;
			tmp_scenes_pnt3d_observations.pos_ = scenes_pnt3ds_observations_[i].pos_;
			tmp_scenes_pnt3d_observations.plane_idxs_ = scenes_pnt3ds_observations_[i].plane_idxs_;
			HWSurveyObservationPnt2dList tmp_pnt2d_list;
			for (int j = 0; j < scenes_pnt3ds_observations_[i].observations_.size(); ++j)
			{
				if (scenes_pnt3ds_observations_[i].observations_[j].is_valid_)
				{
					tmp_pnt2d_list.emplace_back(scenes_pnt3ds_observations_[i].observations_[j]);
				}
			}
			if (tmp_pnt2d_list.size() >= 2)
			{
				tmp_scenes_pnt3d_observations.observations_ = tmp_pnt2d_list;
				scenes_pnt3ds_observation_filter.emplace_back(tmp_scenes_pnt3d_observations);
			}
		}
		scenes_pnt3ds_observations_ = scenes_pnt3ds_observation_filter;
	}

	void HWScenesElements::FilterInvalidTrackLinesAndViewsLinesObservation()
	{
		//std::cerr << "to do next..." << std::endl;
		//back up them
		scenes_lines_pnts3ds_observations_pre_ = scenes_lines_pnts3ds_observations_;
		HWLineSurveyPoint3d2Pnts2dList scenes_lines_points3ds_observation_filter;
		for (int i = 0; i < scenes_lines_pnts3ds_observations_.size(); ++i)
		{
			HWLineSurveyPoint3d2Pnts2d tmp_scenes_line_points3d_observations;
			tmp_scenes_line_points3d_observations.pos_s_ = scenes_lines_pnts3ds_observations_[i].pos_s_;
			tmp_scenes_line_points3d_observations.pos_e_ = scenes_lines_pnts3ds_observations_[i].pos_e_;
			tmp_scenes_line_points3d_observations.plane_idxs_ = scenes_lines_pnts3ds_observations_[i].plane_idxs_;
			tmp_scenes_line_points3d_observations.is_plane_lines_ = scenes_lines_pnts3ds_observations_[i].is_plane_lines_;
			HWLineSurveyObservationPnt2dList tmp_scenes_line_pnts3d_pnt2dlist;
			for (int j = 0; j < scenes_lines_pnts3ds_observations_[i].observations_.size(); ++j)
			{
				HWLineSurveyObservationPnt2d tmp_scenes_line_observation_pnt2d = scenes_lines_pnts3ds_observations_[i].observations_[j];
				if (tmp_scenes_line_observation_pnt2d.is_valid_)
				{
					tmp_scenes_line_pnts3d_pnt2dlist.emplace_back(tmp_scenes_line_observation_pnt2d);
				}
			}
			if (tmp_scenes_line_pnts3d_pnt2dlist.size() >= 2)
			{
				tmp_scenes_line_points3d_observations.observations_ = tmp_scenes_line_pnts3d_pnt2dlist;
				scenes_lines_points3ds_observation_filter.emplace_back(tmp_scenes_line_points3d_observations);
			}
		}
		scenes_lines_pnts3ds_observations_ = scenes_lines_points3ds_observation_filter;
	}

	void HWScenesElements::UpdatePntsTracksByLinesTracks()
	{
		std::cerr << "to do next..." << std::endl;
	}

	void HWScenesElements::SaveOptiCamsIntoCamFiles()
	{
		for (int i = 0; i < hw_cams_views_list_.size(); ++i)
		{
			HWCamViewport tmp_cam_view = hw_cams_views_list_[i];
			int cam_id = tmp_cam_view.hw_camera_id_;
			CameraModel cam = tmp_cam_view.view_pose_;
			std::string cam_path = cam.path_;
			std::ofstream outFile(cam_path);
			Eigen::Matrix4f cam_extr = cam.cam_pose_.inverse();
			outFile << cam_extr(0, 3) << " " << cam_extr(1, 3) << " " << cam_extr(2, 3) << " ";
			outFile << cam_extr(0, 0) << " " << cam_extr(0, 1) << " " << cam_extr(0, 2) << " ";
			outFile << cam_extr(1, 0) << " " << cam_extr(1, 1) << " " << cam_extr(1, 2) << " ";
			outFile << cam_extr(2, 0) << " " << cam_extr(2, 1) << " " << cam_extr(2, 2) << "\n";
			outFile << 0.5 << "\n";
			outFile.close();
		}
	}

	void HWScenesElements::SaveOptiCamsIntoDir(const std::string& dir)
	{
		for (int i = 0; i < hw_cams_views_list_.size(); ++i)
		{
			HWCamViewport tmp_cam_view = hw_cams_views_list_[i];
			int cam_id = tmp_cam_view.hw_camera_id_;
			CameraModel cam = tmp_cam_view.view_pose_;
			std::string cam_base_name = GetBaseName(cam.path_);
			std::string cam_path = dir + "/" + cam_base_name;
			std::ofstream outFile(cam_path);
			Eigen::Matrix4f cam_extr = cam.cam_pose_.inverse();
			outFile << cam_extr(0, 3) << " " << cam_extr(1, 3) << " " << cam_extr(2, 3) << " ";
			outFile << cam_extr(0, 0) << " " << cam_extr(0, 1) << " " << cam_extr(0, 2) << " ";
			outFile << cam_extr(1, 0) << " " << cam_extr(1, 1) << " " << cam_extr(1, 2) << " ";
			outFile << cam_extr(2, 0) << " " << cam_extr(2, 1) << " " << cam_extr(2, 2) << "\n";
			//outFile << 0.5 << "\n";
			outFile << cam.fx_ << " " << cam.fy_ << " " << cam.cx_ << " " << cam.cy_ << "\n";
			outFile.close();
		}
	}

	void HWScenesElements::SaveImagesLinesIntoTxtsFromDir(const std::string& dir)
	{
		scenes_layouts_elements_.get()->SetSceneOutDir(dir);
		scenes_layouts_elements_.get()->SaveAllImagesLayouts2dIntoTxtsFromSceneOutDir();
	}

#if 0
	void HWScenesElements::RunImgsPntsMatchesFromTwoImgsMatch(unsigned int src_cid, unsigned int tgt_cid)
	{
		const std::vector<std::string> image_paths = scenes_cams_->GetImagesPaths();
		int srccamid = static_cast<int>(src_cid);
		int tgtcamid = static_cast<int>(tgt_cid);
		int src_image_id = scenes_cams_->GetImageidFromCamId(srccamid);
		int tgt_image_id = scenes_cams_->GetImageidFromCamId(tgtcamid);
		const CameraModel src_cam = scenes_cams_->GetCameraModelFromCamId(src_cid);
		const CameraModel tgt_cam = scenes_cams_->GetCameraModelFromCamId(tgt_cid);

		//get image to do next...
		HWImage srcimg = scenes_cams_->GetImageFromImgId(src_image_id);
		HWImage tgtimg = scenes_cams_->GetImageFromImgId(tgt_image_id);

		std::vector<cv::KeyPoint> src_keypnts = srcimg.GetSiftFeatruesPosition();
		cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> src_feature_desc = srcimg.GetSiftDescriptorPtr();
		cv::Mat src_cv_img = srcimg.GetImage();
		std::vector<cv::KeyPoint> tgt_keypnts = tgtimg.GetSiftFeatruesPosition();
		cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> tgt_feature_desc = tgtimg.GetSiftDescriptorPtr();
		cv::Mat tgt_cv_img = tgtimg.GetImage();

		cv::Mat src_descriptor_image_, tgt_descriptor_image_;
		src_feature_desc->compute(src_cv_img, src_keypnts, src_descriptor_image_);
		tgt_feature_desc->compute(tgt_cv_img, tgt_keypnts, tgt_descriptor_image_);
		std::vector<cv::DMatch> matches_cv;
		//使用参数来匹配
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::FlannBasedMatcher::create();
		matcher->match(src_descriptor_image_, tgt_descriptor_image_, matches_cv);
		int num_matches = static_cast<int>(matches_cv.size());

		/* Build correspondences from feature matching result. */
		HWTwoViewsCamsMatching match_views;
		//get line match from matches
		for (std::size_t i = 0; i < matches_cv.size(); ++i)
		{
			if (matches_cv[i].queryIdx < 0 || matches_cv[i].trainIdx < 0)
				continue;
			HWCorrespondence2D2DPnt match_pnt;
			match_pnt.p1[0] = src_keypnts[matches_cv[i].queryIdx].pt.x;
			match_pnt.p1[1] = src_keypnts[matches_cv[i].queryIdx].pt.y;
			match_pnt.p2[0] = tgt_keypnts[matches_cv[i].trainIdx].pt.x;
			match_pnt.p2[1] = tgt_keypnts[matches_cv[i].trainIdx].pt.y;

			int src_pnt_idx = matches_cv[i].queryIdx;
			int tgt_pnt_idx = matches_cv[i].trainIdx;

			//check the pnt coorindate 
			if (filter_wrong_lines_match_)
			{
				//get polygon
				Eigen::Vector3f src_pnt3d, tgt_pnt3d;
				bool src_pnt3d_flag = ImagePnt2dProj2ScenePolygonPnt3d(src_cam, match_pnt.p1, src_pnt3d);
				bool tgt_pnt3d_flag = ImagePnt2dProj2ScenePolygonPnt3d(tgt_cam, match_pnt.p2, tgt_pnt3d);
				if (src_pnt3d_flag && tgt_pnt3d_flag)
				{
					//check pnt is close enough
					if (CheckTwoPnt3dTheshold(src_pnt3d, tgt_pnt3d, r_max_threshold_))
					{
						HWPntsMatch cor_pnt;
						cor_pnt.pnts_matches_idx_.first = src_pnt_idx;
						cor_pnt.pnts_matches_idx_.second = tgt_pnt_idx;
						Eigen::Vector3f tri_pnt = (src_pnt3d + tgt_pnt3d) / 2;
						cor_pnt.view12_tri_pnt3d_ = tri_pnt;
						cor_pnt.corresponding_pnts_ = match_pnt;
						match_views.points_matches.emplace_back(cor_pnt);
					}
				}
			}
			//unfiltered_matches.push_back(match);
			//unfiltered_indices.push_back(std::make_pair(matches_cv[i].queryIdx, matches_cv[i].trainIdx));
			//matches->push_back(unfiltered_indices[i]);
		}
		

		/*std::vector<> unfiltered_matches;
		HWCorrespondenceIndices unfiltered_indices;
		for (std::size_t i = 0; i < matches_cv.size(); ++i)
		{
			if (matches_cv[i].queryIdx < 0 || matches_cv[i].trainIdx < 0)
				continue;
			HWCorrespondence2D2D match;
			match.p1[0] = view_1.positions[matches_cv[i].queryIdx].pt.x;
			match.p1[1] = view_1.positions[matches_cv[i].queryIdx].pt.y;
			match.p2[0] = view_2.positions[matches_cv[i].trainIdx].pt.x;
			match.p2[1] = view_2.positions[matches_cv[i].trainIdx].pt.y;
			unfiltered_matches.push_back(match);
			unfiltered_indices.push_back(std::make_pair(matches_cv[i].queryIdx, matches_cv[i].trainIdx));
			matches->push_back(unfiltered_indices[i]);
		}*/


		//HWFeatureSet view_1 = this->viewports->at(view_1_id).features_;
		//HWFeatureSet view_2 = this->viewports->at(view_2_id).features_;
		//std::vector<cv::DMatch> matches_cv;
		//if (view_1.get_options().feature_types != view_2.get_options().feature_types)
		//{
		//	std::cerr << "invalid feature type..." << std::endl;
		//	return;
		//}
		////使用参数来匹配
		//cv::Ptr<cv::DescriptorMatcher> matcher = cv::FlannBasedMatcher::create();
		//matcher->match()
		//matcher->match(*view_1.descriptor_image_, *view_2.descriptor_image_, matches_cv);
		//int num_matches = static_cast<int>(matches_cv.size());
		///* Build correspondences from feature matching result. */
		//HWCorrespondences2D2D unfiltered_matches;
		//HWCorrespondenceIndices unfiltered_indices;
		//for (std::size_t i = 0; i < matches_cv.size(); ++i)
		//{
		//	if (matches_cv[i].queryIdx < 0 || matches_cv[i].trainIdx < 0)
		//		continue;
		//	HWCorrespondence2D2D match;
		//	match.p1[0] = view_1.positions[matches_cv[i].queryIdx].pt.x;
		//	match.p1[1] = view_1.positions[matches_cv[i].queryIdx].pt.y;
		//	match.p2[0] = view_2.positions[matches_cv[i].trainIdx].pt.x;
		//	match.p2[1] = view_2.positions[matches_cv[i].trainIdx].pt.y;
		//	unfiltered_matches.push_back(match);
		//	unfiltered_indices.push_back(std::make_pair(matches_cv[i].queryIdx, matches_cv[i].trainIdx));
		//	matches->push_back(unfiltered_indices[i]);
		//}

		std::string src_path = srcimg.GetImagePath();
		std::string tgt_path = tgtimg.GetImagePath();

	}
#endif

    bool HWScenesElements::SetMatchesLinesValid(unsigned int src_camid, unsigned int srclid, 
        unsigned int tgt_camid, unsigned int tgtlid, bool my_valid)
    {
        //matches_[src_camid]
        std::list<HWMatch>::iterator it;
        for(it = matches_[src_camid][srclid].begin(); it != matches_[src_camid][srclid].end(); ++it)
        {
            if(it->tgt_camID_ == tgt_camid && it->tgt_segID_ == tgtlid)
            {
                it->valid_match_ = my_valid;
                return true;
            }
        }
        return false;
    }

    void HWScenesElements::WriteLine3DIntoObj(const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& lines, 
        const std::string& path)
    {
        std::ofstream fh(path);
        for(std::size_t i = 0; i < lines.size(); ++i)
        {
            fh << "v " << lines[i].first[0] << " " << lines[i].first[1] << " " << lines[i].first[2] << std::endl;
            fh << "v " << lines[i].second[0] << " " << lines[i].second[1] << " " << lines[i].second[2] << std::endl;
        }
        for(std::size_t i = 0; i < lines.size(); ++i)
        {
            fh << "l " << i*2+1 << " " << i*2+2 << std::endl;
        }
        fh.close();
    }

	void HWScenesElements::WritePntsIntoObj(const std::vector<Eigen::Vector3f>& pnts, const std::string& path)
	{
		std::ofstream fh(path);
		for (std::size_t i = 0; i < pnts.size(); ++i)
		{
			fh << "v " << pnts[i][0] << " " << pnts[i][1] << " " << pnts[i][2] << std::endl;
		}
		fh.close();
	}

	const HWScenesCams* HWScenesElements::GetHWScenesCamsImgs()
	{
		return scenes_cams_.get();
	}

	const HWSceneLayouts* HWScenesElements::GetHWLayouts()
	{
		return scenes_layouts_elements_.get();
	}

	void HWScenesElements::ConvertNumpyPntsToPnts(const cnpy::NpyArray& ny_pnts,
		std::vector<Eigen::Vector2f>& pnts)
	{
		pnts.clear();
		std::vector<size_t> ny_shape = ny_pnts.shape;
		if (ny_pnts.word_size == sizeof(double))
		{
			std::cerr << "ConvertNumpyPntsToPnts: double type..." << std::endl;
		}
		if (ny_pnts.word_size == sizeof(float))
		{
			std::cerr << "ConvertNumpyPntsToPnts: float type..." << std::endl;
		}
		const float* ny_pnts_pos = ny_pnts.data<float>();
		int pnts_num = ny_shape[0];
		for (int i = 0; i < pnts_num; ++i)
		{
			Eigen::Vector2f tmp_pnt;
			tmp_pnt[0] = ny_pnts_pos[2 * i];
			tmp_pnt[1] = ny_pnts_pos[2 * i + 1];
			pnts.emplace_back(tmp_pnt);
		}
	}

	void HWScenesElements::ConvertNumpyPntsIdxToPntsIdx(const cnpy::NpyArray& ny_idxs,
		std::vector<Eigen::Vector2i>& pnts_idxs)
	{
		if (ny_idxs.word_size == sizeof(double))
		{
			std::cerr << "ConvertNumpyPntsIdxToPntsIdx: double type..." << std::endl;
		}
		if (ny_idxs.word_size == sizeof(float))
		{
			std::cerr << "ConvertNumpyPntsIdxToPntsIdx: float type..." << std::endl;
		}
		const double* matches_kpts = ny_idxs.data<double>();
		std::vector<size_t> matches_shape = ny_idxs.shape;
		if (matches_shape.size() != 2)
		{
			return;
		}
		pnts_idxs.clear();
		int matches_kpts_num = matches_shape[0] * matches_shape[1];
		std::vector<std::vector<double> > pnts_match_mats;
		pnts_match_mats.resize(matches_shape[0]);
		for (int i = 0; i < matches_shape[0]; ++i)
		{
			std::vector<double> r_idxs;
			for (int j = 0; j < matches_shape[1]; ++j)
			{
				int idx = i * matches_shape[1] + j;
				r_idxs.emplace_back(matches_kpts[idx]);
				if (matches_kpts[idx] > 0)
				{
					/*std::cerr << "pnts: lidx, ridx(" << i << ", " << j <<
					"): " << matches_kpts[idx] << std::endl;*/
					pnts_idxs.emplace_back(Eigen::Vector2i(i, j));
				}
			}
			pnts_match_mats[i] = r_idxs;
		}

#if 0 
		std::vector<int> pntsidxs;
		for (int j = 0; j < matches_kpts_num; ++j)
		{
			if (matches_kpts[j] > 0)
			{
				//std::cerr << matches_kpts[j] << std::endl;
				pntsidxs.emplace_back(j);
			}
		}
		for (int i = 0; i < pntsidxs.size(); ++i)
		{
			int left_idx = static_cast<int> (pntsidxs[i] / matches_shape[1]);
			int right_idx = pntsidxs[i] % matches_shape[1];
			//std::cerr << "left_idx, right_idx: "<< left_idx << ", " << right_idx << std::endl;
			pnts_idxs.emplace_back(Eigen::Vector2i(left_idx, right_idx));
		}
#endif
	}

	void HWScenesElements::ConvertNumpyLinesPntsToLinesPnts(const cnpy::NpyArray& ny_lines,
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& lines)
	{
		lines.clear();
		std::vector<size_t> ny_shape = ny_lines.shape;
		if (ny_lines.word_size == sizeof(double))
		{
			std::cerr << "ConvertNumpyLinesPntsToLinesPnts: double type..." << std::endl;
		}
		if (ny_lines.word_size == sizeof(float))
		{
			std::cerr << "ConvertNumpyLinesPntsToLinesPnts: float type..." << std::endl;
		}
		const float* ny_pnts_pos = ny_lines.data<float>();
		int lines_num = ny_shape[0];
		for (int i = 0; i < lines_num; ++i)
		{
			float line0_s = ny_pnts_pos[4*i];
			float line0_e = ny_pnts_pos[4*i + 1];
			float line1_s = ny_pnts_pos[4*i + 2];
			float line1_e = ny_pnts_pos[4*i + 3];
			std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_line;
			tmp_line.first[0] = line0_s;
			tmp_line.first[1] = line0_e;
			tmp_line.second[0] = line1_s;
			tmp_line.second[1] = line1_e;
			lines.emplace_back(tmp_line);
		}
	}

#if 0
	void HWScenesElements::ConvertNumpyLinesIdxToLinesIdx(const cnpy::NpyArray& ny_idxs,
		std::vector<Eigen::Vector2i>& lines_idxs)
	{
		const float* matches_klines = ny_idxs.data<float>();
		std::vector<size_t> matches_shape = ny_idxs.shape;
		if (matches_shape.size() != 2)
		{
			return;
		}
		lines_idxs.clear();
		std::vector<std::vector<float> > lines_match_mats;
		lines_match_mats.resize(matches_shape[0]);
		int icount = 0;
		for (int i = 0; i < matches_shape[0]; ++i)
		{
			std::vector<float> r_idxs;
			for (int j = 0; j < matches_shape[1]; ++j)
			{
				int idx = i * matches_shape[1] + j;
				r_idxs.emplace_back(matches_klines[idx]);
				if (matches_klines[idx] > 0)
				{
					std::cerr << icount << ": lidx, ridx(" << i << ", " << j <<
						"): " << matches_klines[idx] << std::endl;
					lines_idxs.emplace_back(Eigen::Vector2i(i, j));
					++icount;
				}
			}
			lines_match_mats[i] = r_idxs;
		}

	}
#endif

	void HWScenesElements::ConvertNumpyLinesIdxToLinesIdx(const cnpy::NpyArray& ny_idxs,
		std::vector<Eigen::Vector2i>& lines_idxs)
	{

		if (ny_idxs.word_size == sizeof(double))
		{
			std::cerr << "ConvertNumpyLinesIdxToLinesIdx: double type..." << std::endl;
		}
		if (ny_idxs.word_size == sizeof(float))
		{
			std::cerr << "ConvertNumpyLinesIdxToLinesIdx: float type..." << std::endl;
		}
		//if (ny_idxs.word_size == sizeof(int))
		//{
		//	std::cerr << "int type..." << std::endl;
		//}
		//if (ny_idxs.word_size == sizeof(uchar))
		//{
		//	std::cerr << "uchar type..." << std::endl;
		//}
		//if (ny_idxs.word_size == sizeof(unsigned int))
		//{
		//	std::cerr << "unsigned int type..." << std::endl;
		//}
		//if (ny_idxs.word_size == sizeof(long int))
		//{
		//	std::cerr << "long int type..." << std::endl;
		//}
		//if (ny_idxs.word_size == sizeof(short int))
		//{
		//	std::cerr << "short int type..." << std::endl;
		//}
		////short int long int

		const double* matches_klines = ny_idxs.data<double>();	//const type
		std::vector<size_t> matches_shape = ny_idxs.shape;
		if (matches_shape.size() != 2)
		{
			return;
		}
		lines_idxs.clear();
		std::vector<std::vector<double> > lines_match_mats;
		lines_match_mats.resize(matches_shape[0]);
		//int icount = 0;
		for (int i = 0; i < matches_shape[0]; ++i)
		{
			std::vector<double> r_idxs;
			for (int j = 0; j < matches_shape[1]; ++j)
			{
				int idx = i * matches_shape[1] + j;
				r_idxs.emplace_back(matches_klines[idx]);
				if (matches_klines[idx] > 0.0)
				{
					/*std::cerr << icount << ": lidx, ridx(" << i << ", " << j <<
						"): " << matches_klines[idx] << std::endl;*/
					lines_idxs.emplace_back(Eigen::Vector2i(i, j));
					//++icount;
				}
			}
			lines_match_mats[i] = r_idxs;
		}

#if 0
		std::vector<int> linesidxs;
		for (int j = 0; j < matches_klines_num; ++j)
		{
			if (matches_klines[j] > 0)
			{
				//std::cerr << matches_kpts[j] << std::endl;
				linesidxs.emplace_back(j);
			}
		}
		for (int i = 0; i < linesidxs.size(); ++i)
		{
			int left_idx = static_cast<int> (linesidxs[i] / matches_shape[1]);
			int right_idx = linesidxs[i] % matches_shape[1];
			//std::cerr << "left_idx, right_idx: " << left_idx << ", " << right_idx << std::endl;
			lines_idxs.emplace_back(Eigen::Vector2i(left_idx, right_idx));
		}
#endif

	}

	void HWScenesElements::PrintPyNpyShape(const cnpy::NpyArray& ny_data)
	{
		std::vector<size_t> my_shape = ny_data.shape;
		std::cerr << "[ ";
		for (int i = 0; i < my_shape.size(); ++i)
		{
			if(i != my_shape.size() - 1)
				std::cerr << my_shape[i] <<", ";
			else
				std::cerr << my_shape[i] << " ]";
		}
		std::cerr << std::endl;
	}

	void HWScenesElements::PrintHWSecenesElementCamsInfo()
	{
		int cams_nums = scenes_cams_->GetCamerasNum();
		std::vector<CameraModel> scene_cams_model = scenes_cams_->GetCamerasModels();
		std::cerr << "cameras models info: " << cams_nums << std::endl;
		for (int i = 0; i < scene_cams_model.size(); ++i)
		{
			std::cerr << i << ": " << std::endl;
			CameraModel tmp_cam = scene_cams_model[i];
			std::cerr << "path: " << tmp_cam.path_ << std::endl;
			float fx = tmp_cam.fx_;
			float fy = tmp_cam.fy_;
			float cx = tmp_cam.cx_;
			float cy = tmp_cam.cy_;
			std::cerr << "intrinsic: " << fx << " " << fy << " " << cx << " " << cy << std::endl;
			std::cerr << "extrinsic: " << std::endl;
			std::cerr << tmp_cam.cam_pose_ << std::endl;
			if (tmp_cam.valid_)
			{
				std::cerr << "cam is valid ..." << std::endl;
			}
			else
			{
				std::cerr << "cam is invalid ..." << std::endl;
			}
			std::cerr << "result cam_id: " << tmp_cam.cam_id_ << std::endl;
			std::cerr << "result image_id: " << tmp_cam.image_id_ << std::endl;
			std::cerr << "layout id: " << tmp_cam.layout_id_ << std::endl << std::endl;
		}	
	}

	void HWScenesElements::PrintHWSecnesElementLayoutsInfo()
	{
		if (scenes_layouts_elements_)
		{
			int layout_num = scenes_layouts_elements_->GetScenelayout2DNum();
			std::cerr << "scenes layout info: " << layout_num << std::endl;
			const std::vector<std::unique_ptr<HWSceneLayout2D> >& scenes_lyouts = scenes_layouts_elements_->GetSceneLayouts2D();
			for (int i = 0; i < scenes_lyouts.size(); ++i)
			{
				std::cerr << i << ": " << std::endl;
				std::cerr << "scene_layout id: " << scenes_lyouts[i]->GetLayoutId() << std::endl;
				std::cerr << "scenes_layout path: " << scenes_lyouts[i]->GetLayout2DPath() << std::endl;
				std::cerr << "scene_layout to image_id: " << scenes_lyouts[i]->GetImageId() << std::endl;
				std::cerr << "lines nums: " << scenes_lyouts[i]->GetLayoutLinesNum() << std::endl;
				//to do next...
			}
		}
	}

	void HWScenesElements::PrintHWScenesPolygonsInfo()
	{
		int polygons_num = associated_polygons_.size();
		std::cerr << "HWPolygon info: " << polygons_num << std::endl;
		for (int i = 0; i < associated_polygons_.size(); ++i)
		{
			std::cerr << "pnts num: " << i << ": " << std::endl;
			associated_polygons_[i]->ScoutCornerPoints3D();
			std::cerr << std::endl << std::endl;
		}
	}

	void HWScenesElements::WriteHWPolygonInterLineToLine3d(const std::string& path)
	{
		//test the match
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > lines3d_first;
		for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		{
			const HWLineTrack3D& const track_line = scenes_line3d_track_list_[i];
			//std::vector<std::pair<int, float> > track2inter_vs;
			std::pair<Eigen::Vector3f, Eigen::Vector3f> tl;
			tl.first = track_line.s_pos_;
			tl.second = track_line.e_pos_;
			Eigen::Vector3f t_m = (tl.first + tl.second) / 2.0;
			int inter_idx0 = scenes_line3d_track_list_[i].polygon_intersect_line_idx_[0];
			if (inter_idx0 != -1)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> il;
				il.first = polygons_intersection_lines_[inter_idx0].ls3d_;
				il.second = polygons_intersection_lines_[inter_idx0].le3d_;
				Eigen::Vector3f i_m = (il.first + il.second) / 2.0;
				std::pair<Eigen::Vector3f, Eigen::Vector3f> tim = std::make_pair(t_m, i_m);
				lines3d_first.emplace_back(tl);
				lines3d_first.emplace_back(il);
				lines3d_first.emplace_back(tim);
			}
		}
		WriteLine3DIntoObj(lines3d_first, path);

		//std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > lines3d_second;
		//for (int i = 0; i < scenes_line3d_track_list_.size(); ++i)
		//{
		//	const HWLineTrack3D& const track_line = scenes_line3d_track_list_[i];
		//	//std::vector<std::pair<int, float> > track2inter_vs;
		//	std::pair<Eigen::Vector3f, Eigen::Vector3f> tl;
		//	tl.first = track_line.s_pos_;
		//	tl.second = track_line.e_pos_;
		//	Eigen::Vector3f t_m = (tl.first + tl.second) / 2.0;
		//	int inter_idx1 = scenes_line3d_track_list_[i].polygon_intersect_line_idx_[1];
		//	if (inter_idx1 != -1)
		//	{
		//		std::pair<Eigen::Vector3f, Eigen::Vector3f> il;
		//		il.first = polygons_intersection_lines_[inter_idx1].ls3d_;
		//		il.second = polygons_intersection_lines_[inter_idx1].le3d_;
		//		Eigen::Vector3f i_m = (il.first + il.second) / 2.0;
		//		std::pair<Eigen::Vector3f, Eigen::Vector3f> tim = std::make_pair(t_m, i_m);
		//		lines3d_second.emplace_back(tl);
		//		lines3d_second.emplace_back(il);
		//		lines3d_second.emplace_back(tim);
		//	}
		//}
		//std::string path_second = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/lines_match_images_1/trackline3d2InterLinesecond.obj";
		//WriteLine3DIntoObj(lines3d_second, path_second);
	}

	bool HWScenesElements::GetLayoutPickedLineFromLayidLineId(int layid, int lid, Eigen::Vector3f& ls, Eigen::Vector3f& le)
	{
		const HWSceneLayout2D tmplyout = scenes_layouts_elements_->GetLayout2DFromId(layid);
		if (tmplyout.GetPickedLineFromIdx(lid, ls, le))
		{
			return true;
		}
		return false;
	}

	//draw single line to debug
	void HWScenesElements::DrawLayoutPickedLineIntoImage(const std::string& path, int layid, int lid)
	{
		const HWSceneLayout2D tmplyout = scenes_layouts_elements_->GetLayout2DFromId(layid);
		//get image
		const int imageid = tmplyout.GetImageId();
		const HWImage tmp_img = scenes_cams_->GetImageFromImgId(imageid);
		const std::string img_path = tmp_img.GetImagePath();
		cv::Mat img_cv = cv::imread(img_path);
		//get layout line
		Eigen::Vector3f ls, le;
		if (tmplyout.GetPickedLineFromIdx(lid, ls, le))
		{
			cv::Point2f lscv, lecv;
			lscv.x = ls[0];
			lscv.y = ls[1];
			lecv.x = le[0];
			lecv.y = le[1];
			cv::line(img_cv, lscv, lecv, cv::Scalar(0, 255, 100));
		}
		cv::imwrite(path, img_cv);
	}

	bool HWScenesElements::CheckLineSeg3d2LineSeg3dInSameLine(const std::pair<Eigen::Vector3f, Eigen::Vector3f>& lseg1,
		const std::pair<Eigen::Vector3f, Eigen::Vector3f>& lseg2, float angle_threshold, float dist_threshold)
	{
		//std::cerr << "check the two line segments if they are the same. to do next..." << std::endl;
		float line_angle = ComputeAngleBetweenTwoLineSegs(lseg1.first, lseg1.second, lseg2.first, lseg2.second);
		if (line_angle > angle_threshold)
		{
			return false;
		}
		//
		float l1s2l2 = PntDist2Line3D(lseg1.first, lseg2.first, lseg2.second);
		float l1e2l2 = PntDist2Line3D(lseg1.second, lseg2.first, lseg2.second);

		float l1distl2 = (l1s2l2 + l1e2l2) / 2.0;
		if (l1distl2 > dist_threshold)
		{
			return false;
		}

		/*std::vector<float> dist2line;
		float l1stl2_dist = Point2LineSegMinDistance(lseg1.first, lseg2.first, lseg2.second);
		float l1etl2_dist = Point2LineSegMinDistance(lseg1.second, lseg2.first, lseg2.second);
		float l2stl1_dist = Point2LineSegMinDistance(lseg2.first, lseg1.first, lseg1.second);
		float l2etl1_dist = Point2LineSegMinDistance(lseg2.second, lseg1.first, lseg1.second);
		dist2line.emplace_back(l1stl2_dist);
		dist2line.emplace_back(l1etl2_dist);
		dist2line.emplace_back(l2stl1_dist);
		dist2line.emplace_back(l2etl1_dist);
		std::sort(dist2line.begin(), dist2line.end());
		if()*/
		return true;
	}

	bool HWScenesElements::CheckLineSeg2d2LineSeg2dWithThreshold(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& lseg1,
		const std::pair<Eigen::Vector2f, Eigen::Vector2f>& lseg2, float angle_threshold, float dist_threshold)
	{
		Eigen::Vector2f src_dir = lseg1.second - lseg1.first;
		src_dir.normalize();
		Eigen::Vector2f tgt_dir = lseg2.second - lseg2.first;
		tgt_dir.normalize();
		float line_angle = ComputeAngleFromTwoLinesVector2D(src_dir, tgt_dir);
		if (line_angle > angle_threshold)
		{
			return false;
		}
		float s2t_sl_dist, s2t_el_dist;
		s2t_sl_dist = PntDist2Line2D(lseg1.first, lseg2.first, lseg2.second);
		s2t_el_dist = PntDist2Line2D(lseg1.second, lseg2.first, lseg2.second);
		float l2l_dist = (s2t_sl_dist + s2t_el_dist) / 2.0;
		if (l2l_dist > dist_threshold)
		{
			return false;
		}
		return true;
	}

	bool HWScenesElements::CheckTwoPnt3dTheshold(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float dist_threshold)
	{
		float p1p2dist = (p1 - p2).norm();
		if (p1p2dist < dist_threshold)
		{
			return true;
		}
		return false;
	}

	bool HWScenesElements::CheckTwoPnt2dTheshold(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, float dist_threshold)
	{
		float p1p2dist = (p1 - p2).norm();
		if (p1p2dist < dist_threshold)
		{
			return true;
		}
		return false;
	}

	void HWScenesElements::InitialConvertCamsAndLayoutsIntoViews()
	{
		//
		hw_cams_views_list_.clear();
		const std::vector<CameraModel> cams = scenes_cams_->GetCamerasModels();
		//std::string images_dir = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/line_scene_test/";
		for (int i = 0; i < cams.size(); ++i)
		{
			CameraModel tmp_cam = cams[i];
			HWCamViewport tmp_view;
			tmp_view.hw_camera_id_ = tmp_cam.cam_id_;
			tmp_view.hw_image_id_ = tmp_cam.image_id_;
			tmp_view.hw_layout_id_ = tmp_cam.layout_id_;
			tmp_view.view_pose_ = tmp_cam;
			tmp_view.focal_length_x_ = tmp_cam.fx_;
			tmp_view.focal_length_y_ = tmp_cam.fy_;
			int cam_id = tmp_cam.cam_id_;
			int image_id = tmp_cam.image_id_;
			//std::cerr << "cam_id, image_id: " << cam_id << ", " << image_id << std::endl;
			//get the feature pose
			HWImage tmp_image = scenes_cams_->GetImageFromImgId(image_id);
			//important to get tmp_image feature position
			const std::vector<cv::KeyPoint> tmp_features_position = tmp_image.GetSiftFeatruesPosition();
			for (int j = 0; j < tmp_features_position.size(); ++j)
			{
				Eigen::Vector2f tmp_position = Eigen::Vector2f(tmp_features_position[j].pt.x, tmp_features_position[j].pt.y);
				tmp_view.features_poistion_.emplace_back(tmp_position);
			}
			int features_num = static_cast<int>(tmp_features_position.size());
			tmp_view.track_ids_.resize(features_num, -1);
			int tmp_layout_id = tmp_image.GetLayoutId();
			//std::cerr << "tm_layout_id: " << tmp_layout_id << std::endl;
			HWSceneLayout2D tmp_layout2d = scenes_layouts_elements_->GetLayout2DFromId(tmp_layout_id);
			//std::cerr << "tmp_layout2d num: " << tmp_layout2d.GetLayoutLinesNum() << std::endl;
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > scene_layout_pnts;
			tmp_layout2d.GetAllPairLinesPnts(scene_layout_pnts);
			//UpdateNearNeighborLinesSegments();GetNeighborLinesIdxs();//add two function
			tmp_layout2d.UpdateNearNeighborLinesSegments();
			const std::vector<int> tmp_lines_near_idxs = tmp_layout2d.GetNeighborLinesIdxs();

#if 0
			//filter the neighbor lines and show remained lines
			/*
			to test it
			*/
			std::string lay_image_path = images_dir + std::to_string(tmp_view.hw_image_id_) + "_ly_filter.png";
			//const cv::Mat& tmp_cv_img = tmp_image.GetImage();
			std::cerr << "lay_image_path: " << lay_image_path << std::endl;
			const std::string tmp_cv_image_path = tmp_image.GetImagePath();
			cv::Mat& tmp_cv_img = cv::imread(tmp_cv_image_path);
			for (int j = 0; j < scene_layout_pnts.size(); ++j)
			{
				if (tmp_lines_near_idxs[j] == -1)
				{
					std::pair<Eigen::Vector2f, Eigen::Vector2f> lcv = scene_layout_pnts[j];
					cv::Point lscv(lcv.first[0], lcv.first[1]);
					cv::Point lecv(lcv.second[0], lcv.second[1]);
					cv::line(tmp_cv_img, lscv, lecv, cv::Scalar(0, 255, 0), 2);
				}
			}
			cv::imwrite(lay_image_path, tmp_cv_img);
			//end test
#endif

			//std::cerr << "adfadfsf" << std::endl;
			for (int j = 0; j < scene_layout_pnts.size(); ++j)
			{
				tmp_view.lines_segments_.emplace_back(scene_layout_pnts[j]);
			}
			int lines_num = static_cast<int>(scene_layout_pnts.size());
			//std::cerr << "lines_num: " << lines_num << std::endl;
			tmp_view.lines_track_ids_.resize(lines_num, -1);	//line 2d idx to track line 3d idx
			hw_cams_views_list_.emplace_back(tmp_view);
		}
		views_initial_updated_flag_ = true;
	}

	void HWScenesElements::UpdateGroupViewsLines2dIntoLines2dBasedOnLinesDir()
	{
		scenes_layouts_elements_->UpdateAllLayoutsLineCombinationProcessBasedOnLinesDir();
	}

	void HWScenesElements::InitialConvertCamsAndLayoutsAndPntsIntoViewsNetWork()
	{
		//
		hw_cams_views_list_.clear();
		const std::vector<CameraModel> cams = scenes_cams_->GetCamerasModels();
		//std::string images_dir = "D:/vc_project_new/huawei_data_indoor/thesis_test/single_room/complete_obj/line_scene_test/";
		for (int i = 0; i < cams.size(); ++i)
		{
			CameraModel tmp_cam = cams[i];
			HWCamViewport tmp_view;
			tmp_view.hw_camera_id_ = tmp_cam.cam_id_;
			tmp_view.hw_image_id_ = tmp_cam.image_id_;
			tmp_view.hw_layout_id_ = tmp_cam.layout_id_;
			tmp_view.view_pose_ = tmp_cam;
			tmp_view.focal_length_x_ = tmp_cam.fx_;
			tmp_view.focal_length_y_ = tmp_cam.fy_;
			int cam_id = tmp_cam.cam_id_;
			int image_id = tmp_cam.image_id_;
			//std::cerr << "cam_id, image_id: " << cam_id << ", " << image_id << std::endl;
			//get the feature pose
			HWImage tmp_image = scenes_cams_->GetImageFromImgId(image_id);
			//important to get tmp_image feature position
			const std::vector<cv::Point2f> tmp_network_pnts_position = tmp_image.GetImageNetWorkPnts();// GetSiftFeatruesPosition();
			for (int j = 0; j < tmp_network_pnts_position.size(); ++j)
			{
				Eigen::Vector2f tmp_position = Eigen::Vector2f(tmp_network_pnts_position[j].x, tmp_network_pnts_position[j].y);
				tmp_view.features_poistion_.emplace_back(tmp_position);
			}
			int features_num = static_cast<int>(tmp_network_pnts_position.size());
			std::cerr << "features_num: " << features_num << std::endl;
			tmp_view.track_ids_.resize(features_num, -1);
			int tmp_layout_id = tmp_image.GetLayoutId();
			//std::cerr << "tm_layout_id: " << tmp_layout_id << std::endl;
			HWSceneLayout2D tmp_layout2d = scenes_layouts_elements_->GetLayout2DFromId(tmp_layout_id);
			//std::cerr << "tmp_layout2d num: " << tmp_layout2d.GetLayoutLinesNum() << std::endl;
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > scene_layout_pnts;
			tmp_layout2d.GetAllPairLinesPnts(scene_layout_pnts);
			//UpdateNearNeighborLinesSegments();GetNeighborLinesIdxs();//add two function
			tmp_layout2d.UpdateNearNeighborLinesSegments();
			const std::vector<int> tmp_lines_near_idxs = tmp_layout2d.GetNeighborLinesIdxs();

#if 0
			//filter the neighbor lines and show remained lines
			/*
			to test it
			*/
			std::string lay_image_path = images_dir + std::to_string(tmp_view.hw_image_id_) + "_ly_filter.png";
			//const cv::Mat& tmp_cv_img = tmp_image.GetImage();
			std::cerr << "lay_image_path: " << lay_image_path << std::endl;
			const std::string tmp_cv_image_path = tmp_image.GetImagePath();
			cv::Mat& tmp_cv_img = cv::imread(tmp_cv_image_path);
			for (int j = 0; j < scene_layout_pnts.size(); ++j)
			{
				if (tmp_lines_near_idxs[j] == -1)
				{
					std::pair<Eigen::Vector2f, Eigen::Vector2f> lcv = scene_layout_pnts[j];
					cv::Point lscv(lcv.first[0], lcv.first[1]);
					cv::Point lecv(lcv.second[0], lcv.second[1]);
					cv::line(tmp_cv_img, lscv, lecv, cv::Scalar(0, 255, 0), 2);
				}
			}
			cv::imwrite(lay_image_path, tmp_cv_img);
			//end test
#endif

			//get grouped lines 2d
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > scene_layout_grouped_lines
				= tmp_layout2d.GetAllGroupedLines2dPnts();
			std::vector<std::pair<int, int> > scene_layout_linesids_to_grouped_ids =
				tmp_layout2d.GetAllLines2dIdxsToAllGroupedLines2dIdxs();

			//std::cerr << "adfadfsf" << std::endl;
			for (int j = 0; j < scene_layout_pnts.size(); ++j)
			{
				tmp_view.lines_segments_.emplace_back(scene_layout_pnts[j]);
			}
			tmp_view.grouped_lines_segments_ = scene_layout_grouped_lines;
			tmp_view.lines2d_idxs_to_grouped_lines2d_idxs_ = scene_layout_linesids_to_grouped_ids;

			int lines_num = static_cast<int>(scene_layout_pnts.size());
			//std::cerr << "lines_num: " << lines_num << std::endl;
			tmp_view.lines_track_ids_.resize(lines_num, -1);	//line 2d idx to track line 3d idx
			hw_cams_views_list_.emplace_back(tmp_view);
		}
		views_initial_updated_flag_ = true;
	}

	void HWScenesElements::ReloadImagesLinesPairsFromImportedData()
	{
		if(import_data_from_pairs_lines_txt_state_)
		{
			//
			matched_.clear();
			matched_fundamentals_.clear();
			matches_.clear();
			num_matches_.clear();
			UpdateImportDataToMatched();
		}
	}

	void HWScenesElements::ReloadImagesPntsParisAndLinesPairsFromImportedData()
	{
		if (import_data_from_python_numpy_state_)
		{
			//
			matched_.clear();
			matched_fundamentals_.clear();
			matches_.clear();
			num_matches_.clear();
			UpdateImportedNetLinesToMatched();
			UpdateImportedNetPntsToHWImages();
		}
	}

	void HWScenesElements::UpdateImportDataToMatched()
	{
		//get cams string
		const std::vector<std::string> cams_paths = scenes_cams_.get()->GetCamerasPaths();
		//num_matches_.resize(cams_paths.size());
		for (int i = 0; i < cams_paths.size(); ++i)
		{
			num_matches_[i] = 0;
		}
		for (int i = 0; i < cams_paths.size(); ++i)
		{
			int src_id = i;
			int src_lid = scenes_cams_.get()->GetLayoutIdFromCamId(src_id);
			HWSceneLayout2D srcly = scenes_layouts_elements_->GetLayout2DFromId(src_lid);
			const std::vector<Eigen::Vector2i> srclines = srcly.GetLinesIdxs();
			matches_[i].resize(srclines.size());
		}
		//update the matched_
		std::vector<std::string> cams_base_names = GetAllBaseNamesFromPaths(cams_paths);
		for (int i = 0; i < images_lines_pairs_imported_.size(); ++i)
		{
			std::string cam1_path = images_lines_pairs_imported_[i].base_path_pair.first;
			std::string cam2_path = images_lines_pairs_imported_[i].base_path_pair.second;
			std::string cam1_base_name = GetPathPrefix(cam1_path);
			std::string cam2_base_name = GetPathPrefix(cam2_path);
			/*std::cerr << "cam1_base_name, cam2_base_name: " << cam1_base_name << 
				", " << cam2_base_name << std::endl;*/
			int cam1_idx = FindStrIdxFromVecStrsNew(cam1_base_name, cams_base_names);
			int cam2_idx = FindStrIdxFromVecStrsNew(cam2_base_name, cams_base_names);
			if (cam1_idx != -1 && cam2_idx != -1)
			{
				//get layout from cam idx
				//std::cerr << "cam1_idx, cam2_idx: " << cam1_idx << ", " << cam2_idx << std::endl;
				int lay1_id = scenes_cams_.get()->GetLayoutIdFromCamId(cam1_idx);
				int lay2_id = scenes_cams_.get()->GetLayoutIdFromCamId(cam2_idx);
				matched_[cam1_idx].insert(cam2_idx);
				//std::cerr << "lay1_id, lay2_id: " << lay1_id << ", " << lay2_id << std::endl;

				//set lines pairs to matches_
				std::pair<Eigen::Vector2f, Eigen::Vector2f> line1 = images_lines_pairs_imported_[i].line1;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> line2 = images_lines_pairs_imported_[i].line2;

				CameraModel src_model = scenes_cams_->GetCameraModelFromCamId(cam1_idx);
				CameraModel tgt_model = scenes_cams_->GetCameraModelFromCamId(cam2_idx);
				Eigen::Matrix3f F = GetFundamentalMatrix(src_model, tgt_model);
				matched_fundamentals_[cam1_idx][cam2_idx] = F;

				int lay1_line_idx = scenes_layouts_elements_->FindLayoutLineIdxFromLinePos(lay1_id, line1);
				int lay2_line_idx = scenes_layouts_elements_->FindLayoutLineIdxFromLinePos(lay2_id, line2);
				//std::cerr << "lay1_line_idx, lay2_line_idx: " << lay1_line_idx << ", " << lay2_line_idx << std::endl;

				if (lay1_line_idx >= 0 && lay2_line_idx >= 0)
				{
					HWMatch M;
					M.src_camID_ = cam1_idx;
					M.src_segID_ = lay1_line_idx;
					M.tgt_camID_ = cam2_idx;
					M.tgt_segID_ = lay2_line_idx;
					M.valid_match_ = true;
					//std::cerr << "1111111 " << std::endl;
					matches_[cam1_idx][lay1_line_idx].emplace_back(M);
					num_matches_[cam1_idx] += 1;
					//std::cerr << "22222222" << std::endl;
				}

				//matches_[cam2_idx][lay2_line_idx].emplace_back(M);
			}
		}
	}

	void HWScenesElements::UpdateImportedNetLinesToMatched()
	{
		//lines pair to the hw scenes elements
		std::vector<CameraModel> camsmodels = scenes_cams_->GetCamerasModels();
		//num_matches_.resize(cams_paths.size());
		std::vector<std::string> cams_paths;
		cams_paths.resize(camsmodels.size());
		for (int i = 0; i < camsmodels.size(); ++i)
		{
			num_matches_[i] = 0;
			cams_paths[i] = camsmodels[i].path_;
		}
		for (int i = 0; i < camsmodels.size(); ++i)
		{
			int src_lid = camsmodels[i].layout_id_;
			HWSceneLayout2D srcly = scenes_layouts_elements_->GetLayout2DFromId(src_lid);
			const std::vector<Eigen::Vector2i> srclines = srcly.GetLinesIdxs();
			matches_[i].resize(srclines.size());
		}
		//update the matched_
		std::vector<std::string> cams_base_names = GetAllBaseNamesFromPaths(cams_paths);
		for (int i = 0; i < images_lines_pairs_imported_.size(); ++i)
		{
			std::string cam1_path = images_lines_pairs_imported_[i].base_path_pair.first;
			std::string cam2_path = images_lines_pairs_imported_[i].base_path_pair.second;
			std::string cam1_base_name = GetPathPrefix(cam1_path);
			std::string cam2_base_name = GetPathPrefix(cam2_path);
			/*std::cerr << "cam1_base_name, cam2_base_name: " << cam1_base_name <<
			", " << cam2_base_name << std::endl;*/
			int cam1_idx = FindStrIdxFromVecStrsNew(cam1_base_name, cams_base_names);
			int cam2_idx = FindStrIdxFromVecStrsNew(cam2_base_name, cams_base_names);
			if (cam1_idx != -1 && cam2_idx != -1)
			{
				//get layout from cam idx
				//std::cerr << "cam1_idx, cam2_idx: " << cam1_idx << ", " << cam2_idx << std::endl;
				int lay1_id = scenes_cams_.get()->GetLayoutIdFromCamId(cam1_idx);
				int lay2_id = scenes_cams_.get()->GetLayoutIdFromCamId(cam2_idx);
				matched_[cam1_idx].insert(cam2_idx);
				//std::cerr << "lay1_id, lay2_id: " << lay1_id << ", " << lay2_id << std::endl;

				//set lines pairs to matches_
				std::pair<Eigen::Vector2f, Eigen::Vector2f> line1 = images_lines_pairs_imported_[i].line1;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> line2 = images_lines_pairs_imported_[i].line2;

				CameraModel src_model = scenes_cams_->GetCameraModelFromCamId(cam1_idx);
				CameraModel tgt_model = scenes_cams_->GetCameraModelFromCamId(cam2_idx);
				Eigen::Matrix3f F = GetFundamentalMatrix(src_model, tgt_model);
				matched_fundamentals_[cam1_idx][cam2_idx] = F;

				int lay1_line_idx = scenes_layouts_elements_->FindLayoutLineIdxFromLinePos(lay1_id, line1);
				int lay2_line_idx = scenes_layouts_elements_->FindLayoutLineIdxFromLinePos(lay2_id, line2);
				//std::cerr << "lay1_line_idx, lay2_line_idx: " << lay1_line_idx << ", " << lay2_line_idx << std::endl;

				if (lay1_line_idx >= 0 && lay2_line_idx >= 0)
				{
					HWMatch M;
					M.src_camID_ = cam1_idx;
					M.src_segID_ = lay1_line_idx;
					M.tgt_camID_ = cam2_idx;
					M.tgt_segID_ = lay2_line_idx;
					M.valid_match_ = true;
					//std::cerr << "1111111 " << std::endl;
					matches_[cam1_idx][lay1_line_idx].emplace_back(M);
					num_matches_[cam1_idx] += 1;
					//std::cerr << "22222222" << std::endl;
				}
				//matches_[cam2_idx][lay2_line_idx].emplace_back(M);
			}
		}
	}

	void HWScenesElements::UpdateImportedNetPntsToHWImages()
	{
		//pnts pair to the hw scenes elements
		std::cerr << "start to import net points into HWImages..." << std::endl;
		//std::vector<CameraModel> camsmodels = scenes_cams_->GetCamerasModels();
		////num_matches_.resize(cams_paths.size());
		//std::vector<std::string> cams_paths;
		//cams_paths.resize(camsmodels.size());
		//for (int i = 0; i < camsmodels.size(); ++i)
		//{
		//	num_matches_[i] = 0;
		//	cams_paths[i] = camsmodels[i].path_;
		//}
		////point matches
		std::cerr << "do it in the next..." << std::endl;
		std::cerr << "end import net points into HWImages..." << std::endl;
	}

	void HWScenesElements::UpdateImportedNetPntsToHWImagesIds()
	{
		//lines pair to the hw scenes elements
		std::vector<CameraModel> camsmodels = scenes_cams_->GetCamerasModels();
		//num_matches_.resize(cams_paths.size());
		std::vector<std::string> cams_paths;
		cams_paths.resize(camsmodels.size());
		for (int i = 0; i < camsmodels.size(); ++i)
		{
			cams_paths[i] = camsmodels[i].path_;
		}
		std::vector<std::string> cams_base_names = GetAllBaseNamesFromPaths(cams_paths);
		//images_pnts_pairs_imported_
		for (int i = 0; i < images_pnts_pairs_imported_.size(); ++i)
		{
			std::string cam1_path = images_pnts_pairs_imported_[i].base_path_pair.first;
			std::string cam2_path = images_pnts_pairs_imported_[i].base_path_pair.second;
			std::string cam1_base_name = GetPathPrefix(cam1_path);
			std::string cam2_base_name = GetPathPrefix(cam2_path);
			/*std::cerr << "cam1_base_name, cam2_base_name: " << cam1_base_name <<
			", " << cam2_base_name << std::endl;*/
			int cam1_idx = FindStrIdxFromVecStrsNew(cam1_base_name, cams_base_names);
			int cam2_idx = FindStrIdxFromVecStrsNew(cam2_base_name, cams_base_names);
			if (cam1_idx != -1 && cam2_idx != -1)
			{
				images_pnts_pairs_imported_[i].camid_to_camid_.first = cam1_idx;
				images_pnts_pairs_imported_[i].camid_to_camid_.second = cam2_idx;
			}
		}
	}

	void HWScenesElements::UpdateImportedNetLinesToHWImageIds()
	{
		//lines pair to the hw scenes elements
		std::vector<CameraModel> camsmodels = scenes_cams_->GetCamerasModels();
		//num_matches_.resize(cams_paths.size());
		std::vector<std::string> cams_paths;
		cams_paths.resize(camsmodels.size());
		for (int i = 0; i < camsmodels.size(); ++i)
		{
			cams_paths[i] = camsmodels[i].path_;
		}
		std::vector<std::string> cams_base_names = GetAllBaseNamesFromPaths(cams_paths);
		//images_lines_pairs_imported_
		for (int i = 0; i < images_lines_pairs_imported_.size(); ++i)
		{
			std::string cam1_path = images_lines_pairs_imported_[i].base_path_pair.first;
			std::string cam2_path = images_lines_pairs_imported_[i].base_path_pair.second;
			std::string cam1_base_name = GetPathPrefix(cam1_path);
			std::string cam2_base_name = GetPathPrefix(cam2_path);
			/*std::cerr << "cam1_base_name, cam2_base_name: " << cam1_base_name <<
			", " << cam2_base_name << std::endl;*/
			int cam1_idx = FindStrIdxFromVecStrsNew(cam1_base_name, cams_base_names);
			int cam2_idx = FindStrIdxFromVecStrsNew(cam2_base_name, cams_base_names);
			if (cam1_idx != -1 && cam2_idx != -1)
			{
				images_lines_pairs_imported_[i].camid_to_camid_.first = cam1_idx;
				images_lines_pairs_imported_[i].camid_to_camid_.second = cam2_idx;
			}
		}
	}

	void HWScenesElements::FilterMatchedImagesPairByLinesMatchNum(int line_pair_num)
	{
		//std::vector<bool> matched_flag;
		std::map<unsigned int, std::set<unsigned int> >::iterator iiter = matched_.begin();
		for (; iiter != matched_.end(); ++iiter)
		{
			int src_camid = iiter->first;
			std::set<unsigned int>::iterator jiter = iiter->second.begin();
			while (jiter != iiter->second.end())
			{
				int tgt_camid = *jiter;
				//get line matched num from image pair
				std::vector<HWCorrespondence2D2DLines> lines_pairs;
				GetImagesLinesPairFromImagesPair(src_camid, tgt_camid, lines_pairs);
				int lines_pairs_num = static_cast<int>(lines_pairs.size());
				if (lines_pairs_num < line_pair_num)
				{
					iiter->second.erase(jiter++);
				}
				else
				{
					jiter++;
				}
			}
		}
	}

	std::vector<std::string> HWScenesElements::GetAllBaseNamesFromPaths(const std::vector<std::string>& paths)
	{
		std::vector<std::string> base_names;
		for (int i = 0; i < paths.size(); ++i)
		{
			std::string base_name_siffux = GetBaseName(paths[i]);
			std::string base_name = GetPathPrefix(base_name_siffux);
			base_names.emplace_back(base_name);
		}
		return base_names;
	}

	std::vector<std::string> HWScenesElements::GetAllBaseNameDropLastPattern(const std::vector<std::string>& paths)
	{
		std::vector<std::string> base_names_dropped;
		for (int i = 0; i < paths.size(); ++i)
		{
			std::string base_name_drop = paths[i].substr(0, paths[i].find_last_of("_"));
			//std::cerr << "base_name_drop: " << base_name_drop << std::endl;
			//std::string base_name = GetPathPrefix(base_name_siffux);
			base_names_dropped.emplace_back(base_name_drop);
		}
		return base_names_dropped;
	}

    void HWScenesElements::WritePickeLayoutIntoPickedImage(int image_idx, int lyidx, const std::string& path)
    {
		HWImage scene_img = scenes_cams_->GetImageFromImgId(image_idx);
		HWSceneLayout2D scene_ly = scenes_layouts_elements_->GetLayout2DFromId(lyidx);
		//std::cerr << "111111" << std::endl;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > lines_pnts;
		scene_ly.GetAllPairLinesPnts(lines_pnts);
		//std::cerr << "121212121212" << std::endl;
		if (!scene_img.ImageLoaded())
		{
			//std::cerr << "123123123123" << std::endl;
			scene_img.LoadHWImage();
		}
		//std::cerr << "222222" << std::endl;
		const cv::Mat src_img = scene_img.GetImage();
		cv::Mat img = src_img.clone();
		//std::cerr << "333333" << std::endl;
		for (int i = 0; i < lines_pnts.size(); ++i)
		{
			//std::cerr << "i " << i << std::endl;
			Eigen::Vector2f ls = lines_pnts[i].first;
			Eigen::Vector2f le = lines_pnts[i].second;
			cv::Point2f lscv(ls[0], ls[1]);
			cv::Point2f	lecv(le[0], le[1]);
			cv::circle(img, lscv, 4, cv::Scalar(255, 0, 0), 2);
			cv::circle(img, lecv, 4, cv::Scalar(255, 0, 0), 2);
			cv::line(img, lscv, lecv, cv::Scalar(0, 0, 255), 2);
		}
		cv::imwrite(path, img);
    }

	void HWScenesElements::WriteNetworkPntsIntoPickedImage(int imageid, const std::string& path)
	{
		scenes_cams_->WriteImageNetworkPntsIntoImageByImgid(imageid, path);
	}

	void HWScenesElements::GenerateImageIdLyidFromCamsid(int camid, int& imgid, int& lyid)
	{
		const CameraModel tmpcmodel = scenes_cams_->GetCameraModelFromCamId(camid);
		imgid = tmpcmodel.image_id_;
		lyid = tmpcmodel.layout_id_;
	}

	const std::string& HWScenesElements::GetImagePathFromImageid(int imageid)
	{
		const HWImage tmp_img = scenes_cams_->GetImageFromImgId(imageid);
		const std::string image_path = tmp_img.GetImagePath();
		return image_path;
	}

	cv::Mat HWScenesElements::MixImageFromTwoImages(const cv::Mat& img0, const cv::Mat& img1, float fweight)
	{
		cv::Mat mixed_img;
		cv::addWeighted(img0, fweight, img1, 1 - fweight, 0.0, mixed_img);
		return mixed_img;
	}

	std::vector<HWMatch> HWScenesElements::GetHWMatchFromMatches(unsigned int src_cid, unsigned int tgt_cid)
	{
		std::vector<HWMatch> views_matches;
		std::vector<std::list<HWMatch> > src_view_matches = matches_[src_cid];
		for (int i = 0; i < src_view_matches.size(); ++i)
		{
			std::list<HWMatch>::iterator it = src_view_matches[i].begin();
			for (; it != src_view_matches[i].end(); ++it)
			{
				if (it->tgt_camID_ == tgt_cid)
				{
					if(it->valid_match_)
						views_matches.emplace_back(*it);
				}
			}
		}
		return views_matches;
	}

	std::pair<Eigen::Vector2f, Eigen::Vector2f> HWScenesElements::GetLayoutLineFromCamIdAndLid(unsigned int src_cid, unsigned int lid)
	{
		int imgid = scenes_cams_->GetCameraModelFromCamId(src_cid).image_id_;
		int layoutid = scenes_cams_->GetImageFromImgId(imgid).GetLayoutId();
		std::pair<Eigen::Vector3f, Eigen::Vector3f> lyline_pnt_3d;
		GetLayoutPickedLineFromLayidLineId(layoutid, lid, lyline_pnt_3d.first, lyline_pnt_3d.second);
		std::pair<Eigen::Vector2f, Eigen::Vector2f> lyline_pnt;
		lyline_pnt.first = Eigen::Vector2f(lyline_pnt_3d.first[0], lyline_pnt_3d.first[1]);
		lyline_pnt.second = Eigen::Vector2f(lyline_pnt_3d.second[0], lyline_pnt_3d.second[1]);
		return lyline_pnt;
	}

	/*
	* Merges tracks and updates viewports accordingly.
	*/
	void HWScenesElements::unify_new_lines_tracks(int view1_line_tid, int view2_line_tid)
	{
		/* Unify in larger track. */
		if (scenes_line3d_track_list_[view1_line_tid].lines_features_.size()
			< scenes_line3d_track_list_[view2_line_tid].lines_features_.size())
			std::swap(view1_line_tid, view2_line_tid);

		HWLineTrack3D& track1 = scenes_line3d_track_list_[view1_line_tid];
		HWLineTrack3D& track2 = scenes_line3d_track_list_[view2_line_tid];

		for (std::size_t k = 0; k < track2.lines_features_.size(); ++k)
		{
			int const view_id = track2.lines_features_[k].view_id_;
			int const line_id = track2.lines_features_[k].line_id_;
			hw_cams_views_list_[view_id].lines_track_ids_[line_id] = view1_line_tid;
		}
		track1.lines_features_.insert(track1.lines_features_.end(),
			track2.lines_features_.begin(), track2.lines_features_.end());
		track1.line_views_pnts_.insert(track1.line_views_pnts_.end(),
			track2.line_views_pnts_.begin(), track2.line_views_pnts_.end());
		
		/* Free old track's memory. clear() does not work. */
		track2.lines_features_ = HWLineFeatureReferenceIdList();
	}

	void HWScenesElements::unify_new_pnts_tracks(int view1_pnt_tid, int view2_pnt_tid)
	{
		/* Unify in larger track. */
		if (scenes_pnts3d_track_list_[view1_pnt_tid].features_.size()
			< scenes_pnts3d_track_list_[view2_pnt_tid].features_.size())
			std::swap(view1_pnt_tid, view2_pnt_tid);

		HWTrack3D& track1 = scenes_pnts3d_track_list_[view1_pnt_tid];
		HWTrack3D& track2 = scenes_pnts3d_track_list_[view2_pnt_tid];

		for (std::size_t k = 0; k < track2.features_.size(); ++k)
		{
			int const view_id = track2.features_[k].view_id_;
			int const line_id = track2.features_[k].feature_id_;
			hw_cams_views_list_[view_id].track_ids_[line_id] = view1_pnt_tid;
		}
		track1.features_.insert(track1.features_.end(),
			track2.features_.begin(), track2.features_.end());
		/* Free old track's memory. clear() does not work. */
		track2.features_ = HWFeatureReferenceIdList();
	}

	void HWScenesElements::unify_lines3d_into_grouped_lines3d_tracks(int view1_line3d_tid, int view2_line3d_tid,
		HWLineTrack3D& grouped_line3d_track)
	{
		/* Unify in larger track. */
		HWLineTrack3D& track1 = scenes_line3d_track_list_[view1_line3d_tid];
		HWLineTrack3D& track2 = scenes_line3d_track_list_[view2_line3d_tid];

		for (std::size_t k = 0; k < track2.lines_features_.size(); ++k)
		{
			int const view_id = track2.lines_features_[k].view_id_;
			int const line_id = track2.lines_features_[k].line_id_;
			//hw_cams_views_list_[view_id].lines_track_ids_[line_id] = view1_line_tid;
		}
		grouped_line3d_track.is_valid_ = track1.is_valid_;
		grouped_line3d_track.polygon_idxs_ = track1.polygon_idxs_;
		grouped_line3d_track.lines_features_ = track1.lines_features_;
		grouped_line3d_track.line_views_pnts_ = track1.line_views_pnts_;
		grouped_line3d_track.s_pos_ = track1.s_pos_;
		grouped_line3d_track.e_pos_ = track1.e_pos_;

		grouped_line3d_track.lines_features_.insert(grouped_line3d_track.lines_features_.end(),
			track2.lines_features_.begin(), track2.lines_features_.end());
		grouped_line3d_track.line_views_pnts_.insert(grouped_line3d_track.line_views_pnts_.end(),
			track2.line_views_pnts_.begin(), track2.line_views_pnts_.end());
	}

	void HWScenesElements::unify_new_lines_tracks_labels(int view1_line_tid, int view2_line_tid)
	{
		/* Unify in larger track. */
		if (scenes_line3d_track_list_[view1_line_tid].lines_features_.size()
			< scenes_line3d_track_list_[view2_line_tid].lines_features_.size())
			std::swap(view1_line_tid, view2_line_tid);

		HWLineTrack3D& track1 = scenes_line3d_track_list_[view1_line_tid];
		HWLineTrack3D& track2 = scenes_line3d_track_list_[view2_line_tid];

		for (std::size_t k = 0; k < track2.lines_features_.size(); ++k)
		{
			int const view_id = track2.lines_features_[k].view_id_;
			int const line_id = track2.lines_features_[k].line_id_;
			hw_cams_views_list_[view_id].lines_track_ids_[line_id] = view1_line_tid;
		}
		track1.lines_features_.insert(track1.lines_features_.end(),
			track2.lines_features_.begin(), track2.lines_features_.end());
		track1.polygon_idxs_.insert(track1.polygon_idxs_.end(),
			track2.polygon_idxs_.begin(), track2.polygon_idxs_.end());	//insert track1's polygon idx into track2
		track1.line_views_pnts_.insert(track1.line_views_pnts_.end(),
			track2.line_views_pnts_.begin(), track2.line_views_pnts_.end());

		/* Free old track's memory. clear() does not work. */
		track2.lines_features_ = HWLineFeatureReferenceIdList();
	}

	void HWScenesElements::unify_new_pnts_tracks_labels(int view1_pnt_tid, int view2_pnt_tid)
	{
		/* Unify in larger track. */
		if (scenes_pnts3d_track_list_[view1_pnt_tid].features_.size()
			< scenes_pnts3d_track_list_[view2_pnt_tid].features_.size())
			std::swap(view1_pnt_tid, view2_pnt_tid);

		HWTrack3D& track1 = scenes_pnts3d_track_list_[view1_pnt_tid];
		HWTrack3D& track2 = scenes_pnts3d_track_list_[view2_pnt_tid];

		for (std::size_t k = 0; k < track2.features_.size(); ++k)
		{
			int const view_id = track2.features_[k].view_id_;
			int const line_id = track2.features_[k].feature_id_;
			hw_cams_views_list_[view_id].track_ids_[line_id] = view1_pnt_tid;
		}
		track1.features_.insert(track1.features_.end(),
			track2.features_.begin(), track2.features_.end());
		track1.polygon_idxs_.insert(track1.polygon_idxs_.end(),
			track2.polygon_idxs_.begin(), track2.polygon_idxs_.end());	//insert track1's polygon idx into track2
		/* Free old track's memory. clear() does not work. */
		track2.features_ = HWFeatureReferenceIdList();
	}

	bool HWScenesElements::IsValueValidFlag(float v)
	{
		return (v != std::numeric_limits<float>::quiet_NaN());
	}

	float HWScenesElements::ComputeTwoLineSegsEnergyValue(const std::pair<Eigen::Vector3f, Eigen::Vector3f>& l0,
		const std::pair<Eigen::Vector3f, Eigen::Vector3f>& l1)
	{
		//two dir
		Eigen::Vector3f l0d = l0.second - l0.first;
		Eigen::Vector3f l1d = l1.second - l1.first;
		l0d.normalize();
		l1d.normalize();
		float l0l1cross = std::abs(l0d.dot(l1d));
		float l0l1_deg = std::acosf(l0l1cross)*180.0f / MATH_PI;
		//l0 start pnt to l1
		//Eigen::Vector3f l1s = l1.first;
		//float l0sl1 = ComputePnt3DToLine3DDistF(l1.first, l1d, l0.first);
		//float l0el1 = ComputePnt3DToLine3DDistF(l1.first, l1d, l0.second);
		/*float l0sl1 = ComputePnt3dToLineSegment3DDistF(l1.first, l1.second, l0.first);
		float l0el1 = ComputePnt3dToLineSegment3DDistF(l1.first, l1.second, l0.second);*/

		float l0sl1 = Point3D2LineSegMinDistance(l0.first, l1.first, l1.second);
		float l0el1 = Point3D2LineSegMinDistance(l0.second, l1.first, l1.second);

		float l0tol1 = (l0sl1 + l0el1) / 2.0;
		float e = l0l1_deg + l0tol1*5;
		return e;
	}

	bool HWScenesElements::CheckPairsInPairsSet(const PairSetValue& v, const std::vector<PairSetValue>& vec)
	{
		for (int i = 0; i < vec.size(); ++i)
		{
			if (vec[i] == v)
				return true;
		}
		return false;
	}

    Eigen::Matrix3f HWScenesElements::GetFundamentalMatrix(CameraModel& src, CameraModel& tgt)
    {
        Eigen::Matrix4f sp = src.cam_pose_;
        Eigen::Matrix4f tp = tgt.cam_pose_;
        Eigen::Matrix4f se = sp.inverse();
        Eigen::Matrix4f te = tp.inverse();
        Eigen::Matrix4f re = te * sp;
        Eigen::Matrix3f r_extr = re.topLeftCorner(3,3);
        Eigen::Vector3f t_extr = re.topRightCorner(3,1);
        //compute f matrix
        Eigen::Matrix3f em = ComputeSkewMatrixFromLfVector(t_extr) * r_extr;
        //deal with intrinsic
        Eigen::Matrix3f sk = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f tk = Eigen::Matrix3f::Identity();
        sk(0,0) = src.fx_; sk(1,1) = src.fy_; sk(0,2) = src.cx_; sk(1,2) = src.cy_;
        tk(0,0) = tgt.fx_; tk(1,1) = tgt.fy_; tk(0,2) = tgt.cx_; tk(1,2) = tgt.cy_;
        Eigen::Matrix3f fm = (tk.inverse()).transpose()*em*sk.inverse();
        return fm;
    }

    std::size_t HWScenesElements::GetIdxFromPairVecFirst(unsigned int idx, const std::vector<std::pair<int, CameraModel>>& vec)
    {
        for(std::size_t i = 0; i < vec.size(); ++i)
        {
            if(idx == static_cast<unsigned int>(vec[i].first))
            {
                return i;
            }
        }
        return vec.size();
    }

	int HWScenesElements::GetImageLabelIdxFromImagesLabelPthByHWImagePath(const std::vector<std::string>& images_labels_paths, const std::string& image_path)
	{
		int idx = -1;
		std::vector<std::string> images_base_labels_names = GetAllBaseNamesFromPaths(images_labels_paths);
		std::vector<std::string> images_base_labels_names_dropped = GetAllBaseNameDropLastPattern(images_base_labels_names);
		std::string image_base_path_suffix = GetBaseName(image_path);
		std::string image_base_path = GetPathPrefix(image_base_path_suffix);
		for (int i = 0; i < images_base_labels_names_dropped.size(); ++i)
		{
			if (image_base_path == images_base_labels_names_dropped[i])
			{
				idx = i;
			}
		}
		return idx;
	}
}
