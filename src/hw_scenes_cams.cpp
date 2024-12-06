#include"hw_scenes_cams.h"
//#include"util/misc.h"
#include"hw_algorithms.h"

namespace HW
{
	#define MATH_PI_SCENES         3.14159265358979323846264338327950288

    HWScenesCams::HWScenesCams()
    {

    }

    HWScenesCams::~HWScenesCams()
    {
        
    }

    void HWScenesCams::SetImagesDir(const std::string& imgs_dir)
    {
        image_dir_ = imgs_dir;
    }

    void HWScenesCams::SetCamsDir(const std::string& cams_dir)
    {
        cams_dir_ = cams_dir;
    }

    void HWScenesCams::SetImagesOutDir(const std::string& images_dir)
    {
        images_out_dir_ = images_dir;
    }

    void HWScenesCams::SetCamsOutDir(const std::string& cams_dir)
    {
        cams_out_dir_ = cams_dir;
    }


    void HWScenesCams::LoadElements()
    {
		/*bool use_integer_flag = true;
		for (int i = 0; i < cams_files.size(); ++i)
		{
		std::string file_basename = HW::GetBaseNameWithoutSuffix(cams_files[i]);
		if (file_basename.empty())
		{
		continue;
		}
		if (!HW::IsStrFrontElementInteger(file_basename))
		{
		use_integer_flag = false;
		break;
		}
		}*/
        std::vector<std::string> files_list;
        //std::vector<std::string> images_list;
        std::cerr <<"HWScenesCams image_dir: " << image_dir_ << std::endl;      
		if(!image_dir_.empty())
        {
            files_list = GetFilesListFromDir(image_dir_);
            std::cerr <<"files_list: " << files_list.size() << std::endl;
            std::vector<std::string> images_list;
            for(std::size_t i = 0; i < files_list.size(); ++i)
            {
                if(files_list[i].find(".jpg") != std::string::npos ||
                    files_list[i].find(".JPG") != std::string::npos ||
                    files_list[i].find(".png") != std::string::npos ||
                    files_list[i].find(".PNG") != std::string::npos ||
                    files_list[i].find(".tiff") != std::string::npos ||
                    files_list[i].find(".TIFF") != std::string::npos || 
					files_list[i].find(".jpeg") != std::string::npos ||
					files_list[i].find(".JPEG") != std::string::npos)
                {
					std::string file_name = GetLeftSlashPathName(files_list[i]);
                    images_list.emplace_back(file_name);
                }
            }

			bool use_integer_flag = true;
			for (int i = 0; i < images_list.size(); ++i)
			{
				std::string file_basename = HW::GetBaseNameWithoutSuffix(images_list[i]);
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
				std::vector<std::pair<int, std::string> > images_files_idxs_str;
				for (int i = 0; i < images_list.size(); ++i)
				{
					//sort by file names
					std::string file_basename = HW::GetBaseNameWithoutSuffix(images_list[i]);
					int imageid = std::stoi(file_basename);
					std::pair<int, std::string> tmp_file_name;
					tmp_file_name.first = imageid;
					tmp_file_name.second = images_list[i];
					images_files_idxs_str.emplace_back(tmp_file_name);
				}
				std::sort(images_files_idxs_str.begin(), images_files_idxs_str.end(), CompareHWStrPairByIdxCmp);
				for (std::size_t i = 0; i < images_files_idxs_str.size(); ++i)
				{
					HWImage image;
					//std::cerr << "image path: " << images_files_idxs_str[i].second << std::endl;
					images_files_path_.emplace_back(images_files_idxs_str[i].second);
					unsigned int img_id = images_files_path_.size() - 1;
					image.SetImageId(img_id);
					image.SetHWImagePath(images_files_idxs_str[i].second);
					images_vec_.emplace_back(image);
				}
			}
			else
			{
				//sort the image paths
				std::sort(images_list.begin(), images_list.end());
				for (std::size_t i = 0; i < images_list.size(); ++i)
				{
					HWImage image;
					images_files_path_.emplace_back(images_list[i]);
					unsigned int img_id = images_files_path_.size() - 1;
					image.SetImageId(img_id);
					image.SetHWImagePath(images_list[i]);
					images_vec_.emplace_back(image);
				}
			}
            //std::cerr <<"images_files_path_: " << images_files_path_.size() << std::endl;
        }

        std::cerr << "HWScenesCams cams_dir_: " << cams_dir_ << std::endl;
        if(!cams_dir_.empty())
        {
			std::vector<std::string> cams_files_list;
			for (std::size_t i = 0; i < files_list.size(); ++i)
			{
				if (files_list[i].find(".CAM") != std::string::npos ||
					files_list[i].find(".cam") != std::string::npos)
				{
					std::string file_name = GetLeftSlashPathName(files_list[i]);
					cams_files_list.emplace_back(file_name);
				}
			}
			bool use_integer_flag = true;
			for (int i = 0; i < cams_files_list.size(); ++i)
			{
				std::string file_basename = HW::GetBaseNameWithoutSuffix(cams_files_list[i]);
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
				std::vector<std::pair<int, std::string> > cams_files_idxs_str;
				for (int i = 0; i < cams_files_list.size(); ++i)
				{
					//sort by file names
					std::string file_basename = HW::GetBaseNameWithoutSuffix(cams_files_list[i]);
					int camid = std::stoi(file_basename);
					std::pair<int, std::string> tmp_file_name;
					tmp_file_name.first = camid;
					tmp_file_name.second = cams_files_list[i];
					cams_files_idxs_str.emplace_back(tmp_file_name);
				}
				std::sort(cams_files_idxs_str.begin(), cams_files_idxs_str.end(), CompareHWStrPairByIdxCmp);
				for (int i = 0; i < cams_files_idxs_str.size(); ++i)
				{
					//std::cerr << "cam path: " << cams_files_idxs_str[i].second << std::endl;
					cams_path_.emplace_back(cams_files_idxs_str[i].second);
					unsigned int cam_id = cams_path_.size() - 1;
					//set false
					CameraModel tmp_cam;
					tmp_cam.cam_id_ = cam_id;
					tmp_cam.LoadCamFromCam(cams_files_idxs_str[i].second);
					tmp_cam.valid_ = false;
					cams_models_.emplace_back(tmp_cam);
				}
			}
			else
			{
				std::sort(cams_files_list.begin(), cams_files_list.end());
				for (int i = 0; i < cams_files_list.size(); ++i)
				{
					cams_path_.emplace_back(cams_files_list[i]);
					unsigned int cam_id = cams_path_.size() - 1;
					//set false
					CameraModel tmp_cam;
					tmp_cam.cam_id_ = cam_id;
					tmp_cam.LoadCamFromCam(cams_files_list[i]);
					tmp_cam.valid_ = false;
					cams_models_.emplace_back(tmp_cam);
				}
			}
        }
		//update image and cams (Important...)
		UpdateCamsModelsId2ImagesModelsId();
    }

	void HWScenesCams::LoadElementsOld()
	{
		std::vector<std::string> files_list;
		//std::vector<std::string> images_list;
		std::cerr << "image_dir: " << image_dir_ << std::endl;

		if (!image_dir_.empty())
		{
			files_list = GetFilesListFromDir(image_dir_);
			std::cerr << "files_list: " << files_list.size() << std::endl;
			std::vector<std::string> images_list;
			for (std::size_t i = 0; i < files_list.size(); ++i)
			{
				if (files_list[i].find(".jpg") != std::string::npos ||
					files_list[i].find(".JPG") != std::string::npos ||
					files_list[i].find(".png") != std::string::npos ||
					files_list[i].find(".PNG") != std::string::npos ||
					files_list[i].find(".tiff") != std::string::npos ||
					files_list[i].find(".TIFF") != std::string::npos ||
					files_list[i].find(".jpeg") != std::string::npos ||
					files_list[i].find(".JPEG") != std::string::npos)
				{
					std::string file_name = GetLeftSlashPathName(files_list[i]);
					images_list.emplace_back(file_name);
				}
			}
			//sort the image paths
			std::sort(images_list.begin(), images_list.end());
			for (std::size_t i = 0; i < images_list.size(); ++i)
			{
				HWImage image;
				images_files_path_.emplace_back(images_list[i]);
				unsigned int img_id = images_files_path_.size() - 1;
				image.SetImageId(img_id);
				image.SetHWImagePath(images_list[i]);
				images_vec_.emplace_back(image);
			}
			//std::cerr <<"images_files_path_: " << images_files_path_.size() << std::endl;
		}

		std::cerr << "HWScenesCams cams_dir_: " << cams_dir_ << std::endl;
		if (!cams_dir_.empty())
		{
			for (std::size_t i = 0; i < files_list.size(); ++i)
			{
				if (files_list[i].find(".CAM") != std::string::npos ||
					files_list[i].find(".cam") != std::string::npos)
				{
					std::string file_name = GetLeftSlashPathName(files_list[i]);
					cams_path_.emplace_back(file_name);
					unsigned int cam_id = cams_path_.size() - 1;
					//set false
					CameraModel tmp_cam;
					tmp_cam.cam_id_ = cam_id;
					tmp_cam.LoadCamFromCam(file_name);
					tmp_cam.valid_ = false;
					cams_models_.emplace_back(tmp_cam);
				}
			}
		}

		//update image and cams (Important...)
		UpdateCamsModelsId2ImagesModelsId();
	}

	void HWScenesCams::SetCamerasModels(std::vector<CameraModel>& cams_models)
	{
		cams_path_.clear();
		for (std::size_t i = 0; i < cams_models.size(); ++i)
		{
			cams_models_.emplace_back(cams_models[i]);
			cams_path_.emplace_back(cams_models_[i].path_);
		}
	}

	void HWScenesCams::SetHWImageModelsFromPaths(std::vector<std::string>& images_paths)
	{
		images_files_path_.clear();
		for (std::size_t i = 0; i < images_paths.size(); ++i)
		{
			HWImage tmp_image;
			tmp_image.SetHWImagePath(images_paths[i]);
			tmp_image.SetImageId(i);
			images_vec_.emplace_back(tmp_image);
			images_files_path_.emplace_back(images_paths[i]);
		}
	}

    void HWScenesCams::SetLayoutIdFromImageIdx(std::size_t imgidx, unsigned int lid)
    {
        if(imgidx >= images_vec_.size() || imgidx < 0)
        {
            return;
        }
        images_vec_[imgidx].SetLayoutId(lid);
    }

	void HWScenesCams::SetLayoutIdByImageIdx(unsigned int imgidx, unsigned int lyid)
	{
		if (imgidx >= images_vec_.size() || imgidx < 0)
		{
			return;
		}
		images_vec_[imgidx].SetLayoutId(lyid);
	}

	void HWScenesCams::SetLayoutIdByImageId(unsigned int imgid, unsigned int lyid)
	{
		int imgidx = -1;
		for (int i = 0; i < images_vec_.size(); ++i)
		{
			if (images_vec_[i].GetImageId() == imgid)
			{
				imgidx = i;
				break;
			}
		}
		if (imgidx != -1)
		{
			images_vec_[imgidx].SetLayoutId(lyid);
		}
	}

	void HWScenesCams::SetLayoutIdByCamIdx(unsigned int camidx, unsigned int lyid)
	{
		if (camidx >= cams_models_.size() || camidx < 0)
		{
			return;
		}
		cams_models_[camidx].layout_id_ = lyid;
	}

	void HWScenesCams::SetLayoutIdByCamId(unsigned int camid, unsigned int lyid)
	{
		int camidx = -1;
		for (int i = 0; i < cams_models_.size(); ++i)
		{
			if (cams_models_[i].cam_id_ == camid)
			{
				camidx = i;
				break;
			}
		}
		if (camidx != -1)
		{
			cams_models_[camidx].layout_id_ = lyid;
		}
	}

	void HWScenesCams::UpdateLinesSegsDetectedFromImgid(int imgid)
	{
		int img_idx = GetImageIdxFromImgId(imgid);
		if (img_idx == -1)
		{
			return;
		}
		//images_vec_[img_idx].DetectLinesSegsOpencvLbd();
		//images_vec_[img_idx].DetectLineSegsOpencvLsd();
		images_vec_[img_idx].DetectLinesSegsOpencv();
		//DetectLineSegsOpencvLsd
	}

	void HWScenesCams::UpdateSiftImageFromImgid(int imgid)
	{
		int img_idx = GetImageIdxFromImgId(imgid);
		if (img_idx == -1)
		{
			return;
		}
		images_vec_[img_idx].ComputeSiftFeatures();
	}

	void HWScenesCams::UpdateImageLoadedFromImgid(int imgid)
	{
		int img_idx = GetImageIdxFromImgId(imgid);
		if (img_idx == -1)
		{
			return;
		}
		images_vec_[img_idx].LoadHWImage();
	}

	void HWScenesCams::SetImageSiftKeyPointsFromImgid(int imgid, const std::vector<cv::KeyPoint>& key_positions)
	{
		int img_idx = GetImageIdxFromImgId(imgid);
		if (img_idx == -1)
		{
			return;
		}
		images_vec_[img_idx].SetSiftKeyPoints(key_positions);
	}

	void HWScenesCams::SetImageNetworkPointsFromImgid(int imgid, const std::vector<cv::Point2f>& img_pnts)
	{
		int img_idx = GetImageIdxFromImgId(imgid);
		if (img_idx == -1)
		{
			return;
		}
		images_vec_[img_idx].SetImageNetWorkPntsPos(img_pnts);
	}

	void HWScenesCams::WriteImageNetworkPntsIntoImageByImgid(int imgid, const std::string& path)
	{
		int img_idx = GetImageIdxFromImgId(imgid);
		if (img_idx == -1)
		{
			return;
		}
		images_vec_[img_idx].WriteNetWorkPntsIntoOwnImage(path);
	}

	bool HWScenesCams::GetImageNetworkPointsLoadedFromImgid(int imgid)
	{
		int img_idx = GetImageIdxFromImgId(imgid);
		if (img_idx == -1)
		{
			return false;
		}
		return images_vec_[img_idx].GetImageNetworkPntsLoaded();
	}

    unsigned int HWScenesCams::GetCamIdFromImageId(int imgid)
    {
        for(std::size_t i = 0; i < images_vec_.size(); ++i)
        {
            if(images_vec_[i].GetImageId() == imgid)
            {
                return images_vec_[i].GetCamId();
            }
        }
        return std::numeric_limits<unsigned int>::max();
    }

	unsigned int HWScenesCams::GetLayoutIdFromCamId(int camid)
	{
		for (std::size_t i = 0; i < cams_models_.size(); ++i)
		{
			if (cams_models_[i].cam_id_ == camid)
			{
				return cams_models_[i].layout_id_;
			}
		}
		return std::numeric_limits<unsigned int>::max();
	}

    unsigned int HWScenesCams::GetImageidFromCamId(int camid)
    {
        for(std::size_t i = 0; i < cams_models_.size(); ++i)
        {
            if(cams_models_[i].cam_id_ == camid)
            {
                return cams_models_[i].image_id_;
            }
        }
        return std::numeric_limits<unsigned int>::max();
    }

	unsigned int HWScenesCams::GetCamidxFromCamId(int camid)
	{
		for (std::size_t i = 0; i < cams_models_.size(); ++i)
		{
			if (cams_models_[i].cam_id_ == camid)
			{
				return i;
			}
		}
		return std::numeric_limits<unsigned int>::max();
	}

    void HWScenesCams::GroupCamsIdBasedOnCamDirect()
    {
        //
        for(int i = 0; i < cams_models_.size(); ++i)
        {
            std::vector<int> group;
            Eigen::Vector3f src_cam_dir = cams_models_[i].cam_pose_.block<3, 1>(0, 2);
            //group.emplace_back(i);
            //visited[i] = true;
            int src_camid = cams_models_[i].cam_id_;
            for(int j =0; j < cams_models_.size(); ++j)
            {
                if(src_camid == cams_models_[j].cam_id_)
                {
                    //same camera
                    continue;
                }
                Eigen::Vector3f tgt_cam_dir = cams_models_[j].cam_pose_.block<3, 1>(0, 2);
                float cams_angle = src_cam_dir.dot(tgt_cam_dir) / src_cam_dir.norm() / tgt_cam_dir.norm();
                float cams_degree = std::acos(cams_angle)*180.0 / MATH_PI_SCENES;
                if(cams_degree < KANGLE_THRESHOLD)
                {
                    int tgt_camid = cams_models_[j].cam_id_; 
                    group.emplace_back(tgt_camid);
                    //visited[j] = true;
                }
            }
            HWCamsIdxGroup camsidxs;
            camsidxs.cams_idxs_ = group;
            cams_groups_[src_camid] = camsidxs;
            //cams_groups_.emplace_back(camsidxs);
        }
    }

	void HWScenesCams::GroupCamsIdBasedOnLSDLinesParis()
	{
		for (int i = 0; i < cams_models_.size(); ++i)
		{
			std::vector<int> group;
			//Eigen::Vector3f src_cam_dir = cams_models_[i].cam_pose_.block<3, 1>(0, 2);
			//group.emplace_back(i);
			//visited[i] = true;
			int src_camid = cams_models_[i].cam_id_;
			Eigen::Vector3f src_cam_dir = cams_models_[i].cam_pose_.block<3, 1>(0, 2);
			Eigen::Vector3f src_cam_pos = cams_models_[i].cam_pose_.block<3, 1>(0, 3);
			for (int j = 0; j < cams_models_.size(); ++j)
			{
				if (src_camid == cams_models_[j].cam_id_)
				{
					//same camera
					continue;
				}
				Eigen::Vector3f tgt_cam_dir = cams_models_[j].cam_pose_.block<3, 1>(0, 2);
				Eigen::Vector3f tgt_cam_pos = cams_models_[j].cam_pose_.block<3, 1>(0, 3);
				float cams_angle = src_cam_dir.dot(tgt_cam_dir) / src_cam_dir.norm() / tgt_cam_dir.norm();
				float cams_angle_degree = std::acos(cams_angle)*180.0 / MATH_PI_SCENES;
				float dist_s2t = (tgt_cam_pos - src_cam_pos).norm();

				if (cams_angle_degree < KANGLE_THRESHOLD && 
					dist_s2t < cam_pos_to_cam_pos_threshold)
				{
					int tgt_camid = cams_models_[j].cam_id_;

					group.emplace_back(tgt_camid);
				}
			}

			//group cams

			HWCamsIdxGroup camsidxs;
			camsidxs.cams_idxs_ = group;
			cams_groups_[src_camid] = camsidxs;
		}
	}

	void HWScenesCams::UpdateImagesModelLoadImgs()
	{
		for (int i = 0; i < images_vec_.size(); ++i)
		{
			if (!images_vec_[i].ImageLoaded())
			{
				images_vec_[i].LoadHWImage();
			}
		}
	}

	void HWScenesCams::MatchLinesBasedOnLinesFeatureLbd(int src_id, int tgt_id,
		std::vector<cv::DMatch>& lines2lines)
	{
		int src_image_id = cams_models_[src_id].image_id_;
		int tgt_image_id = cams_models_[tgt_id].image_id_;
		const HWImage& src_image = images_vec_[src_image_id];
		const HWImage& tgt_image = images_vec_[tgt_image_id];

		const cv::Mat left_lbd = src_image.GetLbdImageLinesDescriptors();
		const cv::Mat right_lbd = src_image.GetLbdImageLinesDescriptors();

		/* create a BinaryDescriptorMatcher object */
		cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher> bdm 
			= cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();

		/* require match */
		std::vector<cv::DMatch> matches;
		bdm->match(left_lbd, right_lbd, matches);

		/* select best matches */
		std::vector<cv::DMatch> good_matches;
		for (int i = 0; i < (int)matches.size(); i++)
		{
			if (matches[i].distance < lines_descriptor_matches_dist_threshold)
				good_matches.push_back(matches[i]);
		}
		lines2lines = good_matches;

	}

	void HWScenesCams::MatchLinesBasedOnLinesFeatureLsd(int src_id, int tgt_id,
		std::vector<cv::DMatch>& lines2lines)
	{
		int src_image_id = cams_models_[src_id].image_id_;
		int tgt_image_id = cams_models_[tgt_id].image_id_;
		const HWImage& src_image = images_vec_[src_image_id];
		const HWImage& tgt_image = images_vec_[tgt_image_id];
		std::vector<cv::line_descriptor::KeyLine> src_lsd_lines = src_image.GetLsdImageKeyLinesOpencv();
		std::vector<cv::line_descriptor::KeyLine> tgt_lsd_lines = tgt_image.GetLsdImageKeyLinesOpencv();

		const cv::Mat left_lbd = src_image.GetLsdImageLinesDescriptors();
		const cv::Mat right_lbd = src_image.GetLsdImageLinesDescriptors();

		/* create a BinaryDescriptorMatcher object */
		cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher> bdm
			= cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();

		/* require match */
		std::vector<cv::DMatch> matches;
		bdm->match(left_lbd, right_lbd, matches);

		/* select best matches */
		std::vector<cv::DMatch> good_matches;
		for (int i = 0; i < (int)matches.size(); i++)
		{
			if (matches[i].distance < lines_descriptor_matches_dist_threshold)
				good_matches.push_back(matches[i]);
		}
		lines2lines = good_matches;

		/* plot matches */
		cv::Mat src_img_cv = src_image.GetImage();
		cv::Mat tgt_img_cv = tgt_image.GetImage();
		cv::Mat src_img_cv_tmp = src_img_cv.clone();
		cv::Mat tgt_img_cv_tmp = tgt_img_cv.clone();
		cv::Mat lsd_outImg;
		cv::resize(src_img_cv_tmp, src_img_cv_tmp, cv::Size(src_img_cv_tmp.cols / 2, src_img_cv_tmp.rows / 2), 0, 0, cv::INTER_LINEAR_EXACT);
		cv::resize(tgt_img_cv_tmp, tgt_img_cv_tmp, cv::Size(tgt_img_cv_tmp.cols / 2, tgt_img_cv_tmp.rows / 2), 0, 0, cv::INTER_LINEAR_EXACT);
		std::vector<char> lsd_mask(matches.size(), 1);
		cv::line_descriptor::drawLineMatches(src_img_cv_tmp, src_lsd_lines, tgt_img_cv_tmp, tgt_lsd_lines, good_matches, lsd_outImg, cv::Scalar::all(-1), cv::Scalar::all(-1), lsd_mask,
			cv::line_descriptor::DrawLinesMatchesFlags::DEFAULT);

		imshow("LSD matches", lsd_outImg);
		cv::waitKey();
	}

	void HWScenesCams::GroupCamsIdBasedOnLSDLinesParisAndCamParams()
	{
		std::cerr << "to group the cameras idxs..." << std::endl;
		for (int i = 0; i < cams_models_.size(); ++i)
		{
			std::vector<int> group;
			//Eigen::Vector3f src_cam_dir = cams_models_[i].cam_pose_.block<3, 1>(0, 2);
			//group.emplace_back(i);
			//visited[i] = true;
			int src_camid = cams_models_[i].cam_id_;
			for (int j = 0; j < cams_models_.size(); ++j)
			{
				if (src_camid == cams_models_[j].cam_id_)
				{
					//same camera
					continue;
				}
			}
		}
		std::cerr << "end group the cameras idxs..." << std::endl;
	}

    std::size_t HWScenesCams::GetCamerasNum()
    {
        return cams_models_.size();
    }

	const std::size_t HWScenesCams::GetCamerasNum() const
	{
		return cams_models_.size();
	}

    std::size_t HWScenesCams::GetImagesNum()
    {
        return images_files_path_.size(); //to do next for it
    }

	const std::size_t HWScenesCams::GetImagesNum() const
	{
		return images_files_path_.size(); //to do next for it
	}

    const std::vector<CameraModel> & HWScenesCams::GetCamerasModels()
    {
        return cams_models_;
    }

	const std::vector<CameraModel>& HWScenesCams::GetCamerasModels() const
	{
		return cams_models_;
	}

    const std::vector<HWImage>& HWScenesCams::GetImagesModels()
    {
        return images_vec_;
    }

	const std::vector<HWImage>& HWScenesCams::GetImagesModels() const
	{
		return images_vec_;
	}

    const CameraModel& HWScenesCams::GetCameraModelFromCamId(int cam_id)
    {
        for(std::size_t i = 0; i < cams_models_.size(); ++i)
        {
            if(cam_id == cams_models_[i].cam_id_)
            {
                return cams_models_[i];
            }
        }
        return CameraModel();
    }

	const CameraModel& HWScenesCams::GetCameraModelFromCamId(int cam_id) const
	{
		for (std::size_t i = 0; i < cams_models_.size(); ++i)
		{
			if (cam_id == cams_models_[i].cam_id_)
			{
				return cams_models_[i];
			}
		}
		return CameraModel();
	}

    const HWImage& HWScenesCams::GetImageFromImgId(int img_id)
    {
        for(std::size_t i = 0; i < images_vec_.size(); ++i)
        {
            if(img_id == images_vec_[i].GetImageId())
            {
				//std::cerr << "asdfasdfasdfasdfasdfasdfasdfasdfasdfa" << std::endl;
                return images_vec_[i];
            }
        }
        return HWImage();
    }

	const HWImage& HWScenesCams::GetImageFromImgId(int img_id) const
	{
		for (std::size_t i = 0; i < images_vec_.size(); ++i)
		{
			if (img_id == images_vec_[i].GetImageId())
			{
				return images_vec_[i];
			}
		}
		return HWImage();
	}

    const std::vector<std::string>& HWScenesCams::GetCamerasPaths()
    {
        return cams_path_;
    }

    const std::vector<std::string>& HWScenesCams::GetImagesPaths()
    {
        return images_files_path_;
    }

    const std::map<unsigned int, HWCamsIdxGroup>& HWScenesCams::GetCamsGroups()
    {
        return cams_groups_;
    }

    void HWScenesCams::SaveAllCamsPoseIntoObj(const std::string& path, float aixlen)
    {
        //get cams obj
        std::ofstream fh(path);
        std::vector<Eigen::Matrix4f> cmps;
        for(std::size_t i = 0; i < cams_models_.size(); ++i)
        {
            cmps.emplace_back(cams_models_[i].cam_pose_);
        }
        for(std::size_t i = 0; i < cmps.size(); ++i)
        {
            //x,y,z aix
            Eigen::Vector3f lx = cmps[i].block<3,1>(0,0);
            Eigen::Vector3f ly = cmps[i].block<3,1>(0,1);
            Eigen::Vector3f lz = cmps[i].block<3,1>(0,2);
            Eigen::Vector3f t = cmps[i].block<3,1>(0,3);
            lx.normalize();
            ly.normalize();
            lz.normalize();
            Eigen::Vector3f ex = t+aixlen*lx;
            Eigen::Vector3f ey = t+aixlen*ly;
            Eigen::Vector3f ez = t+aixlen*lz;
            fh << "v " << t[0] << " " << t[1] << " " << t[2]
                << " " << 255 << " " << 255 << " " << 255 << std::endl;
            fh << "v " << ex[0] << " " << ex[1] << " " << ex[2] 
                << " " << 255 << " " << 0 << " " << 0 << std::endl;
            fh << "v " << ey[0] << " " << ey[1] << " " << ey[2] 
                << " " << 0 << " " << 255 << " " << 0 << std::endl;
            fh << "v " << ez[0] << " " << ez[1] << " " << ez[2] 
                << " " << 0 << " " << 0 << " " << 255 << std::endl;
        }
        for(std::size_t i = 0 ; i < cmps.size(); ++i)
        {
            fh << "l " << 4*i+1 << " " << 4*i+2 << std::endl;
            fh << "l " << 4*i+1 << " " << 4*i+3 << std::endl;
            fh << "l " << 4*i+1 << " " << 4*i+4 << std::endl;
        }
        fh.close();
    }

    void HWScenesCams::SaveSelectedCamsPoseIntoObj(const std::string& path,  
            const std::vector<CameraModel>& cams, float aixlen)
    {
        //get cams obj
        std::ofstream fh(path);
        std::vector<Eigen::Matrix4f> cmps;
        for(std::size_t i = 0; i < cams.size(); ++i)
        {
            cmps.emplace_back(cams[i].cam_pose_);
        }
        for(std::size_t i = 0; i < cmps.size(); ++i)
        {
            //x,y,z aix
            Eigen::Vector3f lx = cmps[i].block<3,1>(0,0);
            Eigen::Vector3f ly = cmps[i].block<3,1>(0,1);
            Eigen::Vector3f lz = cmps[i].block<3,1>(0,2);
            Eigen::Vector3f t = cmps[i].block<3,1>(0,3);
            lx.normalize();
            ly.normalize();
            lz.normalize();
            Eigen::Vector3f ex = t+aixlen*lx;
            Eigen::Vector3f ey = t+aixlen*ly;
            Eigen::Vector3f ez = t+aixlen*lz;
            fh << "v " << t[0] << " " << t[1] << " " << t[2]
                << " " << 255 << " " << 255 << " " << 255 << std::endl;
            fh << "v " << ex[0] << " " << ex[1] << " " << ex[2] 
                << " " << 255 << " " << 0 << " " << 0 << std::endl;
            fh << "v " << ey[0] << " " << ey[1] << " " << ey[2] 
                << " " << 0 << " " << 255 << " " << 0 << std::endl;
            fh << "v " << ez[0] << " " << ez[1] << " " << ez[2] 
                << " " << 0 << " " << 0 << " " << 255 << std::endl;
        }
        for(std::size_t i = 0 ; i < cmps.size(); ++i)
        {
            fh << "l " << 4*i+1 << " " << 4*i+2 << std::endl;
            fh << "l " << 4*i+1 << " " << 4*i+3 << std::endl;
            fh << "l " << 4*i+1 << " " << 4*i+4 << std::endl;
        }
        fh.close();  
    }

	void HWScenesCams::UpdateCamsModelsId2ImagesModelsId()
	{
		//update image and cams
		for (std::size_t i = 0; i < images_vec_.size(); ++i)
		{
			const std::string image_path = images_vec_[i].GetImagePath();
			for (std::size_t j = 0; j < cams_models_.size(); ++j)
			{
				const std::string cam_path = cams_models_[j].path_;
				//std::cerr << "cam_path: " << cam_path << std::endl;
				bool corresponding = CompareBaseNameFromTwoPaths(image_path, cam_path);
				if (corresponding)
				{
					//std::cerr << "corresponding..." << std::endl;
					//std::cerr << "cam_path: " << cam_path << std::endl;
					//std::cerr << "image_path: " << image_path << std::endl;
					cv::Mat img = cv::imread(images_vec_[i].GetImagePath());
					if (std::abs(cams_models_[j].fx_ - 0.5) < KMIN_FLOAT_THRESHOLD && !img.empty())
					{
						int w = img.cols;
						int h = img.rows;
						cams_models_[j].fx_ = float(0.5*w);
						cams_models_[j].fy_ = float(0.5*w);
						cams_models_[j].cx_ = float(0.5*w);
						cams_models_[j].cy_ = float(0.5*h);
						cams_models_[j].valid_ = true;
					}
					else
					{
						cams_models_[j].valid_ = true;
					}
					images_vec_[i].SetCamId(cams_models_[j].cam_id_);
					cams_models_[j].image_id_ = images_vec_[i].GetImageId();
					imagesToCamsId_[images_vec_[i].GetImageId()] = cams_models_[j].cam_id_;
					break;
				}
			}
		}
	}

    bool HWScenesCams::CompareBaseNameFromTwoPaths(const std::string& spath, const std::string& tpath)
    {
        std::string src_name = spath.substr(spath.find_last_of("/") + 1, 
            spath.find_last_of(".") - spath.find_last_of("/") - 1);
        std::string tgt_name = tpath.substr(tpath.find_last_of("/") + 1, 
            tpath.find_last_of(".") - tpath.find_last_of("/") - 1);
        if(src_name == tgt_name)
        {
            return true;
        }
        return false;
    }

    int HWScenesCams::CompareBaseNameVec(const std::string& str, const std::vector<std::string>& strs_vec)
    {
         std::string src_name = str.substr(str.find_last_of("/") + 1, 
            str.find_last_of(".") - str.find_last_of("/") - 1);
        for(std::size_t i = 0; i < strs_vec.size(); ++i)
        {
            std::string tgt_name = strs_vec[i].substr(strs_vec[i].find_last_of("/") + 1, 
            strs_vec[i].find_last_of(".") - strs_vec[i].find_last_of("/") - 1);
            if(src_name == tgt_name)
            {
                return (int)i;
            }
        }
        return -1;
    }

	int HWScenesCams::GetImageIdxFromImgId(int imgid)
	{
		for (std::size_t i = 0; i < images_vec_.size(); ++i)
		{
			if (imgid == images_vec_[i].GetImageId())
			{
				return i;
			}
		}
		return -1;
	}

	bool HWScenesCams::CompareHWStrPairByIdxCmp(const std::pair<int, std::string>& a, const std::pair<int, std::string>& b)
	{
		return a.first < b.first;
	}
}
