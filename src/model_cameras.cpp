#include"model_cameras.h"
#include<Windows.h>
#include<io.h>
#include<opencv2\opencv.hpp>
#include"hw_cmns.h"

ModelCameras::ModelCameras()
{
	screen_near_ = 0.001;
	screen_far_ = 1000.0;
	offscreen_width_ = 0;
	offscreen_height_ = 0;
}

ModelCameras::~ModelCameras()
{

}

void ModelCameras::LoadImagesFromRealityCaputureImgsdir(const std::string& img_dir)
{
	//
	std::vector<std::string> abs_files;
	GetCurrentDirFiles(img_dir, abs_files);
	if (abs_files.empty())
		std::cout << "LoadImagesFromRealityCaputureImgsdir: file empty..." << std::endl;
	std::vector<std::string> abs_image_files;
	std::string ignore_txt;
	for (int i = 0; i < abs_files.size(); ++i)
	{
		if (abs_files[i].find(".JPG") != std::string::npos
			|| abs_files[i].find(".jpg") != std::string::npos
			|| abs_files[i].find(".png") != std::string::npos
			|| abs_files[i].find(".PNG") != std::string::npos)
			abs_image_files.emplace_back(abs_files[i]);
		if (abs_files[i].find(".txt") != std::string::npos)
		{
			ignore_txt = abs_files[i];
		}
	}
	//保存起来
	for (int i = 0; i < abs_image_files.size(); ++i)
	{
		cv::Mat img = cv::imread(abs_image_files[i]);
		Eigen::Vector2i tmp_wh;
		tmp_wh[0] = img.cols;
		tmp_wh[1] = img.rows;
		images_paths_.emplace_back(abs_image_files[i]);
		cams_imgs_width_height_.emplace_back(tmp_wh);
	}
	cams_poses_.resize(images_paths_.size());
	std::vector<std::string> ignore_files;
	if (!ignore_txt.empty())
	{
		ReadFileIgnoreFilesFromTXT(ignore_txt, ignore_files);
	}
	std::vector<std::string> imgs_name;
	for (std::size_t i = 0; i < abs_image_files.size(); i++)
	{
		std::string abs_img_file = abs_image_files[i];
		std::string img_file = abs_img_file.substr(abs_img_file.find_last_of("/") + 1, abs_img_file.length() - abs_img_file.find_last_of("/"));
		std::cerr << "img_file: " << img_file << std::endl;
		imgs_name.emplace_back(img_file);
	}
	for (int i = 0; i < imgs_name.size(); ++i)
	{
		std::cerr << "i: " << i << std::endl;
		bool fl = FindStrFromStrVec(ignore_files, imgs_name[i]);
		SetPickedCamSate(i, !fl);
	}
}

void ModelCameras::LoadCamsFromCAMDIR(const std::string& dir)
{
	std::vector<std::string> files;
	GetCurrentDirFiles(dir, files);
	if (files.empty())
		std::cout << "LoadCamsFromCAMDIR: file empty..." << std::endl;
	//
	std::vector<std::string> cams_files;
	std::vector<std::string> images_files;
	for (int i = 0; i < files.size(); ++i)
	{
		if (files[i].find(".CAM") != std::string::npos ||
			files[i].find(".cam") != std::string::npos)
		{
			std::string cam_left_slash_name = HW::GetLeftSlashPathName(files[i]);
			cams_files.emplace_back(cam_left_slash_name);
		}
		if (files[i].find(".JPG") != std::string::npos
			|| files[i].find(".jpg") != std::string::npos
			|| files[i].find(".png") != std::string::npos
			|| files[i].find(".PNG") != std::string::npos
			|| files[i].find(".JPEG") != std::string::npos
			|| files[i].find(".jpeg") != std::string::npos
			|| files[i].find(".TIFF") != std::string::npos
			|| files[i].find(".tiff") != std::string::npos)
		{
			images_files.emplace_back(files[i]);
		}
	}
	bool use_integer_flag = true;
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
	}
	if (use_integer_flag)
	{
		std::vector<std::pair<int, std::string> > cams_files_idxs_str;
		std::vector<std::pair<int, std::string> > images_files_idxs_str;
		for (int i = 0; i < cams_files.size(); ++i)
		{
			//sort by file names
			std::string file_basename = HW::GetBaseNameWithoutSuffix(cams_files[i]);
			int camid = std::stoi(file_basename);
			std::pair<int, std::string> tmp_file_name;
			tmp_file_name.first = camid;
			tmp_file_name.second = cams_files[i];
			cams_files_idxs_str.emplace_back(tmp_file_name);
		}
		for (int i = 0; i < images_files.size(); ++i)
		{
			//sort by file names
			std::string file_basename = HW::GetBaseNameWithoutSuffix(images_files[i]);
			int imageid = std::stoi(file_basename);
			std::pair<int, std::string> tmp_file_name;
			tmp_file_name.first = imageid;
			tmp_file_name.second = images_files[i];
			images_files_idxs_str.emplace_back(tmp_file_name);
		}
		//sort all cams and images names
		std::sort(cams_files_idxs_str.begin(), cams_files_idxs_str.end(), CompareHWStrPairByIdxCmp);
		std::sort(images_files_idxs_str.begin(), images_files_idxs_str.end(), CompareHWStrPairByIdxCmp);
		for (int i = 0; i < images_files_idxs_str.size(); ++i)
		{
			//std::cerr << "image path: " << images_files_idxs_str[i].second << std::endl;
			images_paths_.emplace_back(images_files_idxs_str[i].second);
		}
		for (int i = 0; i < cams_files_idxs_str.size(); ++i)
		{
			CameraModel tmp_model;
			//std::cerr << "cam path: " << cams_files_idxs_str[i].second << std::endl;
			tmp_model.LoadCamFromCam(cams_files_idxs_str[i].second);
			tmp_model.image_id_ = i;
			tmp_model.cam_id_ = i;
			if (std::abs(tmp_model.cx_) < 1e-2)
			{
				//std::cout << "the cam img 222: " << i << std::endl;
				cv::Mat img = cv::imread(images_paths_[i]);
				std::cout << "the cam img path: " << images_paths_[i] << std::endl;
				if (!img.empty())
				{
					//重新修改相机的参数
					//std::cout << "the cam img: " << i << std::endl;
					tmp_model.fx_ = 0.5*img.cols;
					tmp_model.fy_ = 0.5*img.cols;
					tmp_model.cx_ = 0.5*img.cols;
					tmp_model.cy_ = 0.5*img.rows;
					std::cout << "the cam img cx: " << tmp_model.cx_ << std::endl;
				}
			}
			cams_poses_.emplace_back(tmp_model);
		}
	}
	else
	{
		std::sort(cams_files.begin(), cams_files.end());
		std::sort(images_files.begin(), images_files.end());
		std::cout << "cams files size is: " << cams_files.size() << std::endl;
		for (int i = 0; i < cams_files.size(); ++i)
		{
			CameraModel tmp_model;
			tmp_model.LoadCamFromCam(cams_files[i]);
			//std::cout << "the cam img 333: " << i <<":  " << tmp_model.cx_ << std::endl;
			//get corresponding data
			/*
			get tmp_model.image from images_paths_ i
			to do next (provide compare images_paths with cams_files[i])
			*/
			tmp_model.image_id_ = i;
			tmp_model.cam_id_ = i;
			images_paths_.emplace_back(images_files[i]);
			if (std::abs(tmp_model.cx_) < 1e-2)
			{
				//
				//std::cout << "the cam img 222: " << i << std::endl;
				cv::Mat img = cv::imread(images_paths_[i]);
				std::cout << "the cam img path: " << images_paths_[i] << std::endl;
				if (!img.empty())
				{
					//重新修改相机的参数
					//std::cout << "the cam img: " << i << std::endl;
					tmp_model.fx_ = 0.5*img.cols;
					tmp_model.fy_ = 0.5*img.cols;
					tmp_model.cx_ = 0.5*img.cols;
					tmp_model.cy_ = 0.5*img.rows;
					std::cout << "the cam img cx: " << tmp_model.cx_ << std::endl;
				}
			}
			cams_poses_.emplace_back(tmp_model);
		}
	}
}

void ModelCameras::LoadCamsFromCAMDIROld(const std::string& dir)
{
	std::vector<std::string> files;
	GetCurrentDirFiles(dir, files);
	if (files.empty())
		std::cout << "LoadCamsFromCAMDIR: file empty..." << std::endl;
	//
	std::vector<std::string> cams_files;
	for (int i = 0; i < files.size(); ++i)
	{
		if (files[i].find(".CAM") != std::string::npos ||
			files[i].find(".cam") != std::string::npos)
		{
			std::string cam_left_slash_name = HW::GetLeftSlashPathName(files[i]);
			cams_files.emplace_back(cam_left_slash_name);
		}
		if (files[i].find(".JPG") != std::string::npos
			|| files[i].find(".jpg") != std::string::npos
			|| files[i].find(".png") != std::string::npos
			|| files[i].find(".PNG") != std::string::npos)
		{
			images_paths_.emplace_back(files[i]);
		}
	}
	std::sort(cams_files.begin(), cams_files.end());
	std::sort(images_paths_.begin(), images_paths_.end());
	std::cout << "cams files size is: " << cams_files.size() << std::endl;
	for (int i = 0; i < cams_files.size(); ++i)
	{
		CameraModel tmp_model;
		tmp_model.LoadCamFromCam(cams_files[i]);
		//std::cout << "the cam img 333: " << i <<":  " << tmp_model.cx_ << std::endl;
		//get corresponding data
		/*
		get tmp_model.image from images_paths_ i
		to do next (provide compare images_paths with cams_files[i])
		*/
		tmp_model.image_id_ = i;
		tmp_model.cam_id_ = i;
		if (std::abs(tmp_model.cx_) < 1e-2)
		{
			//
			//std::cout << "the cam img 222: " << i << std::endl;
			cv::Mat img = cv::imread(images_paths_[i]);
			std::cout << "the cam img path: " << images_paths_[i] << std::endl;
			if (!img.empty())
			{
				//重新修改相机的参数
				//std::cout << "the cam img: " << i << std::endl;
				tmp_model.fx_ = 0.5*img.cols;
				tmp_model.fy_ = 0.5*img.cols;
				tmp_model.cx_ = 0.5*img.cols;
				tmp_model.cy_ = 0.5*img.rows;
				std::cout << "the cam img cx: " << tmp_model.cx_ << std::endl;
			}
		}
		cams_poses_.emplace_back(tmp_model);
	}
}

void ModelCameras::AddCamModel(CameraModel& c_m)
{
	cams_poses_.emplace_back(c_m);
}

void ModelCameras::SetCamModel(int idx, CameraModel& c_m)
{
	//std::cerr << "start to cp to cammodel..." << std::endl;
	if (idx >= cams_poses_.size() || idx < 0)
		return;
	cams_poses_[idx] = c_m;
	//std::cerr << "end cp to cammodel..." << std::endl;
}

bool ModelCameras::SetPickedCamModelLyoutId(int idx, int lay_id)
{
	if (idx >= cams_poses_.size() || idx < 0)
	{
		return false;
	}
	cams_poses_[idx].layout_id_ = lay_id;
	return true;
}

void ModelCameras::AddCamImage(const std::string& c_img)
{
	images_paths_.emplace_back(c_img);
	cv::Mat img = cv::imread(c_img);
	offscreen_width_ = img.cols;
	offscreen_height_ = img.rows;
}

int ModelCameras::GetModelCamerasNum()
{
	return (int)cams_poses_.size();
}

void ModelCameras::GetCurrentDirFiles(const std::string & path, std::vector<std::string> & files)
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

void ModelCameras::SaveCamsModelsIntoCamFiles()
{
	if (cams_poses_.size() != images_paths_.size())
	{
		std::cerr << "cams num is not equal to images path num " << std::endl;
		return;
	}
	for (int i = 0; i < cams_poses_.size(); ++i)
	{
		//
		std::string path = images_paths_[i];
		std::string path_log = path.substr(0, path.find_last_of(".")) + ".cam";
		std::cerr << "path log: " << path_log << std::endl;
		Eigen::Matrix4f cm = cams_poses_[i].cam_pose_.inverse();	//需要转化为kTCWRCW,当时自己写反了
		std::ofstream fh(path_log);
		fh << cm(0, 3) << " " << cm(1, 3) << " " << cm(2, 3) << " "
			<< cm(0, 0) << " " << cm(0, 1) << " " << cm(0, 2) << " "
			<< cm(1, 0) << " " << cm(1, 1) << " " << cm(1, 2) << " "
			<< cm(2, 0) << " " << cm(2, 1) << " " << cm(2, 2) << std::endl;
		fh << cams_poses_[i].fx_ << " " << cams_poses_[i].fy_ << " " << cams_poses_[i].cx_ << " " << cams_poses_[i].cy_;
		fh << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0;	//这是distortion,它设置为0
	}
}

void ModelCameras::SaveCamsPoseIntoObjFile(const std::string& file_path, float aix_len)
{
	std::ofstream fh(file_path);
	std::vector<Eigen::Matrix4f> cams_pose;
	for (int i = 0; i < cams_poses_.size(); ++i)
	{
		Eigen::Matrix4f tmp_cam;
		GetPickedCamPose(i, tmp_cam);
		cams_pose.emplace_back(tmp_cam);
	}

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

	//计算它的轴
	for (int i = 0; i < cams_pose.size(); ++i)
	{
		fh << "l " << 4 * i + 1 << " " << 4 * i + 2 << std::endl;
		fh << "l " << 4 * i + 1 << " " << 4 * i + 3 << std::endl;
		fh << "l " << 4 * i + 1 << " " << 4 * i + 4 << std::endl;
	}
	fh.close();
}

void ModelCameras::SavePickedPoseIntoObjFile(int idx, const std::string& file_path, float aix_len)
{
	std::ofstream fh(file_path);
	Eigen::Matrix4f cam_pose;
	GetPickedCamPose(idx, cam_pose);
	Eigen::Matrix3f R_wc = cam_pose.topLeftCorner(3, 3);
	Eigen::Vector3f T_wc = cam_pose.topRightCorner(3, 1);

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

	fh << "l " << 1 << " " << 2 << std::endl;
	fh << "l " << 1 << " " << 3 << std::endl;
	fh << "l " << 1 << " " << 4 << std::endl;
	fh.close();
}

void ModelCameras::SaveCamPoseIntoObjFile(Eigen::Matrix4f& cam_pose, const std::string& file_path, float aix_len)
{
	std::ofstream fh(file_path);

	Eigen::Matrix3f R_wc = cam_pose.topLeftCorner(3, 3);
	Eigen::Vector3f T_wc = cam_pose.topRightCorner(3, 1);

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

	fh << "l " << 1 << " " << 2 << std::endl;
	fh << "l " << 1 << " " << 3 << std::endl;
	fh << "l " << 1 << " " << 4 << std::endl;
	fh.close();
}

bool ModelCameras::operator==(const ModelCameras& c)
{
	if (this->cams_idx_ == c.cams_idx_ &&
		this->screen_near_ == c.screen_near_ &&
		this->screen_far_ == c.screen_far_ &&
		this->offscreen_width_ == c.offscreen_width_ && 
		this->offscreen_height_ == c.offscreen_height_ &&
		//this->cams_poses_ == c.cams_poses_ &&
		this->images_paths_ == c.images_paths_ &&
		this->cams_imgs_width_height_ == c.cams_imgs_width_height_)
	{
		//return true;
		if (this->cams_poses_.size() == c.cams_poses_.size())
		{
			for (int i = 0; i < this->cams_poses_.size(); ++i)
			{
				if (!(this->cams_poses_[i] == c.cams_poses_[i]))
					return false;
			}
			return true;
		}
		else
		{
			return false;
		}
	}
	return false;
}

ModelCameras ModelCameras::operator=(const ModelCameras& c)
{
	if ((*this) == c)
		return *this;
	this->cams_idx_ = c.cams_idx_;
	this->screen_near_ = c.screen_near_;
	this->screen_far_ = c.screen_far_;
	this->offscreen_width_ = c.offscreen_width_;
	this->offscreen_height_ = c.offscreen_height_;
	this->images_paths_ = c.images_paths_;
	this->cams_poses_ = c.cams_poses_;
	this->cams_imgs_width_height_ = c.cams_imgs_width_height_;
	return *this;
}

void ModelCameras::ReadFileIgnoreFilesFromTXT(const std::string& txt_path, std::vector<std::string>& ignore_files)
{
	std::ifstream fh(txt_path);
	if (fh.is_open())
	{
		ignore_files.clear();
		while (!fh.eof())
		{
			std::string line_buf;
			std::getline(fh, line_buf);
			ignore_files.emplace_back(line_buf);
		}
		fh.close();
	}
}

bool ModelCameras::FindStrFromStrVec(const std::vector<std::string>& vec, const std::string& str)
{
	for (int i = 0; i < vec.size(); ++i)
	{
		if (vec[i] == str)
		{
			return true;
		}
	}
	return false;
}

bool ModelCameras::CompareHWStrPairByIdxCmp(const std::pair<int, std::string>& a,
	const std::pair<int, std::string>& b)
{
	return a.first < b.first;
}

void ModelCameras::GetModelCameras(std::vector<CameraModel>& cams)
{
	cams = cams_poses_;
}

const std::vector<std::string>& ModelCameras::GetImagesPaths()
{
	return images_paths_;
}

void ModelCameras::GetPickedCamModel(int idx, CameraModel& cam)
{
	if (idx > cams_poses_.size())
		return;

	//默认拷贝构造函数
	cam = cams_poses_[idx];
}

void ModelCameras::GetPickedCamPose(int idx, Eigen::Matrix4f& cam_pose)
{
	if (idx > cams_poses_.size())
		return;

	//默认拷贝构造函数
	cam_pose = cams_poses_[idx].cam_pose_;
}

int ModelCameras::GetPickedCamModelLayoutId(int cam_idx)
{
	if (cam_idx >= (int)cams_poses_.size() || cam_idx < 0)
	{
		return -1;
	}
	return cams_poses_[cam_idx].layout_id_;
}

bool ModelCameras::GetPickedCamImgWidthAndHeight(int idx, int& img_width, int& img_height)
{
	if (idx > cams_imgs_width_height_.size() || images_paths_.empty())
		return false;
	img_width = cams_imgs_width_height_[idx][0];
	img_height = cams_imgs_width_height_[idx][1];
	return true;
}

const std::vector<Eigen::Vector2i>& ModelCameras::GetAllCamsImgsWidthAndHeight()
{
	return cams_imgs_width_height_;
}

void ModelCameras::GetRenderingFarNearParams(float& screen_near, float& screen_far)
{
	//
	screen_near = screen_near_;
	screen_far = screen_far_;
}

std::string ModelCameras::GetPickedCamImagePath(int idx)
{
	if (idx > images_paths_.size() || images_paths_.empty())
		return std::string("");
	
	return images_paths_[idx];
}

void ModelCameras::SetRenderingWindowSize(int width, int height)
{
	offscreen_height_ = height;
	offscreen_width_ = width;
}

bool ModelCameras::SetRenderingWindowSizeFromInerImg()
{
	if (!images_paths_.empty() && offscreen_width_ == 0)
	{
		cv::Mat img = cv::imread(images_paths_[0]);
		offscreen_width_ = img.cols;
		offscreen_height_ = img.rows;
		return true;
	}
	if (offscreen_width_ > 0 && offscreen_height_ > 0)
	{
		return true;
	}
	return false;
}

bool ModelCameras::SetRenderingWindowSizeFromInerSelectedImg(int cam_idx)
{
	if (images_paths_.empty())
		return false;
	if (cam_idx < 0 || cam_idx >= images_paths_.size())
		return false;
	cv::Mat img = cv::imread(images_paths_[cam_idx]);
	offscreen_width_ = img.cols;
	offscreen_height_ = img.rows;
	return true;
}

void ModelCameras::SetRenderingFarNearParams(float screen_near, float screen_far)
{
	screen_near_ = screen_near;
	screen_far_ = screen_far;
}

void ModelCameras::SetPickedCamSate(int idx, bool c_valid)
{
	if (idx > cams_poses_.size())
		return;
	cams_poses_[idx].valid_ = c_valid;
}

void ModelCameras::GetRenderingWindowSize(int& width, int& height)
{
	if (!images_paths_.empty() && offscreen_width_ == 0)
	{
		std::cerr << "GetRenderingWindowSize..." << std::endl;
		cv::Mat img = cv::imread(images_paths_[0]);
		offscreen_width_ = img.cols;
		offscreen_height_ = img.rows;
	}
	width = offscreen_width_;
	height = offscreen_height_;
}

void ModelCameras::GenerateRenderProjModelView(int idx, Eigen::Matrix4f& proj, Eigen::Matrix4f& modelview)
{
	if (idx > cams_poses_.size())
	{
		std::cout << "select wrong cam idx: " << std::endl;
		return;
	}
	
	Eigen::Matrix4f cam_extr = cams_poses_[idx].cam_pose_.inverse();
	modelview = cam_extr;
	
	//std::cout << "the cams pose: \n" << cams_poses_[idx].cam_pose_ << std::endl;

	float fx = offscreen_width_ / 2, fy = offscreen_width_ / 2, 
		cx = offscreen_width_ / 2, cy = offscreen_height_ / 2;

	//y轴
	modelview(0, 1) *= -1;
	modelview(1, 1) *= -1;
	modelview(2, 1) *= -1;
	modelview(3, 1) *= -1;
	//z轴
	modelview(0, 2) *= -1;
	modelview(1, 2) *= -1;
	modelview(2, 2) *= -1;
	modelview(3, 2) *= -1;

	float screen_far = screen_far_;
	float screen_near = screen_near_;

	proj(0, 0) = 2 * fx / offscreen_width_;
	proj(0, 1) = 0.0f;
	proj(0, 2) = 0.0f;
	proj(0, 3) = 0.0f;

	proj(1, 0) = 0.0f;
	proj(1, 1) = 2 * fy / offscreen_height_;
	proj(1, 2) = 0.0f;
	proj(1, 3) = 0.0f;

	proj(2, 0) = (1.0f - 2 * cx / offscreen_width_);
	proj(2, 1) = 2 * cy / offscreen_height_ - 1.0f;
	proj(2, 2) = -(screen_far + screen_near) / (screen_far - screen_near);
	proj(2, 3) = -1.0f;

	proj(3, 0) = 0.0f;
	proj(3, 1) = 0.0f;
	proj(3, 2) = -2.0f * screen_far * screen_near / (screen_far - screen_near);
	proj(3, 3) = 0.0f;
}

