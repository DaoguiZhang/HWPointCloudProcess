#pragma once
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<vector>
#include<fstream>
#include<iostream>
#include<sstream>
#include<string>

enum CamFileType
{
	kTCWRWC,
	kTWCRCW,	//混合体
	kTWCRWC,	//相机位姿
	kTCWRCW		//相机的外参
};

class CameraModel
{
public:

	std::string path_;
	int cam_id_;
	int image_id_;
	int layout_id_;
	float fx_, fy_, cx_, cy_;
	std::vector<float> distortion_;
	Eigen::Matrix4f cam_pose_;	//T_wc
	bool valid_ = false;

public:
	CameraModel()
	{
		this->cam_id_ = -1;
		this->image_id_ = -1;
		this->layout_id_ = -1;
		this->fx_ = 0.0;
		this->fy_ = 0.0;
		this->cx_ = 0.0;
		this->cy_ = 0.0;
		//this->distortion_;
		//this->cam_pose_ = other.cam_pose_;
		this->valid_ = false;
	}

	CameraModel(CameraModel& other)
	{
		this->path_ = other.path_;
		this->cam_id_ = other.cam_id_;
		this->image_id_ = other.image_id_;
		this->layout_id_ = other.layout_id_;
		this->fx_ = other.fx_;
		this->fy_ = other.fy_;
		this->cx_ = other.cx_;
		this->cy_ = other.cy_;
		this->distortion_ = other.distortion_;
		this->cam_pose_ = other.cam_pose_;
		this->valid_ = other.valid_;
	}

	CameraModel(const CameraModel& other)
	{
		this->path_ = other.path_;
		this->cam_id_ = other.cam_id_;
		this->image_id_ = other.image_id_;
		this->layout_id_ = other.layout_id_;
		this->fx_ = other.fx_;
		this->fy_ = other.fy_;
		this->cx_ = other.cx_;
		this->cy_ = other.cy_;
		this->distortion_ = other.distortion_;
		this->cam_pose_ = other.cam_pose_;
		this->valid_ = other.valid_;
	}

	~CameraModel()
	{
	}

	void LoadCamFromCam(const std::string path, CamFileType camtype = kTCWRCW)
	{
		std::ifstream fh(path);
		if (!fh.eof())
		{
			std::string line;
			std::getline(fh, line);
			std::stringstream ss(line);

			if (camtype == kTCWRCW)
			{
				Eigen::Matrix4f cam_extr;
				ss >> cam_extr(0, 3) >> cam_extr(1, 3) >> cam_extr(2, 3)
					>> cam_extr(0, 0) >> cam_extr(0, 1) >> cam_extr(0, 2)
					>> cam_extr(1, 0) >> cam_extr(1, 1) >> cam_extr(1, 2)
					>> cam_extr(2, 0) >> cam_extr(2, 1) >> cam_extr(2, 2);
				cam_extr(3, 0) = 0;
				cam_extr(3, 1) = 0;
				cam_extr(3, 2) = 0;
				cam_extr(3, 3) = 1;
				//std::cout << "the cam extr: \n" << cam_extr << std::endl;
				cam_pose_ = cam_extr.inverse();
			}
			else if (camtype == kTWCRWC)
			{
				ss >> cam_pose_(0, 3) >> cam_pose_(1, 3) >> cam_pose_(2, 3)
					>> cam_pose_(0, 0) >> cam_pose_(0, 1) >> cam_pose_(0, 2)
					>> cam_pose_(1, 0) >> cam_pose_(1, 1) >> cam_pose_(1, 2)
					>> cam_pose_(2, 0) >> cam_pose_(2, 1) >> cam_pose_(2, 2);
				cam_pose_(3, 0) = 0;
				cam_pose_(3, 1) = 0;
				cam_pose_(3, 2) = 0;
				cam_pose_(3, 3) = 1;
			}
			else if (camtype == kTWCRCW)
			{
				Eigen::Vector3f Twc;
				fh >> Twc[0] >> Twc[1] >> Twc[2];
				Eigen::Matrix3f Rcw;
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						fh >> Rcw(i, j);
					}
				}
				Eigen::Matrix3f Rwc = Rcw.transpose();
				cam_pose_.topLeftCorner(3, 3) = Rwc;
				cam_pose_.topRightCorner(3, 1) = Twc;
			}
			else if (camtype == kTCWRWC)
			{
				Eigen::Vector3f Tcw;
				fh >> Tcw[0] >> Tcw[1] >> Tcw[2];
				Eigen::Matrix3f Rwc;
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						fh >> Rwc(i, j);
					}
				}

				Eigen::Vector3f Twc = -Rwc * Tcw;

				for (int i = 0; i < 3; i++) {
					cam_pose_(i, 3) = Twc[i];
				}
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						cam_pose_(i, j) = Rwc(i, j);
					}
				}
				cam_pose_(3, 0) = 0;
				cam_pose_(3, 1) = 0;
				cam_pose_(3, 2) = 0;
				cam_pose_(3, 3) = 1;
			}
			
			std::getline(fh, line);
			if (line.size() > 3)
			{
				std::stringstream ss1(line);
				//test
				//std::cout << "the line is: " << line << std::endl;
				//std::cout << ss1.str() << std::endl;
				ss1 >> fx_ >> fy_ >> cx_ >> cy_;
			}
			if (line.size() <= 3)
			{
				std::stringstream ss1(line);
				ss1 >> fx_;
				fy_ = fx_;
				cx_ = 0.0;
				cy_ = 0.0;
				//std::cout << "the cam121112211: " << cx_ << std::endl;
			}
			//ignore distortion, do its later!!!
		}
		fh.close();
		path_ = path;
	}

	CameraModel operator=(const CameraModel& c)
	{
		this->path_ = c.path_;
		this->cam_id_ = c.cam_id_;
		this->image_id_ = c.image_id_;
		this->layout_id_ = c.layout_id_;
		this->fx_ = c.fx_;
		this->fy_ = c.fy_;
		this->cx_ = c.cx_;
		this->cy_ = c.cy_;
		this->distortion_ = c.distortion_;
		this->cam_pose_ = c.cam_pose_;
		this->valid_ = c.valid_;
		return *this;
	}

	bool operator==(const CameraModel& c)
	{
		if (this->path_ == c.path_ &&
			this->cam_id_ == c.cam_id_ &&
			this->image_id_ == c.image_id_ &&
			this->layout_id_ == c.layout_id_ &&
			this->fx_ == c.fx_&&
			this->fy_ == c.fy_&&
			this->cx_ == c.cx_&&
			this->cy_ == c.cy_&&
			this->distortion_ == c.distortion_&&
			this->cam_pose_ == c.cam_pose_&&
			this->valid_ == c.valid_)
		{
			return true;
		}
		return false;
	}

	//get k
	Eigen::Matrix3f GetKMatrix()
	{
		Eigen::Matrix3f m = Eigen::Matrix3f::Identity();
		m(0, 0) = fx_;
		m(1, 1) = fy_;
		m(2, 2) = 1.0;
		m(0, 2) = cx_;
		m(1, 2) = cy_;
		return m;
	}

	//get k
	const Eigen::Matrix3f GetKMatrix() const
	{
		Eigen::Matrix3f m = Eigen::Matrix3f::Identity();
		m(0, 0) = fx_;
		m(1, 1) = fy_;
		m(2, 2) = 1.0;
		m(0, 2) = cx_;
		m(1, 2) = cy_;
		return m;
	}

	//cam pose = (0,0,0) and z = 1, then raypnt = ReinvKinv*p; otherwise raypnt + t is true
	Eigen::Vector3f GetRayNormalized(const Eigen::Vector3f& p)
	{
		Eigen::Matrix3f K = GetKMatrix();
		Eigen::Matrix3f kinv = K.inverse();

		//Eigen::Matrix3d R_extr = cam_pose_.topLeftCorner(3,3).transpose();
		Eigen::Matrix3f R_extr_inv = cam_pose_.topLeftCorner(3, 3);
		Eigen::Matrix3f ReinvKinv = R_extr_inv*kinv;

		Eigen::Vector3f raypnt = ReinvKinv*p;
		return raypnt.normalized();
	}

	//cam pose = (0,0,0) and z = 1, then raypnt = ReinvKinv*p; otherwise raypnt + t is true
	const Eigen::Vector3f GetRayNormalized(const Eigen::Vector3f& p) const
	{
		Eigen::Matrix3f K = GetKMatrix();
		Eigen::Matrix3f kinv = K.inverse();

		//Eigen::Matrix3d R_extr = cam_pose_.topLeftCorner(3,3).transpose();
		Eigen::Matrix3f R_extr_inv = cam_pose_.topLeftCorner(3, 3);
		Eigen::Matrix3f ReinvKinv = R_extr_inv*kinv;

		Eigen::Vector3f raypnt = ReinvKinv*p;
		return raypnt.normalized();
	}


	//get cam pose c
	Eigen::Vector3f GetCamC()
	{
		Eigen::Vector3f c = cam_pose_.topRightCorner(3, 1);
		return c;
	}

	//get cam pose c
	const Eigen::Vector3f GetCamC() const
	{
		Eigen::Vector3f c = cam_pose_.topRightCorner(3, 1);
		return c;
	}

	Eigen::Vector2f WorldPnt3d2ImgPnt(const Eigen::Vector3f& p)
	{
		Eigen::Vector4f poff(p[0], p[1], p[2], 1.0);
		Eigen::Vector4f camp = cam_pose_.inverse()*poff;
		float z = camp[2];
		float u = fx_*camp[0] / z + cx_;
		float v = fy_*camp[1] / z + cy_;

		return Eigen::Vector2f(u, v);
	}

	//img_p[0]: u0; img_p[1]:u1
	Eigen::Vector3f DepthPnt2CamPnt(const Eigen::Vector3f& img_p)
	{
		float x_c = img_p[2] * (img_p[0] - cx_) / fx_;
		float y_c = img_p[2] * (img_p[1] - cy_) / fy_;
		float z_c = img_p[2];
		return Eigen::Vector3f(x_c, y_c, z_c);
	}

	//
	Eigen::Vector3f DepthPnt2WorldPnt(const Eigen::Vector3f& img_p)
	{
		Eigen::Vector3f pcam = DepthPnt2CamPnt(img_p);
		Eigen::Vector4f pcam_offine(pcam[0], pcam[1], pcam[2], 1.0);
		Eigen::Vector4f pworld = cam_pose_*pcam_offine;
		return Eigen::Vector3f(pworld[0], pworld[1], pworld[2]);
	}

	Eigen::Vector3f CamPnt2WorldPnt(const Eigen::Vector3f& cp)
	{
		Eigen::Vector4f pcam_offine(cp[0], cp[1], cp[2], 1.0);
		Eigen::Vector4f pworld = cam_pose_*pcam_offine;
		return Eigen::Vector3f(pworld[0], pworld[1], pworld[2]);
	}

	void SaveMeOnlyIntoObj(const std::string& path, float aix_len)
	{
		//GetPickedCamPose(idx, cam_pose);
		std::ofstream fh(path, std::ios::out);
		if (!fh.is_open())
		{
			std::cerr << "the path: " << path << " open failed..." << std::endl;
			return;
		}
		Eigen::Matrix3f R_wc = cam_pose_.topLeftCorner(3, 3);
		Eigen::Vector3f T_wc = cam_pose_.topRightCorner(3, 1);

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

	void SaveMeCamPoseParamIntoCam()
	{
		std::ofstream fh(path_);
		fh << fx_ << " " << fy_ << " " << cx_ << " " << cy_ << std::endl;
		fh << cam_pose_(0, 0) << " " << cam_pose_(0, 1) << " " << cam_pose_(0, 2) << " " << cam_pose_(0, 3) << std::endl;
		fh << cam_pose_(1, 0) << " " << cam_pose_(1, 1) << " " << cam_pose_(1, 2) << " " << cam_pose_(1, 3) << std::endl;
		fh << cam_pose_(2, 0) << " " << cam_pose_(2, 1) << " " << cam_pose_(2, 2) << " " << cam_pose_(2, 3) << std::endl;
		fh << cam_pose_(3, 0) << " " << cam_pose_(3, 1) << " " << cam_pose_(3, 2) << " " << cam_pose_(3, 3) << std::endl;
		fh.close();
	}

	void SaveMeCamExtrParamIntoCam()
	{
		std::ofstream fh(path_);
		fh << fx_ << " " << fy_ << " " << cx_ << " " << cy_ << std::endl;
		Eigen::Matrix4f cam_extr = cam_pose_.inverse();
		fh << cam_extr(0, 0) << " " << cam_extr(0, 1) << " " << cam_extr(0, 2) << " " << cam_extr(0, 3) << std::endl;
		fh << cam_extr(1, 0) << " " << cam_extr(1, 1) << " " << cam_extr(1, 2) << " " << cam_extr(1, 3) << std::endl;
		fh << cam_extr(2, 0) << " " << cam_extr(2, 1) << " " << cam_extr(2, 2) << " " << cam_extr(2, 3) << std::endl;
		fh << cam_extr(3, 0) << " " << cam_extr(3, 1) << " " << cam_extr(3, 2) << " " << cam_extr(3, 3) << std::endl;
		fh.close();
	}

	void GenerateRenderProjModelMatrix(float offscreen_width_, float offscreen_height_, 
		float screen_near, float screen_far,  Eigen::Matrix4f& proj_view)
	{
		proj_view(0, 0) = 2 * fx_ / offscreen_width_;
		proj_view(0, 1) = 0.0f;
		proj_view(0, 2) = 0.0f;
		proj_view(0, 3) = 0.0f;

		proj_view(1, 0) = 0.0f;
		proj_view(1, 1) = 2 * fy_ / offscreen_height_;
		proj_view(1, 2) = 0.0f;
		proj_view(1, 3) = 0.0f;

		proj_view(2, 0) = (1.0f - 2 * cx_ / offscreen_width_);
		proj_view(2, 1) = 2 * cy_ / offscreen_height_ - 1.0f;
		proj_view(2, 2) = -(screen_far + screen_near) / (screen_far - screen_near);
		proj_view(2, 3) = -1.0f;

		proj_view(3, 0) = 0.0f;
		proj_view(3, 1) = 0.0f;
		proj_view(3, 2) = -2.0f * screen_far * screen_near / (screen_far - screen_near);
		proj_view(3, 3) = 0.0f;
	}

};



class ModelCameras {

public:

	ModelCameras();
	~ModelCameras();

	//从realitycaputure读取图片等等
	void LoadImagesFromRealityCaputureImgsdir(const std::string& img_dir);

	//从相机HW的相机的格式读取相机位姿和图片
	void LoadCamsFromCAMDIR(const std::string& dir);
	void LoadCamsFromCAMDIROld(const std::string& dir);
	void AddCamModel(CameraModel& c_m);
	void SetCamModel(int idx, CameraModel& c_m);
	bool SetPickedCamModelLyoutId(int idx, int lay_id);
	void AddCamImage(const std::string& c_img);

	int GetModelCamerasNum();
	void GetModelCameras(std::vector<CameraModel>& cams);
	const std::vector<std::string>& GetImagesPaths();
	void GetPickedCamModel(int idx, CameraModel& cam);
	void GetPickedCamPose(int idx, Eigen::Matrix4f& cam_pose);
	int GetPickedCamModelLayoutId(int cam_idx);	//get layout id
	bool GetPickedCamImgWidthAndHeight(int idx, int& img_width, int& img_height);
	const std::vector<Eigen::Vector2i>& GetAllCamsImgsWidthAndHeight();
	void GetRenderingFarNearParams(float& screen_near, float& screen_far);
	std::string GetPickedCamImagePath(int idx);

	void SetRenderingWindowSize(int width, int height);
	bool SetRenderingWindowSizeFromInerImg();
	bool SetRenderingWindowSizeFromInerSelectedImg(int cam_idx);
	void SetRenderingFarNearParams(float screen_near, float screen_far);
	void SetPickedCamSate(int idx, bool c_valid);
	void GetRenderingWindowSize(int& width, int& height);
	void GenerateRenderProjModelView(int idx, Eigen::Matrix4f& proj, Eigen::Matrix4f& modelview);
	void GetCurrentDirFiles(const std::string & path, std::vector<std::string> & files);

	void SaveCamsModelsIntoCamFiles();

	void SaveCamsPoseIntoObjFile(const std::string& file_path, float aix_len);

	void SavePickedPoseIntoObjFile(int idx, const std::string& file_path, float aix_len);

	void SaveCamPoseIntoObjFile(Eigen::Matrix4f& cam_pose, const std::string& file_path, float aix_len);

	ModelCameras operator=(const ModelCameras& c);

	bool operator==(const ModelCameras& c);

	std::vector<std::vector<int> > cams_idx_;

private:

	void ReadFileIgnoreFilesFromTXT(const std::string& txt_path, std::vector<std::string>& ignore_files);
	bool FindStrFromStrVec(const std::vector<std::string>& vec, const std::string& str);
	static bool CompareHWStrPairByIdxCmp(const std::pair<int, std::string>& a, 
		const std::pair<int, std::string>& b);

	float screen_near_, screen_far_;
	int offscreen_width_, offscreen_height_;
	std::vector<std::string> images_paths_;
	std::vector<CameraModel> cams_poses_;
	std::vector<Eigen::Vector2i> cams_imgs_width_height_;	//对应的图像的长宽
};
