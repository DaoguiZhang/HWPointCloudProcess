#include"model_cameras.h"
#include<opencv2/opencv.hpp>

namespace HW
{
	//added by zdg
	struct BundlerPoint2d
	{
		int cam_idx;
		int feature_idx;
		float x;
		float y;
	};

	struct BundlerPoint3d
	{
		Eigen::Vector3f pnt3d_pos;
		Eigen::Vector3i pnt3d_color;
		int views_list_num;
		std::vector<BundlerPoint2d> views_list;
	};

	class HWBundle : public ModelCameras
	{
	public:

		HWBundle();

		~HWBundle();
		
		void SetDir(std::string& b_dir);

		void LoadDataFromRealityCaptureDataDir();

		void LoadDataFromRealityCaptureData(const std::string& bundle_dir);

		void ParseBundleFileFromRealityCapture(const std::string& bundle_path);

		void LoadAlignMatrixFromCloudCampair(const std::string& cl_path);

		void SetAlignMatrix(Eigen::Matrix4f& al_m);

		void ConvertPnts2AlignCamsAndAlignPnts();

		void SaveAlignedCamsIntoLogFiles();

		void GetBundleAllCamsModels(std::vector<CameraModel>& cams_model);

		void GetBundleAllCamsPoses(std::vector<Eigen::Matrix4f>& cams_poses);

		bool GetBundlePickedCamsModel(int idx, CameraModel& cam_model);

		bool GetBundlePickedCamsPos(int idx, Eigen::Matrix4f& cam_model);

		//转化为正常的相机位姿和其点云
		bool ConvertBundleData2CamsAndPnts3d();

		//测试顶点反投到图片上，用于验证自己写的bundle接口是否正确
		void ProjectBundlePnts3d2PickedImg(cv::Mat& c_img, int idx);

		//保存有bundle的顶点到图片上顶点
		void SaveBundlePnts3dAndAllImgs();

		//原始的点云反投到图片上，验证写的bundle点云转化正常的slam的结果是否正确
		void ProjectPnts3d2PickedImg(cv::Mat& c_img, int idx);

		//原始的点云反投到注册的图像上，验证注册后的接口是否正确
		void ProjectAlignPnts3d2PickedImg(cv::Mat& c_img, int idx);

		//保存bundle的3D点云
		void SaveOriginBundlePnts3dIntoObj();

		//保存有bundle的顶点到图片上顶点
		void SaveOriginPnts3dAndAllImgs();

		//保存注册好的顶点到图片上顶点
		void SaveAlignedPnts3dAndAllImgs();

		//保存转化好的点云文件，查看它是否和稠密点云注册在一起
		void SaveOriginPnts3dIntoObj();

		//保存转化好的点云文件，查看它是否和华为的polygon注册在一起
		void SaveAlignedPnts3dIntoObj();

		//保存相机位姿，可视化出来
		void SaveAlignedCamsPoseIntoObj();

	private:

		void SplitStr(std::vector<std::string>& split_strs, std::string const& str, char delim = ' ', bool keep_empty = false);

		bool has_aligned_;

		std::string b_dir_;

		//三维点云的
		std::vector<BundlerPoint3d> bundle_pnts_3d_;
		//相机参数
		ModelCameras* bundle_cams_;

		//用于保存转化为正常的点云和相机位姿
		std::vector<Eigen::Vector3f> pnts_3d_;
		ModelCameras* cams_;

		//相机外参的刚体转化矩阵
		Eigen::Matrix4f rigid_matrix_;
		
		//注册到华为点云的点云和相机位姿
		//旋转后的点云位置
		std::vector<Eigen::Vector3f> align_pnts_3d_;
		//旋转后的相机位姿
		ModelCameras* align_cams_;
	};
}

