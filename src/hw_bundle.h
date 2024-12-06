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

		//ת��Ϊ���������λ�˺������
		bool ConvertBundleData2CamsAndPnts3d();

		//���Զ��㷴Ͷ��ͼƬ�ϣ�������֤�Լ�д��bundle�ӿ��Ƿ���ȷ
		void ProjectBundlePnts3d2PickedImg(cv::Mat& c_img, int idx);

		//������bundle�Ķ��㵽ͼƬ�϶���
		void SaveBundlePnts3dAndAllImgs();

		//ԭʼ�ĵ��Ʒ�Ͷ��ͼƬ�ϣ���֤д��bundle����ת��������slam�Ľ���Ƿ���ȷ
		void ProjectPnts3d2PickedImg(cv::Mat& c_img, int idx);

		//ԭʼ�ĵ��Ʒ�Ͷ��ע���ͼ���ϣ���֤ע���Ľӿ��Ƿ���ȷ
		void ProjectAlignPnts3d2PickedImg(cv::Mat& c_img, int idx);

		//����bundle��3D����
		void SaveOriginBundlePnts3dIntoObj();

		//������bundle�Ķ��㵽ͼƬ�϶���
		void SaveOriginPnts3dAndAllImgs();

		//����ע��õĶ��㵽ͼƬ�϶���
		void SaveAlignedPnts3dAndAllImgs();

		//����ת���õĵ����ļ����鿴���Ƿ�ͳ��ܵ���ע����һ��
		void SaveOriginPnts3dIntoObj();

		//����ת���õĵ����ļ����鿴���Ƿ�ͻ�Ϊ��polygonע����һ��
		void SaveAlignedPnts3dIntoObj();

		//�������λ�ˣ����ӻ�����
		void SaveAlignedCamsPoseIntoObj();

	private:

		void SplitStr(std::vector<std::string>& split_strs, std::string const& str, char delim = ' ', bool keep_empty = false);

		bool has_aligned_;

		std::string b_dir_;

		//��ά���Ƶ�
		std::vector<BundlerPoint3d> bundle_pnts_3d_;
		//�������
		ModelCameras* bundle_cams_;

		//���ڱ���ת��Ϊ�����ĵ��ƺ����λ��
		std::vector<Eigen::Vector3f> pnts_3d_;
		ModelCameras* cams_;

		//�����εĸ���ת������
		Eigen::Matrix4f rigid_matrix_;
		
		//ע�ᵽ��Ϊ���Ƶĵ��ƺ����λ��
		//��ת��ĵ���λ��
		std::vector<Eigen::Vector3f> align_pnts_3d_;
		//��ת������λ��
		ModelCameras* align_cams_;
	};
}

