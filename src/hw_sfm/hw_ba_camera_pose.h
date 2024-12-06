#ifndef HW_BA_CAMERA_POSE_HEADER
#define HW_BA_CAMERA_POSE_HEADER

#include<vector>

#include<Eigen/Core>

#include"model_cameras.h"

/*
*note
*The camera pose is the 3*4 matrix P = K[R | t], K is 3*3 calibration
*matrix, R is a 3*3 rotation matrix and t is a 3*1 translation vector 
*it is camera extrinsic and intrinsic
* p_c = [R|t]*p_w	(it is important)
*
*     | fx  0   px |
* K = | 0   fy  py | 
      | 0   0   1  |
*/
namespace HWSFM
{
	struct HWBACameraPose
	{
	public:
		//
		HWBACameraPose(void);
		//initial R with Identity and t with zero
		void init_canonical_form(void);
		//return camera with K[R|t]
		void fill_krt_matrix(Eigen::Matrix<double, 3, 4>& result) const;
		//initializes the K matrix from focal length and principal point
		void set_k_matrix(double fx, double fy, double px, double py);
		double get_focal_x() const;
		double get_focal_y() const;
		//return the camera position
		void fill_camera_pose(Eigen::Vector3d& camera_pose) const;
		//return true if K matrix is valid

		//to do raycast polygon 3d
		void setCamModelFromHWModel(const CameraModel& cm);

		void Covert2HWModel(CameraModel& cm);

		bool is_valid() const;

		void FillTFromDouble(double* i_t);
		void FillRFromDouble(double* i_r);
		void FillKFromDouble(double* i_k);

		void ConvertT2Double(double* o_t);
		void ConvertR2Double(double* o_r);
		void ConvertK2Double(double* o_k);

	public:
		Eigen::Matrix3d K_;
		Eigen::Matrix3d R_;
		Eigen::Vector3d t_;
	};


	inline HWBACameraPose::HWBACameraPose()
		:K_(Eigen::Matrix3d::Zero())
		,R_(Eigen::Matrix3d::Zero())
		,t_(Eigen::Vector3d::Zero())
	{
	}

	inline void HWBACameraPose::init_canonical_form()
	{
		R_.Identity();
		t_ = Eigen::Vector3d::Zero();
	}


	inline void HWBACameraPose::fill_krt_matrix(Eigen::Matrix<double, 3, 4>& result) const
	{
		Eigen::Matrix<double, 3, 3> KR = K_ * R_;
		Eigen::Matrix<double, 3, 1> Kt = K_ * t_;
		result.topLeftCorner(3, 3) = KR;
		result.topRightCorner(3, 1) = Kt;	//TEST the operation
	}


	inline void HWBACameraPose::set_k_matrix(double fx, double fy, double px, double py)
	{
		K_ = Eigen::Matrix3d::Zero();
		K_(0, 0) = fx; K_(0, 2) = px;
		K_(1, 1) = fy; K_(1, 2) = py;
		K_(2, 2) = 1.0;
	}


	inline double HWBACameraPose::get_focal_x() const
	{
		return K_(0, 0);
	}


	inline double HWBACameraPose::get_focal_y() const
	{
		return K_(1, 1);
	}


	inline void HWBACameraPose::fill_camera_pose(Eigen::Vector3d& camera_pose) const
	{
		camera_pose = -R_.transpose()*t_;
	}

	inline void HWBACameraPose::setCamModelFromHWModel(const CameraModel& cm)
	{
		double fx = (double)cm.fx_;
		double fy = (double)cm.fy_;
		double cx = (double)cm.cx_;
		double cy = (double)cm.cy_;
		this->set_k_matrix(fx, fy, cx, cy);
		Eigen::Matrix4f tmp_c = cm.cam_pose_.inverse();
		R_(0, 0) = (double)tmp_c(0, 0); R_(0, 1) = (double)tmp_c(0, 1); R_(0, 2) = (double)tmp_c(0, 2);
		R_(1, 0) = (double)tmp_c(1, 0); R_(1, 1) = (double)tmp_c(1, 1); R_(1, 2) = (double)tmp_c(1, 2);
		R_(2, 0) = (double)tmp_c(2, 0); R_(2, 1) = (double)tmp_c(2, 1); R_(2, 2) = (double)tmp_c(2, 2);
		t_[0] = (double)tmp_c(0, 3); t_[1] = (double)tmp_c(1, 3); t_[2] = (double)tmp_c(2, 3);
	}

	inline void HWBACameraPose::Covert2HWModel(CameraModel& cm)
	{
		cm.fx_ = (float)K_(0, 0);
		cm.fy_ = (float)K_(1, 1);
		cm.cx_ = (float)K_(0, 2);
		cm.cy_ = (float)K_(1, 2);
		Eigen::Matrix4f tmp_c;
		tmp_c(0, 0) = (float)R_(0, 0); tmp_c(0, 1) = (float)R_(0, 1); tmp_c(0, 2) = (float)R_(0, 2);
		tmp_c(1, 0) = (float)R_(1, 0); tmp_c(1, 1) = (float)R_(1, 1); tmp_c(1, 2) = (float)R_(1, 2);
		tmp_c(2, 0) = (float)R_(2, 0); tmp_c(2, 1) = (float)R_(2, 1); tmp_c(2, 2) = (float)R_(2, 2);
		tmp_c(0, 3) = (float)t_[0]; tmp_c(1, 3) = (float)t_[1]; tmp_c(2, 3) = (float)t_[2];
		tmp_c(3, 3) = 1.0f;
		cm.cam_pose_ = tmp_c.inverse();
	}

	inline bool HWBACameraPose::is_valid() const
	{
		return this->K_(0, 0) != 0.0;
	}

	inline void HWBACameraPose::FillTFromDouble(double* i_t)
	{
		for (int i = 0; i < 3; ++i)
		{
			t_[i] = *i_t;
			++i_t;
		}
	}

	inline void HWBACameraPose::FillRFromDouble(double* i_r)
	{
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				R_(i, j) = *i_r;
				++i_r;
			}
		}
	}

	inline void HWBACameraPose::FillKFromDouble(double* i_k)
	{
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				K_(i, j) = *i_k;
				++i_k;
			}
		}
	}

	inline void HWBACameraPose::ConvertT2Double(double* o_t)
	{
		for (int i = 0; i < 3; ++i)
		{
			*o_t = t_[i];
			++o_t;
		}
	}

	inline void HWBACameraPose::ConvertR2Double(double* o_r)
	{
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				*o_r = R_(i, j);
				++o_r;
			}
		}
	}

	inline void HWBACameraPose::ConvertK2Double(double* o_k)
	{
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				*o_k = K_(i, j);
				++o_k;
			}
		}
	}
}


#endif // !BA_CAMERA_POSE_HEADER

