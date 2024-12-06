#include"hw_scene_bundle_adjustment.h"
#include "hw_cmns.h"
#include"hw_view_cmns.h"

namespace HW
{
	namespace BA
	{

#define UsePluckerLineMethod  0	

		void print_cam_test_r_info(double* r)
		{
			std::cerr << "rotation: \n" << r[0] << " " << r[1] << " " << r[2] << std::endl
				<< r[3] << " " << r[4] << " " << r[5] << std::endl
				<< r[6] << " " << r[7] << " " << r[8] << std::endl;
		}

		//class PoseGraphcostFunc : public ceres::SizedCostFunction<2, 9>
		//{
		//public:
		//	PoseGraphcostFunc(double ob_x, double ob_y)
		//		:observered_x_(ob_x), observered_y_(ob_y)
		//	{
		//		K_ = Eigen::Matrix3d::Identity();
		//	}
		//	void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
		//	{
		//		K_ = K;
		//	}
		//	virtual ~PoseGraphcostFunc() {};
		//	virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
		//	{
		//		/*
		//		matrix->(1,9)
		//		parameters: (w[0],w[1],w[2],t[0],t[1],t[2],X[0],X[1],X[2])
		//		camera parameters: (w[0], w[1], w[2], t[0], t[1], t[2])
		//		p3d parameters: (X[0], X[1], X[2])
		//		*/
		//		double w[3];
		//		double w_rotation[9];
		//		w[0] = parameters[0][0];
		//		w[1] = parameters[0][1];
		//		w[2] = parameters[0][2];
		//		HW::rodrigues_to_matrix3d(w, w_rotation);
		//		double t[3];
		//		t[0] = parameters[0][3];
		//		t[1] = parameters[0][4];
		//		t[2] = parameters[0][5];
		//		double p3d[3];
		//		p3d[0] = parameters[0][6];
		//		p3d[1] = parameters[0][7];
		//		p3d[2] = parameters[0][8];
		//		double const rx = w_rotation[0] * p3d[0] + w_rotation[1] * p3d[1] + w_rotation[2] * p3d[2];	//row major
		//		double const ry = w_rotation[3] * p3d[0] + w_rotation[4] * p3d[1] + w_rotation[5] * p3d[2];	//row major
		//		double const rz = w_rotation[6] * p3d[0] + w_rotation[7] * p3d[1] + w_rotation[8] * p3d[2];	//row major
		//		double const xc = rx + t[0];	//in camera coordinate
		//		double const yc = ry + t[1];	//in camera coordinate
		//		double const zc = rz + t[2];	//in camera coordinate
		//		double const fx = K_(0, 0);		//fx
		//		double const fy = K_(1, 1);		//fy
		//		double const cx = K_(0, 2);		//cx
		//		double const cy = K_(1, 2);		//cy
		//		double const iu = fx * xc / zc + cx;	//image pixel
		//		double const iv = fy * yc / zc + cy;	//image pixel
		//		residuals[0] = observered_x_ - iu;
		//		residuals[1] = observered_y_ - iv;
		//		/*
		//		const double y = parameters[1][0];
		//		compute the reprojection error
		//		std::cerr << "w1,w2,w3: " << w1 << ", " << w2 << ", " << w3 << std::endl;
		//		std::cerr << "x,y,z: " << x << ", " << y << ", " << z << std::endl;
		//		residuals[0] = observered_x_ - w0 * 2 - w1 * 3 - w2 * 4 - t0 * 2 - t1 * 2 - t2 * 2 - x * 2 - y * 2 - z * 2;
		//		residuals[1] = observered_y_ - w0 * 4 - w1 * 3 - w2 * 2 - t0 * 2 - t1 * 2 - t2 * 2 - x * 2 - y * 2 - z * 2;
		//		*/
		//		if (jacobians != NULL && jacobians[0] != NULL)
		//		{
		//			Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J(jacobians[0]);
		//			//compute the jacobians
		//			//rx = r_0P; ry = r_1P; rz = r_2P
		//			J(0, 0) = -fx*xc*ry / (zc*zc);
		//			J(0, 1) = fx*rz / zc + (fx*xc*rx) / (zc*zc);
		//			J(0, 2) = -fx*ry / zc;
		//			J(0, 3) = fx / zc;
		//			J(0, 4) = 0.0;
		//			J(0, 5) = -fx*xc / (zc*zc);
		//			J(0, 6) = fx*w[0] / zc - fx*xc*w[6] / (zc*zc);
		//			J(0, 7) = fx*w[1] / zc - fx*xc*w[7] / (zc*zc);
		//			J(0, 8) = fx*w[2] / zc - fx*xc*w[8] / (zc*zc);
		//			J(1, 0) = -fy*rz / zc - (fy*yc*ry) / (zc*zc);
		//			J(1, 1) = fy*yc*rx / (zc*zc);
		//			J(1, 2) = fy*rx / zc;
		//			J(1, 3) = 0;
		//			J(1, 4) = fy / zc;
		//			J(1, 5) = -fy*yc / (zc*zc);
		//			J(1, 6) = fy*w[3] / zc - fy*yc*w[6] / (zc*zc);
		//			J(1, 7) = fy*w[4] / zc - fy*yc*w[7] / (zc*zc);
		//			J(1, 8) = fy*w[5] / zc - fy*yc*w[8] / (zc*zc);
		//			//jacobians[0][0] = -2 * y;
		//			//jacobians[0][1] = -x;
		//		}
		//		/*if (jacobians != NULL && jacobians[1] != NULL)
		//		{
		//		jacobians[1][0] = -y;
		//		jacobians[1][1] = -x;
		//		}*/
		//		return true;
		//	}
		//	Eigen::Matrix3d K_;
		//	double observered_x_;
		//	double observered_y_;
		//};

		//optimize camera pose and point 3d
		class PosePntcostFunc : public ceres::SizedCostFunction<2, 6, 3>
		{
		public:
			PosePntcostFunc(const Eigen::Vector2d& ob)
			{
				observation_[0] = ob[0];
				observation_[1] = ob[1];
				K_ = Eigen::Matrix3d::Identity();
				energy_scale_ = 1.0;
			}
			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}
			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}
			virtual ~PosePntcostFunc() {};
			//compute the jacobian
			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				parameters 1 -> (1,3)
				parameter 1: p3d parameters(x[0],x[1],x[2])
				*/
				double w[3];
				double wr[9];
				w[0] = parameters[0][0];
				w[1] = parameters[0][1];
				w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(w, wr);
				double t[3];
				t[0] = parameters[0][3];
				t[1] = parameters[0][4];
				t[2] = parameters[0][5];
				double p3d[3];
				p3d[0] = parameters[1][0];
				p3d[1] = parameters[1][1];
				p3d[2] = parameters[1][2];
				double const rx = wr[0] * p3d[0] + wr[1] * p3d[1] + wr[2] * p3d[2];	//row major
				double const ry = wr[3] * p3d[0] + wr[4] * p3d[1] + wr[5] * p3d[2];	//row major
				double const rz = wr[6] * p3d[0] + wr[7] * p3d[1] + wr[8] * p3d[2];	//row major
				double const xc = rx + t[0];	//in camera coordinate
				double const yc = ry + t[1];	//in camera coordinate
				double const zc = rz + t[2];	//in camera coordinate
				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy
				//print_cam_test_r_info(wr);
				//std::cerr << "fx, fy, cx, cy: " << fx << ", " << fy << ", " << cx << ", " << cy << std::endl;
				double const iu = fx * xc / zc + cx;	//image pixel
				double const iv = fy * yc / zc + cy;	//image pixel
				/*std::cerr << "observation_[0]->(iu), observation_[1]->(iv): " << observation_[0] << "->(" << iu << " )" << ", " 
					<< observation_[1] << "->(" << iv << " )" << std::endl;*/
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				residuals[0] = (-observation_[0] + iu) * energy_scale_sqrt;
				residuals[1] = (-observation_[1] + iv) * energy_scale_sqrt;
				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J0(jacobians[0]);
					//compute the jacobians
					//rx = r_0P; ry = r_1P; rz = r_2P
					J0(0, 0) = (-fx*xc*ry / (zc*zc)) * energy_scale_sqrt;
					J0(0, 1) = (fx*rz / zc + (fx*xc*rx) / (zc*zc)) * energy_scale_sqrt;
					J0(0, 2) = (-fx*ry / zc) * energy_scale_sqrt;
					J0(0, 3) = (fx / zc) * energy_scale_sqrt;
					J0(0, 4) = 0.0 * energy_scale_sqrt;
					J0(0, 5) = (-fx*xc / (zc*zc)) * energy_scale_sqrt;

					J0(1, 0) = (-fy*rz / zc - (fy*yc*ry) / (zc*zc)) * energy_scale_sqrt;
					J0(1, 1) = (fy*yc*rx / (zc*zc)) * energy_scale_sqrt;
					J0(1, 2) = (fy*rx / zc)* energy_scale_sqrt;
					J0(1, 3) = 0.0 * energy_scale_sqrt;
					J0(1, 4) = (fy / zc) * energy_scale_sqrt;
					J0(1, 5) = (-fy*yc / (zc*zc)) * energy_scale_sqrt;
				}
				if (jacobians != NULL && jacobians[1] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J1(jacobians[1]);
					J1(0, 0) = (fx*wr[0] / zc - fx*xc*wr[6] / (zc*zc)) * energy_scale_sqrt;
					J1(0, 1) = (fx*wr[1] / zc - fx*xc*wr[7] / (zc*zc)) * energy_scale_sqrt;
					J1(0, 2) = (fx*wr[2] / zc - fx*xc*wr[8] / (zc*zc)) * energy_scale_sqrt;
					J1(1, 0) = (fy*wr[3] / zc - fy*yc*wr[6] / (zc*zc)) * energy_scale_sqrt;
					J1(1, 1) = (fy*wr[4] / zc - fy*yc*wr[7] / (zc*zc)) * energy_scale_sqrt;
					J1(1, 2) = (fy*wr[5] / zc - fy*yc*wr[8] / (zc*zc)) * energy_scale_sqrt;
				}

				return true;
			}
		private:
			double energy_scale_;
			Eigen::Matrix3d K_;
			Eigen::Vector2d observation_;
		};

#if 0
		//optimize camera pose only. fix point 3d, cost: e = (u,v) - project(T_cw * pnt3d_w_)
		class CamPoseOnlyFromPntProjCostFunc : public ceres::SizedCostFunction<2, 6>
		{
		public:
			CamPoseOnlyFromPntProjCostFunc(const Eigen::Vector2d& ob)
			{
				observation_[0] = ob[0];
				observation_[1] = ob[1];
				pnt3d_ = Eigen::Vector3d::Zero();
				K_ = Eigen::Matrix3d::Identity();
				energy_scale_ = 1.0;
			}
			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}
			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}
			void SetPoint3dFromPnt(const Eigen::Vector3d& p3d)
			{
				pnt3d_ = p3d;
			}
			void SetPlaneInfo(const Eigen::Vector4d& planeinfo)
			{
				plane_info_ = planeinfo;
			}
			virtual ~CamPoseOnlyFromPntProjCostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				*/
				double w[3];
				double wr[9];
				w[0] = parameters[0][0];
				w[1] = parameters[0][1];
				w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(w, wr);
				double t[3];
				t[0] = parameters[0][3];
				t[1] = parameters[0][4];
				t[2] = parameters[0][5];
				double p3d[3];
				p3d[0] = pnt3d_[0];
				p3d[1] = pnt3d_[1];
				p3d[2] = pnt3d_[2];
				double const rx = wr[0] * p3d[0] + wr[1] * p3d[1] + wr[2] * p3d[2];	//row major
				double const ry = wr[3] * p3d[0] + wr[4] * p3d[1] + wr[5] * p3d[2];	//row major
				double const rz = wr[6] * p3d[0] + wr[7] * p3d[1] + wr[8] * p3d[2];	//row major
				double const xc = rx + t[0];	//in camera coordinate
				double const yc = ry + t[1];	//in camera coordinate
				double const zc = rz + t[2];	//in camera coordinate
				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy
												//print_cam_test_r_info(wr);
												//std::cerr << "fx, fy, cx, cy: " << fx << ", " << fy << ", " << cx << ", " << cy << std::endl;
				double const iu = fx * xc / zc + cx;	//image pixel
				double const iv = fy * yc / zc + cy;	//image pixel
														/*std::cerr << "observation_[0]->(iu), observation_[1]->(iv): " << observation_[0] << "->(" << iu << " )" << ", "
														<< observation_[1] << "->(" << iv << " )" << std::endl;*/
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				residuals[0] = (-observation_[0] + iu) * energy_scale_sqrt;
				residuals[1] = (-observation_[1] + iv) * energy_scale_sqrt;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J0(jacobians[0]);
					//compute the jacobians
					//rx = r_0P; ry = r_1P; rz = r_2P
					J0(0, 0) = (-fx*xc*ry / (zc*zc)) * energy_scale_sqrt;
					J0(0, 1) = (fx*rz / zc + (fx*xc*rx) / (zc*zc)) * energy_scale_sqrt;
					J0(0, 2) = (-fx*ry / zc) * energy_scale_sqrt;
					J0(0, 3) = (fx / zc) * energy_scale_sqrt;
					J0(0, 4) = 0.0 * energy_scale_sqrt;
					J0(0, 5) = (-fx*xc / (zc*zc)) * energy_scale_sqrt;

					J0(1, 0) = (-fy*rz / zc - (fy*yc*ry) / (zc*zc)) * energy_scale_sqrt;
					J0(1, 1) = (fy*yc*rx / (zc*zc)) * energy_scale_sqrt;
					J0(1, 2) = (fy*rx / zc)* energy_scale_sqrt;
					J0(1, 3) = 0.0 * energy_scale_sqrt;
					J0(1, 4) = (fy / zc) * energy_scale_sqrt;
					J0(1, 5) = (-fy*yc / (zc*zc)) * energy_scale_sqrt;
				}
				return true;
			}

		private:
			double energy_scale_;
			Eigen::Vector3d pnt3d_;
			Eigen::Vector4d plane_info_;
			Eigen::Matrix3d K_;
			Eigen::Vector2d observation_;
		};

		//optimize camera pose only. fix point 3d, cost: e = line*project(T_cw * p3d_W_)
		class CamPoseOnlyFromLineProjCostFunc :public ceres::SizedCostFunction<1, 6>
		{
		public:
			CamPoseOnlyFromLineProjCostFunc(const Eigen::Vector2d& ob_line_s,
				const Eigen::Vector2d& ob_line_e)
			{
				//pnt_3d_ = pnt3d;
				observation_line_s_ = ob_line_s;
				observation_line_e_ = ob_line_e;
				energy_scale_ = 1.0;
			}
			virtual ~CamPoseOnlyFromLineProjCostFunc() {}
			void SetKMatrix(const Eigen::Matrix3d& k)
			{
				K_ = k;
			}
			void SetPoint3DFromPnt(const Eigen::Vector3d& p3d)
			{
				p3d_w_ = p3d;
			}
			void SetPlaneInfo(const Eigen::Vector4d& planeinfo)
			{
				plane_info_ = planeinfo;
			}
			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}
			virtual bool Evaluate(double const* const* parameters,
				double* residuals,
				double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				*/
				double rv_w[3];
				double r_cw[9];
				rv_w[0] = parameters[0][0];
				rv_w[1] = parameters[0][1];
				rv_w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(rv_w, r_cw);
				Eigen::Vector3d t_cw;
				t_cw[0] = parameters[0][3];
				t_cw[1] = parameters[0][4];
				t_cw[2] = parameters[0][5];
				Eigen::Vector3d p3d_w = p3d_w_;
				//p3d_w[0] = parameters[1][0];
				//p3d_w[1] = parameters[1][1];
				//p3d_w[2] = parameters[1][2];
				//p3d_w[3] = 1.0;
				double const rx = r_cw[0] * p3d_w[0] + r_cw[1] * p3d_w[1] + r_cw[2] * p3d_w[2];	//row major
				double const ry = r_cw[3] * p3d_w[0] + r_cw[4] * p3d_w[1] + r_cw[5] * p3d_w[2];	//row major
				double const rz = r_cw[6] * p3d_w[0] + r_cw[7] * p3d_w[1] + r_cw[8] * p3d_w[2];	//row major
				double const xc = rx + t_cw[0];	//in camera coordinate
				double const yc = ry + t_cw[1];	//in camera coordinate
				double const zc = rz + t_cw[2];	//in camera coordinate
				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy
												//print_cam_test_r_info(r_cw);
												//std::cerr << "fx, fy, cx, cy: " << fx << ", " << fy << ", " << cx << ", " << cy << std::endl;
				double const iu = fx * xc / zc + cx;	//image pixel
				double const iv = fy * yc / zc + cy;	//image pixel
														/*std::cerr << "observation_[0]->(iu), observation_[1]->(iv): " << observation_[0] << "->(" << iu << " )" << ", "
														<< observation_[1] << "->(" << iv << " )" << std::endl;*/
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				//uvi to observation_line_ dist
				Eigen::Vector3d ob_line = HW::GetLineFuncFromLine2Pnts2d(observation_line_s_, observation_line_e_);
				double denorm = std::sqrt(ob_line[0] * ob_line[0] + ob_line[1] * ob_line[1]);
				residuals[0] = energy_scale_sqrt * (iu*ob_line[0] + iv*ob_line[1] + ob_line[2]) / denorm;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> J0(jacobians[0]);
					Eigen::Matrix<double, 2, 6> jaco_uvi_to_wrt;
					//compute the jacobians
					//rx = r_0P; ry = r_1P; rz = r_2P
					jaco_uvi_to_wrt(0, 0) = (-fx*xc*ry / (zc*zc));
					jaco_uvi_to_wrt(0, 1) = (fx*rz / zc + (fx*xc*rx) / (zc*zc));
					jaco_uvi_to_wrt(0, 2) = (-fx*ry / zc);
					jaco_uvi_to_wrt(0, 3) = (fx / zc);
					jaco_uvi_to_wrt(0, 4) = 0.0;
					jaco_uvi_to_wrt(0, 5) = (-fx*xc / (zc*zc));

					jaco_uvi_to_wrt(1, 0) = (-fy*rz / zc - (fy*yc*ry) / (zc*zc));
					jaco_uvi_to_wrt(1, 1) = (fy*yc*rx / (zc*zc));
					jaco_uvi_to_wrt(1, 2) = (fy*rx / zc);
					jaco_uvi_to_wrt(1, 3) = 0.0;
					jaco_uvi_to_wrt(1, 4) = (fy / zc);
					jaco_uvi_to_wrt(1, 5) = (-fy*yc / (zc*zc));

					J0(0, 0) = (ob_line[0] * jaco_uvi_to_wrt(0, 0) + ob_line[1] * jaco_uvi_to_wrt(1, 0)) * energy_scale_sqrt / denorm;
					J0(0, 1) = (ob_line[0] * jaco_uvi_to_wrt(0, 1) + ob_line[1] * jaco_uvi_to_wrt(1, 1)) * energy_scale_sqrt / denorm;
					J0(0, 2) = (ob_line[0] * jaco_uvi_to_wrt(0, 2) + ob_line[1] * jaco_uvi_to_wrt(1, 2)) * energy_scale_sqrt / denorm;
					J0(0, 3) = (ob_line[0] * jaco_uvi_to_wrt(0, 3) + ob_line[1] * jaco_uvi_to_wrt(1, 3)) * energy_scale_sqrt / denorm;
					J0(0, 4) = (ob_line[0] * jaco_uvi_to_wrt(0, 4) + ob_line[1] * jaco_uvi_to_wrt(1, 4)) * energy_scale_sqrt / denorm;
					J0(0, 5) = (ob_line[0] * jaco_uvi_to_wrt(0, 5) + ob_line[1] * jaco_uvi_to_wrt(1, 5)) * energy_scale_sqrt / denorm;
				}

				return true;
			}

		private:
			//double x_;
			Eigen::Vector2d observation_line_s_;
			Eigen::Vector2d observation_line_e_;
			Eigen::Vector3d p3d_w_;
			Eigen::Vector3d plane_info_;
			Eigen::Matrix3d K_;
			double energy_scale_;
		};
#endif

		//optimize camera pose only. fix plane 3d, cost: e = (u,v) - project(T_cw * pnt3d_w_)
		class CamPoseOnlyFromPntProjCostFunc : public ceres::SizedCostFunction<2, 6>
		{
		public:
			CamPoseOnlyFromPntProjCostFunc(const Eigen::Vector2d& ob)
			{
				observation_[0] = ob[0];
				observation_[1] = ob[1];
				pnt3d_ = Eigen::Vector3d::Zero();
				K_ = Eigen::Matrix3d::Identity();
				energy_scale_ = 1.0;
			}
			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}
			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}
			void SetPoint3dFromPnt(const Eigen::Vector3d& p3d)
			{
				pnt3d_ = p3d;
			}
			void SetPlaneInfo(const Eigen::Vector4d& planeinfo)
			{
				plane_info_ = planeinfo;
			}
			virtual ~CamPoseOnlyFromPntProjCostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				*/
				double w[3];
				double wr[9];
				w[0] = parameters[0][0];
				w[1] = parameters[0][1];
				w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(w, wr);
				double t[3];
				t[0] = parameters[0][3];
				t[1] = parameters[0][4];
				t[2] = parameters[0][5];

				//compute p3d
				Eigen::Matrix3d r_cw;
				r_cw << wr[0], wr[1], wr[2],
					wr[3], wr[4], wr[5],
					wr[6], wr[7], wr[8];
				Eigen::Vector3d cm_exrt_t 
					= Eigen::Vector3d(t[0], t[1], t[2]);
				Eigen::Vector3d cm_c = -r_cw.transpose() * cm_exrt_t;
				Eigen::Vector3d ray_d = r_cw.inverse() * K_.inverse() * Eigen::Vector3d(observation_[0], observation_[1], 1.0);
				Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info_[0], plane_info_[1], plane_info_[2]);
				double ray_len = (-plane_info_[3] - cm_c.dot(plane_n)) / ray_d.dot(plane_n);
				Eigen::Vector3d ray_plane_pnt = cm_c + ray_len * ray_d;
				double p3d[3];
				p3d[0] = ray_plane_pnt[0];
				p3d[1] = ray_plane_pnt[1];
				p3d[2] = ray_plane_pnt[2];

				double const rx = wr[0] * p3d[0] + wr[1] * p3d[1] + wr[2] * p3d[2];	//row major
				double const ry = wr[3] * p3d[0] + wr[4] * p3d[1] + wr[5] * p3d[2];	//row major
				double const rz = wr[6] * p3d[0] + wr[7] * p3d[1] + wr[8] * p3d[2];	//row major
				double const xc = rx + t[0];	//in camera coordinate
				double const yc = ry + t[1];	//in camera coordinate
				double const zc = rz + t[2];	//in camera coordinate
				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy
				//print_cam_test_r_info(wr);
				//std::cerr << "fx, fy, cx, cy: " << fx << ", " << fy << ", " << cx << ", " << cy << std::endl;
				double const iu = fx * xc / zc + cx;	//image pixel
				double const iv = fy * yc / zc + cy;	//image pixel
				/*std::cerr << "observation_[0]->(iu), observation_[1]->(iv): " << observation_[0] << "->(" << iu << " )" << ", "
					<< observation_[1] << "->(" << iv << " )" << std::endl;*/
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				residuals[0] = (-observation_[0] + iu) * energy_scale_sqrt;
				residuals[1] = (-observation_[1] + iv) * energy_scale_sqrt;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J0(jacobians[0]);
					//compute the jacobians
					//rx = r_0P; ry = r_1P; rz = r_2P
					J0(0, 0) = (-fx*xc*ry / (zc*zc)) * energy_scale_sqrt;
					J0(0, 1) = (fx*rz / zc + (fx*xc*rx) / (zc*zc)) * energy_scale_sqrt;
					J0(0, 2) = (-fx*ry / zc) * energy_scale_sqrt;
					J0(0, 3) = (fx / zc) * energy_scale_sqrt;
					J0(0, 4) = 0.0 * energy_scale_sqrt;
					J0(0, 5) = (-fx*xc / (zc*zc)) * energy_scale_sqrt;

					J0(1, 0) = (-fy*rz / zc - (fy*yc*ry) / (zc*zc)) * energy_scale_sqrt;
					J0(1, 1) = (fy*yc*rx / (zc*zc)) * energy_scale_sqrt;
					J0(1, 2) = (fy*rx / zc)* energy_scale_sqrt;
					J0(1, 3) = 0.0 * energy_scale_sqrt;
					J0(1, 4) = (fy / zc) * energy_scale_sqrt;
					J0(1, 5) = (-fy*yc / (zc*zc)) * energy_scale_sqrt;
				}
				return true;
			}

		private:
			double energy_scale_;
			Eigen::Vector3d pnt3d_;
			Eigen::Vector4d plane_info_;
			Eigen::Matrix3d K_;
			Eigen::Vector2d observation_;
		};

		//optimize camera pose only. fix plane 3d, cost: e = line*project(T_cw * p3d_W_)
		class CamPoseOnlyFromLineProjCostFunc :public ceres::SizedCostFunction<1, 6>
		{
		public:
			CamPoseOnlyFromLineProjCostFunc(const Eigen::Vector2d& ob_line_s,
				const Eigen::Vector2d& ob_line_e)
			{
				//pnt_3d_ = pnt3d;
				observation_line_s_ = ob_line_s;
				observation_line_e_ = ob_line_e;
				energy_scale_ = 1.0;
			}
			virtual ~CamPoseOnlyFromLineProjCostFunc() {}
			void SetKMatrix(const Eigen::Matrix3d& k)
			{
				K_ = k;
			}
			void SetPoint3DFromPnt(const Eigen::Vector3d& p3d)
			{
				p3d_w_ = p3d;
			}
			void SetPlaneInfo(const Eigen::Vector4d& planeinfo)
			{
				plane_info_ = planeinfo;
			}
			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}
			virtual bool Evaluate(double const* const* parameters,
				double* residuals,
				double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				*/
				double rv_w[3];
				double r_cw[9];
				rv_w[0] = parameters[0][0];
				rv_w[1] = parameters[0][1];
				rv_w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(rv_w, r_cw);
				Eigen::Vector3d t_cw;
				t_cw[0] = parameters[0][3];
				t_cw[1] = parameters[0][4];
				t_cw[2] = parameters[0][5];

				////compute p3d
				//Eigen::Matrix3d Er_cw;
				//Er_cw << r_cw[0], r_cw[1], r_cw[2],
				//	r_cw[3], r_cw[4], r_cw[5],
				//	r_cw[6], r_cw[7], r_cw[8];
				//Eigen::Vector3d cm_exrt_t
				//	= Eigen::Vector3d(t_cw[0], t_cw[1], t_cw[2]);
				//Eigen::Vector3d cm_c = -Er_cw.transpose() * cm_exrt_t;
				//Eigen::Vector3d ray_d = Er_cw.inverse() * K_.inverse() * Eigen::Vector3d(observation_[0], observation_[1], 1.0);
				//Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info_[0], plane_info_[1], plane_info_[2]);
				//double ray_len = (-plane_info_[3] - cm_c.dot(plane_n)) / ray_d.dot(plane_n);
				//Eigen::Vector3d ray_plane_pnt = cm_c + ray_len * ray_d;
				//double p3d[3];
				//p3d[0] = ray_plane_pnt[0];
				//p3d[1] = ray_plane_pnt[1];
				//p3d[2] = ray_plane_pnt[2];

				Eigen::Vector3d p3d_w = p3d_w_;
				//p3d_w[0] = parameters[1][0];
				//p3d_w[1] = parameters[1][1];
				//p3d_w[2] = parameters[1][2];
				//p3d_w[3] = 1.0;
				double const rx = r_cw[0] * p3d_w[0] + r_cw[1] * p3d_w[1] + r_cw[2] * p3d_w[2];	//row major
				double const ry = r_cw[3] * p3d_w[0] + r_cw[4] * p3d_w[1] + r_cw[5] * p3d_w[2];	//row major
				double const rz = r_cw[6] * p3d_w[0] + r_cw[7] * p3d_w[1] + r_cw[8] * p3d_w[2];	//row major
				double const xc = rx + t_cw[0];	//in camera coordinate
				double const yc = ry + t_cw[1];	//in camera coordinate
				double const zc = rz + t_cw[2];	//in camera coordinate
				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy
				//print_cam_test_r_info(r_cw);
				//std::cerr << "fx, fy, cx, cy: " << fx << ", " << fy << ", " << cx << ", " << cy << std::endl;
				double const iu = fx * xc / zc + cx;	//image pixel
				double const iv = fy * yc / zc + cy;	//image pixel
				/*std::cerr << "observation_[0]->(iu), observation_[1]->(iv): " << observation_[0] << "->(" << iu << " )" << ", "
					<< observation_[1] << "->(" << iv << " )" << std::endl;*/
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				//uvi to observation_line_ dist
				Eigen::Vector3d ob_line = HW::GetLineFuncFromLine2Pnts2d(observation_line_s_, observation_line_e_);
				double denorm = std::sqrt(ob_line[0] * ob_line[0] + ob_line[1] * ob_line[1]);
				residuals[0] = energy_scale_sqrt * (iu*ob_line[0] + iv*ob_line[1] + ob_line[2]) / denorm;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> J0(jacobians[0]);
					Eigen::Matrix<double, 2, 6> jaco_uvi_to_wrt;
					//compute the jacobians
					//rx = r_0P; ry = r_1P; rz = r_2P
					jaco_uvi_to_wrt(0, 0) = (-fx*xc*ry / (zc*zc));
					jaco_uvi_to_wrt(0, 1) = (fx*rz / zc + (fx*xc*rx) / (zc*zc));
					jaco_uvi_to_wrt(0, 2) = (-fx*ry / zc);
					jaco_uvi_to_wrt(0, 3) = (fx / zc);
					jaco_uvi_to_wrt(0, 4) = 0.0;
					jaco_uvi_to_wrt(0, 5) = (-fx*xc / (zc*zc));

					jaco_uvi_to_wrt(1, 0) = (-fy*rz / zc - (fy*yc*ry) / (zc*zc));
					jaco_uvi_to_wrt(1, 1) = (fy*yc*rx / (zc*zc));
					jaco_uvi_to_wrt(1, 2) = (fy*rx / zc);
					jaco_uvi_to_wrt(1, 3) = 0.0;
					jaco_uvi_to_wrt(1, 4) = (fy / zc);
					jaco_uvi_to_wrt(1, 5) = (-fy*yc / (zc*zc));

					J0(0, 0) = (ob_line[0] * jaco_uvi_to_wrt(0, 0) + ob_line[1] * jaco_uvi_to_wrt(1, 0)) * energy_scale_sqrt / denorm;
					J0(0, 1) = (ob_line[0] * jaco_uvi_to_wrt(0, 1) + ob_line[1] * jaco_uvi_to_wrt(1, 1)) * energy_scale_sqrt / denorm;
					J0(0, 2) = (ob_line[0] * jaco_uvi_to_wrt(0, 2) + ob_line[1] * jaco_uvi_to_wrt(1, 2)) * energy_scale_sqrt / denorm;
					J0(0, 3) = (ob_line[0] * jaco_uvi_to_wrt(0, 3) + ob_line[1] * jaco_uvi_to_wrt(1, 3)) * energy_scale_sqrt / denorm;
					J0(0, 4) = (ob_line[0] * jaco_uvi_to_wrt(0, 4) + ob_line[1] * jaco_uvi_to_wrt(1, 4)) * energy_scale_sqrt / denorm;
					J0(0, 5) = (ob_line[0] * jaco_uvi_to_wrt(0, 5) + ob_line[1] * jaco_uvi_to_wrt(1, 5)) * energy_scale_sqrt / denorm;
				}
				
				return true;
			}

		private:
			//double x_;
			Eigen::Vector2d observation_line_s_;
			Eigen::Vector2d observation_line_e_;
			Eigen::Vector3d p3d_w_;
			Eigen::Vector4d plane_info_;
			Eigen::Matrix3d K_;
			double energy_scale_;
		};

#if 0		//test for single function
		class PntcostFuncTest : public ceres::SizedCostFunction<2, 3, 3>
		{
		public:
			PntcostFuncTest(const Eigen::Vector2d& ob)
			{
				observation_[0] = ob[0];
				observation_[1] = ob[1];
			}

			virtual ~PntcostFuncTest() {};
			//compute the jacobian
			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				parameters 0 -> (1,3)
				parameters 0: W parameters(w[0],w[1],w[2])
				parameters 1 -> (1,3)
				parameter 1: XYZ parameters(x[0],x[1],x[2])
				*/
				double w[3];
				w[0] = parameters[0][0];
				w[1] = parameters[0][1];
				w[2] = parameters[0][2];
				double x[3];
				x[0] = parameters[1][0];
				x[1] = parameters[1][1];
				x[2] = parameters[1][2];
				residuals[0] = observation_[0] - 2 * w[0] - 3 * w[1] - 4 * w[2] - 2 * x[0] - 2 * x[1] - 2 * x[2];
				residuals[1] = observation_[1] - 4 * w[0] - 3 * w[1] - 2 * w[2] - 2 * x[0] - 2 * x[1] - 2 * x[2];
				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J0(jacobians[0]);
					//compute the jacobians
					//rx = r_0P; ry = r_1P; rz = r_2P
					J0(0, 0) = -2;
					J0(0, 1) = -3;
					J0(0, 2) = -4;
					J0(1, 0) = -4;
					J0(1, 1) = -3;
					J0(1, 2) = -2;
				}
				if (jacobians != NULL && jacobians[1] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J1(jacobians[1]);
					J1(0, 0) = -2;
					J1(0, 1) = -2;
					J1(0, 2) = -2;
					J1(1, 0) = -2;
					J1(1, 1) = -2;
					J1(1, 2) = -2;
				}
				return true;
			}
		private:
			Eigen::Vector2d observation_;
		};


		class PosePnt3DcostFunc : public ceres::SizedCostFunction<2, 3>
		{
		public:
			PosePnt3DcostFunc(const Eigen::Vector2d& ob)
			{
				observation_[0] = ob[0];
				observation_[1] = ob[1];
				K_ = Eigen::Matrix3d::Identity();
				for (int i = 0; i < 6; ++i)
				{
					W_RT_[i] = 0.0;
				}
			}
			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}
			void SetRTW(double* rtw)
			{
				for (int i = 0; i < 6; ++i)
				{
					W_RT_[i] = rtw[i];
				}
			}
			virtual ~PosePnt3DcostFunc() {};
			//compute the jacobian
			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				parameters 0 -> (1,3)
				parameter 0: p3d parameters(x[0],x[1],x[2])
				*/
				double w[3];
				double wr[9];
				w[0] = W_RT_[0];
				w[1] = W_RT_[1];
				w[2] = W_RT_[2];
				HW::rodrigues_to_matrix3d(w, wr);
				double t[3];
				t[0] = W_RT_[3];
				t[1] = W_RT_[4];
				t[2] = W_RT_[5];
				double p3d[3];
				p3d[0] = parameters[0][0];
				p3d[1] = parameters[0][1];
				p3d[2] = parameters[0][2];
				double const rx = wr[0] * p3d[0] + wr[1] * p3d[1] + wr[2] * p3d[2];	//row major
				double const ry = wr[3] * p3d[0] + wr[4] * p3d[1] + wr[5] * p3d[2];	//row major
				double const rz = wr[6] * p3d[0] + wr[7] * p3d[1] + wr[8] * p3d[2];	//row major
				double const xc = rx + t[0];	//in camera coordinate
				double const yc = ry + t[1];	//in camera coordinate
				double const zc = rz + t[2];	//in camera coordinate
				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy
				//print_cam_test_r_info(wr);
				//std::cerr << "fx, fy, cx, cy: " << fx << ", " << fy << ", " << cx << ", " << cy << std::endl;
				double const iu = fx * xc / zc + cx;	//image pixel
				double const iv = fy * yc / zc + cy;	//image pixel
				//std::cerr << "observation_[0], observation_[1]: " << observation_[0] << ", " << observation_[1] << std::endl;
				//std::cerr << "iu, iv: " << iu << ", " << iv << std::endl;
				residuals[0] = -observation_[0] + iu;
				residuals[1] = -observation_[1] + iv;
				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J0(jacobians[0]);
					J0(0, 0) = fx*wr[0] / zc - fx*xc*wr[6] / (zc*zc);
					J0(0, 1) = fx*wr[1] / zc - fx*xc*wr[7] / (zc*zc);
					J0(0, 2) = fx*wr[2] / zc - fx*xc*wr[8] / (zc*zc);
					J0(1, 0) = fy*wr[3] / zc - fy*yc*wr[6] / (zc*zc);
					J0(1, 1) = fy*wr[4] / zc - fy*yc*wr[7] / (zc*zc);
					J0(1, 2) = fy*wr[5] / zc - fy*yc*wr[8] / (zc*zc);
				}
				return true;
			}
		private:
			double W_RT_[6];
			Eigen::Matrix3d K_;
			Eigen::Vector2d observation_;
		};
		
		class PosePntWRcostFunc : public ceres::SizedCostFunction<2, 6>
		{
		public:
			PosePntWRcostFunc(const Eigen::Vector2d& ob)
			{
				observation_[0] = ob[0];
				observation_[1] = ob[1];
				K_ = Eigen::Matrix3d::Identity();
				for (int i = 0; i < 3; ++i)
				{
					pnt_3d_[0] = 0.0;
					pnt_3d_[1] = 0.0;
					pnt_3d_[2] = 0.0;
				}
			}
			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}
			void SetPoint3d(double* pnt3d_pos)
			{
				for (int i = 0; i < 3; ++i)
				{
					pnt_3d_[0] = pnt3d_pos[0];
					pnt_3d_[1] = pnt3d_pos[1];
					pnt_3d_[2] = pnt3d_pos[2];
				}
			}

			virtual ~PosePntWRcostFunc() {};
			//compute the jacobian
			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				*/
				double w[3];
				double wr[9];
				w[0] = parameters[0][0];
				w[1] = parameters[0][1];
				w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(w, wr);
				double t[3];
				t[0] = parameters[0][3];
				t[1] = parameters[0][4];
				t[2] = parameters[0][5];
				double p3d[3];
				p3d[0] = pnt_3d_[0];
				p3d[1] = pnt_3d_[1];
				p3d[2] = pnt_3d_[2];
				double const rx = wr[0] * p3d[0] + wr[1] * p3d[1] + wr[2] * p3d[2];	//row major
				double const ry = wr[3] * p3d[0] + wr[4] * p3d[1] + wr[5] * p3d[2];	//row major
				double const rz = wr[6] * p3d[0] + wr[7] * p3d[1] + wr[8] * p3d[2];	//row major
				double const xc = rx + t[0];	//in camera coordinate
				double const yc = ry + t[1];	//in camera coordinate
				double const zc = rz + t[2];	//in camera coordinate
				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy
				//print_cam_test_r_info(wr);
				//std::cerr << "fx, fy, cx, cy: " << fx << ", " << fy << ", " << cx << ", " << cy << std::endl;
				double const iu = fx * xc / zc + cx;	//image pixel
				double const iv = fy * yc / zc + cy;	//image pixel
				//std::cerr << "observation_[0], observation_[1]: " << observation_[0] << ", " << observation_[1] << std::endl;
				//std::cerr << "iu, iv: " << iu << ", " << iv << std::endl;
				residuals[0] = -observation_[0] + iu;
				residuals[1] = -observation_[1] + iv;
				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J0(jacobians[0]);
					//compute the jacobians
					//rx = r_0P; ry = r_1P; rz = r_2P
					J0(0, 0) = -fx*xc*ry / (zc*zc);
					J0(0, 1) = fx*rz / zc + (fx*xc*rx) / (zc*zc);
					J0(0, 2) = -fx*ry / zc;
					J0(0, 3) = fx / zc;
					J0(0, 4) = 0.0;
					J0(0, 5) = -fx*xc / (zc*zc);

					J0(1, 0) = -fy*rz / zc - (fy*yc*ry) / (zc*zc);
					J0(1, 1) = fy*yc*rx / (zc*zc);
					J0(1, 2) = fy*rx / zc;
					J0(1, 3) = 0.0;
					J0(1, 4) = fy / zc;
					J0(1, 5) = -fy*yc / (zc*zc);
				}
				return true;
			}
		private:
			Eigen::Matrix3d K_;
			double pnt_3d_[3];
			Eigen::Vector2d observation_;
		};

#endif

#if 0
		//to delete it
		class PosePnt2PlaneCostFunc : public ceres::SizedCostFunction<1, 6, 1>
		{
		public:
			PosePnt2PlaneCostFunc(const Eigen::Vector2d& ob)
			{
				uv_[0] = ob[0];
				uv_[1] = ob[1];
				K_ = Eigen::Matrix3d::Identity();
				plane_fuc_ = Eigen::Vector4d::Zero();
				energy_scale_ = 1.0;
			}

			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}

			void SetPlaneFunc(const Eigen::Vector4d& fuc)
			{
				plane_fuc_ = fuc;
			}

			virtual ~PosePnt2PlaneCostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				parameters 1 -> (1,1)
				parameter 1: camera depth parameters z(d)
				*/
				double w[3];
				double wr[9];
				w[0] = parameters[0][0];
				w[1] = parameters[0][1];
				w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(w, wr);	//world to camera
				double t[3];
				t[0] = parameters[0][3];
				t[1] = parameters[0][4];
				t[2] = parameters[0][5];
				double d;
				d = parameters[1][0];
				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy

				double const xc = d * (uv_[0] - cx) / fx;	//in camera coordinate
				double const yc = d * (uv_[1] - cy) / fy;	//in camera coordinate
				double const zc = d;	//in camera coordinate

				//camera to world coordinate
				double rc2w[9];
				HW::ConvertMatrix3d_to_Matrix3d_Transposed(wr, rc2w);
				double rwv[3];
				rwv[0] = rc2w[0] * xc + rc2w[1] * yc + rc2w[2] * zc;
				rwv[1] = rc2w[3] * xc + rc2w[4] * yc + rc2w[5] * zc;
				rwv[2] = rc2w[6] * xc + rc2w[7] * yc + rc2w[8] * zc;
				double t_w[3];
				t_w[0] = -(rc2w[0] * t[0] + rc2w[1] * t[1] + rc2w[2] * t[2]);
				t_w[1] = -(rc2w[3] * t[0] + rc2w[4] * t[1] + rc2w[5] * t[2]);
				t_w[2] = -(rc2w[6] * t[0] + rc2w[7] * t[1] + rc2w[8] * t[2]);
				double pw[3];
				pw[0] = rwv[0] + t_w[0];	//in world coordinate
				pw[1] = rwv[1] + t_w[1];	//in world coordinate
				pw[2] = rwv[2] + t_w[2];	//in world coordinate

				//compute the residuals, world pnt to plane distance
				residuals[0] = plane_fuc_[0] * pw[0] + plane_fuc_[1] * pw[1] + plane_fuc_[2] * pw[2] + plane_fuc_[3];

				double rv0 = rc2w[0] * (xc - t[0]) + rc2w[1] * (yc - t[1]) + rc2w[2] * (zc - t[2]);
				double rv1 = rc2w[3] * (xc - t[0]) + rc2w[4] * (yc - t[1]) + rc2w[5] * (zc - t[2]);
				double rv2 = rc2w[6] * (xc - t[0]) + rc2w[7] * (yc - t[1]) + rc2w[8] * (zc - t[2]);

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					//double pl_square = std::pow(pl_value, 2);
					Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> J0(jacobians[0]);

					J0(0, 0) = (plane_fuc_[1] * rv2 - plane_fuc_[2] * rv1);
					J0(0, 1) = (-plane_fuc_[0] * rv2 + plane_fuc_[2] * rv0);
					J0(0, 2) = (plane_fuc_[0] * rv1 - plane_fuc_[1] * rv0);
					J0(0, 3) = -(plane_fuc_[0] * rc2w[0] + plane_fuc_[1] * rc2w[3] + plane_fuc_[2] * rc2w[6]);
					J0(0, 4) = -(plane_fuc_[0] * rc2w[1] + plane_fuc_[1] * rc2w[4] + plane_fuc_[2] * rc2w[7]);
					J0(0, 5) = -(plane_fuc_[0] * rc2w[2] + plane_fuc_[1] * rc2w[5] + plane_fuc_[2] * rc2w[8]);

				}
				if (jacobians != NULL && jacobians[1] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J1(jacobians[1]);
					J1(0, 0) = plane_fuc_[0] * (rc2w[0] * (uv_[0] - cx) / fx + rc2w[1] * (uv_[1] - cy) / fy + rc2w[2])
						+ plane_fuc_[1] * (rc2w[3] * (uv_[0] - cx) / fx + rc2w[4] * (uv_[1] - cy) / fy + rc2w[5])
						+ plane_fuc_[2] * (rc2w[6] * (uv_[0] - cx) / fx + rc2w[7] * (uv_[1] - cy) / fy + rc2w[8]);	//d derive
				}
				return true;
			}

		private:
			Eigen::Matrix3d K_;
			double energy_scale_;
			Eigen::Vector4d plane_fuc_;
			//uv_[0]: u; uv_[1]:v
			Eigen::Vector2d uv_;
		};

		//to do next... use meituan's thesis
		class PluckerLineCostFunctor 
		{
		public:
			PluckerLineCostFunctor(const Eigen::Vector3d& pnt3d)
			{
				pnt_3d_ = pnt3d;
			}

			template<typename T>
			bool operator()(const T* const params, T* residual) const
			{
				T line_dir[3] = { T(params[0]), T(params[1]), T(params[2]) };
				T line_pos[3] = { T(params[3]), T(params[4]), T(params[5]) };
				T point[3] = { T(pnt_3d_[0]),T(pnt_3d_[1]),T(pnt_3d_[2]) };
				
				T cross_product[3];
				cross_product[0] = line_dir[1] * (point[2] - line_pos[2]) - line_dir[2] * (point[1] - line_pos[1]);
				cross_product[1] = line_dir[2] * (point[0] - line_pos[0]) - line_dir[0] * (point[2] - line_pos[2]);
				cross_product[2] = line_dir[0] * (point[1] - line_pos[1]) - line_dir[1] * (point[0] - line_pos[0]);

				T distance = std::sqrt(cross_product[0] * cross_product[0]
					+ cross_product[1] * cross_product[1] + cross_product[2] * cross_product[2]);
				residual[0] = distance;
				return true;
			}

		private:
			Eigen::Vector3d pnt_3d_;
		};
#endif

		//Energy: plucker line(no corresponding polygon inter line) project to image line distance, 
		class PoseOthonormalLines2ProjectFunc : public ceres::SizedCostFunction<2, 6, 4>
		{
		public:
			PoseOthonormalLines2ProjectFunc(const Eigen::Vector2d& ob_line_s, 
				const Eigen::Vector2d& ob_line_e)
			{
				//pnt_3d_ = pnt3d;
				observation_line_s_ = ob_line_s;
				observation_line_e_ = ob_line_e;
				energy_scale_ = 1.0;
			}
			virtual ~PoseOthonormalLines2ProjectFunc() {}
			void SetKMatrix(const Eigen::Matrix3d& k)
			{
				K_ = k;
			}
			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}

			virtual bool Evaluate(double const* const* parameters,
				double* residuals,
				double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				parameters 1 -> (1,4)
				parameter 1: othonormal represatation (theta1, theta2, theta3, phi)
				*/
				Eigen::Vector3d rv_w;
				Eigen::Matrix3d r_cw;
				rv_w[0] = parameters[0][0];
				rv_w[1] = parameters[0][1];
				rv_w[2] = parameters[0][2];
				r_cw = HW::AxisAngleToRotationMatrixD(rv_w);
				Eigen::Vector3d t_cw;
				t_cw[0] = parameters[0][3];
				t_cw[1] = parameters[0][4];
				t_cw[2] = parameters[0][5];

				Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
				transform_matrix.block<3, 3>(0, 0) = r_cw;
				transform_matrix.block<3, 1>(0, 3) = t_cw;
				//std::cerr << "transform_matrix: \n" << transform_matrix << std::endl;

				//line in world->so(3): theta1, theta2, theta3; so(2): phi
				Eigen::Vector4d line_otho_w;
				line_otho_w[0] = parameters[1][0];
				line_otho_w[1] = parameters[1][1];
				line_otho_w[2] = parameters[1][2];
				line_otho_w[3] = parameters[1][3];	//phi contain distance

				//othonormal line to plucker line coordinate(n_w = head(3), p times q; d_w = tail(3), q - p)
				//Eigen::Matrix<double, 6, 1> line_plu_w = HW::OrthLineToPluckerLineNew(line_otho_w);
				//OrthLineToPluckerLine
				Eigen::Matrix<double, 6, 1> line_plu_w = HW::OrthLineToPluckerLine(line_otho_w);
				//get plucker transformed matrix(6 * 6)
				Eigen::Matrix<double, 6, 6> plu_transform = HW::GetPluckerTransformMatrix(transform_matrix);
				//std::cerr << "plu_transform: \n" << plu_transform << std::endl;

				//line in world coordinate to line in camera coordinate()
				Eigen::Matrix<double, 6, 1> line_plu_c = plu_transform*line_plu_w;
				Eigen::Vector3d plu_n_c = line_plu_c.head(3);	//n 
				Eigen::Vector3d plu_d_c = line_plu_c.tail(3);
				//get plucker k matrix
				Eigen::Matrix3d plu_k = HW::GetPluckerKMatrix(K_);
				//std::cerr << "plu_k: \n" << plu_k << std::endl;

				Eigen::Vector3d l_i = plu_k*plu_n_c;
				double l_demon = std::sqrt(l_i[0] * l_i[0] + l_i[1] * l_i[1]);
				//std::cerr << "l_i: " << l_i.transpose() << std::endl;

				double s_l = observation_line_s_[0] * l_i[0] + observation_line_s_[1] * l_i[1] + l_i[2];
				double e_l = observation_line_e_[0] * l_i[0] + observation_line_e_[1] * l_i[1] + l_i[2];
				
				double energy_scale_sqrt = std::sqrt(energy_scale_);

				//if delete the r_demon(or not )
				residuals[0] = energy_scale_sqrt * s_l / l_demon;
				residuals[1] = energy_scale_sqrt * e_l / l_demon;
				//std::cerr << "residuals: " << residuals[0] << ", " << residuals[1] << std::endl;

				Eigen::Matrix<double, 2, 3> jaco_resi_to_li;
				jaco_resi_to_li(0, 0) = -l_i[0] * s_l / std::pow(l_demon, 2) + observation_line_s_[0] / l_demon;
				jaco_resi_to_li(0, 1) = -l_i[1] * s_l / std::pow(l_demon, 2) + observation_line_s_[1] / l_demon;
				jaco_resi_to_li(0, 2) = 1.0 / l_demon;
				jaco_resi_to_li(1, 0) = -l_i[0] * e_l / std::pow(l_demon, 2) + observation_line_e_[0] / l_demon;
				jaco_resi_to_li(1, 1) = -l_i[1] * e_l / std::pow(l_demon, 2) + observation_line_e_[1] / l_demon;
				jaco_resi_to_li(1, 2) = 1.0 / l_demon;
				//std::cerr << "jaco_resi_to_li: \n" << jaco_resi_to_li << std::endl;

				Eigen::Matrix<double, 3, 6> jaco_li_to_Lc;
				jaco_li_to_Lc.setZero();
				jaco_li_to_Lc.block<3, 3>(0, 0) = plu_k;
				//jaco_li_to_Lc.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
				//std::cerr << "jaco_li_to_Lc: \n" << jaco_li_to_Lc << std::endl;

				Eigen::Matrix<double, 6, 6> jaco_Lc_to_Lw;
				jaco_Lc_to_Lw = plu_transform;
				//std::cerr << "jaco_Lc_to_Lw: \n" << jaco_Lc_to_Lw << std::endl;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J0(jacobians[0]);					
					
					/*Eigen::Matrix<double, 6, 6> jac_lc_rt;
					jac_lc_rt.setZero();
					jac_lc_rt.block<3, 3>(0, 0) = -HW::ComputeSkewMatrixFromVector3d(r_cw*line_otho_w.tail(3));
					jac_lc_rt.block<3, 3>(0, 3) = -HW::ComputeSkewMatrixFromVector3d(r_cw*line_otho_w.head(3)) 
						- HW::ComputeSkewMatrixFromVector3d(t_cw) * HW::ComputeSkewMatrixFromVector3d(r_cw*line_otho_w.tail(3));
					Eigen::Matrix<double, 4, 6> jac_error_pose;
					jac_error_pose.block<1, 6>(0, 0) = jaco_resi_to_li * jaco_li_to_Lc * jac_lc_rt;
					jac_error_pose.block<1, 6>(1, 0) = jaco_resi_to_li * jaco_li_to_Lc * jac_lc_rt;
					jac_error_pose.block<2, 6>(2, 0) = Eigen::Matrix<double, 2, 6>::Zero();
					J0 = jac_error_pose.block<2, 6>(0, 0);*/

					//jaco_lc_to_rvw_rtw
					Eigen::Matrix<double, 6, 6> jaco_lc_to_rvw_rtw;
					//[R_cw*n_w]_(times)=skew(R_cw*n_w)
					Eigen::Matrix3d Rn_w_skew = HW::ComputeSkewMatrixFromVector3d(r_cw*line_plu_w.head(3));
					Eigen::Matrix3d Rd_w_skew = HW::ComputeSkewMatrixFromVector3d(r_cw*line_plu_w.tail(3));
					Eigen::Matrix3d T_cw_skew = HW::ComputeSkewMatrixFromVector3d(t_cw);
					Eigen::Matrix3d TRd_w_skew = HW::ComputeSkewMatrixFromVector3d(T_cw_skew*r_cw*line_plu_w.tail(3));
					//std::cerr << "Rn_w_skew: \n" << Rn_w_skew << std::endl;
					//std::cerr << "Rd_w_skew: \n" << Rd_w_skew << std::endl;
					//std::cerr << "TRd_w_skew: \n" << TRd_w_skew << std::endl;
					jaco_lc_to_rvw_rtw.block<3, 3>(0, 0) = -Rn_w_skew - TRd_w_skew;
					jaco_lc_to_rvw_rtw.block<3, 3>(0, 3) = -Rd_w_skew;
					jaco_lc_to_rvw_rtw.block<3, 3>(3, 0) = -Rd_w_skew;
					jaco_lc_to_rvw_rtw.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();
					//std::cerr << "jaco_lc_to_rvw_rtw: \n" << jaco_lc_to_rvw_rtw << std::endl;
					Eigen::Matrix<double, 2, 6> jaco_obline_to_rw_rt;
					jaco_obline_to_rw_rt = jaco_resi_to_li * jaco_li_to_Lc*jaco_lc_to_rvw_rtw;
					//std::cerr << "jaco_obline_to_rw_rt: \n" << jaco_obline_to_rw_rt << std::endl;
					//compute the jacobians
					J0(0, 0) = energy_scale_sqrt * jaco_obline_to_rw_rt(0, 0);
					J0(0, 1) = energy_scale_sqrt * jaco_obline_to_rw_rt(0, 1);
					J0(0, 2) = energy_scale_sqrt * jaco_obline_to_rw_rt(0, 2);
					J0(0, 3) = energy_scale_sqrt * jaco_obline_to_rw_rt(0, 3);
					J0(0, 4) = energy_scale_sqrt * jaco_obline_to_rw_rt(0, 4);
					J0(0, 5) = energy_scale_sqrt * jaco_obline_to_rw_rt(0, 5);
					J0(1, 0) = energy_scale_sqrt * jaco_obline_to_rw_rt(1, 0);
					J0(1, 1) = energy_scale_sqrt * jaco_obline_to_rw_rt(1, 1);
					J0(1, 2) = energy_scale_sqrt * jaco_obline_to_rw_rt(1, 2);
					J0(1, 3) = energy_scale_sqrt * jaco_obline_to_rw_rt(1, 3);
					J0(1, 4) = energy_scale_sqrt * jaco_obline_to_rw_rt(1, 4);
					J0(1, 5) = energy_scale_sqrt * jaco_obline_to_rw_rt(1, 5);
				}
				if (jacobians != NULL && jacobians[1] != NULL)
				{
					Eigen::Matrix3d otho_so3_w = HW::GetOrthRMatrixFromPlucker(line_plu_w);
					Eigen::Matrix2d otho_so2_w = HW::GetOrthWFromPlucker(line_plu_w);
					//PluckerLineToOrthLineW;
					Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J1(jacobians[1]);
					Eigen::Matrix<double, 6, 4> jaco_Lw_to_Lotho;
					jaco_Lw_to_Lotho = ComputeJacobianFromPlukerToOrth(otho_so3_w, otho_so2_w);

					//double otho_so2_w1 = otho_so2_w(0, 0);
					//double otho_so2_w2 = otho_so2_w(1, 0);
					//Eigen::Matrix3d so2w1_othoso3_w1_skew = HW::ComputeSkewMatrixFromVector3d(otho_so2_w1*otho_so3_w.col(0));
					//Eigen::Matrix3d so2w2_othoso3_w2_skew = HW::ComputeSkewMatrixFromVector3d(otho_so2_w2*otho_so3_w.col(1));
					//jaco_Lw_to_Lotho.block<3, 3>(0, 0) = -so2w1_othoso3_w1_skew;
					//jaco_Lw_to_Lotho.block<3, 3>(3, 0) = -so2w2_othoso3_w2_skew;
					//jaco_Lw_to_Lotho.block<3, 1>(0, 3) = -otho_so2_w2*otho_so3_w.col(0);
					//jaco_Lw_to_Lotho.block<3, 1>(3, 3) = otho_so2_w1*otho_so3_w.col(1);

					Eigen::Matrix<double, 2, 4> jaco_obline_to_otho_line;
					jaco_obline_to_otho_line = jaco_resi_to_li*jaco_li_to_Lc*jaco_Lc_to_Lw*jaco_Lw_to_Lotho;

					J1(0, 0) = energy_scale_sqrt * jaco_obline_to_otho_line(0, 0);
					J1(0, 1) = energy_scale_sqrt * jaco_obline_to_otho_line(0, 1);
					J1(0, 2) = energy_scale_sqrt * jaco_obline_to_otho_line(0, 2);
					J1(0, 3) = energy_scale_sqrt * jaco_obline_to_otho_line(0, 3);
					J1(1, 0) = energy_scale_sqrt * jaco_obline_to_otho_line(1, 0);
					J1(1, 1) = energy_scale_sqrt * jaco_obline_to_otho_line(1, 1);
					J1(1, 2) = energy_scale_sqrt * jaco_obline_to_otho_line(1, 2);
					J1(1, 3) = energy_scale_sqrt * jaco_obline_to_otho_line(1, 3);
				}		
				return true;

			}
		private:
			//double x_;
			Eigen::Vector2d observation_line_s_;
			Eigen::Vector2d observation_line_e_;
			Eigen::Matrix3d K_;
			double energy_scale_;
		};

		class PoseOthonormalLines2ProjectImagePosePartFunc : public ceres::SizedCostFunction<2, 6>
		{
		public:
			PoseOthonormalLines2ProjectImagePosePartFunc(const Eigen::Vector2d& ob_line_s,
				const Eigen::Vector2d& ob_line_e)
			{
				//pnt_3d_ = pnt3d;
				observation_line_s_ = ob_line_s;
				observation_line_e_ = ob_line_e;
				energy_scale_ = 1.0;
				orthonormal_line_ = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
			}
			virtual ~PoseOthonormalLines2ProjectImagePosePartFunc() {}
			void SetKMatrix(const Eigen::Matrix3d& k)
			{
				K_ = k;
			}
			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}
			void SetOthonormalLine(const Eigen::Vector4d& orthon_line)
			{
				orthonormal_line_ = orthon_line;
			}

			virtual bool Evaluate(double const* const* parameters,
				double* residuals,
				double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				*/
				Eigen::Vector3d rv_w;
				Eigen::Matrix3d r_cw;
				rv_w[0] = parameters[0][0];
				rv_w[1] = parameters[0][1];
				rv_w[2] = parameters[0][2];
				r_cw = HW::AxisAngleToRotationMatrixD(rv_w);
				Eigen::Vector3d t_cw;
				t_cw[0] = parameters[0][3];
				t_cw[1] = parameters[0][4];
				t_cw[2] = parameters[0][5];

				Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
				transform_matrix.block<3, 3>(0, 0) = r_cw;
				transform_matrix.block<3, 1>(0, 3) = t_cw;
				//std::cerr << "transform_matrix: \n" << transform_matrix << std::endl;

				//line in world->so(3): theta1, theta2, theta3; so(2): phi
				Eigen::Vector4d line_otho_w;
				line_otho_w[0] = orthonormal_line_[0];
				line_otho_w[1] = orthonormal_line_[1];
				line_otho_w[2] = orthonormal_line_[2];
				line_otho_w[3] = orthonormal_line_[3];	//phi contain distance

				//othonormal line to plucker line coordinate(n_w = head(3), p times q; d_w = tail(3), q - p)
				Eigen::Matrix<double, 6, 1> line_plu_w = HW::OrthLineToPluckerLineNew(line_otho_w);
				//get plucker transformed matrix(6 * 6)
				Eigen::Matrix<double, 6, 6> plu_transform = HW::GetPluckerTransformMatrix(transform_matrix);
				//std::cerr << "plu_transform: \n" << plu_transform << std::endl;

				//line in world coordinate to line in camera coordinate()
				Eigen::Matrix<double, 6, 1> line_plu_c = plu_transform*line_plu_w;
				Eigen::Vector3d plu_n_c = line_plu_c.head(3);	//n 
				Eigen::Vector3d plu_d_c = line_plu_c.tail(3);
				//get plucker k matrix
				Eigen::Matrix3d plu_k = HW::GetPluckerKMatrix(K_);
				//std::cerr << "plu_k: \n" << plu_k << std::endl;

				Eigen::Vector3d l_i = plu_k*plu_n_c;
				double l_demon = std::sqrt(l_i[0] * l_i[0] + l_i[1] * l_i[1]);
				//std::cerr << "l_i: " << l_i.transpose() << std::endl;

				double s_l = observation_line_s_[0] * l_i[0] + observation_line_s_[1] * l_i[1] + l_i[2];
				double e_l = observation_line_e_[0] * l_i[0] + observation_line_e_[1] * l_i[1] + l_i[2];

				double energy_scale_sqrt = std::sqrt(energy_scale_);

				//if delete the r_demon(or not )
				residuals[0] = energy_scale_sqrt * s_l / l_demon;
				residuals[1] = energy_scale_sqrt * e_l / l_demon;
				//std::cerr << "residuals: " << residuals[0] << ", " << residuals[1] << std::endl;

				Eigen::Matrix<double, 2, 3> jaco_resi_to_li;
				jaco_resi_to_li(0, 0) = -l_i[0] * s_l / std::pow(l_demon, 2) + observation_line_s_[0] / l_demon;
				jaco_resi_to_li(0, 1) = -l_i[1] * s_l / std::pow(l_demon, 2) + observation_line_s_[1] / l_demon;
				jaco_resi_to_li(0, 2) = 1.0 / l_demon;
				jaco_resi_to_li(1, 0) = -l_i[0] * e_l / std::pow(l_demon, 2) + observation_line_e_[0] / l_demon;
				jaco_resi_to_li(1, 1) = -l_i[1] * e_l / std::pow(l_demon, 2) + observation_line_e_[1] / l_demon;
				jaco_resi_to_li(1, 2) = 1.0 / l_demon;
				//std::cerr << "jaco_resi_to_li: \n" << jaco_resi_to_li << std::endl;

				Eigen::Matrix<double, 3, 6> jaco_li_to_Lc;
				jaco_li_to_Lc.block<3, 3>(0, 0) = plu_k;
				jaco_li_to_Lc.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
				//std::cerr << "jaco_li_to_Lc: \n" << jaco_li_to_Lc << std::endl;

				Eigen::Matrix<double, 6, 6> jaco_Lc_to_Lw;
				jaco_Lc_to_Lw = plu_transform;
				//std::cerr << "jaco_Lc_to_Lw: \n" << jaco_Lc_to_Lw << std::endl;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J0(jacobians[0]);
					//jaco_lc_to_rvw_rtw
					Eigen::Matrix<double, 6, 6> jaco_lc_to_rvw_rtw;
					//[R_cw*n_w]_(times)=skew(R_cw*n_w)
					Eigen::Matrix3d Rn_w_skew = HW::ComputeSkewMatrixFromVector3d(r_cw*line_plu_w.head(3));
					Eigen::Matrix3d Rd_w_skew = HW::ComputeSkewMatrixFromVector3d(r_cw*line_plu_w.tail(3));
					Eigen::Matrix3d T_cw_skew = HW::ComputeSkewMatrixFromVector3d(t_cw);
					Eigen::Matrix3d TRd_w_skew = HW::ComputeSkewMatrixFromVector3d(T_cw_skew*r_cw*line_plu_w.tail(3));
					//std::cerr << "Rn_w_skew: \n" << Rn_w_skew << std::endl;
					//std::cerr << "Rd_w_skew: \n" << Rd_w_skew << std::endl;
					//std::cerr << "TRd_w_skew: \n" << TRd_w_skew << std::endl;

					jaco_lc_to_rvw_rtw.block<3, 3>(0, 0) = -Rn_w_skew - TRd_w_skew;
					jaco_lc_to_rvw_rtw.block<3, 3>(0, 3) = -Rd_w_skew;
					jaco_lc_to_rvw_rtw.block<3, 3>(3, 0) = -Rd_w_skew;
					jaco_lc_to_rvw_rtw.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();
					//std::cerr << "jaco_lc_to_rvw_rtw: \n" << jaco_lc_to_rvw_rtw << std::endl;

					Eigen::Matrix<double, 2, 6> jaco_obline_to_rw_rt;
					jaco_obline_to_rw_rt = jaco_resi_to_li * jaco_li_to_Lc*jaco_lc_to_rvw_rtw;
					//std::cerr << "jaco_obline_to_rw_rt: \n" << jaco_obline_to_rw_rt << std::endl;
					//compute the jacobians
					J0(0, 0) = energy_scale_sqrt * jaco_obline_to_rw_rt(0, 0);
					J0(0, 1) = energy_scale_sqrt * jaco_obline_to_rw_rt(0, 1);
					J0(0, 2) = energy_scale_sqrt * jaco_obline_to_rw_rt(0, 2);
					J0(0, 3) = energy_scale_sqrt * jaco_obline_to_rw_rt(0, 3);
					J0(0, 4) = energy_scale_sqrt * jaco_obline_to_rw_rt(0, 4);
					J0(0, 5) = energy_scale_sqrt * jaco_obline_to_rw_rt(0, 5);

					J0(1, 0) = energy_scale_sqrt * jaco_obline_to_rw_rt(1, 0);
					J0(1, 1) = energy_scale_sqrt * jaco_obline_to_rw_rt(1, 1);
					J0(1, 2) = energy_scale_sqrt * jaco_obline_to_rw_rt(1, 2);
					J0(1, 3) = energy_scale_sqrt * jaco_obline_to_rw_rt(1, 3);
					J0(1, 4) = energy_scale_sqrt * jaco_obline_to_rw_rt(1, 4);
					J0(1, 5) = energy_scale_sqrt * jaco_obline_to_rw_rt(1, 5);
				}

#if 0
				if (jacobians != NULL && jacobians[1] != NULL)
				{
					Eigen::Matrix3d otho_so3_w = HW::GetOrthRMatrixFromPlucker(line_plu_w);
					Eigen::Matrix2d otho_so2_w = HW::GetOrthWFromPlucker(line_plu_w);
					Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J1(jacobians[1]);
					double otho_so2_w1 = otho_so2_w(0, 0);
					double otho_so2_w2 = otho_so2_w(1, 0);

					Eigen::Matrix<double, 6, 4> jaco_Lw_to_Lotho;
					Eigen::Matrix3d so2w1_othoso3_w1_skew = HW::ComputeSkewMatrixFromVector3d(otho_so2_w1*otho_so3_w.col(0));
					Eigen::Matrix3d so2w2_othoso3_w2_skew = HW::ComputeSkewMatrixFromVector3d(otho_so2_w2*otho_so3_w.col(1));

					jaco_Lw_to_Lotho.block<3, 3>(0, 0) = -so2w1_othoso3_w1_skew;
					jaco_Lw_to_Lotho.block<3, 3>(3, 0) = -so2w2_othoso3_w2_skew;
					jaco_Lw_to_Lotho.block<3, 1>(0, 3) = -otho_so2_w2*otho_so3_w.col(0);
					jaco_Lw_to_Lotho.block<3, 1>(3, 3) = otho_so2_w1*otho_so3_w.col(1);

					Eigen::Matrix<double, 2, 4> jaco_obline_to_otho_line;
					jaco_obline_to_otho_line = jaco_resi_to_li*jaco_li_to_Lc*jaco_Lc_to_Lw*jaco_Lw_to_Lotho;

					J1(0, 0) = energy_scale_sqrt * jaco_obline_to_otho_line(0, 0);
					J1(0, 1) = energy_scale_sqrt * jaco_obline_to_otho_line(0, 1);
					J1(0, 2) = energy_scale_sqrt * jaco_obline_to_otho_line(0, 2);
					J1(0, 3) = energy_scale_sqrt * jaco_obline_to_otho_line(0, 3);
					J1(1, 0) = energy_scale_sqrt * jaco_obline_to_otho_line(1, 0);
					J1(1, 1) = energy_scale_sqrt * jaco_obline_to_otho_line(1, 1);
					J1(1, 2) = energy_scale_sqrt * jaco_obline_to_otho_line(1, 2);
					J1(1, 3) = energy_scale_sqrt * jaco_obline_to_otho_line(1, 3);
				}
#endif
				return true;

			}

		private:
			Eigen::Vector2d observation_line_s_;
			Eigen::Vector2d observation_line_e_;
			double energy_scale_;
			Eigen::Vector4d orthonormal_line_;
			Eigen::Matrix3d K_;
		};

		class OthonormalLines2ProjectImagePartFunc : public ceres::SizedCostFunction<2, 4>
		{
		public:
			OthonormalLines2ProjectImagePartFunc(const Eigen::Vector2d& ob_line_s,
				const Eigen::Vector2d& ob_line_e)
			{
				//pnt_3d_ = pnt3d;
				observation_line_s_ = ob_line_s;
				observation_line_e_ = ob_line_e;
				energy_scale_ = 1.0;
				cam_extr_.setOnes();
			}
			virtual ~OthonormalLines2ProjectImagePartFunc() {}
			void SetKMatrix(const Eigen::Matrix3d& k)
			{
				K_ = k;
			}
			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}

			void SetCamExtr(Eigen::Matrix<double, 1, 6> cam_extr)
			{
				cam_extr_ = cam_extr;
			}

			virtual bool Evaluate(double const* const* parameters,
				double* residuals,
				double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				*/
				Eigen::Vector3d rv_w;
				Eigen::Matrix3d r_cw;
				rv_w[0] = cam_extr_(0, 0);
				rv_w[1] = cam_extr_(0, 1);
				rv_w[2] = cam_extr_(0, 2);
				r_cw = HW::AxisAngleToRotationMatrixD(rv_w);
				Eigen::Vector3d t_cw;
				t_cw[0] = cam_extr_(0, 3);
				t_cw[1] = cam_extr_(0, 4);
				t_cw[2] = cam_extr_(0, 5);

				Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
				transform_matrix.block<3, 3>(0, 0) = r_cw;
				transform_matrix.block<3, 1>(0, 3) = t_cw;
				//std::cerr << "transform_matrix: \n" << transform_matrix << std::endl;

				//line in world->so(3): theta1, theta2, theta3; so(2): phi
				Eigen::Vector4d line_otho_w;
				line_otho_w[0] = parameters[0][0];
				line_otho_w[1] = parameters[0][1];
				line_otho_w[2] = parameters[0][2];
				line_otho_w[3] = parameters[0][3];	//phi contain distance

				//othonormal line to plucker line coordinate(n_w = head(3), p times q; d_w = tail(3), q - p)
				Eigen::Matrix<double, 6, 1> line_plu_w = HW::OrthLineToPluckerLineNew(line_otho_w);
				//get plucker transformed matrix(6 * 6)
				Eigen::Matrix<double, 6, 6> plu_transform = HW::GetPluckerTransformMatrix(transform_matrix);
				//std::cerr << "plu_transform: \n" << plu_transform << std::endl;

				//line in world coordinate to line in camera coordinate()
				Eigen::Matrix<double, 6, 1> line_plu_c = plu_transform*line_plu_w;
				Eigen::Vector3d plu_n_c = line_plu_c.head(3);	//n 
				Eigen::Vector3d plu_d_c = line_plu_c.tail(3);
				//get plucker k matrix
				Eigen::Matrix3d plu_k = HW::GetPluckerKMatrix(K_);
				//std::cerr << "plu_k: \n" << plu_k << std::endl;

				Eigen::Vector3d l_i = plu_k*plu_n_c;
				double l_demon = std::sqrt(l_i[0] * l_i[0] + l_i[1] * l_i[1]);
				//std::cerr << "l_i: " << l_i.transpose() << std::endl;

				double s_l = observation_line_s_[0] * l_i[0] + observation_line_s_[1] * l_i[1] + l_i[2];
				double e_l = observation_line_e_[0] * l_i[0] + observation_line_e_[1] * l_i[1] + l_i[2];

				double energy_scale_sqrt = std::sqrt(energy_scale_);

				//if delete the r_demon(or not )
				residuals[0] = energy_scale_sqrt * s_l / l_demon;
				residuals[1] = energy_scale_sqrt * e_l / l_demon;

				Eigen::Matrix<double, 2, 3> jaco_resi_to_li;
				jaco_resi_to_li(0, 0) = -l_i[0] * s_l / std::pow(l_demon, 2) + observation_line_s_[0] / l_demon;
				jaco_resi_to_li(0, 1) = -l_i[1] * s_l / std::pow(l_demon, 2) + observation_line_s_[1] / l_demon;
				jaco_resi_to_li(0, 2) = 1.0 / l_demon;
				jaco_resi_to_li(1, 0) = -l_i[0] * e_l / std::pow(l_demon, 2) + observation_line_e_[0] / l_demon;
				jaco_resi_to_li(1, 1) = -l_i[1] * e_l / std::pow(l_demon, 2) + observation_line_e_[1] / l_demon;
				jaco_resi_to_li(1, 2) = 1.0 / l_demon;
				//std::cerr << "jaco_resi_to_li: \n" << jaco_resi_to_li << std::endl;

				Eigen::Matrix<double, 3, 6> jaco_li_to_Lc;
				jaco_li_to_Lc.block<3, 3>(0, 0) = plu_k;
				jaco_li_to_Lc.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
				//std::cerr << "jaco_li_to_Lc: \n" << jaco_li_to_Lc << std::endl;

				Eigen::Matrix<double, 6, 6> jaco_Lc_to_Lw;
				jaco_Lc_to_Lw = plu_transform;
				//std::cerr << "jaco_Lc_to_Lw: \n" << jaco_Lc_to_Lw << std::endl;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
#if 0
					Eigen::Matrix3d otho_so3_w = HW::GetOrthRMatrixFromPlucker(line_plu_w);
					Eigen::Matrix2d otho_so2_w = HW::GetOrthWFromPlucker(line_plu_w);
					Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J0(jacobians[0]);
					double otho_so2_w1 = otho_so2_w(0, 0);
					double otho_so2_w2 = otho_so2_w(1, 0);

					Eigen::Matrix<double, 6, 4> jaco_Lw_to_Lotho;
					Eigen::Matrix3d so2w1_othoso3_w1_skew = HW::ComputeSkewMatrixFromVector3d(otho_so2_w1*otho_so3_w.col(0));
					Eigen::Matrix3d so2w2_othoso3_w2_skew = HW::ComputeSkewMatrixFromVector3d(otho_so2_w2*otho_so3_w.col(1));

					jaco_Lw_to_Lotho.block<3, 3>(0, 0) = -so2w1_othoso3_w1_skew;
					jaco_Lw_to_Lotho.block<3, 3>(3, 0) = -so2w2_othoso3_w2_skew;
					jaco_Lw_to_Lotho.block<3, 1>(0, 3) = -otho_so2_w2*otho_so3_w.col(0);
					jaco_Lw_to_Lotho.block<3, 1>(3, 3) = otho_so2_w1*otho_so3_w.col(1);
#else
					Eigen::Matrix3d otho_so3_w = HW::GetOrthRMatrixFromPlucker(line_plu_w);
					Eigen::Matrix2d otho_so2_w = HW::GetOrthWFromPlucker(line_plu_w);
					Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J0(jacobians[0]);
					double otho_so2_w1 = otho_so2_w(0, 0);
					double otho_so2_w2 = otho_so2_w(1, 0);
					Eigen::Matrix<double, 6, 4> jaco_Lw_to_Lotho;
					jaco_Lw_to_Lotho.setZero();
					jaco_Lw_to_Lotho.block<3, 1>(0, 1) = -otho_so2_w1*otho_so3_w.col(2);
					jaco_Lw_to_Lotho.block<3, 1>(0, 2) = otho_so2_w1*otho_so3_w.col(1);
					jaco_Lw_to_Lotho.block<3, 1>(0, 3) = -otho_so2_w2*otho_so3_w.col(0);
					jaco_Lw_to_Lotho.block<3, 1>(3, 0) = -otho_so2_w2*otho_so3_w.col(2);
					jaco_Lw_to_Lotho.block<3, 1>(3, 2) = -otho_so2_w2*otho_so3_w.col(0);
					jaco_Lw_to_Lotho.block<3, 1>(3, 3) = otho_so2_w1*otho_so3_w.col(1);
#endif

					Eigen::Matrix<double, 2, 4> jaco_obline_to_otho_line;
					jaco_obline_to_otho_line = jaco_resi_to_li*jaco_li_to_Lc*jaco_Lc_to_Lw*jaco_Lw_to_Lotho;

					J0(0, 0) = energy_scale_sqrt * jaco_obline_to_otho_line(0, 0);
					J0(0, 1) = energy_scale_sqrt * jaco_obline_to_otho_line(0, 1);
					J0(0, 2) = energy_scale_sqrt * jaco_obline_to_otho_line(0, 2);
					J0(0, 3) = energy_scale_sqrt * jaco_obline_to_otho_line(0, 3);
					J0(1, 0) = energy_scale_sqrt * jaco_obline_to_otho_line(1, 0);
					J0(1, 1) = energy_scale_sqrt * jaco_obline_to_otho_line(1, 1);
					J0(1, 2) = energy_scale_sqrt * jaco_obline_to_otho_line(1, 2);
					J0(1, 3) = energy_scale_sqrt * jaco_obline_to_otho_line(1, 3);
				}

				return true;
			}

		private:

			Eigen::Vector2d observation_line_s_;
			Eigen::Vector2d observation_line_e_;
			double energy_scale_;
			Eigen::Matrix<double, 1, 6> cam_extr_;
			Eigen::Matrix3d K_;
		};

		///Energy: plucker line to image line distance, do not know which polygon to attatched 
		class OthonormalLines2PlaneFunc : public ceres::SizedCostFunction<1, 4>
		{
		public:
			OthonormalLines2PlaneFunc(const Eigen::Vector4d& plane_fun)
			{
				//pnt_3d_ = pnt3d;
				w_plane_fun_ = plane_fun;
				energy_scale_ = 1.0;
			}
			virtual ~OthonormalLines2PlaneFunc() {}
			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}
			virtual bool Evaluate(double const* const* parameters,
				double* residuals,
				double** jacobians) const
			{
				/*
				parameters 1 -> (1,4)
				parameter 1: othonormal represatation (theta1, theta2, theta3, phi)
				*/
				//line to plane distance
				//line in world->so(3): theta1, theta2, theta3; so(2): phi
				Eigen::Vector4d line_otho_w;
				line_otho_w[0] = parameters[0][0];
				line_otho_w[1] = parameters[0][1];
				line_otho_w[2] = parameters[0][2];
				line_otho_w[3] = parameters[0][3];	//phi contain distance
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				//how to compute the distance between line an plane
				double distance2plane = 1.0;	//to do next...
				residuals[0] = energy_scale_sqrt *  distance2plane;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J0(jacobians[0]);
					//how to compute the jacobean?

				}
				return true;
			}
		private:
			//Ax+By+Cz+D = 0
			Eigen::Vector4d w_plane_fun_;
			double energy_scale_;
		};


		class Pnts3D2PlaneCostFunc : public ceres::SizedCostFunction<1, 3>
		{
		public:
			Pnts3D2PlaneCostFunc(double* plane_fun)
			{
				plane_fun_[0] = plane_fun[0];
				plane_fun_[1] = plane_fun[1];
				plane_fun_[2] = plane_fun[2];
				plane_fun_[3] = plane_fun[3];
				energy_scale_ = 1.0;
			}
			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}
			virtual ~Pnts3D2PlaneCostFunc() {};
			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				Eigen::Vector3d pw;
				pw[0] = parameters[0][0];
				pw[1] = parameters[0][1];
				pw[2] = parameters[0][2];
				//compute demon, should do 
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				residuals[0] = energy_scale_sqrt * (plane_fun_[0]* pw[0] + plane_fun_[1]*pw[1] + plane_fun_[2]*pw[2] + plane_fun_[3]);
				if (jacobians != NULL && jacobians[0] != NULL)
				{
					jacobians[0][0] = energy_scale_sqrt * plane_fun_[0];
					jacobians[0][1] = energy_scale_sqrt * plane_fun_[1];
					jacobians[0][2] = energy_scale_sqrt * plane_fun_[2];
				}
				return true;
			}
		private:
			Eigen::Vector4d plane_fun_;
			double energy_scale_;
		};

		class PosePnt3DToImgLines2ProjectFunc : public ceres::SizedCostFunction<1, 6, 3>
		{
		public:
			PosePnt3DToImgLines2ProjectFunc(const Eigen::Vector2d& ob_line_s,
				const Eigen::Vector2d& ob_line_e)
			{
				//pnt_3d_ = pnt3d;
				observation_line_s_ = ob_line_s;
				observation_line_e_ = ob_line_e;
				energy_scale_ = 1.0;
			}
			virtual ~PosePnt3DToImgLines2ProjectFunc() {}
			void SetKMatrix(const Eigen::Matrix3d& k)
			{
				K_ = k;
			}
			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}
			virtual bool Evaluate(double const* const* parameters,
				double* residuals,
				double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				parameters 1 -> (1,3)
				parameter 1: pnt3d parameters(x[0],x[1],x[2])
				*/
				/*Eigen::Vector3d rv_w;
				Eigen::Matrix3d r_cw;
				rv_w[0] = parameters[0][0];
				rv_w[1] = parameters[0][1];
				rv_w[2] = parameters[0][2];
				r_cw = HW::AxisAngleToRotationMatrixD(rv_w);*/
				double rv_w[3];
				double r_cw[9];
				rv_w[0] = parameters[0][0];
				rv_w[1] = parameters[0][1];
				rv_w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(rv_w, r_cw);
				Eigen::Vector3d t_cw;
				t_cw[0] = parameters[0][3];
				t_cw[1] = parameters[0][4];
				t_cw[2] = parameters[0][5];
				Eigen::Vector3d p3d_w;
				p3d_w[0] = parameters[1][0];
				p3d_w[1] = parameters[1][1];
				p3d_w[2] = parameters[1][2];
				//p3d_w[3] = 1.0;
				double const rx = r_cw[0] * p3d_w[0] + r_cw[1] * p3d_w[1] + r_cw[2] * p3d_w[2];	//row major
				double const ry = r_cw[3] * p3d_w[0] + r_cw[4] * p3d_w[1] + r_cw[5] * p3d_w[2];	//row major
				double const rz = r_cw[6] * p3d_w[0] + r_cw[7] * p3d_w[1] + r_cw[8] * p3d_w[2];	//row major
				double const xc = rx + t_cw[0];	//in camera coordinate
				double const yc = ry + t_cw[1];	//in camera coordinate
				double const zc = rz + t_cw[2];	//in camera coordinate
				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy
				//print_cam_test_r_info(r_cw);
				//std::cerr << "fx, fy, cx, cy: " << fx << ", " << fy << ", " << cx << ", " << cy << std::endl;
				double const iu = fx * xc / zc + cx;	//image pixel
				double const iv = fy * yc / zc + cy;	//image pixel
				/*std::cerr << "observation_[0]->(iu), observation_[1]->(iv): " << observation_[0] << "->(" << iu << " )" << ", "
				<< observation_[1] << "->(" << iv << " )" << std::endl;*/
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				//uvi to observation_line_ dist
				Eigen::Vector3d ob_line = HW::GetLineFuncFromLine2Pnts2d(observation_line_s_, observation_line_e_);
				double denorm = std::sqrt(ob_line[0] * ob_line[0] + ob_line[1] * ob_line[1]);
				residuals[0] = energy_scale_sqrt * (iu*ob_line[0] + iv*ob_line[1] + ob_line[2]) / denorm;
				
				/*residuals[0] = (-observation_[0] + iu) * energy_scale_sqrt;
				residuals[1] = (-observation_[1] + iv) * energy_scale_sqrt;*/

				//Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
				//transform_matrix.block<3, 3>(0, 0) = r_cw;
				//transform_matrix.block<3, 1>(0, 3) = t_cw;
				//Eigen::Vector4d p3d_c = transform_matrix*p3d_w;
				//Eigen::Vector3d p3d_normal_c =
				//	Eigen::Vector3d(p3d_c[0] / p3d_c[2], p3d_c[1] / p3d_c[2], 1.0);
				//Eigen::Vector3d uvi = K_*p3d_normal_c;
				////uvi to observation_line_ dist
				//Eigen::Vector3d ob_line = HW::GetLineFuncFromLine2Pnts2d(observation_line_s_, observation_line_e_);
				//double energy_scale_sqrt = std::sqrt(energy_scale_);
				////dist
				//double denorm = std::sqrt(ob_line[0] * ob_line[0] + ob_line[1] * ob_line[1]);
				//residuals[0] = energy_scale_sqrt * uvi.dot(ob_line) / denorm;

				/*Eigen::Matrix<double, 1, 2> jaco_residual_to_pi;
				jaco_residual_to_pi(0, 0) = ob_line[0] / denorm;
				jaco_residual_to_pi(0, 1) = ob_line[1] / denorm;
				Eigen::Matrix<double, 2, 3> k_fi_matrix = Eigen::Matrix<double, 2, 3>::Zero();
				k_fi_matrix(0, 0) = K_(0, 0);
				k_fi_matrix(1, 1) = K_(1, 1);
				k_fi_matrix(0, 2) = K_(0, 2);
				k_fi_matrix(1, 2) = K_(1, 2);
				Eigen::Matrix<double, 3, 3> k_fc_matrix;
				k_fc_matrix.setZero();
				k_fc_matrix(0, 0) = 1.0 / p3d_c[2];
				k_fc_matrix(1, 1) = 1.0 / p3d_c[2];
				k_fc_matrix(0, 2) = -p3d_c[0] / std::pow(p3d_c[2], 2);
				k_fc_matrix(1, 2) = -p3d_c[1] / std::pow(p3d_c[2], 2);
				Eigen::Matrix<double, 2, 3> jaco_pi_to_pc;
				jaco_pi_to_pc = k_fi_matrix*k_fc_matrix;*/

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> J0(jacobians[0]);
					Eigen::Matrix<double, 2, 6> jaco_uvi_to_wrt;
					//compute the jacobians
					//rx = r_0P; ry = r_1P; rz = r_2P
					jaco_uvi_to_wrt(0, 0) = (-fx*xc*ry / (zc*zc));
					jaco_uvi_to_wrt(0, 1) = (fx*rz / zc + (fx*xc*rx) / (zc*zc));
					jaco_uvi_to_wrt(0, 2) = (-fx*ry / zc);
					jaco_uvi_to_wrt(0, 3) = (fx / zc);
					jaco_uvi_to_wrt(0, 4) = 0.0;
					jaco_uvi_to_wrt(0, 5) = (-fx*xc / (zc*zc));

					jaco_uvi_to_wrt(1, 0) = (-fy*rz / zc - (fy*yc*ry) / (zc*zc));
					jaco_uvi_to_wrt(1, 1) = (fy*yc*rx / (zc*zc));
					jaco_uvi_to_wrt(1, 2) = (fy*rx / zc);
					jaco_uvi_to_wrt(1, 3) = 0.0;
					jaco_uvi_to_wrt(1, 4) = (fy / zc);
					jaco_uvi_to_wrt(1, 5) = (-fy*yc / (zc*zc));

					J0(0, 0) = (ob_line[0] * jaco_uvi_to_wrt(0, 0) + ob_line[1] * jaco_uvi_to_wrt(1, 0)) * energy_scale_sqrt / denorm;
					J0(0, 1) = (ob_line[0] * jaco_uvi_to_wrt(0, 1) + ob_line[1] * jaco_uvi_to_wrt(1, 1)) * energy_scale_sqrt / denorm;
					J0(0, 2) = (ob_line[0] * jaco_uvi_to_wrt(0, 2) + ob_line[1] * jaco_uvi_to_wrt(1, 2)) * energy_scale_sqrt / denorm;
					J0(0, 3) = (ob_line[0] * jaco_uvi_to_wrt(0, 3) + ob_line[1] * jaco_uvi_to_wrt(1, 3)) * energy_scale_sqrt / denorm;
					J0(0, 4) = (ob_line[0] * jaco_uvi_to_wrt(0, 4) + ob_line[1] * jaco_uvi_to_wrt(1, 4)) * energy_scale_sqrt / denorm;
					J0(0, 5) = (ob_line[0] * jaco_uvi_to_wrt(0, 5) + ob_line[1] * jaco_uvi_to_wrt(1, 5)) * energy_scale_sqrt / denorm;

					//Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> J0(jacobians[0]);
					////compute the jacobians
					//Eigen::Matrix<double, 3, 6> jaco_pc_to_rwt;
					//Eigen::Matrix<double, 3, 3> jaco_w2r;
					//jaco_w2r.setIdentity();
					//Eigen::Vector3d tpr;
					//tpr[0] = p3d_c[0];
					//tpr[1] = p3d_c[1];
					//tpr[2] = p3d_c[2];
					//Eigen::Matrix3d jaco_w2t = -HW::ComputeSkewMatrixFromVector3d(tpr);
					//jaco_pc_to_rwt.block<3, 3>(0, 0) = jaco_w2r;
					//jaco_pc_to_rwt.block<3, 3>(0, 3) = jaco_w2t;
					//Eigen::Matrix<double, 1, 6> jaco_residual_to_rwt;
					//jaco_residual_to_rwt = jaco_residual_to_pi*jaco_pi_to_pc*jaco_pc_to_rwt;

					/*J0(0, 0) = energy_scale_sqrt * jaco_residual_to_rwt(0, 0);
					J0(0, 1) = energy_scale_sqrt * jaco_residual_to_rwt(0, 1);
					J0(0, 2) = energy_scale_sqrt * jaco_residual_to_rwt(0, 2);
					J0(0, 3) = energy_scale_sqrt * jaco_residual_to_rwt(0, 3);
					J0(0, 4) = energy_scale_sqrt * jaco_residual_to_rwt(0, 4);
					J0(0, 5) = energy_scale_sqrt * jaco_residual_to_rwt(0, 5);*/

				}
				if (jacobians != NULL && jacobians[1] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J1(jacobians[1]);

					Eigen::Matrix<double, 2, 3> jaco_uvi_to_pc;
					jaco_uvi_to_pc(0, 0) = (fx*r_cw[0] / zc - fx*xc*r_cw[6] / (zc*zc));
					jaco_uvi_to_pc(0, 1) = (fx*r_cw[1] / zc - fx*xc*r_cw[7] / (zc*zc));
					jaco_uvi_to_pc(0, 2) = (fx*r_cw[2] / zc - fx*xc*r_cw[8] / (zc*zc));
					jaco_uvi_to_pc(1, 0) = (fy*r_cw[3] / zc - fy*yc*r_cw[6] / (zc*zc));
					jaco_uvi_to_pc(1, 1) = (fy*r_cw[4] / zc - fy*yc*r_cw[7] / (zc*zc));
					jaco_uvi_to_pc(1, 2) = (fy*r_cw[5] / zc - fy*yc*r_cw[8] / (zc*zc));
					J1(0, 0) = (ob_line[0] * jaco_uvi_to_pc(0, 0) + ob_line[1] * jaco_uvi_to_pc(1, 0)) * energy_scale_sqrt / denorm;
					J1(0, 1) = (ob_line[0] * jaco_uvi_to_pc(0, 1) + ob_line[1] * jaco_uvi_to_pc(1, 1)) * energy_scale_sqrt / denorm;
					J1(0, 2) = (ob_line[0] * jaco_uvi_to_pc(0, 2) + ob_line[1] * jaco_uvi_to_pc(1, 2)) * energy_scale_sqrt / denorm;

					/*Eigen::Matrix<double, 3, 3> jaco_pc_to_pw;
					jaco_pc_to_pw.block<3, 3>(0, 0) = r_cw;
					Eigen::Matrix<double, 1, 3> jaco_residual_to_pw;
					jaco_residual_to_pw = jaco_residual_to_pi*jaco_pi_to_pc*jaco_pc_to_pw;*/
					/*J1(0, 0) = energy_scale_sqrt * jaco_residual_to_pw(0, 0);
					J1(0, 1) = energy_scale_sqrt * jaco_residual_to_pw(0, 1);
					J1(0, 2) = energy_scale_sqrt * jaco_residual_to_pw(0, 2);*/

				}
				return true;
			}

		private:
			//double x_;
			Eigen::Vector2d observation_line_s_;
			Eigen::Vector2d observation_line_e_;
			Eigen::Matrix3d K_;
			double energy_scale_;
		};

		class PosePnt3DToImgLines2ProjectFuncOld : public ceres::SizedCostFunction<1, 6, 3>
		{
		public:
			PosePnt3DToImgLines2ProjectFuncOld(const Eigen::Vector2d& ob_line_s,
				const Eigen::Vector2d& ob_line_e)
			{
				//pnt_3d_ = pnt3d;
				observation_line_s_ = ob_line_s;
				observation_line_e_ = ob_line_e;
				energy_scale_ = 1.0;
			}
			virtual ~PosePnt3DToImgLines2ProjectFuncOld() {}
			void SetKMatrix(const Eigen::Matrix3d& k)
			{
				K_ = k;
			}
			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}
			virtual bool Evaluate(double const* const* parameters,
				double* residuals,
				double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				parameters 1 -> (1,3)
				parameter 1: pnt3d parameters(x[0],x[1],x[2])
				*/
				Eigen::Vector3d rv_w;
				Eigen::Matrix3d r_cw;
				rv_w[0] = parameters[0][0];
				rv_w[1] = parameters[0][1];
				rv_w[2] = parameters[0][2];
				r_cw = HW::AxisAngleToRotationMatrixD(rv_w);
				Eigen::Vector3d t_cw;
				t_cw[0] = parameters[0][3];
				t_cw[1] = parameters[0][4];
				t_cw[2] = parameters[0][5];
				Eigen::Vector4d p3d_w;
				p3d_w[0] = parameters[1][0];
				p3d_w[1] = parameters[1][1];
				p3d_w[2] = parameters[1][2];
				p3d_w[3] = 1.0;
				Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
				transform_matrix.block<3, 3>(0, 0) = r_cw;
				transform_matrix.block<3, 1>(0, 3) = t_cw;
				Eigen::Vector4d p3d_c = transform_matrix*p3d_w;
				Eigen::Vector3d p3d_normal_c = 
					Eigen::Vector3d(p3d_c[0] / p3d_c[2], p3d_c[1] / p3d_c[2], 1.0);
				Eigen::Vector3d uvi = K_*p3d_normal_c;
				//uvi to observation_line_ dist
				Eigen::Vector3d ob_line = HW::GetLineFuncFromLine2Pnts2d(observation_line_s_, observation_line_e_);
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				//dist
				double denorm = std::sqrt(ob_line[0] * ob_line[0] + ob_line[1] * ob_line[1]);
				residuals[0] = energy_scale_sqrt * uvi.dot(ob_line) / denorm;

				Eigen::Matrix<double, 1, 2> jaco_residual_to_pi;
				jaco_residual_to_pi(0, 0) = ob_line[0] / denorm;
				jaco_residual_to_pi(0, 1) = ob_line[1] / denorm;

				Eigen::Matrix<double, 2, 3> k_fi_matrix = Eigen::Matrix<double, 2, 3>::Zero();
				k_fi_matrix(0, 0) = K_(0, 0);
				k_fi_matrix(1, 1) = K_(1, 1);
				k_fi_matrix(0, 2) = K_(0, 2);
				k_fi_matrix(1, 2) = K_(1, 2);
				Eigen::Matrix<double, 3, 3> k_fc_matrix;
				k_fc_matrix.setZero();
				k_fc_matrix(0, 0) = 1.0 / p3d_c[2];
				k_fc_matrix(1, 1) = 1.0 / p3d_c[2];
				k_fc_matrix(0, 2) = -p3d_c[0] / std::pow(p3d_c[2], 2);
				k_fc_matrix(1, 2) = -p3d_c[1] / std::pow(p3d_c[2], 2);
				Eigen::Matrix<double, 2, 3> jaco_pi_to_pc;
				jaco_pi_to_pc = k_fi_matrix*k_fc_matrix;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> J0(jacobians[0]);
					//compute the jacobians
					Eigen::Matrix<double, 3, 6> jaco_pc_to_rwt;
					Eigen::Matrix<double, 3, 3> jaco_w2r;
					jaco_w2r.setIdentity();
					Eigen::Vector3d tpr;
					tpr[0] = p3d_c[0];
					tpr[1] = p3d_c[1];
					tpr[2] = p3d_c[2];
					Eigen::Matrix3d jaco_w2t = -HW::ComputeSkewMatrixFromVector3d(tpr);
					jaco_pc_to_rwt.block<3, 3>(0, 0) = jaco_w2r;
					jaco_pc_to_rwt.block<3, 3>(0, 3) = jaco_w2t;
					
					Eigen::Matrix<double, 1, 6> jaco_residual_to_rwt;
					jaco_residual_to_rwt = jaco_residual_to_pi*jaco_pi_to_pc*jaco_pc_to_rwt;

					J0(0, 0) = energy_scale_sqrt * jaco_residual_to_rwt(0, 0);
					J0(0, 1) = energy_scale_sqrt * jaco_residual_to_rwt(0, 1);
					J0(0, 2) = energy_scale_sqrt * jaco_residual_to_rwt(0, 2);
					J0(0, 3) = energy_scale_sqrt * jaco_residual_to_rwt(0, 3);
					J0(0, 4) = energy_scale_sqrt * jaco_residual_to_rwt(0, 4);
					J0(0, 5) = energy_scale_sqrt * jaco_residual_to_rwt(0, 5);

				}
				if (jacobians != NULL && jacobians[1] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J1(jacobians[1]);
					
					Eigen::Matrix<double, 3, 3> jaco_pc_to_pw;
					jaco_pc_to_pw.block<3, 3>(0, 0) = r_cw;
					Eigen::Matrix<double, 1, 3> jaco_residual_to_pw;
					jaco_residual_to_pw = jaco_residual_to_pi*jaco_pi_to_pc*jaco_pc_to_pw;
					
					J1(0, 0) = energy_scale_sqrt * jaco_residual_to_pw(0, 0);
					J1(0, 1) = energy_scale_sqrt * jaco_residual_to_pw(0, 1);
					J1(0, 2) = energy_scale_sqrt * jaco_residual_to_pw(0, 2);
				}
				return true;
			}

		private:
			//double x_;
			Eigen::Vector2d observation_line_s_;
			Eigen::Vector2d observation_line_e_;
			Eigen::Matrix3d K_;
			double energy_scale_;
		};

		//
		class Plane2Pnts3DCostFunc :public ceres::SizedCostFunction<1, 4>
		{
		public:
			Plane2Pnts3DCostFunc(const Eigen::Vector3d& pnt)
			{
				w_pnt_[0] = pnt[0];
				w_pnt_[1] = pnt[1];
				w_pnt_[2] = pnt[2];
				energy_scale_ = 1.0;
			}

			void SetEnergyScale(double energy_s)
			{
				energy_scale_ = energy_s;
			}

			virtual ~Plane2Pnts3DCostFunc() {};
			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				const double a = parameters[0][0];
				const double b = parameters[0][1];
				const double c = parameters[0][2];
				const double d = parameters[0][3];
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				residuals[0] = energy_scale_sqrt *(a*w_pnt_[0] + b*w_pnt_[1] + c*w_pnt_[2] + d);
				if (jacobians != NULL && jacobians[0] != NULL)
				{
					jacobians[0][0] = energy_scale_sqrt * w_pnt_[0];
					jacobians[0][1] = energy_scale_sqrt * w_pnt_[1];
					jacobians[0][2] = energy_scale_sqrt * w_pnt_[2];
					jacobians[0][3] = energy_scale_sqrt * 1.0;
				}
				return true;
			}
		private:
			Eigen::Vector3d w_pnt_;
			double energy_scale_;
		};

		HWSceneBundleAdjustment::HWSceneBundleAdjustment()
		{
			bundle_max_iteration_num_per_epoch_ = 500;
			bundle_max_iteration_ = 2;
			epoch_max_number_ = 1;
			reproject_error_average_threhold_ = 0.5;
			bundle_cams_pnts_loss_value_ = std::numeric_limits<double>::max();
			bundle_plane_lines_loss_value_ = std::numeric_limits<double>::max();

			pnts_to_images_pnts_lambda_ = 1.0;
			pnts_to_planes_lambda_ = 100.0;
			lines_inter_to_polygons_lines_lambda_ = 10.0;	//change its
			lines_inter_to_images_lines_lambda_ = 10.0;
			lines_to_polygons_lines_lambda_ = 100000.0;
			lines_to_images_lines_lambda_ = 1.0;
			polygons_to_lines3d_lambda_ = 1.0;
		}

		void HWSceneBundleAdjustment::setBundleOptimizationIterNum(int iter_max_num)
		{
			bundle_max_iteration_ = iter_max_num;
		}

		void HWSceneBundleAdjustment::setBundleReprojectAverageError(double average_error)
		{
			reproject_error_average_threhold_ = average_error;
		}

		void HWSceneBundleAdjustment::setCameras(const std::vector<CameraParams>& cams)
		{
			cams_.resize(cams.size());
			for (int i = 0; i < cams.size(); ++i)
			{
				cams_[i] = cams[i];
			}
		}

		void HWSceneBundleAdjustment::setPoints(const std::vector<Point3D>& points)
		{
			points_3d_.resize(points.size());
			for (int i = 0; i < points.size(); ++i)
			{
				points_3d_[i] = points[i];
			}
		}

		void HWSceneBundleAdjustment::setPluckerLine3DPoints(const std::vector<PluckerLinePoint3D>& plucker_lines)
		{
			std::cerr << "to do next..." << std::endl;
		}

		void HWSceneBundleAdjustment::setLine3DPoints(const std::vector<LinePoint3D>& lines_pnts)
		{
			lines_points_3d_.resize(lines_pnts.size());
			for (int i = 0; i < lines_pnts.size(); ++i)
			{
				//
				lines_points_3d_[i] = lines_pnts[i];
			}
			lines_plucker_scale_3d_.resize(lines_pnts.size(), 1.0);
		}

		void HWSceneBundleAdjustment::setObservations(const std::vector<PntObservation>& pnts_observations)
		{
			pnts_observations_.resize(pnts_observations.size());
			for (int i = 0; i < pnts_observations.size(); ++i)
			{
				pnts_observations_[i] = pnts_observations[i];
			}
		}

		void HWSceneBundleAdjustment::setLinesObservations(const std::vector<LineObservation>& lines_observations)
		{
			lines_observations_.resize(lines_observations.size());
			for (int i = 0; i < lines_observations.size(); ++i)
			{
				lines_observations_[i] = lines_observations[i];
			}
		}

		void HWSceneBundleAdjustment::setPlanes(const std::vector<Plane3D>& planes)
		{
			planes_.resize(planes.size());
			for (int i = 0; i < planes_.size(); ++i)
			{
				planes_[i] = planes[i];
			}
		}

		const std::vector<CameraParams>& HWSceneBundleAdjustment::GetCameras()
		{
			return cams_;
		}

		const std::vector<Point3D>& HWSceneBundleAdjustment::GetPoints()
		{
			return points_3d_;
		}

		const std::vector<LinePoint3D>& HWSceneBundleAdjustment::GetLine3DPoints()
		{
			return lines_points_3d_;
		}

		const std::vector<PntObservation>& HWSceneBundleAdjustment::GetObservations()
		{
			return pnts_observations_;
		}

		const std::vector<LineObservation>& HWSceneBundleAdjustment::GetLinesObservations()
		{
			return lines_observations_;
		}

		void HWSceneBundleAdjustment::UpdatePreparedBundleData()
		{
			std::cerr << "------start update prepared bundle data------" << std::endl;
			UpdateCameraParamPartToCameraExtr();
			UpdatePoint3DPartToWorldPnt();
			UpdateLinePoint3DPart2LineWorldPnt();
			UpdatePlanes2WorldPlanes();

#if UsePluckerLineMethod
			//update 
			UpdateLineWorldPnt2LineWorldOthoLine();
			UpdatePlaneLineWorldPnt2PlaneLineWorldOthoLine();
#endif
			////test it
			//int ce_cams_num = cams_extrs_.size();
			//int ce_w_num = w_pnts_.size();
			//int ce_w_lines_num = w_lines_.size();
			//std::cerr << "ce_cams_num: " << ce_cams_num << std::endl;
			//std::cerr << "ce_w_num: " << ce_w_num << std::endl;
			//std::cerr << "ce_w_lines_num: " << ce_w_lines_num << std::endl;
			////end test
			std::cerr << "------end update the prepared bundle data------" << std::endl;
		}

		void HWSceneBundleAdjustment::UpdateCameraParamPartToCameraExtr()
		{
			cams_extrs_.resize(cams_.size());
			//R to cam vector
			for (int i = 0; i < cams_.size(); ++i)
			{
				CameraParams cam = cams_[i];
				Eigen::Matrix4d cam_extr = cam.cam_extr_;
				Eigen::Matrix3d cam_r = cam_extr.topLeftCorner(3, 3);
				Eigen::Vector3d cam_t = cam_extr.topRightCorner(3,1);
				Eigen::Vector3d camrv = HW::RotationMatrixToAxisAngleD(cam_r);
				cams_extrs_[i].cam_extr_[0] = camrv[0];
				cams_extrs_[i].cam_extr_[1] = camrv[1];
				cams_extrs_[i].cam_extr_[2] = camrv[2];
				cams_extrs_[i].cam_extr_[3] = cam_t[0];
				cams_extrs_[i].cam_extr_[4] = cam_t[1];
				cams_extrs_[i].cam_extr_[5] = cam_t[2];
			}
		}

		void HWSceneBundleAdjustment::UpdatePoint3DPartToWorldPnt()
		{
			w_pnts_.resize(points_3d_.size());
			for (int i = 0; i < points_3d_.size(); ++i)
			{
				w_pnts_[i].plane_idxs_ = points_3d_[i].plane_idxs_;
				w_pnts_[i].pos_[0] = points_3d_[i].pos_[0];
				w_pnts_[i].pos_[1] = points_3d_[i].pos_[1];
				w_pnts_[i].pos_[2] = points_3d_[i].pos_[2];
			}
		}

		void HWSceneBundleAdjustment::UpdateLinePoint3DPart2LineWorldPnt()
		{
			w_lines_.clear();
			w_planes_lines_.clear();
			for (int i = 0; i < lines_points_3d_.size(); ++i)
			{
				if (lines_points_3d_[i].is_plane_line_)
				{
					//std::cerr << "UpdateLinePoint3DPart2LineWorldPnt i: " << i << std::endl;
					std::pair<int, PlaneInterLineWorldPnt> tmp_line;
					tmp_line.first = i;
					int plane_idx_num = static_cast<int>(lines_points_3d_[i].plane_idxs_.size());
					//std::cerr << "plane_idx_num: " << plane_idx_num << std::endl;
					tmp_line.second.plane_idxs_ = lines_points_3d_[i].plane_idxs_;
					tmp_line.second.pos_s_[0] = lines_points_3d_[i].pos_s_[0];
					tmp_line.second.pos_s_[1] = lines_points_3d_[i].pos_s_[1];
					tmp_line.second.pos_s_[2] = lines_points_3d_[i].pos_s_[2];
					tmp_line.second.pos_e_[0] = lines_points_3d_[i].pos_e_[0];
					tmp_line.second.pos_e_[1] = lines_points_3d_[i].pos_e_[1];
					tmp_line.second.pos_e_[2] = lines_points_3d_[i].pos_e_[2];
					w_planes_lines_.emplace_back(tmp_line);
				}
				else
				{
					//to be plucker coordinate. to do next.
					std::pair<int, LineWorldPnt> tmp_line;
					tmp_line.first = i;
					tmp_line.second.plane_idxs_ = lines_points_3d_[i].plane_idxs_;
					tmp_line.second.pos_s_[0] = lines_points_3d_[i].pos_s_[0];
					tmp_line.second.pos_s_[1] = lines_points_3d_[i].pos_s_[1];
					tmp_line.second.pos_s_[2] = lines_points_3d_[i].pos_s_[2];
					tmp_line.second.pos_e_[0] = lines_points_3d_[i].pos_e_[0];
					tmp_line.second.pos_e_[1] = lines_points_3d_[i].pos_e_[1];
					tmp_line.second.pos_e_[2] = lines_points_3d_[i].pos_e_[2];
					w_lines_.emplace_back(tmp_line);
				}
			}
		}
		
		void HWSceneBundleAdjustment::UpdatePlanes2WorldPlanes()
		{
			w_planes_.resize(planes_.size());
			for (int i = 0; i < planes_.size(); ++i)
			{
				w_planes_[i].f_[0] = planes_[i].f_[0];
				w_planes_[i].f_[1] = planes_[i].f_[1];
				w_planes_[i].f_[2] = planes_[i].f_[2];
				w_planes_[i].f_[3] = planes_[i].f_[3];
			}
		}

		void HWSceneBundleAdjustment::UpdateLineWorldPnt2LineWorldOthoLine()
		{
			int wline_num = static_cast<int> (w_lines_.size());
			w_lines_otho_lines_.resize(wline_num);
			for (int i = 0; i < w_lines_otho_lines_.size(); ++i)
			{
				Eigen::Vector3d line_s; 
				line_s[0] = w_lines_[i].second.pos_s_[0];
				line_s[1] = w_lines_[i].second.pos_s_[1];
				line_s[2] = w_lines_[i].second.pos_s_[2];
				Eigen::Vector3d line_e;
				line_e[0] = w_lines_[i].second.pos_e_[0];
				line_e[1] = w_lines_[i].second.pos_e_[1];
				line_e[2] = w_lines_[i].second.pos_e_[2];
				//to plucker lines
				Eigen::Vector3d ld = line_e - line_s;
				Eigen::Vector3d ln = line_s.cross(line_e);
				////normlize the plucker lines
				//Eigen::Vector3d ldn = ld / ld.norm();
				//Eigen::Vector3d lnn = ln / ld.norm();
				Eigen::Matrix<double, 6, 1> line_plu;
				line_plu.head(3) = ln;
				line_plu.tail(3) = ld;
				//plucker line to otho line
				Eigen::Matrix<double, 6, 1> line_plu_normal = line_plu.normalized();
				double plu_scale = line_plu.norm();
				Eigen::Vector4d otho_line = HW::PluckerLineToOrthLine(line_plu_normal);
				w_lines_otho_lines_[i].first = w_lines_[i].first;
				lines_plucker_scale_3d_[w_lines_otho_lines_[i].first] = plu_scale;
				w_lines_otho_lines_[i].second.line_otho_[0] = otho_line[0];
				w_lines_otho_lines_[i].second.line_otho_[1] = otho_line[1];
				w_lines_otho_lines_[i].second.line_otho_[2] = otho_line[2];
				w_lines_otho_lines_[i].second.line_otho_[3] = otho_line[3];
				w_lines_otho_lines_[i].second.plane_idxs_ = w_lines_[i].second.plane_idxs_;
			}
		}

		void HWSceneBundleAdjustment::UpdatePlaneLineWorldPnt2PlaneLineWorldOthoLine()
		{
			int wline_num = static_cast<int> (w_planes_lines_.size());
			w_plane_lines_otho_lines_.resize(wline_num);
			for (int i = 0; i < w_plane_lines_otho_lines_.size(); ++i)
			{
				Eigen::Vector3d line_s;
				line_s[0] = w_planes_lines_[i].second.pos_s_[0];
				line_s[1] = w_planes_lines_[i].second.pos_s_[1];
				line_s[2] = w_planes_lines_[i].second.pos_s_[2];
				Eigen::Vector3d line_e;
				line_e[0] = w_planes_lines_[i].second.pos_e_[0];
				line_e[1] = w_planes_lines_[i].second.pos_e_[1];
				line_e[2] = w_planes_lines_[i].second.pos_e_[2];
				//to plucker lines
				Eigen::Vector3d ld = line_e - line_s;
				Eigen::Vector3d ln = line_s.cross(line_e);
				////normlize the plucker lines
				//Eigen::Vector3d ldn = ld / ld.norm();
				//Eigen::Vector3d lnn = ln / ld.norm();
				Eigen::Matrix<double, 6, 1> line_plu;
				line_plu.head(3) = ln;
				line_plu.tail(3) = ld;
				//plucker line to otho line
				Eigen::Matrix<double, 6, 1> line_plu_normal = line_plu.normalized();
				double plu_scale = line_plu.norm();
				Eigen::Vector4d otho_line = HW::PluckerLineToOrthLine(line_plu_normal);
				w_plane_lines_otho_lines_[i].first = w_planes_lines_[i].first;
				lines_plucker_scale_3d_[w_plane_lines_otho_lines_[i].first] = plu_scale;
				w_plane_lines_otho_lines_[i].second.line_otho_[0] = otho_line[0];
				w_plane_lines_otho_lines_[i].second.line_otho_[1] = otho_line[1];
				w_plane_lines_otho_lines_[i].second.line_otho_[2] = otho_line[2];
				w_plane_lines_otho_lines_[i].second.line_otho_[3] = otho_line[3];
				w_plane_lines_otho_lines_[i].second.plane_idxs_ = w_planes_lines_[i].second.plane_idxs_;
			}
		}

		void HWSceneBundleAdjustment::UpdateBundleDataToData()
		{
			std::cerr << "start to update bundle data to data..." << std::endl;
			UpdateBundleCameraToCameraExtr();
			UpdateBundlePnt3DToPnt3D();
#if UsePluckerLineMethod
			UpdateOthoLine2BundleLineWorldLine();	//to change next... delete w_plane_lines_otho_lines_
#endif
			UpdateBundleLineWorldPnts2LinePnts();
			UpdateBunldePlanesToPlanes();
			std::cerr << "end update bundle data to data..." << std::endl;
		}

		void HWSceneBundleAdjustment::UpdateBundleCameraToCameraExtr()
		{
			for (int i = 0; i < cams_extrs_.size(); ++i)
			{
				Eigen::Vector3d rv;
				Eigen::Vector3d rt;
				for (int j = 0; j < 3; ++j)
				{
					rv[j] = cams_extrs_[i].cam_extr_[j];
				}
				for (int j = 0; j < 3; ++j)
				{
					rt[j] = cams_extrs_[i].cam_extr_[j + 3];
				}
				Eigen::Matrix3d R = HW::AxisAngleToRotationMatrixD(rv);	//important
				cams_[i].cam_extr_(0, 0) = R(0, 0); cams_[i].cam_extr_(0, 1) = R(0, 1); cams_[i].cam_extr_(0, 2) = R(0, 2);
				cams_[i].cam_extr_(1, 0) = R(1, 0); cams_[i].cam_extr_(1, 1) = R(1, 1); cams_[i].cam_extr_(1, 2) = R(1, 2);
				cams_[i].cam_extr_(2, 0) = R(2, 0); cams_[i].cam_extr_(2, 1) = R(2, 1); cams_[i].cam_extr_(2, 2) = R(2, 2);
				cams_[i].cam_extr_(0, 3) = rt[0]; cams_[i].cam_extr_(1, 3) = rt[1]; cams_[i].cam_extr_(2, 3) = rt[2];
				cams_[i].cam_extr_(3, 3) = 1.0;
			}
		}

		void HWSceneBundleAdjustment::UpdateBundlePnt3DToPnt3D()
		{
			for (int i = 0; i < w_pnts_.size(); ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					points_3d_[i].pos_[j] = w_pnts_[i].pos_[j];
				}
			}
		}

		void HWSceneBundleAdjustment::UpdateOthoLine2BundleLineWorldLine()
		{
			for (int i = 0; i < w_lines_otho_lines_.size(); ++i)
			{
				//
				Eigen::Matrix<double, 6, 1> plu_line;
				Eigen::Vector4d line_otho_v;
				line_otho_v[0] = w_lines_otho_lines_[i].second.line_otho_[0];
				line_otho_v[1] = w_lines_otho_lines_[i].second.line_otho_[1];
				line_otho_v[2] = w_lines_otho_lines_[i].second.line_otho_[2];
				line_otho_v[3] = w_lines_otho_lines_[i].second.line_otho_[3];
				plu_line = HW::OrthLineToPluckerLineNew(line_otho_v);
				//plu_line to d 
				Eigen::Matrix<double, 6, 1> plu_line_d_normlize;
				plu_line_d_normlize = HW::PluckerLineDirNormalized(plu_line);
				//line pnt project to plu_line_d_normlize
				Eigen::Vector3d ln = plu_line_d_normlize.head(3);
				Eigen::Vector3d ld = plu_line_d_normlize.tail(3);
				Eigen::Vector3d ls3d;
				ls3d[0] = w_lines_[i].second.pos_s_[0];
				ls3d[1] = w_lines_[i].second.pos_s_[1];
				ls3d[2] = w_lines_[i].second.pos_s_[2];
				Eigen::Vector3d le3d;
				le3d[0] = w_lines_[i].second.pos_e_[0];
				le3d[1] = w_lines_[i].second.pos_e_[1];
				le3d[2] = w_lines_[i].second.pos_e_[2];
				//w_lines_[i].second.pos_e_;
				Eigen::Vector3d n_ls = ln - ls3d.cross(ld);
				Eigen::Vector3d ls3d_proj = ls3d + ld.cross(n_ls);
				Eigen::Vector3d n_le = ln - le3d.cross(ld);
				Eigen::Vector3d le3d_proj = le3d + ld.cross(n_le);
				w_lines_[i].second.pos_s_[0] = ls3d_proj[0];
				w_lines_[i].second.pos_s_[1] = ls3d_proj[1];
				w_lines_[i].second.pos_s_[2] = ls3d_proj[2];
				w_lines_[i].second.pos_e_[0] = le3d_proj[0];
				w_lines_[i].second.pos_e_[1] = le3d_proj[1];
				w_lines_[i].second.pos_e_[2] = le3d_proj[2];
			}

#if 0
			//if delete because it is optimized from other pipeline
			for (int i = 0; i < w_plane_lines_otho_lines_.size(); ++i)
			{
				//important
				//
				Eigen::Matrix<double, 6, 1> plu_line;
				Eigen::Vector4d line_otho_v;
				line_otho_v[0] = w_plane_lines_otho_lines_[i].second.line_otho_[0];
				line_otho_v[1] = w_plane_lines_otho_lines_[i].second.line_otho_[1];
				line_otho_v[2] = w_plane_lines_otho_lines_[i].second.line_otho_[2];
				line_otho_v[3] = w_plane_lines_otho_lines_[i].second.line_otho_[3];
				plu_line = HW::OrthLineToPluckerLineNew(line_otho_v);
				//plu_line to d 
				Eigen::Matrix<double, 6, 1> plu_line_d_normlize;
				plu_line_d_normlize = HW::PluckerLineDirNormalized(plu_line);
				//line pnt project to plu_line_d_normlize
				Eigen::Vector3d ln = plu_line_d_normlize.head(3);
				Eigen::Vector3d ld = plu_line_d_normlize.tail(3);
				Eigen::Vector3d ls3d;
				ls3d[0] = w_planes_lines_[i].second.pos_s_[0];
				ls3d[1] = w_planes_lines_[i].second.pos_s_[1];
				ls3d[2] = w_planes_lines_[i].second.pos_s_[2];
				Eigen::Vector3d le3d;
				le3d[0] = w_planes_lines_[i].second.pos_e_[0];
				le3d[1] = w_planes_lines_[i].second.pos_e_[1];
				le3d[2] = w_planes_lines_[i].second.pos_e_[2];
				//w_lines_[i].second.pos_e_;
				Eigen::Vector3d n_ls = ln - ls3d.cross(ld);
				Eigen::Vector3d ls3d_proj = ls3d + ld.cross(n_ls);
				Eigen::Vector3d n_le = ln - le3d.cross(ld);
				Eigen::Vector3d le3d_proj = le3d + ld.cross(n_le);
				w_planes_lines_[i].second.pos_s_[0] = ls3d_proj[0];
				w_planes_lines_[i].second.pos_s_[1] = ls3d_proj[1];
				w_planes_lines_[i].second.pos_s_[2] = ls3d_proj[2];
				w_planes_lines_[i].second.pos_e_[0] = le3d_proj[0];
				w_planes_lines_[i].second.pos_e_[1] = le3d_proj[1];
				w_planes_lines_[i].second.pos_e_[2] = le3d_proj[2];
			}
#endif
		}

		void HWSceneBundleAdjustment::UpdateBundleLineWorldPnts2LinePnts()
		{
			//to do next... display the optimized lines
			for (int i = 0; i < w_planes_lines_.size(); ++i)
			{
				int plane_line_idx = w_planes_lines_[i].first;
				lines_points_3d_[plane_line_idx].pos_s_[0] = w_planes_lines_[i].second.pos_s_[0];
				lines_points_3d_[plane_line_idx].pos_s_[1] = w_planes_lines_[i].second.pos_s_[1];
				lines_points_3d_[plane_line_idx].pos_s_[2] = w_planes_lines_[i].second.pos_s_[2];
				lines_points_3d_[plane_line_idx].pos_e_[0] = w_planes_lines_[i].second.pos_e_[0];
				lines_points_3d_[plane_line_idx].pos_e_[1] = w_planes_lines_[i].second.pos_e_[1];
				lines_points_3d_[plane_line_idx].pos_e_[2] = w_planes_lines_[i].second.pos_e_[2];
			}
			for (int i = 0; i < w_lines_.size(); ++i)
			{
				int plane_line_idx = w_lines_[i].first;
				lines_points_3d_[plane_line_idx].pos_s_[0] = w_lines_[i].second.pos_s_[0];
				lines_points_3d_[plane_line_idx].pos_s_[1] = w_lines_[i].second.pos_s_[1];
				lines_points_3d_[plane_line_idx].pos_s_[2] = w_lines_[i].second.pos_s_[2];
				lines_points_3d_[plane_line_idx].pos_e_[0] = w_lines_[i].second.pos_e_[0];
				lines_points_3d_[plane_line_idx].pos_e_[1] = w_lines_[i].second.pos_e_[1];
				lines_points_3d_[plane_line_idx].pos_e_[2] = w_lines_[i].second.pos_e_[2];
			}
		}

		void HWSceneBundleAdjustment::UpdateBunldePlanesToPlanes()
		{
			for (int i = 0; i < planes_.size(); ++i)
			{
				w_planes_[i].f_[0] = planes_[i].f_[0];
				w_planes_[i].f_[1] = planes_[i].f_[1];
				w_planes_[i].f_[2] = planes_[i].f_[2];
				w_planes_[i].f_[3] = planes_[i].f_[3];
			}
		}

		Eigen::Matrix4d HWSceneBundleAdjustment::ConvertBundleCameraToCamExtr(double* bundle_cam)
		{
			Eigen::Vector3d rv;
			Eigen::Vector3d rt;
			for (int j = 0; j < 3; ++j)
			{
				rv[j] = bundle_cam[j];
			}
			for (int j = 0; j < 3; ++j)
			{
				rt[j] = bundle_cam[j + 3];
			}
			Eigen::Matrix3d R = HW::AxisAngleToRotationMatrixD(rv);	//important
			Eigen::Matrix4d cam;
			cam.setZero();
			cam(0, 0) = R(0, 0); cam(0, 1) = R(0, 1); cam(0, 2) = R(0, 2);
			cam(1, 0) = R(1, 0); cam(1, 1) = R(1, 1); cam(1, 2) = R(1, 2);
			cam(2, 0) = R(2, 0); cam(2, 1) = R(2, 1); cam(2, 2) = R(2, 2);
			cam(0, 3) = rt[0]; cam(1, 3) = rt[1]; cam(2, 3) = rt[2];
			cam(3, 3) = 1.0;
			return cam;
		}

		bool HWSceneBundleAdjustment::optimize()
		{
			std::cerr << "---start to optimize the function---" << std::endl;
			lm_optimize();
			std::cerr << "---end optimize the function---" << std::endl;
			return true;
		}

		bool HWSceneBundleAdjustment::optimize_cams_only()
		{
			std::cerr << "---start to optimize cams only the function---" << std::endl;
			lm_optimize_only_cams();
			std::cerr << "---end optimize cams only the function---" << std::endl;
			return true;
		}

		void HWSceneBundleAdjustment::print_status() const
		{

		}

		void HWSceneBundleAdjustment::print_cams_info()
		{
			int cams_num = static_cast<int>(cams_extrs_.size());
			std::cerr << "cameras info number: " << cams_num << std::endl;
			for (int i = 0; i < cams_extrs_.size(); ++i)
			{
				double* tmp_extr = cams_extrs_[i].cam_extr_;
				for (int j = 0; j < 6; ++j)
				{
					std::cerr << tmp_extr[j] << " ";
				}
				std::cerr << std::endl;
			}
			std::cerr <<"end cameras information"<< std::endl;
		}

		void HWSceneBundleAdjustment::print_pnt3d_info()
		{
			int pnts_num = static_cast<int>(w_pnts_.size());
			std::cerr << "points info number: " << pnts_num << std::endl;
			for (int i = 0; i < w_pnts_.size(); ++i)
			{
				double* tmp_pnt = w_pnts_[i].pos_;
				for (int j = 0; j < 3; ++j)
				{
					std::cerr << tmp_pnt[j] << " ";
				}
				std::cerr << std::endl;
			}
			std::cerr << "end points information" << std::endl;
		}

		void HWSceneBundleAdjustment::print_lines3d_info()
		{

		}

		void HWSceneBundleAdjustment::print_cam_r_info(double* r)
		{
			std::cerr << "test_rodrigues: \n" << r[0] << " " << r[1] << " " << r[2] << std::endl
				<< r[3] << " " << r[4] << " " << r[5] << std::endl
				<< r[6] << " " << r[7] << " " << r[8] << std::endl;
		}

		bool HWSceneBundleAdjustment::lm_optimize_only_cams()
		{
			print_cams_info();
			epoch_max_number_ = 1;
			for (int k = 0; k < epoch_max_number_; ++k)
			{
				lm_optimize_epoch_only_cams();
			}
			return true;
		}

		bool HWSceneBundleAdjustment::lm_optimize_epoch_only_cams()
		{
			for (int i = 0; i < pnts_observations_.size(); ++i)
			{
			}

#if 0
			//add pnt project to image error
			for (int i = 0; i < pnts_observations_.size(); ++i)
			{
				Eigen::Vector2d pnt_2d_ob = pnts_observations_[i].pos_;
				int camid = pnts_observations_[i].camera_id;
				int pnt3d_id = pnts_observations_[i].point_id;
				CamPoseOnlyFromPntProjCostFunc* tmp_posefunc = new CamPoseOnlyFromPntProjCostFunc(pnt_2d_ob);
				tmp_posefunc->SetKMatrix(cams_[camid].cam_k_);
				Eigen::Vector3d tmp_pnt3d = Eigen::Vector3d(w_pnts_[pnt3d_id].pos_[0], 
					w_pnts_[pnt3d_id].pos_[1], w_pnts_[pnt3d_id].pos_[2]);
				tmp_posefunc->SetPoint3dFromPnt(tmp_pnt3d);
				tmp_posefunc->SetEnergyScale(pnts_to_images_pnts_lambda_);
				pnts_cost_func_ = tmp_posefunc;
				all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, cams_extrs_[camid].cam_extr_);
			}

			//add line two end pnts project to image error(line*project(T_cw*lspnt_w + line*project(T_cw*lepnt_w)))
			for (int i = 0; i < lines_observations_.size(); ++i)
			{
				Eigen::Vector2d ls_2d_ob = lines_observations_[i].ls2d_;
				Eigen::Vector2d le_2d_ob = lines_observations_[i].le2d_;
				int camid = lines_observations_[i].camera_id;
				int line3d_id = lines_observations_[i].line3d_id;
				//line3d_id to  w_lines_(polygon line)
				int wline_idx = GetWlineIdxFromLineid(line3d_id);
				if (wline_idx != -1)
				{
					/*std::vector<int> plane_idxs = w_planes_lines_[wplane_idx].second.plane_idxs_;
					int plane_num = static_cast<int>(plane_idxs.size());*/

					////test
					////std::cerr << "otho_line_idx: " << otho_line_idx << std::endl;
					//std::cerr << "line3d idx: " << line3d_id << std::endl;
					//std::cerr << "pnt_s_2d_ob, pnt_e_2d_ob: " << ls_2d_ob.transpose() << ", "
					//	<< le_2d_ob.transpose() << std::endl;
					//Eigen::Matrix4d camextr = ConvertBundleCameraToCamExtr(cams_extrs_[camid].cam_extr_);
					//std::cerr << "camextr: \n" << camextr << std::endl;
					////plucker line
					////end test
					//std::cerr << "plane num: " << plane_num << std::endl;
					/*if (plane_num == 2)
					{
					}*/
					//int pidx0 = plane_idxs[0];	//single plane
					//double* plane0_fun = w_planes_[pidx0].f_;
					//Eigen::Vector2d ob_line_s, ob_line_e;

					CamPoseOnlyFromLineProjCostFunc* tmp_line_proj_pnt_fun = new CamPoseOnlyFromLineProjCostFunc(ls_2d_ob, le_2d_ob);
					tmp_line_proj_pnt_fun->SetKMatrix(cams_[camid].cam_k_);
					tmp_line_proj_pnt_fun->SetEnergyScale(lines_to_images_lines_lambda_);
					Eigen::Vector3d tmp_line_spnt = Eigen::Vector3d(w_lines_[wline_idx].second.pos_s_[0], w_lines_[wline_idx].second.pos_s_[1],
						w_lines_[wline_idx].second.pos_s_[2]);
					tmp_line_proj_pnt_fun->SetPoint3DFromPnt(tmp_line_spnt);
					pnts_cost_func_ = tmp_line_proj_pnt_fun;
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, cams_extrs_[camid].cam_extr_);
					
					Eigen::Vector3d tmp_line_epnt = Eigen::Vector3d(w_lines_[wline_idx].second.pos_e_[0], w_lines_[wline_idx].second.pos_e_[1],
						w_lines_[wline_idx].second.pos_e_[2]);
					tmp_line_proj_pnt_fun->SetPoint3DFromPnt(tmp_line_epnt);
					pnts_cost_func_ = tmp_line_proj_pnt_fun;
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, cams_extrs_[camid].cam_extr_);
				}
			}
#endif

			//how to deal with these options, to do next...
			ceres::Solver::Options options;
			options.dynamic_sparsity = true;
			options.max_num_iterations = bundle_max_iteration_num_per_epoch_;
			//options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
			options.minimizer_type = ceres::TRUST_REGION;
			options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			options.trust_region_strategy_type = ceres::DOGLEG;
			options.minimizer_progress_to_stdout = true;
			//options.dogleg_type = ceres::SUBSPACE_DOGLEG;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &all_cams_pnts_problems_, &summary);
			std::cerr << summary.BriefReport() << "\n";
			//set the loss average value to 

			return true;
		}

		bool HWSceneBundleAdjustment::lm_optimize()
		{

#if 0	//test pnt optimization
			Eigen::Vector2d ob1 = Eigen::Vector2d(9, 7);
			Eigen::Vector2d ob2 = Eigen::Vector2d(10, 5);
			Eigen::Vector2d ob3 = Eigen::Vector2d(11, 3);
			Eigen::Vector2d ob4 = Eigen::Vector2d(20, 10);
			Eigen::Vector2d ob5 = Eigen::Vector2d(5, 14);
			Eigen::Vector2d ob6 = Eigen::Vector2d(15, 22);
			std::vector<Eigen::Vector2d> pnts_obs;
			pnts_obs.emplace_back(ob1);
			pnts_obs.emplace_back(ob2);
			pnts_obs.emplace_back(ob3);
			pnts_obs.emplace_back(ob4);
			pnts_obs.emplace_back(ob5);
			pnts_obs.emplace_back(ob6);
			double x[3] = { 0.0, 0.0, 0.0 };
			double x_i[3] = { 0.0, 0.0, 0.0 };
			double w[3] = { 0.0, 0.0, 0.0 };
			double w_i[3] = { 0.0, 0.0, 0.0 };
			for (int i = 0; i < 6; ++i)
			{
				pnts_cost_func_ = new PntcostFuncTest(pnts_obs[i]);
				all_problems_.AddResidualBlock(pnts_cost_func_, NULL, w, x);
			}
			ceres::Solver::Options options;
			//options.dynamic_sparsity = true;
			options.max_num_iterations = 100;
			//options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
			options.minimizer_type = ceres::TRUST_REGION;
			//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			options.trust_region_strategy_type = ceres::DOGLEG;
			options.minimizer_progress_to_stdout = true;
			//options.dogleg_type = ceres::SUBSPACE_DOGLEG;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &all_problems_, &summary);
			std::cerr << summary.BriefReport() << "\n";

			std::cerr << "x_i, w_i: " << x_i[0] << " " << x_i[1] << " " << x_i[2] << "; "
				<< w_i[0] << " " << w_i[1] << ", " << w_i[2] << std::endl;
			std::cerr << "x, w: " << x[0] << " " << x[1] << " " << x[2] << "; "
				<< w[0] << " " << w[1] << ", " << w[2] << std::endl;
#endif

#if 0	//test plane fitting
			Eigen::Vector3d p0(1.0, 2.0, 2.0);
			Eigen::Vector3d p1(3.0, 3.0, 2.0);
			Eigen::Vector3d p2(4.0, 5.0, 2.0);
			Eigen::Vector3d p3(1.0, 4.0, 2.0);
			Eigen::Vector3d p4(4.0, 5.0, 2.0);
			Eigen::Vector3d p5(3.0, 2.0, 2.0);
			Eigen::Vector3d p6(1.0, 10.0, 2.0);
			std::vector<Eigen::Vector3d> plane_pnts;
			plane_pnts.emplace_back(p0);
			plane_pnts.emplace_back(p1);
			plane_pnts.emplace_back(p2);
			plane_pnts.emplace_back(p3);
			plane_pnts.emplace_back(p4);
			plane_pnts.emplace_back(p5);
			plane_pnts.emplace_back(p6);
			double coeff[4] = { 1,0,1,5 };
			double s_coeff[4] = { 1,0,1,5 };
			for (int i = 0; i < plane_pnts.size(); ++i)
			{
				planes_cost_func_ = new Pnts2PlaneCostFunc(plane_pnts[i]);
				all_problems_.AddResidualBlock(planes_cost_func_, NULL, coeff);
			}
			ceres::Solver::Options options;
			//options.dynamic_sparsity = true;
			options.max_num_iterations = 100;
			//options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
			options.minimizer_type = ceres::TRUST_REGION;
			//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			options.trust_region_strategy_type = ceres::DOGLEG;
			options.minimizer_progress_to_stdout = true;
			//options.dogleg_type = ceres::SUBSPACE_DOGLEG;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &all_problems_, &summary);
			std::cerr << summary.BriefReport() << "\n";

			std::cerr << "s_coeff: " << s_coeff[0] << " " << s_coeff[1] << " " << s_coeff[2] << " " 
				<< s_coeff[3] << std::endl;
			std::cerr << "coeff: " << coeff[0] << " " << coeff[1] << " " << coeff[2] << " "
				<< coeff[3] << std::endl;
#endif

#if 1
			//std::cerr << "before information" << std::endl;
			////test change the cam info
			//for (int i = 0; i < cams_extrs_.size(); ++i)
			//{
			//	cams_extrs_[i].cam_extr_[0] = -1.0;
			//	cams_extrs_[i].cam_extr_[1] = -1.0;
			//	cams_extrs_[i].cam_extr_[2] = 1.0;
			//	cams_extrs_[i].cam_extr_[3] = 1.0;
			//	cams_extrs_[i].cam_extr_[4] = -1.0;
			//	cams_extrs_[i].cam_extr_[5] = 0.0;
			//}
			////end change the cam info

			print_cams_info();
			bundle_max_iteration_ = 1;
			for (int k = 0; k < bundle_max_iteration_; ++k)
			{
				//how to deal with these two function???? to do next...
				lm_optimize_single_iter_cams_pnts();
				std::cerr << "after opimize the camera and pnts" << std::endl;
				print_cams_info();

				//loss function
				//lm_optimize_single_iter_planes_lines();
			}

#endif

#if 0	//only optimize point 3d
			//change the ponits 3d
			for (int i = 0; i < w_pnts_.size(); ++i)
			{
				w_pnts_[i].pos_[0] -= 1.0;
				w_pnts_[i].pos_[1] += 1.0;
				w_pnts_[i].pos_[2] -= 1.0;
			}
			//end change the points 3d

			//add pnts function
			for (int i = 0; i < pnts_observations_.size(); ++i)
			{
				Eigen::Vector2d pnt_2d_ob = pnts_observations_[i].pos_;
				int camid = pnts_observations_[i].camera_id;
				int pnt3d_id = pnts_observations_[i].point_id;
				PosePnt3DcostFunc* tmp_posefunc = new PosePnt3DcostFunc(pnt_2d_ob);
				tmp_posefunc->SetKMatrix(cams_[camid].cam_k_);
				tmp_posefunc->SetRTW(cams_extrs_[camid].cam_extr_);
				pnts_cost_func_ = tmp_posefunc;
				all_problems_.AddResidualBlock(pnts_cost_func_, NULL, w_pnts_[pnt3d_id].pos_);
			}
			ceres::Solver::Options options;
			//options.dynamic_sparsity = true;
			options.max_num_iterations = 100;
			//options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
			options.minimizer_type = ceres::TRUST_REGION;
			//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			options.trust_region_strategy_type = ceres::DOGLEG;
			options.minimizer_progress_to_stdout = true;
			//options.dogleg_type = ceres::SUBSPACE_DOGLEG;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &all_problems_, &summary);
			std::cerr << summary.BriefReport() << "\n";

			//print the info
			std::cerr << "after information" << std::endl;
#endif

#if 0	//only optimize camera pose
			////change the ponits 3d
			//std::cerr << "before information" << std::endl;
			////test change the cam info
			//for (int i = 0; i < cams_extrs_.size(); ++i)
			//{
			//	cams_extrs_[i].cam_extr_[0] = -1.0;
			//	cams_extrs_[i].cam_extr_[1] = -1.0;
			//	cams_extrs_[i].cam_extr_[2] = 1.0;
			//	cams_extrs_[i].cam_extr_[3] = 1.0;
			//	cams_extrs_[i].cam_extr_[4] = -1.0;
			//	cams_extrs_[i].cam_extr_[5] = 0.0;
			//}
			////end change the cam info
			print_cams_info();
			//add pnts function
			for (int i = 0; i < pnts_observations_.size(); ++i)
			{
				Eigen::Vector2d pnt_2d_ob = pnts_observations_[i].pos_;
				int camid = pnts_observations_[i].camera_id;
				int pnt3d_id = pnts_observations_[i].point_id;
				PosePntWRcostFunc* tmp_posefunc = new PosePntWRcostFunc(pnt_2d_ob);
				tmp_posefunc->SetKMatrix(cams_[camid].cam_k_);
				tmp_posefunc->SetPoint3d(w_pnts_[pnt3d_id].pos_);
				pnts_cost_func_ = tmp_posefunc;
				all_problems_.AddResidualBlock(pnts_cost_func_, NULL, cams_extrs_[camid].cam_extr_);
			}
			ceres::Solver::Options options;
			//options.dynamic_sparsity = true;
			options.max_num_iterations = 100;
			//options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
			options.minimizer_type = ceres::TRUST_REGION;
			//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			options.trust_region_strategy_type = ceres::DOGLEG;
			options.minimizer_progress_to_stdout = true;
			//options.dogleg_type = ceres::SUBSPACE_DOGLEG;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &all_problems_, &summary);
			std::cerr << summary.BriefReport() << "\n";

			//print the info
			std::cerr << "after information" << std::endl;
			print_cams_info();
#endif

			return true;
		}
		
		bool HWSceneBundleAdjustment::lm_optimize_single_iter_cams_pnts()
		{

#if 1		//test: pnt 3d to image pnt distance
			//add pnts project to images function
			for (int i = 0; i < pnts_observations_.size(); ++i)
			{
				Eigen::Vector2d pnt_2d_ob = pnts_observations_[i].pos_;
				int camid = pnts_observations_[i].camera_id;
				int pnt3d_id = pnts_observations_[i].point_id;
				PosePntcostFunc* tmp_posefunc = new PosePntcostFunc(pnt_2d_ob);
				tmp_posefunc->SetKMatrix(cams_[camid].cam_k_);
				tmp_posefunc->SetEnergyScale(pnts_to_images_pnts_lambda_);
				pnts_cost_func_ = tmp_posefunc;
				all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, cams_extrs_[camid].cam_extr_, w_pnts_[pnt3d_id].pos_);
			}

			//add pnt3d to plane dist function, important(to do next...)
			for (int i = 0; i < w_pnts_.size(); ++i)
			{
				std::vector<int> plane_i = w_pnts_[i].plane_idxs_;
				if (!plane_i.empty())
				{
					int pidx = plane_i[0];	//single plane
					//std::cerr << "pidx: " << pidx << std::endl;
					double* plane_fun = w_planes_[pidx].f_;
					Pnts3D2PlaneCostFunc* tmp_pnt_fun = new Pnts3D2PlaneCostFunc(plane_fun);
					tmp_pnt_fun->SetEnergyScale(pnts_to_planes_lambda_);
					pnts_cost_func_ = tmp_pnt_fun;
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, w_pnts_[i].pos_);
				}
			}

#endif

#if 0		//test: pnt to two polygons distance
			//add world plane line 3d(two end pnts) to polygon intersection line dist function
			for (int i = 0; i < w_planes_lines_.size(); ++i)
			{
				//std::cerr << "w_plane_lines_->i: " << i << std::endl;
				std::vector<int> plane_idxs = w_planes_lines_[i].second.plane_idxs_;
				int plane_num = static_cast<int>(plane_idxs.size());
				//std::cerr << "plane num: " << plane_num << std::endl;
				if (plane_num == 2)
				{
					int pidx0 = plane_idxs[0];	//single plane
					double* plane0_fun = w_planes_[pidx0].f_;
					Pnts3D2PlaneCostFunc* tmp_pnt0_fun = new Pnts3D2PlaneCostFunc(plane0_fun);
					tmp_pnt0_fun->SetEnergyScale(lines_inter_to_polygons_lines_lambda_);
					pnts_cost_func_ = tmp_pnt0_fun;
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, w_planes_lines_[i].second.pos_s_);
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, w_planes_lines_[i].second.pos_e_);

					int pidx1 = plane_idxs[1];	//single plane
					//std::cerr << "pidx0, pidx1: " << pidx0 <<", " << pidx1 << std::endl;
					double* plane1_fun = w_planes_[pidx1].f_;
					Pnts3D2PlaneCostFunc* tmp_pnt1_fun = new Pnts3D2PlaneCostFunc(plane1_fun);
					pnts_cost_func_ = tmp_pnt1_fun;
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, w_planes_lines_[i].second.pos_s_);
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, w_planes_lines_[i].second.pos_e_);
				}
			}
#endif

#if 1		//test: line pnts project to image line distance, PosePnt3DToImgLines2ProjectFunc
			//pnts project to line distance, PosePnt3DToImgLines2ProjectFunc (polygons intersection line)
			for (int i = 0; i < lines_observations_.size(); ++i)
			{
				Eigen::Vector2d ls_2d_ob = lines_observations_[i].ls2d_;
				Eigen::Vector2d le_2d_ob = lines_observations_[i].le2d_;
				int camid = lines_observations_[i].camera_id;
				int line3d_id = lines_observations_[i].line3d_id;
				//line3d_id to  w_planes_lines_(polygon intersection line) 
				int wplane_line_idx = GetWPlanelineIdxFromLineid(line3d_id);
				if (wplane_line_idx != -1)
				{
					/*std::vector<int> plane_idxs = w_planes_lines_[wplane_line_idx].second.plane_idxs_;
					int plane_num = static_cast<int>(plane_idxs.size());*/
					
					////test
					////std::cerr << "otho_line_idx: " << otho_line_idx << std::endl;
					//std::cerr << "line3d idx: " << line3d_id << std::endl;
					//std::cerr << "pnt_s_2d_ob, pnt_e_2d_ob: " << ls_2d_ob.transpose() << ", "
					//	<< le_2d_ob.transpose() << std::endl;
					//Eigen::Matrix4d camextr = ConvertBundleCameraToCamExtr(cams_extrs_[camid].cam_extr_);
					//std::cerr << "camextr: \n" << camextr << std::endl;
					////plucker line
					////end test

					//std::cerr << "plane num: " << plane_num << std::endl;
					//int pidx0 = plane_idxs[0];	//single plane
					//double* plane0_fun = w_planes_[pidx0].f_;
					//Eigen::Vector2d ob_line_s, ob_line_e;
					PosePnt3DToImgLines2ProjectFunc* tmp_line_proj_pnt_fun = new PosePnt3DToImgLines2ProjectFunc(ls_2d_ob, le_2d_ob);
					tmp_line_proj_pnt_fun->SetKMatrix(cams_[camid].cam_k_);
					tmp_line_proj_pnt_fun->SetEnergyScale(lines_inter_to_images_lines_lambda_);
					pnts_cost_func_ = tmp_line_proj_pnt_fun;
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, cams_extrs_[camid].cam_extr_, w_planes_lines_[wplane_line_idx].second.pos_s_);
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, cams_extrs_[camid].cam_extr_, w_planes_lines_[wplane_line_idx].second.pos_e_);
				}	
			}
			std::cerr << "---------------end add world line to polygon distance--------------" << std::endl;
#endif

#if 1		//if many lines to polygon 3d line. not the polygon intersection line
			//add world plane line 3d(two end pnts) to plane dist function 
			//--------(the line attached to only one polygon)--------
			for (int i = 0; i < w_lines_.size(); ++i)
			{
				//std::cerr << "w_lines_->i: " << i << std::endl;
				std::vector<int> plane_idxs = w_lines_[i].second.plane_idxs_;
				int plane_num = static_cast<int>(plane_idxs.size());
				//std::cerr << "plane num: " << plane_num << std::endl;
				if (plane_num >= 1)
				{
					int pidx0 = plane_idxs[0];	//single plane, to do next
					if (pidx0 == -1)
					{
						continue;
					}
					double* plane0_fun = w_planes_[pidx0].f_;
					Pnts3D2PlaneCostFunc* tmp_pnt0_fun = new Pnts3D2PlaneCostFunc(plane0_fun);
					tmp_pnt0_fun->SetEnergyScale(lines_to_polygons_lines_lambda_);
					pnts_cost_func_ = tmp_pnt0_fun;
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, w_lines_[i].second.pos_s_);
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, w_lines_[i].second.pos_e_);
				}
			}
#endif

#if 1		//test: line pnts 3d project to image lines distance (reprojected error)
			//pnts project to line distance, PosePnt3DToImgLines2ProjectFunc
			for (int i = 0; i < lines_observations_.size(); ++i)
			{
				Eigen::Vector2d ls_2d_ob = lines_observations_[i].ls2d_;
				Eigen::Vector2d le_2d_ob = lines_observations_[i].le2d_;
				int camid = lines_observations_[i].camera_id;
				int line3d_id = lines_observations_[i].line3d_id;
				//line3d_id to  w_lines_(polygon line)
				int wline_idx = GetWlineIdxFromLineid(line3d_id);
				if (wline_idx != -1)
				{
					/*std::vector<int> plane_idxs = w_planes_lines_[wplane_idx].second.plane_idxs_;
					int plane_num = static_cast<int>(plane_idxs.size());*/

					////test
					////std::cerr << "otho_line_idx: " << otho_line_idx << std::endl;
					//std::cerr << "line3d idx: " << line3d_id << std::endl;
					//std::cerr << "pnt_s_2d_ob, pnt_e_2d_ob: " << ls_2d_ob.transpose() << ", "
					//	<< le_2d_ob.transpose() << std::endl;
					//Eigen::Matrix4d camextr = ConvertBundleCameraToCamExtr(cams_extrs_[camid].cam_extr_);
					//std::cerr << "camextr: \n" << camextr << std::endl;
					////plucker line
					////end test

					//std::cerr << "plane num: " << plane_num << std::endl;
					/*if (plane_num == 2)
					{
					}*/
					//int pidx0 = plane_idxs[0];	//single plane
					//double* plane0_fun = w_planes_[pidx0].f_;
					//Eigen::Vector2d ob_line_s, ob_line_e;
					PosePnt3DToImgLines2ProjectFunc* tmp_line_proj_pnt_fun = new PosePnt3DToImgLines2ProjectFunc(ls_2d_ob, le_2d_ob);
					tmp_line_proj_pnt_fun->SetKMatrix(cams_[camid].cam_k_);
					tmp_line_proj_pnt_fun->SetEnergyScale(lines_to_images_lines_lambda_);
					pnts_cost_func_ = tmp_line_proj_pnt_fun;
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, cams_extrs_[camid].cam_extr_, w_lines_[wline_idx].second.pos_s_);
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL, cams_extrs_[camid].cam_extr_, w_lines_[wline_idx].second.pos_e_);
				}
			}
#endif


/*
------------------------------plucker line function optimizatin(delete)-----------------
*/
#if 0		//add world lines 3d to project to image line(error function)
			//add world lines 3d to project to image line(error function)
			for (int i = 0; i < lines_observations_.size(); ++i)
			{
				Eigen::Vector2d pnt_s_2d_ob = lines_observations_[i].ls2d_;
				Eigen::Vector2d pnt_e_2d_ob = lines_observations_[i].le2d_;
				int camid = lines_observations_[i].camera_id;
				int line_id = lines_observations_[i].line3d_id;
				//get line 3d
				int otho_line_idx = GetWOtholineIdxFromLineid(line_id);
				if (otho_line_idx != -1)
				{
					////test
					//std::cerr << "otho_line_idx: " << otho_line_idx << std::endl;
					//std::cerr << "line3d idx: " << line_id << std::endl;
					//double* otholine = w_lines_otho_lines_[otho_line_idx].second.line_otho_;
					//Eigen::Vector4d otholine_vec = Eigen::Vector4d(otholine[0], otholine[1], otholine[2], otholine[3]);
					//Eigen::Matrix<double, 6, 1> tmp_plucker_line = OrthLineToPluckerLineNew(otholine_vec);
					//std::cerr << "tmp_plucker_line: " << tmp_plucker_line.transpose() << std::endl;
					//std::cerr << "pnt_s_2d_ob, pnt_e_2d_ob: " << pnt_s_2d_ob.transpose() << ", "
					//	<< pnt_e_2d_ob.transpose() << std::endl;
					//Eigen::Matrix4d camextr = ConvertBundleCameraToCamExtr(cams_extrs_[camid].cam_extr_);
					//std::cerr << "camextr: \n" << camextr << std::endl;
					////plucker line
					////end test
					PoseOthonormalLines2ProjectFunc* tmp_posefunc =
						new PoseOthonormalLines2ProjectFunc(pnt_s_2d_ob, pnt_e_2d_ob);
					tmp_posefunc->SetKMatrix(cams_[camid].cam_k_);
					tmp_posefunc->SetEnergyScale(lines_to_images_lines_lambda_);
					pnts_cost_func_ = tmp_posefunc;
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL,
						cams_extrs_[camid].cam_extr_, w_lines_otho_lines_[otho_line_idx].second.line_otho_);
				}
			}
#endif

#if 0		//test only optimize the pose --- add world lines 3d to project to image line(error function)
			/*
			---- add world lines 3d to project to image line(error function) ----
						test only optimize the pose
			*/
			//add world lines 3d to project to image line(error function)
			//大概率是线段对应有问题，需要检测一下。
			for (int i = 0; i < lines_observations_.size(); ++i)
			{
				Eigen::Vector2d pnt_s_2d_ob = lines_observations_[i].ls2d_;
				Eigen::Vector2d pnt_e_2d_ob = lines_observations_[i].le2d_;
				int camid = lines_observations_[i].camera_id;
				int line_id = lines_observations_[i].line3d_id;
				//get line 3d
				int otho_line_idx = GetWOtholineIdxFromLineid(line_id);
				if (otho_line_idx != -1)
				{
					////test
					//std::cerr << "otho_line_idx: " << otho_line_idx << std::endl;
					//std::cerr << "line3d idx: " << line_id << std::endl;
					//double* otholine = w_lines_otho_lines_[otho_line_idx].second.line_otho_;
					//Eigen::Vector4d otholine_vec = Eigen::Vector4d(otholine[0], otholine[1], otholine[2], otholine[3]);
					//Eigen::Matrix<double, 6, 1> tmp_plucker_line = OrthLineToPluckerLineNew(otholine_vec);
					//std::cerr << "tmp_plucker_line: " << tmp_plucker_line.transpose() << std::endl;
					//std::cerr << "pnt_s_2d_ob, pnt_e_2d_ob: " << pnt_s_2d_ob.transpose() << ", "
					//	<< pnt_e_2d_ob.transpose() << std::endl;
					//Eigen::Matrix4d camextr = ConvertBundleCameraToCamExtr(cams_extrs_[camid].cam_extr_);
					//std::cerr << "camextr: \n" << camextr << std::endl;
					////plucker line
					////end test
					PoseOthonormalLines2ProjectImagePosePartFunc* tmp_posefunc =
						new PoseOthonormalLines2ProjectImagePosePartFunc(pnt_s_2d_ob, pnt_e_2d_ob);
					tmp_posefunc->SetKMatrix(cams_[camid].cam_k_);
					tmp_posefunc->SetEnergyScale(lines_to_images_lines_lambda_);
					Eigen::Vector4d tmp_line_otho;
					tmp_line_otho[0] = w_lines_otho_lines_[otho_line_idx].second.line_otho_[0];
					tmp_line_otho[1] = w_lines_otho_lines_[otho_line_idx].second.line_otho_[1];
					tmp_line_otho[2] = w_lines_otho_lines_[otho_line_idx].second.line_otho_[2];
					tmp_line_otho[3] = w_lines_otho_lines_[otho_line_idx].second.line_otho_[3];
					tmp_posefunc->SetOthonormalLine(tmp_line_otho);
					pnts_cost_func_ = tmp_posefunc;
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL,
						cams_extrs_[camid].cam_extr_);
				}
			}
#endif

#if 0	
			//test only optimize the orthonormal line --- add world lines 3d to project to image line(error function)
			/*
			---- add world lines 3d to project to image line(error function) ----
			test only optimize the line 3d(othonormal line)

			-------compute jaco wrong------ to do next. because another method is needed
			*/
			//add world lines 3d to project to image line(error function)
			for (int i = 0; i < lines_observations_.size(); ++i)
			{
				Eigen::Vector2d pnt_s_2d_ob = lines_observations_[i].ls2d_;
				Eigen::Vector2d pnt_e_2d_ob = lines_observations_[i].le2d_;
				int camid = lines_observations_[i].camera_id;
				int line_id = lines_observations_[i].line3d_id;
				//get line 3d
				int otho_line_idx = GetWOtholineIdxFromLineid(line_id);
				if (otho_line_idx != -1)
				{
					////test
					//std::cerr << "otho_line_idx: " << otho_line_idx << std::endl;
					//std::cerr << "line3d idx: " << line_id << std::endl;
					//double* otholine = w_lines_otho_lines_[otho_line_idx].second.line_otho_;
					//Eigen::Vector4d otholine_vec = Eigen::Vector4d(otholine[0], otholine[1], otholine[2], otholine[3]);
					//Eigen::Matrix<double, 6, 1> tmp_plucker_line = OrthLineToPluckerLineNew(otholine_vec);
					//std::cerr << "tmp_plucker_line: " << tmp_plucker_line.transpose() << std::endl;
					//std::cerr << "pnt_s_2d_ob, pnt_e_2d_ob: " << pnt_s_2d_ob.transpose() << ", "
					//	<< pnt_e_2d_ob.transpose() << std::endl;
					//Eigen::Matrix4d camextr = ConvertBundleCameraToCamExtr(cams_extrs_[camid].cam_extr_);
					//std::cerr << "camextr: \n" << camextr << std::endl;
					////plucker line
					////end test
					OthonormalLines2ProjectImagePartFunc* tmp_posefunc =
						new OthonormalLines2ProjectImagePartFunc(pnt_s_2d_ob, pnt_e_2d_ob);
					tmp_posefunc->SetKMatrix(cams_[camid].cam_k_);
					tmp_posefunc->SetEnergyScale(lines_to_images_lines_lambda_);
					/*Eigen::Vector4d tmp_line_otho;
					tmp_line_otho[0] = w_lines_otho_lines_[otho_line_idx].second.line_otho_[0];
					tmp_line_otho[1] = w_lines_otho_lines_[otho_line_idx].second.line_otho_[1];
					tmp_line_otho[2] = w_lines_otho_lines_[otho_line_idx].second.line_otho_[2];
					tmp_line_otho[3] = w_lines_otho_lines_[otho_line_idx].second.line_otho_[3];*/
					Eigen::Matrix<double, 1, 6> tmp_cam_extr;
					tmp_cam_extr(0, 0) = cams_extrs_[camid].cam_extr_[0]; tmp_cam_extr(0, 1) = cams_extrs_[camid].cam_extr_[1];
					tmp_cam_extr(0, 2) = cams_extrs_[camid].cam_extr_[2]; tmp_cam_extr(0, 3) = cams_extrs_[camid].cam_extr_[3];
					tmp_cam_extr(0, 4) = cams_extrs_[camid].cam_extr_[4]; tmp_cam_extr(0, 5) = cams_extrs_[camid].cam_extr_[5];
					tmp_posefunc->SetCamExtr(tmp_cam_extr);
					pnts_cost_func_ = tmp_posefunc;
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL,
						w_lines_otho_lines_[otho_line_idx].second.line_otho_);
				}
			}

#endif
			
#if 0	//(important->delete the line project to image) cost function: compute distance between projected line and observed image line
			//add world plane lines 3d to project to image line(error function)
			for (int i = 0; i < lines_observations_.size(); ++i)
			{
				Eigen::Vector2d pnt_s_2d_ob = lines_observations_[i].ls2d_;
				Eigen::Vector2d pnt_e_2d_ob = lines_observations_[i].le2d_;
				int camid = lines_observations_[i].camera_id;
				int line_id = lines_observations_[i].line3d_id;
				//get line 3d
				int otho_plane_line_idx = GetWPlaneOtholineIdxFromLineid(line_id);
				if (otho_plane_line_idx != -1)
				{
					////test
					////std::cerr << "otho_plane_line_idx: " << otho_plane_line_idx << std::endl;
					//std::cerr << "line3d idx: " << line_id << std::endl;
					///*double* otholine = w_plane_lines_otho_lines_[otho_plane_line_idx].second.line_otho_;
					//Eigen::Vector4d otholine_vec = Eigen::Vector4d(otholine[0], otholine[1], otholine[2], otholine[3]);
					//Eigen::Matrix<double, 6, 1> tmp_plucker_line = OrthLineToPluckerLineNew(otholine_vec);
					//std::cerr << "tmp_plucker_line: " << tmp_plucker_line.transpose() << std::endl;*/
					//std::cerr << "pnt_s_2d_ob, pnt_e_2d_ob: " << pnt_s_2d_ob.transpose() << ", "
					//	<< pnt_e_2d_ob.transpose() << std::endl;
					//Eigen::Matrix4d camextr = ConvertBundleCameraToCamExtr(cams_extrs_[camid].cam_extr_);
					//std::cerr << "camextr: \n" << camextr << std::endl;
					////end test

					PoseOthonormalLines2ProjectFunc* tmp_posefunc =
						new PoseOthonormalLines2ProjectFunc(pnt_s_2d_ob, pnt_e_2d_ob);
					tmp_posefunc->SetKMatrix(cams_[camid].cam_k_);
					pnts_cost_func_ = tmp_posefunc;
					all_cams_pnts_problems_.AddResidualBlock(pnts_cost_func_, NULL,
						cams_extrs_[camid].cam_extr_, w_plane_lines_otho_lines_[otho_plane_line_idx].second.line_otho_);
				}
			}
#endif
/*
------------------------------end plucker line function optimizatin-----------------
*/

			//for (int i = 0; i < w_plane_lines_otho_lines_.size(); ++i)
			//{
			//	//line to plucker coordinate
			//}

			//how to deal with these options, to do next...
			ceres::Solver::Options options;
			options.dynamic_sparsity = true;
			options.max_num_iterations = 300;
			//options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
			options.minimizer_type = ceres::TRUST_REGION;
			options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			options.trust_region_strategy_type = ceres::DOGLEG;
			options.minimizer_progress_to_stdout = true;
			//options.dogleg_type = ceres::SUBSPACE_DOGLEG;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &all_cams_pnts_problems_, &summary);
			std::cerr << summary.BriefReport() << "\n";
			//set the loss average value to 
			return true;
		}

		bool HWSceneBundleAdjustment::lm_optimize_single_iter_planes_lines()
		{

#if 1
			//delete all the line to plane 3d distance function
			//add pnt3d to plane dist function
			for (int i = 0; i < w_pnts_.size(); ++i)
			{
				std::vector<int> plane_i = w_pnts_[i].plane_idxs_;
				if (!plane_i.empty())
				{
					int pidx = plane_i[0];	//single plane
					//std::cerr << "pidx: " << pidx << std::endl;
					//Eigen::Vector4d plane_fun = planes_[pidx].f_;
					Eigen::Vector3d pnt3d_pos;
					pnt3d_pos[0] = w_pnts_[i].pos_[0];
					pnt3d_pos[1] = w_pnts_[i].pos_[1];
					pnt3d_pos[2] = w_pnts_[i].pos_[2];
					Plane2Pnts3DCostFunc* tmp_pnt_fun = new Plane2Pnts3DCostFunc(pnt3d_pos);
					pnts_cost_func_ = tmp_pnt_fun;
					all_planes_lines_problems_.AddResidualBlock(pnts_cost_func_, NULL, planes_[pidx].f_.data());
				}
			}
#endif
			//add polygon funtion to optimized line 3d(two end pnts) dist function
			for (int i = 0; i < w_planes_lines_.size(); ++i)
			{
				//std::cerr << "w_plane_lines_->i: " << i << std::endl;
				std::vector<int> plane_idxs = w_planes_lines_[i].second.plane_idxs_;
				int plane_num = static_cast<int>(plane_idxs.size());
				//std::cerr << "plane num: " << plane_num << std::endl;
				if (plane_num == 2)
				{
					int pidx0 = plane_idxs[0];
					int pidx1 = plane_idxs[1];
					Eigen::Vector3d pnt3d_s_pos;
					pnt3d_s_pos[0] = w_planes_lines_[i].second.pos_s_[0];
					pnt3d_s_pos[1] = w_planes_lines_[i].second.pos_s_[1];
					pnt3d_s_pos[2] = w_planes_lines_[i].second.pos_s_[2];
					Plane2Pnts3DCostFunc* tmp_pnt_s_fun = new Plane2Pnts3DCostFunc(pnt3d_s_pos);
					tmp_pnt_s_fun->SetEnergyScale(polygons_to_lines3d_lambda_);
					pnts_cost_func_ = tmp_pnt_s_fun;
					all_planes_lines_problems_.AddResidualBlock(pnts_cost_func_, NULL, planes_[pidx0].f_.data());
					all_planes_lines_problems_.AddResidualBlock(pnts_cost_func_, NULL, planes_[pidx1].f_.data());

					Eigen::Vector3d pnt3d_e_pos;
					pnt3d_e_pos[0] = w_planes_lines_[i].second.pos_e_[0];
					pnt3d_e_pos[1] = w_planes_lines_[i].second.pos_e_[1];
					pnt3d_e_pos[2] = w_planes_lines_[i].second.pos_e_[2];
					Plane2Pnts3DCostFunc* tmp_pnt_e_fun = new Plane2Pnts3DCostFunc(pnt3d_e_pos);
					tmp_pnt_e_fun->SetEnergyScale(polygons_to_lines3d_lambda_);
					pnts_cost_func_ = tmp_pnt_e_fun;
					all_planes_lines_problems_.AddResidualBlock(pnts_cost_func_, NULL, planes_[pidx0].f_.data());
					all_planes_lines_problems_.AddResidualBlock(pnts_cost_func_, NULL, planes_[pidx1].f_.data());
				}
			}


			//add world plane line 3d(two end pnts) to plane dist function 
			//--------(the line attached to only one polygon)--------
			for (int i = 0; i < w_lines_.size(); ++i)
			{
				//std::cerr << "w_plane_lines_->i: " << i << std::endl;
				std::vector<int> plane_idxs = w_lines_[i].second.plane_idxs_;
				int plane_num = static_cast<int>(plane_idxs.size());
				//std::cerr << "plane num: " << plane_num << std::endl;
				if (plane_num >= 1)
				{
					int pidx0 = plane_idxs[0];	//single plane, to do next
					if (pidx0 == -1)
					{
						continue;
					}
					Eigen::Vector3d pnt3d_s_pos;
					pnt3d_s_pos[0] = w_planes_lines_[i].second.pos_s_[0];
					pnt3d_s_pos[1] = w_planes_lines_[i].second.pos_s_[1];
					pnt3d_s_pos[2] = w_planes_lines_[i].second.pos_s_[2];
					Plane2Pnts3DCostFunc* tmp_pnt_s_fun = new Plane2Pnts3DCostFunc(pnt3d_s_pos);
					tmp_pnt_s_fun->SetEnergyScale(polygons_to_lines3d_lambda_);
					pnts_cost_func_ = tmp_pnt_s_fun;
					all_planes_lines_problems_.AddResidualBlock(pnts_cost_func_, NULL, planes_[pidx0].f_.data());
					
					Eigen::Vector3d pnt3d_e_pos;
					pnt3d_e_pos[0] = w_planes_lines_[i].second.pos_e_[0];
					pnt3d_e_pos[1] = w_planes_lines_[i].second.pos_e_[1];
					pnt3d_e_pos[2] = w_planes_lines_[i].second.pos_e_[2];
					Plane2Pnts3DCostFunc* tmp_pnt_e_fun = new Plane2Pnts3DCostFunc(pnt3d_e_pos);
					tmp_pnt_e_fun->SetEnergyScale(polygons_to_lines3d_lambda_);
					pnts_cost_func_ = tmp_pnt_e_fun;
					all_planes_lines_problems_.AddResidualBlock(pnts_cost_func_, NULL, planes_[pidx0].f_.data());
				}
			}

			//how to deal with these options, to do next...
			ceres::Solver::Options options;
			//options.dynamic_sparsity = true;
			options.max_num_iterations = 100;
			//options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
			options.minimizer_type = ceres::TRUST_REGION;
			//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			options.trust_region_strategy_type = ceres::DOGLEG;
			options.minimizer_progress_to_stdout = true;
			//options.dogleg_type = ceres::SUBSPACE_DOGLEG;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &all_planes_lines_problems_, &summary);
			std::cerr << summary.BriefReport() << "\n";

			return true;
		}

		int HWSceneBundleAdjustment::GetWPlanelineIdxFromLineid(int lineid)
		{
			int find_idx = -1;
			for (int i = 0; i < w_planes_lines_.size(); ++i)
			{
				if (w_planes_lines_[i].first == lineid)
				{
					find_idx = i;
					return find_idx;
				}
			}
			return find_idx;
		}

		int HWSceneBundleAdjustment::GetWlineIdxFromLineid(int lineid)
		{
			int find_idx = -1;
			for (int i = 0; i < w_lines_.size(); ++i)
			{
				if (w_lines_[i].first == lineid)
				{
					find_idx = i;
					return find_idx;
				}
			}
			return find_idx;
		}

		int HWSceneBundleAdjustment::GetWOtholineIdxFromLineid(int lineid)
		{
			int find_idx = -1;
			for (int i = 0; i < w_lines_otho_lines_.size(); ++i)
			{
				if (w_lines_otho_lines_[i].first == lineid)
				{
					find_idx = i;
					return find_idx;
				}
			}
			return find_idx;
		}

		int HWSceneBundleAdjustment::GetWPlaneOtholineIdxFromLineid(int lineid)
		{
			int find_idx = -1;
			for (int i = 0; i < w_plane_lines_otho_lines_.size(); ++i)
			{
				if (w_plane_lines_otho_lines_[i].first == lineid)
				{
					find_idx = i;
					return find_idx;
				}
			}
			return find_idx;
		}
	}
}