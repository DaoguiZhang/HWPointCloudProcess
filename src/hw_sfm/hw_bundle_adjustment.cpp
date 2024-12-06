#include"hw_bundle_adjustment.h"
#include"ceres/ceres.h"
//#include"ceres/ceres_local_parameterization.hpp"
#include"ceres/rotation.h"
#include"hw_cmns.h"

namespace HWSFM
{
	namespace BA
	{

#if 1
		//struct SnavelyReprojectionError {
		//	SnavelyReprojectionError(double observerx, double observery)
		//		:observered_x_(observerx), observered_y_(observery) {}
		//	//
		//	template <typename T>
		//	bool operator()(const T* const camera, const T* const point, T* residuals) const
		//	{
		//		T p[3];	//world coordinate pnt
		//		ceres::AngleAxisRotatePoint(camera, point, p);
		//		//camera[3,4,5] are the translation
		//		p[0] += camera[3];
		//		p[1] += camera[4];
		//		p[2] += camera[5];
		//		//compute the center of distortion. the sign change comes from
		//		//the camera model that bundler assumers
		//		//the camera coordinate system has a z axis
		//	}
		//	double observered_x_;
		//	double observered_y_;
		//};

#if 0
		class PoseGraphcostFunc : public ceres::SizedCostFunction<2, 1, 1> 
		{
		public:
			virtual ~PoseGraphcostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				const double x = parameters[0][0];
				const double y = parameters[1][0];
				residuals[0] = 9 - x*x;
				residuals[1] = 9 - y*y;
				if (jacobians != NULL && jacobians[0] != NULL)
				{
					jacobians[0][0] = -2 * x;
					jacobians[0][1] = 0;
				}
				if (jacobians != NULL && jacobians[1] != NULL)
				{
					jacobians[1][0] = -2 * y;
					jacobians[1][1] = 0;
				}
				return true;
			}
		};

		class PoseGraphcostFunc : public ceres::SizedCostFunction<2, 2>
		{

		public:

			PoseGraphcostFunc(double ob_x, double ob_y)
				:observered_x_(ob_x), observered_y_(ob_y){}

			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}
			virtual ~PoseGraphcostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				const double x = parameters[0][0];
				const double y = parameters[0][1];	//to do next...
				//const double y = parameters[1][0];

				//compute the reprojection error
				residuals[0] = observered_x_ - 2*x*y;
				residuals[1] = observered_y_ - x*y;
				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J(jacobians[0]);

					J(0, 0) = -2 * y;
					J(0, 1) = -x;
					J(1, 0) = -y;
					J(1, 1) = -x;

					//jacobians[0][0] = -2 * y;
					//jacobians[0][1] = -x;
				}
				/*if (jacobians != NULL && jacobians[1] != NULL)
				{
					jacobians[1][0] = -y;
					jacobians[1][1] = -x;
				}*/
				return true;
			}

			Eigen::Matrix3d K_;
			double observered_x_;
			double observered_y_;
		};

		class PoseGraphcostFunc : public ceres::SizedCostFunction<2, 6>
		{

		public:

			PoseGraphcostFunc(double ob_x, double ob_y)
				:observered_x_(ob_x), observered_y_(ob_y) {}

			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}
			virtual ~PoseGraphcostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				const double w1 = parameters[0][0];
				const double w2 = parameters[0][1];
				const double w3 = parameters[0][2];
				const double x = parameters[0][3];
				const double y = parameters[0][4];	//to do next...
				const double z = parameters[0][5];
				//const double y = parameters[1][0];

				//compute the reprojection error
				/*std::cerr << "w1,w2,w3: " << w1 << ", " << w2 << ", " << w3 << std::endl;
				std::cerr << "x,y,z: " << x << ", " << y << ", " << z << std::endl;*/
				residuals[0] = observered_x_ - w1 * 2 - w2 * 3 - w3 * 4 - x * 2 - y * 2 - z * 2;
				residuals[1] = observered_y_ - w1 * 4 - w2 * 3 - w3 * 2 - x * 2 - y * 2 - z * 2;
				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[0]);

					J(0, 0) = -2;
					J(0, 1) = -3;
					J(0, 2) = -4;
					J(0, 3) = -2;
					J(0, 4) = -2;
					J(0, 5) = -2;

					J(1, 0) = -4;
					J(1, 1) = -3;
					J(1, 2) = -2;
					J(1, 3) = -2;
					J(1, 4) = -2;
					J(1, 5) = -2;

					//jacobians[0][0] = -2 * y;
					//jacobians[0][1] = -x;
				}
				/*if (jacobians != NULL && jacobians[1] != NULL)
				{
				jacobians[1][0] = -y;
				jacobians[1][1] = -x;
				}*/
				return true;
			}

			Eigen::Matrix3d K_;
			double observered_x_;
			double observered_y_;
		};

		class PoseGraphcostFunc : public ceres::SizedCostFunction<2, 9>
		{

		public:

			PoseGraphcostFunc(double ob_x, double ob_y)
				:observered_x_(ob_x), observered_y_(ob_y) {}

			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}
			virtual ~PoseGraphcostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				const double w0 = parameters[0][0];
				const double w1 = parameters[0][1];
				const double w2 = parameters[0][2];

				double w_rotation[3];
				w_rotation[0] = parameters[0][0];
				w_rotation[1] = parameters[0][1];
				w_rotation[2] = parameters[0][2];

				const double t0 = parameters[0][3];
				const double t1 = parameters[0][4];
				const double t2 = parameters[0][5];
				const double x = parameters[0][6];
				const double y = parameters[0][7];	//to do next...
				const double z = parameters[0][8];

				//const double y = parameters[1][0];

				//compute the reprojection error
				/*std::cerr << "w1,w2,w3: " << w1 << ", " << w2 << ", " << w3 << std::endl;
				std::cerr << "x,y,z: " << x << ", " << y << ", " << z << std::endl;*/
				residuals[0] = observered_x_ - w0 * 2 - w1 * 3 - w2 * 4 - t0 * 2 - t1 * 2 - t2 * 2 - x * 2 - y * 2 - z * 2;
				residuals[1] = observered_y_ - w0 * 4 - w1 * 3 - w2 * 2 - t0 * 2 - t1 * 2 - t2 * 2 - x * 2 - y * 2 - z * 2;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J(jacobians[0]);

					J(0, 0) = -2;
					J(0, 1) = -3;
					J(0, 2) = -4;
					J(0, 3) = -2;
					J(0, 4) = -2;
					J(0, 5) = -2;
					J(0, 6) = -2;
					J(0, 7) = -2;
					J(0, 8) = -2;

					J(1, 0) = -4;
					J(1, 1) = -3;
					J(1, 2) = -2;
					J(1, 3) = -2;
					J(1, 4) = -2;
					J(1, 5) = -2;
					J(1, 6) = -2;
					J(1, 7) = -2;
					J(1, 8) = -2;

					//jacobians[0][0] = -2 * y;
					//jacobians[0][1] = -x;
				}
				/*if (jacobians != NULL && jacobians[1] != NULL)
				{
				jacobians[1][0] = -y;
				jacobians[1][1] = -x;
				}*/
				return true;
			}

			Eigen::Matrix3d K_;
			double observered_x_;
			double observered_y_;
		};

#endif

		/*
		f(x,y) = (1-x)^2 + 100(y - x^2)^2; general unconstrained minimization 
		*/
		class Rosenbrock final : public ceres::FirstOrderFunction
		{
		public:
			~Rosenbrock() override {}

			// parameters相当于是CostFunction中的参数块
			// gradient相当于是CostFunction中的雅克比矩阵
			// 由于只有一个参数块，parameters和gradient都是一维数组
			bool Evaluate(const double* parameters, double* cost, double* gradient) const override
			{
				const double x = parameters[0];
				const double y = parameters[1];
				cost[0] = (1.0 - x) * (1.0 - x) + 100.0 * (y - x * x) * (y - x * x);
				// 与CostFunction中求雅克比矩阵过程类似
				if (gradient) {
					gradient[0] = -2.0 * (1.0 - x) - 200.0 * (y - x * x) * 2.0 * x;
					gradient[1] = 200.0 * (y - x * x);
				}
				return true;
			}

			int NumParameters() const override { return 2; }
		};

		class PoseGraphcostFunc : public ceres::SizedCostFunction<2, 9>
		{

		public:

			PoseGraphcostFunc(double ob_x, double ob_y)
				:observered_x_(ob_x), observered_y_(ob_y)
			{
				K_ = Eigen::Matrix3d::Identity();
			}

			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}
			virtual ~PoseGraphcostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				matrix->(1,9)
				parameters: (w[0],w[1],w[2],t[0],t[1],t[2],X[0],X[1],X[2])
				camera parameters: (w[0], w[1], w[2], t[0], t[1], t[2])
				p3d parameters: (X[0], X[1], X[2])
				*/
				double w[3];
				double w_rotation[9];
				w[0] = parameters[0][0];
				w[1] = parameters[0][1];
				w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(w, w_rotation);

				double t[3];
				t[0] = parameters[0][3];
				t[1] = parameters[0][4];
				t[2] = parameters[0][5];

				double p3d[3];
				p3d[0] = parameters[0][6];
				p3d[1] = parameters[0][7];
				p3d[2] = parameters[0][8];

				double const rx = w[0] * p3d[0] + w[1] * p3d[1] + w[2] * p3d[2];	//row major
				double const ry = w[3] * p3d[0] + w[4] * p3d[1] + w[5] * p3d[2];	//row major
				double const rz = w[6] * p3d[0] + w[7] * p3d[1] + w[8] * p3d[2];	//row major
				double const xc = rx + t[0];	//in camera coordinate
				double const yc = ry + t[1];	//in camera coordinate
				double const zc = rz + t[2];	//in camera coordinate
				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy
				double const iu = fx * xc / zc + cx;	//image pixel
				double const iv = fy * yc / zc + cy;	//image pixel

				residuals[0] = observered_x_ - iu;
				residuals[1] = observered_y_ - iv;

				/*
				const double y = parameters[1][0];
				compute the reprojection error
				std::cerr << "w1,w2,w3: " << w1 << ", " << w2 << ", " << w3 << std::endl;
				std::cerr << "x,y,z: " << x << ", " << y << ", " << z << std::endl;				
				residuals[0] = observered_x_ - w0 * 2 - w1 * 3 - w2 * 4 - t0 * 2 - t1 * 2 - t2 * 2 - x * 2 - y * 2 - z * 2;
				residuals[1] = observered_y_ - w0 * 4 - w1 * 3 - w2 * 2 - t0 * 2 - t1 * 2 - t2 * 2 - x * 2 - y * 2 - z * 2;
				*/

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J(jacobians[0]);

					//compute the jacobians
					//rx = r_0P; ry = r_1P; rz = r_2P
					J(0, 0) = -fx*xc*ry / (zc*zc);	
					J(0, 1) = fx*rz / zc + (fx*xc*rx) / (zc*zc);
					J(0, 2) = -fx*ry / zc;
					J(0, 3) = fx / zc;
					J(0, 4) = 0.0;
					J(0, 5) = -fx*xc / (zc*zc);
					J(0, 6) = fx*w[0] / zc - fx*xc*w[6] / (zc*zc);
					J(0, 7) = fx*w[1] / zc - fx*xc*w[7] / (zc*zc);
					J(0, 8) = fx*w[2] / zc - fx*xc*w[8] / (zc*zc);

					J(1, 0) = -fy*rz / zc - (fy*yc*ry) / (zc*zc);
					J(1, 1) = fy*yc*rx / (zc*zc);
					J(1, 2) = fy*rx / zc;
					J(1, 3) = 0;
					J(1, 4) = fy / zc;
					J(1, 5) = -fy*yc / (zc*zc);
					J(1, 6) = fy*w[3] / zc - fy*yc*w[6] / (zc*zc);
					J(1, 7) = fy*w[4] / zc - fy*yc*w[7] / (zc*zc);
					J(1, 8) = fy*w[5] / zc - fy*yc*w[8] / (zc*zc);

					//jacobians[0][0] = -2 * y;
					//jacobians[0][1] = -x;
				}
				/*if (jacobians != NULL && jacobians[1] != NULL)
				{
				jacobians[1][0] = -y;
				jacobians[1][1] = -x;
				}*/
				return true;
			}

			Eigen::Matrix3d K_;
			double observered_x_;
			double observered_y_;
		};

		class PoseGrapLineFunc : public ceres::SizedCostFunction<2, 8>
		{
		public:
			PoseGrapLineFunc(double ob_x, double ob_y)
				:observered_x_(ob_x), observered_y_(ob_y)
			{
				K_ = Eigen::Matrix3d::Identity();
			}

			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}
			virtual ~PoseGrapLineFunc() {};
			
			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				matrix->(1,8)
				parameters: (w[0],w[1],w[2],t[0],t[1],t[2],d[0],d[1])
				camera parameters: (w[0], w[1], w[2], t[0], t[1], t[2])
				p3d parameters: (X[0], X[1], X[2])
				*/
				if (jacobians != NULL && jacobians[0] != NULL)
				{

				}
				return true;
			}

		private:
			Eigen::Matrix3d K_;
			double observered_x_;
			double observered_y_;
		};

#if 0
		//wrong... check it if necessary
		class GrapLineFittingFunc : public ceres::SizedCostFunction<3, 6>
		{
		public:
			GrapLineFittingFunc(const Eigen::Vector3d& ob_s)
			{
				observered_s_ = ob_s;
				K_ = Eigen::Matrix3d::Identity();
				RT_ = Eigen::Matrix4d::Identity();
			}
			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}
			void SetRTMatrix4d(const Eigen::Matrix4d& rt)
			{
				RT_ = rt;
			}
			virtual ~GrapLineFittingFunc() {};
			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				matrix->(1,6)
				parameters: (m[0],m[1],m[2],d[0],d[1],d[2])
				*/
				//get line function 
				Eigen::Vector3d L;
				L[0] = parameters[0][0];
				L[1] = parameters[0][1];
				L[2] = parameters[0][2];
				Eigen::Vector3d D;
				D[0] = parameters[0][3];
				D[1] = parameters[0][4];
				D[2] = parameters[0][5];
				//get camera intrinsic
				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy
				//get camera extrinsic
				Eigen::Vector3d T = RT_.topRightCorner(3, 1);
				Eigen::Matrix3d RC = RT_.topLeftCorner(3, 3);
				Eigen::Matrix3d R_tr = RC.transpose();	//transpose
				//pnt in image coordinate to pnt in camera coordinate 
				double const xc_s = observered_s_[2] * (observered_s_[0] - cx) / fx;	//in camera coordinate
				double const yc_s = observered_s_[2] * (observered_s_[1] - cy) / fy;	//in camera coordinate
				double const zc_s = observered_s_[2];	//in camera coordinate
				double const xcT0_s = xc_s - T[0];
				double const ycT1_s = yc_s - T[1];
				double const zcT2_s = zc_s - T[2];
				Eigen::Vector3d pw_s;
				pw_s[0] = R_tr(0, 0)*xcT0_s + R_tr(0, 1)*ycT1_s + R_tr(0, 2)*zcT2_s;	//x in world coordinate
				pw_s[1] = R_tr(1, 0)*xcT0_s + R_tr(1, 1)*ycT1_s + R_tr(1, 2)*zcT2_s;	//y in world coordinate
				pw_s[2] = R_tr(2, 0)*xcT0_s + R_tr(2, 1)*ycT1_s + R_tr(2, 2)*zcT2_s;	//z in world coordinate
				//Eigen::Matrix3d skew_pw = HW::ComputeSkewMatrixFromVector3d(pw);
				Eigen::Vector3d pwD_s = pw_s.cross(D);
				residuals[0] = L[0] - pwD_s[0];
				residuals[1] = L[1] - pwD_s[1];
				residuals[2] = L[2] - pwD_s[2];
				std::cerr << "pw_s: " << pw_s.transpose() << std::endl;
				std::cerr << "pwD_s: " << pwD_s.transpose() << std::endl;
				////pnt in image coordinate to pnt in camera coordinate 
				//double const xc_e = observered_e_[2] * (observered_e_[0] - cx) / fx;	//in camera coordinate
				//double const yc_e = observered_e_[2] * (observered_e_[1] - cy) / fy;	//in camera coordinate
				//double const zc_e = observered_e_[2];	//in camera coordinate
				//double const xcT0_e = xc_e - T[0];
				//double const ycT1_e = yc_e - T[1];
				//double const zcT2_e = zc_e - T[2];
				//Eigen::Vector3d pw_e;
				//pw_e[0] = R_tr(0, 0)*xcT0_e + R_tr(0, 1)*ycT1_e + R_tr(0, 2)*zcT2_e;	//x in world coordinate
				//pw_e[1] = R_tr(1, 0)*xcT0_e + R_tr(1, 1)*ycT1_e + R_tr(1, 2)*zcT2_e;	//y in world coordinate
				//pw_e[2] = R_tr(2, 0)*xcT0_e + R_tr(2, 1)*ycT1_e + R_tr(2, 2)*zcT2_e;	//z in world coordinate
				////Eigen::Matrix3d skew_pw = HW::ComputeSkewMatrixFromVector3d(pw);
				//Eigen::Vector3d pwD_e = pw_e.cross(D);
				//residuals[3] = L[0] - pwD_e[0];
				//residuals[4] = L[1] - pwD_e[1];
				//residuals[5] = L[2] - pwD_e[2];
				if (jacobians != NULL && jacobians[0] != NULL)
				{
					//
					Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J(jacobians[0]);

					//compute the jacobians[L,D] D is constraint
					J(0, 0) = 1.0;
					J(0, 1) = 0.0;
					J(0, 2) = 0.0;
					J(0, 3) = 0.0;
					J(0, 4) = -pw_s[2];
					J(0, 5) = pw_s[1];

					J(1, 0) = 0.0;
					J(1, 1) = 1.0;
					J(1, 2) = 0.0;
					J(1, 3) = pw_s[2];;
					J(1, 4) = 0.0;
					J(1, 5) = -pw_s[0];

					J(2, 0) = 0.0;
					J(2, 1) = 0.0;
					J(2, 2) = 1.0;
					J(2, 3) = -pw_s[1];
					J(2, 4) = pw_s[0];
					J(2, 5) = 0.0;
				}
				return true;
			}

		private:
			Eigen::Matrix3d K_;
			Eigen::Matrix4d RT_;
			Eigen::Vector3d observered_s_;	//(observered_s_[0],observered_s_[1]) = (u,v); observered_s_[2] = d
			//Eigen::Vector3d observered_e_;	//(observered_e_[0],observered_e_[1]) = (u,v); observered_e_[2] = d
		};
#endif

		struct LineFitCostFunctor 
		{
			LineFitCostFunctor(double x, double y, double z)
				:x_(x), y_(y), z_(z){}

			template<typename T>
			bool operator()(const T* px, const T* py, const T* pz, 
				const T* dir_x, const T* dir_y, const T* dir_z, T* residual) const
			{
				//define the point on the line
				T p[3] = { T(px[0]), T(py[0]), T(pz[0]) };
				//define the direction vector of the line
				T dir[3] = { T(dir_x[0]), T(dir_y[0]), T(dir_z[0]) };
				//compute the projection of the point on the line
				T t = (x_ - p[0])*dir[0] + (y_ - p[1])*dir[1] + (z_ - p[2])*dir[2];
				T proj[3] = { p[0] + t*dir[0], p[1] + t*dir[1],  p[2] + t*dir[2] };
				//compute the distance between the point and the line
				residual[0] = sqrt(pow(proj[0] - x_, 2) + pow(proj[1] - y_, 2) + pow(proj[2] - z_, 2));
				return true;
			}

		private:
			const double x_;
			const double y_;
			const double z_;
		};

		//
		class Pnts2PlaneCostFunc:public ceres::SizedCostFunction<1, 4>
		{
		public:
			Pnts2PlaneCostFunc(double x, double y, double z)
				:x_(x), y_(y), z_(z) 
			{
				K_ = Eigen::Matrix3d::Identity();
				RT_ = Eigen::Matrix4d::Identity();
			}
			
			void SetKMarix(const Eigen::Matrix3d& K)
			{
				K_ = K;
			}

			void SetRTMarix(const Eigen::Matrix4d& RT)
			{
				RT_ = RT;
			}

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				const double a = parameters[0][0];
				const double b = parameters[0][1];
				const double c = parameters[0][2];
				const double d = parameters[0][3];


				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy

				Eigen::Vector3d pc;
				pc[0] = d * (x_ - cx) / fx;	//in camera coordinate
				pc[1] = d * (y_ - cy) / fy;	//in camera coordinate
				pc[2] = z_;	//in camera coordinate

				//compute the pnt in world coordinate
				Eigen::Matrix3d RTR = RT_.topLeftCorner(3, 3);
				Eigen::Vector3d RTT = RT_.topRightCorner(3, 1);
				Eigen::Matrix3d RTRT = RTR.transpose();

				Eigen::Vector3d pw = RTRT*(pc - RTT);
				residuals[0] = a*pw[0] + b*pw[1] + c*pw[2] + d;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					jacobians[0][0] = pw[0];
					jacobians[0][1] = pw[1];
					jacobians[0][2] = pw[2];
					jacobians[0][3] = 1.0;
				}
				return true;
			}
		private:
			const double x_;	//in image coordinate
			const double y_;	//in image coordinate
			const double z_;	//depth

			Eigen::Matrix3d K_;
			Eigen::Matrix4d RT_;
		};

		//two pnts can index to two plane(polygon margin), one pnt to one plane 
		class PoseGrapPnt2PlaneFunc : public ceres::SizedCostFunction<1, 11>
		{
		public:
			PoseGrapPnt2PlaneFunc(double obu, double obv)
			{
				observered_u_ = obu;
				observered_v_ = obv;
				K_ = Eigen::Matrix3d::Identity();
			}

			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}
			virtual ~PoseGrapPnt2PlaneFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				matrix->(1,11)
				parameters: (w[0],w[1],w[2],t[0],t[1],t[2],PL[0],PL[1],PL[2],PL[3], d)
				camera parameters: (w[0], w[1], w[2], t[0], t[1], t[2])
				plane parameters: (PL[0],PL[1],PL[2])
				image pnt depth parameters: (d)
				*/
				double w[3];
				double w_rotation[9];
				w[0] = parameters[0][0];
				w[1] = parameters[0][1];
				w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(w, w_rotation);	//world to camera

				double t[3];
				t[0] = parameters[0][3];
				t[1] = parameters[0][4];
				t[2] = parameters[0][5];

				double PL[4];
				PL[0] = parameters[0][6];
				PL[1] = parameters[0][7];
				PL[2] = parameters[0][8];
				PL[2] = parameters[0][9];

				double d;
				d = parameters[0][10];
				
				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy

				//double const rx = w[0] * p3d[0] + w[1] * p3d[1] + w[2] * p3d[2];	//row major
				//double const ry = w[3] * p3d[0] + w[4] * p3d[1] + w[5] * p3d[2];	//row major
				//double const rz = w[6] * p3d[0] + w[7] * p3d[1] + w[8] * p3d[2];	//row major

				double const xc = d * (observered_u_ - cx) / fx;	//in camera coordinate
				double const yc = d * (observered_v_ - cy) / fy;	//in camera coordinate
				double const zc = d;	//in camera coordinate
				
				//camera to world coordinate
				double rc2w[9];
				HW::ConvertMatrix3d_to_Matrix3d_Transposed(w_rotation, rc2w);
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

				double pl_value = PL[0] * PL[0] + PL[1] * PL[1] + PL[2] * PL[2];
				double pl_sqrt = std::sqrtl(pl_value);
				double p2pl = (PL[0] * pw[0] + PL[1] * pw[1] + PL[2] * pw[2] + PL[4]);

				//compute the residuals, world pnt to plane distance
				residuals[0] = p2pl / pl_sqrt;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					//to do next
					double pl_square = std::pow(pl_value, 2);
					Eigen::Map<Eigen::Matrix<double, 1, 11, Eigen::RowMajor>> J(jacobians[0]);

					double rv0 = rc2w[0] * (xc - t[0]) + rc2w[1] * (yc - t[1]) + rc2w[2] * (zc - t[2]);
					double rv1 = rc2w[3] * (xc - t[0]) + rc2w[4] * (yc - t[1]) + rc2w[5] * (zc - t[2]);
					double rv2 = rc2w[6] * (xc - t[0]) + rc2w[7] * (yc - t[1]) + rc2w[8] * (zc - t[2]);

					J(0, 0) = (PL[1] * rv2 - PL[2] * rv1) / pl_sqrt;
					J(0, 1) = (-PL[0] * rv2 + PL[2] * rv0) / pl_sqrt;
					J(0, 2) = (PL[0] * rv1 - PL[1] * rv0) / pl_sqrt;
					J(0, 3) = -(PL[0] * rc2w[0] + PL[1] * rc2w[3] + PL[2] * rc2w[6]) / pl_sqrt;
					J(0, 4) = -(PL[0] * rc2w[1] + PL[1] * rc2w[4] + PL[2] * rc2w[7]) / pl_sqrt;
					J(0, 5) = -(PL[0] * rc2w[2] + PL[1] * rc2w[5] + PL[2] * rc2w[8]) / pl_sqrt;
					J(0, 6) = (pw[0] * pl_sqrt - PL[0] * p2pl / pl_sqrt) / pl_value;
					J(0, 7) = (pw[1] * pl_sqrt - PL[1] * p2pl / pl_sqrt) / pl_value;
					J(0, 8) = (pw[2] * pl_sqrt - PL[2] * p2pl / pl_sqrt) / pl_value;
					J(0, 9) = 1 / pl_sqrt;
					J(0, 10) = PL[0] * (rc2w[0] * (observered_u_ - cx) / fx + rc2w[1] * (observered_v_ - cy) / fy + rc2w[2]) / pl_sqrt
						+ PL[1] * (rc2w[3] * (observered_u_ - cx) / fx + rc2w[4] * (observered_v_ - cy) / fy + rc2w[5]) / pl_sqrt
						+ PL[2] * (rc2w[6] * (observered_u_ - cx) / fx + rc2w[7] * (observered_v_ - cy) / fy + rc2w[8]) / pl_sqrt;	//d derive
				}
				return true;
			}

		private:
			Eigen::Matrix3d K_;
			double observered_u_;	//u
			double observered_v_;	//v
		};

		class PoseGraphPnt2PlaneCostFunc : public ceres::SizedCostFunction<1, 7>
		{
		public:
			PoseGraphPnt2PlaneCostFunc(double obu, double obv)
			{
				observered_u_ = obu;
				observered_v_ = obv;
				K_ = Eigen::Matrix3d::Identity();
			}

			void SetKMatrix(const Eigen::Matrix3d& K)	//set the intrisinc matrix
			{
				K_ = K;
			}

			void SetPlaneFunc(const Eigen::Vector4d& fuc)
			{
				plane_fuc_ = fuc;
			}

			virtual ~PoseGraphPnt2PlaneCostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				matrix->(1,7)
				parameters: (w[0], w[1], w[2], t[0], t[1], t[2], d)
				camera parameters: (w[0], w[1], w[2], t[0], t[1], t[2])
				image pnt depth parameters: (d)
				*/
				double w[3];
				double w_rotation[9];
				w[0] = parameters[0][0];
				w[1] = parameters[0][1];
				w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(w, w_rotation);	//world to camera

				double t[3];
				t[0] = parameters[0][3];
				t[1] = parameters[0][4];
				t[2] = parameters[0][5];

				/*double PL[4];
				PL[0] = parameters[0][6];
				PL[1] = parameters[0][7];
				PL[2] = parameters[0][8];
				PL[2] = parameters[0][9];*/

				double d;
				d = parameters[0][10];

				double const fx = K_(0, 0);		//fx
				double const fy = K_(1, 1);		//fy
				double const cx = K_(0, 2);		//cx
				double const cy = K_(1, 2);		//cy

				//double const rx = w[0] * p3d[0] + w[1] * p3d[1] + w[2] * p3d[2];	//row major
				//double const ry = w[3] * p3d[0] + w[4] * p3d[1] + w[5] * p3d[2];	//row major
				//double const rz = w[6] * p3d[0] + w[7] * p3d[1] + w[8] * p3d[2];	//row major

				double const xc = d * (observered_u_ - cx) / fx;	//in camera coordinate
				double const yc = d * (observered_v_ - cy) / fy;	//in camera coordinate
				double const zc = d;	//in camera coordinate

				//camera to world coordinate
				double rc2w[9];
				HW::ConvertMatrix3d_to_Matrix3d_Transposed(w_rotation, rc2w);
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
				residuals[0] = plane_fuc_[0]*pw[0] + plane_fuc_[1] * pw[1] + plane_fuc_[2] * pw[2] + plane_fuc_[3];

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					//to do next
					//double pl_square = std::pow(pl_value, 2);
					Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J(jacobians[0]);

					double rv0 = rc2w[0] * (xc - t[0]) + rc2w[1] * (yc - t[1]) + rc2w[2] * (zc - t[2]);
					double rv1 = rc2w[3] * (xc - t[0]) + rc2w[4] * (yc - t[1]) + rc2w[5] * (zc - t[2]);
					double rv2 = rc2w[6] * (xc - t[0]) + rc2w[7] * (yc - t[1]) + rc2w[8] * (zc - t[2]);

					J(0, 0) = (plane_fuc_[1] * rv2 - plane_fuc_[2] * rv1);
					J(0, 1) = (-plane_fuc_[0] * rv2 + plane_fuc_[2] * rv0);
					J(0, 2) = (plane_fuc_[0] * rv1 - plane_fuc_[1] * rv0);
					J(0, 3) = -(plane_fuc_[0] * rc2w[0] + plane_fuc_[1] * rc2w[3] + plane_fuc_[2] * rc2w[6]);
					J(0, 4) = -(plane_fuc_[0] * rc2w[1] + plane_fuc_[1] * rc2w[4] + plane_fuc_[2] * rc2w[7]);
					J(0, 5) = -(plane_fuc_[0] * rc2w[2] + plane_fuc_[1] * rc2w[5] + plane_fuc_[2] * rc2w[8]);
					J(0, 6) = plane_fuc_[0] * (rc2w[0] * (observered_u_ - cx) / fx + rc2w[1] * (observered_v_ - cy) / fy + rc2w[2])
						+ plane_fuc_[1] * (rc2w[3] * (observered_u_ - cx) / fx + rc2w[4] * (observered_v_ - cy) / fy + rc2w[5])
						+ plane_fuc_[2] * (rc2w[6] * (observered_u_ - cx) / fx + rc2w[7] * (observered_v_ - cy) / fy + rc2w[8]);	//d derive
				}
				return true;
			}

		private:
			Eigen::Matrix3d K_;
			Eigen::Vector4d plane_fuc_;
			double observered_u_;	//u
			double observered_v_;	//v
		};

		HWBundleAdjustment::HWBAStatus HWBundleAdjustment::optimize()
		{
			HWBundleAdjustment::HWBAStatus tmp_status;

			//do it
			lm_optimize();

			return tmp_status;
		}

		void HWBundleAdjustment::FittingLine3dFromPnts3d(const std::vector<Eigen::Vector3d>& pnts, Eigen::Vector4d& line_func)
		{
			Eigen::Vector3d centroid(0.0, 0.0, 0.0);
			for (int i = 0; i < pnts.size(); ++i)
			{
				centroid += pnts[i];
			}
			centroid /= pnts.size();
			//compute the covariance matrix of the points
			Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
			for (int i = 0; i < pnts.size(); ++i)
			{
				cov += (pnts[i] - centroid)*(pnts[i] - centroid).transpose();
			}
			//compute the eigenvectors and eigenvalues of the convariance matrix
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
			Eigen::Vector3d ld = eigensolver.eigenvectors().col(2);
			
			//test
			Eigen::Vector3d lv = eigensolver.eigenvalues();
			Eigen::Vector3d lvec0 = eigensolver.eigenvectors().col(0);
			Eigen::Vector3d lvec1 = eigensolver.eigenvectors().col(1);
			Eigen::Vector3d lvec2 = eigensolver.eigenvectors().col(2);
			std::cerr << "lv: " << lv.transpose() << std::endl;
			std::cerr << "lvec 0: " << lvec0.transpose() << std::endl;
			std::cerr << "lvec 1: " << lvec1.transpose() << std::endl;
			std::cerr << "lvec 2: " << lvec2.transpose() << std::endl;
			//end test

			std::cerr << "ld: " << ld.transpose() << std::endl;
			
			//compute the start pnt of the line
			Eigen::Vector3d ls = centroid - ld*(centroid.dot(ld) / ld.dot(ld));
			//compute the end pnt of the line
			Eigen::Vector3d le = centroid + ld*(centroid.dot(ld) / ld.dot(ld));
			line_func[0] = ld[0];
			line_func[1] = ld[1];
			line_func[2] = ld[2];
			//compute the line function d

			std::cerr << "Equation of the line: (" << ls.transpose() << ") + t*(" << ld.transpose() << ")" << std::endl;
		}

		void HWBundleAdjustment::print_status() const
		{

		}

		void HWBundleAdjustment::lm_optimize()
		{
			////using ceres to optimize the camera pose, pnt 3d and line 3d
			//Eigen::Matrix3f c_r = Eigen::Matrix3f::Identity();
			////c_r.Identity();
			//float c_theta = std::acosf((c_r.trace() - 1) / 2);
			//Eigen::Vector3f c_w = Eigen::Matrix3f::Identity();
			////c_w.Identity();
			//if (std::abs(c_theta) < HW::KMIN_FLOAT_THRESHOLD)
			//{
			//	c_w[0] = c_r(2, 1) - c_r(1, 2);
			//	c_w[1] = c_r(0, 2) - c_r(2, 0);
			//	c_w[2] = c_r(1, 0) - c_r(0, 1);
			//}
			//c_w.Identity();
			//Eigen::Vector3f tmp_cw = c_w*c_theta;

# if 0
			double x = 1.0;
			const double initial_x = x;
			double y = 1.0;
			const double initial_y = y;
			double xy[2];
			xy[0] = x;
			xy[1] = y;
			ceres::Problem problem;
			ceres::CostFunction * costfunction;
			double obx = 9;
			double oby = 7;
			costfunction = new PoseGraphcostFunc(obx, oby);
			problem.AddResidualBlock(costfunction, NULL, xy);
			ceres::Solver::Options options;
			options.minimizer_progress_to_stdout = true;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);

			std::cerr << summary.BriefReport() << "\n";
			std::cerr << "x, y: " << initial_x << " " << initial_y << " -> " << xy[0] << " " << xy[1] << "\n";


			Eigen::Vector2d ob1 = Eigen::Vector2d(9, 7);
			Eigen::Vector2d ob2 = Eigen::Vector2d(10, 5);
			Eigen::Vector2d ob3 = Eigen::Vector2d(11, 3);
			Eigen::Vector2d ob4 = Eigen::Vector2d(20, 10);
			Eigen::Vector2d ob5 = Eigen::Vector2d(5, 14);
			Eigen::Vector2d ob6 = Eigen::Vector2d(15, 22);

			/*Eigen::Vector2d ob1 = Eigen::Vector2d(9, 7);
			Eigen::Vector2d ob2 = Eigen::Vector2d(10, 5);
			Eigen::Vector2d ob3 = Eigen::Vector2d(11, 3);
			Eigen::Vector2d ob4 = Eigen::Vector2d(20, 10);
			Eigen::Vector2d ob5 = Eigen::Vector2d(5, 14);
			Eigen::Vector2d ob6 = Eigen::Vector2d(15, 22);*/

			double xy[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			double xy_initial[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			double xxyy[2][6];
			xxyy[0][0] = 0.0; xxyy[0][1] = 0.0; xxyy[0][2] = 0.0;
			xxyy[0][3] = 0.0; xxyy[0][4] = 0.0; xxyy[0][5] = 0.0;

			xxyy[1][0] = 0.0; xxyy[1][1] = 0.0; xxyy[1][2] = 0.0;
			xxyy[1][3] = 0.0; xxyy[1][4] = 0.0; xxyy[1][5] = 0.0;

			ceres::Problem problem;
			ceres::CostFunction * costfunction;

			for (int i = 0; i < 2; ++i)
			{
				costfunction = new PoseGraphcostFunc(ob1[0], ob1[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob2[0], ob2[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob3[0], ob3[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob4[0], ob4[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob5[0], ob5[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob6[0], ob6[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);
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
			ceres::Solve(options, &problem, &summary);

			std::cerr << summary.BriefReport() << "\n";

			std::cerr << "xy_initial: " << xy_initial[0] << ", " << xy_initial[1] << ", " << xy_initial[2] << ", "
				<< xy_initial[3] << ", " << xy_initial[4] << ", " << xy_initial[5] << std::endl;
			/*std::cerr << "xy: " << xy[0] << ", " << xy[1] << ", " << xy[2] << ", "
			<< xy[3] << ", " << xy[4] << ", " << xy[5] << std::endl;*/

			std::cerr << "xy_0: " << xxyy[0][0] << ", " << xxyy[0][1] << ", " << xxyy[0][2] << ", "
				<< xxyy[0][3] << ", " << xxyy[0][4] << ", " << xxyy[0][5] << std::endl;
			std::cerr << "xy_1: " << xxyy[1][0] << ", " << xxyy[1][1] << ", " << xxyy[1][2] << ", "
				<< xxyy[1][3] << ", " << xxyy[1][4] << ", " << xxyy[1][5] << std::endl;

			//std::cerr << "x, y: " << initial_x << " " << initial_y << " -> " << xy[0] << " " << xy[1] << "\n";


			Eigen::Vector2d ob1 = Eigen::Vector2d(9, 7);
			Eigen::Vector2d ob2 = Eigen::Vector2d(10, 5);
			Eigen::Vector2d ob3 = Eigen::Vector2d(11, 3);
			Eigen::Vector2d ob4 = Eigen::Vector2d(20, 10);
			Eigen::Vector2d ob5 = Eigen::Vector2d(5, 14);
			Eigen::Vector2d ob6 = Eigen::Vector2d(15, 22);
			Eigen::Vector2d ob7 = Eigen::Vector2d(22, 80);
			Eigen::Vector2d ob8 = Eigen::Vector2d(36, 100);
			Eigen::Vector2d ob9 = Eigen::Vector2d(40, 108);

			/*Eigen::Vector2d ob1 = Eigen::Vector2d(9, 7);
			Eigen::Vector2d ob2 = Eigen::Vector2d(10, 5);
			Eigen::Vector2d ob3 = Eigen::Vector2d(11, 3);
			Eigen::Vector2d ob4 = Eigen::Vector2d(20, 10);
			Eigen::Vector2d ob5 = Eigen::Vector2d(5, 14);
			Eigen::Vector2d ob6 = Eigen::Vector2d(15, 22);*/

			double xy[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			double xy_initial[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			double xxyy[2][9];
			xxyy[0][0] = 0.0; xxyy[0][1] = 0.0; xxyy[0][2] = 0.0;
			xxyy[0][3] = 0.0; xxyy[0][4] = 0.0; xxyy[0][5] = 0.0;
			xxyy[0][6] = 0.0; xxyy[0][7] = 0.0; xxyy[0][8] = 0.0;

			xxyy[1][0] = 0.0; xxyy[1][1] = 0.0; xxyy[1][2] = 0.0;
			xxyy[1][3] = 0.0; xxyy[1][4] = 0.0; xxyy[1][5] = 0.0;
			xxyy[1][6] = 0.0; xxyy[1][7] = 0.0; xxyy[1][8] = 0.0;

			ceres::Problem problem;
			ceres::CostFunction * costfunction;

			for (int i = 0; i < 2; ++i)
			{
				costfunction = new PoseGraphcostFunc(ob1[0], ob1[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob2[0], ob2[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob3[0], ob3[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob4[0], ob4[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob5[0], ob5[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob6[0], ob6[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob7[0], ob7[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob8[0], ob8[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob9[0], ob9[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);
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
			ceres::Solve(options, &problem, &summary);

			std::cerr << summary.BriefReport() << "\n";

			std::cerr << "xy_initial: " << xy_initial[0] << ", " << xy_initial[1] << ", " << xy_initial[2] << ", "
				<< xy_initial[3] << ", " << xy_initial[4] << ", " << xy_initial[5] << ", " << xy_initial[6] << ", " << xy_initial[7] << ", " << xy_initial[8] << std::endl;
			/*std::cerr << "xy: " << xy[0] << ", " << xy[1] << ", " << xy[2] << ", "
			<< xy[3] << ", " << xy[4] << ", " << xy[5] << std::endl;*/

			std::cerr << "xy_0: " << xxyy[0][0] << ", " << xxyy[0][1] << ", " << xxyy[0][2] << ", "
				<< xxyy[0][3] << ", " << xxyy[0][4] << ", " << xxyy[0][5] << ", " << xxyy[0][6] << ", " << xxyy[0][7] << ", " << xxyy[0][8] << std::endl;
			std::cerr << "xy_1: " << xxyy[1][0] << ", " << xxyy[1][1] << ", " << xxyy[1][2] << ", "
				<< xxyy[1][3] << ", " << xxyy[1][4] << ", " << xxyy[1][5] << ", " << xxyy[1][6] << ", " << xxyy[1][7] << ", " << xxyy[1][8] << std::endl;

			//std::cerr << "x, y: " << initial_x << " " << initial_y << " -> " << xy[0] << " " << xy[1] << "\n";

#endif


#if 0
			double parameters[2] = { -1.2, 1.0 };
			ceres::FirstOrderFunction* function = new Rosenbrock;
			ceres::GradientProblem problem(function);
			ceres::GradientProblemSolver::Options options;
			options.minimizer_progress_to_stdout = true;
			ceres::GradientProblemSolver::Summary summary;
			ceres::Solve(options, problem, parameters, &summary);
			std::cout << summary.FullReport() << std::endl;
			std::cout << "Initial x = " << -1.2 << ", y = " << 1.0 << std::endl;
			std::cout << "Final   x = " << parameters[0] << ", y = " << parameters[1] << std::endl;
			//std::system("pause");
			//return;
#endif

#if 0
			Eigen::Vector2d ob1 = Eigen::Vector2d(9, 7);
			Eigen::Vector2d ob2 = Eigen::Vector2d(10, 5);
			Eigen::Vector2d ob3 = Eigen::Vector2d(11, 3);
			Eigen::Vector2d ob4 = Eigen::Vector2d(20, 10);
			Eigen::Vector2d ob5 = Eigen::Vector2d(5, 14);
			Eigen::Vector2d ob6 = Eigen::Vector2d(15, 22);
			Eigen::Vector2d ob7 = Eigen::Vector2d(22, 80);
			Eigen::Vector2d ob8 = Eigen::Vector2d(36, 100);
			Eigen::Vector2d ob9 = Eigen::Vector2d(40, 108);

			/*Eigen::Vector2d ob1 = Eigen::Vector2d(9, 7);
			Eigen::Vector2d ob2 = Eigen::Vector2d(10, 5);
			Eigen::Vector2d ob3 = Eigen::Vector2d(11, 3);
			Eigen::Vector2d ob4 = Eigen::Vector2d(20, 10);
			Eigen::Vector2d ob5 = Eigen::Vector2d(5, 14);
			Eigen::Vector2d ob6 = Eigen::Vector2d(15, 22);*/

			double xy[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			double xy_initial[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			
			double xxyy[2][9];
			xxyy[0][0] = 0.0; xxyy[0][1] = 0.0; xxyy[0][2] = 0.0; 
			xxyy[0][3] = 0.0; xxyy[0][4] = 0.0; xxyy[0][5] = 0.0;
			xxyy[0][6] = 0.0; xxyy[0][7] = 0.0; xxyy[0][8] = 0.0;

			xxyy[1][0] = 0.0; xxyy[1][1] = 0.0; xxyy[1][2] = 0.0;
			xxyy[1][3] = 0.0; xxyy[1][4] = 0.0; xxyy[1][5] = 0.0;
			xxyy[1][6] = 0.0; xxyy[1][7] = 0.0; xxyy[1][8] = 0.0;

			ceres::Problem problem;
			ceres::CostFunction * costfunction;

			for (int i = 0; i < 2; ++i)
			{
				costfunction = new PoseGraphcostFunc(ob1[0], ob1[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob2[0], ob2[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob3[0], ob3[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob4[0], ob4[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob5[0], ob5[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob6[0], ob6[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob7[0], ob7[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob8[0], ob8[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);

				costfunction = new PoseGraphcostFunc(ob9[0], ob9[1]);
				problem.AddResidualBlock(costfunction, NULL, xxyy[i]);
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
			ceres::Solve(options, &problem, &summary);

			std::cerr << summary.BriefReport() << "\n";

			std::cerr << "xy_initial: " << xy_initial[0] << ", " << xy_initial[1] << ", " << xy_initial[2] << ", "
				<< xy_initial[3] << ", " << xy_initial[4] << ", " << xy_initial[5] <<", " << xy_initial[6] << ", " << xy_initial[7] << ", " << xy_initial[8] << std::endl;
			/*std::cerr << "xy: " << xy[0] << ", " << xy[1] << ", " << xy[2] << ", "
				<< xy[3] << ", " << xy[4] << ", " << xy[5] << std::endl;*/

			std::cerr << "xy_0: " << xxyy[0][0] << ", " << xxyy[0][1] << ", " << xxyy[0][2] << ", "
				<< xxyy[0][3] << ", " << xxyy[0][4] << ", " << xxyy[0][5] <<", " << xxyy[0][6] << ", " << xxyy[0][7] << ", " << xxyy[0][8] << std::endl;
			std::cerr << "xy_1: " << xxyy[1][0] << ", " << xxyy[1][1] << ", " << xxyy[1][2] << ", "
				<< xxyy[1][3] << ", " << xxyy[1][4] << ", " << xxyy[1][5] <<", " << xxyy[1][6] << ", " << xxyy[1][7] << ", " << xxyy[1][8] << std::endl;

			//std::cerr << "x, y: " << initial_x << " " << initial_y << " -> " << xy[0] << " " << xy[1] << "\n";
#endif

#if 0
			ceres::Problem problem;
			ceres::CostFunction * costfunction;

			double line_params[6];
			line_params[0] = 1.0; line_params[1] = 1.0; line_params[2] = 1.0;
			line_params[3] = 1.0; line_params[4] = 1.0; line_params[5] = 1.0;
			double initial_params[6];
			for (int i = 0; i < 6; ++i)
			{
				initial_params[i] = line_params[i];
			}

			//line_params[]
			std::vector<Eigen::Vector3d> points;
			points.push_back(Eigen::Vector3d(27.0, 39.0, 2.0));
			points.push_back(Eigen::Vector3d(8.0, 5.0, 2.0));
			points.push_back(Eigen::Vector3d(8.0, 9.0, 2.0));
			points.push_back(Eigen::Vector3d(16.0, 22.0, 2.0));
			points.push_back(Eigen::Vector3d(44.0, 71.0, 2.0));
			points.push_back(Eigen::Vector3d(35.0, 44.0, 2.0));
			points.push_back(Eigen::Vector3d(43.0, 57.0, 2.0));
			points.push_back(Eigen::Vector3d(19.0, 24.0, 2.0));
			points.push_back(Eigen::Vector3d(27.0, 39.0, 2.0));
			points.push_back(Eigen::Vector3d(37.0, 52.0, 2.0));
			for (int i = 0; i < points.size(); ++i)
			{
				costfunction = new GrapLineFittingFunc(points[i]);
				problem.AddResidualBlock(costfunction, NULL, line_params);
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
			ceres::Solve(options, &problem, &summary);

			std::cerr << summary.BriefReport() << "\n";
			std::cerr << "initial line params: [" << initial_params[0] << " " << initial_params[1] << " " << initial_params[2] << " "
				<< initial_params[3] << " " << initial_params[4] << " " << initial_params[5] << " ]" << std::endl;
			std::cerr << "line params: [" << line_params[0] << " " << line_params[1] << " " << line_params[2] << " "
				<< line_params[3] << " " << line_params[4] << " " << line_params[5] << " ]" << std::endl;
#endif

#if 0	//use the covariance method (important)
			//line_params[]
			std::vector<Eigen::Vector3d> points;
			points.push_back(Eigen::Vector3d(1.0, 2.0, 3.0));
			points.push_back(Eigen::Vector3d(2.0, 4.0, 6.0));
			points.push_back(Eigen::Vector3d(3.0, 6.0, 9.0));
			points.push_back(Eigen::Vector3d(4.0, 8.0, 12.0));

			/*points.push_back(Eigen::Vector3d(27.0, 39.0, 2.0));
			points.push_back(Eigen::Vector3d(8.0, 5.0, 2.0));
			points.push_back(Eigen::Vector3d(8.0, 9.0, 2.0));
			points.push_back(Eigen::Vector3d(16.0, 22.0, 2.0));
			points.push_back(Eigen::Vector3d(44.0, 71.0, 2.0));
			points.push_back(Eigen::Vector3d(35.0, 44.0, 2.0));
			points.push_back(Eigen::Vector3d(43.0, 57.0, 2.0));
			points.push_back(Eigen::Vector3d(19.0, 24.0, 2.0));
			points.push_back(Eigen::Vector3d(27.0, 39.0, 2.0));
			points.push_back(Eigen::Vector3d(37.0, 52.0, 2.0));*/
			Eigen::Vector4d line_fc(0.0, 0.0, 0.0, 0.0);
			FittingLine3dFromPnts3d(points, line_fc);

#endif

#if 0	//plane fitting
			ceres::Problem problem;
			double x[3] = { 1,2,3 };
			double y[3] = { 4,5,6 };
			double z[3] = { 7,8,9 };
			double a[4];
			a[0] = 1;
			a[1] = 1;
			a[2] = 1;
			a[3] = 1;

			for (int i = 0; i < 3; ++i)
			{
				ceres::CostFunction* cost_fuction = new Pnts2PlaneCostFunc(x[i], y[i], z[i]);
				problem.AddResidualBlock(cost_fuction, nullptr, a);
			}
			ceres::Solver::Options options;
			options.max_num_iterations = 100;
			//options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
			options.minimizer_type = ceres::TRUST_REGION;
			//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			options.trust_region_strategy_type = ceres::DOGLEG;
			options.minimizer_progress_to_stdout = true;
			//options.dogleg_type = ceres::SUBSPACE_DOGLEG;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			std::cerr << summary.BriefReport() << "\n";
			std::cerr << "abcd: " << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << std::endl;
#endif

#if 0	//line fitting (use the least sqaure method with ceres)
			std::vector<Eigen::Vector3d> points;
			/*points.push_back(Eigen::Vector3d(1.0, 2.0, 3.0));
			points.push_back(Eigen::Vector3d(2.0, 4.0, 6.0));
			points.push_back(Eigen::Vector3d(3.0, 6.0, 9.0));
			points.push_back(Eigen::Vector3d(4.0, 8.0, 12.0));*/

			points.push_back(Eigen::Vector3d(27.0, 39.0, 2.0));
			points.push_back(Eigen::Vector3d(8.0, 5.0, 2.0));
			points.push_back(Eigen::Vector3d(8.0, 9.0, 2.0));
			points.push_back(Eigen::Vector3d(16.0, 22.0, 2.0));
			points.push_back(Eigen::Vector3d(44.0, 71.0, 2.0));
			points.push_back(Eigen::Vector3d(35.0, 44.0, 2.0));
			points.push_back(Eigen::Vector3d(43.0, 57.0, 2.0));
			points.push_back(Eigen::Vector3d(19.0, 24.0, 2.0));
			points.push_back(Eigen::Vector3d(27.0, 39.0, 2.0));
			points.push_back(Eigen::Vector3d(37.0, 52.0, 2.0));
			Eigen::Vector4d line_fc(0.0, 0.0, 0.0, 0.0);
			double dir[3] = {1,0,0};
			double p[3] = { 26.0,30.0,5.0 };
			ceres::Problem problem;
			for (int i = 0; i < points.size(); ++i)
			{
				ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<LineFitCostFunctor, 1,1,1,1,1,1,1>
					(new LineFitCostFunctor(points[i][0],points[i][1], points[i][2]));// = LineFitCostFunctor::Create(points[i]);
				problem.AddResidualBlock(cost_function, NULL, &p[0], &p[1], &p[2], &dir[0], &dir[1], &dir[2]);
			}
			ceres::Solver::Options options;
			options.max_num_iterations = 1000;
			//options.linear_solver_type = ceres::DENSE_QR;
			//Run the solver
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			//print the results
			std::cerr << summary.FullReport() << std::endl;
			std::cerr << "Line pnt: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
			std::cerr << "Line dir: " << dir[0] << ", " << dir[1] << ", " << dir[2] << std::endl;
			std::cerr << "final residual: " << summary.final_cost << std::endl;
#endif

#if 1
			for (int i = 0; i < observations_->size(); ++i)
			{
				//get cam id
				int cam_id = (*observations_)[i].camera_id;
				int pnt_id = (*observations_)[i].point_id;

			}
#endif
		}

		void HWBundleAdjustment::compute_reprojection_errors()
		{

		}

		void HWBundleAdjustment::analytic_jacobian()
		{

		}



#endif

	}
}