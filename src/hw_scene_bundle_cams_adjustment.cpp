#include"hw_scene_bundle_cams_adjustment.h"
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

		Eigen::Matrix3d VectorColMultiplyVectorRow3DZDGLocal(const Eigen::Vector3d& colv, const Eigen::Vector3d& rowv)
		{
			Eigen::Matrix3d tmp_m;
			tmp_m.block<1, 3>(0, 0) = colv[0] * rowv;
			tmp_m.block<1, 3>(1, 0) = colv[1] * rowv;
			tmp_m.block<1, 3>(2, 0) = colv[2] * rowv;
			return tmp_m;
		}

#if 0
		//optimize camera pose only. fix plane 3d, cost: e = (T0_wc * K0^(-1) * (u0,v0,1)) - (T1_wc * K1^(-1) * (u1,v1,1))
		class CamPoseOnlyFromReprojectNeighborPntsCostFunc : public ceres::SizedCostFunction<3, 6, 6>
		{
		public:
			CamPoseOnlyFromReprojectNeighborPntsCostFunc(const Eigen::Vector2d& src_ob, const Eigen::Vector2d& tgt_ob)
			{
				src_observation_[0] = src_ob[0];
				src_observation_[1] = src_ob[1];
				tgt_observation_[0] = tgt_ob[0];
				tgt_observation_[1] = tgt_ob[1];

				pnt3d_ = Eigen::Vector3d::Zero();	//delete next...
				src_K_ = Eigen::Matrix3d::Identity();
				tgt_K_ = Eigen::Matrix3d::Identity();
				energy_scale_ = 1.0;
			}
			void SetTwoNeighborKMatrix(const Eigen::Matrix3d& src_K, const Eigen::Matrix3d& tgt_K)	//set the intrisinc matrix
			{
				src_K_ = src_K;
				tgt_K_ = tgt_K;
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
			virtual ~CamPoseOnlyFromReprojectNeighborPntsCostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				*/
				double src_w[3];
				double src_wr[9];
				src_w[0] = parameters[0][0];
				src_w[1] = parameters[0][1];
				src_w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(src_w, src_wr);
				double src_t[3];
				src_t[0] = parameters[0][3];
				src_t[1] = parameters[0][4];
				src_t[2] = parameters[0][5];

				double tgt_w[3];
				double tgt_wr[9];
				tgt_w[0] = parameters[1][0];
				tgt_w[1] = parameters[1][1];
				tgt_w[2] = parameters[1][2];
				HW::rodrigues_to_matrix3d(tgt_w, tgt_wr);
				double tgt_t[3];
				tgt_t[0] = parameters[1][3];
				tgt_t[1] = parameters[1][4];
				tgt_t[2] = parameters[1][5];

				//compute src ray p3d ()
				Eigen::Matrix3d src_r_cw;
				src_r_cw << src_wr[0], src_wr[1], src_wr[2],
					src_wr[3], src_wr[4], src_wr[5],
					src_wr[6], src_wr[7], src_wr[8];
				Eigen::Vector3d src_cm_exrt_t
					= Eigen::Vector3d(src_t[0], src_t[1], src_t[2]);
				Eigen::Vector3d src_cm_c = -src_r_cw.transpose() * src_cm_exrt_t;
				Eigen::Vector3d src_ray_d = src_r_cw.transpose() * src_K_.inverse() * Eigen::Vector3d(src_observation_[0], src_observation_[1], 1.0);
				////test
				//std::cerr << "src_r_cw:  \n" << src_r_cw << std::endl;
				//std::cerr << "src_r_cw transpose: " << src_r_cw.transpose() << std::endl;
				//std::cerr << "src_r_cw inverse: " << src_r_cw.inverse() << std::endl;
				////end test
				Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info_[0], plane_info_[1], plane_info_[2]);
				double src_ray_len = (-plane_info_[3] - src_cm_c.dot(plane_n)) / src_ray_d.dot(plane_n);
				Eigen::Vector3d src_ray_plane_pnt = src_cm_c + src_ray_len * src_ray_d;
				//std::cerr << "src_ray_plane_pnt: " << src_ray_plane_pnt.transpose() << std::endl;

				//compute tgt ray p3d ()
				Eigen::Matrix3d tgt_r_cw;
				tgt_r_cw << tgt_wr[0], tgt_wr[1], tgt_wr[2],
					tgt_wr[3], tgt_wr[4], tgt_wr[5],
					tgt_wr[6], tgt_wr[7], tgt_wr[8];
				Eigen::Vector3d tgt_cm_exrt_t
					= Eigen::Vector3d(tgt_t[0], tgt_t[1], tgt_t[2]);
				Eigen::Vector3d tgt_cm_c = -tgt_r_cw.transpose() * tgt_cm_exrt_t;
				Eigen::Vector3d tgt_ray_d = tgt_r_cw.inverse() * tgt_K_.inverse() * Eigen::Vector3d(tgt_observation_[0], tgt_observation_[1], 1.0);

				////test
				//std::cerr << "tgt_r_cw:  \n" << tgt_r_cw << std::endl;
				//std::cerr << "tgt_r_cw transpose: " << tgt_r_cw.transpose() << std::endl;
				//std::cerr << "tgt_r_cw inverse: " << tgt_r_cw.inverse() << std::endl;
				////end test

				//Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info_[0], plane_info_[1], plane_info_[2]);
				double tgt_ray_len = (-plane_info_[3] - tgt_cm_c.dot(plane_n)) / tgt_ray_d.dot(plane_n);
				Eigen::Vector3d tgt_ray_plane_pnt = tgt_cm_c + tgt_ray_len * tgt_ray_d;
				//std::cerr << "tgt_ray_plane_pnt: " << tgt_ray_plane_pnt.transpose() << std::endl;

				//get dist to each other
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				residuals[0] = (src_ray_plane_pnt[0] - tgt_ray_plane_pnt[0]) * energy_scale_sqrt;
				residuals[1] = (src_ray_plane_pnt[1] - tgt_ray_plane_pnt[1]) * energy_scale_sqrt;
				residuals[2] = (src_ray_plane_pnt[2] - tgt_ray_plane_pnt[2]) * energy_scale_sqrt;

				Eigen::Vector3d jaco_resi_erro_to_src_p3d = Eigen::Vector3d::Ones()*energy_scale_sqrt;

				//compute src_cm_c to w 
				Eigen::Matrix3d jaco_src_cmc_to_w = HW::ComputeSkewMatrixFromVector3d(src_cm_c);
				//compute src_ray_d to w
				Eigen::Matrix3d jaco_src_ray_to_w = HW::ComputeSkewMatrixFromVector3d(src_ray_d);
				//compute jaco src_cm_c to t
				Eigen::Matrix3d jaco_src_cmc_to_t = -src_r_cw.transpose();
				//compute src_ray_d to t
				Eigen::Matrix3d jaco_src_ray_to_t;
				jaco_src_ray_to_t.setZero();

				//compute src_cm_c to w 
				Eigen::Matrix3d jaco_tgt_cmc_to_w = HW::ComputeSkewMatrixFromVector3d(tgt_cm_c);
				//compute src_ray_d to w
				Eigen::Matrix3d jaco_tgt_ray_to_w = HW::ComputeSkewMatrixFromVector3d(tgt_ray_d);
				//compute jaco src_cm_c to t
				Eigen::Matrix3d jaco_tgt_cmc_to_t = -tgt_r_cw.transpose();
				//compute src_ray_d to t
				Eigen::Matrix3d jaco_tgt_ray_to_t;
				jaco_tgt_ray_to_t.setZero();
				/*std::cerr << "observation_[0]->(iu), observation_[1]->(iv): " << observation_[0] << "->(" << iu << " )" << ", "
				<< observation_[1] << "->(" << iv << " )" << std::endl;*/

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J0(jacobians[0]);
					//compute the jacobians
					double srcray_plane = src_ray_d.dot(plane_n);
					double srccmc_plane = src_cm_c.dot(plane_n);

					//double srcray_dot_plane = src_ray_d.dot(plane_n);
					//double srccmc_dot_plane = src_cm_c.dot(plane_n);

					//
					Eigen::Vector3d jaco_src_dpokn_to_w = -jaco_src_cmc_to_w*plane_n;
					Eigen::Matrix3d jaco_src_dpoknd_to_w =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_src_dpokn_to_w, src_ray_d);
					Eigen::Matrix3d jaco_src_didpokn_to_w = jaco_src_ray_to_w*(-plane_info_[3] - srccmc_plane);
					Eigen::Vector3d jaco_src_dion_to_w = jaco_src_ray_to_w*plane_n;
					Eigen::Vector3d dponv = (-plane_info_[3] - srccmc_plane)*src_ray_d;

					//tgt_ray_d.dot(plane_n) to w
					//Eigen::Vector3d jaco_srcray_dot_plane_n_to_w = plane_n.transpose() * jaco_src_ray_to_w;
					//src_cm_c.dot(plane_n) to w
					//Eigen::Vector3d jaco_srccmc_dot_planen_to_w = plane_n.transpose() * jaco_src_cmc_to_w;
					//Eigen::Vector3d jaco_src_ray_len_to_w = plane_info_[3] * jaco_srcray_dot_plane_n_to_w / (srcray_dot_plane * srcray_dot_plane)
					//	+ srccmc_dot_plane * jaco_srcray_dot_plane_n_to_w / (srcray_dot_plane * srcray_dot_plane) - jaco_srccmc_dot_planen_to_w / srcray_dot_plane;



					Eigen::Matrix3d jaco_src_diondponv_to_w =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_src_dion_to_w, dponv);

					Eigen::Matrix3d jaco_src_pnt3d_to_w = jaco_src_cmc_to_w + (jaco_src_dpoknd_to_w + jaco_src_didpokn_to_w) / srcray_plane
						- (jaco_src_diondponv_to_w) / (srcray_plane*srcray_plane);

					Eigen::Vector3d jaco_src_dpokn_to_t = -jaco_src_cmc_to_t*plane_n;
					Eigen::Matrix3d jaco_src_dpoknd_to_t =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_src_dpokn_to_t, src_ray_d);

					//Eigen::Matrix3d jaco_src_cm_c_to_w = 

					//Eigen::Vector3d dponvt = (-plane_info_[3] - srccmc_plane)*src_ray_d;
					Eigen::Matrix3d jaco_src_pnt3d_to_t = jaco_src_cmc_to_t + jaco_src_dpoknd_to_t / srcray_plane;

					J0(0, 0) = jaco_src_pnt3d_to_w(0, 0) * energy_scale_sqrt;
					J0(0, 1) = jaco_src_pnt3d_to_w(0, 1) * energy_scale_sqrt;
					J0(0, 2) = jaco_src_pnt3d_to_w(0, 2) * energy_scale_sqrt;
					J0(0, 3) = jaco_src_pnt3d_to_t(0, 0) * energy_scale_sqrt;
					J0(0, 4) = jaco_src_pnt3d_to_t(0, 1) * energy_scale_sqrt;
					J0(0, 5) = jaco_src_pnt3d_to_t(0, 2) * energy_scale_sqrt;

					J0(1, 0) = jaco_src_pnt3d_to_w(1, 0) * energy_scale_sqrt;
					J0(1, 1) = jaco_src_pnt3d_to_w(1, 1) * energy_scale_sqrt;
					J0(1, 2) = jaco_src_pnt3d_to_w(1, 2) * energy_scale_sqrt;
					J0(1, 3) = jaco_src_pnt3d_to_t(1, 0) * energy_scale_sqrt;
					J0(1, 4) = jaco_src_pnt3d_to_t(1, 1) * energy_scale_sqrt;
					J0(1, 5) = jaco_src_pnt3d_to_t(1, 2) * energy_scale_sqrt;

					J0(2, 0) = jaco_src_pnt3d_to_w(2, 0) * energy_scale_sqrt;
					J0(2, 1) = jaco_src_pnt3d_to_w(2, 1) * energy_scale_sqrt;
					J0(2, 2) = jaco_src_pnt3d_to_w(2, 2) * energy_scale_sqrt;
					J0(2, 3) = jaco_src_pnt3d_to_t(2, 0) * energy_scale_sqrt;
					J0(2, 4) = jaco_src_pnt3d_to_t(2, 1) * energy_scale_sqrt;
					J0(2, 5) = jaco_src_pnt3d_to_t(2, 2) * energy_scale_sqrt;
				}
				if (jacobians != NULL && jacobians[1] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J1(jacobians[1]);

					//compute the jacobians
					double tgtray_plane = tgt_ray_d.dot(plane_n);
					double tgtcmc_plane = tgt_cm_c.dot(plane_n);
					//
					Eigen::Vector3d jaco_tgt_dpokn_to_w = -jaco_tgt_cmc_to_w*plane_n;
					Eigen::Matrix3d jaco_tgt_dpoknd_to_w =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_dpokn_to_w, tgt_ray_d);
					Eigen::Matrix3d jaco_tgt_didpokn_to_w = jaco_tgt_ray_to_w*(-plane_info_[3] - tgtcmc_plane);
					Eigen::Vector3d jaco_tgt_dion_to_w = jaco_tgt_ray_to_w*plane_n;
					Eigen::Vector3d dponv = (-plane_info_[3] - tgtcmc_plane)*tgt_ray_d;
					Eigen::Matrix3d jaco_tgt_diondponv_to_w =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_dion_to_w, dponv);

					Eigen::Matrix3d jaco_tgt_pnt3d_to_w = (jaco_tgt_dpoknd_to_w + jaco_tgt_didpokn_to_w) / tgtray_plane
						- (jaco_tgt_diondponv_to_w) / (tgtray_plane*tgtray_plane);

					Eigen::Vector3d jaco_tgt_dpokn_to_t = -jaco_tgt_cmc_to_t*plane_n;
					Eigen::Matrix3d jaco_tgt_dpoknd_to_t =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_dpokn_to_t, tgt_ray_d);
					//Eigen::Vector3d dponvt = (-plane_info_[3] - srccmc_plane)*src_ray_d;
					Eigen::Matrix3d jaco_tgt_pnt3d_to_t = jaco_tgt_dpoknd_to_t / tgtray_plane;

					J1(0, 0) = -jaco_tgt_pnt3d_to_w(0, 0) * energy_scale_sqrt;
					J1(0, 1) = -jaco_tgt_pnt3d_to_w(0, 1) * energy_scale_sqrt;
					J1(0, 2) = -jaco_tgt_pnt3d_to_w(0, 2) * energy_scale_sqrt;
					J1(0, 3) = -jaco_tgt_pnt3d_to_t(0, 0) * energy_scale_sqrt;
					J1(0, 4) = -jaco_tgt_pnt3d_to_t(0, 1) * energy_scale_sqrt;
					J1(0, 5) = -jaco_tgt_pnt3d_to_t(0, 2) * energy_scale_sqrt;

					J1(1, 0) = -jaco_tgt_pnt3d_to_w(1, 0) * energy_scale_sqrt;
					J1(1, 1) = -jaco_tgt_pnt3d_to_w(1, 1) * energy_scale_sqrt;
					J1(1, 2) = -jaco_tgt_pnt3d_to_w(1, 2) * energy_scale_sqrt;
					J1(1, 3) = -jaco_tgt_pnt3d_to_t(1, 0) * energy_scale_sqrt;
					J1(1, 4) = -jaco_tgt_pnt3d_to_t(1, 1) * energy_scale_sqrt;
					J1(1, 5) = -jaco_tgt_pnt3d_to_t(1, 2) * energy_scale_sqrt;

					J1(2, 0) = -jaco_tgt_pnt3d_to_w(2, 0) * energy_scale_sqrt;
					J1(2, 1) = -jaco_tgt_pnt3d_to_w(2, 1) * energy_scale_sqrt;
					J1(2, 2) = -jaco_tgt_pnt3d_to_w(2, 2) * energy_scale_sqrt;
					J1(2, 3) = -jaco_tgt_pnt3d_to_t(2, 0) * energy_scale_sqrt;
					J1(2, 4) = -jaco_tgt_pnt3d_to_t(2, 1) * energy_scale_sqrt;
					J1(2, 5) = -jaco_tgt_pnt3d_to_t(2, 2) * energy_scale_sqrt;
				}
				return true;
			}

		private:
			double energy_scale_;
			Eigen::Vector3d pnt3d_;
			Eigen::Vector4d plane_info_;
			Eigen::Matrix3d src_K_;
			Eigen::Matrix3d tgt_K_;
			Eigen::Vector2d src_observation_;
			Eigen::Vector2d tgt_observation_;
		};

		//optimize camera pose only. fix plane 3d,  plucker line cost: e = m1 - p_0 \times d1;
		class CamPoseOnlyFromReprojectNeighborLinesCostFunc : public ceres::SizedCostFunction<6, 6, 6>
		{
		public:
			CamPoseOnlyFromReprojectNeighborLinesCostFunc(const std::pair<Eigen::Vector2d, Eigen::Vector2d>& src_ob,
				const std::pair<Eigen::Vector2d, Eigen::Vector2d>& tgt_ob)
			{
				src_line_observation_ = src_ob;
				tgt_line_observation_ = tgt_ob;
				pnt3d_ = Eigen::Vector3d::Zero();	//delete next...
				src_K_ = Eigen::Matrix3d::Identity();
				tgt_K_ = Eigen::Matrix3d::Identity();
				energy_scale_ = 1.0;
			}
			void SetTwoNeighborKMatrix(const Eigen::Matrix3d& src_K, const Eigen::Matrix3d& tgt_K)	//set the intrisinc matrix
			{
				src_K_ = src_K;
				tgt_K_ = tgt_K;
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
			virtual ~CamPoseOnlyFromReprojectNeighborLinesCostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				*/
				double src_w[3];
				double src_wr[9];
				src_w[0] = parameters[0][0];
				src_w[1] = parameters[0][1];
				src_w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(src_w, src_wr);
				double src_t[3];
				src_t[0] = parameters[0][3];
				src_t[1] = parameters[0][4];
				src_t[2] = parameters[0][5];

				//compute src ray p3d ()
				Eigen::Matrix3d src_r_cw;
				src_r_cw << src_wr[0], src_wr[1], src_wr[2],
					src_wr[3], src_wr[4], src_wr[5],
					src_wr[6], src_wr[7], src_wr[8];
				Eigen::Vector3d src_cm_exrt_t
					= Eigen::Vector3d(src_t[0], src_t[1], src_t[2]);
				Eigen::Vector3d src_cm_c = -src_r_cw.transpose() * src_cm_exrt_t;
				//src ls ray plane pnt
				Eigen::Vector3d src_ls_ray_d = src_r_cw.inverse() * src_K_.inverse() * Eigen::Vector3d(src_line_observation_.first[0], src_line_observation_.first[1], 1.0);
				Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info_[0], plane_info_[1], plane_info_[2]);
				double src_ls_ray_len = (-plane_info_[3] - src_cm_c.dot(plane_n)) / src_ls_ray_d.dot(plane_n);
				Eigen::Vector3d src_ls_ray_plane_pnt = src_cm_c + src_ls_ray_len * src_ls_ray_d;
				//src le ray plane pnt
				Eigen::Vector3d src_le_ray_d = src_r_cw.inverse() * src_K_.inverse() * Eigen::Vector3d(src_line_observation_.second[0], src_line_observation_.second[1], 1.0);
				//Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info_[0], plane_info_[1], plane_info_[2]);
				double src_le_ray_len = (-plane_info_[3] - src_cm_c.dot(plane_n)) / src_le_ray_d.dot(plane_n);
				Eigen::Vector3d src_le_ray_plane_pnt = src_cm_c + src_le_ray_len * src_le_ray_d;

				double tgt_w[3];
				double tgt_wr[9];
				tgt_w[0] = parameters[1][0];
				tgt_w[1] = parameters[1][1];
				tgt_w[2] = parameters[1][2];
				HW::rodrigues_to_matrix3d(tgt_w, tgt_wr);
				double tgt_t[3];
				tgt_t[0] = parameters[1][3];
				tgt_t[1] = parameters[1][4];
				tgt_t[2] = parameters[1][5];

				//compute tgt ray p3d ()
				Eigen::Matrix3d tgt_r_cw;
				tgt_r_cw << tgt_wr[0], tgt_wr[1], tgt_wr[2],
					tgt_wr[3], tgt_wr[4], tgt_wr[5],
					tgt_wr[6], tgt_wr[7], tgt_wr[8];
				Eigen::Vector3d tgt_cm_exrt_t
					= Eigen::Vector3d(tgt_t[0], tgt_t[1], tgt_t[2]);
				Eigen::Vector3d tgt_cm_c = -tgt_r_cw.transpose() * tgt_cm_exrt_t;
				//tgt ls ray plane pnt
				Eigen::Vector3d tgt_ls_ray_d = tgt_r_cw.inverse() * tgt_K_.inverse() * Eigen::Vector3d(tgt_line_observation_.first[0], tgt_line_observation_.first[1], 1.0);
				double tgt_ls_ray_len = (-plane_info_[3] - tgt_cm_c.dot(plane_n)) / tgt_ls_ray_d.dot(plane_n);
				Eigen::Vector3d tgt_ls_ray_plane_pnt = tgt_cm_c + tgt_ls_ray_len * tgt_ls_ray_d;
				//tgt le ray plane pnt
				Eigen::Vector3d tgt_le_ray_d = tgt_r_cw.inverse() * tgt_K_.inverse() * Eigen::Vector3d(tgt_line_observation_.second[0], tgt_line_observation_.second[1], 1.0);
				double tgt_le_ray_len = (-plane_info_[3] - tgt_cm_c.dot(plane_n)) / tgt_le_ray_d.dot(plane_n);
				Eigen::Vector3d tgt_le_ray_plane_pnt = tgt_cm_c + tgt_le_ray_len * tgt_le_ray_d;
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				Eigen::Matrix<double, 6, 1> tgt_plucker_line = HW::GetPluckerLineFromLineEndPnts(tgt_ls_ray_plane_pnt, tgt_le_ray_plane_pnt);

				Eigen::Vector3d tgt_plucker_ld = tgt_plucker_line.block<3, 1>(3, 0);
				Eigen::Vector3d tgt_plucker_lm = tgt_plucker_line.block<3, 1>(0, 0);

				Eigen::Vector3d src_ls_to_tgt_line = tgt_plucker_lm - src_ls_ray_plane_pnt.cross(tgt_plucker_ld);
				Eigen::Vector3d src_le_to_tgt_line = tgt_plucker_lm - src_le_ray_plane_pnt.cross(tgt_plucker_ld);
				residuals[0] = src_ls_to_tgt_line[0] * energy_scale_sqrt;
				residuals[1] = src_ls_to_tgt_line[1] * energy_scale_sqrt;
				residuals[2] = src_ls_to_tgt_line[2] * energy_scale_sqrt;
				residuals[3] = src_le_to_tgt_line[0] * energy_scale_sqrt;
				residuals[4] = src_le_to_tgt_line[1] * energy_scale_sqrt;
				residuals[5] = src_le_to_tgt_line[2] * energy_scale_sqrt;

				//compute src_ls_cm_c and src_le_cm_c to wi first cam pos w 
				Eigen::Matrix3d jaco_src_ls_le_cmc_to_wi = HW::ComputeSkewMatrixFromVector3d(src_cm_c);
				//compute src_ls_ray_d to w
				Eigen::Matrix3d jaco_src_ls_ray_to_wi = HW::ComputeSkewMatrixFromVector3d(src_ls_ray_d);
				//compute src_le_ray_d to w
				Eigen::Matrix3d jaco_src_le_ray_to_wi = HW::ComputeSkewMatrixFromVector3d(src_le_ray_d);

				//compute jaco src_ls_cm_c and src_le_cm_c to t
				Eigen::Matrix3d jaco_src_ls_le_cmc_to_ti = -src_r_cw.transpose();
				//compute src_ls_ray_d and src_le_ray_d to t
				Eigen::Matrix3d jaco_src_ls_le_ray_to_ti;
				jaco_src_ls_le_ray_to_ti.setZero();

				//compute tgt_ls_cm_c and tgt_le_cm_c to wj second cam pos wj 
				Eigen::Matrix3d jaco_tgt_ls_le_cmc_to_wj = HW::ComputeSkewMatrixFromVector3d(tgt_cm_c);
				//compute tgt_ls_ray_d to wj
				Eigen::Matrix3d jaco_tgt_ls_ray_to_wj = HW::ComputeSkewMatrixFromVector3d(tgt_ls_ray_d);
				//compute src_le_ray_d to w
				Eigen::Matrix3d jaco_tgt_le_ray_to_wj = HW::ComputeSkewMatrixFromVector3d(tgt_le_ray_d);

				//compute jaco src_ls_cm_c and src_le_cm_c to t
				Eigen::Matrix3d jaco_tgt_ls_le_cmc_to_tj = -tgt_r_cw.transpose();
				//compute src_ls_ray_d and src_le_ray_d to t
				Eigen::Matrix3d jaco_tgt_ls_le_ray_to_tj;
				jaco_tgt_ls_le_ray_to_tj.setZero();

				///*std::cerr << "observation_[0]->(iu), observation_[1]->(iv): " << observation_[0] << "->(" << iu << " )" << ", "
				//	<< observation_[1] << "->(" << iv << " )" << std::endl;*/
				//double energy_scale_sqrt = std::sqrt(energy_scale_);
				//residuals[0] = (-observation_[0] + iu) * energy_scale_sqrt;
				//residuals[1] = (-observation_[1] + iv) * energy_scale_sqrt;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J0(jacobians[0]);

					//compute the jacobians
					double srcray_ls_plane = src_ls_ray_d.dot(plane_n);
					double srccmc_plane = src_cm_c.dot(plane_n);
					//
					Eigen::Vector3d jaco_src_ls_le_dpokn_to_w = -jaco_src_ls_le_cmc_to_wi*plane_n;
					Eigen::Matrix3d jaco_src_ls_dpoknd_to_w =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_src_ls_le_dpokn_to_w, src_ls_ray_d);
					Eigen::Matrix3d jaco_src_ls_didpokn_to_w = jaco_src_ls_ray_to_wi*(-plane_info_[3] - srccmc_plane);
					Eigen::Vector3d jaco_src_ls_dion_to_w = jaco_src_ls_ray_to_wi*plane_n;
					Eigen::Vector3d ls_dponv = (-plane_info_[3] - srccmc_plane)*src_ls_ray_d;
					Eigen::Matrix3d jaco_src_ls_diondponv_to_w =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_src_ls_dion_to_w, ls_dponv);

					Eigen::Matrix3d jaco_src_ls_pnt3d_to_w = (jaco_src_ls_dpoknd_to_w + jaco_src_ls_didpokn_to_w) / srcray_ls_plane
						- (jaco_src_ls_diondponv_to_w) / (srcray_ls_plane*srcray_ls_plane);

					Eigen::Vector3d jaco_src_ls_le_dpokn_to_t = -jaco_src_ls_le_cmc_to_ti*plane_n;
					Eigen::Matrix3d jaco_src_ls_dpoknd_to_t =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_src_ls_le_dpokn_to_t, src_ls_ray_d);
					//Eigen::Vector3d dponvt = (-plane_info_[3] - srccmc_plane)*src_ray_d;
					Eigen::Matrix3d jaco_src_ls_pnt3d_to_t = jaco_src_ls_dpoknd_to_t / srcray_ls_plane;

					Eigen::Matrix3d jaco_erro_to_src_ls_to_wi = HW::ComputeSkewMatrixFromVector3d(tgt_plucker_ld)*jaco_src_ls_pnt3d_to_w;
					Eigen::Matrix3d jaco_erro_to_src_ls_to_ti = HW::ComputeSkewMatrixFromVector3d(tgt_plucker_ld)*jaco_src_ls_pnt3d_to_t;

					J0.block<3, 3>(0, 0) = jaco_erro_to_src_ls_to_wi * energy_scale_sqrt;
					J0.block<3, 3>(0, 3) = jaco_erro_to_src_ls_to_ti * energy_scale_sqrt;

					//compute the jacobians
					double srcray_le_plane = src_le_ray_d.dot(plane_n);
					//double srccmc_plane = src_cm_c.dot(plane_n);
					//
					//Eigen::Vector3d jaco_src_ls_le_dpokn_to_w = -jaco_src_ls_le_cmc_to_w*plane_n;
					Eigen::Matrix3d jaco_src_le_dpoknd_to_w =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_src_ls_le_dpokn_to_w, src_le_ray_d);
					Eigen::Matrix3d jaco_src_le_didpokn_to_w = jaco_src_le_ray_to_wi*(-plane_info_[3] - srccmc_plane);
					Eigen::Vector3d jaco_src_le_dion_to_w = jaco_src_le_ray_to_wi*plane_n;
					Eigen::Vector3d le_dponv = (-plane_info_[3] - srccmc_plane)*src_le_ray_d;
					Eigen::Matrix3d jaco_src_le_diondponv_to_w =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_src_le_dion_to_w, le_dponv);

					Eigen::Matrix3d jaco_src_le_pnt3d_to_w = (jaco_src_le_dpoknd_to_w + jaco_src_le_didpokn_to_w) / srcray_le_plane
						- (jaco_src_le_diondponv_to_w) / (srcray_le_plane*srcray_le_plane);

					//Eigen::Vector3d jaco_src_ls_le_dpokn_to_t = -jaco_src_ls_le_cmc_to_t*plane_n;
					Eigen::Matrix3d jaco_src_le_dpoknd_to_t =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_src_ls_le_dpokn_to_t, src_le_ray_d);
					//Eigen::Vector3d dponvt = (-plane_info_[3] - srccmc_plane)*src_ray_d;
					Eigen::Matrix3d jaco_src_le_pnt3d_to_t = jaco_src_le_dpoknd_to_t / srcray_le_plane;

					Eigen::Matrix3d jaco_erro_to_src_le_to_wi = HW::ComputeSkewMatrixFromVector3d(tgt_plucker_ld)*jaco_src_le_pnt3d_to_w;
					Eigen::Matrix3d jaco_erro_to_src_le_to_ti = HW::ComputeSkewMatrixFromVector3d(tgt_plucker_ld)*jaco_src_le_pnt3d_to_t;

					J0.block<3, 3>(3, 0) = jaco_erro_to_src_le_to_wi * energy_scale_sqrt;
					J0.block<3, 3>(3, 3) = jaco_erro_to_src_le_to_ti * energy_scale_sqrt;

				}
				if (jacobians != NULL && jacobians[1] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J1(jacobians[1]);

					//compute the jacobians
					double tgtray_ls_plane = tgt_ls_ray_d.dot(plane_n);
					double tgtcmc_plane = tgt_cm_c.dot(plane_n);
					//
					Eigen::Vector3d jaco_tgt_ls_le_dpokn_to_wj = -jaco_tgt_ls_le_cmc_to_wj*plane_n;
					Eigen::Matrix3d jaco_tgt_ls_dpoknd_to_wj =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ls_le_dpokn_to_wj, src_le_ray_d);
					Eigen::Matrix3d jaco_tgt_ls_didpokn_to_wj = jaco_tgt_ls_ray_to_wj*(-plane_info_[3] - tgtcmc_plane);
					Eigen::Vector3d jaco_tgt_ls_dion_to_wj = jaco_tgt_ls_ray_to_wj*plane_n;
					Eigen::Vector3d ls_dponv = (-plane_info_[3] - tgtcmc_plane)*tgt_ls_ray_d;
					Eigen::Matrix3d jaco_tgt_ls_diondponv_to_wj =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ls_dion_to_wj, ls_dponv);

					Eigen::Matrix3d jaco_tgt_ls_pnt3d_to_wj = (jaco_tgt_ls_dpoknd_to_wj + jaco_tgt_ls_didpokn_to_wj) / tgtray_ls_plane
						- (jaco_tgt_ls_diondponv_to_wj) / (tgtray_ls_plane*tgtray_ls_plane);

					Eigen::Vector3d jaco_tgt_ls_le_dpokn_to_tj = -jaco_tgt_ls_le_cmc_to_tj*plane_n;
					Eigen::Matrix3d jaco_tgt_ls_dpoknd_to_tj =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ls_le_dpokn_to_tj, tgt_ls_ray_d);
					//Eigen::Vector3d dponvt = (-plane_info_[3] - srccmc_plane)*src_ray_d;
					Eigen::Matrix3d jaco_tgt_ls_pnt3d_to_tj = jaco_tgt_ls_dpoknd_to_tj / tgtray_ls_plane;

					//compute the jacobians
					double tgtray_le_plane = tgt_le_ray_d.dot(plane_n);
					//Eigen::Vector3d jaco_src_ls_le_dpokn_to_w = -jaco_src_ls_le_cmc_to_w*plane_n;
					Eigen::Matrix3d jaco_tgt_le_dpoknd_to_wj =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ls_le_dpokn_to_wj, tgt_le_ray_d);
					Eigen::Matrix3d jaco_tgt_le_didpokn_to_wj = jaco_tgt_le_ray_to_wj*(-plane_info_[3] - tgtcmc_plane);
					Eigen::Vector3d jaco_tgt_le_dion_to_wj = jaco_tgt_le_ray_to_wj*plane_n;
					Eigen::Vector3d le_dponv = (-plane_info_[3] - tgtcmc_plane)*tgt_le_ray_d;
					Eigen::Matrix3d jaco_tgt_le_diondponv_to_wj =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_le_dion_to_wj, le_dponv);

					Eigen::Matrix3d jaco_tgt_le_pnt3d_to_wj = (jaco_tgt_le_dpoknd_to_wj + jaco_tgt_le_didpokn_to_wj) / tgtray_le_plane
						- (jaco_tgt_le_diondponv_to_wj) / (tgtray_le_plane*tgtray_le_plane);

					//Eigen::Vector3d jaco_src_ls_le_dpokn_to_t = -jaco_src_ls_le_cmc_to_t*plane_n;
					Eigen::Matrix3d jaco_tgt_le_dpoknd_to_tj =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ls_le_dpokn_to_tj, src_le_ray_d);
					//Eigen::Vector3d dponvt = (-plane_info_[3] - srccmc_plane)*src_ray_d;
					Eigen::Matrix3d jaco_tgt_le_pnt3d_to_tj = jaco_tgt_le_dpoknd_to_tj / tgtray_le_plane;

					Eigen::Vector3d tgt_le_ls_ray_ld = tgt_le_ray_d - tgt_ls_ray_d;
					Eigen::Matrix3d jaco_tgt_le_ls_ray_ld_to_wj = jaco_tgt_le_pnt3d_to_wj - jaco_tgt_ls_pnt3d_to_wj;
					double tgt_le_ls_ray_ld_norm = tgt_le_ls_ray_ld.norm();
					Eigen::Vector3d jaco_tgt_le_ls_ray_ld_norm_to_wj = VectorDotMultiplyMatrix3d(tgt_le_ls_ray_ld, jaco_tgt_le_ls_ray_ld_to_wj) / tgt_le_ls_ray_ld_norm;
					Eigen::Matrix3d jaco_tgt_le_ls_plucker_ld_to_wj = (tgt_le_ls_ray_ld_norm * jaco_tgt_le_ls_ray_ld_to_wj - VectorColMultiplyVectorRow3D(jaco_tgt_le_ls_ray_ld_norm_to_wj, tgt_le_ls_ray_ld)) / (tgt_le_ls_ray_ld_norm*tgt_le_ls_ray_ld_norm);
					Eigen::Matrix3d jaco_tgt_ls_cross_plucker_ld_to_wj = -ComputeSkewMatrixFromVector3d(tgt_plucker_ld)*jaco_tgt_ls_pnt3d_to_wj
						+ ComputeSkewMatrixFromVector3d(tgt_ls_ray_plane_pnt) * jaco_tgt_le_ls_plucker_ld_to_wj;
					Eigen::Matrix3d jaco_src_ls_cross_plucker_ld_to_wj = ComputeSkewMatrixFromVector3d(src_ls_ray_plane_pnt)*jaco_tgt_le_ls_plucker_ld_to_wj;
					Eigen::Matrix3d jaco_src_ls_to_plucker_line_error_to_wj = jaco_tgt_ls_cross_plucker_ld_to_wj - jaco_src_ls_cross_plucker_ld_to_wj;

					Eigen::Matrix3d jaco_tgt_le_ls_ray_ld_to_tj = jaco_tgt_le_pnt3d_to_tj - jaco_tgt_ls_pnt3d_to_tj;
					//double tgt_le_ls_ray_ld_norm = tgt_le_ls_ray_ld.norm();
					Eigen::Vector3d jaco_tgt_le_ls_ray_ld_norm_to_tj = VectorDotMultiplyMatrix3d(tgt_le_ls_ray_ld, jaco_tgt_le_ls_ray_ld_to_tj) / tgt_le_ls_ray_ld_norm;
					Eigen::Matrix3d jaco_tgt_le_ls_plucker_ld_to_tj = (tgt_le_ls_ray_ld_norm * jaco_tgt_le_ls_ray_ld_to_tj - VectorColMultiplyVectorRow3D(jaco_tgt_le_ls_ray_ld_norm_to_tj, tgt_le_ls_ray_ld)) / (tgt_le_ls_ray_ld_norm*tgt_le_ls_ray_ld_norm);
					Eigen::Matrix3d jaco_tgt_ls_cross_plucker_ld_to_tj = -ComputeSkewMatrixFromVector3d(tgt_plucker_ld)*jaco_tgt_ls_pnt3d_to_tj
						+ ComputeSkewMatrixFromVector3d(tgt_ls_ray_plane_pnt) * jaco_tgt_le_ls_plucker_ld_to_tj;
					Eigen::Matrix3d jaco_src_ls_cross_plucker_ld_to_tj = ComputeSkewMatrixFromVector3d(src_ls_ray_plane_pnt)*jaco_tgt_le_ls_plucker_ld_to_tj;
					Eigen::Matrix3d jaco_src_ls_to_plucker_line_error_to_tj = jaco_tgt_ls_cross_plucker_ld_to_tj - jaco_src_ls_cross_plucker_ld_to_tj;

					J1.block<3, 3>(0, 0) = jaco_src_ls_to_plucker_line_error_to_wj * energy_scale_sqrt;
					J1.block<3, 3>(0, 3) = jaco_src_ls_to_plucker_line_error_to_tj * energy_scale_sqrt;

					//it is tgt_plucker_lm
					Eigen::Matrix3d jaco_tgt_le_cross_plucker_ld_to_wj = jaco_tgt_ls_cross_plucker_ld_to_wj;
					Eigen::Matrix3d jaco_src_le_cross_plucker_ld_to_wj = ComputeSkewMatrixFromVector3d(src_le_ray_plane_pnt)*jaco_tgt_le_ls_plucker_ld_to_wj;
					Eigen::Matrix3d jaco_src_le_to_plucker_line_error_to_wj = jaco_tgt_le_cross_plucker_ld_to_wj - jaco_src_le_cross_plucker_ld_to_wj;

					//it is tgt_plucker_lm
					Eigen::Matrix3d jaco_tgt_le_cross_plucker_ld_to_tj = jaco_tgt_ls_cross_plucker_ld_to_tj;
					Eigen::Matrix3d jaco_src_le_cross_plucker_ld_to_tj = ComputeSkewMatrixFromVector3d(src_le_ray_plane_pnt)*jaco_tgt_le_ls_plucker_ld_to_tj;
					Eigen::Matrix3d jaco_src_le_to_plucker_line_error_to_tj = jaco_tgt_le_cross_plucker_ld_to_wj - jaco_src_le_cross_plucker_ld_to_tj;

					J1.block<3, 3>(3, 0) = jaco_src_le_to_plucker_line_error_to_wj * energy_scale_sqrt;
					J1.block<3, 3>(3, 3) = jaco_src_le_to_plucker_line_error_to_tj * energy_scale_sqrt;

				}
				return true;
			}

		private:
			double energy_scale_;
			Eigen::Vector3d pnt3d_;
			Eigen::Vector4d plane_info_;
			Eigen::Matrix3d src_K_;
			Eigen::Matrix3d tgt_K_;
			std::pair<Eigen::Vector2d, Eigen::Vector2d> src_line_observation_;
			std::pair<Eigen::Vector2d, Eigen::Vector2d> tgt_line_observation_;
		};
#endif

		//optimize camera pose only. fix plane 3d, cost: e = (T0_wc * K0^(-1) * (u0,v0,1)) - (T1_wc * K1^(-1) * (u1,v1,1))
		class CamPoseOnlyFromReprojectNeighborPntsCostFunc : public ceres::SizedCostFunction<3, 6, 6>
		{
		public:
			CamPoseOnlyFromReprojectNeighborPntsCostFunc(const Eigen::Vector2d& src_ob, const Eigen::Vector2d& tgt_ob)
			{
				src_observation_[0] = src_ob[0];
				src_observation_[1] = src_ob[1];
				tgt_observation_[0] = tgt_ob[0];
				tgt_observation_[1] = tgt_ob[1];

				pnt3d_ = Eigen::Vector3d::Zero();	//delete next...
				src_K_ = Eigen::Matrix3d::Identity();
				tgt_K_ = Eigen::Matrix3d::Identity();
				energy_scale_ = 1.0;
			}
			void SetTwoNeighborKMatrix(const Eigen::Matrix3d& src_K, const Eigen::Matrix3d& tgt_K)	//set the intrisinc matrix
			{
				src_K_ = src_K;
				tgt_K_ = tgt_K;
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
			virtual ~CamPoseOnlyFromReprojectNeighborPntsCostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				*/
				/*double src_w[3];
				double src_wr[9];
				src_w[0] = parameters[0][0];
				src_w[1] = parameters[0][1];
				src_w[2] = parameters[0][2];
				HW::rodrigues_to_matrix3d(src_w, src_wr);
				double src_t[3];
				src_t[0] = parameters[0][3];
				src_t[1] = parameters[0][4];
				src_t[2] = parameters[0][5];
				Eigen::Matrix3d src_r_cw;
				src_r_cw << src_wr[0], src_wr[1], src_wr[2],
				src_wr[3], src_wr[4], src_wr[5],
				src_wr[6], src_wr[7], src_wr[8];
				Eigen::Vector3d src_cm_exrt_t
				= Eigen::Vector3d(src_t[0], src_t[1], src_t[2]);
				*/

				/*double tgt_w[3];
				double tgt_wr[9];
				tgt_w[0] = parameters[1][0];
				tgt_w[1] = parameters[1][1];
				tgt_w[2] = parameters[1][2];
				HW::rodrigues_to_matrix3d(tgt_w, tgt_wr);
				double tgt_t[3];
				tgt_t[0] = parameters[1][3];
				tgt_t[1] = parameters[1][4];
				tgt_t[2] = parameters[1][5];
				Eigen::Matrix3d tgt_r_cw;
				tgt_r_cw << tgt_wr[0], tgt_wr[1], tgt_wr[2],
					tgt_wr[3], tgt_wr[4], tgt_wr[5],
					tgt_wr[6], tgt_wr[7], tgt_wr[8];
				Eigen::Vector3d tgt_cm_exrt_t
					= Eigen::Vector3d(tgt_t[0], tgt_t[1], tgt_t[2]);*/

				//compute src ray p3d ()
				Eigen::Vector3d src_w;
				src_w[0] = parameters[0][0];
				src_w[1] = parameters[0][1];
				src_w[2] = parameters[0][2];
				Eigen::Matrix3d src_r_cw;
				src_r_cw = HW::AxisAngleToRotationMatrixD(src_w);
				Eigen::Vector3d src_t_cw;
				src_t_cw[0] = parameters[0][3];
				src_t_cw[1] = parameters[0][4];
				src_t_cw[2] = parameters[0][5];
				Eigen::Vector3d src_cm_exrt_t = src_t_cw;

				Eigen::Vector3d src_cm_c = -src_r_cw.transpose() * src_cm_exrt_t;
				Eigen::Vector3d src_ray_d = src_r_cw.transpose() * src_K_.inverse() * Eigen::Vector3d(src_observation_[0], src_observation_[1], 1.0);
				////test
				//std::cerr << "src_r_cw:  \n" << src_r_cw << std::endl;
				//std::cerr << "src_r_cw transpose: " << src_r_cw.transpose() << std::endl;
				//std::cerr << "src_r_cw inverse: " << src_r_cw.inverse() << std::endl;
				////end test
				Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info_[0], plane_info_[1], plane_info_[2]);
				double src_ray_len = (-plane_info_[3] - src_cm_c.dot(plane_n)) / src_ray_d.dot(plane_n);
				Eigen::Vector3d src_ray_plane_pnt = src_cm_c + src_ray_len * src_ray_d;
				//std::cerr << "src_ray_plane_pnt: " << src_ray_plane_pnt.transpose() << std::endl;
				////test
				//if (src_ray_plane_pnt[0] != src_ray_plane_pnt[0]
				//	|| src_ray_plane_pnt[1] != src_ray_plane_pnt[1]
				//	|| src_ray_plane_pnt[2] != src_ray_plane_pnt[2])
				//{
				//	std::cerr << "src_r_cw:  \n" << src_r_cw << std::endl;
				//	std::cerr << "src_r_cw transpose: " << src_r_cw.transpose() << std::endl;
				//	std::cerr << "src_w: " << src_w[0] << " " << src_w[1] << " " << src_w[2] << std::endl;
				//	std::cerr << "plane_n: " << plane_n.transpose() << std::endl;
				//	std::cerr << "src_ray_d: " << src_ray_d.transpose() << std::endl;
				//	std::cerr << "src_ray_plane_pnt: " << src_ray_plane_pnt.transpose() << std::endl;
				//}
				////end test

				//compute tgt ray p3d ()
				Eigen::Vector3d tgt_w;
				tgt_w[0] = parameters[1][0];
				tgt_w[1] = parameters[1][1];
				tgt_w[2] = parameters[1][2];
				Eigen::Matrix3d tgt_r_cw;
				tgt_r_cw = HW::AxisAngleToRotationMatrixD(tgt_w);
				Eigen::Vector3d tgt_t_cw;
				tgt_t_cw[0] = parameters[1][3];
				tgt_t_cw[1] = parameters[1][4];
				tgt_t_cw[2] = parameters[1][5];
				Eigen::Vector3d tgt_cm_exrt_t = tgt_t_cw;
				Eigen::Vector3d tgt_cm_c = -tgt_r_cw.transpose() * tgt_cm_exrt_t;
				Eigen::Vector3d tgt_ray_d = tgt_r_cw.transpose() * tgt_K_.inverse() * Eigen::Vector3d(tgt_observation_[0], tgt_observation_[1], 1.0);
				
				////test
				//std::cerr << "tgt_r_cw:  \n" << tgt_r_cw << std::endl;
				//std::cerr << "tgt_r_cw transpose: " << tgt_r_cw.transpose() << std::endl;
				//std::cerr << "tgt_r_cw inverse: " << tgt_r_cw.inverse() << std::endl;
				////end test

				//Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info_[0], plane_info_[1], plane_info_[2]);
				double tgt_ray_len = (-plane_info_[3] - tgt_cm_c.dot(plane_n)) / tgt_ray_d.dot(plane_n);
				Eigen::Vector3d tgt_ray_plane_pnt = tgt_cm_c + tgt_ray_len * tgt_ray_d;
				//std::cerr << "tgt_ray_plane_pnt: " << tgt_ray_plane_pnt.transpose() << std::endl;

				////test
				//if (tgt_ray_plane_pnt[0] != tgt_ray_plane_pnt[0]
				//	|| tgt_ray_plane_pnt[1] != tgt_ray_plane_pnt[1]
				//	|| tgt_ray_plane_pnt[2] != tgt_ray_plane_pnt[2])
				//{
				//	std::cerr << "tgt_r_cw:  \n" << tgt_r_cw << std::endl;
				//	std::cerr << "tgt_r_cw transpose: " << tgt_r_cw.transpose() << std::endl;
				//	std::cerr << "tgt_w: " << tgt_w[0] << " " << tgt_w[1] << " " << tgt_w[2] << std::endl;
				//	std::cerr << "plane_n: " << plane_n.transpose() << std::endl;
				//	std::cerr << "tgt_ray_d: " << tgt_ray_d.transpose() << std::endl;
				//	std::cerr << "tgt_ray_plane_pnt: " << tgt_ray_plane_pnt.transpose() << std::endl;
				//}
				////end test

				//get dist to each other
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				residuals[0] = (src_ray_plane_pnt[0] - tgt_ray_plane_pnt[0]) * energy_scale_sqrt;
				residuals[1] = (src_ray_plane_pnt[1] - tgt_ray_plane_pnt[1]) * energy_scale_sqrt;
				residuals[2] = (src_ray_plane_pnt[2] - tgt_ray_plane_pnt[2]) * energy_scale_sqrt;
				
				Eigen::Vector3d jaco_resi_erro_to_src_p3d = Eigen::Vector3d::Ones()*energy_scale_sqrt;

				//compute src_cm_c to w 
				Eigen::Matrix3d jaco_src_cmc_to_w = HW::ComputeSkewMatrixFromVector3d(src_cm_c);
				//compute src_ray_d to w
				Eigen::Matrix3d jaco_src_ray_to_w = HW::ComputeSkewMatrixFromVector3d(src_ray_d);
				//compute jaco src_cm_c to t
				Eigen::Matrix3d jaco_src_cmc_to_t = -src_r_cw.transpose();
				//compute src_ray_d to t
				Eigen::Matrix3d jaco_src_ray_to_t;
				jaco_src_ray_to_t.setZero();

				//compute src_cm_c to w 
				Eigen::Matrix3d jaco_tgt_cmc_to_w = HW::ComputeSkewMatrixFromVector3d(tgt_cm_c);
				//compute src_ray_d to w
				Eigen::Matrix3d jaco_tgt_ray_to_w = HW::ComputeSkewMatrixFromVector3d(tgt_ray_d);
				//compute jaco src_cm_c to t
				Eigen::Matrix3d jaco_tgt_cmc_to_t = -tgt_r_cw.transpose();
				//compute src_ray_d to t
				Eigen::Matrix3d jaco_tgt_ray_to_t;
				jaco_tgt_ray_to_t.setZero();
				/*std::cerr << "observation_[0]->(iu), observation_[1]->(iv): " << observation_[0] << "->(" << iu << " )" << ", "
					<< observation_[1] << "->(" << iv << " )" << std::endl;*/

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J0(jacobians[0]);
					////compute the jacobians
					//double srcray_plane = src_ray_d.dot(plane_n);
					//double srccmc_plane = src_cm_c.dot(plane_n);

					double srcray_dot_plane = src_ray_d.dot(plane_n);
					double srccmc_dot_plane = src_cm_c.dot(plane_n);

					////
					//Eigen::Vector3d jaco_src_dpokn_to_w = -jaco_src_cmc_to_w*plane_n;
					//Eigen::Matrix3d jaco_src_dpoknd_to_w = 
					//	VectorColMultiplyVectorRow3DZDGLocal(jaco_src_dpokn_to_w, src_ray_d);
					//Eigen::Matrix3d jaco_src_didpokn_to_w = jaco_src_ray_to_w*(-plane_info_[3] - srccmc_plane);
					//Eigen::Vector3d jaco_src_dion_to_w = jaco_src_ray_to_w*plane_n;
					//Eigen::Vector3d dponv = (-plane_info_[3] - srccmc_plane)*src_ray_d;
					
					//tgt_ray_d.dot(plane_n) to w test it...
					Eigen::Vector3d jaco_srcray_dot_plane_n_to_w = plane_n.transpose() * jaco_src_ray_to_w;
					//src_cm_c.dot(plane_n) to w
					Eigen::Vector3d jaco_srccmc_dot_planen_to_w = plane_n.transpose() * jaco_src_cmc_to_w;
					Eigen::Vector3d jaco_src_ray_len_to_w = plane_info_[3] * jaco_srcray_dot_plane_n_to_w / (srcray_dot_plane * srcray_dot_plane)
						 + srccmc_dot_plane * jaco_srcray_dot_plane_n_to_w / (srcray_dot_plane * srcray_dot_plane)  - jaco_srccmc_dot_planen_to_w / srcray_dot_plane;

					/*Eigen::Matrix3d jaco_src_plane_pnt3d_to_w = jaco_src_cmc_to_w + VectorColMultiplyVectorRow3DZDGLocal(jaco_src_ray_len_to_w, src_ray_d)
						+ jaco_src_ray_to_w * src_ray_len;*/
					Eigen::Matrix3d jaco_src_plane_pnt3d_to_w = jaco_src_cmc_to_w + VectorColMultiplyVectorRow3DZDGLocal(src_ray_d, jaco_src_ray_len_to_w)
						+ jaco_src_ray_to_w * src_ray_len;


					/*Eigen::Matrix3d jaco_src_diondponv_to_w =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_src_dion_to_w, dponv);
					Eigen::Matrix3d jaco_src_pnt3d_to_w = jaco_src_cmc_to_w + (jaco_src_dpoknd_to_w + jaco_src_didpokn_to_w) / srcray_plane
						- (jaco_src_diondponv_to_w)/(srcray_plane*srcray_plane);*/

					//tgt_ray_d.dot(plane_n) to t test it...
					Eigen::Vector3d jaco_srcray_dot_plane_n_to_t = plane_n.transpose() * jaco_src_ray_to_t;
					//src_cm_c.dot(plane_n) to t
					Eigen::Vector3d jaco_srccmc_dot_planen_to_t = plane_n.transpose() * jaco_src_cmc_to_t;
					Eigen::Vector3d jaco_src_ray_len_to_t = plane_info_[3] * jaco_srcray_dot_plane_n_to_t / (srcray_dot_plane * srcray_dot_plane)
						+ srccmc_dot_plane * jaco_srcray_dot_plane_n_to_t / (srcray_dot_plane * srcray_dot_plane) - jaco_srccmc_dot_planen_to_t / srcray_dot_plane;
					/*Eigen::Matrix3d jaco_src_plane_pnt3d_to_t = jaco_src_cmc_to_t + VectorColMultiplyVectorRow3DZDGLocal(jaco_src_ray_len_to_t, src_ray_d)
						+ jaco_src_ray_to_t * src_ray_len;*/
					Eigen::Matrix3d jaco_src_plane_pnt3d_to_t = jaco_src_cmc_to_t + VectorColMultiplyVectorRow3DZDGLocal(src_ray_d, jaco_src_ray_len_to_t)
						+ jaco_src_ray_to_t * src_ray_len;

					/*Eigen::Vector3d jaco_src_dpokn_to_t = -jaco_src_cmc_to_t*plane_n;
					Eigen::Matrix3d jaco_src_dpoknd_to_t =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_src_dpokn_to_t, src_ray_d);*/
					//Eigen::Vector3d dponvt = (-plane_info_[3] - srccmc_plane)*src_ray_d;
					//Eigen::Matrix3d jaco_src_pnt3d_to_t = jaco_src_cmc_to_t +  jaco_src_dpoknd_to_t / srcray_dot_plane;

					J0(0, 0) = jaco_src_plane_pnt3d_to_w(0, 0) * energy_scale_sqrt;
					J0(0, 1) = jaco_src_plane_pnt3d_to_w(0, 1) * energy_scale_sqrt;
					J0(0, 2) = jaco_src_plane_pnt3d_to_w(0, 2) * energy_scale_sqrt;
					J0(0, 3) = jaco_src_plane_pnt3d_to_t(0, 0) * energy_scale_sqrt;
					J0(0, 4) = jaco_src_plane_pnt3d_to_t(0, 1) * energy_scale_sqrt;
					J0(0, 5) = jaco_src_plane_pnt3d_to_t(0, 2) * energy_scale_sqrt;

					J0(1, 0) = jaco_src_plane_pnt3d_to_w(1, 0) * energy_scale_sqrt;
					J0(1, 1) = jaco_src_plane_pnt3d_to_w(1, 1) * energy_scale_sqrt;
					J0(1, 2) = jaco_src_plane_pnt3d_to_w(1, 2) * energy_scale_sqrt;
					J0(1, 3) = jaco_src_plane_pnt3d_to_t(1, 0) * energy_scale_sqrt;
					J0(1, 4) = jaco_src_plane_pnt3d_to_t(1, 1) * energy_scale_sqrt;
					J0(1, 5) = jaco_src_plane_pnt3d_to_t(1, 2) * energy_scale_sqrt;

					J0(2, 0) = jaco_src_plane_pnt3d_to_w(2, 0) * energy_scale_sqrt;
					J0(2, 1) = jaco_src_plane_pnt3d_to_w(2, 1) * energy_scale_sqrt;
					J0(2, 2) = jaco_src_plane_pnt3d_to_w(2, 2) * energy_scale_sqrt;
					J0(2, 3) = jaco_src_plane_pnt3d_to_t(2, 0) * energy_scale_sqrt;
					J0(2, 4) = jaco_src_plane_pnt3d_to_t(2, 1) * energy_scale_sqrt;
					J0(2, 5) = jaco_src_plane_pnt3d_to_t(2, 2) * energy_scale_sqrt;
				}
				if (jacobians != NULL && jacobians[1] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J1(jacobians[1]);

					//compute the jacobians
					double tgtray_dot_plane = tgt_ray_d.dot(plane_n);
					double tgtcmc_dot_plane = tgt_cm_c.dot(plane_n);
					
					//tgt_ray_d.dot(plane_n) to w test it...
					Eigen::Vector3d jaco_tgtray_dot_plane_n_to_w = plane_n.transpose() * jaco_tgt_ray_to_w;
					//tgt_cm_c.dot(plane_n) to w
					Eigen::Vector3d jaco_tgtcmc_dot_planen_to_w = plane_n.transpose() * jaco_tgt_cmc_to_w;
					Eigen::Vector3d jaco_tgt_ray_len_to_w = plane_info_[3] * jaco_tgtray_dot_plane_n_to_w / (tgtray_dot_plane * tgtray_dot_plane)
						+ tgtcmc_dot_plane * jaco_tgtray_dot_plane_n_to_w / (tgtray_dot_plane * tgtray_dot_plane) - jaco_tgtcmc_dot_planen_to_w / tgtray_dot_plane;

					/*Eigen::Matrix3d jaco_tgt_plane_pnt3d_to_w = jaco_tgt_cmc_to_w + VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ray_len_to_w, tgt_ray_d)
						+ jaco_tgt_ray_to_w * tgt_ray_len;*/
					Eigen::Matrix3d jaco_tgt_plane_pnt3d_to_w = jaco_tgt_cmc_to_w + VectorColMultiplyVectorRow3DZDGLocal(tgt_ray_d, jaco_tgt_ray_len_to_w)
						+ jaco_tgt_ray_to_w * tgt_ray_len;

					//tgt_ray_d.dot(plane_n) to t test it...
					Eigen::Vector3d jaco_tgtray_dot_plane_n_to_t = plane_n.transpose() * jaco_tgt_ray_to_t;
					//tgt_cm_c.dot(plane_n) to t
					Eigen::Vector3d jaco_tgtcmc_dot_planen_to_t = plane_n.transpose() * jaco_tgt_cmc_to_t;
					Eigen::Vector3d jaco_tgt_ray_len_to_t = plane_info_[3] * jaco_tgtray_dot_plane_n_to_t / (tgtray_dot_plane * tgtray_dot_plane)
						+ tgtcmc_dot_plane * jaco_tgtray_dot_plane_n_to_t / (tgtray_dot_plane * tgtray_dot_plane) - jaco_tgtcmc_dot_planen_to_t / tgtray_dot_plane;
					/*Eigen::Matrix3d jaco_tgt_plane_pnt3d_to_t = jaco_tgt_cmc_to_t + VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ray_len_to_t, tgt_ray_d)
						+ jaco_tgt_ray_to_t * tgt_ray_len;*/
					Eigen::Matrix3d jaco_tgt_plane_pnt3d_to_t = jaco_tgt_cmc_to_t + VectorColMultiplyVectorRow3DZDGLocal(tgt_ray_d, jaco_tgt_ray_len_to_t)
						+ jaco_tgt_ray_to_t * tgt_ray_len;

					J1(0, 0) = -jaco_tgt_plane_pnt3d_to_w(0, 0) * energy_scale_sqrt;
					J1(0, 1) = -jaco_tgt_plane_pnt3d_to_w(0, 1) * energy_scale_sqrt;
					J1(0, 2) = -jaco_tgt_plane_pnt3d_to_w(0, 2) * energy_scale_sqrt;
					J1(0, 3) = -jaco_tgt_plane_pnt3d_to_t(0, 0) * energy_scale_sqrt;
					J1(0, 4) = -jaco_tgt_plane_pnt3d_to_t(0, 1) * energy_scale_sqrt;
					J1(0, 5) = -jaco_tgt_plane_pnt3d_to_t(0, 2) * energy_scale_sqrt;

					J1(1, 0) = -jaco_tgt_plane_pnt3d_to_w(1, 0) * energy_scale_sqrt;
					J1(1, 1) = -jaco_tgt_plane_pnt3d_to_w(1, 1) * energy_scale_sqrt;
					J1(1, 2) = -jaco_tgt_plane_pnt3d_to_w(1, 2) * energy_scale_sqrt;
					J1(1, 3) = -jaco_tgt_plane_pnt3d_to_t(1, 0) * energy_scale_sqrt;
					J1(1, 4) = -jaco_tgt_plane_pnt3d_to_t(1, 1) * energy_scale_sqrt;
					J1(1, 5) = -jaco_tgt_plane_pnt3d_to_t(1, 2) * energy_scale_sqrt;

					J1(2, 0) = -jaco_tgt_plane_pnt3d_to_w(2, 0) * energy_scale_sqrt;
					J1(2, 1) = -jaco_tgt_plane_pnt3d_to_w(2, 1) * energy_scale_sqrt;
					J1(2, 2) = -jaco_tgt_plane_pnt3d_to_w(2, 2) * energy_scale_sqrt;
					J1(2, 3) = -jaco_tgt_plane_pnt3d_to_t(2, 0) * energy_scale_sqrt;
					J1(2, 4) = -jaco_tgt_plane_pnt3d_to_t(2, 1) * energy_scale_sqrt;
					J1(2, 5) = -jaco_tgt_plane_pnt3d_to_t(2, 2) * energy_scale_sqrt;
				}
				return true;
			}

		private:
			double energy_scale_;
			Eigen::Vector3d pnt3d_;
			Eigen::Vector4d plane_info_;
			Eigen::Matrix3d src_K_;
			Eigen::Matrix3d tgt_K_;
			Eigen::Vector2d src_observation_;
			Eigen::Vector2d tgt_observation_;
		};

		//optimize camera pose only. fix plane 3d,  plucker line cost: e = m1 - p_0 \times d1;
		class CamPoseOnlyFromReprojectNeighborLinesCostFunc : public ceres::SizedCostFunction<6, 6, 6>
		{
		public:
			CamPoseOnlyFromReprojectNeighborLinesCostFunc(const std::pair<Eigen::Vector2d, Eigen::Vector2d>& src_ob, 
				const std::pair<Eigen::Vector2d, Eigen::Vector2d>& tgt_ob)
			{
				src_line_observation_ = src_ob;
				tgt_line_observation_ = tgt_ob;
				pnt3d_ = Eigen::Vector3d::Zero();	//delete next...
				src_K_ = Eigen::Matrix3d::Identity();
				tgt_K_ = Eigen::Matrix3d::Identity();
				energy_scale_ = 1.0;
			}
			void SetTwoNeighborKMatrix(const Eigen::Matrix3d& src_K, const Eigen::Matrix3d& tgt_K)	//set the intrisinc matrix
			{
				src_K_ = src_K;
				tgt_K_ = tgt_K;
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
			virtual ~CamPoseOnlyFromReprojectNeighborLinesCostFunc() {};

			virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
			{
				/*
				parameters 0 -> (1,6)
				parameters 0: camera parameters(w[0],w[1],w[2],t[0],t[1],t[2])
				*/

				//double src_w[3];
				//double src_wr[9];
				//src_w[0] = parameters[0][0];
				//src_w[1] = parameters[0][1];
				//src_w[2] = parameters[0][2];
				//HW::rodrigues_to_matrix3d(src_w, src_wr);
				//double src_t[3];
				//src_t[0] = parameters[0][3];
				//src_t[1] = parameters[0][4];
				//src_t[2] = parameters[0][5];
				////compute src ray p3d ()
				//Eigen::Matrix3d src_r_cw;
				//src_r_cw << src_wr[0], src_wr[1], src_wr[2],
				//	src_wr[3], src_wr[4], src_wr[5],
				//	src_wr[6], src_wr[7], src_wr[8];
				//Eigen::Vector3d src_cm_exrt_t
				//	= Eigen::Vector3d(src_t[0], src_t[1], src_t[2]);

				Eigen::Vector3d src_w;
				src_w[0] = parameters[0][0];
				src_w[1] = parameters[0][1];
				src_w[2] = parameters[0][2];
				Eigen::Matrix3d src_r_cw;
				src_r_cw = HW::AxisAngleToRotationMatrixD(src_w);
				Eigen::Vector3d src_t_cw;
				src_t_cw[0] = parameters[0][3];
				src_t_cw[1] = parameters[0][4];
				src_t_cw[2] = parameters[0][5];
				Eigen::Vector3d src_cm_exrt_t = src_t_cw;

				Eigen::Vector3d src_cm_c = -src_r_cw.transpose() * src_cm_exrt_t;
				//src ls ray plane pnt
				Eigen::Vector3d src_ls_ray_d = src_r_cw.transpose() * src_K_.inverse() * Eigen::Vector3d(src_line_observation_.first[0], src_line_observation_.first[1], 1.0);
				Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info_[0], plane_info_[1], plane_info_[2]);
				double src_ls_ray_len = (-plane_info_[3] - src_cm_c.dot(plane_n)) / src_ls_ray_d.dot(plane_n);
				Eigen::Vector3d src_ls_ray_plane_pnt = src_cm_c + src_ls_ray_len * src_ls_ray_d;
				//src le ray plane pnt
				Eigen::Vector3d src_le_ray_d = src_r_cw.transpose() * src_K_.inverse() * Eigen::Vector3d(src_line_observation_.second[0], src_line_observation_.second[1], 1.0);
				//Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info_[0], plane_info_[1], plane_info_[2]);
				double src_le_ray_len = (-plane_info_[3] - src_cm_c.dot(plane_n)) / src_le_ray_d.dot(plane_n);
				Eigen::Vector3d src_le_ray_plane_pnt = src_cm_c + src_le_ray_len * src_le_ray_d;

				//double tgt_w[3];
				//double tgt_wr[9];
				//tgt_w[0] = parameters[1][0];
				//tgt_w[1] = parameters[1][1];
				//tgt_w[2] = parameters[1][2];
				//HW::rodrigues_to_matrix3d(tgt_w, tgt_wr);
				//double tgt_t[3];
				//tgt_t[0] = parameters[1][3];
				//tgt_t[1] = parameters[1][4];
				//tgt_t[2] = parameters[1][5];
				////compute tgt ray p3d ()
				//Eigen::Matrix3d tgt_r_cw;
				//tgt_r_cw << tgt_wr[0], tgt_wr[1], tgt_wr[2],
				//	tgt_wr[3], tgt_wr[4], tgt_wr[5],
				//	tgt_wr[6], tgt_wr[7], tgt_wr[8];
				//Eigen::Vector3d tgt_cm_exrt_t
				//	= Eigen::Vector3d(tgt_t[0], tgt_t[1], tgt_t[2]);

				Eigen::Vector3d tgt_w;
				tgt_w[0] = parameters[1][0];
				tgt_w[1] = parameters[1][1];
				tgt_w[2] = parameters[1][2];
				Eigen::Matrix3d tgt_r_cw;
				tgt_r_cw = HW::AxisAngleToRotationMatrixD(tgt_w);
				Eigen::Vector3d tgt_t_cw;
				tgt_t_cw[0] = parameters[1][3];
				tgt_t_cw[1] = parameters[1][4];
				tgt_t_cw[2] = parameters[1][5];
				Eigen::Vector3d tgt_cm_exrt_t = tgt_t_cw;

				Eigen::Vector3d tgt_cm_c = -tgt_r_cw.transpose() * tgt_cm_exrt_t;
				//tgt ls ray plane pnt
				Eigen::Vector3d tgt_ls_ray_d = tgt_r_cw.transpose() * tgt_K_.inverse() * Eigen::Vector3d(tgt_line_observation_.first[0], tgt_line_observation_.first[1], 1.0);
				double tgt_ls_ray_len = (-plane_info_[3] - tgt_cm_c.dot(plane_n)) / tgt_ls_ray_d.dot(plane_n);
				Eigen::Vector3d tgt_ls_ray_plane_pnt = tgt_cm_c + tgt_ls_ray_len * tgt_ls_ray_d;
				//tgt le ray plane pnt
				Eigen::Vector3d tgt_le_ray_d = tgt_r_cw.transpose() * tgt_K_.inverse() * Eigen::Vector3d(tgt_line_observation_.second[0], tgt_line_observation_.second[1], 1.0);
				double tgt_le_ray_len = (-plane_info_[3] - tgt_cm_c.dot(plane_n)) / tgt_le_ray_d.dot(plane_n);
				Eigen::Vector3d tgt_le_ray_plane_pnt = tgt_cm_c + tgt_le_ray_len * tgt_le_ray_d;
				double energy_scale_sqrt = std::sqrt(energy_scale_);
				Eigen::Matrix<double, 6, 1> tgt_plucker_line = HW::GetPluckerLineFromLineEndPnts(tgt_ls_ray_plane_pnt, tgt_le_ray_plane_pnt);

				Eigen::Vector3d tgt_plucker_ld = tgt_plucker_line.block<3, 1>(3, 0);
				Eigen::Vector3d tgt_plucker_lm = tgt_plucker_line.block<3, 1>(0, 0);

				Eigen::Vector3d src_ls_to_tgt_line = tgt_plucker_lm - src_ls_ray_plane_pnt.cross(tgt_plucker_ld);
				Eigen::Vector3d src_le_to_tgt_line = tgt_plucker_lm - src_le_ray_plane_pnt.cross(tgt_plucker_ld);
				residuals[0] = src_ls_to_tgt_line[0] * energy_scale_sqrt;
				residuals[1] = src_ls_to_tgt_line[1] * energy_scale_sqrt;
				residuals[2] = src_ls_to_tgt_line[2] * energy_scale_sqrt;
				residuals[3] = src_le_to_tgt_line[0] * energy_scale_sqrt;
				residuals[4] = src_le_to_tgt_line[1] * energy_scale_sqrt;
				residuals[5] = src_le_to_tgt_line[2] * energy_scale_sqrt;

				//compute src_ls_cm_c and src_le_cm_c to wi first cam pos wi 
				Eigen::Matrix3d jaco_src_ls_le_cmc_to_wi = HW::ComputeSkewMatrixFromVector3d(src_cm_c);
				//compute src_ls_ray_d to wi
				Eigen::Matrix3d jaco_src_ls_ray_to_wi = HW::ComputeSkewMatrixFromVector3d(src_ls_ray_d);
				//compute src_le_ray_d to wi
				Eigen::Matrix3d jaco_src_le_ray_to_wi = HW::ComputeSkewMatrixFromVector3d(src_le_ray_d);

				//compute jaco src_ls_cm_c and src_le_cm_c to ti
				Eigen::Matrix3d jaco_src_ls_le_cmc_to_ti = -src_r_cw.transpose();
				//compute src_ls_ray_d and src_le_ray_d to ti
				Eigen::Matrix3d jaco_src_ls_le_ray_to_ti;
				jaco_src_ls_le_ray_to_ti.setZero();

				//compute tgt_ls_cm_c and tgt_le_cm_c to wj second cam pos wj 
				Eigen::Matrix3d jaco_tgt_ls_le_cmc_to_wj = HW::ComputeSkewMatrixFromVector3d(tgt_cm_c);
				//compute tgt_ls_ray_d to wj
				Eigen::Matrix3d jaco_tgt_ls_ray_to_wj = HW::ComputeSkewMatrixFromVector3d(tgt_ls_ray_d);
				//compute src_le_ray_d to w
				Eigen::Matrix3d jaco_tgt_le_ray_to_wj = HW::ComputeSkewMatrixFromVector3d(tgt_le_ray_d);

				//compute jaco src_ls_cm_c and src_le_cm_c to t
				Eigen::Matrix3d jaco_tgt_ls_le_cmc_to_tj = -tgt_r_cw.transpose();
				//compute src_ls_ray_d and src_le_ray_d to t
				Eigen::Matrix3d jaco_tgt_ls_le_ray_to_tj;
				jaco_tgt_ls_le_ray_to_tj.setZero();

				///*std::cerr << "observation_[0]->(iu), observation_[1]->(iv): " << observation_[0] << "->(" << iu << " )" << ", "
				//	<< observation_[1] << "->(" << iv << " )" << std::endl;*/
				//double energy_scale_sqrt = std::sqrt(energy_scale_);
				//residuals[0] = (-observation_[0] + iu) * energy_scale_sqrt;
				//residuals[1] = (-observation_[1] + iv) * energy_scale_sqrt;

				if (jacobians != NULL && jacobians[0] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J0(jacobians[0]);

					//compute the jacobians
					double srclsray_dot_plane = src_ls_ray_d.dot(plane_n);
					double srccmc_dot_plane = src_cm_c.dot(plane_n);

					//src_ls_ray_d.dot(plane_n) to wi test it...
					Eigen::Vector3d jaco_srclsray_dot_plane_n_to_wi = plane_n.transpose() * jaco_src_ls_ray_to_wi;
					//src_cm_c.dot(plane_n) to wi
					Eigen::Vector3d jaco_srclscmc_dot_planen_to_wi = plane_n.transpose() * jaco_src_ls_le_cmc_to_wi;
					Eigen::Vector3d jaco_src_ls_ray_len_to_wi = plane_info_[3] * jaco_srclsray_dot_plane_n_to_wi / (srclsray_dot_plane * srclsray_dot_plane)
						+ srccmc_dot_plane * jaco_srclsray_dot_plane_n_to_wi / (srclsray_dot_plane * srclsray_dot_plane) - jaco_srclscmc_dot_planen_to_wi / srclsray_dot_plane;
					/*Eigen::Matrix3d jaco_src_ls_plane_pnt3d_to_wi = jaco_src_ls_le_cmc_to_wi + VectorColMultiplyVectorRow3DZDGLocal(jaco_src_ls_ray_len_to_wi, src_ls_ray_d)
						+ jaco_src_ls_ray_to_wi * src_ls_ray_len;*/
					Eigen::Matrix3d jaco_src_ls_plane_pnt3d_to_wi = jaco_src_ls_le_cmc_to_wi + VectorColMultiplyVectorRow3DZDGLocal(src_ls_ray_d, jaco_src_ls_ray_len_to_wi)
						+ jaco_src_ls_ray_to_wi * src_ls_ray_len;

					//src_ls_ray_d.dot(plane_n) to ti test it...
					Eigen::Vector3d jaco_srclsray_dot_plane_n_to_ti = plane_n.transpose() * jaco_src_ls_le_ray_to_ti;
					//src_ls_cm_c.dot(plane_n) to ti
					Eigen::Vector3d jaco_srclscmc_dot_planen_to_ti = plane_n.transpose() * jaco_src_ls_le_cmc_to_ti;
					Eigen::Vector3d jaco_src_ls_ray_len_to_ti = plane_info_[3] * jaco_srclsray_dot_plane_n_to_ti / (srclsray_dot_plane * srclsray_dot_plane)
						+ srccmc_dot_plane * jaco_srclsray_dot_plane_n_to_ti / (srclsray_dot_plane * srclsray_dot_plane) - jaco_srclscmc_dot_planen_to_ti / srclsray_dot_plane;
					//Eigen::Matrix3d jaco_src_ls_plane_pnt3d_to_ti = jaco_src_ls_le_cmc_to_ti + VectorColMultiplyVectorRow3DZDGLocal(jaco_src_ls_ray_len_to_ti, src_ls_ray_d)
					//	+ jaco_src_ls_le_ray_to_ti * src_ls_ray_len;	//jaco_src_ls_le_ray_to_ti0
					Eigen::Matrix3d jaco_src_ls_plane_pnt3d_to_ti = jaco_src_ls_le_cmc_to_ti + VectorColMultiplyVectorRow3DZDGLocal(src_ls_ray_d, jaco_src_ls_ray_len_to_ti)
						+ jaco_src_ls_le_ray_to_ti * src_ls_ray_len;	//jaco_src_ls_le_ray_to_ti0
					
					Eigen::Matrix3d jaco_erro_to_src_ls_to_wi = HW::ComputeSkewMatrixFromVector3d(tgt_plucker_ld)*jaco_src_ls_plane_pnt3d_to_wi;
					Eigen::Matrix3d jaco_erro_to_src_ls_to_ti = HW::ComputeSkewMatrixFromVector3d(tgt_plucker_ld)*jaco_src_ls_plane_pnt3d_to_ti;

					J0.block<3, 3>(0, 0) = jaco_erro_to_src_ls_to_wi * energy_scale_sqrt;
					J0.block<3, 3>(0, 3) = jaco_erro_to_src_ls_to_ti * energy_scale_sqrt;

					//compute the jacobians
					double srcleray_dot_plane = src_le_ray_d.dot(plane_n);

					//src_ls_ray_d.dot(plane_n) to wi test it...
					Eigen::Vector3d jaco_srcleray_dot_plane_n_to_wi = plane_n.transpose() * jaco_src_le_ray_to_wi;
					//src_cm_c.dot(plane_n) to wi
					Eigen::Vector3d jaco_srclecmc_dot_planen_to_wi = plane_n.transpose() * jaco_src_ls_le_cmc_to_wi;
					Eigen::Vector3d jaco_src_le_ray_len_to_wi = plane_info_[3] * jaco_srcleray_dot_plane_n_to_wi / (srcleray_dot_plane * srcleray_dot_plane)
						+ srccmc_dot_plane * jaco_srcleray_dot_plane_n_to_wi / (srcleray_dot_plane * srcleray_dot_plane) - jaco_srclecmc_dot_planen_to_wi / srcleray_dot_plane;
					/*Eigen::Matrix3d jaco_src_le_plane_pnt3d_to_wi = jaco_src_ls_le_cmc_to_wi + VectorColMultiplyVectorRow3DZDGLocal(jaco_src_le_ray_len_to_wi, src_le_ray_d)
						+ jaco_src_le_ray_to_wi * src_le_ray_len;*/
					Eigen::Matrix3d jaco_src_le_plane_pnt3d_to_wi = jaco_src_ls_le_cmc_to_wi + VectorColMultiplyVectorRow3DZDGLocal(src_le_ray_d, jaco_src_le_ray_len_to_wi)
						+ jaco_src_le_ray_to_wi * src_le_ray_len;

					//src_ls_ray_d.dot(plane_n) to ti test it...
					Eigen::Vector3d jaco_srcleray_dot_plane_n_to_ti = plane_n.transpose() * jaco_src_ls_le_ray_to_ti;
					//src_ls_cm_c.dot(plane_n) to ti
					Eigen::Vector3d jaco_srclecmc_dot_planen_to_ti = plane_n.transpose() * jaco_src_ls_le_cmc_to_ti;
					Eigen::Vector3d jaco_src_le_ray_len_to_ti = plane_info_[3] * jaco_srcleray_dot_plane_n_to_ti / (srcleray_dot_plane * srcleray_dot_plane)
						+ srccmc_dot_plane * jaco_srcleray_dot_plane_n_to_ti / (srcleray_dot_plane * srcleray_dot_plane) - jaco_srclecmc_dot_planen_to_ti / srcleray_dot_plane;
					//Eigen::Matrix3d jaco_src_le_plane_pnt3d_to_ti = jaco_src_ls_le_cmc_to_ti + VectorColMultiplyVectorRow3DZDGLocal(jaco_src_le_ray_len_to_ti, src_le_ray_d)
					//	+ jaco_src_ls_le_ray_to_ti * src_le_ray_len;	//jaco_src_ls_le_ray_to_ti Ϊ0
					Eigen::Matrix3d jaco_src_le_plane_pnt3d_to_ti = jaco_src_ls_le_cmc_to_ti + VectorColMultiplyVectorRow3DZDGLocal(src_le_ray_d, jaco_src_le_ray_len_to_ti)
						+ jaco_src_ls_le_ray_to_ti * src_le_ray_len;	//jaco_src_ls_le_ray_to_ti Ϊ0

					Eigen::Matrix3d jaco_erro_to_src_le_to_wi = HW::ComputeSkewMatrixFromVector3d(tgt_plucker_ld)*jaco_src_le_plane_pnt3d_to_wi;
					Eigen::Matrix3d jaco_erro_to_src_le_to_ti = HW::ComputeSkewMatrixFromVector3d(tgt_plucker_ld)*jaco_src_le_plane_pnt3d_to_ti;

					J0.block<3, 3>(3, 0) = jaco_erro_to_src_le_to_wi * energy_scale_sqrt;
					J0.block<3, 3>(3, 3) = jaco_erro_to_src_le_to_ti * energy_scale_sqrt;

				}
				if (jacobians != NULL && jacobians[1] != NULL)
				{
					Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J1(jacobians[1]);
					
					//compute the jacobians
					double tgtlsray_dot_plane = tgt_ls_ray_d.dot(plane_n);
					double tgtcmc_dot_plane = tgt_cm_c.dot(plane_n);
					//tgt_ls_ray_d.dot(plane_n) to wj test it...
					Eigen::Vector3d jaco_tgtlsray_dot_plane_n_to_wj = plane_n.transpose() * jaco_tgt_ls_ray_to_wj;
					//tgtls_cm_c.dot(plane_n) to wj = tgt_cm_c.dot(plane_n) to wj
					Eigen::Vector3d jaco_tgtlscmc_dot_planen_to_wj = plane_n.transpose() * jaco_tgt_ls_le_cmc_to_wj;
					Eigen::Vector3d jaco_tgt_ls_ray_len_to_wj = plane_info_[3] * jaco_tgtlsray_dot_plane_n_to_wj / (tgtlsray_dot_plane * tgtlsray_dot_plane)
						+ tgtcmc_dot_plane * jaco_tgtlsray_dot_plane_n_to_wj / (tgtlsray_dot_plane * tgtlsray_dot_plane) - jaco_tgtlscmc_dot_planen_to_wj / tgtlsray_dot_plane;
					/*Eigen::Matrix3d jaco_tgt_ls_plane_pnt3d_to_wj = jaco_tgt_ls_le_cmc_to_wj + VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ls_ray_len_to_wj, tgt_ls_ray_d)
						+ jaco_tgt_ls_ray_to_wj * tgt_ls_ray_len;*/
					Eigen::Matrix3d jaco_tgt_ls_plane_pnt3d_to_wj = jaco_tgt_ls_le_cmc_to_wj + VectorColMultiplyVectorRow3DZDGLocal(tgt_ls_ray_d, jaco_tgt_ls_ray_len_to_wj)
						+ jaco_tgt_ls_ray_to_wj * tgt_ls_ray_len;

					//tgt_ls_ray_d.dot(plane_n) to tj test it...
					Eigen::Vector3d jaco_tgtlsray_dot_plane_n_to_tj = plane_n.transpose() * jaco_tgt_ls_le_ray_to_tj;
					//tgt_ls_cm_c.dot(plane_n) to tj
					Eigen::Vector3d jaco_tgtlscmc_dot_planen_to_tj = plane_n.transpose() * jaco_tgt_ls_le_cmc_to_tj;
					Eigen::Vector3d jaco_tgt_ls_ray_len_to_tj = plane_info_[3] * jaco_tgtlsray_dot_plane_n_to_tj / (tgtlsray_dot_plane * tgtlsray_dot_plane)
						+ tgtcmc_dot_plane * jaco_tgtlsray_dot_plane_n_to_tj / (tgtlsray_dot_plane * tgtlsray_dot_plane) - jaco_tgtlscmc_dot_planen_to_tj / tgtlsray_dot_plane;
					//Eigen::Matrix3d jaco_tgt_ls_plane_pnt3d_to_tj = jaco_tgt_ls_le_cmc_to_tj + VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ls_ray_len_to_tj, tgt_ls_ray_d)
					//	+ jaco_tgt_ls_le_ray_to_tj * tgt_ls_ray_len;	//jaco_tgt_ls_le_ray_to_tj Ϊ0
					Eigen::Matrix3d jaco_tgt_ls_plane_pnt3d_to_tj = jaco_tgt_ls_le_cmc_to_tj + VectorColMultiplyVectorRow3DZDGLocal(tgt_ls_ray_d, jaco_tgt_ls_ray_len_to_tj)
						+ jaco_tgt_ls_le_ray_to_tj * tgt_ls_ray_len;	//jaco_tgt_ls_le_ray_to_tj Ϊ0
					//compute the jacobians
					double tgtleray_dot_plane = tgt_le_ray_d.dot(plane_n);

					//tgt_le_ray_d.dot(plane_n) to wj test it...
					Eigen::Vector3d jaco_tgtleray_dot_plane_n_to_wj = plane_n.transpose() * jaco_tgt_le_ray_to_wj;
					//tgtle_cm_c.dot(plane_n) to wj = tgt_cm_c.dot(plane_n) to wj
					Eigen::Vector3d jaco_tgtlecmc_dot_planen_to_wj = plane_n.transpose() * jaco_tgt_ls_le_cmc_to_wj;
					Eigen::Vector3d jaco_tgt_le_ray_len_to_wj = plane_info_[3] * jaco_tgtleray_dot_plane_n_to_wj / (tgtleray_dot_plane * tgtlsray_dot_plane)
						+ tgtcmc_dot_plane * jaco_tgtleray_dot_plane_n_to_wj / (tgtleray_dot_plane * tgtleray_dot_plane) - jaco_tgtlecmc_dot_planen_to_wj / tgtleray_dot_plane;
					/*Eigen::Matrix3d jaco_tgt_le_plane_pnt3d_to_wj = jaco_tgt_ls_le_cmc_to_wj + VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_le_ray_len_to_wj, tgt_le_ray_d)
						+ jaco_tgt_le_ray_to_wj * tgt_le_ray_len;*/
					Eigen::Matrix3d jaco_tgt_le_plane_pnt3d_to_wj = jaco_tgt_ls_le_cmc_to_wj + VectorColMultiplyVectorRow3DZDGLocal(tgt_le_ray_d, jaco_tgt_le_ray_len_to_wj)
						+ jaco_tgt_le_ray_to_wj * tgt_le_ray_len;

					//tgt_le_ray_d.dot(plane_n) to tj test it...
					Eigen::Vector3d jaco_tgtleray_dot_plane_n_to_tj = plane_n.transpose() * jaco_tgt_ls_le_ray_to_tj;
					//tgt_le_cm_c.dot(plane_n) to tj
					Eigen::Vector3d jaco_tgtlecmc_dot_planen_to_tj = plane_n.transpose() * jaco_tgt_ls_le_cmc_to_tj;
					Eigen::Vector3d jaco_tgt_le_ray_len_to_tj = plane_info_[3] * jaco_tgtleray_dot_plane_n_to_tj / (tgtleray_dot_plane * tgtleray_dot_plane)
						+ tgtcmc_dot_plane * jaco_tgtleray_dot_plane_n_to_tj / (tgtleray_dot_plane * tgtleray_dot_plane) - jaco_tgtlecmc_dot_planen_to_tj / tgtleray_dot_plane;
					//Eigen::Matrix3d jaco_tgt_le_plane_pnt3d_to_tj = jaco_tgt_ls_le_cmc_to_tj + VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_le_ray_len_to_tj, tgt_le_ray_d)
					//	+ jaco_tgt_ls_le_ray_to_tj * tgt_le_ray_len;	//jaco_tgt_ls_le_ray_to_tj Ϊ0
					Eigen::Matrix3d jaco_tgt_le_plane_pnt3d_to_tj = jaco_tgt_ls_le_cmc_to_tj + VectorColMultiplyVectorRow3DZDGLocal(tgt_le_ray_d, jaco_tgt_le_ray_len_to_tj)
						+ jaco_tgt_ls_le_ray_to_tj * tgt_le_ray_len;	//jaco_tgt_ls_le_ray_to_tj Ϊ0


					/*Eigen::Vector3d jaco_tgt_ls_le_dpokn_to_wj = -jaco_tgt_ls_le_cmc_to_wj*plane_n;
					Eigen::Matrix3d jaco_tgt_ls_dpoknd_to_wj =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ls_le_dpokn_to_wj, src_le_ray_d);
					Eigen::Matrix3d jaco_tgt_ls_didpokn_to_wj = jaco_tgt_ls_ray_to_wj*(-plane_info_[3] - tgtcmc_plane);
					Eigen::Vector3d jaco_tgt_ls_dion_to_wj = jaco_tgt_ls_ray_to_wj*plane_n;
					Eigen::Vector3d ls_dponv = (-plane_info_[3] - tgtcmc_plane)*tgt_ls_ray_d;
					Eigen::Matrix3d jaco_tgt_ls_diondponv_to_wj =
						VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ls_dion_to_wj, ls_dponv);
					Eigen::Matrix3d jaco_tgt_ls_pnt3d_to_wj = (jaco_tgt_ls_dpoknd_to_wj + jaco_tgt_ls_didpokn_to_wj) / tgtray_ls_plane
						- (jaco_tgt_ls_diondponv_to_wj) / (tgtray_ls_plane*tgtray_ls_plane);*/

					//Eigen::Vector3d jaco_tgt_ls_le_dpokn_to_tj = -jaco_tgt_ls_le_cmc_to_tj*plane_n;
					//Eigen::Matrix3d jaco_tgt_ls_dpoknd_to_tj =
					//	VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ls_le_dpokn_to_tj, tgt_ls_ray_d);
					////Eigen::Vector3d dponvt = (-plane_info_[3] - srccmc_plane)*src_ray_d;
					//Eigen::Matrix3d jaco_tgt_ls_pnt3d_to_tj = jaco_tgt_ls_dpoknd_to_tj / tgtray_ls_plane;

					////compute the jacobians
					//double tgtray_le_plane = tgt_le_ray_d.dot(plane_n);
					////Eigen::Vector3d jaco_src_ls_le_dpokn_to_w = -jaco_src_ls_le_cmc_to_w*plane_n;
					//Eigen::Matrix3d jaco_tgt_le_dpoknd_to_wj =
					//	VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ls_le_dpokn_to_wj, tgt_le_ray_d);
					//Eigen::Matrix3d jaco_tgt_le_didpokn_to_wj = jaco_tgt_le_ray_to_wj*(-plane_info_[3] - tgtcmc_plane);
					//Eigen::Vector3d jaco_tgt_le_dion_to_wj = jaco_tgt_le_ray_to_wj*plane_n;
					//Eigen::Vector3d le_dponv = (-plane_info_[3] - tgtcmc_plane)*tgt_le_ray_d;
					//Eigen::Matrix3d jaco_tgt_le_diondponv_to_wj =
					//	VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_le_dion_to_wj, le_dponv);

					//Eigen::Matrix3d jaco_tgt_le_pnt3d_to_wj = (jaco_tgt_le_dpoknd_to_wj + jaco_tgt_le_didpokn_to_wj) / tgtray_le_plane
					//	- (jaco_tgt_le_diondponv_to_wj) / (tgtray_le_plane*tgtray_le_plane);
					////Eigen::Vector3d jaco_src_ls_le_dpokn_to_t = -jaco_src_ls_le_cmc_to_t*plane_n;
					//Eigen::Matrix3d jaco_tgt_le_dpoknd_to_tj =
					//	VectorColMultiplyVectorRow3DZDGLocal(jaco_tgt_ls_le_dpokn_to_tj, src_le_ray_d);
					////Eigen::Vector3d dponvt = (-plane_info_[3] - srccmc_plane)*src_ray_d;
					//Eigen::Matrix3d jaco_tgt_le_pnt3d_to_tj = jaco_tgt_le_dpoknd_to_tj / tgtray_le_plane;


					//plucker line to wj tj  compute their jaco...
					Eigen::Vector3d tgt_le_ls_ray_ld = tgt_le_ray_d - tgt_ls_ray_d;
					Eigen::Matrix3d jaco_tgt_le_ls_ray_ld_to_wj = jaco_tgt_le_plane_pnt3d_to_wj - jaco_tgt_ls_plane_pnt3d_to_wj;
					double tgt_le_ls_ray_ld_norm = tgt_le_ls_ray_ld.norm();
					Eigen::Vector3d jaco_tgt_le_ls_ray_ld_norm_to_wj = VectorDotMultiplyMatrix3d(tgt_le_ls_ray_ld, jaco_tgt_le_ls_ray_ld_to_wj) / tgt_le_ls_ray_ld_norm;
					Eigen::Matrix3d jaco_tgt_le_ls_plucker_ld_to_wj = (tgt_le_ls_ray_ld_norm * jaco_tgt_le_ls_ray_ld_to_wj - VectorColMultiplyVectorRow3D(tgt_le_ls_ray_ld, jaco_tgt_le_ls_ray_ld_norm_to_wj)) / (tgt_le_ls_ray_ld_norm*tgt_le_ls_ray_ld_norm);
					Eigen::Matrix3d jaco_tgt_ls_cross_plucker_ld_m_to_wj = -ComputeSkewMatrixFromVector3d(tgt_plucker_ld)*jaco_tgt_ls_plane_pnt3d_to_wj
						+ ComputeSkewMatrixFromVector3d(tgt_ls_ray_plane_pnt) * jaco_tgt_le_ls_plucker_ld_to_wj;
					Eigen::Matrix3d jaco_src_ls_cross_plucker_ld_to_wj = ComputeSkewMatrixFromVector3d(src_ls_ray_plane_pnt)*jaco_tgt_le_ls_plucker_ld_to_wj;
					Eigen::Matrix3d jaco_src_ls_to_plucker_line_error_to_wj = jaco_tgt_ls_cross_plucker_ld_m_to_wj - jaco_src_ls_cross_plucker_ld_to_wj;

					Eigen::Matrix3d jaco_tgt_le_ls_ray_ld_to_tj = jaco_tgt_le_plane_pnt3d_to_tj - jaco_tgt_ls_plane_pnt3d_to_tj;
					//double tgt_le_ls_ray_ld_norm = tgt_le_ls_ray_ld.norm();
					Eigen::Vector3d jaco_tgt_le_ls_ray_ld_norm_to_tj = VectorDotMultiplyMatrix3d(tgt_le_ls_ray_ld, jaco_tgt_le_ls_ray_ld_to_tj) / tgt_le_ls_ray_ld_norm;
					Eigen::Matrix3d jaco_tgt_le_ls_plucker_ld_to_tj = (tgt_le_ls_ray_ld_norm * jaco_tgt_le_ls_ray_ld_to_tj - VectorColMultiplyVectorRow3D(tgt_le_ls_ray_ld, jaco_tgt_le_ls_ray_ld_norm_to_tj)) / (tgt_le_ls_ray_ld_norm*tgt_le_ls_ray_ld_norm);
					Eigen::Matrix3d jaco_tgt_ls_cross_plucker_ld_to_tj = -ComputeSkewMatrixFromVector3d(tgt_plucker_ld)*jaco_tgt_ls_plane_pnt3d_to_tj
						+ ComputeSkewMatrixFromVector3d(tgt_ls_ray_plane_pnt) * jaco_tgt_le_ls_plucker_ld_to_tj;
					Eigen::Matrix3d jaco_src_ls_cross_plucker_ld_to_tj = ComputeSkewMatrixFromVector3d(src_ls_ray_plane_pnt)*jaco_tgt_le_ls_plucker_ld_to_tj;
					Eigen::Matrix3d jaco_src_ls_to_plucker_line_error_to_tj = jaco_tgt_ls_cross_plucker_ld_to_tj - jaco_src_ls_cross_plucker_ld_to_tj;

					J1.block<3, 3>(0, 0) = jaco_src_ls_to_plucker_line_error_to_wj * energy_scale_sqrt;
					J1.block<3, 3>(0, 3) = jaco_src_ls_to_plucker_line_error_to_tj * energy_scale_sqrt;

					//it is tgt_plucker_lm
					Eigen::Matrix3d jaco_tgt_le_cross_plucker_ld_m_to_wj = jaco_tgt_ls_cross_plucker_ld_m_to_wj;
					Eigen::Matrix3d jaco_src_le_cross_plucker_ld_to_wj = ComputeSkewMatrixFromVector3d(src_le_ray_plane_pnt)*jaco_tgt_le_ls_plucker_ld_to_wj;
					Eigen::Matrix3d jaco_src_le_to_plucker_line_error_to_wj = jaco_tgt_le_cross_plucker_ld_m_to_wj - jaco_src_le_cross_plucker_ld_to_wj;

					//it is tgt_plucker_lm
					Eigen::Matrix3d jaco_tgt_le_cross_plucker_ld_to_tj = jaco_tgt_ls_cross_plucker_ld_to_tj;
					Eigen::Matrix3d jaco_src_le_cross_plucker_ld_to_tj = ComputeSkewMatrixFromVector3d(src_le_ray_plane_pnt)*jaco_tgt_le_ls_plucker_ld_to_tj;
					Eigen::Matrix3d jaco_src_le_to_plucker_line_error_to_tj = jaco_tgt_le_cross_plucker_ld_m_to_wj - jaco_src_le_cross_plucker_ld_to_tj;

					J1.block<3, 3>(3, 0) = jaco_src_le_to_plucker_line_error_to_wj * energy_scale_sqrt;
					J1.block<3, 3>(3, 3) = jaco_src_le_to_plucker_line_error_to_tj * energy_scale_sqrt;

				}
				return true;
			}

		private:
			double energy_scale_;
			Eigen::Vector3d pnt3d_;
			Eigen::Vector4d plane_info_;
			Eigen::Matrix3d src_K_;
			Eigen::Matrix3d tgt_K_;
			std::pair<Eigen::Vector2d, Eigen::Vector2d> src_line_observation_;
			std::pair<Eigen::Vector2d, Eigen::Vector2d> tgt_line_observation_;
		};

		HWSceneBundleCamsAdjustment::HWSceneBundleCamsAdjustment()
		{
			bundle_max_iteration_num_per_epoch_ = 500;
			bundle_max_iteration_ = 2;
			epoch_max_number_ = 1;
			reproject_error_average_threhold_ = 0.5;
			bundle_cams_pnts_loss_value_ = std::numeric_limits<double>::max();
			bundle_plane_lines_loss_value_ = std::numeric_limits<double>::max();

			pnts_to_pnts_lambda_ = 1.0;
			lines_to_lines_lambda_ = 10.0;
		}

		void HWSceneBundleCamsAdjustment::setBundleOptimizationIterNum(int iter_max_num)
		{
			bundle_max_iteration_ = iter_max_num;
		}

		void HWSceneBundleCamsAdjustment::setBundleReprojectAverageError(double average_error)
		{
			reproject_error_average_threhold_ = average_error;
		}

		void HWSceneBundleCamsAdjustment::setCameras(const std::vector<CameraParams>& cams)
		{
			cams_.resize(cams.size());
			for (int i = 0; i < cams.size(); ++i)
			{
				cams_[i] = cams[i];
			}
		}

		void HWSceneBundleCamsAdjustment::setViewsPoints(const std::vector<Point3D>& views_points)
		{
			points_views_3d_.resize(views_points.size());
			for (int i = 0; i < views_points.size(); ++i)
			{
				points_views_3d_[i] = views_points[i];
			}
		}

		void HWSceneBundleCamsAdjustment::setPluckerLine3DPoints(const std::vector<PluckerLinePoint3D>& plucker_lines)
		{
			std::cerr << "to do next..." << std::endl;
		}

		void HWSceneBundleCamsAdjustment::setViewsLine3DPoints(const std::vector<LinePoint3D>& views_lines_pnts)
		{
			lines_views_points_3d_.resize(views_lines_pnts.size());
			for (int i = 0; i < views_lines_pnts.size(); ++i)
			{
				//
				lines_views_points_3d_[i] = views_lines_pnts[i];
			}
		}

		void HWSceneBundleCamsAdjustment::setViewObservationPntsTracklist(const std::vector<PointWorld3dTrackList>& pnts_tracks)
		{
			points_3d_tracklist_.resize(pnts_tracks.size());
			for (int i = 0; i < pnts_tracks.size(); ++i)
			{
				//
				points_3d_tracklist_[i] = pnts_tracks[i];
			}
		}

		void HWSceneBundleCamsAdjustment::setViewObservationlinesTracklist(const std::vector<LineWorld3dTrackList>& lines_tracks)
		{
			lines_3d_tracklist_.resize(lines_tracks.size());
			for (int i = 0; i < lines_tracks.size(); ++i)
			{
				//
				lines_3d_tracklist_[i] = lines_tracks[i];
			}
		}

		void HWSceneBundleCamsAdjustment::setPlanes(const std::vector<Plane3D>& planes)
		{
			planes_.resize(planes.size());
			for (int i = 0; i < planes_.size(); ++i)
			{
				planes_[i] = planes[i];
			}
		}

		const std::vector<CameraParams>& HWSceneBundleCamsAdjustment::GetCameras()
		{
			return cams_;
		}

		void HWSceneBundleCamsAdjustment::UpdatePreparedBundleData()
		{
			std::cerr << "------start update prepared bundle data------" << std::endl;
			UpdateCameraParamPartToCameraExtr();
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

		void HWSceneBundleCamsAdjustment::UpdateCameraParamPartToCameraExtr()
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

		void HWSceneBundleCamsAdjustment::UpdatePlanes2WorldPlanes()
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

		void HWSceneBundleCamsAdjustment::UpdateBundleDataToData()
		{
			std::cerr << "start to update bundle data to data..." << std::endl;
			UpdateBundleCameraToCameraExtr();
#if UsePluckerLineMethod
			UpdateOthoLine2BundleLineWorldLine();	//to change next... delete w_plane_lines_otho_lines_
#endif
			UpdateBunldePlanesToPlanes();
			std::cerr << "end update bundle data to data..." << std::endl;
		}

		void HWSceneBundleCamsAdjustment::UpdateBundleCameraToCameraExtr()
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

		void HWSceneBundleCamsAdjustment::UpdateTracklistPntsToTracklistPntsOpti()
		{
			//points_3d_tracklist_opti_.clear();
			pre_points_views_3d_ = points_views_3d_;
			for (int i = 0; i < points_3d_tracklist_.size(); ++i)
			{
				PointWorld3dTrackList pnt_3d_track = points_3d_tracklist_[i];
				if (pnt_3d_track.plane_idx_.empty())
				{
					continue;
				}
				int plane_id = pnt_3d_track.plane_idx_[0];
				if (plane_id == -1)
				{
					continue;
				}
				Plane3D plane_info = planes_[plane_id];
				for (int j = 0; j < pnt_3d_track.pnts3d_to_views_observations_.size(); ++j)
				{
					PntViewObservation pnt_ob_pnt = pnt_3d_track.pnts3d_to_views_observations_[j];
					int camid = pnt_ob_pnt.camera_id;
					CameraParams cam = cams_[camid];
					Eigen::Vector2d img_pnt = pnt_ob_pnt.pos_;
					int view_pnt_id = pnt_ob_pnt.point_view_id;
					if (view_pnt_id != -1)
					{
						Eigen::Vector3d view_pnt3d = ImgPntRayPlaneInfoPnt3dNew(cam, plane_info, img_pnt);
						points_views_3d_[view_pnt_id].pos_ = view_pnt3d;

						//Eigen::Vector2d reproj_img_pnt = Pnt3dReprojectImagePntFromCameraParams(cam, view_pnt3d);
						//Eigen::Vector3d view_pnt3d_test = ImgPntRayPlaneInfoPnt3dTest_depth1(cam, img_pnt);
						//Eigen::Vector2d reproj_img_pnt_test = Pnt3dReprojectImagePntFromCameraParamsTest1(cam, view_pnt3d_test);
						////Pnt3dReprojectImagePntFromCameraParamsTest1
						//Eigen::Vector2d reproj_img_pnt_11_test = Pnt3dReprojectImagePntFromCameraParamsTest2(cam, view_pnt3d_test);
						//std::cerr << "observation img pnt: " << img_pnt.transpose() << std::endl;
						//std::cerr << "reproj_img_pnt pnt: " << reproj_img_pnt.transpose() << std::endl;
						//std::cerr << "reproj_img_pnt_test pnt: " << reproj_img_pnt_test.transpose() << std::endl;
						//std::cerr << "reproj_img_pnt_11_test pnt: " << reproj_img_pnt_11_test.transpose() << std::endl;
						//Eigen::Vector3d view_pnt3d_w = ImgPntRayPlaneInfoPnt3dFromCamCoord(cam, plane_info, img_pnt);
						//Eigen::Vector2d reproj_img_pnt_from_viewpnt3d_w = Pnt3dReprojectImagePntFromCameraParams(cam, view_pnt3d);
						//std::cerr << "view_pnt3d pnt: " << view_pnt3d.transpose() << std::endl;
						//std::cerr << "view_pnt3d_w pnt: " << view_pnt3d_w.transpose() << std::endl;
						//std::cerr << "observation img pnt: " << img_pnt.transpose() << std::endl;
						//std::cerr << "reproj_img_pnt_from_viewpnt3d_w pnt: " << reproj_img_pnt_from_viewpnt3d_w.transpose() << std::endl;
					}
				}
			}
		}

		void HWSceneBundleCamsAdjustment::UpdateTracklistLinesToTracklistLinesOpti()
		{
			pre_lines_views_points_3d_ = lines_views_points_3d_;
			for (int i = 0; i < lines_3d_tracklist_.size(); ++i)
			{
				LineWorld3dTrackList line_3d_track = lines_3d_tracklist_[i];
				if (line_3d_track.plane_idx_.empty())
				{
					continue;
				}
				int plane_id = line_3d_track.plane_idx_[0];
				if (plane_id == -1)
				{
					continue;
				}
				Plane3D plane_info = planes_[plane_id];
				for (int j = 0; j < line_3d_track.lines3d_to_views_observations_.size(); ++j)
				{
					LineViewObservation line_ob_pnts = line_3d_track.lines3d_to_views_observations_[j];
					int camid = line_ob_pnts.camera_id;
					CameraParams cam = cams_[camid];
					Eigen::Vector2d img_line_spnt = line_ob_pnts.ls2d_;
					Eigen::Vector2d img_line_epnt = line_ob_pnts.le2d_;

					Eigen::Vector3d view_ls_pnt3d = ImgPntRayPlaneInfoPnt3dNew(cam, plane_info, img_line_spnt);
					Eigen::Vector3d view_le_pnt3d = ImgPntRayPlaneInfoPnt3dNew(cam, plane_info, img_line_epnt);

					int view_line_id = line_ob_pnts.line3d_view_id;
					if (view_line_id != -1)
					{
						lines_views_points_3d_[view_line_id].pos_s_ = view_ls_pnt3d;
						lines_views_points_3d_[view_line_id].pos_e_ = view_le_pnt3d;
					}
				}
			}
		}

		void HWSceneBundleCamsAdjustment::UpdateBunldePlanesToPlanes()
		{
			for (int i = 0; i < planes_.size(); ++i)
			{
				planes_[i].f_[0] = w_planes_[i].f_[0];
				planes_[i].f_[1] = w_planes_[i].f_[1];
				planes_[i].f_[2] = w_planes_[i].f_[2];
				planes_[i].f_[3] = w_planes_[i].f_[3];
			}
		}

		const std::vector<Point3D>& HWSceneBundleCamsAdjustment::GetViewsPoints()
		{
			return points_views_3d_;
		}

		const std::vector<LinePoint3D>& HWSceneBundleCamsAdjustment::GetViewsLine3DPoints()
		{
			return lines_views_points_3d_;
		}

		const std::vector<Point3D>& HWSceneBundleCamsAdjustment::GetPreViewsPoints()
		{
			return pre_points_views_3d_;
		}

		const std::vector<LinePoint3D>& HWSceneBundleCamsAdjustment::GetPreViewsLine3DPoints()
		{
			return pre_lines_views_points_3d_;
		}

		const std::vector<PointWorld3dTrackList>& HWSceneBundleCamsAdjustment::GetViewObservationPntsTracklist()
		{
			return points_3d_tracklist_;
		}

		const std::vector<LineWorld3dTrackList>& HWSceneBundleCamsAdjustment::GetViewObservationlinesTracklist()
		{
			return lines_3d_tracklist_;
		}

		Eigen::Matrix4d HWSceneBundleCamsAdjustment::ConvertBundleCameraToCamExtr(double* bundle_cam)
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

		bool HWSceneBundleCamsAdjustment::optimize_cams_only()
		{
			std::cerr << "---start to optimize cams only the function---" << std::endl;
			lm_optimize_only_cams();
			std::cerr << "---end optimize cams only the function---" << std::endl;
			return true;
		}

		bool HWSceneBundleCamsAdjustment::optimize_cams_only_grouped_line()
		{
			std::cerr << "---start to optimize cams only the function with grouped lines---" << std::endl;
			lm_optimize_only_cams_grouped_lines();
			std::cerr << "---end optimize cams only the function with grouped lines---" << std::endl;
			return true;
		}

		void HWSceneBundleCamsAdjustment::print_status() const
		{
			std::cerr << "to do next..." << std::endl;
		}

		void HWSceneBundleCamsAdjustment::print_cams_info()
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

		void HWSceneBundleCamsAdjustment::print_pnt3d_info()
		{
			std::cerr << "to do next..." << std::endl;
		}

		void HWSceneBundleCamsAdjustment::print_lines3d_info()
		{
			std::cerr << "to do next..." << std::endl;
		}

		void HWSceneBundleCamsAdjustment::print_cam_r_info(double* r)
		{
			std::cerr << "test_rodrigues: \n" << r[0] << " " << r[1] << " " << r[2] << std::endl
				<< r[3] << " " << r[4] << " " << r[5] << std::endl
				<< r[6] << " " << r[7] << " " << r[8] << std::endl;
		}

		bool HWSceneBundleCamsAdjustment::lm_optimize_only_cams()
		{
			print_cams_info();
			epoch_max_number_ = 1;
			for (int k = 0; k < epoch_max_number_; ++k)
			{
				lm_optimize_epoch_only_cams();
			}
			return true;
		}

		bool HWSceneBundleCamsAdjustment::lm_optimize_only_cams_grouped_lines()
		{
			print_cams_info();
			epoch_max_number_ = 1;
			for (int k = 0; k < epoch_max_number_; ++k)
			{
				lm_optimize_epoch_only_cams_grouped_lines();
			}
			return true;
		}

		bool HWSceneBundleCamsAdjustment::lm_optimize_epoch_only_cams()
		{
			int planes_info_num = static_cast<int>(planes_.size());

#if 1	//test the points energy
			//std::cerr << "planes_info_num: " << planes_info_num << std::endl;
			//add image pnt reproject to polygon pnts and compute their dist each other error
			for (int i = 0; i < points_3d_tracklist_.size(); ++i)
			{
				//std::cerr << "points_3d_tracklist_->i: " << i << std::endl;
				PointWorld3dTrackList pnt2track = points_3d_tracklist_[i];
				//get plane id (they have same plane id)
				std::vector<int> track_plane_idx = pnt2track.plane_idx_;
				int track_plane_idxs_num = static_cast<int>(track_plane_idx.size());
				//std::cerr << "track_plane_idxs_num: " << track_plane_idxs_num << std::endl;
				if (!track_plane_idx.empty())
				{
					//get one plane id
					int plane_idx_pri0 = track_plane_idx[0];
					//std::cerr << "plane_idx_pri0: " << plane_idx_pri0 << std::endl;
					if (plane_idx_pri0 != -1)
					{
						//get another plane idx
						//int plane_idx_pri1 = track_plane_idx[1]; (to do next...)
						int pnt_track_ob_num = static_cast<int> (pnt2track.pnts3d_to_views_observations_.size());
						//get plane info
						Plane3D tmp_plane_info = planes_[plane_idx_pri0];
						//std::cerr << "pnt_track_ob_num: " << pnt_track_ob_num << std::endl;
						std::vector<int> pnts3d_to_views_obs_idxs;
						for (int j = 0; j < pnt2track.pnts3d_to_views_observations_.size(); ++j)
						{
							int tmp_pnts3d_to_views_obs_idx = j;
							pnts3d_to_views_obs_idxs.emplace_back(tmp_pnts3d_to_views_obs_idx);
						}
						//std::cerr << "123123123123123123" << std::endl;
						if (pnt_track_ob_num >= 2)
						{
							std::vector<std::pair<int, int> > pnts3d_local_obs_idxs 
								= GenerateCompletePairsFromNVector(pnts3d_to_views_obs_idxs);
							for (int j = 0; j < pnts3d_local_obs_idxs.size(); ++j)
							{
								int src_idx = pnts3d_local_obs_idxs[j].first;
								int tgt_idx = pnts3d_local_obs_idxs[j].second;
								//std::cerr << "j, src_idx, tgt_idx: " << j << ", " << src_idx << ", " << tgt_idx << std::endl;
								//get src idx observation and tgt idx observation
								PntViewObservation src_view_observation = pnt2track.pnts3d_to_views_observations_[src_idx];
								PntViewObservation tgt_view_observation = pnt2track.pnts3d_to_views_observations_[tgt_idx];
								CamPoseOnlyFromReprojectNeighborPntsCostFunc * tmp_reproject_fun = 
									new CamPoseOnlyFromReprojectNeighborPntsCostFunc(src_view_observation.pos_, tgt_view_observation.pos_);
								int src_camid = src_view_observation.camera_id;
								int tgt_camid = tgt_view_observation.camera_id;
								//std::cerr << "src_camid, tgt_camid: " << src_camid << ", " << tgt_camid << std::endl;
								Eigen::Matrix3d srck = cams_[src_camid].cam_k_;
								Eigen::Matrix3d tgtk = cams_[tgt_camid].cam_k_;
								tmp_reproject_fun->SetTwoNeighborKMatrix(srck, tgtk);
								tmp_reproject_fun->SetPlaneInfo(tmp_plane_info.f_);
								tmp_reproject_fun->SetEnergyScale(pnts_to_pnts_lambda_);
								//pnts_plane_cost_func_ = tmp_reproject_fun;
								all_cams_pnts_problems_.AddResidualBlock(tmp_reproject_fun, NULL,
									cams_extrs_[src_camid].cam_extr_, cams_extrs_[tgt_camid].cam_extr_);
							}
						}
					}
				}
			}
#endif

#if 1	//test the lines energy
			//add image line reproject to polygon lines and compute their dist each other error
			for (int i = 0; i < lines_3d_tracklist_.size(); ++i)
			{
				//int 
				LineWorld3dTrackList line2track = lines_3d_tracklist_[i];
				//get plane id (they have same plane id)
				std::vector<int> track_plane_idx = line2track.plane_idx_;
				if (!track_plane_idx.empty())
				{
					//get one plane id
					int plane_idx_pri0 = track_plane_idx[0];
					if (plane_idx_pri0 != -1)
					{
						//get another plane idx
						//int plane_idx_pri1 = track_plane_idx[1]; (to do next...)
						int line_track_ob_num = static_cast<int> (line2track.lines3d_to_views_observations_.size());
						//get plane info
						Plane3D tmp_plane_info = planes_[plane_idx_pri0];
						std::vector<int> lines3d_to_views_obs_idxs;
						for (int j = 0; j < line2track.lines3d_to_views_observations_.size(); ++j)
						{
							int tmp_lines3d_to_views_obs_idx = j;
							lines3d_to_views_obs_idxs.emplace_back(tmp_lines3d_to_views_obs_idx);
						}
						if (line_track_ob_num >= 2)
						{
							std::vector<std::pair<int, int> > lines3d_local_obs_idxs;
							lines3d_local_obs_idxs = GenerateCompletePairsFromNVector(lines3d_to_views_obs_idxs);
							for (int j = 0; j < lines3d_local_obs_idxs.size(); ++j)
							{
								int src_idx = lines3d_local_obs_idxs[j].first;
								int tgt_idx = lines3d_local_obs_idxs[j].second;
								//get src idx observation and tgt idx observation
								LineViewObservation src_line_view_observation = line2track.lines3d_to_views_observations_[src_idx];
								LineViewObservation tgt_line_view_observation = line2track.lines3d_to_views_observations_[tgt_idx];
								std::pair<Eigen::Vector2d, Eigen::Vector2d> src_line_pos = std::make_pair(src_line_view_observation.ls2d_, src_line_view_observation.le2d_);
								std::pair<Eigen::Vector2d, Eigen::Vector2d> tgt_line_pos = std::make_pair(tgt_line_view_observation.ls2d_, tgt_line_view_observation.le2d_);
								
								CamPoseOnlyFromReprojectNeighborLinesCostFunc * tmp_line_src_reproject_fun =
									new CamPoseOnlyFromReprojectNeighborLinesCostFunc(src_line_pos, tgt_line_pos);
								int src_camid = src_line_view_observation.camera_id;
								int tgt_camid = tgt_line_view_observation.camera_id;
								Eigen::Matrix3d srck = cams_[src_camid].cam_k_;
								Eigen::Matrix3d tgtk = cams_[tgt_camid].cam_k_;
								tmp_line_src_reproject_fun->SetTwoNeighborKMatrix(srck, tgtk);
								tmp_line_src_reproject_fun->SetPlaneInfo(tmp_plane_info.f_);
								tmp_line_src_reproject_fun->SetEnergyScale(lines_to_lines_lambda_);
								lines_plane_cost_func_ = tmp_line_src_reproject_fun;
								all_cams_pnts_problems_.AddResidualBlock(lines_plane_cost_func_, NULL,
									cams_extrs_[src_camid].cam_extr_, cams_extrs_[tgt_camid].cam_extr_);

								CamPoseOnlyFromReprojectNeighborLinesCostFunc * tmp_line_tgt_reproject_fun =
									new CamPoseOnlyFromReprojectNeighborLinesCostFunc(tgt_line_pos, src_line_pos);
								//int src_camid = src_line_view_observation.camera_id;
								//int tgt_camid = tgt_line_view_observation.camera_id;
								//Eigen::Matrix3d srck = cams_[src_camid].cam_k_;
								//Eigen::Matrix3d tgtk = cams_[tgt_camid].cam_k_;
								tmp_line_tgt_reproject_fun->SetTwoNeighborKMatrix(tgtk, srck);
								tmp_line_tgt_reproject_fun->SetPlaneInfo(tmp_plane_info.f_);
								tmp_line_tgt_reproject_fun->SetEnergyScale(lines_to_lines_lambda_);
								lines_plane_cost_func_ = tmp_line_tgt_reproject_fun;
								all_cams_pnts_problems_.AddResidualBlock(lines_plane_cost_func_, NULL,
									cams_extrs_[tgt_camid].cam_extr_, cams_extrs_[src_camid].cam_extr_);
							}
						}
					}
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

		bool HWSceneBundleCamsAdjustment::lm_optimize_epoch_only_cams_grouped_lines()
		{
			int planes_info_num = static_cast<int>(planes_.size());

#if 1	//test the points energy
			//std::cerr << "planes_info_num: " << planes_info_num << std::endl;
			//add image pnt reproject to polygon pnts and compute their dist each other error
			for (int i = 0; i < points_3d_tracklist_.size(); ++i)
			{
				//std::cerr << "points_3d_tracklist_->i: " << i << std::endl;
				PointWorld3dTrackList pnt2track = points_3d_tracklist_[i];
				//get plane id (they have same plane id)
				std::vector<int> track_plane_idx = pnt2track.plane_idx_;
				int track_plane_idxs_num = static_cast<int>(track_plane_idx.size());
				//std::cerr << "track_plane_idxs_num: " << track_plane_idxs_num << std::endl;
				if (!track_plane_idx.empty())
				{
					//get one plane id
					int plane_idx_pri0 = track_plane_idx[0];
					//std::cerr << "plane_idx_pri0: " << plane_idx_pri0 << std::endl;
					if (plane_idx_pri0 != -1)
					{
						//get another plane idx
						//int plane_idx_pri1 = track_plane_idx[1]; (to do next...)
						int pnt_track_ob_num = static_cast<int> (pnt2track.pnts3d_to_views_observations_.size());
						//get plane info
						Plane3D tmp_plane_info = planes_[plane_idx_pri0];
						//std::cerr << "pnt_track_ob_num: " << pnt_track_ob_num << std::endl;
						std::vector<int> pnts3d_to_views_obs_idxs;
						for (int j = 0; j < pnt2track.pnts3d_to_views_observations_.size(); ++j)
						{
							int tmp_pnts3d_to_views_obs_idx = j;
							pnts3d_to_views_obs_idxs.emplace_back(tmp_pnts3d_to_views_obs_idx);
						}
						//std::cerr << "123123123123123123" << std::endl;
						if (pnt_track_ob_num >= 2)
						{
							std::vector<std::pair<int, int> > pnts3d_local_obs_idxs
								= GenerateCompletePairsFromNVector(pnts3d_to_views_obs_idxs);
							for (int j = 0; j < pnts3d_local_obs_idxs.size(); ++j)
							{
								int src_idx = pnts3d_local_obs_idxs[j].first;
								int tgt_idx = pnts3d_local_obs_idxs[j].second;
								//std::cerr << "j, src_idx, tgt_idx: " << j << ", " << src_idx << ", " << tgt_idx << std::endl;
								//get src idx observation and tgt idx observation
								PntViewObservation src_view_observation = pnt2track.pnts3d_to_views_observations_[src_idx];
								PntViewObservation tgt_view_observation = pnt2track.pnts3d_to_views_observations_[tgt_idx];
								CamPoseOnlyFromReprojectNeighborPntsCostFunc * tmp_reproject_fun =
									new CamPoseOnlyFromReprojectNeighborPntsCostFunc(src_view_observation.pos_, tgt_view_observation.pos_);
								int src_camid = src_view_observation.camera_id;
								int tgt_camid = tgt_view_observation.camera_id;
								//std::cerr << "pnts src_camid, tgt_camid: " << src_camid << ", " << tgt_camid << std::endl;
								Eigen::Matrix3d srck = cams_[src_camid].cam_k_;
								Eigen::Matrix3d tgtk = cams_[tgt_camid].cam_k_;

								tmp_reproject_fun->SetTwoNeighborKMatrix(srck, tgtk);
								tmp_reproject_fun->SetPlaneInfo(tmp_plane_info.f_);	//set its
								tmp_reproject_fun->SetEnergyScale(pnts_to_pnts_lambda_);
								//pnts_plane_cost_func_ = tmp_reproject_fun;
								all_cams_pnts_problems_.AddResidualBlock(tmp_reproject_fun, NULL,
									cams_extrs_[src_camid].cam_extr_, cams_extrs_[tgt_camid].cam_extr_);
							}
						}
					}
				}
			}
#endif

#if 1	//test the lines energy
			//add image line reproject to polygon lines and compute their dist each other error
			for (int i = 0; i < lines_3d_tracklist_.size(); ++i)
			{
				//int 
				LineWorld3dTrackList line2track = lines_3d_tracklist_[i];
				//get plane id (they have same plane id)
				std::vector<int> track_plane_idx = line2track.plane_idx_;
				if (!track_plane_idx.empty())
				{
					//get one plane id
					int plane_idx_pri0 = track_plane_idx[0];
					if (plane_idx_pri0 != -1)
					{
						//get another plane idx
						//int plane_idx_pri1 = track_plane_idx[1]; (to do next...)
						int line_track_ob_num = static_cast<int> (line2track.lines3d_to_views_observations_.size());
						//get plane info
						Plane3D tmp_plane_info = planes_[plane_idx_pri0];
						std::vector<int> lines3d_to_views_obs_idxs;
						for (int j = 0; j < line2track.lines3d_to_views_observations_.size(); ++j)
						{
							int tmp_lines3d_to_views_obs_idx = j;
							lines3d_to_views_obs_idxs.emplace_back(tmp_lines3d_to_views_obs_idx);
						}
						if (line_track_ob_num >= 2)
						{
							std::vector<std::pair<int, int> > lines3d_local_obs_idxs;
							lines3d_local_obs_idxs = GenerateCompletePairsFromNVector(lines3d_to_views_obs_idxs);
							for (int j = 0; j < lines3d_local_obs_idxs.size(); ++j)
							{
								int src_idx = lines3d_local_obs_idxs[j].first;
								int tgt_idx = lines3d_local_obs_idxs[j].second;
								//get src idx observation and tgt idx observation
								LineViewObservation src_line_view_observation = line2track.lines3d_to_views_observations_[src_idx];
								LineViewObservation tgt_line_view_observation = line2track.lines3d_to_views_observations_[tgt_idx];
								std::pair<Eigen::Vector2d, Eigen::Vector2d> src_line_pos = std::make_pair(src_line_view_observation.ls2d_, src_line_view_observation.le2d_);
								std::pair<Eigen::Vector2d, Eigen::Vector2d> tgt_line_pos = std::make_pair(tgt_line_view_observation.ls2d_, tgt_line_view_observation.le2d_);
								int src_camid = src_line_view_observation.camera_id;
								int tgt_camid = tgt_line_view_observation.camera_id;
								//it is slow, if speed it, we should genarate pair by grouped views,to do next
								//std::cerr << "lines src_camid, tgt_camid: " << src_camid << ", " << tgt_camid << std::endl;
								if (src_camid == tgt_camid)
								{
									continue;
								}
								CamPoseOnlyFromReprojectNeighborLinesCostFunc * tmp_line_src_reproject_fun =
									new CamPoseOnlyFromReprojectNeighborLinesCostFunc(src_line_pos, tgt_line_pos);
								Eigen::Matrix3d srck = cams_[src_camid].cam_k_;
								Eigen::Matrix3d tgtk = cams_[tgt_camid].cam_k_;
								tmp_line_src_reproject_fun->SetTwoNeighborKMatrix(srck, tgtk);
								tmp_line_src_reproject_fun->SetPlaneInfo(tmp_plane_info.f_);
								tmp_line_src_reproject_fun->SetEnergyScale(lines_to_lines_lambda_);
								lines_plane_cost_func_ = tmp_line_src_reproject_fun;
								all_cams_pnts_problems_.AddResidualBlock(lines_plane_cost_func_, NULL,
									cams_extrs_[src_camid].cam_extr_, cams_extrs_[tgt_camid].cam_extr_);

								CamPoseOnlyFromReprojectNeighborLinesCostFunc * tmp_line_tgt_reproject_fun =
									new CamPoseOnlyFromReprojectNeighborLinesCostFunc(tgt_line_pos, src_line_pos);
								//int src_camid = src_line_view_observation.camera_id;
								//int tgt_camid = tgt_line_view_observation.camera_id;
								//Eigen::Matrix3d srck = cams_[src_camid].cam_k_;
								//Eigen::Matrix3d tgtk = cams_[tgt_camid].cam_k_;
								tmp_line_tgt_reproject_fun->SetTwoNeighborKMatrix(tgtk, srck);
								tmp_line_tgt_reproject_fun->SetPlaneInfo(tmp_plane_info.f_);
								tmp_line_tgt_reproject_fun->SetEnergyScale(lines_to_lines_lambda_);
								lines_plane_cost_func_ = tmp_line_tgt_reproject_fun;
								all_cams_pnts_problems_.AddResidualBlock(lines_plane_cost_func_, NULL,
									cams_extrs_[tgt_camid].cam_extr_, cams_extrs_[src_camid].cam_extr_);
							}
						}
					}
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

		std::vector<std::pair<int, int> > HWSceneBundleCamsAdjustment::GenerateCompletePairsFromNVector(const std::vector<int> vec)
		{
			std::vector<std::pair<int, int> > tmp_complete_pairs;
			int vec_num = static_cast<int>(vec.size());
			if (vec_num < 2)
			{
				return tmp_complete_pairs;
			}
			for (int i = 0; i < vec.size() - 1; ++i)
			{
				for (int j = i + 1; j < vec.size(); ++j)
				{
					std::pair<int, int> tmp_pair = std::make_pair(vec[i], vec[j]);
					tmp_complete_pairs.emplace_back(tmp_pair);
				}
			}
			return tmp_complete_pairs;
		}

		Eigen::Vector3d HWSceneBundleCamsAdjustment::ImgPntRayPlaneInfoPnt3d(const CameraParams& cam,
			const Plane3D& plane_info, const Eigen::Vector2d& img_pnt)
		{
			Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info.f_[0], plane_info.f_[1], plane_info.f_[2]);
			Eigen::Matrix3d rcw = cam.cam_extr_.topLeftCorner(3, 3);
			Eigen::Vector3d tcw = cam.cam_extr_.topRightCorner(3, 1);
			Eigen::Vector3d cmc = -rcw.transpose() * tcw;
			Eigen::Matrix3d camk = cam.cam_k_;
			double fx = camk(0, 0);
			double fy = camk(1, 1);
			double cx = camk(0, 2);
			double cy = camk(1, 2);
			double x_c = (img_pnt[0] - cx) / fx;
			double y_c = (img_pnt[1] - cy) / fy;
			
			Eigen::Vector3d cmc_ray_d = rcw.transpose() * Eigen::Vector3d(x_c, y_c, 1.0);
			//需要对其归一化，这样才行。
			//cmc_ray_d.norm();	//为啥这样求
			double cmc_ray_len = (-plane_info.f_[3] - cmc.dot(plane_n)) / cmc_ray_d.dot(plane_n);
			//std::cerr << "ImgPntRayPlaneInfoPnt3d->cmc_ray_len: " << cmc_ray_len << std::endl;
			Eigen::Vector3d cmc_ray_plane_pnt = cmc + cmc_ray_len * cmc_ray_d;
			return cmc_ray_plane_pnt;
		}

		Eigen::Vector3d HWSceneBundleCamsAdjustment::ImgPntRayPlaneInfoPnt3dNew(const CameraParams& cam,
			const Plane3D& plane_info, const Eigen::Vector2d& img_pnt)
		{
			Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info.f_[0], plane_info.f_[1], plane_info.f_[2]);
			Eigen::Matrix3d rcw = cam.cam_extr_.topLeftCorner(3, 3);
			Eigen::Vector3d tcw = cam.cam_extr_.topRightCorner(3, 1);
			Eigen::Vector3d cmc = -rcw.transpose() * tcw;
			Eigen::Matrix3d camk = cam.cam_k_;
			Eigen::Vector3d cmc_ray_d = rcw.transpose() * camk.inverse()*Eigen::Vector3d(img_pnt[0], img_pnt[1], 1.0);

			double cmc_ray_len = (-plane_info.f_[3] - cmc.dot(plane_n)) / cmc_ray_d.dot(plane_n);
			Eigen::Vector3d cmc_ray_plane_pnt = cmc + cmc_ray_len * cmc_ray_d;
			return cmc_ray_plane_pnt;
		}

		Eigen::Vector3d HWSceneBundleCamsAdjustment::ImgPntRayPlaneInfoPnt3dFromCamCoord(const CameraParams& cam,
			const Plane3D& plane_info, const Eigen::Vector2d& img_pnt)
		{
			//Ax+By+Cz+D = 0;
			Eigen::Vector3d plane_n = Eigen::Vector3d(plane_info.f_[0], plane_info.f_[1], plane_info.f_[2]);
			double plane_D = plane_info.f_[3];
			Eigen::Vector3d plane_ov_w = -plane_D * plane_n / (plane_n.dot(plane_n));	//
			Eigen::Matrix3d rcw = cam.cam_extr_.topLeftCorner(3, 3);
			Eigen::Vector3d tcw = cam.cam_extr_.topRightCorner(3, 1);
			Eigen::Vector3d plane_ov_c = rcw * plane_ov_w + tcw;
			Eigen::Vector3d plane_nv_c = rcw * plane_n;	//只有方向
			Eigen::Matrix3d camk = cam.cam_k_;
			double fx = camk(0, 0);
			double fy = camk(1, 1);
			double cx = camk(0, 2);
			double cy = camk(1, 2);
			double x_c = (img_pnt[0] - cx) / fx;
			double y_c = (img_pnt[1] - cy) / fy;
			//相机坐标系下的方向，因为原点为(0,0,0)
			Eigen::Vector3d cam_c_ray_d = Eigen::Vector3d(x_c, y_c, 1.0);
			Eigen::Vector3d cam_c_o = Eigen::Vector3d(0.0, 0.0, 0.0);
			//
			double cam_c_ray_len = (plane_ov_c - cam_c_o).dot(plane_nv_c) / (cam_c_ray_d.dot(plane_nv_c));
			Eigen::Vector3d uv_ray_plane_pnt_c = cam_c_ray_len * cam_c_ray_d;
			Eigen::Vector3d uv_ray_plane_pnt_w = rcw.transpose() * uv_ray_plane_pnt_c - rcw.transpose()*tcw;
			return uv_ray_plane_pnt_w;
		}

		Eigen::Vector2d HWSceneBundleCamsAdjustment::Pnt3dReprojectImagePntFromCameraParams(const CameraParams& cam, const Eigen::Vector3d& pnt_world)
		{
			Eigen::Vector4d test_view_pnt3d_world = Eigen::Vector4d(pnt_world[0], pnt_world[1], pnt_world[2], 1.0);
			Eigen::Vector4d test_view_pnt3d_cam = cam.cam_extr_*test_view_pnt3d_world;
			//std::cerr << "test_view_pnt3d_world: " << test_view_pnt3d_world.transpose() << std::endl;
			//std::cerr << "test_view_pnt3d_cam: " << test_view_pnt3d_cam.transpose() << std::endl;
			double u_new = cam.cam_k_(0, 0)*test_view_pnt3d_cam[0] / test_view_pnt3d_cam[2] + cam.cam_k_(0, 2);
			double v_new = cam.cam_k_(1, 1)*test_view_pnt3d_cam[1] / test_view_pnt3d_cam[2] + cam.cam_k_(1, 2);
			return Eigen::Vector2d(u_new, v_new);
		}

		Eigen::Vector3d HWSceneBundleCamsAdjustment::ImgPntRayPlaneInfoPnt3dTest_depth1(const CameraParams& cam, const Eigen::Vector2d& img_pnt)
		{
			Eigen::Matrix3d rcw = cam.cam_extr_.topLeftCorner(3, 3);
			Eigen::Vector3d tcw = cam.cam_extr_.topRightCorner(3, 1);
			Eigen::Vector3d cmc = -rcw.transpose() * tcw;
			Eigen::Matrix3d camk = cam.cam_k_;
			Eigen::Vector3d cmc_ray_d = rcw.transpose() * camk.inverse() * Eigen::Vector3d(img_pnt[0], img_pnt[1], 1.0);
			Eigen::Vector3d cmc_ray_plane_pnt = cmc + cmc_ray_d;
			return cmc_ray_plane_pnt;
		}

		Eigen::Vector2d HWSceneBundleCamsAdjustment::Pnt3dReprojectImagePntFromCameraParamsTest1(const CameraParams& cam, const Eigen::Vector3d& pnt_world)
		{
			//Eigen::Vector4d test_view_pnt3d_world = Eigen::Vector4d(pnt_world[0], pnt_world[1], pnt_world[2], 1.0);
			//Eigen::Vector4d test_view_pnt3d_cam = cam.cam_extr_*test_view_pnt3d_world;
			Eigen::Matrix3d cam_r_cw = cam.cam_extr_.topLeftCorner(3, 3);
			Eigen::Vector3d cam_t_cw = cam.cam_extr_.topRightCorner(3, 1);
			Eigen::Vector3d p_c = cam_r_cw * pnt_world + cam_t_cw;
			Eigen::Vector3d p_i = cam.cam_k_*p_c;
			std::cerr << "P_i: " << p_i.transpose() << std::endl;
			return Eigen::Vector2d(p_i[0], p_i[1]);
		}

		Eigen::Vector2d HWSceneBundleCamsAdjustment::Pnt3dReprojectImagePntFromCameraParamsTest2(const CameraParams& cam, const Eigen::Vector3d& pnt_world)
		{
			Eigen::Matrix3d cam_r_cw = cam.cam_extr_.topLeftCorner(3, 3);
			Eigen::Vector3d cam_t_cw = cam.cam_extr_.topRightCorner(3, 1);
			Eigen::Vector3d p_c = cam_r_cw * pnt_world + cam_t_cw;
			double u_new = cam.cam_k_(0, 0)*p_c[0] - cam.cam_k_(0, 2);
			double v_new = cam.cam_k_(1, 1)*p_c[1] - cam.cam_k_(1, 2);
			return Eigen::Vector2d(u_new, v_new);
		}
	}
}