#pragma once
#ifndef HW_BUNDLE_ADJUSTMENT_HEADER
#define HW_BUNDLE_ADJUSTMENT_HEADER

#include <vector>
//#include"hw_sfm_logging.h"
#include"hw_cmns.h"
#include"hw_ba_sparse_matrix.h"
#include"hw_ba_dense_vector.h"
#include"hw_ba_linear_solver.h"
#include"hw_ba_types.h"
#include <ceres/ceres.h>
#define GOOGLE_GLOG_DLL_DECL
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include "glog/logging.h"

namespace HWSFM
{
	namespace BA
	{
		class HWBundleAdjustment
		{
		public:
			enum HWBAMode
			{
				BA_CAMERAS = 1,
				BA_POINTS = 2,
				BA_CAMERAS_POINTS = 3,
				BA_CAMERAS_LINES = 4,
				BA_CAMERAS_POINTS_LINES_PLANES = 5,
				BA_CAMERAS_LINES_PLANES = 6
			};

			struct HWBAOptions
			{
				HWBAOptions(void);
				bool verbose_output;
				HWBAMode bundle_mode;
				bool fixed_intrinsics; //fix 内参, 不优化
				int lm_max_iterations;
				int lm_min_iterations;
				double lm_delta_threshold;
				double lm_mse_threshold;
				//LinearSolver::Options linear_ops;
			};

			struct HWBAStatus
			{
				HWBAStatus(void);
				double initial_mse;
				double final_mse;
				int num_lm_iterations;
				int num_lm_successful_iterations;
				int num_lm_unsuccessful_intrations;
				int num_cg_iterations;
				std::size_t runtime_ms;
			};

		public:
			HWBundleAdjustment(HWBAOptions const& options);

			void setCameras(std::vector<Camera>* cams);
			void setPoints(std::vector<Point3D>* points);
			void setLine3DPoints(std::vector<PluckerLinePoint3D>* plucker_lines);
			void setObservations(std::vector<Observation>* observations);
			void setLinesObservations(std::vector<Observation>* lines_observations);

			HWBAStatus optimize();
			void FittingLine3dFromPnts3d(const std::vector<Eigen::Vector3d>& pnts, Eigen::Vector4d& line_func);
			void print_status() const;

		private:

			//void rodrigues_to_matrix_test(float const* r, float* m);

			void lm_optimize();

			void compute_reprojection_errors();

			void analytic_jacobian();

			HWBAOptions opts_;
			HWBAStatus status_;
			std::vector<Camera>* cameras_;
			std::vector<Point3D>* points_;
			std::vector<PluckerLinePoint3D>* plucker_lines_3d_;	//do not use
			std::vector<Observation>* observations_;
			std::vector<LineObservation>* lines_observations_;
			int num_cam_params_;

			ceres::Problem all_functions_problem_;
			ceres::CostFunction* pnts_cost_function_;
			ceres::CostFunction* line_cost_function_;
			ceres::CostFunction* plane_cost_function_;
			
			std::vector<CameraExtrPnt> cams_params_pnts_;
		};

		/* ------------------------ Implementation ------------------------ */

		inline
			HWBundleAdjustment::HWBAOptions::HWBAOptions(void)
			: verbose_output(false)
			, bundle_mode(BA_CAMERAS_POINTS_LINES_PLANES)
			, lm_max_iterations(50)
			, lm_min_iterations(0)
			, lm_delta_threshold(1e-4)
			, lm_mse_threshold(1e-8)
		{
		}

		inline
			HWBundleAdjustment::HWBAStatus::HWBAStatus(void)
			: initial_mse(0.0)
			, final_mse(0.0)
			, num_lm_iterations(0)
			, num_lm_successful_iterations(0)
			//, num_lm_unsuccessful_iterations(0)
			, num_cg_iterations(0)
		{
		}

		inline
			HWBundleAdjustment::HWBundleAdjustment(HWBAOptions const& options)
			: opts_(options)
			, cameras_(nullptr)
			, points_(nullptr)
			, observations_(nullptr)
			, num_cam_params_(options.fixed_intrinsics ? 6 : 9)
		{
			//this->opts_.linear_opts_.camera_block_dim = this->num_cam_params;
		}

		inline void
			HWBundleAdjustment::setCameras(std::vector<Camera>* cameras)
		{
			this->cameras_ = cameras;
		}

		inline void
			HWBundleAdjustment::setPoints(std::vector<Point3D>* points)
		{
			this->points_ = points;
		}

		inline void HWBundleAdjustment::setLine3DPoints(std::vector<PluckerLinePoint3D>* plucker_lines)
		{
			this->plucker_lines_3d_ = plucker_lines;
		}

		inline void
			HWBundleAdjustment::setObservations(std::vector<Observation>* points_2d)
		{
			this->observations_ = points_2d;
		}

		inline void HWBundleAdjustment::setLinesObservations(std::vector<Observation>* lines_observations)
		{
			//this->lines_observations_ = lines_observations;
		}
	}
}

#endif