#pragma once
#ifndef HW_SCENE_BUNDLE_ADJUSTMENT_H
#define HW_SCENE_BUNDLE_ADJUSTMENT_H

#include"hw_scene_bundle_type.h"
#include <ceres/ceres.h>
#define GOOGLE_GLOG_DLL_DECL
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include "glog/logging.h"

namespace HW
{
	namespace BA
	{
		class HWSceneBundleAdjustment
		{
		public:

			HWSceneBundleAdjustment();

			void setBundleOptimizationIterNum(int iter_max_num);
			void setBundleReprojectAverageError(double average_error);

			void setCameras(const std::vector<CameraParams>& cams);
			void setPoints(const std::vector<Point3D>& points);
			void setPluckerLine3DPoints(const std::vector<PluckerLinePoint3D>& plucker_lines);
			void setLine3DPoints(const std::vector<LinePoint3D>& lines_pnts);
			void setObservations(const std::vector<PntObservation>& pnts_observations);
			void setLinesObservations(const std::vector<LineObservation>& lines_observations);
			void setPlanes(const std::vector<Plane3D>& planes);

			const std::vector<CameraParams>& GetCameras();
			const std::vector<Point3D>& GetPoints();
			const std::vector<LinePoint3D>& GetLine3DPoints();
			const std::vector<PntObservation>& GetObservations();
			const std::vector<LineObservation>& GetLinesObservations();

			void UpdatePreparedBundleData();

			void UpdateCameraParamPartToCameraExtr();
			void UpdatePoint3DPartToWorldPnt();
			void UpdateLinePoint3DPart2LineWorldPnt();
			void UpdatePlanes2WorldPlanes();
			void UpdateLineWorldPnt2LineWorldOthoLine();
			void UpdatePlaneLineWorldPnt2PlaneLineWorldOthoLine();

			//update optimized results
			void UpdateBundleDataToData();
			void UpdateBundleCameraToCameraExtr();
			void UpdateBundlePnt3DToPnt3D();
			void UpdateOthoLine2BundleLineWorldLine();
			void UpdateBundleLineWorldPnts2LinePnts();
			void UpdateBunldePlanesToPlanes();

			Eigen::Matrix4d ConvertBundleCameraToCamExtr(double* bundle_cam);

			bool optimize();
			
			bool optimize_cams_only();

			void print_status() const;

			//debug
			void print_cams_info();
			void print_pnt3d_info();
			void print_lines3d_info();
			void print_cam_r_info(double* r);

		private:

			//max iter number
			int bundle_max_iteration_;
			int bundle_max_iteration_num_per_epoch_;
			int epoch_max_number_;	//max epoch's number
			double reproject_error_average_threhold_;

			//fixe lines 3d and pnts 3d. then optimize the cams extr
			bool lm_optimize_only_cams();
			bool lm_optimize_epoch_only_cams();


			bool lm_optimize();

			//fix planes and lines then optimize cams extr and pnts 3d 
			bool lm_optimize_single_iter_cams_pnts();

			//fix cams extr and pnts 3d then optimize planes and lines
			bool lm_optimize_single_iter_planes_lines();

			//get line idx from line id, if not found, return -1
			int GetWPlanelineIdxFromLineid(int lineid);
			int GetWlineIdxFromLineid(int lineid);
			int GetWOtholineIdxFromLineid(int lineid);
			int GetWPlaneOtholineIdxFromLineid(int lineid);


			std::vector<CameraParams> cams_;
			std::vector<Point3D> points_3d_;
			std::vector<LinePoint3D> lines_points_3d_;
			std::vector<double> lines_plucker_scale_3d_;
			std::vector<Plane3D> planes_;
			std::vector<PntObservation> pnts_observations_;
			std::vector<LineObservation> lines_observations_;
			//get optimization params
			std::vector<CameraExtrPnt> extr_cams_pnts_;

			std::vector<CameraExtr> cams_extrs_;
			std::vector<WorldPnt> w_pnts_;
			//w_lines: first value->lines_points_3d_'s idx; second value->LineWorldPnt  
			std::vector<std::pair<int, LineWorldPnt> > w_lines_;	//to plucker represent... to do next..
			std::vector<std::pair<int, PlaneInterLineWorldPnt> > w_planes_lines_;	//sample pnt 3d to optimize the pose(two pnts(too few))
			std::vector<std::pair<int, LineWorldOthoLine> > w_lines_otho_lines_;
			std::vector<std::pair<int, PlaneLineWorldOthoLine> > w_plane_lines_otho_lines_;
			std::vector<PlaneWorldFun> w_planes_;

			ceres::Problem all_cams_pnts_problems_;
			ceres::Problem all_planes_lines_problems_;
			ceres::CostFunction* pnts_cost_func_;
			ceres::CostFunction* lines_cost_func_;
			ceres::CostFunction* planes_cost_func_;
			
			//pnts to image projected pnts
			double pnts_to_images_pnts_lambda_;
			//pnts to polygons' planes
			double pnts_to_planes_lambda_;
			
			//lines corresponding polygon intersacted lines
			double lines_inter_to_polygons_lines_lambda_;
			double lines_inter_to_images_lines_lambda_;

			//lines not corresponding polygon intersacted lines
			double lines_to_polygons_lines_lambda_;	//just the 
			double lines_to_images_lines_lambda_;	//important, only optimize the pose

			double polygons_to_lines3d_lambda_;	//opitimize the polygon plane function params

			double bundle_cams_pnts_loss_value_;	//optimized value
			double bundle_plane_lines_loss_value_;	//optimized value
		};
	}
}

#endif