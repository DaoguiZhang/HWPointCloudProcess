#pragma once
#ifndef HW_SCENE_BUNDLE_CAMS_ADJUSTMENT_H
#define HW_SCENE_BUNDLE_CAMS_ADJUSTMENT_H

#include"hw_scene_bundle_type.h"
#include <ceres/ceres.h>
#define GOOGLE_GLOG_DLL_DECL
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include "glog/logging.h"

namespace HW
{
	namespace BA
	{
		class HWSceneBundleCamsAdjustment
		{
		public:

			HWSceneBundleCamsAdjustment();

			void setBundleOptimizationIterNum(int iter_max_num);
			void setBundleReprojectAverageError(double average_error);

			void setCameras(const std::vector<CameraParams>& cams);
			void setViewsPoints(const std::vector<Point3D>& views_points);
			void setPluckerLine3DPoints(const std::vector<PluckerLinePoint3D>& plucker_lines);
			void setViewsLine3DPoints(const std::vector<LinePoint3D>& views_lines_pnts);
			void setViewObservationPntsTracklist(const std::vector<PointWorld3dTrackList>& pnts_tracks);
			void setViewObservationlinesTracklist(const std::vector<LineWorld3dTrackList>& lines_tracks);
			void setPlanes(const std::vector<Plane3D>& planes);

			const std::vector<CameraParams>& GetCameras();
			void UpdatePreparedBundleData();
			
			void UpdateCameraParamPartToCameraExtr();
			void UpdatePlanes2WorldPlanes();

			//update optimized results
			void UpdateBundleDataToData();
			void UpdateBundleCameraToCameraExtr();
			//cam and plane info. compute new pnts tracklist opti
			void UpdateTracklistPntsToTracklistPntsOpti();
			void UpdateTracklistLinesToTracklistLinesOpti();
			void UpdateBunldePlanesToPlanes();

			const std::vector<Point3D>& GetViewsPoints();
			const std::vector<LinePoint3D>& GetViewsLine3DPoints();
			const std::vector<Point3D>& GetPreViewsPoints();
			const std::vector<LinePoint3D>& GetPreViewsLine3DPoints();
			const std::vector<PointWorld3dTrackList>& GetViewObservationPntsTracklist();
			const std::vector<LineWorld3dTrackList>& GetViewObservationlinesTracklist();
			
			Eigen::Matrix4d ConvertBundleCameraToCamExtr(double* bundle_cam);
			bool optimize_cams_only();
			bool optimize_cams_only_grouped_line();
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
			bool lm_optimize_only_cams_grouped_lines();
			bool lm_optimize_epoch_only_cams();
			bool lm_optimize_epoch_only_cams_grouped_lines();

			std::vector<std::pair<int, int> > GenerateCompletePairsFromNVector(const std::vector<int> vec);
			Eigen::Vector3d ImgPntRayPlaneInfoPnt3d(const CameraParams& cam, 
				const Plane3D& plane_info, const Eigen::Vector2d& img_pnt);
			Eigen::Vector3d ImgPntRayPlaneInfoPnt3dNew(const CameraParams& cam,
				const Plane3D& plane_info, const Eigen::Vector2d& img_pnt);
			Eigen::Vector3d ImgPntRayPlaneInfoPnt3dFromCamCoord(const CameraParams& cam,
				const Plane3D& plane_info, const Eigen::Vector2d& img_pnt);
			Eigen::Vector2d Pnt3dReprojectImagePntFromCameraParams(const CameraParams& cam, const Eigen::Vector3d& pnt_world);

			Eigen::Vector3d ImgPntRayPlaneInfoPnt3dTest_depth1(const CameraParams& cam, const Eigen::Vector2d& img_pnt);
			Eigen::Vector2d Pnt3dReprojectImagePntFromCameraParamsTest1(const CameraParams& cam, const Eigen::Vector3d& pnt_world);
			Eigen::Vector2d Pnt3dReprojectImagePntFromCameraParamsTest2(const CameraParams& cam, const Eigen::Vector3d& pnt_world);

			std::vector<CameraParams> cams_;
			std::vector<Point3D> points_views_3d_;	//contain all view points 3d
			std::vector<Point3D> pre_points_views_3d_;
			std::vector<LinePoint3D> lines_views_points_3d_;
			std::vector<LinePoint3D> pre_lines_views_points_3d_;

			std::vector<Plane3D> planes_;
			
			//get optimization params
			std::vector<PointWorld3dTrackList> points_3d_tracklist_;
			std::vector<LineWorld3dTrackList> lines_3d_tracklist_;

			//the params to optimize
			std::vector<CameraExtr> cams_extrs_;
			std::vector<PlaneWorldFun> w_planes_;
			
			//std::vector<CameraExtrPnt> extr_cams_pnts_;
			std::vector<PointWorld3dTrackList> points_3d_tracklist_opti_;
			std::vector<LineWorld3dTrackList> lines_3d_tracklist_opti_;

			ceres::Problem all_cams_pnts_problems_;
			ceres::Problem all_planes_lines_problems_;
			ceres::CostFunction* pnts_plane_cost_func_;
			ceres::CostFunction* lines_plane_cost_func_;
			
			//pnts to pnt
			double pnts_to_pnts_lambda_;
			double lines_to_lines_lambda_;
			
			//lines corresponding polygon intersacted lines
			double lines_inter_to_polygons_lines_lambda_;
			
			double bundle_cams_pnts_loss_value_;	//optimized value
			double bundle_plane_lines_loss_value_;	//optimized value
		};
	}
}

#endif