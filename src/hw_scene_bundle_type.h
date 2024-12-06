#ifndef HW_SCENE_BUNDLE_TYPE_H
#define HW_SCENE_BUNDLE_TYPE_H

#pragma once
#include<Eigen/Eigen>
#include<Eigen/Core>

namespace HW
{
	namespace BA
	{
		/** Camera representation for bundle adjustment. */
		struct CameraExtrPnt
		{
			//w0, w1, w2, t0, t1, t2, p0, p1, p2
			CameraExtrPnt(void)
			{
				std::fill(this->cam_pnt_params_, this->cam_pnt_params_ + 9, 0.0);
			}
			double cam_pnt_params_[9];
			//bool is_constant = false;
		};

		struct CameraExtr
		{
			//cam_extr_(w0, w1, w2, t0, t1, t2)
			CameraExtr(void)
			{
				std::fill(this->cam_extr_, this->cam_extr_ + 6, 0.0);
			}
			double cam_extr_[6];
		};

		struct WorldPnt
		{
			//pos_(p0, p1, p2)
			WorldPnt(void)
			{
				std::fill(this->pos_, this->pos_ + 3, 0.0);
			}
			double pos_[3];
			std::vector<int> plane_idxs_;
		};

		struct LineWorldPnt
		{
			//pos_(p0, p1, p2); d_(d0, d1, d2)
			LineWorldPnt(void)
			{
				std::fill(this->pos_s_, this->pos_s_ + 3, 0.0);
				std::fill(this->pos_e_, this->pos_e_ + 3, 0.0);
			}
			double pos_s_[3];
			double pos_e_[3];
			//important: combine lines to polygons, to do next...
			std::vector<int> plane_idxs_;
		};

		struct LineWorldOthoLine
		{
			LineWorldOthoLine(void)
			{
				std::fill(this->line_otho_, this->line_otho_ + 4, 0.0);
			}
			double line_otho_[4];
			//important: combine lines to polygons, to do next...
			std::vector<int> plane_idxs_;
		};

		struct PlaneInterLineWorldPnt
		{
			//pos_(p0, p1, p2); d_(d0, d1, d2)
			PlaneInterLineWorldPnt(void)
			{
				std::fill(this->pos_s_, this->pos_s_ + 3, 0.0);
				std::fill(this->pos_e_, this->pos_e_ + 3, 0.0);
			}
			//assign to two polygon intersection line
			double pos_s_[3];
			double pos_e_[3];
			//important: combine lines to polygons, to do next
			std::vector<int> plane_idxs_;
		};

		struct PlaneLineWorldOthoLine
		{
			PlaneLineWorldOthoLine(void)
			{
				std::fill(this->line_otho_, this->line_otho_ + 4, 0.0);
			}
			double line_otho_[4];
			//important: combine lines to polygons, to do next...
			std::vector<int> plane_idxs_;
		};

		struct PlaneWorldFun
		{
			PlaneWorldFun()
			{
				std::fill(this->f_, this->f_ + 4, 0.0);
			}
			double f_[4];
		};

		struct CameraParams
		{
			CameraParams()
			{
				cam_k_ = Eigen::Matrix3d::Identity();
				cam_extr_ = Eigen::Matrix4d::Identity();
				is_constant = false;
			}
			Eigen::Matrix3d cam_k_;
			std::vector<double> distortion_;
			Eigen::Matrix4d cam_extr_;	//T_cw
			bool is_constant = false;
		};

		/** 3D point representation for bundle adjustment. */
		struct Point3D
		{
			Eigen::Vector3d pos_;
			std::vector<int> plane_idxs_;
			bool is_constant = false;
		};
		/** Line 3D points representation for bundle adjustment. */
		struct LinePoint3D
		{
			Eigen::Vector3d pos_s_;
			Eigen::Vector3d pos_e_;
			std::vector<int> plane_idxs_;
			bool is_constant = false;
			bool is_plane_line_ = false;
		};

		struct LineSegPoint3D
		{
			Eigen::Vector3d s_pos_;
			Eigen::Vector3d e_pos_;
			bool is_constant = false;
		};

		/** plane representation for bundle adjustment. */
		struct Plane3D
		{
			Eigen::Vector4d f_;
			bool is_constant = false;
		};

		/** line 3D representation for bundle adjustment. */
		struct PluckerLinePoint3D
		{
			//should meet [3]*[3]+[4]*[4]+[5]*[5] = 1
			//[0]*[3]+[1]*[4]+[2]*[5] = 0
			Eigen::Vector3d m_;
			Eigen::Vector3d d_;
			bool is_constant = false;
		};

		/** Observation of a 3D point for a camera. */
		struct PntObservation
		{
			Eigen::Vector2d pos_;
			int camera_id;
			int point_id;
			std::vector<int> plane_id;	//point id 2 plane id
		};

		/** Observation of a line 3D for a camera. */
		struct LineObservation
		{
			Eigen::Vector2d ls2d_;
			Eigen::Vector2d le2d_;
			int camera_id;
			int line3d_id;	//point to LinePoint3D
			//int plane_id[2];	//line id 2 plane id,can be multi plane idxs
			std::vector<int> plane_id;
		};

		/** Observation of a 3D point for a camera, contain single view pose 3d. */
		struct PntViewObservation
		{
			Eigen::Vector2d pos_;	//image observation pos
			int camera_id;
			int point_view_id;	//idx point to view points(every view point to a point 3d, no use maybe)
			std::vector<int> plane_view_id;	//point id 2 plane id
		};

		/** Observation of a 3D point for a camera, contain single view pose 3d. */
		struct LineViewObservation
		{
			Eigen::Vector2d ls2d_;	//image observation line start pnt
			Eigen::Vector2d le2d_;	//image observation line start pnt
			int camera_id;
			int line3d_view_id;	//point to LinePoint3D
			//int plane_id[2];	//line id 2 plane id,can be multi plane idxs
			std::vector<int> plane_view_id;	//no use

			//Eigen::Vector2d pos_;
			//int camera_id;
			//int point_id;
			//std::vector<int> plane_id;	//point id 2 plane id
			//Eigen::Vector3d pos_view_3d_;	//initial pos 3d 
		};

		/*Pnts 3d track list -> they have same plane info */
		struct PointWorld3dTrackList
		{
			double pnt_3d_[3];
			std::vector<int> plane_idx_;	//contain plane info
			std::vector<PntViewObservation> pnts3d_to_views_observations_;
		};

		/*Lines 3d track list -> they have same plane info  */
		struct LineWorld3dTrackList
		{
			double pnt_s_[3];
			double pnt_e_[3];
			bool is_plane_intersectin_line_ = false;
			std::vector<int> plane_idx_;	//contain plane info
			std::vector<LineViewObservation> lines3d_to_views_observations_;
		};
	}
}

#endif // !HW_SCENE_BUNDLE_TYPE_H
