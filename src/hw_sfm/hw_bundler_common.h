/*
* Copyright (C) 2015, Simon Fuhrmann
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the LICENSE.txt file for details.
*/

#ifndef HW_SFM_BUNDLER_COMMON_HEADER
#define HW_SFM_BUNDLER_COMMON_HEADER

#include <string>
#include <unordered_map>
#include <vector>

//#include "util/aligned_memory.h"
//#include "mve/image.h"
#include "hw_ba_camera_pose.h"
#include "hw_correspondence.h"
//#include "sfm/feature_set.h"
//#include "hw_sift.h"
//#include "hw_surf.h"
//#include "sfm/defines.h"
#include<Eigen/Eigen>
#include<opencv2/xfeatures2d/nonfree.hpp>
#include"hw_feature_set.h"

namespace HWSFM
{
	/* -------------------- Common Data Structures -------------------- */
	typedef std::pair<Eigen::Vector2f, Eigen::Vector2f> BUNDLELINEPIONT2DTYPE;
	typedef Eigen::Vector2i BUNDLELINEIDXTYPE;
	typedef struct BundlerLine3DNode
	{
		Eigen::Vector2i adj_poly_idxs_;	//two polygon idx
		Eigen::Vector3f ls3d_;	//polygon intersection line start
		Eigen::Vector3f le3d_;	//polygon intersection line end
		std::vector<int> cams_visible_idxs_;	//
		std::vector<std::pair<int, BUNDLELINEPIONT2DTYPE> > sample_imgs_line_pnts_;	//img idx; line pnts(图片的坐标)
		//polygon line对应的所看到的图像的线段并且保存它们的值, int2 first 是img idx, second是line idx
		std::vector<std::pair<BUNDLELINEIDXTYPE, float> > Imgidx2Imgev_;

	}BundlerLine3DNode;

	/*
	目的：保存来自hw_scene_data的数据
	*/
	struct HWBundleViewsMatch
	{
		// correspondence
		unsigned int src_view_camID_;	//image id = cam id
		unsigned int src_view_segID_;
		unsigned int tgt_view_camID_;
		unsigned int tgt_view_segID_;

		bool valid_match_;

		//check two lines if interior or exterior
		bool is_polygon_interior_;
		Eigen::Vector2i adj_poly_idxs_;	//3d line index to two polygon idx

		// scores
		double overlap_score_;
		double score3D_;

		// depths
		double depth_p1_;
		double depth_p2_;
		double depth_q1_;
		double depth_q2_;

		//world coordiante pnts from two views
		Eigen::Vector3f world_p1_;
		Eigen::Vector3f world_p2_;
		Eigen::Vector3f world_q1_;
		Eigen::Vector3f world_q2_;
	};

	typedef std::pair<cv::Vec2f, cv::Vec2f> HWBundleLineSegment2D;

	/**
	* Per-viewport information.
	* Not all data is required for every step. It should be populated on demand
	* and cleaned as early as possible to keep memory consumption in bounds.
	*/
	struct HWViewport
	{
		HWViewport(void);

		/** Initial focal length estimate for the image. */
		float focal_length;
		/** Radial distortion parameter. */
		float radial_distortion[2];
		/** Principal point parameter. */
		float principal_point[2];

		/** Camera pose for the viewport. */
		HWBACameraPose pose;
		int hw_camera_id_;	//camera id to CameraModels

		/** The actual image data for debugging purposes. Usually nullptr! */
		//mve::ByteImage::Ptr image;	//to do next zdg 
		cv::Ptr<cv::Mat> image_;

		/** Per-feature information. */
		//sfm::FeatureSet features;	//no feature set, to do next zdg ... sift...
		HWFeatureSet features_;	//
		std::vector<HWBundleLineSegment2D> lines_segments_;

		//cv::xfeatures2d::s
		/** Per-feature track ID, -1 if not part of a track. */
		std::vector<int> track_ids;
		
		/** Per-line track ID, -1 if not part of a track. */
		std::vector<int> lines_track_ids;

		/** Backup map from features to tracks that were removed due to errors. */
		std::unordered_map<int, int> backup_tracks;
	};

	/** The list of all viewports considered for bundling. */
	typedef std::vector<HWViewport> HWViewportList;

	/* --------------- Data Structure for Feature Tracks -------------- */

	/** References a 2D feature in a specific view. */
	struct HWFeatureReference
	{
		HWFeatureReference(int view_id, int feature_id);

		int view_id;
		int feature_id;
	};

	/** The list of all feature references inside a track. */
	typedef std::vector<HWFeatureReference> HWFeatureReferenceList;

	/** References a 2D feature in a specific view. */
	struct HWLineFeatureReference
	{
		HWLineFeatureReference(int view_id, int line_id);

		int view_id;
		int line_id;
	};

	/** The list of all feature references inside a track. */
	typedef std::vector<HWLineFeatureReference> HWLineFeatureReferenceList;

	/** Representation of a feature track. */
	struct HWTrack
	{
		bool is_valid(void) const;
		void invalidate(void);
		void remove_view(int view_id);

		cv::Vec3f pos;
		cv::Vec3b color;
		HWFeatureReferenceList features;
	};

	/** The list of all tracks. */
	typedef std::vector<HWTrack> HWTrackList;

	/** Representation of a feature track. */
	struct HWLineTrack
	{
		bool is_valid(void) const;
		void invalidate(void);
		void remove_view(int view_id);

		//math::Vec3f pos;
		cv::Vec3b color;
		cv::Vec3f pos;
		cv::Vec3f ld;	//line direct
		HWLineFeatureReferenceList linesfeatures;
	};

	/** The list of all tracks. */
	typedef std::vector<HWTrack> HWTrackList;
	typedef std::vector<HWLineTrack> HWLineTrackList;

	/* Observation of a survey point in a specific view. */
	struct HWSurveyObservation
	{
		HWSurveyObservation(int view_id, float x, float y);

		int view_id;
		cv::Vec2f pos;
	};

	/* Observation of a survey line in a specific view. */
	struct HWLineSurveyObservation
	{
		HWLineSurveyObservation(int view_id, cv::Vec2f& s, cv::Vec2f& t);

		int view_id;
		cv::Vec2f pos_s;
		cv::Vec2f pos_e;
		//math::Vec2f pos;
	};

	/** The list of all survey point observations inside a survey point. */
	typedef std::vector<HWSurveyObservation> HWSurveyObservationList;
	/** The list of all survey line observations inside a survey point. */
	typedef std::vector<HWLineSurveyObservation> HWLineSurveyObservationList;

	/** Representation of a survey point. 3D TO 2D data structure, by trianglating */
	struct HWSurveyPoint
	{
		cv::Vec3f pos;
		HWSurveyObservationList observations;
	};

	/** Representation of a line segment. */
	struct HWLineSurveyPoint
	{
		//math::Vec3f pos;
		cv::Vec3f pos_s;
		cv::Vec3f pos_e;
		HWLineSurveyObservationList observations;
	};

	/** The list of all survey poins. */
	typedef std::vector<HWSurveyPoint> HWSurveyPointList;

	/** The list of all survey lines. */
	typedef std::vector<HWLineSurveyPoint> HWLineSurveyPointList;

	/* ------------- Data Structures for Feature Matching ------------- */

	/** The matching result between two views. */
	struct HWTwoViewMatching
	{
		bool operator< (HWTwoViewMatching const& rhs) const;
		HWTwoViewMatching operator = (const HWTwoViewMatching& rhs);
		bool valid_pair = true;
		int view_1_id;
		int view_2_id;
		HWCorrespondenceIndices points_matches;	// points idx to point idx
		HWCorrespondenceIndices lines_matches;	// lines idx to line idx
		std::vector<bool> lines_matches_interior;	//check line match if it is interior if true->interior
	};

	/** The matching result between several pairs of views. */
	typedef std::vector<HWTwoViewMatching> HWPairwiseMatching;

	/* ------------------ Input/Output for Prebundle ------------------ */

	/**
	* Saves the pre-bundle data to file, which records all viewport and
	* matching data necessary for incremental structure-from-motion.
	*/
	void
		save_prebundle_to_file(HWViewportList const& viewports,
			HWPairwiseMatching const& matching, std::string const& filename);

	/**
	* Loads the pre-bundle data from file, initializing viewports and matching.
	*/
	void
		load_prebundle_from_file(std::string const& filename,
			HWViewportList* viewports, HWPairwiseMatching* matching);

	/**
	* Loads survey points and their observations from file.
	*
	* Survey file are ASCII files that start with the signature
	* MVE_SURVEY followed by a newline, followed by the number of survey points
	* and survey point observations.
	* Each survey point is a 3D point followed by a newline. Each survey point
	* observation is a line starting with the index of the survey point, followed
	* by the view id an the 2D location within the image. The (x, y) coordinates
	* have to be normalized such that the center of the image is (0, 0) and the
	* larger image dimension is one. This means that all image coordinates are
	* between (-0.5,-0.5) and (0.5, 0.5)
	*
	* MVE_SURVEY
	* <num_points> <num_observations>
	* <survey_point> // x y z
	* ...
	* <survey_point_observation> // survey_point_id view_id x y
	* ...
	*/
	void
		load_survey_from_file(std::string const& filename,
			HWSurveyPointList* survey_points);

	/* ---------------------- Feature undistortion -------------------- */

	/*math::Vec2f
		undistort_feature(math::Vec2f const& f, double const k1, double const k2,
			float const focal_length);*/

	//to do next zdg
	cv::Vec2f undistort_feature(cv::Vec2f const& f, double const k1, double const k2,
		float const focal_length);

	/*math::Vec2f undistort_feature(Eigen::Vector2f const& f, double const k1, double const k2,
			float const focal_length);*/

	/* ------------------------ Implementation ------------------------ */

	inline
		HWFeatureReference::HWFeatureReference(int view_id, int feature_id)
		: view_id(view_id)
		, feature_id(feature_id)
	{
	}

	inline
		HWLineFeatureReference::HWLineFeatureReference(int view_id, int line_id)
		: view_id(view_id)
		, line_id(line_id)
	{
	}

	inline
		HWSurveyObservation::HWSurveyObservation(int view_id, float x, float y)
		: view_id(view_id)
		, pos(x, y)
	{
	}

	inline
		HWLineSurveyObservation::HWLineSurveyObservation(int view_id, cv::Vec2f& s, cv::Vec2f& t)
		: view_id(view_id)
		, pos_s(s)
		, pos_e(t)
	{
	}

	inline bool
		HWTwoViewMatching::operator< (HWTwoViewMatching const& rhs) const
	{
		return this->view_1_id == rhs.view_1_id
			? this->view_2_id < rhs.view_2_id
			: this->view_1_id < rhs.view_1_id;
	}

	inline  HWTwoViewMatching
		HWTwoViewMatching::operator = (const HWTwoViewMatching& rhs)
	{
		HWTwoViewMatching lhs;
		lhs.view_1_id = rhs.view_1_id;
		lhs.view_2_id = rhs.view_2_id;
		lhs.valid_pair = rhs.valid_pair;
		lhs.lines_matches = rhs.lines_matches;
		lhs.lines_matches_interior = rhs.lines_matches_interior;
		lhs.points_matches = rhs.points_matches;
		return lhs;
	}

	inline
		HWViewport::HWViewport(void)
		: focal_length(0.0f)
	{
		std::fill(this->radial_distortion, this->radial_distortion + 2, 0.0f);
		std::fill(this->principal_point, this->principal_point + 2, 0.5f);
	}

	inline bool
		HWTrack::is_valid(void) const
	{
		return !std::isnan(this->pos[0]);
	}

	inline bool
		HWLineTrack::is_valid(void) const
	{
		return !std::isnan(this->pos[0]);	//to do next
	}

}

#endif
