#ifndef HW_VIEW_CMNS_H
#define HW_VIEW_CMNS_H

#include"hw_cmns.h"
#include<set>
#include<unordered_map>
#include"model_cameras.h"

namespace HW
{
	//------------------------------------------------------------------------------
	/**
	* Per-viewport information.
	* Not all data is required for every step. It should be populated on demand
	* and cleaned as early as possible to keep memory consumption in bounds.
	*/
	struct HWCamViewport
	{
		HWCamViewport(void)
		{
			focal_length_x_ = 0.0;
			focal_length_y_ = 0.0;
			radial_distortion_[0] = 0.0;
			radial_distortion_[1] = 0.0;
			principal_point_[0] = 0.0;
			principal_point_[1] = 0.0;
			hw_camera_id_ = -1;
			hw_image_id_ = -1;
			hw_layout_id_ = -1;
			cam_optimized_valid_ = false;
		}

		/** Initial focal length estimate for the image. */
		float focal_length_x_;
		float focal_length_y_;
		/** Radial distortion parameter. */
		float radial_distortion_[2];
		/** Principal point parameter. */
		float principal_point_[2];

		/** Camera pose for the viewport. */
		CameraModel view_pose_;
		int hw_camera_id_;	//camera id to CameraModels id 
		int hw_image_id_;
		int hw_layout_id_;

		/** The actual image data for debugging purposes. Usually nullptr! */
		//mve::ByteImage::Ptr image;	//to do next zdg 
		//cv::Ptr<cv::Mat> image_;

		/** Per-feature information. */
		//sfm::FeatureSet features;	//no feature set, to do next zdg ... sift...
		std::vector<Eigen::Vector2f> features_poistion_;	//
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > lines_segments_;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > lines_segments_3d_;	//line segment 3d point 

		/*set the line 2d idx to combined grouped line idx*/
		std::vector<std::pair<int, int> > lines2d_idxs_to_grouped_lines2d_idxs_;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > grouped_lines_segments_;
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > grouped_lines_segments_3d_;	//line segment 3d point 

		//cv::xfeatures2d::s
		/** Per-feature track ID, -1 if not part of a track. view 2d pnt to HWTrack3D id */
		std::vector<int> track_ids_;

		/** Per-line track ID, -1 if not part of a track. */
		std::vector<int> lines_track_ids_;

		/** Backup map from features to tracks that were removed due to errors. */
		std::unordered_map<int, int> backup_tracks_;

		/* optimized view if is true */
		bool cam_optimized_valid_;		//check the cam is optimized correctly...

		void operator = (const HWCamViewport& other)
		{
			this->backup_tracks_ = other.backup_tracks_;
			this->features_poistion_ = other.features_poistion_;
			this->focal_length_x_ = other.focal_length_x_;
			this->focal_length_y_ = other.focal_length_y_;
			this->hw_camera_id_ = other.hw_camera_id_;
			this->hw_image_id_ = other.hw_image_id_;
			this->hw_layout_id_ = other.hw_layout_id_;
			this->lines_segments_ = other.lines_segments_;
			this->lines_segments_3d_ = other.lines_segments_3d_;
			this->lines2d_idxs_to_grouped_lines2d_idxs_ = other.lines2d_idxs_to_grouped_lines2d_idxs_;
			this->grouped_lines_segments_ = other.grouped_lines_segments_;
			this->grouped_lines_segments_3d_ = other.grouped_lines_segments_3d_;
			this->lines_track_ids_ = other.lines_track_ids_;
			this->principal_point_[0] = other.principal_point_[0];
			this->principal_point_[1] = other.principal_point_[1];
			this->radial_distortion_[0] = other.radial_distortion_[0];
			this->radial_distortion_[1] = other.radial_distortion_[1];
			this->track_ids_ = other.track_ids_;
			this->view_pose_ = other.view_pose_;
			this->cam_optimized_valid_ = other.cam_optimized_valid_;
		}
	};

	/** The list of all viewports considered for bundling. */
	typedef std::vector<HWCamViewport> HWCamsViewportList;

	/* --------------- Data Structure for Polygons Intersetions Lines-------------- */
	struct HWPolygonInterLines
	{
		Eigen::Vector2i polygons_idxs_;
		Eigen::Vector3f ls3d_;
		Eigen::Vector3f le3d_;
		std::vector<int> adjacent_lines_idxs_;
		std::vector<int> cams_visibility_idx_;
	};

	/* --------------- Data Structure for Feature Tracks -------------- */
	/** References a 2D feature in a specific view. */
	struct HWFeatureReferenceId
	{
		HWFeatureReferenceId(int view_id, int feature_id) :
			view_id_(view_id), feature_id_(feature_id)
		{}

		int view_id_;
		int feature_id_;
	};

	/** The list of all feature references inside a track. */
	typedef std::vector<HWFeatureReferenceId> HWFeatureReferenceIdList;

	/** References a 2D feature in a specific view. */
	struct HWLineFeatureReferenceId
	{
		HWLineFeatureReferenceId(int view_id, int line_id) :
			view_id_(view_id), line_id_(line_id) {}

		int view_id_;
		int line_id_;
		int line_group_id = -1;
	};

	/** The list of all feature references inside a track. */
	typedef std::vector<HWLineFeatureReferenceId> HWLineFeatureReferenceIdList;

	/** Representation of a feature track. */
	struct HWTrack3D
	{
		bool is_valid(void) const
		{
			return (pos_[0] != std::numeric_limits<float>::quiet_NaN());
		}
		void invalidate(void)
		{
			pos_[0] = std::numeric_limits<float>::quiet_NaN();
			pos_[1] = std::numeric_limits<float>::quiet_NaN();
			pos_[2] = std::numeric_limits<float>::quiet_NaN();
		}
		void remove_view(int view_id)	//remove image id
		{
			//std::vector<Eigen::Vector3f> pnt3d_iter
			for (HWFeatureReferenceIdList::iterator iter = this->features_.begin();
				iter != this->features_.end();)
			{
				if (iter->view_id_ == view_id)
				{
					iter = this->features_.erase(iter);
				}
				else
				{
					iter++;
				}
			}
		}
		bool is_valid_ = true;
		Eigen::Vector3f pos_;
		Eigen::Vector3f color_;
		std::vector<int> polygon_idxs_;
		HWFeatureReferenceIdList features_;
		std::vector<int> pnts_views_to_polygon_idxs_;	//every view point has a polygon idx
		//same number to features(views_features_pnts3d_ is features_ projected pnts 3d )
		std::vector<Eigen::Vector3f> views_features_pnts3d_;	
	};

	/** The list of all tracks. */
	typedef std::vector<HWTrack3D> HWTrack3DList;

	/** Representation of a feature track. */
	struct HWLineTrack3D
	{
		bool is_valid(void) const
		{
			return (s_pos_[0] != std::numeric_limits<float>::quiet_NaN());
		}
		void invalidate(void)
		{
			s_pos_[0] = std::numeric_limits<float>::quiet_NaN();
			s_pos_[1] = std::numeric_limits<float>::quiet_NaN();
			s_pos_[2] = std::numeric_limits<float>::quiet_NaN();
			e_pos_[0] = std::numeric_limits<float>::quiet_NaN();
			e_pos_[1] = std::numeric_limits<float>::quiet_NaN();
			e_pos_[2] = std::numeric_limits<float>::quiet_NaN();
		}
		void remove_view(int view_id)
		{
			std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >::iterator myiter = line_views_pnts_.begin();
			for (HWLineFeatureReferenceIdList::iterator iter = this->lines_features_.begin();
				iter != this->lines_features_.end();)
			{
				if (iter->view_id_ == view_id)
				{
					iter = this->lines_features_.erase(iter);
					myiter = this->line_views_pnts_.erase(myiter);
				}
				else
				{
					iter++;
					myiter++;
				}
			}
		}
		bool is_valid_ = true;
		Eigen::Vector3f color_;
		Eigen::Vector3f s_pos_;	//line start pnt
		Eigen::Vector3f e_pos_;	//line end pnt
		std::vector<int> polygon_idxs_;
		Eigen::Vector2i polygon_intersect_line_idx_ = Eigen::Vector2i(-1,-1);	//point to 
		HWLineFeatureReferenceIdList lines_features_;	//
		//line track list (one view line 2d segment to one view line point),
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > line_views_pnts_;
		std::vector<int> line_views_to_polygon_idxs_;	//every view line has a polygon idx
		std::vector<int> view_line_polygon_inter_line_idxs_;
	};

	/** Representation of a feature track. */
	struct HWGroupedLineTrack3D
	{
		bool is_valid(void) const
		{
			return (s_pos_[0] != std::numeric_limits<float>::quiet_NaN());
		}
		void invalidate(void)
		{
			s_pos_[0] = std::numeric_limits<float>::quiet_NaN();
			s_pos_[1] = std::numeric_limits<float>::quiet_NaN();
			s_pos_[2] = std::numeric_limits<float>::quiet_NaN();
			e_pos_[0] = std::numeric_limits<float>::quiet_NaN();
			e_pos_[1] = std::numeric_limits<float>::quiet_NaN();
			e_pos_[2] = std::numeric_limits<float>::quiet_NaN();
		}
		void remove_view(int view_id)
		{
			std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >::iterator myiter = line_views_pnts_.begin();
			for (HWLineFeatureReferenceIdList::iterator iter = this->lines_features_.begin();
				iter != this->lines_features_.end();)
			{
				if (iter->view_id_ == view_id)
				{
					iter = this->lines_features_.erase(iter);
					myiter = this->line_views_pnts_.erase(myiter);
				}
				else
				{
					iter++;
					myiter++;
				}
			}
		}
		bool is_valid_ = true;
		Eigen::Vector3f color_;
		Eigen::Vector3f s_pos_;	//line start pnt
		Eigen::Vector3f e_pos_;	//line end pnt
		std::vector<int> polygon_idxs_;
		Eigen::Vector2i polygon_intersect_line_idx_ = Eigen::Vector2i(-1, -1);	//point to 
		HWLineFeatureReferenceIdList lines_features_;	//
		//line track list (one view line 2d segment to one view line point),
		std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > line_views_pnts_;
		std::vector<int> line_views_to_polygon_idxs_;	//every view line has a polygon idx
		std::vector<int> view_line_polygon_inter_line_idxs_;
		std::vector<int> id_to_hwline_track3d_idx_;	//idx to HWLineTrack3D
	};

	/** The list of all tracks. */
	typedef std::vector<HWTrack3D> HWTrack3DList;
	typedef std::vector<HWLineTrack3D> HWLineTrack3DList;
	typedef std::vector<HWGroupedLineTrack3D> HWGroupedLineTrack3DList;

	// potential match
	typedef std::pair<Eigen::Vector2f, Eigen::Vector2f> HWSCENELINEPIONT2DTYPE;
	typedef Eigen::Vector2i HWSCENELINEIDXTYPE;
	typedef struct HWSceneLine3DNode
	{
		Eigen::Vector2i adj_poly_idxs_;	//two polygon idx
		Eigen::Vector3f ls3d_;	//polygon intersection line start
		Eigen::Vector3f le3d_;	//polygon intersection line end
		std::vector<int> cams_visible_idxs_;	//
		std::vector<std::pair<int, HWSCENELINEPIONT2DTYPE> > sample_imgs_line_pnts_;	//img idx; line pnts(图片的坐标)
																						//polygon line对应的所看到的图像的线段并且保存它们的值, int2 first 是img idx, second是line idx
		std::vector<std::pair<HWSCENELINEIDXTYPE, float> > Imgidx2Imgev_;

	}HWSceneLine3DNode;

	/** pnts match between two pnts*/
	struct HWCorrespondence2D2DPnt
	{
		Eigen::Vector2f p1;
		Eigen::Vector2f p2;
	};

	/** pnts match between two pnts*/
	struct HWCorrespondence2D2DLines
	{
		std::pair<Eigen::Vector2f, Eigen::Vector2f> line1;
		std::pair<Eigen::Vector2f, Eigen::Vector2f> line2;
	};

	struct HWLineCorrespondLine2D
	{
		std::pair<std::string, std::string> base_path_pair;
		std::pair<int, int> camid_to_camid_;
		bool lines_valid_;
		std::pair<Eigen::Vector2f, Eigen::Vector2f> line1;
		std::pair<Eigen::Vector2f, Eigen::Vector2f> line2;
	};

	struct HWLineCorrespondPnt2D
	{
		std::pair<std::string, std::string> base_path_pair;
		std::pair<int, int> camid_to_camid_;
		bool points_valid_;
		Eigen::Vector2f pnt1;
		Eigen::Vector2f pnt2;
	};

	/** The matching result between two views. */
	struct HWPntsMatch
	{
		std::pair<int, int> pnts_matches_idx_;	//to original idx
		bool valid_match_;
		HWCorrespondence2D2DPnt corresponding_pnts_;
		Eigen::Vector3f view12_tri_pnt3d_;
		Eigen::Vector2i adj_poly_idxs_;

		Eigen::Vector2i left_polygon_idx_ = Eigen::Vector2i(-1, -1);
		Eigen::Vector2i right_polygon_idx_ = Eigen::Vector2i(-1, -1);
	};

	struct HWLinesPntsMatch
	{
		std::pair<int, int> line_matches_idx_;	//to original line idx
		bool valid_match_;
		//
		Eigen::Vector2f image_p1_;
		Eigen::Vector2f image_p2_;
		Eigen::Vector2f image_q1_;
		Eigen::Vector2f image_q2_;

		//world coordiante pnts from two views
		Eigen::Vector3f world_p1_;
		Eigen::Vector3f world_p2_;
		Eigen::Vector3f world_q1_;
		Eigen::Vector3f world_q2_;

		Eigen::Vector2i adj_poly_idxs_;
		Eigen::Vector2i p_polygon_idx_ = Eigen::Vector2i(-1, -1);
		Eigen::Vector2i q_polygon_idx_ = Eigen::Vector2i(-1, -1);

	};

	/* Observation of a survey point in a specific view. */
	struct HWSurveyObservationPnt2d
	{
		HWSurveyObservationPnt2d()
		{
			view_id_ = -1;
			is_valid_ = true;
		}
		HWSurveyObservationPnt2d(int view_id, float x, float y) :
			view_id_(view_id),
			pos_(Eigen::Vector2f(x, y))
		{
			is_valid_ = true;
		}

		int view_id_;	//equal to cam id
		Eigen::Vector2f pos_;
		Eigen::Vector3f view_pos_3d_;
		std::vector<int> plane_idxs_;
		bool is_valid_;
	};

	/* Observation of a survey line in a specific view. */
	struct HWLineSurveyObservationPnt2d
	{
		HWLineSurveyObservationPnt2d()
		{
			view_id_ = -1;
			is_valid_ = true;
			//plane_idxs_[0] = -1;
			//plane_idxs_[1] = -1;
		}
		HWLineSurveyObservationPnt2d(int view_id, Eigen::Vector2f& s, Eigen::Vector2f& t) :
			view_id_(view_id),
			pos_s_(s),
			pos_e_(t)
		{
			is_valid_ = true;
			//plane_idxs_[0] = -1;
			//plane_idxs_[1] = -1;
		}

		int view_id_;	//equal to cam id
		Eigen::Vector2f pos_s_;
		Eigen::Vector2f pos_e_;
		Eigen::Vector3f view_line_pos_s_3d_;
		Eigen::Vector3f view_line_pos_e_3d_;
		std::vector<int> plane_idxs_;
		bool is_valid_;
	};

	/** The list of all survey point observations inside a survey point. */
	typedef std::vector<HWSurveyObservationPnt2d> HWSurveyObservationPnt2dList;
	/** The list of all survey line observations inside a survey point. */
	typedef std::vector<HWLineSurveyObservationPnt2d> HWLineSurveyObservationPnt2dList;

	/** Representation of a survey point. 3D TO 2D data structure, by trianglating */
	struct HWSurveyPoint3d2Pnts2d
	{
		Eigen::Vector3f pos_;
		std::vector<int> plane_idxs_;
		HWSurveyObservationPnt2dList observations_;
	};

	/** Representation of a line segment. */
	struct HWLineSurveyPoint3d2Pnts2d
	{
		//math::Vec3f pos;
		Eigen::Vector3f pos_s_;
		Eigen::Vector3f pos_e_;
		std::vector<int> plane_idxs_;
		bool is_plane_lines_ = false;
		HWLineSurveyObservationPnt2dList observations_;
	};

	/** The list of all survey points. */
	typedef std::vector<HWSurveyPoint3d2Pnts2d> HWSurveyPoint3d2Pnts2dList;

	/** The list of all survey lines. */
	typedef std::vector<HWLineSurveyPoint3d2Pnts2d> HWLineSurveyPoint3d2Pnts2dList;

	struct HWTwoViewsCamsMatching
	{
		bool valid_pair = true;
		int view_1_id;	//cam id = view id
		int view_2_id;	//cam id = view id
		std::vector<HWPntsMatch> points_matches;	// point to point
		std::vector<HWLinesPntsMatch> lines_matches;	// lines to line

		std::vector<bool> lines_matches_interior;	//check line match if it is interior if true->interior

		bool operator< (HWTwoViewsCamsMatching const& rhs) const
		{
			return this->view_1_id == rhs.view_1_id
				? this->view_2_id < rhs.view_2_id
				: this->view_1_id < rhs.view_1_id;
		}
		HWTwoViewsCamsMatching operator = (const HWTwoViewsCamsMatching& rhs)
		{
			//HWTwoViewsCamsMatching lhs;
			this->view_1_id = rhs.view_1_id;
			this->view_2_id = rhs.view_2_id;
			this->valid_pair = rhs.valid_pair;
			this->lines_matches = rhs.lines_matches;
			this->lines_matches_interior = rhs.lines_matches_interior;
			this->points_matches = rhs.points_matches;
			return *this;
		}

	};

	/** The matching result between several pairs of views. */
	typedef std::vector<HWTwoViewsCamsMatching> HWPairwisesMatchingView;

	bool IsHWPoint3dValidZDG(const Eigen::Vector3f& v_p3d);

	bool IsHWLineSegmentValidZDG(const std::pair<Eigen::Vector3f, Eigen::Vector3f>& lineseg);

	bool FittingLine3dFromPnts3d3f(const std::vector<Eigen::Vector3f>& pnts, Eigen::Vector3f& ls, Eigen::Vector3f& le);

	bool FittingLine3dFromPnts3d3d(const std::vector<Eigen::Vector3d>& pnts, Eigen::Vector3d& pnt, Eigen::Vector3d& ld);
	
	/*
	img_pnt(img->u, img->v, img->depth)
	*/
	Eigen::Vector3d GetWPntFromImgPntDByRKMatrix(const Eigen::Matrix3d& K, const Eigen::Matrix4d& RT, const Eigen::Vector3d& img_pnt);

	float ComputePnt3dToLineSegment3DDistF(const Eigen::Vector3f& ls, const Eigen::Vector3f& le, const Eigen::Vector3f& pnt);

	void ComputePntProj2Line3DF(const Eigen::Vector3f& lpnt, const Eigen::Vector3f& ldir, const Eigen::Vector3f& pnt, Eigen::Vector3f& proj);

	double ComputePnt3DToLine3DDistF(const Eigen::Vector3f& lpnt, const Eigen::Vector3f& ldir, const Eigen::Vector3f& pnt);

	Eigen::Matrix3d GetPluckerKMatrix(const Eigen::Matrix3d& km);

	Eigen::Matrix<double, 6, 6> GetPluckerTransformMatrix(const Eigen::Matrix4d& T);

	Eigen::Matrix<double, 6, 1> OrthLineToPluckerLine(const Eigen::Vector4d& orthline);
	
	Eigen::Matrix<double, 6, 1> OrthLineToPluckerLineNew(const Eigen::Vector4d& orthline);

	Eigen::Vector4d PluckerLineToOrthLine(Eigen::Matrix<double, 6, 1> pluckerline);

	Eigen::Matrix3d PluckerLineToOrthLineR(Eigen::Matrix<double, 6, 1> pluckerline);

	Eigen::Matrix2d PluckerLineToOrthLineW(Eigen::Matrix<double, 6, 1> pluckerline);

	Eigen::Matrix<double, 6, 4> ComputeJacobianFromPlukerToOrth(const Eigen::Matrix3d& u, const Eigen::Matrix2d& w);

	void UpdateOrthCoordDFromDeltaD(Eigen::Vector4d& D, Eigen::Vector4d& deltaD, Eigen::Vector4d& plusD);

	//normalize the scale
	Eigen::Matrix<double, 6, 1> PluckerLineNormalized(Eigen::Matrix<double, 6, 1> pluckerline);

	//d (line dir) normalize the scale
	Eigen::Matrix<double, 6, 1> PluckerLineDirNormalized(Eigen::Matrix<double, 6, 1> pluckerline);

	Eigen::Matrix2d GetOrthWFromPlucker(const Eigen::Matrix<double, 1, 6>& pluckerline);

	Eigen::Matrix3d GetOrthRMatrixFromPlucker(const Eigen::Matrix<double, 6, 1>& pluckerline);

	Eigen::Vector3d GetLineFuncFromLine2Pnts2d(const Eigen::Vector2d& ls, const Eigen::Vector2d& le);

	void FileLineBufferProcess(std::string& line, const std::string& comment_str = "#");

	Eigen::Matrix3d VectorColMultiplyVectorRow3D(const Eigen::Vector3d& colv, const Eigen::Vector3d& rowv);

	Eigen::Matrix<double, 6, 1> GetPluckerLineFromLineEndPnts(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

	Eigen::Vector3d VectorDotMultiplyMatrix3d(const Eigen::Vector3d& v, const Eigen::Matrix3d& m);

	//std::string GetBaseNameWithoutSuffix(const std::string& path);
	bool GetPairFromVectorPairsByPairFirst(const std::vector<std::pair<int, int> >& pairs_vec, int value_first, std::pair<int, int>& my_pair);
	bool GetPairFromVectorPairsByPairSecond(const std::vector<std::pair<int, int> >& pairs_vec, int value_second, std::pair<int, int>& my_pair);
}

#endif