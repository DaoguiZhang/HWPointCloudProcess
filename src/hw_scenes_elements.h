#ifndef HW_SCENES_ELEMENTS_H
#define HW_SCENES_ELEMENTS_H

#include "hw_algorithms.h"
//#include "base/reconstruction.h"
#include "hw_scenes_cams.h"
#include "hw_scene_layouts.h"	//这个和polygon一致，后续改一下

//set hw_scene_polygons to select lines
//#include "hw_scene_polygons.h"
#include"hw_polygon.h"
#include "scene_elements.h"
#include"hw_scene_proj_layout3d.h"
#include"hw_view_cmns.h"
#include"cnpy.h"
#include"hw_image_labels.h"

//#include<colmap/util/option_manager.h>

/*
cams params; images; layout2d; layout3d; all store here 
*/

namespace HW
{
	struct PolygonIterLineOption
	{
		float line_dist_threshold_ = 0.3;	//m
		float line_angle_threhold_ = 10;	//degree
	};

	const float MATCHES_LINE_DIST_THRESHOLD_ZDG = 25.0f;

    class HWScenesElements
    {
    public:

		typedef struct PolygonInterLineScene
		{
			Eigen::Vector2i adj_poly_idxs_;
			Eigen::Vector3f ls3d_;
			Eigen::Vector3f le3d_;
		} PolygonInterLineScene;

		//to use the visiable 
		typedef struct View2PolygonInterLinesData
		{
			int image_idx_;	//equal to idx or cam idx
			std::vector<int> view_visible_lines_idxs_;
			std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > view_visiable_lines_pnts_;
		} View2PolygonInterLinesData;

		typedef struct PairSetValue
		{
			std::pair<int, int> value_;
			bool operator == (const PairSetValue& other) const
			{
				if (this->value_ == other.value_)
					return true;
				if (this->value_.first == other.value_.second
					&& this->value_.second == other.value_.first)
					return true;
				return false;
			}
		}PairSetValue;

        HWScenesElements();
        ~HWScenesElements();
        void SetDir(std::string& dir);
        void LoadAllElements(); //load all data from dir
		void ImportPairsLinesFromTxt(const std::string& path);
		void ImportLinesPntsPairsFromPythonDir(const std::string& dir);

		void SetCamerasModels(ModelCameras* models);
		void SetLayoutsModels(SceneElements* lay_models);
		void SetHWPolygonsModels(std::vector<HWPlane*> hw_polygons);
		void SetHWImagesLabels(HWImageLabels* polygons_images_labels);
		void SetPolygonsIntersectLines(const std::vector<HWPolygonInterLines>& polygons_lines);
		void SetRmaxThreshold(double rmax);
		void SetImportDataFromSceneElementsSate(bool data_state);
		void SetImportDataFromPairsLinesTxtState(bool pair_txt_state);
		void SetImportDataFromPythonNumpyState(bool nmpy_state);
		void SetRMaxPolygonThresholdRayCast(double r_polygon_max);
		const std::vector<CameraModel>& GetCamerasModels();
		const std::vector<HW::HWImage>& GetImagesModels();
		void UpdateImageModelsLoadImgs();	//load all images from image paths
		HWSceneLayouts* GetLayoutsModels();
		const std::map<unsigned int, std::set<unsigned int> > GetImagesLinesMatchesCamsIds();
		void GetHWPolygonModels(std::vector<HWPlane*>& hw_polygons);
		HWSurveyPoint3d2Pnts2dList* GetSurveyPnts2PntsList();
		HWLineSurveyPoint3d2Pnts2dList* GetSurveyLines3D2Pnts2dList();
		HWCamsViewportList* GetViewsPortList();
		const HWPairwisesMatchingView& GetImagesPairsMatchesViews();
		std::vector<std::pair<std::string, std::string> > GetCamsPairsIdsNamesFromScenesLinesPntsRefined();
		const std::vector<HWLineCorrespondLine2D>& GetImportedCorrespondLines2D();
		const std::vector<HWLineCorrespondPnt2D>& GetImportedCorrespondPnts2D();
		bool GetImportDataFromSceneElementsSate();
		bool GetImportDataFromPairsLinesTxtState();
		bool GetImportDataFromPythonNumpyState();

		void BackUpHWCamViewportsList();
		void HWUpdateId();  //update the cams id to layout id
        void HWUpdate();    //update the cams idx to layouts idx
		void HWUpdateAllIds();	//update cams id, image id and layout id
		void HWUpdateCamsLyoutsId();	//update layout id to cams and inverse
		void HWUpdateImagesLyoutsId();	//update layout id to images and inverse
        bool IsUpdate();

        void HWUpdateCamsGroup();
		void HWUpdateScenceLayoutsFilterSameLine();
		void HWUpdateScenceLayoutsNearLines();
		void HWUpdateRemoveSceneLayoutsNearLines();
		void HWUpdateRemoveSceneLayoutShortLines();
        //void HWUpdateMatchIdxFromCamsGroup();

        //------------------matches lines--------------//
		void SetDataFromPolygonsIntersectionLines(const std::vector<HWSceneLine3DNode>& lines_node);

		void SetPolygonInterLinesFromHWPolygonsLines(const std::vector<PolygonInterLineScene>& polygons_lines);

		void SetView2PolygonLineDataFromView2Lines(const std::vector<View2PolygonInterLinesData>& lines_data);

		int FindLine3dIdxFromImageIdLineId(int image_id, int lid);

		int FindLine3dIdxFromImageIdLineIdPair(int src_img_id, int src_lid, int tgt_img_id, int tgt_lid);

        void UpdateCamsFundmentalMatrixs();

		void UpdateLinesMatchesFromCamsGroupsLSD();
        void UpdateLinesMatchesForCamsImgs();

		void UpdateLinesMatchesFromCamsAndPolygons();

        double MutualOverlap(const std::vector<Eigen::Vector3f>& collinear_pnts);

        //src id, tgt id (cam id), F is two Fundmantal matrix
        void MatchLinesFromTwoImgs(unsigned int src_id, 
            unsigned int tgt_id, Eigen::Matrix3f& F);

		void MatchLinesFromTwoCamidsLSD(unsigned int src_id,
			unsigned int tgt_id, Eigen::Matrix3f& F);

		void RunLayout2dProjectPolygon();

        Eigen::Vector2f TrainglulationDepthFromTwoViews(const unsigned int src_camID, const Eigen::Vector3f& p1,
                                                const Eigen::Vector3f& p2, const unsigned int tgt_camID,
                                                const Eigen::Vector3f& line_q1, const Eigen::Vector3f& line_q2); 
        
		//world coordinate pnt from two view lines
		bool TrianglulationWorldPntsFromTwoViewsLines(const unsigned int src_camID, const Eigen::Vector3f& p1,
			const Eigen::Vector3f& p2, const unsigned int tgt_camID, const Eigen::Vector3f& line_q1, 
			const Eigen::Vector3f& line_q2, std::pair<Eigen::Vector3f, Eigen::Vector3f>& p1p2_world_pnts);

        //test matches for images(src id, tgt_id are cams ids)
        void DrawTwoImgsLinesMathed(unsigned int src_id, unsigned int tgt_id);
		void DrawTwoImgLinesMatchedIntoMat(unsigned int src_id, unsigned int tgt_id, cv::Mat& fusion_mat);
		//src id, tgt_id are cams ids; src_id->pair_pnts.first; tgt_id->pair_pnts.second 
		void DrawOnePairPntsMatchedIntoMat(unsigned int src_id, unsigned int tgt_id, 
			const std::pair<Eigen::Vector2f, Eigen::Vector2f>& pair_pnts,  cv::Mat& fusion_mat);
		void DrawTwoImgsPntsMatchedIntoMatFromPairMatchingViewOrigin(int idx, cv::Mat& fusion_mat);
		void DrawTwoImgsLinesMatchedIntoMatFromPairMatchingViewOrigin(int idx, cv::Mat& fusion_mat);
		void DrawTwoImgsPntsAndLinesMatchedIntoMatFromPairMatchingViewOrigin(int idx);

		int GetPairwisesMatchingViewsImagesPairsNum();
		void DrawTwoImgsPntsMatchedIntoMatFromPairMatchingView(int idx, cv::Mat& fusion_mat);
		void DrawTwoImgsLinesMatchedIntoMatFromPairMatchingView(int idx, cv::Mat& fusion_mat);
		void DrawTwoImgsPntsAndLinesMatchedIntoMatFromPairMatchingView(int idx);
		int GetViewIdxFromHWCamsViewportListByCamId(int cam_id);
		void DrawNetworkLinesFromHWImageIntoMat(int camid, cv::Mat& line_mat);
		void DrawNetworkGroupedLinesFromHWImageIntoMat(int camid, cv::Mat& line_mat);
		void DrawNetworkGroupedLinesFromHWViewIntoMat(int camid, cv::Mat& line_mat);

		//if image labels is loaded, draw it
		void DrawTwoImgsPntsMatchedIntoMatFromPairMatchingViewAndImageLabels(int idx, cv::Mat& fusion_mat);
		void DrawTwoImgsLinesMatchedIntoMatFromPairMatchingViewAndImageLabels(int idx, cv::Mat& fusion_mat);

		void DrawTwoImgsPntsMatchedIntoMatFromPairMatchingViewAndImageLabelsAssigned(int idx, cv::Mat& fusion_mat);
		void DrawTwoImgsLinesMatchedIntoMatFromPairMatchingViewAndImageLabelsAssigned(int idx, cv::Mat& fusion_mat);

		//Get pair Image (images and images pnts) from scenes_lines_pnts_matches_'s idx
		bool GetTwoImageAndPairPntsIdxFromPairMatchingView(int idx, cv::Mat& src_img, std::vector<Eigen::Vector2f>& src_pnts,
			cv::Mat& tgt_img, std::vector<Eigen::Vector2f>& tgt_pnts);

		bool GetTwoImageAndPairLinesIdxFromPairMatchingView(int idx, cv::Mat& src_img, std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& src_lines,
			cv::Mat& tgt_img, std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& tgt_lines);

		bool GetTwoImagesPairPntsFromCamsViewportByPairMatchingViewIdx(int idx, 
			std::vector<Eigen::Vector2f>& src_pnts, std::vector<Eigen::Vector2f>& tgt_pnts);
		
		bool GetTwoImagesPairLinesFromCamsViewportByPairMatchingViewIdx(int idx, std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& src_lines,
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& tgt_lines);

		bool GetTwoPairImagesFromCamsViewportByPairMatchingViewIdx(int idx, cv::Mat& src_img, cv::Mat& tgt_img);

		bool GetTwoPairLabelsImagesFromCamsViewportByPairMatchingViewIdx(int idx, cv::Mat& src_label_img, cv::Mat& tgt_label_img);

		void GetTwoImgsLinesMatched(unsigned int src_id, unsigned int tgt_id, 
			std::vector<HWCorrespondence2D2DLines>& lines_pairs);

		//no check on overlap, important
		void GetImagesLinesPairFromImagesPair(unsigned int src_id, unsigned int tgt_id,
			std::vector<HWCorrespondence2D2DLines>& lines_pairs);

		Eigen::Vector2i GetViewLineProjectToImageLineIdx(const HWLineFeatureReferenceId& view_line_pair);
		//polygon_inter_line_idx: first the polygon intersection line idx,  view: view_line_pair
		std::pair<Eigen::Vector2i, LineDistMeasure> ComputeViewVisiableLine3dToImageLineDist(const HWLineFeatureReferenceId& view_line_pair);
		LineDistMeasure ComputeLine3dToImageViewLineDist(const std::pair<Eigen::Vector3f, Eigen::Vector3f> line_3d,
			const HWLineFeatureReferenceId& view_line_pair);
		LineDistMeasure ComputeViewProjLineToImageLineSegDist(const std::pair<Eigen::Vector2f, Eigen::Vector2f> line_seg0, 
			const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_seg1);
		//line dist not line segment distance(important)
		LineDistMeasure ComputeImageLineToImageLineDist(const std::pair<Eigen::Vector2f, Eigen::Vector2f> line_seg0,
			const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_seg1);

        //------------------end match lines------------//

        void SaveLayoutsintoImages();
        //to change it because of its slow speed

		void SaveAllLayouts2dIntoDir(const std::string& outdir);

        //project image pnt to polygons then check it to 
        void ImageLayoutRay2ScenePolygonPnt3d(unsigned int image_id, 
            std::vector<std::pair<bool, bool>>& vec_valid, 
            std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& pnts3d);
        
        void ImagePickedLineRay2ScenePolygonPnt3d(unsigned int image_id, unsigned int lid, 
            std::pair<bool, bool>& ps_valid, std::pair<Eigen::Vector3f, Eigen::Vector3f>& pnts3d);

		//input polygon exisiting....
		bool ImagePnt2dProj2ScenePolygonPnt3d(CameraModel cam, 
			const Eigen::Vector2f& pnt2d, Eigen::Vector3f& pnt3d);

		bool ImagePnt2dPosProj2ScenePolygonsPnt3dPos(const CameraModel& cam,
			const Eigen::Vector2f& pnt2d, Eigen::Vector3f& pnt3d);
		//image pnt project all polygons. if intersect, return polygon idx or return -1
		int ImagePnt2dPosProj2ScenePolygonsPnt3dPosIdx(const CameraModel& cam,
			const Eigen::Vector2f& pnt2d, Eigen::Vector3f& pnt3d);
		//image pnt project picked polygon. if intersect, compute the intersect point. and return true
		bool ImagePnt2dPosProj2ScenePickedPolygonPnt3dPos(const CameraModel& cam,
			const Eigen::Vector2f& pnt2d, int polygon_idx, Eigen::Vector3f& pnt3d);
		//image pnt project picked polygon(plane). if intersect, compute the intersect point. and return true
		bool ImagePnt2dPosProj2ScenePickedPolygonPlanePnt3dPos(const CameraModel& cam,
			const Eigen::Vector2f& pnt2d, int polygon_idx, Eigen::Vector3f& pnt3d);

        //input polygon exisiting....
        bool LinePntRay2Polygons(const Eigen::Vector3f& raypnt, 
            const Eigen::Vector3f& raydir, float threshold, int& polyid, Eigen::Vector3f& pnt3d);
		//input polygons exisited (Line:(raypnt, raydir); threshold(point3d dist to polygon); polyid(selected idx))
		bool LinePntRay2PickedPolygon(const Eigen::Vector3f& raypnt,
			const Eigen::Vector3f& raydir, float threshold, int polyid, Eigen::Vector3f& pnt3d);
		bool LinePntRay2PickedPolygonPlane(const Eigen::Vector3f& raypnt,
			const Eigen::Vector3f& raydir, int polyid, Eigen::Vector3f& pnt3d);

        void FilterErrorLineMatchesByPolygons();
		void FilterErrorLineMatchesWithPolygonsByReprojection();
		void FilterErrorLinesMatchesByPolygonsNetNumpy();
		void FilterErrorPointsMatchesByPolygonsNetNumpy();

		//convert all network data initial views
		void RunConvertNumpyPntsAndLinesIntoViews();

		//assign all render plane labels to lines 2d (all lines 2d store into hw_cams_views_list_)
		void AssignAllRenderPlanesLabelsToHWCamsViewsListLines2d();	//delete
		//assign all render plane labels to pairs lines 2d (all pair lines 2d store into scenes_lines_pnts_matches_)
		//scenes_lines_pnts_matches_ p_polygon_idx_ q_polygon_idx_
		void AssignAllRenderPlanesLabelsToHWPairwisesLines2d();
		void AssignAllRenderPlanesLabelsToHWPairwisesPnts2d();

		//assign plane labels to signle line
		void AssignRenderPlanesLabelsToSingleLine2d(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_sg, 
			const cv::Mat& image_labes, const cv::Mat& image_intersection_labels, Eigen::Vector2i& label_assigned);
		//assign plane labels to signle point
		void AssignRenderPlanesLabelsToSinglePnt2d(const Eigen::Vector2f& single_point,
			const cv::Mat& image_labes, const cv::Mat& image_intersection_labels, Eigen::Vector2i& label_assigned);

		void SampleNLinePntsFromLineSeg(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_seg, 
			int sample_num, std::vector<Eigen::Vector2i>& line_pnts);
		//sample N's points from line segment
		void SampleNLineFloatPntsFromLineSeg(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_seg,
			int sample_num, std::vector<Eigen::Vector2f>& line_pnts);
		void ComputeImagesPntsRayToPolygonsIdxs(std::vector<Eigen::Vector2f>& image_pnts, 
			const CameraModel& cam_model, std::vector<std::pair<int, int> >& polygons_idxs_counts);
		//get first two polygon idxs
		void ExtractPolygonIdxFromPolygonsIdxsCounts(const std::vector<std::pair<int, int> >& polygons_idxs_counts, 
			Eigen::Vector2i& polygon_idxs);

		int ExtractPlaneLabelFromLabelsVec(const std::vector<unsigned short int>& line_pnts_labels);

		//convert matched_ and matches_ into scenes_lines_pnts_matches_
		void RunLineMatchesIntoViews();
		void RunLineMatchesIntoViewsFilterWithLinesMatchNum(int line_match_threshold);

		//scenes_lines_pnts_matches_ (src_view_id->src_cam_id, tgt_view_id->tgt_cam_id)
		int FindViewPairCamsIdsFromViewsLinePntsMatches(int src_view_id, int tgt_view_id);

		void RunAllImgsSiftDetector();

		void RunAllLinesSegmentsDetector();

		void RunConvertAllImagesLinesSegmentsToLayouts();

		void RunAllLayoutsSegmentsCombinations();

		void RunImgsPntsMatchesFromLineImgsMatch();

		void RunImgsPntsMatchesFromLineImgsMatchNew();

		/*
		find the DMatch from DMatch vec
		*/
		int FindCVDMatchIdx(const std::vector<cv::DMatch>& matchs_vec, const cv::DMatch& match_v);

		/*
		Generate track line 3d points from Images lines match 
		line 3d to multi views lines(data from scenes_lines_pnts_matches_ to scenes_line3d_track_list_)
		*/
		void RunTrackLines3dListFromImgsLinesMatch();

		/*
		Generate track line 3d points from Images lines match
		line 3d to multi views lines(data from scenes_lines_pnts_matches_ to scenes_line3d_track_list_)
		*/
		void RunTrackLines3dListFromImgsLinesMatchFilterByGroupLines();

		/*
		filter the grouped track lines that is not in the polygons, set them false 
		*/
		void FilterGroupedTrackLines3dListNotInPolygons();

		/*
		Generate track line 3d points from Images lines match(filter some wrong match)
		line 3d to multi views lines(data from scenes_lines_pnts_matches_ to scenes_line3d_track_list_)
		*/
		void RunTrackLines3dListFromImgsLinesMatchFilter();

		/*
		Generate track line 3d points from Images lines match and images labels
		line 3d to multi views lines(data from scenes_lines_pnts_matches_ to scenes_line3d_track_list_)
		*/
		void RunTrackLines3dListFromImgsLinesMatchWithLabels();

		/*
		Generate track 3d points from Images points match
		pnts 3d to multi views lines(data from scenes_lines_pnts_matches_ to scenes_pnts3d_track_list_)
		*/
		void RunTrackPnts3dListFromImgsFeaturesMatch();

		/*
		Generate track 3d points from Images points match
		pnts 3d to multi views lines(data from scenes_lines_pnts_matches_ to scenes_pnts3d_track_list_)
		*/
		void RunTrackPnts3dListFromImgsFeaturesMatchFilter();

		/*
		filter the grouped track lines that is not in the polygons, set them false
		*/
		void FilterGroupedTrackPnts3dListNotInPolygons();

		/*
		filter wrong track 3d points if tracklist has the duplicated view id 
		Tracklist points 3d to multi views points(scenes_pnts3d_track_list_)
		*/
		void FilterDuplicatedTrackPnts3dListFromDuplicatedViewsId();

		/*
		Generate track 3d points from Images points match and images labels
		pnts 3d to multi views lines(data from scenes_lines_pnts_matches_ to scenes_pnts3d_track_list_)
		*/
		void RunTrackPnts3dListFromImgsFeaturesMatchWithLabels();

		/*
		Generate 3D Pnts and Pnts 3d to pnts 2d from track list
		one pnt 3d to pnts 2d(point 3d to multiviews points 2d)
		*/
		void RunTrackPnts3dToPnts2Pnts3dObservations();
		void RunTrackPnts3dToPnts2Pnts3dObservationsNoPolygonInterLine();

		/*
		filter wrong track 3D Lines if tracklist has the duplicated view id
		Tracklist lines 3d to multi views lines(scenes_pnts3d_track_list_)
		*/
		void FilterDuplicatedTrackLines3dListFromDuplicatedViewsId();

		/*
		group track 3D Lines if two tracklist has the same group idx view id,
		Tracklist lines 3d to multi views lines(scenes_pnts3d_track_list_)
		*/
		void GroupTrackLines3dListBySameGroupsIds();
		
		/*find the track line if in grouped track lines 3d(scenes_line3d_grouped_track_list_)
		if find it return idx from scenes_line3d_grouped_track_list_ or return false
		*/
		int CheckMyTrackLine3dInGroupedTrackLine3d(const HWLineTrack3D& tmp_track_line);

		bool IsHWLine3dInHWGroupedLine3d(const HWLineTrack3D& tmp_track_line, const HWGroupedLineTrack3D& grouped_track_line);

		bool IsTwoViewLineRefereneIdsInSameViewGroupedLine(const HWLineFeatureReferenceId& hw_line_ref0, 
			const HWLineFeatureReferenceId& hw_line_ref1);

		void ConvertAllHWLine3dToHWGroupedLinePart(const HWLineTrack3D& tmp_track_line, HWGroupedLineTrack3D& grouped_track_line);

		void AddAllHWLine3dToHWGroupedLinePart(const HWLineTrack3D& tmp_track_line, HWGroupedLineTrack3D& grouped_track_line);

		/*
		seprate track 3D Lines if two tracklist has the different polygon ids,
		Tracklist lines 3d to multi views lines(scenes_pnts3d_track_list_)
		*/
		void SeprateTrackLines3dListByDifferentPolygonsIds();

		/*
		recompute initial lines 3d pnts because of track lines 3d(scenes_lines3d_track_list_)
		*/
		void ReComputeSceneLinesGroupedTracks3dList3dLinePosByPolygonsIds();

		/*
		recompute initial Pnts 3d pnts because of track Pnts 3d(scenes_pnts3d_track_list_)
		*/
		void ReComputeScenePntsGroupedTracks3dList3dPntsPosByPolygonsIds();

		/*
		Generate 3D Lines and Lines 3d to Lines 2d from track list
		pnt line 3d to lines 2d(lines 3d to multiviews lines 2d)
		*/
		void RunTrackLinesPnts3dToLinesPnts2LinesPnts3dObservations();
		void RunTrackLinesPnts3dToLinesPnts2LinesPnts3dObservationsNew();	//to do next...
		void RunTrackLinesPnts3dToLinesPnts2LinesPnts3dObservationsNoPolygonInterLine();	//to do next...
		void RunGroupedTrackLinesPnts3dToLinesPnts2LinesPnts3dObservationsNoPolygonInterLine();
		void RunTrackLinesPnts3dToLinesPnts2LinesPnts3dObservationsPolygonInterLine();	//to do next...

		/*
		Fit 3D Lines from track list 
		pnt line 3d to lines 2d(lines 3d to multiviews lines 2d)
		*/
		void RunFittingLinesFromTrackLinePnt3dlist();

		/*
		Fit 3D Lines from track list views lines
		//pnt line 3d to lines 2d(lines 3d to multiviews lines 2d)
		*/
		void RunFittingLines3DFromTrackLinePnt3dlistFilter();

		/*
		Fit 3D Lines from track list views lines
		//pnt line 3d to lines 2d(lines 3d to multiviews lines 2d)
		*/
		void RunFittingLines3DFromTrackGroupedLinePnt3dlistFilter();

		/*
		set all the view line same polygon id from track list
		*/
		void SetAllTheViewTrackLine3dSamePolygonIds();

		/*
		Fit 3D Lines by cam near polygon from track list
		pnt line 3d to lines 2d(lines 3d to multiviews lines 2d)
		*/
		void RunFittingLinesByPolygonNearCamsFromTrackLinePnt3dlist();

		/*
		Fit 3D Lines by cam near polygon from track list
		pnt line 3d to lines 2d(lines 3d to multiviews lines 2d)
		*/
		void RunFittingLinesByPolygonNearCorrectedCamsFromTrackLinePnt3dlist();

		/*
		get adjacent lines for every
		*/
		void RunPolygonsInterLinesAdjacent();

		/*
		match track lines to polygons intersection lines
		important... test the matches 
		*/
		void RunTrackLinesMatchPolygonsInterLines();

		/*
		filter the error match lines.
		the match line: match track lines to polygons intersection lines
		important... test the matches
		*/
		void FilterErrorTrackLinesMatchInterLines();

		/*
		filter the error match lines
		the match line: match track lines to image lines
		important... test the matches
		*/
		void FilterErrorTrackLines2RedauntImagesLines();

		/*
		filter the invalid points tracks 3d and invalid view points assigned to the points tracks 3d
		important... test the tracks
		*/
		void FilterInvalidTrackPointsAndViewsPointsObservation();

		/*
		filter the invalid lines tracks 3d and invalid view lines assigned to the lines tracks 3d
		important... test the tracks
		*/
		void FilterInvalidTrackLinesAndViewsLinesObservation();

		/*
		update the pnts tracks with lines tracks
		*/
		void UpdatePntsTracksByLinesTracks();

		/*
		save cameras params into cams
		*/
		void SaveOptiCamsIntoCamFiles();

		/*
		save cameras params into dir
		*/
		void SaveOptiCamsIntoDir(const std::string& dir);

		/*
		save the detected image lines into dir
		*/
		void SaveImagesLinesIntoTxtsFromDir(const std::string& dir);

#if 0
		void RunImgsPntsMatchesFromTwoImgsMatch(unsigned int src_cid, unsigned int tgt_cid);
#endif

        bool SetMatchesLinesValid(unsigned int src_camid, unsigned int srclid, 
            unsigned int tgt_camid, unsigned int tgtlid, bool my_valid);

        //debug
        void WriteLine3DIntoObj(const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> >& lines, 
            const std::string& path);
		void WritePntsIntoObj(const std::vector<Eigen::Vector3f>& pnts, const std::string& path);

		const HWScenesCams* GetHWScenesCamsImgs();
		const HWSceneLayouts* GetHWLayouts();

		void ConvertNumpyPntsToPnts(const cnpy::NpyArray& ny_pnts, 
			std::vector<Eigen::Vector2f>& pnts);
		void ConvertNumpyPntsIdxToPntsIdx(const cnpy::NpyArray& ny_idxs,
			std::vector<Eigen::Vector2i>& pnts_idxs);

		void ConvertNumpyLinesPntsToLinesPnts(const cnpy::NpyArray& ny_lines,
			std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& lines);
		void ConvertNumpyLinesIdxToLinesIdx(const cnpy::NpyArray& ny_idxs,
			std::vector<Eigen::Vector2i>& lines_idxs);
		
		void PrintPyNpyShape(const cnpy::NpyArray& ny_data);
		void PrintHWSecenesElementCamsInfo();
		void PrintHWSecnesElementLayoutsInfo();
		void PrintHWScenesPolygonsInfo();
		void WriteHWPolygonInterLineToLine3d(const std::string& path);
		//void DrawImageLayoutFromCamId();
		//void DrawImagesLinesMatches(int lid);
		bool GetLayoutPickedLineFromLayidLineId(int layid, int lid, Eigen::Vector3f& ls, Eigen::Vector3f& le);
		void DrawLayoutPickedLineIntoImage(const std::string& path, int layid, int lid);

		bool CheckLineSeg3d2LineSeg3dInSameLine(const std::pair<Eigen::Vector3f, Eigen::Vector3f>& lseg1,
			const std::pair<Eigen::Vector3f, Eigen::Vector3f>& lseg2, float angle_threshold, float dist_threshold);

		bool CheckLineSeg2d2LineSeg2dWithThreshold(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& lseg1,
			const std::pair<Eigen::Vector2f, Eigen::Vector2f>& lseg2, float angle_threshold, float dist_threshold);

		bool CheckTwoPnt3dTheshold(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, float dist_threshold);

		bool CheckTwoPnt2dTheshold(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, float dist_threshold);
		
		/*
		(initial data convert)convert scenes_cams_ and scenes_layouts_elements_ into hw_cams_views_list_
		*/
		void InitialConvertCamsAndLayoutsIntoViews();

		/*
		network data (lsd, superpoints and so on)
		scenes_layouts_elements_ update the lines. group them into long lines
		*/
		void UpdateGroupViewsLines2dIntoLines2dBasedOnLinesDir();

		/*
		network data (lsd, superpoints and so on)
		(initial data convert)convert scenes_cams_ and scenes_layouts_elements_ into hw_cams_views_list_
		*/
		void InitialConvertCamsAndLayoutsAndPntsIntoViewsNetWork();

		/*
		convert the scene_line3d_node_ data to matched_,matched_fundamentals_,matches_, 
		num_matches_
		*/
		void ReloadImagesLinesPairsFromImportedData();

		void ReloadImagesPntsParisAndLinesPairsFromImportedData();

		void UpdateImportDataToMatched();

		void UpdateImportedNetLinesToMatched();	//images lines
		void UpdateImportedNetPntsToHWImages();	//copy data to hw images
		void UpdateImportedNetPntsToHWImagesIds(); //
		void UpdateImportedNetLinesToHWImageIds();

		void FilterMatchedImagesPairByLinesMatchNum(int line_pair_num);

		void WritePickeLayoutIntoPickedImage(int image_idx, int lyidx, const std::string& path);
		void WriteNetworkPntsIntoPickedImage(int imageid, const std::string& path);
		void GenerateImageIdLyidFromCamsid(int camid, int& imgid, int& lyid);
		const std::string& GetImagePathFromImageid(int imageid);

		cv::Mat MixImageFromTwoImages(const cv::Mat& img0, const cv::Mat& img1, float fweight);

    private:

        //bool Ray2Plane3D(const Eigen::Vector3d& o, const Eigen::Vector3d& dir, 
        //const Eigen::Vector4d fn, Eigen::Vector3d& pnt3d);
		std::vector<std::string> GetAllBaseNamesFromPaths(const std::vector<std::string>& paths);
		std::vector<std::string> GetAllBaseNameDropLastPattern(const std::vector<std::string>& paths);
		Eigen::Matrix3f GetFundamentalMatrix(CameraModel& src, CameraModel& tgt);
        std::size_t GetIdxFromPairVecFirst(unsigned int idx, const std::vector<std::pair<int, CameraModel>>& vec);
        
		int GetImageLabelIdxFromImagesLabelPthByHWImagePath(const std::vector<std::string>& images_labels_paths, const std::string& image_path);
		std::vector<HWMatch> GetHWMatchFromMatches(unsigned int src_cid, unsigned int tgt_cid);
		std::pair<Eigen::Vector2f, Eigen::Vector2f> GetLayoutLineFromCamIdAndLid(unsigned int src_cid, unsigned int lid);

		/*
		*Merges lines and pnts tracks and updates viewports accordingly.
		*/
		void unify_new_lines_tracks(int view1_line_tid, int view2_line_tid);

		void unify_new_pnts_tracks(int view1_pnt_tid, int view2_pnt_tid);

		/*
		*Merges lines tracks and updates viewports accordingly.
		*/
		void unify_lines3d_into_grouped_lines3d_tracks(int view1_line3d_tid, int view2_line3d_tid, 
			HWLineTrack3D& grouped_line3d_track);

		/*
		*Merges lines and pnts tracks and updates viewports accordingly. with labels(polygon_idx)
		*/
		void unify_new_lines_tracks_labels(int view1_line_tid, int view2_line_tid);

		void unify_new_pnts_tracks_labels(int view1_pnt_tid, int view2_pnt_tid);

		bool IsValueValidFlag(float v);

		float ComputeTwoLineSegsEnergyValue(const std::pair<Eigen::Vector3f, Eigen::Vector3f>& l0,
			const std::pair<Eigen::Vector3f, Eigen::Vector3f>& l1);

		bool CheckPairsInPairsSet(const PairSetValue& v, const std::vector<PairSetValue>& vec);

        double r_max_threshold_;
		double r_max_polygon_threshold_;
		float r_max_angle_threshold_;
		float r_max_line_dist_threshold_;
		float r_max_pnts_dist_threshold_;
		double image_features_ratio_;
		PolygonIterLineOption trl2Interl_option_;
		float cam_to_polygon_inter_line_threhold_;	//filter other polygon inter lines

        std::string scene_dir_;
        bool data_updated_; //check if it's complement or not
        std::unique_ptr<HWScenesCams> scenes_cams_;
        std::unique_ptr<HWSceneLayouts> scenes_layouts_elements_;
		std::vector<HWSceneProjLayout3D> scenes_layout3ds_;	//project to scene polygon

        std::vector<std::pair<int, int> > scenes_imagesidx_layoutsidx_; //cam id to layout idx
        std::map<unsigned int, unsigned int> scenes_imgid_to_layoutid_; //cam id to layout id

        //HW::HWScenePolygons* scene_polygon_data_ = nullptr;
        //std::shared_ptr<HWScenePolygons*> associated_scenes_polygons_;
		//std::vector<>
		//HWPolygon* associated_polygons_;
		std::vector<HWPlane*> associated_polygons_;
		//set the images labels state
		bool images_labels_loaded_ = false;
		HWImageLabels* associsated_images_labels_;

		//to check polygon inter line if visiable
		std::vector<PolygonInterLineScene> polygons_intersection_scene_lines_;
		std::vector<View2PolygonInterLinesData> views_visiable_polygon_inter_lines_;

		/**/
		bool filter_wrong_lines_match_;
		bool filter_wrong_pnts_match_;

		/*
		data assigned by HWPolygon intersection lines 3d
		*/
		std::vector<HWSceneLine3DNode> scene_line3d_node_;

        double epipolar_overlap_; 

		//set the data complete
		bool import_data_from_scene_elements_state_;
		bool import_data_from_pairs_lines_txt_state_;
		bool import_data_from_python_numpy_state_;
		bool use_images_lines_pairs_imported_;
		bool use_images_pnts_pairs_imported_;
		std::vector<HWLineCorrespondLine2D> images_lines_pairs_imported_;	//next, to filter the wrong lines matches(no use)
		std::vector<HWLineCorrespondLine2D> images_lines_pairs_imported_backup_;
		std::vector<HWLineCorrespondPnt2D> images_pnts_pairs_imported_;	//next, to filter the wrong pnts matches(no use)

        //lines matches to do next
        std::map<unsigned int,std::set<unsigned int> > matched_;    //store cams matched: first src cam id, second: matched cams ids

		//first src cam id, second cam id vector and matrix3d
        std::map<unsigned int,std::map<unsigned int, Eigen::Matrix3f> > matched_fundamentals_;	//compute the fundamental matrix
        std::map<unsigned int,std::vector<std::list<HWMatch> > > matches_;  //store line matches
        std::map<unsigned int,unsigned int> num_matches_;

		//the idx same to cams_models_'s idx
		std::vector<cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> > scenes_sift_detectors_;
		std::vector<std::vector<cv::KeyPoint> > scenes_sifts_positions_;

		bool views_initial_updated_flag_;
		
		//store all original data(contain cameral models; features positions; line segments positions)
		HWCamsViewportList hw_cams_views_list_;	
		HWCamsViewportList hw_cams_views_list_prev_;

		HWPairwisesMatchingView scenes_lines_pnts_matches_;

		//filter wrong line matches with the polygon data: threshold
		float lines_match_angle_threshold_ = 8.0;
		float lines_match_dist2line_threshold_ = 10.0;
		float pnts_match_pnt2pnt_threshold_ = 10.0;

		//point 3d and line 3d
		HWTrack3DList scenes_pnts3d_track_list_;
		HWLineTrack3DList scenes_line3d_track_list_;
		HWGroupedLineTrack3DList scenes_line3d_grouped_track_list_;

		//observatons important
		HWSurveyPoint3d2Pnts2dList scenes_pnt3ds_observations_;	//
		HWLineSurveyPoint3d2Pnts2dList scenes_lines_pnts3ds_observations_;	//
		HWSurveyPoint3d2Pnts2dList scenes_pnt3ds_observations_pre_;	//
		HWLineSurveyPoint3d2Pnts2dList scenes_lines_pnts3ds_observations_pre_;	//

		//then convert data to hw_bundle adjustment
		std::vector<HWPolygonInterLines> polygons_intersection_lines_;
    };
}


#endif