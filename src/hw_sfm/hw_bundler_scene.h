/*
store all bundler points, cams and points and so on

then optimize them

*/
#pragma once

#ifndef HW_BUNDLER_SCENE_HEADER
#define HW_BUNDLER_SCENE_HEADER

#include<set>
#include"hw_bundle_defines.h"
#include"hw_bundler_common.h"
#include"hw_bundler_incremental.h"
#include"hw_bundler_tracks.h"

//#include "PolygonGLWindow.h"

namespace HWSFM
{
	class HWBundlerScene
	{
	public:

		HWBundlerScene();

		HWBundlerScene(HWViewportList* views, HWTrackList* tracks, HWLineTrackList* lines_tracks,
			HWSurveyPointList* survey_points, HWLineSurveyPointList* survey_list);

		~HWBundlerScene();

		void SetHWViewportList(HWViewportList* views);
		void SetHWTracktList(HWTrackList* points_tracklist);
		void SetHWLineTrackList(HWLineTrackList* lines_tracklist);
		void SetHWSurveyPointList(HWSurveyPointList* survey_pointlist);
		void SetHWLineSurveyPointList(HWLineSurveyPointList* survey_linelist);

		//copy data from hw lines matches with polygons intersections
		void TransmissionHWLinesPairsFromIntersections(const std::vector<BundlerLine3DNode>& hw_lines_tracks_list);
		//copy data from hw lines matches with polygon interior
		void TransmissionHWLinesPairsInterior(const std::map<unsigned int, std::set<unsigned int> >& views_match,
			const std::map<unsigned int, std::vector<std::list<HWBundleViewsMatch> > >& lines_match);
		
		void UnitHWLinesIntersectionInterior2HWScenes();	//update views_matched_, lines_matches_

		void ConvertPolygonMatches2HWPairwiseMatchingData();

		void UpdateAllViewsFeatures();

		void UpdateAllMatchesViews();
		
		void UpdateAllTracksFromMatches();

		void UpdateAllSurveyDataFromTracksByTriangule();

		void UpdateAllBundlerAdjustments();

	private:

		HWViewportList* viewports_;
		HWTrackList* tracks_;
		HWLineTrackList* lines_tracks_;
		HWSurveyPointList* survey_points_;
		HWLineSurveyPointList* survey_lines_points_;

		//variables generated during HWBundleScene program execution
		std::vector<BundlerLine3DNode> initial_line3d_data_;	//line 3d tracklist (to image view and line and so on)
		std::map<unsigned int, std::set<unsigned int> > views_matched_;	//image id to images ids
		std::map<unsigned int, std::vector<std::list<HWBundleViewsMatch> > > lines_matches_;	//line to line match
		HWPairwiseMatching* initial_bundler_views_pairs_;	//initial view pairs (lines pairs and points pairs)
		
		HWMatching* bundler_scene_matching_;	//
		HWPairwiseMatching* bundler_pairs_matches_;	//output of bundler_scene_matching_
		HWBundleTracks* bundler_scene_tracks_;
		HWBundleIncremental* bundler_scene_;

		std::vector<HWBACameraPose> cams_updates_;
	};
}

#endif
