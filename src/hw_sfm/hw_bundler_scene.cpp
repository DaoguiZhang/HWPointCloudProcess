#include"hw_bundler_scene.h"
#include<iostream>

namespace HWSFM
{
	HWBundlerScene::HWBundlerScene()
	{
		viewports_ = NULL;
		tracks_ = NULL;
		lines_tracks_ = NULL;
		survey_points_ = NULL;
		survey_lines_points_ = NULL;
		initial_bundler_views_pairs_ = NULL;
		bundler_scene_matching_ = NULL;
		bundler_pairs_matches_ = NULL;
		bundler_scene_tracks_ = NULL;
		bundler_scene_ = NULL;
	}

	HWBundlerScene::HWBundlerScene(HWViewportList* views, HWTrackList* tracks, HWLineTrackList* lines_tracks,
		HWSurveyPointList* survey_points, HWLineSurveyPointList* survey_list)
	{
		viewports_ = views;
		tracks_ = tracks;
		lines_tracks_ = lines_tracks;
		survey_points_ = survey_points;
		survey_lines_points_ = survey_list;
	}

	HWBundlerScene::~HWBundlerScene()
	{
		if (viewports_)
		{
			delete viewports_;
			viewports_ = NULL;
		}
		if (tracks_)
		{
			delete tracks_;
			tracks_ = NULL;
		}
		if (lines_tracks_)
		{
			delete lines_tracks_;
			lines_tracks_ = NULL;
		}
		if (survey_points_)
		{
			delete survey_points_;
			survey_points_ = NULL;
		}
		if (survey_lines_points_)
		{
			delete survey_lines_points_;
			survey_lines_points_ = NULL;
		}
		if (initial_bundler_views_pairs_)
		{
			delete initial_bundler_views_pairs_;
			initial_bundler_views_pairs_ = NULL;
		}
		if (bundler_scene_matching_)
		{
			delete bundler_scene_matching_;
			bundler_scene_matching_ = NULL;
		}
		if (bundler_scene_tracks_)
		{
			delete bundler_scene_tracks_;
			bundler_scene_tracks_ = NULL;
		}
		if (bundler_scene_)
		{
			delete bundler_scene_;
			bundler_scene_ = NULL;
		}
		if (bundler_pairs_matches_)
		{
			delete bundler_pairs_matches_;
			bundler_pairs_matches_ = NULL;
		}
	}

	void HWBundlerScene::SetHWViewportList(HWViewportList* views)
	{
		viewports_ = views;
	}

	void HWBundlerScene::SetHWTracktList(HWTrackList* points_tracklist)
	{
		tracks_ = points_tracklist;
	}

	void HWBundlerScene::SetHWLineTrackList(HWLineTrackList* lines_tracklist)
	{
		lines_tracks_ = lines_tracklist;
	}

	void HWBundlerScene::SetHWSurveyPointList(HWSurveyPointList* survey_pointlist)
	{
		survey_points_ = survey_pointlist;
	}

	void HWBundlerScene::SetHWLineSurveyPointList(HWLineSurveyPointList* survey_linelist)
	{
		survey_lines_points_ = survey_linelist;
	}

	void HWBundlerScene::TransmissionHWLinesPairsFromIntersections(const std::vector<BundlerLine3DNode>& hw_lines_tracks_list)
	{
		initial_line3d_data_.resize(hw_lines_tracks_list.size());
		for (int i = 0; i < hw_lines_tracks_list.size(); ++i)
		{
			initial_line3d_data_[i] = hw_lines_tracks_list[i];
		}
	}

	void HWBundlerScene::TransmissionHWLinesPairsInterior(const std::map<unsigned int, std::set<unsigned int> >& views_match,
		const std::map<unsigned int, std::vector<std::list<HWBundleViewsMatch> > >& lines_match)
	{
		views_matched_ = views_match;
		lines_matches_ = lines_match;
	}

	void HWBundlerScene::UnitHWLinesIntersectionInterior2HWScenes()
	{
		//BundlerLine3DNode to HWBundleViewsMatch and set
		for (std::size_t i = 0; i < initial_line3d_data_.size(); ++i)
		{
			//to do next...
			BundlerLine3DNode line3d_pnts = initial_line3d_data_[i];
			
		}
		HWPairwiseMatching* tmp_initial = new HWPairwiseMatching();
		std::map<unsigned int, std::set<unsigned int> >::iterator src_views_it = views_matched_.begin();
		for (;src_views_it != views_matched_.end(); ++src_views_it)
		{
			unsigned int src_id = src_views_it->first;
			std::set<unsigned int>::iterator tgt_view_it = src_views_it->second.begin();
			for (; tgt_view_it != src_views_it->second.end(); ++tgt_view_it)
			{
				//
			}
		}
		//tmp_initial->emplace_back();
	}

	void HWBundlerScene::ConvertPolygonMatches2HWPairwiseMatchingData()
	{
		if (initial_line3d_data_.empty())
		{
			std::cerr << "initial lines pair..." << std::endl;
			return;
		}
		for (int i = 0; i<initial_line3d_data_.size(); ++i)
		{
			//to do it
		}

	}

	void HWBundlerScene::UpdateAllViewsFeatures()
	{
		//update data from viewports_
		if (viewports_ == NULL)
		{
			std::cerr << "viewports_ is null..." << std::endl;
		}
		int views_num = static_cast<int>(viewports_->size());
		std::cerr << "viewport num: " << views_num << std::endl;
		for (int i = 0; i < views_num; ++i)
		{
			HWViewport &view = viewports_->at(i);
			cv::Ptr<cv::Mat> img = view.image_;
			view.features_.compute_features(*img);
		}
	}

	void HWBundlerScene::UpdateAllMatchesViews()
	{
		//Get Matches from HWLineSurveyPointList
		HWMatching::Options tmp_opt;
		HWMatching* tmp_match = new HWMatching(tmp_opt);
		//tmp_match->init_pairs();
		HWPairwiseMatching* my_pairs_match = new HWPairwiseMatching();
		tmp_match->compute(my_pairs_match);
		bundler_pairs_matches_ = my_pairs_match;
	}

	void HWBundlerScene::UpdateAllTracksFromMatches()
	{
		if (bundler_scene_matching_ == NULL)
		{
			std::cerr << "invalid scene matching..." << std::endl;
			return;
		}
		HWBundleTracks::Options tmp_opt;
		HWBundleTracks* tmp_tracks = new HWBundleTracks(tmp_opt);
		//update bundler track lists
		bundler_scene_tracks_->compute(*bundler_pairs_matches_, viewports_, tracks_, lines_tracks_);
	}

	void HWBundlerScene::UpdateAllSurveyDataFromTracksByTriangule()
	{
		std::cerr << "start to create survey_points_, survey_lines_points_" << std::endl;
		//triangle from points tracks to survey_points_

		//triangle from lines tracks to survey_lines_points_

		std::cerr << "to do next..." << std::endl;
	}

	void HWBundlerScene::UpdateAllBundlerAdjustments()
	{
		//bundler cameras' poses from 
		HWBundleIncremental::Options tmp_opts;
		HWBundleIncremental* tmp_bundle = new HWBundleIncremental(tmp_opts);
		bundler_scene_ = tmp_bundle;
		bundler_scene_->initialize(viewports_, tracks_, lines_tracks_, survey_points_, survey_lines_points_);
		bundler_scene_->bundle_adjustment_full();
	}

}