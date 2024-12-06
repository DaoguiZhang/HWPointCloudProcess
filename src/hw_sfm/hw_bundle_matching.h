#pragma once

#ifndef HW_FEATURES_MATCHES_WRAPPER_H
#define HW_FEATURES_MATCHES_WRAPPER_H

#include <memory>
#include <stdexcept>
#include <vector>
#include <string>
#include <sstream>
#include<set>

//使用opencv的特征提取和匹配
#include"hw_bundler_common.h"
#include"hw_feature_set.h"

namespace HWSFM
{
	class HWMatching
	{
	public:

		enum HWMATCHERMATHOD
		{
			HWNORM1 = 1,
			HWNORM2 = 2,
			HWNORMHAMMING = 3,
			HWNORMHAMMING2 = 4
		};

		enum HWMatcherType
		{
			HWMATCHER_BFMATCHER,
			HWMATCHER_FLANNBASEDMATCHER
		};

		/** Options for feature matching. */
		struct Options
		{
			/** Options for RANSAC computation of the fundamental matrix. */
			//sfm::RansacFundamental::Options ransac_opts;	//to do next
			/** Minimum number of matching features before RANSAC. */
			int min_feature_matches = 24;
			/** Minimum number of matching features after RANSAC. */
			int min_matching_inliers = 12;
			/** Perform low-resolution matching to reject unlikely pairs. */
			bool use_lowres_matching = false;
			
			/*
			Minimum number of matching lines segment
			*/
			int min_lines_segments_mathes = 5;

			/** Number of features used for low-res matching. */
			int num_lowres_features = 500;
			/** Minimum number of matches from low-res matching. */
			int min_lowres_matches = 5;
			/** Only match to a few previous frames. Disabled by default. */
			int match_num_previous_frames = 0;
			/** Matcher type. Exhaustive by default. */
			HWMatcherType matcher_type = HWMatcherType::HWMATCHER_BFMATCHER;

			HWMATCHERMATHOD matcher_method = HWMATCHERMATHOD::HWNORM2;
		};

		struct Progress
		{
			std::size_t num_total;
			std::size_t num_done;
		};

	public:

		HWMatching(Options const& options, Progress* progress = nullptr);

		/**
		* Initialize matching by passing features to the matcher for
		* preprocessing.
		*/
		void init(HWViewportList* viewports);

		/**
		* Initialize matching by passing pairs_initial to the matcher for
		* preprocessing.
		*/
		void init_pairs(HWPairwiseMatching* pairs_inital);

		/**
		* Initialize matching by passing pairs_initial to the matcher for
		* preprocessing.
		*/
		void SetInitialViewsPairsFromHWLinesData(std::map<unsigned int, std::set<unsigned int> >* views);
		void SetInitialViewsLinesPairsFromHWLinesData(std::map<unsigned int, std::vector<std::list<HWBundleViewsMatch> > >* lines);


		/**
		* Computes the pairwise matching between all pairs of views.
		* Computation requires both descriptor data and 2D feature positions
		* in the viewports.
		*/
		void compute(HWPairwiseMatching* pairwise_matching);

		/*
		*remove all the wrong lines matches and points matches
		*/
		void filter_all_points_lines_matches(HWPairwiseMatching* pairwise_matching);

	private:

		void two_view_matching(int view_1_id, int view_2_id,
			HWCorrespondenceIndices* matches, std::stringstream& message);

	private:

		Options opts;
		Progress* progress;
		cv::Ptr<cv::DescriptorMatcher> matcher;	//使用这个函数进行特征点匹配
		//初始的视角匹配，用于后续的优化（它里面包括所有的点特征，和线段特征的匹配）
		int initial_views_matches_num_;

		std::map<unsigned int, std::set<unsigned int> >* initial_views_matched_p_;
		std::map<unsigned int, std::vector<std::list<HWBundleViewsMatch> > >* initial_lines_matches_p_;

		HWPairwiseMatching* initial_matches_;	
		//HWCorrespondenceIndices initial_matches_;	//
		HWViewportList* viewports;	//匹配，里面包括点特征，和线段特征的匹配
	};

}

#endif