#include"hw_bundle_matching.h"

#include <iostream>
#include <fstream>
#include <cstring>
#include <cerrno>
#include <stdexcept>
#include"hw_cams_pairs_model.h"

namespace HWSFM
{
	HWMatching::HWMatching(Options const& options, Progress* progress)
		:opts(options),
		progress(progress)
	{
		switch (options.matcher_type)
		{
		case (HWMatcherType::HWMATCHER_BFMATCHER):
		{
			if (options.matcher_method == HWMATCHERMATHOD::HWNORM1)
			{
				this->matcher = cv::BFMatcher::create(cv::NORM_L1);
			}
			else if (options.matcher_method == HWMATCHERMATHOD::HWNORM2)
			{
				this->matcher = cv::BFMatcher::create(cv::NORM_L2);
			}
			else if (options.matcher_method == HWMATCHERMATHOD::HWNORMHAMMING)
			{
				this->matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
			}
			else if (options.matcher_method == HWMATCHERMATHOD::HWNORMHAMMING2)
			{
				this->matcher = cv::BFMatcher::create(cv::NORM_HAMMING2);
			}
			else
			{
				this->matcher = cv::BFMatcher::create(cv::NORM_L2);
			}
			break;
		}
		case(HWMatcherType::HWMATCHER_FLANNBASEDMATCHER):
		{
			this->matcher = cv::FlannBasedMatcher::create();
			break;
		}
		default:
			break;
		}
	}

	void HWMatching::init(HWViewportList* viewports)
	{
		if (viewports == nullptr)
			throw std::invalid_argument("Viewports must not be null");

		this->viewports = viewports;
		//this->matcher->init(viewports);

		/* Free descriptors. */
		for (std::size_t i = 0; i < viewports->size(); i++)
			viewports->at(i).features_.clear_descriptors();
	}
	
	void HWMatching::init_pairs(HWPairwiseMatching* pairs_inital)
	{
		initial_matches_ = pairs_inital;
	}

	void HWMatching::SetInitialViewsPairsFromHWLinesData(std::map<unsigned int, std::set<unsigned int> >* views)
	{
		//
		initial_views_matched_p_ = views;
	}

	void HWMatching::SetInitialViewsLinesPairsFromHWLinesData(std::map<unsigned int, std::vector<std::list<HWBundleViewsMatch> > >* lines)
	{
		initial_lines_matches_p_ = lines;
	}

	void HWMatching::compute(HWPairwiseMatching* pairwise_matching)
	{
		if (initial_matches_->empty())
		{
			std::cerr << "initial empty..." << std::endl;
			return;
		}
		//从线段的匹配得到
		for (std::size_t i = 0; i < initial_matches_->size(); ++i)
		{
			int view1_id = initial_matches_->at(i).view_1_id;
			int view2_id = initial_matches_->at(i).view_2_id;
			HWCorrespondenceIndices matches;
			std::stringstream message;
			this->two_view_matching(view1_id, view2_id, &matches, message);
			if (matches.empty())
			{
				std::cout << "\rPair (" << view1_id << ","
					<< view2_id << ") rejected, "
					<< message.str() << std::endl;
				continue;
			}

			/* Successful two view matching. Add the pair. */
			HWTwoViewMatching matching;
			matching.view_1_id = view1_id;
			matching.view_2_id = view2_id;
			std::swap(matching.points_matches, matches);

			pairwise_matching->push_back(matching);
			std::cout << "\rPair (" << view1_id << ","
				<< view2_id << ") matched, " << matching.points_matches.size() << std::endl;
			std::cout << "\r line Pair (" << view1_id << ","
				<< view2_id << ") matched, " << matching.lines_matches.size() << std::endl;
		}
		//copy result to viewports
	}

	void HWMatching::filter_all_points_lines_matches(HWPairwiseMatching* pairwise_matching)
	{
		//construct HWCamsPairsModel data
		HW::HWCamsPairsModel* all_initial_pairs = new HW::HWCamsPairsModel();
		
		//remove some pairs which do not meet conditions
		//to do next....upadte them from initial camera
		//how to do it
	}

	//use opencv matcher, how to wrapper them
	void HWMatching::two_view_matching(int view_1_id, int view_2_id,
		HWCorrespondenceIndices* matches, std::stringstream& message)
	{
		HWFeatureSet view_1 = this->viewports->at(view_1_id).features_;
		HWFeatureSet view_2 = this->viewports->at(view_2_id).features_;
		std::vector<cv::DMatch> matches_cv;

		if (view_1.get_options().feature_types != view_2.get_options().feature_types)
		{
			std::cerr << "invalid feature type..." << std::endl;
			return;
		}

		//使用参数来匹配
		matcher->match(*view_1.descriptor_image_, *view_2.descriptor_image_, matches_cv);
		int num_matches = static_cast<int>(matches_cv.size());

		/* Build correspondences from feature matching result. */
		HWCorrespondences2D2D unfiltered_matches;
		HWCorrespondenceIndices unfiltered_indices;
		for (std::size_t i = 0; i < matches_cv.size(); ++i)
		{
			if (matches_cv[i].queryIdx < 0 || matches_cv[i].trainIdx < 0)
				continue;
			HWCorrespondence2D2D match;
			match.p1[0] = view_1.positions[matches_cv[i].queryIdx].pt.x;
			match.p1[1] = view_1.positions[matches_cv[i].queryIdx].pt.y;
			match.p2[0] = view_2.positions[matches_cv[i].trainIdx].pt.x;
			match.p2[1] = view_2.positions[matches_cv[i].trainIdx].pt.y;
			unfiltered_matches.push_back(match);
			unfiltered_indices.push_back(std::make_pair(matches_cv[i].queryIdx, matches_cv[i].trainIdx));
			matches->push_back(unfiltered_indices[i]);
		}
	}
}