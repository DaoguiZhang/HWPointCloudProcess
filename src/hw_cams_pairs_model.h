/*
important to note
filter the wrong lines pairs and points pairs
*/
#pragma once

#ifndef HW_CAMS_PAIRS_MODEL_HEADER
#define HW_CAMS_PAIRS_MODEL_HEADER

#include "hw_polygon.h"
#include "model_cameras.h"
#include"hw_sfm/hw_bundler_common.h"

namespace HW
{
	class HWCamsPairsModel
	{
	public:
		HWCamsPairsModel();
		~HWCamsPairsModel();

		void set_polygons(const std::vector<HWPlane*>& in_polygon);
		void set_views_ports(HWSFM::HWViewportList* views);
		void set_inital_pairs(HWSFM::HWPairwiseMatching* pairs_matches);

		void filter_wrong_pairs_from_polygons(HWSFM::HWPairwiseMatching* updated_pairs);

	private:
		std::vector<HWPlane*> polygons_;	//
		//std::pair<CameraModel, CameraModel> l2r_cams_;
		HWSFM::HWPairwiseMatching* initial_pairs_;
		HWSFM::HWViewportList* views_ports_;

	};
}

#endif