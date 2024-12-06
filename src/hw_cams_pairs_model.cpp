#include"hw_cams_pairs_model.h"

namespace HW
{
	HWCamsPairsModel::HWCamsPairsModel()
	{
	}

	HWCamsPairsModel::~HWCamsPairsModel()
	{
	}

	void HWCamsPairsModel::set_polygons(const std::vector<HWPlane*>& in_polygon)
	{
		polygons_.resize(in_polygon.size());
		for (std::size_t i = 0; i < in_polygon.size(); ++i)
		{
			polygons_[i] = in_polygon[i];
		}
	}

	void HWCamsPairsModel::set_views_ports(HWSFM::HWViewportList* views)
	{
		
	}

	void HWCamsPairsModel::set_inital_pairs(HWSFM::HWPairwiseMatching* pairs_matches)
	{
		initial_pairs_ = pairs_matches;
	}

	void HWCamsPairsModel::filter_wrong_pairs_from_polygons(HWSFM::HWPairwiseMatching* updated_pairs)
	{
		std::cerr << "start to filter initial_pairs_ " << std::endl;
		std::cerr << "end to filter initial_pairs_ " << std::endl;
	}
}