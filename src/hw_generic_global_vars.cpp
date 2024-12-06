#include"hw_generic_global_vars.h"
#include"hw_singleton.h"

//static HW::HWSingleton<HW::HWParams> g_algorithm_params;

namespace HW
{
	//const HWSORParams& HWParams::GetSORParams()
	//{
	//	if (!g_algorithm_params.instance)
	//	{
	//		//已经放入参数
	//		g_algorithm_params.instance = new HWParams();
	//	}
	//	return g_algorithm_params.instance->sor_params_;
	//}
	//const HWMLSParams& HWParams::GetMLSParams()
	//{
	//	if (!g_algorithm_params.instance)
	//	{
	//		//已经放入参数
	//		g_algorithm_params.instance = new HWParams();
	//	}
	//	return g_algorithm_params.instance->mls_params_;
	//}
	//const HWConstructionParams& HWParams::GetConstructionParams()
	//{
	//	if (!g_algorithm_params.instance)
	//	{
	//		//已经放入参数
	//		g_algorithm_params.instance = new HWParams();
	//	}
	//	return g_algorithm_params.instance->construction_params_;
	//}
	//const HWSimplificationParams& HWParams::GetSimplificationParams()
	//{
	//	if (!g_algorithm_params.instance)
	//	{
	//		//已经放入参数
	//		g_algorithm_params.instance = new HWParams();
	//	}
	//	return g_algorithm_params.instance->simplification_params_;
	//}

	void HWParams::SetSORParamsPntNum(int point_num)
	{
		sor_params_.point_num_ = point_num;
	}

	void HWParams::SetSORParamsMultiplier(float multiplier)
	{
		sor_params_.devialtion_multiplier_ = multiplier;
	}

	void HWParams::SetSORParams(HWSORParams& in_param)
	{
		sor_params_.point_num_ = in_param.point_num_;
		sor_params_.devialtion_multiplier_ = in_param.devialtion_multiplier_;
	}

	void HWParams::SetMLSParams(HWMLSParams& in_param)
	{
		mls_params_.compute_normal_flag_ = in_param.compute_normal_flag_;
		mls_params_.gassian_param_sqrt_ = in_param.gassian_param_sqrt_;
		mls_params_.polynomial_order_ = in_param.polynomial_order_;
		mls_params_.search_radius_ = in_param.search_radius_;
		mls_params_.upsample_methods_ = in_param.upsample_methods_;
	}

	void HWParams::SetConstructionParams(HWConstructionParams& in_param)
	{
		construction_params_.volume_center_.x = in_param.volume_center_.x;
		construction_params_.volume_center_.y = in_param.volume_center_.y;
		construction_params_.volume_center_.z = in_param.volume_center_.z;
		construction_params_.volume_dimensions_.x = in_param.volume_dimensions_.x;
		construction_params_.volume_dimensions_.y = in_param.volume_dimensions_.y;
		construction_params_.volume_dimensions_.z = in_param.volume_dimensions_.z;
		construction_params_.voxel_length_ = in_param.voxel_length_;
	}

	void HWParams::SetSimplificationParams(HWSimplificationParams& in_param)
	{
		simplification_params_.boundary_preserve_flag_ = in_param.boundary_preserve_flag_;
		simplification_params_.boundary_preserve_weight_ = in_param.boundary_preserve_weight_;
		simplification_params_.preserve_normal_flag_ = in_param.preserve_normal_flag_;
		simplification_params_.target_percentage_ = in_param.target_percentage_;
		simplification_params_.target_faces_num_ = in_param.target_faces_num_;
		simplification_params_.quality_threshold_ = in_param.quality_threshold_;
		simplification_params_.topology_preserve_flag_ = in_param.topology_preserve_flag_;
		simplification_params_.optimal_placement_flag_ = in_param.optimal_placement_flag_;
		simplification_params_.planar_quadric_flag_ = in_param.planar_quadric_flag_;
	}

	/*void HWParams::ReleaseInstance()
	{
		g_algorithm_params.release();
	}*/
}