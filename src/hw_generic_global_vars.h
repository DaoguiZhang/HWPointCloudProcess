#pragma once
#ifndef HW_GENERIC_GLOBAL_VARS
#define HW_GENERIC_GLOBAL_VARS
#include "math_utils.h"

namespace HW
{
	enum UpsampleMethods
	{
		kNone = 0,
		kSampleLocalPlane = 1,
		kRandomUniformDensity,
		kVoxelGridDilation,
	};

	class HWSORParams
	{
	public:
		HWSORParams() {};
		HWSORParams(int in_point_num, float in_devialtion_multiplier):
			point_num_(in_point_num), devialtion_multiplier_(in_devialtion_multiplier)
		{}
		int point_num_;
		float devialtion_multiplier_;
	};
	class HWMLSParams
	{
	public:
		HWMLSParams() {}
		HWMLSParams(float in_radius, bool in_normal_flag, float in_order, float in_gassian_param_sqrt, 
			UpsampleMethods in_method):search_radius_(in_radius), compute_normal_flag_(in_normal_flag),
			polynomial_order_(in_order), gassian_param_sqrt_(in_gassian_param_sqrt),
			upsample_methods_(in_method)
			{}
		float search_radius_;
		bool compute_normal_flag_;
		bool use_polynomial_flag_;
		float polynomial_order_;
		float gassian_param_sqrt_;
		UpsampleMethods upsample_methods_;
		//sample local plane
		float upsampling_radius_;
		float upsampling_step_;
		//random uniform density
		int step_point_density_;
		//voxel grid dilation
		float dilation_voxel_size_;
		int dilation_iterations_;
	};

	class HWExtractPlaneParams
	{
	public:
		HWExtractPlaneParams() {}
		~HWExtractPlaneParams() {}

		int plane_min_pnt_num_;
		float ceil_height_;
		float plane_merge_height_;
		float max_normal_deviation_;
		float min_plane_width_;

		//the set for merge plane param
		int plane_min_pnt_num_merge_;
		float xy_ratio_merge_;
		float min_area__merge_;
	};

	class HWConstructionParams
	{
	public:
		HWConstructionParams() {}
		HWConstructionParams(float in_length, int3 in_volume_dimensions, float3 in_center)
		{
			voxel_length_ = in_length;
			volume_dimensions_ = in_volume_dimensions;
			volume_center_ = in_center;
		}
		float voxel_length_;
		float volume_trunc_length_;
		int3 volume_dimensions_;
		float3 volume_center_;
	};

	class HWSimplificationParams
	{
	public:
		HWSimplificationParams() {}

		HWSimplificationParams(int in_faces_num, float in_reduction_percentage, bool in_boundary_preserve,
			float in_boundary_weight, bool in_normal_flag, float quality_threshold, bool topology_preserve_flag,
		bool planar_quadric_flag, bool optimal_placement_flag)
            :target_faces_num_(in_faces_num),
			target_percentage_(in_reduction_percentage), boundary_preserve_flag_(in_boundary_preserve),
			boundary_preserve_weight_(in_boundary_weight), preserve_normal_flag_(in_normal_flag), 
			quality_threshold_(quality_threshold), topology_preserve_flag_(topology_preserve_flag),
			planar_quadric_flag_(planar_quadric_flag), optimal_placement_flag_(optimal_placement_flag)
		{}

		int target_faces_num_;
		float target_percentage_;
		bool boundary_preserve_flag_;
		float boundary_preserve_weight_;
		bool preserve_normal_flag_;
		float quality_threshold_;
		bool topology_preserve_flag_;
		bool planar_quadric_flag_;
		bool optimal_placement_flag_;
	};

	class HWParams
	{
	public:
		static HWParams& getInstance() {

			static HWParams instance;
			return instance;
		}

		/*const HWSORParams& GetSORParams();
		const HWMLSParams& GetMLSParams();
		const HWConstructionParams& GetConstructionParams();
		const HWSimplificationParams& GetSimplificationParams();*/
		void SetSORParamsPntNum(int point_num);
		void SetSORParamsMultiplier(float multiplier);

		void SetSORParams(HWSORParams& in_param);
		void SetMLSParams(HWMLSParams& in_param);
		void SetConstructionParams(HWConstructionParams& in_param);
		void SetSimplificationParams(HWSimplificationParams& in_param);

	private:
		HWParams() 
		{
			sor_params_.point_num_ = 6;
			sor_params_.devialtion_multiplier_ = 1.0;
			mls_params_.search_radius_ = 0.01;
			mls_params_.compute_normal_flag_ = true;
			mls_params_.use_polynomial_flag_ = false;
			mls_params_.upsampling_radius_ = 0.0100;
			mls_params_.upsampling_step_ = 0.0100;
			mls_params_.gassian_param_sqrt_ = 0.000100;
			////mls_params_.polynomial_order_ = 2.0;
			mls_params_.upsample_methods_ = kNone;
			plane_params_.plane_min_pnt_num_ = 500;
			plane_params_.ceil_height_ = 3.0;
			plane_params_.plane_merge_height_ = 2.0;
			plane_params_.max_normal_deviation_ = 20.00;
			plane_params_.min_plane_width_ = 0.1;
			plane_params_.plane_min_pnt_num_merge_ = 50000;
			plane_params_.xy_ratio_merge_ = 0.02;
			plane_params_.min_area__merge_ = 2.0;
			construction_params_.volume_dimensions_.x = 10000;
			construction_params_.volume_dimensions_.y = 10000;
			construction_params_.volume_dimensions_.z = 10000;
			construction_params_.volume_center_.x = 0.0f;
			construction_params_.volume_center_.y = 0.0f;
			construction_params_.volume_center_.z = 0.0f;
			construction_params_.voxel_length_ = 0.02;
			construction_params_.volume_trunc_length_ = 0.04;
			simplification_params_.boundary_preserve_flag_ = true;
			simplification_params_.boundary_preserve_weight_ = 1.0;
			simplification_params_.preserve_normal_flag_ = true;
			simplification_params_.target_percentage_ = 0.5;
			simplification_params_.target_faces_num_ = 1000;
			simplification_params_.quality_threshold_=0.3;
			simplification_params_.topology_preserve_flag_=false;
			simplification_params_.planar_quadric_flag_=true;
			simplification_params_.optimal_placement_flag_=true;
		}

		//! Release unique instance (if any)
		//static void ReleaseInstance();

	public:
		HWSORParams sor_params_;
		HWMLSParams mls_params_;
		HWExtractPlaneParams plane_params_;
		HWConstructionParams construction_params_;
		HWSimplificationParams simplification_params_;

		//global HW
		int unique_obj_label_ = -1;
	};
}

#endif