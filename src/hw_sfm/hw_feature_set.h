/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef HW_SFM_FEATURE_SET_HEADER
#define HW_SFM_FEATURE_SET_HEADER

#include <vector>

//#include "math/vector.h"
//#include "util/aligned_memory.h"
//#include "sfm/sift.h"
//#include "sfm/surf.h"
//#include "sfm/defines.h"
#include<opencv2/xfeatures2d.hpp>
#include<opencv2/xfeatures2d/nonfree.hpp>

//SFM_NAMESPACE_BEGIN
namespace HWSFM
{
	struct HWSiftOptions
	{
		//sift detect params, to do next
		int hw_nfeatures = 0;
		int hw_nOctaveLayers = 3;
		double hw_contrastThreshold = 0.04;
		double hw_edgeThreshold = 10;
		double hw_sigma = 1.5;
	};
	struct HWSurfOptions
	{
		//surf detect params
		double hw_hessianThreshold = 100;
		int hw_nOctaves = 4;
		int hw_nOctaveLayers = 3;
		bool hw_extended = false;
		bool hw_upright = false;
	};

	class HWFeatureSet
	{
	public:
		/** Bitmask with feature types. */
		enum HWFeatureTypes
		{
			HW_FEATURE_SIFT = 1 << 0,
			HW_FEATURE_SURF = 1 << 1,
			HW_FEATURE_ALL = 0xFF
		};

		/** Options for feature detection and matching. */
		struct Options
		{
			Options(void);
			HWFeatureTypes feature_types;
			//cv::xfeatures2d::SiftDescriptorExtractor
			HWSiftOptions sift_opts;
			HWSurfOptions surf_opts;
		};

	public:
		HWFeatureSet(void);

		explicit HWFeatureSet(Options const& options);

		void set_options(Options const& options);

		const Options& get_options();

		/** Computes the features specified in the options. */
		void compute_features(const cv::Mat& image);

		/** Normalizes the features positions w.r.t. the image dimensions. 
		descard it...*/
		void normalize_feature_positions(float px, float py);

		/** Clear descriptor data. */
		void clear_descriptors(void);

	public:
		/** Image dimension used for feature computation. */
		int width, height;
		/** Per-feature image position. */
		std::vector<cv::KeyPoint> positions;
		/** Per-feature image color. */
		std::vector<cv::Vec3b> colors;
		/** The SIFT descriptors. */
		cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> sift_descriptor_;
		
		/** The SURF descriptors. */
		//cv::xfeatures2d::SurfDescriptorExtractor surf_descriptors;
		cv::Ptr<cv::xfeatures2d::SurfDescriptorExtractor> surf_descriptor_;

		cv::Ptr<cv::Mat> descriptor_image_;

	private:
		void compute_sift(const cv::Mat& image);
		void compute_surf(const cv::Mat& image);

	private:
		Options opts;
	};

	/**
	* The FeatureSet holds per-feature information for a single view, and
	* allows to transparently compute and match multiple feature types.
	*/


	/* ------------------------ Implementation ------------------------ */

	inline
		HWFeatureSet::Options::Options(void)
		: feature_types(HW_FEATURE_SIFT)
	{

	}

	inline
		HWFeatureSet::HWFeatureSet(void)
	{
	}

	inline
		HWFeatureSet::HWFeatureSet(Options const& options)
		: opts(options)
	{
	}

	inline void
		HWFeatureSet::set_options(Options const& options)
	{
		this->opts = options;
	}

	const HWFeatureSet::Options& HWFeatureSet::get_options()
	{
		return opts;
	}
}

#endif /* SFM_FEATURE_SET_HEADER */
