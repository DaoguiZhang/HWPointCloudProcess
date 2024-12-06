/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <algorithm>

#include "hw_feature_set.h"

namespace HWSFM
{
	void HWFeatureSet::compute_features(const cv::Mat& image)
	{
		this->colors.clear();
		this->positions.clear();
		this->width = image.cols;
		this->height = image.rows;
		if (opts.feature_types == HWFeatureTypes::HW_FEATURE_SIFT)
		{
			this->compute_sift(image);
		}
		if (opts.feature_types == HWFeatureTypes::HW_FEATURE_SURF)
		{
			this->compute_surf(image);
		}
	}

	void HWFeatureSet::normalize_feature_positions(float px, float py)
	{
		/* Normalize image coordinates. */
		float const fwidth = static_cast<float>(this->width);
		float const fheight = static_cast<float>(this->height);
		float const fnorm = std::max(fwidth, fheight);
		for (std::size_t i = 0; i < this->positions.size(); ++i)
		{
			cv::KeyPoint& pos = this->positions[i];
			pos.pt.x = (pos.pt.x + 0.5f - fwidth * px) / fnorm;
			pos.pt.y = (pos.pt.y + 0.5f - fheight * py) / fnorm;
		}
	}

	void HWFeatureSet::clear_descriptors(void)
	{
		descriptor_image_->release();
		sift_descriptor_->clear();
		surf_descriptor_->clear();
	}

	void HWFeatureSet::compute_sift(const cv::Mat& image)
	{
		sift_descriptor_ = cv::xfeatures2d::SIFT::create(
			opts.sift_opts.hw_nfeatures, opts.sift_opts.hw_nOctaveLayers, 
			opts.sift_opts.hw_contrastThreshold, opts.sift_opts.hw_edgeThreshold,
			opts.sift_opts.hw_sigma);

		sift_descriptor_->detect(image, positions);
		std::cerr << "the sift kepoints num: " << positions.size() << std::endl;
		sift_descriptor_->compute(image, positions, *descriptor_image_);
		//sift_descriptor_->detectAndCompute(image, cv::Mat(), positions, *descriptor_image_);
	}

	void HWFeatureSet::compute_surf(const cv::Mat& image)
	{
		surf_descriptor_ = cv::xfeatures2d::SURF::create(
			opts.surf_opts.hw_hessianThreshold, opts.surf_opts.hw_nOctaves,
			opts.surf_opts.hw_nOctaveLayers, opts.surf_opts.hw_extended,
			opts.surf_opts.hw_upright);

		surf_descriptor_->detect(image, positions);
		std::cerr << "the surf kepoints num: " << positions.size() << std::endl;
		surf_descriptor_->compute(image, positions, *descriptor_image_);
	}
}