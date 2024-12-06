/*
* Copyright (C) 2015, Simon Fuhrmann
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the LICENSE.txt file for details.
*/

#pragma once

#ifndef HW_FEATURES_DETECT_WRAPPER_H
#define HW_FEATURES_DETECT_WRAPPER_H

#if 0
#include <iostream>

#include "util/file_system.h"
#include "util/timer.h"
#include "mve/image.h"
#include "mve/image_tools.h"
#include "mve/image_io.h"

#include "sfm/surf.h"
#include "sfm/sift.h"
#include "sfm/visualizer.h"

namespace HWSFM
{
	bool
		sift_compare(sfm::Sift::Descriptor const& d1, sfm::Sift::Descriptor const& d2)
	{
		return d1.scale > d2.scale;
	}

	bool
		surf_compare(sfm::Surf::Descriptor const& d1, sfm::Surf::Descriptor const& d2)
	{
		return d1.scale > d2.scale;
	}

	void SiftDetectorRun(const mve::ByteImage::Ptr& image, const sfm::Sift::Options& sift_options,
		sfm::Sift::Descriptors& sift_descr, sfm::Sift::Keypoints& sift_keypoints);


	void SurfDetectorRun(const mve::ByteImage::Ptr& image, const sfm::Surf::Options& Surf_options,
		sfm::Surf::Descriptors& Surf_descr, sfm::Surf::Keypoints& Surf_keypoints);
	
	//test the SiftDetectorRun
	void RunDetectorZDGTest();
}

#endif
#endif
