/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef HW_SFM_CORRESPONDENCE_HEADER
#define HW_SFM_CORRESPONDENCE_HEADER

#include <vector>

//#include "math/matrix.h"
//#include "sfm/defines.h"

//SFM_NAMESPACE_BEGIN
namespace HWSFM
{
	struct HWCorrespondence2D2D;
	typedef std::vector<HWCorrespondence2D2D> HWCorrespondences2D2D;

	struct HWCorrespondence2D3D;
	typedef std::vector<HWCorrespondence2D3D> HWCorrespondences2D3D;

	/** The IDs of a matching feature pair or line pair in two images. */
	typedef std::pair<int, int> HWCorrespondenceIndex;
	/** A list of all matching feature pairs or all matching line pairs in two images. */
	typedef std::vector<HWCorrespondenceIndex> HWCorrespondenceIndices;

	/**
	* Two image coordinates which correspond to each other in terms of observing
	* the same point in the scene.
	* TODO: Rename this to Correspondence2D2D.
	*/
	struct HWCorrespondence2D2D
	{
		double p1[2];
		double p2[2];
	};

	/**
	* A 3D point and an image coordinate which correspond to each other in terms
	* of the image observing this 3D point in the scene.
	*/
	struct HWCorrespondence2D3D
	{
		double p3d[3];
		double p2d[2];
	};
}



//SFM_NAMESPACE_END

#endif  // SFM_CORRESPONDENCE_HEADER
