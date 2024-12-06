/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef HWSFM_BUNDLER_TRACKS_HEADER
#define HWSFM_BUNDLER_TRACKS_HEADER

#include"hw_ba_camera_pose.h"
#include"hw_bundler_common.h"
#include"hw_bundle_matching.h"

namespace HWSFM
{
	/**
	* Bundler Component: Generation of tracks from pairwise matching result.
	*
	* As input this component requires all the pairwise matching results.
	* Additionally, to color the tracks, a color for each feature must be set.
	*/
	class HWBundleTracks
	{
	public:
		struct Options
		{
			Options(void);

			/** Produce status messages on the console. */
			bool verbose_output;
		};

	public:

		explicit HWBundleTracks(Options const& options);

		/**
		* Computes viewport connectivity information by propagating track IDs.
		* Computation requires feature positions and colors in the viewports.
		* A color for each track is computed as the average color from features.
		* Per-feature track IDs are added to the viewports.
		*/
		void compute(HWPairwiseMatching const& matching,
			HWViewportList* viewports, HWTrackList* tracks, HWLineTrackList* line_tracks);

	private:
		int remove_invalid_tracks(HWViewportList* viewports, HWTrackList* tracks);

	private:
		Options opts;
	};

	/* ------------------------ Implementation ------------------------ */

	inline
		HWBundleTracks::Options::Options(void)
		: verbose_output(false)
	{
	}

	inline
		HWBundleTracks::HWBundleTracks(Options const& options)
		: opts(options)
	{
	}
}

#endif /* SFM_BUNDLER_TRACKS_HEADER */
