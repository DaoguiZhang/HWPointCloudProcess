#pragma once
#ifndef HW_TSDF_H
#define HW_TSDF_H

#include "hw_object.h"
#include "ScalableTSDFVolume.h"

namespace HW 
{
	class HWTSDF :public HWObject
	{
	public:
		HWTSDF();
		~HWTSDF();
		bool Show();
		bool Save();

		//open3d::ScalableTSDFVolume* scale_volume_;
		int3 volume_resolutions_;
		float voxel_length_;
		float weight_;
	};
}


#endif