#pragma once
#ifndef HW_COMMON_H
#define HW_COMMON_H

#include "math_utils.h"

namespace HW
{
	enum ElementType
	{
		kHWMesh = 0,
		kHWPointCloud,
		kHWTSDF,
		kHWPlane,
		kHWPolygon
	};

	enum ProcessType
	{
		kHWSNap = 0,
		kHWShowPolygon,
		kHWESimp,
		kHWContruction,
		KHWMls,
		kHWSor
	};

	enum PlyFormat
	{
		kBinary = 0,
		kAscci
	};
}
#endif