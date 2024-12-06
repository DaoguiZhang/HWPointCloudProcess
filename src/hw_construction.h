#pragma once
#ifndef HW_CONSTRUCTION
#define HW_CONSTRUCTION

#include "hw_pcl_functor.h"
#include "hw_tsdf.h"

namespace HW
{
	class HWConstruction :public HWPCLFunctor
	{
	public:
		HWConstruction();
		~HWConstruction();
		void Process(HWObject* in_element, HWObject* out_element);
	};
}

#endif