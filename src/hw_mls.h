#pragma once
#ifndef HW_MLS
#define HW_MLS

#include <pcl/surface/mls.h>

#include "hw_pcl_functor.h"

namespace HW
{
	class HWMLS :public HWPCLFunctor
	{
	public:
		HWMLS();
		~HWMLS();

		void Process(HWObject* in_element, HWObject* out_element);
	};
}
#endif