#pragma once
#ifndef HW_SIMP_H
#define HW_SIMP_H

#include "hw_pcl_functor.h"

namespace HW
{
	class HWSIMP :public HWPCLFunctor
	{
	public:
		HWSIMP();
		~HWSIMP();
		void Process(HWObject* in_element, HWObject* out_element);
	};
}
#endif