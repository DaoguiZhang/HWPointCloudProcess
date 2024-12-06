#pragma once

#ifndef HW_PCL_FUNCTOR_H
#define HW_PCL_FUNCTOR_H

#include "hw_db_tree.h"
#include "hw_generic_global_vars.h"
#include "hw_point_cloud.h"
#include "hw_mesh.h"
#include "hw_tsdf.h"

namespace HW {



	class HWPCLFunctor
	{
	public:
		virtual void Process(HWObject* in_element, HWObject* out_element) = 0;
		virtual HWObject* NewProcess(HWObject* in_element, HWObject* out_element) { return nullptr; }
		//HWObject*
		HWObject* resulted_element_;
	};
}


#endif