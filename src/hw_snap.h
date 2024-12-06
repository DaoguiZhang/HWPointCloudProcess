#pragma once
#ifndef HW_SNAP
#define HW_SNAP

#include "hw_pcl_functor.h"
#include "hw_plane.h"
#include "PolygonWindow.h"
//#include "ceres/ceres.h"
//#include "glog/logging.h"

namespace HW
{
	class HWSNAP :public HWPCLFunctor
	{
		
	public:
		HWSNAP();
		~HWSNAP();

		void Process(HWObject* in_element, HWObject* out_element) override;

	private:
		//ªÒ»°plane_vec_
		HWPlane* GetPlaneObject(int i);
		void SetProcessType(ProcessType& my_type);
		ProcessType GetProcessType();

		ProcessType type_process_;
		PolygonWindow* widget;
		std::vector<HWPlane*> planes_vec_;
	};
}
#endif
