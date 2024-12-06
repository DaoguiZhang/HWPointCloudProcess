#pragma once
#ifndef PCL_TO_HW_CLOUD
#define PCL_TO_HW_CLOUD

//system
#include<list>

#include <pcl/surface/mls.h>
#include <pcl/PCLPointCloud2.h>

#include "hw_point_cloud.h"
#include "hw_pcl_functor.h"

namespace HW
{
	//! PCL to CC cloud converter
	/** NOTE: THIS METHOD HAS SOME PROBLEMS. IT CANNOT CORRECTLY LOAD NON-FLOAT FIELDS!
	THIS IS DUE TO THE FACT THE POINT TYPE WITH A SCALAR WE USE HERE IS FLOAT
	IF YOU TRY TO LOAD A FIELD THAT IS INT YOU GET A PCL WARN!
	**/
	class PCLTOHWCloud
	{
	public:

		//! Default constructor
		PCLTOHWCloud(pcl::PCLPointCloud2::Ptr hw_cloud);

		//! Converts input cloud (see constructor) to a ccPointCloud
		HWPointCloud* GetCloud();

		bool AddXYZ(HWPointCloud *cloud);
		bool AddNormals(HWPointCloud *cloud);
		bool AddRGB(HWPointCloud *cloud);
		//bool AddScalarField(HWPointCloud *cloud, const std::string& name, bool overwrite_if_exist = true);

	private:

		//! Associated PCL cloud
		pcl::PCLPointCloud2::Ptr pcl_cloud_;
	};
}

#endif