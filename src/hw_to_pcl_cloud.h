#pragma once
#ifndef HW_TO_PCL_CLOUD_H
#define HW_TO_PCL_CLOUD_H

#include <pcl/surface/mls.h>
#include <pcl/PCLPointCloud2.h>

#include "hw_pcl_functor.h"

namespace HW
{
	class HWTOPCLCloud
	{
	public:
		//explicit HW2PCLReader(const HWObject* cc_cloud);
		explicit HWTOPCLCloud(HWPointCloud* hw_cloud);
		pcl::PCLPointCloud2::Ptr GetGenericField(std::string field_name) const;
		pcl::PCLPointCloud2::Ptr GetXYZ() const;
		pcl::PointCloud<pcl::PointXYZ>::Ptr GetXYZ2() const;
		pcl::PCLPointCloud2::Ptr GetNormals() const;
		pcl::PCLPointCloud2::Ptr GetColors() const;

		enum Fields { COORD_X = 0, COORD_Y, COORD_Z, NORM_X, NORM_Y, NORM_Z };
		pcl::PCLPointCloud2::Ptr GetOneOf(Fields field) const;
		//暂且不提出来
		pcl::PCLPointCloud2::Ptr GetFloatScalarField(const std::string& field_name) const;
		bool checkIfFieldExists(const std::string& field_name) const;
		pcl::PCLPointCloud2::Ptr GetAsHWPointCloud(std::list<std::string>& requested_fields) const;

		//! Converts all the data in a HWPointCloud to a pcl::PointCloud2
		/** This is useful for saving a HWPointCloud into a PCD file.
		For pcl filters other methods are suggested (to get only the necessary bits of data)
		**/
		pcl::PCLPointCloud2::Ptr GetAsHWPointCloud() const;

		static std::string GetSimplifiedSFName(const std::string& ccSfName);

	private:
		HWPointCloud* hw_cloud_;
	};
}

#endif
