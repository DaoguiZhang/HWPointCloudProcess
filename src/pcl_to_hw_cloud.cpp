//PCL
#include <pcl/common/io.h>
#include <pcl/PCLPointField.h>
typedef pcl::PCLPointField PCLScalarField;

//system
#include <assert.h>

#include"pcl_to_hw_cloud.h"

namespace HW
{
	size_t GetNumberOfPoints(pcl::PCLPointCloud2::Ptr sm_cloud)
	{
		return static_cast<size_t>(sm_cloud ? sm_cloud->width * sm_cloud->height : 0);
	}

	bool ExistField(pcl::PCLPointCloud2::Ptr sm_cloud, std::string name)
	{
		if (sm_cloud)
			for (std::vector< PCLScalarField >::const_iterator it = sm_cloud->fields.begin(); it != sm_cloud->fields.end(); ++it)
				if (it->name == name)
					return true;

		return false;
	}

	//
	PCLTOHWCloud::PCLTOHWCloud(pcl::PCLPointCloud2::Ptr pcl_cloud)
		: pcl_cloud_(pcl_cloud)
	{
		assert(pcl_cloud_);
	}

	HWPointCloud* PCLTOHWCloud::GetCloud()
	{
		if (!pcl_cloud_)
		{
			assert(false);
			return 0;
		}

		//get the fields list
		std::list<std::string> fields;
		for (std::vector< PCLScalarField >::const_iterator it = pcl_cloud_->fields.begin(); it != pcl_cloud_->fields.end(); ++it)
		{
			if (it->name != "_") //PCL padding fields
				fields.push_back(it->name);
		}

		//begin with checks and conversions
		//be sure we have x, y, and z fields
		if (!ExistField(pcl_cloud_, "x") || !ExistField(pcl_cloud_, "y") || !ExistField(pcl_cloud_, "z"))
			return 0;

		//create cloud
		HWPointCloud* cloud = new HWPointCloud();

		//push points inside
		if (!AddXYZ(cloud))
		{
			delete cloud;
			return 0;
		}
		//printf("cloud: %d\n", cloud->GetVertices().size());
		//remove x,y,z fields from the vector of field names
		fields.remove("x");
		fields.remove("y");
		fields.remove("z");

		//do we have normals?
		if (ExistField(pcl_cloud_, "normal_x") || ExistField(pcl_cloud_, "normal_y") || ExistField(pcl_cloud_, "normal_z"))
		{
			AddNormals(cloud);

			//remove the corresponding fields
			fields.remove("normal_x");
			fields.remove("normal_y");
			fields.remove("normal_z");
		}

		//The same for colors
		if (ExistField(pcl_cloud_, "rgb"))
		{
			AddRGB(cloud);

			//remove the corresponding field
			fields.remove("rgb");
		}
		//The same for colors
		else if (ExistField(pcl_cloud_, "rgba"))
		{
			AddRGB(cloud);

			//remove the corresponding field
			fields.remove("rgba");
		}

		////All the remaining fields will be stored as scalar fields
		//for (std::list<std::string>::const_iterator name = fields.begin(); name != fields.end(); ++name)
		//{
		//	AddScalarField(cloud, *name);
		//}

		//printf("cloud11: %d\n", cloud->GetVertices().size());

		return cloud;
	}

	bool PCLTOHWCloud::AddXYZ(HWPointCloud *cloud)
	{
		assert(pcl_cloud_ && cloud);
		if (!pcl_cloud_ || !cloud)
			return false;

		size_t pointCount = GetNumberOfPoints(pcl_cloud_);
		//printf("pcl_cloud_ size: %d\n", pointCount);

		/*if (!cloud->(static_cast<unsigned>(pointCount)))
			return false;*/

		//add xyz to the given cloud taking xyz infos from the sm cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		//FROM_PCL_CLOUD(*pcl_cloud_, *pcl_cloud);
		pcl::fromPCLPointCloud2(*pcl_cloud_, *pcl_cloud);

		//loop
		for (size_t i = 0; i < pointCount; ++i)
		{
			float3 tmp_point;
			tmp_point.x = pcl_cloud->at(i).x;
			tmp_point.y = pcl_cloud->at(i).y;
			tmp_point.z = pcl_cloud->at(i).z;

			cloud->AddPoint(tmp_point);
		}

		return true;
	}

	bool PCLTOHWCloud::AddNormals(HWPointCloud *cloud)
	{
		assert(pcl_cloud_ && cloud);
		if (!pcl_cloud_ || !cloud)
			return false;

		pcl::PointCloud<pcl::Normal>::Ptr pcl_cloud_normals(new pcl::PointCloud<pcl::Normal>);
		//FROM_PCL_CLOUD(*pcl_cloud_, *pcl_cloud_normals);
		pcl::fromPCLPointCloud2(*pcl_cloud_, *pcl_cloud_normals);

		/*if (!cloud->reserveTheNormsTable())
			return false;*/

		size_t pointCount = GetNumberOfPoints(pcl_cloud_);

		//loop
		for (size_t i = 0; i < pointCount; ++i)
		{
			float3 tmp_normal;
			tmp_normal.x = static_cast<float>(pcl_cloud_normals->at(i).normal_x);
			tmp_normal.y = static_cast<float>(pcl_cloud_normals->at(i).normal_y);
			tmp_normal.z = static_cast<float>(pcl_cloud_normals->at(i).normal_z);

			cloud->AddNormal(tmp_normal);
		}
		//cloud->showNormals(true);
		return true;
	}

	bool PCLTOHWCloud::AddRGB(HWPointCloud * cloud)
	{
		assert(pcl_cloud_ && cloud);
		if (!pcl_cloud_ || !cloud)
			return false;

		pcl::PointCloud<pcl::RGB>::Ptr pcl_cloud_rgb(new pcl::PointCloud<pcl::RGB>);
		//FROM_PCL_CLOUD(*pcl_cloud_, *pcl_cloud_rgb);
		pcl::fromPCLPointCloud2(*pcl_cloud_, *pcl_cloud_rgb);

		/*if (!cloud->reserveTheRGBTable())
			return false;*/

		size_t pointCount = GetNumberOfPoints(pcl_cloud_);

		//loop
		for (size_t i = 0; i < pointCount; ++i)
		{
			uchar3 tmp_color;
			tmp_color.x = static_cast<uchar>(pcl_cloud_rgb->points[i].r);
			tmp_color.y = static_cast<uchar>(pcl_cloud_rgb->points[i].g);
			tmp_color.z = static_cast<uchar>(pcl_cloud_rgb->points[i].b);

			cloud->AddColor(tmp_color);
		}
		//cloud->showColors(true);
		return true;
	}
}