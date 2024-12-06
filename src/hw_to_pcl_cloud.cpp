#include "hw_to_pcl_cloud.h"

namespace HW
{
	HWTOPCLCloud::HWTOPCLCloud(HWPointCloud* hw_cloud)
	{
		hw_cloud_ = hw_cloud;
		assert(hw_cloud_);
	}

	pcl::PCLPointCloud2::Ptr HWTOPCLCloud::GetGenericField(std::string field_name) const
	{
		pcl::PCLPointCloud2::Ptr pcl_cloud;

		if (field_name == "x")
		{
			pcl_cloud = GetOneOf(COORD_X);
		}
		else if (field_name == "y")
		{
			pcl_cloud = GetOneOf(COORD_Y);
		}
		else if (field_name == "z")
		{
			pcl_cloud = GetOneOf(COORD_Y);
		}
		else if (field_name == "normal_x")
		{
			pcl_cloud = GetOneOf(NORM_X);
		}
		else if (field_name == "normal_y")
		{
			pcl_cloud = GetOneOf(NORM_Y);
		}
		else if (field_name == "normal_z")
		{
			pcl_cloud = GetOneOf(NORM_Z);
		}
		else if (field_name == "xyz")
		{
			pcl_cloud = GetXYZ();
		}
		else if (field_name == "normal_xyz")
		{
			pcl_cloud = GetNormals();
		}
		else if (field_name == "rgb")
		{
			pcl_cloud = GetColors();
		}
		return pcl_cloud;
	}

	pcl::PCLPointCloud2::Ptr HWTOPCLCloud::GetOneOf(Fields field) const
	{
		//assert(m_cc_cloud);

		pcl::PCLPointCloud2::Ptr pcl_cloud;

		std::string name;
		unsigned char dim = 0;
		switch (field)
		{
		case COORD_X:
			name = "x";
			dim = 0;
			break;
		case COORD_Y:
			name = "y";
			dim = 1;
			break;
		case COORD_Z:
			name = "z";
			dim = 2;
			break;
		case NORM_X:
			if (!hw_cloud_->HasNormal())
				return pcl_cloud;
			name = "normal_x";
			dim = 0;
			break;
		case NORM_Y:
			if (!hw_cloud_->HasNormal())
				return pcl_cloud;
			name = "normal_y";
			dim = 1;
			break;
		case NORM_Z:
			if (!hw_cloud_->HasNormal())
				return pcl_cloud;
			name = "normal_z";
			dim = 2;
			break;
		default:
			//unhandled field?!
			assert(false);
			return pcl_cloud;
		};

		assert(/*dim >= 0 && */dim <= 2);

		//try
		//{
		//	PointCloud<FloatScalar>::Ptr pcl_cloud(new PointCloud<FloatScalar>);

		//	unsigned pointCloud = m_cc_cloud->size();
		//	pcl_cloud->resize(pointCloud);

		//	for (unsigned i = 0; i < pointCloud; ++i)
		//	{
		//		switch (field)
		//		{
		//		case COORD_X:
		//		case COORD_Y:
		//		case COORD_Z:
		//		{
		//			const CCVector3* P = m_cc_cloud->getPoint(i);
		//			pcl_cloud->at(i).S5c4laR = static_cast<float>(P->u[dim]);
		//		}
		//		break;
		//		case NORM_X:
		//		case NORM_Y:
		//		case NORM_Z:
		//		{
		//			const CCVector3& N = m_cc_cloud->getPointNormal(i);
		//			pcl_cloud->at(i).S5c4laR = static_cast<float>(N.u[dim]);
		//		}
		//		break;
		//		default:
		//			//unhandled field?!
		//			assert(false);
		//			break;
		//		};
		//	}

		//	hw_cloud_ = PCLCloud::Ptr(new PCLCloud);
		//	TO_PCL_CLOUD(*pcl_cloud, *hw_cloud_);
		//	hw_cloud_->fields[0].name = name;
		//}
		//catch (...)
		//{
		//	//any error (memory, etc.)
		//	hw_cloud_.reset();
		//}

		return pcl_cloud;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr HWTOPCLCloud::GetXYZ2() const
	{
		assert(hw_cloud_);

		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		try
		{
			unsigned pointCount = hw_cloud_->GetVertices().size();
			pcl_cloud->resize(pointCount);

			for (unsigned i = 0; i < pointCount; ++i)
			{
				float3 p = hw_cloud_->GetAPoint(i);
				pcl_cloud->at(i).x = static_cast<float>(p.x);
				pcl_cloud->at(i).y = static_cast<float>(p.y);
				pcl_cloud->at(i).z = static_cast<float>(p.z);
			}
		}
		catch (...)
		{
			//any error (memory, etc.)
			pcl_cloud.reset();
		}

		return pcl_cloud;
	}

	pcl::PCLPointCloud2::Ptr HWTOPCLCloud::GetXYZ() const
	{
		pcl::PCLPointCloud2::Ptr mls_pcl_cloud;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud = GetXYZ2();
		if (pcl_cloud)
		{
			mls_pcl_cloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
			pcl::toPCLPointCloud2(*pcl_cloud, *mls_pcl_cloud);
			//TO_PCL_CLOUD(*pcl_cloud, *mls_pcl_cloud);
		}

		return mls_pcl_cloud;
	}

	pcl::PCLPointCloud2::Ptr HWTOPCLCloud::GetNormals() const
	{
		if (!hw_cloud_ || !hw_cloud_->HasNormal())
			return pcl::PCLPointCloud2::Ptr(static_cast<pcl::PCLPointCloud2*>(0));

		pcl::PCLPointCloud2::Ptr mls_pcl_cloud(new pcl::PCLPointCloud2);
		try
		{
			pcl::PointCloud<pcl::Normal>::Ptr pcl_cloud(new pcl::PointCloud<pcl::Normal>);

			unsigned pointCount = hw_cloud_->GetNormal().size();
			pcl_cloud->resize(pointCount);

			for (unsigned i = 0; i < pointCount; ++i)
			{
				float3 N = hw_cloud_->GetNormal()[i];
				pcl_cloud->at(i).normal_x = N.x;
				pcl_cloud->at(i).normal_y = N.y;
				pcl_cloud->at(i).normal_z = N.z;
			}

			pcl::toPCLPointCloud2(*pcl_cloud, *mls_pcl_cloud);
		}
		catch (...)
		{
			//any error (memory, etc.)
			mls_pcl_cloud.reset();
		}

		return mls_pcl_cloud;
	}

	pcl::PCLPointCloud2::Ptr HWTOPCLCloud::GetColors() const
	{
		if (!hw_cloud_ || !hw_cloud_->HasColor())
			return pcl::PCLPointCloud2::Ptr(static_cast<pcl::PCLPointCloud2*>(0));

		pcl::PCLPointCloud2::Ptr mls_pcl_cloud(new pcl::PCLPointCloud2);
		try
		{
			pcl::PointCloud<pcl::RGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::RGB>);

			unsigned pointCount = hw_cloud_->GetPointsColor().size();
			pcl_cloud->resize(pointCount);

			for (unsigned i = 0; i < pointCount; ++i)
			{
				const uchar3& rgb = hw_cloud_->GetPointsColor()[i];
				pcl_cloud->at(i).r = static_cast<uint8_t>(rgb.x);
				pcl_cloud->at(i).g = static_cast<uint8_t>(rgb.y);
				pcl_cloud->at(i).b = static_cast<uint8_t>(rgb.z);
			}

			pcl::toPCLPointCloud2(*pcl_cloud, *mls_pcl_cloud);
		}
		catch (...)
		{
			//any error (memory, etc.)
			mls_pcl_cloud.reset();
		}

		return mls_pcl_cloud;
	}

	bool HWTOPCLCloud::checkIfFieldExists(const std::string& field_name) const
	{
		if ((field_name == "x") || (field_name == "y") || (field_name == "z") || (field_name == "xyz"))
			return (hw_cloud_->GetVertices().size() != 0);

		else if ((field_name == "normal_x") || (field_name == "normal_y") || (field_name == "normal_z") || (field_name == "normal_xyz"))
			return hw_cloud_->HasNormal();

		else if (field_name == "rgb")
			return hw_cloud_->HasColor();
		return false;
		/*else
		return (m_cc_cloud->getScalarFieldIndexByName(field_name.c_str()) >= 0);*/
	}

	pcl::PCLPointCloud2::Ptr HWTOPCLCloud::GetAsHWPointCloud(std::list<std::string>& requested_fields) const
	{
		//preliminary check
		{
			for (std::list<std::string>::const_iterator it = requested_fields.begin(); it != requested_fields.end(); ++it)
			{
				bool exists = checkIfFieldExists(*it);
				if (!exists) //all check results must be true
					return pcl::PCLPointCloud2::Ptr(static_cast<pcl::PCLPointCloud2*>(0));
			}
		}
		
		//are we asking for x, y, and z all togheters?
		bool got_xyz = (std::find(requested_fields.begin(), requested_fields.end(), "xyz") != requested_fields.end());
		if (got_xyz)
		{
			//remove from the requested fields lists x y and z as single occurrencies
			requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("x")), requested_fields.end());
			requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("y")), requested_fields.end());
			requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("z")), requested_fields.end());
		}

		//same for normals
		bool got_normal_xyz = (std::find(requested_fields.begin(), requested_fields.end(), "normal_xyz") != requested_fields.end());
		if (got_normal_xyz)
		{
			requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("normal_x")), requested_fields.end());
			requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("normal_y")), requested_fields.end());
			requested_fields.erase(std::remove(requested_fields.begin(), requested_fields.end(), std::string("normal_z")), requested_fields.end());
		}

		//a vector for PointCloud2 clouds
		pcl::PCLPointCloud2::Ptr firstCloud;

		//load and merge fields/clouds one-by-one
		{
			for (std::list<std::string>::const_iterator it = requested_fields.begin(); it != requested_fields.end(); ++it)
			{
				if (!firstCloud)
				{
					firstCloud = GetGenericField(*it);
				}
				else
				{
					pcl::PCLPointCloud2::Ptr otherCloud = GetGenericField(*it);
					if (otherCloud)
					{
						pcl::PCLPointCloud2::Ptr mls_tmp(new pcl::PCLPointCloud2); //temporary cloud
						pcl::concatenateFields(*firstCloud, *otherCloud, *mls_tmp);
						firstCloud = mls_tmp;
					}
				}
			}
		}

		return firstCloud;
	}

	pcl::PCLPointCloud2::Ptr HWTOPCLCloud::GetAsHWPointCloud() const
	{
		//does the cloud have some points?
		if (!hw_cloud_ || hw_cloud_->GetVertices().size() == 0)
		{
			assert(false);
			return pcl::PCLPointCloud2::Ptr(static_cast<pcl::PCLPointCloud2*>(0));
		}

		//container
		std::list<std::string> fields;
		try
		{
			fields.push_back("xyz");
			if (hw_cloud_->HasNormal())
				fields.push_back("normal_xyz");
			if (hw_cloud_->HasColor())
				fields.push_back("rgb");
			/*for (unsigned i = 0; i < hw_cloud_->getNumberOfScalarFields(); ++i)
			fields.push_back(hw_cloud_->getScalarField(static_cast<int>(i))->getName());*/
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return pcl::PCLPointCloud2::Ptr(static_cast<pcl::PCLPointCloud2*>(0));
		}

		return GetAsHWPointCloud(fields);
	}
}