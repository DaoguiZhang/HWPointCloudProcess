#include "hw_mls.h"
#include "hw_to_pcl_cloud.h"
#include "pcl_to_hw_cloud.h"

//#define LP_PCL_PATCH_ENABLED

//scalar没有提出来，等其它弄好了，就把scalar那块提出来

namespace HW
{
	template <typename PointInT, typename PointOutT>
	int SmoothMLS(const typename pcl::PointCloud<PointInT>::Ptr &incloud,
		typename pcl::PointCloud<PointOutT>::Ptr &outcloud
#ifdef LP_PCL_PATCH_ENABLED
		, pcl::PointIndicesPtr &mapping_ids
#endif
	)
	{
		typename pcl::search::KdTree<PointInT>::Ptr tree(new pcl::search::KdTree<PointInT>);
		//typename pcl::search::Octree<PointInT>::Ptr tree(new pcl::search::Octree<PointInT>);
//#ifdef _OPENMP
//		//create the smoothing object
//		pcl::MovingLeastSquaresOMP< PointInT, PointOutT > smoother;
//		int n_threads = omp_get_max_threads();
//		smoother.setNumberOfThreads(n_threads);
//#else
		pcl::MovingLeastSquares< PointInT, PointOutT > smoother;
//#endif
		smoother.setInputCloud(incloud);
		printf("pcl point cloud count: %d\n", incloud->size());
		//pcl配置有错？
		smoother.setSearchMethod(tree);
		//设置参数
		smoother.setSearchRadius(HWParams::getInstance().mls_params_.search_radius_);
		smoother.setComputeNormals(HWParams::getInstance().mls_params_.compute_normal_flag_);
		if (HWParams::getInstance().mls_params_.use_polynomial_flag_)
		{
			int order = (int)HWParams::getInstance().mls_params_.polynomial_order_;
			smoother.setPolynomialOrder((int)HWParams::getInstance().mls_params_.polynomial_order_);
			smoother.setSqrGaussParam(HWParams::getInstance().mls_params_.gassian_param_sqrt_);
			
			/*printf("polynomial order: %d\n", order);
			printf("polynomial order: %f\n", HWParams::getInstance().mls_params_.polynomial_order_);
			printf("gauss sqaurt: %f\n", HWParams::getInstance().mls_params_.gassian_param_sqrt_);*/
		}
		
		/*printf("search_radius_: %f\n", HWParams::getInstance().mls_params_.search_radius_);
		if (HWParams::getInstance().mls_params_.compute_normal_flag_)
		{
			printf("true\n");
		}*/
		

	/*	if (HWParams::getInstance().mls_params_.polynomial_fit_)
		{
			smoother.setPolynomialOrder(HWParams::getInstance().mls_params_.polynomial_order_);
			smoother.setSqrGaussParam(params.sqr_gauss_param_);
		}*/

		switch (HWParams::getInstance().mls_params_.upsample_methods_)
		{
		case (kNone):
			{
				smoother.setUpsamplingMethod(pcl::MovingLeastSquares<PointInT, PointOutT>::NONE);
				printf("none method...\n");
				//no need to set other parameters here!
				break;
			}
		case (kSampleLocalPlane):
			{
				smoother.setUpsamplingMethod(pcl::MovingLeastSquares<PointInT, PointOutT>::SAMPLE_LOCAL_PLANE);
				smoother.setUpsamplingRadius(HWParams::getInstance().mls_params_.upsampling_radius_);
				smoother.setUpsamplingStepSize(HWParams::getInstance().mls_params_.upsampling_step_);
				break;
			}
		case (kRandomUniformDensity):
			{
				smoother.setUpsamplingMethod(pcl::MovingLeastSquares<PointInT, PointOutT>::RANDOM_UNIFORM_DENSITY);
				smoother.setPointDensity(HWParams::getInstance().mls_params_.step_point_density_);
				break;
			}
		case (kVoxelGridDilation):
			{
				smoother.setUpsamplingMethod(pcl::MovingLeastSquares<PointInT, PointOutT>::VOXEL_GRID_DILATION);
				smoother.setDilationVoxelSize(static_cast<float>(HWParams::getInstance().mls_params_.dilation_voxel_size_));
				smoother.setDilationIterations(HWParams::getInstance().mls_params_.dilation_iterations_);
				break;
			}
		}

		smoother.process(*outcloud);

#ifdef LP_PCL_PATCH_ENABLED
		mapping_ids = smoother.getCorrespondingIndices();
#endif
		return 1;
	}

	HWMLS::HWMLS()
	{
	}
	HWMLS::~HWMLS()
	{
	}

	void HWMLS::Process(HWObject* in_element, HWObject* out_element)
	{
		HWPointCloud *in_cloud = dynamic_cast<HWPointCloud *>(in_element);

		std::list<std::string> req_fields;
		try
		{
			req_fields.push_back("xyz"); // always needed
			//if (cloud->getCurrentDisplayedScalarField())
			//{
			//	//keep the current scalar field (if any)
			//	req_fields.push_back(cloud->getCurrentDisplayedScalarField()->getName());
			//}
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			printf("bad xyz...\n");
			return;
		}
		
		//take out the xyz info
		pcl::PCLPointCloud2::Ptr mls_pcl_cloud = HWTOPCLCloud(in_cloud).GetAsHWPointCloud(req_fields);
		if (!mls_pcl_cloud)
		{
			return;
		}
			
		//get as pcl point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		//FROM_PCL_CLOUD(*sm_cloud, *pcl_cloud);
		pcl::fromPCLPointCloud2(*mls_pcl_cloud, *pcl_cloud);

		//create storage for outcloud
		pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);

		//printf("storage....\n");

		//time
		clock_t start_time, end_time;
		start_time = clock();

		std::cout << "Start to MLS points..." << std::endl;

#ifdef LP_PCL_PATCH_ENABLED
		pcl::PointIndicesPtr mapping_indices;
		SmoothMLS<pcl::PointXYZ, pcl::PointNormal>(pcl_cloud, normals, mapping_indices);
#else
		SmoothMLS<pcl::PointXYZ, pcl::PointNormal>(pcl_cloud, normals);
#endif
		end_time = clock();
		std::cout << "MLS time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;
		std::cout << "End MLS..." << std::endl;

		pcl::PCLPointCloud2::Ptr mls_pcl_normals(new pcl::PCLPointCloud2);
		pcl::toPCLPointCloud2(*normals, *mls_pcl_normals);

		//printf("mls_pcl_normals size: %d\n", mls_pcl_normals->data.size());

		resulted_element_ = PCLTOHWCloud(mls_pcl_normals).GetCloud();

		resulted_element_->SetObjectType(in_element->GetObjectType());

		//test
		if (resulted_element_->GetObjectType() == kHWPointCloud)
		{
			printf("kHWPointCloud\n");
		}
		//end test

		////recompute the normal
		//HW::HWPointCloud* mls_pc = static_cast<HW::HWPointCloud*>(resulted_element_);
		//for (int i = 0; i < in_cloud->GetNormal().size(); ++i)
		//{
		//	if (dot(in_cloud->GetNormal()[i], mls_pc->GetNormal()[i]) < 0)
		//	{
		//		float3 tmp_point = make_float3(mls_pc->GetNormal()[i].x * -1, 
		//			mls_pc->GetNormal()[i].y * -1, mls_pc->GetNormal()[i].z * -1);
		//		mls_pc->SetAPoint(tmp_point, i);
		//	}
		//}

		out_element = resulted_element_;
		if (!out_element)
		{
			//conversion failed (not enough memory?)
			printf("not enough memory?\n");
			return;
		}
		//保存
		
		//printf("out point cloud size: %d\n", resulted_element_->GetVertices().size());
		//printf("out point cloud size: %d\n", out_element->GetVertices().size());
		//保存这个点云？
		//new_cloud->setName(cloud->getName() + QString("_smoothed")); //original name + suffix
		//new_cloud->setDisplay(cloud->getDisplay());

#ifdef LP_PCL_PATCH_ENABLED
		//copy the original scalar fields here
		copyScalarFields(cloud, new_cloud, mapping_indices, true);
		//copy the original colors here
		copyRGBColors(cloud, new_cloud, mapping_indices, true);
#endif

		////copy global shift & scale
		//new_cloud->setGlobalScale(cloud->getGlobalScale());
		//new_cloud->setGlobalShift(cloud->getGlobalShift());
		////disable original cloud
		//cloud->setEnabled(false);
		//if (cloud->getParent())
		//	cloud->getParent()->addChild(new_cloud);

		return;
	}
}