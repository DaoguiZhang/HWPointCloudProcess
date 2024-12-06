#include <fstream>
#include <iostream>
#include <algorithm>

#include "hw_point_cloud.h"
#include "tinyply.h"
#include "rply.h"


namespace HW
{
	HWPointCloud::HWPointCloud()
	{
		SetObjectType(kHWPointCloud);
	}
	HWPointCloud::HWPointCloud(const std::vector<float3>& in_points)
	{
		points_num_ = in_points.size();
		points_.resize(points_num_);
		for (int i = 0; i < in_points.size(); ++i)
		{
			points_[i] = in_points[i];
		}
		SetObjectType(kHWPointCloud);
	}

	HWPointCloud::HWPointCloud(const std::vector<float3>& in_points, const std::vector<float3>& in_normal)
	{
		points_num_ = in_points.size();
		points_.resize(points_num_);
		for (int i = 0; i < in_points.size(); ++i)
		{
			points_[i] = in_points[i];
		}
		normal_.resize(in_normal.size());
		for (int i = 0; i < in_normal.size(); ++i)
		{
			normal_[i] = in_normal[i];
		}
	}
	HWPointCloud::~HWPointCloud()
	{}
	
	bool HWPointCloud::HasNormal()
	{
		if (!normal_.empty() && normal_.size() == points_.size())
			return true; 
		else
			return false;
	}

	bool HWPointCloud::HasColor()
	{
		if (!points_color_.empty() && points_color_.size() == points_.size())
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void HWPointCloud::CopyPointsFromPlyData(std::shared_ptr<tinyply::PlyData>& point_element)
	{
		const size_t num_vertices_bytes = point_element->buffer.size_bytes();
		points_.resize(point_element->count);
		std::memcpy(points_.data(), point_element->buffer.get(), num_vertices_bytes);
		points_num_ = point_element->count;
	}

	void HWPointCloud::CopyNormalsFromPlyData(std::shared_ptr<tinyply::PlyData>& normal_element)
	{
		const size_t num_normal_bytes = normal_element->buffer.size_bytes();
		normal_.resize(normal_element->count);
		std::memcpy(normal_.data(), normal_element->buffer.get(), num_normal_bytes);
	}

	void HWPointCloud::CopyColorsFromPlyData(std::shared_ptr<tinyply::PlyData>& color_element)
	{
		const size_t num_color_bytes = color_element->buffer.size_bytes();
		points_color_.resize(color_element->count);
		std::memcpy(points_color_.data(), color_element->buffer.get(), num_color_bytes);
	}

	void HWPointCloud::AddPoint(float3& point)
	{
		points_.emplace_back(point);
	}

	void HWPointCloud::SetPoint(const std::vector<float3>& v)
	{
		points_ = v;
	}

	void HWPointCloud::AddNormal(float3& point_normal)
	{
		normal_.emplace_back(point_normal);
	}

	void HWPointCloud::SetNormal(const std::vector<float3>& v)
	{
		normal_ = v;
	}

	void HWPointCloud::AddColor(uchar3& point_color)
	{
		points_color_.emplace_back(point_color);
	}

	void HWPointCloud::SetColor(uchar3 & point_color, int idx)
	{
		points_color_[idx] = point_color;	
	}

	void HWPointCloud::SetColor(const std::vector<uchar3>& v)
	{
		points_color_ = v;
	}

	void HWPointCloud::SetSemantic(const std::vector<int>& sem)
	{
		points_semantic_label_.clear();
		points_semantic_label_.resize(sem.size());
		for (int i = 0; i < sem.size(); i++) {
			points_semantic_label_[i] = sem[i];
		}
	}

	void HWPointCloud::SetPlanesIdx(std::vector<int>& planes_idx)
	{
		planes_idx_.assign(planes_idx.begin(), planes_idx.end());
	}

	void HWPointCloud::SetPlanesIsWide(std::vector<int>& planes_iswide)
	{
		planes_iswide_.assign(planes_iswide.begin(), planes_iswide.end());
	}

	void HWPointCloud::SetBoundingBox(float3 & box_min, float3 & box_max)
	{
		box_min_ = box_min;
		box_max_ = box_max;
	}

	float3 HWPointCloud::GetAPoint(int idx)
	{
		if (idx < 0 || idx >= points_.size())
		{
			printf("point cloud out of range...\n");
			return float3();
		}
		else
		{
			return points_[idx];
		}
	}

	float3 HWPointCloud::GetANormal(int idx)
	{
		if (idx < 0 || idx >= normal_.size())
		{
			printf("point cloud out of range...\n");
			return float3();
		}
		else
		{
			return normal_[idx];
		}
	}

	uchar3 HWPointCloud::GetAColor(int idx)
	{
		if (idx < 0 || idx >= points_color_.size())
		{
			printf("point cloud out of range...\n");
			return uchar3();
		}
		else
		{
			return points_color_[idx];
		}
	}

	bool HWPointCloud::ReadPly(const std::string& file)
	{
		//将ReadPly这一块读入变得general，这样不会出现因为格式得小问题，出现闪退现象
		std::ifstream fhd(file, std::ios::binary);
		if (fhd.fail()) throw std::runtime_error("failed to open " + file);
		//std::cout << "test" << std::endl;
		//system("pause");
		tinyply::PlyFile file_ply;
		file_ply.parse_header(fhd);

		bool has_vertex_flag = false;
		bool has_normal_flag = false;
		bool has_color_flag = false;
		bool has_alpha_flag = false;
		bool has_faces_flag = false;

		std::cout << "........................................................................\n";
		for (auto c : file_ply.get_comments()) std::cout << "Comment: " << c << std::endl;

		for (auto e : file_ply.get_elements())
		{
			std::cout << "element - " << e.name << " (" << e.size << ")" << std::endl;
			if (e.name == "face" && e.size > 0)
				has_faces_flag = true;

			for (auto p : e.properties)
			{
				if (p.name == "x")
					has_vertex_flag = true;
				if (p.name == "nx")
					has_normal_flag = true;
				if (p.name == "red")
					has_color_flag = true;
				if (p.name == "alpha")
					has_alpha_flag = true;

				std::cout << "\tproperty - " << p.name << " (" << tinyply::PropertyTable[p.propertyType].str << ")" << std::endl;
			}
		}
		std::cout << "........................................................................\n";

		//从文件头判断ply里面得元素
		std::shared_ptr<tinyply::PlyData> face_info, point_vertices, point_normal, point_color;

		//通过分析头文件，可以分开读取ply文件
		if (!has_faces_flag)
		{
			//读取点云
			if (has_vertex_flag)
			{
				try
				{
					point_vertices = file_ply.request_properties_from_element("vertex", { "x", "y", "z" });
				}
				catch (const std::exception & e)
				{
					std::cerr << "tinyply exception: " << e.what() << std::endl;
					return false;
				}
			}
			else
			{
				std::cout << "read ply vertex failed." << std::endl;
				return false;
			}

			if (has_normal_flag)
			{
				try
				{
					point_normal = file_ply.request_properties_from_element("vertex", { "nx", "ny", "nz" });
				}
				catch (const std::exception & e)
				{
					std::cerr << "tinyply exception: " << e.what() << std::endl;
					return false;
				}
			}

			if (has_color_flag)
			{
				if (!has_alpha_flag)
				{
					try
					{
						point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue" });
					}
					catch (const std::exception & e)
					{
						std::cerr << "tinyply exception: " << e.what() << std::endl;
						return false;
					}
				}
				else
				{
					try
					{
						point_color = file_ply.request_properties_from_element("vertex", { "red", "green", "blue", "alpha" });
					}
					catch (const std::exception & e)
					{
						std::cerr << "tinyply exception: " << e.what() << std::endl;
						return false;
					}
				}
			}

			//将主体读入到file_ply中
			file_ply.read(fhd);

			if (point_vertices) std::cout << "\tRead " << point_vertices->count << " total vertices " << std::endl;
			if (point_normal) std::cout << "\tRead " << point_normal->count << " total vertex normals " << std::endl;
			if (point_color) std::cout << "\tRead " << point_color->count << " total vertex color " << std::endl;

			//把ply读取的vertex拷贝到HWPointCloud
			CopyPointsFromPlyData(point_vertices);
			if (has_normal_flag)
				CopyNormalsFromPlyData(point_normal);
			//std::cout << "asdfsadfs" << std::endl;
			//std::system("pause");

			if (has_alpha_flag)
			{
				std::vector<uchar4> temp_color_vec;
				const size_t num_color_bytes = point_color->buffer.size_bytes();
				temp_color_vec.resize(point_color->count);
				std::memcpy(temp_color_vec.data(), point_color->buffer.get(), num_color_bytes);

				for (int i = 0; i < temp_color_vec.size(); ++i)
				{
					uchar3 one_point_color = make_uchar3(temp_color_vec[i].x, temp_color_vec[i].y, temp_color_vec[i].z);
					//system("pause");
					AddColor(one_point_color);
				}
			}
			else if (has_color_flag)
			{
				CopyColorsFromPlyData(point_color);
			}
			return true;
		}
		else
		{
			return false;
		}
	}

	void HWPointCloud::AddRandoNormNoise()
	{
		if (!points_.empty())
		{
			for (int i = 0; i < points_.size(); ++i)
			{
				float noise_value = (std::rand() - RAND_MAX/2) / double(RAND_MAX);
				//std::cout << "noise: " << noise_value << std::endl;
				points_[i].x += noise_value * normal_[i].x * 0.05;
				noise_value = (std::rand() - RAND_MAX / 2) / double(RAND_MAX);
				points_[i].y += noise_value * normal_[i].y * 0.05;
				noise_value = (std::rand() - RAND_MAX / 2) / double(RAND_MAX);
				points_[i].z += noise_value * normal_[i].z * 0.05;

				noise_value = (std::rand() - RAND_MAX / 2) / double(RAND_MAX);
				normal_[i].x += noise_value * 0.2;
				noise_value = (std::rand() - RAND_MAX / 2) / double(RAND_MAX);
				normal_[i].y += noise_value * 0.2;
				noise_value = (std::rand() - RAND_MAX / 2) / double(RAND_MAX);
				normal_[i].z += noise_value * 0.2;

				normalize(normal_[i]);
			}
		}
	}

	const std::vector<float3>& HWPointCloud::GetVertices()
	{
		return points_;
	}

	const std::vector<float3>& HWPointCloud::GetNormal()
	{
		return normal_;
	}

	const std::vector<uchar3>& HWPointCloud::GetPointsColor()
	{
		return points_color_;
	}
	const std::vector<int>& HWPointCloud::GetPointsSemanticLabel()
	{
		// TODO: ????? return ??
		return points_semantic_label_;
	}
	int HWPointCloud::GetSemanticLabel()
	{
		std::unordered_map<int, int> labels;
		for (int label : points_semantic_label_) {
			labels[points_semantic_label_[label]]++;
		}

		int most_label = 0;
		int most_label_count = 0;
		for (const auto& label : labels) {
			if (label.second > most_label_count) {
				most_label_count = label.second;
				most_label = label.first;
			}
		}

		return most_label;
	}
	const std::vector<int>& HWPointCloud::GetPlanesIdx()
	{
		return planes_idx_;
	}

	const std::vector<int>& HWPointCloud::GetPlanesIsWide()
	{
		return planes_iswide_;
	}

	void HWPointCloud::getBoundingBox(float3 & box_min, float3 & box_max)
	{
		box_min = box_min_;
		box_max = box_max_;
	}

	bool HWPointCloud::SetAPoint(float3 in_point, int idx)
	{
		if (idx < 0 || idx >= points_.size())
		{
			printf("point cloud out of range...\n");
			return false;
		}
		points_[idx].x = in_point.x;
		points_[idx].y = in_point.y;
		points_[idx].z = in_point.z;
		return true;
	}

	bool HWPointCloud::Show()
	{
		return true;
	}

	bool HWPointCloud::Save(std::string& file)
	{	
		return true;
	}

	bool HWPointCloud::SavePly(const std::string& file, const PlyFormat& type)
	{

		//test
		//AddRandoNormNoise();
		//end test

		bool write_ascii = false;
		if (type == PlyFormat::kAscci)
		{
			write_ascii = true;
		}

		p_ply ply_file = ply_create(file.c_str(),
			write_ascii ? PLY_ASCII : PLY_LITTLE_ENDIAN, NULL, 0, NULL);
		if (!ply_file) {
			printf("Write PLY failed: unable to open file: %s\n", file.c_str());
			return false;
		}
		ply_add_comment(ply_file, "Created by HW3D");
		ply_add_element(ply_file, "vertex",
			static_cast<long>(points_.size()));
		ply_add_property(ply_file, "x", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		ply_add_property(ply_file, "y", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		ply_add_property(ply_file, "z", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		if (this->HasNormal()) {
			ply_add_property(ply_file, "nx", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
			ply_add_property(ply_file, "ny", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
			ply_add_property(ply_file, "nz", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		}
		if (this->HasColor()) {
			//printf()
			ply_add_property(ply_file, "red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
			ply_add_property(ply_file, "green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
			ply_add_property(ply_file, "blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
			ply_add_property(ply_file, "alpha", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
		}
		if (!ply_write_header(ply_file)) {
			printf("Write PLY failed: unable to write header.\n");
			ply_close(ply_file);
			return false;
		}

		for (size_t i = 0; i < points_.size(); i++)
		{
			ply_write(ply_file, (float)points_[i].x);
			ply_write(ply_file, (float)points_[i].y);
			ply_write(ply_file, (float)points_[i].z);
			if (this->HasNormal())
			{
				ply_write(ply_file, (float)normal_[i].x);
				ply_write(ply_file, (float)normal_[i].y);
				ply_write(ply_file, (float)normal_[i].z);
			}
			if (this->HasColor())
			{
				ply_write(ply_file, (unsigned char)std::min(255.0, std::max(0.0,
					points_color_[i].x * 1.0)));
				ply_write(ply_file, (unsigned char)std::min(255.0, std::max(0.0,
					points_color_[i].y * 1.0)));
				ply_write(ply_file, (unsigned char)std::min(255.0, std::max(0.0,
					points_color_[i].z * 1.0)));
				ply_write(ply_file, (unsigned char)std::min(255.0, std::max(0.0,
					0.0 * 1.0)));
			}
		}

		ply_close(ply_file);

        printf("saved ply file to %s.\n", file.c_str());

		return true;
	}

	void HWPointCloud::AllocateInfoSpace(int num)
	{
		ave_normals_.resize(num);
		ave_points_.resize(num);
		plane_normals_.resize(num);
		world_to_plane_.resize(num);
		bounding_box_mins_.resize(num);
		bounding_box_maxs_.resize(num);
	}

	void HWPointCloud::RemoveInfoSpace()
	{
		ave_normals_.clear();
		ave_points_.clear();
		plane_normals_.clear();
		world_to_plane_.clear();
		bounding_box_mins_.clear();
		bounding_box_maxs_.clear();
	}

	void HWPointCloud::InitGeometricInfo()
	{
		std::cout << "begin InotGeometricInfo" << std::endl;
		int num_plane = planes_idx_.size() - 1;
		AllocateInfoSpace(num_plane);
		for (int i = 0; i < num_plane; ++i) {
			ComputeSingleGeometricInfo(i);
		}
		std::cout << "end InotGeometricInfo" << std::endl;
	}

	void HWPointCloud::Merge(HWPointCloud& pc)
	{
		points_num_ += pc.points_num_;
		points_.insert(points_.end(), pc.points_.begin(), pc.points_.end());
		points_color_.insert(points_color_.end(), pc.points_color_.begin(), pc.points_color_.end());
		points_semantic_label_.insert(points_semantic_label_.end(), pc.points_semantic_label_.begin(), pc.points_semantic_label_.end());
		normal_.insert(normal_.end(), pc.normal_.begin(), pc.normal_.end());
		//planes_idx_
		int last_idx = planes_idx_.back();
		int original_length = planes_idx_.size();
		planes_idx_.insert(planes_idx_.end(), pc.planes_idx_.begin() + 1, pc.planes_idx_.end());
		for (int i = original_length; i != planes_idx_.size(); ++i) {
			planes_idx_[i] += last_idx;
		}
		planes_iswide_.insert(planes_iswide_.end(), pc.planes_iswide_.begin(), pc.planes_iswide_.end());

		//box_min
		box_min_ = make_float3(
			fmin(box_min_.x, pc.box_min_.x),
			fmin(box_min_.y, pc.box_min_.z),
			fmin(box_min_.y, pc.box_min_.z)
		);
		//box_max
		box_max_ = make_float3(
			fmax(box_max_.x, pc.box_max_.x),
			fmax(box_max_.y, pc.box_max_.y),
			fmax(box_max_.z, pc.box_max_.z)
		);
	}

	float HWPointCloud::ComputePointCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
	{
		//
		if (cloud->empty())
			return 0.0;

		float res = 0.0;
		int n_points = 0.0;
		int nres;
		std::vector<int> indices(2);
		std::vector<float> sqr_dist(2);
		pcl::search::KdTree<pcl::PointXYZ> tree;
		tree.setInputCloud(cloud);
		for (int i = 0; i < cloud->size(); ++i)
		{
			if (!pcl_isfinite((*cloud)[i].x))
				continue;
			//consider the second neigbor
			nres = tree.nearestKSearch(i, 2, indices, sqr_dist);
			if (nres == 2)
			{
				res += std::sqrt(sqr_dist[1]);
				++n_points;
			}
		}
		if (n_points != 0)
			res /= n_points;
		return res;
	}

	int HWPointCloud::ComputePointsNumPerSquareMeter()
	{
		std::cout << "start compute point resolution..." << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		p_cloud->resize(points_.size());
		for (int i = 0; i < points_.size(); ++i)
		{
			//printf("%d\n", i);
			p_cloud->points[i].x = points_[i].x;
			p_cloud->points[i].y = points_[i].y;
			p_cloud->points[i].z = points_[i].z;
		}
		//获取点云的密度
		std::cout << "the computed point resolution size is：" << points_.size() << std::endl;
		float cloud_resolution = ComputePointCloudResolution(p_cloud);
		std::cerr << "the pointcloud resolution is: " << cloud_resolution << std::endl;
		if (std::abs(cloud_resolution) < 1e-5 || cloud_resolution < 0.0)
		{
			cloud_resolution = 1e-5;
		}
		int points_cloud_num = static_cast<int>(1.0 / (cloud_resolution * cloud_resolution));
		return  points_cloud_num;
	}

	Eigen::Matrix3f HWPointCloud::ComputeCovarianceMatrix(int id, int start_idx, int end_idx)
	{
		if (start_idx == end_idx) return Eigen::Matrix3f();

		Eigen::Matrix3f covariance_matrix;
		Eigen::Vector3f ave_point(ave_points_[id].x, ave_points_[id].y, ave_points_[id].z);
		float num_points = end_idx - start_idx;

		for (int i = start_idx; i != end_idx; ++i) {
			Eigen::Vector3f point(points_[i].x, points_[i].y, points_[i].z);
			Eigen::Matrix3f multiplied_matrix = (point - ave_point) * (point - ave_point).transpose();
			covariance_matrix += multiplied_matrix;
		}
		covariance_matrix = covariance_matrix / num_points;
		return covariance_matrix;
	}

	void HWPointCloud::ComputeAvePointAndNormal(int id, int start_idx, int end_idx)
	{
		if (start_idx == end_idx) return;

		float3 point = make_float3(0, 0, 0);
		float3 normal = make_float3(0, 0, 0);
		for (int i = start_idx; i != end_idx; ++i) {
			point += points_[i];
			normal += normal_[i];
		}
		float inverse_num = 1.0f / float(end_idx - start_idx);
		point *= inverse_num;
		normal *= inverse_num;
		ave_points_[id] = point;
		ave_normals_[id] = normal;
	}

	void HWPointCloud::ComputeBoundingBox(int id, int start_idx, int end_idx)
	{
		if (start_idx == end_idx) return;

		float3 min_point = make_float3(FLT_MAX);
		float3 max_point = make_float3(-FLT_MAX);
		for (int i = start_idx; i != end_idx; ++i) {
			float3 curr = points_[i];
			min_point.x = fmin(min_point.x, curr.x);
			min_point.y = fmin(min_point.y, curr.y);
			min_point.z = fmin(min_point.z, curr.z);

			max_point.x = fmax(max_point.x, curr.x);
			max_point.y = fmax(max_point.y, curr.y);
			max_point.z = fmax(max_point.z, curr.z);
		}
		bounding_box_mins_[id] = min_point;
		bounding_box_maxs_[id] = max_point;
	}

	void HWPointCloud::ComputeWorldToPlaneMatrix(int id, int start_idx, int end_idx)
	{
		Eigen::Matrix3f covariance_matrix = ComputeCovarianceMatrix(id, start_idx, end_idx);
		Eigen::Matrix3f eigen_values = Eigen::Matrix3f::Zero(3, 3);
		Eigen::Matrix3f eigen_vectors = Eigen::Matrix3f::Zero(3, 3);
		Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);
		eigen_values = es.pseudoEigenvalueMatrix();
		eigen_vectors = es.pseudoEigenvectors();

		//std::cout << "the eigen matrix value is: \n" << eigen_values << std::endl;
		//std::cout << "the eigen matrix vector is: \n" << eigen_vector << std::endl;

		int min_eigen_idx = -1;
		int max_eigen_idx = -1;
		float min_value = FLT_MAX;
		float max_value = -FLT_MAX;

		for (int i = 0; i < 3; ++i)
		{
			if (min_value > eigen_values(i, i))
			{
				min_value = eigen_values(i, i);
				min_eigen_idx = i;
			}

			if (max_value < eigen_values(i, i))
			{
				max_value = eigen_values(i, i);
				max_eigen_idx = i;
			}
		}

		//average_pnts_normal_ = ComputeAverageVertexNormal();
		Eigen::Vector3f PCA_normal = eigen_vectors.col(min_eigen_idx);
		Eigen::Vector3f ave_normal(ave_normals_[id].x, ave_normals_[id].y, ave_normals_[id].z);
		Eigen::Matrix3f r_matrix;
		r_matrix.col(0) = eigen_vectors.col(max_eigen_idx);
		if (PCA_normal.dot(ave_normal) > 0)
			r_matrix.col(2) = PCA_normal;
		else r_matrix.col(2) = -PCA_normal;

		Eigen::Vector3f plane_normal = r_matrix.col(2);
		plane_normals_[id] = make_float3(plane_normal(0), plane_normal(1), plane_normal(2));

		//???????
		r_matrix.col(1) = r_matrix.col(0).cross(r_matrix.col(2));

		float3 ave_point = ave_points_[id];
		Eigen::Vector3f t_matrix(ave_point.x, ave_point.y, ave_point.z);

		Eigen::Matrix4f plane_to_world;
		plane_to_world.topLeftCorner(3, 3) = r_matrix;
		plane_to_world(0, 3) = t_matrix(0);
		plane_to_world(1, 3) = t_matrix(1);
		plane_to_world(2, 3) = t_matrix(2);
		plane_to_world(3, 3) = 1;
		plane_to_world.row(3) = Eigen::RowVector4f(0, 0, 0, 1);

		world_to_plane_[id] = plane_to_world.inverse();
	}

	void HWPointCloud::ComputeSingleGeometricInfo(int id)
	{
		if (id >= planes_idx_.size() - 1) {
			std::cerr << "error, ComputeSingleGeometricInfo, id >= planes_idx_.size() - 1" << std::endl;
			return;
		}

		int first = planes_idx_[id];
		int last = planes_idx_[id + 1];
		ComputeAvePointAndNormal(id, first, last);
		ComputeBoundingBox(id, first, last);
		ComputeWorldToPlaneMatrix(id, first, last);
	}

	bool HWPointCloud::ShouldMerge(const HWPointCloud & c1, const HWPointCloud & c2, int a, int b)
	{
		printf("merge %d and %d\n", a, b);
		float3 ave_point1 = c1.ave_points_[a];
		float3 plane_normal1 = c1.plane_normals_[a];
		float3 min_corner1 = c1.bounding_box_mins_[a];
		float3 max_corner1 = c1.bounding_box_maxs_[a];

		float3 ave_point2 = c2.ave_points_[b];
		float3 plane_normal2 = c2.plane_normals_[b];
		float3 min_corner2 = c2.bounding_box_mins_[b];
		float3 max_corner2 = c2.bounding_box_maxs_[b];

		//?????????8?
		if (abs(dot(plane_normal1, plane_normal2)) < 0.995) return false;
		std::cout << "normal" << std::endl;

		float3 p1p2 = ave_point2 - ave_point1;
		//???????????????5cm
		if (abs(dot(p1p2, plane_normal1)) > 0.05) return false;
		std::cout << "distance" << std::endl;

		float3 diagnal1 = max_corner1 - min_corner1;
		float3 diagnal2 = max_corner2 - min_corner2;
		float volume1 = sqrt(dot(diagnal1, diagnal1));
		float volume2 = sqrt(dot(diagnal2, diagnal2));
		float min_volume = fmin(volume1, volume2);

		float3 min_corner = make_float3(
			fmax(min_corner1.x, min_corner2.x),
			fmax(min_corner1.y, min_corner2.y),
			fmax(min_corner1.z, min_corner2.z)
		);
		float3 max_corner = make_float3(
			fmin(max_corner1.x, max_corner1.x),
			fmin(max_corner1.y, max_corner1.y),
			fmin(max_corner1.z, max_corner1.z)
		);

		float3 diagnal = max_corner - min_corner;
		if (diagnal.x < 0 || diagnal.y < 0 || diagnal.z < 0) return false;
		std::cout << "diagnal" << std::endl;

		float volume = sqrt(dot(diagnal, diagnal));
		//?????????
		if (volume < 0.5 * min_volume) return false;
		std::cout << "volume" << std::endl;


		//printf("plane_normal1 = %f, %f, %f\n", plane_normal1.x, plane_normal1.y, plane_normal1.z);
		//printf("plane_normal2 = %f, %f, %f\n", plane_normal2.x, plane_normal2.y, plane_normal2.z);
		//std::cout << "abs(dot(plane_normal1, plane_normal2) = " << abs(dot(plane_normal1, plane_normal2)) << std::endl;

		//printf("ave_point1 = %f, %f, %f\n", ave_point1.x, ave_point1.y, ave_point1.z);
		//printf("ave_point2 = %f, %f, %f\n", ave_point1.x, ave_point1.y, ave_point1.z);
		//printf("p1p2 = %f, %f, %f\n", p1p2.x, p1p2.y, p1p2.z);

		//std::cout << "min_corner1 = " << min_corner1.x << ", " << min_corner1.y << ", " << min_corner1.z << std::endl;
		//std::cout << "min_corner2 = " << min_corner2.x << ", " << min_corner2.y << ", " << min_corner2.z << std::endl;
		//std::cout << "min_corner = " << min_corner.x << ", " << min_corner.y << ", " << min_corner.z << std::endl;

		//std::cout << "max_corner1 = " << max_corner1.x << ", " << max_corner1.y << ", " << max_corner1.z << std::endl;
		//std::cout << "max_corner2 = " << max_corner1.x << ", " << max_corner2.y << ", " << max_corner2.z << std::endl;
		//std::cout << "max_corner = " << max_corner.x << ", " << max_corner.y << ", " << max_corner.z << std::endl;

		//std::cout << "diagnal = " << diagnal.x << ", " << diagnal.y << ", " << diagnal.z << std::endl;
		//std::cout << "volume = " << volume << std::endl;

		return true;
	}

	void MergeClutteredHWPointCloud(HWPointCloud& target, HWPointCloud& c1, HWPointCloud& c2)
	{
		std::cout << "begin MergeHWPointCloud" << std::endl;
		const std::vector<float3> &points1 = c1.points_;
		const std::vector<float3> &points2 = c2.points_;
		if (points1.empty() || points2.empty()) {
			std::cerr << "no points to merge" << std::endl;
			return;
		}
		if (points1.size() < points2.size()) {
			MergeClutteredHWPointCloud(target, c2, c1);
			return;
		}
		c1.InitGeometricInfo();
		c2.InitGeometricInfo();

		const std::vector<float3>& normals1 = c1.normal_;
		const std::vector<float3>& normals2 = c2.normal_;
		std::cout << "point1.size() = " << points1.size() << ", point2.size() = " << points2.size() << std::endl;

		int num_plane1 = c1.planes_idx_.size() - 1;
		int num_plane2 = c2.planes_idx_.size() - 1;
		if (num_plane1 == 0 || num_plane2 == 0) return;
		std::cout << "num_plane1 = " << num_plane1 << ", num_plane2 = " << num_plane2 << std::endl;

		std::vector<bool> is_merged(points2.size(), false);
		std::vector<int> idxs{ 0 };
		std::vector<float3> points;
		std::vector<float3> normals;
		std::vector<uchar3> colors;

		std::cout << "merge points2 to points1" << std::endl;
		for (int i = 0; i < num_plane1; ++i) {
			int idx = idxs.back();
			std::cout << "i = " << i << ", idx = " << idx << std::endl;
			ccColor::Rgb col = ccColor::Generator::Random();

			int start_idx1 = c1.planes_idx_[i];
			int end_idx1 = c1.planes_idx_[i + 1];
			int point_num1 = end_idx1 - start_idx1;

			points.insert(points.end(), points1.begin() + start_idx1, points1.begin() + end_idx1);
			if (!c1.normal_.empty() && !c2.normal_.empty()) {
				normals.insert(normals.end(), normals1.begin() + start_idx1, normals1.begin() + end_idx1);
			}
			colors.insert(colors.end(), point_num1, make_uchar3(col.r, col.g, col.b));
			idx += point_num1;

			for (int j = 0; j < num_plane2; ++j) {
				//std::cout << "j = " << j << std::endl;
				if (is_merged[j]) continue;
				if (!HWPointCloud::ShouldMerge(c1, c2, i, j)) continue;
				std::cout << "should merge " << i << " and " << j << std::endl;
				//std::cout << "should merge" << std::endl;

				int start_idx2 = c2.planes_idx_[j];
				int end_idx2 = c2.planes_idx_[j + 1];
				int point_num2 = end_idx2 - start_idx2;

				points.insert(points.end(), points2.begin() + start_idx2, points2.begin() + end_idx2);
				if (!c1.normal_.empty() && !c2.normal_.empty()) {
					normals.insert(normals.end(), normals2.begin() + start_idx2, normals2.begin() + end_idx2);
				}
				colors.insert(colors.end(), point_num2, make_uchar3(col.r, col.g, col.b));
				idx += point_num2;

				is_merged[j] = true;
			}
			idxs.push_back(idx);
		}

		std::cout << "merge points2" << std::endl;
		for (int j = 0; j < num_plane2; ++j) {
			if (is_merged[j]) continue;
			int idx = idxs.back();
			ccColor::Rgb col = ccColor::Generator::Random();

			int start_idx = c2.planes_idx_[j];
			int end_idx = c2.planes_idx_[j + 1];
			int point_num = end_idx - start_idx;

			points.insert(points.end(), points2.begin() + start_idx, points2.begin() + end_idx);
			if (!c1.normal_.empty() && !c2.normal_.empty()) {
				normals.insert(normals.end(), normals2.begin() + start_idx, normals2.begin() + end_idx);
			}
			colors.insert(colors.end(), point_num, make_uchar3(col.r, col.g, col.b));
			idx += point_num;
			idxs.push_back(idx);
		}
		std::cout << "final points.size() = " << points.size() << std::endl;
		std::cout << "planes.size() = " << idxs.size() - 1 << std::endl;

		std::cout << "final steps" << std::endl;

		std::cout << "begin bounding box" << std::endl;
		float3 box_min = make_float3(FLT_MAX);
		float3 box_max = make_float3(-FLT_MAX);
		for (const auto& p : points) {
			box_min.x = fmin(box_min.x, p.x);
			box_min.y = fmin(box_min.y, p.y);
			box_min.z = fmin(box_min.z, p.z);

			box_max.x = fmax(box_max.x, p.x);
			box_max.y = fmax(box_max.y, p.y);
			box_max.z = fmax(box_max.z, p.z);
		}


		target.SetPlanesIdx(idxs);
		std::vector<int> is_wide(points.size(), 0);
		target.SetPlanesIsWide(is_wide);
		std::string full_name1 = c1.GetObjectName();
		std::string full_name2 = c2.GetObjectName();
		std::cout << "full_name1 = " << full_name1 << std::endl;
		std::cout << "full_name2 = " << full_name2 << std::endl;
		std::string name1 = full_name1.substr(full_name1.find_last_of("/\\") + 1, full_name1.find_last_of(".") - full_name1.find_last_of("/\\") - 1);
		std::string name2 = full_name2.substr(full_name2.find_last_of("/\\") + 1, full_name2.find_last_of(".") - full_name2.find_last_of("/\\") - 1);
		std::cout << "name1 = " << name1 << std::endl;
		std::cout << "name2 = " << name2 << std::endl;
		std::string new_name = full_name1.substr(0, full_name1.find_last_of("/\\") + 1) + name1 + "_merge_" + name2 + ".ply";
		std::cout << "new_name = " << new_name << std::endl;
		target.SetObjectName(new_name);
		target.SetObjectType(HW::kHWPointCloud);
		target.SetBoundingBox(box_min, box_max);
		target.SetPoint(points);
		target.SetNormal(normals);
		target.SetColor(colors);

		c1.RemoveInfoSpace();
		c2.RemoveInfoSpace();
		std::cout << "end MergeHWPointCloud" << std::endl;
	}

	//void MergeHWPointCloud(HWPointCloud& target, HWPointCloud& c1, HWPointCloud& c2)
	//{
	//	std::vector<HWPlane*> vec1 = c1.planes_vec_;
	//	std::vector<HWPlane*> vec2 = c2.planes_vec_;
	//	if (vec1.empty()) c1.InitPlanesVec();
	//	if (vec2.empty()) c2.InitPlanesVec();

	//	std::vector<bool> is_merged(vec2.size(), false);
	//	std::vector<int> idxs{ 0 };
	//	std::vector<float3> points;
	//	std::vector<float3> normals;
	//	std::vector<uchar3> colors;

	//	for (int i = 0; i < vec1.size(); ++i) {
	//		int idx = idxs.back();
	//		ccColor::Rgb col = ccColor::Generator::Random();

	//		HWPlane* p1 = vec1[i];
	//		const std::vector<float3> &pnts1 = p1->GetOriginPnts();
	//		points.insert(points.end(), pnts1.begin(), pnts1.end());
	//		if (!c1.normal_.empty() && !c2.normal_.empty()) {
	//			const std::vector<float3>& n1 = p1->GetOriginPntsNormal();
	//			normals.insert(normals.end(), n1.begin(), n1.end());
	//		}
	//		colors.insert(colors.end(), pnts1.size(), make_uchar3(col.r, col.g, col.b));
	//		idx += pnts1.size();

	//		for (int j = 0; j < vec2.size(); ++j) {
	//			HWPlane* p2 = vec2[j];
	//			if (p1->IsSamePlane(p2)) {
	//				const std::vector<float3> &pnts2 = p2->GetOriginPnts();
	//				points.insert(points.end(), pnts2.begin(), pnts2.end());
	//				if (!c1.normal_.empty() && !c2.normal_.empty()) {
	//					const std::vector<float3> &n2 = p2->GetOriginPntsNormal();
	//					normals.insert(normals.end(), n2.begin(), n2.end());
	//				}
	//				colors.insert(colors.end(), pnts2.size(), make_uchar3(col.r, col.g, col.b));
	//				idx += pnts2.size();
	//				is_merged[j] = true;
	//			}
	//		}
	//		idxs.push_back(idx);
	//	}

	//	for (int i = 0; i < vec2.size(); ++i) {
	//		if (is_merged[i]) continue;
	//		int idx = idxs.back();
	//		ccColor::Rgb col = ccColor::Generator::Random();

	//		HWPlane* p2 = vec2[i];
	//		const std::vector<float3>& pnts2 = p2->GetOriginPnts();
	//		points.insert(points.end(), pnts2.begin(), pnts2.end());
	//		if (!c1.normal_.empty() && !c2.normal_.empty()) {
	//			const std::vector<float3>& n2 = p2->GetOriginPntsNormal();
	//			normals.insert(normals.end(), n2.begin(), n2.end());
	//		}
	//		colors.insert(colors.end(), pnts2.size(), make_uchar3(col.r, col.g, col.b));
	//		idx += pnts2.size();
	//		idxs.push_back(idx);
	//	}


	//	float3 box_min = make_float3(FLT_MAX);
	//	float3 box_max = make_float3(FLT_MIN);
	//	for (const auto& p : points) {
	//		box_min = min(box_min, p);
	//		box_max = max(box_max, p);
	//	}

	//	target.SetPlanesIdx(idxs);
	//	std::vector<int> is_wide(points.size(), 0);
	//	target.SetPlanesIsWide(is_wide);
	//	std::string full_name1 = c1.GetObjectName();
	//	std::string full_name2 = c2.GetObjectName();
	//	std::string name1 = full_name1.substr(full_name1.find_last_of("/"), full_name1.find_last_of(".obj"));
	//	std::string name2 = full_name1.substr(full_name2.find_last_of("/"), full_name2.find_last_of(".obj"));
	//	std::string new_name = full_name1.substr(0, full_name1.find_last_of("/")) + name1 + "_merge_" + name2 + ".obj";
	//	target.SetObjectName(new_name);
	//	target.SetObjectType(HW::kHWPointCloud);
	//	target.SetBoundingBox(box_min, box_max);
	//	target.SetPoint(points);
	//	target.SetNormal(normals);
	//	target.SetColor(colors);
	//}



	////std::pair<std::map<std::string, HW::HWObject*>::iterator, bool> pre_iter;
	//auto pre_iter = db_tree_->AccessTreeElements().insert(std::make_pair(new_name, &target));
	////?????,?????????
	//if (!pre_iter.second)
	//{
	//	std::string rename = hw_pc_name.substr(0, hw_pc_name.find_last_of(".")) + "_cloud.ply";
	//	pre_iter = db_tree_->AccessTreeElements().insert(std::make_pair(rename, add_pc));
	//}
}