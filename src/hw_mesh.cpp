#include <unordered_map>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <direct.h>

//local
#include "hw_mesh.h"
#include "rply.h"
#include "Helper.h"
#include "tinyply.h"

#include <iostream>
#include <fstream>
#include <vector>


#define M_PI       3.14159265358979323846
#define err        1e-4
#define XHT_CONVERT_PANO 0

//std::clock_t start1, end1;

namespace HW
{
	HWMesh::HWMesh()
	{
		scale_volume_ = NULL;
	}
	HWMesh::~HWMesh()
	{
		Clear();
	}



	void HWMesh::Clear()
	{
		if (scale_volume_ != NULL)
		{
			delete scale_volume_;
		}
		points_.clear();
		normal_.clear();
		faces_normal_.clear();
		color_.clear();
		faces_.clear();
	}

	bool HWMesh::Show()
	{
		return true;
	}

	bool HWMesh::Save(std::string& file)
	{
		return true;
	}

	//从文件中读取mesh
	bool HWMesh::ReadPly(const std::string& file)
	{
		std::ifstream fhd(file, std::ios::binary);
		if (fhd.fail()) throw std::runtime_error("failed to open " + file);
		tinyply::PlyFile file_ply;
		file_ply.parse_header(fhd);
		std::shared_ptr<tinyply::PlyData> point_vertices, point_normal, face_ver_idx;

		try
		{
			point_vertices = file_ply.request_properties_from_element("vertex", { "x", "y", "z" });
		}
		catch (const std::exception & e)
		{
			std::cerr << "tinyply exception: " << e.what() << std::endl;
			return false;
		}

		try
		{
			point_normal = file_ply.request_properties_from_element("vertex", { "nx", "ny", "nz" });
		}
		catch (const std::exception & e)
		{
			std::cerr << "tinyply exception: " << e.what() << std::endl;
			return false;
		}

		try
		{
			face_ver_idx = file_ply.request_properties_from_element("face", { "vertex_indices" });
		}
		catch (const std::exception & e)
		{
			std::cerr << "tinyply exception: " << e.what() << std::endl;
			return false;
		}

		//将主体读入到file_ply中
		file_ply.read(fhd);

		//read ply file to HWPointCloud
		if (point_vertices) std::cout << "\tRead " << point_vertices->count << " total vertices " << std::endl;
		if (point_normal) std::cout << "\tRead " << point_normal->count << " total vertex normals " << std::endl;
		if (face_ver_idx) std::cout << "\tRead " << face_ver_idx->count << " total faces " << std::endl;

		const size_t num_vertices_bytes = point_vertices->buffer.size_bytes();
		points_.resize(point_vertices->count);
		std::memcpy(points_.data(), point_vertices->buffer.get(), num_vertices_bytes);

		const size_t num_normal_bytes = point_normal->buffer.size_bytes();
		normal_.resize(point_normal->count);
		std::memcpy(normal_.data(), point_normal->buffer.get(), num_normal_bytes);

		//face
		const size_t num_faces_bytes = face_ver_idx->buffer.size_bytes();
		faces_.resize(face_ver_idx->count);
		std::memcpy(faces_.data(), face_ver_idx->buffer.get(), num_faces_bytes);

		return true;
	}

	/*bool HWMesh::ReadPlyStream(tinyply::PlyFile& ply_file)
	{

	}*/

	bool HWMesh::SavePly(const std::string& file, const PlyFormat& type)
	{
		bool write_ascii = false;
		if (type == PlyFormat::kAscci)
		{
			write_ascii = true;
		}

		p_ply ply_file = ply_create(file.c_str(),
			write_ascii ? PLY_ASCII : PLY_LITTLE_ENDIAN, NULL, 0, NULL);

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
			ply_add_property(ply_file, "red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
			ply_add_property(ply_file, "green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
			ply_add_property(ply_file, "blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
		}
		ply_add_element(ply_file, "face",
			static_cast<long>(faces_.size()));
		ply_add_property(ply_file, "vertex_indices", PLY_LIST, PLY_UCHAR, PLY_UINT);

		if (!ply_write_header(ply_file)) {
			printf("Write PLY failed: unable to write header.\n");
			ply_close(ply_file);
			return false;
		}

		for (size_t i = 0; i < points_.size(); i++) {
			//const auto &vertex = mesh.vertices_[i];
			ply_write(ply_file, (float)points_[i].x);
			ply_write(ply_file, (float)points_[i].y);
			ply_write(ply_file, (float)points_[i].z);
			if (this->HasNormal()) {
				//const auto &normal = mesh.vertex_normals_[i];
				ply_write(ply_file, (float)normal_[i].x);
				ply_write(ply_file, (float)normal_[i].y);
				ply_write(ply_file, (float)normal_[i].z);
			}
			if (this->HasColor()) {
				//const auto &color = mesh.vertex_colors_[i];
				ply_write(ply_file, (float)color_[i].x);
				ply_write(ply_file, (float)color_[i].y);
				ply_write(ply_file, (float)color_[i].z);
			}
		}
		for (size_t i = 0; i < this->faces_.size(); i++) {
			ply_write(ply_file, 3);
			ply_write(ply_file, faces_[i].x);
			ply_write(ply_file, faces_[i].y);
			ply_write(ply_file, faces_[i].z);
		}
		ply_close(ply_file);

        printf("saved ply file to %s.\n", file.c_str());
		return true;
	}

	bool HWMesh::SaveObj(const std::string& file)
	{
		if (faces_info_.empty())
		{
			std::cout << "faces info is empty..." << std::endl;
			return false;
		}

		std::string texture_dir = file.substr(0, file.find_last_of("/"));
		std::string file_name_no_siffux = file.substr(0, file.find_last_of("."));
		std::string file_absolute_path_name = file_name_no_siffux + ".obj";

		std::string file_name = file_name_no_siffux.substr(file_name_no_siffux.find_last_of("/") + 1, file_name_no_siffux.length()
			- file_name_no_siffux.find_last_of("/") - 1);

		std::cout << "file name: " << file_name << std::endl;

		//只保存ascci格式
		std::ofstream obj_out(file_absolute_path_name.c_str());

		obj_out << "# " << "Blender v2.64 (sub 0) OBJ File: 'blend'" << std::endl;
		//write mtl
		obj_out << "mtllib " << file_name << ".mtl" << std::endl;

		//写入顶点
		for (int i = 0; i < points_.size(); ++i)
		{
			obj_out << "v " << points_[i].x << " " << points_[i].y << " " << points_[i].z << std::endl;
		}

		//写入vt
		for (int i = 0; i < faces_tex_coord_.size(); ++i)
		{

			obj_out << "vt " << faces_tex_coord_[i].x << " " << faces_tex_coord_[i].y << std::endl;

		}

		//写入normal
		for (int i = 0; i < normal_.size(); ++i)
		{
			obj_out << "vn " << normal_[i].x << " " << normal_[i].y << " " << normal_[i].z << std::endl;
		}
		//obj_out << "usemtl texture0" << std::endl;
		//写入面片
		for (int i = 0, texID = 0; i < faces_.size(); ++i)
		{

			if (i == texture2faceID_[texID]) {
				obj_out << "usemtl texture" << texID << std::endl;
				texID++;
			}

			obj_out << "f " << faces_[i].x + 1 << "/" << 3 * i + 1 << "/" << faces_[i].x + 1
				<< " " << faces_[i].y + 1 << "/" << 3 * i + 2 << "/" << faces_[i].y + 1
				<< " " << faces_[i].z + 1 << "/" << 3 * i + 3 << "/" << faces_[i].z + 1 << std::endl;
		}


		std::string mtl_path = file_name_no_siffux + ".mtl";
		//写入mtl文件
		std::ofstream mtl_out(mtl_path);
		if (mtl_out.is_open())
		{
			//write header
			for (int i = 0; i < texture2faceID_.size(); i++) {
				mtl_out << "newmtl texture" << i << std::endl;
				mtl_out << "Ns 96.078431" << std::endl;
				mtl_out << "Ka 0.000000 0.000000 0.000000" << std::endl;
				mtl_out << "Kd 0.640000 0.640000 0.640000" << std::endl;
				mtl_out << "Ks 0.500000 0.500000 0.500000" << std::endl;
				mtl_out << "Ni 1.000000" << std::endl;
				mtl_out << "d 1.000000" << std::endl;
				mtl_out << "illum 2" << std::endl;
				mtl_out << "map_Kd " << "texture" << i << ".jpg" << std::endl;
				mtl_out << std::endl << std::endl;
			}
			mtl_out.close();
		}

#if 0

		//设置访问机制
		std::vector<int> points_access_flag;
		points_access_flag.resize(points_.size());

		//设置删除机制
		std::vector<bool> faces_tex_coord_deleted;
		faces_tex_coord_deleted.resize(faces_tex_coord_.size());
		for (int i = 0; i < faces_tex_coord_deleted.size(); ++i)
		{
			faces_tex_coord_deleted[i] = false;
		}

		//group 三角面片，按照每个pano图片的索引来faces_的索引
		//构建对应点的texture
		for (int i = 0; i < textures_faces_info_idx_.size(); ++i)
		{
			for (int i = 0; i < points_access_flag.size(); ++i)
			{
				points_access_flag[i] = -2;
			}
			//第i张图片,在group中删除重复的纹理坐标
			for (int j = 0; j < textures_faces_info_idx_[i].size(); ++j)
			{
				FaceVersTexNormIdx* tmp_face = &faces_info_[textures_faces_info_idx_[i][j]];

				//判断这个三角行三个顶点的纹理坐标是否重复,判断顶点是否重复
				/*bool tri_vers_duplicated_flag[3];
				tri_vers_duplicated_flag[0] = false;
				tri_vers_duplicated_flag[1] = false;
				tri_vers_duplicated_flag[2] = false;*/

				if (points_access_flag[tmp_face->vers_idx_.x] != -2)
				{
					//已经重复,处理重复
					//先删除
					faces_tex_coord_deleted[tmp_face->tex_idx_.x] = true;
					//再赋值，第一次存下来的tex_idx
					tmp_face->tex_idx_.x = points_access_flag[tmp_face->vers_idx_.x];
				}
				else
				{
					points_access_flag[tmp_face->vers_idx_.x] = tmp_face->tex_idx_.x;
				}

				if (points_access_flag[tmp_face->vers_idx_.y] != -2)
				{
					//已经重复,处理重复
					//先删除
					faces_tex_coord_deleted[tmp_face->tex_idx_.y] = true;
					//再赋值，第一次存下来的tex_idx
					tmp_face->tex_idx_.y = points_access_flag[tmp_face->vers_idx_.y];
				}
				else
				{
					points_access_flag[tmp_face->vers_idx_.y] = tmp_face->tex_idx_.y;
				}

				if (points_access_flag[tmp_face->vers_idx_.z] != -2)
				{
					//已经重复,处理重复
					//先删除
					faces_tex_coord_deleted[tmp_face->tex_idx_.z] = true;
					//再赋值，第一次存下来的tex_idx
					tmp_face->tex_idx_.z = points_access_flag[tmp_face->vers_idx_.z];
				}
				else
				{
					points_access_flag[tmp_face->vers_idx_.z] = tmp_face->tex_idx_.z;
				}
			}
		}

		//faces_tex_coord_deleted里面存了要删除的顶点索引
		std::vector<float2> remain_tex_coord;
		std::vector<int> tex_coord_to_deleted_coord_idx;
		tex_coord_to_deleted_coord_idx.resize(faces_tex_coord_.size());
		for (int i = 0; i < tex_coord_to_deleted_coord_idx.size(); ++i)
		{
			tex_coord_to_deleted_coord_idx[i] = -1;
		}
		int undeleted_idx = 0;
		for (int i = 0; i < faces_tex_coord_deleted.size(); ++i)
		{
			//索引i
			if (!faces_tex_coord_deleted[i])
			{
				remain_tex_coord.emplace_back(faces_tex_coord_[i]);
				tex_coord_to_deleted_coord_idx[i] = undeleted_idx;
				undeleted_idx++;
			}
		}

		std::cout << "remaining tex number is: " << undeleted_idx << std::endl;
		system("pause");

		//重新映射索引
		//group 三角面片，按照每个pano图片的索引来faces_的索引
		//构建对应点的texture
		for (int i = 0; i < textures_faces_info_idx_.size(); ++i)
		{
			//第i张图片,在group中删除重复的纹理坐标
			for (int j = 0; j < textures_faces_info_idx_[i].size(); ++j)
			{
				FaceVersTexNormIdx* tmp_face = &faces_info_[textures_faces_info_idx_[i][j]];

				if (tex_coord_to_deleted_coord_idx[tmp_face->tex_idx_.x] != -1)
					tmp_face->tex_idx_.x = tex_coord_to_deleted_coord_idx[tmp_face->tex_idx_.x];

				if (tex_coord_to_deleted_coord_idx[tmp_face->tex_idx_.y] != -1)
					tmp_face->tex_idx_.y = tex_coord_to_deleted_coord_idx[tmp_face->tex_idx_.y];

				if (tex_coord_to_deleted_coord_idx[tmp_face->tex_idx_.z] != -1)
					tmp_face->tex_idx_.z = tex_coord_to_deleted_coord_idx[tmp_face->tex_idx_.z];
			}
		}

		//		//保存所有的重新排序的顶点
		//		std::vector<float3> remapping_vertices;
		//		std::vector<float3> remapping_noramls;
		//		std::vector<float2> remapping_faces_tex_coords;
		//		std::vector<int3> remapping_faces_idx;
		//		
		//		//开始排序faces_
		//		for (int i = 0; i < pano_face_idx_.size(); ++i)
		//		{
		//			remapping_faces_idx.emplace_back(faces_[pano_face_idx_[i]]);
		//			remapping_faces_tex_coords.emplace_back(pano_face_idx_[i] * 3);
		//			remapping_faces_tex_coords.emplace_back(pano_face_idx_[i] * 3 + 1);
		//			remapping_faces_tex_coords.emplace_back(pano_face_idx_[i] * 3 + 2);
		//			//remapping_faces_idx.emplace_back(faces_[i]);
		//		}
		//
		//		//重新排列texture_coord
		//		std::vector<float2> points_tex_coord; 
		//		std::vector<bool> points_duplicated_flag;
		//
		//		points_tex_coord.resize(points_.size());
		//		points_duplicated_flag.resize(points_.size());
		//		for (int i = 0; i < points_tex_coord.size(); ++i)
		//		{
		//			points_tex_coord[i].x = -1.0;
		//			points_tex_coord[i].y = -1.0;
		//			points_duplicated_flag[i] = false;
		//		}
		//
		//		//保存另一个
		//		std::vector<std::pair<int, float2> > vers_duplicated_tex_coords;
		//		
		//		int tmp_idx_start = 0;
		//		for (int pano_idx = 0; pano_idx < pano_faces_num_.size(); ++pano_idx)
		//		{
		//			int face_idx_end = tmp_idx_start + pano_faces_num_[pano_idx];
		//			while (tmp_idx_start < face_idx_end)
		//			{
		//				//取出每个顶点的索引
		//				int3 tri_vers_idx = remapping_faces_idx[tmp_idx_start];
		//				
		//				if (points_duplicated_flag[tri_vers_idx.x])
		//				{
		//					//获取这个顶点的纹理坐标
		//					float2 tmp_coord = remapping_faces_tex_coords[3 * tmp_idx_start];
		//					vers_duplicated_tex_coords.emplace_back(std::make_pair(pano_idx, tmp_coord));
		//				}
		//				else
		//				{
		//					points_tex_coord[tri_vers_idx.x] = remapping_faces_tex_coords[3 * tmp_idx_start];
		//					points_duplicated_flag[tri_vers_idx.x] = true;
		//				}
		//
		//				if (points_duplicated_flag[tri_vers_idx.y])
		//				{
		//					//获取这个顶点的纹理坐标
		//					float2 tmp_coord = remapping_faces_tex_coords[3 * tmp_idx_start + 1];
		//					vers_duplicated_tex_coords.emplace_back(std::make_pair(pano_idx, tmp_coord));
		//				}
		//				else
		//				{
		//					points_tex_coord[tri_vers_idx.y] = remapping_faces_tex_coords[3 * tmp_idx_start + 1];
		//					points_duplicated_flag[tri_vers_idx.y] = true;
		//				}
		//
		//				if (points_duplicated_flag[tri_vers_idx.z])
		//				{
		//					//获取这个顶点的纹理坐标
		//					float2 tmp_coord = remapping_faces_tex_coords[3 * tmp_idx_start + 2];
		//					vers_duplicated_tex_coords.emplace_back(std::make_pair(pano_idx, tmp_coord));
		//				}
		//				else
		//				{
		//					points_tex_coord[tri_vers_idx.z] = remapping_faces_tex_coords[3 * tmp_idx_start + 2];
		//					points_duplicated_flag[tri_vers_idx.z] = true;
		//				}
		//
		//				++tmp_idx_start;
		//			}
		//		}
		//
		//		//保存出顶点
		//
		//		//重投影faces_的索引和vertices的映射，保持拓扑结构
		//		//这一版本没有去除重复点，是整体全部复制
		//		//去除重复顶点
		//		//这个结构：它的顺序和points_的顺序一致,里面的值是和remapping后的索引,它用于去除重复
		//		//std::vector<int> vertices_idx_to_remapping_idx;
		//		//vertices_idx_to_remapping_idx.resize(points_.size());
		//		//for (int i = 0; i < vertices_idx_to_remapping_idx.size(); ++i)
		//		//{
		//		//	vertices_idx_to_remapping_idx [i]= -1;
		//		//}
		//		//int face_idx_start = 0;
		//		//for (int pano_idx = 0; pano_idx < pano_faces_num_.size(); ++pano_idx)
		//		//{
		//		//	int face_idx_end = face_idx_start + pano_faces_num_[pano_idx];
		//
		//		//	while (face_idx_start < face_idx_end)
		//		//	{
		//		//		//取出面的索引
		//		//		int3 tri_vers_idx = remapping_faces_idx[face_idx_start];
		//		//		if(vertices_idx_to_remapping_idx[tri_vers_idx.x] == -1)
		//		//		{
		//		//			remapping_vertices.emplace_back(points_[tri_vers_idx.x]);
		//		//			vertices_idx_to_remapping_idx[tri_vers_idx.x] = 3 * face_idx_start;
		//		//			remapping_faces_coords.emplace_back(faces_tex_coord_[pano_face_idx_[face_idx_start] * 3]);
		//		//			remapping_noramls.emplace_back(normal_[tri_vers_idx.x]);
		//		//			remapping_faces_idx[face_idx_start].x = 3 * face_idx_start + 1;
		//		//		}
		//		//		else
		//		//		{
		//		//			//remapping_faces_idx[face_idx_start].x = 
		//		//		}
		//		//		if (vertices_idx_to_remapping_idx[tri_vers_idx.y] == -1)
		//		//		{
		//		//			remapping_vertices.emplace_back(points_[tri_vers_idx.y]);
		//		//		}
		//		//		if (vertices_idx_to_remapping_idx[tri_vers_idx.z] == -1)
		//		//		{
		//		//			remapping_vertices.emplace_back(points_[tri_vers_idx.z]);
		//		//		}
		//		//		
		//		//		
		//		//		remapping_faces_coords.emplace_back(faces_tex_coord_[pano_face_idx_[face_idx_start] * 3 + 1]);
		//		//		remapping_faces_coords.emplace_back(faces_tex_coord_[pano_face_idx_[face_idx_start] * 3 + 2]);
		//		//		
		//		//		remapping_noramls.emplace_back(normal_[tri_vers_idx.y]);
		//		//		remapping_noramls.emplace_back(normal_[tri_vers_idx.z]);
		//		//		
		//		//		remapping_faces_idx[face_idx_start].y = 3 * face_idx_start + 2;
		//		//		remapping_faces_idx[face_idx_start].z = 3 * face_idx_start + 3;
		//
		//		//		++face_idx_start;
		//		//	}
		//		//}
		//
		//		/*std::cout << "face_idx_start is: " << face_idx_start << std::endl;
		//		std::cout << "the remapping vertices num is: " << remapping_vertices.size() << std::endl;
		//		std::cout << "the remapping faces num is: " << remapping_faces_idx.size() << std::endl;
		//*/
		//		
		//		/*int pano_idx = 0;
		//		int pano_face_idx_start = 0;
		//		int pano_vertice_idx_start = 0;*/
		//
		//		//for (int i = 0; i < remapping_faces_idx.size(); ++i)
		//		//{
		//		//	if (i >= pano_faces_num_[pano_idx] + pano_face_idx_start)
		//		//	{
		//		//		pano_idx++;
		//		//		pano_face_idx_start += pano_faces_num_[pano_idx];
		//		//	}
		//		//	//取出索引
		//		//	for(int i = 0; i < )
		//		//}
		//
		//		////重投影faces_的索引和vertices的映射，保持拓扑结构
		//		//int pano_idx = 0;
		//		//int pano_face_idx_start = 0;
		//		//int pano_vertice_idx_start = 0;
		//		//for (int i = 0; i < remapping_faces_idx.size(); ++i)
		//		//{
		//		//	if (i >= pano_faces_num_[pano_idx] + pano_face_idx_start)
		//		//	{
		//		//		pano_idx++;
		//		//		pano_face_idx_start += pano_faces_num_[pano_idx];
		//		//	}
		//		//	if (pano_idx == 0)
		//		//	{
		//		//		pano_vertice_idx_start = 0;
		//		//	}
		//		//	else
		//		//	{
		//		//		pano_vertice_idx_start = pano_vertice_idx_start + pano_vertices_num_[pano_idx - 1];
		//		//	}
		//		//		
		//		//	//获取三角形的三个顶点
		//		//	bool vers_idx_flag[3];
		//		//	vers_idx_flag[0] = true;
		//		//	vers_idx_flag[1] = true;
		//		//	vers_idx_flag[2] = true;
		//		//	//这个部分可以再改进,加速
		//		//	for (int j = 0; j < pano_vertices_idx_.size(); ++j)
		//		//	{
		//		//		if (pano_vertices_idx_[j] == remapping_faces_idx[i].x && vers_idx_flag[0])
		//		//		{
		//		//			remapping_faces_idx[i].x = j;
		//		//			vers_idx_flag[0] = false;
		//		//		}
		//		//		if (pano_vertices_idx_[j] == remapping_faces_idx[i].y && vers_idx_flag[1])
		//		//		{
		//		//			remapping_faces_idx[i].y = j;
		//		//			vers_idx_flag[1] = false;
		//		//		}
		//		//		if (pano_vertices_idx_[j] == remapping_faces_idx[i].z && vers_idx_flag[2])
		//		//		{
		//		//			remapping_faces_idx[i].z = j;
		//		//			vers_idx_flag[2] = false;
		//		//		}	
		//		//	}
		//		//}
		//
		//		//保存obj格式
		//		/*std::string mesh_dir = file.substr(0, file.find_last_of("/"));
		//		std::cout << "texture mesh dir is: " << mesh_dir << std::endl;
		//		std::string pano_path = mesh_dir + "/pano/";
		//		if (_access(pano_path.c_str(), 0) == -1)
		//		{
		//			std::cout << "teture mesh dir has no pano directory~" << std::endl;
		//			return;
		//		}*/

		//已经构建好了索引的排序
		std::string file_name_no_siffux = file.substr(0, file.find_last_of("."));
		std::string file_absolute_path_name = file_name_no_siffux + ".obj";

		std::string file_name = file_name_no_siffux.substr(file_name_no_siffux.find_last_of("/") + 1, file_name_no_siffux.length()
			- file_name_no_siffux.find_last_of("/") - 1);

		std::cout << "file name: " << file_name << std::endl;

		//只保存ascci格式
		std::ofstream obj_out(file_absolute_path_name.c_str());

		////test
		//for (int i = 0; i < remapping_vertices.size(); ++i)
		//{
		//	//
		//}
		////end test
		//system("pause");

		//write obj header
		obj_out << "# " << "Blender v2.64 (sub 0) OBJ File: 'blend'" << std::endl;
		//write mtl
		obj_out << "mtllib " << file_name << ".mtl" << std::endl;

		//写入顶点
		for (int i = 0; i < points_.size(); ++i)
		{
			obj_out << "v " << points_[i].x << " " << points_[i].y << " " << points_[i].z << std::endl;
		}

		//写入vt
		for (int i = 0; i < faces_tex_coord_.size(); ++i)
		{
			if (!faces_tex_coord_deleted[i])
			{
				obj_out << "vt " << faces_tex_coord_[i].x << " " << faces_tex_coord_[i].y << std::endl;
			}
		}

		//写入normal
		for (int i = 0; i < normal_.size(); ++i)
		{
			obj_out << "vn " << normal_[i].x << " " << normal_[i].y << " " << normal_[i].z << std::endl;
		}

		//写入面片
		for (int i = 0; i < textures_faces_info_idx_.size(); ++i)
		{
			obj_out << "usemtl pano_image_" << i << std::endl;
			for (int j = 0; j < textures_faces_info_idx_[i].size(); ++j)
			{
				FaceVersTexNormIdx* tmp_face = &faces_info_[textures_faces_info_idx_[i][j]];

				////凡是三角面片中有索引，它里面的顶点tex为-1就不存这个三角形
				if (std::abs(remain_tex_coord[tmp_face->tex_idx_.x].x + 1.0) < 1e-4)
					continue;

				obj_out << "f " << tmp_face->vers_idx_.x + 1 << "/" << tmp_face->tex_idx_.x + 1 << "/" << tmp_face->normals_idx_.x + 1
					<< " " << tmp_face->vers_idx_.y + 1 << "/" << tmp_face->tex_idx_.y + 1 << "/" << tmp_face->normals_idx_.y + 1
					<< " " << tmp_face->vers_idx_.z + 1 << "/" << tmp_face->tex_idx_.z + 1 << "/" << tmp_face->normals_idx_.z + 1 << std::endl;
			}
		}

		//	//写入顶点和面片
		//	/*int face_start = 0;
		//	for (int pano_idx = 0; pano_idx < pano_faces_num_.size(); ++pano_idx)
		//	{
		//		int face_idx_end = face_start + pano_faces_num_[pano_idx];
		//		while (face_start < face_idx_end)
		//		{
		//			obj_out << "v " << remapping_vertices[3 * face_start].x << " " << remapping_vertices[3 * face_start].y
		//				<< " " << remapping_vertices[3 * face_start].z << std::endl;
		//			obj_out << "v " << remapping_vertices[3 * face_start + 1].x << " " << remapping_vertices[3 * face_start + 1].y
		//				<< " " << remapping_vertices[3 * face_start + 1].z << std::endl;
		//			obj_out << "v " << remapping_vertices[3 * face_start + 2].x << " " << remapping_vertices[3 * face_start + 2].y
		//				<< " " << remapping_vertices[3 * face_start + 2].z << std::endl;
		//			++face_start;
		//		}
		//		face_start -= pano_faces_num_[pano_idx];
		//		while (face_start < face_idx_end)
		//		{
		//			obj_out << "vt " << remapping_faces_coords[3 * face_start].x << " " << remapping_faces_coords[3 * face_start].y << std::endl;
		//			obj_out << "vt " << remapping_faces_coords[3 * face_start + 1].x << " " << remapping_faces_coords[3 * face_start + 1].y << std::endl;
		//			obj_out << "vt " << remapping_faces_coords[3 * face_start + 2].x << " " << remapping_faces_coords[3 * face_start + 2].y << std::endl;
		//			++face_start;
		//		}
		//		face_start -= pano_faces_num_[pano_idx];
		//		while (face_start < face_idx_end)
		//		{
		//			obj_out << "vn " << remapping_noramls[3 * face_start].x << " " << remapping_noramls[3 * face_start].y
		//				<< " " << remapping_noramls[3 * face_start].z << std::endl;
		//			obj_out << "vn " << remapping_noramls[3 * face_start + 1].x << " " << remapping_noramls[3 * face_start + 1].y
		//				<< " " << remapping_noramls[3 * face_start + 1].z << std::endl;
		//			obj_out << "vn " << remapping_noramls[3 * face_start + 2].x << " " << remapping_noramls[3 * face_start + 2].y
		//				<< " " << remapping_noramls[3 * face_start + 2].z << std::endl;
		//			++face_start;
		//		}
		//		face_start -= pano_faces_num_[pano_idx];
		//		obj_out << "usemtl pano_image_" << pano_idx << std::endl;
		//		while (face_start < face_idx_end)
		//		{
		//			obj_out << "f " << remapping_faces_idx[face_start].x << "/"<< remapping_faces_idx[face_start].x <<"/" << remapping_faces_idx[face_start].x
		//				<< " " << remapping_faces_idx[face_start].y << "/" << remapping_faces_idx[face_start].y << "/" << remapping_faces_idx[face_start].y
		//				<< " " << remapping_faces_idx[face_start].z << "/" << remapping_faces_idx[face_start].z << "/" << remapping_faces_idx[face_start].z << std::endl;
		//			++face_start;
		//		}
		//	}*/
		//	//for (int i = 0; i < remapping_vertices.size(); ++i)
		//	//{
		//	//	obj_out << "v " << remapping_vertices[i].x << " " << remapping_vertices[i].y << " " << remapping_vertices[i].z << std::endl;
		//	//	//obj_out << "vt " << remapping_vertices[i].x << " " << remapping_vertices[i].y << " " << remapping_vertices[i].z << std::endl;
		//	//	obj_out << "vn " << remapping_noramls[i].x << " " << remapping_noramls[i].y << " " << remapping_noramls[i].z << std::endl;
		//	//}
		//	//for (int i = 0; i < remapping_faces_idx.size(); ++i)
		//	//{
		//	//	obj_out << "f " << remapping_faces_idx[i].x << "/0/" << remapping_faces_idx[i].x
		//	//		<< " " << remapping_faces_idx[i].y << "/0/" << remapping_faces_idx[i].y
		//	//		<<" " << remapping_faces_idx[i].z <<"/0/" << remapping_faces_idx[i].z << std::endl;
		//	//}
		//	//std::vector<std::pair<int, int>> 
		//	//重新进行点云重排
		///*	for (int i = 0; i < pano_vertices_idx_.size(); ++i)
		//	{
		//		obj_out << "v " << points_[pano_vertices_idx_[i]].x << " " << points_[pano_vertices_idx_[i]].y << " " << points_[pano_vertices_idx_[i]].z << std::endl;
		//		if(HasTextureCoord())
		//			obj_out << "vt " << tex_coord_[pano_vertices_idx_[i]].x << " " << tex_coord_[pano_vertices_idx_[i]].y << std::endl;
		//		if(HasNormal)
		//			obj_out << "vn " << normal_[pano_vertices_idx_[i]].x << " " << normal_[pano_vertices_idx_[i]].y << " " << normal_[pano_vertices_idx_[i]].z << std::endl;
		//	}*/
		//		/*pOutFile << "f " << m_indices[3 * i] + 1 << "/0/" << m_indices[3 * i] + 1 << " "
		//			<< m_indices[3 * i + 1] + 1 << "/0/" << m_indices[3 * i + 1] + 1 << " "
		//			<< m_indices[3 * i + 2] + 1 << "/0/" << m_indices[3 * i + 2] + 1 << std::endl;*/
		//	//for (int i = 0; i < points_.size(); ++i)
		//	//{
		//	//	obj_out << "v " << points_[i].x << " " << points_[i].y << " " << points_[i].z << std::endl;
		//	//	
		//	//	/*if(HasTextureCoord)
		//	//		obj_out<<"vt "<<t*/
		//	//	if(HasNormal())
		//	//		obj_out << "vn " << normal_[i].x << " " << normal_[i].y << " " << normal_[i].z << std::endl;
		//	//	/*if(HasColor())
		//	//		obj_out << "vt " << normal_[i].x << " " << normal_[i].y << " " << normal_[i].z << std::endl;*/
		//	//}
		obj_out.close();

		std::string mtl_path = file_name_no_siffux + ".mtl";
		std::vector<std::string> pano_dir_path_list;
		std::string pano_dir = "./pano/";
		for (int i = 0; i < pano_images_path_.size(); ++i)
		{
			std::string pano_absolute_path = pano_images_path_[i];
			std::string pano_name = pano_images_path_[i].substr(pano_images_path_[i].find_last_of("/") + 1,
				pano_images_path_[i].length() - pano_images_path_[i].find_last_of("/") - 1);

			std::cout << "pano name: " << pano_name << std::endl;

			std::string pano_path = pano_dir + pano_name;
			pano_dir_path_list.emplace_back(pano_path);
		}

		system("pause");

		//写入mtl文件
		std::ofstream mtl_out(mtl_path);
		if (mtl_out.is_open())
		{
			//write header
			mtl_out << "# Blender MTL File: " << "'" << file_name << ".blend'" << std::endl;
			mtl_out << "# Material Count: " << (int)pano_dir_path_list.size() << std::endl;

			for (int i = 0; i < pano_dir_path_list.size(); ++i)
			{
				mtl_out << "newmtl pano_image_" << i << std::endl;
				mtl_out << "Ns 96.078431" << std::endl;
				mtl_out << "Ka 0.000000 0.000000 0.000000" << std::endl;
				mtl_out << "Kd 0.640000 0.640000 0.640000" << std::endl;
				mtl_out << "Ks 0.500000 0.500000 0.500000" << std::endl;
				mtl_out << "Ni 1.000000" << std::endl;
				mtl_out << "d 1.000000" << std::endl;
				mtl_out << "illum 2" << std::endl;
				mtl_out << "map_Kd " << pano_dir_path_list[i] << std::endl;
				mtl_out << std::endl << std::endl;
			}

			mtl_out.close();
		}
#endif
        printf("saved obj file to %s.\n", file.c_str());
		return true;
	}

	bool HWMesh::HasNormal()
	{
		return !normal_.empty();
	}

	bool HWMesh::HasColor()
	{
		return !color_.empty();
	}

	const std::vector<float3>& HWMesh::GetVertices()
	{
		return points_;
	}

	const std::vector<float3>& HWMesh::GetNormal()
	{
		return normal_;
	}

	const std::vector<uchar3>& HWMesh::GetPointsColor()
	{
		return color_;
	}

	const std::vector<int3>& HWMesh::GetFaces()
	{
		return faces_;
	}

	open3d::ScalableTSDFVolume * HWMesh::GetVolume()
	{
		return scale_volume_;
	}

	void HWMesh::SetVolume(open3d::ScalableTSDFVolume * in_volume)
	{
		scale_volume_ = in_volume;
	}

	float3 HWMesh::GetAPoint(int idx)
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

	//copy element from file
	void HWMesh::CopyPointsFromPlyData(std::shared_ptr<tinyply::PlyData>& point_element)
	{
		const size_t num_vertices_bytes = point_element->buffer.size_bytes();
		points_.resize(point_element->count);

		std::memcpy(points_.data(), point_element->buffer.get(), num_vertices_bytes);

		points_num_ = point_element->count;
	}

	void HWMesh::CopyNormalsFromPlyData(std::shared_ptr<tinyply::PlyData>& normal_element)
	{
		const size_t num_normal_bytes = normal_element->buffer.size_bytes();
		normal_.resize(normal_element->count);
		std::memcpy(normal_.data(), normal_element->buffer.get(), num_normal_bytes);
	}

	void HWMesh::CopyFacesFromPlyData(std::shared_ptr<tinyply::PlyData>& face_element)
	{
		const size_t num_faces_bytes = face_element->buffer.size_bytes();
		faces_.resize(face_element->count);

		printf("num_faces_bytes: %d\n", num_faces_bytes);
		printf("face_element->count: %d\n", face_element->count);

		std::memcpy(faces_.data(), face_element->buffer.get(), num_faces_bytes);
	}

	void HWMesh::CopyPointsColorsFromPlyData(std::shared_ptr<tinyply::PlyData>& point_color_element)
	{
		const size_t num_color_bytes = point_color_element->buffer.size_bytes();
		color_.resize(point_color_element->count);

		printf("num_colors_bytes: %d\n", num_color_bytes);
		printf("point_color_element->count: %d\n", point_color_element->count);

		std::memcpy(color_.data(), point_color_element->buffer.get(), num_color_bytes);
	}

	void HWMesh::AddPoint(float3& point)
	{
		points_.emplace_back(point);
	}

	void HWMesh::AddNormal(float3& point_normal)
	{
		normal_.emplace_back(point_normal);
	}

	void HWMesh::AddColor(uchar3& point_color)
	{
		color_.emplace_back(point_color);
	}

	void HWMesh::AddFaceIdx(int3& face_idx)
	{
		faces_.emplace_back(face_idx);
	}

	void HWMesh::ChangeColor(uchar3 & point_color, int idx)
	{
		if (idx < 0 || idx >= points_.size())
		{
			printf("color index out of range...\n");
		}
		else {
			color_[idx] = point_color;
		}
	}

	bool HWMesh::HasFacesNormals()
	{
		return (faces_normal_.size() > 0 && faces_normal_.size() == faces_.size());
	}

	bool HWMesh::HasTextureCoord()
	{
		return faces_tex_coord_.size() > 0;
	}

	void HWMesh::ComputeTriNormals(bool normalized)
	{
		faces_normal_.resize(faces_.size());
		for (size_t i = 0; i < faces_.size(); i++) {
			auto &triangle = faces_[i];
			float3 v01 = points_[triangle.y] - points_[triangle.x];
			float3 v02 = points_[triangle.z] - points_[triangle.x];
			faces_normal_[i] = cross(v01, v02);
		}
		if (normalized)
		{
			NormalizeNormals();
		}
	}

	void HWMesh::NormalizeNormals()
	{
		for (int i = 0; i < normal_.size(); ++i)
		{
			normal_[i] = normalized(normal_[i]);
		}
		for (int i = 0; i < faces_normal_.size(); ++i)
		{
			faces_normal_[i] = normalized(faces_normal_[i]);
		}
	}

	void HWMesh::ComputeVersNormals(bool normalized)
	{
		if (HasFacesNormals() == false)
		{
			ComputeTriNormals(false);
		}
		if (HasNormal() == false)
		{
			normal_.resize(points_.size());
		}
		for (int i = 0; i < faces_normal_.size(); ++i)
		{
			auto &triangle = faces_[i];
			normal_[triangle.x] += faces_normal_[i];
			normal_[triangle.y] += faces_normal_[i];
			normal_[triangle.z] += faces_normal_[i];
		}

		if (normalized)
		{
			NormalizeNormals();
		}
	}

	void HWMesh::RemoveDuplicatedVertices()
	{
		//删除重复点
		typedef std::tuple<float, float, float> Coordinate3;
		std::unordered_map<Coordinate3, size_t, open3d::hash_tuple::hash<Coordinate3>>
			point_to_old_index;
		std::vector<int> index_old_to_new(points_.size());
		bool has_vert_normal = HasNormal();
		bool has_vert_color = HasColor();
		size_t old_vertex_num = points_.size();
		size_t k = 0;                                            // new index
		for (size_t i = 0; i < old_vertex_num; i++) {            // old index
			Coordinate3 coord = std::make_tuple(points_[i].x, points_[i].y,
				points_[i].z);
			if (point_to_old_index.find(coord) == point_to_old_index.end()) {
				point_to_old_index[coord] = i;
				points_[k] = points_[i];
				if (has_vert_normal) normal_[k] = normal_[i];
				if (has_vert_color) color_[k] = color_[i];
				index_old_to_new[i] = (int)k;
				k++;
			}
			else {
				index_old_to_new[i] = index_old_to_new[point_to_old_index[coord]];
			}
		}
		points_.resize(k);
		if (has_vert_normal) normal_.resize(k);
		if (has_vert_color) color_.resize(k);
		if (k < old_vertex_num) {
			for (auto &triangle : faces_) {
				triangle.x = index_old_to_new[triangle.x];
				triangle.y = index_old_to_new[triangle.y];
				triangle.z = index_old_to_new[triangle.z];
			}
		}
	}

	void HWMesh::RemoveDuplicatedTriangles()
	{
		//删除重复的三角面片
		typedef std::tuple<int, int, int> Index3;
		std::unordered_map<Index3, size_t, open3d::hash_tuple::hash<Index3>>
			triangle_to_old_index;
		bool has_tri_normal = HasNormal();
		size_t old_triangle_num = faces_.size();
		size_t k = 0;
		for (size_t i = 0; i < old_triangle_num; i++) {
			Index3 index;
			// We first need to find the minimum index. Because triangle (0-1-2) and
			// triangle (2-0-1) are the same.
			if (faces_[i].x <= faces_[i].y) {
				if (faces_[i].x <= faces_[i].z) {
					index = std::make_tuple(faces_[i].x, faces_[i].y,
						faces_[i].z);
				}
				else {
					index = std::make_tuple(faces_[i].z, faces_[i].x,
						faces_[i].y);
				}
			}
			else {
				if (faces_[i].y <= faces_[i].z) {
					index = std::make_tuple(faces_[i].y, faces_[i].z,
						faces_[i].x);
				}
				else {
					index = std::make_tuple(faces_[i].z, faces_[i].x,
						faces_[i].y);
				}
			}
			if (triangle_to_old_index.find(index) == triangle_to_old_index.end()) {
				triangle_to_old_index[index] = i;
				faces_[k] = faces_[i];
				if (has_tri_normal) normal_[k] = normal_[i];
				k++;
			}
		}
		faces_.resize(k);
		if (has_tri_normal) normal_.resize(k);
	}

	void HWMesh::RemoveNonManifoldTriangles()
	{
		//删除manifold三角形
		// Non-manifold triangles are degenerate triangles that have one vertex as
		// its multiple end-points. They are usually the product of removing
		// duplicated vertices.
		bool has_tri_normal = HasNormal();
		size_t old_triangle_num = faces_.size();
		size_t k = 0;
		for (size_t i = 0; i < old_triangle_num; i++) {
			const auto &triangle = faces_[i];
			if (triangle.x != triangle.y && triangle.y != triangle.z &&
				triangle.z != triangle.x) {
				faces_[k] = faces_[i];
				if (has_tri_normal) normal_[k] = normal_[i];
				k++;
			}
		}
		faces_.resize(k);
		if (has_tri_normal) normal_.resize(k);
	}

	void HWMesh::RemoveNonManifoldVertices()
	{
		//删除manifold点
		// Non-manifold vertices are vertices without a triangle reference. They
		// should not exist in a valid triangle mesh.
		std::vector<bool> vertex_has_reference(points_.size(), false);
		for (const auto &triangle : faces_) {
			vertex_has_reference[triangle.x] = true;
			vertex_has_reference[triangle.y] = true;
			vertex_has_reference[triangle.z] = true;
		}
		std::vector<int> index_old_to_new(points_.size());
		bool has_vert_normal = HasNormal();
		bool has_vert_color = HasColor();
		size_t old_vertex_num = points_.size();
		size_t k = 0;                                            // new index
		for (size_t i = 0; i < old_vertex_num; i++) {            // old index
			if (vertex_has_reference[i]) {
				points_[k] = points_[i];
				if (has_vert_normal) normal_[k] = normal_[i];
				if (has_vert_color) color_[k] = color_[i];
				index_old_to_new[i] = (int)k;
				k++;
			}
			else {
				index_old_to_new[i] = -1;
			}
		}
		points_.resize(k);
		if (has_vert_normal) normal_.resize(k);
		if (has_vert_color) color_.resize(k);
		if (k < old_vertex_num) {
			for (auto &triangle : faces_) {
				triangle.x = index_old_to_new[triangle.x];
				triangle.y = index_old_to_new[triangle.y];
				triangle.z = index_old_to_new[triangle.z];
			}
		}
	}

	void HWMesh::Purge()
	{
		RemoveDuplicatedVertices();
		RemoveDuplicatedTriangles();
		RemoveNonManifoldTriangles();
		RemoveNonManifoldVertices();
	}
	void HWMesh::edgeSwap()
	{
		HalfEdge HE;
		for (int i = 0; i < points_.size(); i++) {
			HE.addVert(points_[i].x, points_[i].y, points_[i].z);
		}
		for (int i = 0; i < faces_.size(); i++) {
			HE.addFace(faces_[i].x, faces_[i].y, faces_[i].z);
		}
		HE.swapEdges();
		std::vector<HalfEdge::HE_face *> &HE_f = HE.getFaces();
		std::vector<HalfEdge::HE_halfedge *> &HE_e = HE.getEdges();
		for (int i = 0; i < HE_f.size(); i++) {

			HalfEdge::HE_face *f = HE_f[i];
			HalfEdge::HE_halfedge *e1 = HE_e[f->e_idx];
			HalfEdge::HE_halfedge *e2 = HE_e[e1->next];
			HalfEdge::HE_halfedge *e3 = HE_e[e2->next];
			faces_[i].x = e1->v_idx;
			faces_[i].y = e2->v_idx;
			faces_[i].z = e3->v_idx;
		}
		return;
	}


	void HWMesh::ConvertPano()
	{

		std::string file_name = GetObjectName();

		std::string mesh_dir = file_name.substr(0, file_name.find_last_of("/"));
		std::cout << "texture mesh dir is: " << mesh_dir << std::endl;
		std::string pano_path = mesh_dir + "/Pano/20190523191604/";
		std::string scene_dir = mesh_dir + "/20190523191604_5/";
		std::cout << scene_dir << std::endl;
		if (_access(pano_path.c_str(), 0) == -1) {
			std::cout << "teture mesh dir has no pano directory~" << std::endl;
			return;
		}
		if (_access(scene_dir.c_str(), 0) == -1) {
			mkdir(scene_dir.c_str());
		}

        // true: use .ptb file; false: use .txt file
        const bool usePtbFile = false;
        std::string pano_info;
        if (usePtbFile) {
            pano_info = pano_path + "output.ptb";
        }
        else {
            pano_info = pano_path + "pose.txt";
        }
		//open3d::ScalableTSDFVolume * volume = GetVolume();
		/*if (volume == NULL)
		{
		std::cout << "the select mesh volume is null" << std::endl;
		return;
		}*/

		std::vector<int> imgs_vertices_num;

		//open3d::ScalableTSDFVolume * volume = new open3d::ScalableTSDFVolume();
		std::vector<float3> vertices = GetVertices();

		for (int i = 0; i < vertices.size(); i++) {
			uchar3 color;
			color.x = 0;
			color.y = 0;
			color.y = 0;
			AddColor(color);
		}

		std::ifstream ifs(pano_info);
		if (!ifs.is_open()) {
			std::cout << "Can not open the pano info: " << pano_info << std::endl;
			return;
		}

		//全景图片的索引
		int image_idx = -1;
		std::string line;
		//std::getline(ifs, line);

		OpenGLWidget window;
		window.resize(512, 512);
		window.setWindowTitle("OpenGLWindow");
		//window.show();
		//window.hide();
		window.showMinimized();
		window.Init(&points_, &faces_);

		while (std::getline(ifs, line)) {
			int pano_idx;
			char pano_name[256];
			int timestamp;
			//double w, x, y, z;
			Eigen::Vector3f camera_pos;
			Eigen::Quaternionf quaternion;
			Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rotation_matrix;
            if (usePtbFile) {
                sscanf(line.c_str(), "%s %d %f %f %f", &pano_name, &timestamp, &camera_pos(0),
                    &camera_pos(1), &camera_pos(2));
                std::getline(ifs, line);
                sscanf(line.c_str(), "%f %f %f %f %f", &rotation_matrix(0, 0), &rotation_matrix(0, 1), &rotation_matrix(0, 2)
                    , &rotation_matrix(1, 0), &rotation_matrix(1, 1));
                std::getline(ifs, line);
                sscanf(line.c_str(), "%f %f %f %f", &rotation_matrix(1, 2), &rotation_matrix(2, 0), &rotation_matrix(2, 1), &rotation_matrix(2, 2));
            }
            else {
                sscanf(line.c_str(), "%s %f %f %f %f %f %f %f %f %f %f %f %f", &pano_name, &camera_pos(0), &camera_pos(1), &camera_pos(2), 
                    &rotation_matrix(0, 0), &rotation_matrix(0, 1), &rotation_matrix(0, 2), 
                    &rotation_matrix(1, 0), &rotation_matrix(1, 1), &rotation_matrix(1, 2), 
                    &rotation_matrix(2, 0), &rotation_matrix(2, 1), &rotation_matrix(2, 2));
            }
			//采样
			std::string pn_str = pano_name;
			std::string idx_str = pn_str.substr(pn_str.find_last_of("_") + 1, pn_str.find_last_of(".") - pn_str.find_last_of("_") - 1);
			pano_idx = std::stoi(idx_str);
            const int SUBSIMPLE_PARAM = 5;    // 抽样比例
			if (pano_idx % SUBSIMPLE_PARAM) {
				continue;
			}
			Eigen::Matrix3f rotation_matrix_inv = rotation_matrix.inverse();
			pano_idx = pano_idx / SUBSIMPLE_PARAM;
			std::string pano_image_path = pano_path + pano_name;
			std::cout << "pano_image_path:" << pano_image_path << "\n";
			camera_pos.x() -= 795700.0;
			camera_pos.y() -= 2533500.0;

            // 保存camera.ply
#if 0
			std::string filename = mesh_dir + "/camera.ply";
			std::ofstream out(filename);
			std::vector<Eigen::Vector3f> vec;
			std::vector<Eigen::Vector3i> color;
			for (int i = 0; i < 100; i++) {
				Eigen::Vector3f p = camera_pos + i*0.1*rotation_matrix_inv.col(0);
				vec.push_back(p);
				color.push_back(Eigen::Vector3i(255, 0, 0));
				p = camera_pos + i*0.1*rotation_matrix_inv.col(1);
				vec.push_back(p);
				color.push_back(Eigen::Vector3i(0, 255, 0));
				p = camera_pos + i*0.1*rotation_matrix_inv.col(2);
				vec.push_back(p);
				color.push_back(Eigen::Vector3i(0, 0, 255));
			}
			out << "ply";
			out << "\nformat " << "ascii" << " 1.0";
			out << "\nelement vertex " << vec.size();
			out << "\nproperty float x"
				"\nproperty float y"
				"\nproperty float z";
			out << "\nproperty uchar red"
				"\nproperty uchar green"
				"\nproperty uchar blue";
			out << "\nend_header\n";
			for (int i = 0; i < vec.size(); i++) {
				out << vec[i][0] << " " << vec[i][1] << " " << vec[i][2] << " " << color[i][0] << " " << color[i][1] << " " << color[i][2] << std::endl;
			}
			out.close();
			//return;
#endif
			
			Eigen::Matrix<float, 3, 3, Eigen::RowMajor> opengl_camera;
			//rotation_matrix = quaternion.matrix();

			opengl_camera.col(0) = -rotation_matrix_inv.col(2);
			opengl_camera.col(1) = rotation_matrix_inv.col(1);
			opengl_camera.col(2) = rotation_matrix_inv.col(0);
			//camera_pos_vec_.emplace_back(camera_pos);
			//rotation_matrix_vec_.emplace_back(rotation_matrix);
			Eigen::Matrix<float, 4, 4, Eigen::RowMajor> glcamera_pose;
			glcamera_pose.block(0, 0, 3, 3) << opengl_camera;
			glcamera_pose.block(0, 3, 3, 1) << camera_pos;
			glcamera_pose.row(3) << 0, 0, 0, 1;
			Eigen::Matrix<float, 4, 4, Eigen::RowMajor> glcamera_pose_inv = glcamera_pose.inverse();
			Eigen::Matrix<float, 4, 4, Eigen::RowMajor> rotate;
			rotate << 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1;
			//rotate << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1;
			//Eigen::Matrix4f view = rotate*glcamera_pose_inv;
			Eigen::Matrix<float, 4, 4, Eigen::RowMajor> view = glcamera_pose_inv;
			Eigen::Matrix<float, 4, 4, Eigen::RowMajor> glview;
			glview.setIdentity();
			//view.setIdentity();
			//view.row(0) *= -1;
			//view.row(2) *= -1;
			//view.row(1) = glcamera_pose_inv.row(2);
			//view.row(2) = -glcamera_pose_inv.row(1);
			/*	if (pano_images_path_.size() !=99)
			continue;*/
			//std::vector<bool> pano2mesh(faces_.size());
			window.loadTexture(QImage(pano_image_path.data()));

			for (int i = 0; i <= 3; i++) {
				if (i > 0 && i < 4) {
					view = rotate*view;
					glview = rotate*glview;
				}
				/*if (i == 4) {
					view = glcamera_pose_inv;
					view.row(1) = -glcamera_pose_inv.row(2);
					view.row(2) = glcamera_pose_inv.row(1);
					glview.setIdentity();
					glview.row(2).swap(glview.row(1));
					glview.row(1) *= -1;
				}
				if (i == 5) {
					view = glcamera_pose_inv;
					view.row(1) = glcamera_pose_inv.row(2);
					view.row(2) = -glcamera_pose_inv.row(1);
					glview.setIdentity();
					glview.row(2).swap(glview.row(1));
					glview.row(2) *= -1;
				}*/
				//std::cout << view << std::endl;
				window.setViewMatrix(glview, glcamera_pose_inv);
				cv::Mat colorImg;// (img_length, img_length, CV_8UC3);
				window.renderToImg(colorImg);
				//cv::imshow("color", colorImg);
				//cv::waitKey(0);

				std::vector<int> pngCompressionParams;
				pngCompressionParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
				pngCompressionParams.push_back(0);
				char img_dir[256], cam_dir[256];
				sprintf(img_dir, "%s%s_cam%d.PNG", scene_dir.c_str(), pano_name, i);
				cv::imwrite(img_dir, colorImg, pngCompressionParams);
				sprintf(cam_dir, "%s%s_cam%d.CAM", scene_dir.c_str(), pano_name, i);
				std::ofstream outFile(cam_dir);
				outFile << view(0, 3) << " " << -view(1, 3) << " " << -view(2, 3) << " ";
				/*for (int row = 0; row < 3; row++) {
				for (int col = 0; col < 3; col++) {
				outFile << view(row, col) << " ";
				}
				}*/
				outFile << view(0, 0) << " " << view(0, 1) << " " << view(0, 2) << " ";
				outFile << -view(1, 0) << " " << -view(1, 1) << " " << -view(1, 2) << " ";
				outFile << -view(2, 0) << " " << -view(2, 1) << " " << -view(2, 2) << "\n";
#if XHT_CONVERT_PANO
				outFile << 0.5f << "\n";
#else
				outFile << 0.5f * colorImg.cols << " " << 0.5f * colorImg.rows << " "
					<< 0.5f * colorImg.cols << " " << 0.5f * colorImg.rows << std::endl;
#endif
				outFile.close();
				//system("pause");
			}
		}
		//window.close();
		//Texturing();
	}

    void HWMesh::ConvertPano(std::string folderName)
    {

        std::string file_name = GetObjectName();

        std::string mesh_dir = file_name.substr(0, file_name.find_last_of("/"));
        std::cout << "texture mesh dir is: " << mesh_dir << std::endl;
        std::string pano_path = mesh_dir + "/Pano/" + folderName + "/";
        std::string scene_dir = mesh_dir + "/" + folderName + "_5/";
        std::cout << scene_dir << std::endl;
        if (_access(pano_path.c_str(), 0) == -1) {
            std::cout << "teture mesh dir has no pano directory~" << std::endl;
            return;
        }
        if (_access(scene_dir.c_str(), 0) == -1) {
            mkdir(scene_dir.c_str());
        }

        // true: use .ptb file; false: use .txt file
        const bool usePtbFile = false;
        std::string pano_info;
        if (usePtbFile) {
            pano_info = pano_path + "output.ptb";
        }
        else {
            pano_info = pano_path + "pose.txt";
        }
        //open3d::ScalableTSDFVolume * volume = GetVolume();
        /*if (volume == NULL)
        {
        std::cout << "the select mesh volume is null" << std::endl;
        return;
        }*/

        std::vector<int> imgs_vertices_num;

        //open3d::ScalableTSDFVolume * volume = new open3d::ScalableTSDFVolume();
        std::vector<float3> vertices = GetVertices();

        for (int i = 0; i < vertices.size(); i++) {
            uchar3 color;
            color.x = 0;
            color.y = 0;
            color.y = 0;
            AddColor(color);
        }

        std::ifstream ifs(pano_info);
        if (!ifs.is_open()) {
            std::cout << "Can not open the pano info: " << pano_info << std::endl;
            return;
        }

        //全景图片的索引
        int image_idx = -1;
        std::string line;
        //std::getline(ifs, line);

        OpenGLWidget window;
        window.resize(512, 512);
        window.setWindowTitle("OpenGLWindow");
        //window.show();
        //window.hide();
        window.showMinimized();
        window.Init(&points_, &faces_);

        while (std::getline(ifs, line)) {
            int pano_idx;
            char pano_name[256];
            int timestamp;
            //double w, x, y, z;
            Eigen::Vector3f camera_pos;
            Eigen::Quaternionf quaternion;
            Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rotation_matrix;
            if (usePtbFile) {
                sscanf(line.c_str(), "%s %d %f %f %f", &pano_name, &timestamp, &camera_pos(0),
                    &camera_pos(1), &camera_pos(2));
                std::getline(ifs, line);
                sscanf(line.c_str(), "%f %f %f %f %f", &rotation_matrix(0, 0), &rotation_matrix(0, 1), &rotation_matrix(0, 2)
                    , &rotation_matrix(1, 0), &rotation_matrix(1, 1));
                std::getline(ifs, line);
                sscanf(line.c_str(), "%f %f %f %f", &rotation_matrix(1, 2), &rotation_matrix(2, 0), &rotation_matrix(2, 1), &rotation_matrix(2, 2));
            }
            else {
                sscanf(line.c_str(), "%s %f %f %f %f %f %f %f %f %f %f %f %f", &pano_name, &camera_pos(0), &camera_pos(1), &camera_pos(2),
                    &rotation_matrix(0, 0), &rotation_matrix(0, 1), &rotation_matrix(0, 2),
                    &rotation_matrix(1, 0), &rotation_matrix(1, 1), &rotation_matrix(1, 2),
                    &rotation_matrix(2, 0), &rotation_matrix(2, 1), &rotation_matrix(2, 2));
            }
            //采样
            std::string pn_str = pano_name;
            std::string idx_str = pn_str.substr(pn_str.find_last_of("_") + 1, pn_str.find_last_of(".") - pn_str.find_last_of("_") - 1);
            pano_idx = std::stoi(idx_str);
            const int SUBSIMPLE_PARAM = 5;    // 抽样比例
            if (pano_idx % SUBSIMPLE_PARAM) {
                continue;
            }
            Eigen::Matrix3f rotation_matrix_inv = rotation_matrix.inverse();
            pano_idx = pano_idx / SUBSIMPLE_PARAM;
            std::string pano_image_path = pano_path + pano_name;
            std::cout << "pano_image_path:" << pano_image_path << "\n";
            {
                cv::Mat panoImgData = cv::imread(pano_image_path);
                if (panoImgData.empty() || !panoImgData.data) {
                    std::cout << "pano_image :" << pano_image_path << "do not exist!\n";
                    continue;
                }
                else
                {
                    panoImgData.release();
                }
            }
            camera_pos.x() -= 795700.0;
            camera_pos.y() -= 2533500.0;

            // 保存camera.ply
#if 0
            std::string filename = mesh_dir + "/camera.ply";
            std::ofstream out(filename);
            std::vector<Eigen::Vector3f> vec;
            std::vector<Eigen::Vector3i> color;
            for (int i = 0; i < 100; i++) {
                Eigen::Vector3f p = camera_pos + i * 0.1*rotation_matrix_inv.col(0);
                vec.push_back(p);
                color.push_back(Eigen::Vector3i(255, 0, 0));
                p = camera_pos + i * 0.1*rotation_matrix_inv.col(1);
                vec.push_back(p);
                color.push_back(Eigen::Vector3i(0, 255, 0));
                p = camera_pos + i * 0.1*rotation_matrix_inv.col(2);
                vec.push_back(p);
                color.push_back(Eigen::Vector3i(0, 0, 255));
            }
            out << "ply";
            out << "\nformat " << "ascii" << " 1.0";
            out << "\nelement vertex " << vec.size();
            out << "\nproperty float x"
                "\nproperty float y"
                "\nproperty float z";
            out << "\nproperty uchar red"
                "\nproperty uchar green"
                "\nproperty uchar blue";
            out << "\nend_header\n";
            for (int i = 0; i < vec.size(); i++) {
                out << vec[i][0] << " " << vec[i][1] << " " << vec[i][2] << " " << color[i][0] << " " << color[i][1] << " " << color[i][2] << std::endl;
            }
            out.close();
            //return;
#endif

            Eigen::Matrix<float, 3, 3, Eigen::RowMajor> opengl_camera;
            //rotation_matrix = quaternion.matrix();

            opengl_camera.col(0) = -rotation_matrix_inv.col(2);
            opengl_camera.col(1) = rotation_matrix_inv.col(1);
            opengl_camera.col(2) = rotation_matrix_inv.col(0);
            //camera_pos_vec_.emplace_back(camera_pos);
            //rotation_matrix_vec_.emplace_back(rotation_matrix);
            Eigen::Matrix<float, 4, 4, Eigen::RowMajor> glcamera_pose;
            glcamera_pose.block(0, 0, 3, 3) << opengl_camera;
            glcamera_pose.block(0, 3, 3, 1) << camera_pos;
            glcamera_pose.row(3) << 0, 0, 0, 1;
            Eigen::Matrix<float, 4, 4, Eigen::RowMajor> glcamera_pose_inv = glcamera_pose.inverse();
            Eigen::Matrix<float, 4, 4, Eigen::RowMajor> rotate;
            rotate << 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1;
            //rotate << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1;
            //Eigen::Matrix4f view = rotate*glcamera_pose_inv;
            Eigen::Matrix<float, 4, 4, Eigen::RowMajor> view = glcamera_pose_inv;
            Eigen::Matrix<float, 4, 4, Eigen::RowMajor> glview;
            glview.setIdentity();
            //view.setIdentity();
            //view.row(0) *= -1;
            //view.row(2) *= -1;
            //view.row(1) = glcamera_pose_inv.row(2);
            //view.row(2) = -glcamera_pose_inv.row(1);
            /*	if (pano_images_path_.size() !=99)
            continue;*/
            //std::vector<bool> pano2mesh(faces_.size());
            window.loadTexture(QImage(pano_image_path.data()));

            for (int i = 0; i <= 3; i++) {
                if (i > 0 && i < 4) {
                    view = rotate * view;
                    glview = rotate * glview;
                }
                /*if (i == 4) {
                    view = glcamera_pose_inv;
                    view.row(1) = -glcamera_pose_inv.row(2);
                    view.row(2) = glcamera_pose_inv.row(1);
                    glview.setIdentity();
                    glview.row(2).swap(glview.row(1));
                    glview.row(1) *= -1;
                }
                if (i == 5) {
                    view = glcamera_pose_inv;
                    view.row(1) = glcamera_pose_inv.row(2);
                    view.row(2) = -glcamera_pose_inv.row(1);
                    glview.setIdentity();
                    glview.row(2).swap(glview.row(1));
                    glview.row(2) *= -1;
                }*/
                //std::cout << view << std::endl;
                window.setViewMatrix(glview, glcamera_pose_inv);
                cv::Mat colorImg;// (img_length, img_length, CV_8UC3);
                window.renderToImg(colorImg);
                //cv::imshow("color", colorImg);
                //cv::waitKey(0);

                std::vector<int> pngCompressionParams;
                pngCompressionParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
                pngCompressionParams.push_back(0);
                char img_dir[256], cam_dir[256];
                sprintf(img_dir, "%s%s_cam%d.PNG", scene_dir.c_str(), pano_name, i);
                cv::imwrite(img_dir, colorImg, pngCompressionParams);
                sprintf(cam_dir, "%s%s_cam%d.CAM", scene_dir.c_str(), pano_name, i);
                std::ofstream outFile(cam_dir);
                outFile << view(0, 3) << " " << -view(1, 3) << " " << -view(2, 3) << " ";
                /*for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                outFile << view(row, col) << " ";
                }
                }*/
                outFile << view(0, 0) << " " << view(0, 1) << " " << view(0, 2) << " ";
                outFile << -view(1, 0) << " " << -view(1, 1) << " " << -view(1, 2) << " ";
                outFile << -view(2, 0) << " " << -view(2, 1) << " " << -view(2, 2) << "\n";
#if XHT_CONVERT_PANO
                outFile << 0.5f << "\n";
#else
                outFile << 0.5f * colorImg.cols << " " << 0.5f * colorImg.rows << " "
                    << 0.5f * colorImg.cols << " " << 0.5f * colorImg.rows << std::endl;
#endif
                outFile.close();
                //system("pause");
            }
        }
        //window.close();
        //Texturing();
    }

	void HWMesh::DoTextureMapping(bool isRemoved)
	{
        //1：convertPano(),并return；0：关闭
#if 0
        std::vector<std::string> folderNames = { "20190519094117",
"20190519094134", "20190519094151", "20190519094207", "20190519094223", "20190519094237", 
"20190519094256", "20190520110720", "20190523191521", "20190523191541", "20190523191604",
"20190523191618", "20190523191639", "20190523191658", "20190523191718", "20190523191735",
"20190523192730", "20190523192817", "20190523192835", "20190523192851", "20190523192915",
"20190523192943" };

        for (int i = 0; i < 22; i++) {
            clock_t t1 = clock();

            std::cout << "folderName_: " << folderNames[i] << std::endl;
            std::string folderName_ = folderNames[i];
            ConvertPano(folderName_);

            clock_t t2 = clock();
            std::cout << "Converting time: " << (double)(t2 - t1) / CLOCKS_PER_SEC << "s" << std::endl;
        }
		return;
#endif

		color_.resize(points_.size());
		std::string file_name = GetObjectName();

		std::string mesh_dir = file_name.substr(0, file_name.find_last_of("/"));
		std::string cam_xml = mesh_dir + "/sensor_frame.xml";
		//std::cout << cam_xml << std::endl;
		std::string json_dir = mesh_dir + "/info";
		//std::cout << json_dir << std::endl;
		std::string img_dir = mesh_dir + "/cam/";
		std::string scene_dir = mesh_dir + "/scene/";
		if (_access(scene_dir.c_str(), 0) == -1) {
			mkdir(scene_dir.c_str());
		}

		HW::OCamModel oCamModel;
		oCamModel.readIntrinsic(cam_xml);
		oCamModel.readExtrinsic(json_dir);
		
		int ext_num = oCamModel.extrinsics_.size(), intr_num = oCamModel.intrinsics_.size();
		printf("Convert fisheye images to normal images, ext_num = %d, intr_num = %d\n", ext_num, intr_num);

		clock_t start_time, end_time;
		start_time = clock();
        // 1：convert fish eye images； 0：skip
#if 0
		char cam_dir[256];
		float flen = 0.5;
#pragma omp parallel for
		for (int offset = 0; offset < ext_num; offset++) {
			int i = offset / intr_num;
			/*if (i % 3 != 0)
				continue;*/
			int j = offset%intr_num;

			sprintf(cam_dir, "%s%05d-cam%d.CAM", scene_dir.c_str(), i, j);
			std::ofstream outFile(cam_dir);
			//std::cout << cam_dir << std::endl;
			Eigen::Matrix4f& ext = oCamModel.extrinsics_[offset];
			HW::OCamModel::Intrinsic& intr = oCamModel.intrinsics_[j];
			/*outFile << -ext(1, 3) << " " << ext(0, 3) << " " << ext(2, 3) << " ";
			outFile << -ext(1, 0) << " " << -ext(1, 1) << " " << -ext(1, 2) << " ";
			outFile << ext(0, 0) << " " << ext(0, 1) << " " << ext(0, 2) << " ";
			outFile << ext(2, 0) << " " << ext(2, 1) << " " << ext(2, 2) << "\n";*/
			outFile << ext(0, 3)  << " " << ext(1, 3) << " " << ext(2, 3) << " ";
			outFile << ext(0, 0) << " " << ext(0, 1) << " " << ext(0, 2) << " ";
			outFile << ext(1, 0) << " " << ext(1, 1) << " " << ext(1, 2) << " ";
			outFile << ext(2, 0) << " " << ext(2, 1) << " " << ext(2, 2) << "\n";
			outFile << flen << " " << intr.c << " " << intr.d << " " << intr.e << " " << intr.cx << " " << intr.cy;
			for (int idx = 0; idx < intr.coeffs.size(); idx++)
				outFile << " " << intr.coeffs[idx];
			outFile << "\n";
			outFile.close();
			char img_dir_origin[256];
			sprintf(img_dir_origin, "%s%05d-cam%d.jpg", img_dir.c_str(), i, j);
			//std::cout << img_dir_origin << std::endl;

			cv::Mat img_origin = cv::imread(img_dir_origin, cv::IMREAD_COLOR);
			cv::Mat img_tmp, img_rot, image;
			if (j != 5) {
				cv::transpose(img_origin, img_tmp);
				cv::flip(img_tmp, image, 0);
				//cv::imwrite(img_dir, image);
			}
			else image = img_origin;
			cv::Mat mapx_persp = cv::Mat(image.rows, image.cols, CV_32FC1);
			cv::Mat mapy_persp = cv::Mat(image.rows, image.cols, CV_32FC1);
			int width = mapx_persp.cols; //New width
			int height = mapx_persp.rows;//New height     
			float Nxc = height / 2.0;
			float Nyc = width / 2.0;
			float Nz = width * flen;
			Eigen::Vector3f vertex;
			Eigen::Vector2f pixel;
			for (int row = 0; row < height; row++) {
				for (int col = 0; col < width; col++) {
					vertex[1] = (row - Nxc);
					vertex[0] = (col - Nyc);
					vertex[2] = Nz;
					int coeff_num = intr.coeffs.size();
					float norm = std::sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1]);
					float theta = std::atan(-vertex[2] / norm);
					//printf("theta:%f\n", theta);
					float t, t_i;
					float rho, x, y;
					if (norm != 0) {
						t = theta;
						rho = intr.coeffs[0];
						t_i = 1;
						//printf("rho:  %f\n", rho);
						for (int i = 1; i < coeff_num; i++) {
							t_i *= t;
							rho += t_i*intr.coeffs[i];
							//printf("rho:  %f\n", rho);
						}
						x = vertex[1] * rho / norm;
						y = vertex[0] * rho / norm;
						pixel[0] = x*intr.c + y*intr.d + intr.cx;
						pixel[1] = x*intr.e + y + intr.cy;
						//printf("rho:  %f\n", rho);
						//printf("xy:%f %f\n", x, y);
					}
					else {
						pixel[0] = intr.cx;
						pixel[1] = intr.cy;
					}
					//std::cout << "pixel[0]: " << pixel[0] << "  pixel[1]: " << pixel[1] << std::endl;
					if (pixel[0] >= 0 && pixel[0] < height&&pixel[1] >= 0 && pixel[1] < width) {
						mapx_persp.at<float>(row, col) = pixel[1];
						mapy_persp.at<float>(row, col) = pixel[0];
						//std::cout << "mapx_persp: " << mapx_persp.at<float>(row, col) << " ";
						//std::cout << "mapy_persp: " << mapy_persp.at<float>(row, col) << std::endl;
					}
				}
			}
			/*for (int row = 0; row < height; row++) {
				for (int col = 0; col < width; col++) {
					std::cout << mapx_persp.at<float>(row, col) << "  "<< mapy_persp.at<float>(row, col) <<std::endl;
				}
			}*/
			cv::Mat image_persp, img1, img_RGBA, img2,img3;
			image_persp.create(image.rows, image.cols, image.type());
			cv::remap(image, image_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
			/*cv::cvtColor(image_persp, img_RGBA, CV_BGR2BGRA);
			for (int u = 0; u < img_RGBA.cols; u++) {
				for (int v = 0; v < img_RGBA.rows; v++) {
					float col = mapx_persp.at<float>(v, u);
					float row = mapy_persp.at<float>(v, u);
					if (row==0||col==0) {
						img_RGBA.at<cv::Vec4b>(v, u) = cv::Vec4b(0, 0, 0, 0);
					}
				}
			}*/
			img1.create(image_persp.size() / 4, CV_32SC3);
			cv::resize(image_persp, img1, image_persp.size()/4);
			cv::transpose(img1, img2);
			cv::flip(img2, img3, 1);
			//cv::Rect rect = cv::Rect(0, 0.1*img1.rows, img1.cols, 0.8*img1.rows);
			//img2 = img1(rect);

			char img_out[256];
			sprintf(img_out, "%s%05d-cam%d.png", scene_dir.c_str(), i, j);
			std::vector<int> pngCompressionParams;
			pngCompressionParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
			pngCompressionParams.push_back(0);
			cv::imwrite(img_out, img1, pngCompressionParams);
			//system("pause");
		} 
#endif
		end_time = clock();
		std::cout << "Converting time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;
		start_time = clock();
		Texturing(isRemoved);
		std::cout << "\tSaving model... " << std::flush;
		tex::Model::save(model, conf.out_prefix);
		//std::cout<< out_prefix_ << std::endl;
		std::cout << "done." << std::endl;
		end_time = clock();
		std::cout << "Texturing time: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;
	}

	void HWMesh::Texturing(bool isRemoved)
	{
		std::vector<math::Vec3f> faces_mean_color;
		std::string mesh_path = GetObjectName();
		std::string mesh_dir = mesh_path.substr(0, mesh_path.find_last_of("/"));
		std::string scene_dir = mesh_dir + "/scene";
		std::string mesh_head = mesh_path.substr(0, mesh_path.find_last_of("."));
		std::string occ_mesh_path = mesh_head + "_occ.ply";
		if (_access(mesh_path.c_str(), 0) == -1) {
			SavePly(mesh_path, PlyFormat::kBinary);
		}
		std::vector<float3>().swap(points_);
		std::vector<float3>().swap(normal_);
		std::vector<float3>().swap(faces_normal_);
		std::vector<uchar3>().swap(color_);
		std::vector<int3>().swap(faces_);
		Timer timer;
		util::WallTimer wtimer;
		conf.in_scene = scene_dir;
		if (isRemoved) {
			conf.in_mesh = mesh_path;
			conf.in_mesh_origin = occ_mesh_path;
		}
        else {
            conf.in_mesh = mesh_path;
        }
		std::string textured_mesh_dir = mesh_dir + "/obj";
		if (_access(textured_mesh_dir.c_str(), 0) == -1) {
			mkdir(textured_mesh_dir.c_str());
		}
		conf.out_prefix = textured_mesh_dir + "/textured";
		conf.settings.data_term = tex::DataTerm::DATA_TERM_AREA;
		conf.settings.outlier_removal = tex::OutlierRemoval::OUTLIER_REMOVAL_GAUSS_DAMPING;
		//conf.settings.global_seam_leveling = false;    // 颜色调整开关，默认为true
		//conf.settings.local_seam_leveling = false;
        conf.settings.keep_unseen_faces = true;
		conf.write_view_selection_model = false;

		if (!util::fs::dir_exists(util::fs::dirname(conf.out_prefix).c_str())) {
			std::cerr << "Destination directory does not exist!" << std::endl;
			std::exit(EXIT_FAILURE);
		}

		std::cout << "Load and prepare mesh: " << std::endl;
		std::cout << "    conf.in_mesh:        "<< conf.in_mesh << std::endl;
		std::cout << "    conf.in_mesh_origin: "<< conf.in_mesh_origin << std::endl;
		mve::TriangleMesh::Ptr mesh;
		try {
			mesh = mve::geom::load_ply_mesh(conf.in_mesh);
			//mesh_origin = mve::geom::load_ply_mesh(conf.in_mesh_origin);
		}
		catch (std::exception& e) {
			std::cerr << "\tCould not load mesh: " << e.what() << std::endl;
			system("pause");
			std::exit(EXIT_FAILURE);
		}
		mve::MeshInfo mesh_info(mesh);
		tex::prepare_mesh(&mesh_info, mesh);

		std::cout << "Generating texture views: " << std::endl;
		tex::TextureViews texture_views;

#if XHT_CONVERT_PANO	//1用瀚彤的文件格式，0用新定义的文件格式
		tex::generate_texture_views(conf.in_scene, &texture_views);
#else	
		tex::generate_texture_views_pinhole(conf.in_scene, &texture_views);
#endif		
		write_string_to_file(conf.out_prefix + ".conf", conf.to_string());
		timer.measure("Loading");

		std::size_t const num_faces = mesh->get_faces().size() / 3;

		std::cout << "Building adjacency graph: " << std::endl;
		tex::Graph graph(num_faces);
		tex::build_adjacency_graph(mesh, mesh_info, &graph);

		if (conf.labeling_file.empty()) {
			std::cout << "View selection:" << std::endl;
			util::WallTimer rwtimer;

			tex::DataCosts data_costs(num_faces, texture_views.size());
			tex::DataCosts face_luminance(num_faces, texture_views.size());
			if (conf.data_cost_file.empty()) {
				tex::calculate_data_costs(mesh, conf.in_mesh_origin, &texture_views, conf.settings, &data_costs, &face_luminance);

				if (conf.write_intermediate_results) {
					std::cout << "\tWriting data cost file... " << std::flush;
					tex::DataCosts::save_to_file(data_costs, conf.out_prefix + "_data_costs.spt");
					std::cout << "done." << std::endl;
				}
			}
			else {
				std::cout << "\tLoading data cost file... " << std::flush;
				try {
					tex::DataCosts::load_from_file(conf.data_cost_file, &data_costs);
				}
				catch (util::FileException e) {
					std::cout << "failed!" << std::endl;
					std::cerr << e.what() << std::endl;
					std::exit(EXIT_FAILURE);
				}
				std::cout << "done." << std::endl;
			}
			timer.measure("Calculating data costs");

			tex::view_selection(data_costs, &graph, conf.settings, texture_views.size(), face_luminance);
			data_costs.clear();
			face_luminance.clear();
			timer.measure("Running MRF optimization");
			std::cout << "\tTook: " << rwtimer.get_elapsed_sec() << "s" << std::endl;

			/* Write labeling to file. */
			if (conf.write_intermediate_results) {
				std::vector<std::size_t> labeling(graph.num_nodes());
				for (std::size_t i = 0; i < graph.num_nodes(); ++i) {
					labeling[i] = graph.get_label(i);
				}
				vector_to_file(conf.out_prefix + "_labeling.vec", labeling);
			}
		}
		else {
			std::cout << "Loading labeling from file... " << std::flush;

			/* Load labeling from file. */
			std::vector<std::size_t> labeling = vector_from_file<std::size_t>(conf.labeling_file);
			if (labeling.size() != graph.num_nodes()) {
				std::cerr << "Wrong labeling file for this mesh/scene combination... aborting!" << std::endl;
				std::exit(EXIT_FAILURE);
			}

			/* Transfer labeling to graph. */
			for (std::size_t i = 0; i < labeling.size(); ++i) {
				const std::size_t label = labeling[i];
				if (label > texture_views.size()) {
					std::cerr << "Wrong labeling file for this mesh/scene combination... aborting!" << std::endl;
					std::exit(EXIT_FAILURE);
				}
				graph.set_label(i, label);
			}

			std::cout << "done." << std::endl;
		}

		tex::TextureAtlases texture_atlases;
		{
			/* Create texture patches and adjust them. */
			tex::TexturePatches texture_patches;
			tex::VertexProjectionInfos vertex_projection_infos;
			std::cout << "Generating texture patches:" << std::endl;
			tex::generate_texture_patches(graph, mesh, mesh_info, &texture_views,
				conf.settings, &vertex_projection_infos, &texture_patches);

			if (conf.settings.global_seam_leveling) {
				std::cout << "Running global seam leveling:" << std::endl;
				tex::global_seam_leveling(graph, mesh, mesh_info, vertex_projection_infos, &texture_patches);
				timer.measure("Running global seam leveling");
			}
			else {
				ProgressCounter texture_patch_counter("Calculating validity masks for texture patches", texture_patches.size());
#pragma omp parallel for schedule(dynamic)
#if !defined(_MSC_VER)
				for (std::size_t i = 0; i < texture_patches.size(); ++i) {
#else
				for (std::int64_t i = 0; i < texture_patches.size(); ++i) {
#endif
					texture_patch_counter.progress<SIMPLE>();
					TexturePatch::Ptr texture_patch = texture_patches[i];
					std::vector<math::Vec3f> patch_adjust_values(texture_patch->get_faces().size() * 3, math::Vec3f(0.0f));
					texture_patch->adjust_colors(patch_adjust_values);
					texture_patch_counter.inc();
				}
				timer.measure("Calculating texture patch validity masks");
				}

			if (conf.settings.local_seam_leveling) {
				std::cout << "Running local seam leveling:" << std::endl;
				tex::local_seam_leveling(graph, mesh, vertex_projection_infos, &texture_patches);
			}
			timer.measure("Running local seam leveling");

			/* Generate texture atlases. */
			std::cout << "Generating texture atlases:" << std::endl;
			tex::generate_texture_atlases(&texture_patches, conf.settings, &texture_atlases);
			}

		/* Create and write out obj model. */
		{
			std::cout << "Building objmodel:" << std::endl;				
			tex::build_model(mesh, texture_atlases, &model);
			timer.measure("Building OBJ model");
			std::cout << "\tSaving model... " << std::flush;
			tex::Model::save(model, conf.out_prefix);
			std::cout << "done." << std::endl;
			timer.measure("Saving");
		}

		std::cout << "Whole texturing procedure took: " << wtimer.get_elapsed_sec() << "s" << std::endl;
		timer.measure("Total");
		//if (conf.write_timings) {
		//	timer.write_to_file(conf.out_prefix + "_timings.csv");
		//}
        
        // @zk for material
        {
            bool write_material_images = false;
            //if (write_material_images) {
            //    texture_atlases.clear();
            //    std::cout << "Generating material_decomposed_R texture patches:" << std::endl;
            //    {
            //        tex::TexturePatches texture_patches;

            //        //get_material_images(texture_views);
            //        for (auto& texture_view : texture_views) {
            //            std::string image_name = texture_view.get_image_name();
            //            std::string material_name = image_name.substr(0, image_name.size() - 4) + "_decomposed_R.png";
            //            texture_view.reset_image_name(material_name);
            //        }

            //        tex::VertexProjectionInfos vertex_projection_infos;
            //        tex::generate_texture_patches(graph, mesh, mesh_info, &texture_views,
            //            conf.settings, &vertex_projection_infos, &texture_patches);
            //        tex::generate_texture_atlases(&texture_patches, conf.settings, &texture_atlases);
            //    }

            //    std::cout << "Building material objmodel:" << std::endl;
            //    {
            //        tex::Model model;
            //        tex::build_model(mesh, texture_atlases, &model);
            //        std::cout << "\tSaving model... " << std::flush;
            //        tex::Model::save(model, conf.out_prefix + "_decomposed_R");
            //        std::cout << "done." << std::endl;
            //    }
            //}

            if (write_material_images) {
                texture_atlases.clear();
                std::cout << "Generating material_materialSeg_f0_roughness texture patches:" << std::endl;
                {
                    tex::TexturePatches texture_patches;

                    //get_material_images(texture_views);
                    for (auto& texture_view : texture_views) {
                        std::string image_name = texture_view.get_image_name();
                        std::string material_name = image_name.substr(0, image_name.size() - 17) + "_materialSeg_f0_roughness.png";
                        texture_view.reset_image_name(material_name);
                    }

                    tex::VertexProjectionInfos vertex_projection_infos;
                    tex::generate_texture_patches(graph, mesh, mesh_info, &texture_views,
                        conf.settings, &vertex_projection_infos, &texture_patches);
                    tex::generate_texture_atlases(&texture_patches, conf.settings, &texture_atlases);
                }

                std::cout << "Building material objmodel:" << std::endl;
                {
                    tex::Model model;
                    tex::build_model(mesh, texture_atlases, &model);
                    std::cout << "\tSaving model... " << std::flush;
                    tex::Model::save(model, conf.out_prefix + "_materialSeg_f0_roughness");
                    std::cout << "done." << std::endl;
                }
            }

            if (write_material_images) {
                texture_atlases.clear();
                std::cout << "Generating material_materialSeg_rhos texture patches:" << std::endl;
                {
                    tex::TexturePatches texture_patches;

                    //get_material_images(texture_views);
                    for (auto& texture_view : texture_views) {
                        std::string image_name = texture_view.get_image_name();
                        std::string material_name = image_name.substr(0, image_name.size() - 29) + "_materialSeg_rhos.png";
                        texture_view.reset_image_name(material_name);
                    }

                    tex::VertexProjectionInfos vertex_projection_infos;
                    tex::generate_texture_patches(graph, mesh, mesh_info, &texture_views,
                        conf.settings, &vertex_projection_infos, &texture_patches);
                    tex::generate_texture_atlases(&texture_patches, conf.settings, &texture_atlases);
                }

                std::cout << "Building material objmodel:" << std::endl;
                {
                    tex::Model model;
                    tex::build_model(mesh, texture_atlases, &model);
                    std::cout << "\tSaving model... " << std::flush;
                    tex::Model::save(model, conf.out_prefix + "_materialSeg_rhos");
                    std::cout << "done." << std::endl;
                }
            }
        }

		if (conf.write_view_selection_model) {
			texture_atlases.clear();
			std::cout << "Generating debug texture patches:" << std::endl;
			{
				tex::TexturePatches texture_patches;
				generate_debug_embeddings(&texture_views);
				tex::VertexProjectionInfos vertex_projection_infos; // Will only be written
				tex::generate_texture_patches(graph, mesh, mesh_info, &texture_views,
					conf.settings, &vertex_projection_infos, &texture_patches);
				tex::generate_texture_atlases(&texture_patches, conf.settings, &texture_atlases);
			}
		
			std::cout << "Building debug objmodel:" << std::endl;
			{
				tex::Model model;
				tex::build_model(mesh, texture_atlases, &model);
				std::cout << "\tSaving model... " << std::flush;
				tex::Model::save(model, conf.out_prefix + "_view_selection");
				std::cout << "done." << std::endl;
			}
		}
	}

	int2 HWMesh::Vert2uv(float3& vert, int panoID)
	{
		Eigen::Vector3f d1 = Eigen::Vector3f(vert.x, vert.y, vert.z) - camera_pos_vec_[panoID];//顶点到相机位置
		float d_norm = d1.norm();

		Eigen::Vector3f d_xy = d1 - d1.dot(rotation_matrix_vec_[panoID].col(2)) / rotation_matrix_vec_[panoID].col(2).dot(rotation_matrix_vec_[panoID].col(2))* rotation_matrix_vec_[panoID].col(2);
		float cosa = rotation_matrix_vec_[panoID].col(1).dot(d_xy) / rotation_matrix_vec_[panoID].col(1).norm() / d_xy.norm();
		if (cosa > 1.0f)
			cosa = 1.0f;
		else if (cosa < -1.0f)
			cosa = -1.0f;
		float a = std::acos(cosa) * 180 / M_PI;
		float cosx = rotation_matrix_vec_[panoID].col(0).dot(d_xy);
		if (cosx < 0) {
			a = -a;
			if (a <= -90)
				a = 360 + a;
		}
		float cosb = rotation_matrix_vec_[panoID].col(2).dot(d1) / rotation_matrix_vec_[panoID].col(2).norm() / d_norm;
		float b = std::acos(cosb) * 180 / M_PI;
		b -= 90;
		int u = 1.0f * a / 360.0f * pano_width_ + 1.0f * (pano_width_ - 1) / 4;
		if (u < 0)
			u = 0;
		else if (u >= pano_width_)
			u = pano_width_ - 1;
		int v = 1.0f * b / 90.0f * (pano_height_ / 2) + 1.0f*(pano_height_ - 1) / 2;
		if (v < 0)
			v = 0;
		else if (v >= pano_height_)
			v = pano_height_ - 1;
		return make_int2(u, v);
	}

	cv::Point HWMesh::RayExtention(int2 uv, float cx, float cy)
	{
		cv::Point v;
		float rayx = uv.x - cx, rayy = uv.y - cy;
		float min = abs(rayx) < abs(rayy) ? abs(rayx) : abs(rayy);
		if (min == 0)
			min = abs(rayx) > abs(rayy) ? abs(rayx) : abs(rayy);
		float offsetx = 2 * rayx / min;
		if (abs(offsetx) > 5)
			offsetx = 5 * abs(rayx) / rayx;
		v.x = uv.x + offsetx;
		if (v.x < 0) {
			v.x = 0;
			v.y = uv.y;
		}
		else {
			float offsety = 2 * rayy / min;
			if (abs(offsety) > 5)
				offsety = 5 * abs(rayy) / rayy;
			v.y = uv.y + offsety;
			if (v.y < 0) {
				v.y = 0;
				v.x = uv.x;
			}
			if (v.y >= pano_height_) {
				v.y = pano_height_;
				v.x = uv.x;
			}
		}
		return v;
	}

	//bool HWMesh::ConstructVerIdxToMeshIdx()
	//{
	//	//
	//	for(int i = 0; i )
	//}
	}