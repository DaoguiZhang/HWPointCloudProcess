#pragma once
#ifndef HW_MESH_H
#define HW_MESH_H

#include <opencv2/opencv.hpp>
#include "hw_object.h"
#include "ScalableTSDFVolume.h"
#include "io.h"
#include "OpenGLWidget.h"
#include "OCamModel.h"
#include "OpenGLWindow.h"

#include <util/timer.h>
#include <util/system.h>
#include <util/file_system.h>
#include <mve/mesh_io_ply.h>

#include "util.h"
#include "timer.h"
#include "debug.h"
#include "texturing.h"
#include "progress_counter.h"
#include "arguments.h"
#include "HalfEdge.h"
namespace open3d
{
	class ScalableTSDFVolume;
}

namespace HW
{
	class FaceVersTexNormIdx
	{
	public:
		int3 vers_idx_;
		int3 normals_idx_;
		int3 tex_idx_;
	};

	/*class TextureMesh
	{
	std::vector<float3> vertices_;
	std::vector<float2> tex_vertices_;
	std::vector<float3> noramls_;
	std::vector<FaceVersTexNormIdx> faces_info_;
	};*/

	class HWMesh :public HWObject
	{
	public:
		HWMesh();
		~HWMesh();
		void Clear();
		bool Show();
		bool Save(std::string& file);
		bool ReadPly(const std::string& file);
		//bool ReadPlyStream(tinyply::PlyFile& ply_file);
		bool SavePly(const std::string& file, const PlyFormat& type);

		//存为OBJ格式文件，这个函数只对Mesh
		bool SaveObj(const std::string& file);

		bool HasNormal();
		bool HasColor();

		const std::vector<float3> &GetVertices();
		const std::vector<float3> &GetNormal();
		const std::vector<uchar3> &GetPointsColor();
		const std::vector<int3> &GetFaces();
		open3d::ScalableTSDFVolume * GetVolume();
		void SetVolume(open3d::ScalableTSDFVolume * in_volume);

		float3 GetAPoint(int idx);

		//copy data from file
		void CopyPointsFromPlyData(std::shared_ptr<tinyply::PlyData>& point_element);

		void CopyNormalsFromPlyData(std::shared_ptr<tinyply::PlyData>& normal_element);

		void CopyFacesFromPlyData(std::shared_ptr<tinyply::PlyData>& face_element);

		void CopyPointsColorsFromPlyData(std::shared_ptr<tinyply::PlyData>& point_color_element);

		void AddPoint(float3& point);
		void AddNormal(float3& point_normal);
		void AddColor(uchar3& point_color);
		void AddFaceIdx(int3& face_idx);
		//void AddFaceIdxInt(int x_idx, int y_idx, int z_idx);

		void ChangeColor(uchar3& point_color, int idx);

		//Check mesh
		bool HasFacesNormals();

		bool HasTextureCoord();

		/// Function to compute triangle normals, usually called before rendering
		void ComputeTriNormals(bool normalized = true);

		/// Function to compute vertex normals, usually called before rendering
		void ComputeVersNormals(bool normalized = true);

		//normalized norm
		void NormalizeNormals();

		/// Function to remove duplicated and non-manifold vertices/triangles
		void Purge();
		void edgeSwap();
		//-------------texture----------------//
		void DoTextureMapping(bool isRemoved);
		void ConvertPano();
        void ConvertPano(std::string folderName);
		void Texturing(bool isRemoved);
		
		int2 Vert2uv(float3& vert, int panoID);
		static bool comp(const int2& a, const int2& b)
		{
			if (a.y != b.y)
				return a.y < b.y;
			else return a.x < b.x;
		}
		float tri_area(const int2& uv1, const int2& uv2, const int2& uv3) {
			return 0.5f*abs(uv1.x*uv2.y + uv2.x*uv3.y + uv3.x*uv1.y - uv1.y*uv2.x - uv2.y*uv3.x - uv3.y*uv1.x);
		}
		cv::Point RayExtention(int2 uv, float cx, float cy);

	protected:
		void RemoveDuplicatedVertices();
		void RemoveDuplicatedTriangles();
		void RemoveNonManifoldVertices();
		void RemoveNonManifoldTriangles();

		//输入顶点，和face idx 
		//bool ConstructVerIdxToMeshIdx();

	private:
		int points_num_;
		int faces_num_;
		std::vector<float3> points_;
		std::vector<float3> normal_;
		std::vector<float3> faces_normal_;
		std::vector<uchar3> color_;
		std::vector<int3> faces_;

		//volume
		open3d::ScalableTSDFVolume *scale_volume_;

		std::vector<float2> faces_tex_coord_;

		//std::vector<int> pano_face_idx_;
		//std::vector<int> pano_faces_num_;

		//std::vector<std::string> pano_images_path_;

		std::vector<int> texture2faceID_;

		std::vector<FaceVersTexNormIdx> faces_info_;

		int pano_width_, pano_height_;
		std::vector<Eigen::Vector3f> camera_pos_vec_;
		std::vector<Eigen::Matrix3f> rotation_matrix_vec_;
		//std::vector<cv::Mat> pano2meshID_vec_;
		std::vector<std::vector<bool>> pano2mesh_vec_;
		//std::vector<cv::Mat> pano_depth_vec_;
		struct Pano_info {
			int panoID;
			int2 uv1, uv2, uv3;
			float area;
		};
		tex::Model model;
		Arguments conf;
		OpenGLWindow* openGLWindow_;
	};
}
#endif