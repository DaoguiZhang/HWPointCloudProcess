#pragma once
#ifndef HW_CAM_MODEL_H
#define HW_CAM_MODEL_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <io.h>
#include "tinyxml.h"
#include "json.hpp"
#include<opencv2/opencv.hpp>

namespace HW {

	enum CamFromOrigin {
		kScene = 0,
		kPhotos,
		kOther
	};

	typedef struct Intrinsic
	{
		float fx, fy, cx, cy;
		std::vector<float> distortion;
	} Intrinsic;

	class HWCAMModel
	{
	public:
		HWCAMModel();
		~HWCAMModel();
		void ReadIntrinsic(std::string path);

		/*
		目的：读取.xml文件的相机的参数，参数然后转化为相机的位置T_wc，从相机坐标系到世界坐标系；
		内参直接读进去，后续再处理
		输入：.xml文件
		输出：相机的T_wc;内参
		*/
		void ReadIntrinsicExtrinsic(std::string path);

		int GetCamsSize();
		int GetImagesPthsSize();
		void ReadExtrinsic(std::string dir);
		void GetCurrentDirFiles(const std::string & path, std::vector<std::string> & files);

		void GetAllDirFiles(const std::string & path, std::vector<std::string> & files);

		/*
		目的：获取真实的图片的位置,文件读取的图片的位置不准确
		输入：真实的图片位置
		输出：图片的真实位置放入到images_path_上,和原始的相机位置要匹配才行
		*/
		void ReadRealImagesPath(const std::vector<std::string>& real_path);

		/*
		目的：设置图片的分辨率
		输入：img_ratio
		输出：
		*/
		void SetImageRatio(float img_ratio);

		/*
		目的：设置采样的图片
		输入：sample_ratio
		输出：
		*/
		void SetSampleImageRatio(float sample_ratio);

		/*
		目的：设置图片的输出的Dir
		输入：输出图片的dir
		输出：images_out_dir
		*/
		void SetImgOutDir(const std::string& dir);

		/*
		目的：输出相机位置，包括内参和外参和ratio匹配
		输入：
		输出：
		*/
		void SaveCamsIntoCam();

		/*
		目的：把相机位姿存为OBJ格式
		输入：相机的索引，相机的文件目录
		输出：
		*/
		//这是采样的顶点
		void SaveCamPoseIntoOBJ(std::vector<Eigen::Matrix4f>& cams_pose, const std::string& path);
		//这是只需要画两个顶点，优先选择这个部分画出相机位姿
		void SaveCamsPoseIntoObjFile(std::vector<Eigen::Matrix4f>& cams_pose, 
			const std::string& file_path, const float& aix_len);

		/*
		目的：把相机位姿存为OBJ格式
		输入：相机的索引，相机的文件目录
		输出：
		*/
		void DrawCamsPoseIntoOBJ(std::vector<int> selects_idx, const std::string& path);

		/*
		目的：输出图片,和ratio相匹配
		输入：
		输出：images_out_dir
		*/
		void SaveImagesIntoPNG();

		/*
		目的：输出图片,和ratio相匹配
		输入：
		输出：images_out_dir
		*/
		void SaveSurplusImagesIntoTXT(std::vector<std::string>& surplus_names);

	private:
		
		float image_ratio_;
		int sample_ratio_;
		std::string out_dir_;

		int image_width_;
		int image_height_;
		std::vector<std::string> images_path_;
		std::vector<Intrinsic> intrinsics_;
		std::vector<Eigen::Matrix4f> T_WC_;	//相机坐标系到世界坐标系
	};
}
#endif // !1

