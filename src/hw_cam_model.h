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
		Ŀ�ģ���ȡ.xml�ļ�������Ĳ���������Ȼ��ת��Ϊ�����λ��T_wc�����������ϵ����������ϵ��
		�ڲ�ֱ�Ӷ���ȥ�������ٴ���
		���룺.xml�ļ�
		����������T_wc;�ڲ�
		*/
		void ReadIntrinsicExtrinsic(std::string path);

		int GetCamsSize();
		int GetImagesPthsSize();
		void ReadExtrinsic(std::string dir);
		void GetCurrentDirFiles(const std::string & path, std::vector<std::string> & files);

		void GetAllDirFiles(const std::string & path, std::vector<std::string> & files);

		/*
		Ŀ�ģ���ȡ��ʵ��ͼƬ��λ��,�ļ���ȡ��ͼƬ��λ�ò�׼ȷ
		���룺��ʵ��ͼƬλ��
		�����ͼƬ����ʵλ�÷��뵽images_path_��,��ԭʼ�����λ��Ҫƥ�����
		*/
		void ReadRealImagesPath(const std::vector<std::string>& real_path);

		/*
		Ŀ�ģ�����ͼƬ�ķֱ���
		���룺img_ratio
		�����
		*/
		void SetImageRatio(float img_ratio);

		/*
		Ŀ�ģ����ò�����ͼƬ
		���룺sample_ratio
		�����
		*/
		void SetSampleImageRatio(float sample_ratio);

		/*
		Ŀ�ģ�����ͼƬ�������Dir
		���룺���ͼƬ��dir
		�����images_out_dir
		*/
		void SetImgOutDir(const std::string& dir);

		/*
		Ŀ�ģ�������λ�ã������ڲκ���κ�ratioƥ��
		���룺
		�����
		*/
		void SaveCamsIntoCam();

		/*
		Ŀ�ģ������λ�˴�ΪOBJ��ʽ
		���룺�����������������ļ�Ŀ¼
		�����
		*/
		//���ǲ����Ķ���
		void SaveCamPoseIntoOBJ(std::vector<Eigen::Matrix4f>& cams_pose, const std::string& path);
		//����ֻ��Ҫ���������㣬����ѡ��������ֻ������λ��
		void SaveCamsPoseIntoObjFile(std::vector<Eigen::Matrix4f>& cams_pose, 
			const std::string& file_path, const float& aix_len);

		/*
		Ŀ�ģ������λ�˴�ΪOBJ��ʽ
		���룺�����������������ļ�Ŀ¼
		�����
		*/
		void DrawCamsPoseIntoOBJ(std::vector<int> selects_idx, const std::string& path);

		/*
		Ŀ�ģ����ͼƬ,��ratio��ƥ��
		���룺
		�����images_out_dir
		*/
		void SaveImagesIntoPNG();

		/*
		Ŀ�ģ����ͼƬ,��ratio��ƥ��
		���룺
		�����images_out_dir
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
		std::vector<Eigen::Matrix4f> T_WC_;	//�������ϵ����������ϵ
	};
}
#endif // !1

