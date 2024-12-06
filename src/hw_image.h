#pragma once
#ifndef HW_IMAGE_ZDG_H
#define HW_IMAGE_ZDG_H

#include"hw_cmns.h"
//#include"util/bitmap.h"
#include<opencv2/opencv.hpp>
#include<opencv2/xfeatures2d.hpp>
#include<opencv2/xfeatures2d/nonfree.hpp>
#include<opencv2/line_descriptor.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>

namespace HW
{
	struct HWImageSiftOptions
	{
		//sift detect params, to do next
		int hw_nfeatures = 0;
		int hw_nOctaveLayers = 3;
		double hw_contrastThreshold = 0.04;
		double hw_edgeThreshold = 10;
		double hw_sigma = 1.5;
	};

	class HWImage
	{
	public:

		HWImage();
		HWImage(const HWImage& other);
		HWImage(HWImage& other);
		~HWImage();
		void SetHWImagePath(std::string& path);
		void SetImageId(unsigned int mid);
		void SetCamId(unsigned int cid);
		void SetLayoutId(unsigned int lid);
		void SetSiftOption(const HWImageSiftOptions& opt);
		void SetSiftKeyPoints(const std::vector<cv::KeyPoint> positions);
		void SetImageNetWorkPntsPos(const std::vector<cv::Point2f>& pnts);
		void SetImageNerWorkColor(const Eigen::Vector3i& c);
		void LoadHWImage();
		void ComputeSiftFeatures();
		void DetectLinesSegsOpencv();
		void DetectLinesSegsOpencvLbd();
		void DetectLineSegsOpencvLsd();
		void DrawSingleLine(Eigen::Vector2f& ls, Eigen::Vector2f& le, cv::Vec3i& c);
		const std::string& GetImagePath();
		const std::string& GetImagePath() const;
		const unsigned int& GetCamId();
		const unsigned int& GetCamId() const;
		const unsigned int& GetImageId();
		const unsigned int& GetImageId() const;
		const unsigned int& GetLayoutId();
		const unsigned int& GetLayoutId() const;

		const std::vector<cv::KeyPoint>& GetSiftFeatruesPosition();
		const std::vector<cv::KeyPoint>& GetSiftFeatruesPosition() const;
		const cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor>& GetSiftDescriptorPtr();
		const cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor>& GetSiftDescriptorPtr() const;
		
		const std::vector<cv::Vec4f>& GetLinesSegmentsOpencv() const;
		const std::vector<cv::Vec4f>& GetLinesSegmentsOpencv();
		
		const std::vector<cv::line_descriptor::KeyLine> GetLbdImageKeyLinesOpencv();
		const std::vector<cv::line_descriptor::KeyLine> GetLbdImageKeyLinesOpencv() const;
		const cv::Mat& GetLbdImageLinesDescriptors();
		const cv::Mat& GetLbdImageLinesDescriptors() const;

		const std::vector<cv::line_descriptor::KeyLine> GetLsdImageKeyLinesOpencv();
		const std::vector<cv::line_descriptor::KeyLine> GetLsdImageKeyLinesOpencv() const;
		const cv::Mat& GetLsdImageLinesDescriptors();
		const cv::Mat& GetLsdImageLinesDescriptors() const;

		const std::vector<cv::Point2f> GetImageNetWorkPnts();
		const std::vector<cv::Point2f> GetImageNetWorkPnts() const;

		void WriteNetWorkPntsIntoOwnImage(const std::string& path);

		bool ImageLoaded();
		const cv::Mat& GetImage();
		const cv::Mat& GetImage() const;

		bool GetImageNetworkPntsLoaded();

		HWImage operator=(const HWImage& other);

	private:

		unsigned int cam_id_;
		unsigned int image_id_;
		unsigned int layout_id_;

		HWImageSiftOptions opt_;

		/** Per-feature image position. */
		std::vector<cv::KeyPoint> positions_;
		/** Per-feature image color. */
		std::vector<cv::Vec3b> img_colors_;
		bool features_detected_;

		std::string image_path_;
		bool load_img_;
		cv::Mat image_;

		std::vector<cv::Vec4f> lines_segments_;
		cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> sift_descriptor_;

		std::vector<cv::line_descriptor::KeyLine> lbd_octave_;
		cv::Mat lbd_;

		std::vector<cv::line_descriptor::KeyLine> lsd_octave_;
		cv::Mat lsd_descriptor_;

		bool loaded_network_pnts_;
		std::vector<cv::Point2f> image_network_pnts_;	//load from network points
		Eigen::Vector3i hw_image_rgb_;
	};
}


#endif // !HW_IMAGE_H


