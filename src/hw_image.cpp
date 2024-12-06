#include"hw_image.h"
#include<boost/filesystem.hpp>


namespace HW
{
	HWImage::HWImage()
	{
		cam_id_ = KMAXMUMLIMIT;
		image_id_ = KMAXMUMLIMIT;
		layout_id_ = KMAXMUMLIMIT;
		load_img_ = false;
		features_detected_ = false;
		loaded_network_pnts_ = false;
		hw_image_rgb_ = Eigen::Vector3i(255, 0, 0);
	}

	HWImage::HWImage(HWImage& other)
	{
		this->cam_id_ = other.cam_id_;
		this->image_id_ = other.image_id_;
		this->layout_id_ = other.layout_id_;
		this->opt_ = other.opt_;
		this->image_path_ = other.image_path_;
		this->load_img_ = other.load_img_;
		this->image_ = other.image_;
		this->positions_ = other.positions_;
		this->img_colors_ = other.img_colors_;
		this->features_detected_ = other.features_detected_;
		this->sift_descriptor_ = other.sift_descriptor_;
		this->lines_segments_ = other.lines_segments_;
		this->lbd_ = other.lbd_;
		this->lbd_octave_ = other.lbd_octave_;
		this->lsd_descriptor_ = other.lsd_descriptor_;
		this->lsd_octave_ = other.lsd_octave_;
		this->image_network_pnts_ = other.image_network_pnts_;
		this->loaded_network_pnts_ = other.loaded_network_pnts_;
		this->hw_image_rgb_ = other.hw_image_rgb_;
	}

	HWImage::HWImage(const HWImage& other)
	{
		this->cam_id_ = other.cam_id_;
		this->image_id_ = other.image_id_;
		this->layout_id_ = other.layout_id_;
		this->opt_ = other.opt_;
		this->image_path_ = other.image_path_;
		this->load_img_ = other.load_img_;
		this->image_ = other.image_;
		this->positions_ = other.positions_;
		this->img_colors_ = other.img_colors_;
		this->features_detected_ = other.features_detected_;
		this->sift_descriptor_ = other.sift_descriptor_;
		this->lines_segments_ = other.lines_segments_;
		this->lbd_ = other.lbd_;
		this->lbd_octave_ = other.lbd_octave_;
		this->lsd_descriptor_ = other.lsd_descriptor_;
		this->lsd_octave_ = other.lsd_octave_;
		this->image_network_pnts_ = other.image_network_pnts_;
		this->loaded_network_pnts_ = other.loaded_network_pnts_;
		this->hw_image_rgb_ = other.hw_image_rgb_;
	}


	HWImage::~HWImage()
	{

	}

	void HWImage::SetHWImagePath(std::string& path)
	{
		image_path_ = path;
	}

	void HWImage::SetImageId(unsigned int mid)
	{
		image_id_ = mid;
	}

	void HWImage::SetCamId(unsigned int cid)
	{
		cam_id_ = cid;
	}

	void HWImage::SetLayoutId(unsigned int lid)
	{
		layout_id_ = lid;
	}

	void HWImage::SetSiftOption(const HWImageSiftOptions& opt)
	{
		opt_ = opt;
	}

	void HWImage::SetSiftKeyPoints(const std::vector<cv::KeyPoint> positions)
	{
		positions_.resize(positions.size());
		for (int i = 0; i < positions.size(); ++i)
		{
			positions_[i] = positions[i];
		}
	}

	void HWImage::SetImageNetWorkPntsPos(const std::vector<cv::Point2f>& pnts)
	{
		image_network_pnts_.resize(pnts.size());
		for (int i = 0; i < pnts.size(); ++i)
		{
			image_network_pnts_[i] = pnts[i];
		}
		loaded_network_pnts_ = true;
	}

	void HWImage::SetImageNerWorkColor(const Eigen::Vector3i& c)
	{
		hw_image_rgb_ = c;
	}

	void HWImage::LoadHWImage()
	{
		if (boost::filesystem::is_regular_file(image_path_))
		{
			image_ = cv::imread(image_path_);
			load_img_ = true;
			return;
		}
		load_img_ = false;
	}

	void HWImage::ComputeSiftFeatures()
	{
		//std::cerr << "start to sift feature" << std::endl;
		if (!load_img_)
		{
			//std::cerr << "the image load path: " << image_path_ << std::endl;
			LoadHWImage();
		}
		if (!load_img_)
		{
			std::cerr << "load image failed... please check it... " << std::endl;
			return;
		}
		sift_descriptor_ = cv::xfeatures2d::SIFT::create(
			opt_.hw_nfeatures, opt_.hw_nOctaveLayers,
			opt_.hw_contrastThreshold, opt_.hw_edgeThreshold,
			opt_.hw_sigma);

		sift_descriptor_->detect(image_, positions_);
		std::cerr << "the sift kepoints num: " << positions_.size() << std::endl;
	}

	void HWImage::DetectLinesSegsOpencv()
	{
		if (!image_.empty())
		{
			lines_segments_.clear();
			//set lsd params later(to do next...)
			std::vector<cv::Vec4f> lsegs;
			cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
			cv::Mat image_gray;
			cv::cvtColor(image_, image_gray, cv::COLOR_RGB2GRAY);
			//cv::COLOR_BGR2GRAY
			//std::cerr << "start to detect my lines..." << std::endl;
			ls->detect(image_gray, lines_segments_);
			//cv::Mat draw_image_lines(image_);
			//ls->drawSegments(draw_image_lines, lines_segments_);
			//cv::imshow("image_lines", draw_image_lines);
			//cv::waitKey(0);
			//std::cerr << "end detect my lines..." << std::endl;
		}
		else
		{
			std::cerr << "Error: empty image mat..." << std::endl;
		}
	}

	void HWImage::DetectLinesSegsOpencvLbd()
	{
		//lines_segments_.clear();
		//set lsd params later(to do next...)
		//std::vector<cv::Vec4f> lsegs;
		if (!image_.empty())
		{
			//cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
			//cv::Mat image_gray;
			//cv::cvtColor(image_, image_gray, cv::COLOR_RGB2GRAY);
			////cv::COLOR_BGR2GRAY
			//std::cerr << "start to detect my lines..." << std::endl;
			//ls->detect(image_gray, lines_segments_);
			////cv::Mat draw_image_lines(image_);
			////ls->drawSegments(draw_image_lines, lines_segments_);
			////cv::imshow("image_lines", draw_image_lines);
			////cv::waitKey(0);
			//std::cerr << "end detect my lines..." << std::endl;

			/* create binary masks */
			cv::Mat mask = cv::Mat::ones(image_.size(), CV_8UC1);
			/* create a pointer to a BinaryDescriptor object with default parameters */
			cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
			/* compute lines and descriptors */
			std::vector<cv::line_descriptor::KeyLine> keylines_;
			cv::Mat descr;
			(*bd)(image_, mask, keylines_, descr, false, false);
			/* select keylines from first octave and their descriptors */
			for (int i = 0; i < (int)keylines_.size(); i++)
			{
				if (keylines_[i].octave == 0)
				{
					lbd_octave_.push_back(keylines_[i]);
					lbd_.push_back(descr.row(i));
				}
			}
			lines_segments_.clear();
			for (int i = 0; i < lbd_octave_.size(); ++i)
			{
				cv::Vec4f kline;
				cv::Point2f s = lbd_octave_[i].getStartPoint();
				cv::Point2f e = lbd_octave_[i].getEndPoint();
				kline[0] = s.x;
				kline[1] = s.y;
				kline[2] = e.x;
				kline[3] = e.y;
				lines_segments_.emplace_back(kline);
			}
		}
		else
		{
			std::cerr << "Error: empty image mat..." << std::endl;
		}
	}

	void HWImage::DetectLineSegsOpencvLsd()
	{
		if (!image_.empty())
		{
			/* create binary masks */
			cv::Mat mask = cv::Mat::ones(image_.size(), CV_8UC1);
			/* create a pointer to a BinaryDescriptor object with default parameters */
			cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();

			cv::Ptr<cv::line_descriptor::LSDDetector> lsd = cv::line_descriptor::LSDDetector::createLSDDetector();
			/* detect lines */
			std::vector<cv::line_descriptor::KeyLine> klsd;
			cv::Mat lsd_descr;
			lsd->detect(image_, klsd, 2, 2, mask);

			/* compute descriptors for lines from first octave */
			bd->compute(image_, klsd, lsd_descr);

			/* select lines and descriptors from first octave */
			lsd_octave_.clear();
			lsd_descriptor_.release();
			for (int i = 0; i < (int)klsd.size(); i++)
			{
				if (klsd[i].octave == 1)
				{
					lsd_octave_.push_back(klsd[i]);
					lsd_descriptor_.push_back(lsd_descr.row(i));
				}
			}

			lines_segments_.clear();
			for (int i = 0; i < lsd_octave_.size(); ++i)
			{
				cv::Vec4f kline;
				cv::Point2f s = lsd_octave_[i].getStartPoint();
				cv::Point2f e = lsd_octave_[i].getEndPoint();
				kline[0] = s.x;
				kline[1] = s.y;
				kline[2] = e.x;
				kline[3] = e.y;
				lines_segments_.emplace_back(kline);
			}
		}
		else
		{
			std::cerr << "Error: empty image mat..." << std::endl;
		}
	}

	void HWImage::DrawSingleLine(Eigen::Vector2f& ls, Eigen::Vector2f& le, cv::Vec3i& c)
	{
		//to draw single line
		std::cerr << "to do next..." << std::endl;
	}

	const std::string& HWImage::GetImagePath()
	{
		return image_path_;
	}

	const std::string& HWImage::GetImagePath() const
	{
		return image_path_;
	}

	const unsigned int& HWImage::GetCamId()
	{
		return cam_id_;
	}

	const unsigned int& HWImage::GetCamId() const
	{
		return cam_id_;
	}

	const unsigned int& HWImage::GetImageId()
	{
		return image_id_;
	}

	const unsigned int& HWImage::GetImageId() const
	{
		return image_id_;
	}

	const unsigned int& HWImage::GetLayoutId()
	{
		return layout_id_;
	}

	const unsigned int& HWImage::GetLayoutId() const
	{
		return layout_id_;
	}

	const std::vector<cv::KeyPoint>& HWImage::GetSiftFeatruesPosition()
	{
		return positions_;
	}

	const std::vector<cv::KeyPoint>& HWImage::GetSiftFeatruesPosition() const
	{
		return positions_;
	}

	const cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor>& HWImage::GetSiftDescriptorPtr()
	{
		return sift_descriptor_;
	}

	const cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor>& HWImage::GetSiftDescriptorPtr() const
	{
		return sift_descriptor_;
	}

	const std::vector<cv::Vec4f>& HWImage::GetLinesSegmentsOpencv() const
	{
		return lines_segments_;
	}

	const std::vector<cv::Vec4f>& HWImage::GetLinesSegmentsOpencv()
	{
		return lines_segments_;
	}

	const std::vector<cv::line_descriptor::KeyLine> HWImage::GetLbdImageKeyLinesOpencv()
	{
		return lbd_octave_;
	}

	const std::vector<cv::line_descriptor::KeyLine> HWImage::GetLbdImageKeyLinesOpencv() const
	{
		return lbd_octave_;
	}

	const cv::Mat& HWImage::GetLbdImageLinesDescriptors()
	{
		return lbd_;
	}

	const cv::Mat& HWImage::GetLbdImageLinesDescriptors() const
	{
		return lbd_;
	}

	const std::vector<cv::line_descriptor::KeyLine> HWImage::GetLsdImageKeyLinesOpencv()
	{
		return lsd_octave_;
	}

	const std::vector<cv::line_descriptor::KeyLine> HWImage::GetLsdImageKeyLinesOpencv() const
	{
		return lsd_octave_;
	}

	const cv::Mat& HWImage::GetLsdImageLinesDescriptors()
	{
		return lsd_descriptor_;
	}

	const cv::Mat& HWImage::GetLsdImageLinesDescriptors() const
	{
		return lsd_descriptor_;
	}

	const std::vector<cv::Point2f> HWImage::GetImageNetWorkPnts()
	{
		return image_network_pnts_;
	}

	const std::vector<cv::Point2f> HWImage::GetImageNetWorkPnts() const
	{
		return image_network_pnts_;
	}

	void HWImage::WriteNetWorkPntsIntoOwnImage(const std::string& path)
	{
		cv::Mat image = cv::imread(image_path_);
		for (int i = 0; i < image_network_pnts_.size(); ++i)
		{
			cv::Point2f p = image_network_pnts_[i];
			cv::circle(image, p, 5, cv::Scalar(hw_image_rgb_[0], hw_image_rgb_[1], hw_image_rgb_[2]), 2);
		}
		cv::imwrite(path, image);
	}

	bool HWImage::ImageLoaded()
	{
		return load_img_;
	}

    const cv::Mat& HWImage::GetImage()
    {
        return image_;
    }

	const cv::Mat& HWImage::GetImage() const
	{
		return image_;
	}

	bool HWImage::GetImageNetworkPntsLoaded()
	{
		return loaded_network_pnts_;
	}

    HWImage HWImage::operator=(const HWImage& other)
    {
		
		this->cam_id_ = other.cam_id_;
		this->image_id_ = other.image_id_;
		this->layout_id_ = other.layout_id_;
		this->opt_ = other.opt_;
		this->image_path_ = other.image_path_;
		this->load_img_ = other.load_img_;
		this->image_ = other.image_;
		this->positions_ = other.positions_;
		this->img_colors_ = other.img_colors_;
		this->features_detected_ = other.features_detected_;
		this->sift_descriptor_ = other.sift_descriptor_;
		this->lines_segments_ = other.lines_segments_;
		this->lbd_ = other.lbd_;
		this->lbd_octave_ = other.lbd_octave_;
		this->lsd_descriptor_ = other.lsd_descriptor_;
		this->lsd_octave_ = other.lsd_octave_;
		this->image_network_pnts_ = other.image_network_pnts_;
		this->loaded_network_pnts_ = other.loaded_network_pnts_;
		this->hw_image_rgb_ = other.hw_image_rgb_;
		return *this;
    }

}
