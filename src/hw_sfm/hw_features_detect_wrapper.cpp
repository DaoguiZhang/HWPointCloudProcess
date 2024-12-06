#include"hw_features_detect_wrapper.h"

#if 0
namespace HWSFM
{

	void SiftDetectorRun(const mve::ByteImage::Ptr& image, const sfm::Sift::Options& sift_options,
		sfm::Sift::Descriptors& sift_descr, sfm::Sift::Keypoints& sift_keypoints)
	{
		sfm::Sift sift(sift_options);
		sift.set_image(image);
		util::WallTimer timer;
		sift.process();
		std::cout << "Computed SIFT features in "
			<< timer.get_elapsed() << "ms." << std::endl;

		sift_descr = sift.get_descriptors();
		sift_keypoints = sift.get_keypoints();
	}

	void SurfDetectorRun(const mve::ByteImage::Ptr& image, const sfm::Surf::Options& Surf_options,
		sfm::Surf::Descriptors& Surf_descr, sfm::Surf::Keypoints& Surf_keypoints)
	{
		sfm::Surf surf(Surf_options);
		surf.set_image(image);
		util::WallTimer timer;
		surf.process();
		std::cout << "Computed SIFT features in "
			<< timer.get_elapsed() << "ms." << std::endl;

		Surf_descr = surf.get_descriptors();
		Surf_keypoints = surf.get_keypoints();
	}

	void RunDetectorZDGTest()
	{
		//to do next...
		std::string image_path = "";
		mve::ByteImage::Ptr img;
		try
		{
			std::cerr << "start to load image ... " << std::endl;
			img = mve::image::load_file(image_path);
		}
		catch (const std::exception& e)
		{
			std::cerr << "Error: " << e.what() << std::endl;
			return;
		}

		sfm::Surf::Descriptors surf_descr;
		sfm::Surf::Keypoints surf_keypoints;
		sfm::Surf::Options surf_options;
		surf_options.verbose_output = true;
		surf_options.debug_output = true;
		SurfDetectorRun(img, surf_options, surf_descr, surf_keypoints);

		sfm::Sift::Descriptors sift_descr;
		sfm::Sift::Keypoints sift_keypoints;
		sfm::Sift::Options sift_options;
		sift_options.verbose_output = true;
		sift_options.debug_output = true;
		SiftDetectorRun(img, sift_options, sift_descr, sift_keypoints);

		//show the detecting performance
		/* Draw features. */
		std::vector<sfm::Visualizer::Keypoint> surf_drawing;
		for (std::size_t i = 0; i < surf_descr.size(); ++i)
		{
			sfm::Visualizer::Keypoint kp;
			kp.orientation = surf_descr[i].orientation;
			kp.radius = surf_descr[i].scale;
			kp.x = surf_descr[i].x;
			kp.y = surf_descr[i].y;
			surf_drawing.push_back(kp);
		}

		std::vector<sfm::Visualizer::Keypoint> sift_drawing;
		for (std::size_t i = 0; i < sift_descr.size(); ++i)
		{
			sfm::Visualizer::Keypoint kp;
			kp.orientation = sift_descr[i].orientation;
			kp.radius = sift_descr[i].scale;
			kp.x = sift_descr[i].x;
			kp.y = sift_descr[i].y;
			sift_drawing.push_back(kp);
		}

		mve::ByteImage::Ptr surf_image = sfm::Visualizer::draw_keypoints(img,
			surf_drawing, sfm::Visualizer::RADIUS_BOX_ORIENTATION);
		mve::ByteImage::Ptr sift_image = sfm::Visualizer::draw_keypoints(img,
			sift_drawing, sfm::Visualizer::RADIUS_BOX_ORIENTATION);

		/* Save the two images for SIFT and SURF. */
		std::string surf_out_fname = "/tmp/" + util::fs::replace_extension
		(util::fs::basename(image_path), "surf.png");
		std::string sift_out_fname = "/tmp/" + util::fs::replace_extension
		(util::fs::basename(image_path), "sift.png");

		std::cout << "Writing output file: " << surf_out_fname << std::endl;
		mve::image::save_file(surf_image, surf_out_fname);
		std::cout << "Writing output file: " << sift_out_fname << std::endl;
		mve::image::save_file(sift_image, sift_out_fname);
	}

}
#endif