/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <util/timer.h>
#include <util/tokenizer.h>
#include <mve/image_io.h>
#include <mve/image_tools.h>
#include <mve/bundle_io.h>
#include <mve/scene.h>

#include "progress_counter.h"
#include "texturing.h"

TEX_NAMESPACE_BEGIN

void
from_mve_scene(std::string const & scene_dir, std::string const & image_name,
    std::vector<TextureView> * texture_views) {

    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(scene_dir);
    } catch (std::exception& e) {
        std::cerr << "Could not open scene: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    std::size_t num_views = scene->get_views().size();
    texture_views->reserve(num_views);

    ProgressCounter view_counter("\tLoading", num_views);
    for (std::size_t i = 0; i < num_views; ++i) {
        view_counter.progress<SIMPLE>();

        mve::View::Ptr view = scene->get_view_by_id(i);
        if (view == NULL) {
            view_counter.inc();
            continue;
        }

        if (!view->has_image(image_name, mve::IMAGE_TYPE_UINT8)) {
            std::cout << "Warning: View " << view->get_name() << " has no byte image "
                << image_name << std::endl;
            continue;
        }

        mve::View::ImageProxy const * image_proxy = view->get_image_proxy(image_name);

        if (image_proxy->channels < 3) {
            std::cerr << "Image " << image_name << " of view " <<
                view->get_name() << " is not a color image!" << std::endl;
            exit(EXIT_FAILURE);
        }
		//std::cout << "flen:     " << view->get_camera().flen << std::endl;
        texture_views->push_back(
            TextureView(view->get_id(), view->get_camera(), util::fs::abspath(
            util::fs::join_path(view->get_directory(), image_proxy->filename))));
        view_counter.inc();
    }
}

void
from_images_and_camera_files(std::string const & path, std::vector<TextureView> * texture_views) {
    util::fs::Directory dir(path);
    std::sort(dir.begin(), dir.end());
    std::vector<std::string> files;
    for (std::size_t i = 0; i < dir.size(); ++i) {
        util::fs::File const & cam_file = dir[i];
        if (cam_file.is_dir) continue;

        std::string cam_file_ext = util::string::uppercase(util::string::right(cam_file.name, 4));
        if (cam_file_ext != ".CAM") continue;

        std::string prefix = util::string::left(cam_file.name, cam_file.name.size() - 4);
        if (prefix.empty()) continue;

        /* Find corresponding image file. */
        int step = 1;
        for (std::size_t j = i + 1; j < dir.size(); j += step) {
            util::fs::File const & img_file = dir[j];

            /* Since the files are sorted we can break - no more files with the same prefix exist. */
            if (util::string::left(img_file.name, prefix.size()) != prefix) {
                if (step == 1) {
                    j = i;
                    step = -1;
                    continue;
                } else {
                    break;
                }
            }

            /* Image file (based on extension)? */
            std::string img_file_ext = util::string::uppercase(util::string::right(img_file.name, 4));
            if (img_file_ext != ".PNG" && img_file_ext != ".JPG" &&
                img_file_ext != "TIFF" && img_file_ext != "JPEG") continue;

            files.push_back(cam_file.get_absolute_name());
            files.push_back(img_file.get_absolute_name());
            break;
        }
    }
    ProgressCounter view_counter("\tLoading", files.size() / 2);
    #pragma omp parallel for
#if !defined(_MSC_VER)
    for (std::size_t i = 0; i < files.size(); i += 2) {
#else
    for (std::int64_t i = 0; i < files.size(); i += 2) {
#endif
        view_counter.progress<SIMPLE>();
        std::string cam_file = files[i];
        std::string img_file = files[i + 1];
		
        /* Read CAM file. */
		std::ifstream infile(cam_file.c_str());// , std::ios::binary);
        if (!infile.good())
            throw util::FileException(util::fs::basename(cam_file), std::strerror(errno));
        std::string cam_int_str, cam_ext_str;
        std::getline(infile, cam_ext_str);
        std::getline(infile, cam_int_str);
        util::Tokenizer tok_ext, tok_int;
        tok_ext.split(cam_ext_str);
        tok_int.split(cam_int_str);
        #pragma omp critical
        if (tok_ext.size() != 12 || tok_int.size() < 1) {
            std::cerr << "Invalid CAM file: " << util::fs::basename(cam_file) << std::endl;
            std::exit(EXIT_FAILURE);
        }

        /* Create cam_info and eventually undistort image. */
        mve::CameraInfo cam_info;
        cam_info.set_translation_from_string(tok_ext.concat(0, 3));
        cam_info.set_rotation_from_string(tok_ext.concat(3, 0));
		
        std::stringstream ss(cam_int_str);
		ss >> cam_info.flen;
		if (ss.peek() && !ss.eof()) {
			ss >> cam_info.c_;
		}
        if (ss.peek() && !ss.eof())
            ss >> cam_info.d_;
        if (ss.peek() && !ss.eof())
            ss >> cam_info.e_;
        if (ss.peek() && !ss.eof())
            ss >> cam_info.cx_;
        if (ss.peek() && !ss.eof())
            ss >> cam_info.cy_;
		float coeff;
		while (ss.peek() && !ss.eof()) {
			ss >> coeff;
			cam_info.coeffs_.emplace_back(coeff);
		}

        std::string image_file = util::fs::abspath(util::fs::join_path(path, img_file));
        /*if (cam_info.dist[0] != 0.0f) {
            mve::ByteImage::Ptr image = mve::image::load_file(img_file);
            if (cam_info.dist[1] != 0.0f) {
                image = mve::image::image_undistort_k2k4<uint8_t>(image,
                    cam_info.flen, cam_info.dist[0], cam_info.dist[1]);
            } else {
                image = mve::image::image_undistort_vsfm<uint8_t>(image,
                    cam_info.flen, cam_info.dist[0]);
            }

            image_file = std::string("/tmp/") + util::fs::basename(img_file);
            mve::image::save_png_file(image, image_file);
        }*/

        #pragma omp critical
        texture_views->push_back(TextureView(i / 2, cam_info, image_file));

        view_counter.inc();
    }
}

void
from_nvm_scene(std::string const & nvm_file, std::vector<TextureView> * texture_views) {
    /*std::vector<mve::NVMCameraInfo> nvm_cams;
    mve::Bundle::Ptr bundle = mve::load_nvm_bundle(nvm_file, &nvm_cams);
    mve::Bundle::Cameras& cameras = bundle->get_cameras();

    ProgressCounter view_counter("\tLoading", cameras.size());
    #pragma omp parallel for
#if !defined(_MSC_VER)
    for (std::size_t i = 0; i < cameras.size(); ++i) {
#else
    for (std::int64_t i = 0; i < cameras.size(); ++i) {
#endif
        view_counter.progress<SIMPLE>();
        mve::CameraInfo& mve_cam = cameras[i];
        mve::NVMCameraInfo const& nvm_cam = nvm_cams[i];

        mve::ByteImage::Ptr image = mve::image::load_file(nvm_cam.filename);

        int const maxdim = std::max(image->width(), image->height());
        mve_cam.flen = mve_cam.flen / static_cast<float>(maxdim);

        image = mve::image::image_undistort_vsfm<uint8_t>
            (image, mve_cam.flen, nvm_cam.radial_distortion);

        std::string image_file = std::string("/tmp/") + util::fs::basename(nvm_cam.filename);
        mve::image::save_png_file(image, image_file);

        #pragma omp critical
        texture_views->push_back(TextureView(i, mve_cam, image_file));

        view_counter.inc();
    }*/
}

void generate_texture_views_pinhole(const std::string& in_scene, TextureViews* texture_views)
{
	/* SCENE_FOLDER */
	if (util::fs::dir_exists(in_scene.c_str())) {
		from_scene_cams_images_pinhole(in_scene, texture_views);
	}
}

void from_scene_cams_images_pinhole(const std::string& in_scene, TextureViews* texture_views)
{

#if 0
	TextureView tmp_view; 
	mve::CameraInfo cam_info; 
	std::string path;
	tmp_view.SetViewParametersFromCamPinholeInfoImg(2, cam_info, path);
#endif

#if 1
	util::fs::Directory dir(in_scene);
	std::sort(dir.begin(), dir.end());
	std::vector<std::string> files;
	
	for (std::int64_t i = 0; i < dir.size(); ++i)
	{
		util::fs::File const & cam_file = dir[i];
		if (cam_file.is_dir) continue;

		std::string cam_file_ext = util::string::uppercase(util::string::right(cam_file.name, 4));
		if (cam_file_ext != ".CAM") continue;

		std::string prefix = util::string::left(cam_file.name, cam_file.name.size() - 4);
		if (prefix.empty()) continue;

		/* Find corresponding image file. */
		int step = 1;
		for (std::size_t j = i + 1; j < dir.size(); j += step) {
			util::fs::File const & img_file = dir[j];

			/* Since the files are sorted we can break - no more files with the same prefix exist. */
			if (util::string::left(img_file.name, prefix.size()) != prefix) {
				if (step == 1) {
					j = i;
					step = -1;
					continue;
				}
				else {
					break;
				}
			}

			/* Image file (based on extension)? */
			std::string img_file_ext = util::string::uppercase(util::string::right(img_file.name, 4));
			if (img_file_ext != ".PNG" && img_file_ext != ".JPG" &&
				img_file_ext != "TIFF" && img_file_ext != "JPEG") continue;

			files.push_back(cam_file.get_absolute_name());
			files.push_back(img_file.get_absolute_name());
			break;
		}
	}

	ProgressCounter view_counter("\tLoading", files.size() / 2);

#pragma omp parallel for
#if !defined(_MSC_VER)
	for (std::size_t i = 0; i < files.size(); i += 2) {
#else
	for (std::int64_t i = 0; i < files.size(); i += 2) {
#endif
		view_counter.progress<SIMPLE>();
		std::string cam_file = files[i];
		std::string img_file = files[i + 1];

		/* Read CAM file. */
		std::ifstream infile(cam_file.c_str());// , std::ios::binary);
		if (!infile.good())
			throw util::FileException(util::fs::basename(cam_file), std::strerror(errno));
		std::string cam_extr, cam_intr_part0, cam_intr_part1;
		std::getline(infile, cam_extr);
		std::getline(infile, cam_intr_part0);
		std::getline(infile, cam_intr_part1);
		util::Tokenizer extr_tok, intr_part0_tok, intr_part1_tok;
		/*按照空格分成几个string*/
		extr_tok.split(cam_extr);
		intr_part0_tok.split(cam_intr_part0);
		intr_part1_tok.split(cam_intr_part1);	//暂且不读入

#pragma omp critical
		if (extr_tok.size() != 12 || cam_intr_part0.size() < 1) {
			std::cerr << "Invalid CAM file: " << util::fs::basename(cam_file) << std::endl;
			std::exit(EXIT_FAILURE);
		}
#if 1
		/* Create cam_info and eventually undistort image. */
		mve::CameraInfo cam_info;
		//cam extrisinc info
		cam_info.set_translation_from_string(extr_tok.concat(0, 3));
		cam_info.set_rotation_from_string(extr_tok.concat(3, 0));

		//cam intrisinc info
		//fx,fy,cx,cy
		std::stringstream ss(cam_intr_part0);
		ss >> cam_info.flen;
		if (ss.peek() && !ss.eof())
			ss >> cam_info.flen;
		if (ss.peek() && !ss.eof())
			ss >> cam_info.ppoint[0];
		if (ss.peek() && !ss.eof())
			ss >> cam_info.ppoint[1];
		
		ss.clear();
		ss << cam_intr_part1;
		//K1,K2,K3,P1,P2
		if (ss.peek() && !ss.eof())
			ss >> cam_info.dist[0];
		if (ss.peek() && !ss.eof())
			ss >> cam_info.dist[1];
		if (ss.peek() && !ss.eof())
			ss >> cam_info.dist[2];
		if (ss.peek() && !ss.eof())
			ss >> cam_info.dist[3];
		if (ss.peek() && !ss.eof())
			ss >> cam_info.dist[4];

		std::string image_file = util::fs::abspath(util::fs::join_path(in_scene, img_file));
		/*if (cam_info.dist[0] != 0.0f) {
		mve::ByteImage::Ptr image = mve::image::load_file(img_file);
		if (cam_info.dist[1] != 0.0f) {
		image = mve::image::image_undistort_k2k4<uint8_t>(image,
		cam_info.flen, cam_info.dist[0], cam_info.dist[1]);
		} else {
		image = mve::image::image_undistort_vsfm<uint8_t>(image,
		cam_info.flen, cam_info.dist[0]);
		}

		image_file = std::string("/tmp/") + util::fs::basename(img_file);
		mve::image::save_png_file(image, image_file);
		}*/
#endif

#pragma omp critical
		texture_views->push_back(TextureView(true, i / 2, cam_info, image_file));

		view_counter.inc();
	}
#endif
}

void
generate_texture_views(std::string const & in_scene, std::vector<TextureView> * texture_views) {
    /* Determine input format. */

    /* BUNDLEFILE */
    if (util::fs::file_exists(in_scene.c_str())) {
        std::string const & file = in_scene;
        std::string extension = util::string::uppercase(util::string::right(file, 3));
        if (extension == "NVM") from_nvm_scene(file, texture_views);
    }

    /* SCENE_FOLDER */
    if (util::fs::dir_exists(in_scene.c_str())) {
        from_images_and_camera_files(in_scene, texture_views);
    }

    /* MVE_SCENE::EMBEDDING */
    size_t pos = in_scene.rfind("::");
    if (pos != std::string::npos) {
        std::string scene_dir = in_scene.substr(0, pos);
        std::string image_name = in_scene.substr(pos + 2, in_scene.size());
        from_mve_scene(scene_dir, image_name, texture_views);
    }

    std::size_t num_views = texture_views->size();
    if (num_views == 0) {
        std::cerr << "No proper input scene descriptor given." << std::endl
            << "A input descriptor can be:" << std::endl
            << "BUNDLE_FILE - a bundle file (currently onle .nvm files are supported)" << std::endl
            << "SCENE_FOLDER - a folder containing images and .cam files" << std::endl
            << "MVE_SCENE::EMBEDDING - a mve scene and embedding" << std::endl;
        exit(EXIT_FAILURE);
    }
}

TEX_NAMESPACE_END
