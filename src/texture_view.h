/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef TEX_TEXTUREVIEW_HEADER
#define TEX_TEXTUREVIEW_HEADER

#include <string>
#include <vector>

#include <math/vector.h>
#include <mve/camera.h>
#include <mve/image.h>

#include "tri.h"
#include "settings.h"

TEX_NAMESPACE_BEGIN

/** Struct containing the quality and mean color of a face within a view. */
struct FaceProjectionInfo {
    std::uint16_t view_id;
    float quality;
	std::uint16_t view_angle;
    math::Vec3f mean_color;

    bool operator<(FaceProjectionInfo const & other) const {
        return view_id < other.view_id;
    }
};

/**
  * Class representing a view with specialized functions for texturing.
  */
class TextureView {
    private:
        std::size_t id;

        math::Vec3f pos;
        math::Vec3f viewdir;
        math::Matrix3f projection;
        math::Matrix4f world_to_cam;
		float c, d, e, cx, cy;
		std::vector<float> coeffs;
        int width;
        int height;
        std::string image_file;
        mve::ByteImage::Ptr image;
        mve::ByteImage::Ptr gradient_magnitude;
        std::vector<bool> validity_mask;


    public:
        /** Returns the id of the TexureView which is consistent for every run. */
        std::size_t get_id(void) const;

        /** Returns the 2D pixel coordinates of the given vertex projected into the view. */
        math::Vec2f get_pixel_coords(math::Vec3f const & vertex) const;
        /** Returns the RGB pixel values [0, 1] for the given vertex projected into the view, calculated by linear interpolation. */
        math::Vec3f get_pixel_values(math::Vec3f const & vertex) const;

		float get_mesh_camera_dist(math::Vec3f const & v1, math::Vec3f const & v2, math::Vec3f const & v3) const;
        /** Returns whether the pixel location is valid in this view.
          * The pixel location is valid if its inside the visible area and,
          * if a validity mask has been generated, all surrounding (integer coordinate) pixels are valid in the validity mask.
          */
        bool valid_pixel(math::Vec2f pixel) const;

        /** TODO */
        bool inside(math::Vec3f const & v1, math::Vec3f const & v2, math::Vec3f const & v3) const;

        /** Returns the RGB pixel values [0, 1] for the give pixel location. */
        math::Vec3f get_pixel_values(math::Vec2f const & pixel) const;

        /** Constructs a TextureView from the give mve::CameraInfo containing the given image. */
        TextureView(std::size_t id, mve::CameraInfo const & camera, std::string const & image_file);
		
		TextureView(bool use_pinhole, std::size_t id, mve::CameraInfo const & camera,
			std::string const & image_file);

		/** set view parameters from the given mve::CameraInfo containing the given image. */
		void SetViewParametersFromCamPinholeInfoImg(std::size_t id, mve::CameraInfo const & camera, std::string const & image_file);

        /** Returns the position. */
        math::Vec3f get_pos(void) const;
        /** Returns the viewing direction. */
        math::Vec3f get_viewing_direction(void) const;
        /** Returns the width of the corresponding image. */
        int get_width(void) const;
        /** Returns the height of the corresponding image. */
        int get_height(void) const;
        /** Returns a reference pointer to the corresponding image. */
        mve::ByteImage::Ptr get_image(void) const;

        /** Exchange encapsulated image. */
        void bind_image(mve::ByteImage::Ptr new_image);

        /** Loads the corresponding image. */
        void load_image(void);
        /** Generates the validity mask. */
        void generate_validity_mask(void);
        /** Generates the gradient magnitude image for the encapsulated image. */
        void generate_gradient_magnitude(void);

        /** Releases the validity mask. */
        void release_validity_mask(void);
        /** Releases the gradient magnitude image. */
        void release_gradient_magnitude(void);
        /** Releases the corresponding image. */
        void release_image(void);

        /** Erodes the validity mask by one pixel. */
        void erode_validity_mask(void);

        void
        get_face_info(math::Vec3f const & v1, math::Vec3f const & v2, math::Vec3f const & v3,
            FaceProjectionInfo * face_info, Settings const & settings) const;

        void
        export_triangle(math::Vec3f v1, math::Vec3f v2, math::Vec3f v3, std::string const & filename) const;

        void
        export_validity_mask(std::string const & filename) const;

        // @zk
        std::string get_image_name() const;
        // @zk
        void reset_image_name(std::string image_name);
};


inline std::size_t
TextureView::get_id(void) const {
    return id;
}

inline math::Vec3f
TextureView::get_pos(void) const {
    return pos;
}

inline math::Vec3f
TextureView::get_viewing_direction(void) const {
    return viewdir;
}

inline int
TextureView::get_width(void) const {
    return width;
}

inline int
TextureView::get_height(void) const {
    return height;
}

inline mve::ByteImage::Ptr
TextureView::get_image(void) const {
    assert(image != NULL);
    return image;
}

inline bool
TextureView::inside(math::Vec3f const & v1, math::Vec3f const & v2, math::Vec3f const & v3) const {
    math::Vec2f p1 = get_pixel_coords(v1);
    math::Vec2f p2 = get_pixel_coords(v2);
    math::Vec2f p3 = get_pixel_coords(v3);
    return valid_pixel(p1) && valid_pixel(p2) && valid_pixel(p3);
}

inline math::Vec2f
TextureView::get_pixel_coords(math::Vec3f const & vertex_w) const {
	/*math::Vec3f vertex = world_to_cam.mult(vertex_w, 1.0f);
	math::Vec2f pixel;
	int coeff_num = coeffs.size();
	float norm = std::sqrt(vertex[0] * vertex[0] + vertex[1] * vertex[1]);
	float theta = std::atan(-vertex[2] / norm);
	float t, t_i;
	float rho, x, y;
	if (norm != 0) {
		t = theta;
		rho = coeffs[0];
		t_i = 1;
		for (int i = 1; i < coeff_num; i++) {
			t_i *= t;
			rho += t_i*coeffs[i];
		}
		x = vertex[1] * rho / norm;
		y = vertex[0] * rho / norm;
		pixel[1] = x*c + y*d + cx;
		pixel[0] = x*e + y + cy;
	}
	else {
		pixel[1] = cx;
		pixel[0] = cy;
	}
    return math::Vec2f(pixel[0]-0.5f,pixel[1]-0.5f);*/
	math::Vec3f pixel = projection * world_to_cam.mult(vertex_w, 1.0f);
	pixel /= pixel[2];
	return math::Vec2f(pixel[0] - 0.5f, pixel[1] - 0.5f);
}

inline float 
TextureView::get_mesh_camera_dist(math::Vec3f const & v1, math::Vec3f const & v2, math::Vec3f const & v3) const {
	math::Vec3f vertex = (v1 + v2 + v3) / 3;
	return (vertex - pos).norm();
}

inline math::Vec3f
TextureView::get_pixel_values(math::Vec3f const & vertex) const {
    math::Vec2f pixel = get_pixel_coords(vertex);
    return get_pixel_values(pixel);
}

inline math::Vec3f
TextureView::get_pixel_values(math::Vec2f const & pixel) const {
    assert(image != NULL);
    math::Vec3uc values;
    image->linear_at(pixel[0], pixel[1], *values);
    return math::Vec3f(values) / 255.0f;
}

inline void
TextureView::bind_image(mve::ByteImage::Ptr new_image) {
    image = new_image;
}

inline void
TextureView::release_validity_mask(void) {
    assert(validity_mask.size() == static_cast<std::size_t>(width * height));
    validity_mask = std::vector<bool>();
}

inline void
TextureView::release_gradient_magnitude(void) {
    assert(gradient_magnitude != NULL);
    gradient_magnitude.reset();
}

inline void
TextureView::release_image(void) {
    assert(image != NULL);
    image.reset();
}

// @zk
inline std::string
TextureView::get_image_name() const {
    return image_file;
}

// @zk
inline void
TextureView::reset_image_name(std::string image_name) {
    image_file = image_name;
}

TEX_NAMESPACE_END

#endif /* TEX_TEXTUREVIEW_HEADER */
