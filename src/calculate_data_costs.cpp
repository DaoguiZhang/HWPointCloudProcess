/*
 * Copyright (C) 2015, Nils Moehrle, Michael Waechter
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <numeric>

#include <mve/image_color.h>
#include <mve/mesh_io_ply.h>
#include <acc/bvh_tree.h>
#include <Eigen/Core>
#include <Eigen/LU>

#include "util.h"
#include "histogram.h"
#include "texturing.h"
#include "sparse_table.h"
#include "progress_counter.h"

typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;
//int global_num_views;
//std::vector<math::Vec3f> global_face_mean_color;

TEX_NAMESPACE_BEGIN

/**
 * Dampens the quality of all views in which the face's projection
 * has a much different color than in the majority of views.
 * Returns whether the outlier removal was successfull.
 *
 * @param infos contains information about one face seen from several views
 * @param settings runtime configuration.
 */
bool
photometric_outlier_detection(std::vector<FaceProjectionInfo> * infos, Settings const & settings) {
    if (infos->size() == 0) return true;

    /* Configuration variables. */
    double const gauss_rejection_threshold = 6e-3;

    /* If all covariances drop below this we stop outlier detection. */
    double const minimal_covariance = 5e-4;

    int const outlier_detection_iterations = 10;
    int const minimal_num_inliers = 4;

    float outlier_removal_factor = std::numeric_limits<float>::signaling_NaN();
    switch (settings.outlier_removal) {
        case OUTLIER_REMOVAL_NONE: return true;
        case OUTLIER_REMOVAL_GAUSS_CLAMPING:
            outlier_removal_factor = 1.0f;
        break;
        case OUTLIER_REMOVAL_GAUSS_DAMPING:
            outlier_removal_factor = 0.2f;
        break;
    }

    Eigen::MatrixX3d inliers(infos->size(), 3);
    std::vector<std::uint32_t> is_inlier(infos->size(), 1);
	bool flag = false;
	int cnt = 0;
    for (std::size_t row = 0; row < infos->size(); ++row) {
        inliers.row(row) = mve_to_eigen(infos->at(row).mean_color).cast<double>();
		//printf("Y: %f\n", infos->at(row).mean_color[0]);
		if (infos->at(row).view_angle <= 60) {
			flag = true;
			cnt++;
		}
    }
	//std::cout << "infos->size():  " << infos->size() << "\n";
	//std::cout << "cnt:  " << cnt << "\n";
	
	if (1.0f*cnt/ infos->size() >=0.3&&cnt>=4) {
		for (std::size_t row = 0; row < infos->size(); ++row) {
			if (infos->at(row).view_angle > 60) {
				//is_inlier[row] = 0;
				infos->at(row).quality = 0.0f;
			}
			//else
				//std::cout << "veiw_id:  " << infos->at(row).view_id <<"   "<< infos->at(row).quality <<"\n";
		}
	}
	//system("pause");
	//inliers.resize(std::accumulate(is_inlier.begin(), is_inlier.end(), 0), Eigen::NoChange);
	//for (std::size_t row = 0, inlier_row = 0; row < infos->size(); ++row) {
	//	if (is_inlier[row]) {
	//		inliers.row(inlier_row++) = mve_to_eigen(infos->at(row).mean_color).cast<double>();
	//		//printf("inliers view_id: %d\n", infos->at(row).view_id);
	//	}
	//}

    Eigen::RowVector3d var_mean;
    Eigen::Matrix3d covariance;
    Eigen::Matrix3d covariance_inv;

    for (int i = 0; i < outlier_detection_iterations; ++i) {
		//if (inliers.rows() - cnt < minimal_num_inliers) {
        if (cnt < minimal_num_inliers) {
            return false;
        }

        /* Calculate the inliers' mean color and color covariance. */
        //var_mean = inliers.colwise().mean();
		for (int i = 0; i < 3; i++) {
			float sum = 0;
			float weights = 0;
			for (int j = 0; j < inliers.rows(); j++) {
				float weight = std::sqrt(infos->at(j).quality);
				sum += weight*inliers(j, i);
				weights += weight;
			}
			var_mean[i] = sum / weights;
		}
        Eigen::MatrixX3d centered = inliers.rowwise() - var_mean;
        covariance = (centered.adjoint() * centered) / double(inliers.rows() - 1);
        /* If all covariances are very small we stop outlier detection
         * and only keep the inliers (set quality of outliers to zero). */
        if (covariance.array().abs().maxCoeff() < minimal_covariance) {
            for (std::size_t row = 0; row < infos->size(); ++row) {
                if (!is_inlier[row]) infos->at(row).quality = 0.0f;
            }
            return true;
        }

        /* Invert the covariance. FullPivLU is not the fastest way but
         * it gives feedback about numerical stability during inversion. */
        Eigen::FullPivLU<Eigen::Matrix3d> lu(covariance);
        if (!lu.isInvertible()) {
            return false;
        }
        covariance_inv = lu.inverse();

        /* Compute new number of inliers (all views with a gauss value above a threshold). */
        for (std::size_t row = 0; row < infos->size(); ++row) {
            Eigen::RowVector3d color = mve_to_eigen(infos->at(row).mean_color).cast<double>();
            double gauss_value = multi_gauss_unnormalized<double, 3>(color, var_mean, covariance_inv);
            is_inlier[row] = (gauss_value >= gauss_rejection_threshold ? 1 : 0);
			//printf("view_id: %d   gauss_value: %f\n", infos->at(row).view_id, gauss_value);
        }
        /* Resize Eigen matrix accordingly and fill with new inliers. */
        inliers.resize(std::accumulate(is_inlier.begin(), is_inlier.end(), 0), Eigen::NoChange);
        for (std::size_t row = 0, inlier_row = 0; row < infos->size(); ++row) {
            if (is_inlier[row]) {
                inliers.row(inlier_row++) = mve_to_eigen(infos->at(row).mean_color).cast<double>();
				//printf("inliers view_id: %d\n", infos->at(row).view_id);
            }
        }
    }

    covariance_inv *= outlier_removal_factor;
    for (FaceProjectionInfo & info : *infos) {
        Eigen::RowVector3d color = mve_to_eigen(info.mean_color).cast<double>();
        double gauss_value = multi_gauss_unnormalized<double, 3>(color, var_mean, covariance_inv);
        assert(0.0 <= gauss_value && gauss_value <= 1.0);
        switch(settings.outlier_removal) {
            case OUTLIER_REMOVAL_NONE: return true;
            case OUTLIER_REMOVAL_GAUSS_DAMPING:
                info.quality *= gauss_value;
            break;
            case OUTLIER_REMOVAL_GAUSS_CLAMPING:
                if (gauss_value < gauss_rejection_threshold) info.quality = 0.0f;
            break;
        }
		//info.quality *= info.mean_color[0];
    }
    return true;
}

void
calculate_face_projection_infos(mve::TriangleMesh::ConstPtr mesh, std::string in_mesh_origin,
    std::vector<TextureView> * texture_views, Settings const & settings,
    FaceProjectionInfos * face_projection_infos) {

    std::vector<unsigned int> const & faces = mesh->get_faces();
    std::vector<math::Vec3f> const & vertices = mesh->get_vertices();
	mve::TriangleMesh::ConstPtr mesh_origin;
    mve::TriangleMesh::NormalList const & face_normals = mesh->get_face_normals();

    std::size_t const num_views = texture_views->size();

    util::WallTimer timer;
	BVHTree *bvh_tree=new BVHTree(faces, vertices);
	if (!in_mesh_origin.empty()) {	
		mesh_origin = mve::geom::load_ply_mesh(in_mesh_origin);
		std::vector<unsigned int> const & faces_origin = mesh_origin->get_faces();
		std::vector<math::Vec3f> const & vertices_origin = mesh_origin->get_vertices();
		std::cout << "\tBuilding BVH from " << faces_origin.size() / 3 << " faces... " << std::flush;
		bvh_tree = new BVHTree(faces_origin, vertices_origin);
	}
	else
		std::cout << "\tBuilding BVH from " << faces.size() / 3 << " faces... " << std::flush;
    std::cout << "done. (Took: " << timer.get_elapsed() << " ms)" << std::endl;

	/*faces_mean_color.resize(faces.size() / 3 * num_views);
	for (int i = 0; i < faces_mean_color.size(); i++)
	{
		faces_mean_color[i] = math::Vec3f(-1, -1, -1);
	}*/

    ProgressCounter view_counter("\tCalculating face qualities", num_views);
    #pragma omp parallel
    {
        std::vector<std::pair<std::size_t, FaceProjectionInfo> > projected_face_view_infos;

        #pragma omp for schedule(dynamic)
#if !defined(_MSC_VER)
        for (std::uint16_t j = 0; j < static_cast<std::uint16_t>(num_views); ++j) {
#else

        for (std::int32_t j = 0; j < num_views; ++j) {
#endif

            view_counter.progress<SIMPLE>();

            TextureView * texture_view = &texture_views->at(j);
            texture_view->load_image();
            texture_view->generate_validity_mask();

            if (settings.data_term == DATA_TERM_GMI) {
                texture_view->generate_gradient_magnitude();
                texture_view->erode_validity_mask();
            }

            math::Vec3f const & view_pos = texture_view->get_pos();
            math::Vec3f const & viewing_direction = texture_view->get_viewing_direction();

            for (std::size_t i = 0; i < faces.size(); i += 3) {
                std::size_t face_id = i / 3;

                math::Vec3f const & v1 = vertices[faces[i]];
                math::Vec3f const & v2 = vertices[faces[i + 1]];
                math::Vec3f const & v3 = vertices[faces[i + 2]];
                math::Vec3f const & face_normal = face_normals[face_id];
                math::Vec3f const face_center = (v1 + v2 + v3) / 3.0f;

                /* Check visibility and compute quality */

                math::Vec3f view_to_face_vec = (face_center - view_pos).normalized();
                math::Vec3f face_to_view_vec = (view_pos - face_center).normalized();
				float view_to_face_dist = (face_center - view_pos).norm();
                /* Backface and basic frustum culling */
#if 0
                float viewing_angle = std::min(face_to_view_vec.dot(face_normal), viewing_direction.dot(view_to_face_vec));
				//float viewing_angle = face_to_view_vec.dot(face_normal);
                if (viewing_angle < 0.0f)
                    continue;

                if (std::acos(viewing_angle) > MATH_DEG2RAD(75.0f))
               continue;
#else
				float viewing_angle = MATH_RAD2DEG(std::acos(face_to_view_vec.dot(face_normal)));
				if (viewing_angle > 90.0f|| viewing_direction.dot(view_to_face_vec)<0.0f)
					continue;
				if (viewing_angle >80.0f)
					continue;
				
				if (abs(face_center[2]) <= 0.1f)
					viewing_angle = 60.0f;
				
#endif
                /* Projects into the valid part of the TextureView? */
                if (!texture_view->inside(v1, v2, v3))
                    continue;

                if (settings.geometric_visibility_test) {
                    /* Viewing rays do not collide? */
                    bool visible = true;
                    math::Vec3f const * samples[] = {&v1, &v2, &v3};
                    // TODO: random monte carlo samples...
                    
                    int vertexHitNum = 0;    // @zk
                    for (std::size_t k = 0; k < sizeof(samples) / sizeof(samples[0]); ++k) {
                        BVHTree::Ray ray;
                        ray.origin = *samples[k];
                        ray.dir = view_pos - ray.origin;
                        ray.tmax = ray.dir.norm();
                        ray.tmin = ray.tmax * 0.0001f;
                        ray.dir.normalize();

                        BVHTree::Hit hit;
                        //if (bvh_tree->intersect(ray, &hit)) {
                        //    visible = false;
                        //    break;
                        //}
                        if (bvh_tree->intersect(ray, &hit)) {
                            ++vertexHitNum;
                        }
                    }
                    if (vertexHitNum >= 3) visible = false;    // @zk
                    if (!visible) continue;
                }

                FaceProjectionInfo info = {j, 0.0f,(std::uint16_t)viewing_angle, math::Vec3f(0.0f, 0.0f, 0.0f)};

                /* Calculate quality. */
                texture_view->get_face_info(v1, v2, v3, &info, settings);

				/*if (view_to_face_dist <= 3.0f&&abs(face_center[2]) >= 0.1f)
					info.quality = info.quality * view_to_face_dist*view_to_face_dist / 16.0;*/

                if (info.quality == 0.0) continue;

                /* Change color space. */
                mve::image::color_rgb_to_ycbcr(*(info.mean_color));
				//std::cout << "ok1" << "\n";
				//faces_mean_color[face_id*num_views + j] = info.mean_color;
				//std::cout << "ok2" << "\n";
				//info.quality *= info.mean_color(0)*info.mean_color(0);
				/*if (info.mean_color(2) == info.mean_color(1)) {
					info.mean_color(1) = info.mean_color(0);
					info.mean_color(2) = info.mean_color(0);
				}*/
                std::pair<std::size_t, FaceProjectionInfo> pair(face_id, info);
                projected_face_view_infos.push_back(pair);
            }

            texture_view->release_image();
            texture_view->release_validity_mask();
            if (settings.data_term == DATA_TERM_GMI) {
                texture_view->release_gradient_magnitude();
            }
            view_counter.inc();
        }

        //std::sort(projected_face_view_infos.begin(), projected_face_view_infos.end());

        #pragma omp critical
        {
            for (std::size_t i = projected_face_view_infos.size(); 0 < i; --i) {
                std::size_t face_id = projected_face_view_infos[i - 1].first;
                FaceProjectionInfo const & info = projected_face_view_infos[i - 1].second;
                face_projection_infos->at(face_id).push_back(info);
            }
            projected_face_view_infos.clear();
        }
    }
}

void
postprocess_face_infos(Settings const & settings,
        FaceProjectionInfos * face_projection_infos,
        DataCosts * data_costs, DataCosts * face_luminance) {

    ProgressCounter face_counter("\tPostprocessing face infos",
        face_projection_infos->size());
    #pragma omp parallel for schedule(dynamic)
#if !defined(_MSC_VER)
    for (std::size_t i = 0; i < face_projection_infos->size(); ++i) {
#else
    for (std::int64_t i = 0; i < face_projection_infos->size(); ++i) {
#endif
        face_counter.progress<SIMPLE>();

        std::vector<FaceProjectionInfo> & infos = face_projection_infos->at(i);
        if (settings.outlier_removal != OUTLIER_REMOVAL_NONE) {
            photometric_outlier_detection(&infos, settings);

            infos.erase(std::remove_if(infos.begin(), infos.end(),
                [](FaceProjectionInfo const & info) -> bool {return info.quality == 0.0f;}),
                infos.end());
        }
        std::sort(infos.begin(), infos.end());

        face_counter.inc();
    }

    /* Determine the function for the normlization. */
    float max_quality = 0.0f;
    for (std::size_t i = 0; i < face_projection_infos->size(); ++i)
        for (FaceProjectionInfo const & info : face_projection_infos->at(i))
            max_quality = std::max(max_quality, info.quality);

    Histogram hist_qualities(0.0f, max_quality, 10000);
    for (std::size_t i = 0; i < face_projection_infos->size(); ++i)
        for (FaceProjectionInfo const & info : face_projection_infos->at(i))
            hist_qualities.add_value(info.quality);

    float percentile = hist_qualities.get_approx_percentile(0.99f);    //0.99

    /* Calculate the costs. */
    for (std::uint32_t i = 0; i < face_projection_infos->size(); ++i) {
        for (FaceProjectionInfo const & info : face_projection_infos->at(i)) {

            /* Clamp to percentile and normalize. */
            float normalized_quality = std::min(1.0f, info.quality / percentile);
            float data_cost = (1.0f - normalized_quality);
            data_costs->set_value(i, info.view_id, data_cost);
			face_luminance->set_value(i, info.view_id, info.mean_color[0]);
        }

        /* Ensure that all memory is freeed. */
        face_projection_infos->at(i) = std::vector<FaceProjectionInfo>();
    }

    std::cout << "\tMaximum quality of a face within an image: " << max_quality << std::endl;
    std::cout << "\tClamping qualities to " << percentile << " within normalization." << std::endl;
}

void
calculate_data_costs(mve::TriangleMesh::ConstPtr mesh, std::string in_mesh_origin, std::vector<TextureView> * texture_views,
    Settings const & settings, DataCosts * data_costs, DataCosts * face_luminance) {

    std::size_t const num_faces = mesh->get_faces().size() / 3;
    std::size_t const num_views = texture_views->size();

    if (num_faces > std::numeric_limits<std::uint32_t>::max())
        throw std::runtime_error("Exeeded maximal number of faces");
    if (num_views > std::numeric_limits<std::uint16_t>::max())
        throw std::runtime_error("Exeeded maximal number of views");

    FaceProjectionInfos face_projection_infos(num_faces);
    calculate_face_projection_infos(mesh, in_mesh_origin, texture_views, settings, &face_projection_infos);
    postprocess_face_infos(settings, &face_projection_infos, data_costs, face_luminance);
}

TEX_NAMESPACE_END
