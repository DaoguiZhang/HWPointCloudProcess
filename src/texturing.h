/*
 * Copyright (C) 2015, Nils Moehrle
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef TEX_TEXTURING_HEADER
#define TEX_TEXTURING_HEADER

#include <vector>

#include "mve/mesh.h"
#include "mve/mesh_info.h"

#include "mrf/graph.h"

#include "defines.h"
#include "settings.h"
#include "obj_model.h"
#include "uni_graph.h"
#include "texture_view.h"
#include "texture_patch.h"
#include "texture_atlas.h"
#include "sparse_table.h"

#include "seam_leveling.h"

TEX_NAMESPACE_BEGIN

typedef std::vector<TextureView> TextureViews;
typedef std::vector<TexturePatch::Ptr> TexturePatches;
typedef std::vector<TextureAtlas::Ptr> TextureAtlases;
typedef ObjModel Model;
typedef UniGraph Graph;
typedef SparseTable<std::uint32_t, std::uint16_t, float> DataCosts;
typedef SparseTable<std::uint32_t, std::uint16_t, math::Vec3f> FaceLuminance;
typedef std::vector<std::vector<VertexProjectionInfo> > VertexProjectionInfos;
typedef std::vector<std::vector<FaceProjectionInfo> > FaceProjectionInfos;
//extern int global_num_views;
//extern std::vector<math::Vec3f> global_face_mean_color;

/**
  * prepares the mesh for texturing
  *  -removes duplicated faces
  *  -ensures normals (face and vertex)
  */
void
prepare_mesh(mve::MeshInfo * mesh_info, mve::TriangleMesh::Ptr mesh);

/**
  * Generates TextureViews from the in_scene fisheye camera model.
  */
void
generate_texture_views(std::string const & in_scene, TextureViews * texture_views);

/*
*Generates TextureViews from the in_scene, pinhole camera model
*/
void generate_texture_views_pinhole(const std::string& in_scene, TextureViews* texture_views);

/*
*Generates TextureViews from the in_scene, pinhole camera model
*/
void from_scene_cams_images_pinhole(const std::string& in_scene, TextureViews* texture_views);

/**
  * Builds up the meshes face adjacency graph using the vertex_infos
  */
void
build_adjacency_graph(mve::TriangleMesh::ConstPtr mesh,
    mve::MeshInfo const & mesh_info, UniGraph * graph);

/**
 * Calculates the data costs for each face and texture view combination,
 * if the face is visible within the texture view.
 */
void
calculate_data_costs(mve::TriangleMesh::ConstPtr mesh, std::string in_mesh_origin,
    TextureViews * texture_views, Settings const & settings,
    DataCosts * data_costs,
	//std::vector<math::Vec3f> &faces_mean_color
	DataCosts * face_luminance);

void
postprocess_face_infos(Settings const & settings,
    FaceProjectionInfos * projected_face_infos,
    DataCosts * data_costs);

/**
 * Runs the view selection procedure and saves the labeling in the graph
 */
void
view_selection(DataCosts const & data_costs, UniGraph * graph, Settings const & settings, int num_views, DataCosts const & face_luminance);

/**
  * Generates texture patches using the graph to determine adjacent faces with the same label.
  */
void generate_texture_patches(UniGraph const & graph,
    mve::TriangleMesh::ConstPtr mesh,
    mve::MeshInfo const & mesh_info,
    TextureViews * texture_views,
    Settings const & settings,
    VertexProjectionInfos * vertex_projection_infos,
    TexturePatches * texture_patches);

/**
  * Runs the seam leveling procedure proposed by Ivanov and Lempitsky
  * [<A HREF="https://www.google.de/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&cad=rja&sqi=2&ved=0CC8QFjAA&url=http%3A%2F%2Fwww.robots.ox.ac.uk%2F~vilem%2FSeamlessMosaicing.pdf&ei=_ZbvUvSZIaPa4ASi7IGAAg&usg=AFQjCNGd4x5HnMMR68Sn2V5dPgmqJWErCA&sig2=4j47bXgovw-uks9LBGl_sA">Seamless mosaicing of image-based texture maps</A>]
  */
void
global_seam_leveling(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh,
    mve::MeshInfo const & mesh_info,
    VertexProjectionInfos const & vertex_projection_infos,
    TexturePatches * texture_patches);

void
local_seam_leveling(UniGraph const & graph, mve::TriangleMesh::ConstPtr mesh,
    VertexProjectionInfos const & vertex_projection_infos,
    TexturePatches * texture_patches);

void
generate_texture_atlases(TexturePatches * texture_patches,
    Settings const & settings, TextureAtlases * texture_atlases);

/**
  * Builds up an model for the mesh by constructing materials and
  * texture atlases form the texture_patches
  */
void
build_model(mve::TriangleMesh::ConstPtr mesh,
    TextureAtlases const & texture_atlas, Model * model);

TEX_NAMESPACE_END

#endif /* TEX_TEXTURING_HEADER */
