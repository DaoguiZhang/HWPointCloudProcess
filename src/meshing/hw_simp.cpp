#include "hw_simp.h"
#include "quadric_simp.h"
#include "clustering_decimation.h"
#include <wrap/io_trimesh/import_ply.h>
#include <vcg/container/simple_temporary_data.h>
#include <hw_point_cloud.h>


namespace HW {
	HWSIMP::HWSIMP() {}
	HWSIMP::~HWSIMP() {}
	void HWSIMP::Process(HWObject* in_element, HWObject* out_element)
    {
        // create vcg mesh
		HWMesh* in_mesh = dynamic_cast<HWMesh*>(in_element);
		std::vector<float3> vert = in_mesh->GetVertices();
		std::vector<int3> face = in_mesh->GetFaces();
		int vn = vert.size(), fn = face.size(), idx;
		CMeshO cm;
		std::vector<CMeshO::VertexPointer> index;
		CMeshO::VertexIterator vi = Allocator<CMeshO>::AddVertices(cm, vn);
		for (idx = 0; idx < vn; ++idx) {
			(*vi).P()[0] = vert[idx].x;
			(*vi).P()[1] = vert[idx].y;
			(*vi).P()[2] = vert[idx].z;
			++vi;
		}
		index.resize(vn);
		for (idx = 0, vi = cm.vert.begin(); idx<vn; ++idx, ++vi)
			index[idx] = &*vi;
		CMeshO::FaceIterator fi = Allocator<CMeshO>::AddFaces(cm, fn);
		for (idx = 0; idx < fn; ++idx)
		{
			(*fi).V(0) = index[face[idx].x];
			(*fi).V(1) = index[face[idx].y];
			(*fi).V(2) = index[face[idx].z];
			++fi;
		}

#if 0    //  0:use clustering simplification; 1:use quadric edge collapse simplification.
		vcg::tri::UpdateBounding<CMeshO>::Box(cm);
		cm.vert.EnableVFAdjacency();
		cm.face.EnableVFAdjacency();
		vcg::tri::UpdateTopology<CMeshO>::VertexFace(cm);
		vcg::tri::UpdateFlags<CMeshO>::FaceBorderFromVF(cm);
		cm.vert.EnableMark();
		vcg::tri::TriEdgeCollapseQuadricParameter pp;

		int target_faces_num = HW::HWParams::getInstance().simplification_params_.target_faces_num_;
		float target_percentage = HW::HWParams::getInstance().simplification_params_.target_percentage_;
		if (target_percentage != 0) target_faces_num = cm.fn*target_percentage;
		pp.PreserveBoundary = HW::HWParams::getInstance().simplification_params_.boundary_preserve_flag_;
		pp.BoundaryWeight = HW::HWParams::getInstance().simplification_params_.boundary_preserve_weight_;
		pp.QualityThr = HW::HWParams::getInstance().simplification_params_.quality_threshold_;
		pp.PreserveTopology = HW::HWParams::getInstance().simplification_params_.topology_preserve_flag_;
		pp.QualityQuadric = HW::HWParams::getInstance().simplification_params_.planar_quadric_flag_;
		pp.OptimalPlacement = HW::HWParams::getInstance().simplification_params_.optimal_placement_flag_;
		pp.NormalCheck = HW::HWParams::getInstance().simplification_params_.preserve_normal_flag_;
		
		vcg::CallBackPos * cb = 0;
		
		QuadricSimplification(cm, target_faces_num, false, pp, cb);
#else
        // voxel size 0.04m , voxel num 1000000000 is useless.
        clusteringDecimation(cm, 0.04f, 1000000000, false);
#endif

        // clean mesh
		int nullFaces = vcg::tri::Clean<CMeshO>::RemoveFaceOutOfRangeArea(cm, 0);
		int deldupvert = vcg::tri::Clean<CMeshO>::RemoveDuplicateVertex(cm);
		int delvert = vcg::tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm);
		cm.face.DisableFFAdjacency();
		vcg::tri::Allocator<CMeshO>::CompactVertexVector(cm);
		vcg::tri::Allocator<CMeshO>::CompactFaceVector(cm);
		vcg::tri::UpdateBounding<CMeshO>::Box(cm);
		if (cm.fn>0) {
			vcg::tri::UpdateNormal<CMeshO>::PerFaceNormalized(cm);
			vcg::tri::UpdateNormal<CMeshO>::PerVertexAngleWeighted(cm);
		}
		vcg::tri::UpdateNormal<CMeshO>::NormalizePerFace(cm);
		vcg::tri::UpdateNormal<CMeshO>::PerVertexFromCurrentFaceNormal(cm);
		vcg::tri::UpdateNormal<CMeshO>::NormalizePerVertex(cm);

        // trans to HWMesh
		HW::HWMesh* simp_output = new HW::HWMesh();

		vcg::SimpleTempData<CMeshO::VertContainer, int> indices(cm.vert);
		for (idx = 0, vi = cm.vert.begin(); vi != cm.vert.end(); ++vi) {
			indices[vi] = idx;
			if (!(*vi).IsD()) {
				float3 vertice;
				vertice.x = (*vi).P()[0];
				vertice.y = (*vi).P()[1];
				vertice.z = (*vi).P()[2];
				simp_output->AddPoint(vertice);
			}
			idx++;
		}
		CMeshO::FacePointer fp;
		for (fi = cm.face.begin(); fi != cm.face.end(); ++fi) {
			fp = &(*fi);
			if (!fp->IsD()) {
				int3 face;
				face.x = indices[fp->cV(0)];
				face.y = indices[fp->cV(1)];
				face.z = indices[fp->cV(2)];
				simp_output->AddFaceIdx(face);
			}
		}
		simp_output->SetObjectType(in_element->GetObjectType());
		resulted_element_ = simp_output;
		out_element = simp_output;
	}
}