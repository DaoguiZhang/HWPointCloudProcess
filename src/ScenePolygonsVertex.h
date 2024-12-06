#include "hw_plane.h"

namespace HW 
{
	class ScenePolygonsVertex 
	{
	public:
		ScenePolygonsVertex();
		~ScenePolygonsVertex();
		void SetPlanesVec(std::vector<HWPlane*> planesvec);
		void SetThreshold(float dist);
		void BuildPolygonVertex();

		//���վ�������еĵ��ƽ��о���
		void ExtractPntsCluster();
		//
		Eigen::Vector2i GetPolyPntIdxFromPolyPnts(int idx);

		/*
		Ŀ�ģ����µ�polygon cluster������ϣ���ͬһ��ƽ���ϵ���������ֿ�
		���룺polygons_clusters_
		�����polygons_clusters_opt_
		*/
		void OptimizeClustersCombination();

		/*
		Ŀ�ģ�����ƽ�棬��ϳ����polygon������ϣ�������idxs��,����У�����ƽ��ĵ㲻����һ��������
		���룺polygons_clusters_
		�����polygons_clusters_opt_
		*/
		void GenerateClusterCombinBasedOnPlane(std::vector<std::vector<Eigen::Vector2i> >& clusters_pnt, 
			    std::vector<std::vector<Eigen::Vector2i> >& planes_cluster);

		/*
		Ŀ�ģ�����ƽ������У�����ƽ��ĵ㲻����һ��������,�������ǵĸ���
		���룺clusters_pnt
		�����ƽ���ж������������ƽ������
		*/
		std::vector<int> ComputePlanePntsClusterMore2Num(std::vector<std::vector<Eigen::Vector2i> >& clusters_pnt);

		/*
		Ŀ�ģ�����ƽ�棬��ϳ����polygon������ϣ�������idxs��,����У�Ѱ��һ��ƽ�е�ƽ��
		���룺clusters_plane
		�����paralle_idx�����ǻ���clusters_plane���±�
		*/
		void FindAPairParallelPlaneIdx(std::vector<Eigen::Vector2i >& clusters_plane,
			std::vector<int>& paralle_idx);

		/*
		Ŀ�ģ������е�����Ķ������concat
		���룺��������
		���������һ����µ�����
		*/
		void  ConcateArrayData(std::vector<std::vector<Eigen::Vector2i> >& sor_idxs, 
			std::vector<Eigen::Vector2i>& tgtidxs, std::vector<std::vector<Eigen::Vector2i> >& soridxnew);

		/*
		Ŀ�ģ��Ż����е�polygon�Ķ���
		���룺cluster_polyidx �ֳ�������ƽ��
		�����polygon idx, �Լ��Ż���op_pnt����
		*/
		void OptimizeAllPolygonsVertex();

		/*
		Ŀ�ģ��Ż����е�polygon�Ķ���
		���룺cluster_polyidx �ֳ�������ƽ��
		�����polygon idx, �Լ��Ż���op_pnt����
		*/
		void OptimizeAllPolygonsVertexOpt();

		/*
		Ŀ�ģ���cluster_idxs ���㰴��ƽ�����
		���룺cluster_idxs
		�����poly_idxs ÿ��ƽ�����ж��ٸ�����
		*/
		void SplitClusterPntsIntoPolyPnts(std::vector<Eigen::Vector2i>& cluster_idx, 
			std::vector<std::vector<Eigen::Vector2i> >& poly_idxs);

		/*
		Ŀ�꣺��Ϊ��һ��ƽ�����������㣬��Ҫ�����ƽ����������㣬�ֱ���࣬�õ�����polygon
		���룺����, seed_idx��� poly_idxs����Щ����Ϊͬһ��ƽ�������
		�������Ϊƽ��������������󣬲������������
		*/
		void SplitPolyPntsByPlaneIdx(std::vector<Eigen::Vector2i>& poly_idxs, Eigen::Vector2i& seed_idx, 
			std::vector<Eigen::Vector2i>& poly1_idx, std::vector<Eigen::Vector2i>& poly2_idx);

		/*
		Ŀ�ģ��Ż����ƽ���ཻ��һ�����㣬�ֳ�������ƽ���ཻһ������
		���룺cluster_polyidx �ֳ�������ƽ��
		�����polygon idx, �Լ��Ż���op_pnt����
		*/
		bool OptimizePolygonsVertex(std::vector<Eigen::Vector2i>& cluster_polyidx, 
			Eigen::Vector3f& op_pnt, std::vector<int>& planes_idx);

		/*
		Ŀ�ģ�������ϴ���
		���룺�������е���ֵ�������C(n,m)����n��ѡȡm���������, m,n
		������������
		*/
		std::vector<std::vector<int> > GenerateCombineData(std::vector<int>value, int m);

		/*
		Ŀ�ģ�debug����
		���룺cluster �ĵ���
		�����obj���ļ�
		*/
		void WriteClusterPolygonPnts(std::string path);

		/*
		Ŀ�ģ�debug����
		���룺��������
		�����obj���ļ�
		*/
		void WriteSelectedPolygonPnts(std::string path, std::vector<Eigen::Vector3f>& polypnts);

		/*
		Ŀ�ģ�debug����
		���룺ƽ�������
		�������ӡ��������
		*/
		void ScoutVector2i(std::vector<Eigen::Vector2i>& data);

		/*
		Ŀ�ģ�debug����
		���룺ƽ�������
		�������ӡ��������
		*/
		void ScoutVectorVector2i(std::vector<std::vector<Eigen::Vector2i> >& data);

	private:
		
		//���涥���ƽ�����������ƽ���ϵ�����
		float dist_threshold_;
		std::vector<HWPlane*> planes_vec_;
		
		//����;�еĶ���
		std::vector<Eigen::Vector3f> polygons_pnts_;
		std::vector<int> polygons_num_;
		std::vector<std::vector<Eigen::Vector2i> > polygons_clusters_;
		//��cluster�����Ż�����,ȥ��һЩƽ���ϵ���������ͬһcluster��
		std::vector<std::vector<Eigen::Vector2i> > polygons_clusters_opt_;
	};
}