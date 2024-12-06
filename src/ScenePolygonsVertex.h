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

		//按照距离对所有的点云进行聚类
		void ExtractPntsCluster();
		//
		Eigen::Vector2i GetPolyPntIdxFromPolyPnts(int idx);

		/*
		目的：对新的polygon cluster进行组合，在同一个平面上的两个顶点分开
		输入：polygons_clusters_
		输出：polygons_clusters_opt_
		*/
		void OptimizeClustersCombination();

		/*
		目的：基于平面，组合出多个polygon数据组合（类似于idxs）,组合中，两个平面的点不能在一个顶点上
		输入：polygons_clusters_
		输出：polygons_clusters_opt_
		*/
		void GenerateClusterCombinBasedOnPlane(std::vector<std::vector<Eigen::Vector2i> >& clusters_pnt, 
			    std::vector<std::vector<Eigen::Vector2i> >& planes_cluster);

		/*
		目的：基于平面组合中，两个平面的点不能在一个顶点上,计算它们的个数
		输入：clusters_pnt
		输出：平面中多于两个顶点的平面数量
		*/
		std::vector<int> ComputePlanePntsClusterMore2Num(std::vector<std::vector<Eigen::Vector2i> >& clusters_pnt);

		/*
		目的：基于平面，组合出多个polygon数据组合（类似于idxs）,组合中，寻找一对平行的平面
		输入：clusters_plane
		输出：paralle_idx，它是基于clusters_plane的下标
		*/
		void FindAPairParallelPlaneIdx(std::vector<Eigen::Vector2i >& clusters_plane,
			std::vector<int>& paralle_idx);

		/*
		目的：对其中的数组的顶点进行concat
		输入：顶点数组
		输出：和在一起的新的数组
		*/
		void  ConcateArrayData(std::vector<std::vector<Eigen::Vector2i> >& sor_idxs, 
			std::vector<Eigen::Vector2i>& tgtidxs, std::vector<std::vector<Eigen::Vector2i> >& soridxnew);

		/*
		目的：优化所有的polygon的顶点
		输入：cluster_polyidx 分成三三个平面
		输出：polygon idx, 以及优化的op_pnt坐标
		*/
		void OptimizeAllPolygonsVertex();

		/*
		目的：优化所有的polygon的顶点
		输入：cluster_polyidx 分成三三个平面
		输出：polygon idx, 以及优化的op_pnt坐标
		*/
		void OptimizeAllPolygonsVertexOpt();

		/*
		目的：对cluster_idxs 顶点按照平面分类
		输入：cluster_idxs
		输出：poly_idxs 每个平面上有多少个顶点
		*/
		void SplitClusterPntsIntoPolyPnts(std::vector<Eigen::Vector2i>& cluster_idx, 
			std::vector<std::vector<Eigen::Vector2i> >& poly_idxs);

		/*
		目标：因为有一个平面有两个顶点，需要用这个平面的两个顶点，分别聚类，得到两个polygon
		输入：顶点, seed_idx标记 poly_idxs中那些顶点为同一个平面的索引
		输出：因为平面上有两个顶点后，拆分这两个顶点
		*/
		void SplitPolyPntsByPlaneIdx(std::vector<Eigen::Vector2i>& poly_idxs, Eigen::Vector2i& seed_idx, 
			std::vector<Eigen::Vector2i>& poly1_idx, std::vector<Eigen::Vector2i>& poly2_idx);

		/*
		目的：优化多个平面相交于一个顶点，分成三三个平面相交一个顶点
		输入：cluster_polyidx 分成三三个平面
		输出：polygon idx, 以及优化的op_pnt坐标
		*/
		bool OptimizePolygonsVertex(std::vector<Eigen::Vector2i>& cluster_polyidx, 
			Eigen::Vector3f& op_pnt, std::vector<int>& planes_idx);

		/*
		目的：生成组合代码
		输入：将数组中的数值进行组合C(n,m)，从n中选取m个数组组合, m,n
		输出：组合数据
		*/
		std::vector<std::vector<int> > GenerateCombineData(std::vector<int>value, int m);

		/*
		目的：debug数据
		输入：cluster 的点云
		输出：obj的文件
		*/
		void WriteClusterPolygonPnts(std::string path);

		/*
		目的：debug数据
		输入：点云坐标
		输出：obj的文件
		*/
		void WriteSelectedPolygonPnts(std::string path, std::vector<Eigen::Vector3f>& polypnts);

		/*
		目的：debug数据
		输入：平面的索引
		输出：打印出来数据
		*/
		void ScoutVector2i(std::vector<Eigen::Vector2i>& data);

		/*
		目的：debug数据
		输入：平面的索引
		输出：打印出来数据
		*/
		void ScoutVectorVector2i(std::vector<std::vector<Eigen::Vector2i> >& data);

	private:
		
		//保存顶点的平面索引，这个平面上的索引
		float dist_threshold_;
		std::vector<HWPlane*> planes_vec_;
		
		//处理途中的顶点
		std::vector<Eigen::Vector3f> polygons_pnts_;
		std::vector<int> polygons_num_;
		std::vector<std::vector<Eigen::Vector2i> > polygons_clusters_;
		//对cluster进行优化处理,去掉一些平面上的两个点在同一cluster上
		std::vector<std::vector<Eigen::Vector2i> > polygons_clusters_opt_;
	};
}