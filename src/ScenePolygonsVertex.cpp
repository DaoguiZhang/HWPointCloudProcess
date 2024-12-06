#include "ScenePolygonsVertex.h"
#include <pcl/segmentation/extract_clusters.h>
// PointXYZ的欧式聚类需要加.hpp，不然编译失败
#include <pcl/segmentation/impl/extract_clusters.hpp>

namespace HW
{
	bool CmpVector2i(Eigen::Vector2i& a, Eigen::Vector2i& b)
	{
		return (a[0] < b[0]);
	}

	ScenePolygonsVertex::ScenePolygonsVertex()
	{
	}
	ScenePolygonsVertex::~ScenePolygonsVertex()
	{
	}
	void ScenePolygonsVertex::SetPlanesVec(std::vector<HWPlane*> planesvec)
	{
		planes_vec_ = planesvec;
	}
	void ScenePolygonsVertex::SetThreshold(float dist)
	{
		dist_threshold_ = dist;
	}
	void ScenePolygonsVertex::BuildPolygonVertex()
	{
		//
		std::cout << "start to BuildPolygonVertex..." << std::endl;
		int current_sum_idx = 0;
		for (int i = 0; i < planes_vec_.size(); ++i)
		{
			std::vector<float3> polypnts = planes_vec_[i]->GetOuterPolygon();
			for (int j = 0; j < polypnts.size(); ++j)
			{
				polygons_pnts_.emplace_back(Eigen::Vector3f(polypnts[j].x, polypnts[j].y, polypnts[j].z));
			}
			current_sum_idx = current_sum_idx + polypnts.size();
			polygons_num_.emplace_back(current_sum_idx - 1);
		}
	}

	void ScenePolygonsVertex::ExtractPntsCluster()
	{
		std::cout << "start to ExtractPntsCluster..." << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pnts(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		
		//fill in cloud data
		for (int i = 0; i < polygons_pnts_.size(); ++i)
		{
			pcl::PointXYZ pnt;
			pnt.x = polygons_pnts_[i][0];
			pnt.y = polygons_pnts_[i][1];
			pnt.z = polygons_pnts_[i][2];
			cloud_pnts->points.emplace_back(pnt);
		}
		
		tree->setInputCloud(cloud_pnts);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(dist_threshold_);
		ec.setMinClusterSize(3);
		ec.setMaxClusterSize(50);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud_pnts);
		ec.extract(cluster_indices);

		//int j = 0;
		std::vector<std::vector<int> > clusters_idx;
		std::vector<std::vector<Eigen::Vector3f> > clusters_pnts3d;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
			it != cluster_indices.end(); ++it)
		{
			std::vector<int> clusteridx;
			std::vector<Eigen::Vector3f> clusterpnts;
			for (const auto& idx : it->indices)
			{
				clusteridx.emplace_back(idx);
				clusterpnts.emplace_back(Eigen::Vector3f((*cloud_pnts)[idx].x, (*cloud_pnts)[idx].y, (*cloud_pnts)[idx].z));
			}
			clusters_idx.emplace_back(clusteridx);
			clusters_pnts3d.emplace_back(clusterpnts);
		}

		////check code
		//std::string path = "D:\\vc_project_new\\test_data\\lake\\test3\\clusters.obj";
		//std::ofstream fh(path);
		//std::cout << "the clusters_pnts3d.size(): " << clusters_pnts3d.size() << std::endl;
		//for (int i = 0; i < clusters_pnts3d.size(); ++i)
		//{
		//	Eigen::Vector3i color;
		//	std::srand((int)time(0));
		//	int color1 = std::rand() / (255 * 255 * 255);
		//	int color1remain = std::rand() % (255 * 255 * 255);
		//	int color2 = color1remain / (255 * 255);
		//	int color2remain = color1remain % (255 * 255);
		//	int color3 = color2remain % 255;
		//	color[0] = ((i+1)*std::rand()) % 255;
		//	color[1] = ((i+1)*std::rand()) % 255;
		//	color[2] = ((i+1)*std::rand()) % 255;
		//	std::cout << "the clusters_pnts3d[i].size(): " << clusters_pnts3d[i].size() << std::endl;
		//	for (int j = 0; j < clusters_pnts3d[i].size(); ++j)
		//	{
		//		std::cout <<"v: " << clusters_pnts3d[i][j][0] << " " << clusters_pnts3d[i][j][1] << " " << clusters_pnts3d[i][j][2]
		//			<< " " << color[0] << " " << color[1] << " " << color[2] << std::endl;
		//		fh << "v " << clusters_pnts3d[i][j][0] << " " << clusters_pnts3d[i][j][1] << " " << clusters_pnts3d[i][j][2]
		//			<< " " << color[0] << " " << color[1] << " " << color[2] << std::endl;
		//	}
		//}
		//fh.close();
		////end check
		for (int i = 0; i < clusters_idx.size(); ++i)
		{
			std::vector<Eigen::Vector2i> tmppolyidxs;
			for (int j = 0; j < clusters_idx[i].size(); ++j)
			{
				Eigen::Vector2i tmpidx = GetPolyPntIdxFromPolyPnts(clusters_idx[i][j]);
				//std::cout << "tmpidx " << i << ", " << j << ": " << tmpidx[0] << " " << tmpidx[1] << std::endl;
				tmppolyidxs.emplace_back(tmpidx);
			}
			polygons_clusters_.emplace_back(tmppolyidxs);
		}
		/*std::string path2 = "D:\\vc_project_new\\test_data\\lake\\test18\\clusters2.obj";
		WriteClusterPolygonPnts(path2);*/
	}

	Eigen::Vector2i ScenePolygonsVertex::GetPolyPntIdxFromPolyPnts(int idx)
	{
		//通过idx来获取它在哪个polygon和哪个polygon idx
		if (idx >= polygons_pnts_.size() && idx < 0)
			return Eigen::Vector2i(-1, -1);
		int polygonidx = -1;
		for (int i = 0; i < polygons_num_.size(); ++i)
		{
			if (idx <= polygons_num_[i])
			{
				polygonidx = i;
				break;
			}
		}
		if (polygonidx == 0)
		{
			return Eigen::Vector2i(polygonidx, idx);
		}
		else
		{
			Eigen::Vector2i tmpidx(-1, -1);
			int pre_idx = polygons_num_[polygonidx - 1];
			int pntidx = idx - pre_idx;
			tmpidx[0] = polygonidx;
			tmpidx[1] = pntidx - 1;
			return tmpidx;
		}
	}

	void ScenePolygonsVertex::OptimizeClustersCombination()
	{
		//
		for (int i = 0; i < polygons_clusters_.size(); ++i)
		{
			if (polygons_clusters_[i].size() < 3)
			{
				polygons_clusters_opt_.emplace_back(polygons_clusters_[i]);
			}
			else
			{
				std::vector<std::vector<Eigen::Vector2i> > cl_split_combination;
				std::vector<std::vector<Eigen::Vector2i> > cl_split_idx;
				SplitClusterPntsIntoPolyPnts(polygons_clusters_[i], cl_split_idx);
				if (cl_split_idx.size() == polygons_clusters_[i].size())
				{
					polygons_clusters_opt_.emplace_back(polygons_clusters_[i]);
				}
				else
				{
					//GenerateClusterCombinBasedOnPlane(cl_split_idx, cl_split_combination);
					//去掉那些废掉的组合，留下满足条件的组合
					//删除冗余顶点,这一步需要做
					std::vector<int> planemore2idx = ComputePlanePntsClusterMore2Num(cl_split_idx);
					if (planemore2idx.size() == 1)
					{
						int c_idx = planemore2idx[0];
						std::vector<Eigen::Vector2i> tmp_idxvec;
						Eigen::Vector2i tmp_seed;
						for (int j = 0; j < cl_split_idx.size(); ++j)
						{
							if (j == c_idx)
							{
								for (int k = 0; k < 2; k++)
								{
									if (k == 0)
									{
										tmp_idxvec.emplace_back(cl_split_idx[j][k]);
										tmp_seed[0] = j + k;
									}
									if (k == 1)
									{
										tmp_idxvec.emplace_back(cl_split_idx[j][k]);
										tmp_seed[1] = j + k;
									}
								}
							}
							else
							{
								tmp_idxvec.emplace_back(cl_split_idx[j][0]);
							}
						}

						std::vector<Eigen::Vector2i> poly1idxs, poly2idxs;
						//seed 这个polygon，它是尖角顶点才行，其它顶点，可能会错，后续处理。
						SplitPolyPntsByPlaneIdx(tmp_idxvec, tmp_seed, poly1idxs, poly2idxs);
						std::cout << "start to split into 2 polygons..." << std::endl;
						polygons_clusters_opt_.emplace_back(poly1idxs);
						polygons_clusters_opt_.emplace_back(poly2idxs);
					}
					else if(planemore2idx.size() > 1)
					{
						//寻找细长的顶点索引
						int min_lgidx = 0;
						for (int j = 0; j < planemore2idx.size(); ++j)
						{
							//planes_vec_
						}
						//获取sharp 的顶点聚集在一起,后续在做，现在就做基本的
						int c_idx = planemore2idx[0];
						std::vector<Eigen::Vector2i> tmp_idxvec;
						Eigen::Vector2i tmp_seed;
						for (int j = 0; j < cl_split_idx.size(); ++j)
						{
							if (j == c_idx)
							{
								for (int k = 0; k < 2; k++)
								{
									if (k == 0)
									{
										tmp_idxvec.emplace_back(cl_split_idx[j][k]);
										tmp_seed[0] = j + k;
									}
									if (k == 1)
									{
										tmp_idxvec.emplace_back(cl_split_idx[j][k]);
										tmp_seed[1] = j + k;
									}
								}
							}
							else
							{
								for (int k = 0; k < cl_split_idx[j].size(); ++k)
								{
									tmp_idxvec.emplace_back(cl_split_idx[j][k]);
								}
								
							}
						}
						std::cout << "tmp_idxvec: " << std::endl;
						ScoutVector2i(tmp_idxvec);
						std::vector<Eigen::Vector2i> poly1idxs, poly2idxs;
						//seed 这个polygon，它是尖角顶点才行，其它顶点，可能会错，后续处理。
						SplitPolyPntsByPlaneIdx(tmp_idxvec, tmp_seed, poly1idxs, poly2idxs);
						std::cout << "start to split into 2 polygons..." << std::endl;
						polygons_clusters_opt_.emplace_back(poly1idxs);
						polygons_clusters_opt_.emplace_back(poly2idxs);

						//polygons_clusters_opt_.emplace_back(polygons_clusters_[i]);
					}
					else
					{
						polygons_clusters_opt_.emplace_back(polygons_clusters_[i]);
					}
				}
			}
		}
	}

	void ScenePolygonsVertex::GenerateClusterCombinBasedOnPlane(std::vector<std::vector<Eigen::Vector2i> >& clusters_pnt,
		std::vector<std::vector<Eigen::Vector2i> >& planes_cluster)
	{
		std::vector<std::vector<Eigen::Vector2i> > current_idxs_new;
		for (int i = 0; i < clusters_pnt.size(); ++i)
		{
			std::vector<Eigen::Vector2i> tmp_idxs = clusters_pnt[i];
			std::vector<std::vector<Eigen::Vector2i> > current_idxs = current_idxs_new;
			ConcateArrayData(current_idxs, tmp_idxs, current_idxs_new);
		}
		planes_cluster = current_idxs_new;
	}

	std::vector<int> ScenePolygonsVertex::ComputePlanePntsClusterMore2Num(std::vector<std::vector<Eigen::Vector2i> >& clusters_pnt)
	{
		int icount = 0;
		std::vector<int> tmpmore2idx;
		for (int i = 0; i < clusters_pnt.size(); ++i)
		{
			if (clusters_pnt[i].size() >= 2)
			{
				tmpmore2idx.emplace_back(i);
				++icount;
			}
		}
		return  tmpmore2idx;
	}

	void ScenePolygonsVertex::FindAPairParallelPlaneIdx(std::vector<Eigen::Vector2i>& clusters_plane,
		std::vector<int>& paralle_idx)
	{
		bool first_parallel_flag = false;
		for (int i = 0; i < clusters_plane.size(); ++i)
		{
			for (int j = i; j < clusters_plane.size(); ++j)
			{
				if (i == j)
					continue;
				float4 coeffi = planes_vec_[clusters_plane[i][0]]->GetPlaneCoeff();
				float4 coeffj = planes_vec_[clusters_plane[j][0]]->GetPlaneCoeff();
				Eigen::Vector3f n1(coeffi.x, coeffi.y, coeffi.z);
				Eigen::Vector3f n2(coeffj.x, coeffj.y, coeffj.z);
				n1.normalize();
				n2.normalize();
				std::cout << "n1.dot(n2): " << std::abs(n1.dot(n2)) << std::endl;
				//表示它们平面，因为它们法向量几乎相等
				if (std::abs(n1.dot(n2)) > 0.8 )
				{
					if (!first_parallel_flag)
					{
						paralle_idx.emplace_back(i);
						paralle_idx.emplace_back(j);
						first_parallel_flag = true;
					}
					else
					{
						paralle_idx.emplace_back(j);
					}
				}
			}
			if (first_parallel_flag)
				break;
		}
	}

	void  ScenePolygonsVertex::ConcateArrayData(std::vector<std::vector<Eigen::Vector2i> >& sor_idxs,
		std::vector<Eigen::Vector2i>& tgtidxs, std::vector<std::vector<Eigen::Vector2i> >& soridxnew)
	{
		for (int i = 0; i < tgtidxs.size(); ++i)
		{
			for (int j = 0; j < sor_idxs.size(); ++j)
			{
				std::vector<Eigen::Vector2i> tmp_soridx = sor_idxs[j];
				tmp_soridx.emplace_back(tgtidxs[i]);
				soridxnew.emplace_back(tmp_soridx);
			}
		}
	}

	void ScenePolygonsVertex::OptimizeAllPolygonsVertex()
	{
		//对每个顶点的cluster，获取他们的polygon idx，然后获取这些polygon 之间进行相交
		std::cout << "start to OptimizeAllPolygonsVertex..." << std::endl;
		for (int i = 0; i < polygons_clusters_.size(); ++i)
		{
			std::sort(polygons_clusters_[i].begin(), polygons_clusters_[i].end(), CmpVector2i);
			std::vector<Eigen::Vector2i> cluster_pntsidx = polygons_clusters_[i];
			//去除那种重复平面的情况,后续再做
			//std::vector<Eigen::Vector>

			//获取这些polygon的交点
			if (cluster_pntsidx.size() >= 3)
			{
				Eigen::Vector3f optipnt;
				std::vector<int> planesidx;
				OptimizePolygonsVertex(cluster_pntsidx, optipnt, planesidx);
				//将顶点移动到优化处
				for (int j = 0; j < planesidx.size(); ++j)
				{
					//std::cout <<
					//获取顶点
					float3 sorpnt = planes_vec_[cluster_pntsidx[planesidx[j]][0]]->
						GetOuterPolygon()[cluster_pntsidx[planesidx[j]][1]];
					Eigen::Vector3f sor_pnt = Eigen::Vector3f(sorpnt.x, sorpnt.y, sorpnt.z);
					float movedist = (sor_pnt - optipnt).norm();
					if (movedist < (3 * dist_threshold_))
					{
						planes_vec_[cluster_pntsidx[planesidx[j]][0]]
							->SetCornerPt3d(optipnt, cluster_pntsidx[planesidx[j]][1]);
					}
				}
			}
		}
	}

	void ScenePolygonsVertex::OptimizeAllPolygonsVertexOpt()
	{
		//对每个顶点的cluster，获取他们的polygon idx，然后获取这些polygon 之间进行相交
		std::cout << "start to OptimizeAllPolygonsVertexOpt..." << std::endl;
		for (int i = 0; i < polygons_clusters_.size(); ++i)
		{
			std::sort(polygons_clusters_[i].begin(), polygons_clusters_[i].end(), CmpVector2i);
		}
		std::cout << "the polygons_clusters_ size: " << polygons_clusters_.size() << std::endl;
		//ScoutVectorVector2i(polygons_clusters_);
		//获取两个opti顶点
		OptimizeClustersCombination();
		std::cout << "the polygons_clusters_opt_ size: " << polygons_clusters_opt_.size() << std::endl;
		//ScoutVectorVector2i(polygons_clusters_opt_);
		for (int i = 0; i < polygons_clusters_opt_.size(); ++i)
		{
			std::vector<Eigen::Vector2i> sor_cluster_pntsidx = polygons_clusters_opt_[i];
			//寻找polygon平面中的平行平面的索引
			std::vector<int> parallel_idx;
			FindAPairParallelPlaneIdx(sor_cluster_pntsidx, parallel_idx);
			std::vector<Eigen::Vector2i> cluster_pntsidx;
			std::vector<Eigen::Vector2i> cluster_remainidx;
			if (parallel_idx.size() >= 2)
			{
				//保留面积大的平面，去掉那些小的平面
				float max_area = std::numeric_limits<float>::lowest();
				int max_idx = 0;
				for (int j = 0; j < parallel_idx.size(); ++j)
				{
					//planes_vec_[sor_cluster_pntsidx[j][0]]->ComputeArea();
					float my_area = planes_vec_[sor_cluster_pntsidx[parallel_idx[j]][0]]->GetArea();
					//std::cout << "the my_area: " << my_area << std::endl;
					if (my_area > max_area)
					{
						max_idx = parallel_idx[j];
						max_area = my_area;
					}
				}
				//std::cout << "the max_idx: " << max_idx << std::endl;
				for (int j = 0; j < sor_cluster_pntsidx.size(); ++j)
				{
					if (std::find(parallel_idx.begin(), parallel_idx.end(), j) == parallel_idx.end())
					{
						cluster_pntsidx.emplace_back(sor_cluster_pntsidx[j]);
					}
				}
				cluster_pntsidx.emplace_back(sor_cluster_pntsidx[max_idx]);
				//std::cout << "the cluster_pntsidx: " << std::endl;
				//ScoutVector2i(cluster_pntsidx);
				//处理cluster_remainidx
				for (int j = 0; j < parallel_idx.size(); ++j)
				{
					if (max_idx != parallel_idx[j])
					{
						cluster_remainidx.emplace_back(sor_cluster_pntsidx[parallel_idx[j]]);
					}
				}
				//std::cout << "the cluster_remainidx: " << std::endl;
				//ScoutVector2i(cluster_remainidx);
			}
			else
			{
				cluster_pntsidx = polygons_clusters_opt_[i];
			}
			//获取这些polygon的交点
			if (cluster_pntsidx.size() >= 3)
			{
				Eigen::Vector3f optipnt;
				std::vector<int> planesidx;
				OptimizePolygonsVertex(cluster_pntsidx, optipnt, planesidx);
				//将顶点移动到优化处
				for (int j = 0; j < planesidx.size(); ++j)
				{
					//获取顶点
					float3 sorpnt = planes_vec_[cluster_pntsidx[planesidx[j]][0]]->
						GetOuterPolygon()[cluster_pntsidx[planesidx[j]][1]];
					Eigen::Vector3f sor_pnt = Eigen::Vector3f(sorpnt.x, sorpnt.y, sorpnt.z);
					float movedist = (sor_pnt - optipnt).norm();
					Eigen::Vector2f optipnt2d;
					planes_vec_[cluster_pntsidx[planesidx[j]][0]]->Pnt3d2Pnt2D(optipnt, optipnt2d);
					float2 optipnt2df = make_float2(optipnt2d[0], optipnt2d[1]);
					if (movedist < (3 * dist_threshold_)
						&& !planes_vec_[cluster_pntsidx[planesidx[j]][0]]->IsSelfIntersection(cluster_pntsidx[planesidx[j]][1], optipnt2df))
					{
						planes_vec_[cluster_pntsidx[planesidx[j]][0]]
							->SetCornerPt3d(optipnt, cluster_pntsidx[planesidx[j]][1]);
					}
				}
				for (int j = 0; j < cluster_remainidx.size(); ++j)
				{
					//处理平行的平面的顶点
					float3 sorpnt = planes_vec_[cluster_remainidx[j][0]]->
						GetOuterPolygon()[cluster_remainidx[j][1]];
					Eigen::Vector3f sor_pnt = Eigen::Vector3f(sorpnt.x, sorpnt.y, sorpnt.z);
					float movedist = (sor_pnt - optipnt).norm();
					Eigen::Vector2f optipnt2d;
					planes_vec_[cluster_remainidx[j][0]]->Pnt3d2Pnt2D(optipnt, optipnt2d);
					float2 optipnt2df = make_float2(optipnt2d[0], optipnt2d[1]);
					if (movedist < (3 * dist_threshold_)
						&& !planes_vec_[cluster_remainidx[j][0]]->IsSelfIntersection(cluster_remainidx[j][1], optipnt2df))
					{
						planes_vec_[cluster_remainidx[j][0]]
							->SetCornerPt3d(optipnt, cluster_remainidx[j][1]);
					}
				}
			}
			else if(cluster_pntsidx.size() == 2)
			{
				//将两个顶点合并为一个顶点
				float3 sorpnt1 = planes_vec_[cluster_pntsidx[0][0]]->
					GetOuterPolygon()[cluster_pntsidx[0][1]];
				Eigen::Vector3f sor_pnt1 = Eigen::Vector3f(sorpnt1.x, sorpnt1.y, sorpnt1.z);
				float3 sorpnt2 = planes_vec_[cluster_pntsidx[1][0]]->
					GetOuterPolygon()[cluster_pntsidx[1][1]];
				Eigen::Vector3f sor_pnt2 = Eigen::Vector3f(sorpnt2.x, sorpnt2.y, sorpnt2.z);
				Eigen::Vector3f optipnt = (sor_pnt2 + sor_pnt1) / 2;
				Eigen::Vector2f optipnt2d1;
				planes_vec_[cluster_pntsidx[0][0]]->Pnt3d2Pnt2D(optipnt, optipnt2d1);
				float2 optipnt2df1 = make_float2(optipnt2d1[0], optipnt2d1[1]);
				float movedist1 = (sor_pnt1 - optipnt).norm();
				if (movedist1 < (3 * dist_threshold_) 
					&& !planes_vec_[cluster_pntsidx[0][0]]->IsSelfIntersection(cluster_pntsidx[0][1], optipnt2df1))
				{
					planes_vec_[cluster_pntsidx[0][0]]
						->SetCornerPt3d(optipnt, cluster_pntsidx[0][1]);
				}
				float movedist2 = (sor_pnt2 - optipnt).norm();
				Eigen::Vector2f optipnt2d2;
				planes_vec_[cluster_pntsidx[1][0]]->Pnt3d2Pnt2D(optipnt, optipnt2d2);
				float2 optipnt2df2 = make_float2(optipnt2d2[0], optipnt2d2[1]);
				if (movedist2 < (3 * dist_threshold_)
					&& !planes_vec_[cluster_pntsidx[1][0]]->IsSelfIntersection(cluster_pntsidx[1][1], optipnt2df2))
				{
					planes_vec_[cluster_pntsidx[1][0]]
						->SetCornerPt3d(optipnt, cluster_pntsidx[1][1]);
				}
			}
		}
	}

	void ScenePolygonsVertex::SplitClusterPntsIntoPolyPnts(std::vector<Eigen::Vector2i>& cluster_idx,
		std::vector<std::vector<Eigen::Vector2i> >& poly_idxs)
	{
		//处理cluster的顶点
		int polycount = 0;
		int current_idx = -1;
		std::vector<Eigen::Vector2i> tmp_idxs;
		for (int i = 0; i < cluster_idx.size(); ++i)
		{
			//std::cout << "i: " << i << std::endl;
			int p_idx = cluster_idx[i][0];
			if (current_idx != p_idx)
			{
				if (!tmp_idxs.empty())
				{
					poly_idxs.emplace_back(tmp_idxs);
					tmp_idxs.clear();
				}
				polycount++;
				current_idx = p_idx;
			}
			tmp_idxs.emplace_back(cluster_idx[i]);
		}
		//最后的一位
		if (!tmp_idxs.empty())
		{
			poly_idxs.emplace_back(tmp_idxs);
			tmp_idxs.clear();
		}
	}

	void ScenePolygonsVertex::SplitPolyPntsByPlaneIdx(std::vector<Eigen::Vector2i>& poly_idxs, Eigen::Vector2i& seed_idx,
		std::vector<Eigen::Vector2i>& poly1_idx, std::vector<Eigen::Vector2i>& poly2_idx)
	{
		std::vector<Eigen::Vector2i> poly_no_seed_idx;
		Eigen::Vector2i poly1, poly2;
		for (int i = 0; i < poly_idxs.size(); ++i)
		{
			if (i == seed_idx[0])
			{
				poly1 = poly_idxs[i];
			}
			else if (i == seed_idx[1])
			{
				poly2 = poly_idxs[i];
			}
			else
			{
				poly_no_seed_idx.emplace_back(poly_idxs[i]);
			}
		}
		Eigen::Vector3f seed1pnt = planes_vec_[poly1[0]]->
			GetOuterPolygonPnt3d(poly1[1]);
		Eigen::Vector3f seed2pnt = planes_vec_[poly2[0]]->
			GetOuterPolygonPnt3d(poly2[1]);
		poly1_idx.emplace_back(poly1);
		poly2_idx.emplace_back(poly2);
		for (int i = 0; i < poly_no_seed_idx.size(); ++i)
		{
			Eigen::Vector3f tmp_no_seed = planes_vec_[poly_no_seed_idx[i][0]]->
				GetOuterPolygonPnt3d(poly_no_seed_idx[i][1]);
			float dist2seed1 = (tmp_no_seed - seed1pnt).norm();
			float dist2seed2 = (tmp_no_seed - seed2pnt).norm();
			if (dist2seed1 < dist2seed2)
			{
				poly1_idx.emplace_back(poly_no_seed_idx[i]);
			}
			else
			{
				poly2_idx.emplace_back(poly_no_seed_idx[i]);
			}
		}
	}

	bool ScenePolygonsVertex::OptimizePolygonsVertex(std::vector<Eigen::Vector2i>& cluster_polyidx,
		Eigen::Vector3f& op_pnt, std::vector<int>& planes_idx)
	{
		std::cout << "start to optimize single polygon idx..." << std::endl;
		std::vector<int> neighbor_poly_idxs;
		for (int i = 0; i < cluster_polyidx.size(); ++i)
		{
			//neighbor_poly_idxs.emplace_back(cluster_polyidx[i][0]);
			neighbor_poly_idxs.emplace_back(i);
		}
		std::vector<std::vector<int> > data_combined;
		int m = 3;
		data_combined = GenerateCombineData(neighbor_poly_idxs, m);
		std::vector<Eigen::Vector3f> plane_pnts;	//保存三个平面的交点
		plane_pnts.resize(data_combined.size());
		////test
		//for (int i = 0; i < data_combined.size(); ++i)
		//{
		//	std::cout << "combine " << i <<": ";
		//	for (int j = 0; j < data_combined[i].size(); ++j)
		//	{
		//		std::cout << data_combined[i][j] << " ";
		//	}
		//	std::cout << std::endl;
		//}
		////end test
		for (int i = 0; i < data_combined.size(); ++i)
		{
			std::vector<int> pl_idxs;
			for (int j = 0; j < data_combined[i].size(); ++j)
			{
				//std::cout << "i, j " << i << " " << j << ":  " << cluster_polyidx[data_combined[i][j]][0] << " ";
				pl_idxs.emplace_back(cluster_polyidx[data_combined[i][j]][0]);
			}
			std::cout << std::endl;
			std::vector<Eigen::Vector4f> planecoeff;
			for (int j = 0; j < pl_idxs.size(); ++j)
			{
				float4 tmpcoeff = planes_vec_[pl_idxs[j]]->GetPlaneCoeff();
				Eigen::Vector4f tmp_coeff = Eigen::Vector4f(tmpcoeff.x, tmpcoeff.y, tmpcoeff.z, tmpcoeff.w);
				//std::cout << "tmp_coeff: \n" << tmp_coeff << std::endl;
				planecoeff.emplace_back(tmp_coeff);
			}
			//std::cout << "2222222222222222222222222222222222222" << std::endl;
			//构建线性方程组
			Eigen::Matrix3f A;
			Eigen::Vector3f B;
			//这个构建有些问题
			for (int j = 0; j < 3; ++j)
			{
				A(j, 0) = planecoeff[j][0];
				A(j, 1) = planecoeff[j][1];
				A(j, 2) = planecoeff[j][2];
				//A << planecoeff[j][0], planecoeff[j][1], planecoeff[j][2];
				B[j] = -planecoeff[j][3];
			}
			//std::cout << "the A: \n" << A << std::endl;
			//std::cout << "the B: \n" << B << std::endl;
			//std::cout << "33333333333333333333333333333333333" << std::endl;
			Eigen::Vector3f kvalue = A.fullPivHouseholderQr().solve(B);
			plane_pnts[i] = kvalue;
		}

		/*std::string path4 = "D:\\vc_project_new\\test_data\\lake\\test18\\polypnts_opt.obj";
		WriteSelectedPolygonPnts(path4, plane_pnts);*/

		if (plane_pnts.size() > 1)
		{
			//去除异样的顶点
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
			//fill in the cloud data
			for (int i = 0; i < plane_pnts.size(); ++i)
			{
				pcl::PointXYZ pnt;
				pnt.x = plane_pnts[i][0];
				pnt.y = plane_pnts[i][1];
				pnt.z = plane_pnts[i][2];
				cloud->points.emplace_back(pnt);
			}
#if 0
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud(cloud);
			sor.setMeanK(plane_pnts.size());
			sor.setStddevMulThresh(1.0);
			pcl::IndicesPtr myindices;
			myindices = sor.getIndices();
			sor.filter(*cloud_filtered);
#else
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

			tree->setInputCloud(cloud);
			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance(dist_threshold_);	//怎么设置这些polygon
			ec.setMinClusterSize(3);
			ec.setMaxClusterSize(50);
			ec.setSearchMethod(tree);
			ec.setInputCloud(cloud);
			ec.extract(cluster_indices);

			//std::cout << "the cluster_indices size: " << cluster_indices.size() << std::endl;
			std::vector<std::vector<int> > clusters_idx;
			std::vector<std::vector<Eigen::Vector3f> > clusters_pnts3d;
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
				it != cluster_indices.end(); ++it)
			{
				std::vector<int> clusteridx;
				std::vector<Eigen::Vector3f> clusterpnts;
				for (const auto& idx : it->indices)
				{
					clusteridx.emplace_back(idx);
					clusterpnts.emplace_back(Eigen::Vector3f((*cloud)[idx].x, (*cloud)[idx].y, (*cloud)[idx].z));
				}
				clusters_idx.emplace_back(clusteridx);
				clusters_pnts3d.emplace_back(clusterpnts);
			}

			////check code
			//std::string path = "D:\\vc_project_new\\test_data\\lake\\test3\\clusters6.obj";
			//std::ofstream fh(path);
			//std::cout << "the clusters_pnts3d.size(): " << clusters_pnts3d.size() << std::endl;
			//for (int i = 0; i < clusters_pnts3d.size(); ++i)
			//{
			//	Eigen::Vector3i color;
			//	std::srand((int)time(0));
			//	/*int color1 = std::rand() / (255 * 255 * 255);
			//	int color1remain = std::rand() % (255 * 255 * 255);
			//	int color2 = color1remain / (255 * 255);
			//	int color2remain = color1remain % (255 * 255);
			//	int color3 = color2remain % 255;*/
			//	color[0] = ((i+1)*std::rand()) % 255;
			//	color[1] = ((i+1)*std::rand()) % 255;
			//	color[2] = ((i+1)*std::rand()) % 255;
			//	std::cout << "the clusters_pnts3d[i].size(): " << clusters_pnts3d[i].size() << std::endl;
			//	for (int j = 0; j < clusters_pnts3d[i].size(); ++j)
			//	{
			//		std::cout <<"v: " << clusters_pnts3d[i][j][0] << " " << clusters_pnts3d[i][j][1] << " " << clusters_pnts3d[i][j][2]
			//			<< " " << color[0] << " " << color[1] << " " << color[2] << std::endl;
			//		fh << "v " << clusters_pnts3d[i][j][0] << " " << clusters_pnts3d[i][j][1] << " " << clusters_pnts3d[i][j][2]
			//			<< " " << color[0] << " " << color[1] << " " << color[2] << std::endl;
			//	}
			//}
			//fh.close();
			////end check

			//std::cout << "the cluster_indices[0] size: " << cluster_indices.size() << std::endl;
			if (clusters_idx.size() == plane_pnts.size())
			{
				std::cout << "no plane cluster..." << std::endl;
				return false;
			}
			else
			{
				//获取最大的cluster
				int maxnumidx = -1;
				int maxnum = -1;
				for (int i = 0; i < clusters_idx.size(); ++i)
				{
					//std::cout << "the clusters_idx[i].size: " << clusters_idx[i].size() << std::endl;
					int currentnum = clusters_idx[i].size();
					if (maxnum < currentnum)
					{
						//std::cout << "asdfasdfasdfasf" << std::endl;
						maxnum = clusters_idx[i].size();
						maxnumidx = i;
					}
				}
				std::cout << "the maxnum: " << maxnum << std::endl;
				std::cout << "the maxnumidx: " << maxnumidx << std::endl;
				if (maxnumidx != -1 && !clusters_idx[maxnumidx].empty())
				{
					//Eigen::Vector3f tmppnt;
					std::vector<int> clusterspntsidx;
					Eigen::Vector3f sumpnt = Eigen::Vector3f(0.0, 0.0, 0.0);
					for (int i = 0; i < clusters_idx[maxnumidx].size(); ++i)
					{
						int tmppntidx = clusters_idx[maxnumidx][i];
						clusterspntsidx.emplace_back(tmppntidx);
						sumpnt += plane_pnts[tmppntidx];
					}
					op_pnt = sumpnt / clusters_idx[maxnumidx].size();
					std::cout << "the op_pnt: \n" << op_pnt << std::endl;
					std::vector<int> clustersplaneidx;
					for (int i = 0; i < clusterspntsidx.size(); ++i)
					{
						std::vector<int> planesidxs = data_combined[i];
						for (int j = 0; j < planesidxs.size(); ++j)
						{
							if (std::find(clustersplaneidx.begin(), clustersplaneidx.end(), planesidxs[j]) 
								== clustersplaneidx.end())
							{
								//std::cout << "1111111111111" << std::endl;
								clustersplaneidx.emplace_back(planesidxs[j]);
							}
						}
					}
					planes_idx = clustersplaneidx;
					//std::cout << "the planes_idx size: " << planes_idx.size() << std::endl;
					return true;
				}
				else
				{
					std::cout << "cluster maxnumidx == -1..." << std::endl;
					return false;
				}
			}
#endif
		}
		else if (plane_pnts.size() == 1)
		{
			op_pnt = plane_pnts[0];
			planes_idx = std::vector<int>{ 0, 1 , 2 };
			return true;
		}
		return false;
	}

	std::vector<std::vector<int> > ScenePolygonsVertex::GenerateCombineData(std::vector<int>value, int m)
	{
		if (value.size() < m)
			return {};
		std::vector<std::vector<int> > ret;
		if (value.size() == m)
		{
			ret.emplace_back(value);
			return ret;
		}
		else if (m == 1)
		{
			for (auto& it : value)
			{
				ret.emplace_back(std::vector<int>{it});
			}
			return ret;
		}
		else
		{
			while (value.size() >= m)
			{
				auto first = value.front();
				value.erase(value.begin());

				//从剩下的v[~]中选取m-1个数
				auto res = GenerateCombineData(value, m - 1);
				for (auto& it : res)
				{
					it.insert(it.begin(), first);
					ret.emplace_back(it);
				}
			}
		}
		return ret;
	}

	void ScenePolygonsVertex::WriteClusterPolygonPnts(std::string path)
	{
		std::ofstream fh(path);
		std::vector<std::vector<Eigen::Vector3f> > clusters_pnts3d;
		//std::cout << "the polygons_clusters_.size(): " << polygons_clusters_.size() << std::endl;
		for (int i = 0; i < polygons_clusters_.size(); ++i)
		{
			std::vector<Eigen::Vector3f> tmppnts;
			//std::cout << "the polygons_clusters_[i].size(): " << polygons_clusters_[i].size() << std::endl;
			for (int j = 0; j < polygons_clusters_[i].size(); ++j)
			{
				int planeidx = polygons_clusters_[i][j][0];
				int pntidx = polygons_clusters_[i][j][1];
				//std::cout << "the planeidx, pntidx: " << planeidx << " " << pntidx << std::endl;
				float3 pntpos = planes_vec_[planeidx]->GetOuterPolygon()[pntidx];
				//std::cout << "planes_vec_[planeidx]->GetOuterPolygon() size: " << planes_vec_[planeidx]->GetOuterPolygon().size() << std::endl;
				//std::cout << "pntpos: " << pntpos.x << " " << pntpos.y << " " << pntpos.z << std::endl;
				tmppnts.emplace_back(Eigen::Vector3f(pntpos.x, pntpos.y, pntpos.z));
			}
			clusters_pnts3d.emplace_back(tmppnts);
		}
		std::cout << "the clusters_pnts3d.size(): " << clusters_pnts3d.size() << std::endl;
		//从 planesvec获取
		for (int i = 0; i < clusters_pnts3d.size(); ++i)
		{
			Eigen::Vector3i color;
			std::srand((int)time(0));
			int color1 = std::rand() / (255 * 255 * 255);
			int color1remain = std::rand() % (255 * 255 * 255);
			int color2 = color1remain / (255 * 255);
			int color2remain = color1remain % (255 * 255);
			int color3 = color2remain % 255;
			color[0] = ((i + 1)*std::rand()) % 255;
			color[1] = ((i + 1)*std::rand()) % 255;
			color[2] = ((i + 1)*std::rand()) % 255;
			std::cout << "the clusters_pnts3d[i].size(): " << clusters_pnts3d[i].size() << std::endl;
			for (int j = 0; j < clusters_pnts3d[i].size(); ++j)
			{
				std::cout << "v: " << clusters_pnts3d[i][j][0] << " " << clusters_pnts3d[i][j][1] << " " << clusters_pnts3d[i][j][2]
					<< " " << color[0] << " " << color[1] << " " << color[2] << std::endl;

				fh << "v " << clusters_pnts3d[i][j][0] << " " << clusters_pnts3d[i][j][1] << " " << clusters_pnts3d[i][j][2]
					<< " " << color[0] << " " << color[1] << " " << color[2] << std::endl;
			}
		}
		fh.close();
	}

	void ScenePolygonsVertex::WriteSelectedPolygonPnts(std::string path, std::vector<Eigen::Vector3f>& polypnts)
	{
		std::ofstream fh(path);
		for (int i = 0; i < polypnts.size(); ++i)
		{
			fh << "v " << polypnts[i][0] << " " << polypnts[i][1] << " " << polypnts[i][2] << std::endl;
		}
		fh.close();
	}

	void ScenePolygonsVertex::ScoutVector2i(std::vector<Eigen::Vector2i>& data)
	{
		std::cout << "ScoutVector2i: " << std::endl;
		for (int i = 0; i < data.size(); ++i)
		{
			//std::cout << i << "th: " << data[i][0] << " " << data[i][1] << ", ";
			std::cout << data[i][0] << " " << data[i][1] << ", ";
		}
		std::cout << std::endl << std::endl;
	}

	void ScenePolygonsVertex::ScoutVectorVector2i(std::vector<std::vector<Eigen::Vector2i> >& data)
	{
		std::cout << "ScoutVectorVector2i: " << std::endl;
		for (int i = 0; i < data.size(); ++i)
		{
			for (int j = 0; j < data[i].size(); ++j)
			{
				std::cout << data[i][j][0] << " " << data[i][j][1] << ", ";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl << std::endl;
	}
}