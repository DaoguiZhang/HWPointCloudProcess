#include"hw_scene_layout2d.h"
#include<fstream>
#include<vector>
#include<iostream>
#include"hw_algorithms.h"
#include"hw_view_cmns.h"

namespace HW
{
    HWSceneLayout2D::HWSceneLayout2D()
    {
        layout_id_ = KMAXMUMLIMIT;
        image_id_ = KMAXMUMLIMIT;
		lines_near_angle_threshold_ = 5.0f;
		line_near_dist_threshold_ = 5.0f;	//ÏñËØ¾àÀë
        type_ = HWSceneLayoutDimType::kScene2DType;
		pnt_to_line_segment_dist_threshold_ = 1.0;
		lines_combined_angle_threshold_ = 5.0f;
		lines_combined_dist_threshold_ = 3.0f;	//ÏñËØ¾àÀë
		line_length_dist_threshold_ = 50.0f;
		//line_combine_dist_threshold_ = 5.0f;
		group_lines_angle_threshold_ = 5.0f;
		group_lines_dist_threshold_ = 3.0f;
		loaded_layout_state_ = false;
    }

    HWSceneLayout2D::HWSceneLayout2D(const HWSceneLayout2D& other)
    {
        this->layout_id_ = other.layout_id_;
        this->image_id_ = other.image_id_;
        this->layout_path_ = other.layout_path_;
        this->layout_basename_ = other.layout_basename_;
        this->type_ = other.type_;
		this->lines_near_angle_threshold_ = other.lines_near_angle_threshold_;
		this->line_near_dist_threshold_ = other.line_near_dist_threshold_;
		this->line_length_dist_threshold_ = other.line_length_dist_threshold_;
		this->lines_combined_angle_threshold_ = other.lines_combined_angle_threshold_;
		this->lines_combined_dist_threshold_ = other.lines_combined_dist_threshold_;
		this->lines_to_near_idx_ = other.lines_to_near_idx_;
        this->lines_segs_ = other.lines_segs_;
		this->lines_segs_origin_ = other.lines_segs_origin_;
		this->pnts_origin_ = other.pnts_origin_;
        this->pnts_ = other.pnts_;
        this->ly_cam_idx_ = other.ly_cam_idx_;
        this->img_lines_pnts_ = other.img_lines_pnts_;
		this->pnt_to_line_segment_dist_threshold_ = other.pnt_to_line_segment_dist_threshold_;
		this->lines_idxs_to_gouped_lines_idxs_ = other.lines_idxs_to_gouped_lines_idxs_;
		this->grouped_lines_pnts_ = other.grouped_lines_pnts_;
		this->group_lines_angle_threshold_ = other.group_lines_angle_threshold_;
		this->group_lines_dist_threshold_ = other.group_lines_dist_threshold_;
		this->loaded_layout_state_ = other.loaded_layout_state_;
    }

    HWSceneLayout2D::~HWSceneLayout2D()
    {}

    void HWSceneLayout2D::ReadLayout2D(const std::string& path)
    {
        //std::cerr <<"start to read layout from log ..." << std::endl;
        //std::cerr <<"path: " << path << std::endl;
        std::ifstream fh(path);
        if(fh.is_open())
        {
            std::string line_path;
            std::getline(fh, line_path);
            std::string line_head;
            std::getline(fh, line_head);
            std::stringstream ss(line_head);
            int pnts_num = 0;
            int lines_num = 0;
            ss >> pnts_num >> lines_num;
            std::cerr << "pnts num, lines num: " << pnts_num <<", " << lines_num << std::endl;
            //read points
            for(int i = 0; i < pnts_num; ++i)
            {
                //read point position
                std::string line;
                std::getline(fh, line);
                std::stringstream ssf(line);
                float x, y;
                ssf >> x >> y;
                HWLinePoint2D p;
                p.pnt_[0] = x;
                p.pnt_[1] = y;
                p.pnt_[2] = 0.0;
                //add to HWSceneLayout2D
                pnts_.emplace_back(p);
            }
            for(int i = 0; i < lines_num; ++i)
            {
                //read line index
                std::string line;
                std::getline(fh, line);
                std::stringstream ssi(line);
                int idx_s, idx_e;
                ssi >> idx_s >> idx_e;
                Eigen::Vector2i pidx;
                pidx[0] = idx_s;
                pidx[1] = idx_e;
                if(idx_s == -1 || idx_e == -1)
                {
                    continue;
                }
                lines_segs_.emplace_back(pidx);
            }
            fh.close();
			int lines_segs_num = static_cast<int>(lines_segs_.size());
			lines_to_near_idx_.resize(lines_segs_num, -1);
            //get file name
            layout_path_ = path;
            layout_basename_ = GetBaseNameWithoutSuffix(path);
        }
    }

    bool HWSceneLayout2D::ReadLayout2DFromNetPath(const std::string& path)
    {
        std::vector<std::vector<float> > lines_pnts;
        //get file name
        layout_path_ = path;
        layout_basename_ = GetBaseNameWithoutSuffix(path);
        if(ReadLayout2DFromNetData(path, lines_pnts))
        {
            SetSceneLout2DFromNetPnts(lines_pnts);
            return true;
        }
        return false;
    }

    void HWSceneLayout2D::SetHWLayoutDimType(HWSceneLayoutDimType type)
    {
        type_= type;
    }

    bool HWSceneLayout2D::ReadLayout2DFromNetData(const std::string& path, 
        std::vector<std::vector<float>>& lines_pnts)
    {
        std::cerr <<"start to read lines 2d from log..." << std::endl;
        std::ifstream fh(path);
        if(fh.is_open())
        {
            std::string file_path;
            std::getline(fh, file_path);
            std::string line;
            std::getline(fh, line);
            std::stringstream ss(line);
            int lines_num;
            ss >> lines_num;
            for(int i = 0; i < lines_num; ++i)
            {
                std::vector<float> line_pnt(4, 0.0);
                std::string strbuf;
                std::getline(fh, strbuf);
                if(fh.eof())
                {
                    fh.close();
                    return true;
                }
                std::stringstream ssbuf(strbuf);
                ssbuf >> line_pnt[0] >> line_pnt[1] >> line_pnt[2] >> line_pnt[3];
                lines_pnts.emplace_back(line_pnt);
            }
            fh.close();
            return true;
        }
        return false;
    }

	void HWSceneLayout2D::UpdateNearNeighborLinesSegments()
	{
		//
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > lines_segs;
		GetAllPairLinesPnts(lines_segs);
		//
		int lines_num = static_cast<int>(lines_segs.size());
		lines_to_near_idx_.resize(lines_num, -1);
		for (int i = 0; i < lines_segs.size(); ++i)
		{
			Eigen::Vector2f src_ls = lines_segs[i].first;
			Eigen::Vector2f src_le = lines_segs[i].second;
			Eigen::Vector2f src_dir = src_le - src_ls;
			src_dir.normalize();
			for (int j = i; j < lines_segs.size(); ++j)
			{
				if (j == i)
				{
					continue;
				}
				//compute lines near
				Eigen::Vector2f tgt_ls = lines_segs[j].first;
				Eigen::Vector2f tgt_le = lines_segs[j].second;
				Eigen::Vector2f tgt_dir = tgt_le - tgt_ls;
				tgt_dir.normalize();
				float st_angle = ComputeTwoLine2dAngle(src_ls, src_le, tgt_ls, tgt_le);
				float sl_dist = ComputePnt2DToLine2DDistF(tgt_ls, tgt_dir, src_ls);
				float el_dist = ComputePnt2DToLine2DDistF(tgt_ls, tgt_dir, src_le);
				float mean_dist = (sl_dist + el_dist) / 2;
				//std::cerr << "src_ls, src_le: " << src_ls.transpose() << ", " << src_le.transpose() << std::endl;
				//std::cerr << "tgt_ls, tgt_le: " << tgt_ls.transpose() << ", " << tgt_le.transpose() << std::endl;
				//std::cerr << "st_angle, mean dist: " << st_angle << ", " << mean_dist << std::endl;
				//line_near_dist_threshold_ = 10;
				if (mean_dist < line_near_dist_threshold_
					&& st_angle < lines_near_angle_threshold_)
				{
					//std::cerr << "-----------" << i << ", " << j << "-----------" << std::endl;
					lines_to_near_idx_[i] = j;
					lines_to_near_idx_[j] = i;
				}
			}
		}
	}

	void HWSceneLayout2D::GroupLinesSegmentsBasedOnLineDist(std::vector<std::vector<int> >& lines_groups)
	{
		std::cerr << "start to group the neighbor line segments..." << std::endl;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > lines_segs;
		GetAllPairLinesPnts(lines_segs);
		int lines_min_support = 2;
		std::vector<LinePnt2D> lines_segments;
		for (int i = 0; i < lines_segs.size(); ++i)
		{
			LinePnt2D tmp_line;
			tmp_line.line_ = lines_segs[i];
			tmp_line.clusterID = hw_none_classifed_id;
			lines_segments.emplace_back(tmp_line);
		}
		/*std::cerr << "lines_combined_dist_threshold_, lines_combined_angle_threshold_: " <<
			lines_combined_dist_threshold_ << ", " << lines_combined_angle_threshold_ << std::endl;*/
		LinesHWDBSCAN* lines_clusters = new LinesHWDBSCAN(lines_min_support, lines_combined_dist_threshold_,
			lines_combined_angle_threshold_, lines_segments);
		int cluster_num = lines_clusters->Run();
		//std::cerr << "line num: " << lines_segs.size() << std::endl;
		//std::cerr << "cluster_num num: " << cluster_num << std::endl;
		std::vector<LinePnt2D> lines_segments_new = lines_clusters->lines_points_;
		//std::cerr << "lines_segments_new num: " << lines_segments_new.size() << std::endl;
		for (int i = 0; i < lines_segments_new.size(); ++i)
		{
			std::vector<int> line_group;
			if (lines_segments_new[i].clusterID == hw_classifed_noise_id)
			{
				//printf("%d \n", lines_segments_new[i].clusterID);
				line_group.emplace_back(i);
				lines_groups.emplace_back(line_group);
			}
		}
		for (int i = 1; i < cluster_num; ++i)
		{
			//how to do it?
			std::vector<int> line_group;
			for (int j = 0; j < lines_segments_new.size(); ++j)
			{
				if (lines_segments_new[j].clusterID == i)
				{
					line_group.emplace_back(j);
				}
			}
			lines_groups.emplace_back(line_group);
		}
		//std::cerr << "the lines groups: " << lines_groups.size() << std::endl;
		//PrintLinesGroupsIdxsInfo(lines_groups);

		delete lines_clusters;
		std::cerr << "end group the neighbor line segments..." << std::endl;
	}

	void HWSceneLayout2D::GroupLinesSegmentsBasedOnLineDirAndDist()
	{
		std::cerr << "start to group the neighbor line segments..." << std::endl;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > lines_segs;
		GetAllPairLinesPnts(lines_segs);
		std::vector <std::pair<int, int> > lines_idxs_to_gouped_idxs;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > grouped_lines_pnts;
		//important
		std::vector<bool> lines_idx_visited;
		lines_idx_visited.resize(lines_segs.size(), false);
		for (int i = 0; i < lines_segs.size(); ++i)
		{
			if (lines_idx_visited[i])
			{
				continue;
			}
			std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_line_pnts = lines_segs[i];
			bool is_in_grouped_line = false;
			for (int j = 0; j < grouped_lines_pnts.size(); ++j)
			{
				bool is_two_line_same = false;
				std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_grouped_line = grouped_lines_pnts[j];
				is_two_line_same = IsTwoLinesSegsInSameLine(tmp_line_pnts.first, tmp_line_pnts.second,
					tmp_grouped_line.first, tmp_grouped_line.second);
				bool is_parallel_line = false;
				std::vector<Eigen::Vector2f> tmp_all_grouped_lines_pnts;
				if (is_two_line_same)
				{
					std::vector<int> group_idx_to_all_lines = 
						IdsValueFirstFromIdSecondValueFromPairsVector(lines_idxs_to_gouped_idxs, j);
					for (int k = 0; k < group_idx_to_all_lines.size(); ++k)
					{
						int tmp_group_idx_to_all = group_idx_to_all_lines[k];
						std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_group_idx_line_pnt 
							= lines_segs[tmp_group_idx_to_all];
						tmp_all_grouped_lines_pnts.emplace_back(tmp_group_idx_line_pnt.first);
						tmp_all_grouped_lines_pnts.emplace_back(tmp_group_idx_line_pnt.second);
						is_parallel_line = IsTwoLnesSegsParallelLine(tmp_line_pnts.first, tmp_line_pnts.second, 
							tmp_group_idx_line_pnt.first, tmp_group_idx_line_pnt.second);
					}
					if (!is_parallel_line)
					{
						//group new line into grouped lines
						//get all grouped lines pnts
						tmp_all_grouped_lines_pnts.emplace_back(tmp_line_pnts.first);
						tmp_all_grouped_lines_pnts.emplace_back(tmp_line_pnts.second);
						Eigen::Vector2f tmp_grouped_ls, tmp_grouped_le;
						FittingLineLsLePnts2dFromPnts2d2f(tmp_all_grouped_lines_pnts, 
							tmp_grouped_ls, tmp_grouped_le);
						std::pair<int, int> tmp_idx_to_grouped_idx = std::make_pair(i, j);
						lines_idxs_to_gouped_idxs.emplace_back(tmp_idx_to_grouped_idx);
						grouped_lines_pnts[j] = std::make_pair(tmp_grouped_ls, tmp_grouped_le);
						is_in_grouped_line = true;
						break;
					}
				}
			}
			if (!is_in_grouped_line)
			{
				//get new line
				int group_lines_idx = grouped_lines_pnts.size();
				grouped_lines_pnts.emplace_back(tmp_line_pnts);
				std::pair<int, int> tmp_idx_to_grouped_idx = std::make_pair(i, group_lines_idx);
				lines_idxs_to_gouped_idxs.emplace_back(tmp_idx_to_grouped_idx);
			}
			lines_idx_visited[i] = true;
		}
		lines_idxs_to_gouped_lines_idxs_ = lines_idxs_to_gouped_idxs;
		grouped_lines_pnts_ = grouped_lines_pnts;
		std::cerr << "end group the neighbor line segments..." << std::endl;
	}

	bool HWSceneLayout2D::IsTwoLinesSegsInSameLine(const Eigen::Vector2f& sc_ls, const Eigen::Vector2f& sc_le,
		const Eigen::Vector2f& tg_ls, const Eigen::Vector2f& tg_le)
	{
		float line_degree = ComputeTwoLine2dCrossAngle(sc_ls, sc_le, tg_ls, tg_le);
		Eigen::Vector2f scd = sc_le - sc_ls;
		Eigen::Vector2f tgd = tg_le - tg_ls;
		scd.norm();
		tgd.norm();
		Eigen::Vector2f sc_ls_proj;
		ComputePnt2dProj2Line2DF(tg_ls, tgd, sc_ls, sc_ls_proj);
		Eigen::Vector2f sc_le_proj;
		ComputePnt2dProj2Line2DF(tg_ls, tgd, sc_le, sc_le_proj);
		Eigen::Vector2f tg_ls_proj;
		ComputePnt2dProj2Line2DF(sc_ls, scd, tg_ls, tg_ls_proj);
		Eigen::Vector2f tg_le_proj;
		ComputePnt2dProj2Line2DF(sc_ls, scd, tg_le, tg_le_proj);
		float scls_to_tgtline = (sc_ls - sc_ls_proj).norm();
		float scle_to_tgtline = (sc_le - sc_le_proj).norm();
		float tgls_to_scrline = (tg_ls - tg_ls_proj).norm();
		float tgle_to_scrline = (tg_le - tg_le_proj).norm();
		float line_dist = (scls_to_tgtline + scle_to_tgtline + tgls_to_scrline + tgle_to_scrline) / 4.0;
		bool is_in_same_line = false;
		if (line_degree < group_lines_angle_threshold_ && line_dist < group_lines_dist_threshold_)
		{
			is_in_same_line = true;
		}
		return is_in_same_line;
		//bool is_parallel_line = false;
		////check if they are parallel
		////if the two line segments projected pnts overlap
		//if (CheckPntOnLineSegOnCollinearPnts(sc_ls_proj, tg_ls_proj, tg_le_proj)
		//	|| CheckPntOnLineSegOnCollinearPnts(sc_le_proj, tg_ls_proj, tg_le_proj)
		//	|| CheckPntOnLineSegOnCollinearPnts(tg_ls_proj, sc_ls_proj, sc_le_proj)
		//	|| CheckPntOnLineSegOnCollinearPnts(tg_le_proj, sc_ls_proj, sc_le_proj))
		//{
		//	is_parallel_line = false;
		//}

	}
	
	bool HWSceneLayout2D::IsTwoLnesSegsParallelLine(const Eigen::Vector2f& sc_ls, const Eigen::Vector2f& sc_le,
		const Eigen::Vector2f& tg_ls, const Eigen::Vector2f& tg_le)
	{
		Eigen::Vector2f scd = sc_le - sc_ls;
		Eigen::Vector2f tgd = tg_le - tg_ls;
		scd.norm();
		tgd.norm();
		Eigen::Vector2f sc_ls_proj;
		ComputePnt2dProj2Line2DF(tg_ls, tgd, sc_ls, sc_ls_proj);
		Eigen::Vector2f sc_le_proj;
		ComputePnt2dProj2Line2DF(tg_ls, tgd, sc_le, sc_le_proj);
		Eigen::Vector2f tg_ls_proj;
		ComputePnt2dProj2Line2DF(sc_ls, scd, tg_ls, tg_ls_proj);
		Eigen::Vector2f tg_le_proj;
		ComputePnt2dProj2Line2DF(sc_ls, scd, tg_le, tg_le_proj);
		bool is_parallel_line = false;
		//check if they are parallel
		//if the two line segments projected pnts overlap
		if (CheckPntOnLineSegOnCollinearPnts(sc_ls_proj, tg_ls_proj, tg_le_proj)
			|| CheckPntOnLineSegOnCollinearPnts(sc_le_proj, tg_ls_proj, tg_le_proj)
			|| CheckPntOnLineSegOnCollinearPnts(tg_ls_proj, sc_ls_proj, sc_le_proj)
			|| CheckPntOnLineSegOnCollinearPnts(tg_le_proj, sc_ls_proj, sc_le_proj))
		{
			is_parallel_line = false;
		}
		return is_parallel_line;
	}

	std::vector<int> HWSceneLayout2D::IdsValueFirstFromIdSecondValueFromPairsVector(const std::vector<std::pair<int, int> >& values_pair, int value_second)
	{
		std::vector<int> value_ids_first;
		for (int i = 0; i < values_pair.size(); ++i)
		{
			if (value_second == values_pair[i].second)
			{
				value_ids_first.emplace_back(values_pair[i].first);
			}
		}
		return value_ids_first;
	}

	void HWSceneLayout2D::RemovedNearNeighborLinesSegments()
	{
		//
		std::cerr << "start to filter the lines" << std::endl;
		std::vector<int>::iterator idx_iter = this->lines_to_near_idx_.begin();
		for (std::vector<Eigen::Vector2i>::iterator iter = this->lines_segs_.begin();
			iter != this->lines_segs_.end() && idx_iter != this->lines_to_near_idx_.end();)
		{
			if ((*idx_iter) != -1)
			{
				iter = this->lines_segs_.erase(iter);
				idx_iter = this->lines_to_near_idx_.erase(idx_iter);
			}
			else
			{
				iter++;
				idx_iter++;
			}
		}
		std::cerr << "end filter the lines" << std::endl;
	}

	void HWSceneLayout2D::RemoveLinesSegmentsWithLineLength()
	{
		std::cerr << "start to remove the line segment by length..." << std::endl;
		std::cerr << "before the lines_segs_ num: " << lines_segs_.size() << std::endl;
		for (std::vector<Eigen::Vector2i>::iterator iter = this->lines_segs_.begin();
			iter != this->lines_segs_.end();)
		{
			Eigen::Vector3f ls = pnts_[(*iter)[0]].pnt_;
			Eigen::Vector3f le = pnts_[(*iter)[1]].pnt_;
			float dist_line = (le - ls).norm();
			//std::cerr << "ls, le: " << ls.transpose() << ", " << le.transpose() << std::endl;
			//std::cerr << "dist_line: " << dist_line << std::endl;
			//std::cerr << "line_length_dist_threshold_: " << line_length_dist_threshold_ << std::endl;
			if (dist_line < line_length_dist_threshold_)
			{
				iter = this->lines_segs_.erase(iter);
			}
			else
			{
				iter++;
			}
		}
		std::cerr << "after, the lines_segs_ num: " << lines_segs_.size() << std::endl;
		std::cerr << "end remove the line segment by length..." << std::endl;
	}

	void HWSceneLayout2D::StoreLinesPntsToLinesOriginPnts()
	{
		pnts_origin_ = pnts_;
		lines_segs_origin_ = lines_segs_;
	}

	void HWSceneLayout2D::BackUpLineOriginPntsToLinePnts()
	{
		pnts_ = pnts_origin_;
		lines_segs_ = lines_segs_origin_;
	}

	void HWSceneLayout2D::UpdateGroupLinesToLineSegsNew(const std::vector<std::vector<int> >& lines_groups)
	{
		StoreLinesPntsToLinesOriginPnts();
		//clear
		lines_to_near_idx_.clear();
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > lines_new;
		for (int i = 0; i < lines_groups.size(); ++i)
		{
			const std::vector<int> lines_group = lines_groups[i];
			std::vector<Eigen::Vector3f> lines_group_pnts;
			for (int j = 0; j < lines_group.size(); ++j)
			{
				//Eigen::Vector2f
				int line_idx = lines_group[j];
				Eigen::Vector2i line_pnt_idxs = lines_segs_[line_idx];
				Eigen::Vector3f ls = pnts_[line_pnt_idxs[0]].pnt_;
				Eigen::Vector3f le = pnts_[line_pnt_idxs[1]].pnt_;
				ls[2] = 1.0;
				le[2] = 1.0;
				lines_group_pnts.emplace_back(ls);
				lines_group_pnts.emplace_back(le);
			}
			int lines_group_pnts_num = static_cast<int>(lines_group_pnts.size());
			if (lines_group_pnts_num >= 2)
			{
				std::pair<Eigen::Vector3f, Eigen::Vector3f> line_se;
				//fit the line
				FittingLine3dFromPnts3d3f(lines_group_pnts, line_se.first, line_se.second);
				std::pair<Eigen::Vector2f, Eigen::Vector2f> line_se2d;
				line_se2d.first = Eigen::Vector2f(line_se.first[0], line_se.first[1]);
				line_se2d.second = Eigen::Vector2f(line_se.second[0], line_se.second[1]);
				lines_new.emplace_back(line_se2d);
			}
			else
			{
				std::pair<Eigen::Vector2f, Eigen::Vector2f> line_se2d;
				line_se2d.first = Eigen::Vector2f(lines_group_pnts[0][0], lines_group_pnts[0][1]);
				line_se2d.second = Eigen::Vector2f(lines_group_pnts[1][0], lines_group_pnts[1][1]);
				lines_new.emplace_back(line_se2d);
			}
		}
		SetSceneLayoutFromLines2D(lines_new);
		std::cerr << "lines_new size: " << lines_new.size() << std::endl;
	}

	void HWSceneLayout2D::UpdateLinesCombinationProcess()
	{
		std::cerr << "start to combine the lines..." << std::endl;
		std::vector<bool> visited_flag(lines_segs_.size(), false);
		//UpdateNearNeighborLinesSegments();
		//lines_to_near_idx_
		std::vector<std::vector<int> > lines_groups;
		GroupLinesSegmentsBasedOnLineDist(lines_groups);
		UpdateGroupLinesToLineSegsNew(lines_groups);
		std::cerr << "end combine the lines..." << std::endl;
	}

	void HWSceneLayout2D::UpdateLinesCombinationProcessBasedOnLinesDir()
	{
		std::cerr << "start to combine the lines based on line dir and line dist..." << std::endl;
		std::vector<bool> visited_flag(lines_segs_.size(), false);
		//UpdateGroupLinesToLineSegsNew(lines_groups);
		GroupLinesSegmentsBasedOnLineDirAndDist();
		std::cerr << "end combine the lines..." << std::endl;
	}

	void HWSceneLayout2D::SaveLayoutIntoLog(const std::string& path)
	{
		std::ofstream fh(path);
		fh << layout_path_ << std::endl;
		int pnts_num = static_cast<int>(pnts_.size());
		int lines_num = static_cast<int>(lines_segs_.size());
		fh << pnts_num << " " << lines_num << std::endl;
		for (int i = 0; i < pnts_.size(); ++i)
		{
			fh << pnts_[i].pnt_[0] << " " << pnts_[i].pnt_[1] << std::endl;
		}
		for (int i = 0; i < lines_segs_.size(); ++i)
		{
			fh << lines_segs_[i][0] << " " << lines_segs_[i][1] << std::endl;
		}
		fh.close();
	}

	void HWSceneLayout2D::SaveLayoutIntoTxtFromDir(const std::string& dir)
	{
		layout_basename_ = GetBaseNameWithoutSuffix(layout_path_);
		std::string mydir = EnsureTrailingSlashHW(dir);
		std::string path = mydir + layout_basename_ + ".txt";
		std::ofstream fh(path);
		std::cerr << "path: " << path << std::endl;
		const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > all_pair_lines 
			= GetAllPairsLinesPntsFilter();
		if (fh.is_open())
		{
			int lines_pairs_num = static_cast<int>(all_pair_lines.size());
			std::cerr << "lines_pairs_num: " << lines_pairs_num << std::endl;
			fh << lines_pairs_num << std::endl;
			for (int i = 0; i < all_pair_lines.size(); ++i)
			{
				fh << all_pair_lines[i].first[0] << " " << all_pair_lines[i].first[1] <<
					" " << all_pair_lines[i].second[0] << " " << all_pair_lines[i].second[1] << std::endl;
			}
			fh.close();
		}
		
	}

	void HWSceneLayout2D::Clear()
	{
		std::cerr << "start to clear data..." << std::endl;
		layout_path_.clear();
		layout_basename_.clear();;
		lines_to_near_idx_.clear();
		pnts_.clear();
		lines_segs_.clear();
		pnts_origin_.clear();
		lines_segs_origin_.clear();
		img_lines_pnts_.clear();
		std::cerr << "end clear data..." << std::endl;
	}

	void HWSceneLayout2D::ClearLinesSegments()
	{
		pnts_.clear();
		lines_segs_.clear();
		pnts_origin_.clear();
		lines_segs_origin_.clear();
		img_lines_pnts_.clear();
	}

	void HWSceneLayout2D::PrintLinesGroupsIdxsInfo(const std::vector<std::vector<int> >& groups)
	{
		for (int i = 0; i < groups.size(); ++i)
		{
			for (int j = 0; j < groups[i].size(); ++j)
			{
				printf("%d ", groups[i][j]);
			}
			printf("\n");
		}
	}

	float HWSceneLayout2D::ComputeTwoLine2dAngle(const Eigen::Vector2f& sc_ls, const Eigen::Vector2f& sc_le,
		const Eigen::Vector2f& tg_ls, const Eigen::Vector2f& tg_le)
	{
		//two dir
		Eigen::Vector2f l0d = sc_le - sc_ls;
		Eigen::Vector2f l1d = tg_le - tg_ls;
		l0d.normalize();
		l1d.normalize();
		float l0l1_deg = ComputeAngleFromTwoLinesVector2D(l0d, l1d);
		return l0l1_deg;
	}

	float HWSceneLayout2D::ComputeTwoLine2dCrossAngle(const Eigen::Vector2f& sc_ls, const Eigen::Vector2f& sc_le,
		const Eigen::Vector2f& tg_ls, const Eigen::Vector2f& tg_le)
	{
		//two dir
		Eigen::Vector2f l0d = sc_le - sc_ls;
		Eigen::Vector2f l1d = tg_le - tg_ls;
		l0d.normalize();
		l1d.normalize();
		float l0l1_deg = ComputeAngleFromTwoLinesVector2D(l0d, l1d);
		if (l0l1_deg > 90.0)
			l0l1_deg = 180.0 - l0l1_deg;
		return l0l1_deg;
	}

	float HWSceneLayout2D::ComputeTwoLine2dDist(const Eigen::Vector2f& sc_ls, const Eigen::Vector2f& sc_le,
		const Eigen::Vector2f& tg_ls, const Eigen::Vector2f& tg_le)
	{
		//two dir
		Eigen::Vector2f scd = sc_le - sc_ls;
		Eigen::Vector2f tgd = tg_le - tg_ls;
		scd.normalize();
		tgd.normalize();
		float scls_to_tgtline = ComputePnt2DToLine2DDistF(tg_ls, tgd, sc_ls);
		float scle_to_tgtline = ComputePnt2DToLine2DDistF(tg_ls, tgd, sc_le);
		float tgls_to_scrline = ComputePnt2DToLine2DDistF(sc_ls, scd, tg_ls);
		float tgle_to_scrline = ComputePnt2DToLine2DDistF(sc_ls, scd, tg_le);
		float line_to_line_dist = (scls_to_tgtline + scle_to_tgtline + tgls_to_scrline + tgle_to_scrline) / 4.0;
		return line_to_line_dist;
	}

	void HWSceneLayout2D::ComputePnt2dProj2Line2DF(const Eigen::Vector2f& lpnt, const Eigen::Vector2f& ldir, const Eigen::Vector2f& pnt, Eigen::Vector2f& proj)
	{
		float t = (pnt.dot(ldir) - lpnt.dot(ldir)) / ldir.squaredNorm();
		//std::cerr <<"t: " << t <<std::endl;
		proj = t * ldir + lpnt;
	}

	float HWSceneLayout2D::ComputePnt2DToLine2DDistF(const Eigen::Vector2f& lpnt, const Eigen::Vector2f& ldir, const Eigen::Vector2f& pnt)
	{
		Eigen::Vector2f proj;
		ComputePnt2dProj2Line2DF(lpnt, ldir, pnt, proj);
		//std::cerr << "proj: "<< proj.transpose() <<std::endl;
		float d = (proj - pnt).norm();
		return d;
	}

	void HWSceneLayout2D::SetSceneLayout2D(std::vector<Eigen::Vector2f>& pnts)
	{
		std::cerr << "to do next..." << std::endl;
		std::vector<std::vector<float> > layouts_2d;
		for (std::size_t i = 0; i < pnts.size() - 2; i = i + 2)
		{
			std::vector<float> tmp_2d;
			tmp_2d.emplace_back(pnts[i][0]);
			tmp_2d.emplace_back(pnts[i][1]);
			tmp_2d.emplace_back(pnts[i + 1][0]);
			tmp_2d.emplace_back(pnts[i + 1][1]);
			layouts_2d.emplace_back(tmp_2d);
		}
		SetSceneLout2DFromNetPnts(layouts_2d);
	}

	void HWSceneLayout2D::SetSceneLayoutFromLines2D(std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& lines)
	{
		//set from std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >
		std::vector<std::vector<float> > layouts_2d;
		for (std::size_t i = 0; i < lines.size(); ++i)
		{
			std::vector<float> tmp_2d;
			std::pair<Eigen::Vector2f, Eigen::Vector2f> line_pnt = lines[i];
			tmp_2d.emplace_back(line_pnt.first[0]);
			tmp_2d.emplace_back(line_pnt.first[1]);
			tmp_2d.emplace_back(line_pnt.second[0]);
			tmp_2d.emplace_back(line_pnt.second[1]);
			layouts_2d.emplace_back(tmp_2d);
		}
		//std::cerr << "lines layout 2d: " << layouts_2d.size() << std::endl;
		SetSceneLout2DFromNetPnts(layouts_2d);
		std::cerr << "pnts_: " << pnts_.size() << std::endl;
		std::cerr << "lines_segs_: " << lines_segs_.size() << std::endl;
	}

	void HWSceneLayout2D::SetSceneLayoutFromLines2DConst(const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& lines)
	{
		//set from std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >
		std::vector<std::vector<float> > layouts_2d;
		for (std::size_t i = 0; i < lines.size(); ++i)
		{
			std::vector<float> tmp_2d;
			std::pair<Eigen::Vector2f, Eigen::Vector2f> line_pnt = lines[i];
			tmp_2d.emplace_back(line_pnt.first[0]);
			tmp_2d.emplace_back(line_pnt.first[1]);
			tmp_2d.emplace_back(line_pnt.second[0]);
			tmp_2d.emplace_back(line_pnt.second[1]);
			layouts_2d.emplace_back(tmp_2d);
		}
		//std::cerr << "lines layout 2d: " << layouts_2d.size() << std::endl;
		SetSceneLout2DFromNetPnts(layouts_2d);
	}

    void HWSceneLayout2D::SetSceneLout2DFromNetPnts(std::vector<std::vector<float> >& layout_pnts)
    {
		ClearLinesSegments();
        std::vector<Eigen::Vector2f> laypnts;
        std::vector<Eigen::Vector2i> lines_idx;
        for(int i = 0; i < layout_pnts.size(); ++i)
        {
            std::vector<float> line_pnts = layout_pnts[i];
            Eigen::Vector2f spnt = Eigen::Vector2f(line_pnts[0], line_pnts[1]);
            Eigen::Vector2f epnt = Eigen::Vector2f(line_pnts[2], line_pnts[3]);
            laypnts.emplace_back(spnt);
            laypnts.emplace_back(epnt);
            Eigen::Vector2i lidx = Eigen::Vector2i(2*i, 2*i+1);
            lines_idx.emplace_back(lidx);
        }
        pnts_.clear();
        lines_segs_.clear();
        for(int i = 0; i < laypnts.size(); ++i)
        {
            HWLinePoint2D p;
            p.pnt_[0] = laypnts[i][0];
            p.pnt_[1] = laypnts[i][1];
            p.pnt_[2] = 0.0;
            pnts_.emplace_back(p);
        }
        lines_segs_ = lines_idx;
        type_ = HWSceneLayoutDimType::kScene2DType;
		int lines_segs_num = static_cast<int>(lines_segs_.size());
		lines_to_near_idx_.resize(lines_segs_num, -1);
    }

	void HWSceneLayout2D::SetSceneLayoutFromOpencvPnts(const std::vector<std::vector<float> >& layout_pnts)
	{
		ClearLinesSegments();
		std::vector<Eigen::Vector2f> laypnts;
		std::vector<Eigen::Vector2i> lines_idx;
		for (int i = 0; i < layout_pnts.size(); ++i)
		{
			std::vector<float> line_pnts = layout_pnts[i];
			Eigen::Vector2f spnt = Eigen::Vector2f(line_pnts[0], line_pnts[1]);
			Eigen::Vector2f epnt = Eigen::Vector2f(line_pnts[2], line_pnts[3]);
			laypnts.emplace_back(spnt);
			laypnts.emplace_back(epnt);
			Eigen::Vector2i lidx = Eigen::Vector2i(2 * i, 2 * i + 1);
			lines_idx.emplace_back(lidx);
		}
		//pnts_.clear();
		//lines_segs_.clear();
		for (int i = 0; i < laypnts.size(); ++i)
		{
			HWLinePoint2D p;
			p.pnt_[0] = laypnts[i][0];
			p.pnt_[1] = laypnts[i][1];
			p.pnt_[2] = 0.0;
			pnts_.emplace_back(p);
		}
		lines_segs_ = lines_idx;
		type_ = HWSceneLayoutDimType::kScene2DType;
		int lines_segs_num = static_cast<int>(lines_segs_.size());
		lines_to_near_idx_.resize(lines_segs_num, -1);
		loaded_layout_state_ = true;
	}

    void HWSceneLayout2D::SetLayoutId(unsigned int id)
    {
        layout_id_ = id;
    }

    void HWSceneLayout2D::SetImageId(unsigned int id)
    {
        image_id_ = id;
    }

	void HWSceneLayout2D::SetCamId(unsigned int id)
	{
		cam_id_ = id;
	}

    void HWSceneLayout2D::SetImageIdx(int img_idx)
    {
        ly_cam_idx_ = img_idx;
    }

    bool HWSceneLayout2D::SetLinePnts(int line_idx, Eigen::Vector2f& s, Eigen::Vector2f& e)
    {
        if(line_idx < 0 && line_idx >= lines_segs_.size())
        {   
            std::cerr <<"set line pnts: idx out of range..." << std::endl;
            return false;
        }
        Eigen::Vector2i line_ref = lines_segs_[line_idx];
        if(line_ref[0] < 0  || 
            line_ref[0] >= pnts_.size() ||
            line_ref[1] < 0  || 
            line_ref[1] >= pnts_.size())
        {
            std::cerr <<"set pnts: idx out of range..." << std::endl;
            std::cerr <<"need to update: to do next..." << std::endl;
            return false;
        }
        pnts_[line_ref[0]].pnt_ = Eigen::Vector3f(s[0], s[1], 0.0);
        pnts_[line_ref[1]].pnt_ = Eigen::Vector3f(e[0], e[1], 0.0);
        return true;
    }

	void HWSceneLayout2D::SetLayoutPath(const std::string& ly_path)
	{
		layout_path_ = ly_path;
	}

	bool HWSceneLayout2D::IsSamePnt(const Eigen::Vector2f& s, const Eigen::Vector2f& e)
	{
		if (std::abs(e[0] - s[0]) < KMIN_FLOAT_THRESHOLD
			&& std::abs(e[1] - s[1]) < KMIN_FLOAT_THRESHOLD)
		{
			return true;
		}
		return false;
	}

    void HWSceneLayout2D::UpdateLayout2D()
    {
        //std::cerr << "to do next..." << std::endl;
		std::vector<Eigen::Vector2i> lines_idxs;
		std::vector<Eigen::Vector2f> pnts;
		for (int i = 0; i < lines_segs_.size(); ++i)
		{
			Eigen::Vector2f pnt_s;
			pnt_s[0] = pnts_[lines_segs_[i][0]].pnt_[0];
			pnt_s[1] = pnts_[lines_segs_[i][0]].pnt_[1];

			Eigen::Vector2f pnt_e;
			pnt_e[0] = pnts_[lines_segs_[i][1]].pnt_[0];
			pnt_e[1] = pnts_[lines_segs_[i][1]].pnt_[1];
			
			int line_s_idx = -1;
			int line_e_idx = -1;
			for (int j = 0; j < pnts.size(); ++j)
			{
				if (IsSamePnt(pnt_s, pnts[j]))
				{
					line_s_idx = j;
				}
				if (IsSamePnt(pnt_e, pnts[j]))
				{
					line_e_idx = j;
				}
			}
			if (line_s_idx == -1 && line_e_idx == -1)
			{
				line_s_idx = static_cast<int>(pnts.size());
				pnts.emplace_back(pnt_s);
				line_e_idx = line_s_idx + 1;
				pnts.emplace_back(pnt_e);
				Eigen::Vector2i tmp_idx = Eigen::Vector2i(line_s_idx, line_e_idx);
				lines_idxs.emplace_back(tmp_idx);
			}
			else if (line_s_idx != -1 && line_e_idx == -1)
			{
				line_e_idx = static_cast<int>(pnts.size());
				pnts.emplace_back(pnt_e);
				Eigen::Vector2i tmp_idx = Eigen::Vector2i(line_s_idx, line_e_idx);
				lines_idxs.emplace_back(tmp_idx);
			}
			else if (line_s_idx == -1 && line_e_idx != -1)
			{
				line_s_idx = static_cast<int>(pnts.size());
				pnts.emplace_back(pnt_s);
				Eigen::Vector2i tmp_idx = Eigen::Vector2i(line_s_idx, line_e_idx);
				lines_idxs.emplace_back(tmp_idx);
			}
			else
			{
			}
		}

		//update to pnts_ and line_segs_;
		/*pnts_.clear();
		lines_segs_.clear();*/

		int pnts_num = static_cast<int>(pnts.size());
		pnts_.resize(pnts_num);
		for (int i = 0; i < pnts.size(); ++i)
		{
			Eigen::Vector3f tmp_pnt = Eigen::Vector3f(pnts[i][0], pnts[i][1], 0.0);
			pnts_[i].pnt_ = tmp_pnt;
		}
		int lines_num = static_cast<int>(lines_idxs.size());
		lines_segs_.resize(lines_num);
		for (int i = 0; i < lines_idxs.size(); ++i)
		{
			lines_segs_[i] = lines_idxs[i];
		}
    }

    int HWSceneLayout2D::GetLayoutLinesNum()
    {
        return static_cast<int>(lines_segs_.size());
    }

    const std::string& HWSceneLayout2D::GetLayout2DBaseName()
    {
        return layout_basename_;
    }

    const std::string& HWSceneLayout2D::GetLayout2DPath()
    {
        return layout_path_;
    }

    const std::vector<HWLinePoint2D>& HWSceneLayout2D::GetLinesPnts()
    {
        return pnts_;
    }

    const std::vector<Eigen::Vector2i>& HWSceneLayout2D::GetLinesIdxs()
    {
        return lines_segs_;
    }

    const std::pair<HWLinePoint2D, HWLinePoint2D>& HWSceneLayout2D::GetPickedLine(unsigned int lid)
    {   
        //std::cerr <<"azdsfadf" <<std::endl;
        //std::pair <HWLinePoint2D, HWLinePoint2D> tmp;
        //HWLinePoint2D t1;
        //HWLinePoint2D t2;// = HWLinePoint2D();
        //tmp.first = t1;
        //tmp.second = t2;
        //return tmp;

        if(lid >= lines_segs_.size() && lid < 0)
        {
            return std::make_pair(HWLinePoint2D(), HWLinePoint2D());
        }
        int sidx = lines_segs_[lid][0];
        int eidx = lines_segs_[lid][1];
        std::cerr << "sidx, eidx: " << sidx <<", " << eidx << std::endl;
        if(sidx < 0 || sidx >= pnts_.size() || eidx < 0 || eidx >= pnts_.size())
        {
            return std::make_pair(HWLinePoint2D(), HWLinePoint2D());
        }
        //return std::make_pair(HWLinePoint2D(), HWLinePoint2D());
        return std::make_pair(pnts_[sidx], pnts_[eidx]);
    }

    bool HWSceneLayout2D::GetPickedLineFromIdx(unsigned int lid, Eigen::Vector3f& s, Eigen::Vector3f& e)
    {
        if(lid >= lines_segs_.size() || lid < 0)
        {
            return false;
        }
        int sidx = lines_segs_[lid][0];
        int eidx = lines_segs_[lid][1];
        //std::cerr << "sidx, eidx: " << sidx <<", " << eidx << std::endl;
        s = pnts_[sidx].pnt_;
        e = pnts_[eidx].pnt_;
        return true;
    }

	bool HWSceneLayout2D::GetPickedLineFromIdx(unsigned int lid, Eigen::Vector3f& s, Eigen::Vector3f& e) const
	{
		if (lid >= lines_segs_.size() || lid < 0)
		{
			return false;
		}
		int sidx = lines_segs_[lid][0];
		int eidx = lines_segs_[lid][1];
		//std::cerr << "sidx, eidx: " << sidx <<", " << eidx << std::endl;
		s = pnts_[sidx].pnt_;
		e = pnts_[eidx].pnt_;
		return true;
	}


    const unsigned int HWSceneLayout2D::GetLayoutId()
    {
        return layout_id_;
    }

	const unsigned int HWSceneLayout2D::GetLayoutId() const
	{
		return layout_id_;
	}

    const unsigned int HWSceneLayout2D::GetImageId()
    {
        return image_id_;
    }

	const unsigned int HWSceneLayout2D::GetImageId() const
	{
		return image_id_;
	}

	const unsigned int HWSceneLayout2D::GetCamId()
	{
		return cam_id_;
	}

	const unsigned int HWSceneLayout2D::GetCamId() const
	{
		return cam_id_;
	}

    void HWSceneLayout2D::GetAllPairLinesPnts(std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& all_pnts)
    {
        //
        for(std::size_t i = 0; i < lines_segs_.size(); ++i)
        {
            int sidx = lines_segs_[i][0];
            int eidx = lines_segs_[i][1];
			//std::cerr << "sidx, eidx: " << sidx << ", " << eidx << std::endl;
            HW::HWLinePoint2D spnt = pnts_[sidx];
            HW::HWLinePoint2D epnt = pnts_[eidx];
            Eigen::Vector2f sp2d = Eigen::Vector2f(spnt.pnt_[0], spnt.pnt_[1]);
            Eigen::Vector2f ep2d = Eigen::Vector2f(epnt.pnt_[0], epnt.pnt_[1]);
            all_pnts.emplace_back(std::make_pair(sp2d, ep2d));
        }
    }

	const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& HWSceneLayout2D::GetAllPairsLinesPnts() const
	{
		std::cerr << "pnts num: " << pnts_.size() << std::endl;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > all_pnts;
		for (std::size_t i = 0; i < lines_segs_.size(); ++i)
		{
			int sidx = lines_segs_[i][0];
			int eidx = lines_segs_[i][1];
			//std::cerr << "sidx, eidx: " << sidx << ", " << eidx << std::endl;
			HW::HWLinePoint2D spnt = pnts_[sidx];
			HW::HWLinePoint2D epnt = pnts_[eidx];
			Eigen::Vector2f sp2d = Eigen::Vector2f(spnt.pnt_[0], spnt.pnt_[1]);
			Eigen::Vector2f ep2d = Eigen::Vector2f(epnt.pnt_[0], epnt.pnt_[1]);
			all_pnts.emplace_back(std::make_pair(sp2d, ep2d));
		}
		//std::cerr << "end get the lines segments..." << std::endl;
		return all_pnts;
	}

	std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > HWSceneLayout2D::GetAllCurrentPairsLinesPnts()
	{
		std::cerr << "pnts num: " << pnts_.size() << std::endl;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > all_pnts;
		for (std::size_t i = 0; i < lines_segs_.size(); ++i)
		{
			int sidx = lines_segs_[i][0];
			int eidx = lines_segs_[i][1];
			//std::cerr << "sidx, eidx: " << sidx << ", " << eidx << std::endl;
			HW::HWLinePoint2D spnt = pnts_[sidx];
			HW::HWLinePoint2D epnt = pnts_[eidx];
			Eigen::Vector2f sp2d = Eigen::Vector2f(spnt.pnt_[0], spnt.pnt_[1]);
			Eigen::Vector2f ep2d = Eigen::Vector2f(epnt.pnt_[0], epnt.pnt_[1]);
			all_pnts.emplace_back(std::make_pair(sp2d, ep2d));
		}
		//std::cerr << "end get the lines segments..." << std::endl;
		return all_pnts;
	}

	const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& HWSceneLayout2D::GetAllPairsLinesPntsFilter() const
	{
		std::cerr << "pnts num: " << pnts_.size() << std::endl;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > all_pnts;
		for (std::size_t i = 0; i < lines_segs_.size(); ++i)
		{
			int sidx = lines_segs_[i][0];
			int eidx = lines_segs_[i][1];
			if (sidx == eidx)
			{
				continue;
			}
			//std::cerr << "sidx, eidx: " << sidx << ", " << eidx << std::endl;
			HW::HWLinePoint2D spnt = pnts_[sidx];
			HW::HWLinePoint2D epnt = pnts_[eidx];
			Eigen::Vector2f sp2d = Eigen::Vector2f(spnt.pnt_[0], spnt.pnt_[1]);
			Eigen::Vector2f ep2d = Eigen::Vector2f(epnt.pnt_[0], epnt.pnt_[1]);
			all_pnts.emplace_back(std::make_pair(sp2d, ep2d));
		}
		return all_pnts;
	}

	const std::vector <std::pair<int, int> >& HWSceneLayout2D::GetAllLines2dIdxsToAllGroupedLines2dIdxs()
	{
		return lines_idxs_to_gouped_lines_idxs_;
	}

	const std::vector <std::pair<int, int> >& HWSceneLayout2D::GetAllLines2dIdxsToAllGroupedLines2dIdxs() const
	{
		return lines_idxs_to_gouped_lines_idxs_;
	}

	const std::vector <std::pair<Eigen::Vector2f, Eigen::Vector2f> >& HWSceneLayout2D::GetAllGroupedLines2dPnts()
	{
		return grouped_lines_pnts_;
	}

	const std::vector <std::pair<Eigen::Vector2f, Eigen::Vector2f> >& HWSceneLayout2D::GetAllGroupedLines2dPnts() const
	{
		return grouped_lines_pnts_;
	}

	const std::vector<int>& HWSceneLayout2D::GetNeighborLinesIdxs() const
	{
		return lines_to_near_idx_;
	}

	int HWSceneLayout2D::GetLineIdxFromLinePos(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_pos)
	{
		for (int i = 0; i < lines_segs_.size(); ++i)
		{
			int ls_idx = lines_segs_[i][0];
			int le_idx = lines_segs_[i][1];
			Eigen::Vector2f ls_pos, le_pos;
			ls_pos[0] = pnts_[ls_idx].pnt_[0];
			ls_pos[1] = pnts_[ls_idx].pnt_[1];
			le_pos[0] = pnts_[le_idx].pnt_[0];
			le_pos[1] = pnts_[le_idx].pnt_[1];
			float ls2line = PntDist2LineSegment2D(ls_pos, line_pos.first, line_pos.second);
			float le2line = PntDist2LineSegment2D(le_pos, line_pos.first, line_pos.second);
			if (ls2line < pnt_to_line_segment_dist_threshold_
				&& le2line < pnt_to_line_segment_dist_threshold_)
			{
				return i;
			}
		}
		return -1;
	}

	int HWSceneLayout2D::GetLineIdxFromLinePosByProjectLinesThreshold(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_pos)
	{
		int lines_min_support = 2;
		std::vector<LinePnt2D> lines_segments;
		float lines_combined_angle_threshold = 5.0;
		float lines_combined_dist_threshold = 5.0;
		LinesHWDBSCAN* lines_clusters = new LinesHWDBSCAN(lines_min_support, lines_combined_dist_threshold,
			lines_combined_angle_threshold, lines_segments);
		LineDistMeasure dist_l2l_min;
		dist_l2l_min.angle_measure_ = 90.0;
		dist_l2l_min.dist_measure_ = 100000.0f;
		int dist_l2l_min_idx = -1;
		LinePnt2D src_pnt2d;
		src_pnt2d.line_ = line_pos;
		for (int i = 0; i < lines_segs_.size(); ++i)
		{
			int ls_idx = lines_segs_[i][0];
			int le_idx = lines_segs_[i][1];
			Eigen::Vector2f ls_pos, le_pos;
			ls_pos[0] = pnts_[ls_idx].pnt_[0];
			ls_pos[1] = pnts_[ls_idx].pnt_[1];
			le_pos[0] = pnts_[le_idx].pnt_[0];
			le_pos[1] = pnts_[le_idx].pnt_[1];
			std::pair<Eigen::Vector2f, Eigen::Vector2f> tmp_line_seg;
			tmp_line_seg.first = ls_pos;
			tmp_line_seg.second = le_pos;
			LinePnt2D tgt_pnt2d;
			tgt_pnt2d.line_ = tmp_line_seg;
			LineDistMeasure l2ldist = lines_clusters->CalculateDistance(src_pnt2d, tgt_pnt2d);
			//dist_l2l_min to do next...
			if (l2ldist.angle_measure_ < lines_combined_angle_threshold
				&& l2ldist.dist_measure_ < lines_combined_dist_threshold)
			{
				//get min distance
				if (l2ldist < dist_l2l_min)
				{
					dist_l2l_min = l2ldist;
					dist_l2l_min_idx = i;
				}
			}
		}
		delete lines_clusters;
		return dist_l2l_min_idx;
	}

	bool HWSceneLayout2D::GetLayoutLoadedState()
	{
		return loaded_layout_state_;
	}

	void HWSceneLayout2D::UpdateRedauntLineSegments()
	{
		std::cerr << "start to filter the lines" << std::endl;
		for (std::vector<Eigen::Vector2i>::iterator iter = this->lines_segs_.begin();
			iter != this->lines_segs_.end();)
		{
			if ((*iter)[0] == (*iter)[1])
				iter = this->lines_segs_.erase(iter);
			else
				iter++;
		}
		std::cerr << "end filter the lines" << std::endl;
	}

    HWSceneLayout2D HWSceneLayout2D::operator = (const HWSceneLayout2D& other)
    {
        //HWSceneLayout2D c;
        this->layout_id_ = other.layout_id_;
        this->image_id_ = other.image_id_;
        this->layout_path_ = other.layout_path_;
        this->layout_basename_ = other.layout_basename_;
        this->type_ = other.type_;
		this->lines_near_angle_threshold_ = other.lines_near_angle_threshold_;
		this->line_near_dist_threshold_ = other.line_near_dist_threshold_;
		this->line_length_dist_threshold_ = other.line_length_dist_threshold_;
		this->lines_combined_angle_threshold_ = other.lines_combined_angle_threshold_;
		this->lines_combined_dist_threshold_ = other.lines_combined_dist_threshold_;
		this->lines_to_near_idx_ = other.lines_to_near_idx_;
        this->lines_segs_ = other.lines_segs_;
        this->pnts_ = other.pnts_;
        this->ly_cam_idx_ = other.ly_cam_idx_;
        this->img_lines_pnts_ = other.img_lines_pnts_;
		this->lines_segs_origin_ = other.lines_segs_origin_;
		this->pnts_origin_ = other.pnts_origin_;
		this->pnt_to_line_segment_dist_threshold_ = other.pnt_to_line_segment_dist_threshold_;
		this->lines_idxs_to_gouped_lines_idxs_ = other.lines_idxs_to_gouped_lines_idxs_;
		this->grouped_lines_pnts_ = other.grouped_lines_pnts_;
		this->group_lines_angle_threshold_ = other.group_lines_angle_threshold_;
		this->group_lines_dist_threshold_ = other.group_lines_dist_threshold_;
		this->loaded_layout_state_ = other.loaded_layout_state_;
        return *this;
    }


}
