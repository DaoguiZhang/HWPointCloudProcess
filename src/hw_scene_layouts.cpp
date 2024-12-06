#include"hw_scene_layouts.h"

namespace HW
{
    HWSceneLayouts::HWSceneLayouts()
    {

    }
    HWSceneLayouts::HWSceneLayouts(const std::string& dir)
    {
        scene_dir_ = dir; //set dir
    }

    HWSceneLayouts::~HWSceneLayouts()
    {
        scene_layout_3d_ = nullptr;
        scene_layouts_2d_.clear();
    }

    void HWSceneLayouts::SetSceneDir(const std::string& dir)
    {
        scene_dir_ = dir;   //set dir for loading data
    }

	void HWSceneLayouts::SetSceneOutDir(const std::string& outdir)
	{
		scene_out_dir_ = outdir;
	}

    void HWSceneLayouts::SetPickedLayout2DId(std::size_t i, unsigned int id)
    {
        if(i < 0 || i >= scene_layouts_2d_.size())
        {
            return;
        }
        scene_layouts_2d_.at(i)->SetLayoutId(id);

    }

    void HWSceneLayouts::SetPickedLayout2DImageId(std::size_t i, unsigned int imgid)
    {
        if(i < 0 || i >= scene_layouts_2d_.size())
        {
            return;
        }
        scene_layouts_2d_.at(i)->SetImageId(imgid);

    }
    
	void HWSceneLayouts::SetImageIdByLayoutId(unsigned int lyid, unsigned int imgid)
	{
		int lyidx = -1;
		for (int i = 0; i < scene_layouts_2d_.size(); ++i)
		{
			if (lyid == scene_layouts_2d_.at(i)->GetImageId())
			{
				lyidx = i;
				break;
			}
		}
		if (lyidx != -1)
		{
			scene_layouts_2d_.at(lyidx)->SetImageId(imgid);
		}
	}

	void HWSceneLayouts::SetCamIdByLayoutId(unsigned int lyid, unsigned int camid)
	{
		int lyidx = -1;
		for (int i = 0; i < scene_layouts_2d_.size(); ++i)
		{
			if (lyid == scene_layouts_2d_.at(i)->GetCamId())
			{
				lyidx = i;
				break;
			}
		}
		if (lyidx != -1)
		{
			scene_layouts_2d_.at(lyidx)->SetCamId(camid);
		}
	}

	void HWSceneLayouts::SetImageIdByLayoutIdx(unsigned int lyidx, unsigned int imgid)
	{
		if (lyidx < 0 || lyidx >= scene_layouts_2d_.size())
		{
			return;
		}
		scene_layouts_2d_.at(lyidx)->SetImageId(imgid);
	}

	void HWSceneLayouts::SetCamIdByLayoutIdx(unsigned int lyidx, unsigned int camid)
	{
		if (lyidx < 0 || lyidx >= scene_layouts_2d_.size())
		{
			return;
		}
		scene_layouts_2d_.at(lyidx)->SetCamId(camid);
	}

	void HWSceneLayouts::SetSceneLayoutPaths(const std::vector<std::string>& paths)
	{
		//layout_paths_.clear();
		layout_paths_.resize(paths.size());
		for (int i = 0; i < paths.size(); ++i)
		{
			layout_paths_[i] = paths[i];
		}
	}


    void HWSceneLayouts::LoadSceneElements()
    {
        std::cerr <<"HWSceneLayouts: to do next..." <<std::endl;
         std::vector<std::string> files_list;
        //std::vector<std::string> images_list;
        std::cerr <<"image_dir: " << scene_dir_ << std::endl;
        if(!scene_dir_.empty())
        {
            files_list = GetFilesListFromDir(scene_dir_);
			std::vector<std::string> lyout_list;
            std::cerr <<"files_list: " << files_list.size() << std::endl;
            for(std::size_t i = 0; i < files_list.size(); ++i)
            {
                if(files_list[i].find(".log") != std::string::npos ||
                    files_list[i].find(".LOG") != std::string::npos)
                    {
						std::string file_name = GetLeftSlashPathName(files_list[i]);
						lyout_list.emplace_back(file_name);
                    }
            }
			bool use_integer_flag = true;
			for (int i = 0; i < lyout_list.size(); ++i)
			{
				std::string file_basename = HW::GetBaseNameWithoutSuffix(lyout_list[i]);
				if (file_basename.empty())
				{
					continue;
				}
				if (!HW::IsStrFrontElementInteger(file_basename))
				{
					use_integer_flag = false;
					break;
				}
			}
			if (use_integer_flag)
			{
				std::vector<std::pair<int, std::string> > ly_files_idxs_str;
				for (int i = 0; i < lyout_list.size(); ++i)
				{
					//sort by file names
					std::string file_basename = HW::GetBaseNameWithoutSuffix(lyout_list[i]);
					int lyid = std::stoi(file_basename);
					std::pair<int, std::string> tmp_file_name;
					tmp_file_name.first = lyid;
					tmp_file_name.second = lyout_list[i];
					ly_files_idxs_str.emplace_back(tmp_file_name);
				}
				std::sort(ly_files_idxs_str.begin(), ly_files_idxs_str.end(), CompareHWStrPairByIdxLyoutCmp);
				for (int i = 0; i < ly_files_idxs_str.size(); ++i)
				{
					layout_paths_.emplace_back(ly_files_idxs_str[i].second);
				}
			}
			else
			{
				std::sort(lyout_list.begin(), lyout_list.end());
				for (int i = 0; i < lyout_list.size(); ++i)
				{
					layout_paths_.emplace_back(lyout_list[i]);
				}
			}
        }
        if(!layout_paths_.empty())
        {
            //layout paths
            for(std::size_t i = 0; i < layout_paths_.size(); ++i)
            {
				//std::cerr << "lyout path: " << layout_paths_[i] << std::endl;
               scene_layouts_2d_.emplace_back(new HWSceneLayout2D());
               scene_layouts_2d_[i]->SetLayoutId(i);
               //std::cerr <<"i: " << scene_layouts_2d_[i]->GetLayoutId() << std::endl;
               scene_layouts_2d_[i]->ReadLayout2D(layout_paths_[i]);
            }
        }
    }

	void HWSceneLayouts::AddSceneLayout2d(const HWSceneLayout2D& ly2d)
	{
		//add ly2d to scene
		//std::unique_ptr<HWSceneLayout2D> tmp_ly2d(new HWSceneLayout2D());
		//test the function... to do next...
		//*tmp_ly2d = ly2d;
		scene_layouts_2d_.emplace_back(new HWSceneLayout2D());
		*scene_layouts_2d_.back() = ly2d;
		//ly2d.GetImageId();
		//scene_layouts_2d_.back()->SetLayoutPath();
		std::cerr <<"AddSceneLayout2d: " << scene_layouts_2d_.back()->GetLayout2DPath() << std::endl;
	}

	int HWSceneLayouts::FindLayoutLineIdxFromLinePos(int lyoutid,
		const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_pos)
	{
		int find_idx = scene_layouts_2d_[lyoutid].get()->GetLineIdxFromLinePos(line_pos);
		return find_idx;
	}

	void HWSceneLayouts::UpdateFilterAllLayoutsSameLines()
	{
		for (int i = 0; i < scene_layouts_2d_.size(); ++i)
		{
			scene_layouts_2d_[i].get()->UpdateRedauntLineSegments();
		}
	}

	void HWSceneLayouts::UpdateAllLayoutsNearLines()
	{
		for (int i = 0; i < scene_layouts_2d_.size(); ++i)
		{
			scene_layouts_2d_[i].get()->UpdateNearNeighborLinesSegments();
		}
	}

	void HWSceneLayouts::UpdateAllLayoutsLinesCombinationProcess()
	{
		for (int i = 0; i < scene_layouts_2d_.size(); ++i)
		{
			scene_layouts_2d_[i].get()->UpdateLinesCombinationProcess();
		}
	}

	void HWSceneLayouts::UpdateAllLayoutsLineCombinationProcessBasedOnLinesDir()
	{
		for (int i = 0; i < scene_layouts_2d_.size(); ++i)
		{
			scene_layouts_2d_[i].get()->UpdateLinesCombinationProcessBasedOnLinesDir();
		}
	}

	void HWSceneLayouts::RemoveAllLayoutsNearLines()
	{
		for (int i = 0; i < scene_layouts_2d_.size(); ++i)
		{
			scene_layouts_2d_[i].get()->RemovedNearNeighborLinesSegments();
		}
	}

	void HWSceneLayouts::RemoveAllLayoutsShortLinesByLength()
	{
		for (int i = 0; i < scene_layouts_2d_.size(); ++i)
		{
			scene_layouts_2d_[i].get()->RemoveLinesSegmentsWithLineLength();
		}
	}

	void HWSceneLayouts::SetLayoutsFromImgsLinesSegsOpencv(const std::map<int, std::vector<std::vector<float> > >& lines)
	{
		std::map<int, std::vector<std::vector<float> > >::const_iterator iter = lines.begin();
		for (; iter != lines.end(); ++iter)
		{
			int layoutidx = iter->first;
			const std::vector<std::vector<float> > ly_lines = iter->second;
			if (layoutidx >= 0 && layoutidx < scene_layouts_2d_.size())
			{
				scene_layouts_2d_[layoutidx].get()->SetSceneLayoutFromOpencvPnts(ly_lines);
			}
		}
	}

	void HWSceneLayouts::SetLayoutLinesSegsFromLyId(int lyoutid,
		const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > line_pnts)
	{ 
		if (lyoutid >= 0 && lyoutid < scene_layouts_2d_.size())
		{
			scene_layouts_2d_[lyoutid].get()->SetSceneLayoutFromLines2DConst(line_pnts);
		}
	}

	bool HWSceneLayouts::GetLayoutsLoadedStateFromLyId(int lyoutid)
	{
		if (lyoutid >= 0 && lyoutid < scene_layouts_2d_.size())
		{
			return scene_layouts_2d_[lyoutid].get()->GetLayoutLoadedState();
		}
		return false;
	}

	void HWSceneLayouts::SaveAllLayout2dIntoLogs()
	{
		//
		for (int i = 0; i < scene_layouts_2d_.size(); ++i)
		{
			const std::string tmp_path = scene_layouts_2d_[i].get()->GetLayout2DPath();
			std::string tmp_base_path = GetBaseName(tmp_path);
			std::string tmp_outdir = EnsureTrailingSlashHW(scene_out_dir_);
			std::string outdir_path = tmp_outdir + tmp_base_path;
			//save the current layout into the log
			std::cerr << "line out_path: " << outdir_path << std::endl;
			scene_layouts_2d_[i].get()->UpdateRedauntLineSegments();
			scene_layouts_2d_[i].get()->UpdateNearNeighborLinesSegments();
			scene_layouts_2d_[i].get()->RemovedNearNeighborLinesSegments();
			scene_layouts_2d_[i].get()->UpdateLayout2D();
			scene_layouts_2d_[i].get()->SaveLayoutIntoLog(outdir_path);
		}
	}

	const std::vector<std::unique_ptr<HWSceneLayout2D> >& HWSceneLayouts::GetSceneLayouts2D()
	{
		return scene_layouts_2d_;
	}

    int HWSceneLayouts::GetScenelayout2DNum()
    {
        return static_cast<int>(scene_layouts_2d_.size());
    }

	const int HWSceneLayouts::GetScenelayout2DNum() const
	{
		return static_cast<int>(scene_layouts_2d_.size());
	}

    const HWSceneLayout2D& HWSceneLayouts::GetPickedLayout2D(std::size_t idx)
    {
        return *scene_layouts_2d_.at(idx);
    }

    const HWSceneLayout2D& HWSceneLayouts::GetLayout2DFromId(unsigned int id)
    {
        for(std::size_t i =0; i < scene_layouts_2d_.size(); ++i)
        {
            if(scene_layouts_2d_.at(i)->GetLayoutId() == id)
            {
                return *scene_layouts_2d_.at(i);
            }
        }
        return HWSceneLayout2D();
    }

	const HWSceneLayout2D& HWSceneLayouts::GetLayout2DFromId(unsigned int id) const
	{
		for (std::size_t i = 0; i < scene_layouts_2d_.size(); ++i)
		{
			if (scene_layouts_2d_.at(i)->GetLayoutId() == id)
			{
				return *scene_layouts_2d_.at(i);
			}
		}
		return HWSceneLayout2D();
	}
    
	void HWSceneLayouts::SaveAllImagesLayouts2dIntoTxtsFromSceneOutDir()
	{
		if (scene_out_dir_.empty())
		{
			return;
		}
		for (int i = 0; i < scene_layouts_2d_.size(); ++i)
		{
			scene_layouts_2d_[i].get()->SaveLayoutIntoTxtFromDir(scene_out_dir_);
		}
	}

	bool HWSceneLayouts::CompareHWStrPairByIdxLyoutCmp(const std::pair<int, std::string>& a, const std::pair<int, std::string>& b)
	{
		return a.first < b.first;
	}
}

