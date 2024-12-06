#ifndef HW_SCENE_LAYOUT_H
#define HW_SCENE_LAYOUTS_H
#include"hw_cmns.h"
#include"hw_algorithms.h"
#include"hw_scene_layout2d.h"
#include"hw_scene_layout3d.h"

namespace HW
{
    class HWSceneLayouts
    {
    public:
        HWSceneLayouts();
        HWSceneLayouts(const std::string& dir);
        ~HWSceneLayouts();
        void SetSceneDir(const std::string& dir);
		void SetSceneOutDir(const std::string& outdir);
        void SetPickedLayout2DId(std::size_t i, unsigned int id);
        void SetPickedLayout2DImageId(std::size_t i, unsigned int imgid);
		void SetImageIdByLayoutId(unsigned int lyid, unsigned int imgid);
		void SetCamIdByLayoutId(unsigned int lyid, unsigned int camid);
		void SetImageIdByLayoutIdx(unsigned int lyidx, unsigned int imgid);
		void SetCamIdByLayoutIdx(unsigned int lyidx, unsigned int camid);

		void SetSceneLayoutPaths(const std::vector<std::string>& paths);
        void LoadSceneElements();
		void AddSceneLayout2d(const HWSceneLayout2D& ly2d);
		int FindLayoutLineIdxFromLinePos(int lyoutid, 
			const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_pos);
		void UpdateFilterAllLayoutsSameLines();
		void UpdateAllLayoutsNearLines();
		void UpdateAllLayoutsLinesCombinationProcess();
		void UpdateAllLayoutsLineCombinationProcessBasedOnLinesDir();
		void RemoveAllLayoutsNearLines();
		void RemoveAllLayoutsShortLinesByLength();
		void SetLayoutsFromImgsLinesSegsOpencv(const std::map<int, std::vector<std::vector<float> > >& lines);
		void SetLayoutLinesSegsFromLyId(int lyoutid, 
			const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > line_pnts);
		bool GetLayoutsLoadedStateFromLyId(int lyoutid);
		//save all the layout
		void SaveAllLayout2dIntoLogs();
		const std::vector<std::unique_ptr<HWSceneLayout2D> >& GetSceneLayouts2D();

        int GetScenelayout2DNum();
		const int GetScenelayout2DNum() const;
        const HWSceneLayout2D& GetPickedLayout2D(std::size_t idx);
        const HWSceneLayout2D& GetLayout2DFromId(unsigned int id);
		const HWSceneLayout2D& GetLayout2DFromId(unsigned int id) const;

		void SaveAllImagesLayouts2dIntoTxtsFromSceneOutDir();

    private:

		static bool CompareHWStrPairByIdxLyoutCmp(const std::pair<int, std::string>& a, const std::pair<int, std::string>& b);

        std::string scene_dir_;
		std::string scene_out_dir_;
        std::vector<std::string> layout_paths_;

        //store the scenes
        std::unique_ptr<HWSceneLayout3D> scene_layout_3d_;
        std::vector<std::unique_ptr<HWSceneLayout2D> > scene_layouts_2d_;
    };
}

#endif