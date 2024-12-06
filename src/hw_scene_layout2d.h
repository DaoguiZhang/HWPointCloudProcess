#ifndef HW_SCENE_LAYOUT2D_H
#define HW_SCENE_LAYOUT2D_H

#include"hw_cmns.h"
#include"hw_lines_cluster.h"

/*
layout2d
*/

namespace HW
{
    class HWSceneLayout2D
    {
    public:

        HWSceneLayout2D();
        HWSceneLayout2D(const HWSceneLayout2D& other);
        ~HWSceneLayout2D();
        
        //read line from log segments
        void ReadLayout2D(const std::string& path);
        bool ReadLayout2DFromNetPath(const std::string& path);
        void SetHWLayoutDimType(HWSceneLayoutDimType type);

        /*
        inference: get layout 2d
        input: p0p1, p1p2, p2p3,...
        */
        void SetSceneLayout2D(std::vector<Eigen::Vector2f>& pnts);
		void SetSceneLayoutFromLines2D(std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& lines);
		void SetSceneLayoutFromLines2DConst(const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& lines);
		void SetSceneLout2DFromNetPnts(std::vector<std::vector<float> >& layout_pnts);
		void SetSceneLayoutFromOpencvPnts(const std::vector<std::vector<float> >& layout_pnts);

        void SetLayoutId(unsigned int id);
        void SetImageId(unsigned int id);
		void SetCamId(unsigned int id);

        void SetImageIdx(int img_idx);
        bool SetLinePnts(int line_idx, Eigen::Vector2f& s, Eigen::Vector2f& e);
		void SetLayoutPath(const std::string& ly_path);

		bool IsSamePnt(const Eigen::Vector2f& s, const Eigen::Vector2f& e);

        //layout change into layout type
        void UpdateLayout2D();

        int GetLayoutLinesNum();
        const std::string& GetLayout2DBaseName();
        const std::string& GetLayout2DPath();
        //get lines pnts
        const std::vector<HWLinePoint2D>& GetLinesPnts();
        //get lines idxs
        const std::vector<Eigen::Vector2i>& GetLinesIdxs();
        //bug　do not use it. to do next....
        const std::pair<HWLinePoint2D, HWLinePoint2D>& GetPickedLine(unsigned int lid);
        bool GetPickedLineFromIdx(unsigned int lid, Eigen::Vector3f& s, Eigen::Vector3f& e);
		bool GetPickedLineFromIdx(unsigned int lid, Eigen::Vector3f& s, Eigen::Vector3f& e) const;
        const unsigned int GetLayoutId();
		const unsigned int GetLayoutId() const;
        const unsigned int GetImageId();
		const unsigned int GetImageId() const;
		const unsigned int GetCamId();
		const unsigned int GetCamId() const;
        void GetAllPairLinesPnts(std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& all_lines);
		const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& GetAllPairsLinesPnts() const;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > GetAllCurrentPairsLinesPnts();
		const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& GetAllPairsLinesPntsFilter() const;
		const std::vector <std::pair<int, int> >& GetAllLines2dIdxsToAllGroupedLines2dIdxs();
		const std::vector <std::pair<int, int> >& GetAllLines2dIdxsToAllGroupedLines2dIdxs() const;
		const std::vector <std::pair<Eigen::Vector2f, Eigen::Vector2f> >& GetAllGroupedLines2dPnts();
		const std::vector <std::pair<Eigen::Vector2f, Eigen::Vector2f> >& GetAllGroupedLines2dPnts() const;

		const std::vector<int>& GetNeighborLinesIdxs() const;
		int GetLineIdxFromLinePos(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_pos);
		int GetLineIdxFromLinePosByProjectLinesThreshold(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line_pos);
		bool GetLayoutLoadedState();

		void UpdateRedauntLineSegments();
		void UpdateNearNeighborLinesSegments();

		void GroupLinesSegmentsBasedOnLineDist(std::vector<std::vector<int> >& lines_groups);
		void GroupLinesSegmentsBasedOnLineDirAndDist();	//get lines group based one lines 
		bool IsTwoLinesSegsInSameLine(const Eigen::Vector2f& sc_ls, const Eigen::Vector2f& sc_le,
			const Eigen::Vector2f& tg_ls, const Eigen::Vector2f& tg_le);
		bool IsTwoLnesSegsParallelLine(const Eigen::Vector2f& sc_ls, const Eigen::Vector2f& sc_le,
			const Eigen::Vector2f& tg_ls, const Eigen::Vector2f& tg_le);
		std::vector<int> IdsValueFirstFromIdSecondValueFromPairsVector(const std::vector<std::pair<int, int> >& values_pair, int value_second);
		void RemovedNearNeighborLinesSegments();	//remove the lines which are near each other
		void RemoveLinesSegmentsWithLineLength();	//

		void StoreLinesPntsToLinesOriginPnts();

		void BackUpLineOriginPntsToLinePnts();

		void UpdateGroupLinesToLineSegsNew(const std::vector<std::vector<int> >& lines_groups);

		void UpdateLinesCombinationProcess();	//方向一致的线段合成一个线段

		void UpdateLinesCombinationProcessBasedOnLinesDir();	//方向一直，且它们在同一直线上

		void SaveLayoutIntoLog(const std::string& path);

		void SaveLayoutIntoTxtFromDir(const std::string& dir);

        HWSceneLayout2D operator = (const HWSceneLayout2D& other);

		void Clear();

		void ClearLinesSegments();

		void PrintLinesGroupsIdxsInfo(const std::vector<std::vector<int> >& groups);

    private:
		
        bool ReadLayout2DFromNetData(const std::string& path, 
            std::vector<std::vector<float>>& lines_pnts);
        
		float ComputeTwoLine2dAngle(const Eigen::Vector2f& sc_ls, const Eigen::Vector2f& sc_le, 
			const Eigen::Vector2f& tg_ls, const Eigen::Vector2f& tg_le);

		//0-90 degree
		float ComputeTwoLine2dCrossAngle(const Eigen::Vector2f& sc_ls, const Eigen::Vector2f& sc_le,
			const Eigen::Vector2f& tg_ls, const Eigen::Vector2f& tg_le);
		float ComputeTwoLine2dDist(const Eigen::Vector2f& sc_ls, const Eigen::Vector2f& sc_le,
			const Eigen::Vector2f& tg_ls, const Eigen::Vector2f& tg_le);
		
		void ComputePnt2dProj2Line2DF(const Eigen::Vector2f& lpnt, const Eigen::Vector2f& ldir, const Eigen::Vector2f& pnt, 
			Eigen::Vector2f& proj);

		float ComputePnt2DToLine2DDistF(const Eigen::Vector2f& lpnt, const Eigen::Vector2f& ldir, const Eigen::Vector2f& pnt);

        std::string layout_path_;
        std::string layout_basename_;

        unsigned int layout_id_;    //unique identifier ID
        unsigned int image_id_;     //point to image id (update)
		unsigned int cam_id_;	//point to cam id (update)

        int ly_cam_idx_ = -1;
        HWSceneLayoutDimType type_;

		float lines_near_angle_threshold_;
		float line_near_dist_threshold_;

		float line_length_dist_threshold_;

		float lines_combined_angle_threshold_;
		float lines_combined_dist_threshold_;

		std::vector<int> lines_to_near_idx_;

		float pnt_to_line_segment_dist_threshold_;

		//float line_combine_dist_threshold_;

        //points data
        std::vector<HWLinePoint2D> pnts_;
        //point index(start idx and end idx)
        std::vector<Eigen::Vector2i> lines_segs_;

		std::vector<HWLinePoint2D> pnts_origin_;
		std::vector<Eigen::Vector2i> lines_segs_origin_;

        std::vector<std::vector<Eigen::Vector2i> > img_lines_pnts_; //

		std::vector <std::pair<int, int>> lines_idxs_to_gouped_lines_idxs_;
		std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > grouped_lines_pnts_;
		float group_lines_angle_threshold_;
		float group_lines_dist_threshold_;

		bool loaded_layout_state_;
    };
}

#endif