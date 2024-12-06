#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//Qt
#include <QtWidgets/QMainWindow>
#include <QFileDialog>
#include<assert.h>
//#include<io.h>

#include<fstream>
#include<iostream>

//Qt
#include<qmdisubwindow.h>
//#include <QMainWindow>
#include <QMdiArea.h>

//Local
//#include "ccEntityAction.h"
#include "CloudCompareDisplay/ccMainAppInterface.h"
#include "CloudCompareDisplay/ccPickingListener.h"
#include"CloudCompareDisplay/ccGLWidget.h"
#include"math_utils.h"
#include"CloudCompareDisplay/ccPointCloud.h"
//#include"tinyply.h"
#include"CloudCompareDisplay/ccHObject.h"

//CCLib
#include <CloudCompareDisplay/AutoSegmentationTools.h>

//libE57
#include "E57Foundation.h"
#include "E57Simple.h"

//local
#include "ui_mainwindow.h"
#include"hw_db_tree.h"
#include"hw_pcl_functor.h"
#include"hw_mls.h"
#include"hw_sor.h"
#include"hw_construction.h"
#include"hw_simp.h"
#include"hw_snap.h"
#include"hw_plane.h"
#include"hw_polygon.h"
#include"hw_showpolygon.h"
//#include"hw_plane_gl.h"
#include"hw_plane_view_gl.h"

#include"hw_scenes_cams.h"

//#include"Display/OpenGLWindow.h"
//#include "Display/MyTool.h"

class QAction;
class QMdiArea;

class QMdiSubWindow;
class QToolBar;
class QToolButton;

//class cc3DMouseManager;
//class ccCameraParamEditDlg;
//class ccClippingBoxTool;
//class ccComparisonDlg;
//class ccDBRoot;
class ccDrawableObject;
//class ccGamepadManager;
class ccGLWindow;
//class ccGraphicalSegmentationTool;
//class ccGraphicalTransformationTool;
class ccHObject;
//class ccOverlayDialog;
//class ccPluginUIManager;
//class ccPointListPickingDlg;
//class ccPointPairRegistrationDlg;
//class ccPointPropertiesDlg;
//class ccPrimitiveFactoryDlg;
//class ccRecentFiles;
//class ccSectionExtractionTool;
//class ccStdPluginInterface;
//class ccTracePolylineTool;


class MainWindow : public QMainWindow, public ccMainAppInterface, public ccPickingListener
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();
	void CreateConnection();
	void initial();

	//opengl
	//void CreateOpenglWindow();
	//void ShowOpenglObject(const std::string& select_obj);
	void EditUIShow();

	//! Returns the unique instance of this object
	static MainWindow* TheInstance();

	//! Static shortcut to MainWindow::getActiveGLWindow
	static ccGLWindow* GetActiveGLWindow();

	//! Returns a given GL sub-window (determined by its title)
	/** \param title window title
	**/
	static ccGLWindow* GetGLWindow(const QString& title);

	//! Returns all GL sub-windows
	/** \param[in,out] glWindows vector to store all sub-windows
	**/
	static void GetGLWindows(std::vector<ccGLWindow*>& glWindows);

	//! Static shortcut to MainWindow::refreshAll
	static void RefreshAllGLWindow(bool only2D = false);

	//! Static shortcut to MainWindow::updateUI
	static void UpdateUI();

	//! Deletes current main window instance
	static void DestroyInstance();

	//! Returns active GL sub-window (if any)
	ccGLWindow* getActiveGLWindow() override;

	inline  QMainWindow* getMainWindow() override { return this; }
	//inline  const ccHObject::Container& getSelectedEntities() const override { return m_selectedEntities; }
	void createGLWindow(ccGLWindow*& window, QWidget*& widget) const override;
	void destroyGLWindow(ccGLWindow*) const override;
	//! Returns a given views
	ccGLWindow* getGLWindow(int index) const;
	//! Returns the number of 3D views
	int getGLWindowCount() const;

	//! Inherited from ccPickingListener
	void onItemPicked(const PickedItem& pi) override;

	//把HWObject复制到ccObject
	void CopyHWObjToCCObj(const std::string& obj);

	//把HWPointcloud点云传输到ccPointCloud上
	void CopyHWPCToCCPC(HW::HWPointCloud* in_cloud);

	//把HWPlane点云传输到ccPointCloud上
	void CopyHWPlanePCToCCPC(HW::HWPlane* in_cloud);
	
	//把Polygon的对象传输到ccobject上，用于显示
	void CopyHWPolygonToCCPolylines(HW::HWPolygon* in_polygon);

	//把ccPointCloud点云复制到HWPointCloud上
	void CopyCCPCToHWPC(ccPointCloud* pc);
	void CopyCCPCToHWPC(ccPointCloud* pc,std::vector<int>& planes_idx, std::vector<int>& planes_iswide);
	//把HWMesh网格复制到ccMesh网格
	void CopyHWMeshToCCMesh(HW::HWMesh* in_mesh);

	//把ccMesh网格复制到HWMesh网格
	void CopyCCMeshToHWMesh(ccMesh* cc_mesh);

	//把ccPointCloud显示出来
	void AddCCPCViewsList(const ccPointCloud* pc);

	//把ccObject对象
	void AddCCObjectToWeidgetList(ccObject* obj);

	void ShowCurrentObject(const std::string& select_obj);

	//show all selected object
	void ShowAllActivedObjects();

	//! Segment a point cloud by propagating fronts constrained by values of the point cloud associated scalar field
	/** The algorithm is described in Daniel Girardeau-Montaut's PhD manuscript
	(Chapter 3, section 3.3). It consists mainly in propagating a front on
	the surface implicitly represented by the point cloud and making this
	propagation dependent on the scalar values associated to each point (such
	as the distance information computed between the point cloud and another
	entity). The propgation is realized with the Fast Marching Algorithm applied
	on a gridded structure (the octree in this case).
	Warning: be sure to activate an OUTPUT scalar field on the input cloud
	\param theCloud the point cloud to segment
	\param minSeedDist the minimum value associated to the point where to start the propagation from ("security" value)
	\param radius spherical neighborhood size (or 0 for automatic size)
	\param octreeLevel level of subdivision where to apply the gridding (the greater it is, the smaller and numerous the segmented parts will be)
	\param theSegmentedLists the segmented parts (as a list of subsets of points)
	\param applyGaussianFilter to specify if a gaussian filter should be applied after computing the scalar field gradient (to smooth the results)
	\param progressCb the client application can get some notification of the process progress through this callback mechanism (see GenericProgressCallback)
	\param inputOctree the cloud octree if it has already be computed
	\param alpha the gaussian filter kernel size (needed only if a gaussian filtering pass is required)
	\return success
	**/
	/*bool frontPropagationBasedSegmentation(CCLib::GenericIndexedCloudPersist* theCloud,
		PointCoordinateType radius,
		ScalarType minSeedDist,
		unsigned char octreeLevel,
		CCLib::ReferenceCloudContainer& theSegmentedLists,
		CCLib::GenericProgressCallback* progressCb,
		CCLib::DgmOctree* inputOctree,
		bool applyGaussianFilter,
		float alpha);*/
	/*
	\param theCloud the point cloud to select
	\param pi the point selected
	\param octreeLevel octree level
	*/
	bool propagationBasedSelectPoints(CCLib::GenericIndexedCloudPersist* theCloud, 
		unsigned char octreeLevel, 
		CCLib::DgmOctree* inputOctree,
		CCLib::GenericProgressCallback* progressCb,
		const PickedItem& pi);

	/*
	\param octreeLevel 
	\param inputOctree input octree
	\param volume the octree volume
	\param voxelPos the picked voxel position
	\param radius the search radius
	\param voxel_set propagated voxels
	\param travese_mask the search 
	*/
	void propagationBasedSelectedCellPos(
		unsigned char octreeLevel,
		CCLib::DgmOctree* inputOctree,
		VertexRearrangement* volume,
		const int3& voxelPos,
		float radius,
		std::vector<int3>& voxel_set,
		std::unordered_map<Eigen::Vector3i, bool, open3d::hash_eigen::hash<Eigen::Vector3i>>& travese_mask);

	//void propagationProcessBasedOnSelectedPoint(const CCVector3* pickedPoint, 
	//	CCLib::DgmOctree* inputOctree,
	//	PointCoordinateType radius,
	//	//CCLib::GenericIndexedCloudPersist* theCloud,
	//	std::vector<bool>& traverse_flag,
	//	unsigned char octreeLevel,
	//	CCLib::DgmOctree::NeighboursSet& temp_sets);
	//void propagationProcessBasedOnSelectedPointTest(const CCVector3* pickedPoint,
	//	CCLib::DgmOctree* inputOctree,
	//	PointCoordinateType radius,
	//	//CCLib::GenericIndexedCloudPersist* theCloud,
	//	std::vector<bool>& traverse_flag,
	//	unsigned char octreeLevel,
	//	CCLib::DgmOctree::NeighboursSet& temp_sets,
	//	int& recur_level);

	//test， 单纯用于测试，将点云保存出来的函数，后期需要删除掉、
	bool SaveHWPCIntoObjFile(const std::vector<float3> vertices,
		const std::vector<float3> normals, const std::string& path);

	bool SaveCCPCIntoObjFile(ccPointCloud* pc, const std::string& path);
	//end test

	void ConvertMatrixToELUEAngle(ccGLMatrixd cam_param);

	void ConvertCamModelDigram();

private:

	//read ply
	bool ReadPly(const std::string& file_path, bool is_binary = true);

    //read point cloud from .e57 file
    bool ReadE57File(const std::string& file_name);
	
	//read obj(polygon)
	bool ReadObj(const std::string& file_name);

	//read proj（ply+obj）
	bool ReadProjFile(const std::string& file_name);

	//通过名字索引ccHObject对象，如果成果返回对象，失败返回NULL
	ccHObject* GetObjectByStrName(const std::string& obj_name);

	//通过当前选中的对象，并且返回指针，如果没有对象，就返回空
	HW::HWObject* GetCurrentObject();
	
	//利用第一个点云对第二个点云进行去噪处理, 将离的较远的点云都去掉, 去掉的顶点索引放在idxs_rm
	void DenoiseHWPCFromHWPC(HW::HWObject* src, HW::HWObject* tgt, 
		float& pdist, std::vector<bool>& tgt_idxs_rm);
	
	void DenoiseCCPCFromCCPC(ccPointCloud* src, ccPointCloud* tgt,
		float& pdist, std::vector<bool>& tgt_idxs_rm);
	
	std::vector<std::string> proj_sub_names_;
	std::string proj_name_;	//这是工程文件名字

	bool first_show;
	std::vector<std::string> file_names_;
	std::vector<HW::HWObject*> origin_elements_;
	HW::HWDBTree* db_tree_;
	std::vector<HW::HWPCLFunctor*> processes_vec_;

	//mid area
	QMdiArea* mid_area_;

	//------opengl-------//
	QGridLayout *layout_;
	//OpenGLWindow* opengl_window_;
	//PerspectiveCamera* camera_;
	//用于设定点云的颜色
	std::vector<cv::Vec4b> colors_;
	//std::vector<cv::Vec4b>& GenerateColor(int num, cv::Vec4b color);
	
	//add list item
	void AddViewListAItem(const std::string& item_name);

	Ui::MainWindowClass ui;

	//db view list
	QMenu* pop_menu_;
	QAction* add_item_;
	QAction* delete_item_;
	QAction* modify_item_;
	std::vector<std::string> active_objects_names_;

	ccHObject* selected_ccobject;

	std::vector<ccPointCloud* > extracted_planes_objs_;

	//
	//std::map<std::string, std::vector<std::string> > scene_to_planes;

	std::vector<std::pair<std::string, std::vector<std::string> > > scene_to_planes_;

	//需要一个指针数组，管理ccPlane，用于后续的对象释放
	std::vector<ccGenericPrimitive*> root_planes_objs_;

	//-------------opengl-------------//

	void setView(CC_VIEW_ORIENTATION view) override;

	ccGLWindow* m_root_gl_;

	ccHObject* m_ccRoot1;

	//! Currently selected entities;
	ccHObject::Container m_selectedEntities;

	//! UI frozen state (see freezeUI)
	bool m_uiFrozen;

	//! View mode pop-up menu button
	QToolButton* m_viewModePopupButton;

	//! Pivot visibility pop-up menu button
	QToolButton* m_pivotVisibilityPopupButton;

	//! Flag: first time the window is made visible
	bool m_FirstShow;

	//! Point picking hub
	ccPickingHub* m_pickingHub;

	QMdiArea* m_mdiArea;

	ccPolyline* m_polyline_;
	ccHObject* m_polyobj_;

	HW::HWScenesCams* loaded_scenes_cams_;

	std::vector<float3> polygon_;
	std::vector<int> polygon_idx_;
	//-----------------------plane operation-----------------------//
	//ccGLWindow* m_plane_window_;

	HW::HWPlane* m_plane_;
	HW::HWPlaneViewGL* m_plane_window_;
	
	//这是对对象是平面操作，后续的所有操作都是在被选中的平面上上
	bool plane_state_;

	//---------------------end plane operation--------------------//
	//! CloudCompare MDI area overlay dialogs
	struct ccMDIDialogs
	{
		ccOverlayDialog* dialog;
		Qt::Corner position;

		//! Constructor with dialog and position
		ccMDIDialogs(ccOverlayDialog* dlg, Qt::Corner pos)
			: dialog(dlg)
			, position(pos)
		{}
	};

	//! Repositions an MDI dialog at its right position
	void repositionOverlayDialog(ccMDIDialogs& mdiDlg);

	//! Registered MDI area 'overlay' dialogs
	std::vector<ccMDIDialogs> m_mdiDialogs;

	//设置视角的参数
	void SetCameraParams(const ccGLMatrixd& cam_pos);
	void SetCameraPoseFovParams(const ccGLMatrixd& cam_pos, float fov_deg);
	void SetCameraPoseFovArParams(const ccGLMatrixd& cam_pos, float fov_deg, float ar);

	//平面的界面，一系列对平面的操作都在这个界面上完成
	ccGLWindow* newPlaneView(bool allowEntitySelection);

	//------------end opengl-----------------//

public slots:
	//-------------opengl------------//
	ccGLWindow* new3DView(bool allowEntitySelection);
	void redrawAll(bool only2D = false) override;
	void refreshAll(bool only2D = false) override;
	void enableAll() override;
	void disableAll() override;
	void disableAllBut(ccGLWindow* win) override;
	void toggleActiveWindowCenteredPerspective() override;
	//	void toggleActiveWindowCustomLight() override;
	//	void toggleActiveWindowSunLight() override;
	//	void toggleActiveWindowViewerBasedPerspective() override;
	void zoomOnSelectedEntities() override;
	void setGlobalZoom() override;

	void on3DViewActivated(QMdiSubWindow*);
	//	void updateUIWithSelection();
	//	void addToDBAuto(const QStringList& filenames);
	//
	void echoMouseWheelRotate(float);
	//	void echoCameraDisplaced(float ddx, float ddy);
	void echoBaseViewMatRotation(const ccGLMatrixd& rotMat);
	void echoCameraPosChanged(const CCVector3d&);
	void echoPivotPointChanged(const CCVector3d&);

	//--------menu------------//
	void LoadCamFileAction();
	void LoadHWFileSamplePntsTxtAction();
	void RenderCurCamColorImageAction();
	void RenderAllCamsColorImgsAction();
	void LoadFileAction();
	void SaveFileAction();
	void ClearFileAction();
	void ToDoSORFilterAction();
	void ToDoMLSAction();	//moving least square
	void ToDoConstructionAction();
	void ToDoSimplificationAction();
	
	void EditPlanePolygons();
	void SetPlaneOperationState();

	void DoPolygonSnapAction();
	void ShowPolygonAction();
	void MergeSelectedPointClouds();
	void MergeSelectedObjs();
	void ShowPntsInPolyViewAction();

	//-------------db tree view list----------//
	void ShowViewContextMenu(const QPoint& pos);
	void DeleteItemViewAndObject();
	void AddItemViewAndObject();

	//选择当然需要处理的对象
	void ChooseViewListItem();
	//设置需要被显示出来的对象
	void SetActiveObjectsItems(QListWidgetItem *item);

	//-----------------Parameters seting------------------//
	//SOR
	void SetSORPntNumAction();
	void SetSORMultiplierThresholdAction();
	void DoSORFilterAction();
	void CancelSORFilterAction();

	//MLS
	void SetMLSSearchRadiusAction();
	void CheckMLSComputeNormalFlagAction();
	void CheckMLSUsePolynomialFlagAction();
	void SetMLSPolygonOrderAction();
	void SetMLSSqrtGassionAction();
	void SetMLSUpsampleMethodsAction(int current_idx);
	void SetMLSUpsampleRadiusAction();
	void SetMLSUpsampleStepSizeAction();
	void SetMLSRandomDensityAction();

	void DoMLSAction();
	void CancelMLSAction();

	//Extract Plane
	void SetPlaneMinPntsNumAction();
	void SetCeilHeightAction();
	void SetPlaneMergeHeightAction();
	void SetNormalDeviationAction();
	void SetMinPlaneWidthAction();
	void SetPntsNumForMergeAction();
	void SetXYRatioForMergeAction();
	void SetAreaForMergeAction();

	void DoExtractPlaneAction();
	void CancelExtractPlaneAction();

	void RemoveTheObjFromSceneAction();

	//Construction
	void SetConstructionVoxelLengthAction();
	void SetContructionVolumeTruncAction();
	void SetConstructionVolumeX();
	void SetConstructionVolumeY();
	void SetConstructionVolumeZ();
	void DoConstructionAction();
	void CancelConstructionAction();
	/*void SetConstructionVolumeCenterX();
	void SetConstructionVolumeCenterY();
	void SetConstructionVolumeCenterZ();*/

	//Simplification
	void SetSimplifyTargetFaceNumAction();
	void SetSimplifyReductionRatioAction();
	void SetSimplifyQualityThresholdAction();
	void CheckSimplifyBundryPreserveAction();
	void SetSimplifyBundryWeightAction();
	void CheckSimplifyNormalPreserveAction();
	void CheckSimplifyToplogyPreserveAction();
	void CheckSimplifyOptimalAction();
	void CheckSimplifyPlanarSimplifyAction();
	void DoSimplifyAction();
	void CancelSimplifyAction();
	void DoEdgeSwapAction();
	//Texture Mapping
	void DoTextureMappingAction();
	void DoSamplingAction();
	void DoCompletePolygonAction();

	//opengl
	void SetSunLightAction();

	void SetShowWireAction();

	//Tool Action
	// Picking opeations
	void enablePickingOperation(ccGLWindow* win, QString message);
	void cancelPreviousPickingOperation(bool aborted);

	void ActivatePointPickingMode();
	void DeactivatePointPickingMode(bool);
	void doLevel();

	//tool action
	void DoConvertFilesIntoHWFilesAction();
	//----------------------end Parameters seting------------------//
};

#endif // MAINWINDOW_H
