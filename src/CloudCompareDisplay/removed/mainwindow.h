#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//#ifndef CC_MAIN_WINDOW_HEADER
//#define CC_MAIN_WINDOW_HEADER

//
#include<fstream>
#include<iostream>

//Qt
#include<qmdisubwindow.h>
#include <QMainWindow>
#include <QMdiArea.h>
#include <QFileDialog>

//Local
//#include "ccEntityAction.h"
#include "ccMainAppInterface.h"
#include "ccPickingListener.h"
#include"ccGLWidget.h"
#include"math_utils.h"
#include"ccPointCloud.h"
#include"tinyply.h"
#include"ccHObject.h"

//CCLib
#include <AutoSegmentationTools.h>

class QAction;
class QMdiArea;
class QMdiSubWindow;
class QToolBar;
class QToolButton;

class cc3DMouseManager;
class ccCameraParamEditDlg;
class ccClippingBoxTool;
class ccComparisonDlg;
class ccDBRoot;
class ccDrawableObject;
class ccGamepadManager;
class ccGLWindow;
class ccGraphicalSegmentationTool;
class ccGraphicalTransformationTool;
class ccHObject;
class ccOverlayDialog;
class ccPluginUIManager;
class ccPointListPickingDlg;
class ccPointPairRegistrationDlg;
class ccPointPropertiesDlg;
class ccPrimitiveFactoryDlg;
class ccRecentFiles;
class ccSectionExtractionTool;
class ccStdPluginInterface;
class ccTracePolylineTool;

//struct dbTreeSelectionInfo;

#include <QtWidgets/QMainWindow>
#include "ui_mainwindow.h"

class MainWindow : public QMainWindow, public ccMainAppInterface, public ccPickingListener
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();
public:

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

	//! Returns MDI area subwindow corresponding to a given 3D view
	QMdiSubWindow* getMDISubWindow(ccGLWindow* win);

	//! Returns a given views
	//ccGLWindow* getGLWindow(int index) const;

	//! Returns the number of 3D views
	//int getGLWindowCount() const;

	inline  QMainWindow* getMainWindow() override { return this; }
	//inline  const ccHObject::Container& getSelectedEntities() const override { return m_selectedEntities; }
	void createGLWindow(ccGLWindow*& window, QWidget*& widget) const override;
	void destroyGLWindow(ccGLWindow*) const override;

	//!read
	bool readPly(const std::string& file_path);

	void CreateConnection();

	void ShowObject();

private slots:
	void LoadFileAction();
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

private:

	void setView(CC_VIEW_ORIENTATION view) override;

	ccGLWindow* m_root_gl_;

	//DB & DB Tree
	ccDBRoot* m_ccRoot;

	//std::vector<ccHObject*> m_objects_;

	ccHObject* m_ccRoot1;
	//! Currently selected entities;
	ccHObject::Container m_selectedEntities;

	//! UI frozen state (see freezeUI)
	bool m_uiFrozen;

	//! Recent files menu
	ccRecentFiles* m_recentFiles;

	//! 3D mouse
	cc3DMouseManager* m_3DMouseManager;

	//! Gamepad handler
	ccGamepadManager* m_gamepadManager;

	//! View mode pop-up menu button
	QToolButton* m_viewModePopupButton;

	//! Pivot visibility pop-up menu button
	QToolButton* m_pivotVisibilityPopupButton;

	//! Flag: first time the window is made visible
	bool m_FirstShow;

	//! Point picking hub
	ccPickingHub* m_pickingHub;

	QMdiArea* m_mdiArea;

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

	/*** dialogs ***/
	//! Camera params dialog
	ccCameraParamEditDlg* m_cpeDlg;
	//! Graphical segmentation dialog
	ccGraphicalSegmentationTool* m_gsTool;
	//! Polyline tracing tool
	ccTracePolylineTool * m_tplTool;
	//! Section extraction dialog
	ccSectionExtractionTool* m_seTool;
	//! Graphical transformation dialog
	ccGraphicalTransformationTool* m_transTool;
	//! Clipping box dialog
	ccClippingBoxTool* m_clipTool;
	//! Cloud comparison dialog
	ccComparisonDlg* m_compDlg;
	//! Point properties mode dialog
	ccPointPropertiesDlg* m_ppDlg;
	//! Point list picking
	ccPointListPickingDlg* m_plpDlg;
	//! Point-pair registration
	ccPointPairRegistrationDlg* m_pprDlg;
	//! Primitive factory dialog
	ccPrimitiveFactoryDlg* m_pfDlg;

	/*** plugins ***/
	//! Manages plugins - menus, toolbars, and the about dialog
	ccPluginUIManager	*m_pluginUIManager;

	Ui::MainWindowClass ui;
};

#endif // MAINWINDOW_H
