#include "mainwindow.h"

//CCLib Includes
#include <CloudSamplingTools.h>
#include <Delaunay2dMesh.h>
#include <Jacobi.h>
#include <MeshSamplingTools.h>
#include <NormalDistribution.h>
#include <ParallelSort.h>
#include <PointCloud.h>
#include <ScalarFieldTools.h>
#include <StatisticalTestingTools.h>
#include <WeibullDistribution.h>

//for tests
#include <ChamferDistanceTransform.h>
#include <SaitoSquaredDistanceTransform.h>

//qCC_db
#include <cc2DLabel.h>
#include <cc2DViewportObject.h>
#include <ccCameraSensor.h>
#include <ccColorScalesManager.h>
#include <ccFacet.h>
#include <ccFileUtils.h>
#include <ccGBLSensor.h>
#include <ccImage.h>
#include <ccKdTree.h>
#include <ccPlane.h>
#include <ccProgressDialog.h>
#include <ccQuadric.h>
#include <ccSphere.h>
#include <ccSubMesh.h>


//System
#include <iostream>
#include <random>

//global static pointer (as there should only be one instance of MainWindow!)
static MainWindow* s_instance = nullptr;

//default file filter separator
static const QString s_fileFilterSeparator(";;");

enum PickingOperation {
	NO_PICKING_OPERATION,
	PICKING_ROTATION_CENTER,
	PICKING_LEVEL_POINTS,
};
static ccGLWindow* s_pickingWindow = nullptr;
static PickingOperation s_currentPickingOperation = NO_PICKING_OPERATION;
static std::vector<cc2DLabel*> s_levelLabels;
static ccPointCloud* s_levelMarkersCloud = nullptr;
static ccHObject* s_levelEntity = nullptr;

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, m_ccRoot(nullptr)
	, m_uiFrozen(false)
	, m_recentFiles(NULL)
	, m_3DMouseManager(nullptr)
	, m_gamepadManager(nullptr)
	, m_viewModePopupButton(nullptr)
	, m_pivotVisibilityPopupButton(nullptr)
	, m_FirstShow(true)
	, m_pickingHub(nullptr)
	, m_cpeDlg(nullptr)
	, m_gsTool(nullptr)
	, m_tplTool(nullptr)
	, m_seTool(nullptr)
	, m_transTool(nullptr)
	, m_clipTool(nullptr)
	, m_compDlg(nullptr)
	, m_ppDlg(nullptr)
	, m_plpDlg(nullptr)
	, m_pprDlg(nullptr)
	, m_pfDlg(nullptr)
{
	ui.setupUi(this);
	this->resize(900, 600);
	CreateConnection();
	m_ccRoot1 = new ccHObject("root");
	//MDI Area
	{
		m_mdiArea = new QMdiArea(this);
		setCentralWidget(m_mdiArea);
		//connect(m_mdiArea, &QMdiArea::subWindowActivated, this, &MainWindow::updateMenus);
		connect(m_mdiArea, &QMdiArea::subWindowActivated, this, &MainWindow::on3DViewActivated);
		m_mdiArea->installEventFilter(this);
	}
	m_root_gl_ = new3DView(true);
}

MainWindow::~MainWindow()
{
	assert(m_ccRoot && m_mdiArea);
	//m_ccRoot->disconnect();
	//m_mdiArea->disconnect();

	//we don't want any other dialog/function to use the following structures
	ccDBRoot* ccRoot = m_ccRoot;
	m_ccRoot = nullptr;

	////remove all entities from 3D views before quitting to avoid any side-effect
	////(this won't be done automatically since we've just reset m_ccRoot)
	//ccRoot->getRootEntity()->setDisplay_recursive(nullptr);
	/*for (int i = 0; i < getGLWindowCount(); ++i)
	{
		getGLWindow(i)->setSceneDB(0);
	}*/
	m_cpeDlg = nullptr;
	m_gsTool = nullptr;
	m_seTool = nullptr;
	m_transTool = nullptr;
	m_clipTool = nullptr;
	m_compDlg = nullptr;
	m_ppDlg = nullptr;
	m_plpDlg = nullptr;
	m_pprDlg = nullptr;
	m_pfDlg = nullptr;

	//release all 'overlay' dialogs
	while (!m_mdiDialogs.empty())
	{
		ccMDIDialogs mdiDialog = m_mdiDialogs.back();
		m_mdiDialogs.pop_back();

		/*mdiDialog.dialog->disconnect();
		mdiDialog.dialog->stop(false);
		mdiDialog.dialog->setParent(0);*/
		delete mdiDialog.dialog;
	}
	//m_mdiDialogs.clear();
	//m_mdiArea->closeAllSubWindows();

	if (ccRoot)
	{
		delete ccRoot;
		ccRoot = nullptr;
	}
}

bool MainWindow::readPly(const std::string& file_path)
{
	std::ifstream fhd(file_path, std::ios::binary);
	if (fhd.fail()) throw std::runtime_error("failed to open " + file_path);
	tinyply::PlyFile file_ply;
	file_ply.parse_header(fhd);
	std::shared_ptr<tinyply::PlyData> point_vertices, point_normal;

	try
	{
		point_vertices = file_ply.request_properties_from_element("vertex", { "x", "y", "z" });
	}
	catch (const std::exception & e)
	{
		std::cout << "tinyply exception: " << e.what() << std::endl;
		return false;
	}

	try
	{
		point_normal = file_ply.request_properties_from_element("vertex", { "nx", "ny", "nz" });
	}
	catch (const std::exception & e)
	{
		std::cerr << "tinyply exception: " << e.what() << std::endl;
		return false;
	}
	//将主体读入到file_ply中
	file_ply.read(fhd);

	//read ply file to HWPointCloud
	if (point_vertices) std::cout << "\tRead " << point_vertices->count << " total vertices " << std::endl;
	if (point_normal) std::cout << "\tRead " << point_normal->count << " total vertex normals " << std::endl;

	std::vector<float3> tmp_vertices;
	std::vector<float3> tmp_normals;

	const size_t num_vertices_bytes = point_vertices->buffer.size_bytes();
	tmp_vertices.resize(point_vertices->count);
	std::memcpy(tmp_vertices.data(), point_vertices->buffer.get(), num_vertices_bytes);

	const size_t num_normal_bytes = point_normal->buffer.size_bytes();
	tmp_normals.resize(point_normal->count);
	std::memcpy(tmp_normals.data(), point_normal->buffer.get(), num_normal_bytes);
	//
	ccPointCloud* cloud = new ccPointCloud("unnamed - Cloud");
	//顶点传输
	for (int i = 0; i < tmp_vertices.size(); ++i)
	{
		cloud->addPoint(CCVector3(tmp_vertices[i].x, tmp_vertices[i].y, tmp_vertices[i].z));
	}
	for (int i = 0; i < tmp_normals.size(); ++i)
	{
		cloud->addPoint(CCVector3(tmp_normals[i].x, tmp_normals[i].y, tmp_normals[i].z));
	}
	ccHObject* show_obj = new ccHObject("show obj");
	//设置context
	show_obj->addChild(cloud);
	//look for object's parent
	ccHObject* parentObject = show_obj->getParent();
	if (parentObject)
	{
		parentObject = m_ccRoot1;
	}
	else
	{
		m_ccRoot1->addChild(show_obj);
	}
	//bool wasEmpty = (m_ccRoot1->getChildrenNumber() == 0);
	//m_ccRoot1->addChild(show_obj);
	//m_selectedEntities
	//m_ccRoot1 = show_obj;
	return true;
}

void MainWindow::CreateConnection()
{
	connect(this->ui.actionopen, SIGNAL(triggered()), this, SLOT(LoadFileAction()));
}

void MainWindow::LoadFileAction()
{
	//qDebug() << "test...\n";
	QString file_name_qstr = QFileDialog::getOpenFileName(this, tr("select a file"));
	if (file_name_qstr.isEmpty())
	{
		std::cout << "no file loading..." << std::endl;
		return;
	}
	std::string file_name = file_name_qstr.toStdString();
	readPly(file_name);

	ShowObject();

	return;
}

void MainWindow::ShowObject()
{
	//m_ccRoot1->setDisplay(m_root_gl_);
	m_ccRoot1->setDisplay_recursive(m_root_gl_);
}

//void MainWindow::doActionUnroll()
//{
//	//there should be only one point cloud with sensor in current selection!
//	if (!haveOneSelection())
//	{
//		ccConsole::Error("Select one and only one entity!");
//		return;
//	}
//
//	//if selected entity is a mesh, the method will be applied to its vertices
//	bool lockedVertices;
//	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(m_selectedEntities[0], &lockedVertices);
//	if (lockedVertices)
//	{
//		ccUtils::DisplayLockedVerticesWarning(m_selectedEntities[0]->getName(), true);
//		return;
//	}
//
//	//for "real" point clouds only
//	if (!cloud || !cloud->isA(CC_TYPES::POINT_CLOUD))
//	{
//		ccConsole::Error("Method can't be applied on locked vertices or virtual point clouds!");
//		return;
//	}
//	ccPointCloud* pc = static_cast<ccPointCloud*>(cloud);
//
//	ccUnrollDlg unrollDlg(this);
//	unrollDlg.fromPersistentSettings();
//	if (!unrollDlg.exec())
//		return;
//	unrollDlg.toPersistentSettings();
//
//	ccUnrollDlg::Type mode = unrollDlg.getType();
//	PointCoordinateType radius = static_cast<PointCoordinateType>(unrollDlg.getRadius());
//	unsigned char dim = static_cast<unsigned char>(unrollDlg.getAxisDimension());
//	bool exportDeviationSF = unrollDlg.exportDeviationSF();
//	CCVector3* pCenter = nullptr;
//	CCVector3 center;
//	if (mode != ccUnrollDlg::CYLINDER || !unrollDlg.isAxisPositionAuto())
//	{
//		//we need the axis center point
//		center = unrollDlg.getAxisPosition();
//		pCenter = &center;
//	}
//
//	//let's rock unroll ;)
//	ccProgressDialog pDlg(true, this);
//
//	ccPointCloud* output = nullptr;
//	switch (mode)
//	{
//	case ccUnrollDlg::CYLINDER:
//	{
//		double startAngle_deg = 0.0, stopAngle_deg = 360.0;
//		unrollDlg.getAngleRange(startAngle_deg, stopAngle_deg);
//		if (startAngle_deg >= stopAngle_deg)
//		{
//			QMessageBox::critical(this, "Error", "Invalid angular range");
//			return;
//		}
//		output = pc->unrollOnCylinder(radius, dim, pCenter, exportDeviationSF, startAngle_deg, stopAngle_deg, &pDlg);
//	}
//	break;
//
//	case ccUnrollDlg::CONE:
//	{
//		double coneHalfAngle_deg = unrollDlg.getConeHalfAngle();
//		output = pc->unrollOnCone(coneHalfAngle_deg, center, dim, false, 0, exportDeviationSF, &pDlg);
//	}
//	break;
//	
//	case ccUnrollDlg::STRAIGHTENED_CONE:
//	{
//		double coneHalfAngle_deg = unrollDlg.getConeHalfAngle();
//		output = pc->unrollOnCone(coneHalfAngle_deg, center, dim, true, radius, exportDeviationSF, &pDlg);
//	}
//	break;
//	
//	default:
//		assert(false);
//		break;
//	}
//
//	if (output)
//	{
//		pc->setEnabled(false);
//		ccConsole::Warning("[Unroll] Original cloud has been automatically hidden");
//
//		if (pc->getParent())
//		{
//			pc->getParent()->addChild(output);
//		}
//		addToDB(output, true, true, false, true);
//
//		updateUI();
//	}
//}

ccGLWindow* MainWindow::getActiveGLWindow()
{
	//m_ccRoot1->addChild
	if (!m_mdiArea)
	{
		return 0;
	}

	QMdiSubWindow *activeSubWindow = m_mdiArea->activeSubWindow();
	if (activeSubWindow)
	{
		return GLWindowFromWidget(activeSubWindow->widget());
	}
	else
	{
		QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
		if (!subWindowList.isEmpty())
		{
			return GLWindowFromWidget(subWindowList[0]->widget());
		}
	}

	return 0;
}

//QMdiSubWindow* MainWindow::getMDISubWindow(ccGLWindow* win)
//{
//	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
//	for (int i = 0; i < subWindowList.size(); ++i)
//	{
//		if (GLWindowFromWidget(subWindowList[i]->widget()) == win)
//			return subWindowList[i];
//	}
//
//	//not found!
//	return 0;
//}
//
//ccGLWindow* MainWindow::getGLWindow(int index) const
//{
//	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
//	if (index >= 0 && index < subWindowList.size())
//	{
//		ccGLWindow* win = GLWindowFromWidget(subWindowList[index]->widget());
//		assert(win);
//		return win;
//	}
//	else
//	{
//		assert(false);
//		return 0;
//	}
//}
//
//int MainWindow::getGLWindowCount() const
//{
//	return m_mdiArea ? m_mdiArea->subWindowList().size() : 0;
//}
//
//void MainWindow::zoomIn()
//{
//	ccGLWindow* win = MainWindow::getActiveGLWindow();
//	if (win)
//	{
//		//we simulate a real wheel event
//		win->onWheelEvent(15.0f);
//	}
//}
//
//void MainWindow::zoomOut()
//{
//	ccGLWindow* win = MainWindow::getActiveGLWindow();
//	if (win)
//	{
//		//we simulate a real wheel event
//		win->onWheelEvent(-15.0f);
//	}
//}

ccGLWindow* MainWindow::new3DView(bool allowEntitySelection)
{

	//assert(m_ccRoot && m_mdiArea);
	QWidget* viewWidget = nullptr;
	ccGLWindow* view3D = nullptr;

	createGLWindow(view3D, viewWidget);
	if (!viewWidget || !view3D)
	{
		ccLog::Error("Failed to create the 3D view");
		assert(false);
		return nullptr;
	}

	////restore options
	//{
	//	QSettings settings;
	//	bool autoPickRotationCenter = settings.value(ccPS::AutoPickRotationCenter(), true).toBool();
	//	view3D->setAutoPickPivotAtCenter(autoPickRotationCenter);
	//}

	view3D->setAutoPickPivotAtCenter(true);
	viewWidget->setMinimumSize(400, 300);

	m_mdiArea->addSubWindow(viewWidget);

	if (allowEntitySelection)
	{
		/*connect(view3D, &ccGLWindow::entitySelectionChanged, this, [=] (ccHObject *entity) {
		m_ccRoot->selectEntity( entity );
		});

		connect(view3D, &ccGLWindow::entitiesSelectionChanged, this, [=] (std::unordered_set<int> entities){
		m_ccRoot->selectEntities( entities );
		});*/
	}

	//'echo' mode
	connect(view3D, &ccGLWindow::mouseWheelRotated, this, &MainWindow::echoMouseWheelRotate);
	//connect(view3D,	&ccGLWindow::cameraDisplaced, this, &MainWindow::echoCameraDisplaced);
	connect(view3D, &ccGLWindow::viewMatRotated, this, &MainWindow::echoBaseViewMatRotation);
	connect(view3D, &ccGLWindow::cameraPosChanged, this, &MainWindow::echoCameraPosChanged);
	connect(view3D, &ccGLWindow::pivotPointChanged, this, &MainWindow::echoPivotPointChanged);
	//connect(view3D,	&ccGLWindow::pixelSizeChanged, this, &MainWindow::echoPixelSizeChanged);

	//connect(view3D,	&QObject::destroyed, this, &MainWindow::prepareWindowDeletion);
	//connect(view3D,	&ccGLWindow::filesDropped, this, &MainWindow::addToDBAuto, Qt::QueuedConnection); //DGM: we don't want to block the 'dropEvent' method of ccGLWindow instances!
	//connect(view3D,	&ccGLWindow::newLabel, this, &MainWindow::handleNewLabel);
	//connect(view3D,	&ccGLWindow::exclusiveFullScreenToggled, this, &MainWindow::onExclusiveFullScreenToggled);

	//if (m_pickingHub)
	//{
	//	//we must notify the picking hub as well if the window is destroyed
	//	connect(view3D, &QObject::destroyed, m_pickingHub, &ccPickingHub::onActiveWindowDeleted);
	//}

	//view3D->setSceneDB(m_ccRoot->getRootEntity());
	view3D->setSceneDB(m_ccRoot1);
	viewWidget->setAttribute(Qt::WA_DeleteOnClose);
	//m_ccRoot->updatePropertiesView();

	//QMainWindow::statusBar()->showMessage(QString("New 3D View"), 2000);

	viewWidget->showMaximized();
	viewWidget->update();

	return view3D;
}

//void MainWindow::prepareWindowDeletion(QObject* glWindow)
//{
//	if (!m_ccRoot)
//		return;
//
//	//we assume only ccGLWindow can be connected to this slot!
//	ccGLWindow* win = qobject_cast<ccGLWindow*>(glWindow);
//
//	m_ccRoot->hidePropertiesView();
//	m_ccRoot->getRootEntity()->removeFromDisplay_recursive(win);
//	m_ccRoot->updatePropertiesView();
//}
//
//static bool s_autoSaveGuiElementPos = true;
//void MainWindow::doActionResetGUIElementsPos()
//{
//	// show the user it will be maximized
//	showMaximized();
//
//	QSettings settings;
//	settings.remove(ccPS::MainWinGeom());
//	settings.remove(ccPS::MainWinState());
//
//	QMessageBox::information( this,
//							  tr("Restart"),
//							  tr("To finish the process, you'll have to close and restart CloudCompare") );
//	
//	//to avoid saving them right away!
//	s_autoSaveGuiElementPos = false;
//}
//
//void MainWindow::showEvent(QShowEvent* event)
//{
//	QMainWindow::showEvent( event );
//
//	if ( !m_FirstShow )
//	{
//		return;
//	}
//	
//	QSettings settings;
//	QVariant  geometry = settings.value(ccPS::MainWinGeom());
//	
//	if ( geometry.isValid() )
//	{
//		restoreGeometry(geometry.toByteArray());
//		restoreState(settings.value(ccPS::MainWinState()).toByteArray());
//	}
//	
//	m_FirstShow = false;
//	
//	if ( !geometry.isValid() )
//	{
//		showMaximized();
//	}
//	
//	if ( isFullScreen() )
//	{
//		m_UI->actionFullScreen->setChecked( true );
//	}
//	
//#ifdef Q_OS_MAC
//	if ( isFullScreen() )
//	{
//		m_UI->actionFullScreen->setText( tr( "Exit Full Screen" ) );
//	}
//	else
//	{
//		m_UI->actionFullScreen->setText( tr( "Enter Full Screen" ) );
//	}
//#endif
//}
//
//void MainWindow::closeEvent(QCloseEvent *event)
//{
//	// If we don't have anything displayed, then just close...
//	if (m_ccRoot && (m_ccRoot->getRootEntity()->getChildrenNumber() == 0))
//	{
//		event->accept();
//	}
//	else	// ...otherwise confirm
//	{
//		QMessageBox message_box( QMessageBox::Question,
//								 tr("Quit"),
//								 tr("Are you sure you want to quit?"),
//								 QMessageBox::Ok | QMessageBox::Cancel,
//								 this);
//		
//		if ( message_box.exec() == QMessageBox::Ok )
//		{
//			event->accept();
//		}
//		else
//		{
//			event->ignore();
//		}
//	}
//
//	if (s_autoSaveGuiElementPos)
//	{
//		saveGUIElementsPos();
//	}
//}
//
//void MainWindow::saveGUIElementsPos()
//{
//	//save the state as settings
//	QSettings settings;
//	settings.setValue(ccPS::MainWinGeom(), saveGeometry());
//	settings.setValue(ccPS::MainWinState(), saveState());
//}
//
//void MainWindow::moveEvent(QMoveEvent* event)
//{
//	QMainWindow::moveEvent(event);
//
//	updateOverlayDialogsPlacement();
//}
//
//void MainWindow::resizeEvent(QResizeEvent* event)
//{
//	QMainWindow::resizeEvent(event);
//
//	updateOverlayDialogsPlacement();
//}
//
//void MainWindow::registerOverlayDialog(ccOverlayDialog* dlg, Qt::Corner pos)
//{
//	//check for existence
//	for (ccMDIDialogs& mdi : m_mdiDialogs)
//	{
//		if (mdi.dialog == dlg)
//		{
//			//we only update its position in this case
//			mdi.position = pos;
//			repositionOverlayDialog(mdi);
//			return;
//		}
//	}
//
//	//otherwise we add it to DB
//	m_mdiDialogs.push_back(ccMDIDialogs(dlg, pos));
//
//	//automatically update the dialog placement when its shown
//	connect(dlg, &ccOverlayDialog::shown, this, [=]()
//	{
//		//check for existence
//		for (ccMDIDialogs& mdi : m_mdiDialogs)
//		{
//			if (mdi.dialog == dlg)
//			{
//				repositionOverlayDialog(mdi);
//				break;
//			}
//		}		
//	});
//
//	repositionOverlayDialog(m_mdiDialogs.back());
//}
//
//void MainWindow::unregisterOverlayDialog(ccOverlayDialog* dialog)
//{
//	for (std::vector<ccMDIDialogs>::iterator it = m_mdiDialogs.begin(); it != m_mdiDialogs.end(); ++it)
//	{
//		if (it->dialog == dialog)
//		{
//			m_mdiDialogs.erase(it);
//			break;
//		}
//	}
//}
//
//bool MainWindow::eventFilter(QObject *obj, QEvent *event)
//{
//	switch (event->type())
//	{
//	case QEvent::Resize:
//	case QEvent::Move:
//		updateOverlayDialogsPlacement();
//		break;
//	default:
//		//nothing to do
//		break;
//	}
//
//	// standard event processing
//	return QObject::eventFilter(obj, event);
//}
//
//void MainWindow::keyPressEvent(QKeyEvent *event)
//{
//	switch (event->key())
//	{
//		case Qt::Key_Escape:
//		{
//			if ( s_pickingWindow != nullptr )
//			{
//				cancelPreviousPickingOperation( true );
//			}
//			break;
//		}
//			
//		default:
//			QMainWindow::keyPressEvent(event);
//	}	
//}
//
//void MainWindow::updateOverlayDialogsPlacement()
//{
//	for (ccMDIDialogs& mdiDlg : m_mdiDialogs)
//	{
//		repositionOverlayDialog(mdiDlg);
//	}
//}
//
//void MainWindow::repositionOverlayDialog(ccMDIDialogs& mdiDlg)
//{
//	if (!mdiDlg.dialog || !mdiDlg.dialog->isVisible() || !m_mdiArea)
//		return;
//
//	int dx = 0;
//	int dy = 0;
//	static const int margin = 5;
//	switch (mdiDlg.position)
//	{
//	case Qt::TopLeftCorner:
//		dx = margin;
//		dy = margin;
//		break;
//	case Qt::TopRightCorner:
//		dx = std::max(margin, m_mdiArea->width() - mdiDlg.dialog->width() - margin);
//		dy = margin;
//		break;
//	case Qt::BottomLeftCorner:
//		dx = margin;
//		dy = std::max(margin, m_mdiArea->height() - mdiDlg.dialog->height() - margin);
//		break;
//	case Qt::BottomRightCorner:
//		dx = std::max(margin, m_mdiArea->width() - mdiDlg.dialog->width() - margin);
//		dy = std::max(margin, m_mdiArea->height() - mdiDlg.dialog->height() - margin);
//		break;
//	}
//
//	//show();
//	mdiDlg.dialog->move(m_mdiArea->mapToGlobal(QPoint(dx, dy)));
//	mdiDlg.dialog->raise();
//}
//
//void MainWindow::toggleVisualDebugTraces()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->toggleDebugTrace();
//		win->redraw(false, false);
//	}
//}
//
//void MainWindow::toggleFullScreen(bool state)
//{
//	if (state)
//		showFullScreen();
//	else
//		showNormal();
//
//#ifdef Q_OS_MAC
//	if ( state )
//	{
//		m_UI->actionFullScreen->setText( tr( "Exit Full Screen" ) );
//	}
//	else
//	{
//		m_UI->actionFullScreen->setText( tr( "Enter Full Screen" ) );
//	}
//#endif
//}
//
//void MainWindow::toggleExclusiveFullScreen(bool state)
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->toggleExclusiveFullScreen(state);
//	}
//}
//
//void MainWindow::doActionShowHelpDialog()
//{
//	QMessageBox::information( this,
//							  tr("Documentation"),
//							  tr("Please visit http://www.cloudcompare.org/doc") );
//}
//
//void MainWindow::freezeUI(bool state)
//{
//	//freeze standard plugins
//	m_UI->toolBarMainTools->setDisabled(state);
//	m_UI->toolBarSFTools->setDisabled(state);
//	
//	m_pluginUIManager->mainPluginToolbar()->setDisabled(state);
//
//	//freeze plugin toolbars
//	for ( QToolBar *toolbar : m_pluginUIManager->additionalPluginToolbars() )
//	{
//		toolbar->setDisabled(state);
//	}
//
//	m_UI->DockableDBTree->setDisabled(state);
//	m_UI->menubar->setDisabled(state);
//
//	if (state)
//	{
//		m_UI->menuEdit->setDisabled(true);
//		m_UI->menuTools->setDisabled(true);
//	}
//	else
//	{
//		updateMenus();
//	}
//
//	m_uiFrozen = state;
//}
//
//void MainWindow::activateRegisterPointPairTool()
//{
//	if (!haveSelection() || m_selectedEntities.size() > 2)
//	{
//		ccConsole::Error("Select one or two entities (point cloud or mesh)!");
//		return;
//	}
//
//	ccHObject* aligned = m_selectedEntities[0];
//	ccHObject* reference = m_selectedEntities.size() > 1 ? m_selectedEntities[1] : nullptr;
//
//	ccGenericPointCloud* cloud1 = ccHObjectCaster::ToGenericPointCloud(aligned);
//	ccGenericPointCloud* cloud2 = (reference ? ccHObjectCaster::ToGenericPointCloud(reference) : nullptr);
//	if (!cloud1 || (m_selectedEntities.size() > 1 && !cloud2))
//	{
//		ccConsole::Error("Select point clouds or meshes only!");
//		return;
//	}
//
//	//if we have 2 entities, we must ask the user which one is the 'aligned' one and which one is the 'reference' one
//	if (reference)
//	{
//		ccOrderChoiceDlg dlg(	m_selectedEntities[0], "Aligned",
//								m_selectedEntities[1], "Reference",
//								this );
//		if (!dlg.exec())
//			return;
//
//		aligned = dlg.getFirstEntity();
//		reference = dlg.getSecondEntity();
//	}
//
//	//we disable all windows
//	disableAllBut(0);
//
//	if (!m_pprDlg)
//	{
//		m_pprDlg = new ccPointPairRegistrationDlg(m_pickingHub, this, this);
//		connect(m_pprDlg, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateRegisterPointPairTool);
//		registerOverlayDialog(m_pprDlg, Qt::TopRightCorner);
//	}
//
//	ccGLWindow* win = new3DView(true);
//	if (!win)
//	{
//		ccLog::Error("[PointPairRegistration] Failed to create dedicated 3D view!");
//		return;
//	}
//
//	if (!m_pprDlg->init(win, aligned, reference))
//		deactivateRegisterPointPairTool(false);
//
//	freezeUI(true);
//
//	if (!m_pprDlg->start())
//		deactivateRegisterPointPairTool(false);
//	else
//		updateOverlayDialogsPlacement();
//}
//
//void MainWindow::deactivateRegisterPointPairTool(bool state)
//{
//	if (m_pprDlg)
//		m_pprDlg->clear();
//
//	//we enable all GL windows
//	enableAll();
//
//	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
//	if (!subWindowList.isEmpty())
//		subWindowList.first()->showMaximized();
//
//	freezeUI(false);
//
//	updateUI();
//
//	setGlobalZoom();
//}
//
//void MainWindow::activateSectionExtractionMode()
//{
//	if (!haveSelection())
//		return;
//
//	if (!m_seTool)
//	{
//		m_seTool = new ccSectionExtractionTool(this);
//		connect(m_seTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateSectionExtractionMode);
//
//		registerOverlayDialog(m_seTool, Qt::TopRightCorner);
//	}
//
//	//add clouds
//	ccGLWindow* firstDisplay = nullptr;
//	{
//		unsigned validCount = 0;
//		for (ccHObject *entity : getSelectedEntities())
//		{
//			if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
//			{
//				if (m_seTool->addCloud(static_cast<ccGenericPointCloud*>(entity)))
//				{
//					if (!firstDisplay && entity->getDisplay())
//					{
//						firstDisplay = static_cast<ccGLWindow*>(entity->getDisplay());
//					}
//					
//					++validCount;
//				}
//			}
//		}
//
//		if (validCount == 0)
//		{
//			ccConsole::Error("No cloud in selection!");
//			return;
//		}
//	}
//
//	//deselect all entities
//	if (m_ccRoot)
//	{
//		m_ccRoot->unselectAllEntities();
//	}
//
//	ccGLWindow* win = new3DView(false);
//	if (!win)
//	{
//		ccLog::Error("[SectionExtraction] Failed to create dedicated 3D view!");
//		return;
//	}
//
//	if (firstDisplay && firstDisplay->getGlFilter())
//	{
//		win->setGlFilter(firstDisplay->getGlFilter()->clone());
//	}
//	m_seTool->linkWith(win);
//
//	freezeUI(true);
//	m_UI->toolBarView->setDisabled(true);
//
//	//we disable all other windows
//	disableAllBut(win);
//
//	if (!m_seTool->start())
//		deactivateSectionExtractionMode(false);
//	else
//		updateOverlayDialogsPlacement();
//}
//
//void MainWindow::deactivateSectionExtractionMode(bool state)
//{
//	if (m_seTool)
//		m_seTool->removeAllEntities();
//
//	//we enable all GL windows
//	enableAll();
//
//	QList<QMdiSubWindow*> subWindowList = m_mdiArea->subWindowList();
//	if (!subWindowList.isEmpty())
//		subWindowList[0]->showMaximized();
//
//	freezeUI(false);
//	m_UI->toolBarView->setDisabled(false);
//
//	updateUI();
//
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//		win->redraw();
//}
//
//void MainWindow::activateSegmentationMode()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (!win)
//		return;
//
//	if (!haveSelection())
//		return;
//
//	if (!m_gsTool)
//	{
//		m_gsTool = new ccGraphicalSegmentationTool(this);
//		connect(m_gsTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateSegmentationMode);
//
//		registerOverlayDialog(m_gsTool, Qt::TopRightCorner);
//	}
//
//	m_gsTool->linkWith(win);
//
//	for ( ccHObject *entity : getSelectedEntities() )
//	{
//		m_gsTool->addEntity(entity);
//	}
//
//	if (m_gsTool->getNumberOfValidEntities() == 0)
//	{
//		ccConsole::Error("No segmentable entity in active window!");
//		return;
//	}
//
//	freezeUI(true);
//	m_UI->toolBarView->setDisabled(false);
//
//	//we disable all other windows
//	disableAllBut(win);
//
//	if (!m_gsTool->start())
//		deactivateSegmentationMode(false);
//	else
//		updateOverlayDialogsPlacement();
//}
//
//void MainWindow::deactivateSegmentationMode(bool state)
//{
//	bool deleteHiddenParts = false;
//
//	//shall we apply segmentation?
//	if (state)
//	{
//		ccHObject* firstResult = nullptr;
//
//		deleteHiddenParts = m_gsTool->deleteHiddenParts();
//
//		//aditional vertices of which visibility array should be manually reset
//		std::unordered_set<ccGenericPointCloud*> verticesToReset;
//
//		QSet<ccHObject*>& segmentedEntities = m_gsTool->entities();
//		for (QSet<ccHObject*>::iterator p = segmentedEntities.begin(); p != segmentedEntities.end(); )
//		{
//			ccHObject* entity = (*p);
//
//			if (entity->isKindOf(CC_TYPES::POINT_CLOUD) || entity->isKindOf(CC_TYPES::MESH))
//			{
//				//first, do the things that must absolutely be done BEFORE removing the entity from DB (even temporarily)
//				//bool lockedVertices;
//				ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity/*,&lockedVertices*/);
//				assert(cloud);
//				if (cloud)
//				{
//					//assert(!lockedVertices); //in some cases we accept to segment meshes with locked vertices!
//
//					//specific case: labels (do this before temporarily removing 'entity' from DB!)
//					ccHObject::Container labels;
//					if (m_ccRoot)
//					{
//						m_ccRoot->getRootEntity()->filterChildren(labels,true,CC_TYPES::LABEL_2D);
//					}
//					for (ccHObject::Container::iterator it = labels.begin(); it != labels.end(); ++it)
//					{
//						if ((*it)->isA(CC_TYPES::LABEL_2D)) //Warning: cc2DViewportLabel is also a kind of 'CC_TYPES::LABEL_2D'!
//						{
//							//we must search for all dependent labels and remove them!!!
//							//TODO: couldn't we be more clever and update the label instead?
//							cc2DLabel* label = static_cast<cc2DLabel*>(*it);
//							bool removeLabel = false;
//							for (unsigned i = 0; i < label->size(); ++i)
//							{
//								if (label->getPickedPoint(i).entity() == entity)
//								{
//									removeLabel = true;
//									break;
//								}
//							}
//
//							if (removeLabel && label->getParent())
//							{
//								ccLog::Warning(QString("[Segmentation] Label %1 depends on cloud %2 and will be removed").arg(label->getName(), cloud->getName()));
//								ccHObject* labelParent = label->getParent();
//								ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(labelParent);
//								labelParent->removeChild(label);
//								label = nullptr;
//								putObjectBackIntoDBTree(labelParent,objContext);
//							}
//						}
//					} //for each label
//				} // if (cloud)
//
//				//we temporarily detach the entity, as it may undergo
//				//"severe" modifications (octree deletion, etc.) --> see ccPointCloud::createNewCloudFromVisibilitySelection
//				ccHObjectContext objContext = removeObjectTemporarilyFromDBTree(entity);
//
//				//apply segmentation
//				ccHObject* segmentationResult = nullptr;
//				bool deleteOriginalEntity = deleteHiddenParts;
//				if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
//				{
//					ccGenericPointCloud* genCloud = ccHObjectCaster::ToGenericPointCloud(entity);
//					ccGenericPointCloud* segmentedCloud = genCloud->createNewCloudFromVisibilitySelection(!deleteHiddenParts);
//					if (segmentedCloud && segmentedCloud->size() == 0)
//					{
//						delete segmentationResult;
//						segmentationResult = nullptr;
//					}
//					else
//					{
//						segmentationResult = segmentedCloud;
//					}
//
//					deleteOriginalEntity |= (genCloud->size() == 0);
//				}
//				else if (entity->isKindOf(CC_TYPES::MESH)/*|| entity->isA(CC_TYPES::PRIMITIVE)*/) //TODO
//				{
//					if (entity->isA(CC_TYPES::MESH))
//					{
//						segmentationResult = ccHObjectCaster::ToMesh(entity)->createNewMeshFromSelection(!deleteHiddenParts);
//					}
//					else if (entity->isA(CC_TYPES::SUB_MESH))
//					{
//						segmentationResult = ccHObjectCaster::ToSubMesh(entity)->createNewSubMeshFromSelection(!deleteHiddenParts);
//					}
//
//					deleteOriginalEntity |=  (ccHObjectCaster::ToGenericMesh(entity)->size() == 0);
//				}
//
//				if (segmentationResult)
//				{
//					assert(cloud);
//					if (cloud)
//					{
//						//another specific case: sensors (on clouds)
//						for (unsigned i = 0; i < entity->getChildrenNumber(); ++i)
//						{
//							ccHObject* child = entity->getChild(i);
//							assert(child);
//							if (child && child->isKindOf(CC_TYPES::SENSOR))
//							{
//								if (child->isA(CC_TYPES::GBL_SENSOR))
//								{
//									ccGBLSensor* sensor = ccHObjectCaster::ToGBLSensor(entity->getChild(i));
//									//remove the associated depth buffer of the original sensor (derpecated)
//									sensor->clearDepthBuffer();
//									if (deleteOriginalEntity)
//									{
//										//either transfer
//										entity->transferChild(sensor,*segmentationResult);
//									}
//									else
//									{
//										//or copy
//										segmentationResult->addChild(new ccGBLSensor(*sensor));
//									}
//								}
//								else if (child->isA(CC_TYPES::CAMERA_SENSOR))
//								{
//									ccCameraSensor* sensor = ccHObjectCaster::ToCameraSensor(entity->getChild(i));
//									if (deleteOriginalEntity)
//									{
//										//either transfer
//										entity->transferChild(sensor,*segmentationResult);
//									}
//									else
//									{
//										//or copy
//										segmentationResult->addChild(new ccCameraSensor(*sensor));
//									}
//								}
//								else
//								{
//									//unhandled sensor?!
//									assert(false);
//								}
//							}
//						} //for each child
//					}
//
//					//we must take care of the remaining part
//					if (!deleteHiddenParts)
//					{
//						//no need to put back the entity in DB if we delete it afterwards!
//						if (!deleteOriginalEntity)
//						{
//							entity->setName(entity->getName() + QString(".remaining"));
//							putObjectBackIntoDBTree(entity, objContext);
//						}
//					}
//					else
//					{
//						//keep original name(s)
//						segmentationResult->setName(entity->getName());
//						if (entity->isKindOf(CC_TYPES::MESH) && segmentationResult->isKindOf(CC_TYPES::MESH))
//						{
//							ccGenericMesh* meshEntity = ccHObjectCaster::ToGenericMesh(entity);
//							ccHObjectCaster::ToGenericMesh(segmentationResult)->getAssociatedCloud()->setName(meshEntity->getAssociatedCloud()->getName());
//
//							//specific case: if the sub mesh is deleted afterwards (see below)
//							//then its associated vertices won't be 'reset' by the segmentation tool!
//							if (deleteHiddenParts && meshEntity->isA(CC_TYPES::SUB_MESH))
//							{
//								verticesToReset.insert(meshEntity->getAssociatedCloud());
//							}
//						}
//						assert(deleteOriginalEntity);
//						//deleteOriginalEntity = true;
//					}
//
//					if (segmentationResult->isA(CC_TYPES::SUB_MESH))
//					{
//						//for sub-meshes, we have no choice but to use its parent mesh!
//						objContext.parent = static_cast<ccSubMesh*>(segmentationResult)->getAssociatedMesh();
//					}
//					else
//					{
//						//otherwise we look for first non-mesh or non-cloud parent
//						while (objContext.parent && (objContext.parent->isKindOf(CC_TYPES::MESH) || objContext.parent->isKindOf(CC_TYPES::POINT_CLOUD)))
//						{
//							objContext.parent = objContext.parent->getParent();
//						}
//					}
//
//					if (objContext.parent)
//					{
//						objContext.parent->addChild(segmentationResult); //FiXME: objContext.parentFlags?
//					}
//
//					segmentationResult->setDisplay_recursive(entity->getDisplay());
//					segmentationResult->prepareDisplayForRefresh_recursive();
//
//					addToDB(segmentationResult);
//
//					if (!firstResult)
//					{
//						firstResult = segmentationResult;
//					}
//				}
//				else if (!deleteOriginalEntity)
//				{
//					//ccConsole::Error("An error occurred! (not enough memory?)");
//					putObjectBackIntoDBTree(entity,objContext);
//				}
//
//				if (deleteOriginalEntity)
//				{
//					p = segmentedEntities.erase(p);
//
//					delete entity;
//					entity = nullptr;
//				}
//				else
//				{
//					++p;
//				}
//			}
//		}
//
//		//specific actions
//		{
//			for ( ccGenericPointCloud *cloud : verticesToReset )
//			{
//				cloud->resetVisibilityArray();
//			}
//		}
//
//		if (firstResult && m_ccRoot)
//		{
//			m_ccRoot->selectEntity(firstResult);
//		}
//	}
//
//	if (m_gsTool)
//	{
//		m_gsTool->removeAllEntities(!deleteHiddenParts);
//	}
//
//	//we enable all GL windows
//	enableAll();
//
//	freezeUI(false);
//
//	updateUI();
//
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->redraw();
//	}
//}
//
//void MainWindow::activateTracePolylineMode()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (!win)
//	{
//		return;
//	}
//
//	if (!m_tplTool)
//	{
//		m_tplTool = new ccTracePolylineTool(m_pickingHub, this);
//		connect(m_tplTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateTracePolylineMode);
//		registerOverlayDialog(m_tplTool, Qt::TopRightCorner);
//	}
//
//	m_tplTool->linkWith(win);
//
//	freezeUI(true);
//	m_UI->toolBarView->setDisabled(false);
//
//	//we disable all other windows
//	disableAllBut(win);
//
//	if (!m_tplTool->start())
//		deactivateTracePolylineMode(false);
//	else
//		updateOverlayDialogsPlacement();
//}
//
//void MainWindow::deactivateTracePolylineMode(bool)
//{
//	//we enable all GL windows
//	enableAll();
//
//	freezeUI(false);
//
//	updateUI();
//
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->redraw();
//	}
//}
//
//void MainWindow::activatePointListPickingMode()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (!win)
//		return;
//
//	//there should be only one point cloud in current selection!
//	if (!haveOneSelection())
//	{
//		ccConsole::Error("Select one and only one entity!");
//		return;
//	}
//
//	ccHObject* entity = m_selectedEntities[0];
//	if (!entity->isKindOf(CC_TYPES::POINT_CLOUD) && !entity->isKindOf(CC_TYPES::MESH))
//	{
//		ccConsole::Error("Select a cloud or a mesh");
//		return;
//	}
//
//	if (!entity->isVisible() || !entity->isEnabled())
//	{
//		ccConsole::Error("Entity must be visible!");
//		return;
//	}
//
//	if (!m_plpDlg)
//	{
//		m_plpDlg = new ccPointListPickingDlg(m_pickingHub, this);
//		connect(m_plpDlg, &ccOverlayDialog::processFinished, this, &MainWindow::deactivatePointListPickingMode);
//
//		registerOverlayDialog(m_plpDlg, Qt::TopRightCorner);
//	}
//
//	//DGM: we must update marker size spin box value (as it may have changed by the user with the "display dialog")
//	m_plpDlg->markerSizeSpinBox->setValue(win->getDisplayParameters().labelMarkerSize);
//
//	m_plpDlg->linkWith(win);
//	m_plpDlg->linkWithEntity(entity);
//
//	freezeUI(true);
//
//	//we disable all other windows
//	disableAllBut(win);
//
//	if (!m_plpDlg->start())
//		deactivatePointListPickingMode(false);
//	else
//		updateOverlayDialogsPlacement();
//}
//
//void MainWindow::deactivatePointListPickingMode(bool state)
//{
//	if (m_plpDlg)
//	{
//		m_plpDlg->linkWithEntity(nullptr);
//	}
//
//	//we enable all GL windows
//	enableAll();
//
//	freezeUI(false);
//
//	updateUI();
//}
//
//void MainWindow::activatePointPickingMode()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (!win)
//	{
//		return;
//	}
//
//	if (m_ccRoot)
//	{
//		m_ccRoot->unselectAllEntities(); //we don't want any entity selected (especially existing labels!)
//	}
//
//	if (!m_ppDlg)
//	{
//		m_ppDlg = new ccPointPropertiesDlg(m_pickingHub, this);
//		connect(m_ppDlg, &ccOverlayDialog::processFinished,	this, &MainWindow::deactivatePointPickingMode);
//		connect(m_ppDlg, &ccPointPropertiesDlg::newLabel,	this, &MainWindow::handleNewLabel);
//
//		registerOverlayDialog(m_ppDlg, Qt::TopRightCorner);
//	}
//
//	m_ppDlg->linkWith(win);
//
//	freezeUI(true);
//
//	//we disable all other windows
//	disableAllBut(win);
//
//	if (!m_ppDlg->start())
//		deactivatePointPickingMode(false);
//	else
//		updateOverlayDialogsPlacement();
//}
//
//void MainWindow::deactivatePointPickingMode(bool state)
//{
//	//if (m_ppDlg)
//	//	m_ppDlg->linkWith(0);
//
//	//we enable all GL windows
//	enableAll();
//
//	freezeUI(false);
//
//	updateUI();
//}
//
//void MainWindow::activateClippingBoxMode()
//{
//	if ( !haveSelection() )
//	{
//		return;
//	}
//
//	ccGLWindow* win = getActiveGLWindow();
//	if (!win)
//	{
//		return;
//	}
//
//	if (!m_clipTool)
//	{
//		m_clipTool = new ccClippingBoxTool(this);
//		connect(m_clipTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateClippingBoxMode);
//	}
//	m_clipTool->linkWith(win);
//
//	ccHObject::Container selectedEntities = getSelectedEntities(); //we have to use a local copy: 'unselectEntity' will change the set of currently selected entities!
//	for (ccHObject *entity : selectedEntities)
//	{
//		if (m_clipTool->addAssociatedEntity(entity))
//		{
//			//automatically deselect the entity (to avoid seeing its bounding box ;)
//			m_ccRoot->unselectEntity(entity);
//		}
//	}
//
//	if (m_clipTool->getNumberOfAssociatedEntity() == 0)
//	{
//		m_clipTool->close();
//		return;
//	}
//
//	if (m_clipTool->start())
//	{
//		registerOverlayDialog(m_clipTool, Qt::TopRightCorner);
//		freezeUI(true);
//		updateOverlayDialogsPlacement();
//		//deactivate all other GL windows
//		disableAllBut(win);
//	}
//	else
//	{
//		ccConsole::Error("Unexpected error!"); //indeed...
//	}
//}
//
//void MainWindow::deactivateClippingBoxMode(bool state)
//{
//	//we reactivate all GL windows
//	enableAll();
//
//	freezeUI(false);
//
//	updateUI();
//}
//
//void MainWindow::activateTranslateRotateMode()
//{
//	if (!haveSelection())
//		return;
//
//	ccGLWindow* win = getActiveGLWindow();
//	if (!win)
//		return;
//
//	if (!m_transTool)
//		m_transTool = new ccGraphicalTransformationTool(this);
//	assert(m_transTool->getNumberOfValidEntities() == 0);
//	m_transTool->linkWith(win);
//
//	bool rejectedEntities = false;
//	for ( ccHObject *entity : getSelectedEntities() )
//	{
//		if (!m_transTool->addEntity(entity))
//			rejectedEntities = true;
//	}
//
//	if (m_transTool->getNumberOfValidEntities() == 0)
//	{
//		ccConsole::Error("No entity eligible for manual transformation! (see console)");
//		return;
//	}
//	else if (rejectedEntities)
//	{
//		ccConsole::Error("Some entities were ingored! (see console)");
//	}
//
//	//try to activate "moving mode" in current GL window
//	if (m_transTool->start())
//	{
//		connect(m_transTool, &ccOverlayDialog::processFinished, this, &MainWindow::deactivateTranslateRotateMode);
//		registerOverlayDialog(m_transTool, Qt::TopRightCorner);
//		freezeUI(true);
//		updateOverlayDialogsPlacement();
//		//deactivate all other GL windows
//		disableAllBut(win);
//	}
//	else
//	{
//		ccConsole::Error("Unexpected error!"); //indeed...
//	}
//}
//
//void MainWindow::deactivateTranslateRotateMode(bool state)
//{
//	if (m_transTool)
//	{
//		//reselect previously selected entities!
//		if (state && m_ccRoot)
//		{
//			const ccHObject& transformedSet = m_transTool->getValidEntities();
//			try
//			{
//				ccHObject::Container transformedEntities;
//				transformedEntities.resize(transformedSet.getChildrenNumber());
//				for (unsigned i = 0; i < transformedSet.getChildrenNumber(); ++i)
//				{
//					transformedEntities[i] = transformedSet.getChild(i);
//				}
//				m_ccRoot->selectEntities(transformedEntities);
//			}
//			catch (const std::bad_alloc&)
//			{
//				//not enough memory (nothing to do)
//			}
//		}
//		//m_transTool->close();
//	}
//
//	//we reactivate all GL windows
//	enableAll();
//
//	freezeUI(false);
//
//	updateUI();
//}
//
//void MainWindow::testFrameRate()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//		win->startFrameRateTest();
//}
//
//void MainWindow::showDisplayOptions()
//{
//	ccDisplayOptionsDlg displayOptionsDlg(this);
//	connect(&displayOptionsDlg, &ccDisplayOptionsDlg::aspectHasChanged, this, [=] () { redrawAll();	});
//			
//	displayOptionsDlg.exec();
//
//	disconnect(&displayOptionsDlg);
//}
//
//void MainWindow::doActionRenderToFile()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (!win)
//		return;
//
//	ccRenderToFileDlg rtfDlg(win->glWidth(), win->glHeight(), this);
//
//	if (rtfDlg.exec())
//	{
//		QApplication::processEvents();
//		win->renderToFile(rtfDlg.getFilename(), rtfDlg.getZoom(), rtfDlg.dontScalePoints(), rtfDlg.renderOverlayItems());
//	}
//}
//
//void MainWindow::doActionEditCamera()
//{
//	//current active MDI area
//	QMdiSubWindow* qWin = m_mdiArea->activeSubWindow();
//	if (!qWin)
//		return;
//
//	if (!m_cpeDlg)
//	{
//		m_cpeDlg = new ccCameraParamEditDlg(qWin, m_pickingHub);
//		//m_cpeDlg->makeFrameless(); //does not work on linux
//
//		connect(m_mdiArea, &QMdiArea::subWindowActivated,
//				m_cpeDlg, static_cast<void (ccCameraParamEditDlg::*)(QMdiSubWindow *)>(&ccCameraParamEditDlg::linkWith));
//
//		registerOverlayDialog(m_cpeDlg, Qt::BottomLeftCorner);
//	}
//
//	m_cpeDlg->linkWith(qWin);
//	m_cpeDlg->start();
//
//	updateOverlayDialogsPlacement();
//}
//
//void MainWindow::doActionAdjustZoom()
//{
//	//current active MDI area
//	ccGLWindow* win = getActiveGLWindow();
//	if (!win)
//		return;
//
//	const ccViewportParameters& params = win->getViewportParameters();
//	if (params.perspectiveView)
//	{
//		ccConsole::Error("Orthographic mode only!");
//		return;
//	}
//
//	ccAdjustZoomDlg azDlg(win,this);
//
//	if (!azDlg.exec())
//		return;
//
//	//apply zoom
//	double zoom = azDlg.getZoom();
//	win->setZoom(static_cast<float>(zoom));
//	win->redraw();
//}
//
//static unsigned s_viewportIndex = 0;
//void MainWindow::doActionSaveViewportAsCamera()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (!win)
//		return;
//
//	cc2DViewportObject* viewportObject = new cc2DViewportObject(QString("Viewport #%1").arg(++s_viewportIndex));
//	viewportObject->setParameters(win->getViewportParameters());
//	viewportObject->setDisplay(win);
//
//	addToDB(viewportObject);
//}

void MainWindow::zoomOnSelectedEntities()
{
	ccGLWindow* win = nullptr;

	ccHObject tempGroup("TempGroup");
	size_t selNum = m_selectedEntities.size();
	for (size_t i = 0; i < selNum; ++i)
	{
		ccHObject *entity = m_selectedEntities[i];

		if (i == 0 || !win)
		{
			//take the first valid window as reference
			win = static_cast<ccGLWindow*>(entity->getDisplay());
		}

		if (win)
		{
			if (entity->getDisplay() == win)
			{
				tempGroup.addChild(entity, ccHObject::DP_NONE);
			}
			else if (entity->getDisplay() != nullptr)
			{
				ccLog::Error("All selected entities must be displayed in the same 3D view!");
				return;
			}
		}
	}

	if (tempGroup.getChildrenNumber() != 0)
	{
		ccBBox box = tempGroup.getDisplayBB_recursive(false, win);
		if (!box.isValid())
		{
			ccLog::Warning("Selected entities have no valid bounding-box!");
		}
		else
		{
			if (win != nullptr)
			{
				win->updateConstellationCenterAndZoom(&box);
			}
		}
	}

	refreshAll();
}

void MainWindow::setGlobalZoom()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
		win->zoomGlobal();
}

//void MainWindow::setPivotAlwaysOn()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->setPivotVisibility(ccGLWindow::PIVOT_ALWAYS_SHOW);
//		win->redraw();
//
//		//update pop-up menu 'top' icon
//		if (m_pivotVisibilityPopupButton)
//			m_pivotVisibilityPopupButton->setIcon(m_UI->actionSetPivotAlwaysOn->icon());
//	}
//}
//
//void MainWindow::setPivotRotationOnly()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->setPivotVisibility(ccGLWindow::PIVOT_SHOW_ON_MOVE);
//		win->redraw();
//
//		//update pop-up menu 'top' icon
//		if (m_pivotVisibilityPopupButton)
//			m_pivotVisibilityPopupButton->setIcon(m_UI->actionSetPivotRotationOnly->icon());
//	}
//}
//
//void MainWindow::setPivotOff()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->setPivotVisibility(ccGLWindow::PIVOT_HIDE);
//		win->redraw();
//
//		//update pop-up menu 'top' icon
//		if (m_pivotVisibilityPopupButton)
//			m_pivotVisibilityPopupButton->setIcon(m_UI->actionSetPivotOff->icon());
//	}
//}
//
//void MainWindow::setOrthoView(ccGLWindow* win)
//{
//	if (win)
//	{
//		if (!checkStereoMode(win))
//		{
//			return;
//		}
//		win->setPerspectiveState(false, true);
//		win->redraw();
//
//		//update pop-up menu 'top' icon
//		if (m_viewModePopupButton)
//			m_viewModePopupButton->setIcon(m_UI->actionSetOrthoView->icon());
//		if (m_pivotVisibilityPopupButton)
//			m_pivotVisibilityPopupButton->setEnabled(true);
//	}
//}
//
//void MainWindow::setCenteredPerspectiveView(ccGLWindow* win, bool autoRedraw/*=true*/)
//{
//	if (win)
//	{
//		win->setPerspectiveState(true, true);
//		if (autoRedraw)
//			win->redraw();
//
//		//update pop-up menu 'top' icon
//		if (m_viewModePopupButton)
//			m_viewModePopupButton->setIcon(m_UI->actionSetCenteredPerspectiveView->icon());
//		if (m_pivotVisibilityPopupButton)
//			m_pivotVisibilityPopupButton->setEnabled(true);
//	}
//}
//
//void MainWindow::setViewerPerspectiveView(ccGLWindow* win)
//{
//	if (win)
//	{
//		win->setPerspectiveState(true,false);
//		win->redraw();
//
//		//update pop-up menu 'top' icon
//		if (m_viewModePopupButton)
//			m_viewModePopupButton->setIcon(m_UI->actionSetViewerPerspectiveView->icon());
//		if (m_pivotVisibilityPopupButton)
//			m_pivotVisibilityPopupButton->setEnabled(false);
//	}
//}
//
//void MainWindow::enablePickingOperation(ccGLWindow* win, QString message)
//{
//	if (!win)
//	{
//		assert(false);
//		return;
//	}
//
//	assert(m_pickingHub);
//	if (!m_pickingHub->addListener(this))
//	{
//		ccLog::Error("Can't start the picking mechanism (another tool is already using it)");
//		return;
//	}
//
//	//specific case: we prevent the 'point-pair based alignment' tool to process the picked point!
//	//if (m_pprDlg)
//	//	m_pprDlg->pause(true);
//
//	s_pickingWindow = win;
//	win->displayNewMessage(message, ccGLWindow::LOWER_LEFT_MESSAGE, true, 24 * 3600);
//	win->redraw(true, false);
//
//	freezeUI(true);
//}
//
//void MainWindow::cancelPreviousPickingOperation(bool aborted)
//{
//	if (!s_pickingWindow)
//		return;
//
//	switch(s_currentPickingOperation)
//	{
//	case PICKING_ROTATION_CENTER:
//		//nothing to do
//		break;
//	case PICKING_LEVEL_POINTS:
//		if (s_levelMarkersCloud)
//		{
//			s_pickingWindow->removeFromOwnDB(s_levelMarkersCloud);
//			delete s_levelMarkersCloud;
//			s_levelMarkersCloud = nullptr;
//		}
//		break;
//	default:
//		assert(false);
//		break;
//	}
//
//	if (aborted)
//	{
//		s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE); //clear previous messages
//		s_pickingWindow->displayNewMessage("Picking operation aborted", ccGLWindow::LOWER_LEFT_MESSAGE);
//	}
//	s_pickingWindow->redraw(false);
//
//	//specific case: we allow the 'point-pair based alignment' tool to process the picked point!
//	if (m_pprDlg)
//		m_pprDlg->pause(false);
//
//	freezeUI(false);
//
//	m_pickingHub->removeListener(this);
//
//	s_pickingWindow = nullptr;
//	s_currentPickingOperation = NO_PICKING_OPERATION;
//}
//
//void MainWindow::onItemPicked(const PickedItem& pi)
//{
//	if (!s_pickingWindow || !m_pickingHub)
//	{
//		return;
//	}
//
//	if (!pi.entity)
//	{
//		return;
//	}
//
//	if (m_pickingHub->activeWindow() != s_pickingWindow)
//	{
//		ccLog::Warning("The point picked was picked in the wrong window");
//		return;
//	}
//
//	CCVector3 pickedPoint = pi.P3D;
//	switch(s_currentPickingOperation)
//	{
//	case PICKING_LEVEL_POINTS:
//		{
//			//we only accept points picked on the right entity!
//			//if (obj != s_levelEntity)
//			//{
//			//	ccLog::Warning(QString("[Level] Only points picked on '%1' are considered!").arg(s_levelEntity->getName()));
//			//	return;
//			//}
//
//			if (!s_levelMarkersCloud)
//			{
//				assert(false);
//				cancelPreviousPickingOperation(true);
//			}
//
//			for (unsigned i = 0; i < s_levelMarkersCloud->size(); ++i)
//			{
//				const CCVector3* P = s_levelMarkersCloud->getPoint(i);
//				if ((pickedPoint - *P).norm() < 1.0e-6)
//				{
//					ccLog::Warning("[Level] Point is too close from the others!");
//					return;
//				}
//			}
//
//			//add the corresponding marker
//			s_levelMarkersCloud->addPoint(pickedPoint);
//			unsigned markerCount = s_levelMarkersCloud->size();
//			cc2DLabel* label = new cc2DLabel();
//			label->addPickedPoint(s_levelMarkersCloud, markerCount - 1);
//			label->setName(QString("P#%1").arg(markerCount));
//			label->setDisplayedIn2D(false);
//			label->setDisplay(s_pickingWindow);
//			label->setVisible(true);
//			s_levelMarkersCloud->addChild(label);
//			s_pickingWindow->redraw();
//
//			if (markerCount == 3)
//			{
//				//we have enough points!
//				const CCVector3* A = s_levelMarkersCloud->getPoint(0);
//				const CCVector3* B = s_levelMarkersCloud->getPoint(1);
//				const CCVector3* C = s_levelMarkersCloud->getPoint(2);
//				CCVector3 X = *B - *A;
//				CCVector3 Y = *C - *A;
//				CCVector3 Z = X.cross(Y);
//				//we choose 'Z' so that it points 'upward' relatively to the camera (assuming the user will be looking from the top)
//				CCVector3d viewDir = s_pickingWindow->getCurrentViewDir();
//				if (CCVector3d::fromArray(Z.u).dot(viewDir) > 0)
//				{
//					Z = -Z;
//				}
//				Y = Z.cross(X);
//				X.normalize();
//				Y.normalize();
//				Z.normalize();
//
//				ccGLMatrixd trans;
//				double* mat = trans.data();
//				mat[0] = X.x; mat[4] = X.y; mat[8]  = X.z; mat[12] = 0;
//				mat[1] = Y.x; mat[5] = Y.y; mat[9]  = Y.z; mat[13] = 0;
//				mat[2] = Z.x; mat[6] = Z.y; mat[10] = Z.z; mat[14] = 0;
//				mat[3] = 0  ; mat[7] = 0  ; mat[11] = 0  ; mat[15] = 1;
//
//				CCVector3d T = -CCVector3d::fromArray(A->u);
//				trans.apply(T);
//				T += CCVector3d::fromArray(A->u);
//				trans.setTranslation(T);
//
//				assert(haveOneSelection() && m_selectedEntities.front() == s_levelEntity);
//				applyTransformation(trans);
//
//				//clear message
//				s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE, false); //clear previous message
//				s_pickingWindow->setView(CC_TOP_VIEW);
//			}
//			else
//			{
//				//we need more points!
//				return;
//			}
//		}
//		//we use the next 'case' entry (PICKING_ROTATION_CENTER) to redefine the rotation center as well!
//		assert(s_levelMarkersCloud && s_levelMarkersCloud->size() != 0);
//		pickedPoint = *s_levelMarkersCloud->getPoint(0);
//		//break;
//
//	case PICKING_ROTATION_CENTER:
//		{
//			CCVector3d newPivot = CCVector3d::fromArray(pickedPoint.u);
//			//specific case: transformation tool is enabled
//			if (m_transTool && m_transTool->started())
//			{
//				m_transTool->setRotationCenter(newPivot);
//				const unsigned& precision = s_pickingWindow->getDisplayParameters().displayedNumPrecision;
//				s_pickingWindow->displayNewMessage(QString(), ccGLWindow::LOWER_LEFT_MESSAGE, false); //clear previous message
//				s_pickingWindow->displayNewMessage(QString("Point (%1 ; %2 ; %3) set as rotation center for interactive transformation")
//					.arg(pickedPoint.x, 0, 'f', precision)
//					.arg(pickedPoint.y, 0, 'f', precision)
//					.arg(pickedPoint.z, 0, 'f', precision),
//					ccGLWindow::LOWER_LEFT_MESSAGE, true);
//			}
//			else
//			{
//				const ccViewportParameters& params = s_pickingWindow->getViewportParameters();
//				if (!params.perspectiveView || params.objectCenteredView)
//				{
//					//apply current GL transformation (if any)
//					pi.entity->getGLTransformation().apply(newPivot);
//					s_pickingWindow->setPivotPoint(newPivot, true, true);
//				}
//			}
//			//s_pickingWindow->redraw(); //already called by 'cancelPreviousPickingOperation' (see below)
//		}
//		break;
//
//	default:
//		assert(false);
//		break;
//	}
//
//	cancelPreviousPickingOperation(false);
//}
//
//void MainWindow::doLevel()
//{
//	//picking operation already in progress
//	if (s_pickingWindow)
//	{
//		if (s_currentPickingOperation == PICKING_LEVEL_POINTS)
//		{
//			cancelPreviousPickingOperation(true);
//		}
//		else
//		{
//			ccConsole::Error("Stop the other picking operation first!");
//		}
//		return;
//	}
//
//	ccGLWindow* win = getActiveGLWindow();
//	if (!win)
//	{
//		ccConsole::Error("No active 3D view!");
//		return;
//	}
//
//	if (!haveOneSelection())
//	{
//		ccConsole::Error("Select an entity!");
//		return;
//	}
//
//	//create markers cloud
//	assert(!s_levelMarkersCloud);
//	{
//		s_levelMarkersCloud = new ccPointCloud("Level points");
//		if (!s_levelMarkersCloud->reserve(3))
//		{
//			ccConsole::Error("Not enough memory!");
//			return;
//		}
//		win->addToOwnDB(s_levelMarkersCloud);
//	}
//
//	s_levelEntity = m_selectedEntities[0];
//	s_levelLabels.clear();
//	s_currentPickingOperation = PICKING_LEVEL_POINTS;
//
//	enablePickingOperation(win,"Pick three points on the floor plane (click the Level button or press Escape to cancel)");
//}
//
//void MainWindow::doPickRotationCenter()
//{
//	//picking operation already in progress
//	if (s_pickingWindow)
//	{
//		if (s_currentPickingOperation == PICKING_ROTATION_CENTER)
//		{
//			cancelPreviousPickingOperation(true);
//		}
//		else
//		{
//			ccConsole::Error("Stop the other picking operation first!");
//		}
//		return;
//	}
//
//	ccGLWindow* win = getActiveGLWindow();
//	if (!win)
//	{
//		ccConsole::Error("No active 3D view!");
//		return;
//	}
//
//	bool objectCentered = true;
//	bool perspectiveEnabled = win->getPerspectiveState(objectCentered);
//	if (perspectiveEnabled && !objectCentered)
//	{
//		ccLog::Error("Perspective mode is viewer-centered: can't use a point as rotation center!");
//		return;
//	}
//
//	s_currentPickingOperation = PICKING_ROTATION_CENTER;
//	enablePickingOperation(win, "Pick a point to be used as rotation center (click on icon again to cancel)");
//}
//
//ccPointCloud* MainWindow::askUserToSelectACloud(ccHObject* defaultCloudEntity/*=0*/, QString inviteMessage/*=QString()*/)
//{
//	ccHObject::Container clouds;
//	m_ccRoot->getRootEntity()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
//	if (clouds.empty())
//	{
//		ccConsole::Error("No cloud in database!");
//		return 0;
//	}
//	//default selected index
//	int selectedIndex = 0;
//	if (defaultCloudEntity)
//	{
//		for (size_t i = 1; i < clouds.size(); ++i)
//		{
//			if (clouds[i] == defaultCloudEntity)
//			{
//				selectedIndex = static_cast<int>(i);
//				break;
//			}
//		}
//	}
//	//ask the user to choose a cloud
//	{
//		selectedIndex = ccItemSelectionDlg::SelectEntity(clouds, selectedIndex, this, inviteMessage);
//		if (selectedIndex < 0)
//			return 0;
//	}
//
//	assert(selectedIndex >= 0 && static_cast<size_t>(selectedIndex) < clouds.size());
//	return ccHObjectCaster::ToPointCloud(clouds[selectedIndex]);
//}
//
//void MainWindow::toggleSelectedEntitiesProperty( ccEntityAction::TOGGLE_PROPERTY property )
//{
//	if ( !ccEntityAction::toggleProperty( m_selectedEntities, property ) )
//	{
//		return;
//	}
//	
//	refreshAll();
//	updateUI();
//}
//
//void MainWindow::clearSelectedEntitiesProperty( ccEntityAction::CLEAR_PROPERTY property )
//{
//	if ( !ccEntityAction::clearProperty( m_selectedEntities, property, this ) )
//	{
//		return;
//	}
//
//	refreshAll();
//	updateUI();
//}

void MainWindow::setView(CC_VIEW_ORIENTATION view)
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		win->setView(view);
	}
}

//void MainWindow::spawnHistogramDialog(const std::vector<unsigned>& histoValues, double minVal, double maxVal, QString title, QString xAxisLabel)
//{
//	ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
//	hDlg->setAttribute(Qt::WA_DeleteOnClose, true);
//	hDlg->setWindowTitle("Histogram");
//
//	ccHistogramWindow* histogram = hDlg->window();
//	{
//		histogram->setTitle(title);
//		histogram->fromBinArray(histoValues, minVal, maxVal);
//		histogram->setAxisLabels(xAxisLabel, "Count");
//		histogram->refresh();
//	}
//
//	hDlg->show();
//}
//
//void MainWindow::showSelectedEntitiesHistogram()
//{
//	for ( ccHObject *entity : getSelectedEntities() )
//	{
//		//for "real" point clouds only
//		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud( entity );
//		if (cloud)
//		{
//			//we display the histogram of the current scalar field
//			ccScalarField* sf = static_cast<ccScalarField*>(cloud->getCurrentDisplayedScalarField());
//			if (sf)
//			{
//				ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
//				hDlg->setAttribute(Qt::WA_DeleteOnClose, true);
//				hDlg->setWindowTitle(QString("Histogram [%1]").arg(cloud->getName()));
//
//				ccHistogramWindow* histogram = hDlg->window();
//				{
//					unsigned numberOfPoints = cloud->size();
//					unsigned numberOfClasses = static_cast<unsigned>(sqrt(static_cast<double>(numberOfPoints)));
//					//we take the 'nearest' multiple of 4
//					numberOfClasses &= (~3);
//					numberOfClasses = std::max<unsigned>(4, numberOfClasses);
//					numberOfClasses = std::min<unsigned>(256, numberOfClasses);
//
//					histogram->setTitle(QString("%1 (%2 values) ").arg(sf->getName()).arg(numberOfPoints));
//					bool showNaNValuesInGrey = sf->areNaNValuesShownInGrey();
//					histogram->fromSF(sf, numberOfClasses, true, showNaNValuesInGrey);
//					histogram->setAxisLabels(sf->getName(), "Count");
//					histogram->refresh();
//				}
//				hDlg->show();
//			}
//		}
//	}
//}
//
//void MainWindow::doActionCrop()
//{
//	//find candidates
//	std::vector<ccHObject*> candidates;
//	ccBBox baseBB;
//	{
//		const ccHObject::Container& selectedEntities = getSelectedEntities();
//		for ( ccHObject *entity : selectedEntities )
//		{
//			if (	entity->isA(CC_TYPES::POINT_CLOUD)
//				||	entity->isKindOf(CC_TYPES::MESH) )
//			{
//				candidates.push_back(entity);
//				baseBB += entity->getOwnBB();
//			}
//		}
//	}
//
//	if (candidates.empty())
//	{
//		ccConsole::Warning("[Crop] No eligible candidate found!");
//		return;
//	}
//
//	ccBoundingBoxEditorDlg bbeDlg(this);
//	bbeDlg.setBaseBBox(baseBB, false);
//	bbeDlg.showInclusionWarning(false);
//	bbeDlg.setWindowTitle("Crop");
//
//	if (!bbeDlg.exec())
//	{
//		//process cancelled by user
//		return;
//	}
//
//	//deselect all entities
//	if (m_ccRoot)
//	{
//		m_ccRoot->unselectAllEntities();
//	}
//
//	//cropping box
//	ccBBox box = bbeDlg.getBox();
//
//	//process cloud/meshes
//	bool errors = false;
//	bool successes = false;
//	{
//		for ( ccHObject *entity : candidates )
//		{
//			ccHObject* croppedEnt = ccCropTool::Crop(entity, box, true);
//			if (croppedEnt)
//			{
//				croppedEnt->setName(entity->getName() + QString(".cropped"));
//				croppedEnt->setDisplay(entity->getDisplay());
//				croppedEnt->prepareDisplayForRefresh();
//				if (entity->getParent())
//					entity->getParent()->addChild(croppedEnt);
//				entity->setEnabled(false);
//				addToDB(croppedEnt);
//				//select output entity
//				m_ccRoot->selectEntity(croppedEnt, true);
//				successes = true;
//			}
//			else
//			{
//				errors = true;
//			}
//		}
//	}
//
//	if (successes)
//		ccLog::Warning("[Crop] Selected entities have been hidden");
//	if (errors)
//		ccLog::Error("Error(s) occurred! See the Console");
//
//	refreshAll();
//	updateUI();
//}
//
//void MainWindow::doActionClone()
//{
//	ccHObject* lastClone = nullptr;
//	
//	for ( ccHObject *entity : getSelectedEntities() )
//	{
//		ccHObject* clone = nullptr;
//		
//		if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
//		{
//			clone = ccHObjectCaster::ToGenericPointCloud(entity)->clone();
//			if (!clone)
//			{
//				ccConsole::Error(QString("An error occurred while cloning cloud %1").arg(entity->getName()));
//			}
//		}
//		else if (entity->isKindOf(CC_TYPES::PRIMITIVE))
//		{
//			clone = static_cast<ccGenericPrimitive*>(entity)->clone();
//			if (!clone)
//			{
//				ccConsole::Error(QString("An error occurred while cloning primitive %1").arg(entity->getName()));
//			}
//		}
//		else if (entity->isA(CC_TYPES::MESH))
//		{
//			clone = ccHObjectCaster::ToMesh(entity)->cloneMesh();
//			if (!clone)
//			{
//				ccConsole::Error(QString("An error occurred while cloning mesh %1").arg(entity->getName()));
//			}
//		}
//		else if (entity->isA(CC_TYPES::POLY_LINE))
//		{
//			ccPolyline* poly = ccHObjectCaster::ToPolyline(entity);
//			clone = (poly ? new ccPolyline(*poly) : 0);
//			if (!clone)
//			{
//				ccConsole::Error(QString("An error occurred while cloning polyline %1").arg(entity->getName()));
//			}
//		}
//		else if (entity->isA(CC_TYPES::FACET))
//		{
//			ccFacet* facet = ccHObjectCaster::ToFacet(entity);
//			clone = (facet ? facet->clone() : 0);
//			if (!clone)
//			{
//				ccConsole::Error(QString("An error occurred while cloning facet %1").arg(entity->getName()));
//			}
//		}
//		else
//		{
//			ccLog::Warning(QString("Entity '%1' can't be cloned (type not supported yet!)").arg(entity->getName()));
//		}
//
//		if (clone)
//		{
//			//copy GL transformation history
//			clone->setGLTransformationHistory(entity->getGLTransformationHistory());
//			//copy display
//			clone->setDisplay(entity->getDisplay());
//
//			addToDB(clone);
//			lastClone = clone;
//		}
//	}
//
//	if (lastClone && m_ccRoot)
//	{
//		m_ccRoot->selectEntity(lastClone);
//	}
//
//	updateUI();
//}
//
//static double s_constantSFValue = 0.0;
//void MainWindow::doActionAddConstantSF()
//{
//	if (!haveOneSelection())
//	{
//		if (haveSelection())
//			ccConsole::Error("Select only one cloud or one mesh!");
//		return;
//	}
//
//	ccHObject* ent = m_selectedEntities[0];
//
//	bool lockedVertices;
//	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(ent,&lockedVertices);
//
//	//for "real" point clouds only
//	if (!cloud)
//		return;
//	if (lockedVertices && !ent->isAncestorOf(cloud))
//	{
//		ccUtils::DisplayLockedVerticesWarning(ent->getName(),true);
//		return;
//	}
//
//	QString defaultName = "Constant";
//	unsigned trys = 1;
//	while (cloud->getScalarFieldIndexByName(qPrintable(defaultName)) >= 0 || trys > 99)
//	{
//		defaultName = QString("Constant #%1").arg(++trys);
//	}
//
//	//ask for a name
//	bool ok;
//	QString sfName = QInputDialog::getText(this,"New SF name", "SF name (must be unique)", QLineEdit::Normal, defaultName, &ok);
//	if (!ok)
//		return;
//	if (sfName.isNull())
//	{
//		ccLog::Error("Invalid name");
//		return;
//	}
//	if (cloud->getScalarFieldIndexByName(qPrintable(sfName)) >= 0)
//	{
//		ccLog::Error("Name already exists!");
//		return;
//	}
//
//	ScalarType sfValue = static_cast<ScalarType>(QInputDialog::getDouble(this, "Add constant value", "value", s_constantSFValue, -1.0e9, 1.0e9, 8, &ok));
//	if (!ok)
//		return;
//
//	int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
//	if (sfIdx < 0)
//		sfIdx = cloud->addScalarField(qPrintable(sfName));
//	if (sfIdx < 0)
//	{
//		ccLog::Error("An error occurred! (see console)");
//		return;
//	}
//
//	CCLib::ScalarField* sf = cloud->getScalarField(sfIdx);
//	assert(sf);
//	if (sf)
//	{
//		sf->fill(sfValue);
//		sf->computeMinAndMax();
//		cloud->setCurrentDisplayedScalarField(sfIdx);
//		cloud->showSF(true);
//		updateUI();
//		if (cloud->getDisplay())
//			cloud->getDisplay()->redraw(false);
//	}
//
//	ccLog::Print(QString("New scalar field added to %1 (constant value: %2)").arg(cloud->getName()).arg(sfValue));
//}
//
//void MainWindow::doActionScalarFieldFromColor()
//{
//	if ( !ccEntityAction::sfFromColor(m_selectedEntities, this) )
//		return;
//
//	refreshAll();
//	updateUI();
//}
//
//void MainWindow::doActionScalarFieldArithmetic()
//{
//	if ( !ccEntityAction::sfArithmetic(m_selectedEntities, this) )
//		return;
//
//	refreshAll();
//	updateUI();
//}
//
//void MainWindow::doActionFitSphere()
//{
//	double outliersRatio = 0.5;
//	double confidence = 0.99;
//
//	ccProgressDialog pDlg(true, this);
//	pDlg.setAutoClose(false);
//
//	for ( ccHObject *entity : getSelectedEntities() )
//	{
//		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
//		if (!cloud)
//			continue;
//
//		CCVector3 center;
//		PointCoordinateType radius;
//		double rms;
//		if (CCLib::GeometricalAnalysisTools::DetectSphereRobust(cloud,
//			outliersRatio,
//			center,
//			radius,
//			rms,
//			&pDlg,
//			confidence) != CCLib::GeometricalAnalysisTools::NoError)
//		{
//			ccLog::Warning(QString("[Fit sphere] Failed to fit a sphere on cloud '%1'").arg(cloud->getName()));
//			continue;
//		}
//
//		ccLog::Print(QString("[Fit sphere] Cloud '%1': center (%2,%3,%4) - radius = %5 [RMS = %6]")
//			.arg(cloud->getName())
//			.arg(center.x)
//			.arg(center.y)
//			.arg(center.z)
//			.arg(radius)
//			.arg(rms));
//
//		ccGLMatrix trans;
//		trans.setTranslation(center);
//		ccSphere* sphere = new ccSphere(radius, &trans, QString("Sphere r=%1 [rms %2]").arg(radius).arg(rms));
//		cloud->addChild(sphere);
//		//sphere->setDisplay(cloud->getDisplay());
//		sphere->prepareDisplayForRefresh();
//		addToDB(sphere, false, false, false);
//	}
//
//	refreshAll();
//}
//
//void MainWindow::doActionFitPlane()
//{
//	doComputePlaneOrientation(false);
//}
//
//void MainWindow::doActionFitFacet()
//{
//	doComputePlaneOrientation(true);
//}
//
//void MainWindow::doComputePlaneOrientation(bool fitFacet)
//{
//	if (!haveSelection())
//		return;
//
//	double maxEdgeLength = 0.0;
//	if (fitFacet)
//	{
//		bool ok = true;
//		static double s_polygonMaxEdgeLength = 0.0;
//		maxEdgeLength = QInputDialog::getDouble(this, "Fit facet", "Max edge length (0 = no limit)", s_polygonMaxEdgeLength, 0, 1.0e9, 8, &ok);
//		if (!ok)
//			return;
//		s_polygonMaxEdgeLength = maxEdgeLength;
//	}
//
//	ccHObject::Container selectedEntities = getSelectedEntities(); //warning, getSelectedEntites may change during this loop!
//	bool firstEntity = true;
//	
//	for (ccHObject *entity : selectedEntities) 
//	{
//		ccShiftedObject* shifted = nullptr;
//		CCLib::GenericIndexedCloudPersist* cloud = nullptr;
//
//		if (entity->isKindOf(CC_TYPES::POLY_LINE))
//		{
//			ccPolyline* poly = ccHObjectCaster::ToPolyline(entity);
//			cloud = static_cast<CCLib::GenericIndexedCloudPersist*>(poly);
//			shifted = poly;
//		}
//		else
//		{
//			ccGenericPointCloud* gencloud = ccHObjectCaster::ToGenericPointCloud(entity);
//			if (gencloud)
//			{
//				cloud = static_cast<CCLib::GenericIndexedCloudPersist*>(gencloud);
//				shifted = gencloud;
//			}
//		}
//
//		if (cloud)
//		{
//			double rms = 0.0;
//			CCVector3 C, N;
//
//			ccHObject* plane = nullptr;
//			if (fitFacet)
//			{
//				ccFacet* facet = ccFacet::Create(cloud, static_cast<PointCoordinateType>(maxEdgeLength));
//				if (facet)
//				{
//					plane = static_cast<ccHObject*>(facet);
//					N = facet->getNormal();
//					C = facet->getCenter();
//					rms = facet->getRMS();
//
//					//manually copy shift & scale info!
//					if (shifted)
//					{
//						ccPolyline* contour = facet->getContour();
//						if (contour)
//						{
//							contour->setGlobalScale(shifted->getGlobalScale());
//							contour->setGlobalShift(shifted->getGlobalShift());
//						}
//					}
//				}
//			}
//			else
//			{
//				ccPlane* pPlane = ccPlane::Fit(cloud, &rms);
//				if (pPlane)
//				{
//					plane = static_cast<ccHObject*>(pPlane);
//					N = pPlane->getNormal();
//					C = *CCLib::Neighbourhood(cloud).getGravityCenter();
//					pPlane->enableStippling(true);
//				}
//			}
//
//			//as all information appears in Console...
//			forceConsoleDisplay();
//
//			if (plane)
//			{
//				ccConsole::Print(QString("[Orientation] Entity '%1'").arg(entity->getName()));
//				ccConsole::Print("\t- plane fitting RMS: %f", rms);
//
//				//We always consider the normal with a positive 'Z' by default!
//				if (N.z < 0.0)
//					N *= -1.0;
//				ccConsole::Print("\t- normal: (%f,%f,%f)", N.x, N.y, N.z);
//
//				//we compute strike & dip by the way
//				PointCoordinateType dip = 0.0f;
//				PointCoordinateType dipDir = 0.0f;
//				ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
//				QString dipAndDipDirStr = ccNormalVectors::ConvertDipAndDipDirToString(dip, dipDir);
//				ccConsole::Print(QString("\t- %1").arg(dipAndDipDirStr));
//
//				//hack: output the transformation matrix that would make this normal points towards +Z
//				ccGLMatrix makeZPosMatrix = ccGLMatrix::FromToRotation(N, CCVector3(0, 0, PC_ONE));
//				CCVector3 Gt = C;
//				makeZPosMatrix.applyRotation(Gt);
//				makeZPosMatrix.setTranslation(C-Gt);
//				ccConsole::Print("[Orientation] A matrix that would make this plane horizontal (normal towards Z+) is:");
//				ccConsole::Print(makeZPosMatrix.toString(12,' ')); //full precision
//				ccConsole::Print("[Orientation] You can copy this matrix values (CTRL+C) and paste them in the 'Apply transformation tool' dialog");
//
//				plane->setName(dipAndDipDirStr);
//				plane->applyGLTransformation_recursive(); //not yet in DB
//				plane->setVisible(true);
//				plane->setSelectionBehavior(ccHObject::SELECTION_FIT_BBOX);
//
//				entity->addChild(plane);
//				plane->setDisplay(entity->getDisplay());
//				plane->prepareDisplayForRefresh_recursive();
//				addToDB(plane);
//
//				if (firstEntity)
//				{
//					m_ccRoot->unselectAllEntities();
//					m_ccRoot->selectEntity(plane);
//				}
//			}
//			else
//			{
//				ccConsole::Warning(QString("Failed to fit a plane/facet on entity '%1'").arg(entity->getName()));
//			}
//		}
//	}
//
//	refreshAll();
//	updateUI();
//}
//
//void MainWindow::doShowPrimitiveFactory()
//{
//	if (!m_pfDlg)
//		m_pfDlg = new ccPrimitiveFactoryDlg(this);
//
//	m_pfDlg->setModal(false);
//	m_pfDlg->setWindowModality(Qt::NonModal);
//	m_pfDlg->show();
//}
//
//void MainWindow::doComputeGeometricFeature()
//{
//	static ccLibAlgorithms::GeomCharacteristicSet s_selectedCharacteristics;
//
//	ccGeomFeaturesDlg gfDlg(this);
//	double radius = ccLibAlgorithms::GetDefaultCloudKernelSize(m_selectedEntities);
//	gfDlg.setRadius(radius);
//	gfDlg.setSelectedFeatures(s_selectedCharacteristics);
//	
//	if (!gfDlg.exec())
//		return;
//
//	radius = gfDlg.getRadius();
//	if (!gfDlg.getSelectedFeatures(s_selectedCharacteristics))
//	{
//		ccLog::Error("Not enough memory");
//		return;
//	}
//
//	ccLibAlgorithms::ComputeGeomCharacteristics(s_selectedCharacteristics, static_cast<PointCoordinateType>(radius), m_selectedEntities, this);
//
//	refreshAll();
//	updateUI();
//}
//
//void MainWindow::doActionSFGradient()
//{
//	if (!ccLibAlgorithms::ApplyCCLibAlgorithm(ccLibAlgorithms::CCLIB_ALGO_SF_GRADIENT, m_selectedEntities, this))
//		return;
//	refreshAll();
//	updateUI();
//}
//
//void MainWindow::doSphericalNeighbourhoodExtractionTest()
//{
//	size_t selNum = m_selectedEntities.size();
//	if (selNum < 1)
//		return;
//
//	//spherical neighborhood extraction radius
//	PointCoordinateType sphereRadius = ccLibAlgorithms::GetDefaultCloudKernelSize(m_selectedEntities);
//	if (sphereRadius < 0)
//	{
//		ccConsole::Error("Invalid kernel size!");
//		return;
//	}
//
//	bool ok;
//	double val = QInputDialog::getDouble(this, "SNE test", "Radius:", static_cast<double>(sphereRadius), DBL_MIN, 1.0e9, 8, &ok);
//	if (!ok)
//		return;
//	sphereRadius = static_cast<PointCoordinateType>(val);
//
//	QString sfName = QString("Spherical extraction test") + QString(" (%1)").arg(sphereRadius);
//
//	ccProgressDialog pDlg(true, this);
//	pDlg.setAutoClose(false);
//
//	for (size_t i = 0; i < selNum; ++i)
//	{
//		//we only process clouds
//		if (!m_selectedEntities[i]->isA(CC_TYPES::POINT_CLOUD))
//		{
//			continue;
//		}
//		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_selectedEntities[i]);
//
//		int sfIdx = cloud->getScalarFieldIndexByName(qPrintable(sfName));
//		if (sfIdx < 0)
//			sfIdx = cloud->addScalarField(qPrintable(sfName));
//		if (sfIdx < 0)
//		{
//			ccConsole::Error(QString("Failed to create scalar field on cloud '%1' (not enough memory?)").arg(cloud->getName()));
//			return;
//		}
//			
//		ccOctree::Shared octree = cloud->getOctree();
//		if (!octree)
//		{
//			pDlg.reset();
//			pDlg.show();
//			octree = cloud->computeOctree(&pDlg);
//			if (!octree)
//			{
//				ccConsole::Error(QString("Couldn't compute octree for cloud '%1'!").arg(cloud->getName()));
//				return;
//			}
//		}
//
//		CCLib::ScalarField* sf = cloud->getScalarField(sfIdx);
//		sf->fill(NAN_VALUE);
//		cloud->setCurrentScalarField(sfIdx);
//
//		QElapsedTimer eTimer;
//		eTimer.start();
//
//		size_t extractedPoints = 0;
//		unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(sphereRadius);
//		std::random_device rd;   // non-deterministic generator
//		std::mt19937 gen(rd());  // to seed mersenne twister.
//		std::uniform_int_distribution<unsigned> dist(0, cloud->size() - 1);
//
//		const unsigned samples = 1000;
//		for (unsigned j = 0; j < samples; ++j)
//		{
//			unsigned randIndex = dist(gen);
//			CCLib::DgmOctree::NeighboursSet neighbours;
//			octree->getPointsInSphericalNeighbourhood(*cloud->getPoint(randIndex), sphereRadius, neighbours, level);
//			size_t neihgboursCount = neighbours.size();
//			extractedPoints += neihgboursCount;
//			for (size_t k = 0; k < neihgboursCount; ++k)
//				cloud->setPointScalarValue(neighbours[k].pointIndex, static_cast<ScalarType>(sqrt(neighbours[k].squareDistd)));
//		}
//		ccConsole::Print("[SNE_TEST] Mean extraction time = %i ms (radius = %f, mean(neighbours) = %3.1f)", eTimer.elapsed(), sphereRadius, extractedPoints / static_cast<double>(samples));
//
//		sf->computeMinAndMax();
//		cloud->setCurrentDisplayedScalarField(sfIdx);
//		cloud->showSF(true);
//		cloud->prepareDisplayForRefresh();
//	}
//
//	refreshAll();
//	updateUI();
//}
//
//void MainWindow::doCylindricalNeighbourhoodExtractionTest()
//{
//	bool ok;
//	double radius = QInputDialog::getDouble(this, "CNE Test", "radius", 0.02, 1.0e-6, 1.0e6, 6, &ok);
//	if (!ok)
//		return;
//
//	double height = QInputDialog::getDouble(this, "CNE Test", "height", 0.05, 1.0e-6, 1.0e6, 6, &ok);
//	if (!ok)
//		return;
//
//	ccPointCloud* cloud = new ccPointCloud("cube");
//	const unsigned ptsCount = 1000000;
//	if (!cloud->reserve(ptsCount))
//	{
//		ccConsole::Error("Not enough memory!");
//		delete cloud;
//		return;
//	}
//
//	//fill a unit cube with random points
//	{
//		std::random_device rd;   // non-deterministic generator
//		std::mt19937 gen(rd());  // to seed mersenne twister.
//		std::uniform_real_distribution<double> dist(0, 1);
//
//		for (unsigned i = 0; i < ptsCount; ++i)
//		{
//			CCVector3 P(dist(gen),
//				dist(gen),
//				dist(gen));
//
//			cloud->addPoint(P);
//		}
//	}
//
//	//get/Add scalar field
//	static const char DEFAULT_CNE_TEST_TEMP_SF_NAME[] = "CNE test";
//	int sfIdx = cloud->getScalarFieldIndexByName(DEFAULT_CNE_TEST_TEMP_SF_NAME);
//	if (sfIdx < 0)
//		sfIdx = cloud->addScalarField(DEFAULT_CNE_TEST_TEMP_SF_NAME);
//	if (sfIdx < 0)
//	{
//		ccConsole::Error("Not enough memory!");
//		delete cloud;
//		return;
//	}
//	cloud->setCurrentScalarField(sfIdx);
//
//	//reset scalar field
//	cloud->getScalarField(sfIdx)->fill(NAN_VALUE);
//
//	ccProgressDialog pDlg(true, this);
//	ccOctree::Shared octree = cloud->computeOctree(&pDlg);
//	if (octree)
//	{
//		QElapsedTimer subTimer;
//		subTimer.start();
//		unsigned long long extractedPoints = 0;
//		unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(static_cast<PointCoordinateType>(2.5*radius)); //2.5 = empirical
//		const unsigned samples = 1000;
//		std::random_device rd;   // non-deterministic generator
//		std::mt19937 gen(rd());  // to seed mersenne twister.
//		std::uniform_real_distribution<PointCoordinateType> distAngle(0, static_cast<PointCoordinateType>(2 * M_PI));
//		std::uniform_int_distribution<unsigned> distIndex(0, ptsCount - 1);
//
//		for (unsigned j = 0; j < samples; ++j)
//		{
//			//generate random normal vector
//			CCVector3 dir(0, 0, 1);
//			{
//				ccGLMatrix rot;
//				rot.initFromParameters(distAngle(gen),
//					distAngle(gen),
//					distAngle(gen),
//					CCVector3(0, 0, 0));
//				rot.applyRotation(dir);
//			}
//			unsigned randIndex = distIndex(gen);
//
//			CCLib::DgmOctree::CylindricalNeighbourhood cn;
//			cn.center = *cloud->getPoint(randIndex);
//			cn.dir = dir;
//			cn.level = level;
//			cn.radius = static_cast<PointCoordinateType>(radius);
//			cn.maxHalfLength = static_cast<PointCoordinateType>(height / 2);
//
//			octree->getPointsInCylindricalNeighbourhood(cn);
//			//octree->getPointsInSphericalNeighbourhood(*cloud->getPoint(randIndex),radius,neighbours,level);
//			size_t neihgboursCount = cn.neighbours.size();
//			extractedPoints += static_cast<unsigned long long>(neihgboursCount);
//			for (size_t k = 0; k < neihgboursCount; ++k)
//			{
//				cloud->setPointScalarValue(cn.neighbours[k].pointIndex, static_cast<ScalarType>(sqrt(cn.neighbours[k].squareDistd)));
//			}
//		}
//		ccConsole::Print("[CNE_TEST] Mean extraction time = %i ms (radius = %f, height = %f, mean(neighbours) = %3.1f)", subTimer.elapsed(), radius, height, static_cast<double>(extractedPoints) / samples);
//	}
//	else
//	{
//		ccConsole::Error("Failed to compute octree!");
//	}
//
//	ccScalarField* sf = static_cast<ccScalarField*>(cloud->getScalarField(sfIdx));
//	sf->computeMinAndMax();
//	sf->showNaNValuesInGrey(false);
//	cloud->setCurrentDisplayedScalarField(sfIdx);
//	cloud->showSF(true);
//
//	addToDB(cloud);
//
//	refreshAll();
//	updateUI();
//}
//
//void MainWindow::doActionCreateCloudFromEntCenters()
//{
//	size_t selNum = getSelectedEntities().size();
//
//	ccPointCloud* centers = new ccPointCloud("centers");
//	if (!centers->reserve(static_cast<unsigned>(selNum)))
//	{
//		ccLog::Error("Not enough memory!");
//		delete centers;
//		centers = nullptr;
//		return;
//	}
//
//	//look for clouds
//	{
//		for ( ccHObject *entity : getSelectedEntities() )
//		{
//			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
//			
//			if (cloud == nullptr)
//			{
//				continue;
//			}
//			
//			centers->addPoint(cloud->getOwnBB().getCenter());
//			
//			//we display the cloud in the same window as the first (selected) cloud we encounter
//			if (!centers->getDisplay())
//			{
//				centers->setDisplay(cloud->getDisplay());
//			}
//		}
//	}
//
//	if (centers->size() == 0)
//	{
//		ccLog::Error("No cloud in selection?!");
//		delete centers;
//		centers = nullptr;
//	}
//	else
//	{
//		centers->resize(centers->size());
//		centers->setPointSize(10);
//		centers->setVisible(true);
//		addToDB(centers);
//	}
//}
//
//void MainWindow::doActionComputeBestICPRmsMatrix()
//{
//	//look for clouds
//	std::vector<ccPointCloud*> clouds;
//	try
//	{
//		for ( ccHObject *entity : getSelectedEntities() )
//		{
//			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
//			if (cloud)
//			{
//				clouds.push_back(cloud);
//			}
//		}
//	}
//	catch (const std::bad_alloc&)
//	{
//		ccLog::Error("Not enough memory!");
//		return;
//	}
//
//	size_t cloudCount = clouds.size();
//	if (cloudCount < 2)
//	{
//		ccLog::Error("Need at least two clouds!");
//		return;
//	}
//
//	//init matrices
//	std::vector<double> rmsMatrix;
//	std::vector<ccGLMatrix> matrices;
//	std::vector< std::pair<double, double> > matrixAngles;
//	try
//	{
//		rmsMatrix.resize(cloudCount*cloudCount, 0);
//
//		//init all possible transformations
//		static const double angularStep_deg = 45.0;
//		unsigned phiSteps = static_cast<unsigned>(360.0 / angularStep_deg);
//		assert(std::abs(360.0 - phiSteps * angularStep_deg) < ZERO_TOLERANCE);
//		unsigned thetaSteps = static_cast<unsigned>(180.0 / angularStep_deg);
//		assert(std::abs(180.0 - thetaSteps * angularStep_deg) < ZERO_TOLERANCE);
//		unsigned rotCount = phiSteps * (thetaSteps - 1) + 2;
//		matrices.reserve(rotCount);
//		matrixAngles.reserve(rotCount);
//
//		for (unsigned j = 0; j <= thetaSteps; ++j)
//		{
//			//we want to cover the full [0-180] interval! ([-90;90] in fact)
//			double theta_deg = j * angularStep_deg - 90.0;
//			for (unsigned i = 0; i < phiSteps; ++i)
//			{
//				double phi_deg = i * angularStep_deg;
//				ccGLMatrix trans;
//				trans.initFromParameters(	static_cast<float>(phi_deg * CC_DEG_TO_RAD),
//											static_cast<float>(theta_deg * CC_DEG_TO_RAD),
//											0,
//											CCVector3(0,0,0) );
//				matrices.push_back(trans);
//				matrixAngles.push_back( std::pair<double,double>(phi_deg,theta_deg) );
//
//				//for poles, no need to rotate!
//				if (j == 0 || j == thetaSteps)
//					break;
//			}
//		}
//	}
//	catch (const std::bad_alloc&)
//	{
//		ccLog::Error("Not enough memory!");
//		return;
//	}
//
//	//let's start!
//	{
//		ccProgressDialog pDlg(true, this);
//		pDlg.setMethodTitle(tr("Testing all possible positions"));
//		pDlg.setInfo(tr("%1 clouds and %2 positions").arg(cloudCount).arg(matrices.size()));
//		CCLib::NormalizedProgress nProgress(&pDlg, static_cast<unsigned>(((cloudCount*(cloudCount - 1)) / 2)*matrices.size()));
//		pDlg.start();
//		QApplication::processEvents();
//
////#define TEST_GENERATION
//#ifdef TEST_GENERATION
//		ccPointCloud* testSphere = new ccPointCloud();
//		testSphere->reserve(matrices.size());
//#endif
//
//		for (size_t i = 0; i < cloudCount - 1; ++i)
//		{
//			ccPointCloud* A = clouds[i];
//			A->computeOctree();
//
//			for (size_t j = i + 1; j < cloudCount; ++j)
//			{
//				ccGLMatrix transBToZero;
//				transBToZero.toIdentity();
//				transBToZero.setTranslation(-clouds[j]->getOwnBB().getCenter());
//
//				ccGLMatrix transFromZeroToA;
//				transFromZeroToA.toIdentity();
//				transFromZeroToA.setTranslation(A->getOwnBB().getCenter());
//
//#ifndef TEST_GENERATION
//				double minRMS = -1.0;
//				int bestMatrixIndex = -1;
//				ccPointCloud* bestB = nullptr;
//#endif
//				for (size_t k = 0; k < matrices.size(); ++k)
//				{
//					ccPointCloud* B = clouds[j]->cloneThis();
//					if (!B)
//					{
//						ccLog::Error("Not enough memory!");
//						return;
//					}
//
//					ccGLMatrix BtoA = transFromZeroToA * matrices[k] * transBToZero;
//					B->applyRigidTransformation(BtoA);
//
//#ifndef TEST_GENERATION
//					double finalRMS = 0.0;
//					unsigned finalPointCount = 0;
//					CCLib::ICPRegistrationTools::RESULT_TYPE result;
//					CCLib::ICPRegistrationTools::ScaledTransformation registerTrans;
//					CCLib::ICPRegistrationTools::Parameters params;
//					{
//						params.convType = CCLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
//						params.minRMSDecrease = 1.0e-6;
//					}
//
//					result = CCLib::ICPRegistrationTools::Register(A, 0, B, params, registerTrans, finalRMS, finalPointCount);
//
//					if (result >= CCLib::ICPRegistrationTools::ICP_ERROR)
//					{
//						delete B;
//						if (bestB)
//							delete bestB;
//						ccLog::Error("An error occurred while performing ICP!");
//						return;
//					}
//
//					if (minRMS < 0 || finalRMS < minRMS)
//					{
//						minRMS = finalRMS;
//						bestMatrixIndex = static_cast<int>(k);
//						std::swap(bestB, B);
//					}
//
//					if (B)
//					{
//						delete B;
//						B = nullptr;
//					}
//#else
//					addToDB(B);
//
//					//Test sphere
//					CCVector3 Y(0,1,0);
//					matrices[k].apply(Y);
//					testSphere->addPoint(Y);
//#endif
//
//					if (!nProgress.oneStep())
//					{
//						//process cancelled by user
//						return;
//					}
//				}
//
//#ifndef TEST_GENERATION
//				if (bestMatrixIndex >= 0)
//				{
//					assert(bestB);
//					ccHObject* group = new ccHObject(QString("Best case #%1 / #%2 - RMS = %3").arg(i+1).arg(j+1).arg(minRMS));
//					group->addChild(bestB);
//					group->setDisplay_recursive(A->getDisplay());
//					addToDB(group);
//					ccLog::Print(QString("[doActionComputeBestICPRmsMatrix] Comparison #%1 / #%2: min RMS = %3 (phi = %4 / theta = %5 deg.)").arg(i+1).arg(j+1).arg(minRMS).arg(matrixAngles[bestMatrixIndex].first).arg(matrixAngles[bestMatrixIndex].second));
//				}
//				else
//				{
//					assert(!bestB);
//					ccLog::Warning(QString("[doActionComputeBestICPRmsMatrix] Comparison #%1 / #%2: INVALID").arg(i+1).arg(j+1));
//				}
//
//				rmsMatrix[i*cloudCount + j] = minRMS;
//#else
//				addToDB(testSphere);
//				i = cloudCount;
//				break;
//#endif
//			}
//		}
//	}
//
//	//export result as a CSV file
//#ifdef TEST_GENERATION
//	if (false)
//#endif
//	{
//		//persistent settings
//		QSettings settings;
//		settings.beginGroup(ccPS::SaveFile());
//		QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
//
//		QString outputFilename = QFileDialog::getSaveFileName(	this,
//																"Select output file",
//																currentPath,
//																"*.csv",
//																nullptr,
//																CCFileDialogOptions());
//
//		if (outputFilename.isEmpty())
//			return;
//
//		QFile fp(outputFilename);
//		if (fp.open(QFile::Text | QFile::WriteOnly))
//		{
//			QTextStream stream(&fp);
//			//header
//			{
//				stream << "RMS";
//				for ( ccPointCloud *cloud : clouds )
//				{
//					stream << ";";
//					stream << cloud->getName();
//				}
//				stream << endl;
//			}
//
//			//rows
//			for (size_t j = 0; j < cloudCount; ++j)
//			{
//				stream << clouds[j]->getName();
//				stream << ";";
//				for (size_t i = 0; i < cloudCount; ++i)
//				{
//					stream << rmsMatrix[j*cloudCount+i];
//					stream << ";";
//				}
//				stream << endl;
//			}
//
//			ccLog::Print("[doActionComputeBestICPRmsMatrix] Job done");
//		}
//		else
//		{
//			ccLog::Error("Failed to save output file?!");
//		}
//	}
//}
//
//void MainWindow::doActionExportPlaneInfo()
//{
//	ccHObject::Container planes;
//
//	const ccHObject::Container& selectedEntities = getSelectedEntities();
//	if (selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::HIERARCHY_OBJECT))
//	{
//		//a group
//		selectedEntities.front()->filterChildren(planes, true, CC_TYPES::PLANE, false);
//	}
//	else
//	{
//		for (ccHObject* ent : selectedEntities)
//		{
//			if (ent->isKindOf(CC_TYPES::PLANE))
//			{
//				//a single plane
//				planes.push_back(static_cast<ccPlane*>(ent));
//			}
//		}
//	}
//
//	if (planes.size() == 0)
//	{
//		ccLog::Error("No plane in selection");
//		return;
//	}
//
//	//persistent settings
//	QSettings settings;
//	settings.beginGroup(ccPS::SaveFile());
//	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
//
//	QString outputFilename = QFileDialog::getSaveFileName(	this,
//															"Select output file",
//															currentPath,
//															"*.csv",
//															nullptr,
//															CCFileDialogOptions());
//
//	if (outputFilename.isEmpty())
//	{
//		//process cancelled by the user
//		return;
//	}
//
//	QFile csvFile(outputFilename);
//	if (!csvFile.open(QFile::WriteOnly | QFile::Text))
//	{
//		ccConsole::Error("Failed to open file for writing! (check file permissions)");
//		return;
//	}
//
//	//save last saving location
//	settings.setValue(ccPS::CurrentPath(), QFileInfo(outputFilename).absolutePath());
//	settings.endGroup();
//
//	//write CSV header
//	QTextStream csvStream(&csvFile);
//	csvStream << "Name;";
//	csvStream << "Width;";
//	csvStream << "Height;";
//	csvStream << "Cx;";
//	csvStream << "Cy;";
//	csvStream << "Cz;";
//	csvStream << "Nx;";
//	csvStream << "Ny;";
//	csvStream << "Nz;";
//	csvStream << "Dip;";
//	csvStream << "Dip dir;";
//	csvStream << endl;
//
//	QChar separator(';');
//
//	//write one line per plane
//	for (ccHObject* ent : planes)
//	{
//		ccPlane* plane = static_cast<ccPlane*>(ent);
//			
//		CCVector3 C = plane->getOwnBB().getCenter();
//		CCVector3 N = plane->getNormal();
//		PointCoordinateType dip_deg = 0, dipDir_deg = 0;
//		ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip_deg, dipDir_deg);
//
//		csvStream << plane->getName() << separator;		//Name
//		csvStream << plane->getXWidth() << separator;	//Width
//		csvStream << plane->getYWidth() << separator;	//Height
//		csvStream << C.x << separator;					//Cx
//		csvStream << C.y << separator;					//Cy
//		csvStream << C.z << separator;					//Cz
//		csvStream << N.x << separator;					//Nx
//		csvStream << N.y << separator;					//Ny
//		csvStream << N.z << separator;					//Nz
//		csvStream << dip_deg << separator;				//Dip
//		csvStream << dipDir_deg << separator;			//Dip direction
//		csvStream << endl;
//	}
//
//	ccConsole::Print(QString("[I/O] File '%1' successfully saved (%2 plane(s))").arg(outputFilename).arg(planes.size()));
//	csvFile.close();
//}
//
//void MainWindow::doActionExportCloudInfo()
//{
//	//look for clouds
//	ccHObject::Container clouds;
//
//	const ccHObject::Container& selectedEntities = getSelectedEntities();
//	if (selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::HIERARCHY_OBJECT))
//	{
//		//a group
//		selectedEntities.front()->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD, true);
//	}
//	else
//	{
//		for (ccHObject* entity : selectedEntities)
//		{
//			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
//			if (cloud)
//			{
//				clouds.push_back(cloud);
//			}
//		}
//	}
//
//	if (clouds.empty())
//	{
//		ccConsole::Error("Select at least one point cloud!");
//		return;
//	}
//
//	//persistent settings
//	QSettings settings;
//	settings.beginGroup(ccPS::SaveFile());
//	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
//
//	QString outputFilename = QFileDialog::getSaveFileName(	this,
//															"Select output file",
//															currentPath,
//															"*.csv",
//															nullptr,
//															CCFileDialogOptions());
//	if (outputFilename.isEmpty())
//	{
//		//process cancelled by the user
//		return;
//	}
//
//	QFile csvFile(outputFilename);
//	if (!csvFile.open(QFile::WriteOnly | QFile::Text))
//	{
//		ccConsole::Error("Failed to open file for writing! (check file permissions)");
//		return;
//	}
//
//	//save last saving location
//	settings.setValue(ccPS::CurrentPath(), QFileInfo(outputFilename).absolutePath());
//	settings.endGroup();
//
//	//determine the maximum number of SFs
//	unsigned maxSFCount = 0;
//	for (ccHObject* entity : clouds)
//	{
//		maxSFCount = std::max<unsigned>(maxSFCount, static_cast<ccPointCloud*>(entity)->getNumberOfScalarFields());
//	}
//
//	//write CSV header
//	QTextStream csvStream(&csvFile);
//	csvStream << "Name;";
//	csvStream << "Points;";
//	csvStream << "meanX;";
//	csvStream << "meanY;";
//	csvStream << "meanZ;";
//	{
//		for (unsigned i = 0; i < maxSFCount; ++i)
//		{
//			QString sfIndex = QString("SF#%1").arg(i + 1);
//			csvStream << sfIndex << " name;";
//			csvStream << sfIndex << " valid values;";
//			csvStream << sfIndex << " mean;";
//			csvStream << sfIndex << " std.dev.;";
//			csvStream << sfIndex << " sum;";
//		}
//	}
//	csvStream << endl;
//
//	//write one line per cloud
//	{
//		for (ccHObject* entity : clouds)
//		{
//			ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
//
//			CCVector3 G = *CCLib::Neighbourhood(cloud).getGravityCenter();
//			csvStream << cloud->getName() << ";" /*"Name;"*/;
//			csvStream << cloud->size() << ";" /*"Points;"*/;
//			csvStream << G.x << ";" /*"meanX;"*/;
//			csvStream << G.y << ";" /*"meanY;"*/;
//			csvStream << G.z << ";" /*"meanZ;"*/;
//			for (unsigned j = 0; j < cloud->getNumberOfScalarFields(); ++j)
//			{
//				CCLib::ScalarField* sf = cloud->getScalarField(j);
//				csvStream << sf->getName() << ";" /*"SF name;"*/;
//
//				unsigned validCount = 0;
//				double sfSum = 0.0;
//				double sfSum2 = 0.0;
//				for (unsigned k = 0; k < sf->currentSize(); ++k)
//				{
//					const ScalarType& val = sf->getValue(k);
//					if (CCLib::ScalarField::ValidValue(val))
//					{
//						++validCount;
//						sfSum += val;
//						sfSum2 += val*val;
//					}
//				}
//				csvStream << validCount << ";" /*"SF valid values;"*/;
//				double mean = sfSum/validCount;
//				csvStream << mean << ";" /*"SF mean;"*/;
//				csvStream << sqrt(std::abs(sfSum2/validCount - mean*mean)) << ";" /*"SF std.dev.;"*/;
//				csvStream << sfSum << ";" /*"SF sum;"*/;
//			}
//			csvStream << endl;
//		}
//	}
//
//	ccConsole::Print(QString("[I/O] File '%1' successfully saved (%2 cloud(s))").arg(outputFilename).arg(clouds.size()));
//	csvFile.close();
//}
//
//void MainWindow::doActionCloudCloudDist()
//{
//	if (getSelectedEntities().size() != 2)
//	{
//		ccConsole::Error("Select 2 point clouds!");
//		return;
//	}
//
//	if (!m_selectedEntities[0]->isKindOf(CC_TYPES::POINT_CLOUD) ||
//		!m_selectedEntities[1]->isKindOf(CC_TYPES::POINT_CLOUD))
//	{
//		ccConsole::Error("Select 2 point clouds!");
//		return;
//	}
//
//	ccOrderChoiceDlg dlg(	m_selectedEntities[0], "Compared",
//							m_selectedEntities[1], "Reference",
//							this );
//	if (!dlg.exec())
//		return;
//
//	ccGenericPointCloud* compCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getFirstEntity());
//	ccGenericPointCloud* refCloud = ccHObjectCaster::ToGenericPointCloud(dlg.getSecondEntity());
//
//	//assert(!m_compDlg);
//	if (m_compDlg)
//		delete m_compDlg;
//	m_compDlg = new ccComparisonDlg(compCloud, refCloud, ccComparisonDlg::CLOUDCLOUD_DIST, this);
//	connect(m_compDlg, &QDialog::finished, this, &MainWindow::deactivateComparisonMode);
//	m_compDlg->show();
//	//cDlg.setModal(false);
//	//cDlg.exec();
//	freezeUI(true);
//}
//
//void MainWindow::doActionCloudMeshDist()
//{
//	if (getSelectedEntities().size() != 2)
//	{
//		ccConsole::Error("Select 2 entities!");
//		return;
//	}
//
//	bool isMesh[2] = {false,false};
//	unsigned meshNum = 0;
//	unsigned cloudNum = 0;
//	for (unsigned i = 0; i < 2; ++i)
//	{
//		if (m_selectedEntities[i]->isKindOf(CC_TYPES::MESH))
//		{
//			++meshNum;
//			isMesh[i] = true;
//		}
//		else if (m_selectedEntities[i]->isKindOf(CC_TYPES::POINT_CLOUD))
//		{
//			++cloudNum;
//		}
//	}
//
//	if (meshNum == 0)
//	{
//		ccConsole::Error("Select at least one mesh!");
//		return;
//	}
//	else if (meshNum+cloudNum < 2)
//	{
//		ccConsole::Error("Select one mesh and one cloud or two meshes!");
//		return;
//	}
//
//	ccHObject* compEnt = nullptr;
//	ccGenericMesh* refMesh = nullptr;
//
//	if (meshNum == 1)
//	{
//		compEnt = m_selectedEntities[isMesh[0] ? 1 : 0];
//		refMesh = ccHObjectCaster::ToGenericMesh(m_selectedEntities[isMesh[0] ? 0 : 1]);
//	}
//	else
//	{
//		ccOrderChoiceDlg dlg(	m_selectedEntities[0], "Compared",
//								m_selectedEntities[1], "Reference",
//								this );
//		if (!dlg.exec())
//			return;
//
//		compEnt = dlg.getFirstEntity();
//		refMesh = ccHObjectCaster::ToGenericMesh(dlg.getSecondEntity());
//	}
//
//	//assert(!m_compDlg);
//	if (m_compDlg)
//		delete m_compDlg;
//	m_compDlg = new ccComparisonDlg(compEnt, refMesh, ccComparisonDlg::CLOUDMESH_DIST, this);
//	connect(m_compDlg, &QDialog::finished, this, &MainWindow::deactivateComparisonMode);
//	m_compDlg->show();
//
//	freezeUI(true);
//}
//
//void MainWindow::deactivateComparisonMode(int result)
//{
//	//DGM: a bug apperead with recent changes (from CC or QT?)
//	//which prevent us from deleting the dialog right away...
//	//(it seems that QT has not yet finished the dialog closing
//	//when the 'finished' signal is sent).
//	//if(m_compDlg)
//	//	delete m_compDlg;
//	//m_compDlg = 0;
//
//	//if the comparison is a success, we select only the compared entity
//	if (m_compDlg && result == QDialog::Accepted && m_ccRoot)
//	{
//		ccHObject* compEntity = m_compDlg->getComparedEntity();
//		if (compEntity)
//		{
//			m_ccRoot->selectEntity(compEntity);
//		}
//	}
//
//	freezeUI(false);
//
//	updateUI();
//}
//
//void MainWindow::toggleActiveWindowSunLight()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->toggleSunLight();
//		win->redraw(false);
//	}
//}
//
//void MainWindow::toggleActiveWindowCustomLight()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->toggleCustomLight();
//		win->redraw(false);
//	}
//}
//
//void MainWindow::toggleActiveWindowAutoPickRotCenter(bool state)
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->setAutoPickPivotAtCenter(state);
//
//		//save the option
//		{
//			QSettings settings;
//			settings.setValue(ccPS::AutoPickRotationCenter(), state);
//		}
//	}
//}
//
//void MainWindow::toggleActiveWindowShowCursorCoords(bool state)
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->showCursorCoordinates(state);
//	}
//}
//
//void MainWindow::toggleActiveWindowStereoVision(bool state)
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		bool isActive = win->stereoModeIsEnabled();
//		if (isActive == state)
//		{
//			//nothing to do
//			return;
//		}
//
//		if (isActive)
//		{
//			win->disableStereoMode();
//
//			if (win->getStereoParams().glassType == ccGLWindow::StereoParams::NVIDIA_VISION)
//			{
//				//disable (exclusive) full screen
//				m_UI->actionExclusiveFullScreen->setChecked(false);
//			}
//		}
//		else
//		{
//			//display a parameters dialog
//			ccStereoModeDlg smDlg(this);
//			smDlg.setParameters(win->getStereoParams());
//			if (!smDlg.exec())
//			{
//				//cancelled by the user
//				m_UI->actionEnableStereo->blockSignals(true);
//				m_UI->actionEnableStereo->setChecked(false);
//				m_UI->actionEnableStereo->blockSignals(false);
//				return;
//			}
//
//			ccGLWindow::StereoParams params = smDlg.getParameters();
//#ifndef CC_GL_WINDOW_USE_QWINDOW
//			if (!params.isAnaglyph())
//			{
//				ccLog::Error("This version doesn't handle stereo glasses and headsets.\nUse the 'Stereo' version instead.");
//				//activation of the stereo mode failed: cancel selection
//				m_UI->actionEnableStereo->blockSignals(true);
//				m_UI->actionEnableStereo->setChecked(false);
//				m_UI->actionEnableStereo->blockSignals(false);
//				return;
//			}
//#endif
//
//			//force perspective state!
//			if (!win->getViewportParameters().perspectiveView)
//			{
//				setCenteredPerspectiveView(win, false);
//			}
//
//			if (params.glassType == ccGLWindow::StereoParams::NVIDIA_VISION)
//			{
//				//force (exclusive) full screen
//				m_UI->actionExclusiveFullScreen->setChecked(true);
//			}
//
//			if (!win->enableStereoMode(params))
//			{
//				if (params.glassType == ccGLWindow::StereoParams::NVIDIA_VISION)
//				{
//					//disable (exclusive) full screen
//					m_UI->actionExclusiveFullScreen->setChecked(false);
//				}
//
//				//activation of the stereo mode failed: cancel selection
//				m_UI->actionEnableStereo->blockSignals(true);
//				m_UI->actionEnableStereo->setChecked(false);
//				m_UI->actionEnableStereo->blockSignals(false);
//			}
//		}
//		win->redraw();
//	}
//}
//
//bool MainWindow::checkStereoMode(ccGLWindow* win)
//{
//	assert(win);
//
//	if (win && win->getViewportParameters().perspectiveView && win->stereoModeIsEnabled())
//	{
//		ccGLWindow::StereoParams params = win->getStereoParams();
//		bool wasExclusiveFullScreen = win->exclusiveFullScreen();
//		if (wasExclusiveFullScreen)
//		{
//			win->toggleExclusiveFullScreen(false);
//		}
//		win->disableStereoMode();
//
//		if (QMessageBox::question(	this,
//									"Stereo mode",
//									"Stereo-mode only works in perspective mode. Do you want to disable it?",
//									QMessageBox::Yes,
//									QMessageBox::No) == QMessageBox::No )
//		{
//			if (wasExclusiveFullScreen)
//			{
//				win->toggleExclusiveFullScreen(true);
//				win->enableStereoMode(params);
//			}
//			return false;
//		}
//		else
//		{
//			if (win == getActiveGLWindow())
//			{
//				m_UI->actionEnableStereo->setChecked(false);
//			}
//			else
//			{
//				assert(false);
//				m_UI->actionEnableStereo->blockSignals(true);
//				m_UI->actionEnableStereo->setChecked(false);
//				m_UI->actionEnableStereo->blockSignals(false);
//			}
//		}
//	}
//
//	return true;
//}

void MainWindow::toggleActiveWindowCenteredPerspective()
{
	ccGLWindow* win = getActiveGLWindow();
	if (win)
	{
		const ccViewportParameters& params = win->getViewportParameters();
		// && !checkStereoMode(win)
		if (params.perspectiveView && params.objectCenteredView) //we need to check this only if we are already in object-centered perspective mode
		{
			return;
		}
		win->togglePerspective(true);
		win->redraw(false);
		//updateViewModePopUpMenu(win);
		//updatePivotVisibilityPopUpMenu(win);
	}
}

//void MainWindow::toggleActiveWindowViewerBasedPerspective()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		const ccViewportParameters& params = win->getViewportParameters();
//		if (params.perspectiveView && !params.objectCenteredView && !checkStereoMode(win)) //we need to check this only if we are already in viewer-based perspective mode
//		{
//			return;
//		}
//		win->togglePerspective(false);
//		win->redraw(false);
//		updateViewModePopUpMenu(win);
//		updatePivotVisibilityPopUpMenu(win);
//	}
//}
//
//void MainWindow::toggleLockRotationAxis()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		bool wasLocked = win->isRotationAxisLocked();
//		bool isLocked = !wasLocked;
//
//		static CCVector3d s_lastAxis(0.0, 0.0, 1.0);
//		if (isLocked)
//		{
//			ccAskThreeDoubleValuesDlg axisDlg("x", "y", "z", -1.0e12, 1.0e12, s_lastAxis.x, s_lastAxis.y, s_lastAxis.z, 4, "Lock rotation axis", this);
//			if (axisDlg.buttonBox->button(QDialogButtonBox::Ok))
//				axisDlg.buttonBox->button(QDialogButtonBox::Ok)->setFocus();
//			if (!axisDlg.exec())
//				return;
//			s_lastAxis.x = axisDlg.doubleSpinBox1->value();
//			s_lastAxis.y = axisDlg.doubleSpinBox2->value();
//			s_lastAxis.z = axisDlg.doubleSpinBox3->value();
//		}
//		win->lockRotationAxis(isLocked, s_lastAxis);
//
//		m_UI->actionLockRotationAxis->blockSignals(true);
//		m_UI->actionLockRotationAxis->setChecked(isLocked);
//		m_UI->actionLockRotationAxis->blockSignals(false);
//
//		if (isLocked)
//		{
//			win->displayNewMessage(QString("[ROTATION LOCKED]"), ccGLWindow::UPPER_CENTER_MESSAGE, false, 24 * 3600, ccGLWindow::ROTAION_LOCK_MESSAGE);
//		}
//		else
//		{
//			win->displayNewMessage(QString(), ccGLWindow::UPPER_CENTER_MESSAGE, false, 0, ccGLWindow::ROTAION_LOCK_MESSAGE);
//		}
//		win->redraw(true, false);
//	}
//}
//
//void MainWindow::doActionEnableBubbleViewMode()
//{
//	//special case: the selected entity is a TLS sensor or a cloud with a TLS sensor
//	if (m_ccRoot)
//	{
//		ccHObject::Container selectedEntities;
//		m_ccRoot->getSelectedEntities(selectedEntities);
//
//		if (selectedEntities.size() == 1)
//		{
//			ccHObject* ent = selectedEntities.front();
//			ccGBLSensor* sensor = nullptr;
//			if (ent->isA(CC_TYPES::GBL_SENSOR))
//			{
//				sensor = static_cast<ccGBLSensor*>(ent);
//			}
//			else if (ent->isA(CC_TYPES::POINT_CLOUD))
//			{
//				ccHObject::Container sensors;
//				ent->filterChildren(sensors, false, CC_TYPES::GBL_SENSOR, true);
//				if (sensors.size() >= 1)
//				{
//					sensor = static_cast<ccGBLSensor*>(sensors.front());
//				}
//			}
//
//			if (sensor)
//			{
//				sensor->applyViewport();
//				return;
//			}
//		}
//	}
//
//	//otherwise we simply enable the bubble view mode in the active 3D view
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->setBubbleViewMode(true);
//		win->redraw(false);
//	}
//}
//
//void MainWindow::doActionDeleteShader()
//{
//	ccGLWindow* win = getActiveGLWindow();
//	if (win)
//	{
//		win->setShader(nullptr);
//	}
//}
//
//void MainWindow::removeFromDB(ccHObject* obj, bool autoDelete/*=true*/)
//{
//	if (!obj)
//		return;
//
//	//remove dependency to avoid deleting the object when removing it from DB tree
//	if (!autoDelete && obj->getParent())
//		obj->getParent()->removeDependencyWith(obj);
//
//	if (m_ccRoot)
//		m_ccRoot->removeElement(obj);
//}
//
//void MainWindow::setSelectedInDB(ccHObject* obj, bool selected)
//{
//	if (obj && m_ccRoot)
//	{
//		if (selected)
//			m_ccRoot->selectEntity(obj);
//		else
//			m_ccRoot->unselectEntity(obj);
//	}
//}
//
//void MainWindow::addToDB(	ccHObject* obj,
//							bool updateZoom/*=true*/,
//							bool autoExpandDBTree/*=true*/,
//							bool checkDimensions/*=true*/,
//							bool autoRedraw/*=true*/)
//{
//	//let's check that the new entity is not too big nor too far from scene center!
//	if (checkDimensions)
//	{
//		//get entity bounding box
//		ccBBox bBox = obj->getBB_recursive();
//
//		CCVector3 center = bBox.getCenter();
//		PointCoordinateType diag = bBox.getDiagNorm();
//
//		CCVector3d P = CCVector3d::fromArray(center.u);
//		CCVector3d Pshift(0, 0, 0);
//		double scale = 1.0;
//		bool preserveCoordinateShift = true;
//		//here we must test that coordinates are not too big whatever the case because OpenGL
//		//really doesn't like big ones (even if we work with GLdoubles :( ).
//		if (ccGlobalShiftManager::Handle(P, diag, ccGlobalShiftManager::DIALOG_IF_NECESSARY, false, Pshift, &preserveCoordinateShift, &scale))
//		{
//			bool needRescale = (scale != 1.0);
//			bool needShift = (Pshift.norm2() > 0);
//
//			if (needRescale || needShift)
//			{
//				ccGLMatrix mat;
//				mat.toIdentity();
//				mat.data()[0] = mat.data()[5] = mat.data()[10] = static_cast<float>(scale);
//				mat.setTranslation(Pshift);
//				obj->applyGLTransformation_recursive(&mat);
//				ccConsole::Warning(QString("Entity '%1' has been translated: (%2,%3,%4) and rescaled of a factor %5 [original position will be restored when saving]").arg(obj->getName()).arg(Pshift.x,0,'f',2).arg(Pshift.y,0,'f',2).arg(Pshift.z,0,'f',2).arg(scale,0,'f',6));
//			}
//
//			//update 'global shift' and 'global scale' for ALL clouds recursively
//			if (preserveCoordinateShift)
//			{
//				//FIXME: why don't we do that all the time by the way?!
//				ccHObject::Container children;
//				children.push_back(obj);
//				while (!children.empty())
//				{
//					ccHObject* child = children.back();
//					children.pop_back();
//
//					if (child->isKindOf(CC_TYPES::POINT_CLOUD))
//					{
//						ccGenericPointCloud* pc = ccHObjectCaster::ToGenericPointCloud(child);
//						pc->setGlobalShift(pc->getGlobalShift() + Pshift);
//						pc->setGlobalScale(pc->getGlobalScale() * scale);
//					}
//
//					for (unsigned i = 0; i < child->getChildrenNumber(); ++i)
//					{
//						children.push_back(child->getChild(i));
//					}
//				}
//			}
//		}
//	}
//
//	//add object to DB root
//	if (m_ccRoot)
//	{
//		//force a 'global zoom' if the DB was emtpy!
//		if (!m_ccRoot->getRootEntity() || m_ccRoot->getRootEntity()->getChildrenNumber() == 0)
//		{
//			updateZoom = true;
//		}
//		m_ccRoot->addElement(obj, autoExpandDBTree);
//	}
//	else
//	{
//		ccLog::Warning("[MainWindow::addToDB] Internal error: no associated db?!");
//		assert(false);
//	}
//
//	//we can now set destination display (if none already)
//	if (!obj->getDisplay())
//	{
//		ccGLWindow* activeWin = getActiveGLWindow();
//		if (!activeWin)
//		{
//			//no active GL window?!
//			return;
//		}
//		obj->setDisplay_recursive(activeWin);
//	}
//
//	//eventually we update the corresponding display
//	assert(obj->getDisplay());
//	if (updateZoom)
//	{
//		static_cast<ccGLWindow*>(obj->getDisplay())->zoomGlobal(); //automatically calls ccGLWindow::redraw
//	}
//	else if (autoRedraw)
//	{
//		obj->redrawDisplay();
//	}
//}
//
//void MainWindow::onExclusiveFullScreenToggled(bool state)
//{
//	//we simply update the fullscreen action method icon (whatever the window)
//	ccGLWindow* win = getActiveGLWindow();
//	
//	if ( win == nullptr )
//		return;
//
//	m_UI->actionExclusiveFullScreen->blockSignals(true);
//	m_UI->actionExclusiveFullScreen->setChecked(win ? win->exclusiveFullScreen() : false);
//	m_UI->actionExclusiveFullScreen->blockSignals(false);
//
//	if (!state && win->stereoModeIsEnabled() && win->getStereoParams().glassType == ccGLWindow::StereoParams::NVIDIA_VISION)
//	{
//		//auto disable stereo mode as NVidia Vision only works in full screen mode!
//		m_UI->actionEnableStereo->setChecked(false);
//	}
//}
//
//void MainWindow::addToDBAuto(const QStringList& filenames)
//{
//	ccGLWindow* win = qobject_cast<ccGLWindow*>(QObject::sender());
//
//	addToDB(filenames, QString(), win);
//}
//
//void MainWindow::addToDB(	const QStringList& filenames,
//							QString fileFilter/*=QString()*/,
//							ccGLWindow* destWin/*=0*/)
//{
//	//to use the same 'global shift' for multiple files
//	CCVector3d loadCoordinatesShift(0,0,0);
//	bool loadCoordinatesTransEnabled = false;
//
//	FileIOFilter::LoadParameters parameters;
//	{
//		parameters.alwaysDisplayLoadDialog = true;
//		parameters.shiftHandlingMode = ccGlobalShiftManager::DIALOG_IF_NECESSARY;
//		parameters.coordinatesShift = &loadCoordinatesShift;
//		parameters.coordinatesShiftEnabled = &loadCoordinatesTransEnabled;
//		parameters.parentWidget = this;
//	}
//
//	//the same for 'addToDB' (if the first one is not supported, or if the scale remains too big)
//	CCVector3d addCoordinatesShift(0, 0, 0);
//
//	const ccOptions& options = ccOptions::Instance();
//	FileIOFilter::ResetSesionCounter();
//
//	for ( const QString &filename : filenames )
//	{
//		CC_FILE_ERROR result = CC_FERR_NO_ERROR;
//		ccHObject* newGroup = FileIOFilter::LoadFromFile(filename, parameters, result, fileFilter);
//
//		if (newGroup)
//		{
//			if (!options.normalsDisplayedByDefault)
//			{
//				//disable the normals on all loaded clouds!
//				ccHObject::Container clouds;
//				newGroup->filterChildren(clouds, true, CC_TYPES::POINT_CLOUD);
//				for (ccHObject* cloud : clouds)
//				{
//					if (cloud)
//					{
//						static_cast<ccGenericPointCloud*>(cloud)->showNormals(false);
//					}
//				}
//			}
//			
//			if (destWin)
//			{
//				newGroup->setDisplay_recursive(destWin);
//			}
//			addToDB(newGroup, true, true, false);
//
//			m_recentFiles->addFilePath( filename );
//		}
//
//		if (result == CC_FERR_CANCELED_BY_USER)
//		{
//			//stop importing the file if the user has cancelled the current process!
//			break;
//		}
//	}
//
//	QMainWindow::statusBar()->showMessage(QString("%1 file(s) loaded").arg(filenames.size()),2000);
//}
//
//void MainWindow::handleNewLabel(ccHObject* entity)
//{
//	if (entity)
//	{
//		addToDB(entity);
//	}
//	else
//	{
//		assert(false);
//	}
//}
//
//void MainWindow::forceConsoleDisplay()
//{
//	//if the console is hidden, we autoamtically display it!
//	if (m_UI->DockableConsole && m_UI->DockableConsole->isHidden())
//	{
//		m_UI->DockableConsole->show();
//		QApplication::processEvents();
//	}
//}
//
//ccColorScalesManager* MainWindow::getColorScalesManager()
//{
//	return ccColorScalesManager::GetUniqueInstance();
//}
//
//void MainWindow::closeAll()
//{
//	if (!m_ccRoot)
//	{
//		return;
//	}
//	
//	QMessageBox message_box( QMessageBox::Question,
//							 tr("Close all"),
//							 tr("Are you sure you want to remove all loaded entities?"),
//							 QMessageBox::Yes | QMessageBox::No,
//							 this );
//	
//	if (message_box.exec() == QMessageBox::No)
//	{
//		return;
//	}
//	
//	m_ccRoot->unloadAll();
//
//	redrawAll(false);
//}
//
//void MainWindow::doActionLoadFile()
//{
//	//persistent settings
//	QSettings settings;
//	settings.beginGroup(ccPS::LoadFile());
//	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
//	QString currentOpenDlgFilter = settings.value(ccPS::SelectedInputFilter(), BinFilter::GetFileFilter()).toString();
//
//	// Add all available file I/O filters (with import capabilities)
//	const QStringList filterStrings = FileIOFilter::ImportFilterList();
//	const QString &allFilter = filterStrings.at( 0 );
//	
//	if ( !filterStrings.contains( currentOpenDlgFilter ) )
//	{
//		currentOpenDlgFilter = allFilter;
//	}
//	
//	//file choosing dialog
//	QStringList selectedFiles = QFileDialog::getOpenFileNames(	this,
//																tr("Open file(s)"),
//																currentPath,
//																filterStrings.join(s_fileFilterSeparator),
//																&currentOpenDlgFilter,
//																CCFileDialogOptions());
//	if (selectedFiles.isEmpty())
//		return;
//
//	//save last loading parameters
//	currentPath = QFileInfo(selectedFiles[0]).absolutePath();
//	settings.setValue(ccPS::CurrentPath(),currentPath);
//	settings.setValue(ccPS::SelectedInputFilter(),currentOpenDlgFilter);
//	settings.endGroup();
//
//	if (currentOpenDlgFilter == allFilter)
//	{
//		currentOpenDlgFilter.clear(); //this way FileIOFilter will try to guess the file type automatically!
//	}
//	
//	//load files
//	addToDB(selectedFiles, currentOpenDlgFilter);
//}
//
////Helper: check for a filename validity
//static bool IsValidFileName(QString filename)
//{
//#ifdef CC_WINDOWS
//	QString sPattern("^(?!^(PRN|AUX|CLOCK\\$|NUL|CON|COM\\d|LPT\\d|\\..*)(\\..+)?$)[^\\x00-\\x1f\\\\?*:\\"";|/]+$");
//#else
//	QString sPattern("^(([a-zA-Z]:|\\\\)\\\\)?(((\\.)|(\\.\\.)|([^\\\\/:\\*\\?""\\|<>\\. ](([^\\\\/:\\*\\?""\\|<>\\. ])|([^\\\\/:\\*\\?""\\|<>]*[^\\\\/:\\*\\?""\\|<>\\. ]))?))\\\\)*[^\\\\/:\\*\\?""\\|<>\\. ](([^\\\\/:\\*\\?""\\|<>\\. ])|([^\\\\/:\\*\\?""\\|<>]*[^\\\\/:\\*\\?""\\|<>\\. ]))?$");
//#endif
//
//	return QRegExp(sPattern).exactMatch(filename);
//}
//
//void MainWindow::doActionSaveFile()
//{
//	if (!haveSelection())
//		return;
//
//	ccHObject clouds("clouds");
//	ccHObject meshes("meshes");
//	ccHObject images("images");
//	ccHObject polylines("polylines");
//	ccHObject other("other");
//	ccHObject otherSerializable("serializable");
//	ccHObject::Container entitiesToDispatch;
//	entitiesToDispatch.insert(entitiesToDispatch.begin(), m_selectedEntities.begin(), m_selectedEntities.end());
//	ccHObject entitiesToSave;
//	while (!entitiesToDispatch.empty())
//	{
//		ccHObject* child = entitiesToDispatch.back();
//		entitiesToDispatch.pop_back();
//
//		if (child->isA(CC_TYPES::HIERARCHY_OBJECT))
//		{
//			for (unsigned j = 0; j < child->getChildrenNumber(); ++j)
//				entitiesToDispatch.push_back(child->getChild(j));
//		}
//		else
//		{
//			//we put the entity in the container corresponding to its type
//			ccHObject* dest = nullptr;
//			if (child->isA(CC_TYPES::POINT_CLOUD))
//				dest = &clouds;
//			else if (child->isKindOf(CC_TYPES::MESH))
//				dest = &meshes;
//			else if (child->isKindOf(CC_TYPES::IMAGE))
//				dest = &images;
//			else if (child->isKindOf(CC_TYPES::POLY_LINE))
//				dest = &polylines;
//			else if (child->isSerializable())
//				dest = &otherSerializable;
//			else
//				dest = &other;
//
//			assert(dest);
//
//			//we don't want double insertions if the user has clicked both the father and child
//			if (!dest->find(child->getUniqueID()))
//			{
//				dest->addChild(child, ccHObject::DP_NONE);
//				entitiesToSave.addChild(child, ccHObject::DP_NONE);
//			}
//		}
//	}
//
//	bool hasCloud = (clouds.getChildrenNumber() != 0);
//	bool hasMesh = (meshes.getChildrenNumber() != 0);
//	bool hasImages = (images.getChildrenNumber() != 0);
//	bool hasPolylines = (polylines.getChildrenNumber() != 0);
//	bool hasSerializable = (otherSerializable.getChildrenNumber() != 0);
//	bool hasOther = (other.getChildrenNumber() != 0);
//
//	int stdSaveTypes =		static_cast<int>(hasCloud)
//						+	static_cast<int>(hasMesh)
//						+	static_cast<int>(hasImages)
//						+	static_cast<int>(hasPolylines)
//						+	static_cast<int>(hasSerializable);
//	if (stdSaveTypes == 0)
//	{
//		ccConsole::Error("Can't save selected entity(ies) this way!");
//		return;
//	}
//
//	//we set up the right file filters, depending on the selected
//	//entities type (cloud, mesh, etc.).
//	QStringList fileFilters;
//	{
//		for ( const FileIOFilter::Shared &filter : FileIOFilter::GetFilters() )
//		{
//			bool atLeastOneExclusive = false;
//
//			//can this filter export one or several clouds?
//			bool canExportClouds = true;
//			if (hasCloud)
//			{
//				bool isExclusive = true;
//				bool multiple = false;
//				canExportClouds = (		filter->canSave(CC_TYPES::POINT_CLOUD, multiple, isExclusive)
//									&&	(multiple || clouds.getChildrenNumber() == 1) );
//				atLeastOneExclusive |= isExclusive;
//			}
//
//			//can this filter export one or several meshes?
//			bool canExportMeshes = true;
//			if (hasMesh)
//			{
//				bool isExclusive = true;
//				bool multiple = false;
//				canExportMeshes = (		filter->canSave(CC_TYPES::MESH, multiple, isExclusive)
//									&&	(multiple || meshes.getChildrenNumber() == 1) );
//				atLeastOneExclusive |= isExclusive;
//			}
//
//			//can this filter export one or several polylines?
//			bool canExportPolylines = true;
//			if (hasPolylines)
//			{
//				bool isExclusive = true;
//				bool multiple = false;
//				canExportPolylines = (	filter->canSave(CC_TYPES::POLY_LINE, multiple, isExclusive)
//									&&	(multiple || polylines.getChildrenNumber() == 1) );
//				atLeastOneExclusive |= isExclusive;
//			}
//
//			//can this filter export one or several images?
//			bool canExportImages = true;
//			if (hasImages)
//			{
//				bool isExclusive = true;
//				bool multiple = false;
//				canExportImages = (		filter->canSave(CC_TYPES::IMAGE, multiple, isExclusive)
//									&&	(multiple || images.getChildrenNumber() == 1) );
//				atLeastOneExclusive |= isExclusive;
//			}
//
//			//can this filter export one or several other serializable entities?
//			bool canExportSerializables = true;
//			if (hasSerializable)
//			{
//				//check if all entities have the same type
//				{
//					CC_CLASS_ENUM firstClassID = otherSerializable.getChild(0)->getUniqueID();
//					for (unsigned j = 1; j < otherSerializable.getChildrenNumber(); ++j)
//					{
//						if (otherSerializable.getChild(j)->getUniqueID() != firstClassID)
//						{
//							//we add a virtual second 'stdSaveType' so as to properly handle exlusivity
//							++stdSaveTypes;
//							break;
//						}
//					}
//				}
//
//				for (unsigned j = 0; j < otherSerializable.getChildrenNumber(); ++j)
//				{
//					ccHObject* child = otherSerializable.getChild(j);
//					bool isExclusive = true;
//					bool multiple = false;
//					canExportSerializables &= (		filter->canSave(child->getClassID(), multiple, isExclusive)
//												&&	(multiple || otherSerializable.getChildrenNumber() == 1) );
//					atLeastOneExclusive |= isExclusive;
//				}
//			}
//
//			bool useThisFilter =	canExportClouds
//								&&	canExportMeshes
//								&&	canExportImages
//								&&	canExportPolylines
//								&&	canExportSerializables
//								&&	(!atLeastOneExclusive || stdSaveTypes == 1);
//
//			if (useThisFilter)
//			{
//				QStringList ff = filter->getFileFilters(false);
//				for (int j = 0; j < ff.size(); ++j)
//					fileFilters.append(ff[j]);
//			}
//		}
//	}
//
//	//persistent settings
//	QSettings settings;
//	settings.beginGroup(ccPS::SaveFile());
//
//	//default filter
//	QString selectedFilter = fileFilters.first();
//	if (hasCloud)
//		selectedFilter = settings.value(ccPS::SelectedOutputFilterCloud(),selectedFilter).toString();
//	else if (hasMesh)
//		selectedFilter = settings.value(ccPS::SelectedOutputFilterMesh(), selectedFilter).toString();
//	else if (hasImages)
//		selectedFilter = settings.value(ccPS::SelectedOutputFilterImage(), selectedFilter).toString();
//	else if (hasPolylines)
//		selectedFilter = settings.value(ccPS::SelectedOutputFilterPoly(), selectedFilter).toString();
//
//	//default output path (+ filename)
//	QString currentPath = settings.value(ccPS::CurrentPath(), ccFileUtils::defaultDocPath()).toString();
//	QString fullPathName = currentPath;
//	
//	if (haveOneSelection())
//	{
//		//hierarchy objects have generally as name: 'filename.ext (fullpath)'
//		//so we must only take the first part! (otherwise this type of name
//		//with a path inside perturbs the QFileDialog a lot ;))
//		QString defaultFileName(m_selectedEntities.front()->getName());
//		if (m_selectedEntities.front()->isA(CC_TYPES::HIERARCHY_OBJECT))
//		{
//			QStringList parts = defaultFileName.split(' ', QString::SkipEmptyParts);
//			if (!parts.empty())
//			{
//				defaultFileName = parts[0];
//			}
//		}
//
//		//we remove the extension
//		defaultFileName = QFileInfo(defaultFileName).baseName();
//
//		if (!IsValidFileName(defaultFileName))
//		{
//			ccLog::Warning("[I/O] First entity's name would make an invalid filename! Can't use it...");
//			defaultFileName = "project";
//		}
//
//		fullPathName += QString("/") + defaultFileName;
//	}
//
//	//ask the user for the output filename
//	QString selectedFilename = QFileDialog::getSaveFileName(this,
//															tr("Save file"),
//															fullPathName,
//															fileFilters.join(s_fileFilterSeparator),
//															&selectedFilter,
//															CCFileDialogOptions());
//
//	if (selectedFilename.isEmpty())
//	{
//		//process cancelled by the user
//		return;
//	}
//
//	//ignored items
//	if (hasOther)
//	{
//		ccConsole::Warning("[I/O] The following selected entities won't be saved:");
//		for (unsigned i = 0; i < other.getChildrenNumber(); ++i)
//		{
//			ccConsole::Warning(QString("\t- %1s").arg(other.getChild(i)->getName()));
//		}
//	}
//
//	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
//	FileIOFilter::SaveParameters parameters;
//	{
//		parameters.alwaysDisplaySaveDialog = true;
//		parameters.parentWidget = this;
//	}
//
//	//specific case: BIN format
//	if (selectedFilter == BinFilter::GetFileFilter())
//	{
//		if ( haveOneSelection() )
//		{
//			result = FileIOFilter::SaveToFile(m_selectedEntities.front(), selectedFilename, parameters, selectedFilter);
//		}
//		else
//		{
//			//we'll regroup all selected entities in a temporary group
//			ccHObject tempContainer;
//			ConvertToGroup(m_selectedEntities, tempContainer, ccHObject::DP_NONE);
//			if (tempContainer.getChildrenNumber())
//			{
//				result = FileIOFilter::SaveToFile(&tempContainer, selectedFilename, parameters, selectedFilter);
//			}
//			else
//			{
//				ccLog::Warning("[I/O] None of the selected entities can be saved this way...");
//				result = CC_FERR_NO_SAVE;
//			}
//		}
//	}
//	else if (entitiesToSave.getChildrenNumber() != 0)
//	{
//		//ignored items
//		//if (hasSerializable)
//		//{
//		//	if (!hasOther)
//		//		ccConsole::Warning("[I/O] The following selected entites won't be saved:"); //display this warning only if not already done
//		//	for (unsigned i = 0; i < otherSerializable.getChildrenNumber(); ++i)
//		//		ccConsole::Warning(QString("\t- %1").arg(otherSerializable.getChild(i)->getName()));
//		//}
//
//		result = FileIOFilter::SaveToFile(	entitiesToSave.getChildrenNumber() > 1 ? &entitiesToSave : entitiesToSave.getChild(0),
//											selectedFilename,
//											parameters,
//											selectedFilter);
//
//		if (result == CC_FERR_NO_ERROR && m_ccRoot)
//		{
//			m_ccRoot->unselectAllEntities();
//		}
//	}
//
//	//update default filters
//	if (hasCloud)
//		settings.setValue(ccPS::SelectedOutputFilterCloud(),selectedFilter);
//	if (hasMesh)
//		settings.setValue(ccPS::SelectedOutputFilterMesh(), selectedFilter);
//	if (hasImages)
//		settings.setValue(ccPS::SelectedOutputFilterImage(),selectedFilter);
//	if (hasPolylines)
//		settings.setValue(ccPS::SelectedOutputFilterPoly(), selectedFilter);
//
//	//we update current file path
//	currentPath = QFileInfo(selectedFilename).absolutePath();
//	settings.setValue(ccPS::CurrentPath(),currentPath);
//	settings.endGroup();
//}

void MainWindow::on3DViewActivated(QMdiSubWindow* mdiWin)
{
	ccGLWindow* win = mdiWin ? GLWindowFromWidget(mdiWin->widget()) : nullptr;
	//if (win)
	//{
	//	//updateViewModePopUpMenu(win);
	//	//updatePivotVisibilityPopUpMenu(win);
	//	m_UI->actionLockRotationAxis->blockSignals(true);
	//	m_UI->actionLockRotationAxis->setChecked(win->isRotationAxisLocked());
	//	m_UI->actionLockRotationAxis->blockSignals(false);

	//	m_UI->actionEnableStereo->blockSignals(true);
	//	m_UI->actionEnableStereo->setChecked(win->stereoModeIsEnabled());
	//	m_UI->actionEnableStereo->blockSignals(false);

	//	m_UI->actionExclusiveFullScreen->blockSignals(true);
	//	m_UI->actionExclusiveFullScreen->setChecked(win->exclusiveFullScreen());
	//	m_UI->actionExclusiveFullScreen->blockSignals(false);

	//	m_UI->actionShowCursor3DCoordinates->blockSignals(true);
	//	m_UI->actionShowCursor3DCoordinates->setChecked(win->cursorCoordinatesShown());
	//	m_UI->actionShowCursor3DCoordinates->blockSignals(false);

	//	m_UI->actionAutoPickRotationCenter->blockSignals(true);
	//	m_UI->actionAutoPickRotationCenter->setChecked(win->autoPickPivotAtCenter());
	//	m_UI->actionAutoPickRotationCenter->blockSignals(false);
	//}
	//m_UI->actionLockRotationAxis->setEnabled(win != nullptr);
	//m_UI->actionEnableStereo->setEnabled(win != nullptr);
	//m_UI->actionExclusiveFullScreen->setEnabled(win != nullptr);
}

//void MainWindow::updateViewModePopUpMenu(ccGLWindow* win)
//{
//	if (!m_viewModePopupButton)
//		return;
//
//	//update the view mode pop-up 'top' icon
//	if (win)
//	{
//		bool objectCentered = true;
//		bool perspectiveEnabled = win->getPerspectiveState(objectCentered);
//
//		QAction* currentModeAction = nullptr;
//		if (!perspectiveEnabled)
//		{
//			currentModeAction = m_UI->actionSetOrthoView;
//		}
//		else if (objectCentered)
//		{
//			currentModeAction = m_UI->actionSetCenteredPerspectiveView;
//		}
//		else
//		{
//			currentModeAction = m_UI->actionSetViewerPerspectiveView;
//		}
//
//		assert(currentModeAction);
//		m_viewModePopupButton->setIcon(currentModeAction->icon());
//		m_viewModePopupButton->setEnabled(true);
//	}
//	else
//	{
//		m_viewModePopupButton->setIcon(QIcon());
//		m_viewModePopupButton->setEnabled(false);
//	}
//}
//
//void MainWindow::updatePivotVisibilityPopUpMenu(ccGLWindow* win)
//{
//	if (!m_pivotVisibilityPopupButton)
//		return;
//
//	//update the pivot visibility pop-up 'top' icon
//	if (win)
//	{
//		QAction* visibilityAction = nullptr;
//		switch(win->getPivotVisibility())
//		{
//		case ccGLWindow::PIVOT_HIDE:
//			visibilityAction = m_UI->actionSetPivotOff;
//			break;
//		case ccGLWindow::PIVOT_SHOW_ON_MOVE:
//			visibilityAction = m_UI->actionSetPivotRotationOnly;
//			break;
//		case ccGLWindow::PIVOT_ALWAYS_SHOW:
//			visibilityAction = m_UI->actionSetPivotAlwaysOn;
//			break;
//		default:
//			assert(false);
//		}
//
//		if (visibilityAction)
//			m_pivotVisibilityPopupButton->setIcon(visibilityAction->icon());
//
//		//pivot is not available in viewer-based perspective!
//		bool objectCentered = true;
//		win->getPerspectiveState(objectCentered);
//		m_pivotVisibilityPopupButton->setEnabled(objectCentered);
//	}
//	else
//	{
//		m_pivotVisibilityPopupButton->setIcon(QIcon());
//		m_pivotVisibilityPopupButton->setEnabled(false);
//	}
//}
//
//void MainWindow::updateMenus()
//{
//	ccGLWindow* active3DView = getActiveGLWindow();
//	bool hasMdiChild = (active3DView != nullptr);
//	int mdiChildCount = getGLWindowCount();
//	bool hasLoadedEntities = (m_ccRoot && m_ccRoot->getRootEntity() && m_ccRoot->getRootEntity()->getChildrenNumber() != 0);
//	bool hasSelectedEntities = (m_ccRoot && m_ccRoot->countSelectedEntities() > 0);
//
//	//General Menu
//	m_UI->menuEdit->setEnabled(true/*hasSelectedEntities*/);
//	m_UI->menuTools->setEnabled(true/*hasSelectedEntities*/);
//
//	//3D Views Menu
//	m_UI->actionClose3DView    ->setEnabled(hasMdiChild);
//	m_UI->actionCloseAll3DViews->setEnabled(mdiChildCount != 0);
//	m_UI->actionTile3DViews    ->setEnabled(mdiChildCount > 1);
//	m_UI->actionCascade3DViews ->setEnabled(mdiChildCount > 1);
//	m_UI->actionNext3DView     ->setEnabled(mdiChildCount > 1);
//	m_UI->actionPrevious3DView ->setEnabled(mdiChildCount > 1);
//
//	//Shaders & Filters display Menu
//	bool shadersEnabled = (active3DView ? active3DView->areShadersEnabled() : false);
//	m_UI->actionLoadShader->setEnabled(shadersEnabled);
//	m_UI->actionDeleteShader->setEnabled(shadersEnabled);
//
//	//View Menu
//	m_UI->toolBarView->setEnabled(hasMdiChild);
//
//	//oher actions
//	m_UI->actionSegment->setEnabled(hasMdiChild && hasSelectedEntities);
//	m_UI->actionTranslateRotate->setEnabled(hasMdiChild && hasSelectedEntities);
//	m_UI->actionPointPicking->setEnabled(hasMdiChild && hasLoadedEntities);
//	m_UI->actionTestFrameRate->setEnabled(hasMdiChild);
//	m_UI->actionRenderToFile->setEnabled(hasMdiChild);
//	m_UI->actionToggleSunLight->setEnabled(hasMdiChild);
//	m_UI->actionToggleCustomLight->setEnabled(hasMdiChild);
//	m_UI->actionToggleCenteredPerspective->setEnabled(hasMdiChild);
//	m_UI->actionToggleViewerBasedPerspective->setEnabled(hasMdiChild);
//
//	//plugins
//	m_pluginUIManager->updateMenus();
//}
//
//void MainWindow::update3DViewsMenu()
//{
//	m_UI->menu3DViews->clear();
//	m_UI->menu3DViews->addAction(m_UI->actionNew3DView);
//	m_UI->menu3DViews->addSeparator();
//	m_UI->menu3DViews->addAction(m_UI->actionZoomIn);
//	m_UI->menu3DViews->addAction(m_UI->actionZoomOut);
//	m_UI->menu3DViews->addSeparator();
//	m_UI->menu3DViews->addAction(m_UI->actionClose3DView);
//	m_UI->menu3DViews->addAction(m_UI->actionCloseAll3DViews);
//	m_UI->menu3DViews->addSeparator();
//	m_UI->menu3DViews->addAction(m_UI->actionTile3DViews);
//	m_UI->menu3DViews->addAction(m_UI->actionCascade3DViews);
//	m_UI->menu3DViews->addSeparator();
//	m_UI->menu3DViews->addAction(m_UI->actionNext3DView);
//	m_UI->menu3DViews->addAction(m_UI->actionPrevious3DView);
//
//	QList<QMdiSubWindow *> windows = m_mdiArea->subWindowList();
//	if (!windows.isEmpty())
//	{
//		//Dynamic Separator
//		QAction* separator = new QAction(this);
//		separator->setSeparator(true);
//		m_UI->menu3DViews->addAction(separator);
//
//		int i = 0;
//		
//		for ( QMdiSubWindow *window : windows )
//		{
//			ccGLWindow *child = GLWindowFromWidget(window->widget());
//
//			QString text = QString("&%1 %2").arg(++i).arg(child->windowTitle());
//			QAction *action = m_UI->menu3DViews->addAction(text);
//			
//			action->setCheckable(true);
//			action->setChecked(child == getActiveGLWindow());
//			
//			connect(action, &QAction::triggered, this, [=] () {
//				setActiveSubWindow( window );
//			} );
//		}
//	}
//}
//
//void MainWindow::setActiveSubWindow(QWidget *window)
//{
//	if (!window || !m_mdiArea)
//		return;
//	m_mdiArea->setActiveSubWindow(qobject_cast<QMdiSubWindow *>(window));
//}
//
void MainWindow::redrawAll(bool only2D/*=false*/)
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		GLWindowFromWidget(window->widget())->redraw(only2D);
	}
}

void MainWindow::refreshAll(bool only2D/*=false*/)
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		GLWindowFromWidget(window->widget())->refresh(only2D);
	}
}

//void MainWindow::updateUI()
//{
//	updateUIWithSelection();
//	updateMenus();
//	updatePropertiesView();
//}
//
//void MainWindow::updatePropertiesView()
//{
//	if (m_ccRoot)
//	{
//		m_ccRoot->updatePropertiesView();
//	}
//}
//
//void MainWindow::updateUIWithSelection()
//{
//	dbTreeSelectionInfo selInfo;
//
//	m_selectedEntities.clear();
//
//	if (m_ccRoot)
//	{
//		m_ccRoot->getSelectedEntities(m_selectedEntities, CC_TYPES::OBJECT, &selInfo);
//	}
//
//	enableUIItems(selInfo);
//}
//
void MainWindow::enableAll()
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		window->setEnabled(true);
	}
}

void MainWindow::disableAll()
{
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		window->setEnabled(false);
	}
}

void MainWindow::disableAllBut(ccGLWindow* win)
{
	//we disable all other windows
	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		if (GLWindowFromWidget(window->widget()) != win)
		{
			window->setEnabled(false);
		}
	}
}

//void MainWindow::enableUIItems(dbTreeSelectionInfo& selInfo)
//{
//	bool dbIsEmpty = (!m_ccRoot || !m_ccRoot->getRootEntity() || m_ccRoot->getRootEntity()->getChildrenNumber() == 0);
//	bool atLeastOneEntity = (selInfo.selCount > 0);
//	bool atLeastOneCloud = (selInfo.cloudCount > 0);
//	bool atLeastOneMesh = (selInfo.meshCount > 0);
//	//bool atLeastOneOctree = (selInfo.octreeCount > 0);
//	bool atLeastOneNormal = (selInfo.normalsCount > 0);
//	bool atLeastOneColor = (selInfo.colorCount > 0);
//	bool atLeastOneSF = (selInfo.sfCount > 0);
//	bool atLeastOneGrid = (selInfo.gridCound > 0);
//
//	//bool atLeastOneSensor = (selInfo.sensorCount > 0);
//	bool atLeastOneGBLSensor = (selInfo.gblSensorCount > 0);
//	bool atLeastOneCameraSensor = (selInfo.cameraSensorCount > 0);
//	bool atLeastOnePolyline = (selInfo.polylineCount > 0);
//	bool activeWindow = (getActiveGLWindow() != nullptr);
//
//	//menuEdit->setEnabled(atLeastOneEntity);
//	//menuTools->setEnabled(atLeastOneEntity);
//
//	m_UI->actionTracePolyline->setEnabled(!dbIsEmpty);
//	m_UI->actionZoomAndCenter->setEnabled(atLeastOneEntity && activeWindow);
//	m_UI->actionSave->setEnabled(atLeastOneEntity);
//	m_UI->actionClone->setEnabled(atLeastOneEntity);
//	m_UI->actionDelete->setEnabled(atLeastOneEntity);
//	m_UI->actionExportCoordToSF->setEnabled(atLeastOneEntity);
//	m_UI->actionSegment->setEnabled(atLeastOneEntity && activeWindow);
//	m_UI->actionTranslateRotate->setEnabled(atLeastOneEntity && activeWindow);
//	m_UI->actionShowDepthBuffer->setEnabled(atLeastOneGBLSensor);
//	m_UI->actionExportDepthBuffer->setEnabled(atLeastOneGBLSensor);
//	m_UI->actionComputePointsVisibility->setEnabled(atLeastOneGBLSensor);
//	m_UI->actionResampleWithOctree->setEnabled(atLeastOneCloud);
//	m_UI->actionApplyScale->setEnabled(atLeastOneCloud || atLeastOneMesh || atLeastOnePolyline);
//	m_UI->actionApplyTransformation->setEnabled(atLeastOneEntity);
//	m_UI->actionComputeOctree->setEnabled(atLeastOneCloud || atLeastOneMesh);
//	m_UI->actionComputeNormals->setEnabled(atLeastOneCloud || atLeastOneMesh);
//	m_UI->actionChangeColorLevels->setEnabled(atLeastOneCloud || atLeastOneMesh);
//	m_UI->actionEditGlobalShiftAndScale->setEnabled(atLeastOneCloud || atLeastOneMesh || atLeastOnePolyline);
//	m_UI->actionCrop->setEnabled(atLeastOneCloud || atLeastOneMesh);
//	m_UI->actionSetUniqueColor->setEnabled(atLeastOneEntity/*atLeastOneCloud || atLeastOneMesh*/); //DGM: we can set color to a group now!
//	m_UI->actionColorize->setEnabled(atLeastOneEntity/*atLeastOneCloud || atLeastOneMesh*/); //DGM: we can set color to a group now!
//	m_UI->actionDeleteScanGrid->setEnabled(atLeastOneGrid);
//
//	m_UI->actionScalarFieldFromColor->setEnabled(atLeastOneEntity && atLeastOneColor);
//	m_UI->actionComputeMeshAA->setEnabled(atLeastOneCloud);
//	m_UI->actionComputeMeshLS->setEnabled(atLeastOneCloud);
//	m_UI->actionMeshScanGrids->setEnabled(atLeastOneGrid);
//	//actionComputeQuadric3D->setEnabled(atLeastOneCloud);
//	m_UI->actionComputeBestFitBB->setEnabled(atLeastOneEntity);
//	m_UI->actionComputeGeometricFeature->setEnabled(atLeastOneCloud);
//	m_UI->actionRemoveDuplicatePoints->setEnabled(atLeastOneCloud);
//	m_UI->actionFitPlane->setEnabled(atLeastOneEntity);
//	m_UI->actionFitSphere->setEnabled(atLeastOneCloud);
//	m_UI->actionLevel->setEnabled(atLeastOneEntity);
//	m_UI->actionFitFacet->setEnabled(atLeastOneEntity);
//	m_UI->actionFitQuadric->setEnabled(atLeastOneCloud);
//	m_UI->actionSubsample->setEnabled(atLeastOneCloud);
//
//	m_UI->actionSNETest->setEnabled(atLeastOneCloud);
//	m_UI->actionExportCloudInfo->setEnabled(atLeastOneEntity);
//	m_UI->actionExportPlaneInfo->setEnabled(atLeastOneEntity);
//
//	m_UI->actionFilterByValue->setEnabled(atLeastOneSF);
//	m_UI->actionConvertToRGB->setEnabled(atLeastOneSF);
//	m_UI->actionConvertToRandomRGB->setEnabled(atLeastOneSF);
//	m_UI->actionRenameSF->setEnabled(atLeastOneSF);
//	m_UI->actionAddIdField->setEnabled(atLeastOneCloud);
//	m_UI->actionComputeStatParams->setEnabled(atLeastOneSF);
//	m_UI->actionComputeStatParams2->setEnabled(atLeastOneSF);
//	m_UI->actionShowHistogram->setEnabled(atLeastOneSF);
//	m_UI->actionGaussianFilter->setEnabled(atLeastOneSF);
//	m_UI->actionBilateralFilter->setEnabled(atLeastOneSF);
//	m_UI->actionDeleteScalarField->setEnabled(atLeastOneSF);
//	m_UI->actionDeleteAllSF->setEnabled(atLeastOneSF);
//	m_UI->actionMultiplySF->setEnabled(/*TODO: atLeastOneSF*/false);
//	m_UI->actionSFGradient->setEnabled(atLeastOneSF);
//	m_UI->actionSetSFAsCoord->setEnabled(atLeastOneSF && atLeastOneCloud);
//	m_UI->actionInterpolateSFs->setEnabled(atLeastOneCloud || atLeastOneMesh);
//
//	m_UI->actionSamplePointsOnMesh->setEnabled(atLeastOneMesh);
//	m_UI->actionMeasureMeshSurface->setEnabled(atLeastOneMesh);
//	m_UI->actionMeasureMeshVolume->setEnabled(atLeastOneMesh);
//	m_UI->actionFlagMeshVertices->setEnabled(atLeastOneMesh);
//	m_UI->actionSmoothMeshLaplacian->setEnabled(atLeastOneMesh);
//	m_UI->actionConvertTextureToColor->setEnabled(atLeastOneMesh);
//	m_UI->actionSubdivideMesh->setEnabled(atLeastOneMesh);
//	m_UI->actionDistanceToBestFitQuadric3D->setEnabled(atLeastOneCloud);
//	m_UI->actionDistanceMap->setEnabled(atLeastOneMesh || atLeastOneCloud);
//
//	m_UI->menuMeshScalarField->setEnabled(atLeastOneSF && atLeastOneMesh);
//	//actionSmoothMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);
//	//actionEnhanceMeshSF->setEnabled(atLeastOneSF && atLeastOneMesh);
//
//	m_UI->actionOrientNormalsMST->setEnabled(atLeastOneCloud && atLeastOneNormal);
//	m_UI->actionOrientNormalsFM->setEnabled(atLeastOneCloud && atLeastOneNormal);
//	m_UI->actionClearNormals->setEnabled(atLeastOneNormal);
//	m_UI->actionInvertNormals->setEnabled(atLeastOneNormal);
//	m_UI->actionConvertNormalToHSV->setEnabled(atLeastOneNormal);
//	m_UI->actionConvertNormalToDipDir->setEnabled(atLeastOneNormal);
//	m_UI->actionClearColor->setEnabled(atLeastOneColor);
//	m_UI->actionRGBToGreyScale->setEnabled(atLeastOneColor);
//	m_UI->actionEnhanceRGBWithIntensities->setEnabled(atLeastOneColor);
//
//	// == 1
//	bool exactlyOneEntity = (selInfo.selCount == 1);
//	bool exactlyOneGroup = (selInfo.groupCount == 1);
//	bool exactlyOneCloud = (selInfo.cloudCount == 1);
//	bool exactlyOneMesh = (selInfo.meshCount == 1);
//	bool exactlyOneSF = (selInfo.sfCount == 1);
//	bool exactlyOneSensor = (selInfo.sensorCount == 1);
//	bool exactlyOneCameraSensor = (selInfo.cameraSensorCount == 1);
//
//	m_UI->actionConvertPolylinesToMesh->setEnabled(atLeastOnePolyline || exactlyOneGroup);
//	m_UI->actionSamplePointsOnPolyline->setEnabled(atLeastOnePolyline);
//	m_UI->actionMeshTwoPolylines->setEnabled(selInfo.selCount == 2 && selInfo.polylineCount == 2);
//	m_UI->actionCreateSurfaceBetweenTwoPolylines->setEnabled(m_UI->actionMeshTwoPolylines->isEnabled()); //clone of actionMeshTwoPolylines
//	m_UI->actionModifySensor->setEnabled(exactlyOneSensor);
//	m_UI->actionComputeDistancesFromSensor->setEnabled(atLeastOneCameraSensor || atLeastOneGBLSensor);
//	m_UI->actionComputeScatteringAngles->setEnabled(exactlyOneSensor);
//	m_UI->actionViewFromSensor->setEnabled(exactlyOneSensor);
//	m_UI->actionCreateGBLSensor->setEnabled(atLeastOneCloud);
//	m_UI->actionCreateCameraSensor->setEnabled(selInfo.selCount <= 1); //free now
//	m_UI->actionProjectUncertainty->setEnabled(exactlyOneCameraSensor);
//	m_UI->actionCheckPointsInsideFrustum->setEnabled(exactlyOneCameraSensor);
//	m_UI->actionLabelConnectedComponents->setEnabled(atLeastOneCloud);
//	m_UI->actionSORFilter->setEnabled(atLeastOneCloud);
//	m_UI->actionNoiseFilter->setEnabled(atLeastOneCloud);
//	m_UI->actionUnroll->setEnabled(exactlyOneEntity);
//	m_UI->actionStatisticalTest->setEnabled(exactlyOneEntity && exactlyOneSF);
//	m_UI->actionAddConstantSF->setEnabled(exactlyOneCloud || exactlyOneMesh);
//	m_UI->actionEditGlobalScale->setEnabled(exactlyOneCloud || exactlyOneMesh);
//	m_UI->actionComputeKdTree->setEnabled(exactlyOneCloud || exactlyOneMesh);
//	m_UI->actionShowWaveDialog->setEnabled(exactlyOneCloud);
//	m_UI->actionCompressFWFData->setEnabled(atLeastOneCloud);
//
//	m_UI->actionKMeans->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);
//	m_UI->actionFrontPropagation->setEnabled(/*TODO: exactlyOneEntity && exactlyOneSF*/false);
//
//	//actionCreatePlane->setEnabled(true);
//	m_UI->actionEditPlane->setEnabled(selInfo.planeCount == 1);
//
//	m_UI->actionFindBiggestInnerRectangle->setEnabled(exactlyOneCloud);
//
//	m_UI->menuActiveScalarField->setEnabled((exactlyOneCloud || exactlyOneMesh) && selInfo.sfCount > 0);
//	m_UI->actionCrossSection->setEnabled(atLeastOneCloud || atLeastOneMesh || (selInfo.groupCount != 0));
//	m_UI->actionExtractSections->setEnabled(atLeastOneCloud);
//	m_UI->actionRasterize->setEnabled(exactlyOneCloud);
//	m_UI->actionCompute2HalfDimVolume->setEnabled(selInfo.cloudCount == selInfo.selCount && selInfo.cloudCount >= 1 && selInfo.cloudCount <= 2); //one or two clouds!
//
//	m_UI->actionPointListPicking->setEnabled(exactlyOneCloud || exactlyOneMesh);
//
//	// == 2
//	bool exactlyTwoEntities = (selInfo.selCount == 2);
//	bool exactlyTwoClouds = (selInfo.cloudCount == 2);
//	//bool exactlyTwoSF = (selInfo.sfCount == 2);
//
//	m_UI->actionRegister->setEnabled(exactlyTwoEntities);
//	m_UI->actionInterpolateColors->setEnabled(exactlyTwoEntities && atLeastOneColor);
//	m_UI->actionPointPairsAlign->setEnabled(exactlyOneEntity || exactlyTwoEntities);
//	m_UI->actionAlign->setEnabled(exactlyTwoEntities); //Aurelien BEY le 13/11/2008
//	m_UI->actionCloudCloudDist->setEnabled(exactlyTwoClouds);
//	m_UI->actionCloudMeshDist->setEnabled(exactlyTwoEntities && atLeastOneMesh);
//	m_UI->actionCPS->setEnabled(exactlyTwoClouds);
//	m_UI->actionScalarFieldArithmetic->setEnabled(exactlyOneEntity && atLeastOneSF);
//
//	//>1
//	bool atLeastTwoEntities = (selInfo.selCount > 1);
//
//	m_UI->actionMerge->setEnabled(atLeastTwoEntities);
//	m_UI->actionMatchBBCenters->setEnabled(atLeastTwoEntities);
//	m_UI->actionMatchScales->setEnabled(atLeastTwoEntities);
//
//	//standard plugins
//	m_pluginUIManager->handleSelectionChanged();
//}

void MainWindow::echoMouseWheelRotate(float wheelDelta_deg)
{
	/*if (!m_UI->actionEnableCameraLink->isChecked())
	return;*/

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->onWheelEvent(wheelDelta_deg);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

//void MainWindow::echoCameraDisplaced(float ddx, float ddy)
//{
//	if (!m_UI->actionEnableCameraLink->isChecked())
//		return;
//
//	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
//	if (!sendingWindow)
//		return;
//
//	for ( QMdiSubWindow *window : m_mdiArea->subWindowList() )
//	{
//		ccGLWindow *child = GLWindowFromWidget(window->widget());
//		if (child != sendingWindow)
//		{
//			child->blockSignals(true);
//			child->moveCamera(ddx, ddy, 0.0f);
//			child->blockSignals(false);
//			child->redraw();
//		}
//	}
//}

void MainWindow::echoBaseViewMatRotation(const ccGLMatrixd& rotMat)
{
	/*if (!m_UI->actionEnableCameraLink->isChecked())
	return;*/

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->rotateBaseViewMat(rotMat);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::echoCameraPosChanged(const CCVector3d& P)
{
	/*if (!m_UI->actionEnableCameraLink->isChecked())
	return;*/

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;


	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->setCameraPos(P);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

void MainWindow::echoPivotPointChanged(const CCVector3d& P)
{
	/*if (!m_UI->actionEnableCameraLink->isChecked())
	return;*/

	ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
	if (!sendingWindow)
		return;

	for (QMdiSubWindow *window : m_mdiArea->subWindowList())
	{
		ccGLWindow *child = GLWindowFromWidget(window->widget());
		if (child != sendingWindow)
		{
			child->blockSignals(true);
			child->setPivotPoint(P);
			child->blockSignals(false);
			child->redraw();
		}
	}
}

// void MainWindow::echoPixelSizeChanged(float pixelSize)
// {
//	 if (!m_UI->actionEnableCameraLink->isChecked())
//		 return;
//
//	 ccGLWindow* sendingWindow = dynamic_cast<ccGLWindow*>(sender());
//	 if (!sendingWindow)
//		 return;
//
//	 for ( QMdiSubWindow *window : m_mdiArea->subWindowList() )
//	 {
//		 ccGLWindow *child = GLWindowFromWidget(window->widget());
//		 if (child != sendingWindow)
//		 {
//			 child->blockSignals(true);
//			 child->setPixelSize(pixelSize);
//			 child->blockSignals(false);
//			 child->redraw();
//		 }
//	 }
// }
//
//void MainWindow::dispToConsole(QString message, ConsoleMessageLevel level/*=STD_CONSOLE_MESSAGE*/)
//{
//	switch(level)
//	{
//	case STD_CONSOLE_MESSAGE:
//		ccConsole::Print(message);
//		break;
//	case WRN_CONSOLE_MESSAGE:
//		ccConsole::Warning(message);
//		break;
//	case ERR_CONSOLE_MESSAGE:
//		ccConsole::Error(message);
//		break;
//	}
//}
//
//void MainWindow::doActionLoadShader() //TODO
//{
//	ccConsole::Error("Not yet implemented! Sorry ...");
//}
//
//void MainWindow::doActionKMeans()//TODO
//{
//	ccConsole::Error("Not yet implemented! Sorry ...");
//}
//
//void MainWindow::doActionFrontPropagation() //TODO
//{
//	ccConsole::Error("Not yet implemented! Sorry ...");
//}
//
///************** STATIC METHODS ******************/
//
MainWindow* MainWindow::TheInstance()
{
	if (!s_instance)
		s_instance = new MainWindow();
	return s_instance;
}

void MainWindow::DestroyInstance()
{
	delete s_instance;
	s_instance = nullptr;
}

void MainWindow::GetGLWindows(std::vector<ccGLWindow*>& glWindows)
{
	const QList<QMdiSubWindow*> windows; //= TheInstance()->m_mdiArea->subWindowList();

	if (windows.empty())
		return;

	glWindows.clear();
	glWindows.reserve(windows.size());

	for (QMdiSubWindow *window : windows)
	{
		glWindows.push_back(GLWindowFromWidget(window->widget()));
	}
}

ccGLWindow* MainWindow::GetActiveGLWindow()
{
	return TheInstance()->getActiveGLWindow();
}

ccGLWindow* MainWindow::GetGLWindow(const QString& title)
{
	const QList<QMdiSubWindow *> windows;// = TheInstance()->m_mdiArea->subWindowList();

	if (windows.empty())
		return nullptr;

	for (QMdiSubWindow *window : windows)
	{
		ccGLWindow* win = GLWindowFromWidget(window->widget());
		if (win->windowTitle() == title)
			return win;
	}

	return nullptr;
}

void MainWindow::RefreshAllGLWindow(bool only2D/*=false*/)
{
	TheInstance()->refreshAll(only2D);
}

void MainWindow::UpdateUI()
{
	//TheInstance()->updateUI();
}

//ccDBRoot* MainWindow::db()
//{
//	return m_ccRoot;
//}
//
//void MainWindow::addEditPlaneAction( QMenu &menu ) const
//{
//	menu.addAction( m_UI->actionEditPlane );
//}
//
//ccHObject* MainWindow::dbRootObject()
//{
//	return (m_ccRoot ? m_ccRoot->getRootEntity() : nullptr);
//}
//
//ccUniqueIDGenerator::Shared MainWindow::getUniqueIDGenerator()
//{
//	return ccObject::GetUniqueIDGenerator();
//}

void MainWindow::createGLWindow(ccGLWindow*& window, QWidget*& widget) const
{
	bool stereoMode = QSurfaceFormat::defaultFormat().stereo();

	CreateGLWindow(window, widget, stereoMode);
	assert(window && widget);
}

void MainWindow::destroyGLWindow(ccGLWindow* view3D) const
{
	if (view3D)
	{
		view3D->setParent(0);
		delete view3D;
	}
}

//MainWindow::ccHObjectContext MainWindow::removeObjectTemporarilyFromDBTree(ccHObject* obj)
//{
//	ccHObjectContext context;
//
//	assert(obj);
//	if (!m_ccRoot || !obj)
//		return context;
//
//	//mandatory (to call putObjectBackIntoDBTree)
//	context.parent = obj->getParent();
//
//	//remove the object's dependency to its father (in case it undergoes "severe" modifications)
//	if (context.parent)
//	{
//		context.parentFlags = context.parent->getDependencyFlagsWith(obj);
//		context.childFlags = obj->getDependencyFlagsWith(context.parent);
//
//		context.parent->removeDependencyWith(obj);
//		obj->removeDependencyWith(context.parent);
//	}
//
//	m_ccRoot->removeElement(obj);
//
//	return context;
//}
//
//void MainWindow::putObjectBackIntoDBTree(ccHObject* obj, const ccHObjectContext& context)
//{
//	assert(obj);
//	if (!obj || !m_ccRoot)
//		return;
//
//	if (context.parent)
//	{
//		context.parent->addChild(obj,context.parentFlags);
//		obj->addDependency(context.parent,context.childFlags);
//	}
//
//	//DGM: we must call 'notifyGeometryUpdate' as any call to this method
//	//while the object was temporarily 'cut' from the DB tree were
//	//ineffective!
//	obj->notifyGeometryUpdate();
//
//	m_ccRoot->addElement(obj,false);
//}
//
//void MainWindow::doActionGlobalShiftSeetings()
//{
//	QDialog dialog(this);
//	Ui_GlobalShiftSettingsDialog ui;
//	ui.setupUi(&dialog);
//
//	ui.maxAbsCoordSpinBox->setValue(static_cast<int>(log10(ccGlobalShiftManager::MaxCoordinateAbsValue())));
//	ui.maxAbsDiagSpinBox->setValue(static_cast<int>(log10(ccGlobalShiftManager::MaxBoundgBoxDiagonal())));
//
//	if (!dialog.exec())
//	{
//		return;
//	}
//
//	double maxAbsCoord = pow(10.0, static_cast<double>(ui.maxAbsCoordSpinBox->value()));
//	double maxAbsDiag = pow(10.0, static_cast<double>(ui.maxAbsDiagSpinBox->value()));
//
//	ccGlobalShiftManager::SetMaxCoordinateAbsValue(maxAbsCoord);
//	ccGlobalShiftManager::SetMaxBoundgBoxDiagonal(maxAbsDiag);
//
//	ccLog::Print(QString("[Global Shift] Max abs. coord = %1 / max abs. diag = %2")
//		.arg(ccGlobalShiftManager::MaxCoordinateAbsValue(), 0, 'e', 0)
//		.arg(ccGlobalShiftManager::MaxBoundgBoxDiagonal(), 0, 'e', 0));
//
//	//save to persistent settings
//	{
//		QSettings settings;
//		settings.beginGroup(ccPS::GlobalShift());
//		settings.setValue(ccPS::MaxAbsCoord(), maxAbsCoord);
//		settings.setValue(ccPS::MaxAbsDiag(), maxAbsDiag);
//		settings.endGroup();
//	}
//}
//
//void MainWindow::doActionCompressFWFData()
//{
//	for ( ccHObject *entity : getSelectedEntities() )
//	{
//		if (!entity || !entity->isKindOf(CC_TYPES::POINT_CLOUD))
//		{
//			continue;
//		}
//
//		ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
//		cloud->compressFWFData();
//	}
//}
//
//void MainWindow::doActionShowWaveDialog()
//{
//	if (!haveSelection())
//		return;
//
//	ccHObject* entity = haveOneSelection() ? m_selectedEntities[0] : nullptr;
//	if (!entity || !entity->isKindOf(CC_TYPES::POINT_CLOUD))
//	{
//		ccConsole::Error("Select one point cloud!");
//		return;
//	}
//
//	ccPointCloud* cloud = static_cast<ccPointCloud*>(entity);
//	if (!cloud->hasFWF())
//	{
//		ccConsole::Error("Cloud has no associated waveform information");
//		return;
//	}
//
//	ccWaveDialog* wDlg = new ccWaveDialog(cloud, m_pickingHub, this);
//	wDlg->setAttribute(Qt::WA_DeleteOnClose);
//	wDlg->setModal(false);
//	wDlg->show();
//}
//
//void MainWindow::doActionCreatePlane()
//{
//	ccPlaneEditDlg* peDlg = new ccPlaneEditDlg(m_pickingHub, this);
//	peDlg->show();
//}
//
//void MainWindow::doActionEditPlane()
//{
//	if (!haveSelection())
//	{
//		assert(false);
//		return;
//	}
//
//	ccPlane* plane = ccHObjectCaster::ToPlane(m_selectedEntities.front());
//	if (!plane)
//	{
//		assert(false);
//		return;
//	}
//
//	ccPlaneEditDlg* peDlg = new ccPlaneEditDlg(m_pickingHub, this);
//	peDlg->initWithPlane(plane);
//	peDlg->show();
//}

