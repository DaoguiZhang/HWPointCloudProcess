#include "PolygonWindow.h"
namespace HW {
	PolygonWindow::PolygonWindow(QWidget * parent)
	{
		//this->CreateImageWindow();
		this->grabKeyboard();
		this->CreatePolygonGLWindow();
		std::cout << "create polygonglwindow" << std::endl;
		this->CreateButton();
		std::cout << "create button" << std::endl;
		this->CreateConnect();
		std::cout << "create connect" << std::endl;
	}

	PolygonWindow::~PolygonWindow()
	{
	}

	void PolygonWindow::CreateImageWindow()
	{
		image_window_ = new ImageWindow();
		image_window_->resize(1148, 862);
		image_window_->setWindowTitle("ImageWindow");
		image_window_->show();
	}

	void PolygonWindow::CreatePolygonGLWindow()
	{
		QGLFormat glFormat;
		glFormat.setSampleBuffers(true);
		glFormat.setSamples(16);
		polygon_openGLWindow_ = new PolygonGLWindow();
		polygon_openGLWindow_->move(300, 10);
		polygon_openGLWindow_->setFormat(glFormat);
		//this->setCentralWidget(polygon_openGLWindow_);
		polygon_openGLWindow_->resize(1280, 960);
		polygon_openGLWindow_->setWindowTitle("PolygonGLWindow");
		polygon_openGLWindow_->setFocusPolicy(Qt::TabFocus);
		//polygon_openGLWindow_->show();
	}

	void PolygonWindow::CreateButton()
	{
		createDisplayModeGroupBox();
		createButtonGroupBox();
		createParamsGroupBox();
		createSmallPolygonGroupBox();

		mainLayout_ = new QVBoxLayout();
		mainLayout_->addWidget(displayModeGroupBox);
		mainLayout_->addWidget(buttonGroupBox);
		mainLayout_->addWidget(buttonCamsOptiGroupBox);
		mainLayout_->addWidget(paramsGroupBox);
		mainLayout_->addWidget(smallPolygonGroupBox);
		
		//ratio_snap_label = new QLabel("ratio_snap");
		//mainLayout_->addWidget(ratio_snap_label);
		//ratio_snap_spinbox = new QDoubleSpinBox();
		//ratio_snap_spinbox->setRange(0, 20);  // 范围
		//ratio_snap_spinbox->setDecimals(2);  // 精度
		//ratio_snap_spinbox->setSingleStep(0.01);
		//ratio_snap_spinbox->setValue(1.0);
		//mainLayout_->addWidget(ratio_snap_spinbox);

		//ratio_ref_label = new QLabel("ratio_ref");
		//mainLayout_->addWidget(ratio_ref_label);
		//ratio_ref_spinbox = new QDoubleSpinBox();
		//ratio_ref_spinbox->setRange(0, 20);  // 范围
		//ratio_ref_spinbox->setDecimals(2);  // 精度
		//ratio_ref_spinbox->setSingleStep(0.01);
		//ratio_ref_spinbox->setValue(0.5);
		//mainLayout_->addWidget(ratio_ref_spinbox);

		//ratio_ortho_label = new QLabel("ratio_ortho_label");
		//mainLayout_->addWidget(ratio_ortho_label);
		//ratio_ortho_spinbox = new QDoubleSpinBox();
		//ratio_ortho_spinbox->setRange(0, 20);  // 范围
		//ratio_ortho_spinbox->setDecimals(3);  // 精度
		//ratio_ortho_spinbox->setSingleStep(0.001);
		//ratio_ortho_spinbox->setValue(0.001);
		//mainLayout_->addWidget(ratio_ortho_spinbox);

		//ratio_cur_label = new QLabel("ratio_cur");
		//mainLayout_->addWidget(ratio_cur_label);
		//ratio_cur_spinbox = new QDoubleSpinBox();
		//ratio_cur_spinbox->setRange(0, 20);  // 范围
		//ratio_cur_spinbox->setDecimals(3);  // 精度
		//ratio_cur_spinbox->setSingleStep(0.001);
		//ratio_cur_spinbox->setValue(0.1);
		//mainLayout_->addWidget(ratio_cur_spinbox);
		this->setLayout(mainLayout_);
	}

	void PolygonWindow::CreateConnect()
	{
		connect(display_polygons_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isDisplayPolygonsState(int)));
		connect(display_labels_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isDisplaySemanticLabels(int)));
		connect(display_deleted_polygons_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isDisplayDeletedPolygonsState(int)));
		connect(display_point_clouds_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isDisplayPointCloudsState(int)));
		connect(select_polygon_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isSelectPolygon(int)));
		connect(select_small_polygons_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(selectSmallPolygons(int)));
		connect(select_point_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isSelectPoint(int)));
		//connect(edit_point_state_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(setEditPlanePointState(int)));
		//connect(add_point_state_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(setAddPlanePointState(int)));
		
		connect(edit_point_state_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(SetEditPlanePnt3dState(int)));
		connect(add_point_state_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(SetAddPlanePnt3dState(int)));

		connect(select_edge_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isSelectEdge(int)));
		connect(refine_polygon_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isRefinePolygon(int)));
		connect(select_outdoor_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isOutDoorFlag(int)));
		connect(select_new_polygon_vertices_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isSelectNewPolygonVertices(int)));
		connect(split_polygon_checkbox_, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isSplitPolygon(int)));

		connect(plane_unprocess_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(setUnprocessedPlane()));
		connect(polygon_snapping_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonSnapping()));
		connect(polygon_snapping_button_new_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonSnappingWithImgs()));
		connect(polygon_expand_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonExpanding()));
		connect(polygon_expand_seam_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonExpandingSeam()));
		connect(polygon_expand_seam_t2s_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonExpandingSeamBiDirect()));
		connect(optimize_expand_pnt_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonOptimizePolyPnts()));
		connect(point_moving_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPointMoving()));
		connect(point_creating_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPointCreating()));
		connect(polygon_create_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonCreating()));
		connect(polygon_create_button1_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonCreating1()));
		connect(polygon_create_button2_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonCreating2()));
		connect(polygon_create_button3_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonCreating3()));
		connect(polygon_create_button4_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonCreating4()));
		//fixme
		connect(polygon_perpendicular_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonPerpendicular()));
		connect(polygon_inverse_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonInversing()));
		connect(polygon_delete_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonDeleting()));
		connect(point_delete_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPointDeleting()));
		connect(polygon_remove_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doPolygonRemoving()));
		connect(open_image_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(loadImage()));
		connect(open_all_pointcloud_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(loadAllPointCloud()));
		connect(assign_polygon_to_pointcloud_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doAssignPolygonsToPointsCloudAction()));
		connect(save_assigned_pointcloud_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doSaveAssignedPolygonsPntsAction()));
		connect(save_unassigned_pointcloud_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doUnSaveAssignedPolygonsPntsAction()));
		connect(adjust_polygon_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(adustPolygonByLine()));
		connect(back_to_last_state_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(backToLastState()));
		connect(forward_to_next_state_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(forwardToNextState()));
		connect(run_get_manhanttan_polygons_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doGetManhattanPlanesPolygonsAction()));
		connect(save_cloud_points_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(saveCloudPoints()));
		connect(save_refined_tris_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(saveRefinedPlanesTris()));
		connect(save_project_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(saveProject()));
		connect(save_manhanttan_polygons_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(SaveManhattanPlanesPolygonsIntoObjAction()));
		connect(filter_cloud_points_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(filterCloudPoints()));
		connect(auto_expand_button, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(autoExpanding()));
		connect(save_labels_button, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(saveLabels()));
		connect(r_max_spinbox, SIGNAL(valueChanged(double)), polygon_openGLWindow_, SLOT(setRMax(double)));
		connect(r_tri_len_spinbox_, SIGNAL(valueChanged(double)), polygon_openGLWindow_, SLOT(setTriLenMax(double)));
		connect(r_tri_degree_spinbox_, SIGNAL(valueChanged(double)), polygon_openGLWindow_, SLOT(setTriDegreeMax(double)));
		connect(sampling_density_spinbox, SIGNAL(valueChanged(int)), polygon_openGLWindow_, SLOT(setSampleDensity(int)));
		connect(auto_fuse_nearby_points_button, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(autoFuseNearbyPoints()));
		connect(auto_delete_collinear_points_button, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(autoDeleteCollinearPoints()));
		connect(auto_split_polygon_button, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(autoSplitPolygonModel()));
		connect(split_curpoly_with_points_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doCurrentPolygonSplitProcess()));
		connect(merge_cur_selected_polygons_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doMergeCurrentPolygonsProcess()));
		connect(optimize_all_polygons_with_pnts_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doOptimizeAllPolygonsWithPlanesPntsProcess()));
		connect(optimize_all_polygons_contour_with_pnts_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doOptimizeAllPolygonsContourBasedOnPntsPlaneCloudProcess()));
		connect(aligin_polygons_normals_to_original_pnts_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(doAlignPolygonsNormalsWithPntsCloudNormalsProcess()));
		connect(render_polygon_depth_button, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(renderCurrentDepth()));
		connect(run_dso_optimization_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(RunDSOProcess()));
		connect(save_images_polygons_lines_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(SaveImagesLinesAndPolygonsLinesProcessAction()));

		/*-------------------------run bundle adjustment event--------------------------*/
		connect(convert_data2hw_scenes_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(ConvertHWData2HWScenesElementLinesAction()));
		connect(use_lsd_opencv_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(UseImagesLSDOpencvAction()));
		connect(load_lines_pnts_matches_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(LoadImagesLinesPntsPairsAction()));
		connect(convert_images_labels2hw_scenes_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(ConvertRenderPlanesLabels2HWScenesElementLabelsAction()));
		connect(run_lines_matches_button_lsd_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(RunHWImagesPairsFromCamPosesAndPolygonsLSDAction()));
		connect(run_lines_matches_button_lines_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(RunHWImagesPairsFromCamPosesAndPolygonsLinesAction()));
		connect(run_lines_matches_button_lines_pnts_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(RunHWImagePairsFromCamPosesAndPolygonsLinesPntsAction()));
		//ConvertRenderPlanesLabels2HWScenesElementLabelsAction
		connect(convertdata2bundle_scenes_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(ConvertHWDataToHWBundleSceneDataAction()));
		//convertscenedata2bundle_scenes_cams_button_ ConvertHWSceneDataToHWBundleSceneDataOnlyCamsAction
		connect(convertscenedata2bundle_scenes_cams_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(ConvertHWSceneDataToHWBundleSceneDataOnlyCamsAction()));
		connect(save_images_lines_matches_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(SaveImagesLinesPairsIntoLogsAction()));
		//save_images_lines_matches_button_;save_images_names_matches_button_
		connect(save_images_names_matches_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(SaveImagesNamesPairsIntoTxtAction()));
		connect(run_bundle_adjustment_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(RunHWBundleSceneProcessAction()));
		connect(run_bundle_adjustment_cams_only_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(RunHWBundleCamsOnlySceneProcessAction()));

		//run_bundle_adjustment_cams_only_button_;RunHWBundleCamsOnlySceneProcessAction
		connect(save_bundle_result_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(SaveBundleCamsParamsIntoCamsAction()));

		/*-------------------------end bundle adjustment event--------------------------*/

		//isSelectMinHeightState(int state);
		//isSelectMinWidthState(int state);
		//isSelectMinCornerPointsState(int state);
		//isSelectMinAreaState(int state);
		//isSelectMinAreaByVertexNumState(int state);
		//isSelectMinAreaByBoundingboxAreaState(int state)
		connect(select_min_height_checkbox, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isSelectMinHeightState(int)));
		connect(select_min_width_checkbox, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isSelectMinWidthState(int)));
		connect(select_min_corner_points_checkbox, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isSelectMinCornerPointsState(int)));
		connect(select_min_area_checkbox, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isSelectMinAreaState(int)));
		connect(select_min_area_by_vertices_checkbox, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isSelectMinAreaByVertexNumState(int)));
		connect(select_min_area_by_boundingbox_checkbox, SIGNAL(stateChanged(int)), polygon_openGLWindow_, SLOT(isSelectMinAreaByBoundingboxAreaState(int)));

		connect(min_height_spinbox, SIGNAL(valueChanged(double)), polygon_openGLWindow_, SLOT(setMinHeight(double)));
		connect(min_width_spinbox, SIGNAL(valueChanged(double)), polygon_openGLWindow_, SLOT(setMinWidth(double)));
		connect(min_corner_points_spinbox, SIGNAL(valueChanged(int)), polygon_openGLWindow_, SLOT(setMinCornerPoints(int)));
		connect(min_area_spinbox, SIGNAL(valueChanged(double)), polygon_openGLWindow_, SLOT(setMinArea(double)));
		connect(min_area_by_vertices_spinbox, SIGNAL(valueChanged(double)), polygon_openGLWindow_, SLOT(setMinAreaByVertexNum(double)));
		connect(min_area_by_boundingbox_spinbox, SIGNAL(valueChanged(double)), polygon_openGLWindow_, SLOT(setMinAreaByBoundingboxArea(double)));

		connect(set_label_spinbox, SIGNAL(valueChanged(int)), polygon_openGLWindow_, SLOT(setLabel(int)));
		connect(sampling_density_spinbox, SIGNAL(valueChanged(int)), polygon_openGLWindow_, SLOT(setSamplingDensity(int)));
        connect(import_scene_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(ImportSceneElementAction()));
		connect(import_hwscene_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(ImportHWSceneElementsAction()));
		connect(import_images_planes_labels_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(ImportImagesPlanesLabelsAction()));

		connect(import_images_lines_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(ImportImagesLinesPairsFromTxtAction()));

		connect(import_reality_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(ImportBundleElementsAction()));
		connect(Save_aligned_button_, SIGNAL(clicked()), polygon_openGLWindow_, SLOT(SaveAlignedCamsFromBundleElementsAction()));
		//SaveAlignedCamsFromBundleElementsAction
		//connect(ratio_snap_spinbox, SIGNAL(valueChanged(double)), polygon_openGLWindow_, SLOT(setRatioSnap(ratio_snap_spinbox->value())));
		//connect(ratio_ref_spinbox, SIGNAL(valueChanged(double)), polygon_openGLWindow_, SLOT(setRatioRef(ratio_ref_spinbox->value())));
		//connect(ratio_ortho_spinbox, SIGNAL(valueChanged(double)), polygon_openGLWindow_, SLOT(setRatioOrtho(ratio_ortho_spinbox->value())));
		//connect(ratio_cur_spinbox, SIGNAL(valueChanged(double)), polygon_openGLWindow_, SLOT(setRatioCur(ratio_cur_spinbox->value())));

		polygon_openGLWindow_->setRMax(r_max_spinbox->value());
		//polygon_openGLWindow_->setRatioSnap(ratio_snap_spinbox->value());
		//polygon_openGLWindow_->setRatioRef(ratio_ref_spinbox->value());
		//polygon_openGLWindow_->setRatioOrtho(ratio_ortho_spinbox->value());
		//polygon_openGLWindow_->setRatioCur(ratio_cur_spinbox->value());
	}

	void PolygonWindow::SetEditPlanePnt3dState(int state)
	{
		polygon_openGLWindow_->setEditPlanePointState(state);
		if (state == Qt::Checked)
		{
			add_point_state_checkbox_->setChecked(false);
			//polygon_openGLWindow_->setAddPlanePointState(Qt::Unchecked);
		}
	}

	void PolygonWindow::SetAddPlanePnt3dState(int state)
	{
		polygon_openGLWindow_->setAddPlanePointState(state);
		if (state == Qt::Checked)
		{
			edit_point_state_checkbox_->setChecked(false);
			//polygon_openGLWindow_->setEditPlanePointState(Qt::Unchecked);
		}
	}

	void PolygonWindow::SetData(HWPointCloud* in_pointcloud, std::vector<HWPlane*>& planes_vec)
	{
		std::cout << "planes_vec_: " << planes_vec.size() << "\n";
		polygon_openGLWindow_->SetData(in_pointcloud, planes_vec);
	}
	void PolygonWindow::SetData(HWPolygon* polygon)
	{
		polygon_openGLWindow_->SetData(polygon);
	}

	void PolygonWindow::SetAllPointCloud(HWPointCloud* in_pointcloud)
	{
		if (polygon_openGLWindow_)
		{
			polygon_openGLWindow_->SetAllPointCloud(in_pointcloud);
			std::cerr << "polygon_openGLWindow_: start to SetOriginPointCloud2GLBuffer" << std::endl;
			polygon_openGLWindow_->SetOriginPointCloud2GLBuffer();
		}	
	}

	const PolygonGLWindow* PolygonWindow::GetPolygonGLWindowPolygons()
	{
		return polygon_openGLWindow_;
	}

	void PolygonWindow::createDisplayModeGroupBox()
	{
		displayModeGroupBox = new QGroupBox(tr("Display Mode"));
		QGridLayout* layout = new QGridLayout();

		display_polygons_checkbox_ = new QCheckBox(tr("Disp polys"));
		display_polygons_checkbox_->setChecked(true);
		display_labels_checkbox_ = new QCheckBox(tr("Disp labels(1)"));
		display_labels_checkbox_->setChecked(false);
		display_deleted_polygons_checkbox_ = new QCheckBox(tr("Disp Del polys(2)"));
		display_deleted_polygons_checkbox_->setChecked(false);
		display_point_clouds_checkbox_ = new QCheckBox(tr("Disp pnt clouds"));
		display_point_clouds_checkbox_->setChecked(false);
		select_polygon_checkbox_ = new QCheckBox(tr("Select Poly(3)"));
		//select_polygon_checkbox_->setShortcut(QKeySequence(Qt::Key_1));
		//QShortcut* shortcut = new QShortcut(QKeySequence(Qt::Key_1), this);
		//connect(shortcut, SIGNAL(activated()), select_polygon_checkbox_, SLOT(stateChanged(int)));
		select_polygon_checkbox_->setChecked(false);
		select_small_polygons_checkbox_ = new QCheckBox(tr("autofilter(4)"));
		select_small_polygons_checkbox_->setChecked(false);
		//test_radio_button_ = new QRadioButton(tr("test radio button"));
		//test_radio_button_->setChecked(false);
		select_point_checkbox_ = new QCheckBox(tr("Select Pnt(5)"));
		select_point_checkbox_->setChecked(false);
		edit_point_state_checkbox_ = new QCheckBox(tr("Edit Pnt(6)"));
		edit_point_state_checkbox_->setChecked(false);
		add_point_state_checkbox_ = new QCheckBox(tr("Add Pnt(7)"));
		add_point_state_checkbox_->setChecked(false);
		select_edge_checkbox_ = new QCheckBox(tr("Select Edge(8)"));
		select_edge_checkbox_->setChecked(false);
		refine_polygon_checkbox_ = new QCheckBox(tr("Select S&E Pnts"));
		refine_polygon_checkbox_->setChecked(false);
		select_outdoor_checkbox_ = new QCheckBox(tr("Select Outdoor"));
		select_outdoor_checkbox_->setChecked(false);
		select_new_polygon_vertices_checkbox_ = new QCheckBox(tr("Select 3+ pnts"));
		select_new_polygon_vertices_checkbox_->setChecked(false);
		split_polygon_checkbox_ = new QCheckBox(tr("Split poly"));
		split_polygon_checkbox_->setChecked(false);

		layout->addWidget(display_polygons_checkbox_, 0, 0);
		layout->addWidget(display_labels_checkbox_, 0, 1);
		layout->addWidget(display_deleted_polygons_checkbox_, 0, 2);
		layout->addWidget(display_point_clouds_checkbox_);
		layout->addWidget(select_polygon_checkbox_);
		layout->addWidget(select_small_polygons_checkbox_);
		//mainLayout_->addWidget(test_radio_button_);
		layout->addWidget(select_point_checkbox_);
		layout->addWidget(edit_point_state_checkbox_);
		layout->addWidget(add_point_state_checkbox_);
		layout->addWidget(select_edge_checkbox_);
		layout->addWidget(refine_polygon_checkbox_);
		layout->addWidget(select_outdoor_checkbox_);
		layout->addWidget(select_new_polygon_vertices_checkbox_);
		layout->addWidget(split_polygon_checkbox_);

		//layout->setColumnMinimumWidth(1, 8);
		//layout->setColumnMinimumWidth(2, 8);

		displayModeGroupBox->setLayout(layout);
	}

	void PolygonWindow::createButtonGroupBox()
	{
		buttonGroupBox = new QGroupBox(tr("Polygon Operation"));
		QGridLayout* layout = new QGridLayout();

		polygon_snapping_button_ = new QPushButton(tr("Snapping Poly"));
		polygon_snapping_button_new_ = new QPushButton(tr("Snapping Poly New"));
		polygon_expand_button_ = new QPushButton(tr("(SE)xpand Poly"));
		polygon_expand_seam_button_ = new QPushButton(tr("(E)xpand Poly Seam"));
		polygon_expand_seam_t2s_button_ = new QPushButton(tr("(Al+E)xpand Poly Seam"));
		optimize_expand_pnt_button_ = new QPushButton(tr("(O)ptimize Poly pnts"));
		point_moving_button_ = new QPushButton(tr("(M)ove Point)"));
		point_creating_button_ = new QPushButton(tr("Create New Pnt"));
		polygon_create_button_ = new QPushButton(tr("Create(pnt proj)"));
		polygon_create_button1_ = new QPushButton(tr("Create(pnt conn)"));
		polygon_create_button2_ = new QPushButton(tr("Create(poly proj)"));
		polygon_create_button3_ = new QPushButton(tr("Create(3+ pnts)"));
		polygon_create_button4_ = new QPushButton(tr("Create(edge+plane)"));
		polygon_perpendicular_ = new QPushButton(tr("Perpend plane"));
		polygon_inverse_button_ = new QPushButton(tr("(I)nverse Poly"));
		polygon_delete_button_ = new QPushButton(tr("(D)elete Poly"));
		point_delete_button_ = new QPushButton(tr("(D)elete Pnt"));
		polygon_remove_button_ = new QPushButton(tr("Set Poly As Removed"));
		open_image_button_ = new QPushButton(tr("Import Image"));
		open_all_pointcloud_button_ = new QPushButton(tr("Import PointCloud"));
		assign_polygon_to_pointcloud_button_ = new QPushButton(tr("Assign polygons Pnts"));
		save_assigned_pointcloud_button_ = new QPushButton(tr("Save polygon pnts"));
		save_unassigned_pointcloud_button_ = new QPushButton(tr("Save unpolygon pnts"));

		adjust_polygon_button_ = new QPushButton(tr("Adjust Poly by Line"));
		back_to_last_state_ = new QPushButton(tr("(B)ackward"));
		forward_to_next_state_ = new QPushButton(tr("(F)orward"));
		run_get_manhanttan_polygons_button_ = new QPushButton(tr("Construct Manhanttan"));
		save_cloud_points_button_ = new QPushButton(tr("Save cloud pnts"));
		save_refined_tris_button_ = new QPushButton(tr("Save tris"));
		save_project_button_ = new QPushButton(tr("Save Project"));
		save_manhanttan_polygons_button_ = new QPushButton(tr("Save Manhanttan"));
		plane_unprocess_button_ = new QPushButton(tr("Set unprocessed planes"));
		filter_cloud_points_button_ = new QPushButton(tr("Filter cloud pnts"));
		auto_expand_button = new QPushButton(tr("(A)uto Expand Polys"));
		save_labels_button = new QPushButton(tr("Save Labels"));
		auto_fuse_nearby_points_button = new QPushButton(tr("Auto Fuse Pnts"));
		auto_delete_collinear_points_button = new QPushButton(tr("Auto Del Pnts"));
		auto_split_polygon_button = new QPushButton(tr("Split Model"));
		split_curpoly_with_points_button_ = new QPushButton(tr("Split Cur Poly"));
		merge_cur_selected_polygons_button_ = new QPushButton(tr("Merge Cur Polys"));
		optimize_all_polygons_with_pnts_button_ = new QPushButton(tr("Opti All polys"));	//split merge polygons
		optimize_all_polygons_contour_with_pnts_button_ = new QPushButton(tr("Opti All polys contour"));	//opti contour
		aligin_polygons_normals_to_original_pnts_button_ = new QPushButton(tr("Align Poly Normal"));
		import_scene_button_ = new QPushButton(tr("Import Scene"));
		import_hwscene_button_ = new QPushButton(tr("Import HWScene"));	//important, it is hw scene data
		import_images_planes_labels_button_ = new QPushButton(tr("Import Planes Labels"));
		import_images_lines_button_ = new QPushButton(tr("Import Lines Pair"));
		import_reality_button_ = new QPushButton(tr("Import Reality Capture"));
		Save_aligned_button_ = new QPushButton(tr("Save Align Cam"));
		//render_polygon_depth_button
		render_polygon_depth_button = new QPushButton(tr("Render Depth"));
		run_dso_optimization_button_ = new QPushButton(tr("Run DSO"));
		save_images_polygons_lines_button_ = new QPushButton(tr("Save PolyImgLines"));

		convert_data2hw_scenes_button_ = new QPushButton(tr("Convert2LineScenes"));
		use_lsd_opencv_button_ = new QPushButton(tr("Use LSD"));
		load_lines_pnts_matches_button_ = new QPushButton(tr("Import Lines and Pnts"));
		convert_images_labels2hw_scenes_button_ = new QPushButton(tr("ConvertLabels2Scene"));
		run_lines_matches_button_lsd_ = new QPushButton(tr("RunLinesMatch(lsd)"));
		run_lines_matches_button_lines_ = new QPushButton(tr("RunLinesMatch(lines)"));
		run_lines_matches_button_lines_pnts_ = new QPushButton(tr("RunLinesMatch(lines pnts)"));
		convertdata2bundle_scenes_button_ = new QPushButton(tr("Convert2Bundle"));
		convertscenedata2bundle_scenes_cams_button_ = new QPushButton(tr("Convert2BundleCams"));
		save_images_lines_matches_button_ = new QPushButton(tr("SaveImgsLinesMatch"));
		save_images_names_matches_button_ = new QPushButton(tr("SaveImgsNamePair"));
		run_bundle_adjustment_button_ = new QPushButton(tr("RunBundleAdjust"));
		run_bundle_adjustment_cams_only_button_ = new QPushButton(tr("RunBundleCamsOnly"));
		save_bundle_result_button_ = new QPushButton(tr("SaveOptiCamsRes"));
		
		layout->addWidget(plane_unprocess_button_, 0, 0);
		layout->addWidget(polygon_snapping_button_, 0, 1);
		layout->addWidget(polygon_snapping_button_new_, 0, 2);
		layout->addWidget(polygon_expand_button_, 0 , 3);
		layout->addWidget(polygon_expand_seam_button_);
		layout->addWidget(polygon_expand_seam_t2s_button_);
		layout->addWidget(optimize_expand_pnt_button_);
		layout->addWidget(point_moving_button_);
		layout->addWidget(point_creating_button_);
		layout->addWidget(polygon_create_button_);
		layout->addWidget(polygon_create_button1_);
		layout->addWidget(polygon_create_button2_);
		layout->addWidget(polygon_create_button3_);
		layout->addWidget(polygon_create_button4_);
		layout->addWidget(polygon_perpendicular_);
		layout->addWidget(polygon_inverse_button_);
		layout->addWidget(polygon_delete_button_);
		layout->addWidget(point_delete_button_);
		layout->addWidget(polygon_remove_button_);
		layout->addWidget(open_image_button_);
		layout->addWidget(open_all_pointcloud_button_);
		layout->addWidget(assign_polygon_to_pointcloud_button_);
		layout->addWidget(save_assigned_pointcloud_button_);
		layout->addWidget(save_unassigned_pointcloud_button_);
		layout->addWidget(adjust_polygon_button_);
		layout->addWidget(back_to_last_state_);
		layout->addWidget(forward_to_next_state_);
		layout->addWidget(run_get_manhanttan_polygons_button_);
		layout->addWidget(save_cloud_points_button_);
		layout->addWidget(save_manhanttan_polygons_button_);
		layout->addWidget(save_project_button_);
		layout->addWidget(render_polygon_depth_button);
		layout->addWidget(save_refined_tris_button_);
		layout->addWidget(filter_cloud_points_button_);
		layout->addWidget(auto_expand_button);
		layout->addWidget(save_labels_button);
		layout->addWidget(auto_fuse_nearby_points_button);
		layout->addWidget(auto_delete_collinear_points_button);
		layout->addWidget(auto_split_polygon_button);
		layout->addWidget(split_curpoly_with_points_button_);
		layout->addWidget(merge_cur_selected_polygons_button_);
		layout->addWidget(optimize_all_polygons_with_pnts_button_);
		layout->addWidget(optimize_all_polygons_contour_with_pnts_button_);
		layout->addWidget(aligin_polygons_normals_to_original_pnts_button_);
		layout->addWidget(run_dso_optimization_button_);
		layout->addWidget(save_images_polygons_lines_button_);
		//save_images_polygons_lines_button_
		//layout->setColumnMinimumWidth(1, 8);
		//layout->setColumnMinimumWidth(2, 8);
		buttonGroupBox->setLayout(layout);

		buttonCamsOptiGroupBox = new QGroupBox(tr("CamOpti"));
		QGridLayout* cam_layout = new QGridLayout();
		cam_layout->addWidget(import_scene_button_, 0, 0);
		cam_layout->addWidget(import_hwscene_button_, 0, 1);
		cam_layout->addWidget(import_images_planes_labels_button_, 0, 2);
		cam_layout->addWidget(import_reality_button_, 0, 3);	//load realitycapture data
		//cam_layout->addWidget(convert_data2hw_scenes_button_, 0, 3);

		QPushButton* split_buttion0 = new QPushButton();
		split_buttion0->setFocusPolicy(Qt::NoFocus);
		split_buttion0->setMaximumHeight(1);
		QPushButton* split_buttion1 = new QPushButton();
		split_buttion1->setFocusPolicy(Qt::NoFocus);
		split_buttion1->setMaximumHeight(1);
		QPushButton* split_buttion2 = new QPushButton();
		split_buttion2->setFocusPolicy(Qt::NoFocus);
		split_buttion2->setMaximumHeight(1);
		QPushButton* split_buttion3 = new QPushButton();
		split_buttion3->setFocusPolicy(Qt::NoFocus);
		split_buttion3->setMaximumHeight(1);
		cam_layout->addWidget(split_buttion0, 1, 0);
		cam_layout->addWidget(split_buttion1, 1, 1);
		cam_layout->addWidget(split_buttion2, 1, 2);
		cam_layout->addWidget(split_buttion3, 1, 3);

		cam_layout->addWidget(convert_data2hw_scenes_button_, 2, 0);
		cam_layout->addWidget(use_lsd_opencv_button_, 2, 1);	//use opencv lsd 
		cam_layout->addWidget(import_images_lines_button_, 2, 2);	//use only lines from network
		cam_layout->addWidget(load_lines_pnts_matches_button_, 2, 3);	//use the lines and pnts from network
		
		QPushButton* split_buttion4 = new QPushButton();
		split_buttion4->setFocusPolicy(Qt::NoFocus);
		split_buttion4->setMaximumHeight(1);
		QPushButton* split_buttion5 = new QPushButton();
		split_buttion5->setFocusPolicy(Qt::NoFocus);
		split_buttion5->setMaximumHeight(1);
		QPushButton* split_buttion6 = new QPushButton();
		split_buttion6->setFocusPolicy(Qt::NoFocus);
		split_buttion6->setMaximumHeight(1);
		QPushButton* split_buttion7 = new QPushButton();
		split_buttion7->setFocusPolicy(Qt::NoFocus);
		split_buttion7->setMaximumHeight(1);

		cam_layout->addWidget(split_buttion4, 3, 0);
		cam_layout->addWidget(split_buttion5, 3, 1);
		cam_layout->addWidget(split_buttion6, 3, 2);
		cam_layout->addWidget(split_buttion7, 3, 3);

		cam_layout->addWidget(run_lines_matches_button_lsd_, 4, 0);
		cam_layout->addWidget(run_lines_matches_button_lines_, 4, 1);
		cam_layout->addWidget(run_lines_matches_button_lines_pnts_, 4, 2);
		cam_layout->addWidget(convertdata2bundle_scenes_button_, 4, 3);

		//convertscenedata2bundle_scenes_cams_button_
		cam_layout->addWidget(convertscenedata2bundle_scenes_cams_button_, 5, 0);
		cam_layout->addWidget(run_bundle_adjustment_button_, 5, 1);
		cam_layout->addWidget(run_bundle_adjustment_cams_only_button_, 5, 2);
		cam_layout->addWidget(save_images_lines_matches_button_, 5, 3);

		cam_layout->addWidget(Save_aligned_button_, 6, 0);
		cam_layout->addWidget(save_bundle_result_button_, 6, 1);
		cam_layout->addWidget(save_images_names_matches_button_, 6, 2);
		cam_layout->addWidget(convert_images_labels2hw_scenes_button_, 6, 3);	//convert loaded images labels to hw scenes elements
		buttonCamsOptiGroupBox->setLayout(cam_layout);
	}

	void PolygonWindow::createParamsGroupBox()
	{
		paramsGroupBox = new QGroupBox(tr("Params"));
		QGridLayout* layout = new QGridLayout();


		polygon_move_step_label = new QLabel("move(meter)");
		layout->addWidget(polygon_move_step_label, 0, 0);
		polygon_move_step_spinbox = new QDoubleSpinBox();
		polygon_move_step_spinbox->setRange(0, 10);
		polygon_move_step_spinbox->setDecimals(2);
		polygon_move_step_spinbox->setSingleStep(0.01);
		polygon_move_step_spinbox->setValue(0.1);
		layout->addWidget(polygon_move_step_spinbox, 0, 1);

		polygon_rotate_step_label = new QLabel("rotate(degree)");
		layout->addWidget(polygon_rotate_step_label, 0, 2);
		polygon_rotate_step_spinbox = new QDoubleSpinBox();
		polygon_rotate_step_spinbox->setRange(0, 90);
		polygon_rotate_step_spinbox->setDecimals(1);
		polygon_rotate_step_spinbox->setSingleStep(1);
		polygon_rotate_step_spinbox->setValue(10);
		layout->addWidget(polygon_rotate_step_spinbox, 0, 3);

		r_max_label = new QLabel("r_max");
		layout->addWidget(r_max_label);
		r_max_spinbox = new QDoubleSpinBox();
		r_max_spinbox->setRange(0, 20);  // 范围
		r_max_spinbox->setDecimals(2);  // 精度
		r_max_spinbox->setSingleStep(0.01);
		r_max_spinbox->setValue(0.2);
		layout->addWidget(r_max_spinbox);

		set_label_label = new QLabel("semantic label");
		layout->addWidget(set_label_label);
		set_label_spinbox = new QSpinBox();
		set_label_spinbox->setRange(0, 13);
		set_label_spinbox->setSingleStep(1);
		set_label_spinbox->setValue(13);
		layout->addWidget(set_label_spinbox);

		r_tri_len_ = new QLabel("r_tri");
		layout->addWidget(r_tri_len_);
		r_tri_len_spinbox_ = new QDoubleSpinBox();
		r_tri_len_spinbox_->setRange(0, 50);  // 范围
		r_tri_len_spinbox_->setDecimals(2);  // 精度
		r_tri_len_spinbox_->setSingleStep(0.01);
		r_tri_len_spinbox_->setValue(0.2);
		layout->addWidget(r_tri_len_spinbox_);

		r_tri_degree__ = new QLabel("tri deg");
		layout->addWidget(r_tri_degree__);
		r_tri_degree_spinbox_ = new QDoubleSpinBox();
		r_tri_degree_spinbox_->setRange(0, 180);  // 范围
		r_tri_degree_spinbox_->setDecimals(1);  // 精度
		r_tri_degree_spinbox_->setSingleStep(1);
		r_tri_degree_spinbox_->setValue(10);
		layout->addWidget(r_tri_degree_spinbox_);
		
		sampling_density_label = new QLabel("sampling density");
		layout->addWidget(sampling_density_label);
		sampling_density_spinbox = new QSpinBox();
		sampling_density_spinbox->setRange(100, 1000000);
		sampling_density_spinbox->setSingleStep(100);
		sampling_density_spinbox->setValue(100);
		layout->addWidget(sampling_density_spinbox);

		//layout->setColumnMinimumWidth(1, 4);
		//layout->setColumnMinimumWidth(2, 4);
		//layout->setColumnMinimumWidth(3, 4);
		//layout->setColumnMinimumWidth(4, 4);
		paramsGroupBox->setLayout(layout);
	}

	void PolygonWindow::createSmallPolygonGroupBox()
	{
		smallPolygonGroupBox = new QGroupBox(tr("Small Polygon"));
		QGridLayout* layout = new QGridLayout();

		//display_polygons_checkbox_ = new QCheckBox(tr("Display polygons"));
		//display_polygons_checkbox_->setChecked(true);

		select_min_height_checkbox = new QCheckBox(tr("height"));
		select_min_height_checkbox->setChecked(false);
		layout->addWidget(select_min_height_checkbox, 0, 0);
		//min_height_label = new QLabel("min height");
		//layout->addWidget(min_height_label);
		min_height_spinbox = new QDoubleSpinBox();
		min_height_spinbox->setRange(0, 10);
		min_height_spinbox->setDecimals(2);
		min_height_spinbox->setSingleStep(0.01);
		min_height_spinbox->setValue(0.5);
		layout->addWidget(min_height_spinbox, 0, 1);


		select_min_width_checkbox = new QCheckBox(tr("width"));
		select_min_width_checkbox->setChecked(false);
		layout->addWidget(select_min_width_checkbox, 0, 2);
		//min_width_label = new QLabel("min width");
		//layout->addWidget(min_width_label);
		min_width_spinbox = new QDoubleSpinBox();
		min_width_spinbox->setRange(0, 10);
		min_width_spinbox->setDecimals(2);
		min_width_spinbox->setSingleStep(0.01);
		min_width_spinbox->setValue(0.5);
		layout->addWidget(min_width_spinbox, 0, 3);


		select_min_corner_points_checkbox = new QCheckBox(tr("pnts"));
		select_min_corner_points_checkbox->setChecked(false);
		layout->addWidget(select_min_corner_points_checkbox);
		//min_corner_points_label = new QLabel("min corner points num");
		//layout->addWidget(min_corner_points_label);
		min_corner_points_spinbox = new QSpinBox();
		min_corner_points_spinbox->setRange(0, 10);
		min_corner_points_spinbox->setSingleStep(1);
		min_corner_points_spinbox->setValue(4);
		layout->addWidget(min_corner_points_spinbox);

		select_min_area_checkbox = new QCheckBox(tr("area"));
		select_min_area_checkbox->setChecked(false);
		layout->addWidget(select_min_area_checkbox);
		//min_area_label = new QLabel("min area");
		//layout->addWidget(min_area_label);
		min_area_spinbox = new QDoubleSpinBox();
		min_area_spinbox->setRange(0, 100);
		min_area_spinbox->setDecimals(2);
		min_area_spinbox->setSingleStep(0.01);
		min_area_spinbox->setValue(0.5);
		layout->addWidget(min_area_spinbox);


		select_min_area_by_vertices_checkbox = new QCheckBox(tr("area/pnts"));
		select_min_area_by_vertices_checkbox->setChecked(false);
		layout->addWidget(select_min_area_by_vertices_checkbox);
		//min_area_by_vertices_label = new QLabel("min area divided by points num");
		//layout->addWidget(min_area_by_vertices_label);
		min_area_by_vertices_spinbox = new QDoubleSpinBox();
		min_area_by_vertices_spinbox->setRange(0, 10);
		min_area_by_vertices_spinbox->setDecimals(2);
		min_area_by_vertices_spinbox->setSingleStep(0.01);
		min_area_by_vertices_spinbox->setValue(0.1);
		layout->addWidget(min_area_by_vertices_spinbox);


		select_min_area_by_boundingbox_checkbox = new QCheckBox(tr("area/bb"));
		select_min_area_by_boundingbox_checkbox->setChecked(false);
		layout->addWidget(select_min_area_by_boundingbox_checkbox);
		//min_area_by_boundingbox_label = new QLabel("min area divided by boundingbox area");
		//layout->addWidget(min_area_by_boundingbox_label);
		min_area_by_boundingbox_spinbox = new QDoubleSpinBox();
		min_area_by_boundingbox_spinbox->setRange(0, 10);
		min_area_by_boundingbox_spinbox->setDecimals(2);
		min_area_by_boundingbox_spinbox->setSingleStep(0.01);
		min_area_by_boundingbox_spinbox->setValue(0.5);
		layout->addWidget(min_area_by_boundingbox_spinbox);

		//layout->setColumnMinimumWidth(1, 4);
		//layout->setColumnMinimumWidth(2, 4);
		//layout->setColumnMinimumWidth(3, 4);
		//layout->setColumnMinimumWidth(4, 4);

		smallPolygonGroupBox->setLayout(layout);
	}

	void PolygonWindow::keyPressEvent(QKeyEvent * event)
	{
		if (event->key() == Qt::Key_1)
		{
			if (display_labels_checkbox_->checkState() == Qt::Checked) {
				display_labels_checkbox_->setCheckState(Qt::Unchecked);
			}
			else {
				display_labels_checkbox_->setCheckState(Qt::Checked);
			}
		}
		else if (event->key() == Qt::Key_2)
		{
			if (display_deleted_polygons_checkbox_->checkState() == Qt::Checked) 
			{
				display_deleted_polygons_checkbox_->setCheckState(Qt::Unchecked);
			}
			else {
				display_deleted_polygons_checkbox_->setCheckState(Qt::Checked);
			}
		}
		else if (event->key() == Qt::Key_3)
		{
			if (select_polygon_checkbox_->checkState() == Qt::Checked) 
			{
				select_polygon_checkbox_->setCheckState(Qt::Unchecked);
			}
			else 
			{
				select_polygon_checkbox_->setCheckState(Qt::Checked);
				if (select_small_polygons_checkbox_->checkState() == Qt::Checked)
				{
					select_small_polygons_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_point_checkbox_->checkState() == Qt::Checked) 
				{
					select_point_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (edit_point_state_checkbox_->checkState() == Qt::Checked) 
				{
					edit_point_state_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (add_point_state_checkbox_->checkState() == Qt::Checked) 
				{
					add_point_state_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_edge_checkbox_->checkState() == Qt::Checked) 
				{
					select_edge_checkbox_->setCheckState(Qt::Unchecked);
				}
			}
		}
		else if (event->key() == Qt::Key_4)
		{
			if (select_small_polygons_checkbox_->checkState() == Qt::Checked) 
			{
				select_small_polygons_checkbox_->setCheckState(Qt::Unchecked);
			}
			else 
			{
				select_small_polygons_checkbox_->setCheckState(Qt::Checked);
				if (select_polygon_checkbox_->checkState() == Qt::Checked)
				{
					select_polygon_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_point_checkbox_->checkState() == Qt::Checked) 
				{
					select_point_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (edit_point_state_checkbox_->checkState() == Qt::Checked) 
				{
					edit_point_state_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (add_point_state_checkbox_->checkState() == Qt::Checked) 
				{
					add_point_state_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_edge_checkbox_->checkState() == Qt::Checked)
				{
					select_edge_checkbox_->setCheckState(Qt::Unchecked);
				}
			}
		}
		else if (event->key() == Qt::Key_5)
		{
			if (select_point_checkbox_->checkState() == Qt::Checked) {
				select_point_checkbox_->setCheckState(Qt::Unchecked);
			}
			else {
				select_point_checkbox_->setCheckState(Qt::Checked);
				if (select_polygon_checkbox_->checkState() == Qt::Checked)
				{
					select_polygon_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_small_polygons_checkbox_->checkState() == Qt::Checked)
				{
					select_small_polygons_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (edit_point_state_checkbox_->checkState() == Qt::Checked) {
					edit_point_state_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (add_point_state_checkbox_->checkState() == Qt::Checked) {
					add_point_state_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_edge_checkbox_->checkState() == Qt::Checked) {
					select_edge_checkbox_->setCheckState(Qt::Unchecked);
				}
			}
		}
		else if (event->key() == Qt::Key_6)
		{
			if (edit_point_state_checkbox_->checkState() == Qt::Checked) {
				edit_point_state_checkbox_->setCheckState(Qt::Unchecked);
			}
			else {
				edit_point_state_checkbox_->setCheckState(Qt::Checked);
				if (select_polygon_checkbox_->checkState() == Qt::Checked)
				{
					select_polygon_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_small_polygons_checkbox_->checkState() == Qt::Checked)
				{
					select_small_polygons_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_point_checkbox_->checkState() == Qt::Checked) {
					select_point_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (add_point_state_checkbox_->checkState() == Qt::Checked) {
					add_point_state_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_edge_checkbox_->checkState() == Qt::Checked) {
					select_edge_checkbox_->setCheckState(Qt::Unchecked);
				}
			}
		}
		else if (event->key() == Qt::Key_7)
		{
			if (add_point_state_checkbox_->checkState() == Qt::Checked) {
				add_point_state_checkbox_->setCheckState(Qt::Unchecked);
			}
			else {
				add_point_state_checkbox_->setCheckState(Qt::Checked);
				if (select_polygon_checkbox_->checkState() == Qt::Checked)
				{
					select_polygon_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_small_polygons_checkbox_->checkState() == Qt::Checked)
				{
					select_small_polygons_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_point_checkbox_->checkState() == Qt::Checked) {
					select_point_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (edit_point_state_checkbox_->checkState() == Qt::Checked) {
					edit_point_state_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_edge_checkbox_->checkState() == Qt::Checked) {
					select_edge_checkbox_->setCheckState(Qt::Unchecked);
				}
			}
		}
		else if (event->key() == Qt::Key_8)
		{
			if (select_edge_checkbox_->checkState() == Qt::Checked) {
				select_edge_checkbox_->setCheckState(Qt::Unchecked);
			}
			else {
				select_edge_checkbox_->setCheckState(Qt::Checked);
				if (select_polygon_checkbox_->checkState() == Qt::Checked)
				{
					select_polygon_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_small_polygons_checkbox_->checkState() == Qt::Checked)
				{
					select_small_polygons_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (select_point_checkbox_->checkState() == Qt::Checked) {
					select_point_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (edit_point_state_checkbox_->checkState() == Qt::Checked) {
					edit_point_state_checkbox_->setCheckState(Qt::Unchecked);
				}
				if (add_point_state_checkbox_->checkState() == Qt::Checked) {
					add_point_state_checkbox_->setCheckState(Qt::Unchecked);
				}
			}
		}
		else if (event->key() == Qt::Key_D)
		{
			if (polygon_openGLWindow_->CheckPolygonSelectState()
				&& !polygon_openGLWindow_->CheckPointSelectState())
			{
				printf("delete polygon!\n");
				polygon_openGLWindow_->doPolygonDeleting();
			}
			else if (polygon_openGLWindow_->CheckSmallPolygonSelectState())
			{
				printf("delete small polygons!\n");
				polygon_openGLWindow_->doSmallPolygonDeleting();
			}
			else if (polygon_openGLWindow_->CheckPointSelectState()
				&& !polygon_openGLWindow_->CheckPolygonSelectState())
			{
				printf("delete point!\n");
				polygon_openGLWindow_->doPointDeleting();
			}
			else if (polygon_openGLWindow_->CheckPointSelectState()
				&& polygon_openGLWindow_->CheckPolygonSelectState())
			{
				printf("delete point first!\n");
				polygon_openGLWindow_->doPointDeleting();
			}
			else
			{
				printf("no element selected!\n");
			}
		}
		else if (event->modifiers() == (Qt::ShiftModifier) && event->key() == Qt::Key_E)
		{
			if (polygon_openGLWindow_->CheckPolygonSelectState())
			{
				printf("expand polygon!\n");
				polygon_openGLWindow_->doPolygonExpanding();
			}
		}
		else if (event->modifiers() != Qt::ShiftModifier 
        && event->modifiers() != Qt::AltModifier  
        && event->key() == Qt::Key_E)
		{
			if (polygon_openGLWindow_->CheckPolygonSelectState())
			{
				printf("expand polygon Seam!\n");
				polygon_openGLWindow_->doPolygonExpandingSeam();
			}
		}
		else if (event->modifiers() == (Qt::AltModifier) && event->key() == Qt::Key_E)
		{
			if (polygon_openGLWindow_->CheckPolygonSelectState())
			{
				printf("expand polygon Seam in two way!\n");
				polygon_openGLWindow_->doPolygonExpandingSeamBiDirect();
			}
		}
		else if (event->key() == Qt::Key_M)
		{
			if (polygon_openGLWindow_->CheckPointSelectState())
				//&& polygon_openGLWindow_->CheckPolygonSelectState())
			{
				printf("move point!\n");
				polygon_openGLWindow_->doPointMoving();
			}
		}
		else if (event->key() == Qt::Key_Left)
		{
			std::cout << "left" << std::endl;
			polygon_openGLWindow_->doPolygonMoving(polygon_move_step_spinbox->value());
		}
		else if (event->key() == Qt::Key_Right)
		{
			std::cout << "rotate right" << std::endl;
			polygon_openGLWindow_->doPolygonMoving(-polygon_move_step_spinbox->value());
		}
		else if (event->key() == Qt::Key_R)
		{
			std::cout << "rotate right" << std::endl;
			polygon_openGLWindow_->doPolygonRotating(polygon_rotate_step_spinbox->value());
		}
		else if (event->key() == Qt::Key_L)
		{
			std::cout << "rotate left" << std::endl;
			polygon_openGLWindow_->doPolygonRotating(-polygon_rotate_step_spinbox->value());
		}
		else if (event->key() == Qt::Key_C)
		{
			std::cout << "clear selected points" << std::endl;
			polygon_openGLWindow_->ClearSelectedPoints();
		}
		else if (event->key() == Qt::Key_B)
		{
			polygon_openGLWindow_->backToLastState();
		}
		else if (event->key() == Qt::Key_F)
		{
			polygon_openGLWindow_->forwardToNextState();
		}
		else if (event->key() == Qt::Key_A)
		{
			polygon_openGLWindow_->autoExpanding();
		}
		else if (event->key() == Qt::Key_I)
		{
			polygon_openGLWindow_->doPolygonInversing();
		}
		else if (event->key() == Qt::Key_O)
		{
			polygon_openGLWindow_->doPolygonOptimizePolyPnts();
		}
#if 0
        switch (event->key())
        {
        case Qt::Key_1:
            if (display_labels_checkbox_->checkState() == Qt::Checked) {
                display_labels_checkbox_->setCheckState(Qt::Unchecked);
            }
            else {
                display_labels_checkbox_->setCheckState(Qt::Checked);
            }
            break;
        case Qt::Key_2:
            if (display_deleted_polygons_checkbox_->checkState() == Qt::Checked) {
                display_deleted_polygons_checkbox_->setCheckState(Qt::Unchecked);
            }
            else {
                display_deleted_polygons_checkbox_->setCheckState(Qt::Checked);
            }
            break;
        case Qt::Key_3:
            if (select_polygon_checkbox_->checkState() == Qt::Checked) {
                select_polygon_checkbox_->setCheckState(Qt::Unchecked);
            }
            else {
                select_polygon_checkbox_->setCheckState(Qt::Checked);
            }
            break;
        case Qt::Key_4:
            if (select_small_polygons_checkbox_->checkState() == Qt::Checked) {
                select_small_polygons_checkbox_->setCheckState(Qt::Unchecked);
            }
            else {
                select_small_polygons_checkbox_->setCheckState(Qt::Checked);
            }
            break;
        case Qt::Key_5:
            if (select_point_checkbox_->checkState() == Qt::Checked) {
                select_point_checkbox_->setCheckState(Qt::Unchecked);
            }
            else {
                select_point_checkbox_->setCheckState(Qt::Checked);
            }
            break;
        case Qt::Key_6:
            if (edit_point_state_checkbox_->checkState() == Qt::Checked) {
                edit_point_state_checkbox_->setCheckState(Qt::Unchecked);
            }
            else {
                edit_point_state_checkbox_->setCheckState(Qt::Checked);
            }
            break;
        case Qt::Key_7:
			if (add_point_state_checkbox_->checkState() == Qt::Checked) {
				add_point_state_checkbox_->setCheckState(Qt::Unchecked);
			}
			else {
				add_point_state_checkbox_->setCheckState(Qt::Checked);
			}
			break;
        case Qt::Key_8:
            if (select_edge_checkbox_->checkState() == Qt::Checked) {
                select_edge_checkbox_->setCheckState(Qt::Unchecked);
            }
            else {
                select_edge_checkbox_->setCheckState(Qt::Checked);
            }
            break;
        case Qt::Key_D:
            if (polygon_openGLWindow_->CheckPolygonSelectState()
                && !polygon_openGLWindow_->CheckPointSelectState())
            {
                printf("delete polygon!\n");
                polygon_openGLWindow_->doPolygonDeleting();
            }
            else if (polygon_openGLWindow_->CheckSmallPolygonSelectState())
            {
                printf("delete small polygons!\n");
                polygon_openGLWindow_->doSmallPolygonDeleting();
            }
            else if (polygon_openGLWindow_->CheckPointSelectState()
                && !polygon_openGLWindow_->CheckPolygonSelectState())
            {
                printf("delete point!\n");
                polygon_openGLWindow_->doPointDeleting();
            }
            else if (polygon_openGLWindow_->CheckPointSelectState()
                && polygon_openGLWindow_->CheckPolygonSelectState())
            {
                printf("delete point first!\n");
                polygon_openGLWindow_->doPointDeleting();
            }
            else
            {
                printf("no element selected!\n");
            }
            break;
        case Qt::SHIFT:
		{
			if (event->modifiers() == (Qt::ShiftModifier) && event->key() == Qt::Key_E)
			{
				if (polygon_openGLWindow_->CheckPolygonSelectState())
				{
					printf("expand polygon!\n");
					polygon_openGLWindow_->doPolygonExpanding();
				}
				break;
			}
		} 
		case Qt::Key_E:
			if (polygon_openGLWindow_->CheckPolygonSelectState())
			{
				printf("expand polygon Seam!\n");
				polygon_openGLWindow_->doPolygonExpandingSeam();
			}
			break;
        case Qt::Key_M:
            if (polygon_openGLWindow_->CheckPointSelectState())
				//&& polygon_openGLWindow_->CheckPolygonSelectState())
			{
				printf("move point!\n");
				polygon_openGLWindow_->doPointMoving();
			}
			break;
		case Qt::Key_Left:
			std::cout << "left" << std::endl;
			polygon_openGLWindow_->doPolygonMoving(polygon_move_step_spinbox->value());
			break;
		case Qt::Key_Right:
			std::cout << "right" << std::endl;
			polygon_openGLWindow_->doPolygonMoving(-polygon_move_step_spinbox->value());
			break;
		case Qt::Key_R:
			std::cout << "rotate right" << std::endl;
			polygon_openGLWindow_->doPolygonRotating(polygon_rotate_step_spinbox->value());
			break;
		case Qt::Key_L:
			std::cout << "rotate left" << std::endl;
			polygon_openGLWindow_->doPolygonRotating(-polygon_rotate_step_spinbox->value());
			break;
		case Qt::Key_C:
			std::cout << "clear selected points" << std::endl;
			polygon_openGLWindow_->ClearSelectedPoints();
			break;
		case Qt::Key_B:
			polygon_openGLWindow_->backToLastState();
			break;
		case Qt::Key_F:
			polygon_openGLWindow_->forwardToNextState();
			break;
		case Qt::Key_A:
			polygon_openGLWindow_->autoExpanding();
			break;
		case Qt::Key_I:
			polygon_openGLWindow_->doPolygonInversing();
			break;
		default:
			break;
		}
#endif
	}
}

