
#pragma once
#include <QWidget>
#include <QMainWindow>
#include <QPushButton>
#include <QCheckBox>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QRadioButton>
#include <QShortCut>
#include <QGroupBox>
#include "PolygonGLWindow.h"
#include "ImageWindow.h"

namespace HW {
	class PolygonWindow :public QWidget {
		Q_OBJECT


	public:
		PolygonWindow(QWidget *parent = 0);
		~PolygonWindow();
		void CreateImageWindow();
		void CreatePolygonGLWindow();
		void CreateButton();
		void CreateConnect();
		void SetData(HWPointCloud* in_pointcloud,std::vector<HWPlane*>& planes_vec);
		void SetData(HWPolygon* polygon);
		void SetAllPointCloud(HWPointCloud* in_pointcloud);	//不一样的polygon
		const PolygonGLWindow* GetPolygonGLWindowPolygons();

	private:

		void createDisplayModeGroupBox();
		void createButtonGroupBox();
		void createParamsGroupBox();
		void createSmallPolygonGroupBox();

		QGroupBox* displayModeGroupBox;
		QGroupBox* buttonGroupBox;
		QGroupBox* buttonCamsOptiGroupBox;
		QGroupBox* paramsGroupBox;
		QGroupBox* smallPolygonGroupBox;

		QWidget *mainWidget_;
		QVBoxLayout *mainLayout_;

        //import scene
		QPushButton *import_scene_button_;
		QPushButton *import_hwscene_button_;	//all hw scene data
		QPushButton *import_images_planes_labels_button_;
		QPushButton *import_images_lines_button_; //get images lines pair from txt
		QPushButton *import_reality_button_;
		QPushButton *Save_aligned_button_;
		QPushButton* save_labels_button;

		PolygonGLWindow* polygon_openGLWindow_;
		ImageWindow* image_window_;

		/*----------ÏÔÊ¾²Ù×÷×é----------*/
		QCheckBox* display_polygons_checkbox_;
		QCheckBox* display_labels_checkbox_;
		QCheckBox* display_deleted_polygons_checkbox_;
		QCheckBox* display_point_clouds_checkbox_;
		QCheckBox *select_polygon_checkbox_;
		QCheckBox* select_small_polygons_checkbox_;
		QCheckBox *select_point_checkbox_;
		QCheckBox *edit_point_state_checkbox_;
		QCheckBox *add_point_state_checkbox_;
		QCheckBox* select_edge_checkbox_;
		QCheckBox *refine_polygon_checkbox_;
		QCheckBox *select_outdoor_checkbox_;
		QCheckBox *select_new_polygon_vertices_checkbox_;
		QCheckBox* split_polygon_checkbox_;
		/*----------ÏÔÊ¾²Ù×÷×é----------*/

		/*----------°´Å¥²Ù×÷×é----------*/
		QPushButton *polygon_expand_button_;
		QPushButton *polygon_expand_seam_button_;
		QPushButton *polygon_expand_seam_t2s_button_;	//双向expand
		QPushButton *optimize_expand_pnt_button_;
		QPushButton *point_moving_button_;
		QPushButton *point_creating_button_;
		QPushButton *polygon_snapping_button_;
		QPushButton *polygon_snapping_button_new_;
		QPushButton *plane_unprocess_button_;
		QPushButton *polygon_create_button_;
		QPushButton *polygon_create_button1_;
		QPushButton *polygon_create_button2_;
		QPushButton *polygon_create_button3_;
		QPushButton* polygon_create_button4_;
		QPushButton *polygon_perpendicular_;
		QPushButton *polygon_inverse_button_;
		QPushButton *polygon_delete_button_;
		QPushButton *polygon_remove_button_;
		QPushButton *point_delete_button_;
		QPushButton *open_image_button_;
		QPushButton *open_all_pointcloud_button_;
		QPushButton *assign_polygon_to_pointcloud_button_;
		QPushButton *save_assigned_pointcloud_button_;
		QPushButton *save_unassigned_pointcloud_button_;

		QPushButton *adjust_polygon_button_;
		QPushButton *back_to_last_state_;
		QPushButton *forward_to_next_state_;
		QPushButton* run_get_manhanttan_polygons_button_;
		QPushButton *save_cloud_points_button_;
		QPushButton *save_refined_tris_button_;
		QPushButton *save_project_button_;
		QPushButton* save_manhanttan_polygons_button_;
		QPushButton *filter_cloud_points_button_;
		QPushButton *auto_expand_button;
		QPushButton* auto_fuse_nearby_points_button;
		QPushButton* auto_delete_collinear_points_button;
		QPushButton* auto_split_polygon_button;
		QPushButton* split_curpoly_with_points_button_;
		QPushButton* merge_cur_selected_polygons_button_;
		QPushButton* optimize_all_polygons_with_pnts_button_;
		QPushButton* optimize_all_polygons_contour_with_pnts_button_;
		QPushButton* aligin_polygons_normals_to_original_pnts_button_;
		QPushButton* render_polygon_depth_button;

		//show the dso button
		QPushButton* run_dso_optimization_button_;
		QPushButton* save_images_polygons_lines_button_;

		//load data from ScenesElement to HWScenesElement data
		QPushButton* convert_data2hw_scenes_button_;
		QPushButton* use_lsd_opencv_button_;
		QPushButton* run_lines_matches_button_lsd_;
		QPushButton* run_lines_matches_button_lines_;
		QPushButton* run_lines_matches_button_lines_pnts_;
		QPushButton* convert_images_labels2hw_scenes_button_;
		QPushButton* convertdata2bundle_scenes_button_;
		QPushButton* convertscenedata2bundle_scenes_cams_button_;
		QPushButton* save_images_lines_matches_button_;
		QPushButton* save_images_names_matches_button_;
		QPushButton* run_bundle_adjustment_button_;
		QPushButton* run_bundle_adjustment_cams_only_button_;
		QPushButton* save_bundle_result_button_;

		QPushButton* load_lines_pnts_matches_button_;	//important
		//QPushButton* load_images_lines_pnts_matches_button_;

		/*----------°´Å¥²Ù×÷×é----------*/

		/*----------²ÎÊý²Ù×÷×é----------*/
		QLabel *polygon_move_step_label;
		QLabel* polygon_rotate_step_label;
		QLabel *r_max_label;
		QLabel *set_label_label;
		QLabel* sampling_density_label;

		QLabel *r_tri_len_;
		QLabel *r_tri_degree__;

		QDoubleSpinBox* polygon_move_step_spinbox;
		QDoubleSpinBox* polygon_rotate_step_spinbox;
		QDoubleSpinBox *r_max_spinbox;
		QDoubleSpinBox *r_tri_len_spinbox_;
		QDoubleSpinBox *r_tri_degree_spinbox_;
		QSpinBox* set_label_spinbox;
		QSpinBox* sampling_density_spinbox;
		/*----------²ÎÊý²Ù×÷×é----------*/

		//QLabel *ratio_snap_label;
		//QLabel *ratio_ref_label;
		//QLabel *ratio_ortho_label;
		//QLabel *ratio_cur_label;
		//QDoubleSpinBox *ratio_snap_spinbox;
		//QDoubleSpinBox *ratio_ref_spinbox;
		//QDoubleSpinBox *ratio_ortho_spinbox;
		//QDoubleSpinBox *ratio_cur_spinbox;


		/*----------Ð¡Æ½Ãæ²Ù×÷×é----------*/
		QLabel *min_height_label;
		QLabel *min_width_label;
		QLabel *min_corner_points_label;
		QLabel* min_area_label;
		QLabel* min_area_by_vertices_label;
		QLabel* min_area_by_boundingbox_label;

		QCheckBox* select_min_height_checkbox;
		QCheckBox* select_min_width_checkbox;
		QCheckBox* select_min_corner_points_checkbox;
		QCheckBox* select_min_area_checkbox;
		QCheckBox* select_min_area_by_vertices_checkbox;
		QCheckBox* select_min_area_by_boundingbox_checkbox;

		QDoubleSpinBox *min_height_spinbox;
		QDoubleSpinBox *min_width_spinbox;
		QSpinBox *min_corner_points_spinbox;
		QDoubleSpinBox *min_area_spinbox;
		QDoubleSpinBox *min_area_by_vertices_spinbox;
		QDoubleSpinBox *min_area_by_boundingbox_spinbox;
		/*----------Ð¡Æ½Ãæ²Ù×÷×é----------*/


		virtual void keyPressEvent(QKeyEvent * event);

	public slots:
		
	void SetEditPlanePnt3dState(int state);
	void SetAddPlanePnt3dState(int state);
	void hello(void) { std::cout << "hello" << std::endl; }
	};
}