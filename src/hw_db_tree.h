#pragma once
#ifndef HW_DB_TREE
#define HW_DB_TREE

//Qt
#include <QApplication>
#include <QHeaderView>
#include <QinputDialog>
#include <QMenu>
#include <QAbstractItemModel>
#include <QPoint>
#include <QTreeView>
//std
#include <map>
//
#include "hw_object.h"
#include "hw_pcl_functor.h"
#include "hw_point_cloud.h"
#include "hw_mesh.h"
#include "hw_tsdf.h"

namespace HW 
{
	struct HWTreeSelectionInfo
	{
		size_t sel_count;
		size_t normals_count;
		size_t octree_count;
		size_t cloud_count;
		size_t mesh_count;
		size_t kd_tree_count;

		void reset()
		{}
	};

	//! Custom QTreeView widget (for advanced selection behavior)
	class HWCustomQTreeView : public QTreeView
	{
		Q_OBJECT

	public:

		//! Default constructor
		explicit HWCustomQTreeView(QWidget* parent) : QTreeView(parent) {}

	protected:

		//inherited from QTreeView
		QItemSelectionModel::SelectionFlags selectionCommand(const QModelIndex& index, const QEvent* event = nullptr) const override;
	};

	//Custom QTreeView widget
	class HWDBTree : public QTreeView
	{
		Q_OBJECT

	public:
		HWDBTree();
		//! Default constructor
		/** \param dbTreeWidget widget for DB tree display
		\param propertiesTreeWidget widget for selected entity's properties tree display
		\param parent widget QObject parent
		**/
		HWDBTree(HWCustomQTreeView* dbTreeWidget, QTreeView* propertiesTreeWidget, QObject* parent = nullptr);
		~HWDBTree();

		void Clear();

		//增加一个Item
		void AddItemsView(HWObject* in_object);

		//hide Params
		void HidePropertiesParamsView();
		//
		void UpdatePropertiesParamsView();
		
		//add element
		void AddElement(HWObject* in_object);
		
		//! Removes an element from the DB tree
		void RemoveElement(HWObject* in_object);

		//! Remove selected
		void RemoveElements(std::vector<HWObject*> in_objects);
		
		//!find a element in db tree
		HWObject* Find(int unique_id) const;

		void ChangeSelection(const QItemSelection & selected, const QItemSelection & deselected);
		void ReflectObjectPropChange(HWObject* in_obj);
		void RedrawObject(HWObject* in_object);
		void UpdateObject(HWObject* in_object);
		void DeleteSelectedEntities();
		//std::map<int, HWObject*>& AccessTreeElements();
		std::map<std::string, HWObject*>& AccessTreeElements();
		void SaveCurrentObject(HW::PlyFormat file_type);

		//! Associated widget for DB tree
		QTreeView* hw_db_tree_widget_;

		//! Associated widget for selected entity's properties tree
		QTreeView* hw_properties_widget_;
		
		//当前被选中的对象名字，也是唯一的标识符
		std::string current_element_idx_;
		//int current_element_idx_;
		HWObject* selected_element_object_;

	protected:
		//! Context menu action: expand tree branch
		//QAction* expand_branch_;
		//! Context menu action: sort children in alphabetical order
		QAction* sort_childrenAZ_;
		//! Context menu action: sort children by type
		QAction* sort_children_type_;
		//! Context menu action: select object by type and/or by name
		QAction* select_by_type_and_name_;
		//! Context menu action: delete selected entities
		QAction* delete_selected_entities_;
		//! Context menu action: enabled/disable selected entities
		QAction* toggle_selected_entities_;
		//! Context menu action: hide/show selected entities
		QAction* toggle_selected_entities_visibility_;
		//! Context menu action: hide/show selected entities color
		QAction* toggle_selected_entities_color_;
		//! Context menu action: hide/show selected entities normals
		QAction* toggle_selected_entities_normals_;

		//! Last context menu pos
		QPoint m_context_menu_pos_;
	private:

		std::map<std::string, HWObject*> tree_elements_;
		//std::map< int, HWObject*> tree_elements_;

		//db tree view
		//! Entity property that can be toggled
		enum TOGGLE_PROPERTY {
			TG_ENABLE,
			TG_VISIBLE,
			TG_COLOR,
			TG_SF,
			TG_NORMAL,
			TG_MATERIAL,
			TG_3D_NAME
		};

		//! Toggles a given property (enable state, visibility, normal, color, SF, etc.) on selected entities
		void toggleSelectedEntitiesProperty(TOGGLE_PROPERTY prop);

		void ShowContextMenu(const QPoint&);

		void ExpandBranch();
		void SortChildrenAZ();
		void SortChildrenType();
		void SelectByTypeAndName();

		inline void ToggleSelectedEntities() { toggleSelectedEntitiesProperty(TG_ENABLE); }
		inline void ToggleSelectedEntitiesVisibility() { toggleSelectedEntitiesProperty(TG_VISIBLE); }
		inline void ToggleSelectedEntitiesColor() { toggleSelectedEntitiesProperty(TG_COLOR); }
		inline void ToggleSelectedEntitiesNormals() { toggleSelectedEntitiesProperty(TG_NORMAL); }

	signals:
		void DBSelectionChanged();
		void DBIsEmpty();
		void DBIsNotEmptyAnymore();
	};
}

#endif