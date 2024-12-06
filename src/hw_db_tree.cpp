#include "hw_db_tree.h"
namespace HW
{

	HWDBTree::HWDBTree()
	{
		current_element_idx_.clear();
	}

	//HWDBTree::HWDBTree(HWCustomQTreeView* dbTreeWidget, QTreeView* propertiesTreeWidget, QObject* parent = nullptr)
	//{
	//	//
	//	assert(dbTreeWidget);
	//	hw_db_tree_widget_ = dbTreeWidget;
	//	hw_db_tree_widget_->setModel(this);
	//	hw_db_tree_widget_->header();
	//	//drag & drop support
	//	hw_db_tree_widget_->setDragEnabled(true);
	//	hw_db_tree_widget_->setAcceptDrops(true);
	//	hw_db_tree_widget_->setDropIndicatorShown(true);
	//	//context menu on DB tree elements
	//	hw_db_tree_widget_->setContextMenuPolicy(Qt::CustomContextMenu);
	//	expand_branch_ = new QAction("Expand branch", this);
	//	sort_childrenAZ_ = new QAction("Sort children by type", this);
	//	sort_children_type_  = new QAction("Sort children by type", this);
	//	select_by_type_and_name_ = new QAction("Select children by type and/or name", this);
	//	delete_selected_entities_ = new QAction("Delete", this);
	//	toggle_selected_entities_ = new QAction("Toggle", this);
	//	toggle_selected_entities_visibility_ = new QAction("Toggle visibility", this);
	//	toggle_selected_entities_color_ = new QAction("Toggle color", this);
	//	m_context_menu_pos_ = QPoint(-1, -1);
	//	//connect custom context menu actions
	//	connect(hw_db_tree_widget_, &QWidget::customContextMenuRequested, this, &HWDBTree::ShowContextMenu);
	//	connect(expand_branch_, &QAction::triggered, this, &HWDBTree::ExpandBranch);
	//	connect(sort_childrenAZ_, &QAction::triggered, this, &HWDBTree::SortChildrenAZ);
	//	connect(sort_children_type_, &QAction::triggered, this, &HWDBTree::SortChildrenType);
	//	connect(select_by_type_and_name_, &QAction::triggered, this, &HWDBTree::SelectByTypeAndName);
	//	connect(delete_selected_entities_, &QAction::triggered, this, &HWDBTree::DeleteSelectedEntities);
	//	connect(toggle_selected_entities_, &QAction::triggered, this, &HWDBTree::ToggleSelectedEntities);
	//	connect(toggle_selected_entities_visibility_, &QAction::triggered, this, &HWDBTree::ToggleSelectedEntitiesVisibility);
	//	connect(toggle_selected_entities_color_, &QAction::triggered, this, &HWDBTree::ToggleSelectedEntitiesColor);
	//	connect(toggle_selected_entities_normals_, &QAction::triggered, this, &HWDBTree::ToggleSelectedEntitiesNormals);
	//	/*connect(m_toggleSelectedEntitiesMat, &QAction::triggered, this, &ccDBRoot::toggleSelectedEntitiesMat);
	//	connect(m_toggleSelectedEntitiesSF, &QAction::triggered, this, &ccDBRoot::toggleSelectedEntitiesSF);
	//	connect(m_toggleSelectedEntities3DName, &QAction::triggered, this, &ccDBRoot::toggleSelectedEntities3DName);
	//	connect(m_addEmptyGroup, &QAction::triggered, this, &ccDBRoot::addEmptyGroup);
	//	connect(m_alignCameraWithEntity, &QAction::triggered, this, &ccDBRoot::alignCameraWithEntityDirect);
	//	connect(m_alignCameraWithEntityReverse, &QAction::triggered, this, &ccDBRoot::alignCameraWithEntityIndirect);
	//	connect(m_enableBubbleViewMode, &QAction::triggered, this, &ccDBRoot::enableBubbleViewMode);
	//	connect(m_editLabelScalarValue, &QAction::triggered, this, &ccDBRoot::editLabelScalarValue);*/
	//	//other DB tree signals/slots connection
	//	connect(hw_db_tree_widget_->selectionModel(), &QItemSelectionModel::selectionChanged, this, &HWDBTree::ChangeSelection);
	//}

	HWDBTree::~HWDBTree()
	{}

	void HWDBTree::Clear()
	{
		std::map<std::string, HWObject*>::iterator iter = tree_elements_.begin();
		for (; iter != tree_elements_.end(); ++iter)
		{
			if (iter->second != NULL)
			{
				delete iter->second;
			}
		}

		tree_elements_.clear();
		current_element_idx_.swap(current_element_idx_);
	}

	void HWDBTree::RedrawObject(HWObject* in_object)
	{
	}
	/*std::map<int, HWObject*>& HWDBTree::AccessTreeElements()
	{
		return tree_elements_;
	}*/
	std::map<std::string, HWObject*>& HWDBTree::AccessTreeElements()
	{
		return tree_elements_;
	}

	//void HWDBTree::ChangeSelection(const QItemSelection & selected, const QItemSelection & deselected)
	//{
	//	//first unselect
	//	QModelIndexList deselectedItems = deselected.indexes();
	//	for (int i = 0; i < deselectedItems.count(); ++i)
	//	{
	//		HWObject* element = static_cast<HWObject*>(deselectedItems.at(i).internalPointer());
	//		assert(element);
	//		if (element)
	//		{
	//			//element->setSelected(false);
	//			//element->prepareDisplayForRefresh();
	//		}
	//	}
	//	emit DBSelectionChanged();
	//}

	void HWDBTree::SaveCurrentObject(HW::PlyFormat file_type)
	{
		std::map<std::string, HWObject*>::iterator iter = tree_elements_.find(current_element_idx_);
		if (iter == tree_elements_.end())
		{
			printf("no object save...\n");
			return;
		}
		if (iter->second->GetObjectType() == kHWPointCloud)
		{
			iter->second->SavePly(iter->first, file_type);
		}
		else
		{
			HWMesh* selected_mesh = dynamic_cast<HWMesh*> (iter->second);
			if (!selected_mesh->HasTextureCoord())
				selected_mesh->SavePly(iter->first, file_type);
			else
				//selected_mesh->SavePly(iter->first, file_type);
				selected_mesh->SaveObj(iter->first);
		}
	}
}