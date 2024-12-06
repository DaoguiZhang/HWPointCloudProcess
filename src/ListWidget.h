#pragma once
#ifndef LIST_WIDGET
#define LIST_WIDGET

#include<qlistwidget.h>
#include<qmenu.h>

class ListWidget:public QListWidget
{
	void ListWidget::ContextMenuEvent(QContextMenuEvent * event)
	{
		QMenu* popMenu = new QMenu(this);
		popMenu->addAction(new QAction("添加", this));
		popMenu->addAction(new QAction("删除", this));
		popMenu->addAction(new QAction("修改", this));

		popMenu->exec(QCursor::pos()); // 菜单出现的位置为当前鼠标的位置
	}

};

#endif // !LIST_WIDDGET
