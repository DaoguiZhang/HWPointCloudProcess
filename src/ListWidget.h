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
		popMenu->addAction(new QAction("���", this));
		popMenu->addAction(new QAction("ɾ��", this));
		popMenu->addAction(new QAction("�޸�", this));

		popMenu->exec(QCursor::pos()); // �˵����ֵ�λ��Ϊ��ǰ����λ��
	}

};

#endif // !LIST_WIDDGET
