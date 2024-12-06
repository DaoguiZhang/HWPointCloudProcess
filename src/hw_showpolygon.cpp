#include "hw_showpolygon.h"
namespace HW
{
	HWShowPolygon::HWShowPolygon()
	{
	}
	HWShowPolygon::~HWShowPolygon()
	{
	}
	void HWShowPolygon::Process(HWObject * in_element, HWObject * out_element)
	{
		#ifdef LIULINGFEI
		std::cout << "HWShowPolygon::Process" << std::endl;
		#endif
		HWPolygon* in_polygon = dynamic_cast<HWPolygon*> (in_element);
		in_polygon->SetObjectType(ElementType::kHWPolygon);
		PolygonWindow* widget = new PolygonWindow();
		widget->resize(200, 100);
		widget->move(10, 10);
		widget->SetData(in_polygon);
		#ifdef LIULINGFEI
		std::cout << "PolygonWindow::SetData done" << std::endl;
		#endif
		widget->show();
		//�������Polygon
		resulted_element_ = in_element;
		//���������ʾ��
		polyview_widget_ = widget;
	}
}