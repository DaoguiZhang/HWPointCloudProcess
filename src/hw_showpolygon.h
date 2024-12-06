#include "hw_pcl_functor.h"
#include "hw_polygon.h"
#include "PolygonWindow.h"
namespace HW
{
	class HWShowPolygon :public HWPCLFunctor
	{

	public:
		HWShowPolygon();
		~HWShowPolygon();

		void Process(HWObject* in_element, HWObject* out_element);
		PolygonWindow* polyview_widget_;
	};
}