#include "hw_object.h"

namespace HW
{
	HWObject::HWObject()
	{

	}
	HWObject::~HWObject()
	{

	}
	bool HWObject::Show() 
	{
		return true;
	}
	bool HWObject::Save(std::string& file)
	{
		return true;
	}
	const ElementType& HWObject::GetObjectType()
	{
		return object_type_;
	}

	void HWObject::SetObjectType(ElementType obj_type)
	{
		object_type_ = obj_type;
	}

	void HWObject::SetObjectName(std::string object_name)
	{
		object_name_ = object_name;
	}

	const std::string& HWObject::GetObjectName() const
	{
		return object_name_;
	}
}