#pragma once
#ifndef HW_OBJECT_H
#define HW_OBJECT_H
#include <vector>
#include "hw_common.h"
#include "tinyply.h"

namespace HW 
{
	class HWObject
	{
	public:
		HWObject();
		~HWObject();
		virtual bool Show() = 0;
		virtual bool Save(std::string& file) = 0;
		virtual bool SavePly(const std::string& file, const PlyFormat& type) = 0;
		virtual bool ReadPly(const std::string& file) = 0;
		//virtual bool ReadPlyStream(tinyply::PlyFile& ply_file);

		const ElementType& GetObjectType();
		void SetObjectType(ElementType obj_type);

		virtual void SetObjectName(std::string object_name);
		virtual const std::string& GetObjectName() const;

		bool selected_;
		bool visibable_;

	private:
		//object type
		ElementType object_type_;
		std::string object_name_;
	};
}

#endif