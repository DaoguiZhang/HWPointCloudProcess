#pragma once

#ifndef PLY_FILTER_HEADER
#define PLY_FILTER_HEADER

#include "hw_point_cloud.h"
#include "hw_mesh.h"

//#include "FileIOFilter.h"
#include "rply.h"

//! PLY format types
static const char e_ply_type_names[][12] = {
	"PLY_INT8", "PLY_UINT8", "PLY_INT16", "PLY_UINT16",
	"PLY_INT32", "PLY_UIN32", "PLY_FLOAT32", "PLY_FLOAT64",
	"PLY_CHAR", "PLY_UCHAR", "PLY_SHORT", "PLY_USHORT",
	"PLY_INT", "PLY_UINT", "PLY_FLOAT", "PLY_DOUBLE",
	"PLY_LIST"
};

//! PLY format storage modes
static const char e_ply_storage_mode_names[][24] =
{ "PLY_BIG_ENDIAN","PLY_LITTLE_ENDIAN","PLY_ASCII","PLY_DEFAULT" };

//! PLY file properties
struct plyProperty
{
	p_ply_property prop;
	const char* propName;
	e_ply_type type;
	e_ply_type length_type;
	e_ply_type value_type;
	int elemIndex;
};

//! PLY file nuclear element
struct plyElement
{
	p_ply_element elem;
	const char* elementName;
	long elementInstances;
	std::vector<plyProperty> properties;
	int propertiesCount;
	bool isFace;
};

//! Stanford PLY file I/O filter
class PlyFilter
{
public:
	PlyFilter();

	//static accessors
	static void SetDefaultOutputFormat(e_ply_storage_mode format);

	//inherited from FileIOFilter
	bool LoadFile(const std::string& filename, HW::HWObject& out_object);

	//bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;
	//CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;

	//! Custom loading method
	//CC_FILE_ERROR loadFile(const QString& filename, const QString& textureFilename, ccHObject& container, LoadParameters& parameters);

//private:
	//! Internal method
	//CC_FILE_ERROR saveToFile(ccHObject* entity, QString filename, e_ply_storage_mode storageType);
};

#endif //CC_PLY_FILTER_HEADER
