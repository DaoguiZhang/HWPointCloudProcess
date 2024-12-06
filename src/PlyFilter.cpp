#include "PlyFilter.h"


//static int vertex_cb(p_ply_argument argument)
//{
//	//if (s_NotEnoughMemory)
//	//{
//	//	//skip the next pieces of data
//	//	return 1;
//	//}
//	long flags;
//	HW::HWPointCloud* cloud;
//	ply_get_argument_user_data(argument, (void**)(&cloud), &flags);
//
//	double val = ply_get_argument_value(argument);
//
//	cloud->AddPoint()
//
//	static CCVector3d s_Point(0, 0, 0);
//
//	// This looks like it should always be true, 
//	// but it's false if x is NaN.
//	if (val == val)
//	{
//		s_Point.u[flags & POS_MASK] = val;
//	}
//	else
//	{
//		//warning: corrupted data!
//		s_PointDataCorrupted = true;
//		s_Point.u[flags & POS_MASK] = 0;
//		//return 0;
//	}
//
//	if (flags & ELEM_EOL)
//	{
//		//first point: check for 'big' coordinates
//		if (s_PointCount == 0)
//		{
//			bool preserveCoordinateShift = true;
//			if (FileIOFilter::HandleGlobalShift(s_Point, s_Pshift, preserveCoordinateShift, s_loadParameters))
//			{
//				if (preserveCoordinateShift)
//				{
//					cloud->setGlobalShift(s_Pshift);
//				}
//				ccLog::Warning("[PLYFilter::loadFile] Cloud (vertices) has been recentered! Translation: (%.2f ; %.2f ; %.2f)", s_Pshift.x, s_Pshift.y, s_Pshift.z);
//			}
//		}
//
//		cloud->addPoint(CCVector3::fromArray((s_Point + s_Pshift).u));
//		++s_PointCount;
//
//		s_PointDataCorrupted = false;
//		if ((s_PointCount % PROCESS_EVENTS_FREQ) == 0)
//			QCoreApplication::processEvents();
//	}
//
//	return 1;
//}
//
//static int normal_cb(p_ply_argument argument)
//{
//	if (s_NotEnoughMemory)
//	{
//		//skip the next pieces of data
//		return 1;
//	}
//	long flags;
//	ccPointCloud* cloud;
//	ply_get_argument_user_data(argument, (void**)(&cloud), &flags);
//
//	static CCVector3 s_Normal(0, 0, 0);
//	s_Normal.u[flags & POS_MASK] = static_cast<PointCoordinateType>(ply_get_argument_value(argument));
//
//	if (flags & ELEM_EOL)
//	{
//		cloud->addNorm(s_Normal);
//		++s_NormalCount;
//
//		if ((s_NormalCount % PROCESS_EVENTS_FREQ) == 0)
//			QCoreApplication::processEvents();
//	}
//
//	return 1;
//}
//
//static int rgb_cb(p_ply_argument argument)
//{
//	if (s_NotEnoughMemory)
//	{
//		//skip the next pieces of data
//		return 1;
//	}
//	long flags;
//	ccPointCloud* cloud;
//	ply_get_argument_user_data(argument, (void**)(&cloud), &flags);
//
//	p_ply_property prop;
//	ply_get_argument_property(argument, &prop, nullptr, nullptr);
//	e_ply_type type;
//	ply_get_property_info(prop, nullptr, &type, nullptr, nullptr);
//
//	static ccColor::Rgb s_color(0, 0, 0);
//
//	switch (type)
//	{
//	case PLY_FLOAT:
//	case PLY_DOUBLE:
//	case PLY_FLOAT32:
//	case PLY_FLOAT64:
//		s_color.rgb[flags & POS_MASK] = static_cast<ColorCompType>(std::min(std::max(0.0, ply_get_argument_value(argument)), 1.0) * ccColor::MAX);
//		break;
//	case PLY_INT8:
//	case PLY_UINT8:
//	case PLY_CHAR:
//	case PLY_UCHAR:
//		s_color.rgb[flags & POS_MASK] = static_cast<ColorCompType>(ply_get_argument_value(argument));
//		break;
//	default:
//		s_color.rgb[flags & POS_MASK] = static_cast<ColorCompType>(ply_get_argument_value(argument));
//		break;
//	}
//
//	if (flags & ELEM_EOL)
//	{
//		cloud->addRGBColor(s_color);
//		++s_ColorCount;
//
//		if ((s_ColorCount % PROCESS_EVENTS_FREQ) == 0)
//			QCoreApplication::processEvents();
//	}
//
//	return 1;
//}
//
//static int grey_cb(p_ply_argument argument)
//{
//	if (s_NotEnoughMemory)
//	{
//		//skip the next pieces of data
//		return 1;
//	}
//	ccPointCloud* cloud;
//	ply_get_argument_user_data(argument, (void**)(&cloud), nullptr);
//
//	p_ply_property prop;
//	ply_get_argument_property(argument, &prop, nullptr, nullptr);
//	e_ply_type type;
//	ply_get_property_info(prop, nullptr, &type, nullptr, nullptr);
//
//	ColorCompType G;
//
//	switch (type)
//	{
//	case PLY_FLOAT:
//	case PLY_DOUBLE:
//	case PLY_FLOAT32:
//	case PLY_FLOAT64:
//		G = static_cast<ColorCompType>(std::min(std::max(0.0, ply_get_argument_value(argument)), 1.0) * ccColor::MAX);
//		break;
//	case PLY_INT8:
//	case PLY_UINT8:
//	case PLY_CHAR:
//	case PLY_UCHAR:
//		G = static_cast<ColorCompType>(ply_get_argument_value(argument));
//		break;
//	default:
//		G = static_cast<ColorCompType>(ply_get_argument_value(argument));
//		break;
//	}
//
//	cloud->addGreyColor(G);
//	++s_IntensityCount;
//
//	if ((s_IntensityCount % PROCESS_EVENTS_FREQ) == 0)
//		QCoreApplication::processEvents();
//
//	return 1;
//}
//
//static int scalar_cb(p_ply_argument argument)
//{
//	if (s_NotEnoughMemory)
//	{
//		//skip the next pieces of data
//		return 1;
//	}
//	CCLib::ScalarField* sf = 0;
//	ply_get_argument_user_data(argument, (void**)(&sf), nullptr);
//
//	p_ply_element element;
//	long instance_index;
//	ply_get_argument_element(argument, &element, &instance_index);
//
//	ScalarType scal = static_cast<ScalarType>(ply_get_argument_value(argument));
//	sf->setValue(instance_index, scal);
//
//	if ((++s_totalScalarCount % PROCESS_EVENTS_FREQ) == 0)
//		QCoreApplication::processEvents();
//
//	return 1;
//}
//
//static bool s_unsupportedPolygonType = false;
//static int face_cb(p_ply_argument argument)
//{
//	if (s_NotEnoughMemory)
//	{
//		//skip the next pieces of data
//		return 1;
//	}
//	ccMesh* mesh = 0;
//	ply_get_argument_user_data(argument, (void**)(&mesh), nullptr);
//	if (!mesh)
//	{
//		assert(false);
//		return 1;
//	}
//
//	long length, value_index;
//	ply_get_argument_property(argument, nullptr, &length, &value_index);
//	//unsupported polygon type!
//	if (length != 3 && length != 4)
//	{
//		s_unsupportedPolygonType = true;
//		return 1;
//	}
//	if (value_index < 0 || value_index + 1 > length)
//	{
//		return 1;
//	}
//
//	static unsigned s_tri[4];
//	s_tri[value_index] = static_cast<unsigned>(ply_get_argument_value(argument));
//
//	if (value_index < 2)
//	{
//		return 1;
//	}
//
//	if (s_hasQuads && mesh->size() == mesh->capacity())
//	{
//		//we may have more triangles than expected
//		if (!mesh->reserve(mesh->size() + 1024))
//		{
//			s_NotEnoughMemory = true;
//			return 0;
//		}
//	}
//
//	if (value_index == 2)
//	{
//		mesh->addTriangle(s_tri[0], s_tri[1], s_tri[2]);
//		++s_triCount;
//
//		//specifc case: when dealing with quads, we must keep track of the real index(es) of the corresponding triangles
//		if (s_triIsQuad.capacity())
//		{
//			s_triIsQuad.push_back(false);
//		}
//
//		if ((s_triCount % PROCESS_EVENTS_FREQ) == 0)
//			QCoreApplication::processEvents();
//	}
//	else if (value_index == 3)
//	{
//		s_hasQuads = true;
//		if (s_hasMaterials)
//		{
//			//specifc case: when dealing with quads WITH materials, we must keep track of the real index(es) of the corresponding triangles
//			if (s_triIsQuad.capacity() == 0)
//			{
//				if (s_triCount)
//				{
//					s_triIsQuad.resize(s_triCount, false);
//				}
//				s_triIsQuad.reserve(2 * mesh->capacity());
//			}
//			s_triIsQuad.push_back(true);
//		}
//
//		mesh->addTriangle(s_tri[0], s_tri[2], s_tri[3]);
//		++s_triCount;
//
//		if ((s_triCount % PROCESS_EVENTS_FREQ) == 0)
//			QCoreApplication::processEvents();
//	}
//
//	return 1;
//}
//
//static unsigned s_texCoordCount = 0;
//static bool s_invalidTexCoordinates = false;
//static int texCoords_cb(p_ply_argument argument)
//{
//	if (s_NotEnoughMemory)
//	{
//		//skip the next pieces of data
//		return 1;
//	}
//
//	long length, value_index;
//	ply_get_argument_property(argument, nullptr, &length, &value_index);
//	//unsupported/invalid coordinates!
//	if (length != 6 && length != 8)
//	{
//		s_invalidTexCoordinates = true;
//		return 1;
//	}
//	if (value_index < 0 || value_index + 1 > length)
//	{
//		return 1;
//	}
//
//	static float s_texCoord[8];
//	s_texCoord[value_index] = static_cast<float>(ply_get_argument_value(argument));
//
//	if (((value_index + 1) % 2) == 0)
//	{
//		TextureCoordsContainer* texCoords = 0;
//		ply_get_argument_user_data(argument, (void**)(&texCoords), nullptr);
//		assert(texCoords);
//		if (!texCoords)
//			return 1;
//
//		if (texCoords->currentSize() == texCoords->capacity())
//		{
//			if (!texCoords->reserveSafe(texCoords->currentSize() + 1024))
//			{
//				s_NotEnoughMemory = true;
//				return 0;
//			}
//		}
//		texCoords->addElement(TexCoords2D(s_texCoord[value_index - 1], s_texCoord[value_index]));
//		++s_texCoordCount;
//
//		if ((s_texCoordCount % PROCESS_EVENTS_FREQ) == 0)
//			QCoreApplication::processEvents();
//	}
//
//	return 1;
//}
//
//static int s_maxTextureIndex = -1;
//static int texIndexes_cb(p_ply_argument argument)
//{
//	p_ply_element element;
//	long instance_index;
//	ply_get_argument_element(argument, &element, &instance_index);
//
//	int index = static_cast<int>(ply_get_argument_value(argument));
//	if (index > s_maxTextureIndex)
//	{
//		s_maxTextureIndex = -1;
//	}
//
//	ccMesh::triangleMaterialIndexesSet* texIndexes = 0;
//	ply_get_argument_user_data(argument, (void**)(&texIndexes), nullptr);
//	assert(texIndexes);
//	if (!texIndexes)
//	{
//		return 1;
//	}
//	texIndexes->addElement(index);
//
//	if ((texIndexes->currentSize() % PROCESS_EVENTS_FREQ) == 0)
//		QCoreApplication::processEvents();
//
//	return 1;
//}


bool PlyFilter::LoadFile(const std::string& filename, HW::HWObject& out_object)
{
	////reset statics!
	//s_triCount = 0;
	//s_unsupportedPolygonType = false;
	//s_texCoordCount = 0;
	//s_invalidTexCoordinates = false;
	//s_totalScalarCount = 0;
	//s_IntensityCount = 0;
	//s_ColorCount = 0;
	//s_NormalCount = 0;
	//s_PointCount = 0;
	//s_PointDataCorrupted = false;
	//s_NotEnoughMemory = false;
	//s_loadParameters = parameters;
	//s_Pshift = CCVector3d(0, 0, 0);
	//s_hasQuads = false;
	//s_hasMaterials = false;
	//s_triIsQuad.clear();

	/****************/
	/***  Header  ***/
	/****************/

	//open a PLY file for reading
	p_ply ply = ply_open(filename.c_str(), nullptr, 0, nullptr);
	if (!ply)
		return false;

	if (!ply_read_header(ply))
	{
		ply_close(ply);
		return false;
	}



	//storage mode: little/big endian
	e_ply_storage_mode storage_mode;
	//get_plystorage_mode(ply, &storage_mode);

	///*****************/
	///***  Texture  ***/
	///*****************/
	////eventual texture files declared in the comments (keyword: TEXTUREFILE)
	//QStringList textureFileNames;
	////texture coordinates
	//TextureCoordsContainer* texCoords = 0;
	////texture indexes
	//ccMesh::triangleMaterialIndexesSet* texIndexes = 0;
	///******************/
	///***  Comments  ***/
	///******************/
	//{
	//	//display comments
	//	const char* lastComment = nullptr;
	//	while ((lastComment = ply_get_next_comment(ply, lastComment)))
	//	{
	//		//specific case: TextureFile 'filename.ext'
	//		if (QString(lastComment).toUpper().startsWith("TEXTUREFILE "))
	//		{
	//			textureFileNames << QString(lastComment).mid(12).trimmed();
	//		}
	//	}
	//}

	////external texture filename?
	//if (!inputTextureFilename.isEmpty())
	//{
	//	//add it to the set of textures (if it's not already there!)
	//	if (!textureFileNames.contains(inputTextureFilename))
	//	{
	//		textureFileNames.push_back(inputTextureFilename);
	//	}
	//}

	/*******************************/
	/***  Elements & properties  ***/
	/*******************************/

	//Point-based elements (points, colors, normals, etc.)
	std::vector<plyElement> pointElements;
	//Mesh-based elements (vertices, etc.)
	std::vector<plyElement> meshElements;

	//Point-based element properties (coordinates, color components, etc.)
	std::vector<plyProperty> stdProperties;
	//Mesh-based multi-element properties (vertex indexes, etc.)
	std::vector<plyProperty> listProperties;
	//Mesh-based single-element properties (texture index, etc.)
	std::vector<plyProperty> singleProperties;

	try
	{
		//last read element
		plyElement lastElement;
		lastElement.elem = 0;
		while ((lastElement.elem = ply_get_next_element(ply, lastElement.elem)))
		{
			//we get next element info
			ply_get_element_info(lastElement.elem, &lastElement.elementName, &lastElement.elementInstances);

			if (lastElement.elementInstances == 0)
			{
				printf("[PLY] Element %s was ignored as it has 0 instance!", lastElement.elementName);
				continue;
			}

			lastElement.properties.clear();
			lastElement.propertiesCount = 0;
			lastElement.isFace = false;
			//printf("Element: %s\n",lastElement.elementName);

			//last read property
			plyProperty lastProperty;
			lastProperty.prop = 0;
			lastProperty.elemIndex = 0;

			while ((lastProperty.prop = ply_get_next_property(lastElement.elem, lastProperty.prop)))
			{
				//we get next property info
				ply_get_property_info(lastProperty.prop, &lastProperty.propName, &lastProperty.type, &lastProperty.length_type, &lastProperty.value_type);
				//printf("\tProperty: %s (%s)\n",lastProperty.propName,e_ply_type_names[lastProperty.type]);

				if (lastProperty.type == 16) //PLY_LIST
				{
					lastElement.isFace = true;
				}

				lastElement.properties.push_back(lastProperty);
				++lastElement.propertiesCount;
			}

			//if we have a "face-like" element
			if (lastElement.isFace)
			{
				//we store its properties in 'listProperties'
				for (size_t i = 0; i < lastElement.properties.size(); ++i)
				{
					plyProperty& prop = lastElement.properties[i];
					prop.elemIndex = static_cast<int>(meshElements.size());

					if (prop.type == 16)
					{
						//multiple elements per face (vertex indexes, texture coordinates, etc.)
						listProperties.push_back(prop);
					}
					else
					{
						//single element per face (texture index, etc.)
						singleProperties.push_back(prop);
					}
				}
				meshElements.push_back(lastElement);
			}
			else //else if we have a "point-like" element
			{
				//we store its properties in 'stdProperties'
				for (size_t i = 0; i < lastElement.properties.size(); ++i)
				{
					plyProperty& prop = lastElement.properties[i];
					prop.elemIndex = (int)pointElements.size();
					stdProperties.push_back(prop);
				}
				pointElements.push_back(lastElement);
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	//We need some points at least!
	if (pointElements.empty())
	{
		ply_close(ply);
		printf("read no points\n");
		return false;
	}


	/*************************/
	/***  Callbacks setup  ***/
	/*************************/

	//Main point cloud
	//HW::HWPointCloud* cloud = new HW::HWPointCloud();
}
