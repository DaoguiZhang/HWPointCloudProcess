#ifndef CLUSTERING_DECIMATION_H
#define CLUSTERING_DECIMATION_H

#include "ml_mesh_type.h"
#include <vcg/complex/complex.h> 
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/clustering.h>
/*
*summary: do clustering decimation.
*parameters:
*    MeshType: input & output mesh
*    CellNum £ºthe approximate total number of cells composing the grid surrounding the objects
*    CellSize£ºthe absolute length of the edge of the grid cell.
*    Notes:
*       CellNum is used only if the cell edge IS zero.
*       CellSize gives you an absolute measure of the maximum error introduced during the simplification (e.g. half of the cell edge length)
*/

void clusteringDecimation(CMeshO& m,
    float CellSize = 0.f,
    int CellNum = 1000000000,
    bool DupFace = false);

#endif // !CLUSTERING_DECIMATION_H