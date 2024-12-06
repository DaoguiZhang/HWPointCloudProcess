#include "clustering_decimation.h"

void clusteringDecimation(CMeshO& m, float CellSize, int CellNum, bool DupFace)
{
    printf("function clusteringDecimation in!\n");

    vcg::tri::UpdateBounding<CMeshO>::Box(m);
    vcg::tri::UpdateNormal<CMeshO>::PerFace(m);
    printf("Input mesh  vn:%i fn:%i\n", m.VN(), m.FN());

    vcg::tri::Clustering<CMeshO, vcg::tri::AverageColorCell<CMeshO> > Grid;
    Grid.DuplicateFaceParam = DupFace;
    Grid.Init(m.bbox, CellNum, CellSize);

    printf("Clustering to %i cells\n", Grid.Grid.siz[0] * Grid.Grid.siz[1] * Grid.Grid.siz[2]);
    printf("Grid of %i x %i x %i cells\n", Grid.Grid.siz[0], Grid.Grid.siz[1], Grid.Grid.siz[2]);
    printf("with cells size of %.2f x %.2f x %.2f units\n", Grid.Grid.voxel[0], Grid.Grid.voxel[1], Grid.Grid.voxel[2]);

    Grid.AddMesh(m);
    Grid.ExtractMesh(m);
    printf("Output mesh vn:%i fn:%i\n", m.VN(), m.FN());

    printf("function clusteringDecimation out!\n");
    return;
}