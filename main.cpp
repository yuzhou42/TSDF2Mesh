#include <KrisLibrary/geometry/TSDFReconstruction.h>
#include <KrisLibrary/math3d/AABB3D.h>
#include <KrisLibrary/meshing/IO.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>

using namespace Geometry;
using namespace Meshing;
using namespace Math3D;
using namespace std;

#define CELL_SIZE 0.01
#define TRUNCATION_DISTANCE -1

int main (int argc, const char** argv) {
    const char* outfile = "out.obj";
    if(argc <= 1) {
        printf("USAGE: SparseTSDF2Mesh BIN_FILE [OBJ_FILE]");
        return 1;
    }
    const char* infile = argv[1];
    if(argc > 2) 
        outfile = argv[2];

    printf("Reading from %s\n",infile);
    string contents;
    if(!GetFileContents(infile,contents)) {
        printf("Could not read contents of file %s\n",infile);
        return 1;
    }
    if(contents.length() % sizeof(float) != 0) {
        printf("File size should be a multiple of 4, instead got %d",(int)contents.length());
        return 1;
    }
    int numValues = (int)contents.length()/sizeof(float);
    if(numValues % 4 != 0) {
        printf("# of floats in file must be a multiple of 4, instead got %d",numValues);
        return 1;
    }       
    int numPoints = numValues/4;
    float* values = (float*)&contents[0];

    float minValue = 1e100, maxValue = -1e100;
    AABB3D bbox;
    int k=0;
    for(int i=0;i<numPoints;i++,k+=4) {
        minValue = Min(minValue,values[k+3]);
        maxValue = Max(maxValue,values[k+3]);
        bbox.expand(Vector3(values[k],values[k+1],values[k+2]));
    }
    printf("Read %d points with distance in range [%g,%g]\n",numPoints,minValue,maxValue);
    printf("   x range [%g,%g]\n",bbox.bmin.x,bbox.bmax.x);
    printf("   y range [%g,%g]\n",bbox.bmin.y,bbox.bmax.y);
    printf("   z range [%g,%g]\n",bbox.bmin.z,bbox.bmax.z);

    float truncation_distance = TRUNCATION_DISTANCE;
    if(TRUNCATION_DISTANCE < 0) {
        //auto-detect truncation distance
        truncation_distance = Max(-minValue,maxValue)*0.99;
        printf("Auto-detected truncation distance %g\n",truncation_distance);
    }
    printf("Using cell size %g\n",CELL_SIZE);
    SparseTSDFReconstruction tsdf(Vector3(CELL_SIZE),truncation_distance);
    tsdf.tsdf.defaultValue[0] = truncation_distance;
    k=0;
    Vector3 ofs(CELL_SIZE*0.5);
    for(int i=0;i<numPoints;i++,k+=4) {
        tsdf.tsdf.SetValue(Vector3(values[k],values[k+1],values[k+2])+ofs,values[k+3]);
    }

    printf("Extracting mesh\n");
    TriMesh mesh;
    tsdf.ExtractMesh(mesh);
    printf("Saving mesh with %d tris to %s\n",(int)mesh.tris.size(),outfile);
    ofstream out(outfile);
    SaveOBJ(outfile,mesh);
    return 0;
}