#ifndef VOXELWORLD_H
#define VOXELWORLD_H

#include <PolyVoxCore/PolyVoxCore/MaterialDensityPair.h>
#include <PolyVoxCore/PolyVoxCore/CubicSurfaceExtractorWithNormals.h>
#include <PolyVoxCore/PolyVoxCore/SurfaceMesh.h>
#include <PolyVoxCore/PolyVoxCore/SimpleVolume.h>
#include <PolyVoxCore/MarchingCubesSurfaceExtractor.h>

#include "loadobj.h"
#include "loadbinvox.h"
#include "convexoptimization.h"
#include <QVector>
#include <QVector3D>
#include <QDebug>
#include <qopengl.h>
#include <QString>
#include <QTime>
#include <QMatrix4x4>
#include <QVector4D>
#include <math.h>

using namespace PolyVox;

typedef unsigned char byte;

class VoxelWorld
{
public:
    VoxelWorld();

    // Function to load model from .obj file and set voxels at object locations
    void createObject(QString &path);

    // Function to load Voxel model from .binvox file and generate voxel world
    void createVoxObject(QString &path);

    // Extract a mesh
    void generateMesh();

    /* Function called from glwidget that implicitly calls loadOBJ::loadModel
     * arguments : path = path to the .obj or .binvox file
     *             out_vertices = vertices of the reconstructed mesh from occupied voxels
     *             out_indices = vertex indices
     */
    bool getMesh(QVector<GLfloat> &out_vertices, QVector<int> &out_indices);

    // Function to get the voxel environment to be rendered
    bool getVertexList(QVector<GLfloat> &out_vertices, QVector<int> &out_values);

    /* Function to perform morphological dilation on the voxel object
     */
    void dilateObject();

    /* Function to create normals for boundary voxels
     * - This function operates only on the object boundary voxels
     *   (Possible camera positions)
     * - Gradient of the volume is obtained using a 3D sobel filter
     * - (-ve)Gradient will be the voxel normal
     */
    void generateNormals();

    // Function to initialize the background
    void getBackground();

    // Apply Sobel filter in x direction
    float sobelX(QVector3D voxPos);

    // Apply Sobel filter in y direction
    float sobelY(QVector3D voxPos);

    // Apply Sobel filter in z direction
    float sobelZ(QVector3D voxPos);

    // Function to generate a 3D filter around a point
    QVector<QVector3D> getNeighbours(QVector3D filterCenter, int filterSize);

    // Function to optimize possible camera locations and return the corresponding coverage vertices
    void OptimizeCamPos(QVector<GLfloat> &out_verticesCam, QVector<GLfloat> &out_verticesCont);

    // Function to return vertices representing axes
    void drawAxes(QVector<GLfloat> &out_vertices);

private:
    SimpleVolume<MaterialDensityPair44> volData;

    QVector<QVector3D> vertexList;
    LoadOBJ meshObj;

    // Mesh object
    SurfaceMesh<PositionMaterialNormal> mesh;

    //binvox object
    LoadBinVox voxObj;

    // byte array to get voxel information from binvox
    byte *voxObject;

    // Optimization class object
    ConvexOptimization optimize;

    // World Center
    QVector3D worldCenter;

    // Voxel object center (it is cubic because we use binvox)
    QVector3D objectCenter;

    // Distance between object center and world center
    QVector3D centerDist;

    // Coordinates of occupied voxels in voxObject
    QVector<QVector3D> voxObjectCoords;

    // Background voxel positions
    QVector<QVector3D> backgroundPoints;

    // Boundary voxel coordinates
    QVector<QVector3D> boundaryVoxels;

    // Normals of boundary voxels
    QVector<QVector<QVector3D>> boundaryNormals;

    // Object boundaries
    int min_x, min_y, min_z, max_x, max_y, max_z;

    /* Function to rotate given surface normal about specified axis and angle
     * inputs -> inNormal -- surface normal vector
     *           axis -- '1' x-axis or '2' y-axis or '3' z-axis
     *           angle -- angle in degrees by which normal is to be rotated
     * outputs --> Rotated QVector3D
     */
    QVector3D getRotations(QVector3D inNormal, int axis, float angle);

};

#endif // VOXELWORLD_H
