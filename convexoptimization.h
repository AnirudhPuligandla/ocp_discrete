#ifndef CONVEXOPTIMIZATION_H
#define CONVEXOPTIMIZATION_H

#define CL_HPP_ENABLE_EXCEPTIONS
#define CL_HPP_TARGET_OPENCL_VERSION 200

#include <cl2.hpp>
#include <math.h>
#include <memory>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <QFile>
#include <QDataStream>
#include <QDirIterator>
#include <string>
#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QProcess>
#include <ilcplex/ilocplex.h>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>
ILOSTLBEGIN

//using namespace boost::numeric::ublas;

class ConvexOptimization
{
public:
    // Structure to define the view frustum
    struct Frustum
    {
        /* Far plane
         * Although 4 points can be used, we use only 3 because three points sufficient to define the plane
         */
        QVector<QVector3D> far;
        // Top, bottom, left and right planes
        QVector<QVector3D> top, bottom, left, right;
    };

    // Constructor
    ConvexOptimization();

    /* Function to setup variable g
     * inputs -- number of control points 'controlPoints'
     *        -- number of possible camera locations 'comPos'
     *        -- CPLEX model
     *        -- 'var' to contain output camera locations
     *        -- 'c' to contain covered control points after optimization
     *        -- 'con' to store constraints
     */
    void setup_g(QVector<QVector3D> controlPoints, QVector<QVector3D> camPos, QVector<QVector<QVector3D>> normals);

    /* Function to calculate the view frustum for a given camera
     * inputs -- camPos -> camera location
     *        -- CamDir -> Camera orientation
     */
    Frustum calcFrustum(QVector3D camPos, QVector3D CamDir);

    /* Function to check if a point lies inside the FOV frustum
     * inputs -- camFOV -> camera FOV pyramid
     *        -- point -> control point to be checked
     * output -- 'true' if the point lies inside the FOV pyramid, 'false' otherwise
     */
    bool checkControlPoint(Frustum camFOV, QVector3D point);

    /* Function to optimize the objective
     *
     * ControlPoints -> total number of control points
     * camPos -> number of possible camera positions
     * camDir -> Camera orientation (For now it the direction of the voxel normal)
     * optCamPos -> container for optimized camera locations returned by the optimization class
     * contvals -> To store control points after optimization
     * optCamInd -> How indices of optimized camera locations
     * optRotInd -> To hold index of one of the 9 possible rotations at optimized camera locations
     */
    void optimizeObjective(QVector<QVector3D> controlPoints, QVector<QVector3D> camPos, QVector<QVector<QVector3D>> camDir, QVector<QVector3D> &optCamPos, QVector<bool> &contVals, QVector<int> &optCamInd, QVector<int> &optRotInd);

    /* Function to connect to remote CPU machine through ssh
     * and execute the optimization program present there
     * inputs -- numCam -> number of possible camera locations
     *        -- numControl -> number of control points
     * outputs -- The remote program writes output two files in the same location
     */
    void optimizeRemote(int numCam, int numControl, int rotations);

    /* Function to return camera angles
     * -- Returns camera horizontal and vertical angles in a qvector
     */
    void getFOVAngles(float &alpha_h, float &alpha_v);

private:
    // Check constructor implementation for details on these variables
    float alpha_r, alpha_l, alpha_t, alpha_b, f, s_u, s_v;
    int z_n, z_f, z_s, M;
    float c_xy, c_foc, c_z, c_r;
    float w_xy, _w_xy, w_foc, _w_foc, w_z, _w_z, w_r, _w_r;
    float hFar, vFar;

    // Max number of cameras
    int nCam;

    // 2D array for variable g_j,i
    //QVector<QVector<bool>> g;
    //std::map<std::pair<int, int>, bool > g;
    //std::map<int, std::map<int, bool>> g;
    boost::numeric::ublas::compressed_matrix<IloBool> g;

    // World up vector
    QVector3D worldUp;

    // Function to optimize using greedy algorithm
    void greedyAlgo(QVector<QVector3D> arrayControl, QVector<QVector3D> arrayCamPos, QVector<QVector<QVector3D> > normals, QString dirName);
};

#endif // CONVEXOPTIMIZATION_H
