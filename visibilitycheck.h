#ifndef VISIBILITYCHECK_H
#define VISIBILITYCHECK_H

#define CL_HPP_ENABLE_EXCEPTIONS
#define CL_HPP_TARGET_OPENCL_VERSION 200

#include <cl2.hpp>
#include <memory>
#include <algorithm>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <vector>
#include <sys/stat.h>

#include <QVector>
#include <QVector3D>
#include <QDebug>
#include <QFile>
#include <QDataStream>
#include <QString>
#include <QByteArray>

class VisibilityCheck
{
private:
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

    // Structure for cameras
    struct cameraSpecs
    {
        // horizontal and vertical FoV angles
        float hFOV, vFOV;
        // depth of field
        float z_f;
        // horizontal and vertical half FoV values in voxels
        int hFar, vFar;
    };

    // parameter to specify how many vehicle points are allowed in camera FoV
    int numVehAllowed;
    // parameter to specify min. requirec coverage by a potential camera
    int minReqBackCov;
    // percentage of set minBackCov value by total for that model and size
    int minBackPercent;
    // gMatrix file name
    QString gFileName, indFileName;

    // private data members to setup input data
    QVector<QVector3D> camPos, backgroundVox, vehicleVox;
    QVector<QVector<QVector3D>> camDir, camUp, camRight;
    /* Vector to store indexes in 1D
         * Instead of creating new vectors to store only the selected points,
         * We use this vector to refer to the selected points within this class.
         * While, the gmatrix can be constructed only with the selected points.
         */
    QVector<QVector<int>> indices1D;
    // OpenCL program
    cl::Program camProgram;
    // one plus 6 camera specs
    cameraSpecs onePlus6;
    // Sparse compressed matrix 'gMatrix'
    QVector<boost::numeric::ublas::compressed_matrix<bool>> gMatrix;
    // Up-dated matrix in only two dimensions instead of three
    boost::numeric::ublas::compressed_matrix<bool> gMatrix2D;

    // Function to setup camera parameters. Since used by 2 constructors,
    // it is better have this at one place and call from both constructors.
    // This will ensure there is no mis-match in camera parameters
    void initCamParams();
    // function to initialize camParams for smaller simulated vehicle models
    void initCamParamsMini();
    // camera parameters for medium vehicles
    void initCamParamsM();
    // camera parameters for large vehicles
    void initCamParamsL();
    // camera parameters for huge vehicles
    void initCamParamsH();
    //------------parameters for real vehicle models-------
    void initCamParams16();
    void initCamParams32();
    void initCamParams64();
    void initCamParams128();
    void initCamParams256();

public:
    // Default constructor
    VisibilityCheck();
    // Overloaded Constructor
    //VisibilityCheck(QVector<QVector3D> &boundaryVoxels, QVector<QVector3D> &boundaryNormals, QVector<QVector3D> &backgroundVoxels);
    // Overloaded constructor that also performs visibility checks
    VisibilityCheck(QVector<QVector3D> &boundaryVoxels,
                    QVector<QVector3D> &boundaryNormals,
                    QVector<QVector3D> &backgroundVoxels,
                    QVector<QVector3D> &vehicleVoxels,
                    int numRotations);
    /* Function to initialize GPU and associated parameters
     * Also initialize jernel strings
     */
    void setupGPU();
    /* Function to calculate camera up and right vectors
     * this function will also calculate different orientations for each camera position
     */
    void setupCameras();
    /* Funtion to setup the g matrix and write into a file
     */
    void setup_g();
    // function to read gMatrix from file
    void load_g();
    // Function to return the created gMatrix
        void getGMatrix(QVector<boost::numeric::ublas::compressed_matrix<bool>> &gMat){gMat = gMatrix;}
        // Function to return number of created rotations at each camer location
        int getNumRotations(){return camDir.size();}
        /* Getter function to get gMatrix in 2D (control points x (camera positions and orientations put together into 1D))
         * Also returns a mapping of camera positions and orientations indices to 1D (will be useful when self occlusion is implemented)
         * index1D -> each entry points to index [camPos][orientation]
         */
        void getMatrixAnd1D(boost::numeric::ublas::compressed_matrix<bool> &gMat2D, QVector<QVector<int>> &index1D);
        /* Function to verify solution and assign values to background voxels based on camera
         * Note: This also works for greedy algorithm to recheck point occupation for given cam points
         *
         * input - indices of the solution (position + orientation)
         *         backVoxPoints - background voxel points
         * output - Ouput gmatrix gMat2D (required only for greedy algorithm, not for checking solution occupation)
         *          QVector<QVector<int>> showing set of background voxels covered with respective camera
         */
        void getSolutionoccupation(QVector<QVector<int>> &camSolution,
                                   QVector<QVector3D> &backVoxPoints,
                                   boost::numeric::ublas::compressed_matrix<bool> &gMat2D);
        /* Function to get occupied background points by a set of camera points given their three direction vectors
         * The returned indices of the background points correspond to the input backgroundpoints vector
         * inputs -> Set of camera points 'camPosVec'
         *        -> three QVectors3D s for camera direction, up and right vectors
         *        -> set of backgroundPoints 'backPosVec'
         * Output -> Single vector of occupied background points for all the input camera points
         */
        void getBackPointsSet(QVector<QVector3D> &camPosVec,
                              QVector3D &camDirVector,
                              QVector3D &camUpVector,
                              QVector3D &camRightVector,
                              QVector<QVector3D> &backVoxPoints,
                              QVector<QVector3D> &occupiedBackPoints);
        /* Function to calculate coverage given camera points with three direction vectors
         * and a set of background points (usually the complete set without even random sampling)
         *
         * Note: This is an alternative function to 'getSolutionOccupation()' and is independent of this class's object.
         *       While the abve function has to be used wih the same object that was used to setup the data, this one
         *       can be used with any newly created object
         *
         * inputs   -> Set of QVectors camPoints, camDir, camUp, camRight
         *          -> Set of background points for coverage checking
         * output   -> backgroundCoverage - indices of all covered points w.r.t to each camera point
         */
        void getCamCoverage(QVector<QVector3D> &camPosVec,
                            QVector<QVector3D> &camDirVec,
                            QVector<QVector3D> &camUpVec,
                            QVector<QVector3D> &camRightVec,
                            QVector<QVector3D> &backVoxPoints,
                            QVector<QVector<int>> &occupiedBackIndices);
        /* Function to get camera frustum given a cam point and three camera direction, up and right vectors
         */
        void getCamFrustum(QVector<QVector3D> &camPosVec,
                           QVector<QVector3D> &camDirVec,
                           QVector<QVector3D> &camUpVec,
                           QVector<QVector3D> &camRightVec,
                           QVector<QVector<QVector3D> > &camFOVs);
        /* Function to return the cam Dir, up and right vectors given, the point and orientation indices
         * input    -> QVector<int> with two entries for indices of camera point and orientation
         * outputs  -> QVector3D camDir
         *          -> QVector3D camUp
         *          -> QVector3D camRight
         */
        void getCamVectors(QVector<int> camIndices, QVector3D &solCamDir, QVector3D &solCamUp, QVector3D &solCamRight);
        // Function to compute neighbourhood in rwls algorithm
        void computeNeighboursGPU(boost::numeric::ublas::compressed_matrix<bool> &gMat2D_trans,
                               std::vector<std::vector<int> > &N_j);

};

#endif // VISIBILITYCHECK_H
