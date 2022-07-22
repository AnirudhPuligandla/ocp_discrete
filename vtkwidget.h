#ifndef VTKWIDGET_H
#define VTKWIDGET_H

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkVersion.h>
#include <vtkOBJReader.h>
#include <vtkCamera.h>
#include <vtkTexture.h>
#include <vtkPolyData.h>
#include <vtkStructuredGridGeometryFilter.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkStructuredPoints.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkVolumeProperty.h>
#include <vtkPiecewiseFunction.h>
#include <vtkColorTransferFunction.h>
#include <vtkImageTranslateExtent.h>
#include <vtkGenericDataObjectWriter.h>
#include <vtkXMLImageDataWriter.h>
#include <vtkImageShiftScale.h>
#include <vtkCylinderSource.h>

#include <string>
#include <math.h>
#include <stdlib.h>

#include <QProcess>
#include <QDebug>
#include <boost/algorithm/string.hpp>
#include <QString>
#include <QVector>
#include <QVector3D>
#include <QDirIterator>
#include <QFile>
#include <QDataStream>
#include <QTime>
#include <QElapsedTimer>

#include "itkProcessing.h"
#include "visibilitycheck.h"
#include "heuristics.h"
#include "mipoptimizer.h"
#include "simulatedobjects.h"

#if VTK_VERSION_NUMBER >= 89000000000ULL
#define VTK890 1
#endif

#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>

class VTKWidget : public QVTKOpenGLNativeWidget
{
    Q_OBJECT

public:
    VTKWidget(QWidget *parent = 0);
    ~VTKWidget() = default;

    // Functions to set vtk widget dimensions
    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;

    // Renderer function
    void visualizeVTK(vtkNew<vtkActor> &actor);
    // Volume renderer function
    void visualizeVolume(vtkSmartPointer<vtkStructuredPoints> &voxData);
    //

public slots:
    void loadModelPressed();            // Load model from .obj files using vtk
    // slot to display QProcess errors
    void processError(QProcess::ProcessError error);
    /* Perform following pre-processing steps :
     * -- dilate volume
     * -- extract boundary and background voxels
     * -- voxel segmentation
     */
    void dilatePressed();               // Initialize data from model for optimization
    // Load simulated models and setup data for optimization
    void loadSimulated();
    // Function for continuous data model
    void continuousModel();
    /* Function for multi-resolution optimization
     * We will decide 2-3 levels of optimization where for each level camera points are clustered,
     * while simultaneously optimizing and repeat with only the selected clusters
     */
    void multiResolution();
    /* Function for combinatorial multi-resolution optimization
     * Works at different resolutions similarly as the other function
     * But, each resolution is seen a combinatorial optimization problem
     */
    void multiResolutionCombi();
    // function to run combi MR method multiple times in loop
    void combiMRLoop();
    /* Function for testing right number of clusters
     * Same as MR-combi method, but runs in a loop for
     * for diff. number of clusters
     */
    void MRClusterTest();
    /* Function to call optimizer function
     * Performs two kinds of optimizations using Google OR-tools
     * one is to find optimal solution using MIP optimizer
     * Other is to find feasible solutions using CP-SAT solover
     */
    void optimizePressed();
    /* Function to test segmentation method on simulated data
     */
    void simulateTest();
    /* Function to setup result for diplaying in paraview
     */
    void setupResultVolume();
    // Function to define environment dimensions as per size from binvox object
    void getEnvParams(int &extentVal, int &worldCent);
    // Function same as above to get bowl surface radii a1 and a2
    void getBowlRad(float &a1, float &a2);

protected:
    /* Function to optimization on the given data
     * Inputs   -> visibilityCheck - object to visibilityCheck class
     *          -> numCamReq - number of cameras to place
     *          -> backVoxSet - Set of background voxel points
     * Output   -> camSolution - vector with camera position and orientation indices
     * returns  -> time taken for optimization
     */
    int optimizeData(VisibilityCheck &visibilityCheck,
                      int numCamReq,
                     QVector<QVector3D> &backVoxSet,
                      QVector<QVector<int>> &camSolution);
    /* Function To setup and save the result volume with drawn frustums
     * inputs - visibilityCheck object to the visibility checks class
     *        - camSolution, placed camera solution indices
     *        - nameLabel specifying the number to be added at the saved volume file name
     */
    void SetupFinalVolume(QVector<QVector3D> &camSolution,
                          QVector<QVector3D> &camDirs,
                          QVector<QVector3D> &camUps,
                          QVector<QVector3D> &camRights,
                          std::string nameLabel);

private:
    vtkSmartPointer<vtkStructuredPoints> voxWorld;                      // private data memeber to store voxel data
    vtkSmartPointer<vtkStructuredPoints> volBoundary;                   // container for only the dilated boundary
    std::string fileNameVTK;                                            // Global base file

    ITKProcessing volumeProcessor;                                      // Private itkProcessing object to acess image processing on the volume

    QVector<QVector3D> boundaryVoxels;                                  // container for holding the list of voxels on boundary of the vehicle (represented with x,y,z index)
    QVector<QVector3D> vehicleVoxels;                                   // Container to hold voxels occupied by vehicle for self-occlusion visibility check
    QVector<QVector3D> backgroundVoxels;                                // Container to hold list of background points that need to be covered (represented with x,y,z index)
    QVector<QVector3D> boundaryNormals;                                 // Holds the list of surface normals at boundary voxels
    QVector<QVector3D> backgroundNormals;                               // Holds the list of surface normals for background voxels
    QVector<QVector3D> superBackVoxels;                                 // Holds the set of super background voxels after SLIC segmentation
    QVector<QVector3D> superBackNormals;                                // Holds the set of normals to superBackVoxels
    QVector<int> segmentBackLabels;                                     // Holds the set of labels for each background voxel after SLIC
    // vector to store indices of random selected background points
    QVector<QVector<int>> clusteredBackground;
    /* vector to hold 1D indices of 3D coordinates.
     * This is to avoid searching the big 'background voxels' vector.
     * particular entry in this corresponds to the index of that x,y,z
     * coordinate in either sets of boundary or background voxels.
     * Under the belief that uninitialized values are never accessed.
     */
    QVector<unsigned long int> vectorIndices;
    // Global to get back solution
    float backOccSOl;
    // High resolution visibility checks
    int visChecksPoints;

    // -----------------Selector variables-----------------------------
    /* 0 does D-SLIC clustering on voxel surface
     * 1 does D-SLIC clustering on volume
     */
    bool segChoice = 0;
    /* Variable to select segmentation algorithm
     * 0 selects adapted face clustering algorithm
     * 1 selects D-SLIC algorithm
     */
    bool clusterChoice = 1;
    /* Define single or bi-objective optimization
     * 0 selects single objective - coverage maximization
     * 1 selects number of camera minimization given min. coverage required (say, 90%)
     */
    bool objectiveChoice = 0;
    /* Selector for optimization algorithm
     * 0 ---> Selects Greedy algorithm
     * 1 ---> Selects Metro/SA algorithms
     * 2 ---> Selects CPLEX MIP optimization
     * 3 ---> rwls algorithm
     * 4 ---> LH-RPSO algorithm
     */
    int optimizeChoice = 0;
    // Selector variable for MR(1) or SR(0) methods
    bool mr_or_sr;
    // Number of resolutions declared global so it can be passed to rwls
    int numReso=0;
    // Selector for bowl-shaped surface background points
    bool bowlSurface = 1;
    // Variable to select file name and environment dimensions
    // 0 is default size (used all this while)
    int inSize = 32;
};

#endif // VTKGUI_H
