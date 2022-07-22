#ifndef ITKPROCESSING_H
#define ITKPROCESSING_H

#include <vtkImageData.h>
#include <vtkStructuredPoints.h>
#include <vtkSmartPointer.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <Eigen/Dense>

#include <itkVTKImageToImageFilter.h>
#include <itkFlatStructuringElement.h>
#include <itkBinaryDilateImageFilter.h>
#include <itkImageToVTKImageFilter.h>
#include <itkSobelOperator.h>
#include <itkNeighborhoodOperatorImageFilter.h>
#include <itkRescaleIntensityImageFilter.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/common/intensity.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>

#include <QVector>
#include <QVector3D>
#include <QDebug>
#include <QTime>
#include <QElapsedTimer>

#include <math.h>

//#include "visibilitycheck.h"

using PixelType = unsigned int;
using GradientPixelType = float;
//constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, 3>;
using GradientImageType = itk::Image<GradientPixelType, 3>;

// Define undirected graph with weights for edges
typedef boost::property<boost::edge_weight_t, double> EdgeWeight;
/* 1) setS container type for edges ensures that parallel edges do not exist.
 */
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeight> UndirectedGraph;
typedef boost::graph_traits<UndirectedGraph>::edge_iterator edge_iterator;
// Eigen namespace
//using namespace Eigen;

class ITKProcessing
{
public:
    // VTK image to ITK image conversion
    void vtkToItk(vtkSmartPointer<vtkStructuredPoints> &vtkData, ImageType::ConstPointer &itkData);
    // ITK image to VTK image conversion
    void itkToVtk(ImageType::ConstPointer &itkData, vtkSmartPointer<vtkStructuredPoints> &vtkData);
    // VTK to ITK conversion helper function to handle images of type float (for gradients)
    void floatVtkToItk(vtkSmartPointer<vtkStructuredPoints> &vtkData, GradientImageType::ConstPointer &itkData);
    // ITK to VTK conversion for floating point images
    void floatItkToVtk(GradientImageType::ConstPointer &itkData, vtkSmartPointer<vtkStructuredPoints> &vtkData);
    // Function for morphological dilation
    void volumeDilation(vtkSmartPointer<vtkStructuredPoints> &voxData);
    /* Function to calculate surface normals on the boundary voxels -- uses sobel filters
     * voxData         -> input voxel volume
     * gradientVolumes -> output 3 volumes for x,y,z gradients
     */
    void generateNormals(vtkSmartPointer<vtkStructuredPoints> &voxData, QVector<vtkSmartPointer<vtkStructuredPoints>> &gradientVolumes);
    /* Function to do SLIC segmentation on the boundary voxels
     * Inputs : set of 'boundaryVoxels' representing the voxels on the boundary after dilation
     *          set of 'boundaryNormals' representing the voxel normals
     *          Number of required super voxels 'clusters'
     * Outputs : set of 'segmentedVoxelCenters' containing the centers of segmented super voxels
     *           set of 'segmentedVoxelNormals' representing the normal vectors of super voxels
     *           set of labels 'labels' for each voxel on the object boundary
     */
    void slicSegmentation(QVector<QVector3D> &boundaryVoxels,
                          QVector<QVector3D> &boundaryNormals,
                          vtkSmartPointer<vtkStructuredPoints> &voxData,
                          int clusters,
                          QVector<QVector3D> &segmentedVoxelCenters,
                          QVector<QVector3D> &segmentedVoxelNormals,
                          QVector<int> &labels,
                          unsigned int volumeValChoice,
                          QVector<unsigned long int> &vectorIndex);
    // New dslic function from "DSlic" project
    void newDslicSeg(QVector<QVector3D> &boundaryVoxels,
                     QVector<QVector3D> &boundaryNormals,
                     int clusters,
                     QVector<QVector3D> &segmentedVoxelCenters,
                     QVector<QVector3D> &segmentedVoxelNormals,
                     QVector<int> &labels);
    // Fibonacci binning function to use with brute force approach
    void fibonacciBruteForce(pcl::PointCloud<pcl::PointXYZINormal>::Ptr downCloud,
                             int numClust,
                             std::vector<int> &clustCent,
                             std::vector<pcl::PointXYZINormal> &clustCentPoints,
                             pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloudWithNormals);
    // function to calculate distance between two points
    float computeDistance(pcl::PointXYZINormal centPoint, pcl::PointXYZINormal neighPoint, double searchRad);
    /* Function implementing the surface face clustering algorithm
     * Inputs: set of 'boundaryVoxels' representing the voxels on the boundary after dilation
     *          set of 'boundaryNormals' representing the voxel normals
     *          Number of required super voxels 'clusters'
     * Output: set of 'segmentedVoxelCenters' containing the centers of segmented super voxels
     *           set of 'segmentedVoxelNormals' representing the normal vectors of super voxels
     *           set of labels 'labels' for each voxel on the object boundary
     *          vector containing 1D indices for 'boundaryVoxels'
     */
    void faceClustering(QVector<QVector3D> &boundaryVoxels,
                        QVector<QVector3D> &boundaryNormals,
                        int clusters,
                        QVector<unsigned long> &vectorIndex,
                        vtkSmartPointer<vtkStructuredPoints> &voxData,
                        QVector<QVector3D> &segmentedVoxelCenters,
                        QVector<QVector3D> &segmentedVoxelNormals,
                        QVector<int> &labels);

private:
    /* function to return the minimum and maximum extent of occupied voxels
     * input - vtk voxel volume
     * output: 'extentObj' 6 integers specifiying the extent of occupied voxels
     */
    void getOccupiedExtent(vtkSmartPointer<vtkStructuredPoints> &voxData, int (&extentObj)[6]);
    // Parameter S in coputing distance for clustering
    float paramS = 1.0f;
    float paramN = 0.5f;
    // GPU execution class object
    //VisibilityCheck gpuObject;
};

#endif // ITKPROCESSING_H
