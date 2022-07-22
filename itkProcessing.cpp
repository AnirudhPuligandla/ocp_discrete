#include "itkProcessing.h"

void ITKProcessing::vtkToItk(vtkSmartPointer<vtkStructuredPoints> &vtkData, ImageType::ConstPointer &itkData)
{
    // VTK image to ITK image conversion filter
    using FilterType = itk::VTKImageToImageFilter<ImageType>;
    FilterType::Pointer filter = FilterType::New();
    filter->SetInput(vtkData);

    try {
        filter->Update();
    }
    catch (itk::ExceptionObject & error)
    {
        std::cerr << "ERROR: " << error << std::endl;
    }

    itkData = filter->GetOutput();
}

void ITKProcessing::itkToVtk(ImageType::ConstPointer &itkData, vtkSmartPointer<vtkStructuredPoints> &vtkData)
{
    // ITK image to VTK image conversion filter
    using FilterType = itk::ImageToVTKImageFilter<ImageType>;
    FilterType::Pointer filter = FilterType::New();
    filter->SetInput(itkData);
    try
    {
        filter->Update();
    }
    catch(itk::ExceptionObject & error)
    {
        std::cerr << "Error: " << error << std::endl;
    }

    vtkData->DeepCopy(filter->GetOutput());
}

void ITKProcessing::floatVtkToItk(vtkSmartPointer<vtkStructuredPoints> &vtkData, GradientImageType::ConstPointer &itkData)
{
    // VTK image to ITK image conversion filter
    using FilterType = itk::VTKImageToImageFilter<GradientImageType>;
    FilterType::Pointer filter = FilterType::New();
    filter->SetInput(vtkData);

    try {
        filter->Update();
    }
    catch (itk::ExceptionObject & error)
    {
        std::cerr << "ERROR: " << error << std::endl;
    }

    itkData = filter->GetOutput();
}

void ITKProcessing::floatItkToVtk(GradientImageType::ConstPointer &itkData, vtkSmartPointer<vtkStructuredPoints> &vtkData)
{
    // ITK image to VTK image conversion filter
    using FilterType = itk::ImageToVTKImageFilter<GradientImageType>;
    FilterType::Pointer filter = FilterType::New();
    filter->SetInput(itkData);
    try
    {
        filter->Update();
    }
    catch(itk::ExceptionObject & error)
    {
        std::cerr << "Error: " << error << std::endl;
    }

    vtkData->DeepCopy(filter->GetOutput());
}

void ITKProcessing::volumeDilation(vtkSmartPointer<vtkStructuredPoints> &voxData)
{
    // Convert vtk voxel image to itk image
    ImageType::ConstPointer itkData;
    vtkToItk(voxData, itkData);

    // We want to dilate the object (represented with the maximum intensity in the dataset)
    // Get maximum intensity value
    unsigned int maxVal = voxData->GetScalarRange()[1];
    qDebug() << "maximum intensity value: " << maxVal;

    // Create a 3D box structuring element
    using StructuringElementType = itk::FlatStructuringElement<3>;
    StructuringElementType::RadiusType elementRadius;
    elementRadius.Fill(1);              // Use radius=1 for a SE of dimensions 3x3x3
    StructuringElementType structuringElement = StructuringElementType::Box(elementRadius);

    // Create Binary dilate filter
    using BinaryDilateFilterType = itk::BinaryDilateImageFilter<ImageType, ImageType, StructuringElementType>;
    BinaryDilateFilterType::Pointer dilateFilter = BinaryDilateFilterType::New();
    dilateFilter->SetInput(itkData);
    dilateFilter->SetKernel(structuringElement);
    //dilateFilter->SetForegroundValue(2);
    //dilateFilter->SetBackgroundValue(2);
    dilateFilter->SetDilateValue(maxVal);
    dilateFilter->Update();
    itkData = dilateFilter->GetOutput();

    // Convert the output from dilation filter into vtk volume
    itkToVtk(itkData, voxData);
}

void ITKProcessing::generateNormals(vtkSmartPointer<vtkStructuredPoints> &voxData, QVector<vtkSmartPointer<vtkStructuredPoints> > &gradientVolumes)
{
    // Convert the vtk image to ITK image
    GradientImageType::ConstPointer itkData;
    floatVtkToItk(voxData, itkData);

    // Create 3D Sobel operators
    using SobelOperatorType = itk::SobelOperator<GradientPixelType, 3>;
    SobelOperatorType sobelOperatorX, sobelOperatorY, sobelOperatorZ;
    // Define kernel size -- 3x3x3
    itk::Size<3> radius;
    radius.Fill(1);
    // initialize sobel operators
    sobelOperatorX.SetDirection(0);
    sobelOperatorX.CreateToRadius(radius);
    sobelOperatorY.SetDirection(1);
    sobelOperatorY.CreateToRadius(radius);
    sobelOperatorZ.SetDirection(2);
    sobelOperatorZ.CreateToRadius(radius);

    // Filters for neighbourhood operations
    using NeighborhoodOperatorImageFilterType = itk::NeighborhoodOperatorImageFilter<GradientImageType, GradientImageType>;
    NeighborhoodOperatorImageFilterType::Pointer filterX = NeighborhoodOperatorImageFilterType::New();
    NeighborhoodOperatorImageFilterType::Pointer filterY = NeighborhoodOperatorImageFilterType::New();
    NeighborhoodOperatorImageFilterType::Pointer filterZ = NeighborhoodOperatorImageFilterType::New();

    // Filters for rescaling intensity values within [-1,1]
    using RescaleIntensityFiltertype = itk::RescaleIntensityImageFilter<GradientImageType, GradientImageType>;
    RescaleIntensityFiltertype::Pointer rescaleFilterX = RescaleIntensityFiltertype::New();
    RescaleIntensityFiltertype::Pointer rescaleFilterY = RescaleIntensityFiltertype::New();
    RescaleIntensityFiltertype::Pointer rescaleFilterZ = RescaleIntensityFiltertype::New();

    // apply sobel filter in X
    GradientImageType::ConstPointer gradientImageX;
    filterX->SetOperator(sobelOperatorX);
    filterX->SetInput(itkData);
    filterX->Update();
    // rescale the output gradient intensities
    rescaleFilterX->SetInput(filterX->GetOutput());
    rescaleFilterX->SetOutputMinimum(-1.0);
    rescaleFilterX->SetOutputMaximum(1.0);
    rescaleFilterX->Update();
    gradientImageX = rescaleFilterX->GetOutput();

    // Apply sobel filter in Y
    GradientImageType::ConstPointer gradientImageY;
    filterY->SetOperator(sobelOperatorY);
    filterY->SetInput(itkData);
    filterY->Update();
    // rescale the output gradients
    rescaleFilterY->SetInput(filterY->GetOutput());
    rescaleFilterY->SetOutputMinimum(-1.0);
    rescaleFilterY->SetOutputMaximum(1.0);
    rescaleFilterY->Update();
    gradientImageY = rescaleFilterY->GetOutput();

    // Apply sobel filter in Z
    GradientImageType::ConstPointer gradientImageZ;
    filterZ->SetOperator(sobelOperatorZ);
    filterZ->SetInput(itkData);
    filterZ->Update();
    // rescale output gradients
    rescaleFilterZ->SetInput(filterZ->GetOutput());
    rescaleFilterZ->SetOutputMinimum(-1.0);
    rescaleFilterZ->SetOutputMaximum(1.0);
    rescaleFilterZ->Update();
    gradientImageZ = rescaleFilterZ->GetOutput();

    // Add the gradient images into the output QVector after conversion into VTK image
    vtkSmartPointer<vtkStructuredPoints> gradientVTkImageX = vtkSmartPointer<vtkStructuredPoints>::New();
    floatItkToVtk(gradientImageX, gradientVTkImageX);
    gradientVolumes.append(gradientVTkImageX);

    vtkSmartPointer<vtkStructuredPoints> gradientVTkImageY = vtkSmartPointer<vtkStructuredPoints>::New();
    floatItkToVtk(gradientImageY, gradientVTkImageY);
    gradientVolumes.append(gradientVTkImageY);

    vtkSmartPointer<vtkStructuredPoints> gradientVTkImageZ = vtkSmartPointer<vtkStructuredPoints>::New();
    floatItkToVtk(gradientImageZ, gradientVTkImageZ);
    gradientVolumes.append(gradientVTkImageZ);
}

void ITKProcessing::getOccupiedExtent(vtkSmartPointer<vtkStructuredPoints> &voxData, int (&extentObj)[6])
{
    // get the extent of the volume
    int extent[6];
    voxData->GetExtent(extent);
    // initialize the extent of occupied voxels
    extentObj[0] = extent[1];
    extentObj[1] = extent[0];
    extentObj[2] = extent[3];
    extentObj[3] = extent[2];
    extentObj[4] = extent[5];
    extentObj[5] = extent[4];
    // iterate over the volume
    for(int z = 0; z <= extent[5]; z++)
    {
        for(int y = 0; y <= extent[3]; y++)
        {
            for(int x = 0; x <= extent[1]; x++)
            {
                // get the voxel
                unsigned int* val = static_cast<unsigned int*>(voxData->GetScalarPointer(x,y,z));
                // Compare coordinates of occupied voxels and establish boundaries around the vehicle
                if(*val != 0)
                {
                    // boundary in x-axis
                    extentObj[0] = x < extentObj[0] ? x : extentObj[0];
                    extentObj[1] = x > extentObj[1] ? x : extentObj[1];
                    // boundary in y-axis
                    extentObj[2] = y < extentObj[2] ? y : extentObj[2];
                    extentObj[3] = y > extentObj[3] ? y : extentObj[3];
                    // boundary in z-axis
                    extentObj[4] = z < extentObj[4] ? z : extentObj[4];
                    extentObj[5] = z > extentObj[5] ? z : extentObj[5];
                }
            }
        }
    }
}

void ITKProcessing::slicSegmentation(QVector<QVector3D> &boundaryVoxels,
                                     QVector<QVector3D> &boundaryNormals,
                                     vtkSmartPointer<vtkStructuredPoints> &voxData,
                                     int clusters,
                                     QVector<QVector3D> &segmentedVoxelCenters,
                                     QVector<QVector3D> &segmentedVoxelNormals,
                                     QVector<int> &labels,
                                     unsigned int volumeValChoice, QVector<unsigned long> &vectorIndex)
{
    int numCamPos = boundaryVoxels.size();
    int clusterSize = numCamPos/clusters;
    /* Grid interval as defined in the paper -
     * Ideally for volume interval should computed as cube root of 'clusterSize' for a volume
     * But, we use the definition as in the paper considering that we are segmenting a surface
     * (boundary has thickness of only one voxel)
     *
     * Use cube root for background voxels as it is volume not a surface
     */
    int S;
    //if(choice)
        //S = cbrt(numCamPos/clusters);
    //else
        S = sqrt(numCamPos/clusters);

    float residualThreshold = 0.02;//2; // threshold for Residual error E in the paper
    float residualE = 50;         // Initialize the residual error
    float paramM = 0.05;//0.01;            // Parameter m in the paper

    /* value in volume to separately process boundary voxels or background voxels based on 'choice'
     * value in volume is 2 for boundary voxels and 1 for background voxels
     */
//    unsigned int volumeValChoice;
//    if(volumeValChoice)
//        volumeValChoice = 1;
//    else
//        volumeValChoice = 2;

    /* set of cluster centers
     * Holds normals and associated positions [x',y',z',x,y,z]^T
     */
    QVector<QVector<QVector3D>> clusterCenters;

    // volume extents
    int extent[6];
    voxData->GetExtent(extent);

    // Counter for number of centers adjusted
    int adjustedCount = 0;

    // iterate over the set of voxels and initialize cluster centers
    for(int i = 0; i < numCamPos; i++)
    {
        // Break when required no. of clusters are initialized
        if(clusterCenters.size() == clusters)
            break;

        if(i % clusterSize == 0)
        {
            // calculate the magnitude of the current surface normal for neighborhood checking
            float currMag = boundaryNormals[i].length();
            // Vector to hold current cluster center for neighborhood checking
            int centerInd = i;
            // check for boundary conditions
            int x1 = ((boundaryVoxels[i].x() - 1) < extent[0]) ? extent[0] : boundaryVoxels[i].x() - 1;
            int x2 = ((boundaryVoxels[i].x() + 1) > extent[1]) ? extent[1] : boundaryVoxels[i].x() + 1;
            int y1 = ((boundaryVoxels[i].y() - 1) < extent[2]) ? extent[2] : boundaryVoxels[i].y() - 1;
            int y2 = ((boundaryVoxels[i].y() + 1) > extent[3]) ? extent[3] : boundaryVoxels[i].y() + 1;
            int z1 = ((boundaryVoxels[i].z() - 1) < extent[4]) ? extent[4] : boundaryVoxels[i].z() - 1;
            int z2 = ((boundaryVoxels[i].z() + 1) > extent[5]) ? extent[5] : boundaryVoxels[i].z() + 1;

            // Iterate over the 3x3x3 neighbourhood
            for(int x = x1; x <= x2; x++)
            {
                for(int y = y1; y <= y2; y++)
                {
                    for(int z = z1; z <= z2; z++)
                    {
                        // Get the value of the voxel
                        unsigned int* val = static_cast<unsigned int*>(voxData->GetScalarPointer(x,y,z));
                        /* Check if the current voxel index exists in set of boundary voxels
                         * if voxel doesn't exist, skip
                         */
                        if(*val == volumeValChoice)
                        {
                            //int neighbour = boundaryVoxels.indexOf(QVector3D(x,y,z));
                            int neighbour = vectorIndex[x + (y * extent[1]) + (z * extent[1] * extent[3])];
                            // calculate the magnitude of the normal of that neighbour
                            float neighbourMag = boundaryNormals[neighbour].length();
                            // if magnitude of the neighbour is less update the cluster center
                            if(neighbourMag < currMag)
                            {
                                currMag = neighbourMag;
                                centerInd = neighbour;
                            }
                        }

                    }
                }
            }
            // Update the adjusted centers count
            if(centerInd != i)
                adjustedCount++;
            //QVector<float> center = {boundaryNormals[i].x(),boundaryNormals[i].y(),boundaryNormals[i].z(),boundaryVoxels[i].x(),boundaryVoxels[i].y(),boundaryVoxels[i].z()};
            clusterCenters.append({boundaryNormals[centerInd], boundaryVoxels[centerInd]});
        }
    }

    qDebug() << "Number of clusters = " << clusterCenters.size();
    qDebug() << "Number of cluster centers adjusted = " << adjustedCount;
    
    // ------------------------------- SLIC loop --------------------------------
    int numClusters = clusterCenters.size();

    // ----Initialize the vector to hold distances----
    QVector<float> distances;
    distances.resize(numCamPos);
    /* The distances for all voxels are initialized with the maximum possible distances
     * surface normal distances are maximum between (-1,-1,-1) and (1,1,1)
     * while spatial distance is maximum at the extents of the voxel environment
     */
    float maxNormDist = QVector3D(-1,-1,-1).distanceToPoint(QVector3D(1,1,1));
    float maxDist = QVector3D(extent[0],extent[2],extent[4]).distanceToPoint(QVector3D(extent[1],extent[3],extent[5]));
    // distance formula as mentioned in the paper
    float maxTotalDist = sqrt(pow(maxNormDist, 2) + pow(paramM * maxDist / S, 2));
    for(int i = 0; i < numCamPos; i++)
    {
        // initialize all distances with max possible distance
        distances[i] = maxTotalDist;
    }
    // -----Initialize the vector of lables-------
    //QVector<int> labels;
    labels.resize(numCamPos);
    for(int i = 0; i < numCamPos; i++)
    {
        // initialize the labels with -1 to keep track of voxels not associated with any cluster center
        labels[i] = -1;
    }

    // Counter for SLIC iterations
    int itCounter = 0;
    // SLIC iterator loop
    while(residualE >= residualThreshold)
    {
        // Loop over the cluster centers
        for(int i = 0; i < clusterCenters.size(); i++)
        {
            // Search in the neighbourhood of 2Sx2Sx2S of the cluster center
            QVector3D currCenter = clusterCenters[i][1];
            QVector3D currNormal = clusterCenters[i][0];
            /* Create a neighbourhood iterator and Check for image boundary conditions
             * -> ideally, image boundaries cannot be encountered in x,z directions as
             *    boundary voxels are very far from image boundaries
             * -> So, check for image boundaries only in y direction
             */
            // Check for bottom boundary
            int y1 = currCenter.y() - S;
            if(y1 < extent[2])
                y1 = extent[2];
            // Check for top boundary
            int y2 = currCenter.y() + S;
            if(y2 > extent[3])
                y2 = extent[3];

            // check for boundary in x direction
            int x1 = currCenter.x() - S;
            if(x1 < extent[0])
                x1 = extent[0];
            int x2 = currCenter.x() + S;
            if(x2 > extent[1])
                x2 = extent[1];

            // check for boundary in z direction
            int z1 = currCenter.z() - S;
            if(z1 < extent[4])
                z1 = extent[4];
            int z2 = currCenter.z() + S;
            if(z2 > extent[5])
                z2 = extent[5];

            for(int x = x1; x <= x2; x++)
            {
                for(int y = y1; y <= y2; y++)
                {
                    for(int z = z1; z <= z2; z++)
                    {
                        // Get the value at the neighbourhood voxel
                        unsigned int* val = static_cast<unsigned int*>(voxData->GetScalarPointer(x,y,z));
                        // process only the occupied voxels in the neighborhood
                        if(*val == volumeValChoice)
                        {
                            // Get the index of this voxel location from the set of boundary voxels
                            //int currIndex = boundaryVoxels.indexOf(QVector3D(x,y,z));
                            int currIndex = vectorIndex[x + (y * extent[1]) + (z * extent[1] * extent[3])];
                            // Calculate distances according to the paper
                            float normalDistance = boundaryNormals[currIndex].distanceToPoint(currNormal);
                            float spatialDistance = boundaryVoxels[currIndex].distanceToPoint(currCenter);

                            //qDebug()<< "Normal: " << boundaryNormals[currIndex] << " " << "Voxel: " << boundaryVoxels[currIndex];

                            float totalDistance = sqrt(pow(normalDistance, 2) + pow(paramM * spatialDistance / S, 2));
                            // Compare with existing distance and update the label with that of closest cluster center
                            if(totalDistance < distances[currIndex])
                            {
                                // update the distance for later comparison
                                distances[currIndex] = totalDistance;
                                // update the label for the current voxel
                                labels[currIndex] = i;
                            }
                        }
                    }
                }
            }
        }
        /* After all cluster centers are processed for this iteration,
         * generate new centers as the average normal,x,y,z vector of all voxels belonging to that cluster
         */

        // Assign all the outlying voxels to nearest cluster centers
        int outlierCount = 0;
        for(int i = 0; i < numCamPos; i++)
        {
            // Check for voxels without an assigned label
            if(labels[i] == -1)
            {
                // calculate the distance with all the cluster centers
                for(int j = 0; j < numClusters; j++)
                {
                    // calculate distance from the cluster center
                    float normalDistance = boundaryNormals[i].distanceToPoint(clusterCenters[j][0]);
                    float spatialDistance = boundaryVoxels[i].distanceToPoint(clusterCenters[j][1]);
                    float totalDistance = sqrt(pow(normalDistance, 2) + pow(paramM * spatialDistance / S, 2));
                    // Update with the closest distance
                    if(totalDistance < distances[i])
                    {
                        // assign to this cluster
                        labels[i] = j;
                        distances[i] = totalDistance;
                    }
                }
                outlierCount++;
            }
        }
        qDebug() << "Voxels forced assigned: " << outlierCount;

        // Checking for outliers
        int outlierCount1 = 0;
        for(int i = 0; i < numCamPos; i++)
            if(labels[i] == -1)
                outlierCount1++;
        qDebug() << "outlier voxels after reassigning: " << outlierCount1;

        // initialize vector to hold sums of boundary normals and boundary voxels as per the labels
        QVector<QVector3D> normalSum;
        normalSum.resize(numClusters);
        QVector<QVector3D> voxelSum;
        voxelSum.resize(numClusters);
        // Initialize a vector to hold the count of associated voxels with each cluster center
        QVector<int> clusterSizes;
        clusterSizes.resize(numClusters);
        // initialize all the vectors with zeros
        for(int i = 0; i < numClusters; i++)
        {
            normalSum[i] = QVector3D(0,0,0);
            voxelSum[i] = QVector3D(0,0,0);
            clusterSizes[i] = 0;
        }

        // Iterate over the set of labels
        for(int i = 0; i < numCamPos; i++)
        {
            /* get the cluster index from the labels
             * and keep adding the normals and voxel positions corresponding to that cluster
             */
            int tempIndex = labels[i];
            normalSum[tempIndex] += boundaryNormals[i];
            voxelSum[tempIndex] += boundaryVoxels[i];
            clusterSizes[tempIndex] += 1;
        }

        // Delete centers that have no voxels within the cluster
        QVector<int>::iterator iter = clusterSizes.begin();
        while(iter != clusterSizes.end())
        {
            // Check if the cluster size at index is zero
            if(*iter == 0)
            {
                // Get the index of the iteraotr position
                int removeIndex = iter - clusterSizes.begin();
                // delete the entry from clustercenters
                clusterCenters.remove(removeIndex);
                // delete the entry from normal and voxel sum vectors
                normalSum.remove(removeIndex);
                voxelSum.remove(removeIndex);
                // Delete the entry from cluster size vector
                clusterSizes.erase(iter);
                /* Decrement by 1 the labels of the voxels that belong to
                 * centers beyond this point so that they match with the number of clusters
                 */
                for(int  i = 0; i < numCamPos; i++)
                {
                    if(labels[i] > removeIndex)
                        labels[i] = labels[i] - 1;
                }
                // When an entry is removed, the size of vectors reduces by 1
                // And subsequently when iter is incremented, we might miss the next entry
                // So decrease iter by one after deletion so that it stays at same position
                iter--;
            }
            // Increment the iterator
            iter++;
        }
        // Lastly update the number of clusters remaining after removing zeros
        numClusters = clusterCenters.size();

        // make a copy of clusterCenters for calculating residual error
        QVector<QVector3D> clusterCentersOld;
        clusterCentersOld.resize(numClusters);
        for(int i = 0; i < numClusters; i++)
        {
            clusterCentersOld[i] = clusterCenters[i][1];
        }

        // Update the cluster centers with new average values
        for(int i = 0; i < numClusters; i++)
        {
            // Number of voxels in that cluster
            int tempClusterSize = clusterSizes[i];
            // Average the normals at each label
            clusterCenters[i][0] = normalSum[i]/tempClusterSize;
            /* Average the voxel points
             * Round off the average value to the nearest integer
             * because voxel positions cannot take floating point values
             */
            clusterCenters[i][1].setX(round(voxelSum[i].x()/tempClusterSize));
            clusterCenters[i][1].setY(round(voxelSum[i].y()/tempClusterSize));
            clusterCenters[i][1].setZ(round(voxelSum[i].z()/tempClusterSize));
        }

        // Calculate the residual error
        double errorSum = 0;
        for(int i = 0; i < numClusters; i++)
        {
            QVector3D tempL1 = clusterCentersOld[i] - clusterCenters[i][1];
            errorSum += abs(tempL1.x()) + abs(tempL1.y()) + abs(tempL1.z());
        }
        // update the residual error as the average of all the l1 distances
        residualE = errorSum/numClusters;
        itCounter++;
        qDebug() << " Completed " << itCounter << " iterations.";
    }

    // setup output vectors
    for(int i = 0; i < clusterCenters.size(); i++)
    {
        // setup the output container for super voxel centers
        segmentedVoxelCenters.append(clusterCenters[i][1]);
        // setup the output container for normals to super voxel centers
        segmentedVoxelNormals.append(clusterCenters[i][0]);
    }
}

void ITKProcessing::newDslicSeg(QVector<QVector3D> &boundaryVoxels,
                                QVector<QVector3D> &boundaryNormals,
                                int clusters,
                                QVector<QVector3D> &segmentedVoxelCenters,
                                QVector<QVector3D> &segmentedVoxelNormals,
                                QVector<int> &labels)
{
    // Create point cloud with boundaryVoxels and normals
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZINormal>);
    cloudWithNormals->height = 1;
    for(int i = 0; i < boundaryVoxels.size(); i++)
    {
        QVector3D point(boundaryVoxels[i]), normal(boundaryNormals[i]);
        pcl::PointXYZINormal newPoint(point.x(),point.y(),point.z(),1,normal.x(),normal.y(),normal.z());
        cloudWithNormals->push_back(newPoint);
    }

//    pcl::PCDWriter writerSeg;
//    std::string stdFileName = "/media/anirudh/Data/Documents/PhD/Qt_projects/ocp_simulation/resources/Mining_Truck_";
//    writerSeg.write (stdFileName + "_points.pcd", *cloudWithNormals, false);

    int numCamPos = cloudWithNormals->points.size();
    float S = cbrt(numCamPos/clusters);
    int approxClusterSize = numCamPos/clusters;
    // Create a downsampled point cloud with voxel size 2*S*factor
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr centerCloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::VoxelGrid<pcl::PointXYZINormal> sor;
    sor.setInputCloud (cloudWithNormals);
    //sor.setMinimumPointsNumberPerVoxel(approxClusterSize/10);
    float voxSize = S;//2*sqrt(2)*S*factor;
    sor.setLeafSize (voxSize, voxSize, voxSize);
    sor.filter (*centerCloud);

//    pcl::PCDWriter writerSeg1;
//    writerSeg.write (stdFileName + "_points1.pcd", *centerCloud, false);

    // cluster center initialization
    std::vector<int> centerIndex;
    std::vector<pcl::PointXYZINormal> centerPoints;
    //simpleInit(numCamPos, clusters, centerIndex);
    fibonacciBruteForce(centerCloud, clusters, centerIndex, centerPoints,cloudWithNormals);
    // -------------------- DSLIC LOOP------------------------------------------------
    // Initialize residual error and threshold
    float residualThreshold = 0.02; // threshold for Residual error E in the paper
    float residualE = 50;         // Initialize the residual error
    int itCounter = 0;
    int refineItCounter = 0;
    float pointThresh = 1;//0.1f*approxClusterSize;
    // Initialize a distance vector with high numbers
    std::vector<float> distances(numCamPos, 100.0f);
    // variable to keep track of outlier in previous iteration
    int prevOutliers = 0;
    // Create a kdtree to access cluster center neighbours
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>());
    tree->setInputCloud(cloudWithNormals);

    // STart timer
    //QElapsedTimer timer;
    //timer.start();
    // set radius as S/100 to bring it in cm
    double searchRadius = S;
    while(itCounter < 20)
    {
        for(int i = 0; i < centerIndex.size(); i++)
        {
            //int clusterCenterInd = centerIndex[i];
            pcl::PointXYZINormal clusterCenterPoint = centerPoints[i];
            // vectors to store neighbour indices and distances
            std::vector<int> neighbours;
            std::vector<float> neighbourDist;
            // get the neighbours of this cluster center
            if(tree->radiusSearch(clusterCenterPoint, searchRadius, neighbours, neighbourDist) > 0)
            {
                // Loop over all the neighbours
                for(std::size_t j = 0; j < neighbours.size(); j++)
                {
                    // get the index of current neighbour
                    int neighbourIndex = neighbours[j];
                    pcl::PointXYZINormal neighbourPoint = cloudWithNormals->points.at(neighbourIndex);
                    // calculate the distance between these two points
                    float totalD = computeDistance(clusterCenterPoint, neighbourPoint, searchRadius);
                    // if this distance is smaller then update distance and label in corresponding vectors
                    if(totalD < distances[neighbourIndex])
                    {
                        distances[neighbourIndex] = totalD;
                        labels[neighbourIndex] = i;
                    }
                }
            }
        }
        // 1)estimate the size of each cluster center
        // 2)collect the outlying points into a point cloud
        // vector to store indices of points assigned by the clusters
        int numClusters = centerIndex.size();
        std::vector<int> clusterSize(numClusters, 0);
        QVector<QVector<int>> indicesByCluster(centerIndex.size());
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr unassignedPoints(new pcl::PointCloud<pcl::PointXYZINormal>);
        unassignedPoints->height=1;
        for(int i = 0; i < labels.size(); i++)
        {
            int currLab = labels[i];
            // check for voxels without an assigned label
            if(currLab == -1)
                unassignedPoints->push_back((*cloudWithNormals)[i]);
            else
            {
                clusterSize[currLab]+=1;
                // add to the index to the corresponding cluster
                indicesByCluster[currLab].append(i);
            }
        }
        // Update aprroximate cluster size and S
        int outlierCount = unassignedPoints->points.size();
        // If outliers did not enough then refine the clusters and break the loop
        // when approxClusterSize is less than outlier, we have to add 0 clusters, breaking the program in fibonacci initialization
        if((abs(outlierCount-prevOutliers)/float(outlierCount))*100.0 < 10.0 || ((outlierCount/approxClusterSize) < 1))
        {
            break;
        }
        prevOutliers=outlierCount;
        //int aNumber  = (labels.size() - outlierCount)/numClusters;
        // 2)collect indices of cluster centers with less than 10% of required cluster size (S)
        std::vector<int> remIndices;
        for(int i = 0; i < clusterSize.size(); i++)
        {
            if(clusterSize[i] < pointThresh)
                remIndices.push_back(i);
        }
        int toRemove = remIndices.size();
        int toAdd = outlierCount/approxClusterSize;
        // downsample the point cloud and find new clusters by fibonacci binning
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr centersAdded(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::VoxelGrid<pcl::PointXYZINormal> sorUnassign;
        sorUnassign.setInputCloud (unassignedPoints);
        //sorUnassign.setMinimumPointsNumberPerVoxel(approxClusterSize/10);
        sorUnassign.setLeafSize (voxSize, voxSize, voxSize);
        sorUnassign.filter (*centersAdded);
        //ensure that program does not crash when points in cloud is less than toAdd
        toAdd = centersAdded->size() < toAdd ? centersAdded->size() : toAdd;
        // Initialization of new cluster centers
        std::vector<int> centerIndexAdded;
        std::vector<pcl::PointXYZINormal> centerPointsAdded;
        fibonacciBruteForce(centersAdded, toAdd, centerIndexAdded, centerPointsAdded, cloudWithNormals);
        // ------Replacing the ones to be removed is causing unnecessary complexities.
        //-------So, first remove all an then add the rest
        int removeTrans = 0;
        for(int i = 0; i < remIndices.size(); i++)
        {
            int removeIndex = remIndices[i] - removeTrans;
            int removePoint = centerIndex[removeIndex];
            // Mark the cluster center to be removed with -2 to ensure it doesnt get initialized again
            //labels[removePoint] = -2;
            //distances[removePoint] = 100.0f;
            clusterSize.erase(clusterSize.begin()+removeIndex);
            centerIndex.erase(centerIndex.begin()+removeIndex);
            centerPoints.erase(centerPoints.begin()+removeIndex);
            for(int j = 0; j < indicesByCluster[removeIndex].size(); j++)
            {
                int currVoxIndex = indicesByCluster[removeIndex][j];
                labels[currVoxIndex] = -1;
                distances[currVoxIndex] = 100.0f;
            }
            // decrement the rest of the clusters' labels by one
            for(int j = removeIndex+1; j < indicesByCluster.size(); j++)
            {
                for(int k = 0; k < indicesByCluster[j].size(); k++)
                {
                    int currVoxIndex = indicesByCluster[j][k];
                    labels[currVoxIndex] -= 1;
                }
            }
            indicesByCluster.erase(indicesByCluster.begin()+removeIndex);
            removeTrans++;
        }
        // Restimate the centers as mean of all points belonging to the cluster
        for(int i = 0; i < centerPoints.size(); i++)
        {
            Eigen::Vector3f sumS(0,0,0), sumN(0,0,0);
            //Eigen::Vector3i sumC(0,0,0);
            for(int j = 0; j < indicesByCluster[i].size(); j++)
            {
                pcl::PointXYZINormal currVoxPoint = cloudWithNormals->points.at(indicesByCluster[i][j]);
                sumS = sumS + Eigen::Vector3f(currVoxPoint.getArray3fMap());
                //sumC = sumC + Eigen::Vector3i(currVoxPoint.getRGBVector3i());
                sumN = sumN + Eigen::Vector3f(currVoxPoint.getNormalVector3fMap());
            }
            int currClustSize = clusterSize[i];
            // round it to nearest integer
            centerPoints[i].getArray3fMap() = sumS/currClustSize;
            //centerPoints[i].getRGBVector3i() = sumC/currClustSize;
            centerPoints[i].getNormalVector3fMap() = sumN/currClustSize;
        }
        // Now add the rest
        for(int i = 0; i < centerIndexAdded.size(); i++)
        {
            int indexToAdd = centerIndexAdded[i];
            centerIndex.push_back(indexToAdd);
            centerPoints.push_back(centerPointsAdded[i]);
            indicesByCluster.append(QVector<int>({indexToAdd}));
            labels[indexToAdd] = i;
        }
        itCounter++;
        //std::cerr << "Dynamic initialization iterations " << itCounter << std::endl;
    }
    // Force assign unalbelled points to clusters
    for(int i = 0; i < labels.size(); i++)
    {
        if(labels[i]==-1)
        {
            QVector3D currOutlier = boundaryVoxels[i];
            double currD = 100.0;
            for(int j = 0; j < centerPoints.size(); j++)
            {
                pcl::PointXYZINormal currCenter = centerPoints[j];
                double newD = sqrt(pow(currOutlier.x()-currCenter.x,2) + pow(currOutlier.y() - currCenter.y,2) + pow(currOutlier.z()-currCenter.z,2));
                if(newD < currD)
                {
                    currD = newD;
                    labels[i] = j;
                }
            }
        }
    }
    // Restimate the centers as mean of all points belonging to the cluster
    std::for_each(labels.begin(), labels.end(), [](int &n){ n+=1; });

    std::vector<Eigen::Vector3f> gSumS(centerPoints.size()+1,{0,0,0}),gSumN(centerPoints.size()+1,{0,0,0});
    std::vector<int> clusterSize(centerPoints.size()+1,0);
    centerPoints.push_back(pcl::PointXYZINormal(0));
    for(int i = 0; i < labels.size(); i++)
    {
        int pos = labels[i];
        pcl::PointXYZINormal currVoxPoint = cloudWithNormals->points.at(i);
        //Eigen::Vector3i sumC(0,0,0);
        gSumS[pos] = gSumS[pos] + Eigen::Vector3f(currVoxPoint.getArray3fMap());
        gSumN[pos] = gSumN[pos] + Eigen::Vector3f(currVoxPoint.getNormalVector3fMap());
        clusterSize[pos] += 1;
    }
    for(int i = 0; i < centerPoints.size(); i++)
    {
        centerPoints[i].getArray3fMap() = gSumS[i]/clusterSize[i];
        centerPoints[i].getNormalVector3fMap() = gSumN[i]/clusterSize[i];
    }

    //Setup output vectors
    segmentedVoxelCenters.resize(centerPoints.size());
    segmentedVoxelNormals.resize(centerPoints.size());
    for(int  i = 0 ; i < centerPoints.size(); i++)
    {
        pcl::PointXYZINormal currPoint = centerPoints[i];
        segmentedVoxelCenters[i] = QVector3D((currPoint.x), (currPoint.y), (currPoint.z));
        segmentedVoxelNormals[i] = QVector3D(currPoint.normal_x,currPoint.normal_y,currPoint.normal_z);
    }
    //double elapseTime = timer.elapsed();
    // Increment all labels by one

    std::cerr << "Total clusters = " << centerPoints.size() << " init iterations = " << itCounter << " update iterations = " << refineItCounter << std::endl;
}

void ITKProcessing::fibonacciBruteForce(pcl::PointCloud<pcl::PointXYZINormal>::Ptr downCloud,
                                     int numClust,
                                     std::vector<int> &clustCent,
                                     std::vector<pcl::PointXYZINormal> &clustCentPoints,
                                        pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloudWithNormals)
{
    // this method works only with odd number of bins.
    // if numCLust is even, add 1 to make it odd
    int bins;
    if(numClust % 2 == 0)
        bins = numClust*2 + 1;
    else
        bins = numClust*2 - 1;

    // Collect the point normals for fibonacci brute force cluster center initialization
    int numDownSample = downCloud->points.size();
    QVector<QVector3D> voxNormals(numDownSample);
    for(std::size_t point = 0; point < numDownSample; point++)
    {
        voxNormals[point].setX(downCloud->points.at(point).normal_x);
        voxNormals[point].setY(downCloud->points.at(point).normal_y);
        voxNormals[point].setZ(downCloud->points.at(point).normal_z);
    }

    // approximate size of each cluster
    //int clustSize = voxNormals.size()/numClust;
    // polar (theta) and azimuthal (phi) angles for the unit normal vectors
    QVector<double> normTheta(voxNormals.size()), normPhi(voxNormals.size());
    // Theta is calculated from +y axis. range - [0,pi]
    // Phi is calculated from +x axis. range - [0,2pi]
    QVector3D plusY = {0.0f,1.0f,0.0f}, plusX = {1.0f,0.0f,0.0f};
    // Vector to save indices of nomrals accroding to it's bins
    QVector<QVector<int>> normBins(bins);
    // Constant tau from paper
    double tow = (1+sqrt(5))/2;
    // Vector to store the values
    QVector<int> dTrue;
    // Initialize d values from (1-n)/2 to (n-1)/2
    for(int i = (1-bins)/2; i <= (bins - 1)/2; i++)
        dTrue.append(i);
    // Calculate bin centers in spherical coordinates
    QVector<double> binTheta(bins), binPhi(bins);
    for(int i = 0; i < dTrue.size(); i++)
    {
        binTheta[i] = asin(2.0*dTrue[i]/bins) + M_PI/2;
        double tempBinPhi = ((2*M_PI)/tow)*std::fmod(dTrue[i],tow);
        // Conditions to keep phi in range [0,2*pi]
        // taken form paper "Measurement of areas on a sphere using Fibonacci and latitude–longitude lattices"
        // The following loops bring the angle to the range [-pi,pi]
        double tempBinPhi2;
        if(tempBinPhi < -M_PI)
            tempBinPhi2 = 2*M_PI + tempBinPhi;
        else if(tempBinPhi > M_PI)
            tempBinPhi2 = tempBinPhi - 2*M_PI;
        else
            tempBinPhi2 = tempBinPhi;
        // now add pi to all angles to bring to the range [0,2*pi]
        binPhi[i] = tempBinPhi2 + M_PI;
    }

    // loop over normals
    for(int i = 0; i < voxNormals.size(); i++)
    {
        // ------------- Spherical coordinates calculation----------------
        // get the current normal vector because it has to be used repeatedly
        QVector3D currNormal = voxNormals[i];
        //-------calculate theta--------
        double theta = acos(QVector3D::dotProduct(plusY, currNormal));
        normTheta[i] = theta;
        //----- Calculations for phi-------
        // get vsin(theta). V is the boundary normal vector
        QVector3D vSinTheta = currNormal * sin(theta);
        // vsin(theta) cross +x
        QVector3D currCrossProd = QVector3D::crossProduct(plusX, vSinTheta);
        // Value of cross product
        double crossProdVal = currCrossProd.x() + currCrossProd.y() + currCrossProd.z();
        // if value is >= 0, then angle is ang = cos inverse of vsin(theta) dot plusX
        // else if value is < 0, then angle is 2*pi - ang
        double phi;
        if(crossProdVal >= 0.0)
            phi = acos(QVector3D::dotProduct(vSinTheta, plusX));
        else
            phi = 2*M_PI - acos(QVector3D::dotProduct(vSinTheta, plusX));
        // Add the phi value in the vector
        normPhi[i] = phi;
        // Compare with each bin center and select the closest one
        int binIndSelected = 0;
        double minDistance = sqrt(pow(binTheta[0]-theta,2)+pow(binPhi[0]-phi,2));
        for(int j = 1; j < binTheta.size(); j++)
        {
            double nextDistance = sqrt(pow(binTheta[j]-theta,2)+pow(binPhi[j]-phi,2));
            if(nextDistance < minDistance)
            {
                minDistance = nextDistance;
                binIndSelected = j;
            }
        }
        // add this normal at the obtained bin index
        normBins[binIndSelected].append(i);
    }
    // delete bins with less normals than 5% of clustSize
    // store the deleted indices for later use (to initialize remaining cluster centers)
    // Vector to store remaining points from each after adding one cluster center
    QVector<int> binPoints;
    QVector<QVector<int>>::iterator iter = normBins.begin();
    while(iter != normBins.end())
    {
        if(iter->size() < 1/*0.05*clustSize*/)
        {
            binPoints.append(*iter);
            //int removeInd = std::distance(normBins.begin(), iter);
            normBins.erase(iter);
        }
        else
            iter++;
    }

    // Initialize a cluster center at 0 location of each bin entry
    int remainingClusters = numClust;
    QVector<int> remBinEntries;
    std::vector<int> tempClustCent;
    // rough estimate of interval
    int initInterval = ceil(double(voxNormals.size())/numClust);//(voxNormals.size()-normBins.size())/(numClust-normBins.size());
    for(int i = 0; i < normBins.size(); i++)
    {
        tempClustCent.push_back(normBins[i][0]);
        for(int j = initInterval; j < normBins[i].size(); j++)
        {
            remBinEntries.append(normBins[i][j]);
        }
        remainingClusters -= 1;
    }
    // Initialize the remaining clusters at equal interval in the remaining bin entries
    for(int i = 1; i < remBinEntries.size(); i++)
    {
        if(i%initInterval==0)
        {
            tempClustCent.push_back(remBinEntries[i]);
            remainingClusters -= 1;
        }
        if(remainingClusters == 0)
            break;
    }

    // Find the closest points in the original point cloud and initiliaze the cluster centers
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr centTree(new pcl::search::KdTree<pcl::PointXYZINormal>());
    centTree->setInputCloud(cloudWithNormals);
    for(int i = 0; i < tempClustCent.size(); i++)
    {
        std::vector<int> nearInd;
        std::vector<float> nearDist;
        centTree->nearestKSearch(downCloud->points.at(tempClustCent[i]), 1, nearInd, nearDist);
        // Setup the output containers
        clustCent.push_back(nearInd[0]);
        clustCentPoints.push_back(cloudWithNormals->points.at(nearInd[0]));
    }
}

float ITKProcessing::computeDistance(pcl::PointXYZINormal centPoint, pcl::PointXYZINormal neighPoint, double searchRad)
{
    Eigen::Vector3f centerS = centPoint.getArray3fMap();
    //Eigen::Vector3i centerC = centPoint.getRGBVector3i();
    Eigen::Vector3f centerN = centPoint.getNormalVector3fMap();
    Eigen::Vector3f neighS = neighPoint.getArray3fMap();
    //Eigen::Vector3i neighC = neighPoint.getRGBVector3i();
    Eigen::Vector3f neighN = neighPoint.getNormalVector3fMap();
    // ----------Spatial distance--------------
    double spatialD = (sqrt(pow(neighS.x()-centerS.x(),2) + pow(neighS.y()-centerS.y(),2) + pow(neighS.z()-centerS.z(),2)))/searchRad;
    // ----------RGB distance------------------
    //float meanr = (centerC.x() + neighC.x())/2.0;
    //double colorD = (sqrt((2 + (meanr/256))*pow(centerC.x()-neighC.x(),2) + 4*pow(centerC.y()-neighC.y(),2) + (2 + ((255-meanr)/256))*pow(centerC.z()-neighC.z(),2)))/256.0;
    // ----------Normal distance---------------
    double normalD = sqrt(pow(centerN.x()-neighN.x(),2) + pow(centerN.y()-neighN.y(),2) + pow(centerN.z()-neighN.z(),2));
    // ----------Total weighted distance-------
    float totalD = sqrt(paramS*pow(spatialD,2) + pow(normalD,2));
    return totalD;
}

void ITKProcessing::faceClustering(QVector<QVector3D> &boundaryVoxels,
                                   QVector<QVector3D> &boundaryNormals,
                                   int clusters,
                                   QVector<unsigned long> &vectorIndex,
                                   vtkSmartPointer<vtkStructuredPoints> &voxData,
                                   QVector<QVector3D> &segmentedVoxelCenters,
                                   QVector<QVector3D> &segmentedVoxelNormals,
                                   QVector<int> &labels)
{
    /* This is the implementation of the surface clustering algorithm as in
     * Garland, Michael, Andrew Willmott, and Paul S. Heckbert. "Hierarchical face
     * clustering on polygonal surfaces." In Proceedings of the 2001 symposium on
     * Interactive 3D graphics, pp. 49-58. 2001.
     */

    // Initially all voxels are clusters
    int currClusters = boundaryVoxels.size();

    /* Temporary vector to save updated normals of clusters at each iteration.
     * Temporary vector to save updated voxel positions of clusters
     * This is to avoid making changes to the original normals or voxels container vectors
     */
     QVector<QVector3D> tempClusterNormals = boundaryNormals, tempClusterVoxels = boundaryVoxels;
    // vector to keep track of clustered voxels along with their temporary labels
    QVector<QVector<int>> tempLables;
    tempLables.resize(currClusters);
    for(int i = 0; i < currClusters; i++)
        tempLables[i].append(i);

    // Debug
    //qDebug() << "Before clearing : " << tempLables[9] << " " << tempLables[10] << " " << tempLables[11] << " value : " << tempLables[10].empty();
    //tempLables[10].clear();
    //qDebug() << "After clearing : " << tempLables[9] << " " << tempLables[10] << " " << tempLables[11] << " value : " << tempLables[10].empty();

    // boost library graph for voxel surface
    UndirectedGraph g;

    // volume extents
    int extent[6];
    voxData->GetExtent(extent);

    // ---------------------- Construct dual-edge graph-----------------------------
    // Loop over the iso-surface
    for(int i = 0; i < currClusters; i++)
    {
        // Create input vectors vector in Eigen
        Eigen::Vector3f normalN1(boundaryNormals[i].x(), boundaryNormals[i].y(), boundaryNormals[i].z());
        Eigen::Vector3f voxelV1(boundaryVoxels[i].x(), boundaryVoxels[i].y(), boundaryVoxels[i].z());
        // calculate 'd' in fit quadratic equation
        float dVal1 = (normalN1.transpose() * voxelV1);
        dVal1 = -dVal1;
        float tempP1 = 2*dVal1*normalN1.transpose()*voxelV1 + pow(dVal1, 2);
        float quadricP1 = normalN1.transpose()*(voxelV1*voxelV1.transpose())*normalN1 + tempP1;

        /* check the 3x3x3 neighborhood around the voxel for connected edges
         */
        // check for boundary conditions
        int x1 = ((boundaryVoxels[i].x() - 1) < extent[0]) ? extent[0] : boundaryVoxels[i].x() - 1;
        int x2 = ((boundaryVoxels[i].x() + 1) > extent[1]) ? extent[1] : boundaryVoxels[i].x() + 1;
        int y1 = ((boundaryVoxels[i].y() - 1) < extent[2]) ? extent[2] : boundaryVoxels[i].y() - 1;
        int y2 = ((boundaryVoxels[i].y() + 1) > extent[3]) ? extent[3] : boundaryVoxels[i].y() + 1;
        int z1 = ((boundaryVoxels[i].z() - 1) < extent[4]) ? extent[4] : boundaryVoxels[i].z() - 1;
        int z2 = ((boundaryVoxels[i].z() + 1) > extent[5]) ? extent[5] : boundaryVoxels[i].z() + 1;

        // loop over neighbourhood
        for(int x = x1; x <= x2; x++)
        {
            for(int y = y1; y <= y2; y++)
            {
                for(int z = z1; z <= z2; z++)
                {
                    // Get the index of this voxel in the original 'boundaryVoxels' vector
                    int neighborIndex = vectorIndex[x + (y * extent[1]) + (z * extent[1] * extent[3])];
                    // skip the central voxel in the neighbourhood
                    if(neighborIndex == i)
                        continue;
                    // Get the value at the neighbourhood voxel
                    unsigned int* val = static_cast<unsigned int*>(voxData->GetScalarPointer(x,y,z));
                    // process only the occupied voxels in the neighborhood
                    if(*val == 2)
                    {
                        /* if this voxel in neighbourhood lies on the surface,
                         * it implies that it is adjacent to current voxel.
                         * Add this connection as edge in the graph
                         */
                        // calculate d and fit quadratic P for this adjacent voxel
                        Eigen::Vector3f normalN2(boundaryNormals[neighborIndex].x(), boundaryNormals[neighborIndex].y(), boundaryNormals[neighborIndex].z());
                        Eigen::Vector3f voxelV2(boundaryVoxels[neighborIndex].x(), boundaryVoxels[neighborIndex].y(), boundaryVoxels[neighborIndex].z());
                        float dVal2 = (normalN2.transpose() * voxelV2);
                        dVal2 = -dVal2;
                        float tempP2 = 2*dVal2*normalN2.transpose()*voxelV2 + pow(dVal2, 2);
                        float quadricP2 = normalN2.transpose()*(voxelV2*voxelV2.transpose())*normalN2 + tempP2;

                        /* Add edge in the graph
                         * Note that cost of contracting two edges is P1 + P2
                         */
                        boost::add_edge(i,neighborIndex,quadricP1+quadricP2,g);
                    }
                }
            }
        }
    }
    qDebug() << "Initital Graph created";

    // stop iterations when the number of required clusters are formed
    while(currClusters > clusters)
    {
        // Initialize minimum edge weight
        double minEdge = 1000;
        // containers to store indices corresponding to min edge
        boost::graph_traits<UndirectedGraph>::vertex_descriptor startNode, endNode;
        /* Iterate over existing edges and save the edge with smallest weight
         */
        boost::property_map<UndirectedGraph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);
        std::pair<edge_iterator, edge_iterator> edgePair;
        for (edgePair = edges(g); edgePair.first != edgePair.second; ++edgePair.first)
        {
            //std::cout << *edgePair.first << " " << EdgeWeightMap[*edgePair.first] << std::endl;
            //std::cout << boost::source(*edgePair.first,g) << " " << target(*edgePair.first,g) << std::endl;
            // save the edge value and corresponding node indices if current edge weight is smaller
            if(EdgeWeightMap[*edgePair.first] < minEdge)
            {
                minEdge = EdgeWeightMap[*edgePair.first];
                startNode = boost::source(*edgePair.first, g);
                endNode = boost::target(*edgePair.first, g);
            }
        }

        /* Next step is to remove the edge with least value of fit quadric
         * The nodes with the dge have to contracted into one
         * And their other edges have to be reorganized in the graph for next iteration
         */
        // Get all the dges associated to source and target vertices of edge with minimum weight
        boost::graph_traits<UndirectedGraph>::out_edge_iterator out_i, out_end;
        boost::graph_traits<UndirectedGraph>::edge_descriptor e;
        /* Vector to store neighbouring nodes of this start node and end node.
         * Edges need be added to these nodes from the new node after contracting smallest edge.
         * Note that same voxels can be neighbouring to both the voxels that are merged,
         * That can be ignored because the graph should be capable of removing duplicate/parallel edges.
         */
        QVector<boost::graph_traits<UndirectedGraph>::vertex_descriptor> newAdjacentVoxels;
        QVector<double> newAdjacentWeights;

        // Get edges of source Node
//        qDebug() << "---------------------------------";
        for (boost::tie(out_i, out_end) = out_edges(startNode, g); out_i != out_end; ++out_i)
        {
            e = *out_i;
            // Be careful to discard the edge between startnode and endnode if it exists
//            qDebug() <<" Source Node : " << startNode << " Edge : " << "(" << boost::source(e, g) << "," << boost::target(e, g) << ") ";
            if(boost::target(e,g) != endNode)
            {
                newAdjacentVoxels.append(boost::target(e, g));
                newAdjacentWeights.append(EdgeWeightMap[*out_i]);
            }
            //std::cout << "(" << boost::source(e, g) << "," << boost::target(e, g) << ") ";
        }
//        qDebug() << "\n";
        // Repeat previous step for other voxel that is being merged
        for (boost::tie(out_i, out_end) = out_edges(endNode, g); out_i != out_end; ++out_i)
        {
            e = *out_i;
            //qDebug() << " fit quadric : " << EdgeWeightMap[*out_i];
            // Be careful to discard the edge between startnode and endnode if it exists
//            qDebug() <<" Source Node : " << endNode << " Edge : " << "(" << boost::source(e, g) << "," << boost::target(e, g) << ") ";
            if(boost::target(e,g) != startNode)
            {
                newAdjacentVoxels.append(boost::target(e, g));
                newAdjacentWeights.append(EdgeWeightMap[*out_i]);
            }
            //std::cout << "(" << boost::source(e, g) << "," << boost::target(e, g) << ") ";
        }
//        qDebug() << "\n";
//        qDebug() << "Added end nodes : " << newAdjacentVoxels;

        /* Merging two voxels in the graph is not possible.
         * So, we keep track of the clusters by adding the voxels to the entry with smaller index,
         * while, carefully keeping track of the index.
         */
        unsigned int indexToKeep, indexToDiscard;
        if(startNode <= endNode)
        {
            indexToKeep = startNode;
            indexToDiscard = endNode;
        }
        else
        {
            indexToKeep = endNode;
            indexToDiscard = startNode;
        }

        // Change the type of index from index to vertex descriptor
        boost::graph_traits<UndirectedGraph>::vertex_descriptor vertexToKeep = indexToKeep;

        /* If the labels at index to keep are empty, throw error,
         * as this will be unexpected behaviour
         */
        if(tempLables[indexToKeep].empty())
            qDebug() << "!!!!!Fatal error. Labels vector at index " << indexToKeep << " empty";

        // Add the indices existing at larger index to vector entry at smaller index
        tempLables[indexToKeep].append(tempLables[indexToDiscard]);
        // Clear at the discarded index
        tempLables[indexToDiscard].clear();
        // Update the normal at index as the average of the two normals of merging clusters
        tempClusterNormals[indexToKeep] = (tempClusterNormals[indexToKeep] + tempClusterNormals[indexToDiscard])/2;
        // Update voxel position as the average of two voxel positions of merging clusters
        tempClusterVoxels[indexToKeep] = (tempClusterVoxels[indexToKeep] + tempClusterVoxels[indexToDiscard])/2;
        /* Clear the vertices forming this edge from the graph.
         * This also removes all edges from the node that is being removed
         */
        boost::clear_vertex(startNode, g);              // removes all edges
        boost::clear_vertex(endNode, g);
        //boost::remove_vertex(startNode, g);             // removes the node
        //boost::remove_vertex(endNode, g);

        /* Add the new node representing a cluster of the two voxels.
         * It will be represented by 'indexToKeep' to help keep a track
         * of the voxels in belonging to this cluster.
         */
        // Calculate fit quadric of the new cluster
        Eigen::Vector3f normalN1(tempClusterNormals[indexToKeep].x(), tempClusterNormals[indexToKeep].y(), tempClusterNormals[indexToKeep].z());
        Eigen::Vector3f voxelV1(tempClusterVoxels[indexToKeep].x(), tempClusterVoxels[indexToKeep].y(), tempClusterVoxels[indexToKeep].z());
        // calculate 'd' in fit quadratic equation
        float dVal1 = (normalN1.transpose() * voxelV1);
        dVal1 = -dVal1;
        float tempP1 = 2*dVal1*normalN1.transpose()*voxelV1 + pow(dVal1, 2);
        float quadricP1 = normalN1.transpose()*(voxelV1*voxelV1.transpose())*normalN1 + tempP1;

        // Add edges with new cluster
        for(int i = 0; i < newAdjacentVoxels.size(); i++)
            boost::add_edge(vertexToKeep, newAdjacentVoxels[i], quadricP1 + newAdjacentWeights[i],g);

//        qDebug() << "New added edges : ";
//        for (boost::tie(out_i, out_end) = out_edges(vertexToKeep, g); out_i != out_end; ++out_i)
//        {
//            e = *out_i;
//            //qDebug() << " fit quadric : " << EdgeWeightMap[*out_i];
//            // Be careful to discard the edge between startnode and endnode if it exists
//            qDebug() <<" Source Node : " << vertexToKeep << " Edge : " << "(" << boost::source(e, g) << "," << boost::target(e, g) << ") ";
//            //std::cout << "(" << boost::source(e, g) << "," << boost::target(e, g) << ") ";
//        }
//        qDebug() << "\n";
        // reduce 'currClusters' by 1 because two clusters are merged into one
        currClusters--;

        if(currClusters % 10000 == 0)
            qDebug() << " No. of clusters : " << currClusters;

//        boost::graph_traits<UndirectedGraph>::adjacency_iterator ai;
//        boost::graph_traits<UndirectedGraph>::adjacency_iterator ai_end;
//        for (boost::tie(ai, ai_end) = adjacent_vertices(0, g); ai != ai_end; ++ai)
//                std::cout << *ai <<  " ";
//        std::cout << endl;
    }

    // Throw fatal error if currClusters != clusters

    // Initialize output vectors
    labels.resize(boundaryVoxels.size());
    segmentedVoxelCenters.resize(clusters);
    segmentedVoxelNormals.resize(clusters);

    /* Assign labels to each voxel according to their cluster
     * While iterating over the temp labels vector, we also collect
     * corresponding positions and normals from previously stored vectors
     */
    unsigned int finalLabel = 0;
    for(int i = 0; i < tempLables.size(); i++)
    {
        // The tempLabels vector is empty at indices with no clusters
        if(!tempLables[i].empty())
        {
            for(int j = 0; j < tempLables[i].size(); j++)
            {
                labels[tempLables[i][j]] = finalLabel;
            }
            segmentedVoxelCenters[finalLabel] = tempClusterVoxels[i];
            segmentedVoxelNormals[finalLabel] = tempClusterNormals[i];
            finalLabel++;
        }
    }
    qDebug() << "Clustering complete!";
}
