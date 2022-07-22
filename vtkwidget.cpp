/* Handles the simulation environment and all voxel processing and representation
 * voxWorld represents the complete environment and the values represent the following:
 * 0 -> Background points lying outisde the circumference of 12m
 * 1 -> Actually considered background voxels for optimization
 * 2,3,4,5,6 -> covered background points after optimization (for 5 cameras), also for respective camera location
 * 7 -> optimized cluster centers (initial seeds) (final camera position result)
 * 8 -> Represents vehicle (occupied voxels for object)
 * 9 -> Represents vehicle boundaries / initial set of camera locations for optimization
 */

#include "vtkwidget.h"

// To avoid Error: no override found for 'vtkPolyDataMapper'
#include "vtkAutoInit.h"
VTK_MODULE_INIT(vtkRenderingOpenGL2); // VTK built with vtkRenderingOpenGL2
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);

VTKWidget::VTKWidget(QWidget *parent) : QVTKOpenGLNativeWidget(parent)
{
    // Setup objects for initial scene
    vtkNew<vtkNamedColors> colors;
    vtkNew<vtkSphereSource> sphereSource;

    vtkNew<vtkPolyDataMapper> sphereMapper;
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkNew<vtkActor> sphereActor;
    sphereActor->SetMapper(sphereMapper);
    sphereActor->GetProperty()->SetColor(colors->GetColor4d("Tomato").GetData());

    // Call visualize function to Display initial scene
    visualizeVTK(sphereActor);

    // initialize the file
    // Bulldozer model with dimensions 128 voxels
    //fileNameVTK = "/media/anirudh/Data/Documents/PhD/Qt_projects/ocp_simulation/resources/bulldoser.vtk";
    // Bulldozer model with different dimensions
    //fileNameVTK = "/media/anirudh/Data/Documents/PhD/Qt_projects/ocp_simulation/resources/bulldoser_" + std::to_string(inSize) + ".vtk";
    // JCB model
    //fileNameVTK = "/media/anirudh/Data/Documents/PhD/Qt_projects/ocp_simulation/resources/JCP_" + std::to_string(inSize) + ".vtk";
    // Mining Truck
    fileNameVTK = "/media/anirudh/Data/Documents/PhD/Qt_projects/ocp_simulation/resources/Mining_Truck_" + std::to_string(inSize) + ".vtk";
    // Tractor Scraper
    //fileNameVTK = "/media/anirudh/Data/Documents/PhD/Qt_projects/ocp_simulation/resources/Scraper_" + std::to_string(inSize) + ".vtk";
}

QSize VTKWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize VTKWidget::sizeHint() const
{
    return QSize(400, 400);
}

void VTKWidget::getEnvParams(int &extentVal, int &worldCent)
{
    // Default case = previously used parameters
    if(inSize == 0)
    {
        worldCent = 200;
        extentVal = 399;
    }
    else if(inSize == 16)
    {
        worldCent = 30;
        extentVal = 59;
    }
    else if(inSize == 32)
    {
        worldCent = 60;
        extentVal = 119;
    }
    else if(inSize == 64)
    {
        worldCent = 120;
        extentVal = 239;
    }
    else if(inSize == 128)
    {
        worldCent = 240;
        extentVal = 479;
    }
    else if(inSize == 256)
    {
        worldCent = 480;
        extentVal = 959;
    }
}

void VTKWidget::getBowlRad(float &a1, float &a2)
{
    // Default case = previously used parameters
    if(inSize == 0)
    {
        a1 = 190.0f;
        a2 = 140.0f;
    }
    else if(inSize == 16)
    {
        a1 = 24;
        a2 = 18;
    }
    else if(inSize == 32)
    {
        a1 = 48;
        a2 = 36;
    }
    else if(inSize == 64)
    {
        a1 = 96;
        a2 = 72;
    }
    else if(inSize == 128)
    {
        a1 = 192;
        a2 = 144;
    }
    else if(inSize == 256)
    {
        a1 = 384;
        a2 = 288;
    }
}

void VTKWidget::loadModelPressed()
{
    // Split the string check file extension
    std::vector<std::string> splitFileName;
    boost::split(splitFileName, fileNameVTK, [](char c){return c == '.';});

    // If extension is obj we need to voxelize it using binvox
    if(splitFileName[1].compare("obj") == 0)
    {
        // Setup Qprocess environment to voxelize using binvox
        //QString command = "xvfb :99 -screen 0 640x480x24 & setenv DISPLAY=:99 & xvfb-run -s \"-screen 0 640x480x24\" /home/anirudh/Downloads/binVox/binvox -d 128 -t vtk -pb " + QString::fromUtf8(fileNameOBJ.c_str());
        QString command = "/home/pv/opt/binVox/binvox -d 128 -t vtk " + QString::fromUtf8(fileNameVTK.c_str());
        //QString command = "/home/anirudh/opt/foxitsoftware/foxitreader";
        QProcess *binVoxProcess = new QProcess();
        qDebug().noquote()<<"Voxelizing using binvox with the command: " << command;
        binVoxProcess->setProcessChannelMode(QProcess::ForwardedChannels);

        // extract and display qprocess errors
        connect(binVoxProcess, &QProcess::errorOccurred, this, &VTKWidget::processError);

        binVoxProcess->start(command);
        // update filename with newly created vtk file
        fileNameVTK = splitFileName[0] + ".vtk";
    }

    //vtkNew<vtkOBJReader> reader;
    //auto reader = vtkSmartPointer<vtkOBJImporter>::New();

    // read data from vtk file as a structured grid
    vtkNew<vtkGenericDataObjectReader> reader;
    reader->SetFileName(fileNameVTK.c_str());
    reader->Update();

    // Check for type of data in the .vtk file
    if(reader->IsFileStructuredPoints())
    {

        // Read underlying as a 'structuredPoints' dataset
        vtkSmartPointer<vtkStructuredPoints> voxData = vtkSmartPointer<vtkStructuredPoints>::New();
        voxData = reader->GetStructuredPointsOutput();

        // get extent of volume
        int extent[6];
        voxData->GetExtent(extent);
        qDebug() << "Extent Old: " << "(" << extent[0] << "," << extent[1] << ")" << "(" << extent[2] << "," << extent[3] << ")" << "(" << extent[4] << "," << extent[5] << ")";

        // get data dimensions along each axis
        int dimsOld[3];
        voxData->GetDimensions(dimsOld);
        qDebug() << "Dimensions: " << "(" << dimsOld[0] << "," << dimsOld[1] << "," << dimsOld[2] << ")";

        // Containers to hold extent of object boundaries (in order (x1,x2,y1,y2,z1,z2)) 1 -> min. 2-> max.
        int extentObj[6];
        // initialize obejct extent
        extentObj[0] = extent[1];
        extentObj[1] = extent[0];
        extentObj[2] = extent[3];
        extentObj[3] = extent[2];
        extentObj[4] = extent[5];
        extentObj[5] = extent[4];
        // Iterate over the data and get boundaries of occupied voxels within the 128x128x128 grid
        for(int z = 0; z < dimsOld[2]-1; z++)
        {
            for(int y = 0; y < dimsOld[1]-1; y++)
            {
                for(int x = 0; x < dimsOld[0]-1; x++)
                {
                    // Get the value at that (x,y,z) location
                    bool* val = static_cast<bool*>(voxData->GetScalarPointer(x,y,z));
                    //qDebug () << *val;
                    // Compare coordinates of value 1 and establish boundaries around the vehicle
                    if(*val)
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
        qDebug() << "object boundaries: " << "(" << extentObj[0] << "," << extentObj[1] << ")" << "(" << extentObj[2] << "," << extentObj[3] << ")" << "(" << extentObj[4] << "," << extentObj[5] << ")";
        // get object center
        int objCenter[3];
        objCenter[0] = (extentObj[1] - extentObj[0])/2;
        objCenter[1] = (extentObj[3] - extentObj[2])/2;
        objCenter[2] = (extentObj[5] - extentObj[4])/2;

        /* *** Extend this for a general case (i.e, take inputs from user)
         * dimensions of bulldoser is ~ 8.1 x 4.5 x 4 m --- translates to ~ 63.3 mm / voxel on z-axis (length)
         * a distance of 12m roughly translates to 190 voxels, so, we create a 400x(maxBound_y)x400 grid with the vehicle at the center
         *
         * **** for 16 voxel -> 506.250 mm/voxel => 12m = 24 voxels                   => grid = 60 vox
         * **** for 32 voxel -> 253.125 mm/voxel => 12m equivalent to about 48 voxels => grid = 120 vox
         *
         */
        int worldCenter;     // 400/2
        int envDim;
        getEnvParams(envDim, worldCenter);
        // Translate the object in x-z such that the object center lies at 100 in x-z
        int transFactor[3];
        transFactor[0] = worldCenter - objCenter[0];
        transFactor[1] = 0;                         // No translation in y as anything below ground plane is not of interest
        transFactor[2] = worldCenter - objCenter[2];

        vtkNew<vtkImageTranslateExtent> transExtent;
        transExtent->SetTranslation(transFactor);
        transExtent->SetInputData(voxData);
        transExtent->Update();
        voxData->DeepCopy(transExtent->GetOutput());

        // update object center w.r.t. world
        objCenter[0] += transFactor[0];
        objCenter[1] += transFactor[1];
        objCenter[2] += transFactor[2];
        qDebug() << "Object center w.r.t. world: (" << objCenter[0] << "," << objCenter[1] << "," << objCenter[2] << ")";

        // Define and Set new extent for the simulation environment
        int extentNew[6];
        extentNew[0] = 0;
        extentNew[1] = envDim;
        extentNew[2] = 0;
        extentNew[3] = extentObj[3] + 2;                // Space over the top of the vehicle is not of interest
        extentNew[4] = 0;
        extentNew[5] = envDim;

        // Create a new volume and copy contents from translated model
        voxWorld = vtkSmartPointer<vtkStructuredPoints>::New();
        voxWorld->SetExtent(extentNew);
        voxWorld->AllocateScalars(VTK_UNSIGNED_INT, 1);
        // get extent of translated object
        voxData->GetExtent(extent);
        // initialize vehicleVoxels container
        vehicleVoxels.clear();

        for(int z = extentNew[4]; z <= extentNew[5]; z++)
        {
            for(int y = extentNew[2]; y <= extentNew[3]; y++)
            {
                for(int x = extentNew[0]; x <= extentNew[1]; x++)
                {
                    unsigned int* valFinal = static_cast<unsigned int*>(voxWorld->GetScalarPointer(x,y,z));
                    *valFinal = 0;
                    // Check if the index falls within the dimensions of translated volume
                    if(x >= extent[0] && x <= extent[1] && y >= extent[2] && y <= extent[3] && z >= extent[4] && z <= extent[5])
                    {
                        // get the value at that location and update in the voxWorld volume if it is one
                        bool* valTemp = static_cast<bool*>(voxData->GetScalarPointer(x,y,z));
                        if(*valTemp)
                        {
                            *valFinal = 9;
                            // Add the voxel in vehicleVoxels container to use later in visibility checks
                            vehicleVoxels.append(QVector3D(x,y,z));
                        }
                    }
                }
            }
        }

        voxWorld->GetExtent(extent);
        qDebug() << "Extent New: " << "(" << extent[0] << "," << extent[1] << ")" << "(" << extent[2] << "," << extent[3] << ")" << "(" << extent[4] << "," << extent[5] << ")";

        visualizeVolume(voxWorld);
    }
}

void VTKWidget::processError(QProcess::ProcessError error)
{
    qDebug() << "error enum val = " << error << endl;
}

void VTKWidget::dilatePressed()
{
    /* First extract the object boundary voxels, use one of the following:
     * 1) morphological dilation on the volume and get difference of volumes
     * 2) marching cubes (produces triangles or polygons)
     * 3) Dividing cubes (generates points representing the boundary)
     */

    // apply dilation on the volume using itkProcessing class
    vtkSmartPointer<vtkStructuredPoints> dilatedWorld = vtkSmartPointer<vtkStructuredPoints>::New();
    dilatedWorld->DeepCopy(voxWorld);
    int extent[6];
    voxWorld->GetExtent(extent);


    vtkSmartPointer<vtkStructuredPoints> volGradientMagnitude;          // volume showing gradient magnitudes in 3D
    QVector<vtkSmartPointer<vtkStructuredPoints>> gradientVolumes;      // vector holding gradient volumes after dilation in three dimensions

    // Use ITK toolkit for morphological dilation
    volumeProcessor.volumeDilation(dilatedWorld);
    //unsigned int maxVal = dilatedWorld->GetScalarRange()[1];
    //qDebug() << "maximum dilated value: " << maxVal;
    //qDebug() << "Number of cells before dilation: " << voxWorld->GetNumberOfCells();
    //qDebug() << "Number of cells after dilation: " << dilatedWorld->GetNumberOfCells();

    // Convert the image datatype to float to be able to process gradients
    vtkSmartPointer<vtkStructuredPoints> floatDilatedWorld = vtkSmartPointer<vtkStructuredPoints>::New();
    qDebug() << dilatedWorld->GetScalarType();
    vtkNew<vtkImageShiftScale> shiftFilter;
    shiftFilter->SetOutputScalarTypeToFloat();
    shiftFilter->SetInputData(dilatedWorld);
    shiftFilter->SetScale(1);
    shiftFilter->SetShift(0);
    shiftFilter->Update();
    floatDilatedWorld->DeepCopy(shiftFilter->GetOutput());
    //qDebug() << floatDilatedWorld->GetScalarType();
    //qDebug() << floatDilatedWorld->GetScalarRange()[0] << " " << floatDilatedWorld->GetScalarRange()[1];

    // Get Gradient images for the dilated volume
    gradientVolumes.clear();
    volumeProcessor.generateNormals(floatDilatedWorld, gradientVolumes);

    // volume image showing only the voxels added after dilation
    volBoundary = vtkSmartPointer<vtkStructuredPoints>::New();
    volBoundary->SetExtent(extent);
    volBoundary->AllocateScalars(VTK_UNSIGNED_INT, 1);

    // initialize the gradient magnitude image
    volGradientMagnitude = vtkSmartPointer<vtkStructuredPoints>::New();
    volGradientMagnitude->SetExtent(extent);
    volGradientMagnitude->AllocateScalars(VTK_FLOAT, 1);

    // ----------------------------Extract data Points and setup additional descriptive volumes-------------------------------------

    // initialize the containers that hold data points
    boundaryVoxels.clear();
    boundaryNormals.clear();
    backgroundVoxels.clear();
    backgroundNormals.clear();

    qDebug() << gradientVolumes[0]->GetScalarRange()[0] << " " << gradientVolumes[0]->GetScalarRange()[1];

    int volDims[3];
    voxWorld->GetDimensions(volDims);

    // get world center
    int center[3] = {volDims[0]/2, volDims[1]/2, volDims[2]/2};
    // initialize and resize the indices vector
    vectorIndices.clear();
    vectorIndices.resize(volDims[0]*volDims[1]*volDims[2]);

    /* Define radii and height of spherical segment (These will be used only if bowlSurface=true)
     * This is defined by radius of upper circle, a1=190
     * radius of lower cirlce, a2=140
     * height of spherical segment, h= int(0.75 * y_max dimension)
     * radius of sphere (r) and distance to center (i) are needed and have to be calculated as per formulae at:
     * https://rechneronline.de/pi/spherical-segment.php
     */
    float bowl_a1, bowl_a2;
    getBowlRad(bowl_a1, bowl_a2);
    int bowl_h = volDims[1] * 0.75;
    // distance to center i = ( a1² - a2² - h² ) / 2h
    float bowl_i = (pow(bowl_a1,2) - pow(bowl_a2,2) - pow(bowl_h,2))/(2 * bowl_h);
    // radius r = √ (a1² + [ ( a1² - a2² - h² ) / 2h ]²) = √ (a1² + [i]²)
    float bowl_r = sqrt(pow(bowl_a1,2) + pow(bowl_i,2));
    /* Calculate max possible radius of spherical segment at each height (y) until bowl_h
     * bowl_maxR can be calculated using pythagoras theorem where one side is maxR, one is radius of sphere r
     * and the third side is given by i + h - y
     */
    QVector<float> bowl_maxR(bowl_h);
    for(int y = 0; y < bowl_h; y++)
    {
        // maxR = √ (r² - [i + h - y]²)
        bowl_maxR[y] = sqrt(pow(bowl_r,2) - pow((bowl_i + bowl_h - y),2));
    }

    int count = 0;
    int countBackground = 0;
    for(int z = 0; z <= volDims[2]-1; z++)
    {
        for(int y = 0; y <= volDims[1]-1; y++)
        {
            for(int x = 0; x <= volDims[0]-1; x++)
            {
                // Get the values at the location (x,y,z) from different voxel environments
                unsigned int* valWorld = static_cast<unsigned int*>(voxWorld->GetScalarPointer(x,y,z));
                unsigned int* valDilated = static_cast<unsigned int*>(dilatedWorld->GetScalarPointer(x,y,z));
                unsigned int* boundVal = static_cast<unsigned int*>(volBoundary->GetScalarPointer(x,y,z));
                float* normalX = static_cast<float*>(gradientVolumes[0]->GetScalarPointer(x,y,z));
                float* normalY = static_cast<float*>(gradientVolumes[1]->GetScalarPointer(x,y,z));
                float* normalZ = static_cast<float*>(gradientVolumes[2]->GetScalarPointer(x,y,z));
                float* gradientMagnitude = static_cast<float*>(volGradientMagnitude->GetScalarPointer(x,y,z));
                *gradientMagnitude = 0.0f;
                *boundVal = 0;

                /* Calculate the distance of the point from the center of voxWorld for collecting background voxels within 12m
                 * According to environment definition 12m ~= 190 voxels
                 */
                float distance = sqrt(pow(x - center[0], 2) + pow(y - center[1], 2) + pow(z - center[2], 2));

                /* Value 1 at (x,y,z) in dilatedWorld but not in voxWorld represents boundary voxels.
                 * So, change those values to '2' and leave the rest as they are
                 */
                //qDebug() << "dilated value = " << *valDilated;
                if(*valDilated==9 && *valWorld==0)
                {
                    // Calculate magnitude of the gradient vector at each boundary voxel
                    *gradientMagnitude = sqrt(pow(*normalX,2) + pow(*normalY,2) + pow(*normalZ,2));
                    // Add dilated voxels into the global volume voxWorld
                    *valWorld = 8;
                    // Do not add points with normals parallel to y-axis { (0,1,0) and (0,-1,0) } cases (invert the values because sobel filter produces gradients facing into the object)
                    QVector3D voxNormal = QVector3D(-*normalX, -*normalY, -*normalZ);
                    if((voxNormal != QVector3D(0.0f, 1.0f, 0.0f)) && (voxNormal != QVector3D(0.0f, -1.0f, 0.0f)))
                    {
                        // Collect all the voxel coordinates lying on the vehicle boundary
                        boundaryVoxels.append(QVector3D(x,y,z));
                        // Also collect the corresponding surface normal
                        boundaryNormals.append(voxNormal);
                        /* Add this index at the 1D index of (x,y,z) coordinate
                         * 1D index calculated as x + y * max_x + z * max_x * max_y
                         */
                        vectorIndices[x + (y * extent[1]) + (z * extent[1] * extent[3])] = count;
                        // Highlight the boundary voxels with a value '2' in a boundary volume dataset
                        *boundVal = 2;
                        count++;
                    }
                }
                /* Collect all the background voxels (representing zero in both voxWorld and dilatedWorld)
                 * We also want to highlight these voxels with different opacity from the uncollected background voxels
                 */
                else if(*valDilated==0 && distance <= 190.0f && !bowlSurface)
                {
                    backgroundVoxels.append(QVector3D(x,y,z));
                    /* Calculate the normal for this background voxel
                     * Normal will be the direction from this point towards the center of the vehicle
                     */
                    backgroundNormals.append((QVector3D(center[0],center[1],center[2]) - QVector3D(x,y,z)).normalized());
                    /* Add this index at the 1D index of (x,y,z) coordinate
                     * 1D index calculated as x + y * max_x + z * max_x * max_y
                     */
                    vectorIndices[x + (y * extent[1]) + (z * extent[1] * extent[3])] = countBackground;
                    // highlight the voxels with value 1
                    *valWorld = 1;
                    // also highlight these in the boundary volume environment (for SLIC)
                    *boundVal = 1;
                    countBackground++;
                }
                /* Collect background points lying on the bowl shaped surface. Conditions :
                 * voexl must not be vehicle -> valDilated = 0;
                 * y < bowl_h (background voxels over that height are not of interest)
                 */
                else if(*valDilated==0 && bowlSurface && y < bowl_h)
                {
                    // Condition based if y is at ground plane or not
                    bool sphereSegCondition;
                    // Distance of the point from center in x-z plane
                    float sphereSegDistance = sqrt(pow(x - center[0], 2) + pow(z - center[2], 2));

                    // first check if y = 0 implying ground-plane
                    if(y == 0)
                    {
                        sphereSegCondition = (sphereSegDistance <= bowl_maxR[y]);
                    }
                    else
                    {
                        sphereSegCondition = (sphereSegDistance - 0.5f < bowl_maxR[y]) && (bowl_maxR[y] < sphereSegDistance + 0.5f);
                    }

                    // For ground plane collect all voxels that lie within this raius
                    if(sphereSegCondition)
                    {
                        backgroundVoxels.append(QVector3D(x,y,z));
                        /* Calculate the normal for this background voxel
                         * Normal will be the direction from this point towards the center of the vehicle
                         */
                        backgroundNormals.append((QVector3D(center[0],center[1],center[2]) - QVector3D(x,y,z)).normalized());
                        /* Add this index at the 1D index of (x,y,z) coordinate
                         * 1D index calculated as x + y * max_x + z * max_x * max_y
                         */
                        vectorIndices[x + (y * extent[1]) + (z * extent[1] * extent[3])] = countBackground;
                        // highlight the voxels with value 1
                        *valWorld = 1;
                        // also highlight these in the boundary volume environment (for SLIC)
                        *boundVal = 1;
                        countBackground++;
                    }
                }
            }
        }
    }

    // Create a volume to show random selected boundary points
    vtkSmartPointer<vtkStructuredPoints> randomWorld = vtkSmartPointer<vtkStructuredPoints>::New();
    randomWorld->SetExtent(extent);
    randomWorld->AllocateScalars(VTK_UNSIGNED_INT, 1);
    for(int z = 0; z <= volDims[2]-1; z++)
    {
        for(int y = 0; y <= volDims[1]-1; y++)
        {
            for(int x = 0; x <= volDims[0]-1; x++)
            {
                unsigned int* randVox = static_cast<unsigned int *>(randomWorld->GetScalarPointer(x,y,z));
                *randVox = 0;
            }
        }
    }

    /* Process background voxels part only when the appropriate choice
     * parameter is set. Or else we work only on the voxel surface
     * 1 --> segment
     * 0 --> random point selection
     */
    //segChoice = 0;  // To cluster both surface and background volume
    if(segChoice)
    {
        // SLIC segmentation on background voxels
        superBackVoxels.clear();
        superBackNormals.clear();
        segmentBackLabels.clear();
        int backClusters = 20000;//800;
        // value of background voxels is 1
        unsigned int volVal = 1;
        volumeProcessor.slicSegmentation(backgroundVoxels,
                                         backgroundNormals,
                                         volBoundary,
                                         backClusters,
                                         superBackVoxels,
                                         superBackNormals,
                                         segmentBackLabels,
                                         volVal,
                                         vectorIndices);

        // setup points by the cluster for easier access later
        clusteredBackground.resize(superBackVoxels.size());
        // Update boundary volume with background voxel segments
        for(int i = 0; i < countBackground; i++)
        {
            // get voxel coordinates
            QVector3D voxelCoords = backgroundVoxels[i];
            // get the voxel value
            unsigned int* backVox = static_cast<unsigned int *>(volBoundary->GetScalarPointer(voxelCoords.x(), voxelCoords.y(), voxelCoords.z()));
            //Set the value with the label such that all sgements have a value between 1 and 10
            *backVox = (segmentBackLabels[i] % 10) + 1;
            // clustered points setup
            clusteredBackground[segmentBackLabels[i]].append(i);
        }
    }
    /* Clustering background voxels doesnt make sense. Moreover, even 2000 background voxel clusters are too few
     * to give decent accuracy of at least 85%. We need at least upto 20000 background points to get decent accuracy
     * over 90%. Clustering into such high number takes too long, so best is to randomly select desired number of points
     * from the available list of background voxels.
     */
    else
    {
        /* Background voxel random selection
         */
        srand((int)QTime::currentTime().msec());
        // Verify if count is same as background voxels size
        if(countBackground != backgroundVoxels.size())
            qDebug() << "countBackground is not same as backgroundvoxels.size()!!";

        /* QVector to temporarily store indices which can be later added to 'clusteredBackground' container.
         * This Qvector can be cleared each time a new background point is randomly selected. This is to ensure
         * that we dont get any errors later when setting up the result volume
         */
        //QVector<int> tempClusteredBack;
        clusteredBackground.clear();
        // background point sampling
        // 32  -> 20
        // 64  -> 20
        // 128 -> 100
        // 256 -> 250

        for(int i = 0; i < countBackground; i+=20)
        {
            // First add the point into the vector
            //tempClusteredBack.append(i);
            // Genrate a random number and check if it is less than 1% of total
            //int r = (rand() % 100) + 1;
            //if(r <= 1)
            //{
            superBackVoxels.append(backgroundVoxels[i]);
            // Append the point also into clusteredBackground vector
            clusteredBackground.append(QVector<int>({i}));
            // Mark these points in random world volume
            unsigned int* randVox = static_cast<unsigned int *>(randomWorld->GetScalarPointer(backgroundVoxels[i].x(), backgroundVoxels[i].y(), backgroundVoxels[i].z()));
            *randVox = 2;
            // Append the temporary vector in 'clusteredBackground' and reinitialize
            //clusteredBackground.append(tempClusteredBack);
            //tempClusteredBack.clear();
            //}
        }
    }

    /* Update boundary volume with the segments
    for(int i = 0 ; i < count; i++)
    {
        // get the voxel coordinates
        QVector3D voxelCoords = boundaryVoxels[i];
        // get the boundary voxel value
        unsigned int* boundVox = static_cast<unsigned int *>(volBoundary->GetScalarPointer(voxelCoords.x(), voxelCoords.y(), voxelCoords.z()));
        // Set the value with the label such that all sgements have a value between 1 and 10
        // And shift all the values by 50 to distinguish from background super voxels
        *boundVox = (segmentLabels[i] % 10) + 1 + 50;
    }*/

    //-------------------------------------------Save Volumes so far created------------------------------------------
    // Save all the images in .vtk files for verification on paraview
    std::vector<std::string> splitFileName;
    boost::split(splitFileName, fileNameVTK, [](char c){return c == '.';});

    vtkNew<vtkGenericDataObjectWriter> writer;
//    vtkNew<vtkGenericDataObjectWriter> writerFloatDilated;
    vtkNew<vtkGenericDataObjectWriter> writerBoundary;
//    vtkNew<vtkGenericDataObjectWriter> writerGradient;
//    vtkNew<vtkGenericDataObjectWriter> writerGradientX;
//    vtkNew<vtkGenericDataObjectWriter> writerRand;
    // Save the global volume
    writer->SetInputData(voxWorld);
    writer->SetFileName((splitFileName[0] + "_world.vtk").c_str());
    writer->Write();
    // Save the volume model with only boundary voxels
    writerBoundary->SetInputData(volBoundary);
    writerBoundary->SetFileName((splitFileName[0] + "_boundary.vtk").c_str());
    writerBoundary->Write();
//    // Save the floating point dilated volume
//    writerFloatDilated->SetInputData(floatDilatedWorld);
//    writerFloatDilated->SetFileName((splitFileName[0] + "_floatDilated.vtk").c_str());
//    writerFloatDilated->Write();
//    // Save the gradient magnitude image
//    writerGradient->SetInputData(volGradientMagnitude);
//    writerGradient->SetFileName((splitFileName[0] + "_gradientMagnitude.vtk").c_str());
//    writerGradient->Write();
//    // Save gradients image in X direction
//    writerGradientX->SetInputData(gradientVolumes[0]);
//    writerGradientX->SetFileName((splitFileName[0] + "_gradientX.vtk").c_str());
//    writerGradientX->Write();
//    // Save the volume with only random selected background points marked
//    writerRand->SetInputData(randomWorld);
//    writerRand->SetFileName((splitFileName[0] + "_random_selected.vtk").c_str());
//    writerRand->Write();

    // Visualize only the boundary voxels after dilation
    qDebug() << "Number of boundary voxels: " << boundaryVoxels.size();
    qDebug() << "Number of background points collected: " << superBackVoxels.size() << " / " << countBackground;
    visualizeVolume(dilatedWorld);
}

void VTKWidget::loadSimulated()
{
    // Object for getting simulated envirinments
    // 1 -> Simple car
    // 2 -> van
    // 3 -> hatchback
    // 4 -> truck
    // 5 -> bus
    SimulatedObjects simulatedEnv(5);
    // get the voxel data and other vectors for optimization
    voxWorld = simulatedEnv.getSimData(boundaryVoxels, boundaryNormals, vehicleVoxels, vectorIndices);
    qDebug() << "Using Simulated Objects!";
    qDebug() << "# of boundary voxels = " << boundaryVoxels.size();
    // Save the voxel image. Could be useful later to show only objects in paper
    std::vector<std::string> splitFileName;
    boost::split(splitFileName, fileNameVTK, [](char c){return c == '.';});
    vtkNew<vtkXMLImageDataWriter> writer;
    writer->SetInputData(voxWorld);
    writer->SetFileName((splitFileName[0] + "_SimVehicle.vti").c_str());
    writer->Write();

    // ------------------------- Setup Background Points --------------------------------
    int volDims[3];
    voxWorld->GetDimensions(volDims);
    /* Length of average car is considered 4.7m. Simple car object has length of 9 voxels
     * So, 1 vox ~= 0.52m. Hence 12m ~= 23 voxels. Therefore, by calculations for bowl surface above,
     * a1 = 23 and a2 = 17 voxels. and height would be
     * formula for a1 is 12/(4.7/vehicleDims[2])
     */
    int vehicleDims[3];
    // The dimensions are in order width,height,length to match with x,y,z
    simulatedEnv.getVehicleDims(vehicleDims);
    // Define a center of vehicle
    int cent[3] = {volDims[0]/2, volDims[1]/2, volDims[2]/2};
    // calculate a1, a2 and h of bowl surface
    float voxUnit = 4.7/vehicleDims[2];
    const float bowl_a1 = round(12/voxUnit), bowl_a2 = bowl_a1 - 6;
    int bowl_h = round(vehicleDims[1] * 0.75);
    // distance to center i = ( a1² - a2² - h² ) / 2h
    float bowl_i = (pow(bowl_a1,2) - pow(bowl_a2,2) - pow(bowl_h,2))/(2 * bowl_h);
    // radius r = √ (a1² + [ ( a1² - a2² - h² ) / 2h ]²) = √ (a1² + [i]²)
    float bowl_r = sqrt(pow(bowl_a1,2) + pow(bowl_i,2));
    /* Calculate max possible radius of spherical segment at each height (y) until bowl_h
     * bowl_maxR can be calculated using pythagoras theorem where one side is maxR, one is radius of sphere r
     * and the third side is given by i + h - y
     */
    QVector<float> bowl_maxR(bowl_h);
    for(int y = 0; y < bowl_h; y++)
    {
        // maxR = √ (r² - [i + h - y]²)
        bowl_maxR[y] = sqrt(pow(bowl_r,2) - pow((bowl_i + bowl_h - y),2));
    }

    backgroundVoxels.clear();
    for(int z = 0; z <= volDims[2]-1; z++)
    {
        for(int y = 0; y <= volDims[1]-1; y++)
        {
            for(int x = 0; x <= volDims[0]-1; x++)
            {
                // Get the values at the location (x,y,z) from different voxel environments
                unsigned int* valWorld = static_cast<unsigned int*>(voxWorld->GetScalarPointer(x,y,z));
                // Consider voxels with value 0 (not vehicle) and within the bowl surface height
                if(*valWorld == 0 && y < bowl_h)
                {
                    // Condition based if y is at ground plane or not
                    bool sphereSegCondition;
                    // Distance of the point from center in x-z plane
                    float sphereSegDistance = sqrt(pow(x - cent[0], 2) + pow(z - cent[2], 2));
                    // first check if y = 0 implying ground-plane
                    if(y == 0)
                    {
                        sphereSegCondition = (sphereSegDistance <= bowl_maxR[y]);
                    }
                    else
                    {
                        sphereSegCondition = (sphereSegDistance - 0.5f < bowl_maxR[y]) && (bowl_maxR[y] < sphereSegDistance + 0.5f);
                    }
                    // For ground plane collect all voxels that lie within this raius
                    if(sphereSegCondition)
                    {
                        // append the voxel to set of background voxels
                        backgroundVoxels.append(QVector3D(x,y,z));
                        // Mark the voxel in volume with value 1
                        *valWorld = 1;
                    }
                }
            }
        }
    }

    // Selection of 1% of background voxels
    qDebug() << "Background Voxels before random selection = " << backgroundVoxels.size();
//    srand((int)QTime::currentTime().msec());
//    clusteredBackground.clear();
//    for(int i = 0; i < backgroundVoxels.size(); i++)
//    {
//        // Genrate a random number and check if it is less than 1% of total
//        // even 25% points do not give any result, so select 50% points
//        int r = (rand() % 100) + 1;
//        if(r <= 50)
//        {
//            superBackVoxels.append(backgroundVoxels[i]);
//            // Append the point also into clusteredBackground vector
//            clusteredBackground.append(QVector<int>({i}));
//        }
//    }
    // To avoid random selection, select every second point (50%)
    clusteredBackground.clear();
    for(int i = 0; i < backgroundVoxels.size(); i+=4)
    {
        superBackVoxels.append(backgroundVoxels[i]);
        clusteredBackground.append(QVector<int>({i}));
    }
    qDebug() << "Background voxels after random selection = " << superBackVoxels.size();
    // Save the volume to show vehicle with selected background points
    vtkNew<vtkXMLImageDataWriter> writer1;
    writer1->SetInputData(voxWorld);
    writer1->SetFileName((splitFileName[0] + "_SimWorld.vti").c_str());
    writer1->Write();

    // Copy this volume into volBoundary because that is being used in multi-reso optimization
    volBoundary = vtkSmartPointer<vtkStructuredPoints>::New();
    volBoundary->DeepCopy(voxWorld);

    visualizeVolume(voxWorld);
}

void VTKWidget::continuousModel()
{
    // Initialize volume
    voxWorld = vtkSmartPointer<vtkStructuredPoints>::New();
    int extent[6];
    extent[0] = 0;
    extent[1] = 59;
    extent[2] = 0;
    extent[3] = 29;
    extent[4] = 0;
    extent[5] = 59;
    voxWorld->SetExtent(extent);
    voxWorld->AllocateScalars(VTK_UNSIGNED_INT, 1);

    // Define radius of hemisphere
    double radius = 7;
    // volume dimensions
    int volDims[3];
    voxWorld->GetDimensions(volDims);
    int cent[3] = {volDims[0]/2, 0, volDims[2]/2};
    // Bowl surface aroudn the hemisphere
    const float bowl_a1 = 23.0, bowl_a2 = 17;
    int bowl_h = round(radius * 0.75);
    // distance to center i = ( a1² - a2² - h² ) / 2h
    float bowl_i = (pow(bowl_a1,2) - pow(bowl_a2,2) - pow(bowl_h,2))/(2 * bowl_h);
    // radius r = √ (a1² + [ ( a1² - a2² - h² ) / 2h ]²) = √ (a1² + [i]²)
    float bowl_r = sqrt(pow(bowl_a1,2) + pow(bowl_i,2));
    /* Calculate max possible radius of spherical segment at each height (y) until bowl_h
     * bowl_maxR can be calculated using pythagoras theorem where one side is maxR, one is radius of sphere r
     * and the third side is given by i + h - y
     */
    QVector<float> bowl_maxR(bowl_h);
    for(int y = 0; y < bowl_h; y++)
    {
        // maxR = √ (r² - [i + h - y]²)
        bowl_maxR[y] = sqrt(pow(bowl_r,2) - pow((bowl_i + bowl_h - y),2));
    }

    for(int x = 0; x <= extent[1]; x++)
    {
        for(int y = 0; y <= extent[3]; y++)
        {
            for(int z = 0; z <= extent[5]; z++)
            {
                // Get the values at the location (x,y,z) from different voxel environments
                unsigned int* valWorld = static_cast<unsigned int*>(voxWorld->GetScalarPointer(x,y,z));
                // intialize all voxels to zero
                *valWorld = 0;
                // Calculate distance between the point and center
                float dist = sqrt(pow((cent[0]-x),2) + pow((cent[1]-y),2) + pow((cent[2]-z),2));
                // Bowl surface around hemisphere. Draw it only outside the hemisphere
                if((dist > radius) && (y < bowl_h))
                {
                    // Condition based if y is at ground plane or not
                    bool sphereSegCondition;
                    // Distance of the point from center in x-z plane
                    float sphereSegDistance = sqrt(pow(x - cent[0], 2) + pow(z - cent[2], 2));
                    // first check if y = 0 implying ground-plane
                    if(y == 0)
                    {
                        sphereSegCondition = (sphereSegDistance <= bowl_maxR[y]);
                    }
                    else
                    {
                        sphereSegCondition = (sphereSegDistance - 0.5f < bowl_maxR[y]) && (bowl_maxR[y] < sphereSegDistance + 0.5f);
                    }
                    // For ground plane collect all voxels that lie within this raius
                    if(sphereSegCondition)
                    {
                        // append the voxel to set of background voxels
                        backgroundVoxels.append(QVector3D(x,y,z));
                        // Mark the voxel in volume with value 1
                        *valWorld = 1;
                    }
                }
                // If distance is within radius+-1, then set it as boundary voxel
                if(((radius - 1) < dist) && (dist < (radius + 1)))
                {
                    // set the voxel to 2 and add it to set of boundaryVoxels
                    *valWorld = 2;
                    boundaryVoxels.append(QVector3D(x,y,z));
                }
            }
        }
    }
    std::vector<std::string> splitFileName;
    boost::split(splitFileName, fileNameVTK, [](char c){return c == '.';});
    vtkNew<vtkXMLImageDataWriter> writer1;
    writer1->SetInputData(voxWorld);
    writer1->SetFileName((splitFileName[0] + "_ContWorld.vti").c_str());
    writer1->Write();

    // Output containers
    QVector<QVector3D> solCamPos, solCamDir;
    QVector<int> backSol;
    // Set number of cameras to 4
    int numCam = 4;
    // Setup camera parameters as defined in VisibilityChecks for mini model
    QVector3D camParameters(16, 12, 26);
    // --------------------Call MIP optimizer for continuous model------------------------------
    // Timer to calculate optimization time
    QElapsedTimer optimizationTimer;
    int timeMS;
    MIPOptimizer mipOptimizer;
    qDebug() << "Starting OR-tools Optimization!!!";
    optimizationTimer.start();
    mipOptimizer.continuousOptimization(backgroundVoxels, numCam, camParameters, radius, solCamPos, solCamDir, backSol);
    timeMS = optimizationTimer.elapsed();
    qDebug() << "Total optimization time (MIP optimizer) = " << timeMS << "ms = " << timeMS/1000.0 << "s";

    // The output camera positions are float obatined from spherical to cartesian coordinates conversion
    // So, round them off to nearest integers to be shown on final volume with camera FOVs.
    // also calculate the up and right vectors
    QVector<QVector3D> finalSolCamPos(numCam), solCamUp(numCam), solCamRight(numCam);
    QVector3D worldUp(0.0f, 1.0f, 0.0f);
    for(int i = 0; i < numCam; i++)
    {
        finalSolCamPos[i] = QVector3D(round(solCamPos[i].x()), round(solCamPos[i].y()), round(solCamPos[i].z()));
        solCamRight[i] = QVector3D::crossProduct(solCamDir[i], worldUp).normalized();
        solCamUp[i] = QVector3D::crossProduct(solCamRight[i], solCamDir[i]).normalized();
    }

    // setup the final result volume
    SetupFinalVolume(finalSolCamPos, solCamDir, solCamUp, solCamRight, "continuous");

//    // get normals for the voxels0.72
//    vtkSmartPointer<vtkStructuredPoints> volGradientMagnitude;          // volume showing gradient magnitudes in 3D
//    QVector<vtkSmartPointer<vtkStructuredPoints>> gradientVolumes;      // vector holding gradient volumes after dilation in three dimensions
//    // Convert the image datatype to float to be able to process gradients
//    vtkSmartPointer<vtkStructuredPoints> floatWorld = vtkSmartPointer<vtkStructuredPoints>::New();
//    //qDebug() << voxWorld->GetScalarType();
//    vtkNew<vtkImageShiftScale> shiftFilter;
//    shiftFilter->SetOutputScalarTypeToFloat();
//    shiftFilter->SetInputData(voxWorld);
//    shiftFilter->SetScale(1);
//    shiftFilter->SetShift(0);
//    shiftFilter->Update();
//    floatWorld->DeepCopy(shiftFilter->GetOutput());
//    // Get Gradient images for the dilated volume
//    gradientVolumes.clear();
//    volumeProcessor.generateNormals(floatWorld, gradientVolumes);
//    // Collect the normals of voxels on the hemisphere
//    for(int i = 0; i < boundaryVoxels.size(); i++)
//    {
//        QVector3D currVox = boundaryVoxels[i];
//        //qDebug() << currVox << " & i= " << i;
//        int x = currVox.x();
//        int y = currVox.y();
//        int z = currVox.z();
//        // Get the values at the location (x,y,z) from different voxel environments
//        float* normalX = static_cast<float*>(gradientVolumes[0]->GetScalarPointer(x,y,z));
//        float* normalY = static_cast<float*>(gradientVolumes[1]->GetScalarPointer(x,y,z));
//        float* normalZ = static_cast<float*>(gradientVolumes[2]->GetScalarPointer(x,y,z));
//        QVector3D voxNormal = QVector3D(-*normalX, -*normalY, -*normalZ);
//        boundaryNormals.append(voxNormal);
//    }


//    // ----------- Code only for test and Debug-------------
//    QVector3D camParams(16,12,26);
//    for(int j = 0; j < backgroundVoxels.size(); j++)
//    {
//        // pick a set of 4 boundary voxels and normals
//        QVector<QVector3D> selectBoundVox = {boundaryVoxels[6], boundaryVoxels[126], boundaryVoxels[233], boundaryVoxels[578], boundaryVoxels[432]};
//        QVector<QVector3D> selectBoundNorm = {boundaryNormals[6], boundaryNormals[126], boundaryNormals[233], boundaryNormals[578], boundaryNormals[432]};
//        // Calculate d as the dot product between control point and (roll, theta, phi)
//        for(int i = 0; i < selectBoundVox.size(); i++)
//        {
//            double d = QVector3D::dotProduct(backgroundVoxels[j], selectBoundNorm[i]);
//            // Projection on far plane
//            QVector3D c_dash = camParams.z() * ((/*selectBoundVox[i] +*/ backgroundVoxels[j])/d - selectBoundNorm[i]);
//            // World up vector
//            QVector3D worldUp(0.0f, 1.0f, 0.0f);
//            QVector3D right = QVector3D::crossProduct(selectBoundNorm[i], worldUp).normalized();
//            QVector3D up = QVector3D::crossProduct(right, selectBoundNorm[i]).normalized();
//            // Calculate u_dash and v_dash
//            float u_dash = QVector3D::dotProduct(c_dash, right);
//            float v_dash = QVector3D::dotProduct(c_dash, up);
//            qDebug() << "Point : " << backgroundVoxels[j] << " projected : " << c_dash;
//            qDebug() << " u,v values = " << u_dash << " " << v_dash;
//            qDebug() << "Test";
//        }
//    }

    visualizeVolume(voxWorld);
}

void VTKWidget::multiResolution()
{
    /* A container defining number of clusters required at each resolution
     * Since we cannot work with more than 150 clusters, begin with that number
     * and decrease at each resolution.
     * Another container to represent corresponding rotations number can also be created.
     * This enables to work with low number of resolutions for big number of clusters and
     * more rotations for higher resolutions when number of clusters is small.
     */
    //---------------------------------Inititalization------------------------------------
    // Timer to keep total time of optimization at diff. resolutions
    double totalTimer = 0.0;
    double clusterTimer = 0.0;
    // vectors to hold segmented voxels, normals, labels and solution indices
    QVector<QVector3D> superVoxels;
    QVector<QVector3D> superNormals;
    QVector<int> segmentLabels;
    QVector<QVector<int>> camSolution;
    // Vector to hold 3D points and directions of final selected cameras
    QVector<QVector3D> finalCamPoint, finalCamDir, finalCamUp, finalCamRight;
    // Number of rotations, for first resolution can be set to 13
    int numRotations = 29;
    // Number of cameras, for first can be set to 5
    int numCamReq = 5;
    // number of clusters required, for first resolution it is 140
    int clusters = 9;
    int maxAllowedPoints = 50;
    // Value representing the value of boundary voxels (initially the boundary voxels are set to a value 2)
    unsigned int volVal = 2;

    superVoxels.clear();
    superNormals.clear();
    segmentLabels.clear();
    // ------------------- SLIC clustering------------------------------------------------
    // Call the function perform SLIC based segmentation
    volumeProcessor.slicSegmentation(boundaryVoxels,
                                     boundaryNormals,
                                     volBoundary,
                                     clusters,
                                     superVoxels,
                                     superNormals,
                                     segmentLabels,
                                     volVal,
                                     vectorIndices);

    // Setup up points by the cluster for easier access later
    QVector<QVector<int>> clusteredBoundary;
    clusteredBoundary.resize(superVoxels.size());
    for(int i = 0; i < segmentLabels.size(); i++)
    {
        clusteredBoundary[segmentLabels[i]].append(i);
    }


    // Create Visibility check object
    VisibilityCheck visibilityCheck(superVoxels, superNormals, superBackVoxels, vehicleVoxels, numRotations);
    // Optimize
    totalTimer += optimizeData(visibilityCheck, numCamReq, superBackVoxels, camSolution);

    // setup solution to create a volume displaying result at this resolution
    QVector<QVector3D> initCamPoint(numCamReq), initCamDir(numCamReq), initCamUp(numCamReq), initCamRight(numCamReq);
    for(int i = 0; i < camSolution.size(); i++)
    {
        // Add the point to solution container
        initCamPoint[i] = superVoxels[camSolution[i][0]];
        // get the corresponding direction, up and right vectors
        visibilityCheck.getCamVectors(camSolution[i], initCamDir[i], initCamUp[i], initCamRight[i]);
    }
    // Create and save final result volume with these points
    SetupFinalVolume(initCamPoint, initCamDir, initCamUp, initCamRight, "MFirst");

    // Redefine variables for next resolution
    clusters = 9;
    numCamReq = 1;
    numRotations = 97;
    // loop over the camera solutions
    for(int i = 0; i < camSolution.size(); i++)
    {
        // if number of points in selected cluster are more than 150, then cluster again
        int numClusterPoints = clusteredBoundary[camSolution[i][0]].size();
        int currCamInd = camSolution[i][0];
        // setup input data with the points in that cluster
        QVector<QVector3D> clusterInVox, clusterInNorm;
        // New container to hold solution
        QVector<QVector<int>> camSolRes2;
        QVector<QVector<int>> clusterBoundaryRes2;
        QVector<QVector3D> clusterBackVox;
        // these vectors are dependent on the previous initialized visibilityCheck object
        // So, it sometimes gives error in highest resolution part, as the indices passed
        // might be corresponding to the previous iteration. So proper handling is needed
        QVector3D highCamDir, highCamUp, highCamRight;

        if(numClusterPoints > maxAllowedPoints)
        {
            clusterInVox.resize(numClusterPoints);
            clusterInNorm.resize(numClusterPoints);
            // Copies for the vtk Volume and vectorIndices, because we dont want to change the originals
            vtkSmartPointer<vtkStructuredPoints> volBoundaryNew = vtkSmartPointer<vtkStructuredPoints>::New();
            volBoundaryNew->DeepCopy(volBoundary);
            int extent[6];
            volBoundaryNew->GetExtent(extent);
            // VectorIndices
            QVector<unsigned long int> vectorIndicesNew(vectorIndices);

            for(int clust = 0; clust < numClusterPoints; clust++)
            {
                int currTempInd = clusteredBoundary[currCamInd][clust];
                clusterInVox[clust] = boundaryVoxels[currTempInd];
                clusterInNorm[clust] = boundaryNormals[currTempInd];
                // Get the x,y,z values of the point in the volume
                int xSelect = boundaryVoxels[currTempInd].x();
                int ySelect = boundaryVoxels[currTempInd].y();
                int zSelect = boundaryVoxels[currTempInd].z();
                // Get this voxel from the volume and set it to a value 3
                unsigned int* boundValNew = static_cast<unsigned int *>(volBoundaryNew->GetScalarPointer(xSelect, ySelect, zSelect));
                *boundValNew = 3;
                // Update the value in vectorIndices with current index 'clust', to match with the new created voxel and normal containers
                vectorIndicesNew[xSelect + (ySelect * extent[1]) + (zSelect * extent[1] * extent[3])] = clust;
            }
            // Get only the set of background points occupied by all the voxels in this cluster
            // The camera at each point can be considered to have the same orientation vectors as the cluster
            visibilityCheck.getBackPointsSet(clusterInVox, initCamDir[i], initCamUp[i], initCamRight[i], superBackVoxels, clusterBackVox);
            // Set the voxel avlue with 3 for this iteration
            // This will help in differentiating the current selected voxels from the rest of the boundary voxels
            volVal = 3;
            // Divide the solution cluster into further clusters
            superVoxels.clear();
            superNormals.clear();
            segmentLabels.clear();
            // ------------------- SLIC clustering------------------------------------------------
            // Call the function perform SLIC based segmentation
            volumeProcessor.slicSegmentation(clusterInVox,
                                             clusterInNorm,
                                             volBoundaryNew,
                                             clusters,
                                             superVoxels,
                                             superNormals,
                                             segmentLabels,
                                             volVal,
                                             vectorIndicesNew);
            // Setup a 'clusterBoundary' like container for indices of points in these clusters
            for(int clust = 0; clust < numClusterPoints; clust++)
                clusterBoundaryRes2[segmentLabels[clust]].append(clusteredBoundary[currCamInd][clust]);
            // Re-perform visibility checks with same object
            visibilityCheck = VisibilityCheck(superVoxels, superNormals, clusterBackVox, vehicleVoxels, numRotations);
            // Optimize
            totalTimer += optimizeData(visibilityCheck, numCamReq, clusterBackVox, camSolRes2);
            // Assertion to verify camsolution has only one entry
            if(camSolRes2.size() != 1)
                qDebug() << "New cam solution size not accurate! please heck mistake!";
            // First get the direction vectors of the camera placed at the selected cluster
            visibilityCheck.getCamVectors(camSolRes2[0], highCamDir, highCamUp, highCamRight);
        }
        else
        {
            camSolRes2.append(camSolution[i]);
            clusterBoundaryRes2 = clusteredBoundary;
            clusterBackVox = superBackVoxels;
            // initialize the cam vectors for this selected camera
            highCamDir = initCamDir[i];
            highCamUp = initCamUp[i];
            highCamRight = initCamRight[i];
        }

        // -------------------------Highest Resolution-------------------------------------------------------
        // After this step, we work directly at highest resolution without clustering
        int finalClustIndex = camSolRes2[0][0];
        int numFinalInPoints = clusterBoundaryRes2[camSolRes2[0][0]].size();
        clusterInVox.clear();
        clusterInNorm.clear();
        clusterInVox.resize(numFinalInPoints);
        clusterInNorm.resize(numFinalInPoints);
        // Setup the input data for visibility checks
        for(int clust = 0; clust < numFinalInPoints; clust++)
        {
            clusterInVox[clust] = boundaryVoxels[clusterBoundaryRes2[finalClustIndex][clust]];
            clusterInNorm[clust] = boundaryNormals[clusterBoundaryRes2[finalClustIndex][clust]];
        }
        // now get the occupied background points for this cluster
        QVector<QVector3D> highClusterBackVox;
        visibilityCheck.getBackPointsSet(clusterInVox, highCamDir, highCamUp, highCamRight, clusterBackVox, highClusterBackVox);
        // Visibility checks on new set of points at lowest resolution
        visibilityCheck = VisibilityCheck(clusterInVox, clusterInNorm, highClusterBackVox, vehicleVoxels, numRotations);
        // Final Optimization step
        QVector<QVector<int>> finalResCamSol;
        totalTimer += optimizeData(visibilityCheck, numCamReq, highClusterBackVox, finalResCamSol);
        // append the solution to the global solution container declared at the beginning
        finalCamPoint.append(boundaryVoxels[clusterBoundaryRes2[finalClustIndex][finalResCamSol[0][0]]]);
        // get the three camera vectors (direction, up and right)
        QVector3D solCamDir, solCamUp, solCamRight;
        visibilityCheck.getCamVectors(QVector<int>({finalResCamSol[0][0], finalResCamSol[0][1]}), solCamDir, solCamUp, solCamRight);
        // Append these vectors to the final solution container
        finalCamDir.append(solCamDir);
        finalCamUp.append(solCamUp);
        finalCamRight.append(solCamRight);
    }

    // Print the total time taken for optimization
    qDebug() << "Total time for optimization at three resolutions = " << totalTimer;
    // Call the function to setup final volume with camera FoVs
    SetupFinalVolume(finalCamPoint, finalCamDir, finalCamUp, finalCamRight, "MLast");
}

void VTKWidget::multiResolutionCombi()
{
    //-----------Initialization----------------------------------------------------------
    // Object for visibility checks
    VisibilityCheck visibilityCheck;
    // Timer to keep total time of optimization at diff. resolutions
    double totalTimer = 0.0;
    double totalClusterTime = 0.0;
    double totalVisCheckTime = 0.0;
    QElapsedTimer clusterTimer;
    QElapsedTimer visCheckTimer;
    mr_or_sr = 1;
    // Initial number of clusters and rotations.
    // Number of clusters
    // Bulldozer : 95 / 110
    // JCB : 95 / 125
    // Mining truck : 95 / 110
    // tractor : 75(32) / 90(64,128) / 110(256)
    // --------clusters for simulated vehicle models--------
    // car1 : 20(H) / 35,45,50(L) / 30,40,45,50(M) / 30,35 (S)
    // van  : 50,45(H) / 30(L)
    // truck: 50,45(H) / 45,20(L) / 20=50 all (M)
    // bus  : 35(H) / 50(L) / 25,20 (M) / 30,35 (S)
    int clusters = 85;//150;
    int pointLimit = 100; // limit = 200 for real data, 50 for sim data
    int numRotations = 97;//13;
    // counter to keep track of number of resolutions
    numReso = 1;
    // Number of cameras to be placed
    int numCamReq = 5;
    // vectors to specify input for clustering algorithm and optimization at each resolution
    // intialize it with the global boundary voxels and normal vectors
    QVector<QVector3D> clusterInVox(boundaryVoxels), clusterInNorm(boundaryNormals);
    // Copies for the vtk Volume and vectorIndices, because we dont want to change the originals
    vtkSmartPointer<vtkStructuredPoints> volBoundaryNew = vtkSmartPointer<vtkStructuredPoints>::New();
    volBoundaryNew->DeepCopy(volBoundary);
    int extent[6];
    volBoundaryNew->GetExtent(extent);
    // VectorIndices
    QVector<unsigned long int> vectorIndicesNew(vectorIndices);
    // Container to keep track of indices w.r.t. 'boundaryVoxels' of selected points
    QVector<int> selectedPointIndices(boundaryVoxels.size());
    for(int i = 0; i < boundaryVoxels.size(); i++)
        selectedPointIndices[i] = i;
    // Voxel value of selected points to mark in the volume, initially it must be 2 (same as in 'volBoundary')
    int volVal = 2;
    // Variables to keep the best solution in all iterations
    float bestCoverage = 0;
    // Vector to hold 3D points and directions of selected cameras from clustered points
    QVector<QVector3D> solCamPoint(numCamReq), solCamDir(numCamReq), solCamUp(numCamReq), solCamRight(numCamReq);
    // get the number of camera positions
    // For initialization, it is the number of boundary voxels
    int workNumCam = clusterInVox.size();
    // Keep clustering points and optimizing until the number of points reaches below a limit
    while(workNumCam > pointLimit)
    {
        // vectors to hold segmented voxels, normals, labels and solution indices
        QVector<QVector3D> superVoxels;
        QVector<QVector3D> superNormals;
        QVector<int> segmentLabels(workNumCam, -1);
        clusterTimer.start();
        // ------------------- SLIC clustering------------------------------------------------
        // Call the function perform SLIC based segmentation
        //volumeProcessor.slicSegmentation(clusterInVox,clusterInNorm,volBoundaryNew,clusters,superVoxels,superNormals,segmentLabels,volVal,vectorIndicesNew);
        volumeProcessor.newDslicSeg(clusterInVox,clusterInNorm,clusters,superVoxels,superNormals,segmentLabels);
        totalClusterTime += clusterTimer.elapsed();
        // Setup up points by the cluster for easier access later
        QVector<QVector<int>> clusteredBoundary;
        clusteredBoundary.resize(superVoxels.size()+1);

//        vtkSmartPointer<vtkStructuredPoints> segmentWorld = vtkSmartPointer<vtkStructuredPoints>::New();
//        int extent[6];
//        voxWorld->GetExtent(extent);
//        segmentWorld->SetExtent(extent);
//        segmentWorld->AllocateScalars(VTK_UNSIGNED_INT, 1);
//        for(int x = extent[0]; x <= extent[1]; x++)
//        {
//            for(int y = extent[2]; y <= extent[3]; y++)
//            {
//                for(int z = extent[4]; z <= extent[5]; z++)
//                {
//                    unsigned int* voxVal = static_cast<unsigned int*>(segmentWorld->GetScalarPointer(x,y,z));
//                    *voxVal = 0;
//                }
//            }
//        }
        for(int i = 0; i < segmentLabels.size(); i++)
        {
            clusteredBoundary[segmentLabels[i]].append(i);
            //QVector3D poin = boundaryVoxels[i];
            //unsigned int* currVox = static_cast<unsigned int*>(segmentWorld->GetScalarPointer(poin.x(),poin.y(),poin.z()));
            //*currVox = segmentLabels[i]+1;
        }
//        std::vector<std::string> splitFileName;
//        boost::split(splitFileName, fileNameVTK, [](char c){return c == '.';});
//        vtkNew<vtkGenericDataObjectWriter> writer;
//        writer->SetInputData(segmentWorld);
//        writer->SetFileName((splitFileName[0] + "_segments_" + std::to_string(numReso) + ".vtk").c_str());
//        writer->Write();

        // perform visibility checks with same object
        visCheckTimer.start();
        visibilityCheck = VisibilityCheck(superVoxels, superNormals, superBackVoxels, vehicleVoxels, numRotations);
        totalVisCheckTime += visCheckTimer.elapsed();
        // New container to hold solution
        QVector<QVector<int>> camSol;
        // Optimize
        totalTimer += optimizeData(visibilityCheck, numCamReq, superBackVoxels, camSol);

        // if the current solution is better then save it as the best solution
        if(backOccSOl > bestCoverage)
        {
            for(int i = 0; i < numCamReq; i++)
            {
                solCamPoint[i] = superVoxels[camSol[i][0]];
                visibilityCheck.getCamVectors(camSol[i],solCamDir[i],solCamUp[i],solCamRight[i]);
                bestCoverage = backOccSOl;
            }
        }

        // ----------------------------setup data for subsequent iterations------------------------------------
        // For the beginning ignore the selected orientation
        // increment the voxel value in volume
        volVal += 1;
        // Counter to keep track total selected points among 5 clusters
        int selPointCounter = 0;

        clusterInVox.clear();
        clusterInNorm.clear();
        selectedPointIndices.clear();
        for(int i = 0; i < camSol.size(); i++)
        {
            int currSelInd = camSol[i][0];
            // Loop over the points in the current selected cluster
            int numClusterPoints = clusteredBoundary[currSelInd].size();
            for(int j = 0; j < numClusterPoints; j++)
            {
                int currSelPointInd = clusteredBoundary[currSelInd][j];
                clusterInVox.append(boundaryVoxels[currSelPointInd]);
                clusterInNorm.append(boundaryNormals[currSelPointInd]);
                selectedPointIndices.append(currSelPointInd);
                // Get the x,y,z values of the point in the volume
                int xSelect = boundaryVoxels[currSelPointInd].x();
                int ySelect = boundaryVoxels[currSelPointInd].y();
                int zSelect = boundaryVoxels[currSelPointInd].z();
                // Get this voxel from the volume and set it to a value 3
                unsigned int* boundValNew = static_cast<unsigned int *>(volBoundaryNew->GetScalarPointer(xSelect, ySelect, zSelect));
                *boundValNew = 3;
                // Update the value in vectorIndices with current index 'clust', to match with the new created voxel and normal containers
                //vectorIndicesNew[xSelect + (ySelect * extent[1]) + (zSelect * extent[1] * extent[3])] = selPointCounter;
                // increment the selected Points counter
                selPointCounter += 1;
            }
        }
        // Incement the resolution counter
        numReso += 1;
        // Number of clusters for next iteration is divided by half
        //clusters = clusters/2;
        // Number of rotations for next iteration is doubled
        //numRotations = (numRotations - 1)*2 + 1;
        // Update the number of points to work with
        workNumCam = clusterInVox.size();
    }

    // -----------------------------------Highest resolution---------------------------
    // container to hold the solution
    QVector<QVector<int>> highCamSol;
    // visibility checks
    visCheckTimer.start();
    visibilityCheck = VisibilityCheck(clusterInVox, clusterInNorm, superBackVoxels, vehicleVoxels, numRotations);
    totalVisCheckTime += visCheckTimer.elapsed();
    // Optimize
    totalTimer += optimizeData(visibilityCheck, numCamReq, superBackVoxels, highCamSol);
    // Loop over the solution adn get the camera points and associated direction vectors
    // Note: It happens that the overall coverage is decreasing at the highest resolution
    // to keep a track save this pone separately for confirmation/debuggin
    QVector<QVector3D> lastCamPoint(numCamReq), lastCamDir(numCamReq), lastCamUp(numCamReq), lastCamRight(numCamReq);
    if(backOccSOl >= bestCoverage)
    {
        for(int i = 0; i < numCamReq; i++)
        {
            solCamPoint[i] = clusterInVox[highCamSol[i][0]];
            visibilityCheck.getCamVectors(highCamSol[i],solCamDir[i],solCamUp[i],solCamRight[i]);
            bestCoverage = backOccSOl;
        }
    }
    else
    {
        // If it's worse than previous best, then print both solutions
        for(int i = 0; i < numCamReq; i++)
        {
            qDebug() << " Camera " << i+1;
            qDebug() << "Best Position : " << solCamPoint[i] << " orientation : " << solCamDir[i];
            visibilityCheck.getCamVectors(highCamSol[i], lastCamDir[i], lastCamUp[i], lastCamRight[i]);
            lastCamPoint[i] = boundaryVoxels[selectedPointIndices[highCamSol[i][0]]];
            qDebug() << "Last Position : " << lastCamPoint[i] << " orientation : " << lastCamDir[i];
        }
    }
    // Print the total time taken for optimization
    qDebug() << "Total time for optimization at " << numReso << " resolutions = " << totalTimer << " ms";
    qDebug() << "Total clustering time = " << totalClusterTime << "ms";
    qDebug() << "Total visibility checks time = " << totalVisCheckTime << "ms";
    qDebug() << "Final Coverage  = " << bestCoverage << "%";
    // Call the function to setup final volume with camera FoVs
    SetupFinalVolume(solCamPoint, solCamDir, solCamUp, solCamRight, "MCBest");
    // when final result is worse, save the last resolution configuration just comparison
    if(bestCoverage > backOccSOl)
        SetupFinalVolume(lastCamPoint, lastCamDir, lastCamUp, lastCamRight, "MCLast");
}

void VTKWidget::combiMRLoop()
{
    std::vector<float> coverageList(10);
    std::vector<int> timeList(10);
    for(int itaro = 0; itaro < 10; itaro++)
    {
        qDebug() << "************************************Test # " << itaro+1 << " *******************************";
        //-----------Initialization----------------------------------------------------------
        // Object for visibility checks
        VisibilityCheck visibilityCheck;
        // Timer to keep total time of optimization at diff. resolutions
        double totalTimer = 0.0;
        double totalClusterTime = 0.0;
        double totalVisCheckTime = 0.0;
        QElapsedTimer clusterTimer;
        QElapsedTimer visCheckTimer;
        mr_or_sr = 1;
        // Initial number of clusters and rotations.
        // Number of clusters
        // Bulldozer : 95
        // JCB : 95 / 125
        // Mining truck : 95 (32) / 110
        // tractor : 75(32) / 90(64,128)(rwls-32) / 110(256)
        // --------clusters for simulated vehicle models--------
        // car1 : 20(H) / 35,45,50(L) / 30,40,45,50(M) / 30,35 (S)
        // van  : 50,45(H) / 30(L)
        // truck: 50,45,25(H) / 45,20(L) / 20-50 all (M)
        // bus  : 35(H) / 50(L) / 25,20 (M) / 30,35 (S)
        int clusters = 30;//150;
        int pointLimit = 50;
        int numRotations = 97;//13;
        // counter to keep track of number of resolutions
        numReso = 1;
        // Number of cameras to be placed
        int numCamReq = 5;
        // vectors to specify input for clustering algorithm and optimization at each resolution
        // intialize it with the global boundary voxels and normal vectors
        QVector<QVector3D> clusterInVox(boundaryVoxels), clusterInNorm(boundaryNormals);
        // Copies for the vtk Volume and vectorIndices, because we dont want to change the originals
        vtkSmartPointer<vtkStructuredPoints> volBoundaryNew = vtkSmartPointer<vtkStructuredPoints>::New();
        volBoundaryNew->DeepCopy(volBoundary);
        int extent[6];
        volBoundaryNew->GetExtent(extent);
        // VectorIndices
        QVector<unsigned long int> vectorIndicesNew(vectorIndices);
        // Container to keep track of indices w.r.t. 'boundaryVoxels' of selected points
        QVector<int> selectedPointIndices(boundaryVoxels.size());
        for(int i = 0; i < boundaryVoxels.size(); i++)
            selectedPointIndices[i] = i;
        // Voxel value of selected points to mark in the volume, initially it must be 2 (same as in 'volBoundary')
        int volVal = 2;
        // Variables to keep the best solution in all iterations
        float bestCoverage = 0;
        // Vector to hold 3D points and directions of selected cameras from clustered points
        QVector<QVector3D> solCamPoint(numCamReq), solCamDir(numCamReq), solCamUp(numCamReq), solCamRight(numCamReq);
        // get the number of camera positions
        // For initialization, it is the number of boundary voxels
        int workNumCam = clusterInVox.size();
        // Keep clustering points and optimizing until the number of points reaches below a limit
        while(workNumCam > pointLimit)
        {
            // vectors to hold segmented voxels, normals, labels and solution indices
            QVector<QVector3D> superVoxels;
            QVector<QVector3D> superNormals;
            QVector<int> segmentLabels(workNumCam, -1);
            clusterTimer.start();
            // ------------------- SLIC clustering------------------------------------------------
            // Call the function perform SLIC based segmentation
            //volumeProcessor.slicSegmentation(clusterInVox,clusterInNorm,volBoundaryNew,clusters,superVoxels,superNormals,segmentLabels,volVal,vectorIndicesNew);
            volumeProcessor.newDslicSeg(clusterInVox,clusterInNorm,clusters,superVoxels,superNormals,segmentLabels);
            totalClusterTime += clusterTimer.elapsed();
            // Setup up points by the cluster for easier access later
            QVector<QVector<int>> clusteredBoundary;
            clusteredBoundary.resize(superVoxels.size()+1);
            for(int i = 0; i < segmentLabels.size(); i++)
                clusteredBoundary[segmentLabels[i]].append(i);

            // perform visibility checks with same object
            visCheckTimer.start();
            visibilityCheck = VisibilityCheck(superVoxels, superNormals, superBackVoxels, vehicleVoxels, numRotations);
            totalVisCheckTime += visCheckTimer.elapsed();
            qDebug() << "visibility checks time = " << visCheckTimer.elapsed() << "ms";
            // New container to hold solution
            QVector<QVector<int>> camSol;
            // Optimize
            totalTimer += optimizeData(visibilityCheck, numCamReq, superBackVoxels, camSol);
            // if the current solution is better then save it as the best solution
            if(backOccSOl > bestCoverage)
            {
                for(int i = 0; i < numCamReq; i++)
                {
                    solCamPoint[i] = superVoxels[camSol[i][0]];
                    visibilityCheck.getCamVectors(camSol[i],solCamDir[i],solCamUp[i],solCamRight[i]);
                    bestCoverage = backOccSOl;
                }
            }
            // ----------------------------setup data for subsequent iterations------------------------------------
            // For the beginning ignore the selected orientation
            // increment the voxel value in volume
            volVal += 1;
            // Counter to keep track total selected points among 535 clusters
            int selPointCounter = 0;

            clusterInVox.clear();
            clusterInNorm.clear();
            selectedPointIndices.clear();
            for(int i = 0; i < camSol.size(); i++)
            {
                int currSelInd = camSol[i][0];
                // Loop over the points in the current selected cluster
                int numClusterPoints = clusteredBoundary[currSelInd].size();
                for(int j = 0; j < numClusterPoints; j++)
                {
                    int currSelPointInd = clusteredBoundary[currSelInd][j];
                    clusterInVox.append(boundaryVoxels[currSelPointInd]);
                    clusterInNorm.append(boundaryNormals[currSelPointInd]);
                    selectedPointIndices.append(currSelPointInd);
                    // Get the x,y,z values of the point in the volume
                    int xSelect = boundaryVoxels[currSelPointInd].x();
                    int ySelect = boundaryVoxels[currSelPointInd].y();
                    int zSelect = boundaryVoxels[currSelPointInd].z();
                    // Get this voxel from the volume and set it to a value 3
                    unsigned int* boundValNew = static_cast<unsigned int *>(volBoundaryNew->GetScalarPointer(xSelect, ySelect, zSelect));
                    *boundValNew = 3;
                    // Update the value in vectorIndices with current index 'clust', to match with the new created voxel and normal containers
                    //vectorIndicesNew[xSelect + (ySelect * extent[1]) + (zSelect * extent[1] * extent[3])] = selPointCounter;
                    // increment the selected Points counter
                    selPointCounter += 1;
                }
            }
            // Incement the resolution counter
            numReso += 1;
            // Update the number of points to work with
            workNumCam = clusterInVox.size();
        }

        // -----------------------------------Highest resolution---------------------------
        // container to hold the solution
        QVector<QVector<int>> highCamSol;
        // visibility checks
        visCheckTimer.start();
        visibilityCheck = VisibilityCheck(clusterInVox, clusterInNorm, superBackVoxels, vehicleVoxels, numRotations);
        totalVisCheckTime += visCheckTimer.elapsed();
        // Optimize
        totalTimer += optimizeData(visibilityCheck, numCamReq, superBackVoxels, highCamSol);
        // Loop over the solution adn get the camera points and as35sociated direction vectors
        // Note: It happens that the overall coverage is decreasing at the highest resolution
        // to keep a track save this pone separately for confirmation/debuggin
        QVector<QVector3D> lastCamPoint(numCamReq), lastCamDir(numCamReq), lastCamUp(numCamReq), lastCamRight(numCamReq);
        if(backOccSOl >= bestCoverage)
        {
            for(int i = 0; i < numCamReq; i++)
            {
                solCamPoint[i] = clusterInVox[highCamSol[i][0]];
                visibilityCheck.getCamVectors(highCamSol[i],solCamDir[i],solCamUp[i],solCamRight[i]);
                bestCoverage = backOccSOl;
            }
        }
        else
        {
            // If it's worse than previous best, then print both solutions
            for(int i = 0; i < numCamReq; i++)
            {
                qDebug() << " Camera " << i+1;
                qDebug() << "Best Position : " << solCamPoint[i] << " orientation : " << solCamDir[i];
                visibilityCheck.getCamVectors(highCamSol[i], lastCamDir[i], lastCamUp[i], lastCamRight[i]);
                lastCamPoint[i] = boundaryVoxels[selectedPointIndices[highCamSol[i][0]]];
                qDebug() << "Last Position : " << lastCamPoint[i] << " orientation : " << lastCamDir[i];
            }
        }
        // Print the total time taken for optimization
        qDebug() << "Total time for optimization at " << numReso << " resolutions = " << totalTimer << " ms";
        qDebug() << "Total clustering time = " << totalClusterTime << "ms";
        qDebug() << "Total visibility checks time = " << totalVisCheckTime << "ms";
        qDebug() << "Final Coverage  = " << bestCoverage << "%";
        // when final result is worse, save the last resolution configuration just comparison
        if(bestCoverage > backOccSOl)
            SetupFinalVolume(lastCamPoint, lastCamDir, lastCamUp, lastCamRight, "MCLast");
        else
            SetupFinalVolume(solCamPoint, solCamDir, solCamUp, solCamRight, "MCBest");

        // Fill the final values in the containers
        coverageList[itaro] = bestCoverage;
        timeList[itaro] = totalTimer + totalClusterTime + totalVisCheckTime;
    }
    qDebug() << "******************Final result Set****************";
    qDebug() << coverageList;
    qDebug() << timeList;
}

void VTKWidget::MRClusterTest()
{
    //-----------Initialization----------------------------------------------------------
    // Object for visibility checks
    VisibilityCheck visibilityCheck;
    // Timer to keep total time of optimization at diff. resolutions
    double totalTimer = 0.0;
    double totalClusterTime = 0.0;
    double totalVisCheckTime = 0.0;
    QElapsedTimer clusterTimer;
    QElapsedTimer visCheckTimer;
    // Initial number of clusters and rotations. For subsequent resolutions,
    // clusters can be halved, while rotations can be doubled
    QVector<int> clusters;
    int clustLow = 90, clustHigh = 106;
    int stepIncrement = 1;
    for(int i = clustLow; i <= clustHigh; i+=stepIncrement)
    {
        clusters.append(i);
    }
    // containers to store solutions and times for each cluster size
    QVector<float> clustObjectives(clusters.size());
    QVector<int> clustTimeOpt(clusters.size()), clustTimeClust(clusters.size()), clustTimeVis(clusters.size()), highResPoints(clusters.size());
    //int clusters = 60;//150;
    int pointLimit = 50;
    int numRotations = 97;//13;
    // counter to keep track of number of resolutions
    int numReso = 1;
    // Number of cameras to be placed
    int numCamReq = 5;
    // Run the test may times for different number of clusters
    for(int currClust = 0; currClust < clusters.size(); currClust++)
    {
        // vectors to specify input for clustering algorithm and optimization at each resolution
        // intialize it with the global boundary voxels and normal vectors
        QVector<QVector3D> clusterInVox(boundaryVoxels), clusterInNorm(boundaryNormals);
        // Copies for the vtk Volume and vectorIndices, because we dont want to change the originals
        vtkSmartPointer<vtkStructuredPoints> volBoundaryNew = vtkSmartPointer<vtkStructuredPoints>::New();
        volBoundaryNew->DeepCopy(volBoundary);
        int extent[6];
        volBoundaryNew->GetExtent(extent);
        // VectorIndices
        QVector<unsigned long int> vectorIndicesNew(vectorIndices);
        // Container to keep track of indices w.r.t. 'boundaryVoxels' of selected points
        QVector<int> selectedPointIndices(boundaryVoxels.size());
        for(int i = 0; i < boundaryVoxels.size(); i++)
            selectedPointIndices[i] = i;
        // Voxel value of selected points to mark in the volume, initially it must be 2 (same as in 'volBoundary')
        int volVal = 2;
        // Vector to hold 3D points and directions of selected cameras from clustered points
        float bestCoverage = 0;
        QVector<QVector3D> solCamPoint(numCamReq), solCamDir(numCamReq), solCamUp(numCamReq), solCamRight(numCamReq);
        // get the number of camera positions
        // For initialization, it is the number of boundary voxels
        int workNumCam = clusterInVox.size();
        // Keep clustering points and optimizing until the number of points reaches below a limit
        while(workNumCam > pointLimit)
        {
            // vectors to hold segmented voxels, normals, labels and solution indices
            QVector<QVector3D> superVoxels;
            QVector<QVector3D> superNormals;
            QVector<int> segmentLabels(workNumCam, -1);
            clusterTimer.start();
            // ------------------- SLIC clustering------------------------------------------------
            // Call the function perform SLIC based segmentation
//            volumeProcessor.slicSegmentation(clusterInVox,
//                                             clusterInNorm,
//                                             volBoundaryNew,
//                                             clusters[currClust],
//                                             superVoxels,
//                                             superNormals,
//                                             segmentLabels,
//                                             volVal,
//                                             vectorIndicesNew);
            volumeProcessor.newDslicSeg(clusterInVox,clusterInNorm,clusters[currClust],superVoxels,superNormals,segmentLabels);
            totalClusterTime += clusterTimer.elapsed();
            // Setup up points by the cluster for easier access later
            QVector<QVector<int>> clusteredBoundary;
            clusteredBoundary.resize(superVoxels.size()+1);
            for(int i = 0; i < segmentLabels.size(); i++)
            {
                clusteredBoundary[segmentLabels[i]].append(selectedPointIndices[i]);
            }
            // perform visibility checks with same object
            visCheckTimer.start();
            visibilityCheck = VisibilityCheck(superVoxels, superNormals, superBackVoxels, vehicleVoxels, numRotations);
            totalVisCheckTime += visCheckTimer.elapsed();
            // New container to hold solution
            QVector<QVector<int>> camSol;
            // Optimize
            totalTimer += optimizeData(visibilityCheck, numCamReq, superBackVoxels, camSol);

            // if the current solution is better then save it as the best solution
            if(backOccSOl > bestCoverage)
            {
                for(int i = 0; i < numCamReq; i++)
                {
                    solCamPoint[i] = superVoxels[camSol[i][0]];
                    visibilityCheck.getCamVectors(camSol[i],solCamDir[i],solCamUp[i],solCamRight[i]);
                    bestCoverage = backOccSOl;
                }
            }

            // ----------------------------setup data for subsequent iterations------------------------------------
            // For the beginning ignore the selected orientation
            // increment the voxel value in volume
            volVal += 1;
            // Counter to keep track total selected points among 5 clusters
            int selPointCounter = 0;

            clusterInVox.clear();
            clusterInNorm.clear();
            selectedPointIndices.clear();
            for(int i = 0; i < camSol.size(); i++)
            {
                int currSelInd = camSol[i][0];
                // Loop over the points in the current selected cluster
                int numClusterPoints = clusteredBoundary[currSelInd].size();
                for(int j = 0; j < numClusterPoints; j++)
                {
                    int currSelPointInd = clusteredBoundary[currSelInd][j];
                    clusterInVox.append(boundaryVoxels[currSelPointInd]);
                    clusterInNorm.append(boundaryNormals[currSelPointInd]);
                    selectedPointIndices.append(currSelPointInd);
                    // Get the x,y,z values of the point in the volume
                    int xSelect = boundaryVoxels[currSelPointInd].x();
                    int ySelect = boundaryVoxels[currSelPointInd].y();
                    int zSelect = boundaryVoxels[currSelPointInd].z();
                    // Get this voxel from the volume and set it to a value 3
                    unsigned int* boundValNew = static_cast<unsigned int *>(volBoundaryNew->GetScalarPointer(xSelect, ySelect, zSelect));
                    *boundValNew = 3;
                    // Update the value in vectorIndices with current index 'clust', to match with the new created voxel and normal containers
                    vectorIndicesNew[xSelect + (ySelect * extent[1]) + (zSelect * extent[1] * extent[3])] = selPointCounter;
                    // increment the selected Points counter
                    selPointCounter += 1;
                }
            }
            // Incement the resolution counter
            numReso += 1;
            // Number of clusters for next iteration is divided by half
            //clusters = clusters/2;
            // Number of rotations for next iteration is doubled
            //numRotations = (numRotations - 1)*2 + 1;
            // Update the number of points to work with
            workNumCam = clusterInVox.size();
        }

        // -----------------------------------Highest resolution---------------------------
        // container to hold the solution
        QVector<QVector<int>> highCamSol;
        // visibility checks
        visCheckTimer.start();
        visibilityCheck = VisibilityCheck(clusterInVox, clusterInNorm, superBackVoxels, vehicleVoxels, numRotations);
        totalVisCheckTime += visCheckTimer.elapsed();
        // Optimize
        totalTimer += optimizeData(visibilityCheck, numCamReq, superBackVoxels, highCamSol);
        // Vector to hold 3D points and directions of selected cameras from clustered points
        QVector<QVector3D> lastCamPoint(numCamReq), lastCamDir(numCamReq), lastCamUp(numCamReq), lastCamRight(numCamReq);
        if(backOccSOl >= bestCoverage)
        {
            for(int i = 0; i < numCamReq; i++)
            {
                solCamPoint[i] = clusterInVox[highCamSol[i][0]];
                visibilityCheck.getCamVectors(highCamSol[i],solCamDir[i],solCamUp[i],solCamRight[i]);
                bestCoverage = backOccSOl;
            }
        }
        else
        {
            // If it's worse than previous best, then print both solutions
            for(int i = 0; i < numCamReq; i++)
            {
                //qDebug() << " Camera " << i+1;
                //qDebug() << "Best Position : " << solCamPoint[i] << " orientation : " << solCamDir[i];
                visibilityCheck.getCamVectors(highCamSol[i], lastCamDir[i], lastCamUp[i], lastCamRight[i]);
                lastCamPoint[i] = boundaryVoxels[selectedPointIndices[highCamSol[i][0]]];
                //qDebug() << "Last Position : " << lastCamPoint[i] << " orientation : " << lastCamDir[i];
            }
        }

        // Print the total time taken for optimization
        qDebug() << "Total time for optimization at " << numReso << " resolutions = " << totalTimer << " ms";
        qDebug() << "Total clustering time = " << totalClusterTime << "ms";
        qDebug() << "Total visibility checks time = " << totalVisCheckTime << "ms";

        // setup output containers
        clustObjectives[currClust] = bestCoverage;
        clustTimeOpt[currClust] = totalTimer;
        clustTimeClust[currClust] = totalClusterTime;
        clustTimeVis[currClust] = totalVisCheckTime;
        highResPoints[currClust] = visChecksPoints;

        // Print coverage value for each cluster
        qDebug() << "*****************************************************************";
        qDebug() << "Clusters = " << clusters[currClust] << " , coverage = " << bestCoverage;
        qDebug() << "*****************************************************************";
    }
    // Call the function to setup final volume with camera FoVs
    //SetupFinalVolume(solCamPoint, solCamDir, solCamUp, solCamRight, "MCLast");

    // calculate mean and variance
    double sum = std::accumulate(clustObjectives.begin(), clustObjectives.end(), 0.0);
    double mean = sum / clustObjectives.size();
    double sq_sum = std::inner_product(clustObjectives.begin(), clustObjectives.end(), clustObjectives.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / clustObjectives.size() - mean * mean);
    qDebug() << "mean = " << mean;
    qDebug() << "variance = " << stdev;

    // sort by accuracies
    qDebug() << "Clusters :\n" << clusters;
    qDebug() << "Objective values :\n" << clustObjectives;
    qDebug() << "Optimization times :\n" << clustTimeOpt;
    qDebug() << "Clustering times :\n" << clustTimeClust;
    qDebug() << "Visibility Checks times :\n" << clustTimeVis;
    qDebug() << "Points in highest resolution :\n" << highResPoints;
}

void VTKWidget::optimizePressed()
{
    // This function does optimization without any prior clustering. It is assumed that
    // the number of input data points setup is within the limit of 150 boundary voxels
    // Object for visibility checks
    VisibilityCheck visibilityCheck;
    // Number of camera rotations at each camera point
    int numRotations = 97;
    // Number of cameras required
    int numCamReq = 5;
    // Timer to keep total time of optimization at diff. resolutions
    double totalTimer = 0.0;
    // Timer to track time taken for visibility checks
    double visibilityTime = 0.0;
    QElapsedTimer visibilityTimer;
    mr_or_sr = 0;
    // perform visibility checks with same object

    visibilityTimer.start();
    visibilityCheck = VisibilityCheck(boundaryVoxels, boundaryNormals, superBackVoxels, vehicleVoxels, numRotations);
    visibilityTime = visibilityTimer.elapsed();
    qDebug() << "visibility checks time = " << visibilityTime << "ms";
    // New container to hold solution
    QVector<QVector<int>> camSol;
    // Optimize
    totalTimer += optimizeData(visibilityCheck, numCamReq, superBackVoxels, camSol);
    // Vector to hold 3D points and directions of selected cameras from clustered points
    QVector<QVector3D> solCamPoint, solCamDir, solCamUp, solCamRight;
    // Loop over the solution adn get the camera points and associated direction vectors
    for(int i = 0; i < camSol.size(); i++)
    {
        solCamPoint.append(boundaryVoxels[camSol[i][0]]);
        // Get the direction vectors
        QVector3D tempCamDir, tempCamUp, tempCamRight;
        visibilityCheck.getCamVectors(camSol[i], tempCamDir, tempCamUp, tempCamRight);
        solCamDir.append(tempCamDir);
        solCamUp.append(tempCamUp);
        solCamRight.append(tempCamRight);
    }
    qDebug() << "Total time for Single Resolution optimization  = " << totalTimer << " ms";
    qDebug() << "Total time for visibility checks = " << visibilityTime << " ms";
    // Call the function to setup final volume with camera FoVs
    SetupFinalVolume(solCamPoint, solCamDir, solCamUp, solCamRight, "S");
}

int VTKWidget::optimizeData(VisibilityCheck &visibilityCheck,
                             int numCamReq,
                            QVector<QVector3D> &backVoxSet,
                             QVector<QVector<int>> &camSolution)
{
    //-----------------------------------Optimization-----------------------------------------------------
    // get the 2D matrix and 1D indices
    boost::numeric::ublas::compressed_matrix<bool> gMatrix;
    QVector<QVector<int>> index1D;
    visibilityCheck.getMatrixAnd1D(gMatrix, index1D);
    qDebug() << "Camera poses selected after self-occlusion checks = " << index1D.size();
    // Create containers to store optimized solution
    QVector<int> backSolution;
    // Timer to calculate optimization time
    QElapsedTimer optimizationTimer;
    int timeMS;
    // Greedy, Metropolis-sampling and BIP Optimization
    if(optimizeChoice == 0)
    {
        // Object for heuristics algorithms
        Heuristics heuristics;
        qDebug() << "Starting Greedy Optimization!!!";
        optimizationTimer.start();
        heuristics.greedyAlgo(backVoxSet, numCamReq, visibilityCheck, camSolution, backSolution);
        timeMS = optimizationTimer.elapsed();
        qDebug() << "Optimization time (Greedy) = " << timeMS << "ms = " << timeMS/1000.0 << "s";
    }
    else if(optimizeChoice == 1)
    {
        // Run this algorithm 10 times and calculate best and average scores
        QVector<int> timesSet(10);
        QVector<float> covSet(10);
        QVector<QVector<int>> backOccSet(10);
        QVector<QVector<QVector<int>>> camSolSet(10);
        for(int i = 0; i < 10; i++)
        {
            // Object for heuristics algorithms
            Heuristics heuristics;
            int tempTime;
            QVector<int> tempBackOccSet;
            QVector<QVector<int>> tempCamSol;
            qDebug() << "Starting Metropolis/SA Optimization!!!";
            optimizationTimer.start();
            heuristics.metroSA(backVoxSet.size(), index1D.size(), numCamReq, index1D, gMatrix, tempCamSol, tempBackOccSet);
            tempTime = optimizationTimer.elapsed();

            backOccSet[i] = tempBackOccSet;
            camSolSet[i] = tempCamSol;
            covSet[i] = (float)tempBackOccSet.size()/backVoxSet.size()*100.0;
            timesSet[i] = tempTime;
            qDebug() << "Optimization time (metropolis/SA) = " << tempTime << "ms";
        }
        auto maxPosIter = std::max_element(covSet.begin(), covSet.end());
        int maxPos = std::distance(covSet.begin(), maxPosIter);
        qDebug() << "*********************************";
        qDebug() << "best coverage = " << covSet[maxPos];
        qDebug() << covSet;
        backSolution = backOccSet[maxPos];
        camSolution = camSolSet[maxPos];
        timeMS = timesSet[0];
    }
    else if(optimizeChoice == 2)
    {
        // Object for OR optimizer
        MIPOptimizer mipOptimizer;
        qDebug() << "Starting OR-tools Optimization!!!";
        optimizationTimer.start();
        mipOptimizer.singleObj(backVoxSet.size(), index1D.size(), numCamReq, index1D, gMatrix, camSolution, backSolution);
        //mipOptimizer.remoteOptimization(backVoxSet.size(), index1D.size(), numCamReq, index1D, gMatrix, camSolution, backSolution);
        timeMS = optimizationTimer.elapsed();
        qDebug() << "Total optimization time (MIP optimizer) = " << timeMS << "ms = " << timeMS/1000.0 << "s";
    }
    else if(optimizeChoice == 3)
    {
        // Run this algorithm 10 times and calculate best and average scores
        QVector<int> timesSet(10);
        QVector<float> covSet(10);
        QVector<QVector<int>> backOccSet(10);
        QVector<QVector<QVector<int>>> camSolSet(10);
        for(int i = 0; i < 10; i++)
        {
            Heuristics heuristics;
            int tempTime;
            QVector<int> tempBackOccSet;
            QVector<QVector<int>> tempCamSol;
            qDebug() << "Starting rwls algorithm!!!";
            optimizationTimer.start();
            heuristics.rwls(numCamReq, index1D, gMatrix, tempCamSol, tempBackOccSet, fileNameVTK, mr_or_sr, numReso);
            tempTime = optimizationTimer.elapsed();

            backOccSet[i] = tempBackOccSet;
            camSolSet[i] = tempCamSol;
            covSet[i] = (float)tempBackOccSet.size()/backVoxSet.size()*100.0;
            timesSet[i] = tempTime;
            qDebug() << "time for optimization (rwls) = " << tempTime << "ms";
        }
        auto maxPosIter = std::max_element(covSet.begin(), covSet.end());
        int maxPos = std::distance(covSet.begin(), maxPosIter);
        qDebug() << "*********************************";
        qDebug() << "best coverage = " << covSet[maxPos];
        qDebug() << covSet;
        backSolution = backOccSet[maxPos];
        camSolution = camSolSet[maxPos];
        timeMS = timesSet[0];
        //qDebug() << "Total optimization time (rwls) = " << timeMS << "ms = " << timeMS/1000.0 << "s";
    }
    else if(optimizeChoice == 4)
    {
        // Run this algorithm 10 times and calculate best and average scores
        QVector<int> timesSet(10);
        QVector<float> covSet(10);
        QVector<QVector<int>> backOccSet(10);
        QVector<QVector<QVector<int>>> camSolSet(10);
        for(int i = 0; i < 10; i++)
        {
            // Object for heuristics algorithms
            Heuristics heuristics;
            int tempTime;
            QVector<int> tempBackOccSet;
            QVector<QVector<int>> tempCamSol;
            qDebug() << "Starting LH-RPSO Optimization!!!";
            optimizationTimer.start();
            heuristics.LHRPSO(numCamReq, index1D, gMatrix, tempCamSol, tempBackOccSet);
            tempTime = optimizationTimer.elapsed();
            backOccSet[i] = tempBackOccSet;
            camSolSet[i] = tempCamSol;
            covSet[i] = (float)tempBackOccSet.size()/backVoxSet.size()*100.0;
            timesSet[i] = tempTime;
            qDebug() << "Optimization time (LH-RPSO) = " << tempTime << "ms";
        }
        auto maxPosIter = std::max_element(covSet.begin(), covSet.end());
        int maxPos = std::distance(covSet.begin(), maxPosIter);
        qDebug() << "*********************************";
        qDebug() << "best coverage = " << covSet[maxPos];
        qDebug() << covSet;
        backSolution = backOccSet[maxPos];
        camSolution = camSolSet[maxPos];
        timeMS = timesSet[0];
    }
    qDebug() << "Objective value = " << backSolution.size() << " / " << backVoxSet.size();
    backOccSOl = ((float)backSolution.size()/backVoxSet.size())*100;
    visChecksPoints = index1D.size();
    return timeMS;
}

void VTKWidget::SetupFinalVolume(QVector<QVector3D> &camSolution,
                                 QVector<QVector3D> &camDirs,
                                 QVector<QVector3D> &camUps,
                                 QVector<QVector3D> &camRights,
                                 std::string nameLabel)
{
    //-----------------------------Verify background point occupation for individual camera-------------

    // Note: It is best to verify against the complete set of background points.
    // Update: The note above is not really true, because there are
    // 2million contorl points and they take too much space in visibility checks.
    // Update2: We can use complete set as background points for bowl surface are only ~60k

    QVector<QVector<int>> backSolutionVerified;
    // Container for solution camera frustums
    QVector<QVector<QVector3D>> camFrustums;
    // Get solution occupation
    VisibilityCheck visibilityCheck;
    visibilityCheck.getCamCoverage(camSolution, camDirs, camUps, camRights, backgroundVoxels, backSolutionVerified);
    // First get the camera frustums
    visibilityCheck.getCamFrustum(camSolution, camDirs, camUps, camRights, camFrustums);
    vtkSmartPointer<vtkStructuredPoints> solutionWorldVerified = vtkSmartPointer<vtkStructuredPoints>::New();
    solutionWorldVerified->DeepCopy(voxWorld);
    /* Volume to store only the covered Background points.
     * This volume can be then displayed over the solution volume as points
     */
    vtkSmartPointer<vtkStructuredPoints> solutionBackPoints = vtkSmartPointer<vtkStructuredPoints>::New();
    int extent[6];
    voxWorld->GetExtent(extent);
    solutionBackPoints->SetExtent(extent);
    solutionBackPoints->AllocateScalars(VTK_UNSIGNED_INT, 1);
    /* This method can support only 5 cameras because in 'voxWorld' only values from 3-7 are free
     * So vehicle voxel values must be changed in 'voxWorld' before using this method
     * one solution can be to push boundary voxel values further depending on no. of cameras more than 5
     */
    int verifiedVoxVal = 3;
    // Container to mark the occupied points
    QVector<bool> backgroundCoverageVerify(backgroundVoxels.size(), 0);

    for(int i = 0; i < camSolution.size(); i++)
    {
        // Get number of voxels in boundary cluster with index camSolution[i]
        //int boundClusterSize = clusteredBoundary[camSolution[i][0]].size();
        //for(int j = 0; j < boundClusterSize; j++)
        //{
            // Get voxel at each index and set value to 'startVal'
            //QVector3D currVox = boundaryVoxels[clusteredBoundary[camSolution[i][0]][j]];
        QVector3D currVox = camSolution[i];
        unsigned int* solVerified = static_cast<unsigned int*>(solutionWorldVerified->GetScalarPointer(currVox.x(), currVox.y(), currVox.z()));
        *solVerified = verifiedVoxVal;
        //}
        // repeat for background voxels
        int numSolClusters = backSolutionVerified[i].size();
        //totalOccupied += numSolClusters;
        for(int j = 0; j < numSolClusters; j++)
        {
            /* For bowl surface points, we are checking with complete backgroundVoxels set
             * So, here, we don't use the clustered background data points,
             * but work directly with backgroundVoxels
             */
            if(bowlSurface)
            {
                // Get the background voxel and set it to 'startVal'
                QVector3D currVox = backgroundVoxels[backSolutionVerified[i][j]];
                unsigned int* solVerified = static_cast<unsigned int*>(solutionWorldVerified->GetScalarPointer(currVox.x(), currVox.y(), currVox.z()));
                unsigned int* solCovered = static_cast<unsigned int*>(solutionBackPoints->GetScalarPointer(currVox.x(), currVox.y(), currVox.z()));
                *solVerified = verifiedVoxVal;
                *solCovered = verifiedVoxVal;
                // Mark this voxel in the verification container
                backgroundCoverageVerify[backSolutionVerified[i][j]] = 1;
                // Increment totalOccupied for final accuracy
                //totalOccupied++;
            }
            // if not, we work with clusteredBround points dataset because numbe rof background points is too much
            else
            {
                // Get number of voxels in each cluster
                int clusterSize = clusteredBackground[backSolutionVerified[i][j]].size();
                // Add cluster size into 'totalOccupied' counter
                //totalOccupied += clusterSize;
                for(int k = 0; k < clusterSize; k++)
                {
                    // Get the background voxel and set it to 'startVal'
                    QVector3D currVox = backgroundVoxels[clusteredBackground[backSolutionVerified[i][j]][k]];
                    unsigned int* solVerified = static_cast<unsigned int*>(solutionWorldVerified->GetScalarPointer(currVox.x(), currVox.y(), currVox.z()));
                    unsigned int* solCovered = static_cast<unsigned int*>(solutionBackPoints->GetScalarPointer(currVox.x(), currVox.y(), currVox.z()));
                    *solVerified = verifiedVoxVal;
                    *solCovered = verifiedVoxVal;
                    // Mark this voxel in the verification container
                    backgroundCoverageVerify[clusteredBackground[backSolutionVerified[i][j]][k]] = 1;
                }
            }
        }
        // Increment 'verifiedVoxVal' so that points for different cameras can be distinguished
        verifiedVoxVal++;
    }
    // -----------------------------------Draw Lines for camera view frustum----------------------------------

    // Resize the image extent in y-direction to allow full frustum marking
    int extentNew[6];

    // extents as per sizes (min. and max limits)
    // Tiny, Small      - [-24, 79]
    // Medium           - [-29, 99]
    // Large            - [-59, 139]
    // Huge             - [-89, 179]

    // 16               - [-24, 79]
    // 32               - [-49, 159]
    // 64               - [-129, 324]
    // 128              - [-214, 624] , y=[-229, 249]
    // 256              - [-449, 1249] , y=[-449, 449]

    /*
    // Set x_min and z_min to -50, y_min to -100
    extentNew[0] = extentNew[4] = -100;
    extentNew[2] = -200;
    // Set x_max and z_max to 450, y_max to 150
    extentNew[1] = extentNew[5] = 499;
    extentNew[3] = 249;81.5
    */
    extentNew[0] = extentNew[4] = -214;
    extentNew[2] = -229;
    extentNew[1] = extentNew[5] = 624;
    extentNew[3] = 249;
    vtkSmartPointer<vtkStructuredPoints> solutionCameras = vtkSmartPointer<vtkStructuredPoints>::New();
    solutionCameras->SetExtent(extentNew);
    solutionCameras->AllocateScalars(VTK_UNSIGNED_INT, 1);
    // Initialize the dataset
    for(int x = extentNew[0]; x <= extentNew[1]; x++)
    {
        for(int y = extentNew[2]; y <= extentNew[3]; y++)
        {
            for(int z = extentNew[4]; z <= extentNew[5]; z++)
            {
                // check for condition if voxel liew within boundaries of other datasets
                bool boundCondition = (x >= extent[0]) && (x <= extent[1]) && (y >= extent[2]) && (y <= extent[3]) && (z >= extent[4]) && (z <= extent[5]);
                // if condition true, implies voxel lies within old boundaries, set it to value from dataset
                if(boundCondition)
                {
                    unsigned int* solVerified = static_cast<unsigned int*>(solutionWorldVerified->GetScalarPointer(x, y, z));
                    unsigned int* solCameras = static_cast<unsigned int*>(solutionCameras->GetScalarPointer(x,y,z));
                    *solCameras = *solVerified;
                }
                // else, set it to zero
                else
                {
                    unsigned int* solCameras = static_cast<unsigned int*>(solutionCameras->GetScalarPointer(x,y,z));
                    *solCameras = 0;
                }
            }
        }
    }

    // Loop over the number of cameras
    for(int i = 0; i < camSolution.size(); i++)
    {
        // Define camera frustum lines using two points for each line
        // Each entry is one line defined with two points
        QVector<QVector<QVector3D>> frustumLines;
        // top-left line
        frustumLines.append({camSolution[i], camFrustums[i][0]});
        // bottom-left
        frustumLines.append({camSolution[i], camFrustums[i][1]});
        // bottom-right
        frustumLines.append({camSolution[i], camFrustums[i][2]});
        // top-right
        frustumLines.append({camSolution[i], camFrustums[i][3]});
        // far-plane left
        frustumLines.append({camFrustums[i][0], camFrustums[i][1]});
        // far-plane bottom
        frustumLines.append({camFrustums[i][1], camFrustums[i][2]});
        // far-plane right
        frustumLines.append({camFrustums[i][2], camFrustums[i][3]});
        // far-plane top
        frustumLines.append({camFrustums[i][3], camFrustums[i][0]});

        // Loop over all the lines
        for(int j = 0; j < frustumLines.size(); j++)
        {
            // Get the two points making the line
            QVector3D p1(frustumLines[j][0]), p2(frustumLines[j][1]);

            // find min and max (x,y,z) indices and check boundary conditions
            // remove boundary condition checking
            // x
            int minX, maxX, minY, maxY, minZ, maxZ;
            minX = p1.x() < p2.x() ? p1.x() : p2.x();
            maxX = p1.x() > p2.x() ? p1.x() : p2.x();
            // y
            minY = p1.y() < p2.y() ? p1.y() : p2.y();
            maxY = p1.y() > p2.y() ? p1.y() : p2.y();
            // z
            minZ = p1.z() < p2.z() ? p1.z() : p2.z();
            maxZ = p1.z() > p2.z() ? p1.z() : p2.z();
            // Loop over min and max coordinates
            for(int x = minX - 3; x <= maxX + 3; x++)
            {
                for(int y = minY - 3; y <= maxY + 3; y++)
                {
                    for(int z = minZ - 3; z <= maxZ + 3; z++)
                    {
                        // get distance from line
                        QVector3D currPoint(x,y,z);
                        QVector3D lineDir(p2 - p1);
                        lineDir.normalize();
                        // Check distance
                        float dist = currPoint.distanceToLine(p1, lineDir);
                        if( dist <= 0.5f)
                        {
                            // Get the voxel and set it to value 10
                            unsigned int* solCameras = static_cast<unsigned int*>(solutionCameras->GetScalarPointer(x, y, z));
                            *solCameras = 10;
                        }
                    }
                }
            }
        }
    }

    // File name
    std::vector<std::string> splitFileName;
    boost::split(splitFileName, fileNameVTK, [](char c){return c == '.';});
    // Write into file
    /*vtkNew<vtkGenericDataObjectWriter> writerSolutionVerified;          // For writing volume after cross-verification with differing values depending on segments
    writerSolutionVerified->SetInputData(solutionWorldVerified);
    writerSolutionVerified->SetFileName((splitFileName[0] + "_solution_verified.vtk").c_str());
    writerSolutionVerified->Write();
    // writer for new extendede dataset
    vtkNew<vtkGenericDataObjectWriter> writerSolutionCameras;
    writerSolutionCameras->SetInputData(solutionCameras);
    writerSolutionCameras->SetFileName((splitFileName[0] + "_solution_Final.vtk").c_str());
    writerSolutionCameras->Write();
    // Writer for covered back points volume
    vtkNew<vtkGenericDataObjectWriter> writerSolutionBackPoints;
    writerSolutionBackPoints->SetInputData(solutionBackPoints);
    writerSolutionBackPoints->SetFileName((splitFileName[0] + "_solution_covered_points.vtk").c_str());
    writerSolutionBackPoints->Write();*/
    // Save solution volume in .vti file
    vtkNew<vtkXMLImageDataWriter> writerVTI;
    writerVTI->SetInputData(solutionCameras);
    writerVTI->SetFileName((splitFileName[0] + "_solution_Final_" + nameLabel + ".vti").c_str());
    writerVTI->Write();
    // Save solutionVerified volume in .vti file
    vtkNew<vtkXMLImageDataWriter> writerVerifiedVTI;
    writerVTI->SetInputData(solutionWorldVerified);
    writerVTI->SetFileName((splitFileName[0] + "_solution_verified.vti").c_str());
    writerVTI->Write();

    // Print verified accuracy
    // Counter for total number occupied background points
    float totalOccupied = 0.0;
    for(int j = 0; j < backgroundVoxels.size(); j++)
        totalOccupied += backgroundCoverageVerify[j];
    // Accuracy if bowl surface
    qDebug() << "Coverage after verification (%) = " << (totalOccupied/backgroundVoxels.size())*100;
}

void VTKWidget::setupResultVolume()
{
    VisibilityCheck visibilityCheck;
    // Setup containers for storing output values
    QVector<QVector<bool>> camVals;
    //--------------Added to let program compile and run-----------
    QVector<QVector3D> superVoxels = boundaryVoxels;
    QVector<int> segmentLabels(boundaryVoxels.size());
    //-------------------------------------------------------------
    camVals.resize(superVoxels.size());
    for(int i = 0; i < superVoxels.size(); i++)
    {
        camVals[i].resize(9);
    }
    // COntainer to store occupied background values
    QVector<bool> backVals;

    // variables to read data from output files
    bool value;
    qint32 i,j;

    QString dirName = "/media/anirudh/Data/Documents/PhD/Qt_projects/virtual_machine/MIP_optimizer/gmat32Down/";
    QDir dir(dirName);
    int fileCount = dir.count() - 2;            // Do not consider the '.' and '..' entries

    // Container to store camera location indices
    QVector<QVector<int>> solutionIndex;

    // Once optimization is finished read the output files to grab results
    QFile camFile(dirName + "outputCam.dat");
    camFile.open(QIODevice::ReadOnly);
    QDataStream inCam(&camFile);
    while(!camFile.atEnd())
    {
        inCam >> i >> j >> value;
        camVals[i][j] = value;
        // if the value is one, save the camera location and orientation
        if(value)
            solutionIndex.append({i,j});
    }
    camFile.close();
    //qDebug() << solutionIndex;

    QFile contFile(dirName + "outputCont.dat");
    contFile.open(QIODevice::ReadOnly);
    QDataStream inCont(&contFile);
    while(!contFile.atEnd())
    {
        inCont >> value;
        backVals.append(value);
    }
    contFile.close();

    // Create container for storing occupied background voxels for each camera
    QVector<QVector<int>> occupiedBackVoxels;
    occupiedBackVoxels.resize(solutionIndex.size());
    //visibilityCheck.getSolutionoccupation(solutionIndex, superBackVoxels,  occupiedBackVoxels);

    //---------- Update the voxworld volume ----------
    // Counter to keep track of voxel value
    int voxValCounter = 2;
    // First show camera positions with value 5
    for(int a = 0; a < solutionIndex.size(); a++)
    {
        // Get all voxels with same label as this supervoxel
        for(int b = 0; b < segmentLabels.size(); b++)
        {
            // if label is same, highlight that voxel
            //qDebug() << solutionIndex[a][0] << " " << segmentLabels[b]
            if(segmentLabels[b] == solutionIndex[a][0])
            {
                // Get the supervoxel location from voxWorld
                unsigned int* valWorld = static_cast<unsigned int*>(voxWorld->GetScalarPointer(boundaryVoxels[b].x(), boundaryVoxels[b].y(), boundaryVoxels[b].z()));
                // set the value with 5
                *valWorld = voxValCounter;
            }
        }
        voxValCounter++;
    }
    // Show occupied nackground points with value 2
    for(int a = 0; a < backVals.size(); a++)
    {
        if(backVals[a])
        {
            // get all the background voxels with label a
            for(int b = 0; b < segmentBackLabels.size(); b++)
            {
                if(segmentBackLabels[b] == a)
                {
                    // set the voxel with value 2
                    unsigned int* valWorld = static_cast<unsigned int*>(voxWorld->GetScalarPointer(backgroundVoxels[b].x(), backgroundVoxels[b].y(), backgroundVoxels[b].z()));
                    *valWorld = 2;
                }
            }
        }
    }

    // Wrtie the volume into file
    std::vector<std::string> splitFileName;
    boost::split(splitFileName, fileNameVTK, [](char c){return c == '.';});
    vtkNew<vtkGenericDataObjectWriter> writer;

    // Save the global volume
    writer->SetInputData(voxWorld);
    writer->SetFileName((splitFileName[0] + "_world.vtk").c_str());
    writer->Write();

    visualizeVolume(voxWorld);
}

void VTKWidget::simulateTest()
{
    // Create a new volume
    vtkSmartPointer<vtkStructuredPoints> simulateData = vtkSmartPointer<vtkStructuredPoints>::New();
    // Containers for surface voxels and normals
    QVector<QVector3D> simulatedSurfaceVoxels, simulatedSurfaceNormals;
    /* We want to create a separate volume for calculating surface normals,
     * so that volume inside geometric object doenst conflict when displaying final clusters
     */
    vtkSmartPointer<vtkStructuredPoints> simulateFilledData = vtkSmartPointer<vtkStructuredPoints>::New();
    // ITkProcessing class object
    ITKProcessing simulatedVolumeProcessor;
    // string for distinct file names for each geometric object
    std::string geomObjName;
    // number of clusters that will be decided based on geometric object of choice
    int clusters;

    // Simulated data dimensions
    int cubeLen = 9;
    int dims[6] = {0,19,0,19,0,19};
    simulateData->SetExtent(dims);
    simulateData->AllocateScalars(VTK_UNSIGNED_INT, 1);
    // Get vol dimensions and calculate boundaries in voxels
    int volDims[3];
    simulateData->GetDimensions(volDims);

    // Swtich case statements for different 3D primitive structures
    /* 0 = cube
     * 1 = cylinder
     * 2 = sphere
     */
    const int geometricObject = 1;
    switch (geometricObject)
    {
    case 0 :
    {
        // Create geometric object name for creating filenames later
        geomObjName = "_cube_";
        // Set number clusters=6 because cube has 6 faces
        clusters = 6;

        int cent[3] = {volDims[0]/2, volDims[1]/2, volDims[2]/2};
        int minBound[3] = {cent[0]-int(cubeLen/2), cent[1]-int(cubeLen/2), cent[2]-int(cubeLen/2)};
        int maxBound[3] = {cent[0]+int(cubeLen/2), cent[1]+int(cubeLen/2), cent[2]+int(cubeLen/2)};

        /* For a cube wih center (10,10,10), the corners are given by
         * (6,6,6) till (14,14,14). To draw a hollow cube, we draw all the 6 planes
         * by filling voxels between two diagonal points lying on each plane
         */

        /* Construct cube faces in x-y plane at z = 6 and z = 14
         *
         * At the same time collect those voxels into the Surface voxels container
         * face by face, to see if it can help during initialization
         */

        // Temporary containers for each face
        QVector<QVector3D> tempFaceVoxels1, tempFaceVoxels2;
        tempFaceVoxels1.clear();
        tempFaceVoxels2.clear();
        simulatedSurfaceVoxels.clear();
        simulatedSurfaceNormals.clear();

        int zStatic = minBound[2], zStatic1 = maxBound[2];
        for(int x = minBound[0]; x <= maxBound[0]; x++)
        {
            for(int y = minBound[1]; y <= maxBound[1]; y++)
            {
                unsigned int* currVal = static_cast<unsigned int*>(simulateData->GetScalarPointer(x,y,zStatic));
                unsigned int* currVal1 = static_cast<unsigned int*>(simulateData->GetScalarPointer(x,y,zStatic1));
                // Process only if not already set
                if(*currVal != 2)
                {
                    // Set these voxel to 2
                    *currVal = 2;
                    // Add the voxels into containers
                    tempFaceVoxels1.append(QVector3D(x,y,zStatic));
                }
                // repeat for other face
                if(*currVal1 != 2)
                {
                    *currVal1 = 2;
                    tempFaceVoxels2.append(QVector3D(x,y,zStatic1));
                }
            }
        }

        // move voxel locations from temporary to final container and clear temporary ones for next face
        simulatedSurfaceVoxels.append(tempFaceVoxels1);
        simulatedSurfaceVoxels.append(tempFaceVoxels2);
        tempFaceVoxels1.clear();
        tempFaceVoxels2.clear();

        // Construct cube faces in y-z plane at x = 6 and x = 14
        int xStatic = minBound[0], xStatic1 = maxBound[0];
        for(int z = minBound[2]; z <= maxBound[2]; z ++)
        {
            for(int y = minBound[1]; y <= maxBound[1]; y++)
            {
                unsigned int* currVal = static_cast<unsigned int*>(simulateData->GetScalarPointer(xStatic,y,z));
                unsigned int* currVal1 = static_cast<unsigned int*>(simulateData->GetScalarPointer(xStatic1,y,z));
                // Process only if not already set
                if(*currVal != 2)
                {
                    // Set these voxel to 2
                    *currVal = 2;
                    // Add the voxels into containers
                    tempFaceVoxels1.append(QVector3D(xStatic,y,z));
                }
                // repeat for other face
                if(*currVal1 != 2)
                {
                    *currVal1 = 2;
                    tempFaceVoxels2.append(QVector3D(xStatic1,y,z));
                }
            }
        }

        // move voxel locations from temporary to final container and clear temporary ones for next face
        simulatedSurfaceVoxels.append(tempFaceVoxels1);
        simulatedSurfaceVoxels.append(tempFaceVoxels2);
        tempFaceVoxels1.clear();
        tempFaceVoxels2.clear();


        // Construct cube faces in x-z plane at y = 6 and y = 14
        int yStatic = minBound[1], yStatic1 = maxBound[1];
        for(int z = minBound[2]; z <= maxBound[2]; z++)
        {
            for(int x = minBound[0]; x <= maxBound[0]; x++)
            {
                unsigned int* currVal = static_cast<unsigned int*>(simulateData->GetScalarPointer(x,yStatic,z));
                unsigned int* currVal1 = static_cast<unsigned int*>(simulateData->GetScalarPointer(x,yStatic1,z));
                // Process only if not already set
                if(*currVal != 2)
                {
                    // Set these voxel to 2
                    *currVal = 2;
                    // Add the voxels into containers
                    tempFaceVoxels1.append(QVector3D(x,yStatic,z));
                }
                // repeat for other face
                if(*currVal1 != 2)
                {
                    *currVal1 = 2;
                    tempFaceVoxels2.append(QVector3D(x,yStatic1,z));
                }
            }
        }
        // move voxel locations from temporary to final container and clear temporary ones for next face
        simulatedSurfaceVoxels.append(tempFaceVoxels1);
        simulatedSurfaceVoxels.append(tempFaceVoxels2);
        tempFaceVoxels1.clear();
        tempFaceVoxels2.clear();

        simulateFilledData->DeepCopy(simulateData);
        /* Fill the cube volume, because just the surface provides
         * surface normals only at the corners and edges
         */
        for(int z = minBound[2]; z <= maxBound[2]; z++)
        {
            for(int y = minBound[1]; y <= maxBound[1]; y++)
            {
                for(int x = minBound[0]; x <= maxBound[0]; x++)
                {
                    // Get the voxel
                    unsigned int* currVal = static_cast<unsigned int*>(simulateFilledData->GetScalarPointer(x,y,z));
                    // set the voxel if it is not yet set
                    if(*currVal != 2)
                    {
                        *currVal = 2;
                    }
                }
            }
        }
        break;
    }
    case 1:
    {
        // Create geometry name for creating filenames later
        geomObjName = "_cylinder_";
        // Set number of clusters=10 because cylinder is constructed with roughly 8 sides and 2 more on top and bottom
        clusters = 6;

        // initialize filled volume image
        simulateFilledData->SetExtent(dims);
        simulateFilledData->AllocateScalars(VTK_UNSIGNED_INT, 1);
        // initialize all to 0
        for(int z = dims[4]; z <= dims[5]; z++)
        {
            for(int y = dims[2]; y <= dims[3]; y++)
            {
                for(int x = dims[0]; x <= dims[1]; x++)
                {
                    unsigned int* currvalCyl = static_cast<unsigned int*>(simulateFilledData->GetScalarPointer(x,y,z));
                    unsigned int* simulateDataVox = static_cast<unsigned int*>(simulateData->GetScalarPointer(x,y,z));
                    *simulateDataVox = 0;
                    *currvalCyl = 0;
                }
            }
        }
        /* We want to draw a cylinder of radius 8 voxels and height 16 voxels (in y-axis)
         * We draw circles at center (10,y,10) where y=[6,14) to form cylinder
         */
        for(int y = 2; y <= 17; y++)
        {
            // center for each circle
            int cylCenter[3] = {10,y,10};
            // loop over x and z and set all voxels that lie within the radius
            int cylRadius = 8;
            for(int z = dims[4]; z < dims[5]; z++)
            {
                for(int x = dims[0]; x < dims[1]; x++)
                {
                    unsigned int* currvalCyl = static_cast<unsigned int*>(simulateFilledData->GetScalarPointer(x,y,z));
                    // If within cylRadius, set it to 1
                    float dist = sqrt(pow(x - cylCenter[0],2) + pow(y - cylCenter[1],2) + pow(z - cylCenter[2],2));
                    if(dist < cylRadius)
                    {
                        *currvalCyl = 2;
                    }
                }
            }
        }
        // We can get the surface of cylinder with morphological dilation
        vtkSmartPointer<vtkStructuredPoints> dilatedSimulateData = vtkSmartPointer<vtkStructuredPoints>::New();
        dilatedSimulateData->DeepCopy(simulateFilledData);
        simulatedVolumeProcessor.volumeDilation(dilatedSimulateData);
        /* Collect all voxels
         * For cylinder, collect in x-z dimensions first, then in y
         * This could help with cluster center initialization
         */
        simulatedSurfaceVoxels.clear();
        for(int z = dims[4]; z <= dims[5]; z++)
        {
            for(int y = dims[2]; y <= dims[3]; y++)
            {
                for(int x = dims[0]; x <= dims[1]; x++)
                {
                    unsigned int* currVox = static_cast<unsigned int*>(dilatedSimulateData->GetScalarPointer(x,y,z));
                    unsigned int* currFilledVox = static_cast<unsigned int*>(simulateFilledData->GetScalarPointer(x,y,z));
                    // collect boundary voxels that are 2 in dilated volume but 0 in original volume
                    if(*currVox == 2 && *currFilledVox == 0)
                    {
                        // add the voxels into the container
                        simulatedSurfaceVoxels.append(QVector3D(x,y,z));
                    }
                }
            }
        }
        // Copy the dilated volume into 'simulateFilledData' so that
        // surface voxel locations do not conflict when collecting surface normals
        simulateFilledData->DeepCopy(dilatedSimulateData);
        // Setup the 'simulateData' volume with the collected surface voxels.
        for(int i = 0; i < simulatedSurfaceVoxels.size(); i++)
        {
            // voxel on surface from simulateData volume
            unsigned int* currSurfaceVox =
                    static_cast<unsigned int*>(simulateData->GetScalarPointer(simulatedSurfaceVoxels[i].x(),
                                                                              simulatedSurfaceVoxels[i].y(),
                                                                              simulatedSurfaceVoxels[i].z()));
            // Set this voxel to 2
            *currSurfaceVox = 2;
        }
        break;
    }
    }

    // Visualize cube
    visualizeVolume(simulateData);

    // Gradient volume containers
    QVector<vtkSmartPointer<vtkStructuredPoints>> simulatedGradientVolumes;
    // Convert volume type to float
    vtkSmartPointer<vtkStructuredPoints> floatSimulatedVol = vtkSmartPointer<vtkStructuredPoints>::New();
    vtkNew<vtkImageShiftScale> shiftFilter;
    shiftFilter->SetOutputScalarTypeToFloat();
    shiftFilter->SetInputData(simulateFilledData);
    shiftFilter->SetScale(1);
    shiftFilter->SetShift(0);
    shiftFilter->Update();
    floatSimulatedVol->DeepCopy(shiftFilter->GetOutput());
    // get surface normals
    simulatedVolumeProcessor.generateNormals(floatSimulatedVol, simulatedGradientVolumes);

    // Vector indices for voxels
    QVector<unsigned long int> SimulatedVectorIndices(volDims[0]*volDims[1]*volDims[2]);
    //int count = 0;

    // Collect Surface voxels and normals
    for(int i = 0; i < simulatedSurfaceVoxels.size(); i++)
    {
        // Get the voxel coordinates
        int x = simulatedSurfaceVoxels[i].x();
        int y = simulatedSurfaceVoxels[i].y();
        int z = simulatedSurfaceVoxels[i].z();
        // get the voxel value from gradient images
        float* normalX = static_cast<float*>(simulatedGradientVolumes[0]->GetScalarPointer(x,y,z));
        float* normalY = static_cast<float*>(simulatedGradientVolumes[1]->GetScalarPointer(x,y,z));
        float* normalZ = static_cast<float*>(simulatedGradientVolumes[2]->GetScalarPointer(x,y,z));
        // add the normal to surfaceNormals container after inverting sign
        simulatedSurfaceNormals.append(QVector3D(-*normalX,-*normalY,-*normalZ));
        /* Update at vector index locations
         * vector indices differ for cube and cylinder,
         * due to the order in which the voxels are collected
         */
        //if(geometricObject == 0)
        //{
            // For cube
        SimulatedVectorIndices[x + (y * dims[1]) + (z * dims[1] * dims[3])] = i;
        //}
        /*else if(geometricObject == 1)
        {
            // For cylinder
            SimulatedVectorIndices[x + (z * dims[1]) + (y * dims[1] * dims[5])] = i;
        }*/
        //count++;
    }

    // Setup output containers for segmentation
    QVector<QVector3D> superSimulatedVoxels, superSimulatedNormals;
    QVector<int> simulatedSegmentLabels;
    // value of surface voxels (currently 2)
    unsigned int surfVal = 2;
    // Call Segmentation algorithms
    if(clusterChoice)
    {
        // ------------------- SLIC clustering-----------------------------------
        // Call the function perform SLIC based segmentation
        volumeProcessor.slicSegmentation(simulatedSurfaceVoxels,
                                         simulatedSurfaceNormals,
                                         simulateData,
                                         clusters,
                                         superSimulatedVoxels,
                                         superSimulatedNormals,
                                         simulatedSegmentLabels,
                                         surfVal,
                                         SimulatedVectorIndices);
    }
    else
    {
        // -------------------Face clustering algorithm--------------------------
                // Number of required boundary voxels after segmentation
        volumeProcessor.faceClustering(simulatedSurfaceVoxels,
                                       simulatedSurfaceNormals,
                                       clusters,
                                       SimulatedVectorIndices,
                                       simulateData,
                                       superSimulatedVoxels,
                                       superSimulatedNormals,
                                       simulatedSegmentLabels);
    }

    // Save all the images in .vtk files for verification on paraview
    std::vector<std::string> splitFileName;
    boost::split(splitFileName, fileNameVTK, [](char c){return c == '.';});
    // Save un-segmented image
    vtkNew<vtkGenericDataObjectWriter> writerInputImage;
    writerInputImage->SetInputData(simulateData);
    writerInputImage->SetFileName((splitFileName[0] + geomObjName + "raw.vtk").c_str());
    writerInputImage->Write();

    // Update boundary volume with the segments
    for(int i = 0 ; i < simulatedSurfaceVoxels.size(); i++)
    {
        // get the voxel coordinates
        QVector3D voxelCoords = simulatedSurfaceVoxels[i];
        // get the boundary voxel value
        unsigned int* boundVox = static_cast<unsigned int *>(simulateData->GetScalarPointer(voxelCoords.x(), voxelCoords.y(), voxelCoords.z()));
        /* Set the value with the label such that all sgements have a value between 1 and 10
         * And shift all the values by 50 to distinguish from background super voxels
         */
        *boundVox = (simulatedSegmentLabels[i] % clusters) + 1;
    }

    // Save segmented image
    vtkNew<vtkGenericDataObjectWriter> writerSimulated;
    // Save the volume model with only boundary voxels
    writerSimulated->SetInputData(simulateData);
    writerSimulated->SetFileName((splitFileName[0] + geomObjName + "segmented.vtk").c_str());
    writerSimulated->Write();
}

void VTKWidget::visualizeVTK(vtkNew<vtkActor> &actor)
{
    // Colors object to set background color
    vtkNew<vtkNamedColors> colors;
    vtkColor3d backgroundColor = colors->GetColor3d("SteelBlue");

    // Setup openGL render window
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;

#if VTK890
    setRenderWindow(renderWindow);
#else
    SetRenderWindow(renderWindow);
#endif

    // Setup renderer
    vtkNew<vtkRenderer> renderer;
    renderer->AddActor(actor);
    renderer->SetBackground(backgroundColor.GetData());
    renderer->ResetCamera();
    renderer->GetActiveCamera()->Azimuth(30);
    renderer->GetActiveCamera()->Elevation(30);
    renderer->GetActiveCamera()->Dolly(1.5);
    renderer->ResetCameraClippingRange();

#if VTK890
    renderWindow->AddRenderer(renderer);
    renderWindow->SetWindowName("VTK Widget");
#else
    renderWindow->AddRenderer(renderer);
    renderWindow->SetWindowName("VTK Widget");
#endif

    renderWindow->Render();
}

void VTKWidget::visualizeVolume(vtkSmartPointer<vtkStructuredPoints> &voxelData)
{
    // setup data for visualization
    vtkNew<vtkSmartVolumeMapper> mapper;
    mapper->SetBlendModeToComposite();
    //mapper->SetInputConnection(reader->GetOutputPort());
    mapper->SetInputData(voxelData);
    mapper->SetRequestedRenderModeToRayCast();
    // VTK volume property
    vtkNew<vtkVolumeProperty> volumeProperty;
    volumeProperty->ShadeOff();
    volumeProperty->SetInterpolationType(VTK_LINEAR_INTERPOLATION);

    // Define opacity for data
    vtkNew<vtkPiecewiseFunction> compositeOpacity;
    compositeOpacity->AddPoint(0.0, 0.001);
    compositeOpacity->AddPoint(1.0, 0.01);
    compositeOpacity->AddPoint(2.0, 0.6);                       // Give slightly different opacity for boundary voxels
    compositeOpacity->AddPoint(3.0, 1.0);
    //volumeProperty->SetScalarOpacity(compositeOpacity);       // composite opacity first
    volumeProperty->SetGradientOpacity(compositeOpacity);

    // Define colors
    vtkNew<vtkColorTransferFunction> color;
    color->AddRGBPoint(0.0, 1.0, 1.0, 1.0);                     // background points are white
    //color->AddRGBPoint(1.0, 1.0, 0.0, 0.0);                     // vehicle is in red
    color->AddRGBPoint(3.0, 1.0, 0.0, 0.0);                     // Boundary voxels in green
    volumeProperty->SetColor(color);

    // VTK volume (replaces actor)
    vtkNew<vtkVolume> volume;
    volume->SetMapper(mapper);
    volume->SetProperty(volumeProperty);

    // Colors object to set background color
    vtkNew<vtkNamedColors> colors;
    vtkColor3d backgroundColor = colors->GetColor3d("Grey");

    // Setup openGL render window
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;

#if VTK890
    setRenderWindow(renderWindow);
#else
    SetRenderWindow(renderWindow);
#endif

    // Setup renderer
    vtkNew<vtkRenderer> renderer;
    renderer->AddViewProp(volume);
    renderer->SetBackground(backgroundColor.GetData());
    renderer->ResetCamera();
    renderer->GetActiveCamera()->Azimuth(30);
    renderer->GetActiveCamera()->Elevation(30);
    renderer->GetActiveCamera()->Dolly(1.5);
    renderer->ResetCameraClippingRange();

#if VTK890
    renderWindow->AddRenderer(renderer);
    renderWindow->SetWindowName("VTK Widget");
#else
    renderWindow->AddRenderer(renderer);
    renderWindow->SetWindowName("VTK Widget");
#endif

    renderWindow->Render();
}
