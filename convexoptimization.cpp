#include "convexoptimization.h"

ConvexOptimization::ConvexOptimization()
{
    /* Initialize camera specification parameters
     * alpha_l, alpha_r --- horizontal FOV angles
     * alpha_b, alpha_t --- vertical FOV angles
     * f --- focal length (mm)
     * s_u, s_v --- horizontal and vertical pixel dimensions (mm/pixel)
     * z_n, z_f --- distances specifying focus range with z_n < z_f (voxels)
     * r_a --- min. required resolution threshold
     * N_bd --- number of required voxels to be initially covered
     * N_delta --- min. required number of voxels to be covered in current recursion
     *
     * Considering a 2/3 inch sensor of dimensions 8.8mmx6.6mm
     * with image resolution 1680x1260 pixels
     */

    /* https://www.edmundoptics.eu/resources/application-notes/imaging/understanding-focal-length-and-field-of-view/
     * Refer above website (ex 2) for calculations
     *
     * we are referring to samsung galazy S10 wide angle camera (FOV = 77 degree) ()
     */
    alpha_r = alpha_l = 38.5f * static_cast<float>((M_PI/180));
    alpha_t = alpha_b = 38.5f * static_cast<float>((M_PI/180));

    /* z_n, z_f, z_s depend on the object dimensions.
     * For e.g, these calculation are based on
     * our BMW object fit in a cube with s=128/64/32 voxels
     *
     * Using original dimensions of BMW X5 it can be
     * made realistic considering each voxel has a
     * dimension of ~38.45/77/154 mm
     */
    z_n = 5;
    z_f = 40;

    // Let us consider to place 4 cameras
    nCam = 3;

    // World up direction is pointing in the positive y direction
    worldUp = QVector3D(0.0f, 1.0f, 0.0f);

    /* Get the half horizaontal and vertical distances on the far plane canvas
     * This can be done using sensor dimensions, f and similar triangle properties
     * See description in the beginning for sensor dimensions
     */
    hFar = z_f * tan(alpha_l);
    vFar = z_f * tan(alpha_t);
    //qDebug()<<"horizontal FOV/2 = " << hFar << " vertical FOV/2 = " << vFar;

    // Number of cameras that have to cover a control point
    M = 1;

    //------------------- Following parameters not relevant----------------------------------

    // Assume f as mentioned in paper
    //f = 8.0f;

    s_u = s_v = 0.0052f;

    // z_s is the center of focus point
    z_s = (z_n + z_f)/2;

    // Initialize scale factors for weights in the translation determination objective function
    c_xy = 1/(z_s * std::max(std::tan(alpha_l), std::tan(alpha_r)));
    c_foc = 2/(z_f - z_n);
    c_z = 1/(z_f - z_n);
    c_r = f/(z_n * std::max(s_u, s_v));

    /* Initialize weights
     * For simplicity lets consider them all = 1 for now
     */
    _w_xy = _w_foc = _w_z = _w_r = 1;
    w_xy = _w_xy * c_xy;
    w_foc = _w_foc * c_foc;
    w_z = _w_z * c_z;
    w_r = _w_r * c_r;
}

void ConvexOptimization::getFOVAngles(float &alpha_h, float &alpha_v)
{
    alpha_h = alpha_r;
    alpha_v = alpha_t;
}

ConvexOptimization::Frustum ConvexOptimization::calcFrustum(QVector3D camPos, QVector3D CamDir)
{
    // Camera right vector
    QVector3D camRight = QVector3D::crossProduct(CamDir, worldUp).normalized();
    // Camera up vector
    QVector3D camUp = QVector3D::crossProduct(camRight, CamDir).normalized();

    //qDebug()<<"Camera Orientations are\n"<<"Camera direction: "<<CamDir<<" Camera right: "<<camRight<<" Camera Up: "<<camUp<<"\n";

    // Far plane center
    QVector3D farCenter = camPos + CamDir*z_f;
    // Create a structure for the frustum
    Frustum camFOV;
    // Far Plane top left
    camFOV.far.append((farCenter + (camUp*vFar)) - camRight*hFar);
    // Far plane bottom left
    camFOV.far.append(camFOV.far[0] - camUp*2*vFar);
    // Far plane bottom right
    camFOV.far.append(camFOV.far[1] + camRight*2*hFar);
    // Far plane top right
    camFOV.far.append(camFOV.far[2] + camUp*2*vFar);

    // Left plane
    camFOV.left.append(camPos);             // Camera location
    camFOV.left.append(camFOV.far[1]);      // common point with far plane bottom left
    camFOV.left.append(camFOV.far[0]);      // common point with far plane top left

    // Bottom plane
    camFOV.bottom.append(camPos);           // Camera location
    camFOV.bottom.append(camFOV.far[2]);    // far plane bottom right
    camFOV.bottom.append(camFOV.far[1]);    // far plane bottom left

    // Right plane
    camFOV.right.append(camPos);            // Camera location
    camFOV.right.append(camFOV.far[3]);     // far plane top right
    camFOV.right.append(camFOV.far[2]);     // far plane bottom right

    // Top plane
    camFOV.top.append(camPos);              // Camera location
    camFOV.top.append(camFOV.far[0]);       // Far plane top left
    camFOV.top.append(camFOV.far[3]);       // Far plane top right

    return camFOV;
}

bool ConvexOptimization::checkControlPoint(Frustum camFOV, QVector3D point)
{
    bool check = 0;
    /*
    qDebug()<<"Far size = "<<camFOV.far.size()<<" top left : "<<camFOV.far[0]<<"\n";
    qDebug()<<"Left size = "<<camFOV.left.size()<<" camera : "<<camFOV.left[0]<<" = "<<point<<"\n";
    qDebug()<<"Bottom size = "<<camFOV.bottom.size()<<" bottom left : "<<camFOV.bottom[1]<<"\n";
    qDebug()<<"Right size = "<<camFOV.right.size()<<" bottom right : "<<camFOV.right[2]<<"\n";
    qDebug()<<"Top size = "<<camFOV.top.size()<<" top right : "<<camFOV.top[2]<<"\n";

    qDebug()<<"Distance of "<<point<<" from three points on far plane "<<camFOV.far[0]<<" "<<camFOV.far[1]<<" "<<camFOV.far[3]<<" is "<<point.distanceToPlane(camFOV.far[0], camFOV.far[1], camFOV.far[3])<<"\n";
    qDebug()<<"Distance of "<<point<<" from three points on left plane "<<camFOV.left[0]<<" "<<camFOV.left[1]<<" "<<camFOV.left[2]<<" is "<<point.distanceToPlane(camFOV.left[0], camFOV.left[1], camFOV.left[2])<<"\n";
    qDebug()<<"Distance of "<<point<<" from three points on bottom plane "<<camFOV.bottom[0]<<" "<<camFOV.bottom[1]<<" "<<camFOV.bottom[2]<<" is "<<point.distanceToPlane(camFOV.bottom[0], camFOV.bottom[1], camFOV.bottom[2])<<"\n";
    qDebug()<<"Distance of "<<point<<" from three points on right plane "<<camFOV.right[0]<<" "<<camFOV.right[1]<<" "<<camFOV.right[2]<<" is "<<point.distanceToPlane(camFOV.right[0], camFOV.right[1], camFOV.right[2])<<"\n";
    qDebug()<<"Distance of "<<point<<" from three points on top plane "<<camFOV.top[0]<<" "<<camFOV.top[1]<<" "<<camFOV.top[2]<<" is "<<point.distanceToPlane(camFOV.top[0], camFOV.top[1], camFOV.top[2])<<"\n";
    */
    // If all the distances are either positive or zero it means the point is inside the FOV frustum
    //qDebug()<< " Checking far : "<< camFOV.far << " with point : "<<point<<" distance = " << point.distanceToPlane(camFOV.far[0], camFOV.far[1], camFOV.far[3]);
    if(point.distanceToPlane(camFOV.far[0], camFOV.far[1], camFOV.far[3]) >= 0)
    {
        //qDebug()<< " Checking left : "<< camFOV.left << " with point : "<<point<<" distance = " << point.distanceToPlane(camFOV.left[0], camFOV.left[1], camFOV.left[2]);
        if(point.distanceToPlane(camFOV.left[0], camFOV.left[1], camFOV.left[2]) >= 0)
        {
            //qDebug()<< " Checking bottom : "<< camFOV.bottom << " with point : "<<point<<" distance = " << point.distanceToPlane(camFOV.bottom[0], camFOV.bottom[1], camFOV.bottom[2]);
            if(point.distanceToPlane(camFOV.bottom[0], camFOV.bottom[1], camFOV.bottom[2]) >= 0)
            {
                //qDebug()<< " Checking right : "<< camFOV.right << " with point : "<<point<<" distance = " << point.distanceToPlane(camFOV.right[0], camFOV.right[1], camFOV.right[2]);
                if(point.distanceToPlane(camFOV.right[0], camFOV.right[1], camFOV.right[2]) >= 0)
                {
                    //qDebug()<< " Checking top : "<< camFOV.top << " with point : "<<point<<" distance = " << point.distanceToPlane(camFOV.top[0], camFOV.top[1], camFOV.top[2]);
                    if(point.distanceToPlane(camFOV.top[0], camFOV.top[1], camFOV.top[2]) >= 0)
                    {
                        check = 1;
                    }
                }
            }
        }
    }

    return check;
}

void ConvexOptimization::setup_g(QVector<QVector3D> controlPoints, QVector<QVector3D> camPos, QVector<QVector<QVector3D>> normals)
{
    /* Initialize variable g_j,i
     * where j is no. of possible camera locations and i is number of control points
     * 2D array of dimensions j x i
     * Each row in g[j][i] will contain 1s at locations i indicating the control points that are covered by camera at position j
     */

    // initialize dimensions of g
    int numControl = controlPoints.size();
    int numCamPos = camPos.size();
    int numRotations = normals[0].size();

    /* GPU device selection
     *
     * Filter for a 2.0 platform and set it as default
     */
    std::vector<cl::Platform> platforms;
    cl::Platform::get(&platforms);
    cl::Platform plat;
    //qDebug()<<"No. of platforms: "<<platforms.size()<<"\n";
    for (auto &p : platforms)
    {
        std::string platver = p.getInfo<CL_PLATFORM_VERSION>();
        //std::string platvend = p.getInfo<CL_PLATFORM_VENDOR>();
        std::cout << platver << "\n";
        //std::cout << platvend << "\n";
        if(platver.find("OpenCL 2.") != std::string::npos)
            plat = p;
    }
    if(plat() == 0)
    {
        qDebug()<< "No OpenCL 2.0 platform found.\n";
    }

    cl::Platform newP = cl::Platform::setDefault(plat);
    if(newP != plat)
    {
        qDebug()<<"Error setting default platform.\n";
    }

    // Kernel to calculate frustum at each camera location
    std::string kernel1{
        "kernel void frustumCalc(global const float3 *camPoints, global const float3 *camDirection, global float16 *camFrustum, global float3 *farPlanePoint, float3 camParams){"
        "  float3 worldUp = (float3)(0.0f, 1.0f, 0.0f);"
        "  int iterId = get_global_id(0);"
        "  float3 camRight = normalize(cross(camDirection[iterId], worldUp));"
        "  float3 camUp = normalize(cross(camRight, camDirection[iterId]));"
        // Field of view parameter calculation
        "  float3 farCenter = camPoints[iterId] + camDirection[iterId]*camParams.z;"
        "  float3 far_tl = farCenter + camUp*camParams.y - camRight*camParams.x;"
        "  float3 far_bl = far_tl - camUp*2*camParams.y;"
        "  float3 far_br = far_bl + camRight*2*camParams.x;"
        "  float3 far_tr = far_br + camUp*2*camParams.y;"
        "  float3 farNormal = normalize(cross(far_bl - far_tl, far_tr - far_tl));"
        "  float3 leftNormal = normalize(cross(far_bl - camPoints[iterId], far_tl - camPoints[iterId]));"
        "  float3 bottomNormal = normalize(cross(far_br - camPoints[iterId], far_bl - camPoints[iterId]));"
        "  float3 rightNormal = normalize(cross(far_tr - camPoints[iterId], far_br - camPoints[iterId]));"
        "  float3 topNormal = normalize(cross(far_tl - camPoints[iterId], far_tr - camPoints[iterId]));"
        "  camFrustum[iterId] = (float16)(farNormal, leftNormal, bottomNormal, rightNormal, topNormal, 0.0f);"
        "  farPlanePoint[iterId] = far_tl;"
        //"  printf(\"Processed id %d \", iterId);"
        //"printf(\" -----%d------\", count);"
    "}"};

    // Kernel to check control points for each camera location
    std::string kernel2{
        "kernel void checkControlPoints(global const float3 *contPoints, const float16 camFrustum, float3 camPoint, float3 far_tl, global int *outMatG){"
        "  int iterId = get_global_id(0);"
        "  float3 farNormal = (float3)(camFrustum.s0, camFrustum.s1, camFrustum.s2);"
        "  float3 leftNormal = (float3)(camFrustum.s3, camFrustum.s4, camFrustum.s5);"
        "  float3 bottomNormal = (float3)(camFrustum.s6, camFrustum.s7, camFrustum.s8);"
        "  float3 rightNormal = (float3)(camFrustum.s9, camFrustum.sA, camFrustum.sB);"
        "  float3 topNormal = (float3)(camFrustum.sC, camFrustum.sD, camFrustum.sE);"
        "  int check = 0;"
        "  if(dot(contPoints[iterId] - far_tl, farNormal) >= 0)"
        "    if(dot(contPoints[iterId] - camPoint, leftNormal) >= 0)"
        "      if(dot(contPoints[iterId] - camPoint, bottomNormal) >= 0)"
        "        if(dot(contPoints[iterId] - camPoint, rightNormal) >= 0)"
        "          if(dot(contPoints[iterId] - camPoint, topNormal) >= 0)"
        "            check = 1;"
        "  outMatG[iterId] = check;"
    "}"};

    // Newer Opencl simpler string interface
    std::vector<std::string> programStrings{kernel1, kernel2};

    cl::Program camProgram(programStrings);
#if defined (CL_HPP_ENABLE_EXCEPTIONS)
    try{
        camProgram.build("-cl-std=CL2.0");
    }
    catch(...){
        // Print build info for all devices
        cl_int buildErr = CL_SUCCESS;
        auto buildInfo = camProgram.getBuildInfo<CL_PROGRAM_BUILD_LOG>(&buildErr);
        for(auto &pair : buildInfo)
        {
            std::cerr << pair.second << std::endl << std::endl;
        }
    }
#else
    cl_int buildErr = camProgram.build("-cl-std=CL2.0");
    if(buildErr != CL_SUCCESS)
    {
        std::cerr << "Build error: "<< buildErr << "\n";
    }
#endif

    cl::SVMAllocator<int, cl::SVMTraitCoarse<>> svmAlloc;
    qDebug() << "Max alloc size: " << svmAlloc.max_size() << " bytes\n";

    // SVM allocations for frustum calculation kernel
    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camPoints(svmAlloc);        // input vector containing camera locations
    std::vector<std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camDirection(svmAlloc);     // input vector containing camera directions
    std::vector<cl_float16, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camFrustum(svmAlloc);      // output vector to store the calculated frustum
    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> farPlanePoint(svmAlloc);    // output vector to store far plane top left point coordinates

    // SVM allocations for control points checking kernel
    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> contPoints(svmAlloc);
    std::vector<cl_int, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> outMatG(svmAlloc);

    auto frustumCalcKernel =
            cl::KernelFunctor<
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float16, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                cl_float3
            >(camProgram, "frustumCalc");

    auto pointCheckKernel =
            cl::KernelFunctor<
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                cl_float16,
                cl_float3,
                cl_float3,
                std::vector<cl_int, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&
            >(camProgram, "checkControlPoints");

    // Setup the camera parameters (hfar, vfar and z_f) to pass it to the kernel function
    cl_float3 camParams;
    camParams.x = hFar;
    camParams.y = vFar;
    camParams.z = z_f;

    // Add another loop here for directions
    
    // Initialize camPoints buffer
    camPoints.clear();
    camPoints.resize(numCamPos);
    //Initialize camera direction buffer
    camDirection.clear();
    camDirection.resize(numRotations);

    // Resize the second dimension of camDirection vector
    for(int j = 0; j < numRotations; j++)
    {
        camDirection[j].resize(numCamPos);
    }

    for(int i = 0; i < numCamPos; i++)
    {
        // Initialize camera positions buffer
        camPoints[i].x = camPos[i].x();
        camPoints[i].y = camPos[i].y();
        camPoints[i].z = camPos[i].z();
        // Initialize camera direction buffer
        for(int j = 0; j < numRotations; j++)
        {
            camDirection[j][i].x = normals[i][j].x();
            camDirection[j][i].y = normals[i][j].y();
            camDirection[j][i].z = normals[i][j].z();
        }
    }

    cl::unmapSVM(camPoints);
    cl::unmapSVM(camDirection);

    // Write the calculated g matrix to binary files
    QString fileName("/media/anirudh/Data/Documents/PhD/Qt_projects/virtual_machine/MIP_optimizer/gmat32Down/g" + QString::number(1) + ".dat");
    QFile file(fileName);
    file.open(QIODevice::WriteOnly);
    QDataStream out(&file);

    // Loop over number of rotations and process all points for each rotation
    for(int k = 0; k < numRotations; k++)
    {
        // Initialize frustum conatiner vector
        camFrustum.clear();
        camFrustum.resize(numCamPos);
        // Initialize far_tl point container buffer
        farPlanePoint.clear();
        farPlanePoint.resize(numCamPos);

        cl::unmapSVM(camFrustum);
        cl::unmapSVM(farPlanePoint);

        cl_int error;
        frustumCalcKernel(
                    cl::EnqueueArgs(cl::NDRange(numCamPos)),
                    camPoints,
                    camDirection[k],
                    camFrustum,
                    farPlanePoint,
                    camParams,
                    error
                    );

        // Grab the output SVM matrix
        cl::mapSVM(camFrustum);
        cl::mapSVM(farPlanePoint);

        /* Divide the data into blocks for memory efficiency
         * Process all the cam positions and about 10k control points
         * and save the data into binary files
         */
        int blockSize = numControl;
        int count = 0;

        // process blocks of control points
        while(count < numControl)
        {
            // Inititlize contPoints vector
            contPoints.clear();
            contPoints.resize(blockSize);
            for (int j = 0; j < blockSize; j++)
            {
                // Initialize control points
                contPoints[j].x = controlPoints[count + j].x();
                contPoints[j].y = controlPoints[count + j].y();
                contPoints[j].z = controlPoints[count + j].z();
            }

            for (int i = 0; i < numCamPos; i++)
            {
                // Initialize dimensions for output g buffer
                outMatG.clear();
                outMatG.resize(blockSize);

                cl::unmapSVM(contPoints);
                cl::unmapSVM(outMatG);

                cl_int error;
                pointCheckKernel(
                            cl::EnqueueArgs(cl::NDRange(numControl)),
                            contPoints,
                            camFrustum[i],
                            camPoints[i],
                            farPlanePoint[i],
                            outMatG,
                            error
                            );

                cl::mapSVM(outMatG);

                for (int j = 0; j < blockSize; j++)
                {
                    if(outMatG[j] == 1)
                    {
                        out << (qint32)k << (qint32)i << (qint32)j << (bool)outMatG[j];
                    }
                }
            }

            // Increment count by blockSize for next block of camera positions
            count = count + blockSize;

            // Take care of the last iteration because number of elements might be lesss than 1000
            if(count/blockSize == numControl/blockSize)
            {
                blockSize = numControl % blockSize;
            }
        }
        // Unfortunately, there is no way to check for a default or know if a kernel needs one
        // so the user has to create one
        // We can't preemptively do so on device creation because they cannot then replace it
        //cl::DeviceCommandQueue defaultDeviceQueue;
        //defaultDeviceQueue = cl::DeviceCommandQueue::makeDefault();
    }
    file.close();
}

void ConvexOptimization::greedyAlgo(QVector<QVector3D> arrayControl, QVector<QVector3D> arrayCamPos, QVector<QVector<QVector3D>> normals, QString dirName)
{
    int controlPoints = arrayControl.size();
    int camPos = arrayCamPos.size();
    int numRotations = normals[0].size();

    // Max number of cameras allowed
    int nCam = 5;

    // number of control points covered
    //int nbCovered = 0;

    // Number of cameras placed
    int nCamPlaced = 0;

    // Stopping criteria (minimum required coverage)
    //int reqCoverage = (95 * controlPoints)/100;

    // Intialize output containers for optimized camera positions and covered control points
    QVector<bool> c(controlPoints, 0);
    QVector<QVector<bool> > var(camPos);
    for(int i = 0; i < camPos; i++)
    {
        var[i].resize(numRotations);
        for(int j = 0; j < numRotations; j++)
            var[i][j] = 0;
    }

    // Container to check for global camera positions
    QVector<int> globalPos(nCam);
    // int to hold global camera position
    int globalPos1;
    // Copy of control point coordinates to keep track of covered control points
    QVector<QVector3D> arrayControlCopy(arrayControl);

    // Loop for algorithm
    while (/*nbCovered <= reqCoverage &&*/ nCamPlaced < nCam)
    {
        // ---------------- Setup g matrix -----------------------------------
        QVector<boost::numeric::ublas::compressed_matrix<IloBool>> g;
        for(int i = 0; i < numRotations; i++)
        {
            boost::numeric::ublas::compressed_matrix<IloBool> g_temp(camPos, controlPoints);
            g.append(g_temp);
        }
        /* Take the first entry of the returned list of files present in the directory
         * Because we are considering only few control points for now and they can be fit into one file
         * Append the dirName to get absolute path
         */
        // Open a QDirectory
        QDir dir(dirName);
        QString fileName = dirName + dir.entryList(QDir::AllEntries | QDir::NoDotAndDotDot, QDir::Time | QDir::Reversed)[0];
        //QString fileName = dirName + "g0.dat";

        // Open the file and read the complete data stream
        QFile f(fileName);
        f.open(QIODevice::ReadOnly);
        QDataStream inStream(&f);
        qint32 row,col,rot;
        bool val;

        while(!f.atEnd())
        {
            inStream >> rot >> row >> col >> val;
            g[rot](row, col) = (IloBool)val;
            //qDebug()<< "(" << i << "," << j <<") " << val;
        }
        inStream.~QDataStream();
        f.close();

        // create matrix to hold rank
        QVector<QVector<int>> coverageRank(camPos);
        for(int i = 0; i < camPos; i++)
            coverageRank[i].resize(numRotations);

        // Create rank matrix from g matrix
        for(int k = 0; k < numRotations; k++)
        {
            for(int i = 0; i < camPos; i++)
            {
                int coverageCount = 0;
                for(int j = 0; j < controlPoints; j++)
                {
                    coverageCount += g[k](i, j);
                }
                coverageRank[i][k] = coverageCount;
            }
        }

        // -------------Search for highest rank-------------------------------
        int pos1 = 0, pos2 = 0;
        int maxRank = coverageRank[0][0];
        QVector<QVector<int>>::iterator i = coverageRank.begin();           // Iterator over camera positions
        while (i != coverageRank.end())
        {
            QVector<int>::iterator j = i->begin();
            while(j != i->end())
            {
                if(*j > maxRank)
                {
                    maxRank = *j;
                    pos1 = distance(coverageRank.begin(), i);
                    pos2 = distance(i->begin(), j);
                }
                j++;
            }
            i++;
        }

        //qDebug() << "Estimated position " << pos1;

        /* Keep track of camera location indices w.r.t 'arrayCamPos'
         * as we are removing positions from it and it results in mismatch between the locations
         */
        globalPos[nCamPlaced] = pos1;
        globalPos1 = pos1;
        //qDebug()<< coverageRank[pos1];
        for(int camCounter = 0; camCounter < nCamPlaced; camCounter++)
        {
            // Check each camera position w.r.t. newly obtained camera locations
            if(globalPos[camCounter] <= pos1)
                globalPos1 += 1;
        }

        //qDebug()<< "global position: " << globalPos << globalPos1;
        //qDebug()<< coverageRank[pos1];

        // Place a camera at the position and orientation with highest rank
        var[globalPos1][pos2] = 1;
        // Increment number of newly covered points
        //nbCovered += maxRank;
        // Increment number of cameras placed
        nCamPlaced++;
        // Update control points variable with newly covered points
        for(int j = 0; j < controlPoints; j++)
        {
            if(g[pos2](pos1, j) == 1)
            {
                // Search for the index of this control point in the original copied array
                c[arrayControlCopy.indexOf(arrayControl[j])] = 1;
                /* mark coordinates in 'arrayControl' at these indices with the special value (-1, -1, -1).
                 * Any 'special values' with negative values should be safe as the voxel environment has
                 * only positive coordinates.
                 */
                arrayControl[j] = QVector3D(-1.0f, -1.0f, -1.0f);
            }
        }

        /* Remove covered control points from original list of control points
         * And update the number of uncovered points
         */
        controlPoints -= arrayControl.removeAll(QVector3D(-1.0f, -1.0f, -1.0f));

        /* Remove camera position that has just been added
         * Also remove the corresponding row in the normals matrix
         */
        arrayCamPos.erase(arrayCamPos.begin() + pos1);
        normals.erase(normals.begin() + pos1);
        camPos--;

        // setup g matrix with updated possible camera positions and uncovered control points
        setup_g(arrayControl, arrayCamPos, normals);
    }

    // -------------- Write the output arrays to separate binary files ------------------------------

    // first write camera positions
    QString fileNameOut = dirName + "outputCam.dat";
    QFile file(fileNameOut);
    file.open(QIODevice::WriteOnly);
    QDataStream out(&file);
    for(int k = 0; k < var.size(); k++)
    {
        for (int i = 0; i < var[0].size(); i++)
        {
            out << (qint32)k << (qint32)i << (bool)var[k][i];
        }
    }
    file.close();

    // Now write control points
    QString fileNameOut1 = dirName + "outputCont.dat";
    QFile file1(fileNameOut1);
    file1.open(QIODevice::WriteOnly);
    QDataStream out1(&file1);
    for (int i = 0; i < c.size(); i++)
    {
        out1 << (bool)c[i];
    }
    file1.close();
}

void ConvexOptimization::optimizeRemote(int numCam, int numControl, int rotations)
{
    /* Setup model and optimize it on remote machine
     * Note : Paths are relative to remote system
     */
    QString command = "ssh anirudh@10.53.6.23 ./../../media/data/MIP_optimizer/bin/MIP_optimizer " + QString::number(numCam) + " " + QString::number(numControl) + " " + QString::number(rotations) + " " + QString::number(0) + " /media/data/MIP_optimizer/gmat32Down/";
    QProcess *virtualMachine = new QProcess();
    qDebug()<<"Calling optimization process on remote machine: " << command;
    virtualMachine->setProcessChannelMode(QProcess::ForwardedChannels);
    virtualMachine->start(command);

    if(virtualMachine->waitForFinished(-1))
        qDebug()<< "Optimization complete. Reading output data from files...";
}

void ConvexOptimization::optimizeObjective(QVector<QVector3D> controlPoints, QVector<QVector3D> camPos, QVector<QVector<QVector3D> > camDir, QVector<QVector3D> &optCamPos, QVector<bool> &contVals, QVector<int> &optCamInd, QVector<int> &optRotInd)
{
    // Flag to trigger use of greedy algorithm
    bool greedy = true;

    // Create arrays to hold results from optimization and initialize
    QVector<QVector<bool>> camVals;
    camVals.resize(camPos.size());
    for(int i = 0; i < camPos.size(); i++)
    {
        camVals[i].resize(camDir[0].size());
    }
    // variables to read data from output files
    bool value;
    qint32 i,j;

    QString dirName = "/media/anirudh/Data/Documents/PhD/Qt_projects/virtual_machine/MIP_optimizer/gmat32Down/";
    QDir dir(dirName);
    int fileCount = dir.count() - 2;            // Do not consider the '.' and '..' entries

    /* Construct matrix 'g'
         * if gMatrix.txt file exists, directly run optimization program on CPU server.
         * or else call the function to populate the matrix and
         * then call optimization program on CPU server.
         */
    if(fileCount == 1 && !greedy)
    {
        // Call the function to run remote process (optimization)
        optimizeRemote(camPos.size(), controlPoints.size(), camDir[0].size());
    }
    else if(fileCount == 0 && !greedy)
    {
        // If directory is empty do the calculations for g matrix
        qDebug()<<"File not found or g matrix has not been created yet..creating g matrix";
        setup_g(controlPoints, camPos, camDir);

        // Call the function to run remote process (optimization)
        optimizeRemote(camPos.size(), controlPoints.size(), camDir[0].size());
    }
    else if(fileCount == 1 && greedy)
    {
        // Call function to optimize using greedy algorithm
        greedyAlgo(controlPoints, camPos, camDir, dirName);
    }
    else if(fileCount == 0 && greedy)
    {
        // If directory is empty do the calculations for g matrix
        qDebug()<<"File not found or g matrix has not been created yet..creating g matrix";
        setup_g(controlPoints, camPos, camDir);

        // Call function to optimize using greedy algorithm
        greedyAlgo(controlPoints, camPos, camDir, dirName);
    }

    // Once optimization is finished read the output files to grab results
    QFile camFile(dirName + "outputCam.dat");
    camFile.open(QIODevice::ReadOnly);
    QDataStream inCam(&camFile);
    while(!camFile.atEnd())
    {
        inCam >> i >> j >> value;
        camVals[i][j] = value;
    }
    camFile.close();

    QFile contFile(dirName + "outputCont.dat");
    contFile.open(QIODevice::ReadOnly);
    QDataStream inCont(&contFile);
    while(!contFile.atEnd())
    {
        inCont >> value;
        contVals.append(value);
    }
    contFile.close();

    // Get optimized camera position locations
    Frustum frustum;
    QVector<bool> contCross(controlPoints.size(), 0);           // vector to cross-verify results
    for(int i = 0; i < camVals.size(); i++)
    {
        for(int k = 0; k < camVals[i].size(); k++)
        {
            if(camVals[i][k] == 1)
            {
                optCamPos.append(camPos[i]);
                qDebug()<< "position : " << camPos[i] << " direction : " << camDir[i][k];
                frustum = calcFrustum(camPos[i], camDir[i][k]);

                // Cross verification of results
                int counter = 0;
                for (int j = 0; j < contCross.size(); j++)
                {
                    if(!contCross[j])
                    {
                        contCross[j] = checkControlPoint(frustum, controlPoints[j]);
                        counter++;
                    }
                }
                qDebug()<< counter << " Number of points still unoccupied!";

                optCamPos.append(frustum.far);
                optCamInd.append(i);
                optRotInd.append(k);
               // qDebug()<< "cam Position : "<< camPos[i] <<" Frustum points : "<<frustum.far;
            }
        }
    }

    // Sum all the elements in 'contCross' to calculate accuracy
    qDebug()<< " Verified accuracy : " << (float)contCross.count(1)/contCross.size() * 100.0f << "%";

    /* count how many control points are covered
     * and collect the covered control points
     */
    float controlCount = 0;
    for(int i = 0; i < contVals.size(); i++)
    {
        if(contVals[i] == 1)
            controlCount++;
    }
    qDebug()<< "Surrounding area coverage accuracy = "<< (controlCount/controlPoints.size()) * 100.0f << "%";

}
