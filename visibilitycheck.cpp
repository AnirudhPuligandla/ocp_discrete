#include "visibilitycheck.h"

// Constructor when parameters are not specified
// In this case, we need to setup the kernels and define camera parameters
VisibilityCheck::VisibilityCheck()
{
    // Call function to setup camera parameters
    //initCamParams();
    //initCamParamsMini();
    //initCamParamsM();
    //initCamParamsL();
    //initCamParamsH();

    //initCamParams16();
    //initCamParams32();
    initCamParams64();
    //initCamParams128();
    //initCamParams256();

    // Call function to create kernels and setup other GPU properties
    setupGPU();
}

VisibilityCheck::VisibilityCheck(QVector<QVector3D> &boundaryVoxels,
                                 QVector<QVector3D> &boundaryNormals,
                                 QVector<QVector3D> &backgroundVoxels,
                                 QVector<QVector3D> &vehicleVoxels,
                                 int numRotations)
{
    // Initialize input data
    camPos.clear();
    camDir.clear();
    backgroundVox.clear();
    camUp.clear();
    camRight.clear();

    int numCamPos = boundaryVoxels.size();
    //int numRotations = 53;
    camDir.resize(numRotations);
    camUp.resize(numRotations);
    camRight.resize(numRotations);
    for(int i = 0; i < numRotations; i++)
    {
        camDir[i].resize(numCamPos);
        camUp[i].resize(numCamPos);
        camRight[i].resize(numCamPos);
    }

    camPos = boundaryVoxels;
    camDir[0] = boundaryNormals;
    backgroundVox = backgroundVoxels;
    vehicleVox = vehicleVoxels;

    // Call function to initialize camera parameters
    //initCamParams();

    //initCamParamsMini();
    //initCamParamsM();
    //initCamParamsL();
    //initCamParamsH();

    //initCamParams16();
    //initCamParams32();
    initCamParams64();
    //initCamParams128();
    //initCamParams256();

    // Initialize the kernel strings and other GPU parameters
    setupGPU();

    // generate rotations for all camera positions
    setupCameras();

    /*int inSize = 32;
    gFileName = QString("/media/anirudh/Data/Documents/PhD/Qt_projects/ocp_simulation/resources/Scraper_" + QString::number(inSize) + "_g_" + QString::number(minBackPercent) + ".dat");
    indFileName = QString("/media/anirudh/Data/Documents/PhD/Qt_projects/ocp_simulation/resources/Scraper_" + QString::number(inSize) + "_i_" + QString::number(minBackPercent) + ".dat");
    QByteArray g_ba = gFileName.toLatin1();
    QByteArray i_ba = indFileName.toLatin1();
//     call the funtion to setup the g matrix and write it to file
//     To save we write computed gmatrix into a file,
//     load it and skip these calculations if that is available
    struct stat buf;
    if(!(stat(g_ba.data(), &buf) == 0))*/
        setup_g();
    /*else
    {
        // first read index1D vector
        QFile indFile(indFileName);
        indFile.open(QIODevice::ReadOnly);
        QDataStream indStream(&indFile);
        qint32 ind1,ind2;
        while(!indFile.atEnd())
        {
            indStream >> ind1 >> ind2;
            indices1D.append(QVector<int>({ind1, ind2}));
        }
        indStream.~QDataStream();
        indFile.close();
        // now read gMatrix2D
        gMatrix2D = boost::numeric::ublas::compressed_matrix<bool>(indices1D.size(), backgroundVox.size());
        QFile gFile(gFileName);
        gFile.open(QIODevice::ReadOnly);
        QDataStream gStream(&gFile);
        qint32 gInd1,gInd2;
        while(!gFile.atEnd())
        {
            gStream >> gInd1 >> gInd2;
            gMatrix2D(gInd1,gInd2) = 1;
        }
        gStream.~QDataStream();
        gFile.close();
    }*/
    qDebug() << "Visibility checks completed!";
}

void VisibilityCheck::initCamParams()
{
    /* Define camera specifications considering One plus 6 camera
     * the horizontal field of view angle is 63.1 deg
     * aspect ratio is 16:9
     * depth of field is decided according to the 12m radius (approx. 190 voxels)
     * based on calculations hFar*2 estimates at approx. 223 voxels rounded to 224 (16x14) (multiple of 16)
     * implies hFar = 112, vFar = 63
     *
     * Note : hFar*2 at 256 (16x16) voxels gives a better depth of field at 208.5 voxels.
     *        So z_f = hFar/tan(hFov/2)  for new model will be 208, hFar will be 128
     *        and vFar = (hFar * 9)/16 = 72
     *
     * from aspect ratio it can be said that vertical FoV angle is roughly 35.5 deg
     * and vFar*2 will be 126 voxels
     *
     * Updated model uses AR 4:3 instead of 16:9
     */
    onePlus6.hFOV = 63.1f;
    onePlus6.vFOV = 47.325f;//35.5f;
    onePlus6.z_f = 190;//208;
    onePlus6.hFar = 116;//128;
    onePlus6.vFar = 87;//96;

    // Also specify the numbe rof vehicle points in camera FoV are allowed to pass self-occlusion check
    /* For our camera FOV, the volume can be calculated through volume of pyramid calculations
     * to be 2 million voxels. 5% is 10 lakh voxels which is still a very high number.
     * So for the start lets set the limit as 1000 vehicle points as threhsold value for
     * discarding the occluded cameras
     */
    numVehAllowed = 1000;
    // Define a min. required of points to be covered by a camera
    // 5 cams currently cover 75-90% of ~1000 (1% of total) background points
    // That is about 150-180 points per camera. So set min. limit as 50 points
    minReqBackCov = 100;
}

void VisibilityCheck::initCamParamsMini()
{
    onePlus6.hFOV = 63.1f;
    onePlus6.vFOV = 47.325f;//35.5f;
    onePlus6.z_f = 26;//208;
    onePlus6.hFar = 16;//128;
    onePlus6.vFar = 12;//96;
    numVehAllowed = 1;
    /*  car1    --> 323 or 64 per camera => 70%=45
     */
    minReqBackCov = 45;
}

void VisibilityCheck::initCamParamsM()
{
    onePlus6.hFOV = 63.1f;
    onePlus6.vFOV = 47.325f;//35.5f;
    onePlus6.z_f = 30;//208;
    onePlus6.hFar = 20;//128;
    onePlus6.vFar = 15;//96;
    numVehAllowed = 1;
    /* car1     --> 531 OR 106 per camera => 70%=74
     */
    minReqBackCov = 74; // Linear relation with hFar
}

void VisibilityCheck::initCamParamsL()
{
    onePlus6.hFOV = 63.1f;
    onePlus6.vFOV = 47.325f;//35.5f;
    onePlus6.z_f = 48;//208;
    onePlus6.hFar = 32;//128;
    onePlus6.vFar = 24;//96;
    numVehAllowed = 1;
    /* car1     -> 1649 or 329 per camera => 70%=230
     */
    minReqBackCov = 230; // Linear relation with hFar
}

void VisibilityCheck::initCamParamsH()
{
    onePlus6.hFOV = 63.1f;
    onePlus6.vFOV = 47.325f;//35.5f;
    onePlus6.z_f = 66;//208;
    onePlus6.hFar = 44;//128;
    onePlus6.vFar = 33;//96;
    numVehAllowed = 1;
    /* car1     -> 3362 or 672 per camera => 70%=470, 50%=336
     */
    minReqBackCov = 470; // Linear relation with hFar
}

void VisibilityCheck::initCamParams16()
{
    onePlus6.hFOV = 63.1f;
    onePlus6.vFOV = 47.325f;//35.5f;
    onePlus6.z_f = 24;//208;
    onePlus6.hFar = 16;//128;
    onePlus6.vFar = 12;//96;

    numVehAllowed = 5;
    minReqBackCov = 90;//115; // Linear relation with hFar
}

void VisibilityCheck::initCamParams32()
{
    onePlus6.hFOV = 63.1f;
    onePlus6.vFOV = 47.325f;//35.5f;
    onePlus6.z_f = 54;//60;//208;
    onePlus6.hFar = 36;//40;//128;
    onePlus6.vFar = 27;//30;//96;

    numVehAllowed = 0;
    /* vehicle  -> Size SR   MR
     * Bull     -> 32   280  280
     * JCB      -> 32   280  280
     * Mine     -> 32   300
     * Scraper  -> 32   260
     */
    /* bulldozer -> 334 or 66.8 per camera
     *              50% = 33, 70% = 46, 80% = 53, 95% = 63
     * JCB       -> 330 or 66 per camera
     *              70% = 46
     * Mining    -> 405 or 81 per camera
     *              70% = 56
     * Tractor   -> 294 or 58 percamera
     *              70% = 41
     */
    minReqBackCov = 46; // Linear relation with hFar
    minBackPercent = 70;
}

void VisibilityCheck::initCamParams64()
{
    onePlus6.hFOV = 63.1f;
    onePlus6.vFOV = 47.325f;//35.5f;
    onePlus6.z_f = 132;//208;
    onePlus6.hFar = 88;//128;
    onePlus6.vFar = 66;//96;

    numVehAllowed = 0;
    /* vehicle  -> 32   64  high
     * Bull     -> 280  290
     * JCB      -> 280  280
     * Mine     -> 300
     * Scraper  -> 260  280 200
     */
    /* bulldozer -> 1366 or 273 per camera
     *              50% = 136, 70% = 191, 60% = 163
     *              choice = 105% (1328 points) for SR, 70% for MR
     * JCB       -> 1345 or 269 per camera
     *              70% = 188
     * mining truck -> 1595 or 319 per camera
     *              105% = 335, 100% = 319, 50% = 159, 60% = 191, 70% = 223
     *              choice = 105% (59) for SR, 60% for MR
     * tractor      -> 1324 or 264 per camera
     *              70% = 185
     */
    minReqBackCov = 185;
    minBackPercent = 223;
}

void VisibilityCheck::initCamParams128()
{
    onePlus6.hFOV = 63.1f;
    onePlus6.vFOV = 47.325f;//35.5f;
    onePlus6.z_f = 278;//208;
    onePlus6.hFar = 184;//128;
    onePlus6.vFar = 138;//96;

    numVehAllowed = 0;
    /* vehicle  -> 128
     * Bull     -> 300
     * JCB      -> 300
     * Mine     -> 300/310
     * Scraper  -> 290
     */
    /* bulldozer -> 5419 or 1083 (216) per camera
     *              50% = 541, 60% = 650, 70% = 758 (151), 75% = 812, 80% = 866
     * JCB       -> 1077 or (215) per camera
     *              70% = 150
     * Mining    -> 1289 or 257 per camera
     *              70% = 180
     * tractor   -> 1050 or 210 per camera
     *              70% = 147
     */
    minReqBackCov = 151; // Linear relation with hFar
    minBackPercent = 70;
}

void VisibilityCheck::initCamParams256()
{
    onePlus6.hFOV = 63.1f;
    onePlus6.vFOV = 47.325f;//35.5f;
    onePlus6.z_f = 568;//208;
    onePlus6.hFar = 376;//128;
    onePlus6.vFar = 282;//96;

    numVehAllowed = 0;
    /* vehicle  -> 128
     * Bull     -> 340
     * JCB      -> 320/300
     * Mine     -> 450/380
     * Scraper  -> 360
     */
    /* bulldozer -> 21945 or 4389 (351) per camera
     *              50% = 2194, 60% = 2633, 70% = 3072 (245), (80% = 280, 75% = 263)
     * JCB       -> 21675 or 4335 (346) per camera
     *              70% = 3034 (242), 75% = (59), 80% = (276)
     * Mining    -> 2079 or 415 per camera
     *              70% = 291
     * Tractor   -> 20983 or 4196 per camera
     *              (1679 or 335 per camer)
     *              70% = 2937 (235), 80% = (268)
     */
    minReqBackCov = 235; // Linear relation with hFar
    minBackPercent = 242;
}

void VisibilityCheck::setupGPU()
{
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
        //std::string platvend = p.getInfo<CL_PLATFORM_VENDOR>();image2d_t gmat_trans, const int cam1, global const *cam2
        //std::cout << platver << "\n";
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

    /*std::vector<cl::Device> devices;
    plat.getDevices(CL_DEVICE_TYPE_ALL, &devices);
    cl_device_type dev_type;
    clGetDeviceInfo(devices[0].get(), CL_DEVICE_TYPE, sizeof(dev_type), &dev_type, NULL);
    if (dev_type == CL_DEVICE_TYPE_CPU) {
        qDebug() << "I'm 100%% sure this device is a GPU";
    }*/

    /* Create a openCL kernel for calculating camera up and right vectors from camDir (view direction)
     *
     * involves the following steps:
     * 1) Assume x-z plane faces in +ve y-direction
     * 2) get direction of line intersecting between surface plane and x-z plane
     *      -- given by camDir x (0,1,0)
     * 3) get camera up vector
     *      -- given by camRight x camUp
     *
     * Inputs : camera view direction (surface normal) at voxel 'camDirection'
     * Outputs : camera up and right direction vectors 'camUp', 'camRight'
     */
    std::string cameraKernel{
        "kernel void defineCamera(global const float3 *camDirection, global float3 *camUp, global float3 *camRight){"
        "   int iterId = get_global_id(0);"
        "   float3 xzNormal = (float3)(0.0f, 1.0f, 0.0f);"
        "   camRight[iterId] = normalize(cross(camDirection[iterId], xzNormal));"
        "   camUp[iterId] = normalize(cross(camRight[iterId], camDirection[iterId]));"
    "}"};

    /* OpenCL kernel to calculate orientation vector after rotation about an 'axis' by an 'angle'
     *
     * steps involved:
     * 1) construct the rotation matrix -> get the matrix multiplication output with the 'axis' vector
     *
     * Inputs: primary orientation 'camDirection'
     *         axis of rotation 'axis'
     *         'angle' of rotation in radians
     * Outputs: direction vector after rotation 'rotatedDirection'
     */
    std::string camPoseKernel{
        "kernel void calcOrientations(global const float3 *camDirection, global const float3 *Axis, float angle, global float3 *rotatedDirection){"
        "   int iterId = get_global_id(0);"
        "   float3 axis = Axis[iterId];"
        "   float r11 = cos(angle) + pown(axis.s0, 2) * (1 - cos(angle));"
        "   float r12 = axis.s0 * axis.s1 * (1 - cos(angle)) - axis.s2 * sin(angle);"
        "   float r13 = axis.s0 * axis.s2 * (1 - cos(angle)) + axis.s1 * sin(angle);"
        "   float r21 = axis.s1 * axis.s0 * (1 - cos(angle)) + axis.s2 * sin(angle);"
        "   float r22 = cos(angle) + pown(axis.s1, 2) * (1 - cos(angle));"
        "   float r23 = axis.s1 * axis.s2 * (1 - cos(angle)) - axis.s0 * sin(angle);"
        "   float r31 = axis.s2 * axis.s0 * (1 - cos(angle)) - axis.s1 * sin(angle);"
        "   float r32 = axis.s2 * axis.s1 * (1 - cos(angle)) + axis.s0 * sin(angle);"
        "   float r33 = cos(angle) + pown(axis.s2, 2) * (1 - cos(angle));"
        "   rotatedDirection[iterId].s0 = r11 * camDirection[iterId].s0 + r12 * camDirection[iterId].s1 + r13 * camDirection[iterId].s2;"
        "   rotatedDirection[iterId].s1 = r21 * camDirection[iterId].s0 + r22 * camDirection[iterId].s1 + r23 * camDirection[iterId].s2;"
        "   rotatedDirection[iterId].s2 = r31 * camDirection[iterId].s0 + r32 * camDirection[iterId].s1 + r33 * camDirection[iterId].s2;"
    "}"};

    // Kernel to calculate frustum at each camera location
    std::string frustumKernel{
        "kernel void frustumCalc(global const float3 *camPoints, global const float3 *camDirection, global const float3 *camUp, global const float3 *camRight, global float16 *camFrustum, global float3 *farPlanePoint, float3 camParams){"
        "  int iterId = get_global_id(0);"
        // Field of view parameter calculation
        "  float3 farCenter = camPoints[iterId] + camDirection[iterId]*camParams.z;"
        "  float3 far_tl = farCenter + camUp[iterId]*camParams.y - camRight[iterId]*camParams.x;"
        "  float3 far_bl = far_tl - camUp[iterId]*2*camParams.y;"
        "  float3 far_br = far_bl + camRight[iterId]*2*camParams.x;"
        "  float3 far_tr = far_br + camUp[iterId]*2*camParams.y;"
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
    std::string pointCheckKernel{
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

    /* kernel to check vehicle self-occlusions
     * Note: This is executed before the previous pointCheckKernel
     *
     * 1) take the current camera frustum and set of points comprising the vehicle as input
     * 2) Check for the vehicle points that lie inside the frustum
     */
    std::string occlusionCheckKernel{
        "kernel void checkSelfOcclusion(global const float3 *vehiclePoints, const float16 camFrustum, float3 camPoint, float3 far_tl, global int *occlusion){"
        "  int iterId = get_global_id(0);"
        "  float3 farNormal = (float3)(camFrustum.s0, camFrustum.s1, camFrustum.s2);"
        "  float3 leftNormal = (float3)(camFrustum.s3, camFrustum.s4, camFrustum.s5);"
        "  float3 bottomNormal = (float3)(camFrustum.s6, camFrustum.s7, camFrustum.s8);"
        "  float3 rightNormal = (float3)(camFrustum.s9, camFrustum.sA, camFrustum.sB);"
        "  float3 topNormal = (float3)(camFrustum.sC, camFrustum.sD, camFrustum.sE);"
        "  int check = 0;"
        "  if(dot(vehiclePoints[iterId] - far_tl, farNormal) >= 0)"
        "    if(dot(vehiclePoints[iterId] - camPoint, leftNormal) >= 0)"
        "      if(dot(vehiclePoints[iterId] - camPoint, bottomNormal) >= 0)"
        "        if(dot(vehiclePoints[iterId] - camPoint, rightNormal) >= 0)"
        "          if(dot(vehiclePoints[iterId] - camPoint, topNormal) >= 0)"
        "            check = 1;"
        "  occlusion[iterId] = check;"
    "}"};

    // Kernel to compute neighbourhood for rwls algorithm
    // Inputs -> *gMat - gMat matrix flattened into 1D array
    //           cam1 - index of one camera
    //           numCont - number of control points (for indexing)
    // outputs -> *cam2 - vector<bool> of size(cams) holds true if any of the gMat entries is one
    std::string rwlsNeighbourhoodKernel{
        "kernel void computeNeighbourhood(global const int *gMat, const int cam1, const int numCont, global int *cam2){"
        "  int cam2Ind = get_global_id(0);"
        "  int controlInd = get_global_id(1);"
        "  int gMatInd1 = numCont*cam1 + controlInd;"
        "  int gMatInd2 = numCont*cam2Ind + controlInd;"
        "  int val1 = gMat[gMatInd1];"
        "  int val2 = gMat[gMatInd2];"
        "  if(val1==1 && val2==1)"
        "    cam2[cam2Ind]=1;"
    "}"};

    // Newer Opencl simpler string interface
    std::vector<std::string> programStrings{cameraKernel, camPoseKernel, frustumKernel, pointCheckKernel, occlusionCheckKernel, rwlsNeighbourhoodKernel};

    camProgram = cl::Program(programStrings);

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
}

void VisibilityCheck::setupCameras()
{
    // create SVM data structure type
    cl::SVMAllocator<int, cl::SVMTraitCoarse<>> svmAlloc;
    qDebug() << "Max alloc size: " << svmAlloc.max_size() << " bytes\n";

    // initialize the svm buffers
    int numCamPos = camPos.size();

    // SVM allocations for passing into kernels
    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camDirection(svmAlloc);         // input svm buffer containing camera view direction vectors for all orientations
    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> rotationAxisUp(svmAlloc);         // input svm buffer containing the up rotation axes for corresponding position
    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> rotationAxisRight(svmAlloc);         // input svm buffer containing the right rotation axes for corresponding position

    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camUpVectors(svmAlloc);         // output svm buffers to hold camera up vectors for all view directions
    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camRightVectors(svmAlloc);      // output svm buffer to hold camera right vectors

    // initialize kernels from strings
    auto cameraKernel =
            cl::KernelFunctor<
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&
            >(camProgram, "defineCamera");

    auto camPoseKernel =
            cl::KernelFunctor<
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                cl_float,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&
            >(camProgram, "calcOrientations");

    // get the angle step for horizontal and vertical rotations
    //float angleH = ((90 - onePlus6.hFOV/2)/2)*static_cast<float>((M_PI/180));
    //float angleV = ((90 - onePlus6.vFOV/2)/2)*static_cast<float>((M_PI/180));

    /* test for rotations of 5 degrees instead of calculations above.
     * This can result in 48 rotations - 12*2 in each horizontal and vertical directions,
     * plus the primary view direction. total = 49
     *
     * The vertical direction can take upto 15 steps. So for that benefit, we can
     * for now, select 13 steps. total = 53
     */
    //float angleH = 5.0*static_cast<float>((M_PI/180));
    //float angleV = 5.0*static_cast<float>((M_PI/180));
    // Number of steps in each left,right,top,bottom directions.
    int rotationSteps = (camDir.size() - 1)/4;
    // Expect a rotation upto 120 degree in each of the four directions.
    // So, divide 120 divide by the number of steps to get angles of rotation
    // But this method would require at least 24 rotations in each of 4 directions for 5 degree precision
    // This number becomes (24*4) + 1 = 97
    float angleH = (120.0/rotationSteps)*static_cast<float>((M_PI/180));
    float angleV = (120.0/rotationSteps)*static_cast<float>((M_PI/180));

    /* Create a list to change the angle of rotation in steps of angleH and angleV
     * We use steps of 4 in each horizaontal and vertical directions to obtain 9 rotations in total for each camera position
     * Rotations work in a way that the camera view-dir does exceed 58deg from original
     * ***NOTE***
     * When changing number of rotations here, change it also in the constructor
     * ***NOTE***
     *
     * Use upto 13 steps in each direction
     */
    QVector<int> stepList;
    for(int i = -rotationSteps; i <= -1; i++)
        stepList.append(i);
     for(int i = 1; i <= rotationSteps; i++)
         stepList.append(i);

    //----------First compute the up and right vectors for the primary view direction--------------------------------

    camUpVectors.clear();
    camRightVectors.clear();
    camDirection.clear();
    camDirection.resize(numCamPos);
    camUpVectors.resize(numCamPos);
    camRightVectors.resize(numCamPos);
    // initialize primary orientation data
    for(int i = 0; i < numCamPos; i++)
    {
        camDirection[i].x = camDir[0][i].x();
        camDirection[i].y = camDir[0][i].y();
        camDirection[i].z = camDir[0][i].z();
    }

    // Unmap the input vector
    cl::unmapSVM(camDirection);
    cl::unmapSVM(camUpVectors);
    cl::unmapSVM(camRightVectors);

    // Call the function camera definition kernel
    cl_int error;
    cameraKernel(cl::EnqueueArgs(cl::NDRange(numCamPos)), camDirection, camUpVectors, camRightVectors, error);

    // Grab the output SVM vectors
    cl::mapSVM(camUpVectors);
    cl::mapSVM(camRightVectors);

    // Save the outputs into global containers
    for(int i = 0; i < numCamPos; i++)
    {
        camUp[0][i].setX(camUpVectors[i].x);
        camUp[0][i].setY(camUpVectors[i].y);
        camUp[0][i].setZ(camUpVectors[i].z);

        camRight[0][i].setX(camRightVectors[i].x);
        camRight[0][i].setY(camRightVectors[i].y);
        camRight[0][i].setZ(camRightVectors[i].z);
    }

    //-------------------------Now calculate the 8 rotations and corresponding up and right vectors---------------------

    // Counter to keep track of global rotations
    int counter = 1;
    cl_float angle;

    // intitalize the rotation axes with the up and right vectors of the primary orientation
    rotationAxisUp = camUpVectors;
    rotationAxisRight = camRightVectors;
    // And unmap these vectors here as they are constant throughout
    cl::unmapSVM(rotationAxisUp);
    cl::unmapSVM(rotationAxisRight);

    // Iterate over the number of steps
    foreach (int val, stepList)
    {
        // Vectors for horizontal direction
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camDirectionRotatedH(svmAlloc);  // output svm buffer to hold camera view direction after rotation
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camUpVectorsH(svmAlloc);         // output svm buffers to hold camera up vectors for all view directions
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camRightVectorsH(svmAlloc);      // output svm buffer to hold camera right vectors

        // Vectors for vertical direction
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camDirectionRotatedV(svmAlloc);  // output svm buffer to hold camera view direction after rotation
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camUpVectorsV(svmAlloc);         // output svm buffers to hold camera up vectors for all view directions
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camRightVectorsV(svmAlloc);      // output svm buffer to hold camera right vectors

        //-------------Calculate rotaion in horizontal direction-----------------------------
        // Compute the angle for horizontal rotations
        angle = val * angleH;
        // initialize svm containers
        camDirectionRotatedH.clear();
        camDirectionRotatedH.resize(numCamPos);
        // unmap input vectors for direction calculating kernel
        cl::unmapSVM(camDirectionRotatedH);
        // Call the kernel function to calculate rotated directions
        cl_int error;
        camPoseKernel(cl::EnqueueArgs(cl::NDRange(numCamPos)), camDirection, rotationAxisUp, angle, camDirectionRotatedH, error);

        // ----------------Calculate the up and right vectors for new rotation-------------------
        // re-initialize the output vectors
        camUpVectorsH.clear();
        camRightVectorsH.clear();
        camUpVectorsH.resize(numCamPos);
        camRightVectorsH.resize(numCamPos);
        // unmap the svm vectors
        cl::unmapSVM(camUpVectorsH);
        cl::unmapSVM(camRightVectorsH);
        // Call the function camera definition kernel
        cameraKernel(cl::EnqueueArgs(cl::NDRange(numCamPos)), camDirectionRotatedH, camUpVectorsH, camRightVectorsH, error);

        // grab the output vectors
        cl::mapSVM(camDirectionRotatedH);
        cl::mapSVM(camUpVectorsH);
        cl::mapSVM(camRightVectorsH);
        // save into global containers
        for(int i = 0; i < numCamPos; i++)
        {
            camDir[counter][i].setX(camDirectionRotatedH[i].x);
            camDir[counter][i].setY(camDirectionRotatedH[i].y);
            camDir[counter][i].setZ(camDirectionRotatedH[i].z);

            camUp[counter][i].setX(camUpVectorsH[i].x);
            camUp[counter][i].setY(camUpVectorsH[i].y);
            camUp[counter][i].setZ(camUpVectorsH[i].z);

            camRight[counter][i].setX(camRightVectorsH[i].x);
            camRight[counter][i].setY(camRightVectorsH[i].y);
            camRight[counter][i].setZ(camRightVectorsH[i].z);
        }

        // increment the counter
        counter++;

        //---------------Calculate rotation in vertical direction-----------------------------
        // compute angle for vertical rotations
        angle = val * angleV;
        // Initialize svm containers
        camDirectionRotatedV.clear();
        camDirectionRotatedV.resize(numCamPos);
        // unmap input vectors for direction calculating kernel
        cl::unmapSVM(camDirectionRotatedV);
        // Call the kernel function to calculate rotated directions
        camPoseKernel(cl::EnqueueArgs(cl::NDRange(numCamPos)), camDirection, rotationAxisRight, angle, camDirectionRotatedV, error);

        // ----------------Calculate the up and right vectors for new rotation-------------------
        // re-initialize the output vectors
        camUpVectorsV.clear();
        camRightVectorsV.clear();
        camUpVectorsV.resize(numCamPos);
        camRightVectorsV.resize(numCamPos);
        // unmap the svm vectors
        //cl::unmapSVM(camDirectionRotatedV);
        cl::unmapSVM(camUpVectorsV);
        cl::unmapSVM(camRightVectorsV);
        // Call the function camera definition kernel
        cameraKernel(cl::EnqueueArgs(cl::NDRange(numCamPos)), camDirectionRotatedV, camUpVectorsV, camRightVectorsV, error);

        // grab the output vectors
        cl::mapSVM(camDirectionRotatedV);
        cl::mapSVM(camUpVectorsV);
        cl::mapSVM(camRightVectorsV);
        // Save into global containers
        for(int i = 0; i < numCamPos; i++)
        {
            camDir[counter][i].setX(camDirectionRotatedV[i].x);
            camDir[counter][i].setY(camDirectionRotatedV[i].y);
            camDir[counter][i].setZ(camDirectionRotatedV[i].z);

            camUp[counter][i].setX(camUpVectorsV[i].x);
            camUp[counter][i].setY(camUpVectorsV[i].y);
            camUp[counter][i].setZ(camUpVectorsV[i].z);

            camRight[counter][i].setX(camRightVectorsV[i].x);
            camRight[counter][i].setY(camRightVectorsV[i].y);
            camRight[counter][i].setZ(camRightVectorsV[i].z);
        }

        // increment the counter
        counter++;
    }
}

void VisibilityCheck::setup_g()
{
    // Create the required input and output svm data structures
    cl::SVMAllocator<int, cl::SVMTraitCoarse<>> svmAlloc;

    // SVM allocations for control points checking kernel
    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> contPoints(svmAlloc);
    // SVM allocations for vehicle points self occlusion checking kernel
    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> vehPoints(svmAlloc);

    // Create the kernel functors
    auto frustumCalcKernel =
            cl::KernelFunctor<
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float16, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                cl_float3
            >(camProgram, "frustumCalc");

    auto occlusionKernel =
            cl::KernelFunctor<
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                cl_float16,
                cl_float3,
                cl_float3,
                std::vector<cl_int, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&
            >(camProgram, "checkSelfOcclusion");

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
    camParams.x = onePlus6.hFar;
    camParams.y = onePlus6.vFar;
    camParams.z = onePlus6.z_f;

    /*// Open a file to write the calculated g matrix to binary files
    //QString fileName("/media/anirudh/Data/Documents/PhD/Qt_projects/virtual_machine/MIP_optimizer/gmat32Down/g" + QString::number(1) + ".dat");
    QString fileName("/media/pv/DATA/PhD/Qt_projects/ocp_simulation/resources/g" + QString::number(1) + ".dat");
    QFile file(fileName);
    file.open(QIODevice::WriteOnly);
    QDataStream out(&file);*/

    int numOrientations = camDir.size();
    int numCamPos = camPos.size();
    int numBackPos = backgroundVox.size();
    int numVehicle = vehicleVox.size();
    //qDebug() << numOrientations;

    // initialize svm vectors for calls to background point checking kernel (done outside the loop because it is constant set of points)
    contPoints.clear();
    contPoints.resize(numBackPos);
    for(int i = 0; i < numBackPos; i++)
    {
        contPoints[i].x = backgroundVox[i].x();
        contPoints[i].y = backgroundVox[i].y();
        contPoints[i].z = backgroundVox[i].z();
    }
    cl::unmapSVM(contPoints);

    // Initialize svm vector for call to self occlusion checking kernel
    vehPoints.clear();
    vehPoints.resize(numVehicle);
    for(int i = 0; i < numVehicle; i++)
    {
        vehPoints[i].x = vehicleVox[i].x();
        vehPoints[i].y = vehicleVox[i].y();
        vehPoints[i].z = vehicleVox[i].z();
    }
    cl::unmapSVM(vehPoints);

    // A vector to store indices of covered background points for each accepted camera point
    QVector<QVector<int>> covIndVec;

    // Loop over the number of orientations
    for(int k = 0; k < numOrientations; k++)
    {
        // Initialize the 3D 'gMatrix'
        //boost::numeric::ublas::compressed_matrix<bool> g_temp(numCamPos, numBackPos);
        //gMatrix.append(g_temp);

        // SVM allocations for frustum calculation kernel
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camPoints(svmAlloc);        // input vector containing camera locations
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camDirection(svmAlloc);     // input vector containing camera directions
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camUpVectors(svmAlloc);     // input svm buffer containing camera up vectors
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camRightVectors(svmAlloc);  // input svm buffer containing camera right vectors
        std::vector<cl_float16, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camFrustum(svmAlloc);      // output vector to store the calculated frustum
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> farPlanePoint(svmAlloc);    // output vector to store far plane top left point coordinates

        // initialize the input svm buffers for kernel call to calculate the camera frustums
        camPoints.clear();
        camPoints.resize(numCamPos);
        camDirection.clear();
        camDirection.resize(numCamPos);
        camUpVectors.clear();
        camUpVectors.resize(numCamPos);
        camRightVectors.clear();
        camRightVectors.resize(numCamPos);
        // initialize output svm buffer for fustum calculation kernel
        camFrustum.clear();
        camFrustum.resize(numCamPos);
        farPlanePoint.clear();
        farPlanePoint.resize(numCamPos);
        // Populate the input buffers with data
        for(int i = 0; i < numCamPos; i++)
        {
            // camera positions
            camPoints[i].x = camPos[i].x();
            camPoints[i].y = camPos[i].y();
            camPoints[i].z = camPos[i].z();
            // camera view directions
            camDirection[i].x = camDir[k][i].x();
            camDirection[i].y = camDir[k][i].y();
            camDirection[i].z = camDir[k][i].z();
            // camera up vectors
            camUpVectors[i].x = camUp[k][i].x();
            camUpVectors[i].y = camUp[k][i].y();
            camUpVectors[i].z = camUp[k][i].z();
            // camera right vectors
            camRightVectors[i].x = camRight[k][i].x();
            camRightVectors[i].y = camRight[k][i].y();
            camRightVectors[i].z = camRight[k][i].z();
        }
        // unmap all svm vectors
        cl::unmapSVM(camPoints);
        cl::unmapSVM(camDirection);
        cl::unmapSVM(camUpVectors);
        cl::unmapSVM(camRightVectors);
        cl::unmapSVM(camFrustum);
        cl::unmapSVM(farPlanePoint);

        // Call the kernel function to calculate camera view frustums
        cl_int error;
        frustumCalcKernel(
                    cl::EnqueueArgs(cl::NDRange(numCamPos)),
                    camPoints,
                    camDirection,
                    camUpVectors,
                    camRightVectors,
                    camFrustum,
                    farPlanePoint,
                    camParams,
                    error
                    );

        // map output svm vectors
        //cl::mapSVM(camFrustum);
        //cl::mapSVM(farPlanePoint);

        // loop over the number of camera positions to check each gainst all the vehicle points
        for(int i = 0; i < numCamPos; i++)
        {
            // Boolean variables to check if a cam poisition passes vehicle occlusion check
            // As well as if it passes the min coverage ctiteria
            //bool selfOccCheck = false, minCovCheck = false;
            // Occlusion Checking
            std::vector<cl_int, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> occlusion(svmAlloc);
            // initialize the output occlusion buffer
            occlusion.clear();
            occlusion.resize(numVehicle);
            cl::unmapSVM(occlusion);

            // call the kernel function to check self occlusions
            cl_int errorOcc;
            occlusionKernel(
                        cl::EnqueueArgs(cl::NDRange(numVehicle)),
                        vehPoints,
                        camFrustum[i],
                        camPoints[i],
                        farPlanePoint[i],
                        occlusion,
                        errorOcc
                        );
            // Gather output
            cl::mapSVM(occlusion);
            // Get the number of vehicle points that lie inside this camera's frustum
            int vehicleSum = 0;
            for(int j = 0; j < numVehicle; j++)
                vehicleSum += occlusion[j];

            // If the self occlusion is not passed then skip the iteration and do remaining checks
            // If not, then mark this position as it cleared self occlusion check
            if(vehicleSum > numVehAllowed)
                continue;

            // Background point coverage checks
            std::vector<cl_int, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> outMatG(svmAlloc);
            outMatG.resize(numBackPos);
            cl::unmapSVM(outMatG);
            // A temporary vector to store indices of occupied background points to use later for creating gMatrix
            QVector<int> backOccPointVec;
            // call the kernel function to check all the background points against each camera position and it's frustum
            cl_int error;
            pointCheckKernel(
                        cl::EnqueueArgs(cl::NDRange(numBackPos)),
                        contPoints,
                        camFrustum[i],
                        camPoints[i],
                        farPlanePoint[i],
                        outMatG,
                        error
                        );
            // unmap output svm
            cl::mapSVM(outMatG);
            // Count of occupied background points
            int backOccCount = 0;
            for (int j = 0; j < numBackPos; j++)
            {
                if(outMatG[j] == 1)
                {
                    // Add the count
                    backOccCount += 1;
                    // Add the index in temporary vectos
                    backOccPointVec.append(j);
                }
            }

            // If number of covered points < threshold, skip the iteration
            if(backOccCount < minReqBackCov)
                continue;

            // If we reach this point in the iteration, it means both self occlusion and min. coverage are satisifed
            // So accept the cam position+orientation and save the covered points for creating gMatrix later
            indices1D.append(QVector<int>({i,k}));
            covIndVec.append(backOccPointVec);
        }
    }

    int numSelected = indices1D.size();
    // Now that visibility checks are complete, setup gMatrix with the selected points
    // Also write it into file
    /*QFile gfile(gFileName);
    gfile.open(QIODevice::WriteOnly);
    QDataStream out(&gfile);*/
    gMatrix2D = boost::numeric::ublas::compressed_matrix<bool>(numSelected, numBackPos);
    // Loop over the 'covIndVec' and mark the occupied points for each selected camera
    for(int i = 0; i < numSelected; i++)
    {
        for(int j = 0; j < covIndVec[i].size(); j++)
        {
            //qDebug() << "Current index (" << i << "," << covIndVec[i][j] <<")";
            gMatrix2D(i, covIndVec[i][j]) = 1;
            //out << (qint32)i << (qint32)covIndVec[i][j];
        }
    }
    /*out.~QDataStream();
    gfile.close();
    // Also write indices1D vector to another file
    QFile indFile(indFileName);
    indFile.open(QIODevice::WriteOnly);
    QDataStream indOut(&indFile);
    for(int i = 0; i < indices1D.size(); i++)
        indOut << (qint32)indices1D[i][0] << (qint32)indices1D[i][1];
    indOut.~QDataStream();
    indFile.close();*/
    //qDebug() << "Debugger hold";
    // close the file after processing all the camera positions
    //file.close();
}

void VisibilityCheck::getMatrixAnd1D(boost::numeric::ublas::compressed_matrix<bool> &gMat2D, QVector<QVector<int>> &index1D)
{
    /* For now size of index1D is camPos times orientations
     * But we can simply copy a vector that can be precomputed during self-occlusion checking
     */
    /*
    int numCamPos = camPos.size();
    int numRot = getNumRotations();
    for(int i = 0; i < numCamPos; i++)
    {
        for(int k = 0; k < numRot; k++)
        {
            index1D.append(QVector<int>({i,k}));
        }
    }
    // Verify size is correct
    if(index1D.size() != numCamPos*numRot)
        qDebug() << "SIze mis-match in VisibilityCheck::getMatrixAnd1D function!!";

    // Setup 2D gMatrix
    int size1D = index1D.size();
    int numContr = backgroundVox.size();
    // initialize gMat2D
    gMat2D.resize(numContr, size1D, false);
    for(int j = 0; j < numContr; j++)
    {
        for(int i = 0; i < size1D; i++)
        {
            gMat2D(j, i) = gMatrix[index1D[i][1]](index1D[i][0], j);
        }
    }*/

    // The above code is unnecessary because now these are already constructed during visibility checks
    gMat2D = gMatrix2D;
    index1D = indices1D;
}

void VisibilityCheck::getSolutionoccupation(QVector<QVector<int>> &camSolution,
                                            QVector<QVector3D> &backVoxPoints,
                                            boost::numeric::ublas::compressed_matrix<bool> &gMat2D)
{
    // Create the required input and output svm data structures
    cl::SVMAllocator<int, cl::SVMTraitCoarse<>> svmAlloc;

    // Create the kernel functors
    auto frustumCalcKernel =
            cl::KernelFunctor<
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
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
    camParams.x = onePlus6.hFar;
    camParams.y = onePlus6.vFar;
    camParams.z = onePlus6.z_f;

    // Inititalization
    int numBackPos = backVoxPoints.size();

    // initialize svm vectors for calls to background point checking kernel (done outside the loop because it is constant set of points)
    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> contPoints(svmAlloc);
    contPoints.clear();
    contPoints.resize(numBackPos);
    for(int i = 0; i < numBackPos; i++)
    {
        contPoints[i].x = backVoxPoints[i].x();
        contPoints[i].y = backVoxPoints[i].y();
        contPoints[i].z = backVoxPoints[i].z();
    }
    cl::unmapSVM(contPoints);

    // Initialize gMat2D
    gMat2D.clear();
    //gMat2D(camSolution.size(), numBackPos);
    gMat2D = boost::numeric::ublas::compressed_matrix<bool>(camSolution.size(), numBackPos);

    // Loop over the number of cameras
    for(int n = 0; n < camSolution.size(); n++)
    {
        // SVM allocations for frustum calculation kernel
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camPoints(svmAlloc);        // input vector containing camera locations
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camDirection(svmAlloc);     // input vector containing camera directions
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camUpVectors(svmAlloc);     // input svm buffer containing camera up vectors
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camRightVectors(svmAlloc);  // input svm buffer containing camera right vectors
        std::vector<cl_float16, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camFrustum(svmAlloc);      // output vector to store the calculated frustum
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> farPlanePoint(svmAlloc);    // output vector to store far plane top left point coordinates

        // SVM allocations for control points checking kernel
        std::vector<cl_int, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> outMatG(svmAlloc);

        // initialize the input svm buffers for kernel call to calculate the camera frustums
        camPoints.clear();
        camPoints.resize(1);
        camDirection.clear();
        camDirection.resize(1);
        camUpVectors.clear();
        camUpVectors.resize(1);
        camRightVectors.clear();
        camRightVectors.resize(1);
        // initialize output svm buffer for fustum calculation kernel
        camFrustum.clear();
        camFrustum.resize(1);
        farPlanePoint.clear();
        farPlanePoint.resize(1);

        // Populate the input buffers with data
        // camera positions
        camPoints[0].x = camPos[camSolution[n][0]].x();
        camPoints[0].y = camPos[camSolution[n][0]].y();
        camPoints[0].z = camPos[camSolution[n][0]].z();
        // camera view directions
        camDirection[0].x = camDir[camSolution[n][1]][camSolution[n][0]].x();
        camDirection[0].y = camDir[camSolution[n][1]][camSolution[n][0]].y();
        camDirection[0].z = camDir[camSolution[n][1]][camSolution[n][0]].z();
        // camera up vectors
        camUpVectors[0].x = camUp[camSolution[n][1]][camSolution[n][0]].x();
        camUpVectors[0].y = camUp[camSolution[n][1]][camSolution[n][0]].y();
        camUpVectors[0].z = camUp[camSolution[n][1]][camSolution[n][0]].z();
        // camera right vectors
        camRightVectors[0].x = camRight[camSolution[n][1]][camSolution[n][0]].x();
        camRightVectors[0].y = camRight[camSolution[n][1]][camSolution[n][0]].y();
        camRightVectors[0].z = camRight[camSolution[n][1]][camSolution[n][0]].z();
        // unmap all svm vectors
        cl::unmapSVM(camPoints);
        cl::unmapSVM(camDirection);
        cl::unmapSVM(camUpVectors);
        cl::unmapSVM(camRightVectors);
        cl::unmapSVM(camFrustum);
        cl::unmapSVM(farPlanePoint);

        // Call the kernel function to calculate camera view frustums
        cl_int error;
        frustumCalcKernel(
                    cl::EnqueueArgs(cl::NDRange(1)),
                    camPoints,
                    camDirection,
                    camUpVectors,
                    camRightVectors,
                    camFrustum,
                    farPlanePoint,
                    camParams,
                    error
                    );
        // map output svm vectors
        //cl::mapSVM(camFrustum);
        //cl::mapSVM(farPlanePoint);

        // initialize the output svm buffer
        outMatG.clear();
        outMatG.resize(numBackPos);
        // unmap svm
        //cl::unmapSVM(camFrustum);
        //cl::unmapSVM(farPlanePoint);
        cl::unmapSVM(outMatG);

        // call the kernel function to check all the background points against each camera position and it's frustum
        pointCheckKernel(
                    cl::EnqueueArgs(cl::NDRange(numBackPos)),
                    contPoints,
                    camFrustum[0],
                    camPoints[0],
                    farPlanePoint[0],
                    outMatG,
                    error
                    );

        // unmap output svm
        cl::mapSVM(outMatG);

        // Write the g values variable into the output vector
        for (int j = 0; j < numBackPos; j++)
        {
            if(outMatG[j] == 1)
            {
                // Add the points in gmat2D
                gMat2D(n, j) = 1;
            }
        }
    }
}

void VisibilityCheck::getBackPointsSet(QVector<QVector3D> &camPosVec,
                                       QVector3D &camDirVector,
                                       QVector3D &camUpVector,
                                       QVector3D &camRightVector,
                                       QVector<QVector3D> &backVoxPoints,
                                       QVector<QVector3D> &occupiedBackPoints)
{
    // Create the required input and output svm data structures
    cl::SVMAllocator<int, cl::SVMTraitCoarse<>> svmAlloc;
    // Create the kernel functors
    auto frustumCalcKernel =
            cl::KernelFunctor<
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float16, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                cl_float3
            >(camProgram, "frustumCalc");

    // kernel Functor for point checking
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
    camParams.x = onePlus6.hFar;
    camParams.y = onePlus6.vFar;
    camParams.z = onePlus6.z_f;

    // Inititalization
    int numBackPos = backVoxPoints.size();
    // initialize svm vectors for calls to background point checking kernel (done outside the loop because it is constant set of points)
    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> contPoints(svmAlloc);
    contPoints.clear();
    contPoints.resize(numBackPos);
    for(int i = 0; i < numBackPos; i++)
    {
        contPoints[i].x = backVoxPoints[i].x();
        contPoints[i].y = backVoxPoints[i].y();
        contPoints[i].z = backVoxPoints[i].z();
    }
    cl::unmapSVM(contPoints);

    // Create a vector of booleans to mark points occupied by at least one camera position
    // Initialize the vector with 0s
    QVector<bool> backPointsOccupation(numBackPos, false);

    int numCams = camPosVec.size();
    // Loop over the number of cameras
    for(int n = 0; n < numCams; n++)
    {
        // SVM allocations for frustum calculation kernel
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camPoints(svmAlloc);        // input vector containing camera locations
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camDirection(svmAlloc);     // input vector containing camera directions
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camUpVectors(svmAlloc);     // input svm buffer containing camera up vectors
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camRightVectors(svmAlloc);  // input svm buffer containing camera right vectors
        std::vector<cl_float16, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camFrustum(svmAlloc);      // output vector to store the calculated frustum
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> farPlanePoint(svmAlloc);    // output vector to store far plane top left point coordinates
        // SVM allocations for control points checking kernel
        std::vector<cl_int, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> outMatG(svmAlloc);

        // initialize the input svm buffers for kernel call to calculate the camera frustums
        camPoints.clear();
        camPoints.resize(1);
        camDirection.clear();
        camDirection.resize(1);
        camUpVectors.clear();
        camUpVectors.resize(1);
        camRightVectors.clear();
        camRightVectors.resize(1);
        // initialize output svm buffer for fustum calculation kernel
        camFrustum.clear();
        camFrustum.resize(1);
        farPlanePoint.clear();
        farPlanePoint.resize(1);

        // Populate the input buffers with data
        // camera positions
        camPoints[0].x = camPosVec[n].x();
        camPoints[0].y = camPosVec[n].y();
        camPoints[0].z = camPosVec[n].z();
        // camera view directions
        camDirection[0].x = camDirVector.x();
        camDirection[0].y = camDirVector.y();
        camDirection[0].z = camDirVector.z();
        // camera up vectors
        camUpVectors[0].x = camUpVector.x();
        camUpVectors[0].y = camUpVector.y();
        camUpVectors[0].z = camUpVector.z();
        // camera right vectors
        camRightVectors[0].x = camRightVector.x();
        camRightVectors[0].y = camRightVector.y();
        camRightVectors[0].z = camRightVector.z();
        // unmap all svm vectors
        cl::unmapSVM(camPoints);
        cl::unmapSVM(camDirection);
        cl::unmapSVM(camUpVectors);
        cl::unmapSVM(camRightVectors);
        cl::unmapSVM(camFrustum);
        cl::unmapSVM(farPlanePoint);
        // Call the kernel function to calculate camera view frustums
        cl_int error;
        frustumCalcKernel(
                    cl::EnqueueArgs(cl::NDRange(1)),
                    camPoints,
                    camDirection,
                    camUpVectors,
                    camRightVectors,
                    camFrustum,
                    farPlanePoint,
                    camParams,
                    error
                    );

        // initialize the output svm buffer
        outMatG.clear();
        outMatG.resize(numBackPos);
        cl::unmapSVM(outMatG);

        // call the kernel function to check all the background points against each camera position and it's frustum
        pointCheckKernel(
                    cl::EnqueueArgs(cl::NDRange(numBackPos)),
                    contPoints,
                    camFrustum[0],
                    camPoints[0],
                    farPlanePoint[0],
                    outMatG,
                    error
                    );

        // unmap output svm
        cl::mapSVM(outMatG);
        // Write the g values variable into the output vector
        for (int j = 0; j < numBackPos; j++)
        {
            // Boolean or operation to mark that this point is occupied by at least one camera
            backPointsOccupation[j] = backPointsOccupation[j] || outMatG[j];
        }
    }
    // Setup the output vector
    for(int j = 0; j < numBackPos; j++)
    {
        // if the value in boolean vector is true, add the point into output
        if(backPointsOccupation[j])
            occupiedBackPoints.append(backVoxPoints[j]);
    }
}

void VisibilityCheck::getCamCoverage(QVector<QVector3D> &camPosVec,
                                     QVector<QVector3D> &camDirVec,
                                     QVector<QVector3D> &camUpVec,
                                     QVector<QVector3D> &camRightVec,
                                     QVector<QVector3D> &backVoxPoints,
                                     QVector<QVector<int>> &occupiedBackIndices)
{
    // Create the required input and output svm data structures
    cl::SVMAllocator<int, cl::SVMTraitCoarse<>> svmAlloc;

    // Create the kernel functors
    auto frustumCalcKernel =
            cl::KernelFunctor<
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float16, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                cl_float3
            >(camProgram, "frustumCalc");

    // kernel Functor for point checking
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
    camParams.x = onePlus6.hFar;
    camParams.y = onePlus6.vFar;
    camParams.z = onePlus6.z_f;

    // Inititalization
    int numBackPos = backVoxPoints.size();

    // initialize svm vectors for calls to background point checking kernel (done outside the loop because it is constant set of points)
    std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> contPoints(svmAlloc);
    contPoints.clear();
    contPoints.resize(numBackPos);
    for(int i = 0; i < numBackPos; i++)
    {
        contPoints[i].x = backVoxPoints[i].x();
        contPoints[i].y = backVoxPoints[i].y();
        contPoints[i].z = backVoxPoints[i].z();
    }
    cl::unmapSVM(contPoints);

    int numCams = camPosVec.size();
    // Initialize the output vector for occupied background points
    occupiedBackIndices.resize(numCams);


    // Loop over the number of cameras
    for(int n = 0; n < numCams; n++)
    {
        // SVM allocations for frustum calculation kernel
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camPoints(svmAlloc);        // input vector containing camera locations
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camDirection(svmAlloc);     // input vector containing camera directions
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camUpVectors(svmAlloc);     // input svm buffer containing camera up vectors
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camRightVectors(svmAlloc);  // input svm buffer containing camera right vectors
        std::vector<cl_float16, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> camFrustum(svmAlloc);      // output vector to store the calculated frustum
        std::vector<cl_float3, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> farPlanePoint(svmAlloc);    // output vector to store far plane top left point coordinates

        // SVM allocations for control points checking kernel
        std::vector<cl_int, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> outMatG(svmAlloc);

        // initialize the input svm buffers for kernel call to calculate the camera frustums
        camPoints.clear();
        camPoints.resize(1);
        camDirection.clear();
        camDirection.resize(1);
        camUpVectors.clear();
        camUpVectors.resize(1);
        camRightVectors.clear();
        camRightVectors.resize(1);
        // initialize output svm buffer for fustum calculation kernel
        camFrustum.clear();
        camFrustum.resize(1);
        farPlanePoint.clear();
        farPlanePoint.resize(1);

        // Populate the input buffers with data
        // camera positions
        camPoints[0].x = camPosVec[n].x();
        camPoints[0].y = camPosVec[n].y();
        camPoints[0].z = camPosVec[n].z();
        // camera view directions
        camDirection[0].x = camDirVec[n].x();
        camDirection[0].y = camDirVec[n].y();
        camDirection[0].z = camDirVec[n].z();
        // camera up vectors
        camUpVectors[0].x = camUpVec[n].x();
        camUpVectors[0].y = camUpVec[n].y();
        camUpVectors[0].z = camUpVec[n].z();
        // camera right vectors
        camRightVectors[0].x = camRightVec[n].x();
        camRightVectors[0].y = camRightVec[n].y();
        camRightVectors[0].z = camRightVec[n].z();
        // unmap all svm vectors
        cl::unmapSVM(camPoints);
        cl::unmapSVM(camDirection);
        cl::unmapSVM(camUpVectors);
        cl::unmapSVM(camRightVectors);
        cl::unmapSVM(camFrustum);
        cl::unmapSVM(farPlanePoint);

        // Call the kernel function to calculate camera view frustums
        cl_int error;
        frustumCalcKernel(
                    cl::EnqueueArgs(cl::NDRange(1)),
                    camPoints,
                    camDirection,
                    camUpVectors,
                    camRightVectors,
                    camFrustum,
                    farPlanePoint,
                    camParams,
                    error
                    );

        // initialize the output svm buffer
        outMatG.clear();
        outMatG.resize(numBackPos);
        cl::unmapSVM(outMatG);

        // call the kernel function to check all the background points against each camera position and it's frustum
        pointCheckKernel(
                    cl::EnqueueArgs(cl::NDRange(numBackPos)),
                    contPoints,
                    camFrustum[0],
                    camPoints[0],
                    farPlanePoint[0],
                    outMatG,
                    error
                    );

        // unmap output svm
        cl::mapSVM(outMatG);
        // Write the g values variable into the output vector
        for (int j = 0; j < numBackPos; j++)
        {
            if(outMatG[j] == 1)
            {
                occupiedBackIndices[n].append(j);
            }
        }
    }
}

void VisibilityCheck::getCamVectors(QVector<int> camIndices, QVector3D &solCamDir, QVector3D &solCamUp, QVector3D &solCamRight)
{
    solCamDir = camDir[camIndices[1]][camIndices[0]];
    solCamUp = camUp[camIndices[1]][camIndices[0]];
    solCamRight = camRight[camIndices[1]][camIndices[0]];
}

void VisibilityCheck::getCamFrustum(QVector<QVector3D> &camPosVec,
                                    QVector<QVector3D> &camDirVec,
                                    QVector<QVector3D> &camUpVec,
                                    QVector<QVector3D> &camRightVec,
                                    QVector<QVector<QVector3D>> &camFOVs)
{
    // Setup the camera parameters (hfar, vfar and z_f) to pass it to the kernel function
    float camParams[3];
    camParams[0] = onePlus6.hFar;
    camParams[1] = onePlus6.vFar;
    camParams[2] = onePlus6.z_f;
    // resize output vector
    camFOVs.resize(camPosVec.size());

    // Loop over the number of cameras
    for(int n = 0; n < camPosVec.size(); n++)
    {
        // Get the current camera positiona and orientation vectors
        QVector3D currCamPose = camPosVec[n];
        QVector3D currCamDir = camDirVec[n];
        QVector3D currCamUp = camUpVec[n];
        QVector3D currCamRight = camRightVec[n];

        // For following calculations refer to where frustum calc kernel string is defined
        QVector3D farCenter = currCamPose + currCamDir * camParams[2];
        QVector3D far_tl = farCenter + currCamUp*camParams[1] - currCamRight*camParams[0];
        QVector3D far_bl = far_tl - currCamUp*2*camParams[1];
        QVector3D far_br = far_bl + currCamRight*2*camParams[0];
        QVector3D far_tr = far_br + currCamUp*2*camParams[1];
        //
        /* Arrange the points as QVecto3Ds in the output vector in following order
         * camDirection, top left, bottom left, bottom right, top right
         */
        camFOVs[n].append({far_tl, far_bl, far_br, far_tr});
    }
}

void VisibilityCheck::computeNeighboursGPU(boost::numeric::ublas::compressed_matrix<bool> &gMat2D_trans, std::vector<std::vector<int> > &N_j)
{
    int camPos = gMat2D_trans.size2();
    int controlPoints = gMat2D_trans.size1();
    N_j.clear();
    N_j.resize(camPos);

    // Create the required input and output svm data structures
    cl::SVMAllocator<int, cl::SVMTraitCoarse<>> svmAlloc;
    // Create the kernel functors
    auto computeNeighboursKernel =
            cl::KernelFunctor<
                std::vector<cl_int, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&,
                cl_int,
                cl_int,
                std::vector<cl_int, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>>&
            >(camProgram, "computeNeighbourhood");

    cl_int numControl = controlPoints;
    // setup gMat as a 1D vector
    std::vector<cl_int, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> gMat_flat(svmAlloc);
    gMat_flat.clear();
    gMat_flat.resize(numControl*camPos);
    for(int i = 0; i < camPos; i++)
    {
        for(int j = 0 ; j < controlPoints; j++)
        {
            gMat_flat[controlPoints*i + j] = gMat2D_trans(j,i);
        }
    }
    cl::unmapSVM(gMat_flat);

    // Loop over the set of cameras
    for(int i = 0; i < camPos; i++)
    {
        cl_int currCam = i;
        // vector to hold output - 1 if the camera is a neighbour, 0 if not
        std::vector<cl_int, cl::SVMAllocator<int, cl::SVMTraitCoarse<>>> neighbourOut(svmAlloc);
        neighbourOut.clear();
        neighbourOut.resize(camPos);
        cl::unmapSVM(neighbourOut);
        // Call kernel
        cl_int error;
        computeNeighboursKernel(
                    cl::EnqueueArgs(cl::NDRange(camPos, controlPoints)),
                    gMat_flat,
                    currCam,
                    numControl,
                    neighbourOut,
                    error
                    );
        cl::mapSVM(neighbourOut);
        // assign the index as a neighbour if value is 1
        for(int j = 0; j < neighbourOut.size(); j++)
        {
            // skip the current index as every camera is a neighbour to itself
            if(j !=i)
            {
                if(neighbourOut[j]==1)
                    N_j[i].push_back(j);
            }
        }
        //qDebug() << "Debug holder!";
    }
}
