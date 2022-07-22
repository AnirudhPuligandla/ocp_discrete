#include "simulatedobjects.h"

SimulatedObjects::SimulatedObjects(int objectChoice)
{
    // First define the dimensions of the volume, vehicle, etc. based on the selected size category
    switch (sizeMark)
    {
    case 0 :
        // 0 refers to tiny size with dimensions (7x5x5)
        // Define an extent of about 60 voxels in each dimension
        // This helps in the way that the same volume can be used for other processing
        extent[0] = 0;
        extent[1] = 59;
        extent[2] = 0;
        extent[3] = 9;
        extent[4] = 0;
        extent[5] = 59;
        // Define H=6, L=9, W=5 of the vehicle
        objMaxL = 7;
        objMaxW = 5;
        objMaxH = 5;
        break;
    case 1 :
        // 1 refers to small size with dimensions (9x5x5)
        // volume is same as the tiny case
        extent[0] = 0;
        extent[1] = 59;
        extent[2] = 0;
        extent[3] = 9;
        extent[4] = 0;
        extent[5] = 59;
        // Define H=6, L=9, W=5 of the vehicle
        objMaxL = 9;
        objMaxW = 5;
        objMaxH = 5;
        break;
    case 2 :
        // 2 refers to medium size with dimensions (11x7x7)
        // add 10 to extent dimensions in x-z and 5 in y directions
        // This should be able largely accomodate the 2 voxel increase in each direction
        extent[0] = 0;
        extent[1] = 69;
        extent[2] = 0;
        extent[3] = 13;
        extent[4] = 0;
        extent[5] = 69;

        objMaxL = 11;
        objMaxW = 7;
        objMaxH = 7;
        break;
    case 3 :
        // 3 refers to large size with dimensions (17x13x13)
        extent[0] = 0;
        extent[1] = 99;
        extent[2] = 0;
        extent[3] = 19;
        extent[4] = 0;
        extent[5] = 99;

        objMaxL = 17;
        objMaxW = 13;
        objMaxH = 13;
        break;
    case 4 :
        // 4 refers to huge size with dimensions (23x19x19)
        extent[0] = 0;
        extent[1] = 129;
        extent[2] = 0;
        extent[3] = 29;
        extent[4] = 0;
        extent[5] = 129;

        objMaxL = 23;
        objMaxW = 19;
        objMaxH = 19;
        break;
    }

    switch (objectChoice)
    {
    case 1 :
        // Simple car is of dimensions 5x4x9 starting at height of 2 voxels from ground
        simpleCar();
        break;
    case 2 :
        // Complex car. same as simple car but the top part modified with slanted planes
        van();
        break;
    case 3 :
        // hatchback model, fron is slanted plane, rear ends at full lenth as a box
        hatchBack();
        break;
    case 4 :
        // Truck model, the top part is in front
        truck();
        break;
    case 5 :
        // Bus model, simple bottom less box
        bus();
        break;
    }
}

void SimulatedObjects::simpleCar()
{
    // Create a simple car-like structure with two boxes in x-z planes with length across z-dimension
    voxSimObject = vtkSmartPointer<vtkStructuredPoints>::New();
    voxSimObject->SetExtent(extent);
    voxSimObject->AllocateScalars(VTK_UNSIGNED_INT, 1);
    // get volume dimensions
    int volDims[3];
    voxSimObject->GetDimensions(volDims);
    // Define the center
    int objCenter[3] = {volDims[0]/2,volDims[1]/2,volDims[2]/2};
    // A simple car is defined using 9 planes, 4 for the body, 4 for top part and 1 for the roof
    QVector<QVector<int>> xLims, yLims, zLims;
    QVector<QVector3D> planeNorms;
    // Define limits in y direction
    int yStart = 2;
    int yLim1 = yStart + (objMaxH - 1)/2 - 1;
    int yLim2 = yStart + (objMaxH - 1)/2;
    int yLim3 = yLim1 + (objMaxH - 1)/2;
    int yEnd = objMaxH + 1;
    // Define halfs in x (width) and z (length) directions
    int xHalf = (objMaxW - 1)/2;
    int zHalf = (objMaxL - 1)/2;
    // Define length of the top part
    // Ideally it should be a third of the length, but it needs to be set to nearest odd number that is larger
    int topLength;
    float lThirds = objMaxL/3.0;
    if(int(ceil(lThirds)) % 2 == 0)
        topLength = ceil(lThirds) + 1;
    else
        topLength = ceil(lThirds);
    int topHalfL = (topLength - 1)/2;
    // left y-z plane of the body (dims = 9lx2h)
    xLims.append(QVector<int>({objCenter[0] - xHalf, objCenter[0] - xHalf}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(-1.0f, 0.0f, 0.0f));                    // Normal to y-z plane in -x dir
    // Right y-z plane
    xLims.append(QVector<int>({objCenter[0] + xHalf, objCenter[0] + xHalf}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(1.0f, 0.0f, 0.0f));                     // Normal to y-z plane in +x dir
    // Front x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf -  1)}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] - zHalf}));
    planeNorms.append(QVector3D(0.0f, 0.0f, -1.0f));                     // Normal to x-y plane in -z dir
    // Rear x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] + zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(0.0f, 0.0f, 1.0f));                     // Normal to x-y plane in +z dir
    // top left y-z plane (dims = 3lx3h)
    xLims.append(QVector<int>({objCenter[0] - xHalf, objCenter[0] - xHalf}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] - topHalfL, objCenter[2] + topHalfL}));
    planeNorms.append(QVector3D(-1.0f, 0.0f, 0.0f));                    // Normal to y-z plane in -x dir
    // top right y-z plane
    xLims.append(QVector<int>({objCenter[0] + xHalf, objCenter[0] + xHalf}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] - topHalfL, objCenter[2] + topHalfL}));
    planeNorms.append(QVector3D(1.0f, 0.0f, 0.0f));                     // Normal to y-z plane in +x dir
    // top front x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] - topHalfL, objCenter[2] - topHalfL}));
    planeNorms.append(QVector3D(0.0f, 0.0f, -1.0f));                     // Normal to x-y plane in -z dir
    // top rear x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] + topHalfL, objCenter[2] + topHalfL}));
    planeNorms.append(QVector3D(0.0f, 0.0f, 1.0f));                     // Normal to x-y plane in +z dir
    // Roof plane. construct it one voxel over the height
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yEnd, yEnd}));
    zLims.append(QVector<int>({objCenter[2] - (topHalfL - 1), objCenter[2] + (topHalfL - 1)}));
    // We cannot have normal looking straight up. So make slight perturbation
    planeNorms.append(QVector3D(0.0f, 0.92f, 0.0f));

    // intialize vectorIndices vector
    vectorIndices.clear();
    vectorIndices.resize(volDims[0]*volDims[1]*volDims[2]);
    // Loop over the planes and fill the vehicle voxels
    int totalPlanes = xLims.size();
    camPoints.clear();
    camNormals.clear();
    // Counter to keep track of number of camera voxels being added
    int camVoxCount = 0;

    for(int currPlane = 0; currPlane < totalPlanes; currPlane++)
    {
        int x1 = xLims[currPlane][0], x2 = xLims[currPlane][1];
        int y1 = yLims[currPlane][0], y2 = yLims[currPlane][1];
        int z1 = zLims[currPlane][0], z2 = zLims[currPlane][1];
        QVector3D currNormal = planeNorms[currPlane];
        // loop over the voxel comprising plane
        for(int x = x1; x <= x2; x++)
        {
            for(int y = y1; y <= y2; y++)
            {
                for(int z = z1; z <= z2; z++)
                {
                    // get the voxel from volume and set it to a value 2
                    unsigned int* voxVal = static_cast<unsigned int*>(voxSimObject->GetScalarPointer(x,y,z));
                    *voxVal = 2;
                    // Add the voxels into camera points containers
                    camPoints.append(QVector3D(x,y,z));
                    // Also add the points to vehicle voxels container
                    vehVox.append(QVector3D(x,y,z));
                    // Correspondingly append the normal vector for each voxel on this plane
                    camNormals.append(currNormal);
                    // Mark the count at this vector index
                    vectorIndices[x + (y * extent[1]) + (z * extent[1] * extent[3])] = camVoxCount;
                    camVoxCount++;
                }
            }
        }
    }
}

void SimulatedObjects::van()
{
    // Create a simple car-like structure with two boxes in x-z planes with length across z-dimension
    voxSimObject = vtkSmartPointer<vtkStructuredPoints>::New();
    voxSimObject->SetExtent(extent);
    voxSimObject->AllocateScalars(VTK_UNSIGNED_INT, 1);
    // get volume dimensions
    int volDims[3];
    voxSimObject->GetDimensions(volDims);
    // Define the center
    int objCenter[3] = {volDims[0]/2,volDims[1]/2,volDims[2]/2};
    // A simple car is defined using 9 planes, 4 for the body, 4 for top part and 1 for the roof
    QVector<QVector<int>> xLims, yLims, zLims;
    QVector<QVector3D> planeNorms;
    // Define limits in y direction
    int yStart = 2;
    int yLim1 = yStart + (objMaxH - 1)/2 - 1;
    int yLim2 = yStart + (objMaxH - 1)/2;
    int yLim3 = yLim1 + (objMaxH - 1)/2;
    int yEnd = objMaxH + 1;
    // Define halfs in x (width) and z (length) directions
    int xHalf = (objMaxW - 1)/2;
    int zHalf = (objMaxL - 1)/2;
    // Define length of the top part
    // Ideally it should be a third of the length, but it needs to be set to nearest odd number that is larger
    int topLength;
    float lThirds = objMaxL/3.0;
    if(int(ceil(lThirds)) % 2 == 0)
        topLength = ceil(lThirds) + 1;
    else
        topLength = ceil(lThirds);
    int topHalfL = (topLength - 1)/2;
    // left y-z plane of the body (dims = 9lx2h)
    xLims.append(QVector<int>({objCenter[0] - xHalf, objCenter[0] - xHalf}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(-1.0f, 0.0f, 0.0f));                    // Normal to y-z plane in -x dir
    // Right y-z plane
    xLims.append(QVector<int>({objCenter[0] + xHalf, objCenter[0] + xHalf}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(1.0f, 0.0f, 0.0f));                     // Normal to y-z plane in +x dir
    // Front x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf -  1)}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] - zHalf}));
    planeNorms.append(QVector3D(0.0f, 0.0f, -1.0f));                     // Normal to x-y plane in -z dir
    // Rear x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] + zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(0.0f, 0.0f, 1.0f));                     // Normal to x-y plane in +z dir
    // top left y-z plane (dims = 3lx3h)
    xLims.append(QVector<int>({objCenter[0] - xHalf, objCenter[0] - xHalf}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] - topHalfL, objCenter[2] + topHalfL}));
    planeNorms.append(QVector3D(-1.0f, 0.0f, 0.0f));                    // Normal to y-z plane in -x dir
    // top right y-z plane
    xLims.append(QVector<int>({objCenter[0] + xHalf, objCenter[0] + xHalf}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] - topHalfL, objCenter[2] + topHalfL}));
    planeNorms.append(QVector3D(1.0f, 0.0f, 0.0f));                     // Normal to y-z plane in +x dir
    // Roof plane. construct it one voxel over the height
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yEnd, yEnd}));
    zLims.append(QVector<int>({objCenter[2] - (topHalfL - 1), objCenter[2] + (topHalfL - 1)}));
    // We cannot have normal looking straight up. So make slight perturbation
    planeNorms.append(QVector3D(0.0f, 0.92f, 0.0f));

    // -------Drawing triangles (draw as hypotenuse lines given the limits of other two sides) -----------------
    QVector<QVector<int>> xLimsT, yLimsT, zLimsT;
    QVector<QVector3D> planeNormsT;
    QVector<QVector<QVector3D>> linePoints;
    // front left triangle
    int zTriag1 = objCenter[2] - zHalf;
    int yTriag1 = yLim3;
    int zTriag2 = objCenter[2] - topHalfL;
    int yTriag2 = yLim2;
    int xTriag = objCenter[0] - xHalf;
    while ((zTriag1 <= zTriag2) && (yTriag1 >= yTriag2))
    {
        // Add the two end points to line container
        linePoints.append({QVector3D(xTriag, yTriag1, zTriag2), QVector3D(xTriag, yTriag2, zTriag1)});
        // Add the search space limits for this line
        xLimsT.append({xTriag, xTriag});
        yLimsT.append({yTriag2, yTriag1});
        zLimsT.append({zTriag1, zTriag2});
        // Add the normal vectors (same as top-left plane)
        planeNormsT.append(QVector3D(-1.0f, 0.0f, 0.0f));
        // Increment z and decrement y
        zTriag1++;
        yTriag1--;
    }
    // rear left triangle
    zTriag1 = objCenter[2] + zHalf;
    yTriag1 = yLim3;
    zTriag2 = objCenter[2] + topHalfL;
    yTriag2 = yLim2;
    xTriag = objCenter[0] - xHalf;
    while ((zTriag1 >= zTriag2) && (yTriag1 >= yTriag2))
    {
        // Add the two end points to line container
        linePoints.append({QVector3D(xTriag, yTriag1, zTriag2), QVector3D(xTriag, yTriag2, zTriag1)});
        // Add the search space limits for this line
        xLimsT.append({xTriag, xTriag});
        yLimsT.append({yTriag2, yTriag1});
        zLimsT.append({zTriag2, zTriag1});
        // Add the normal vectors (same as top-left plane)
        planeNormsT.append(QVector3D(-1.0f, 0.0f, 0.0f));
        // Decrement z and y
        zTriag1--;
        yTriag1--;
    }
    // front right triangle
    zTriag1 = objCenter[2] - zHalf;
    yTriag1 = yLim3;
    zTriag2 = objCenter[2] - topHalfL;
    yTriag2 = yLim2;
    xTriag = objCenter[0] + xHalf;
    while ((zTriag1 <= zTriag2) && (yTriag1 >= yTriag2))
    {
        // Add the two end points to line container
        linePoints.append({QVector3D(xTriag, yTriag1, zTriag2), QVector3D(xTriag, yTriag2, zTriag1)});
        // Add the search space limits for this line
        xLimsT.append({xTriag, xTriag});
        yLimsT.append({yTriag2, yTriag1});
        zLimsT.append({zTriag1, zTriag2});
        // Add the normal vectors (same as top-left plane)
        planeNormsT.append(QVector3D(1.0f, 0.0f, 0.0f));
        // Increment z and decrement y
        zTriag1++;
        yTriag1--;
    }
    // rear right triangle
    zTriag1 = objCenter[2] + zHalf;
    yTriag1 = yLim3;
    zTriag2 = objCenter[2] + topHalfL;
    yTriag2 = yLim2;
    xTriag = objCenter[0] + xHalf;
    while ((zTriag1 >= zTriag2) && (yTriag1 >= yTriag2))
    {
        // Add the two end points to line container
        linePoints.append({QVector3D(xTriag, yTriag1, zTriag2), QVector3D(xTriag, yTriag2, zTriag1)});
        // Add the search space limits for this line
        xLimsT.append({xTriag, xTriag});
        yLimsT.append({yTriag2, yTriag1});
        zLimsT.append({zTriag2, zTriag1});
        // Add the normal vectors (same as top-left plane)
        planeNormsT.append(QVector3D(1.0f, 0.0f, 0.0f));
        // Decrement z and y
        zTriag1--;
        yTriag1--;
    }
    // Front plane
    zTriag1 = objCenter[2] - zHalf;
    zTriag2 = objCenter[2] - topHalfL;
    yTriag1 = yLim2;
    yTriag2 = yLim3;
    xTriag = objCenter[0] - xHalf;
    int xTriag2 = objCenter[0] + xHalf;
    // Calculate normal vector
    // Define three points on the plane
    QVector3D planeP1(xTriag, yTriag1, zTriag1), planeP2(xTriag, yTriag2, zTriag2), planeP3(xTriag2, yTriag1, zTriag1);
    QVector3D frontNorm = QVector3D::normal(planeP1, planeP2, planeP3);
    for(int xVal = xTriag; xVal <= xTriag2; xVal++)
    {
        linePoints.append({QVector3D(xVal, yTriag1, zTriag1), QVector3D(xVal, yTriag2, zTriag2)});
        xLimsT.append({xVal, xVal});
        yLimsT.append({yTriag1, yTriag2});
        zLimsT.append({zTriag1, zTriag2});
        // Normal vector is as calculated above
        planeNormsT.append(frontNorm);
    }
    // Rear Plane
    zTriag1 = objCenter[2] + topHalfL;
    zTriag2 = objCenter[2] + zHalf;
    yTriag1 = yLim2;
    yTriag2 = yLim3;
    xTriag = objCenter[0] - xHalf;
    xTriag2 = objCenter[0] + xHalf;
    // Calculate plane
    QVector3D RPlaneP1(xTriag, yTriag1, zTriag2), RPlaneP2(xTriag, yTriag2, zTriag1), RPlaneP3(xTriag2, yTriag1, zTriag2);
    QVector3D rearNorm = QVector3D::normal(RPlaneP1, RPlaneP2, RPlaneP3);
    for(int xVal = xTriag; xVal <= xTriag2; xVal++)
    {
        linePoints.append({QVector3D(xVal, yTriag2, zTriag1), QVector3D(xVal, yTriag1, zTriag2)});
        xLimsT.append({xVal, xVal});
        yLimsT.append({yTriag1, yTriag2});
        zLimsT.append({zTriag1, zTriag2});
        planeNormsT.append(rearNorm);
    }

    // intialize vectorIndices vector
    vectorIndices.clear();
    vectorIndices.resize(volDims[0]*volDims[1]*volDims[2]);
    // Loop over the planes and fill the vehicle voxels
    int totalPlanes = xLims.size();
    camPoints.clear();
    camNormals.clear();
    // Counter to keep track of number of camera voxels being added
    int camVoxCount = 0;

    for(int currPlane = 0; currPlane < totalPlanes; currPlane++)
    {
        int x1 = xLims[currPlane][0], x2 = xLims[currPlane][1];
        int y1 = yLims[currPlane][0], y2 = yLims[currPlane][1];
        int z1 = zLims[currPlane][0], z2 = zLims[currPlane][1];
        QVector3D currNormal = planeNorms[currPlane];
        // loop over the voxel comprising plane
        for(int x = x1; x <= x2; x++)
        {
            for(int y = y1; y <= y2; y++)
            {
                for(int z = z1; z <= z2; z++)
                {
                    // get the voxel from volume and set it to a value 2
                    unsigned int* voxVal = static_cast<unsigned int*>(voxSimObject->GetScalarPointer(x,y,z));
                    *voxVal = 2;
                    // Add the voxels into camera points containers
                    camPoints.append(QVector3D(x,y,z));
                    // Also add the points to vehicle voxels container
                    vehVox.append(QVector3D(x,y,z));
                    // Correspondingly append the normal vector for each voxel on this plane
                    camNormals.append(currNormal);
                    // Mark the count at this vector index
                    vectorIndices[x + (y * extent[1]) + (z * extent[1] * extent[3])] = camVoxCount;
                    camVoxCount++;
                }
            }
        }
    }

    // ---------------------------Draw triangle---------------------------
    int totalLines = linePoints.size();
    for(int currLine = 0; currLine < totalLines; currLine++)
    {
        int x1 = xLimsT[currLine][0], x2 = xLimsT[currLine][1];
        int y1 = yLimsT[currLine][0], y2 = yLimsT[currLine][1];
        int z1 = zLimsT[currLine][0], z2 = zLimsT[currLine][1];
        QVector3D currNormal = planeNormsT[currLine];
        // Get the two end points of the line
        QVector3D p1(linePoints[currLine][0]), p2(linePoints[currLine][1]);

        for(int x = x1; x <= x2; x++)
        {
            for(int y = y1; y <= y2; y++)
            {
                for(int z = z1; z <= z2; z++)
                {
                    // get the voxel from volume
                    unsigned int* voxVal = static_cast<unsigned int*>(voxSimObject->GetScalarPointer(x,y,z));
                    // do not process if the voxel is already set
                    if(*voxVal != 2)
                    {
                        // get distance from line
                        QVector3D currPoint(x,y,z);
                        QVector3D lineDir(p2 - p1);
                        lineDir.normalize();
                        // Check distance
                        float dist = currPoint.distanceToLine(p1, lineDir);
                        if( dist <= 0.5f)
                        {
                            *voxVal = 2;
                            // Add the voxels into camera points containers
                            camPoints.append(QVector3D(x,y,z));
                            // Also add the points to vehicle voxels container
                            vehVox.append(QVector3D(x,y,z));
                            // Correspondingly append the normal vector for each voxel on this plane
                            camNormals.append(currNormal);
                            // Mark the count at this vector index
                            vectorIndices[x + (y * extent[1]) + (z * extent[1] * extent[3])] = camVoxCount;
                            camVoxCount++;
                        }
                    }

                }
            }
        }
    }
}

void SimulatedObjects::hatchBack()
{
    // Create a simple car-like structure with two boxes in x-z planes with length across z-dimension
    voxSimObject = vtkSmartPointer<vtkStructuredPoints>::New();
    voxSimObject->SetExtent(extent);
    voxSimObject->AllocateScalars(VTK_UNSIGNED_INT, 1);
    // get volume dimensions
    int volDims[3];
    voxSimObject->GetDimensions(volDims);
    // Define the center
    int objCenter[3] = {volDims[0]/2,volDims[1]/2,volDims[2]/2};
    // A simple car is defined using 9 planes, 4 for the body, 4 for top part and 1 for the roof
    QVector<QVector<int>> xLims, yLims, zLims;
    QVector<QVector3D> planeNorms;
    // Define limits in y direction
    int yStart = 2;
    int yLim1 = yStart + (objMaxH - 1)/2 - 1;
    int yLim2 = yStart + (objMaxH - 1)/2;
    int yLim3 = yLim1 + (objMaxH - 1)/2;
    int yEnd = objMaxH + 1;
    // Define halfs in x (width) and z (length) directions
    int xHalf = (objMaxW - 1)/2;
    int zHalf = (objMaxL - 1)/2;
    // Define length of the top part
    // Ideally it should be a third of the length, but it needs to be set to nearest odd number that is larger
    int topLength;
    float lThirds = objMaxL/3.0;
    if(int(ceil(lThirds)) % 2 == 0)
        topLength = ceil(lThirds) + 1;
    else
        topLength = ceil(lThirds);
    int topHalfL = (topLength - 1)/2;
    // left y-z plane of the body (dims = 9lx2h)
    xLims.append(QVector<int>({objCenter[0] - xHalf, objCenter[0] - xHalf}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(-1.0f, 0.0f, 0.0f));                    // Normal to y-z plane in -x dir
    // Right y-z plane
    xLims.append(QVector<int>({objCenter[0] + xHalf, objCenter[0] + xHalf}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(1.0f, 0.0f, 0.0f));                     // Normal to y-z plane in +x dir
    // Front x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf -  1)}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] - zHalf}));
    planeNorms.append(QVector3D(0.0f, 0.0f, -1.0f));                     // Normal to x-y plane in -z dir
    // Rear x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] + zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(0.0f, 0.0f, 1.0f));                     // Normal to x-y plane in +z dir
    // top left y-z plane (dims = 6lx3h)
    xLims.append(QVector<int>({objCenter[0] - xHalf, objCenter[0] - xHalf}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] - topHalfL, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(-1.0f, 0.0f, 0.0f));                    // Normal to y-z plane in -x dir
    // top right y-z plane
    xLims.append(QVector<int>({objCenter[0] + xHalf, objCenter[0] + xHalf}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] - topHalfL, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(1.0f, 0.0f, 0.0f));                     // Normal to y-z plane in +x dir
    // top rear x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] + zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(0.0f, 0.0f, 1.0f));                     // Normal to x-y plane in +z dir
    // Roof plane. construct it one voxel over the height
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yEnd, yEnd}));
    zLims.append(QVector<int>({objCenter[2] - (topHalfL - 1), objCenter[2] + (zHalf - 1)}));
    // We cannot have normal looking straight up. So make slight perturbation
    planeNorms.append(QVector3D(0.0f, 0.92f, 0.0f));

    // -------Drawing triangles (draw as hypotenuse lines given the limits of other two sides) -----------------
    QVector<QVector<int>> xLimsT, yLimsT, zLimsT;
    QVector<QVector3D> planeNormsT;
    QVector<QVector<QVector3D>> linePoints;
    // front left triangle
    int zTriag1 = objCenter[2] - zHalf;
    int yTriag1 = yLim3;
    int zTriag2 = objCenter[2] - topHalfL;
    int yTriag2 = yLim2;
    int xTriag = objCenter[0] - xHalf;
    while ((zTriag1 <= zTriag2) && (yTriag1 >= yTriag2))
    {
        // Add the two end points to line container
        linePoints.append({QVector3D(xTriag, yTriag1, zTriag2), QVector3D(xTriag, yTriag2, zTriag1)});
        // Add the search space limits for this line
        xLimsT.append({xTriag, xTriag});
        yLimsT.append({yTriag2, yTriag1});
        zLimsT.append({zTriag1, zTriag2});
        // Add the normal vectors (same as top-left plane)
        planeNormsT.append(QVector3D(-1.0f, 0.0f, 0.0f));
        // Increment z and decrement y
        zTriag1++;
        yTriag1--;
    }
    // front right triangle
    zTriag1 = objCenter[2] - zHalf;
    yTriag1 = yLim3;
    zTriag2 = objCenter[2] - topHalfL;
    yTriag2 = yLim2;
    xTriag = objCenter[0] + xHalf;
    while ((zTriag1 <= zTriag2) && (yTriag1 >= yTriag2))
    {
        // Add the two end points to line container
        linePoints.append({QVector3D(xTriag, yTriag1, zTriag2), QVector3D(xTriag, yTriag2, zTriag1)});
        // Add the search space limits for this line
        xLimsT.append({xTriag, xTriag});
        yLimsT.append({yTriag2, yTriag1});
        zLimsT.append({zTriag1, zTriag2});
        // Add the normal vectors (same as top-left plane)
        planeNormsT.append(QVector3D(1.0f, 0.0f, 0.0f));
        // Increment z and decrement y
        zTriag1++;
        yTriag1--;
    }
    // Front plane
    zTriag1 = objCenter[2] - zHalf;
    zTriag2 = objCenter[2] - topHalfL;
    yTriag1 = yLim2;
    yTriag2 = yLim3;
    xTriag = objCenter[0] - xHalf;
    int xTriag2 = objCenter[0] + xHalf;
    // Calculate normal vector
    // Define three points on the plane
    QVector3D planeP1(xTriag, yTriag1, zTriag1), planeP2(xTriag, yTriag2, zTriag2), planeP3(xTriag2, yTriag1, zTriag1);
    QVector3D frontNorm = QVector3D::normal(planeP1, planeP2, planeP3);
    for(int xVal = xTriag; xVal <= xTriag2; xVal++)
    {
        linePoints.append({QVector3D(xVal, yTriag1, zTriag1), QVector3D(xVal, yTriag2, zTriag2)});
        xLimsT.append({xVal, xVal});
        yLimsT.append({yTriag1, yTriag2});
        zLimsT.append({zTriag1, zTriag2});
        // Normal vector is as calculated above
        planeNormsT.append(frontNorm);
    }

    // intialize vectorIndices vector
    vectorIndices.clear();
    vectorIndices.resize(volDims[0]*volDims[1]*volDims[2]);
    // Loop over the planes and fill the vehicle voxels
    int totalPlanes = xLims.size();
    camPoints.clear();
    camNormals.clear();
    // Counter to keep track of number of camera voxels being added
    int camVoxCount = 0;

    for(int currPlane = 0; currPlane < totalPlanes; currPlane++)
    {
        int x1 = xLims[currPlane][0], x2 = xLims[currPlane][1];
        int y1 = yLims[currPlane][0], y2 = yLims[currPlane][1];
        int z1 = zLims[currPlane][0], z2 = zLims[currPlane][1];
        QVector3D currNormal = planeNorms[currPlane];
        // loop over the voxel comprising plane
        for(int x = x1; x <= x2; x++)
        {
            for(int y = y1; y <= y2; y++)
            {
                for(int z = z1; z <= z2; z++)
                {
                    // get the voxel from volume and set it to a value 2
                    unsigned int* voxVal = static_cast<unsigned int*>(voxSimObject->GetScalarPointer(x,y,z));
                    *voxVal = 2;
                    // Add the voxels into camera points containers
                    camPoints.append(QVector3D(x,y,z));
                    // Also add the points to vehicle voxels container
                    vehVox.append(QVector3D(x,y,z));
                    // Correspondingly append the normal vector for each voxel on this plane
                    camNormals.append(currNormal);
                    // Mark the count at this vector index
                    vectorIndices[x + (y * extent[1]) + (z * extent[1] * extent[3])] = camVoxCount;
                    camVoxCount++;
                }
            }
        }
    }

    // ---------------------------Draw triangle---------------------------
    int totalLines = linePoints.size();
    for(int currLine = 0; currLine < totalLines; currLine++)
    {
        int x1 = xLimsT[currLine][0], x2 = xLimsT[currLine][1];
        int y1 = yLimsT[currLine][0], y2 = yLimsT[currLine][1];
        int z1 = zLimsT[currLine][0], z2 = zLimsT[currLine][1];
        QVector3D currNormal = planeNormsT[currLine];
        // Get the two end points of the line
        QVector3D p1(linePoints[currLine][0]), p2(linePoints[currLine][1]);

        for(int x = x1; x <= x2; x++)
        {
            for(int y = y1; y <= y2; y++)
            {
                for(int z = z1; z <= z2; z++)
                {
                    // get the voxel from volume
                    unsigned int* voxVal = static_cast<unsigned int*>(voxSimObject->GetScalarPointer(x,y,z));
                    // do not process if the voxel is already set
                    if(*voxVal != 2)
                    {
                        // get distance from line
                        QVector3D currPoint(x,y,z);
                        QVector3D lineDir(p2 - p1);
                        lineDir.normalize();
                        // Check distance
                        float dist = currPoint.distanceToLine(p1, lineDir);
                        if( dist <= 0.5f)
                        {
                            *voxVal = 2;
                            // Add the voxels into camera points containers
                            camPoints.append(QVector3D(x,y,z));
                            // Also add the points to vehicle voxels container
                            vehVox.append(QVector3D(x,y,z));
                            // Correspondingly append the normal vector for each voxel on this plane
                            camNormals.append(currNormal);
                            // Mark the count at this vector index
                            vectorIndices[x + (y * extent[1]) + (z * extent[1] * extent[3])] = camVoxCount;
                            camVoxCount++;
                        }
                    }

                }
            }
        }
    }
}

void SimulatedObjects::truck()
{
    // Create a simple car-like structure with two boxes in x-z planes with length across z-dimension
    voxSimObject = vtkSmartPointer<vtkStructuredPoints>::New();
    voxSimObject->SetExtent(extent);
    voxSimObject->AllocateScalars(VTK_UNSIGNED_INT, 1);
    // get volume dimensions
    int volDims[3];
    voxSimObject->GetDimensions(volDims);
    // Define the center
    int objCenter[3] = {volDims[0]/2,volDims[1]/2,volDims[2]/2};
    // A simple car is defined using 9 planes, 4 for the body, 4 for top part and 1 for the roof
    QVector<QVector<int>> xLims, yLims, zLims;
    QVector<QVector3D> planeNorms;
    // Define limits in y direction
    int yStart = 2;
    int yLim1 = yStart + (objMaxH - 1)/2 - 1;
    int yLim2 = yStart + (objMaxH - 1)/2;
    int yLim3 = yLim1 + (objMaxH - 1)/2;
    int yEnd = objMaxH + 1;
    // Define halfs in x (width) and z (length) directions
    int xHalf = (objMaxW - 1)/2;
    int zHalf = (objMaxL - 1)/2;
    // Define length of the top part
    // Ideally it should be a third of the length, but it needs to be set to nearest odd number that is larger
    int topLength;
    float lThirds = objMaxL/3.0;
    if(int(ceil(lThirds)) % 2 == 0)
        topLength = ceil(lThirds) + 1;
    else
        topLength = ceil(lThirds);
    int topHalfL = (topLength - 1)/2;
    // left y-z plane of the body (dims = 9lx2h)
    xLims.append(QVector<int>({objCenter[0] - xHalf, objCenter[0] - xHalf}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(-1.0f, 0.0f, 0.0f));                    // Normal to y-z plane in -x dir
    // Right y-z plane
    xLims.append(QVector<int>({objCenter[0] + xHalf, objCenter[0] + xHalf}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(1.0f, 0.0f, 0.0f));                     // Normal to y-z plane in +x dir
    // Front x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf -  1)}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] - zHalf}));
    planeNorms.append(QVector3D(0.0f, 0.0f, -1.0f));                     // Normal to x-y plane in -z dir
    // Rear x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yStart, yLim1}));
    zLims.append(QVector<int>({objCenter[2] + zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(0.0f, 0.0f, 1.0f));                     // Normal to x-y plane in +z dir
    // top left y-z plane (dims = 3lx3h)
    xLims.append(QVector<int>({objCenter[0] - xHalf, objCenter[0] - xHalf}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] - topHalfL}));
    planeNorms.append(QVector3D(-1.0f, 0.0f, 0.0f));                    // Normal to y-z plane in -x dir
    // top right y-z plane
    xLims.append(QVector<int>({objCenter[0] + xHalf, objCenter[0] + xHalf}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] - topHalfL}));
    planeNorms.append(QVector3D(1.0f, 0.0f, 0.0f));                     // Normal to y-z plane in +x dir
    // top front x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] - zHalf}));
    planeNorms.append(QVector3D(0.0f, 0.0f, -1.0f));                     // Normal to x-y plane in -z dir
    // top rear x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yLim2, yLim3}));
    zLims.append(QVector<int>({objCenter[2] - topHalfL, objCenter[2] - topHalfL}));
    planeNorms.append(QVector3D(0.0f, 0.0f, 1.0f));                     // Normal to x-y plane in +z dir
    // Roof plane. construct it one voxel over the height
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yEnd, yEnd}));
    zLims.append(QVector<int>({objCenter[2] - (zHalf - 1), objCenter[2] - (topHalfL + 1)}));
    // We cannot have normal looking straight up. So make slight perturbation
    planeNorms.append(QVector3D(0.0f, 0.92f, 0.0f));

    // intialize vectorIndices vector
    vectorIndices.clear();
    vectorIndices.resize(volDims[0]*volDims[1]*volDims[2]);
    // Loop over the planes and fill the vehicle voxels
    int totalPlanes = xLims.size();
    camPoints.clear();
    camNormals.clear();
    // Counter to keep track of number of camera voxels being added
    int camVoxCount = 0;

    for(int currPlane = 0; currPlane < totalPlanes; currPlane++)
    {
        int x1 = xLims[currPlane][0], x2 = xLims[currPlane][1];
        int y1 = yLims[currPlane][0], y2 = yLims[currPlane][1];
        int z1 = zLims[currPlane][0], z2 = zLims[currPlane][1];
        QVector3D currNormal = planeNorms[currPlane];
        // loop over the voxel comprising plane
        for(int x = x1; x <= x2; x++)
        {
            for(int y = y1; y <= y2; y++)
            {
                for(int z = z1; z <= z2; z++)
                {
                    // get the voxel from volume and set it to a value 2
                    unsigned int* voxVal = static_cast<unsigned int*>(voxSimObject->GetScalarPointer(x,y,z));
                    *voxVal = 2;
                    // Add the voxels into camera points containers
                    camPoints.append(QVector3D(x,y,z));
                    // Also add the points to vehicle voxels container
                    vehVox.append(QVector3D(x,y,z));
                    // Correspondingly append the normal vector for each voxel on this plane
                    camNormals.append(currNormal);
                    // Mark the count at this vector index
                    vectorIndices[x + (y * extent[1]) + (z * extent[1] * extent[3])] = camVoxCount;
                    camVoxCount++;
                }
            }
        }
    }
}

void SimulatedObjects::bus()
{
    // Create a simple car-like structure with two boxes in x-z planes with length across z-dimension
    voxSimObject = vtkSmartPointer<vtkStructuredPoints>::New();
    voxSimObject->SetExtent(extent);
    voxSimObject->AllocateScalars(VTK_UNSIGNED_INT, 1);
    // get volume dimensions
    int volDims[3];
    voxSimObject->GetDimensions(volDims);
    // Define the center
    int objCenter[3] = {volDims[0]/2,volDims[1]/2,volDims[2]/2};
    // A simple car is defined using 9 planes, 4 for the body, 4 for top part and 1 for the roof
    QVector<QVector<int>> xLims, yLims, zLims;
    QVector<QVector3D> planeNorms;
    // Define limits in y direction
    int yStart = 2;
    int yEnd = objMaxH + 1;
    // Define halfs in x (width) and z (length) directions
    int xHalf = (objMaxW - 1)/2;
    int zHalf = (objMaxL - 1)/2;
    // left y-z plane of the body (dims = 9lxobjMaxh)
    xLims.append(QVector<int>({objCenter[0] - xHalf, objCenter[0] - xHalf}));
    yLims.append(QVector<int>({yStart, yEnd - 1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(-1.0f, 0.0f, 0.0f));                    // Normal to y-z plane in -x dir
    // Right y-z plane
    xLims.append(QVector<int>({objCenter[0] + xHalf, objCenter[0] + xHalf}));
    yLims.append(QVector<int>({yStart, yEnd-1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(1.0f, 0.0f, 0.0f));                     // Normal to y-z plane in +x dir
    // Front x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf -  1)}));
    yLims.append(QVector<int>({yStart, yEnd - 1}));
    zLims.append(QVector<int>({objCenter[2] - zHalf, objCenter[2] - zHalf}));
    planeNorms.append(QVector3D(0.0f, 0.0f, -1.0f));                     // Normal to x-y plane in -z dir
    // Rear x-y plane
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yStart, yEnd - 1}));
    zLims.append(QVector<int>({objCenter[2] + zHalf, objCenter[2] + zHalf}));
    planeNorms.append(QVector3D(0.0f, 0.0f, 1.0f));                     // Normal to x-y plane in +z dir
    // Roof plane. construct it one voxel over the height
    xLims.append(QVector<int>({objCenter[0] - (xHalf - 1), objCenter[0] + (xHalf - 1)}));
    yLims.append(QVector<int>({yEnd, yEnd}));
    zLims.append(QVector<int>({objCenter[2] - (zHalf - 1), objCenter[2] + (zHalf - 1)}));
    // We cannot have normal looking straight up. So make slight perturbation
    planeNorms.append(QVector3D(0.0f, 0.92f, 0.0f));

    // intialize vectorIndices vector
    vectorIndices.clear();
    vectorIndices.resize(volDims[0]*volDims[1]*volDims[2]);
    // Loop over the planes and fill the vehicle voxels
    int totalPlanes = xLims.size();
    camPoints.clear();
    camNormals.clear();
    // Counter to keep track of number of camera voxels being added
    int camVoxCount = 0;

    for(int currPlane = 0; currPlane < totalPlanes; currPlane++)
    {
        int x1 = xLims[currPlane][0], x2 = xLims[currPlane][1];
        int y1 = yLims[currPlane][0], y2 = yLims[currPlane][1];
        int z1 = zLims[currPlane][0], z2 = zLims[currPlane][1];
        QVector3D currNormal = planeNorms[currPlane];
        // loop over the voxel comprising plane
        for(int x = x1; x <= x2; x++)
        {
            for(int y = y1; y <= y2; y++)
            {
                for(int z = z1; z <= z2; z++)
                {
                    // get the voxel from volume and set it to a value 2
                    unsigned int* voxVal = static_cast<unsigned int*>(voxSimObject->GetScalarPointer(x,y,z));
                    *voxVal = 2;
                    // Add the voxels into camera points containers
                    camPoints.append(QVector3D(x,y,z));
                    // Also add the points to vehicle voxels container
                    vehVox.append(QVector3D(x,y,z));
                    // Correspondingly append the normal vector for each voxel on this plane
                    camNormals.append(currNormal);
                    // Mark the count at this vector index
                    vectorIndices[x + (y * extent[1]) + (z * extent[1] * extent[3])] = camVoxCount;
                    camVoxCount++;
                }
            }
        }
    }
}

vtkSmartPointer<vtkStructuredPoints>& SimulatedObjects::getSimData(QVector<QVector3D> &boundaryVoxels,
                                                                   QVector<QVector3D> &boundaryNormals,
                                                                   QVector<QVector3D> &vehicleVox,
                                                                   QVector<unsigned long int> &vecInd)
{
    // Setup the output vectors with data memebers previously setup at initialization
    boundaryVoxels = camPoints;
    boundaryNormals = camNormals;
    vehicleVox = vehVox;
    vecInd = vectorIndices;
    return voxSimObject;
}

void SimulatedObjects::getVehicleDims(int vehicleDims[3])
{
    vehicleDims[0] = objMaxW;
    vehicleDims[1] = objMaxH;
    vehicleDims[2] = objMaxL;
}
