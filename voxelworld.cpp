#include "voxelworld.h"

VoxelWorld::VoxelWorld() : volData(PolyVox::Region(Vector3DInt32(0,0,0), Vector3DInt32(99,39,99)))
{
    //volData = new SimpleVolume<MaterialDensityPair44>(PolyVox::Region(Vector3DInt32(0,0,0), Vector3DInt32(499,499,499)));
    // Initialize the world center
    worldCenter = QVector3D(volData.getWidth()/2, volData.getHeight()/2, volData.getDepth()/2);

    /*  Select files from
     *  1) BMW_32
     *  2) crane
     */
    QString path = "/media/anirudh/Data/Documents/PhD/Qt_projects/ocp_simulation/resources/bulldoser_32.binvox";
    // Fill voxel world with object loaded from .obj file
    createVoxObject(path);
}

void VoxelWorld::createObject(QString &path)
{
    // Load .obj file and read in the list of vertices
    bool res = meshObj.loadModel(path);

    // Obtain the list of vertices
    if(res)
        meshObj.getVertices(vertexList);
    else
        qDebug()<<"Object file could not be loaded";

    qDebug()<<"Size of incoming vertices = "<<vertexList.size();

    for(int i = 0; i < vertexList.size(); i++)
    {
        // New density value
        uint8_t uDensity = MaterialDensityPair44::getMaxDensity();

        // Get rounded vertex coordinates
        int xCoord = static_cast<int>(vertexList[i].x());
        int yCoord = -static_cast<int>(vertexList[i].y());  // without '-' sign the object displays upside down
        int zCoord = static_cast<int>(vertexList[i].z());

//        int xCoord = vertexList[i].x();
//        int yCoord = -vertexList[i].y();

//        int zCoord = vertexList[i].z();

        // Get the old voxel value
        MaterialDensityPair44 voxel = volData.getVoxelAt(xCoord, yCoord, zCoord);

        // Modify the density
        voxel.setDensity(uDensity);

        // Write the voxel value into the volume
        volData.setVoxelAt(xCoord, yCoord, zCoord, voxel);
    }
}

void VoxelWorld::getBackground()
{
    // New density value for polyvox
    //uint8_t uDensity = MaterialDensityPair44::getMaxDensity();
    //uint8_t backDensity = uDensity / 8;

    for (int x = 0; x < volData.getWidth(); x++)
    {
        for (int z = 0; z < volData.getDepth(); z++)
        {
            for (int y = 0; y < volData.getHeight(); y++)
            {
                // Get the voxel and check its density
                MaterialDensityPair44 voxel = volData.getVoxelAt(x,y,z);
                // Only consider unoccupied voxels
                if(voxel.getDensity() == 0)
                {
                    //voxel.setDensity(backDensity);
                    //volData.setVoxelAt(x, y, z, voxel);

                    // Check if voxel lies within 12m (~43 voxels for bulldozer) circumference and discard voxels that lie above the vehicle max_y plane
                    double voxDist = sqrt(pow(objectCenter.x()-x, 2) + pow(objectCenter.y()-y, 2) + pow(objectCenter.z()-z, 2));
                    if(voxDist < 43 && y >= min_y && y <= max_y)
                        backgroundPoints.append(QVector3D(x,y,z));
                }
            }
        }
    }
}

void VoxelWorld::createVoxObject(QString &path)
{
    // Call the binvox function to read binvox file and setup voxel information
    voxObj.loadBinVox(path);

    // Get voxel object dimensions
    int voxDims = voxObj.getDims();
    int d = std::cbrt(voxDims); // cube side length

    // Initialize the voxel object array (bounding box returned by binvox is always cubic)
    voxObject = new byte[voxDims];

    // Get the voxel object
    voxObj.getVoxels(voxObject);

    // Initialize object boundaries in the three dimensions
    // minimum values with object cube length
    min_x = min_y = min_z = d;
    // maximum values with zero
    max_x = max_y = max_z = 0;

    // Traverse the voxel object array to setup our voxel world
    for(int i = 0; i < voxDims; i++)
    {
        // If model has an occupied voxel mirror it in the world
        if(voxObject[i] == 1)
        {
            /* Get 3D voxel coordinates from voxObject 1D array indices
             * refer to http://www.patrickmin.com/binvox/binvox.html (voxel ordering part) for 3D coordinates to 1D index formula
             */
            int xCoord = i/(d * d) + centerDist.x();
            int zCoord = (i / d) % d + centerDist.z();
            int yCoord = i % d;                         // Do not shift y to keep ground plane

            // Save the coordinates to make morphological operations faster
            voxObjectCoords.append(QVector3D(xCoord, yCoord, zCoord));

            min_x = xCoord < min_x ? xCoord : min_x;
            min_y = yCoord < min_y ? yCoord : min_y;
            min_z = zCoord < min_z ? zCoord : min_z;

            max_x = xCoord > max_x ? xCoord : max_x;
            max_y = yCoord > max_y ? yCoord : max_y;
            max_z = zCoord > max_z ? zCoord : max_z;
        }
    }

    // Update the object center after getting min and max in each dimension
    objectCenter.setX((max_x - min_x)/2);
    objectCenter.setY((max_y - min_y)/2);
    objectCenter.setZ((max_z - min_z)/2);
    // calculate distance by which object has to be shifted to bring it to the center of environment
    centerDist = QVector3D(worldCenter.x()-objectCenter.x(), worldCenter.y()-objectCenter.y(), worldCenter.z()-objectCenter.z());       // Keep in mind y distance although it's not being used

    //----------------------Fill the environment with occupied voxels-------------------------------------
    // New density value for polyvox
    uint8_t uDensity = MaterialDensityPair44::getMaxDensity();

    for(int i = 0; i <voxObjectCoords.size(); i++)
    {
        // Get the voxel coordinates and shift x and z values by distance calculated above
        int xCoord = voxObjectCoords[i].x() + centerDist.x();
        int yCoord = voxObjectCoords[i].y();
        int zCoord = voxObjectCoords[i].z() + centerDist.z();

        // Get old voxel value at the location
        MaterialDensityPair44 voxel = volData.getVoxelAt(xCoord, yCoord, zCoord);
        // Modify the density
        voxel.setDensity(uDensity);
        // Write the voxel value into the volume
        volData.setVoxelAt(xCoord, yCoord, zCoord, voxel);

        // Update "voxObjectCoords" with shifted voxel coordinates
        voxObjectCoords[i].setX(xCoord);
        voxObjectCoords[i].setZ(zCoord);
    }

    qDebug()<<"min : ("<<min_x<<","<<min_y<<","<<min_z<<")"<<" max : ("<<max_x<<","<<max_y<<","<<max_z<<") in object frame";

    // Update the min and max dimensions w.r.t. to world frame
    min_x+=centerDist.x();
    max_x+=centerDist.x();
    min_z+=centerDist.z();
    max_z+=centerDist.z();
    // Update object center w.r.t. world coordinates for later use
    objectCenter.setX(objectCenter.x()+min_x);
    objectCenter.setZ(objectCenter.z()+min_z);

    qDebug()<<"min : ("<<min_x<<","<<min_y<<","<<min_z<<")"<<" max : ("<<max_x<<","<<max_y<<","<<max_z<<") in world frame";
    qDebug()<<"Number of occupied voxels: "<<voxObjectCoords.size();
    qDebug()<<"World Center : "<< worldCenter;
    qDebug()<<"Object Center (Updated) : "<< objectCenter;
}

QVector<QVector3D> VoxelWorld::getNeighbours(QVector3D filterCenter, int filterSize)
{
    QVector<QVector3D> filterGrid;
    filterSize = (filterSize - 1)/2;
    //qDebug()<<"*************************\n";
    for (int x = filterCenter.x() - filterSize; x <= filterCenter.x() + filterSize; x++)
    {
        for (int y = filterCenter.y() - filterSize; y <= filterCenter.y() + filterSize; y++)
        {
            for(int z = filterCenter.z() - filterSize; z <= filterCenter.z() + filterSize; z++)
            {
                /* Ignore the filter center because we are working only on occupied voxels
                 * and avoid cases with negative y because it is out of bounds for the environment
                 */
                if(!(x == static_cast<int>(filterCenter.x()) && y == static_cast<int>(filterCenter.y()) && z == static_cast<int>(filterCenter.z())) && y>=0)
                {
                    //qDebug()<<"("<<x<<","<<y<<","<<z<<")\n";
                    filterGrid.append(QVector3D(x,y,z));
                }
            }
        }
    }
    //qDebug()<<"Filter Grid Size: "<<filterGrid.size()<<"\n";

    return filterGrid;
}

void VoxelWorld::drawAxes(QVector<GLfloat> &out_vertices)
{
    // Origin, normal vector and color code
    out_vertices.append({0.0f, 0.0f, 0.0f});
    out_vertices.append({1.0f, 1.0f, 1.0f});
    out_vertices.append({0.39f, 1.0f, 0.0f});   // Green -> x-axis
    // X-Axis point
    out_vertices.append({10.0f, 0.0f, 0.0f});
    out_vertices.append({1.0f, 1.0f, 1.0f});
    out_vertices.append({0.39f, 1.0f, 0.0f});   // Green -> x-axis

    // Origin, normal vector and color code
    out_vertices.append({0.0f, 0.0f, 0.0f});
    out_vertices.append({1.0f, 1.0f, 1.0f});
    out_vertices.append({1.0f, 0.0f, 0.0f});   // Red -> y-axis
    // Y-axis point
    out_vertices.append({0.0f, 10.0f, 0.0f});
    out_vertices.append({1.0f, 1.0f, 1.0f});
    out_vertices.append({1.0f, 0.0f, 0.0f});   // Red -> y-axis

    // Origin, normal vector and color code
    out_vertices.append({0.0f, 0.0f, 0.0f});
    out_vertices.append({1.0f, 1.0f, 1.0f});
    out_vertices.append({0.0f, 0.39f, 1.0f});   // blue -> z-axis
    // Z-axis point
    out_vertices.append({0.0f, 0.0f, 10.0f});
    out_vertices.append({1.0f, 1.0f, 1.0f});
    out_vertices.append({0.0f, 0.39f, 1.0f});   // blue -> z-axis
}

void VoxelWorld::dilateObject()
{
    // New density value for polyvox
    uint8_t uDensity = MaterialDensityPair44::getMaxDensity();
    // Density value for boundary voxels
    uint8_t boundDensity = uDensity * 3/4;

    // iterate over the occupied voxels and fill neighbourhood
    for(int i = 0; i < voxObjectCoords.size(); i++)
    {
        // Get all the neighbour voxel indices
        QVector<QVector3D> neighborhood = getNeighbours(voxObjectCoords[i], 3);

        // Iterate over all the neighbours at each occupied voxel
        for(int j = 0; j < neighborhood.size(); j++)
        {
            // Populate neighbourhood voxels if not already set
            MaterialDensityPair44 voxel = volData.getVoxelAt(neighborhood[j].x(), neighborhood[j].y(), neighborhood[j].z());
            if(voxel.getDensity() != uDensity)
            {
                // Modify the density
                voxel.setDensity(boundDensity);
                // Write the voxel value into the volume
                volData.setVoxelAt(neighborhood[j].x(), neighborhood[j].y(), neighborhood[j].z(), voxel);

                /* Consider all the newly created voxels as boundary of the object
                 * except the ones facing towards the top or bottom
                 */
                if((neighborhood[j].y() >= min_y + 4/*4*/) && (neighborhood[j].y() <= max_y - 2/*2*/))
                    boundaryVoxels.append(QVector3D(neighborhood[j].x(), neighborhood[j].y(), neighborhood[j].z()));
            }
        }
    }
    generateNormals();
    qDebug()<<"Normals generated\n";
}

float VoxelWorld::sobelX(QVector3D voxPos)
{
    // filter size is always constant set to 3 (offset = 1)
    int fSize = 1;

    // output variable
    float dx;

    /*
     * Sobel operator (x-axis).
     * - Kernel definition: \n
     *     [ -1  0  1 ]
     *     [ -2  0  2 ]
     *     [ -1  0  1 ]
     *    1 [ -2  0  2 ]  \n
     *   -- [ -4  0  4 ]  \n
     *   16 [ -2  0  2 ]  \n
     *       [ -1  0  1 ] \n
     *       [ -2  0  2 ] \n
     *       [ -1  0  1 ] \n
     */

    // NOTE: Since ground plane is at y=0, It will be a special case
    if(static_cast<int>(voxPos.y()) != 0)
    {
        dx = (1.0f/16) * (4 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y(), voxPos.z()).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y(), voxPos.z() + fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y(), voxPos.z() - fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z()).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() - fSize, voxPos.z()).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() - fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() - fSize, voxPos.z() - fSize).getDensity()
                          - 4 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y(), voxPos.z()).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y(), voxPos.z() + fSize).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y(), voxPos.z() - fSize).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z()).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() - fSize, voxPos.z()).getDensity()
                          - volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() - fSize, voxPos.z() + fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() - fSize, voxPos.z() - fSize).getDensity());
    }
    else
    {
        /* If operating at a voxel lying on the ground plane,
         * do not consider the neighbourhood at y-1
         */
        dx = (1.0f/12) * (4 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y(), voxPos.z()).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y(), voxPos.z() + fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y(), voxPos.z() - fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z()).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          - 4 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y(), voxPos.z()).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y(), voxPos.z() + fSize).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y(), voxPos.z() - fSize).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z()).getDensity()
                          - volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z() - fSize).getDensity());
    }

    return dx;
}

float VoxelWorld::sobelY(QVector3D voxPos)
{
    // filter size is always constant set to 3 (offset = 1)
    int fSize = 1;

    // output variable
    float dy;

    /*
     * Sobel operator (y-axis).
     * - Kernel definition: \n
     *     [ -1 -2 -1 ]
     *     [  0  0  0 ]
     *     [  1  2  1 ]
     *    1 [ -2 -4 -2 ]  \n
     *   -- [  0  0  0 ]  \n
     *   16 [  2  4  2 ]  \n
     *       [ -1 -2 -1 ] \n
     *       [  0  0  0 ] \n
     *       [  1  2  1 ] \n
     */

    // NOTE: Since ground plane is at y=0, It will be a special case
    if(static_cast<int>(voxPos.y()) != 0)
    {
        dy = (1.0f/16) * (4 * volData.getVoxelAt(voxPos.x(), voxPos.y() + fSize, voxPos.z()).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z()).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z()).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x(), voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x(), voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          - 4 * volData.getVoxelAt(voxPos.x(), voxPos.y() - fSize, voxPos.z()).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() - fSize, voxPos.z()).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() - fSize, voxPos.z()).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x(), voxPos.y() - fSize, voxPos.z() - fSize).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x(), voxPos.y() - fSize, voxPos.z() + fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() - fSize, voxPos.z() - fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() - fSize, voxPos.z() - fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() - fSize, voxPos.z() + fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() - fSize, voxPos.z() + fSize).getDensity());
    }
    else
    {
        /* If operating at a voxel lying on the ground plane,
         * do not consider the neighbourhood at y-1
         */
        dy = (1.0f/32) * (4 * volData.getVoxelAt(voxPos.x(), voxPos.y() + fSize, voxPos.z()).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z()).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z()).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x(), voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x(), voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z() + fSize).getDensity());
    }

    return dy;
}

float VoxelWorld::sobelZ(QVector3D voxPos)
{
    // filter size is always constant set to 3 (offset = 1)
    int fSize = 1;

    // output variable
    float dz;

    /*
     * Sobel operator (z-axis).
     * - Parameter V is a used image type.
     * - Kernel definition: \n
     *     [ -1 -2 -1 ]
     *     [ -2 -4 -2 ]
     *     [ -1 -2 -1 ]
     *    1 [  0  0  0 ]  \n
     *   -- [  0  0  0 ]  \n
     *   16 [  0  0  0 ]  \n
     *       [  1  2  1 ] \n
     *       [  2  4  2 ] \n
     *       [  1  2  1 ] \n
     */
    if(static_cast<int>(voxPos.y()) != 0)
    {
        dz = (1.0f/16) * (4 * volData.getVoxelAt(voxPos.x(), voxPos.y(), voxPos.z() + fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y(), voxPos.z() + fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y(), voxPos.z() + fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x(), voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x(), voxPos.y() - fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() - fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() - fSize, voxPos.z() + fSize).getDensity()
                          - 4 * volData.getVoxelAt(voxPos.x(), voxPos.y(), voxPos.z() - fSize).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y(), voxPos.z() - fSize).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y(), voxPos.z() - fSize).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x(), voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x(), voxPos.y() - fSize, voxPos.z() - fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() - fSize, voxPos.z() - fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() - fSize, voxPos.z() - fSize).getDensity());
    }
    else
    {
        /* If operating at a voxel lying on the ground plane,
         * do not consider the neighbourhood at y-1
         */
        dz = (1.0f/12) * (4 * volData.getVoxelAt(voxPos.x(), voxPos.y(), voxPos.z() + fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y(), voxPos.z() + fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y(), voxPos.z() + fSize).getDensity()
                          + 2 * volData.getVoxelAt(voxPos.x(), voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          + volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z() + fSize).getDensity()
                          - 4 * volData.getVoxelAt(voxPos.x(), voxPos.y(), voxPos.z() - fSize).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() - fSize, voxPos.y(), voxPos.z() - fSize).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x() + fSize, voxPos.y(), voxPos.z() - fSize).getDensity()
                          - 2 * volData.getVoxelAt(voxPos.x(), voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() - fSize, voxPos.y() + fSize, voxPos.z() - fSize).getDensity()
                          - volData.getVoxelAt(voxPos.x() + fSize, voxPos.y() + fSize, voxPos.z() - fSize).getDensity());
    }

    return dz;
}

QVector3D VoxelWorld::getRotations(QVector3D inNormal, int axis, float angle)
{
    // 4x4 matrix values with last row and column initialized to 0s
    float m11,m12,m13,m14=0.0f,m21,m22,m23,m24=0.0f,m31,m32,m33,m34=0.0f,m41=0.0f,m42=0.0f,m43=0.0f,m44=1.0f;
    switch (axis)
    {
    // Construct rotation matrix about x axis
    case 1: m11=1.0f;
        m12=0.0f;
        m13=0.0f;
        m21=0.0f;
        m22=cos(angle);
        m23=-sin(angle);
        m31=0.0f;
        m32=sin(angle);
        m33=cos(angle);
        break;

    case 2: m11=cos(angle);
        m12=0.0f;
        m13=sin(angle);
        m21=0.0f;
        m22=1.0f;
        m23=0.0f;
        m31=-sin(angle);
        m32=0.0f;
        m33=cos(angle);
        break;

    case 3: m11=cos(angle);
        m12=-sin(angle);
        m13=0.0f;
        m21=sin(angle);
        m22=cos(angle);
        m23=0.0f;
        m31=0.0f;
        m32=0.0f;
        m33=1.0f;
        break;

    }
    QMatrix4x4 rotMatrix(m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44);
    // Multiply normal vector with rotation matrix to get rotated vector
    QVector3D rotatedNormal = QVector3D(QVector4D(inNormal) * rotMatrix);
    return rotatedNormal;
}

void VoxelWorld::generateNormals()
{
    qDebug()<<"Generating Normals\n";

    // Get camera FOV angles
    float alpha_h, alpha_v;
    optimize.getFOVAngles(alpha_h, alpha_v);

    // Iterate over boundary voxels and generate normal
    QVector<QVector3D>::iterator i = boundaryVoxels.begin();
    while(i != boundaryVoxels.end())
    {
        /* Get the gradient at the point
         * Normally the gradient is always in the direction of low density to high density values
         * This implies that the gradient (surface normal) will always point inside the object
         * But for our case we need the normals to point outside the object
         * This can be achieved by inverting the sign of the gradient values
         */
        float dx = sobelX(*i);
        float dy = sobelY(*i);
        float dz = sobelZ(*i);

        // Create a normal point
        QVector3D normal = QVector3D(-dx,-dy,-dz);
        normal.normalize();

        /* Save all the normals for later use
         * To avoid the case of normal facing (0,1,0),
         * discard such voxels. Also remove normals facing straight down (0,-1,0)
         *
         * Simultaneously add rotations about x and y axes (4 in each direction)
         */
        QVector<QVector3D> rotationsX, rotationsY;
        QVector<int> angleList = {-2,-1,1,2};
        /* Convert to degrees and divide the remaining part of FOV (within 180 degree) into two parts
         * This will give us the angle by which direction is to be rotated
         */
        float angle_v,angle_h;
        angle_h = (90 - alpha_h * static_cast<float>((180/M_PI)))/2;
        angle_v = (90 - alpha_v * static_cast<float>((180/M_PI)))/2;

        if((normal != QVector3D(0.0f, 1.0f, 0.0f)) && (normal != QVector3D(0.0f, -1.0f, 0.0f)))
        {
            // Check which is the major axis and keep the other two
            int axis1, axis2;
            if(normal.x() >= normal.y())
            {
                if(normal.x() >= normal.z())
                {
                    axis1 = 2;
                    axis2 = 3;
                }
                else
                {
                    axis1 = 1;
                    axis2 = 2;
                }
            }
            else
            {
                if(normal.y() >= normal.z())
                {
                    axis1 = 1;
                    axis2 = 3;
                }
                else
                {
                    axis1 = 1;
                    axis2 = 2;
                }
            }

            // Get rotations of normal vector about the two minor axes
            foreach (int val, angleList)
            {
                rotationsX.append(getRotations(normal, axis1, val * angle_h * static_cast<float>(M_PI/180)));
                rotationsY.append(getRotations(normal, axis2, val * angle_v * static_cast<float>(M_PI/180)));
            }
            QVector<QVector3D> temp;
            temp.append(normal);
            temp.append(rotationsX);
            temp.append(rotationsY);
            boundaryNormals.append(temp);
            i++;
        }
        else
            boundaryVoxels.erase(i);
    }
    qDebug()<<"Generated "<<boundaryNormals.size()<<" normals for "<<boundaryVoxels.size()<<" boundary voxels\n";
    // to verify count all the directions that match our NULL case
    int countP=0,countN=0;
    for(int ind = 0; ind < boundaryNormals.size(); ind++)
    {
        countP += boundaryNormals[ind].count(QVector3D(0.0f, 1.0f, 0.0f));
        countN += boundaryNormals[ind].count(QVector3D(0.0f, -1.0f, 0.0f));
    }
    qDebug()<< countP << " (0,1,0) cases exist!";
    qDebug()<< countN << " (0,-1,0) cases exist!";
}

void VoxelWorld::generateMesh()
{
    // extract mesh using polyvox
    //CubicSurfaceExtractorWithNormals<SimpleVolume<MaterialDensityPair44>> surfaceExtractor(&volData, volData.getEnclosingRegion(), &mesh);
    MarchingCubesSurfaceExtractor< SimpleVolume<MaterialDensityPair44> > surfaceExtractor(&volData, volData.getEnclosingRegion(), &mesh);

    // Execute surface extractor
    surfaceExtractor.execute();
}

bool VoxelWorld::getMesh(QVector<GLfloat> &out_vertices, QVector<int> &out_indices)
{
    // Gnerate a mesh of occupied voxels for rendering
    generateMesh();

    // Extract vertices from the mesh
    const std::vector<PositionMaterialNormal> &meshVertices = mesh.getVertices();
    qDebug()<<"size of mesh vertices = "<<meshVertices.size()<<"\n";

    // Covert the data type to comfortably use QVector (portable across the project)
    for(unsigned long i=0; i< meshVertices.size(); i++)
    {
        // Extract vertex coordinates
        const Vector3DFloat pos = meshVertices[i].getPosition();
        out_vertices.append({pos.getX(), pos.getY(), pos.getZ()});

        // Extract normal and append to the vertex container
        const Vector3DFloat nor = meshVertices[i].getNormal();
        out_vertices.append({nor.getX(), nor.getY(), nor.getZ()});

        // Add the value indicating color for the point - '0' green '1' red '2' blue
        out_vertices.append({0.39f, 1.0f, 0.0f});
    }

    // Get the vertex indices from the mesh
    const std::vector<uint32_t>& meshIndices = mesh.getIndices();
    qDebug()<<"size of mesh indices = "<<meshIndices.size()<<"\n";

    // Convert to use QVector
    for (unsigned long j = 0; j < meshIndices.size(); j++)
    {
        out_indices.append(static_cast<int>(meshIndices[j]));
    }

    return true;
}

bool VoxelWorld::getVertexList(QVector<GLfloat> &out_vertices, QVector<int> &out_values)
{
    // Setup output variables
    for (int x = 0; x < volData.getWidth(); x++)
    {
        for (int z = 0; z < volData.getDepth(); z++)
        {
            for (int y = 0; y < volData.getHeight(); y++)
            {
                MaterialDensityPair44 voxel = volData.getVoxelAt(x, y, z);

                // Collect all the vertices and their density values
                out_vertices.append(x);
                out_vertices.append(y);
                out_vertices.append(z);

                out_values.append(voxel.getDensity());
            }
        }
    }
    return true;
}

void VoxelWorld::OptimizeCamPos(QVector<GLfloat> &out_verticesCam, QVector<GLfloat> &out_verticesCont)
{
    // Vector to hold optimized camera locations
    QVector<QVector3D> optCamPos;
    QVector<bool> optControlPoints;
    // Vector to hold the locations of optimized camera positions inside our list of boundary points
    QVector<int> optCamInd;
    // Vector to hold index of selected orientation from the 9 possible ones
    QVector<int> optRotInd;

    // Generate list of all control Points (background points)
    getBackground();
    // Randomly select 10th of camera positions and 100th of control points
    QVector<QVector3D> backPoints, boundPoints;
    QVector<QVector<QVector3D>> boundNormals;
    srand((int)QTime::currentTime().msec());
    for (int i = 0; i < boundaryVoxels.size(); i++)
    {
        int r = (rand() % 100) + 1;
        //qDebug() << r <<" ";
        if(r <= 5)
        {
            //int r1 = (rand() % 100) + 1;
            //if(r1 <= 30)
            //{
            boundPoints.append(boundaryVoxels[i]);
            boundNormals.append(boundaryNormals[i]);
            //}
        }
    }
    srand((int)QTime::currentTime().msec());
    for (int i = 0; i < backgroundPoints.size(); i++)
    {
        int r = (rand() % 100) + 1;
        //cout << r <<" ";`
        if(r <= 1)
            backPoints.append(backgroundPoints[i]);
    }

    qDebug()<< backPoints.size() <<"/"<< backgroundPoints.size()<< "Number of background points collected\n";
    qDebug()<< boundPoints.size() <<"/"<< boundaryVoxels.size() << "Number of camera locations selected\n";
    // Call the optimization function with available data
    optimize.optimizeObjective(backPoints, boundPoints, boundNormals, optCamPos, optControlPoints, optCamInd, optRotInd);

    /*  Setup data in a way that our openGL widget can display
     *  The shader requires a normal accompanying each vertex
     *  Camera points and frustum are passed in pairs to draw the frustum lines
     */
    for (int i = 0; i < optCamInd.size(); i++)
    {
        // Note: Normal of the bounday voxel point i repeated for all the points of the camera frustum

        int index = optCamInd[i];
        int rotIndex = optRotInd[i];
        int optIndex = i*5;

        /*  For each point an associated value is added to display in different color
         *  optimized camera points and frustums will be displayed in blue i.e, value = 2
         */

        // Add camera location and normal
        out_verticesCam.append({optCamPos[optIndex].x(), optCamPos[optIndex].y(), optCamPos[optIndex].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        // Add frustum top left point
        out_verticesCam.append({optCamPos[optIndex+1].x(), optCamPos[optIndex+1].y(), optCamPos[optIndex+1].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        // add camera location
        out_verticesCam.append({optCamPos[optIndex].x(), optCamPos[optIndex].y(), optCamPos[optIndex].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        // Add frustum bottom left point
        out_verticesCam.append({optCamPos[optIndex+2].x(), optCamPos[optIndex+2].y(), optCamPos[optIndex+2].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        // add camera location
        out_verticesCam.append({optCamPos[optIndex].x(), optCamPos[optIndex].y(), optCamPos[optIndex].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        // Add frustum bottom right point
        out_verticesCam.append({optCamPos[optIndex+3].x(), optCamPos[optIndex+3].y(), optCamPos[optIndex+3].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        // add camera location
        out_verticesCam.append({optCamPos[optIndex].x(), optCamPos[optIndex].y(), optCamPos[optIndex].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        // Add frustum top right point
        out_verticesCam.append({optCamPos[optIndex+4].x(), optCamPos[optIndex+4].y(), optCamPos[optIndex+4].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});

        // above code completes the sides of the FOV pyramid
        // Now we want to draw the base

        // add top left and bottom left points
        out_verticesCam.append({optCamPos[optIndex+1].x(), optCamPos[optIndex+1].y(), optCamPos[optIndex+1].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        out_verticesCam.append({optCamPos[optIndex+2].x(), optCamPos[optIndex+2].y(), optCamPos[optIndex+2].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        // add bottom left and bottom right points
        out_verticesCam.append({optCamPos[optIndex+2].x(), optCamPos[optIndex+2].y(), optCamPos[optIndex+2].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        out_verticesCam.append({optCamPos[optIndex+3].x(), optCamPos[optIndex+3].y(), optCamPos[optIndex+3].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        // add bottom right and top right points
        out_verticesCam.append({optCamPos[optIndex+3].x(), optCamPos[optIndex+3].y(), optCamPos[optIndex+3].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        out_verticesCam.append({optCamPos[optIndex+4].x(), optCamPos[optIndex+4].y(), optCamPos[optIndex+4].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        // add top right and top left points
        out_verticesCam.append({optCamPos[optIndex+4].x(), optCamPos[optIndex+4].y(), optCamPos[optIndex+4].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
        out_verticesCam.append({optCamPos[optIndex+1].x(), optCamPos[optIndex+1].y(), optCamPos[optIndex+1].z()});
        out_verticesCam.append({boundNormals[index][rotIndex].x(), boundNormals[index][rotIndex].y(), boundNormals[index][rotIndex].z()});
        out_verticesCam.append({0.0f, 0.39f, 1.0f});
    }

    // Now setup control points vertices
    for (int i = 0; i < optControlPoints.size(); i++)
    {
        // Add all the covered points to output vertices to be drawn
        // Use zero normal for all points
        // Add control point location
        out_verticesCont.append({backPoints[i].x(), backPoints[i].y(), backPoints[i].z()});
        // add a zero normal for this point
        out_verticesCont.append({1.0f, 1.0f, 1.0f});

        if(optControlPoints[i])
        {
            // Display covered points in green i.e, value = 0
            out_verticesCont.append({0.39f, 1.0f, 0.0f});
        }
        else
        {
            // Display uncovered points in red i.e, value = 1
            out_verticesCont.append({1.0f, 0.0f, 0.0f});
        }

    }
}
