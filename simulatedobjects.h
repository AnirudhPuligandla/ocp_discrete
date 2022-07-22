#ifndef SIMULATEDOBJECTS_H
#define SIMULATEDOBJECTS_H

#include <vtkSmartPointer.h>
#include <vtkStructuredPoints.h>

#include <QVector>
#include <QVector3D>

#include <string>
#include <math.h>

class SimulatedObjects
{
private:
    // Voxel volume for simulated objects
    vtkSmartPointer<vtkStructuredPoints> voxSimObject;
    // Containers to save camera points, camera normals and vehicle voxels
    QVector<QVector3D> camPoints, camNormals, vehVox;
    // Container to save vector indices values
    QVector<unsigned long int> vectorIndices;
    // Dimensions of the simulated vehicle in x,y,z limits
    int objDimensions[6];
    // A variable to define the max height of the simulated object
    // This is useful when defining the bowl surface around the vehicle
    int objMaxH, objMaxL, objMaxW;
    // variable to define the size of the object
    // Defined in 5 sizes, Sample dimensions for simple car are shown here
    // Tiny     - 0   (7x5x5)
    // Small    - 1   (9x5x5)
    // Medium   - 2   (11x7x7)
    // Large    - 3   (17x13x13)
    // Huge     - 4   (23x19x19)
    int sizeMark = 1;
    // class variable for defining volume extent
    int extent[6];

protected:
    // Function to create a simple car-like structue with two boxes
    void simpleCar();
    // Function to create a conplex car-like structure.
    // Same as simple car , but top part with just the frame (remove windows)
    void van();
    // Function to create long car
    // Front part is like a van, rear part as a box (ends at full length / hatchback)
    // front and rear parts at exact halves
    void hatchBack();
    // Function to create a truck
    // The top part is a third of length placed at the front (in simple car it is in the center)
    void truck();
    // Function to create a bus
    // This is a simple bottom less box
    void bus();

public:
    // Constructor that takes a string as input
    // the string specifies the object to be created
    // Possible choices:
    // 1. simple car
    // 2. Van
    // 3. Hatchback
    // 4. Truck
    // 5. Bus
    SimulatedObjects(int objectChoice);
    // Function to get the voxel environment with simulted vehicle
    // Also passes the boundary voxels, normals, vehicle voxels and vectorIndices vectors
    vtkSmartPointer<vtkStructuredPoints>& getSimData(QVector<QVector3D> &boundaryVoxels,
                                                     QVector<QVector3D> &boundaryNormals,
                                                     QVector<QVector3D> &vehicleVox,
                                                     QVector<unsigned long int> &vecInd);
    // Function to return the max height of the simulated object
    void getVehicleDims(int vehicleDims[3]);
};

#endif // SIMULATEDOBJECTS_H
