#ifndef LOADBINVOX_H
#define LOADBINVOX_H

#include <string>
#include <QDebug>
#include <QVector3D>
#include <QVector>

#include <fstream>
#include <iostream>
#include <stdlib.h>

using namespace std;

typedef unsigned char byte;

class LoadBinVox
{
public:
    // Function to read .binvox files
    bool loadBinVox(QString &path);

    // Function to get the voxel values -- this function returns the number of occupied voxels
    void getVoxels(byte *&voxObject);

    // Function to return dimensions of the voxel object bounding box
    int getDims();

private:
    // object dimensions (in voxels)
    int size;
    // Voxel data/values
    byte *voxels;
    // Total number of occupied voxels
    int nr_voxels;
};

#endif // LOADBINVOX_H
