#ifndef LOADOBJ_H
#define LOADOBJ_H

#include <QVector>
#include <QVector3D>
#include <string>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QVector2D>
#include <algorithm>

class LoadOBJ
{
public:
    LoadOBJ();
    bool loadModel(QString &path);
    void getVertices(QVector<QVector3D> &vertexList);

private:
    long totalPoints;
    long totalTriangles;

    // variables to hold object dimensions
    float min_x, max_x, min_y, max_y, min_z, max_z;

    // Function to add ground plane
    QVector<float> addPlane();

    QVector<QVector3D> temp_vertices;
};

#endif // LOADOBJ_H
