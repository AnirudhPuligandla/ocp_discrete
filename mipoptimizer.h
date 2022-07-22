#ifndef MIPOPTIMIZER_H
#define MIPOPTIMIZER_H

#include <QDebug>
#include <QStringList>
#include <QDir>
#include <QFile>
#include <QDataStream>
#include <QVector>
#include <QVector3D>
#include <QProcess>
#include <QTime>
#include <QElapsedTimer>

#include <ilcplex/cplex.h>
#include <ilcplex/ilocplex.h>
#include <ilconcert/ilosys.h>
#include <ilconcert/iloenv.h>
#include <ilconcert/ilomodel.h>
#include <ilconcert/iloexpression.h>
#include <ilcp/cp.h>

#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <cmath>

ILOSTLBEGIN

class MIPOptimizer
{
public:
    // Function for single objective optimization
    void singleObj(int controlPoints,
                   int camPos,
                   int nCam,
                   QVector<QVector<int>> &index1D,
                   boost::numeric::ublas::compressed_matrix<bool> &gMatrix,
                   QVector<QVector<int>> &camSolution,
                   QVector<int> &backSolution);
    // FUnction to run optimization on remote server
    // Returns time taken for optimization
    int remoteOptimization(int controlPoints,
                            int camPos,
                            int nCam,
                            QVector<QVector<int>> &index1D,
                            boost::numeric::ublas::compressed_matrix<bool> &gMatrix,
                            QVector<QVector<int>> &camSolution,
                            QVector<int> &backSolution);
    // Funciton for continuous optimization
    // inputs:
    // controlPoints -> set of control points
    // camPoints     -> set of camera points
    // nCam          -> number of cameras to be placed
    // camParams     -> camera parameters (far plane width and height, z_f)
    // rad           -> radius of hemisphere where cameras are to be placed
    // outputs:
    // camPos        -> set of (theta, phi) of selected camera locations
    // camDir        -> set of (roll, pitch, yaw) of selected cameras' directions
    // occContPoints -> indices of occupied control points
    void continuousOptimization(QVector<QVector3D> &controlPoints,
                                int nCam,
                                QVector3D &camParams,
                                double rad,
                                QVector<QVector3D> &camPos,
                                QVector<QVector3D> &camDir,
                                QVector<int> &backSolution);
};

#endif // MIPOPTIMIZER_H
