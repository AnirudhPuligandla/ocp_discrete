#ifndef HEURISTICS_H
#define HEURISTICS_H

#include <string>
#include <math.h>
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <sys/stat.h>

#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/algorithm/string.hpp>

#include <QVector>
#include <QVector3D>
#include <QDebug>
#include <QTime>
#include <QString>
#include <QByteArray>
#include <QFile>
#include <QDataStream>
#include <QElapsedTimer>

#include "visibilitycheck.h"

class Heuristics
{
public:
    /* Function for greedy heuristic algorithm
     * Inputs : controlPoints   -> control points vector
     *          camPos          -> camera locations vector
     *          camNormals      -> camera location (primary) normals vector
     *          visibilityObj   -> Object to visibility checks class that is currently initialized and used
     * Outputs: camSolution     -> output vector to store optimized camera locaiton indices
     *          backSolution    -> output vector to store occupied control points indices
     */
    void greedyAlgo(QVector<QVector3D> &arrayControlCopy,
                    int nCam,
                    VisibilityCheck &visibilityObj,
                    QVector<QVector<int> > &camSolution,
                    QVector<int> &backSolution);
    /* Function for Metropolis sampling algorithm for FIX problem
     * Inputs : controlPoints   -> number of control points
     *          camPos          -> number of camera positions x orientations
     *          numCamReq       -> number of cameras to be placed
     *          camIndice       -> 2D to 1D indices vector pointing to camera positions and orientations
     *          gMatrix         -> 2D visibility matrix with camera positions and orientations stacked together
     * Outputs: camSolution     -> output vector to store optimized camera locaiton indices
     *          backSolution    -> output vector to store occupied control points indices
     */
    void metroSA(int numControl,
                 int numCam,
                 int numCamsRequired,
                 QVector<QVector<int>> &index1D,
                 boost::numeric::ublas::compressed_matrix<bool> &gMat2D,
                 QVector<QVector<int> > &camSolution,
                 QVector<int> &backSolution);
    /* Funtion for RWLS set cover algorithm
     */
    void rwls(int nCam,
              QVector<QVector<int>> &index1D,
              boost::numeric::ublas::compressed_matrix<bool> &gMat2D,
              QVector<QVector<int>> &camSolution,
              QVector<int> &backSolution,
              std::string &fileName, bool mr_or_sr, int numReso);
    // Function for LH-RPSO algorithm
    void LHRPSO(int nCam,
                QVector<QVector<int>> &index1D,
                boost::numeric::ublas::compressed_matrix<bool> &gMat2D,
                QVector<QVector<int>> &camSolution,
                QVector<int> &backSolution);
    void LHRPSO_old(int nCam,
                QVector<QVector<int>> &index1D,
                boost::numeric::ublas::compressed_matrix<bool> &gMat2D,
                QVector<QVector<int>> &camSolution,
                QVector<int> &backSolution);
private:
    // function to get neighbours for each column (used in rwls algorithm)
    void computeNeighbours(boost::numeric::ublas::compressed_matrix<bool> &gMat2D_trans,
                           std::vector<std::vector<int>> &N_j);
    // function to update scores when a camera is added
    void updateScores(std::vector<int> sol_z, int currCam,
                      std::vector<bool> &a_j,
                      boost::numeric::ublas::compressed_matrix<bool> &gMat2D_trans,
                      std::vector<int> &coverageRank,
                      std::vector<std::vector<int> > &N_j,
                      std::vector<int> &weights);
    // funtion to remove camera for rwls algorithm
    void removeScore(std::vector<int> &sol_z, int currCamInd,
                     std::vector<bool> &a_j,
                     boost::numeric::ublas::compressed_matrix<bool> &gMat2D_trans,
                     std::vector<int> &coverageRank,
                     std::vector<std::vector<int> > &N_j,
                     std::vector<int> &weights);
    // function to calculate maximum scores from the neighbourhood matrix
    void computeMaxScores(std::vector<int> &coverageRank,
                          std::vector<std::vector<int> > &N_j,
                          std::vector<int> &maxN_inds);
    // function to calculate total coverage, given a solution
    int computeCoverage(boost::numeric::ublas::compressed_matrix<bool> &gMat2D_trans,
                        std::vector<int> &sol_z, std::vector<int> &uncovered);
    // Function for Latin hypercube sampling
    void LHSampling(std::default_random_engine &generator, int numCam, int numSamples, int numCamPos, std::vector<std::vector<int> > &particleSet);
    // Function to return total coverage of a set of 5 cameras
    int getTotalCov(boost::numeric::ublas::compressed_matrix<bool> &gMat2DCov, std::vector<int> &camSet, std::vector<bool> &backCovSet);
};

#endif // HEURISTICS_H
