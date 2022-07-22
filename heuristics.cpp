#include "heuristics.h"

void Heuristics::greedyAlgo(QVector<QVector3D> &arrayControlCopy,
                            int nCam,
                            VisibilityCheck &visibilityObj,
                            QVector<QVector<int> > &camSolution,
                            QVector<int> &backSolution)
{
    // Copy of control point coordinates to keep track of covered control points
    QVector<QVector3D> backVoxels(arrayControlCopy);
    // Copies of camPos and normals vectors
    //QVector<QVector3D> boundVoxels(camPosVec);
    //QVector<QVector3D> boundNormals(camNormalsVec);

    int controlPoints = backVoxels.size();
    // Max number of cameras allowed
    //int nCam = 5;

    // number of control points covered
    //int nbCovered = 0;

    // Number of cameras placed
    int nCamPlaced = 0;

    // Stopping criteria (minimum required coverage)
    //int reqCoverage = (95 * controlPoints)/100;

    // Get the created gmatrix and selected indices
    boost::numeric::ublas::compressed_matrix<bool> gMatrix;
    QVector<QVector<int>> camIndices;
    visibilityObj.getMatrixAnd1D(gMatrix, camIndices);
    // A copy of indices vector, to make the indices match even after deletion from one
    QVector<QVector<int>> indicesUnaltered(camIndices);

    // Number of camera points
    int camPos = camIndices.size();

    // Intialize output containers for optimized camera positions and covered control points
    QVector<bool> c(controlPoints, 0);
    QVector<bool> var(camPos);
    for(int i = 0; i < camPos; i++)
    {
        var[i] = 0;
    }

    // Container to check for global camera positions
    QVector<int> globalPos(nCam);
    // int to hold global camera position
    int globalPos1;

    // Loop for algorithm
    while (/*nbCovered <= reqCoverage &&*/ nCamPlaced < nCam)
    {
        // create matrix to hold rank
        QVector<int> coverageRank(camPos);

        // Create rank matrix from g matrix
        for(int i = 0; i < camPos; i++)
        {
            int coverageCount = 0;
            for(int j = 0; j < controlPoints; j++)
            {
                coverageCount += gMatrix(i, j);
            }
            coverageRank[i] = coverageCount;
        }

        // -------------Search for highest rank-------------------------------
        int pos1 = 0, pos2 = 0;
        int maxRank = coverageRank[0];
        //QVector<QVector<int>>::iterator i = coverageRank.begin();           // Iterator over camera positions
        QVector<int>::iterator i = coverageRank.begin();
        while (i != coverageRank.end())
        {
            //QVector<int>::iterator j = i->begin();
            //while(j != i->end())
            //{
            if(*i > maxRank)
            {
                maxRank = *i;
                pos1 = std::distance(coverageRank.begin(), i);
                // pos2 = std::distance(i->begin(), j);
            }
            //j++;
            //}
            i++;
        }

        /* Keep track of camera location indices w.r.t 'boundVox'
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

        // Place a camera at the position and orientation with highest rank
        var[globalPos1] = 1;
        // Increment number of newly covered points
        //nbCovered += maxRank;
        // Increment number of cameras placed
        nCamPlaced++;

        // Vector to store indices of occupied control points by the placed camera
        //QVector<int> backOccupiedPoints;

        // Update control points variable with newly covered points
        for(int j = 0; j < controlPoints; j++)
        {
            if(gMatrix(pos1, j) == 1)
            {
                // Search for the index of this control point in the original copied array
                c[arrayControlCopy.indexOf(backVoxels[j])] = 1;
                /* mark coordinates in 'arrayControl' at these indices with the special value (-1, -1, -1).
                 * Any 'special values' with negative values should be safe as the voxel environment has
                 * only positive coordinates.
                 */
                backVoxels[j] = QVector3D(-1.0f, -1.0f, -1.0f);
                // save the index for later use
                //backOccupiedPoints.append(j);
            }
        }

        /* Remove covered control points from original list of control points
         * And update the number of uncovered points
         */
        int numBackRemove = backVoxels.removeAll(QVector3D(-1.0f, -1.0f, -1.0f));

        // Remove the value from the 1Dindices vector
        camIndices.erase(camIndices.begin() + pos1);
        camPos -= 1;
        // Adjust the controlPoints count
        controlPoints -= numBackRemove;

        if(nCamPlaced == nCam)
            continue;

        // setup g matrix with updated possible camera positions and uncovered control points
        //greedyOccupancy = VisibilityCheck(boundVoxels, boundNormals, backVoxels);
        // Recompute the gMatrix for now available points
        visibilityObj.getSolutionoccupation(camIndices, backVoxels, gMatrix);
    }

    // Save the camera locations solution
    //for(int k = 0; k < var.size(); k++)
    //{
    for (int i = 0; i < var.size(); i++)
    {
        if(var[i] == 1)
            camSolution.append(QVector<int>({indicesUnaltered[i][0], indicesUnaltered[i][1]}));
    }
    //greedyAns = var;
    //}
    qDebug() << "Number of cameras placed with greedy algorithm = " << camSolution.size();
    // Save covered control points solution
    float coverageSum = 0;
    for (int i = 0; i < c.size(); i++)
    {
        if(c[i] == 1)
        {
            backSolution.append(i);
            coverageSum++;
        }
    }
    // Display coverage percentage
    qDebug() <<"Coverage % with greedy algorithm = " << (coverageSum/arrayControlCopy.size())*100;
}

void Heuristics::metroSA(int numControl,
                         int numCam, int numCamsRequired,
                         QVector<QVector<int>> &index1D,
                         boost::numeric::ublas::compressed_matrix<bool> &gMat2D,
                         QVector<QVector<int>> &camSolution,
                         QVector<int> &backSolution)
{
    // Let's say we want to place 5 cameras
    //int numCamsRequired = 5;
    // Container to hold randomly selected cameras
    QVector<int> cam;
    // Global solution containers
    int fCamPlace = 0;
    QVector<int> camPlace;

    /* Initialize the W vector. Since this vector is dynamic, it can be defined as vector
     * pointing to the indices in index1D vector. That way even when elements are removed
     * from W, we can still keep track of original camera indices
     */
    QVector<int> W(numCam);
    for(int i = 0; i < numCam; i++)
        W[i] = i;

    /* Generate 5 random numbers between 0 and index1D.size()
     * We also need to ensure they don't have the same position to satisfy the condition
     * of not placing more than one camera with different orientations at each position
     */
    int numCamRemaining = W.size();
    std::default_random_engine generator((int)QTime::currentTime().msec());
    std::uniform_int_distribution<int> distribution(0, numCam-1);
    while(cam.size() < numCamsRequired)
    {
        int randNum = distribution(generator);
        //qDebug() << randNum;
        // Check with previously added cam positions to see if we got new cam at same position but diff orientation
        bool camRepeat = 0;
        for(int i = 0; i < cam.size(); i++)
        {
            // Check position stored in index1D array
            if(index1D[cam[i]][0] == index1D[randNum][0])
                camRepeat = 1;
        }
        // select new random camera only if 'camRepeat' flag is false
        if(!camRepeat)
        {
            // Add the index in randNum into cam vector
            cam.append(W[randNum]);
            /* Use the same strategy as in greedy algorithm
             * Mark the elements with -1 and remove them all at once later
             */
            W[randNum] = -1;
        }
    }
    // Remove all previously marked elements from W
    numCamRemaining -= W.removeAll(-1);
    // Initiaize camPlace with cam to make we dont miss the case of initial random being best solution
    //camPlace = cam;

    /* Algorithm shows to run until N_c, but size of W is already less than N_c
     * That means we will run out of cameras in W before N_c iterations, so run until numCamRemaining
     */
    // Create a new random generator
    std::default_random_engine generator2((int)QTime::currentTime().msec());
    // distribution for selection from cam
    std::uniform_int_distribution<int> distribution2(0, cam.size()-1);
    // Stop when certain number of iterations are complete
    int numIter = 0;
    int maxIter = numCam;
    while((numCamRemaining > 0) && (numIter <= maxIter))
    {
        int bCam = distribution2(generator2);
        // distribution for selection from W
        std::uniform_int_distribution<int> distribution3(0, numCamRemaining-1);
        int cCam = distribution3(generator2);
        // Check if new cam has same position as any one of the 4 cams remaining
        bool camRepeat = 0;
        // Get actual camera index from remaining values in W vector
        int actualCamIndex = W[cCam];
        // get cam index from index1d array for cCam so that we dont have to go through array 4 times
        int newCamPosIndex = index1D[actualCamIndex][0];
        for(int i = 0; i < cam.size(); i++)
        {
            if(index1D[cam[i]][0] == newCamPosIndex)
                camRepeat = 1;
        }
        // if cCam camera position already exists, then just skip the loop
        if(!camRepeat)
        {
            // Create cam_dash with cam - bCam + cCam
            QVector<int> cam_dash = cam;

            // Construct cam_dash by replacing at index bCam with number in index cCam
            cam_dash[bCam] = W[cCam];
            /* Calculate coverage in terms total number points by each cam and cam_dash
             * summing up coverage by individual cameras results in lots of overlap
             */
            int fCam=0, fCam_dash=0;
            for(int j = 0; j < numControl; j++)
            {
                bool camCover = false, cam_dashCover = false;
                // Loop over the number of cameras
                for(int  i = 0; i < cam.size(); i++)
                {
                    camCover = camCover || gMat2D(cam[i], j);
                    cam_dashCover = cam_dashCover || gMat2D(cam_dash[i], j);
                }
                fCam += camCover;
                fCam_dash += cam_dashCover;
            }
            /*
            // -------calculate net coverage of both cam and cam'--------
            int fCam=0, fCam_dash=0, fBCam=0, fCCam=0;
            // first calculate for cam
            for(int i = 0; i < cam.size(); i++)
            {
                int fTemp = 0;
                // Calculate coverage for this camera by looping over number of control points
                for(int j = 0; j < numControl; j++)
                {
                    fTemp += gMat2D(cam[i], j);
                }
                // If this cam Position is same as bCam then also save it separately for later use
                if(i == bCam)
                    fBCam = fTemp;
                // Add the coverage value to fCam
                fCam += fTemp;
            }
            // Calculate coverage for cCam camera position
            for(int j = 0; j < numControl; j++)
                fCCam += gMat2D(actualCamIndex, j);
            // Construct fCam_dash = fCam - fBCam + fCCam
            fCam_dash = fCam - fBCam + fCCam;
            */
            // Calculate deltaH = fCam_dash/fCam
            double deltaH = double(fCam_dash)/double(fCam);
            // Draw a random number between (0,e)
            //std::uniform_real_distribution<double> distributionReal(1.0, exp(1.0)); // number 0 can cause pole error in log function
            std::uniform_real_distribution<double> distributionReal(0.0, 1.0);
            double uNum = distributionReal(generator2);
            // Check condition
            // variables for debugging
            double conditionNum1 = uNum;
            double conditionNum2 = deltaH < 1.0 ? deltaH : 1.0;
            if(conditionNum1 <= conditionNum2)
            {
                cam = cam_dash;
                if(fCam_dash > fCamPlace)
                {
                    // Set final values
                    fCamPlace = fCam_dash;
                    // setup final solution output
                    camPlace = cam_dash;
                }
            }
            // Remove cCam from W
            W.remove(cCam);
            numCamRemaining--;
        }
        // Increment number of iterations
        numIter++;
    }

    // vector to calculate total number of covered background points
    QVector<bool> backgroundCoverageVerify(numControl, 0);
    // Setup Final Solution
    for(int i = 0; i < camPlace.size(); i++)
    {
        int currCamIndex = camPlace[i];
        camSolution.append(QVector<int>({index1D[currCamIndex][0], index1D[currCamIndex][1]}));
        // Get indices of occupied points for this camera
        for(int j= 0; j < numControl; j++)
        {
            if(gMat2D(currCamIndex, j))
            {
                backSolution.append(j);
                // Set the background voxel with value 1
                backgroundCoverageVerify[j] = 1;
            }
        }
    }
    // Calculate the percentage of covered points
    float coverSum = 0.0;
    for(int i = 0; i < numControl; i++)
        coverSum += backgroundCoverageVerify[i];
    // Print result
    qDebug() << "Coverage % with Metropolis Algorithm = " << coverSum/numControl*100.0;
    qDebug() << "Number of cams placed = " << camSolution.size();
}

void Heuristics::rwls(int nCam,
                      QVector<QVector<int> > &index1D,
                      boost::numeric::ublas::compressed_matrix<bool> &gMat2D,
                      QVector<QVector<int> > &camSolution,
                      QVector<int> &backSolution, std::string &fileName,
                      bool mr_or_sr, int numReso)
{
    // Transpose the matrix to get contorl points in rows and camera positions in columns
    boost::numeric::ublas::compressed_matrix<bool> gMat2D_trans = boost::numeric::ublas::trans(gMat2D);
    // Score (no. of covered points) of each camera
    int camPos = index1D.size();
    int controlPoints = gMat2D_trans.size1();
    // Rows not covered by any camera keep adding process in an infinite loop. mark those to use it as break condition
    /*std::vector<bool> emptyRows(controlPoints,0);
    for(int j = 0; j < controlPoints; j++)
    {
        int controlCov = 0;
        for(int i = 0; i < camPos; i++)
        {
            controlCov += gMat2D_trans(j,i);
        }
        if(controlCov==0)
            emptyRows[j] = 1;
    }*/

    std::vector<int> coverageRank(camPos);
    QVector<int> t_j(camPos,0);
    std::vector<bool> a_j(camPos,true);
    std::vector<int> weights(controlPoints, 1);
    // Indices of selected cameras
    std::vector<int> sol_z;
    // Create rank matrix from g matrix
    for(int i = 0; i < camPos; i++)
    {
        int coverageCount = 0;
        for(int j = 0; j < controlPoints; j++)
        {
            coverageCount += gMat2D_trans(j, i) * weights[j];
        }
        coverageRank[i] = coverageCount;
    }

    // Create set of neighbours for each columnm
    // Neighbour cloumns are any columns that have at least one control point covered by both
    std::vector<std::vector<int>> N_j;
    // object to use GPU neighbourhood calculation function
    VisibilityCheck visibObj;
    // Get the filename without extension
    std::vector<std::string> splitFileName;
    boost::split(splitFileName, fileName, [](char c){return c == '.';});
    QString rwlsName;
    if(mr_or_sr)
        rwlsName = QString(QString::fromStdString(splitFileName[0]) + "_MR_" + QString::number(numReso) + ".dat");
    else
        rwlsName = QString(QString::fromStdString(splitFileName[0]) + "_SR_" + ".dat");
    QByteArray rwls_ba = rwlsName.toLatin1();
    struct stat buf;
    // if(1)
    if(!(stat(rwls_ba.data(), &buf) == 0))
    {
        //computeNeighbours(gMat2D_trans, N_j);
        QElapsedTimer neighbourTimer;
        neighbourTimer.start();
        visibObj.computeNeighboursGPU(gMat2D_trans,N_j);
        double neighbourElapsed = neighbourTimer.elapsed();
        qDebug() << "neighbourhood operations time = " << neighbourElapsed << "ms";
        // Write N_j matrix to a file
        QFile rwlsFile(rwlsName);
        rwlsFile.open(QIODevice::WriteOnly);
        QDataStream rwlsOut(&rwlsFile);
        for(int i = 0; i < N_j.size(); i++)
        {
            for(int j = 0; j < N_j[i].size(); j++)
                rwlsOut << (qint32)i << (qint32)N_j[i][j];
        }
        rwlsOut.~QDataStream();
        rwlsFile.close();
    }
    else
    {
        N_j.resize(camPos);
        // If the file exists, read N_j matrx from file
        QFile rwlsFile(rwlsName);
        rwlsFile.open(QIODevice::ReadOnly);
        QDataStream rwlsStream(&rwlsFile);
        qint32 ind1,ind2;
        while(!rwlsFile.atEnd())
        {
            rwlsStream >> ind1 >> ind2;
            N_j[ind1].push_back(ind2);
        }
        rwlsStream.~QDataStream();
        rwlsFile.close();
    }
    // Create a vector of cameras with max score in each neighbourhood set
    std::vector<int> maxN_inds(camPos);
    computeMaxScores(coverageRank, N_j, maxN_inds);
    qDebug() << "Computed neighbours";
    // Vectors to store best solution and best coverage
    int finCov = 0;
    std::vector<int> finSol(nCam);
    // init Loop
    std::default_random_engine generator((int)QTime::currentTime().msec());
    std::uniform_int_distribution<int> distribution(0, camPos-1);
    while(sol_z.size() < nCam)
    {
        // First get a random number in the list of indices
        int randNum = distribution(generator);
        int currSelectCam = maxN_inds[randNum];
        // if the index already exists in sol_z, then skip the iteration
        std::vector<int>::iterator sol_pos = std::find(sol_z.begin(),sol_z.end(), currSelectCam);
        if(sol_pos != std::end(sol_z))
            continue;
        sol_z.push_back(currSelectCam);
        int currCamScore = coverageRank[currSelectCam];
        coverageRank[currSelectCam] = -currCamScore;
        // recalculate the score
        updateScores(sol_z, currSelectCam, a_j, gMat2D_trans,coverageRank,N_j, weights);
        // Recalculate the maximums in neighbourhood
        computeMaxScores(coverageRank, N_j, maxN_inds);
        // Recompute neighbours matrix with the updated scores
        //computeNeighbours(gMat2D_trans, coverageRank, N_j, N_scores);
    }
    //qDebug() << "completed solution initialization";
    // Use greedy algorithm for initialization
    /*QVector<int> back_sol;
    QVector<QVector<int>> camSol;
    greedyAlgo(arrayControl, nCam, visibilityOb, camSol, back_sol);
    // indices have mis-match, find the solution index in index 1D
    for(int i = 0; i < greedyAns.size(); i++)
    {
        if(greedyAns[i]==1)
        {
            sol_z.push_back(i);
            int currCamScore = coverageRank[i];
            coverageRank[i] = -currCamScore;
            updateScores(sol_z, i, a_j, gMat2D_trans,coverageRank,N_j, weights);
        }
    }*/

    // update best solution
    finSol = sol_z;
    // set of uncovered rows
    std::vector<int> finSet_L;
    std::vector<int> set_L;
    finCov = computeCoverage(gMat2D_trans, finSol, set_L);
    finSet_L = set_L;

    std::default_random_engine generator1((int)QTime::currentTime().msec());
    int numIter = 1;
    std::vector<int> tabu_list;
    bool stopCond = false;
    while(numIter <= 250)
    {
        // Get the camera with highest score (closest to zero)
        std::vector<int> sol_scores(sol_z.size());
        std::vector<int> sol_inds(sol_z.size());
        std::size_t noom(0);
        std::generate(std::begin(sol_inds), std::end(sol_inds), [&]{ return noom++; });
        for(int i = 0; i < sol_z.size(); i++)
            sol_scores[i] = coverageRank[sol_z[i]];
        std::sort(sol_inds.begin(), sol_inds.end(), [&](int i1, int i2){return sol_scores[i1] > sol_scores[i2];});
        // Check if the greatest score is in tabu list
        int tempRemoveCam, removeCam;
        bool removeCond = false;
        int solIter = 0;
        int removeCamInd;
        while(removeCond == false)
        {
            removeCamInd = sol_inds[solIter];
            tempRemoveCam = sol_z[removeCamInd];
            auto tabuLoc = std::find(tabu_list.begin(),tabu_list.end(),tempRemoveCam);
            // If it is not in taboo list, then check in case there is a tie and select this camera for removal
            if(tabuLoc == tabu_list.end())
            {
                int altRemoveInd = sol_inds[solIter+1];
                if(sol_scores[removeCamInd] > sol_scores[altRemoveInd])
                {
                    removeCam = tempRemoveCam;
                    removeCond = true;
                }
                else
                {
                    // If there is a tie select the camera with older timestamp
                    if(t_j[removeCamInd] <= t_j[altRemoveInd])
                    {
                        removeCam = tempRemoveCam;
                        removeCond = true;
                    }
                }
            }
            solIter++;
        }
        //qDebug() << "completed camera removal";
        //qDebug() << "remove cam done";
        removeScore(sol_z, removeCamInd, a_j,gMat2D_trans, coverageRank, N_j,weights);
        // Pick a random row from set of uncovered rows
        std::vector<int> addCamsScores, addCamsVec;
        // If the random row is not covered by any camera,
        // then repeat the process and try to cover a different row
        std::uniform_int_distribution<int> distribution1(0, set_L.size()-1);
        int loopCounter = 0;
        while(addCamsScores.size()<=3)
        {
            int randNum = distribution1(generator1);
            int randRow = set_L[randNum];
            for(int i = 0; i < camPos; i++)
            {
                if((gMat2D_trans(randRow, i)==1) && (a_j[i]==true))
                {
                    addCamsScores.push_back(coverageRank[i]);
                    addCamsVec.push_back(i);
                }
            }
            loopCounter++;
            if(loopCounter>=set_L.size())
            {
                stopCond = true;
                break;
            }
        }
        if(stopCond)
            break;
        //qDebug() << "add cam selected";
        std::vector<int> addCamsInds(addCamsScores.size());
        noom = 0;
        std::generate(std::begin(addCamsInds), std::end(addCamsInds), [&]{ return noom++; });
        std::sort(addCamsInds.begin(), addCamsInds.end(), [&](int i1, int i2){return addCamsScores[i1] > addCamsScores[i2];});
        int tempAddCam, addCam;
        bool addCond = false;
        solIter = 0;
        while(addCond == false)
        {
            int addCamera = addCamsInds[solIter];
            tempAddCam = addCamsVec[addCamera];
            auto tabuLoc = std::find(tabu_list.begin(),tabu_list.end(),tempAddCam);
            // If it is not in taboo list, then check in case there is a tie and select this camera for removal
            if(tabuLoc == tabu_list.end())
            {
                int altCameraInd = addCamsInds[solIter+1];
                int altCamera = addCamsVec[altCameraInd];
                if(addCamsScores[addCamera] > addCamsScores[altCameraInd])
                {
                    addCam = tempAddCam;
                    addCond = true;
                }
                else
                {
                    // If there is a tie select the camera with older timestamp
                    //Q_ASSERT(tempAddCam >= 0 && tempAddCam < t_j.size());
                    //int tSize = t_j.size();
                    //Q_ASSERT(altCamera >= 0 && altCamera < t_j.size());
                    if(t_j[tempAddCam] <= t_j[altCamera])
                    {
                        addCam = tempAddCam;
                        addCond = true;
                    }
                }
            }
            solIter++;
        }
        //qDebug() << "camera added";
        //qDebug() << "add cam done";
        //int addCamIndex = addCamsInds[solIter-1];
        sol_z.push_back(addCam);
        int currCamScore = coverageRank[addCam];
        coverageRank[addCam] = -currCamScore;
        // recalculate the score
        updateScores(sol_z, addCam, a_j, gMat2D_trans,coverageRank,N_j, weights);
        // Increment weights of uncovered points by 1
        set_L.clear();
        int updateCov = computeCoverage(gMat2D_trans, sol_z, set_L);
        if(updateCov > finCov)
        {
            finCov = updateCov;
            finSol = sol_z;
            finSet_L = set_L;
        }
        // if set_L consists of only uncovered rows, then exit the loop and present current best as the final solution
        /*int set_LCounter = 0;
        for(int l=0; l < set_L.size(); l++)
            set_LCounter += emptyRows[set_L[l]];
        if(set_LCounter>set_L.size()-3)
            stopCond = true;*/

        for(int i = 0; i < set_L.size(); i++)
            weights[set_L[i]] += 1;
        // put d in tabu list
        if(tabu_list.size()==2)
        {
            tabu_list.erase(tabu_list.begin()+0);
            tabu_list.push_back(addCam);
        }
        else
            tabu_list.push_back(addCam);
        t_j[removeCam] = numIter;
        t_j[addCam] = numIter;
        //qDebug() << "timestamp update done";
        numIter++;
        //qDebug() << "Iteration = " << numIter;
    }
    // Setup output solution
    for(int i = 0; i < finSol.size(); i++)
        camSolution.append(QVector<int>({index1D[finSol[i]][0], index1D[finSol[i]][1]}));
    std::size_t indCounter(0);
    backSolution.resize(controlPoints);
    std::generate(std::begin(backSolution), std::end(backSolution), [&]{ return indCounter++; });
    for(int j = 0; j < finSet_L.size(); j++)
        backSolution[finSet_L[j]] = -1;
    backSolution.removeAll(-1);
    qDebug() << "Coverage % with RWLS = " << (float)finCov/controlPoints*100.0;
    qDebug() << "Number of cams placed = " << camSolution.size();
}

void Heuristics::computeNeighbours(boost::numeric::ublas::compressed_matrix<bool> &gMat2D_trans,
                                   std::vector<std::vector<int> > &N_j)
{
    int camPos = gMat2D_trans.size2();
    int controlPoints = gMat2D_trans.size1();
    N_j.clear();
    N_j.resize(camPos);
    for(int i = 0; i < camPos; i++)
    {
        // Check each column against all other columns (i.e., camera poses)
        for(int i1 = 0; i1 < camPos; i1++)
        {
            if(i1 != i)
            {
                for(int j = 0; j < controlPoints; j++)
                {
                    bool val1 = gMat2D_trans(j,i);
                    bool val2 = gMat2D_trans(j,i1);
                    if(val1==1 && val2==1)
                    {
                        N_j[i].push_back(i1);
                        break;
                    }
                }
            }
        }
    }
}

void Heuristics::computeMaxScores(std::vector<int> &coverageRank, std::vector<std::vector<int> > &N_j, std::vector<int> &maxN_inds)
{
    int camPos = N_j.size();
    for(int i = 0; i < camPos; i++)
    {
        std::vector<int> currN_set = N_j[i];
        // Get the list of neighbour's scores for this camera
        std::vector<int> currN_scores(currN_set.size()+1);
        currN_scores[0] = coverageRank[i];
        for(int neigh = 1; neigh < currN_set.size(); neigh++)
        {
            int neighCam = currN_set[neigh];
            currN_scores[neigh] = coverageRank[neighCam];
        }
        std::vector<int>::iterator maxPos = std::max_element(currN_scores.begin(), currN_scores.end());
        int maxPosInd = std::distance(currN_scores.begin(), maxPos);
        if(maxPosInd==0)
            maxN_inds[i] = i;
        else
            maxN_inds[i] = N_j[i][maxPosInd-1];
    }
}

void Heuristics::updateScores(std::vector<int> sol_z,
                              int currCam,
                              std::vector<bool> &a_j,
                              boost::numeric::ublas::compressed_matrix<bool> &gMat2D_trans,
                              std::vector<int> &coverageRank,
                              std::vector<std::vector<int> > &N_j,
                              std::vector<int> &weights)
{
    std::vector<int> cam_neighbours = N_j[currCam];
    for(int i = 0; i < cam_neighbours.size(); i++)
    {
        int currNeighbourCam = cam_neighbours[i];
        a_j[currNeighbourCam] = true;
        for(int j = 0; j < gMat2D_trans.size1(); j++)
        {
            bool camVal = gMat2D_trans(j,currCam);
            bool camValN = gMat2D_trans(j, currNeighbourCam);
            if((camVal == 1) && (camValN == 1))
            {
                int covCount = 0;
                for(int n = 0; n < sol_z.size(); n++)
                    covCount += gMat2D_trans(j,sol_z[n]);
                if(covCount == 1)
                    coverageRank[currNeighbourCam] -= gMat2D_trans(j,currNeighbourCam) * weights[j];
                else if(covCount == 2)
                    coverageRank[currNeighbourCam] += gMat2D_trans(j,currNeighbourCam) * weights[j];
            }
        }
    }
}

int Heuristics::computeCoverage(boost::numeric::ublas::compressed_matrix<bool> &gMat2D_trans, std::vector<int> &finSol, std::vector<int> &uncovered)
{
    int covNum = 0;
    int controlPoints = gMat2D_trans.size1();
    std::vector<bool> finCoverage(controlPoints,false);
    for(int i = 0; i < finSol.size(); i++)
    {
        for(int j = 0; j < controlPoints; j++)
        {
            bool contVal = gMat2D_trans(j,finSol[i]);
            if(contVal==true)
                finCoverage[j] = true;
        }
    }
    for(int j = 0; j < finCoverage.size(); j++)
    {
        covNum += finCoverage[j];
        if(finCoverage[j]==false)
            uncovered.push_back(j);
    }
    return covNum;
}

void Heuristics::removeScore(std::vector<int> &sol_z,
                             int currCamInd,
                             std::vector<bool> &a_j,
                             boost::numeric::ublas::compressed_matrix<bool> &gMat2D_trans,
                             std::vector<int> &coverageRank,
                             std::vector<std::vector<int> > &N_j,
                             std::vector<int> &weights)
{
    int currCam = sol_z[currCamInd];
    std::vector<int>::iterator currCamIt = sol_z.begin() + currCamInd;
    sol_z.erase(currCamIt);
    int updateScore = coverageRank[currCam];
    coverageRank[currCam] = -updateScore;
    a_j[currCam] = false;
    std::vector<int> cam_neighbours = N_j[currCam];
    for(int i = 0; i < cam_neighbours.size(); i++)
    {
        int currNeighbourCam = cam_neighbours[i];
        a_j[currNeighbourCam] = true;
        for(int j = 0; j < gMat2D_trans.size1(); j++)
        {
            bool camVal = gMat2D_trans(j,currCam);
            bool camValN = gMat2D_trans(j, currNeighbourCam);
            if((camVal == 1) && (camValN == 1))
            {
                int covCount = 0;
                for(int n = 0; n < sol_z.size(); n++)
                    covCount += gMat2D_trans(j,sol_z[n]);
                if(covCount == 1)
                    coverageRank[currNeighbourCam] -= gMat2D_trans(j,currNeighbourCam) * weights[j];
                else if(covCount == 0)
                    coverageRank[currNeighbourCam] += gMat2D_trans(j,currNeighbourCam) * weights[j];
            }
        }
    }
}

void Heuristics::LHRPSO(int nCam, QVector<QVector<int> > &index1D, boost::numeric::ublas::compressed_matrix<bool> &gMat2D, QVector<QVector<int> > &camSolution, QVector<int> &backSolution)
{
    int camPos = gMat2D.size1();
    int controlPoints = gMat2D.size2();
    //-------------------------Parameter Initialization------------------
    int numParticles = 50;
    int maxIter = 500, numIter = 0;
    double w = 0.8, c1 = 0.6, c2 = 1.2; // alternate values (0.8, 0.6, 1.2) or (0.8, 0.3, 0.6)
    double epsilon = 0.01;
    // initialize the global optium as the best in current initialized values
    std::vector<int> p_g, p_g_scores(nCam,-1);
    std::vector<bool> solCoverage(controlPoints);
    int globalCov = 0;

    // random generator
    std::default_random_engine gener((int)QTime::currentTime().msec());
    // Initialize particle positions
    std::vector<std::vector<int>> particlePos;
    LHSampling(gener, nCam, numParticles, camPos, particlePos);
    // as particles are already selected, now initialize the solution vector
    p_g.resize(nCam);
    // initialize particle velocities as 0
    std::vector<std::vector<float>> particleVels(nCam);
    //LHSampling(nCam, numParticles, camPos, particleV);
    // intialize velocities as floating point values
    for(int i = 0; i < nCam; i++)
    {
        particleVels[i].resize(numParticles);
        for(int j = 0; j < numParticles; j++)
            particleVels[i][j] = 0.0;
    }
    // Initialize the local optimum of a particle as the currently initialized positions
    std::vector<int> p_i(nCam);
    int localCov = 0;
    std::vector<bool> localCovSet(controlPoints);
    std::vector<int> p_i_scores(numParticles);
    for(int j = 0; j < numParticles; j++)
    {
        std::vector<int> tempLocalBest(nCam);
        std::vector<bool> tempSolCoverage(controlPoints);
        for(int i = 0; i < nCam; i++)
            tempLocalBest[i] = particlePos[i][j];
        int tempCov = getTotalCov(gMat2D, tempLocalBest, tempSolCoverage);
        p_i_scores[j] = tempCov;
        if(tempCov > localCov)
        {
            p_i = tempLocalBest;
            localCov = tempCov;
            localCovSet = tempSolCoverage;
        }
    }
    // initialize the global solution with current lcoal optimum
    p_g = p_i;
    globalCov = localCov;
    solCoverage = localCovSet;

    std::default_random_engine randGen((double)QTime::currentTime().msec());
    std::uniform_real_distribution<double> randDist(0.0001, 0.9999);
    // ---------------------- PSO Loop ----------------------------------
    while(numIter < maxIter)
    {
        // calculate the variance of all points with global optimal solution as the centroid
        double totalVariance = 0.0;
        for(int i = 0; i < nCam; i++)
        {
            std::vector<float> distances(numParticles);
            for(int j = 0; j < particlePos[i].size(); j++)
                distances[j] = (float)(abs(p_g[i] - particlePos[i][j]))/p_g[i];
            // calculate the variance for this camera dimension
            double sum = std::accumulate(distances.begin(), distances.end(), 0.0);
            double mean = sum / numParticles;
            double sq_sum = std::inner_product(distances.begin(), distances.end(), distances.begin(), 0.0);
            double stdev = std::sqrt(sq_sum / numParticles - mean * mean);
            totalVariance+=stdev;
        }
        totalVariance=totalVariance/nCam;
        // If the variances in any of the dimension is above the threshold, then perform the reinitialization procedure
        if(totalVariance < epsilon)
        {
            // calculate the weights for each particle set
            // first calculate f(x)-b_g and store in a vector
            std::vector<int> f_b_G(numParticles);
            for(int j = 0; j < numParticles; j++)
            {
                f_b_G[j] = pow(p_i_scores[j]-globalCov,2);
            }
            // calculate variance of this vector
            double sum = std::accumulate(f_b_G.begin(), f_b_G.end(), 0.0);
            double mean = sum / numParticles;
            double sq_sum = std::inner_product(f_b_G.begin(), f_b_G.end(), f_b_G.begin(), 0.0);
            double stdev = std::sqrt(sq_sum / numParticles - mean * mean);
            // now calculate weights q_i for all the particles
            std::vector<double> q_i(numParticles);
            for(int j = 0; j < numParticles; j++)
                q_i[j] = (1/sqrt(2.0*stdev*M_PI))*(exp(-pow(f_b_G[j],2) / (2*stdev)));
            // calculate sum of q_i
            double sum_q = std::accumulate(q_i.begin(), q_i.end(), 0.0);
            // calculate weights Q_i for all particles in this dimension
            std::vector<double> Q_i(numParticles);
            for(int j = 0; j < numParticles; j++)
                Q_i[j] = q_i[j]/sum_q;

            // Now initialize a new set of particles
            std::vector<std::vector<int>> particlesNew;
            LHSampling(gener, nCam, numParticles, camPos, particlesNew);

            // If the weight of some particle set is less than threshold, update all those particles with new values
            double q_t = 0.5;
            for(int j = 0; j < numParticles; j++)
            {
                if(Q_i[j] < q_t)
                {
                    for(int i = 0; i < nCam; i++)
                    {
                        particlePos[i][j] = particlesNew[i][j];
                        float velNew = ((maxIter-numIter)/(2*maxIter))*particleVels[i][j];
                        particleVels[i][j] = velNew;
                    }
                }
            }
        }
        // Update local optimal values
        localCov = 0;
        for(int j = 0; j < numParticles; j++)
        {
            std::vector<int> tempLocalBest(nCam);
            std::vector<bool> tempSolCoverage(controlPoints);
            for(int i = 0; i < nCam; i++)
                tempLocalBest[i] = particlePos[i][j];
            int tempCov = getTotalCov(gMat2D, tempLocalBest, tempSolCoverage);
            p_i_scores[j] = tempCov;
            if(tempCov > localCov)
            {
                p_i = tempLocalBest;
                localCov = tempCov;
                localCovSet = tempSolCoverage;
            }
        }
        // update global optimal values
        if(localCov > globalCov)
        {
            p_g = p_i;
            globalCov = localCov;
            solCoverage = localCovSet;
        }
        // update particle positions and velocities for next iteration
        for(int i = 0; i < nCam; i++)
        {
            for(int j = 0; j < numParticles; j++)
            {
                double r1 = randDist(randGen);
                double r2 = randDist(randGen);
                particleVels[i][j] = w*particleVels[i][j] + c1*r1*(p_i[i]-particlePos[i][j]) + c2*r2*(p_g[i]-particlePos[i][j]);
                int posVal = (int)(particlePos[i][j]+particleVels[i][j]);
                // make sure position does not go beyond the number of cameras
                posVal = posVal >= camPos ? camPos-1 : posVal;
                posVal = posVal < 0 ? 0 : posVal;
                particlePos[i][j] = posVal;
            }
        }
        numIter+=1;
    }
    // Setup final solution
    for(int i = 0; i < nCam; i++)
    {
        int currCamIndex = p_g[i];
        camSolution.append(QVector<int>({index1D[currCamIndex][0], index1D[currCamIndex][1]}));
    }
    // setup background coverage
    for(int j = 0; j < controlPoints; j++)
        if(solCoverage[j]==1)
            backSolution.append(j);
    // Print result
    qDebug() << "Coverage % with LH-RPSO Algorithm = " << (float)(backSolution.size()) /controlPoints*100.0;
}

void Heuristics::LHRPSO_old(int nCam,
                        QVector<QVector<int> > &index1D,
                        boost::numeric::ublas::compressed_matrix<bool> &gMat2D,
                        QVector<QVector<int> > &camSolution,
                        QVector<int> &backSolution)
{
    int camPos = gMat2D.size1();
    int controlPoints = gMat2D.size2();
    //-------------------------Parameter Initialization------------------
    int numParticles = 50;
    int maxIter = 2000, numIter = 0;
    double w = 0.6, c1 = 0.1, c2 = 0.3; // alternate values (0.8, 0.6, 1.2) or (0.8, 0.3, 0.6)
    double epsilon = 30;
    // initialize the global optium as the best in current initialized values
    std::vector<int> p_g, p_g_scores(nCam,-1);
    std::vector<bool> solCoverage(controlPoints);
    int globalCov = 0;

    // random generator
    std::default_random_engine gener((int)QTime::currentTime().msec());
    // Initialize particle positions
    std::vector<std::vector<int>> particlePos;
    LHSampling(gener, nCam, numParticles, camPos, particlePos);
    // as particles are already selected, now initialize the solution vector
    p_g.resize(nCam);
    // initialize particle velocities as 0
    std::vector<std::vector<float>> particleVels(nCam);
    //LHSampling(nCam, numParticles, camPos, particleV);
    // intialize velocities as floating point values
    for(int i = 0; i < nCam; i++)
    {
        particleVels[i].resize(numParticles);
        for(int j = 0; j < numParticles; j++)
            particleVels[i][j] = 0.0;
    }
    // To make score calculations efficient, keep the scores for all camera points in a vector
    std::vector<int> coverageRank(camPos);
    for(int i = 0; i < camPos; i++)
    {
        int coverageCount = 0;
        for(int j = 0; j < controlPoints; j++)
            coverageCount += gMat2D(i, j);
        coverageRank[i] = coverageCount;
    }
    // Initialize the local optimum of a particle as the currently initialized positions
    std::vector<std::vector<int>> p_i(nCam), p_i_scores(nCam);
    for(int i = 0; i < nCam; i++)
    {
        p_i[i].resize(numParticles);
        p_i_scores[i].resize(numParticles);
        for(int j = 0; j < numParticles; j++)
            p_i_scores[i][j] = 0;
    }

    std::default_random_engine randGen((double)QTime::currentTime().msec());
    std::uniform_real_distribution<double> randDist(0.0001, 0.9999);
    // ---------------------- PSO Loop ----------------------------------
    while(numIter < maxIter)
    {
        // calculate the variance of all points with global optimal solution as the centroid
        std::vector<std::vector<float>> distances(nCam);
        std::vector<double> variances(nCam);
        for(int i = 0; i < particlePos.size(); i++)
        {
            distances[i].resize(numParticles);
            for(int j = 0; j < particlePos[i].size(); j++)
                distances[i][j] = p_g_scores[i] - particlePos[i][j];
            // calculate the variance for this camera dimension
            double sum = std::accumulate(distances[i].begin(), distances[i].end(), 0.0);
            double mean = sum / numParticles;
            double sq_sum = std::inner_product(distances[i].begin(), distances[i].end(), distances[i].begin(), 0.0);
            double stdev = std::sqrt(sq_sum / numParticles - mean * mean);
            variances[i] = stdev;
        }
        // If the variances in any of the dimension is above the threshold, then perform the reinitialization procedure
        for(int i = 0; i < nCam; i++)
        {
            if(variances[i] < epsilon)
            {
                // first calculate f(x)-b_g and store in a vector
                std::vector<int> f_b_G(numParticles);
                for(int j = 0; j < numParticles; j++)
                    f_b_G[j] = 0;//pow(coverageRank[particlePos[i][j]] - p_g_scores[i],2);
                // calculate variance of this vector
                double sum = std::accumulate(f_b_G.begin(), f_b_G.end(), 0.0);
                double mean = sum / numParticles;
                double sq_sum = std::inner_product(f_b_G.begin(), f_b_G.end(), f_b_G.begin(), 0.0);
                double stdev = std::sqrt(sq_sum / numParticles - mean * mean);
                // now calculate weights q_i for all the particles
                std::vector<double> q_i(numParticles);
                for(int j = 0; j < numParticles; j++)
                    q_i[j] = (1/sqrt(2.0*stdev*M_PI))*(exp(-pow(f_b_G[j],2) / (2*stdev)));
                // calculate sum of q_i
                double sum_q = std::accumulate(q_i.begin(), q_i.end(), 0.0);
                // calculate weights Q_i for all particles in this dimension
                std::vector<double> Q_i(numParticles);
                for(int j = 0; j < numParticles; j++)
                    Q_i[j] = q_i[j]/sum_q;
                double q_t = 0.5;
                std::vector<double> removeInds;
                int removeCount = 0;
                // Get the count and indices of particles with weight less than threshold
                for(int j = 0; j < numParticles; j++)
                {
                    if(Q_i[j]<q_t)
                    {
                        removeInds.push_back(j);
                        removeCount+=1;
                    }
                }
                // initialize a set of new particle positions and velocities for this solution camera
                std::vector<std::vector<int>> particlesNew;
                LHSampling(gener, 1, removeCount, camPos, particlesNew);
                std::vector<std::vector<float>> particlesNewV(1);
                particlesNewV[0].resize(removeCount);
                for(int j = 0; j < removeCount; j++)
                    particlesNewV[0][j] = 0.0f;
                //LHSampling(1, removeCount, camPos, particlesNewV);
                // Replace particle positions in particles vectors while updating velocities
                for(int j = 0; j < removeCount; j++)
                {
                    int replaceInd = removeInds[j];
                    particlePos[i][replaceInd] = particlesNew[0][j];
                    float velNew = (((maxIter+numIter)/(2*maxIter)) * particlesNewV[0][j]) + (((maxIter-numIter)/(2*maxIter))*particleVels[i][replaceInd]);
                    particleVels[i][replaceInd] = velNew;
                    // also reset the scores of this particle
                    p_i_scores[i][replaceInd] = 0;
                }
                //p_g_scores[i] = 0;
            }
        }
        // Update local optimal values
        for(int i = 0; i < nCam; i++)
        {
            for(int j = 0; j < numParticles; j++)
            {
                int currParticle = particlePos[i][j];
                int partScore = coverageRank[currParticle];
                if(partScore > p_i_scores[i][j])
                {
                    p_i[i][j] = currParticle;
                    p_i_scores[i][j] = partScore;
                }
            }
        }
        // update global optimal values
        std::vector<int> tempP_g(nCam);
        for(int i = 0; i < nCam; i++)
        {
            // create a vector of scores for ech camera and pick the highest score as global optimum
            std::vector<int> currCamScores(numParticles);
            for(int j = 0; j < numParticles; j++)
                currCamScores[j] = p_i_scores[i][j];
            std::vector<int>::iterator maxPos = std::max_element(currCamScores.begin(), currCamScores.end());
            int maxPosInd = std::distance(currCamScores.begin(), maxPos);
            int currOptCam = p_i[i][maxPosInd];
            //int currOptScore = p_i_scores[i][maxPosInd];
            // add this to p_g only if it does not already exist
            std::vector<int>::iterator p_gPos = std::find(p_g.begin(), p_g.end(), currOptCam);
            if(p_gPos == p_g.end())
            {
                tempP_g[i] = currOptCam;
            }
        }
        // if this solution is better than current best, update the final solution
        int currCov = 0;
        std::vector<bool> currControlCover(controlPoints);
        for(int i = 0; i < nCam; i++)
        {
            for(int j = 0; j < controlPoints; j++)
            {
                int currCamInd = tempP_g[i];
                if(gMat2D(currCamInd,j)==1)
                    currControlCover[j] = 1;
            }
        }
        for(int j = 0; j < controlPoints; j++)
            currCov += currControlCover[j];
        if(currCov > globalCov)
        {
            globalCov = currCov;
            p_g = tempP_g;
            solCoverage = currControlCover;
        }
        // update particle positions and velocities for next iteration
        for(int i = 0; i < nCam; i++)
        {
            for(int j = 0; j < numParticles; j++)
            {
                double r1 = randDist(randGen);
                double r2 = randDist(randGen);
                particleVels[i][j] = w*particleVels[i][j] + c1*r1*(p_i[i][j]-particlePos[i][j]) + c2*r2*(p_g[i]-particlePos[i][j]);
                int posVal = (int)(particlePos[i][j]+particleVels[i][j]);
                // make sure position does not go beyond the number of cameras
                posVal = posVal >= camPos ? camPos-1 : posVal;
                posVal = posVal < 0 ? 0 : posVal;
                particlePos[i][j] = posVal;
            }
        }
        numIter+=1;
    }
    // Setup final solution
    for(int i = 0; i < nCam; i++)
    {
        int currCamIndex = p_g[i];
        camSolution.append(QVector<int>({index1D[currCamIndex][0], index1D[currCamIndex][1]}));
    }
    // setup background coverage
    for(int j = 0; j < controlPoints; j++)
        if(solCoverage[j]==1)
            backSolution.append(j);
    // Print result
    qDebug() << "Coverage % with LH-RPSO Algorithm = " << (float)(backSolution.size()) /controlPoints*100.0;
    //qDebug() << "Number of cams placed = " << camSolution.size();
    //qDebug() << "Debugger hold!";
}

void Heuristics::LHSampling(std::default_random_engine &generator, int numCam, int numSamples, int numCamPos, std::vector<std::vector<int>> &particleSet)
{
    particleSet.clear();
    particleSet.resize(numCam);
    std::vector<std::vector<int>> tempParticles(numSamples);
    // vector of camera indices
    std::vector<int> set_s(numCamPos);
    std::size_t noom(0);
    std::generate(set_s.begin(), set_s.end(), [&]{ return noom++; });
    for(int i = 0; i < numSamples; i++)
    {
        //tempParticles[i].clear();
        // dividie set s into numCam intevrvals
        int interval = set_s.size()/numCam;
        for(int j = 0; j < numCam; j++)
        {
            int intStart = j*interval;
            int intEnd = intStart + interval < set_s.size() ? intStart+interval : set_s.size()-1;
            // pick a random number between intStart and intEnd
            std::uniform_int_distribution<int> distribution(intStart, intEnd);
            int randNum = distribution(generator);
            int currRandParticle = set_s[randNum];
            tempParticles[i].push_back(currRandParticle);
        }
    }
    for(int j = 0; j < numCam; j++)
    {
        //particleSet[j].clear();
        for(int i = 0; i < numSamples; i++)
        {
            std::uniform_int_distribution<int> distribution1(0, tempParticles[i].size()-1);
            int randNum = distribution1(generator);
            particleSet[j].push_back(tempParticles[i][randNum]);
            // remove this entry from tempParticles
            tempParticles[i].erase(tempParticles[i].begin()+randNum);
        }
    }
}

int Heuristics::getTotalCov(boost::numeric::ublas::compressed_matrix<bool> &gMat2DCov, std::vector<int> &camSet, std::vector<bool> &backCovSet)
{
    int numCams = camSet.size();
    int numBack = backCovSet.size();
    int totalCov = 0;
    for(int i = 0; i < numCams; i++)
    {
        for(int j = 0; j < numBack; j++)
        {
            bool currCover = gMat2DCov(camSet[i], j);
            if(currCover)
                backCovSet[j] = 1;
        }
    }
    for(int j = 0; j < numBack; j++)
        totalCov += backCovSet[j];
    return  totalCov;
}
