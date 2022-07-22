#include "mipoptimizer.h"

void MIPOptimizer::singleObj(int controlPoints,
                             int camPos,
                             int nCam,
                             QVector<QVector<int> > &index1D,
                             boost::numeric::ublas::compressed_matrix<bool> &gMatrix,
                             QVector<QVector<int> > &camSolution,
                             QVector<int> &backSolution)
{
    IloEnv env;

    try
    {
        // Setup the model and variable arrays
        IloModel model(env);
        IloNumVarArray var(env);
        IloNumVarArray c(env);
        IloObjective coverage(env);
        // create constraints array
        IloRangeArray con(env);
        /* Setup control points variable (an array of boolean variables indicating points to be covered)
         * each variable in c will be either 0 or 1 depending whether the point is covered by only one camera or not
         */
        for (int i = 0; i < controlPoints; i++)
        {
            c.add(IloNumVar(env, 0, 1, ILOBOOL));
        }

        /* Initialize var (variable indicating optimized camera positions)
         * var has a size equal to the number of possible camera locations
         * Each variable in var will be either 0 or 1 depending on whether or not a camera is placed at that location
         */
        for (int i = 0; i < camPos; i++)
        {
            var.add(IloNumVar(env, 0, 1, ILOBOOL));
        }

        /* Add objective function to the model
         * objective -- max sum_i (c[i])
         */
        //IloObjective coverage(env);
        coverage.setExpr(IloSum(c));
        coverage.setSense(IloObjective::Maximize);

        // Array representing variable 'v' in the paper and initialize it
        IloArray<IloNumVarArray> v(env, controlPoints);
        for(IloInt i = 0; i < controlPoints; i++)
        {
            v[i] = IloNumVarArray(env, camPos);
            for(IloInt j = 0; j < camPos; j++)
            {
                v[i][j] = IloNumVar(env, 0, 1, ILOBOOL);
            }
        }

        int M = 1;
        qDebug()<< "------Using "<<nCam<<" cameras---------------";
        //iterate over blocks of control points
        for (IloInt i = 0; i < controlPoints; i++)
        {
            IloExpr v_g(env);
            // Iterate over the possible camera locations
            for (IloInt j = 0; j < camPos; j++)
            {
                /* v_g needs to be linearized because it is product of two binary variables c and var
                 * The following conditions can be used to linearize this product (https://www.leandro-coelho.com/linearization-product-variables/)
                 * v_g[j] <= c[i]
                 * v_g[j] <= var[j]
                 * v_g[j] >= c[i] + var[j] - 1
                 */
                con.add(v[i][j] - var[j] <= 0);
                con.add(v[i][j] - c[i] <= 0);
                con.add(v[i][j] - c[i] - var[j] + 1 >= 0);
                // Expression to represent sum in contraint (15)
                v_g += v[i][j] * gMatrix(j,i);
            }
            // Constraint (15)
            con.add(v_g - c[i] * M >= 0);
            v_g.end();      // very important to end after adding to the model
        }

        IloExprArray var_sum(env);
        for(IloInt i = 0; i < camPos; i++)
        {
            // expression for constraint (20)
            var_sum.add(var[i]);
        }
        // constraint (20)
        con.add(IloSum(var_sum) == nCam);
        var_sum.end();      // very important to end after adding to the model

        /* The index1D container has to be used to check for the constraint to check
         * that multiple cameras are not placed at the same camera position
         */
        // vector that contains, at each entry, vector of indices that have same camera position
        QVector<QVector<int>> indexByCamPos;
        // vector that forms one entry to the indexByCamPos vector
        QVector<int> sameCamPos;
        for(int i = 0; i < index1D.size(); i++)
        {
            // Look for the current index in 'sameCamPos' vector
            int currCamInd = sameCamPos.indexOf(index1D[i][0]);
            // If index is -1, then append camera pos to vector
            // Also append 1D index in indexByCamPos vector as qVector<int>
            if(currCamInd == -1)
            {
                sameCamPos.append(index1D[i][0]);
                indexByCamPos.append(QVector<int>({i}));
            }
            // else, append only 1D index in indexByCamPos at the obtained index
            else
            {
                indexByCamPos[currCamInd].append(i);
            }
        }
        // Loop over created vector and establish constraints
        for(int i = 0; i < indexByCamPos.size(); i++)
        {
            // Expression to represent sum of all orientations for a camera position
            IloExprArray xPosSum(env);
            // Loop over the indices at this entry and create the linear expression
            for(int j = 0; j < indexByCamPos[i].size(); j++)
                xPosSum.add(var[indexByCamPos[i][j]]);
            // Constraint 19
            con.add(IloSum(xPosSum) <= 1);
            xPosSum.end();
        }

        // Add the coverage maximization objective to the model
        model.add(coverage);
        // Setup model constraints
        model.add(con);
        qDebug()<<"Model setup with... "<<var.getSize()<<" variables "<<c.getSize()<<" control points and "<<con.getSize()<<" constraints";

        // Arrays to grab the outputs
        IloNumArray camVals(env);
        IloNumArray contVals(env);

        IloCplex cplex(model);

        cplex.setParam(IloCplex::PreDual, 1);                                                                                       // Turn on presolve dual parameter
        //cplex.setParam(IloCplex::NodeFileInd, 3);                                                                                   // Enable writing node files into disk in compressed format
        //cplex.setParam(IloCplex::WorkDir, "/media/anirudh/Data/Documents/PhD/Qt_projects/ocp_simulation/resources/gMat32Down/");    // Change working directory where node files will be saved
        cplex.setParam(IloCplex::VarSel, 4);                                                                                        // Set to strong branching
        //cplex.setParam(IloCplex::NodeSel, 0);                                                                                       // This should keep CPLEX from essentially running out of memory
        //cplex.setParam(IloCplex::ParallelMode, 1);                                                                                  // Choose deterministic mode
        //cplex.setParam(IloCplex::Threads, 1);                                                                                       // Indicate how many threads to use
        //cplex.setParam(IloCplex::RINSHeur, 100);                                                                                    // Node heuristic
        cplex.setParam(IloCplex::Param::MultiObjective::Display, 2);                                                                // set multi-objective display level to "detailed"
        cplex.setParam(IloCplex::MIPEmphasis, 1);                                                                                   // emphasis on feasibility over optimality
        cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.06);                                                             // allow a tolerance of 6%
        cplex.setParam(IloCplex::RootAlg, 4);                                                                                       // select barrier algorithm for root relaxation
        cplex.setParam(IloCplex::NodeAlg, 4);

        // optimize the problem and obtain solution
        if(!cplex.solve())
        {
            env.error() << "Failed to optimize LP" << endl;
            throw(-1);
        }

        env.out() << "Solution staus = " << cplex.getStatus() << endl;
        env.out() <<"Solution value = " << cplex.getObjValue() << endl;
        cplex.getValues(camVals, var);
        cplex.getValues(contVals, c);
        // Set up the solutions
        for(int i = 0; i < camPos; i++)
        {
            if((int)camVals[i] == 1)
                camSolution.append(QVector<int>({index1D[i][0], index1D[i][1]}));
        }
        // Indices of occupied background points
        float coverageSum = 0;
        for(int i = 0; i < controlPoints; i++)
        {
            if((int)contVals[i] == 1)
            {
                backSolution.append(i);
                coverageSum++;
            }
        }
        qDebug() << "Number of cameras placed with CPLEX MIP optimizer = " << camSolution.size();
        qDebug() <<"Coverage % with MIP optimizer = " << (coverageSum/controlPoints)*100;
    }
    catch (IloException& e)
    {
        cerr << "Concert exception caught: "<< e << endl;
    }
    catch (...)
    {
        cerr << "Unknown exception caught"<< endl;
    }
}

int MIPOptimizer::remoteOptimization(int controlPoints,
                                      int camPos,
                                      int nCam,
                                      QVector<QVector<int>> &index1D,
                                      boost::numeric::ublas::compressed_matrix<bool> &gMatrix,
                                      QVector<QVector<int>> &camSolution,
                                      QVector<int> &backSolution)
{
    // Write gMatrix and index1D vector into files
    // Write the calculated g matrix to binary files
    QString dirName("/media/anirudh/Data/Documents/PhD/Qt_projects/virtual_machine/MIP_optimizer/gmat32Down/");
    QString fileName(dirName + "gMat.dat");
    QString fileNameInd(dirName + "index1D.dat");

    QFile file(fileName);
    file.open(QIODevice::WriteOnly);
    QDataStream out(&file);
    // Loop over the gMatrix and write into file the indices where the value shows 1
    for(int i = 0; i < camPos; i++)
    {
        for(int j = 0; j < controlPoints; j++)
        {
            if(gMatrix(i, j) == 1)
                out << (qint32)i << (qint32)j;
        }
    }
    out.~QDataStream();
    file.close();
    // Index1D into file
    QFile fileInd(fileNameInd);
    fileInd.open(QIODevice::WriteOnly);
    QDataStream outInd(&fileInd);
    for(int i = 0; i < index1D.size(); i++)
        outInd << (qint32)index1D[i][0] << (qint32)index1D[i][1];
    outInd.~QDataStream();
    fileInd.close();

    // To estimate time taken
    int timeMS;
    QElapsedTimer optimizationTimer;

    // Call the optimizer
    optimizationTimer.start();
    QString command = "ssh anirudh@10.53.6.23 ./../../media/data/MIP_optimizer/bin/MIP_optimizer " + QString::number(camPos) + " " + QString::number(controlPoints) + " " + QString::number(nCam) + " " + QString::number(0) + " /media/data/MIP_optimizer/gmat32Down/";
    QProcess *virtualMachine = new QProcess();
    qDebug()<<"Calling optimization process on remote machine: " << command;
    virtualMachine->setProcessChannelMode(QProcess::ForwardedChannels);
    virtualMachine->start(command);

    if(virtualMachine->waitForFinished(-1))
        qDebug()<< "Optimization complete. Reading output data from files...";

    timeMS = optimizationTimer.elapsed();

    // Read the output files and extract the solution
    qint32 i,j;
    // First read camera indices
    QFile camFile(dirName + "outputCam.dat");
    camFile.open(QIODevice::ReadOnly);
    QDataStream inCam(&camFile);
    while(!camFile.atEnd())
    {
        inCam >> i;
        camSolution.append(QVector<int>({index1D[i][0], index1D[i][1]}));
    }
    camFile.close();
    // Next setup the coverage solution vector
    float coverageSum = 0;
    QFile contFile(dirName + "outputCont.dat");
    contFile.open(QIODevice::ReadOnly);
    QDataStream inCont(&contFile);
    while(!contFile.atEnd())
    {
        inCont >> j;
        backSolution.append(i);
        coverageSum++;
    }
    contFile.close();

    qDebug() << "Number of cameras placed with CPLEX MIP optimizer = " << camSolution.size();
    qDebug() <<"Coverage % with MIP optimizer = " << (coverageSum/controlPoints)*100;

    return timeMS;
}

void MIPOptimizer::continuousOptimization(QVector<QVector3D> &controlPoints,
                                          int nCam,
                                          QVector3D &camParams,
                                          double rad,
                                          QVector<QVector3D> &camPos,
                                          QVector<QVector3D> &camDir,
                                          QVector<int> &backSolution)
{
    IloEnv env;

    try
    {
        // Setup the model and variable arrays
        IloModel model(env);
        // Objective (maximize coverage)
        IloObjective coverage(env);
        // create constraints array
        IloRangeArray con(env);
        //-------setup control points array-------------
        // C is array of boolean variables
        IloNumVarArray c(env);
        int numControl = controlPoints.size();
        for (int i = 0; i < numControl; i++)
        {
            c.add(IloNumVar(env, 0, 1, ILOBOOL));
        }
        //----setup 5 variables constituting cameras----
        IloNumVarArray theta(env), phi(env), roll(env), pitch(env), yaw(env);
        for(int i = 0; i < nCam; i++)
        {
            theta.add(IloNumVar(env, 0.0, M_PI/2, ILOFLOAT));
            phi.add(IloNumVar(env, 0.0, M_PI, ILOFLOAT));
            // rotations have [-1,1]
            // but -1 and 1 in pitch conflict with up and right vector calculations, so avoid it
            roll.add(IloNumVar(env, -1.0, 1.0, ILOFLOAT));
            pitch.add(IloNumVar(env, -0.99, 0.99, ILOFLOAT));
            yaw.add(IloNumVar(env, -1.0, 1.0, ILOFLOAT));
        }
        // ------------Setup Objective------------------
        //IloObjective coverage(env);
        coverage.setExpr(IloSum(c));
        coverage.setSense(IloObjective::Maximize);
        // ------Setting up constraints-----------------
        for(int j = 0; j < numControl; j++)
        {
            QVector3D currControl = controlPoints[j];
            for(int i = 0; i < nCam; i++)
            {
                // Expessions for checking if point lies in FOV of placed camera
                // see calculations for point in FOV here: https://math.stackexchange.com/questions/4144827/determine-if-a-point-is-in-a-cameras-field-of-view-3d

                // The camera is to be placed on the hemisphere at (theta, phi).
                // So translate the background point by camera position to place at FOV at origin.
                // First convert camer alocation from spherical to cartesian coordinates.
                IloExpr camCart_x(env), camCart_y(env), camCart_z(env);
                camCart_x = rad * IloSin(phi[i]) * IloCos(theta[i]);
                camCart_y = rad * IloCos(phi[i]);
                camCart_z = rad * IloSin(phi[i]) * IloSin(theta[i]);
                // translate control by these cartesian coordinates
                IloExpr controlTrans_x(env), controlTrans_y(env), controlTrans_z(env);
                controlTrans_x = currControl.x() - camCart_x;
                controlTrans_y = currControl.y() - camCart_y;
                controlTrans_z = currControl.z() - camCart_z;
                //float controlTrans_x = currControl.x(), controlTrans_y = currControl.y(), controlTrans_z = currControl.z();
                //----------------expression d----------------------------------------------
                IloExpr d(env);
                // Calculate d as the dot product between control point and (roll, theta, phi)
                d = (controlTrans_x * roll[i]) + (controlTrans_y * pitch[i]) + (controlTrans_z * yaw[i]);
                // add the condition d > 0. Note: works only with >=
                con.add(d >= 0.001);
                // --calculate projection of control points on far plane-------
                // Note: camera parameters are passed as (h_FOV, v_FOV, z_f)
                IloExpr c_dash_x(env), c_dash_y(env), c_dash_z(env);
                c_dash_x = camParams.z() * (controlTrans_x/d - roll[i]);
                c_dash_y = camParams.z() * (controlTrans_y/d - pitch[i]);
                c_dash_z = camParams.z() * (controlTrans_z/d - yaw[i]);
                //------Calculate camera up and right vectors------------------
                // World up vector
                QVector3D worldUp(0.0f, 1.0f, 0.0f);
                IloExpr up_x(env), up_y(env), up_z(env), up_norm(env);
                IloExpr right_x(env), right_y(env), right_z(env), right_norm(env);
                // Right vector is normalized cross product of (roll,pitch,yaw) and worldUp
                right_x = pitch[i]*worldUp.z() - yaw[i]*worldUp.y();
                right_y = yaw[i]*worldUp.x() - roll[i]*worldUp.z();
                right_z = roll[i]*worldUp.y() - pitch[i]*worldUp.x();
                right_norm = IloPower((IloSquare(right_x)+IloSquare(right_y)+IloSquare(right_z)), 0.5);
                // normalize right vector
                right_x = right_x/right_norm;
                right_y = right_y/right_norm;
                right_z = right_z/right_norm;
                // Up vector is normalized cross product of right vector and (roll,pitch,yaw)
                up_x = right_y*yaw[i] - right_z*pitch[i];
                up_y = right_z*roll[i] - right_x*yaw[i];
                up_z = right_x*pitch[i] - right_y*roll[i];
                up_norm = IloPower(IloSquare(up_x)+IloSquare(up_y)+IloSquare(up_z), 0.5);
                // normalize up vector
                up_x = up_x/up_norm;
                up_y = up_y/up_norm;
                up_z = up_z/up_norm;
                //--------------expression u_dash, v_dash----------------------
                IloExpr u_dash(env), v_dash(env);
                u_dash = (c_dash_x * right_x) + (c_dash_y * right_y) + (c_dash_z * right_z);
                v_dash = (c_dash_x * up_x) + (c_dash_y * up_y) + (c_dash_z * up_z);
                // These numbers should within dimensions of camera's far plane
                con.add(-camParams.x() <= u_dash <= camParams.x());
                con.add(-camParams.y() <= v_dash <= camParams.y());
                // calculate up vector for camera

                camCart_x.end();
                camCart_y.end();
                camCart_z.end();
                controlTrans_x.end();
                controlTrans_y.end();
                controlTrans_z.end();
                d.end();
                c_dash_x.end();
                c_dash_y.end();
                c_dash_z.end();
                right_x.end();
                right_y.end();
                right_z.end();
                right_norm.end();
                up_x.end();
                up_y.end();
                up_z.end();
                up_norm.end();
                u_dash.end();
                v_dash.end();
            }
        }
        // Add the coverage maximization objective to the model
        model.add(coverage);
        // Setup model constraints
        model.add(con);
        int totalVars = theta.getSize() + phi.getSize() + roll.getSize() + pitch.getSize() + yaw.getSize();
        qDebug()<<"Model setup with... "<< totalVars <<" variables "<< c.getSize()<<" control points and "<<con.getSize()<<" constraints";

        //IloCP cp(model);

        IloCplex cplex(model);
        cplex.setParam(IloCplex::PreDual, 1);                                                                                       // Turn on presolve dual parameter
        //cplex.setParam(IloCplex::NodeFileInd, 3);                                                                                   // Enable writing node files into disk in compressed format
        //cplex.setParam(IloCplex::WorkDir, "/media/anirudh/Data/Documents/PhD/Qt_projects/ocp_simulation/resources/gMat32Down/");    // Change working directory where node files will be saved
        cplex.setParam(IloCplex::VarSel, 4);                                                                                        // Set to strong branching
        //cplex.setParam(IloCplex::NodeSel, 0);                                                                                       // This should keep CPLEX from essentially running out of memory
        //cplex.setParam(IloCplex::ParallelMode, 1);                                                                                  // Choose deterministic mode
        //cplex.setParam(IloCplex::Threads, 1);                                                                                       // Indicate how many threads to use
        //cplex.setParam(IloCplex::RINSHeur, 100);                                                                                    // Node heuristic
        cplex.setParam(IloCplex::Param::MultiObjective::Display, 2);                                                                // set multi-objective display level to "detailed"
        cplex.setParam(IloCplex::MIPEmphasis, 1);                                                                                   // emphasis on feasibility over optimality
        cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.06);                                                             // allow a tolerance of 6%
        cplex.setParam(IloCplex::RootAlg, 4);                                                                                       // select barrier algorithm for root relaxation
        cplex.setParam(IloCplex::NodeAlg, 4);

        // optimize the problem and obtain solution
        if(!cplex.solve())
        {
            env.error() << "Failed to optimize LP" << endl;
            throw(-1);
        }
        // Arrays to grab the outputs
        IloNumArray camTheta(env), camPhi(env), camRoll(env), camPitch(env), camYaw(env);
        IloNumArray contVals(env);

        env.out() << "Solution staus = " << cplex.getStatus() << endl;
        env.out() <<"Solution value = " << cplex.getObjValue() << endl;
        cplex.getValues(camTheta, theta);
        cplex.getValues(camPhi, phi);
        cplex.getValues(camRoll, roll);
        cplex.getValues(camPitch, pitch);
        cplex.getValues(camYaw, yaw);
        cplex.getValues(contVals, c);

        // Set up the solutions
        camPos.resize(nCam);
        camDir.resize(nCam);
        for(int i = 0; i < nCam; i++)
        {
            // Get Cartesian coordiantes for (theta,phi)
            camPos[i].setX(rad * sin(camPhi[i]) * cos(camTheta[i]));
            camPos[i].setY(rad * cos(camPhi[i]));
            camPos[i].setZ(rad * sin(camPhi[i]) * sin(camTheta[i]));
            // Set the camera orientation as (roll, pitch, yaw)
            camDir[i] = QVector3D(camRoll[i], camPitch[i], camYaw[i]);
        }
        // Indices of occupied background points
        float coverageSum = 0;
        for(int i = 0; i < numControl; i++)
        {
            if((int)contVals[i] == 1)
            {
                backSolution.append(i);
                coverageSum += 1;
            }
        }
        qDebug() << "Number of cameras placed with CPLEX MIP optimizer = " << camPos.size();
        qDebug() <<"Coverage % with MIP optimizer = " << (coverageSum/numControl)*100;
    }
    catch (IloException& e)
    {
        cerr << "Concert exception caught: "<< e << endl;
    }
    catch (...)
    {
        cerr << "Unknown exception caught"<< endl;
    }
}
