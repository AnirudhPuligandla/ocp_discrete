#include "loadobj.h"

LoadOBJ::LoadOBJ()
{
    totalPoints = 0;
    totalTriangles = 0;

    min_x = 0.0f;
    max_x = 0.0f;
    min_y = 0.0f;
    max_y = 0.0f;
    min_z = 0.0f;
    max_z = 0.0f;
}

void LoadOBJ::getVertices(QVector<QVector3D> &vertexList)
{
    qDebug()<<"size in loadobj = "<<temp_vertices.size();
//    for (int i = 0; i < temp_vertices.size(); i++)
//    {
//        temp_vertices[i] = temp_vertices[i] + QVector3D(max_x, max_y, max_z);
//    }
    vertexList = temp_vertices;
}

QVector<float> LoadOBJ::addPlane()
{
    QVector<float> groundPlane;
    // Draw a plane at the botoom of the object
    /*
     * plane with 4 points (min_x, -, max_z) (min_x, 0, min_z) (max_x, 0, min_z) (max_x, 0, max_z)
     * normal between two edges of plane - (max_x - min_x, min_y, max_z - min_z) - (min_x - min_x, min_y, min_z - max_z)
     */
    QVector3D n = QVector3D::normal(QVector3D(max_x - min_x, min_y, max_z - min_z), QVector3D(min_x - min_x, min_y, max_z - min_z));

    groundPlane.append({min_x, min_y, max_z});
    groundPlane.append({n.x(), n.y(), n.z()});
    groundPlane.append({max_x, min_y, max_z});
    groundPlane.append({n.x(), n.y(), n.z()});
    groundPlane.append({min_x, min_y, min_z});
    groundPlane.append({n.x(), n.y(), n.z()});

    groundPlane.append({min_x, min_y, min_z});
    groundPlane.append({n.x(), n.y(), n.z()});
    groundPlane.append({max_x, min_y, max_z});
    groundPlane.append({n.x(), n.y(), n.z()});
    groundPlane.append({max_x, min_y, min_z});
    groundPlane.append({n.x(), n.y(), n.z()});

    // scale the ground plane
    std::transform(groundPlane.begin(), groundPlane.end(), groundPlane.begin(),
                   std::bind(std::multiplies<float>(), std::placeholders::_1, 3.0f));

    return groundPlane;
}

bool LoadOBJ::loadModel(QString &path)
{
    qDebug()<<"loading OBJ file "<<path<<"\n";
    QVector<int> vertexIndices, uvIndices, normalIndices;
    QVector<QVector3D> temp_normals, temp_faces;
    QVector<QVector2D> temp_uvs;

    QFile file(path);
    if(!file.open(::QIODevice::ReadOnly|::QIODevice::Text))
    {
        qDebug()<<".obj File cannot be opened. Please check the path.\n"<<QDir::currentPath();
        return false;
    }

    QTextStream stream(&file);
    while(!stream.atEnd())
    {
        QString line = stream.readLine();
        // Read the first word
        line = line.simplified();

        if(line.length() > 0 && line.at(0) != QChar::fromLatin1('#'))
        {
            QTextStream lineStream(&line, QIODevice::ReadOnly);
            QString token;
            lineStream >> token;
            // begin reading the lines
            if(token == QStringLiteral("v"))
            {
                float x, y, z;
                lineStream >> x >> y >> z;
//                float val = std::max({abs(x),abs(y),abs(z)});
//                x = x/val;
//                y = y/val;
//                z = z/val;
                //qDebug()<< x << y << z<<"\n";
                temp_vertices.append(QVector3D(x,y,z));
                totalPoints += 3;

                // Check for the min/max object dimensions in each x, y and z (for drawing ground plane)
                min_x = x < min_x ? x : min_x;
                max_x = x > max_x ? x : max_x;
                min_y = y < min_y ? y : min_y;
                max_y = y > max_y ? y : max_y;
                min_z = z < min_z ? z : min_z;
                max_z = z > max_z ? z : max_z;
            }
            else if(token == QStringLiteral("vt"))
            {
                float u, v;
                lineStream >> u>> v;
                temp_uvs.append(QVector2D(u, v));
            }
            else if(token == QStringLiteral("vn"))
            {
                float x, y, z;
                lineStream >> x >> y >> z;
                temp_normals.append(QVector3D(x, y, z));
            }
            else if(token == QStringLiteral("f"))
            {
                QString faceString1, faceString2, faceString3;
                lineStream >> faceString1 >> faceString2 >> faceString3;

                QStringList indices1 = faceString1.split(QChar::fromLatin1('/'));
                QStringList indices2 = faceString2.split(QChar::fromLatin1('/'));
                QStringList indices3 = faceString3.split(QChar::fromLatin1('/'));

                // Append the indices into respective vectors
                vertexIndices.append(indices1[0].toInt());
                vertexIndices.append(indices2[0].toInt());
                vertexIndices.append(indices3[0].toInt());

                uvIndices.append(indices1[1].toInt());
                uvIndices.append(indices2[1].toInt());
                uvIndices.append(indices3[1].toInt());

                normalIndices.append(indices1[2].toInt());
                normalIndices.append(indices2[2].toInt());
                normalIndices.append(indices3[2].toInt());
            }
        }
    }

//    for(int i=0; i<vertexIndices.size(); i++)
//    {
//        //get indices of its attributes
//        int vertexIndex = vertexIndices[i];
//        //int uvIndex = uvIndices[i];
//        int normalIndex = normalIndices[i];

//        //get the attributes thanks to the index and put them in buffer
//        out_vertices.append(temp_vertices[vertexIndex-1].x());
//        out_vertices.append(temp_vertices[vertexIndex-1].y());
//        out_vertices.append(temp_vertices[vertexIndex-1].z());

//        //out_uvs.append(temp_uvs[uvIndex-1].x());
//        //out_uvs.append(temp_uvs[uvIndex-1].y());

//        out_vertices.append(temp_normals[normalIndex-1].x());
//        out_vertices.append(temp_normals[normalIndex-1].y());
//        out_vertices.append(temp_normals[normalIndex-1].z());
//    }

//    for (int i = 0; i < temp_vertices.size(); i++)
//    {
//        out_vertices.append(temp_vertices[i] + QVector3D(max_x, max_y, max_z));
//    }
//    // Add the ground plane to out_vertices
//    out_vertices.append(addPlane());

    return true;
}
