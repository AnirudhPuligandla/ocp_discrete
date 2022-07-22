#include "glwidget.h"
#include <QMouseEvent>
#include <QCoreApplication>
#include <math.h>
#include <QOpenGLShaderProgram>
#include <QVector>
#include <QString>

bool GLWidget::m_transparent = false;

GLWidget::GLWidget(QWidget *parent) : QOpenGLWidget (parent), m_indexObj(QOpenGLBuffer::IndexBuffer)
{
    m_program=nullptr;
    m_dispScale = 0.1f;
    m_xRot = 0;
    m_yRot = 0;
    m_zRot = 0;

    // Initialize window dimensions
    currDims = QPoint(400, 400);
    nearPlane = 0.01f;
    farPlane = 100.0f;

    m_core = QSurfaceFormat::defaultFormat().profile() == QSurfaceFormat::CoreProfile;
    // --transparent causes the clear color to be transparent. Therefore, on systems that
    // support it, the widget will become transparent apart from the logo.
    if (m_transparent) {
        QSurfaceFormat fmt = format();
        fmt.setAlphaBufferSize(8);
        setFormat(fmt);
    }
}

GLWidget::~GLWidget()
{
    cleanup();
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(400, 400);
}

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

void GLWidget::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_xRot) {
        m_xRot = angle;
        //emit xRotationChanged(angle);
        update();
    }
}

void GLWidget::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_yRot) {
        m_yRot = angle;
        //emit yRotationChanged(angle);
        update();
    }
}

void GLWidget::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_zRot) {
        m_zRot = angle;
        //emit zRotationChanged(angle);
        update();
    }
}

static void qLimitScale(float &scaleFactor)
{
    if(scaleFactor < 0.01f)
        scaleFactor = 0.01f;
    else if(scaleFactor > 0.5f)
        scaleFactor = 0.5f;
}

void GLWidget::setScale(float scaleFactor)
{
    qLimitScale(scaleFactor);
    if (scaleFactor != m_dispScale)
        m_dispScale = scaleFactor;

    update();
}

void GLWidget::setSelection(bool isChecked)
{
    selectVertices = isChecked;
}

void GLWidget::loadModelPressed()
{
    // Get list of vertices from .obj file
    //QString path = "/media/anirudh/Data/Documents/PhD/Qt_projects/ocp_simulation/resources/BMW.binvox";

    // Empty the arrays
    vertices.clear();
    vecIndices.clear();

    bool res  = voxelLoader.getMesh(vertices, vecIndices);
    qDebug()<<res<<"   "<<vertices.size();

    // Setup vertex buffer with new list of vertices
    setupBuffer();

    // Update the opengl widget after obtaining the vertices
    update();
}

void GLWidget::dilatePressed()
{
    // Perform dilation
    voxelLoader.dilateObject();

    // Clear previous data and get vertices for the new mesh
    vertices.clear();
    vecIndices.clear();

    bool res  = voxelLoader.getMesh(vertices, vecIndices);
    qDebug()<<res<<"   "<<vertices.size();

    // Setup vertex buffer with new list of vertices
    setupBuffer();

    // Update the opengl widget after obtaining the vertices
    update();
}

void GLWidget::optimizePressed()
{
    /* Call the optimization module
     * Optimization results are stored in the passed vectors
     */
    voxelLoader.OptimizeCamPos(camVertices, contPoints);
    // Add optimized control points locations to the private data memeber vertices
    vertices.append(contPoints);
    vertices.append(camVertices);

    // Get axes
    voxelLoader.drawAxes(axes);
    vertices.append(axes);
    //qDebug()<<"\n"<< (vertices + camVertices).size() << " > "<< vertices.size();

    // Setup vertex buffer with updated list of vertices
    setupBuffer();

    // Repaint the opengl widget
    update();
}

void GLWidget::cleanup()
{
    if (m_program == nullptr)
        return;
    makeCurrent();
    m_obj.destroy();
    //m_indexObj.destroy();
    m_vao.destroy();
    delete m_program;
    m_program = nullptr;
    doneCurrent();
}

static const char *vertexShaderSourceCore =
    "#version 150\n"
    "in vec4 vertex;\n"
    "in vec3 normal;\n"
    "out vec3 vert;\n"
    "out vec3 vertNormal;\n"
    "uniform mat4 projMatrix;\n"
    "uniform mat4 mvMatrix;\n"
    "uniform mat3 normalMatrix;\n"
    "void main() {\n"
    "   vert = vertex.xyz;\n"
    "   vertNormal = normalMatrix * normal;\n"
    "   gl_Position = projMatrix * mvMatrix * vertex;\n"
    "}\n";

static const char *fragmentShaderSourceCore =
    "#version 150\n"
    "in highp vec3 vert;\n"
    "in highp vec3 vertNormal;\n"
    "out highp vec4 fragColor;\n"
    "uniform highp vec3 lightPos;\n"
    "void main() {\n"
    "   highp vec3 L = normalize(lightPos - vert);\n"
    "   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
    "   highp vec3 color = vec3(0.39, 1.0, 0.0);\n"
    "   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
    "   fragColor = vec4(col, 1.0);\n"
    "}\n";

static const char *vertexShaderSource =
    "attribute vec4 vertex;\n"
    "attribute vec3 normal;\n"
    "attribute vec3 colVal;\n"
    "varying vec3 vert;\n"
    "varying vec3 vertNormal;\n"
    "varying vec3 color;\n"
    "uniform mat4 projMatrix;\n"
    "uniform mat4 mvMatrix;\n"
    "uniform mat3 normalMatrix;\n"
    "void main() {\n"
    "   vert = vertex.xyz;\n"
    "   vertNormal = normalMatrix * normal;\n"
    "   color = colVal;\n"
    "   gl_Position = projMatrix * mvMatrix * vertex;\n"
    "}\n";

static const char *fragmentShaderSource =
    "varying highp vec3 vert;\n"
    "varying highp vec3 vertNormal;\n"
    "varying highp vec3 color;\n"
    "uniform highp vec3 lightPos;\n"
    "void main() {\n"
    "   highp vec3 L = normalize(lightPos - vert);\n"
    "   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
    "   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
    "   gl_FragColor = vec4(col, 1.0);\n"
    "}\n";


/*     "   highp vec3 color;\n"
    "   if(valCode < 1.5 && valCode > 0.0)"
    "       color = vec3(1.0, 0.0, 0.39);\n"
    "   else if(valCode > 1.5)"
    "       color = vec3(0.0, 0.39, 1.0);\n"
    "   else"
    "       color = vec3(0.39, 1.0, 0.0);\n"
 */

void GLWidget::initializeGL()
{
    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &GLWidget::cleanup);

    initializeOpenGLFunctions();
    glClearColor(0, 0, 0, m_transparent ? 0 : 1);

    m_program = new QOpenGLShaderProgram;
    //qDebug()<<m_core;
    //int nrAttributes;
    //glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &nrAttributes);
    //qDebug()<< "Maximum nr of vertex attributes supported: " << nrAttributes;
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, m_core ? vertexShaderSourceCore : vertexShaderSource);
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, m_core ? fragmentShaderSourceCore : fragmentShaderSource);
    m_program->bindAttributeLocation("vertex", 0);
    m_program->bindAttributeLocation("normal", 1);
    m_program->bindAttributeLocation("colVal", 2);
    m_program->link();

    m_program->bind();
    m_projMatrixLoc = m_program->uniformLocation("projMatrix");
    m_mvMatrixLoc = m_program->uniformLocation("mvMatrix");
    m_normalMatrixLoc = m_program->uniformLocation("normalMatrix");
    m_lightPosLoc = m_program->uniformLocation("lightPos");

    // Create a vertex array object. In OpenGL ES 2.0 and OpenGL 2.x
    // implementations this is optional and support may not be present
    // at all. Nonetheless the below code works in all cases and makes
    // sure there is a VAO when one is needed.
    m_vao.create();
    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);

    // Buffer
    m_obj.create();
    //m_indexObj.create();                // Only "called" this way but will be used to display camera frustums instead
    setupBuffer();
    qDebug()<<"Object created\n";

//    m_obj1.create();
//    m_obj1.bind();
//    m_obj1.allocate(normals.constData(), normals.size() * sizeof(GLfloat));

    // Store the vertex attribute bindings for the program.
    setupVertexAttribs();

    // Our camera never changes in this example.
    m_camera.setToIdentity();

    QVector3D camPos = QVector3D(0.0, 0.0, 15.0);
    m_camera.translate(-camPos);

    QVector3D camTarget = QVector3D(0.0, 0.0, 0.0);
    QVector3D camDirection = QVector3D(camPos - camTarget).normalized();
    QVector3D worldUp = QVector3D(0.0, 1.0, 0.0);
    QVector3D camRight = QVector3D::crossProduct(worldUp, camDirection).normalized();
    QVector3D camUp = QVector3D::crossProduct(camDirection, camRight);

    m_camera.lookAt(camPos, camTarget, camUp);
    // Light position is fixed.
    m_program->setUniformValue(m_lightPosLoc, QVector3D(0, 0, 200));

    m_program->release();
}

void GLWidget::setupBuffer()
{
    // Bind vertex buffer
    m_obj.bind();
    m_obj.allocate(vertices.constData(), vertices.size() * sizeof(GLfloat));

    // Bind index buffer
//    m_indexObj.bind();
//    m_indexObj.allocate(camVertices.constData(), camVertices.size() * sizeof(GLfloat));

    qDebug()<<"Setting up buffer\n";
}

void GLWidget::setupVertexAttribs()
{
    m_obj.bind();
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glEnableVertexAttribArray(0);
    f->glEnableVertexAttribArray(1);
    f->glEnableVertexAttribArray(2);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), reinterpret_cast<void *>(0));
    f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));
    f->glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(GLfloat), reinterpret_cast<void *>(6 * sizeof (GLfloat)));
    m_obj.release();

//    m_indexObj.bind();
//    QOpenGLFunctions *f1 = QOpenGLContext::currentContext()->functions();
//    f1->glEnableVertexAttribArray(0);
//    f1->glEnableVertexAttribArray(1);
//    f1->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), reinterpret_cast<void *>(0));
//    f1->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));
//    m_indexObj.release();
}

void GLWidget::paintGL()
{
    //qDebug()<<"In paintGL....\n";

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_LINE_WIDTH);
    glEnable(GL_POINT_SIZE);
    glPointSize(2);
    //glLineWidth(2);

    m_world.setToIdentity();
    m_world.rotate(180.0f - (m_xRot / 16.0f), 1, 0, 0);
    m_world.rotate(m_yRot / 16.0f, 0, 1, 0);
    m_world.rotate(m_zRot / 16.0f, 0, 0, 1);
    m_world.scale(m_dispScale);

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
    m_program->bind();
    m_program->setUniformValue(m_projMatrixLoc, m_proj);
    m_program->setUniformValue(m_mvMatrixLoc, m_camera * m_world);
    QMatrix3x3 normalMatrix = m_world.normalMatrix();
    m_program->setUniformValue(m_normalMatrixLoc, normalMatrix);

    //qDebug() << vertices.size();
    //qDebug() << camVertices.size();
    glDrawArrays(GL_POINTS, 0, (vertices.size() - camVertices.size() - axes.size())/9);
    // Draw camera frustums
    glDrawArrays(GL_LINES, (vertices.size() - camVertices.size() - axes.size())/9, (camVertices.size() + axes.size())/9);
    //glDrawElements(GL_POINTS, vecIndices.size(), GL_UNSIGNED_INT, 0);

    m_program->release();
}

void GLWidget::resizeGL(int w, int h)
{
    //qDebug()<<"In resizeGL....\n";
    m_proj.setToIdentity();
    m_proj.perspective(45.0f, GLfloat(w) / h, nearPlane, farPlane);
    currDims = QPoint(w, h);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - m_lastPos.x();
    int dy = event->y() - m_lastPos.y();
    //event->

    /* Handle mouse move events after a press
     * If selectvertices == false , rotate the world
     * If selectVertices == True , perform vertex selection operations  */
    if(!selectVertices)
    {
        if (event->buttons() & Qt::LeftButton) {
            setXRotation(m_xRot + 8 * dy);
            setYRotation(m_yRot + 8 * dx);
        } else if (event->buttons() & Qt::RightButton) {
            setXRotation(m_xRot + 8 * dy);
            setZRotation(m_zRot + 8 * dx);
        }
    }
    m_lastPos = event->pos();
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
    event->delta() > 0 ? m_dispScale += m_dispScale*0.1f : m_dispScale -= m_dispScale*0.1f;
    setScale(m_dispScale);
}
