#ifndef PICKINGTEXTURE_H
#define PICKINGTEXTURE_H

#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QVector>

class PickingTexture
{
public:
    PickingTexture();
    ~PickingTexture();

    bool Init(unsigned int WindowWidth, unsigned int WindowHeight);

    void enableWriting();
    void disableWriting();

    QVector<int> pixelInfo;

private:
    GLuint m_fbo;
    GLuint m_pickingTexture;
    GLuint m_depthTexture;
};

#endif // PICKINGTEXTURE_H
