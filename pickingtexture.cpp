#include "pickingtexture.h"
#include <QOpenGLFunctions>

bool PickingTexture::Init(unsigned int WindowWidth, unsigned int WindowHeight)
{
    // Create opengl functions object
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    // Create FBO
    f->glGenBuffers(1, &m_fbo);
    f->glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

    //Create the texture object for primitive information buffer
    f->glGenTextures(1, &m_pickingTexture);
    f->glBindTexture(GL_TEXTURE_2D, m_pickingTexture);
    f->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, WindowWidth, WindowHeight, 0, GL_RGB, GL_FLOAT, nullptr);
    f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_pickingTexture, 0);

    //Create texture object for depth buffer
    f->glGenTextures(1, &m_depthTexture);
    f->glBindTexture(GL_TEXTURE_2D, m_depthTexture);
    f->glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, WindowWidth, WindowHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_depthTexture, 0);

}
