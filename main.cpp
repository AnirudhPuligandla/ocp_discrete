#include "mainwindow.h"
#include <QDesktopWidget>
#include <QSurfaceFormat>
#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QDebug>

//#include "glwidget.h"
#include "vtkwidget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    setlocale(LC_NUMERIC,"C");

    QCoreApplication::setApplicationName("OCP Simulation");
    QCoreApplication::setOrganizationName("FER");
    QCoreApplication::setApplicationVersion(QT_VERSION_STR);
    QCommandLineParser parser;
    parser.setApplicationDescription(QCoreApplication::applicationName());
    parser.addHelpOption();
    parser.addVersionOption();
    QCommandLineOption multipleSampleOption("multisample", "Multisampling");
    parser.addOption(multipleSampleOption);
    QCommandLineOption coreProfileOption("coreprofile", "Use core profile");
    parser.addOption(coreProfileOption);
    QCommandLineOption transparentOption("transparent", "Transparent window");
    parser.addOption(transparentOption);

    parser.process(app);

    //QSurfaceFormat fmt;
    QSurfaceFormat fmt = QVTKOpenGLNativeWidget::defaultFormat();
    fmt.setDepthBufferSize(24);
    if (parser.isSet(multipleSampleOption))
        fmt.setSamples(8);
    if (parser.isSet(coreProfileOption)) {
        fmt.setVersion(3, 0);
        fmt.setProfile(QSurfaceFormat::CoreProfile);
    }
    QSurfaceFormat::setDefaultFormat(fmt);
    //QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    MainWindow mainWindow;

    //GLWidget::setTransparent(parser.isSet(transparentOption));
    //if (GLWidget::isTransparent()) {
    //    mainWindow.setAttribute(Qt::WA_TranslucentBackground);
    //    mainWindow.setAttribute(Qt::WA_NoSystemBackground, false);
    //}
    mainWindow.resize(mainWindow.sizeHint());
    int desktopArea = QApplication::desktop()->width() *
                     QApplication::desktop()->height();
    int widgetArea = mainWindow.width() * mainWindow.height();
    //qDebug()<<mainWindow.width() <<" "<< mainWindow.height();
    if (((float)widgetArea / (float)desktopArea) < 0.75f)
        mainWindow.show();
    else
        mainWindow.showMaximized();
    return app.exec();
}
