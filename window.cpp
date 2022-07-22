//#include "glwidget.h"
#include "vtkwidget.h"
#include "window.h"
#include "mainwindow.h"
#include <QKeyEvent>
#include <QDesktopWidget>
#include <QApplication>
#include <QMessageBox>
#include <QHBoxLayout>


Window::Window(MainWindow *mw) : mainWindow(mw)
{
    //glWidget = new GLWidget;
    vtkWidget = new VTKWidget();
    selectCamPos = new QCheckBox("select vertices");
    loadModel = new QPushButton("Load model");
    loadSimulated = new QPushButton("Simulated Data");
    continuousButton = new QPushButton("Continuous Model");
    dilate = new QPushButton("Initialize data");
    multiResoButton = new QPushButton("Multi-Resolution");
    multiResoCombiButton = new QPushButton("combinatorial multi-reso");
    mrLoopButton = new QPushButton("MR-combi Loop");
    optimize = new QPushButton("Single Resolution");
    MRclusterTest = new QPushButton("MR clusters test");
    simulateTestButton = new QPushButton("Segmentation Test");
    showResultButton = new QPushButton("Show Result");

    //connect(selectCamPos, &QCheckBox::toggled, glWidget, &GLWidget::setSelection);
    connect(loadModel, &QPushButton::pressed, vtkWidget, &VTKWidget::loadModelPressed);
    connect(loadSimulated, &QPushButton::pressed, vtkWidget, &VTKWidget::loadSimulated);
    connect(continuousButton, &QPushButton::pressed, vtkWidget, &VTKWidget::continuousModel);
    connect(dilate, &QPushButton::pressed, vtkWidget, &VTKWidget::dilatePressed);
    connect(multiResoButton, &QPushButton::pressed, vtkWidget, &VTKWidget::multiResolution);
    connect(multiResoCombiButton, &QPushButton::pressed, vtkWidget, &VTKWidget::multiResolutionCombi);
    connect(mrLoopButton, &QPushButton::pressed, vtkWidget, &VTKWidget::combiMRLoop);
    connect(MRclusterTest, &QPushButton::pressed, vtkWidget, &VTKWidget::MRClusterTest);
    connect(simulateTestButton, &QPushButton::pressed, vtkWidget, &VTKWidget::simulateTest);
    connect(showResultButton, &QPushButton::pressed, vtkWidget, &VTKWidget::setupResultVolume);
    //connect(optimize, &QPushButton::pressed, glWidget, &GLWidget::optimizePressed);
    connect(optimize, &QPushButton::pressed, vtkWidget, &VTKWidget::optimizePressed);

    QVBoxLayout *container = new QVBoxLayout;
    container->addWidget(vtkWidget);
    container->addWidget(selectCamPos);
    // new Hbox layout to distinguish between single and multi-resolution methods
    QHBoxLayout *hContainer = new QHBoxLayout;
    QVBoxLayout *vContainer1 = new QVBoxLayout;
    QVBoxLayout *vContainer2 = new QVBoxLayout;

    vContainer1->addWidget(loadModel);
    vContainer1->addWidget(dilate);
    vContainer1->addWidget(multiResoButton);

    vContainer2->addWidget(loadSimulated);
    vContainer2->addWidget(optimize);
    vContainer2->addWidget(multiResoCombiButton);

    hContainer->addLayout(vContainer1);
    hContainer->addLayout(vContainer2);
    container->addLayout(hContainer);

    container->addWidget(mrLoopButton);
    container->addWidget(MRclusterTest);
    container->addWidget(continuousButton);
    container->addWidget(showResultButton);
    container->addWidget(simulateTestButton);
    setLayout(container);
    show();

    setWindowTitle(tr("OCP Simulation"));
}

void Window::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}
