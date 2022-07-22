#-------------------------------------------------
#
# Project created by QtCreator 2019-03-04T17:02:40
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ocp_simulation
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++14

#INCLUDEPATH += /usr/local/include/PolyVoxCore
INCLUDEPATH += /opt/ibm/ILOG/CPLEX_Studio129/cplex/include
INCLUDEPATH += /opt/ibm/ILOG/CPLEX_Studio129/concert/include
INCLUDEPATH += /opt/ibm/ILOG/CPLEX_Studio129/cpoptimizer/include
INCLUDEPATH += /home/anirudh/opt/boost_1_68_0/
INCLUDEPATH += /usr/include/OpenCL-Headers
INCLUDEPATH += /usr/include/OpenCL-CLHPP/build/include/CL
INCLUDEPATH += /usr/local/include/vtk-8.2
INCLUDEPATH += /usr/local/include/ITK-5.0
INCLUDEPATH += /home/anirudh/opt/eigen-3.3.7
INCLUDEPATH += /usr/local/include/pcl-1.12

DEFINES += IL_STD

SOURCES += \
    heuristics.cpp \
    itkProcessing.cpp \
    loadbinvox.cpp \
        main.cpp \
        mainwindow.cpp \
    mipoptimizer.cpp \
    simulatedobjects.cpp \
    visibilitycheck.cpp \
    vtkwidget.cpp \
    window.cpp

HEADERS += \
    heuristics.h \
    itkProcessing.h \
    loadbinvox.h \
        mainwindow.h \
    mipoptimizer.h \
    simulatedobjects.h \
    visibilitycheck.h \
    vtkwidget.h \
    window.h

#LIBS += -L/usr/local/lib -lPolyVoxCore -lPolyVoxUtil
LIBS += -L/opt/ibm/ILOG/CPLEX_Studio129/cplex/lib/x86-64_linux/static_pic -lilocplex -lcplex -ldl
LIBS += -L/opt/ibm/ILOG/CPLEX_Studio129/cpoptimizer/lib/x86-64_linux/static_pic -lcp
LIBS += -L/opt/ibm/ILOG/CPLEX_Studio129/concert/lib/x86-64_linux/static_pic -lconcert
#LIBS += -L/usr/local/lib/intel-opencl -ligdrcl
#LIBS += -L/usr/local/lib -lopencl_clang
LIBS += -lOpenCL

LIBS += -L/usr/local/lib -lvtkCommonColor-8.2 -lvtkCommonCore-8.2 \
      -lvtkCommonComputationalGeometry-8.2 -lvtkCommonDataModel-8.2 -lvtkCommonExecutionModel-8.2 \
      -lvtkCommonMath-8.2 -lvtkCommonMisc-8.2 -lvtkCommonSystem-8.2 -lvtkCommonTransforms-8.2 -lvtkViewsCore-8.2 -lvtkInfovisCore-8.2 \
      -lvtkFiltersCore-8.2 -lvtkFiltersExtraction-8.2 -lvtkFiltersGeneral-8.2 -lvtkFiltersGeometry-8.2 -lvtkFiltersHybrid-8.2 \
      -lvtkFiltersSources-8.2 -lvtkFiltersStatistics-8.2 -lvtkImagingCore-8.2 -lvtkImagingColor-8.2 \
      -lvtkImagingFourier-8.2 -lvtkImagingMath-8.2 -lvtkImagingMorphological-8.2 -lvtkImagingStencil-8.2 \
      -lvtkIOImport-8.2 -lvtkIOCore-8.2 -lvtkIOExport-8.2 -lvtkIOGeometry-8.2 -lvtkIOImage-8.2 -lvtkIOLegacy-8.2 -lvtkIOPLY-8.2 -lvtkIOXML-8.2 \
      -lvtkInteractionStyle-8.2 -lvtkInteractionWidgets-8.2 -lvtkGUISupportQt-8.2 -lvtkRenderingCore-8.2 -lvtkRenderingOpenGL2-8.2 \
      -lvtkRenderingQt-8.2 -lvtkRenderingVolume-8.2 -lvtkRenderingVolumeOpenGL2-8.2 -lvtklibxml2-8.2 -lvtkmetaio-8.2 \
      -lvtkjsoncpp-8.2 -lvtksys-8.2 -lvtkzlib-8.2 -lvtkDICOMParser-8.2 -lvtkpng-8.2 -lvtktiff-8.2 -lvtkjpeg-8.2 -lvtkexpat-8.2 \
      -lvtkverdict-8.2 -lvtkNetCDF-8.2 -lvtksqlite-8.2 -lvtkexodusII-8.2 -lvtkfreetype-8.2

LIBS += -L/usr/local/lib -lITKBiasCorrection-5.0 -lITKCommon-5.0 -lITKDICOMParser-5.0 \
        -lITKEXPAT-5.0 -lITKgiftiio-5.0 -litkgdcmCommon-5.0 -litkhdf5 -litkhdf5_cpp \
        -lITKIOBioRad-5.0 -lITKIOBMP-5.0 -lITKIOCSV-5.0 -lITKIOGDCM-5.0 -lITKIOGE-5.0 -lITKIOGIPL-5.0 \
        -lITKIOHDF5-5.0 -lITKIOImageBase-5.0 -lITKIOIPL-5.0 -lITKIOJPEG-5.0 -lITKIOLSM-5.0 -lITKIOMeshBase-5.0 \
        -lITKIOMeta-5.0 -lITKIONIFTI-5.0 -lITKIONRRD-5.0 -lITKIOPNG-5.0 -lITKIOSiemens-5.0 -lITKIOSpatialObjects-5.0 \
        -lITKIOStimulate-5.0 -lITKIOTIFF-5.0 -lITKIOTransformBase-5.0 -lITKIOTransformHDF5-5.0 -lITKIOTransformInsightLegacy-5.0 \
        -lITKIOTransformMatlab-5.0 -lITKIOVTK-5.0 -lITKIOXML-5.0 -litkjpeg-5.0 -lITKKLMRegionGrowing-5.0 -lITKLabelMap-5.0 -lITKMesh-5.0 \
        -lITKMetaIO-5.0 -litkNetlibSlatec-5.0 -lITKniftiio-5.0 -lITKNrrdIO-5.0 -litkopenjpeg-5.0 -lITKOptimizers-5.0 \
        -lITKOptimizersv4-5.0 -lITKPath-5.0 -litkpng-5.0 -lITKStatistics-5.0 -lITKPolynomials-5.0 -lITKQuadEdgeMesh-5.0 \
        -lITKSpatialObjects-5.0 -litksys-5.0 -litktiff-5.0 -lITKVideoCore-5.0 -lITKVideoIO-5.0 \
        -litkvnl_algo-5.0 -litkvnl-5.0 -litkv3p_netlib-5.0 -litkvcl-5.0 -lITKVNLInstantiation-5.0 \
        -lITKVTK-5.0 -lITKVtkGlue-5.0 -lITKWatersheds-5.0 -litkzlib-5.0 -lITKznz-5.0 -ldl

LIBS += -L/usr/local/lib -lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_kdtree -lpcl_ml -lpcl_octree -lpcl_search -lpcl_segmentation

FORMS += \
        mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES +=

RESOURCES += \
    data.qrc
