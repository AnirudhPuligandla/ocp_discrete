#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include <QCheckBox>
#include <QPushButton>

class VTKWidget;
//class GLWidget;
class MainWindow;

class Window : public QWidget
{
    Q_OBJECT

public:
    Window(MainWindow *mw);

protected:
    void keyPressEvent(QKeyEvent *event) override;

private:
    //GLWidget *glWidget;
    VTKWidget *vtkWidget;
    MainWindow *mainWindow;
    QCheckBox *selectCamPos;
    QPushButton *loadModel;
    QPushButton *loadSimulated;
    QPushButton *continuousButton;
    QPushButton *dilate;
    QPushButton *multiResoButton;
    QPushButton *multiResoCombiButton;
    QPushButton *mrLoopButton;
    QPushButton *optimize;
    QPushButton *MRclusterTest;
    QPushButton *simulateTestButton;
    QPushButton *showResultButton;
};

#endif // WINDOW_H
