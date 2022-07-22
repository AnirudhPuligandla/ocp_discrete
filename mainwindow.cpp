#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "window.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setCentralWidget(new Window(this));
}

MainWindow::~MainWindow()
{
    delete ui;
}
