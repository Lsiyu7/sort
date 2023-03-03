#include <Eigen/Dense>
#include <QTextCodec>
#include <QTextStream>
#include <Eigen/Core>
#include "robotics.h"
#include <QApplication>
#include "mainwindow.h"

int main(int argc,char *argv[])
{
    QApplication a(argc,argv);
    MainWindow w;
    w.show();
    return a.exec();
}

