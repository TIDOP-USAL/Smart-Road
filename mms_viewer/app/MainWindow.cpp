#include "MainWindow.h"

#include <QDebug>
#include <QMenuBar>
#include <QFileDialog>

#include <iostream>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include <viewer3D/viewer_3d.h>
MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags flags)
    : QMainWindow(parent, flags),
      mOSGWidget(new OSGWidget(this))
{
    QMenuBar* menuBar = this->menuBar();

    QMenu* pcdMenu = menuBar->addMenu("PCD");
    pcdMenu->addAction("PCD", this, SLOT(onLoadPCD()));

    QMenu* oocoMenu = menuBar->addMenu("OutOfCore");
    oocoMenu->addAction("Solid", this, SLOT(onLoadOOCO_Solid()));
    oocoMenu->addAction("Coord_X", this, SLOT(onLoadOOCO_Coord_X()));
    oocoMenu->addAction("Coord_Y", this, SLOT(onLoadOOCO_Coord_Y()));
    oocoMenu->addAction("Coord_Z", this, SLOT(onLoadOOCO_Coord_Z()));
    oocoMenu->addAction("Intensity", this, SLOT(onLoadOOCO_Intensity()));
    oocoMenu->addAction("RGB", this, SLOT(onLoadOOCO_RGB()));

    this->setCentralWidget(mOSGWidget);
}

MainWindow::~MainWindow()
{
}

void MainWindow::onLoadPCD()
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    std::string infile = QFileDialog::getOpenFileName(this,
                                                      tr("Open file"), "", tr("PCD Files (*.pcd)")).toStdString();

    if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (infile, cloud) == -1)  // load the file
        std::cerr << "Couldn't read file " << infile << std::endl;
    else
        std::cout << "Loaded " << cloud.width * cloud.height << " data points from " << infile << std::endl;

    Viewer3DInterface::getInstance()->plotCloud(mOSGWidget, cloud.makeShared());
}

void MainWindow::onLoadOOCO_Solid(){
    std::string infile = QFileDialog::getOpenFileName(this,
                                                      tr("Open file"), "", tr("OOCO_idx Files (*.oct_idx)")).toStdString();
    Viewer3DInterface::getInstance()->plotOOCOctree_Solid(mOSGWidget, infile);
}

void MainWindow::onLoadOOCO_Coord_X(){
    std::string infile = QFileDialog::getOpenFileName(this,
                                                      tr("Open file"), "", tr("OOCO_idx Files (*.oct_idx)")).toStdString();
    Viewer3DInterface::getInstance()->plotOOCOctree_Coord(mOSGWidget, infile, "x");
}

void MainWindow::onLoadOOCO_Coord_Y(){
    std::string infile = QFileDialog::getOpenFileName(this,
                                                      tr("Open file"), "", tr("OOCO_idx Files (*.oct_idx)")).toStdString();
    Viewer3DInterface::getInstance()->plotOOCOctree_Coord(mOSGWidget, infile, "y");
}

void MainWindow::onLoadOOCO_Coord_Z(){
    std::string infile = QFileDialog::getOpenFileName(this,
                                                      tr("Open file"), "", tr("OOCO_idx Files (*.oct_idx)")).toStdString();
    Viewer3DInterface::getInstance()->plotOOCOctree_Coord(mOSGWidget, infile, "z");
}

void MainWindow::onLoadOOCO_Intensity(){
    std::string infile = QFileDialog::getOpenFileName(this,
                                                      tr("Open file"), "", tr("OOCO_idx Files (*.oct_idx)")).toStdString();
    Viewer3DInterface::getInstance()->plotOOCOctree_Intensity(mOSGWidget, infile);
}

void MainWindow::onLoadOOCO_RGB(){
    std::string infile = QFileDialog::getOpenFileName(this,
                                                      tr("Open file"), "", tr("OOCO_idx Files (*.oct_idx)")).toStdString();
    Viewer3DInterface::getInstance()->plotOOCOctree_RGB(mOSGWidget, infile);
}
