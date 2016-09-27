//
// Created by Aksel Sveier. Spring 2016
//

#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Qt
#include <QMainWindow>
#include <QVTKWidget.h>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

//Grabber
#include "k2g.h"

//Ransac
#include <ransac.h>
#include <vsr/space/vsr_cga3D_op.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();
   //files for storing data
    ofstream myfile;

   //Datasets
   boost::shared_ptr<PointCloudT> cl_raw;
   PointCloudT::Ptr cl_filtered;
   PointCloudT::Ptr segmented_plane;

   //Delete these
   PointCloudT::Ptr cam1;
   PointCloudT::Ptr cam2;
   PointCloudT::Ptr cam3;

   //Objects
   ransacSphere ranSph;
   ransacSpheres ranSpheres;
   ransacPlane ranPln;
   ransacCircle ranCir;
   ransacCylinder2 ranCyl;
   Pnt plane; //dual representation of plane
   Pnt sphere; //dual representation of sphere
   Pnt sphere1;
   Pnt sphere2; //dual representation of sphere
   float sphereRadius;
   float cylRadius;
   double cylTresh;
   float inlierTresh;

   std::clock_t t1,t2;
public slots:
  void processFrameAndUpdateGUI();

  void pSliderValueChanged (int value);

  void XsliderChanged (int value);
  void X_2sliderChanged(int value);

  void YsliderChanged (int value);
  void Y_2sliderChanged(int value);

  void ZsliderChanged (int value);
  void Z_2sliderChanged(int value);

public Q_SLOTS:
    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
  void on_pushButton_normalize_clicked();
  void on_pushButton_addNoise_clicked();
  void on_pushButton_findSphereEx_clicked();
  void on_pushButton_removeNoise_clicked();
  void on_pushButton_findCylinderEx_clicked();

protected:

  float x, x_;
  float y, y_;
  float z, z_;

private slots:

  void findPlaneClicked();

  void findSphereClicked();

  void findCircleClicked();

  void sphRadius_valueChanged(double value);

  void startStopKinectClicked();

  void segmentPlaneClicked();

  void removeShapesClicked();

  void refreshClicked();

  void checkBoxSegmentClicked();

  void inlierTresh_valueChanged(double value);



  void findCylinderClicked();

  void cylRadius_valueChanged(double value);

  void meanPointCloudsClicked();

  void nrDatasets_valueChanged(double value);

  void cylTresh_valueChanged(double value);

  void loadImageClicked();



  void doStuffClicked();

  void confirmClicked();

  void saveImageClicked();

  void checkBoxKeepImageClicked();

  void trackSphereClicked();

  void trackSphere(Pnt sph);

  void trackPlaneClicked();

  void trackPlane();

  void trackSphere2Clicked();

  void applyTrans();

  void findAllSpheresClicked();

  void writeClicked();

  void iterations_valueChanged(double arg1);



private:
  K2G *k2g;
  Ui::PCLViewer *ui;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  QTimer* timr;

  //Bools to controle the program
  bool kinStarted = false;
  bool grabNewFrame = true;
  bool segmented = false;
  bool checkBoxSeg = true;
  bool dostuff = false;
  bool allSpheres = false;
  int allit = 0;


  bool sphereFound = false;
  bool planeFound = false;
  bool circleFound = false;
  bool cylinderFound = false;
  bool trackSph1 = false;
  bool trackSph2 = false;
  bool trackPln = false;
  int whichSphere = 0;

  int frames_saved = 1;
  int iterations = 10000;
  bool checkBoxKeep = false;



  //Object based registration
  int obj = 1;
  Eigen::Vector3f O1;
  Eigen::Vector3f O2;
  Eigen::Vector3f O3;


  //Mean Point Cloud
  double nrDatasets;
  PointCloudT::Ptr mean_cloud;
  pcl::VoxelGrid<PointT> voxel;
  pcl::StatisticalOutlierRemoval<PointT> stat;

  //Filter
  pcl::PointIndices::Ptr indices_shown;
  pcl::PassThrough<PointT>::Ptr pass;
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  std::vector<int> indices;
  pcl::PointIndices::Ptr indices_segmented;
  void filter(PointCloudT::Ptr cl);

};

#endif // PCLVIEWER_H
