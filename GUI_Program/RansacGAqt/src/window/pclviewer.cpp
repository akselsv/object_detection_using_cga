//
// Created by Aksel Sveier. Spring 2016
//

#include "pclviewer.h"
#include "build/ui_pclviewer.h"
#include <QtCore>
#include <QFileDialog>
#include <time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>



#include <QVTKWidget.h>

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{

  ui->setupUi (this);
  this->setWindowTitle ("Aksel's interface");

  // Setup the cloud pointer
  cl_raw.reset(new PointCloudT);
  cl_filtered.reset (new PointCloudT);
  segmented_plane.reset(new PointCloudT);
  indices_shown.reset(new pcl::PointIndices);
  indices_segmented.reset(new pcl::PointIndices);
  pass.reset(new pcl::PassThrough<PointT>);
  cam1.reset(new PointCloudT);
  cam2.reset(new PointCloudT);
  cam3.reset(new PointCloudT);



  // The default filter
  x = 2000, x_ = -2000;
  y = 2000, y_ = -2000;
  z = 4000, z_ = 775;

  //Default object parameters
  sphereRadius = 0.02;
  inlierTresh = 5;
  cylRadius = 0.084;
  cylTresh = 10;

  //Mean Point Cloud
  mean_cloud.reset(new PointCloudT);
  nrDatasets = 50;

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  viewer->addCoordinateSystem ();

  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect "find sphere" button and the function
  connect (ui->pushButton_findSphere,  SIGNAL (clicked ()), this, SLOT (findSphereClicked()));

  // Connect "find plane" button and the function
  connect(ui->pushButton_findPlane, SIGNAL(clicked()), this, SLOT(findPlaneClicked()));

  //Connect "Find circle" button and the function
  connect(ui->pushButton_findCircle, SIGNAL(clicked()), this, SLOT(findCircleClicked()));

  //Connect "Find cylinder" button and the function
  connect(ui->pushButton_findCylinder, SIGNAL(clicked()), this, SLOT(findCylinderClicked()));


  // Connect X,Y,Z sliders and their functions
  connect (ui->slider_X, SIGNAL (valueChanged (int)), this, SLOT (XsliderChanged (int)));
  connect (ui->slider_X_2, SIGNAL(valueChanged(int)), this, SLOT(X_2sliderChanged(int)));
  connect (ui->slider_Y, SIGNAL (valueChanged (int)), this, SLOT (YsliderChanged (int)));
  connect (ui->slider_Y_2, SIGNAL(valueChanged(int)), this, SLOT(Y_2sliderChanged(int)));
  connect (ui->slider_Z, SIGNAL (valueChanged (int)), this, SLOT (ZsliderChanged (int)));
  connect (ui->slider_Z_2, SIGNAL(valueChanged(int)), this, SLOT(Z_2sliderChanged(int)));

  // Connect point size slider
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

  //Connect sphere radius box
  connect(ui->box_sphRadius, SIGNAL(valueChanged(double)), this, SLOT (sphRadius_valueChanged(double)));

  //Connect iteration box
  connect(ui->box_iterations,SIGNAL(valueChanged(double)),this, SLOT(iterations_valueChanged(double)));

  //Connect cylinder radius boc
  connect(ui->box_cylRadius, SIGNAL(valueChanged(double)), this, SLOT(cylRadius_valueChanged(double)));
  //Connect inlier treshold box
  connect(ui->box_inlierTresh, SIGNAL(valueChanged(double)), this, SLOT(inlierTresh_valueChanged(double)));

  //Connect "start Kinect" button
  connect(ui->pushButton_startKinect, SIGNAL(clicked()), this, SLOT(startStopKinectClicked()));

  //Connect "segment plane" buttion
  connect(ui->pushButton_segmentPlane, SIGNAL(clicked()), this, SLOT(segmentPlaneClicked()));

  //Connect "Remove shapes" button
  connect(ui->pushButton_removeShapes, SIGNAL(clicked()), this, SLOT(removeShapesClicked()));

  //Connect checkBox "Keep segment" button
  connect(ui->checkBox_segment, SIGNAL(clicked()), this, SLOT(checkBoxSegmentClicked()));

  //Connect checkBox "Keep Image" button
  connect(ui->checkBox_keepImage, SIGNAL(clicked()), this, SLOT(checkBoxKeepImageClicked()));

  //Connect "Refresh" button
  connect(ui->pushButton_refresh, SIGNAL(clicked()), this, SLOT(refreshClicked()));

  //Connect "Mean Point Cloud" button
  connect(ui->pushButton_meanPointClouds, SIGNAL(clicked()), this, SLOT(meanPointCloudsClicked()));

  //Connect "nrDatasets" box
  connect(ui->box_nrDatasets, SIGNAL(valueChanged(double)), this, SLOT(nrDatasets_valueChanged(double)));

  //Connect "cylTresh" box
  connect(ui->box_cylTresh, SIGNAL(valueChanged(double)), this, SLOT(cylTresh_valueChanged(double)));

  //Connect "load image" button
  connect(ui->pushButton_loadImage, SIGNAL(clicked()), this, SLOT(loadImageClicked()));

  //Connect "save image" button
  connect(ui->pushButton_saveImage, SIGNAL(clicked()), this, SLOT(saveImageClicked()));

  //Connect "do stuff" button
  connect(ui->pushButton_doStuff, SIGNAL(clicked()), this, SLOT(doStuffClicked()));

  //Connect "confirm" button
  connect(ui->pushButton_confirm, SIGNAL(clicked()), this, SLOT(confirmClicked()));

  //Connect "track sphere" button
  connect(ui->pushButton_trackSphere, SIGNAL(clicked()), this, SLOT(trackSphereClicked()));

  //Connect "track sphere " button
  connect(ui->pushButton_trackSphere2, SIGNAL(clicked()), this, SLOT(trackSphere2Clicked()));

  //Connect "track plane" button
  connect(ui->pushButton_trackPlane, SIGNAL(clicked()), this, SLOT(trackPlaneClicked()));

  //Connect "find all spheres" button
  connect(ui->pushButton_findAllSpheres, SIGNAL(clicked()), this, SLOT(findAllSpheresClicked()));

  //Connect "Write file" button
  connect(ui->pushButton_write, SIGNAL(clicked()), this, SLOT(writeClicked()));

  //Set up check function, check every 20 ms
  timr = new QTimer(this);
  connect(timr, SIGNAL(timeout()), this, SLOT(processFrameAndUpdateGUI()));
  timr->start(20);

  //viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cl_filtered);
  viewer->addPointCloud<pcl::PointXYZRGB> (cl_filtered,rgb, "cloud");
  pSliderValueChanged (2);
  viewer->resetCamera ();
  ui->qvtkWidget->update ();
}

void PCLViewer::processFrameAndUpdateGUI(){
    if(kinStarted && grabNewFrame){
        cl_raw = k2g->updateCloud(cl_raw);
    }

    if(allSpheres && allit < 1000){
        PCLViewer::findAllSpheresClicked();
        allit++;
    }

    if(trackPln){
        PCLViewer::trackPlane();
    }

    if(trackSph1)
    {
        whichSphere = 1;
        PCLViewer::trackSphere(sphere1);
    }

    if(trackSph2)
    {
        whichSphere = 2;
        PCLViewer::trackSphere(sphere2);
    }

    if(dostuff){
        PCLViewer::applyTrans();
    }

    filter(cl_raw);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cl_filtered);
    viewer->updatePointCloud<pcl::PointXYZRGB>(cl_filtered,rgb,"cloud");
    ui->qvtkWidget->update ();


}

void PCLViewer::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->qvtkWidget->update ();
}

void PCLViewer::XsliderChanged (int value)
{
  x = value;
}

void PCLViewer::X_2sliderChanged(int value)
{
    x_ = value;
}

void PCLViewer::YsliderChanged (int value)
{
    y = value;
}

void PCLViewer::Y_2sliderChanged(int value)
{
    y_ = value;
}

void PCLViewer::ZsliderChanged (int value)
{
    z = value;
}

void PCLViewer::Z_2sliderChanged(int value)
{
    z_ = value;
}

void PCLViewer::sphRadius_valueChanged(double value)
{
    sphereRadius = value;
}

void PCLViewer::findSphereClicked()
{
    //Find sphere
    std::cout << "Looking for sphere" << std::endl;
    if(trackSph1 || trackSph2){
        ranSph.setData(cl_filtered, 0.05, sphereRadius, 10);
    }else{
        ranSph.setData(cl_filtered, 0.05, sphereRadius, 30);
    }
    ranSph.iterations = iterations;

    if(ranSph.compute()){
        sphere = ranSph.ranSph;
        //visualize
        viewer->removeShape("Sphere");
        PointT center;
        center.x = sphere[0];
        center.y = sphere[1];
        center.z = sphere[2];
        double r = sqrt((1/pow(sphere[3],2))*(pow(sphere[0],2)+pow(sphere[1],2)+pow(sphere[2],2)) - (2*sphere[4]/sphere[3])); //Sphere radius, r
        viewer->addSphere(center, r, 1.0,0.0,0.0,  "Sphere");

        std::cout << "Radius: " << r << std::endl;

        std::cout << "Center position: x " << sphere[0] << " , y " << sphere[1] << " , z " << sphere[2] << " distance:  ";
        std::cout << "Number of inliers: " << ranSph.numInliers << " Number of points: " << cl_filtered->points.size() << std::endl;



        sphereFound = true;
        planeFound = false;
        circleFound = false;
        cylinderFound = false;
    }else{
        std::cout << "No sphere found" << std::endl;
    }
}

void PCLViewer::findAllSpheresClicked()
{
    ranSpheres.setData(cl_filtered, 0.01); //define inliers that lie within a 1 cm band around the shape, 0.02 mm resolution of octree
    ranSpheres.setOctreeBounds(x_/1000,y_/1000,z_/1000,x/1000,y/1000,z/1000);
    t1=std::clock();
    ranSpheres.compute();
    t2=std::clock();

    /*
    cout << "Number of points: " << cl_filtered->points.size() << endl;
    cout << "Root length: " << ranSpheres.rootLength << endl;
    cout << "Number of occupied nodes: " << ranSpheres.totalNumberOfPopulatedNodes << " Number of iterations: " << ranSpheres.totalNumberOfIterations << endl;
    cout << "number of spheres: " << ranSpheres.foundSpheres->size() << endl;
    */
    //vizualize
    viewer->removeAllShapes();
    PointT center;
    double r;
    string name;
    ostringstream convert;
    for(int i = 0; i < ranSpheres.foundSpheres->size(); i++){
        sphere = ranSpheres.foundSpheres->at(i).dualSphere;
        center.x = sphere[0];
        center.y = sphere[1];
        center.z = sphere[2];
        r = sqrt((1/pow(sphere[3],2))*(pow(sphere[0],2)+pow(sphere[1],2)+pow(sphere[2],2)) - (2*sphere[4]/sphere[3])); //Sphere radius, r
        convert << i;
        name = convert.str();

        viewer->addSphere(center, r, 1.0,0.0,0.0, name);
        std::cout << "Inliers: " << ranSpheres.indexlist->at(i)->indices.size() << std::endl;
        std::cout << "Radius: " << r << " Distance: " << sqrt(pow(sphere[0],2) + pow(sphere[1],2) + pow(sphere[2],2)) << std::endl;

        std::cout << "Center position: x " << sphere[0] << " , y " << sphere[1] << " , z " << sphere[2] << std::endl;
    }

    //allSpheres = true;
    /*
    //Write to file:
    Eigen::MatrixXf pos(3,3);
    pos.row(0) << -0.5730269245, 0.3579242598, 1.4826794864;
    pos.row(1) << -0.199843679, 0.4202330012, 1.1724824565;
    pos.row(2) << 0.1461141816, 0.3609160308, 1.6354784531;
    Eigen::Matrix3f xyz = Eigen::Matrix3f::Zero();
    Eigen::Vector3f rd = Eigen::Vector3f::Zero();
    Eigen::Vector3i in = Eigen::Vector3i::Zero();


    for(int i= 0; i<ranSpheres.foundSpheres->size();i++){
        sphere = ranSpheres.foundSpheres->at(i).dualSphere;
        for(int j= 0; j<3; j++){
            float dist = sqrt(pow(pos(j,0)-sphere[0],2)+pow(pos(j,1)-sphere[1],2) + pow(pos(j,2)-sphere[2],2));
            if(dist < 0.04){
                xyz(j,0) = sphere[0];
                xyz(j,1) = sphere[1];
                xyz(j,2) = sphere[2];
                rd(j) = sqrt((1/pow(sphere[3],2))*(pow(sphere[0],2)+pow(sphere[1],2)+pow(sphere[2],2)) - (2*sphere[4]/sphere[3])); //Sphere radius, r
                in(j) = ranSpheres.indexlist->at(i)->indices.size();
            }
        }

    }

    ostringstream con;
    con << rd(0) << " " << xyz(0,0) << " " << xyz(0,1) << " " << xyz(0,2) << " " << in(0) << " " << rd(1) << " " << xyz(1,0) << " " << xyz(1,1) << " " << xyz(1,2) << " " << in(1) << " " << rd(2) << " " << xyz(2,0) << " " << xyz(2,1) << " " << xyz(2,2) << " " << in(2) << " " << ranSpheres.rootLength << " " << ranSpheres.totalNumberOfPopulatedNodes << " " << cl_filtered->points.size() << " " << ((float)t2-(float)t1)/(CLOCKS_PER_SEC) << " " << ranSpheres.foundSpheres->size() << "\n";
    myfile.open ("data.txt", std::ios_base::app);
    myfile << con.str();
    myfile.close();
    */
}

void PCLViewer::findPlaneClicked(){
    //Find plane
    std:cout << "Looking for plane" << std::endl;

    if(trackPln){
        ranPln.setData(segmented_plane, inlierTresh/1000, 20);
    }else{
        ranPln.setData(cl_filtered, inlierTresh/1000, 3000);
    }

    ranPln.compute();
    ranPln.segmentPlane(cl_raw);
    plane = ranPln.ranPln;

    //Store the found plane in a seperate cloud
    indices_segmented.reset(new pcl::PointIndices);
    indices_segmented->indices.resize(ranPln.indexlist->size());

    indices_segmented->indices = *ranPln.indexlist;

    indices_shown.reset(new pcl::PointIndices);
    pass.reset(new pcl::PassThrough<PointT>);
    pass->setInputCloud(cl_raw);

    extract.setInputCloud(cl_raw);
    extract.setIndices(indices_segmented);
    extract.setNegative (false);
    extract.filter(indices_shown->indices);

    pass->setIndices(indices_shown);


    pass->filter(*segmented_plane);

    //Visualize

    ModelCoefficients plane_coeff;

    viewer->removeShape("Plane");

    plane_coeff.values.push_back (plane[0]); //normal vector, X
    plane_coeff.values.push_back (plane[1]); //normal vector, Y
    plane_coeff.values.push_back (plane[2]); //normal vector, Z
    plane_coeff.values.push_back (-plane[4]); //distance from origo to plane, d
    viewer-> addPlane(plane_coeff, "Plane");

    std::cout << "x:" << plane[0] << " y: " << plane[1] << " z: " << plane[2] << " d: " << plane[4] << std::endl;

    sphereFound = false;
    planeFound = true;
    circleFound = false;
    cylinderFound = false;

}

void PCLViewer::findCircleClicked()
{
    //Find circle
    std::cout << "Looking for circle" << std::endl;
    ranCir.setData(cl_filtered, 0.05, 0.034);
    ranCir.compute();

    ModelCoefficients circle;
    circle.values.push_back (ranCir.ranCir.circleCenter[0]); //from x
    circle.values.push_back (ranCir.ranCir.circleCenter[1]); //from y
    circle.values.push_back (ranCir.ranCir.circleCenter[2]); //from z
    circle.values.push_back (ranCir.ranCir.plane[0]/100); //to x
    circle.values.push_back (ranCir.ranCir.plane[1]/100); //to y
    circle.values.push_back (ranCir.ranCir.plane[2]/100); //to z
    circle.values.push_back (ranCir.ranCir.radius); //radius
    viewer->addCylinder(circle,"circle");

    sphereFound = false;
    planeFound = false;
    circleFound = true;
    cylinderFound = false;
}

void PCLViewer::findCylinderClicked()
{
    //Find circle
    std::cout << "Looking for cylinder" << std::endl;
    ranCyl.setData(cl_filtered, (float)cylTresh/100, cylRadius);
    ranCyl.compute();
    ranCyl.cylinderLength(ranCyl.ranCyl);

    float length = sqrt(pow(ranCyl.projL[0],2) + pow(ranCyl.projL[1],2) + pow(ranCyl.projL[2],2)) + sqrt(pow(ranCyl.projS[0],2) + pow(ranCyl.projS[1],2) + pow(ranCyl.projS[2],2));

    std::cout << "Length: " << length << " m" << std::endl;

    ModelCoefficients cylinder;
    cylinder.values.push_back (ranCyl.projS[0] + ranCyl.ranCyl.center[0] ); //from x
    cylinder.values.push_back (ranCyl.projS[1] + ranCyl.ranCyl.center[1]); //from y
    cylinder.values.push_back (ranCyl.projS[2] + ranCyl.ranCyl.center[2]); //from z
    cylinder.values.push_back (ranCyl.projL[0]-ranCyl.projS[0]); //to x
    cylinder.values.push_back (ranCyl.projL[1]-ranCyl.projS[1]); //to y
    cylinder.values.push_back (ranCyl.projL[2]-ranCyl.projS[2]); //to z
    cylinder.values.push_back (ranCyl.ranCyl.radius); //radius
    viewer->removeShape("cylinder");
    viewer->addCylinder(cylinder, "cylinder");

    ModelCoefficients cylinder2;
    cylinder2.values.push_back (ranCyl.ranCyl.center[0]); //from x
    cylinder2.values.push_back (ranCyl.ranCyl.center[1]); //from y
    cylinder2.values.push_back (ranCyl.ranCyl.center[2]); //from z
    cylinder2.values.push_back (ranCyl.projS[0]); //to x
    cylinder2.values.push_back (ranCyl.projS[1]); //to y
    cylinder2.values.push_back (ranCyl.projS[2]); //to z
    cylinder2.values.push_back (ranCyl.ranCyl.radius); //radius
    viewer->removeShape("cylinder2");
    viewer->addCylinder(cylinder2, "cylinder2");

    sphereFound = false;
    planeFound = false;
    circleFound = false;
    cylinderFound = true;
}

void PCLViewer::segmentPlaneClicked()
{
    ranPln.segmentPlane(cl_raw);

    if(checkBoxSeg){
        indices_segmented->indices.insert(indices_segmented->indices.end(), ranPln.indexlist->begin(), ranPln.indexlist->end());

    }else{
        indices_segmented.reset(new pcl::PointIndices);
        indices_segmented->indices.resize(ranPln.indexlist->size());
        indices_segmented->indices = *ranPln.indexlist;
    }
    segmented = true;
}

void PCLViewer::startStopKinectClicked()
{
    if(!kinStarted){
        k2g = new K2G(this);
        cl_raw = k2g->getCloud();
        filter(cl_raw);
        kinStarted = true;
        grabNewFrame = true;
        ui->pushButton_startKinect->setText("Stop Kinect2");
    }else{
        k2g->shutDown();
        kinStarted = false;
        grabNewFrame = false;
        ui->pushButton_startKinect->setText("Start Kinect2");

    }

}

void PCLViewer::removeShapesClicked()
{
    viewer->removeAllShapes();
}

void PCLViewer::refreshClicked()
{
    segmented = false;
    grabNewFrame = true;
    obj = 1;
    ui->pushButton_confirm->setText("Confirm Object 1");
}

void PCLViewer::checkBoxSegmentClicked()
{
    checkBoxSeg = !checkBoxSeg;
}

void PCLViewer::checkBoxKeepImageClicked()
{
    checkBoxKeep = !checkBoxKeep;
}

void PCLViewer::inlierTresh_valueChanged(double value)
{
    inlierTresh = value;
}

void PCLViewer::cylRadius_valueChanged(double value)
{
    cylRadius = value;
}

void PCLViewer::filter(PointCloudT::Ptr cl){
    indices_shown.reset(new pcl::PointIndices);
    pass.reset(new pcl::PassThrough<PointT>);
    pass->setInputCloud(cl);

    if(segmented){
        extract.setInputCloud(cl);
        extract.setIndices(indices_segmented);
        extract.setNegative (true);
        extract.filter(indices_shown->indices);
        pass->setIndices(indices_shown);
    }

    pass->setFilterFieldName("x");
    pass->setFilterLimits((float)(x_/1000),(float)(x/1000));
    pass->filter(indices_shown->indices);
    pass->setIndices(indices_shown);
    pass->setFilterFieldName("y");
    pass->setFilterLimits((float)(y_/1000),(float)(y/1000));
    pass->filter(indices_shown->indices);
    pass->setIndices(indices_shown);
    pass->setFilterFieldName("z");
    pass->setFilterLimits((float)(z_/1000),(float)(z/1000));
    pass->filter(*cl_filtered);

}

PCLViewer::~PCLViewer ()
{
   if(kinStarted){
       k2g->shutDown();
   }
  delete ui;
}

void PCLViewer::meanPointCloudsClicked()
{
    //First start kinect
    if(!kinStarted){
        k2g = new K2G(this);
        cl_raw = k2g->getCloud();
        filter(cl_raw);
        kinStarted = true;
        ui->pushButton_startKinect->setText("Stop Kinect2");
    }

    //Get data
    int n = 0;
    mean_cloud.reset(new PointCloudT);
    voxel.setLeafSize(0.01f,0.01f,0.01f);
    stat.setMeanK (50);
    stat.setStddevMulThresh (1.0);
    while(n < nrDatasets){
        cl_raw.reset(new PointCloudT);

        voxel.setInputCloud(k2g->getCloud());
        voxel.filter(*cl_raw);

        stat.setInputCloud(cl_raw);
        cl_raw.reset(new PointCloudT);
        stat.filter (*cl_raw);

        *mean_cloud = *mean_cloud + *cl_raw;
        usleep(15000); // wait 20 ms
        n++;
    }

    k2g->shutDown();
    kinStarted = false;
    grabNewFrame = false;
    ui->pushButton_startKinect->setText("Start Kinect2");

    //Voxel data
    cl_raw.reset(new PointCloudT);
    voxel.setInputCloud(mean_cloud);
    voxel.filter(*cl_raw);

    stat.setInputCloud(cl_raw);
    cl_raw.reset(new PointCloudT);
    stat.filter (*cl_raw);

}

void PCLViewer::nrDatasets_valueChanged(double value)
{
    nrDatasets = value;
}

void PCLViewer::cylTresh_valueChanged(double value)
{
    cylTresh = value;
}

void PCLViewer::loadImageClicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Choose a .pcd file to open"), "/home", "All files (*.*);;PCD Files (*.pcd)");
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);

    if(checkBoxKeep){
        if (io::loadPCDFile<PointXYZRGB>(fileName.toStdString(), *cl_filtered) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file  \n");
        }

        *cl_raw = *cl_raw + *cl_filtered;

    }else{
        cl_raw.reset(new PointCloudT);

        if (io::loadPCDFile<PointXYZRGB>(fileName.toStdString(), *cl_raw) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file  \n");
        }
        if(!cl_raw->points[4].rgb){
            if (io::loadPCDFile<PointXYZ>(fileName.toStdString(), *temp) == -1) //* load the file
            {
                PCL_ERROR("Couldn't read file  \n");
            }
            copyPointCloud(*temp,*cl_raw);
            uint8_t r = 0;
            uint8_t g = 255;
            uint8_t b = 0;
            int32_t rgb = (r << 16) | (g << 8) | b;
            int asdf;
            for (int i= 0; i < cl_raw->points.size(); i++){
                cl_raw->points[i].rgb = *(float *)(&rgb);
            }
        }
    }


    std::cout << "Opened file" << std::endl;
    std::cout << cl_raw->points.size() << std::endl;

    grabNewFrame = false;
}

void PCLViewer::on_pushButton_normalize_clicked(){
    //Calculate centroids
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cl_raw, centroid);

    //Execute normalization
    Eigen::Matrix4f norm = Eigen::Matrix4f::Identity();
    norm(0, 3) = -centroid[0]; norm(1, 3) = -centroid[1]; norm(2, 3) = -centroid[2];
    pcl::transformPointCloud(*cl_raw, *cl_raw, norm);
}

void PCLViewer::on_pushButton_addNoise_clicked(){
    int numberOfNoise = (ui->box_noise->value()/100)*cl_raw->points.size();
    std::cout << numberOfNoise << std::endl;
    //Find max "bounding box" of point cloud
    double x_min = 99999, y_min= 99999, z_min = 99999, x_max = -99999, y_max= -99999, z_max = -99999;

    for(int i = 0; i < cl_raw->points.size(); i++){
        if(cl_raw->points[i].x > x_max){
            x_max = cl_raw->points[i].x;
        }else if(cl_raw->points[i].x < x_min){
            x_min = cl_raw->points[i].x;
        }
        if(cl_raw->points[i].y > y_max){
            y_max = cl_raw->points[i].y;
        }else if(cl_raw->points[i].y < y_min){
            y_min = cl_raw->points[i].y;
        }
        if(cl_raw->points[i].z > z_max){
            z_max = cl_raw->points[i].z;
        }else if(cl_raw->points[i].z < z_min){
            z_min = cl_raw->points[i].z;
        }
    }
    //Add 50% of length
    double x_l, y_l, z_l;
    x_l = x_max-x_min;
    y_l = y_max -y_min;
    z_l = z_max-z_min;
    x_min = x_min - x_l*0.3;
    //y_min = y_min - y_l*0.3;
    z_min = z_min - z_l*0.3;
    x_max = x_max + x_l*0.3;
    //y_max = y_max + y_l*0.3;
    z_max = z_max + z_l*0.3;

    pcl::PointXYZRGB noise;
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 255;
    int32_t rgb = (r << 16) | (g << 8) | b;
    for(int i =  0; i < numberOfNoise; i ++){
        noise.x = x_min + (x_max-x_min)*(((float) rand()) / (float) RAND_MAX);
        noise.y = y_min + (y_max-y_min)*(((float) rand()) / (float) RAND_MAX);
        noise.z = z_min + (z_max-z_min)*(((float) rand()) / (float) RAND_MAX);
        noise.rgb = *(float *)(&rgb);
        cl_raw->push_back(noise);
    }
}

void PCLViewer::on_pushButton_removeNoise_clicked(){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    temp = cl_raw;
    cl_raw.reset(new PointCloudT);

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 255;
    int32_t rgb = (r << 16) | (g << 8) | b;
    for(int i= 0; i< temp->points.size(); i++){
        if(!(temp->points[i].rgb == *(float *)(&rgb))){
            cl_raw->push_back(temp->points[i]);
        }
    }
}

void PCLViewer::on_pushButton_findSphereEx_clicked(){
    std::clock_t t1,t2;

    //naiveSphere testSp;
    //testSp.setData(cl_raw, 0.05, 0.2, 50000, 0.9);
    naiveSphere2 testSp;
    testSp.setData(cl_raw, 0.05, 45298, 0.9);
    t1=std::clock();
    testSp.compute();
    t2=std::clock();
    float diff = ((float)t2-(float)t1);

    //visualize
    Pnt vis = testSp.ranSph.dualSphere;
    viewer->removeShape("SphereEX");
    PointT center;
    center.x = vis[0];
    center.y = vis[1];
    center.z = vis[2];
    double r = testSp.ranSph.radius;
    viewer->addSphere(center, r, 1.0,0.0,0.0,  "SphereEX");
    float dist = sqrt(pow(vis[0],2)+pow(vis[1],2)+pow(vis[2],2));
    //Write to file
    ostringstream convert;
    convert << cl_raw->points.size() << " " << testSp.countedInliers << " " << (float)ui->box_noise->value()/100 << " " << testSp.iterations << " " << diff / CLOCKS_PER_SEC << " " << testSp.spheresChecked << " " << dist << "\n";
    myfile.open ("data.txt", std::ios_base::app);
    myfile << convert.str();
    myfile.close();

    std::cout << "Number of points: " << cl_raw->points.size() << " Inliers: " << testSp.countedInliers << " Noise added: " << (float)ui->box_noise->value()/100 <<" Iterations: " << testSp.iterations << " Time: " << diff / CLOCKS_PER_SEC << " Spheres checked: " << testSp.spheresChecked << " Distance from origo: " << dist << std::endl;
}

void PCLViewer::on_pushButton_findCylinderEx_clicked(){
    std::clock_t t1,t2;

    //ransacCylinder testCy;
    ransacCylinder22 testCy;
    testCy.setData(cl_raw, 0.05, 0.3, 49956, 0.9);
    t1=std::clock();
    testCy.compute();
    t2=std::clock();
    testCy.cylinderLength();
    float diff = ((float)t2-(float)t1);
    float length = sqrt(pow(testCy.projL[0],2) + pow(testCy.projL[1],2) + pow(testCy.projL[2],2)) + sqrt(pow(testCy.projS[0],2) + pow(testCy.projS[1],2) + pow(testCy.projS[2],2));
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*testCy.inlierCloud, centroid);
    float dist = sqrt(pow(centroid[0],2)+pow(centroid[1],2)+pow(centroid[2],2));

    Eigen::Vector3f cylinderAxis, yaxis;
    cylinderAxis[0] = sqrt(pow((testCy.projL[0]-testCy.projS[0]),2));
    cylinderAxis[1] = sqrt(pow((testCy.projL[1]-testCy.projS[1]),2));
    cylinderAxis[2] = sqrt(pow((testCy.projL[2]-testCy.projS[2]),2));
    cylinderAxis = cylinderAxis.normalized();
    yaxis[0]=0; yaxis[1] = 1; yaxis[0] = 0;
    float angle = acos(cylinderAxis.dot(yaxis))* 180.0 / PI;

    //visualize
    ModelCoefficients cylinder;
    cylinder.values.push_back (testCy.projS[0] + testCy.ranCyl.center[0]); //from x
    cylinder.values.push_back (testCy.projS[1] + testCy.ranCyl.center[1]); //from y
    cylinder.values.push_back (testCy.projS[2] + testCy.ranCyl.center[2]); //from z
    cylinder.values.push_back (testCy.projL[0]-testCy.projS[0]); //to x
    cylinder.values.push_back (testCy.projL[1]-testCy.projS[1]); //to y
    cylinder.values.push_back (testCy.projL[2]-testCy.projS[2]); //to z
    cylinder.values.push_back (testCy.ranCyl.radius); //radius
    viewer->removeShape("cylinder");
    viewer->addCylinder(cylinder, "cylinder");

    //Write to file
    ostringstream convert;
    //convert << cl_raw->points.size() << " " << testCy.countedInliers << " " << (float)ui->box_noise->value()/100 << " " << testCy.iterations << " " << diff / CLOCKS_PER_SEC << " " << testCy.cylindersChecked << " " << dist << " " <<length << " " << angle << "\n";
    convert << cl_raw->points.size() << " " << testCy.countedInliers << " " << (float)ui->box_noise->value()/100 << " " << testCy.iterations << " "<< testCy.spheresIterations << " " << diff / CLOCKS_PER_SEC << " "  << " " << dist << " " <<length << " " << angle << "\n";
    myfile.open ("data.txt", std::ios_base::app);
    myfile << convert.str();
    myfile.close();

    //std::cout << "Number of points: " << cl_raw->points.size() << " Inliers: " << testCy.countedInliers << " Noise added: " << (float)ui->box_noise->value()/100 <<" Iterations: " << testCy.iterations << " Time: " << diff / CLOCKS_PER_SEC << " Cylinders checked: " << testCy.cylindersChecked << " Distance from origo: " << dist << " Length: " << length << " Angle: " << angle << std::endl;
    std::cout << "Number of points: " << cl_raw->points.size() << " Inliers: " << testCy.countedInliers << " Noise added: " << (float)ui->box_noise->value()/100 <<" Iterations: " << testCy.iterations << " spheresIterations: " << testCy.spheresIterations << " Time: " << diff / CLOCKS_PER_SEC << " Distance from origo: " << dist << " Length: " << length << " Angle: " << angle << std::endl;
}


void PCLViewer::saveImageClicked()
{
    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::Directory);
    dialog.setOption(QFileDialog::DontUseNativeDialog,true);
    dialog.setOption(QFileDialog::DontResolveSymlinks);
    dialog.setNameFilterDetailsVisible(true);
    dialog.setViewMode(QFileDialog::Detail);
    QStringList filters;
    filters <<"Any files (*)"
            <<"Text files (*.txt)"
            <<"Image files (*.png *.xpm *.jpg)";
    dialog.setOption(QFileDialog::HideNameFilterDetails,false);
    dialog.setNameFilters(filters);
    int res = dialog.exec();
    QString file =  dialog.getExistingDirectory(this, tr("Choose a folder to save image"), "/home");
    //QString dir = QFileDialog::getExistingDirectory(this, tr("Choose a folder to save image"), "/home");
    std::stringstream out;
    out << frames_saved;
    pcl::io::savePCDFile(file.toStdString() + "/cloud" + out.str() + ".pcd", *cl_raw, true);
    frames_saved++;
}

void PCLViewer::doStuffClicked()
{
    if(!dostuff && trackSph1 && trackSph2 && trackPln ){
        dostuff = true;
         ui->pushButton_doStuff->setText("Stop transformation");

    }else{
        dostuff = false;
        ui->pushButton_doStuff->setText("Apply transformation");
    }


}

void PCLViewer::applyTrans(){

    O1[0] = sphere1[0]; O1[1] = sphere1[1]; O1[2] = sphere1[2];
    O2[0] = sphere2[0]; O2[1] = sphere2[1]; O2[2] = sphere2[2];
    O3[0] = plane[0]; O3[1] = plane[1]; O3[2] = plane[2];
    //Apply transformation
    Eigen::Vector3f x = (O2-O1).normalized();
    Eigen::Vector3f y = x.cross(O3.normalized()).normalized();
    Eigen::Vector3f z = x.cross(y).normalized();
    Eigen::Matrix4f t = Eigen::Matrix4f::Identity(), tinv;

    t.block(0,0,3,1) = x;
    t.block(0,1,3,1) = y;
    t.block(0,2,3,1) = z;
    t.block(0,3,3,1) = O1;

    tinv = t.inverse();
    tinv(3,0) = 0; tinv(3,1) = 0; tinv(3,2) = 0; tinv(3,3) = 1;

    pcl::transformPointCloud (*cl_raw, *cl_raw, tinv);

}

void PCLViewer::confirmClicked()
{
    grabNewFrame = false;
    if(obj == 1){
        //Confirm object 1
        if(sphereFound){
            O1[0] = sphere[0]; O1[1] = sphere[1]; O1[2] = sphere[2];
            ui->pushButton_confirm->setText("Confirm Object 2\n(point)");
            obj = 2;
        }else if(planeFound){
            O1[0] = plane[0]; O1[1] = plane[1]; O1[2] = plane[2];
            ui->pushButton_confirm->setText("Confirm Object 2\n(point)");
            obj = 2;
        }else if(circleFound){

        }else if(cylinderFound){

        }else{
            std::cout << "No valid object found!" << std::endl;
        }
    }else if(obj == 2){
        //Confirm object 2
        if(sphereFound){
            O2[0] = sphere[0]; O2[1] = sphere[1]; O2[2] = sphere[2];
            ui->pushButton_confirm->setText("Confirm Object 3\n(point or vector)");
            obj = 3;
        }else if(planeFound){
            O2[0] = plane[0]; O2[1] = plane[1]; O2[2] = plane[2];
            ui->pushButton_confirm->setText("Confirm Object 3\n(point or vector)");
            obj = 3;
        }else if(circleFound){

        }else if(cylinderFound){

        }else{
            std::cout << "No valid object found!" << std::endl;
        }
    }else if(obj == 3){
        //confirm object 3
        if(sphereFound){
            O3[0] = sphere[0]; O3[1] = sphere[1]; O3[2] = sphere[2];
            ui->pushButton_confirm->setText("Apply transformation");
            obj = 4;
        }else if(planeFound){
            O3[0] = plane[0]; O3[1] = plane[1]; O3[2] = plane[2];
            ui->pushButton_confirm->setText("Apply transformation");
            obj = 4;
        }else if(circleFound){

        }else if(cylinderFound){

        }else{
            std::cout << "No valid object found!" << std::endl;
        }
    }else if(obj == 4){
        //Apply transformation
        Eigen::Vector3f x = (O2-O1).normalized();
        Eigen::Vector3f y = x.cross(O3.normalized()).normalized();
        Eigen::Vector3f z = x.cross(y).normalized();
        Eigen::Matrix4f t = Eigen::Matrix4f::Identity(), tinv;

        t.block(0,0,3,1) = x;
        t.block(0,1,3,1) = y;
        t.block(0,2,3,1) = z;
        t.block(0,3,3,1) = O1;

        tinv = t.inverse();
        tinv(3,0) = 0; tinv(3,1) = 0; tinv(3,2) = 0; tinv(3,3) = 1;

        pcl::transformPointCloud (*cl_raw, *cl_raw, tinv);
        ui->pushButton_confirm->setText("Confirm object 1\n (point)");
        obj = 1;
    }
}

void PCLViewer::trackSphereClicked()
{
    if(!trackSph1){
        //if(!kinStarted){
          //  PCLViewer::startStopKinectClicked();
        //}

        if(!sphereFound){
            PCLViewer::findSphereClicked();
            sphereFound = false;
        }
        trackSph1 = true;
        sphere1 = sphere;
        ui->pushButton_trackSphere->setText("Stop tracking 1");
        t1 = std::clock();
    }else{
        trackSph1 = false;
        ui->pushButton_trackSphere->setText("Track sphere 1");
    }
}

void PCLViewer::trackSphere2Clicked()
{
    if(!trackSph2){
        if(!kinStarted){
            PCLViewer::startStopKinectClicked();
        }

        if(!sphereFound){
            PCLViewer::findSphereClicked();
            sphereFound = false;
        }
        sphere2 = sphere;

        trackSph2 = true;

        ui->pushButton_trackSphere2->setText("Stop Tracking 2");
    }else{
        trackSph2 = false;
        ui->pushButton_trackSphere2->setText("Track Sphere 2");
    }
}

void PCLViewer::trackSphere(Pnt sph)
{
    float rad = sqrt((1/pow(sph[3],2))*(pow(sph[0],2)+pow(sph[1],2)+pow(sph[2],2)) - (2*sph[4]/sph[3]));
    //constrain and define search area:
    indices_shown.reset(new pcl::PointIndices);
    pass.reset(new pcl::PassThrough<PointT>);
    pass->setInputCloud(cl_raw);

    pass->setFilterFieldName("x");
    pass->setFilterLimits(sph[0]-(rad+0.05),sph[0]+(rad+0.05));
    pass->filter(indices_shown->indices);
    pass->setIndices(indices_shown);
    pass->setFilterFieldName("y");
    pass->setFilterLimits(sph[1]-(rad+0.05),sph[1]+(rad+0.05));
    pass->filter(indices_shown->indices);
    pass->setIndices(indices_shown);
    pass->setFilterFieldName("z");
    pass->setFilterLimits(sph[2]-(rad+0.05),sph[2]+(rad+0.05));
    PointCloudT::Ptr box(new PointCloudT);
    pass->filter(*box);

    if(box->points.size() > 9){
        ranSph.setData(box, 0.05, sphereRadius,1);
        ranSph.iterations = 100000;
        if(ranSph.compute()){
            Pnt st = ranSph.ranSph;

            PointT center;
            center.x = st[0];
            center.y = st[1];
            center.z = st[2];
            double r = sqrt((1/pow(st[3],2))*(pow(st[0],2)+pow(st[1],2)+pow(st[2],2)) - (2*st[4]/st[3])); //Sphere radius, r
            switch (whichSphere) {
            case 1:
                sphere1 = st;
                //visualize
                if(!dostuff){
                    viewer->removeShape("Sphere1");
                    viewer->addSphere(center, r, 1.0,0.0,0.0,  "Sphere1");
                }
                break;
            case 2:
                sphere2 = st;
                //visualize
                if(!dostuff){
                    viewer->removeShape("Sphere2");
                    viewer->addSphere(center, r, 1.0,0.0,0.0,  "Sphere2");
                }
                break;
            }
            //write result to file..
            t2=std::clock();
            ostringstream convert;
            convert << r << " " << st[0] << " " << st[1] << " " << st[2] << " " << ranSph.numInliers << " " << box->points.size() << " " << ranSph.time << " " << ((float)t2-(float)t1)/(CLOCKS_PER_SEC) << " " << ranSph.actualIt << "\n";
            myfile.open ("data.txt", std::ios_base::app);
            myfile << convert.str();
            myfile.close();
        }
    }
}

void PCLViewer::trackPlaneClicked()
{
    if(!trackPln){
        if(!kinStarted){
            PCLViewer::startStopKinectClicked();
        }

        if(!planeFound){
            PCLViewer::findPlaneClicked();
        }
        trackPln = true;

        ui->pushButton_trackPlane->setText("Stop tracking");
    }else{
        trackPln = false;
        ui->pushButton_trackPlane->setText("Track plane");
    }
}

void PCLViewer::trackPlane()
{
    //constrain and define search area

    if(!(segmented_plane->size() < 10))
    {
        ranPln.setData(segmented_plane, inlierTresh/1000, 20);
        ranPln.compute();
        ranPln.segmentPlane(cl_raw);
        Pnt tplane = ranPln.ranPln;

        //Store the found plane in a seperate cloud
        indices_segmented.reset(new pcl::PointIndices);
        indices_segmented->indices.resize(ranPln.indexlist->size());

        indices_segmented->indices = *ranPln.indexlist;

        indices_shown.reset(new pcl::PointIndices);
        pass.reset(new pcl::PassThrough<PointT>);
        pass->setInputCloud(cl_raw);

        extract.setInputCloud(cl_raw);
        extract.setIndices(indices_segmented);
        extract.setNegative (false);
        extract.filter(indices_shown->indices);

        pass->setIndices(indices_shown);


        pass->filter(*segmented_plane);

        //Visualize
        if(!dostuff){
            ModelCoefficients plane_coeff;

            viewer->removeShape("PlaneT");

            plane_coeff.values.push_back (tplane[0]); //normal vector, X
            plane_coeff.values.push_back (tplane[1]); //normal vector, Y
            plane_coeff.values.push_back (tplane[2]); //normal vector, Z
            plane_coeff.values.push_back (-tplane[4]); //distance from origo to plane, d
            viewer-> addPlane(plane_coeff, "PlaneT");
        }

    }else{
        std::cout << "Tracking lost!" << std::endl;
        trackPln = false;
        ui->pushButton_trackPlane->setText("Track plane");
    }
}

void PCLViewer::writeClicked()
{
    float r = sqrt((1/pow(sphere1[3],2))*(pow(sphere1[0],2)+pow(sphere1[1],2)+pow(sphere1[2],2)) - (2*sphere1[4]/sphere1[3]));
    float x = sphere1[0];
    float y = sphere1[1];
    float z = sphere1[2];
    float d = sqrt(pow(sphere1[0],2) + pow(sphere1[1],2) + pow(sphere1[2],2));
    ostringstream convert;
    convert << r << " " << d << " " << x << " " << y << " " << z << " " << ranSph.numInliers << "\n";
    myfile.open ("data.txt", std::ios_base::app);
    myfile << convert.str();
    myfile.close();
}

void PCLViewer::iterations_valueChanged(double arg1)
{
    iterations = arg1;
}


