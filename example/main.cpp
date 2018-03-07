#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ransac.h>


//Function for creating visualization window and adding a plane to the visualization
boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Pnt plane)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  pcl::ModelCoefficients plane_coeff;
  plane_coeff.values.push_back (plane[0]); //normal vector, X
  plane_coeff.values.push_back (plane[1]); //normal vector, Y
  plane_coeff.values.push_back (plane[2]); //normal vector, Z
  plane_coeff.values.push_back (-plane[4]); //distance from origo to plane, d
  viewer->addPlane(plane_coeff, "Plane");

  return (viewer);
}


// --------------
// -----Main-----
// --------------
int main (){

    //Load point cloud from .pcd file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("sd_1.pcd", *cloud) == -1){
        PCL_ERROR ("Can't read the file \n");
        return(-1);
    }

    //Detect plane
    ransacPlane ranPln; //Initialize Ransac-plane object
    ranPln.setData(cloud, 0.01, 3000); //Set parameters (pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, float inlier_treshold, int inlier_candidates)
    ranPln.compute(); //Perform detection

    //Create viewer and visualize the point cloud and the plane
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = shapesVis(cloud, ranPln.ranPln);

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
