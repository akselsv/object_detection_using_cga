//
// Created by Aksel Sveier. Spring 2016
//

#include <iostream>
#include <cstdlib>
#include <ctime>
//#include <vsr/space/vsr_cga3D_op.h>


/*
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

#include "ransac.h"

#include <vsr/space/vsr_cga3D_op.h>

#include <stdlib.h>
#include <cmath>
#include <limits.h>

#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
*/

#include "pclviewer.h"
#include <QApplication>
#include <QMainWindow>
#include <QtGui>

using namespace std;

int main (int argc, char** argv)
{
    QApplication a (argc, argv);
    PCLViewer w;
    w.show ();
    a.exec();
    return 0;
}


