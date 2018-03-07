This example show how to detect a plane in a point cloud.

To run this example you need to have PCL installed:

- PCL 1.7.2 (depends on: boost, eigen3, flann, vtk) http://pointclouds.org/documentation/tutorials/compiling_pcl_posix.php #Install this#

## How to install and run

### Linux

* Download project source
    ```
    git clone https://github.com/akselsv/object_detection_using_cga.git
    cd object_detection_using_cga/example
    ```
* Make build folder
    ```
    mkdir build
    cd build
    ```
* Build and install
    ```
    cmake ..
    make -j8
    ```
* Run example
    ```
    cd ..
    ./main
    ```
You should see something similar as illustrated in "result", where a point cloud is visualized along with a visualization of the largest plane in the point cloud, which in this case is the floor.



