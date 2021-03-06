/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr &viewer) {

  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");
  Car car4(Vect3(-14, -3, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car4");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);
  cars.push_back(car4);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
    car4.render(viewer);
  }

  return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
               ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud =
      pointProcessorI->FilterCloud(cloud, 0.4,
                                   Eigen::Vector4f{-15, -6.5, -4, 1},
                                   Eigen::Vector4f{40, 6.5, 10, 1});
  // renderPointCloud(viewer, filteredCloud, "filteredCloud");
  pointProcessorI->obstacleDetection(filteredCloud, viewer);
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool renderScene = false;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // Create lidar sensor
  Lidar *lidar = new Lidar(cars, 0.0);

  // Create point processor
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = lidar->scan();
  // renderRays(viewer, lidar->position, point_cloud,
  // Color(254 / 255., 176 / 255., 252 / 255.));
  //   renderPointCloud(viewer, point_cloud, "point_cloud",
  //                    Color(155 / 255., 51 / 255., 249 / 255.));
  ProcessPointClouds<pcl::PointXYZ> *pointProcessor =
      new ProcessPointClouds<pcl::PointXYZ>();
  pointProcessor->obstacleDetection(point_cloud, viewer);
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr &viewer) {

  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
  case XY:
    viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
    break;
  case TopDown:
    viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
    break;
  case Side:
    viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
    break;
  case FPS:
    viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS)
    viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  // simpleHighway(viewer);

  ProcessPointClouds<pcl::PointXYZI> *pointProcessorI =
      new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream =
      pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

  while (!viewer->wasStopped()) {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloud = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloud);

    streamIterator++;
    if (streamIterator == stream.end())
      streamIterator = stream.begin();

    viewer->spinOnce();
  }
}