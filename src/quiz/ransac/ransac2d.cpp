/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <unordered_set>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -10; i < 10; i++) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int maxIterations, float distanceTol) {
  auto startTime = std::chrono::steady_clock::now();
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // Randomly sample subset and fit line
  // Formulas:
  // Line formula Ax + By + C = 0
  // (y1 -y2)x + (x2 -x1)y + (x1*y2 -x2*y1) = 0
  // A = (y1-y2), B = (x2 - x1), C = (x1*y2 - x2*y1)
  pcl::PointXYZ p1, p2;
  int index1, index2;
  float a, b, c, distance;
  while (maxIterations > 0) {
    std::unordered_set<int> inliers;
    while (inliers.size() < 2) {
      inliers.insert(cloud->points.size() * ((double)rand() / (RAND_MAX)));
    }

    auto itr = inliers.begin();
    p1 = cloud->points[*itr];
    itr++;
    p2 = cloud->points[*itr];

    a = (p1.y - p2.y);
    b = (p2.x - p1.x);
    c = (p1.x * p2.y - p2.x * p1.y);

    for (int i = 0; i < cloud->points.size(); i++) {
      pcl::PointXYZ point = cloud->points[i];
      // Point (x,y)
      // Distance d = |Ax+By+C|/sqrt(A^2+B^2)
      distance =
          std::fabs(a * point.x + b * point.y + c) / std::sqrt(a * a + b * b);
      if (distance <= distanceTol) {
        inliers.insert(i);
      }
    }
    // If the current iteration has more inliers, copy the set to the inliers
    // result
    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }

    maxIterations -= 1;
  }
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    int maxIterations, float distanceTol) {
  auto startTime = std::chrono::steady_clock::now();
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // Randomly sample subset and fit plane
  // Formulas:
  // Plane formula: Ax + By + Cz + D = 0
  pcl::PointXYZ p1, p2, p3;
  int index1, index2, index3;
  float a, b, c, d, distance;
  while (maxIterations > 0) {
    std::unordered_set<int> inliers;
    while (inliers.size() < 3) {
      inliers.insert(cloud->points.size() * ((double)rand() / (RAND_MAX)));
    }

    auto itr = inliers.begin();
    p1 = cloud->points[*itr];
    itr++;
    p2 = cloud->points[*itr];
    itr++;
    p3 = cloud->points[*itr];

    //   A = (y_2 - y_1)(z_3 - z_1) - (z_2 - z_1)(y_3 - y_1)
    //   B = (z_2 - z_1)(x_3 - x_1) - (x_2 - x_1)(z_3 - z_1)
    //   C = (x_2 - x_1)(y_3 - y_1) - (y_2 - y_1)(x_3 - x_1)
    //   D = -(A * x_1 + B * y_1 + D * z_1)
    a = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
    b = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
    c = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    d = -(a * p1.x + b * p1.y + c * p1.z);

    for (int i = 0; i < cloud->points.size(); i++) {
      // Given a point P (x, y, z):
      // Distance d = |Ax + By + Cz + D|/sqrt(A^2 + B^2 + C^2)
      pcl::PointXYZ point = cloud->points[i];
      distance = std::fabs(a * point.x + b * point.y + c * point.z + d) /
                 std::sqrt(a * a + b * b + c * c);
      if (distance <= distanceTol) {
        inliers.insert(i);
      }
    }
    // If the current iteration has more inliers, copy the set to the inliers
    // result
    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }

    maxIterations -= 1;
  }
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return inliersResult;
}

int main() {

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

  std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 2D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
