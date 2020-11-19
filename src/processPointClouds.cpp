// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region
  // based filtering

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  // Create two new point clouds, one cloud with obstacles and other with
  // segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr plane_cloud(
      new pcl::PointCloud<PointT>());

  for (int idx : inliers->indices) {
    plane_cloud->points.push_back(cloud->points[idx]);
  }

  // Extract the inliers
  typename pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  // obstactle_cloud is a pointer, therefore you need to dereference it
  // as you pass it into filter
  extract.filter(*obstacle_cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(plane_cloud, obstacle_cloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Segment(typename pcl::PointCloud<PointT>::Ptr cloud,
                                    int maxIterations, float distanceTol) {
  auto startTime = std::chrono::steady_clock::now();
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // Randomly sample subset and fit plane
  // Formulas:
  // Plane formula: Ax + By + Cz + D = 0
  PointT p1, p2, p3;
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
      PointT point = cloud->points[i];
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

  // Create two new point clouds, one cloud with obstacles and other with
  // segmented plane
  typename pcl::PointCloud<PointT>::Ptr plane_cloud(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(
      new pcl::PointCloud<PointT>());

  for (int i = 0; i < cloud->points.size(); i++) {
    if (inliersResult.count(i)) {
      plane_cloud->points.push_back(cloud->points[i]);
    } else {
      obstacle_cloud->points.push_back(cloud->points[i]);
    }
  }

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(plane_cloud, obstacle_cloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Find inliers for the cloud.
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  typename pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    std::cout << "Could not estimate a planar model for the given dataset.\n";
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group
  // detected obstacles

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file) {

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}