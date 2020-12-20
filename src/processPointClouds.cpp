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
void ProcessPointClouds<PointT>::obstacleDetection(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    pcl::visualization::PCLVisualizer::Ptr &viewer) {

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segmented_clouds = this->SegmentPlane(
          cloud, 100, 0.2); // maxIterations, distanceThreshold

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      clouds = this->Segment(cloud, 100, 0.2); // maxIterations, distanceTol
  renderPointCloud(viewer, clouds.first, "plane cloud",
                   Color(133 / 255., 182 / 255., 255 / 255.));
  // renderPointCloud(viewer, clouds.second, "obstacle cloud",
  //                Color(255 / 255., 25 / 255., 251 / 255.));
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters =
      this->ClusteringOwnCode(clouds.second, 0.45, 8,
                              100); // clusterTolerance, minSize, maxSize

  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0.5, 0), Color(0, 1, 0),
                               Color(0, 0, 0.8), Color(0, 0.8, 0.8)};

  for (typename pcl::PointCloud<PointT>::Ptr cluster : clusters) {
    std::cout << "cluster size ";
    this->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                     colors[clusterId % 5]);
    Box box = this->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered{
      new pcl::PointCloud<PointT>};
  typename pcl::PointCloud<PointT>::Ptr cloud_cropped{
      new pcl::PointCloud<PointT>};

  // Create the filtering object
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloud_filtered);

  // Create the cropbox object
  pcl::CropBox<PointT> cropBox(true);
  cropBox.setMin(minPoint);
  cropBox.setMax(maxPoint);
  cropBox.setInputCloud(cloud_filtered);
  cropBox.filter(*cloud_cropped);

  std::vector<int> indices;
  pcl::CropBox<PointT> roofBox(true);
  roofBox.setMin(Eigen::Vector4f{-1.5, -1.7, -1, 1});
  roofBox.setMax(Eigen::Vector4f{2.6, 1.7, -0.3, 1});
  roofBox.setInputCloud(cloud_cropped);
  roofBox.filter(indices);

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (int id : indices) {
    inliers->indices.push_back(id);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_cropped);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_cropped);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloud_cropped;
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
void ProcessPointClouds<PointT>::proximity(
    int id, typename pcl::PointCloud<PointT>::Ptr cloud,
    std::vector<int> &cluster, KdTree *tree, float distanceTol,
    std::vector<bool> &seen) {
  seen[id] = true;
  cluster.push_back(id);
  std::vector<float> point = {cloud->points[id].x, cloud->points[id].y,
                              cloud->points[id].z};
  std::vector<int> nearby_points = tree->search(point, distanceTol);
  for (int point_id : nearby_points) {
    if (!seen[point_id]) {
      proximity(point_id, cloud, cluster, tree, distanceTol, seen);
    }
  }
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(
    typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree,
    float distanceTol, int minSize) {

  std::vector<bool> seen(cloud->points.size(), false);
  std::vector<std::vector<int>> clusters;
  for (int id = 0; id < cloud->points.size(); id++) {
    if (!seen[id]) {
      std::vector<int> cluster;
      proximity(id, cloud, cluster, tree, distanceTol, seen);
      if (cluster.size() >= minSize) {
        clusters.push_back(cluster);
      }
    }
  }

  return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::ClusteringOwnCode(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // Perform euclidean clustering to group detected obstacles
  KdTree *tree = new KdTree;
  for (int i = 0; i < cloud->points.size(); i++) {
    std::vector<float> point = {cloud->points[i].x, cloud->points[i].y,
                                cloud->points[i].z};
    tree->insert(point, i);
  }
  std::vector<std::vector<int>> cluster_indices =
      euclideanCluster(cloud, tree, clusterTolerance, minSize);

  // Own implementation:
  for (auto indices : cluster_indices) {
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    for (int index : indices) {
      cluster->push_back(cloud->points[index]);
    }
    clusters.push_back(cluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // Perform euclidean clustering to group detected obstacles

  typename pcl::search::KdTree<PointT>::Ptr tree(
      new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  typename pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (auto indices : cluster_indices) {
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    for (int index : indices.indices) {
      cluster->push_back(cloud->points[index]);
    }
    clusters.push_back(cluster);
  }

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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

  BoxQ box;
  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cluster, pcaCentroid);
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
      covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  // This line is necessary for proper orientation in some cases. The
  // numbers come out the same without it, but the signs are different and the
  // box doesn't get correctly oriented in some cases.
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
  /* // Note that getting the eigenvectors can also be obtained via the PCL PCA
  interface with something like: pcl::PointCloud<pcl::PointXYZ>::Ptr
  cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloudSegmented);
  pca.project(*cloudSegmented, *cloudPCAprojection);
  std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() <<
  std::endl; std::cerr << std::endl << "EigenValues: " << pca.getEigenValues()
  << std::endl;
  // In this case, pca.getEigenVectors() gives similar eigenVectors to
  eigenVectorsPCA.
  */
  // Transform the original cloud to the origin where the principal components
  // correspond to the axes.

  // Transform the original cloud to the origin where the principal components
  // correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3, 1>(0, 3) =
      -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
  typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected(
      new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cluster, *cloudPointsProjected,
                           projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal =
      0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  // Final transform
  box.bboxQuaternion = eigenVectorsPCA;
  box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
  box.cube_length = maxPoint.x - minPoint.x;
  box.cube_width = maxPoint.y - minPoint.y;
  box.cube_height = maxPoint.z - minPoint.z;

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