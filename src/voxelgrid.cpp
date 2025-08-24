#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile("/home/moon/cloud.pcd", *cloud);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);  // 필터링 크기
  sor.filter(*cloud_filtered);

  pcl::io::savePCDFile("filtered.pcd", *cloud_filtered);
  return 0;
}
