#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

#include <ICP.h> // sparse ICP

#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>



void addNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	       pcl::PointCloud<pcl::Normal>::Ptr normals
)
{

  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud ( cloud );

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud ( cloud );
  normalEstimator.setSearchMethod ( searchTree );
  normalEstimator.setKSearch ( 15 );
  normalEstimator.compute ( *normals );
  
}

void
loadFile(const char* fileName,
	 pcl::PointCloud<pcl::PointXYZ> &cloud
)
{
  pcl::PolygonMesh mesh;
  
  if ( pcl::io::loadPolygonFile ( fileName, mesh ) == -1 )
  {
    PCL_ERROR ( "loadFile faild." );
    return;
  }
  else
    pcl::fromPCLPointCloud2<pcl::PointXYZ> ( mesh.cloud, cloud );
  
  // remove points having values of nan
  std::vector<int> index;
  pcl::removeNaNFromPointCloud ( cloud, cloud, index );
}

int main ( int argc, char** argv )
{
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> () );
  
  {
    // load source
    loadFile ( argv[1], *cloud_source );
    // load target
    loadFile ( argv[2], *cloud_target );
  }
  
  
  // transformed source ---> target
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans ( new pcl::PointCloud<pcl::PointXYZ> () );
  cloud_source_trans = cloud_source;
  
  
  boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
  viewer->setBackgroundColor (0, 0, 0);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color ( cloud_source, 0, 255, 0 );
  viewer->addPointCloud<pcl::PointXYZ> (cloud_source, source_color, "source");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color ( cloud_target, 255, 255, 255 );
  viewer->addPointCloud<pcl::PointXYZ> ( cloud_target, target_color, "target");
  viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target" );
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_trans_color ( cloud_source_trans, 255, 0, 255 );
  viewer->addPointCloud<pcl::PointXYZ> ( cloud_source_trans, source_trans_color, "source trans" );
  viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source trans" );
  
  
  // orthographic (parallel) projection; same with pressing key 'o'
  viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection( 1 );
  
  viewer->resetCamera();
  
  viewer->spinOnce();

  
  
  // prepare normals
  pcl::PointCloud<pcl::Normal>::Ptr target_normals ( new pcl::PointCloud<pcl::Normal> );
  addNormal( cloud_target, target_normals );
  
  
  Eigen::Matrix3Xd X ( 3, cloud_source->size() ); // source, transformed
  Eigen::Matrix3Xd Y ( 3, cloud_target->size() ); // target
  Eigen::Matrix3Xd N ( 3, target_normals->size() ); // target normal
  
  
  for(int i = 0; i < cloud_source->size(); i++)
  {
    X(0,i) = cloud_source->points[i].x;
    X(1,i) = cloud_source->points[i].y;
    X(2,i) = cloud_source->points[i].z;
  }
  for(int i = 0; i < cloud_target->size(); i++)
  {
    Y(0,i) = cloud_target->points[i].x;
    Y(1,i) = cloud_target->points[i].y;
    Y(2,i) = cloud_target->points[i].z;
  }
  for(int i = 0; i < target_normals->size(); i++)
  {
    N(0,i) = target_normals->points[i].normal_x;
    N(1,i) = target_normals->points[i].normal_y;
    N(2,i) = target_normals->points[i].normal_z;
  }
  
  
  while ( !viewer->wasStopped () )
  {
    
    
    // ICP::point_to_plane ( X, Y, N ); // standard ICP with normals
    // ICP::point_to_point ( X, Y ); // standard ICP
    
    SICP::Parameters param;
    param.max_icp = 1;
    param.p = 0.2;
//     SICP::point_to_point ( X, Y, param); // sparse ICP
    SICP::point_to_plane ( X, Y, N, param ); // sparse ICP with normals

    
    for(int i = 0; i < cloud_source_trans->size(); i++)
    {
      cloud_source_trans->points[i].x = X(0,i);
      cloud_source_trans->points[i].y = X(1,i);
      cloud_source_trans->points[i].z = X(2,i);
    }
    
    
    viewer->updatePointCloud ( cloud_source_trans, source_trans_color, "source trans" );
    
    
    viewer->spinOnce();
  }
  
  
  return( 0 );
}

