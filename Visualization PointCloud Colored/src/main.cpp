/*********************************
           HEADERS
**********************************/
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <iostream>
#include <fstream>
#include <string>

#define CLAMP(value, min, max) (((value) >(max)) ? (max) : (((value) <(min)) ? (min) : (value)))

void printUsage (const char* progName){
  //std::cout << "\nUsage: " << progName << " <xyz.txt> <rgb.txt> <xyz_1.txt>"  << std::endl <<
  std::cout << "\nUsage: " << progName << " <xyz.txt> <xyz_1.txt>"  << std::endl <<
               "[q] to exit" << std::endl;
}

using namespace std;

int main(int argc, char **argv){

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>());

  if(argc < 3 or argc > 3){
      printUsage (argv[0]);
      return -1;
  }

  pcl::console::TicToc tt;
  pcl::console::print_highlight ("Loading ");
  
  std::ifstream file1(argv[1]);
  std::ifstream file2(argv[2]);
  //std::ifstream file3(argv[3]);

  //if(!file1.is_open() or !file2.is_open() or !file3.is_open()){
  if(!file1.is_open() or !file2.is_open()){
       std::cout << "Error: Could not find "<< argv[0] << std::endl;
       return -1;
  }
      
  std::cout << "files opened." << std::endl;
  double x_,y_,z_,r_,g_,b_;
  double x_1,y_1,z_1,r_1,g_1,b_1;	  

  //std::ofstream ofs("xyzrgb.txt");

   while(file1 >> x_ >> y_ >> z_ >> r_ >> g_ >> b_){
       pcl::PointXYZRGB pt;
       pt.x = x_;
       pt.y = y_;
       pt.z= z_;         

       //uint8_t r = *(uint8_t *)&r_;
       //uint8_t g = *(uint8_t *)&g_;
       //uint8_t b = *(uint8_t *)&b_;

       pt.r= r_;
       pt.g = g_;
       pt.b = b_;
             
       cloud->points.push_back(pt);
       std::cout << "pt:" << pt << std::endl;
       //ofs << x_ << " " << y_ << " " << z_ << " " << r_ << " " << g_ << " " << b_ << std::endl;
  }
  
  //ofs.close();  

  while(file2 >> x_1 >> y_1 >> z_1 >> r_1 >> g_1 >> b_1){
       pcl::PointXYZRGB pt1;
       pt1.x = x_1;
       pt1.y = y_1;
       pt1.z= z_1;         
      
       pt1.r= r_1;
       pt1.g = g_1;
       pt1.b = b_1;
                          
       cloud2->points.push_back(pt1);
       std::cout << "pt1:" << pt1 << std::endl;
  }
  
  pcl::console::print_info("\nFound txt file.\n");
  pcl::console::print_info ("[done, ");
  pcl::console::print_value ("%g", tt.toc ());
  pcl::console::print_info (" ms : ");
  pcl::console::print_value ("%d", cloud->points.size ());
  pcl::console::print_info (" points]\n");
    
  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;
  cloud->is_dense = true;

  //cloud2->width = (double) cloud2>points.size ();
  //cloud2->height = 1;
  //cloud2->is_dense = true;

  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL VISUALIZER"));
  pcl::visualization::PCLVisualizer viewer = pcl::visualization::PCLVisualizer("PCL VISUALIZER",true);

  viewer.addCoordinateSystem();
  pcl::PointXYZ p1, p2, p3;

  p1.getArray3fMap() << 1, 0, 0;
  p2.getArray3fMap() << 0, 1, 0;
  p3.getArray3fMap() << 0, 0.1,1; 

  viewer.addText3D("x", p1, 0.2, 1, 0, 0, "x_");
  viewer.addText3D("y", p2, 0.2, 0, 1, 0, "y_");
  viewer.addText3D ("z", p3, 0.2, 0, 0, 1, "z_");
  
  viewer.setPosition(0,0);
  viewer.setSize(800,600);

  int PORT1 = 0;
  viewer.createViewPort(0.0,0.0,0.5,1,PORT1);
  viewer.setBackgroundColor(1,1, 1,PORT1); // Setting background to a dark grey
  viewer.addText("Prueba 1",10,10,"PORT1",PORT1);

  int PORT2 = 0;
  viewer.createViewPort(0.5,0.0,1,1,PORT2);
  viewer.setBackgroundColor(1, 1, 1,PORT2);
  viewer.addText("Prueba 2",10,10,"PORT2",PORT2);
  

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer.addPointCloud<pcl::PointXYZRGB>(cloud,rgb,"POINTCLOUD",PORT2);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "POINTCLOUD");
  
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(cloud2,255,255,0);
  //viewer.addPointCloud(cloud2,color_handler,"POINTCLOUD2",PORT1);
  
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud2);
  viewer.addPointCloud<pcl::PointXYZRGB>(cloud2,rgb2,"POINTCLOUD2",PORT1);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "POINTCLOUD2");


  viewer.initCameraParameters();
  viewer.resetCamera();
 
  pcl::console::print_info ("\nAmazing!\n");

  while(!viewer.wasStopped()){
      viewer.spin();
  }

  return 0;
}
