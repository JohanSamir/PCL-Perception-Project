#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
#include <pcl/console/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>

#include <pcl/features/vfh.h>
#include <pcl/visualization/histogram_visualizer.h>


#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sstream>
#include <string>
#include <vector>
#include <stdlib.h> 
#include <math.h> 


using namespace std;

std::vector <std::vector <float> > leer_datos(const char* cadena, char delimeter, int Nfilas, int Ncolumnas);
std::vector <std::vector <float> > crear_matriz(int fil, int col);
std::vector <std::vector <float> > transpose(std::vector <std::vector <float> > matrix);
std::vector <std::vector <float> > mat_mult(std::vector <std::vector <float> > m1, std::vector <std::vector <float> > m2);
std::vector <std::vector <float> > mat_sum(std::vector <std::vector <float> > m1, std::vector <std::vector <float> > m2);
std::vector <std::vector <float> > mat_res(std::vector <std::vector <float> > m1, std::vector <std::vector <float> > m2);
std::vector <std::vector <float> > sigmoid(std::vector <std::vector <float> > m);
std::vector <std::vector <float> > tan_h(std::vector <std::vector <float> > m);
std::vector <std::vector <float> > grab_data(std::vector <std::vector <float> > matriz, int Fi, int Ff, int Ci, int Cf);
std::vector <std::vector <float> > encode(std::vector <std::vector <float> > datos, int salidas);
float average(std::vector <std::vector <float> > mat);
void imp_mat(std::vector <std::vector <float> > mat, string nombre);
int fil_size(std::vector <std::vector <float> > v2d);
int col_size(std::vector <std::vector <float> > v2d);
int leer_filas(const char* cadena, char delimeter);
int leer_columnas(const char* cadena, char delimeter);
float prod_punto(std::vector <std::vector <float> > m1, std::vector <std::vector <float> > m2);
std::vector <float> SVC_prediccion(std::vector <std::vector <float> > weights, std::vector <std::vector <float> > datos);
int get_maxnumberposition(std::vector <std::vector <float> > out);

//Funcion para borrar los parentesis del vector de descriptores
void borrar_caracter(const char* cadena)
{
    std::ifstream file(cadena);
    std::string str;
    ofstream fs("dataset.csv");
    while (std::getline(file, str))
      {
        for(int i = 0; i < str.size(); ++i)
            {
                if (str[i] == '(' || str[i] == ')')
                {
                    str.erase(i,1);
                }
            }
        fs << str << endl;
      }
    fs.close();
}

int
main (int argc, char** argv)
{
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr suelo (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);


    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
    

    pcl::console::TicToc tt;
  pcl::PCDWriter writer;

  
  std::cerr << "Loading Point Cloud...\n", tt.tic ();
  /*if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
  {
    return -1;
  }*/
    ////////// LEER ARCHIVOS PCD Y PLY ////////////
    std::vector<int> filenames;
    bool file_is_pcd = false;

    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

    if (filenames.size () != 1)  
	{
    		filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

	if (filenames.size () != 1) 
	{
		return -1;
    	} else {
      		file_is_pcd = true;
    	}
  	}

  	if (file_is_pcd) {

    		if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud) < 0)  
		{
      			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      		        return -1;
    		}
  		} else {

    		if (pcl::io::loadPLYFile (argv[filenames[0]], *cloud) < 0)  
		{
      			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      			return -1;
		}
  	}

	////////// FIN ---  LEER ARCHIVOS PCD Y PLY ////////////
	
	std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud->points.size () << " points\n";


	pcl::PassThrough<pcl::PointXYZ> passfilter;
	passfilter.setInputCloud(cloud);	
	passfilter.setFilterFieldName("z");
	passfilter.setFilterLimits(0.2, 2.0);
	passfilter.filter(*cloud_in);

	/*	
	std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud->points.size () << " points\n";

	std::cerr << "Segmentation floor...\n", tt.tic ();

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> segmentation; // se crea un objeto de segmentacion utilizando la libreria "SACSegmentation" de PCL
	segmentation.setInputCloud(cloud_in);//La nube de puntos cargada se define como objeto entrada	
	segmentation.setModelType(pcl::SACMODEL_PLANE);	
	segmentation.setMethodType(pcl::SAC_RANSAC);	//metodo de segmentacion (RANSAC)
        segmentation.setDistanceThreshold(0.04);//Se define la distancia minima del modelo
	segmentation.setOptimizeCoefficients(true);//Se activa el rendimiento del coeficiente del modelo (opcional)
	pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
	segmentation.segment(*inlierIndices, *coefficients);

	if (inlierIndices->indices.size() == 0)
		std::cout << "No se puede hacer el plano." << std::endl;
	else
	{

	//Nueva nube a trabajar

		pcl::ExtractIndices<pcl::PointXYZ> extract(true);
		extract.setInputCloud(cloud_in);
		extract.setIndices(inlierIndices);
		extract.setNegative(true);
		extract.filter(*cloud_out);

	//Segmenta el suelo

		pcl::ExtractIndices<pcl::PointXYZ> extract2(true);
  		extract2.setInputCloud(cloud_in);
  		extract2.setIndices(inlierIndices);
  		extract2.filter(*suelo);
	}
	std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_in->points.size () << " points\n";
*/
  	
 	std::cerr << "Downsampling...\n", tt.tic ();

        std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; 
  
  	pcl::VoxelGrid<pcl::PointXYZ> vg;
  	vg.setInputCloud (cloud_in);
  	vg.setLeafSize (0.01, 0.01, 0.01);
  	vg.setDownsampleAllData (true);
  	vg.filter (*cloud_out);

	std::cout << "PointCloud after filtering has: " << cloud_out->points.size ()  << " data points." << std::endl;
  	std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_out->points.size () << " points\n";
	
 
  	std::cerr << "Segmenting to clusters...\n", tt.tic ();  	

  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  	tree->setInputCloud (cloud_out);

  	std::vector<pcl::PointIndices> cluster_indices;

  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  	ec.setClusterTolerance (0.1); 
  	ec.setMinClusterSize (400);
  	ec.setMaxClusterSize (250000);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud_out);
  	ec.extract (cluster_indices);







