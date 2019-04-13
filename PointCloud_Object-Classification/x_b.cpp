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



    int j = 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
  cloud_cluster->clear();
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      
        cloud_cluster->points.push_back (cloud_out->points[*pit]); 
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        /*std::stringstream ss;
        ss << "cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); */
      std::cout << "Cluster: "<< j << cloud_cluster->points.size () << " points." << std::endl;
        ++j;            
    }

    std::cerr << ">> Done: " << tt.toc () << " ms\n";

  //Para cambiar el tipo de nube de puntos
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>); 
    pcl::copyPointCloud(*cloud_out,*cloudRGB);



  std::cerr << "Coloring Clusters...\n", tt.tic (); 

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
      std::vector<unsigned char> colors;

      for (size_t i_segment = 0; i_segment < cluster_indices.size (); i_segment++)
      {
          colors.push_back (static_cast<unsigned char> (rand () % 256));
          colors.push_back (static_cast<unsigned char> (rand () % 256));
          colors.push_back (static_cast<unsigned char> (rand () % 256));
      }

      color_cloud->width = cloudRGB->width;
      color_cloud->height = cloudRGB->height;
      color_cloud->is_dense = cloudRGB->is_dense;


  //Para cambiar el tipo de nube de puntos
      for (size_t i_point = 0; i_point < cloudRGB->points.size (); i_point++)
      {
        pcl::PointXYZRGB point;
        point.x = *(cloudRGB->points[i_point].data);
        point.y = *(cloudRGB->points[i_point].data + 1);
        point.z = *(cloudRGB->points[i_point].data + 2);
        point.r = 255;
        point.g = 255;
        point.b = 255;
        color_cloud->points.push_back (point);
      }

      std::vector< pcl::PointIndices >::iterator i_segment;
      int sig_color = 0;

      for (i_segment = cluster_indices.begin (); i_segment != cluster_indices.end (); i_segment++)
      {
          std::vector<int>::iterator i_point;

          for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
          {
            int index;
            index = *i_point;
            color_cloud->points[index].r = colors[3 * sig_color];
            color_cloud->points[index].g = colors[3 * sig_color + 1];
            color_cloud->points[index].b = colors[3 * sig_color + 2];
          }
          sig_color++;
      }

        std::cerr << ">> Done: " << tt.toc () << " ms\n";

  std::cerr << "VFH...\n", tt.tic (); 
  

  int sig_normal = 0;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> points_cluster;//asi
  std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> normals;

  int cant=1;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
  
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      
    cluster->points.push_back (cloud_out->points[*pit]); 
      cluster->width = cloud_cluster->points.size ();
      cluster->height = 1;
      cluster->is_dense = true;

    points_cluster.push_back(cluster);//guardar cada cluster en una posicion de un vector (save)    
    std::cout << "Cluster "<< cant << ": " << cluster->points.size () << " data points." << std::endl;
    ++cant;
  }
  
  std::cout << "Cluster: "<< points_cluster.size() << std::endl;

  std::vector<pcl::PointCloud<pcl::Normal>::Ptr> extractedNormals;
  //pcl::PointCloud<pcl::Normal>::Ptr cluster_normals(new pcl::PointCloud<pcl::Normal>);
  
  for(size_t i = 0; i < points_cluster.size(); ++i)
      {       
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;

    normalEstimation.setInputCloud((points_cluster[i]));
    

      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      normalEstimation.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cluster_normals (new pcl::PointCloud<pcl::Normal>);
    normalEstimation.setRadiusSearch(0.03);
    
    normalEstimation.compute(*cluster_normals);

    extractedNormals.push_back(cluster_normals);
  }
  
  
  //Se genera el vector flotante para almacenar uno por uno los valores del histograma
  std::vector <std::vector <float> > datos;
  datos = crear_matriz(points_cluster.size(),308);


  for(size_t i = 0; i < points_cluster.size(); ++i)
      {
          pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);

          pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;

    vfh.setInputCloud((points_cluster[i]));
    vfh.setInputNormals((extractedNormals[i]));

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ> ());
    vfh.setSearchMethod(kdtree);

    vfh.setNormalizeBins(true);
    vfh.setNormalizeDistance(false);
    vfh.compute(*descriptor);

    for(int j = 0; j < 308; ++j){
      datos[i][j] = descriptor->points[0].histogram[j];
    }
    std::cerr << "Tamaño " << i+1 << ": " << descriptor->points[0].descriptorSize()  << std::endl;
      }
std::cerr << ">> Done: " << tt.toc () << " ms\n";

int clasificador = 0;
std::vector<int> colorc;

int ctype;
cout << "\nPorfavor ingrese: \n1.Para entrar a red neuronal MLP\n2.Para entrar a SVC\n";
cin >> ctype;
    
if(ctype == 1){ 
//---------------------RED---------------//
    cout << "Entro a la RED MLP" << endl;
    //Definicion de los archivos y las variables
    const char* Pesos_CO = "pesos/Pesos_oculta.csv";
    const char* Bias_CO = "pesos/Bias_oculta.csv";
    const char* Pesos_CS = "pesos/Pesos_salida.csv";
    const char* Bias_CS = "pesos/Bias_salida.csv";
    std::vector <std::vector <float> > X;
    std::vector <std::vector <float> > Wco;
    std::vector <std::vector <float> > bco;
    std::vector <std::vector <float> > Wcs;
    std::vector <std::vector <float> > bcs;
    std::vector <std::vector <float> > OutputCo;
    std::vector <std::vector <float> > Output;
    float error;

    //Lectura de todos los datos
    Wco = leer_datos(Pesos_CO, ',', 308, 30);//308, 12 --- 308, 30
    bco = leer_datos(Bias_CO, ',', 30, 1);//12, 1 --- 30, 1
    Wcs = leer_datos(Pesos_CS, ',', 30, 4);//12, 3 --- 30, 4
    bcs = leer_datos(Bias_CS, ',', 4, 1);//3, 1 ---- 4, 1
    //Se transponen los bias de capa oculta y de capa de salida
    bco = transpose(bco);
    bcs = transpose(bcs);
    int pos;

    //Almacenamiento de los vectores de entrada
    X = datos;
    //Almacenamiento de la salida estimada sin codificar

  
  
  /*int clasificador = 0;
  std::vector<int> colorc;*/


    for (int i = 0; i < fil_size(X); ++i)
    {
        OutputCo = tan_h(mat_sum(mat_mult(grab_data(X,i,i+1,0,308),Wco),bco));//Si entrego solo un vector de una fila pongo X sin grab_data
        Output = sigmoid(mat_sum(mat_mult(OutputCo,Wcs),bcs));

pos = get_maxnumberposition(Output);
//cout << "Posicion de mayor numero es:" << pos << endl;

        cout << "Porcentajes:" << '[' << (Output[0][0])*100 << "% ," << (Output[0][1])*100 << "% ," << (Output[0][2])*100 << "% ," << (Output[0][3])*100 << "%]" << endl;

        if (pos == 0)
        {
            cout << "Objeto: Silla" << endl;
      clasificador = 1;       
        }
        else if (pos == 1)
        {
            cout << "Objeto: Mesa" << endl;
      clasificador = 2;
        }
        else if (pos == 2)
        {
            cout << "Objeto: Mueble" << endl;
      clasificador = 3;
        }
        else if (pos == 3)
  {
      cout << "Objeto: Desconocido" << endl;
      clasificador = 0;
  }
  

        imp_mat(Output, "Salida: ");
  
  colorc.push_back(clasificador);//guardar cada cluster en una posicion de un vector (save) 
        cout << "Filas: " << fil_size(Output) << "\n" << "Columnas: " << col_size(Output) << "\n" << endl;

    }
  //cout <<  << colorc.size();
  std::cout << "tamaños= "<< colorc.size() << std::endl;


    for (int i = 0; i < colorc.size(); ++i)
    {
  std::cout << "Type= "<< colorc [i] << std::endl;  
    } 

    //std::cout << "Objetos clasificados: "<< colorc.size() << std::endl;
  //std::cout << "Clasificador: "<< colorc << std::endl;
}//final if red
//--------------------------------------//
else if(ctype == 2){
//---------------------SVM---------------//
    cout << "Entro al SVC" << endl;
    const char* Pesos = "pesos/Weights_SVM.csv";
    std::vector <std::vector <float> > X;
    std::vector <std::vector <float> > Weights;
    std::vector <float> Output;
    int indice;

    Weights = leer_datos(Pesos, ',', 4, 308);

    X = datos;

    /*int clasificador = 0;
    std::vector<int> colorc;*/

    for (int i = 0; i < fil_size(X); ++i){

        Output = SVC_prediccion(Weights,grab_data(X,i,i+1,0,308));
        indice = distance(Output.begin(), max_element(Output.begin(), Output.begin() + Output.size()));
        cout << "Salida: [" << Output[0] <<","<< Output[1] <<","<< Output[2] <<","<< Output[3] <<"]"<< endl;
        cout << "Indice de mayor valor: " << *max_element(Output.begin(), Output.begin() + Output.size()) << " El valor del indice es: " << indice << endl;

        if (indice == 0)
        {
            cout << "Objeto: Desconocido" << endl;
      clasificador = 0;
        }
        else if (indice == 1)
        {
            cout << "Objeto: Mesa" << endl;
      clasificador = 2;
        }
        else if (indice == 2)
        {
            cout << "Objeto: Mueble" << endl;
      clasificador = 3;
        }
        else if (indice == 3)
        {
            cout << "Objeto: Silla" << endl;
      clasificador = 1;
        }

        colorc.push_back(clasificador);//guardar cada cluster en una posicion de un vector (save)

    }

    std::cout << "tamaños= "<< colorc.size() << std::endl;


    for (int i = 0; i < colorc.size(); ++i)
    {
  std::cout << "Type= "<< colorc [i] << std::endl;  
    } 
}//fin if svc
//--------------------------------------//

pcl::PointCloud<pcl::PointXYZRGB>::Ptr clasificada(new pcl::PointCloud<pcl::PointXYZRGB>); 
    std::vector< pcl::PointIndices >::iterator i_clas;
      int k = 0;


      clasificada->width = cloudRGB->width;
      clasificada->height = cloudRGB->height;
      clasificada->is_dense = cloudRGB->is_dense;


      for (size_t i_point = 0; i_point < cloudRGB->points.size (); i_point++)
      {
        pcl::PointXYZRGB point;
        point.x = *(cloudRGB->points[i_point].data);
        point.y = *(cloudRGB->points[i_point].data + 1);
        point.z = *(cloudRGB->points[i_point].data + 2);
        point.r = 255;
        point.g = 255;
        point.b = 255;
        clasificada->points.push_back (point);
      }

      for (i_clas = cluster_indices.begin (); i_clas != cluster_indices.end (); i_clas++)

      {       
      
    if (colorc [k] == 0)
    {
            std::vector<int>::iterator i_point;
      for (i_point = i_clas->indices.begin (); i_point != i_clas->indices.end (); i_point++)
            {
            int rgb;
            rgb = *i_point;
      //Blanco 246-246-246
            clasificada->points[rgb].r = 255;
            clasificada->points[rgb].g = 0;
            clasificada->points[rgb].b = 0;
            }
          }

    if (colorc [k] == 1)
    {
            std::vector<int>::iterator i_point;
      for (i_point = i_clas->indices.begin (); i_point != i_clas->indices.end (); i_point++)
            {
            int rgb;
            rgb = *i_point;
      //Amarillo 237-255-033
            clasificada->points[rgb].r = 237;
            clasificada->points[rgb].g = 255;
            clasificada->points[rgb].b = 033;
            }
          }

    if (colorc [k] == 2)
    {
            std::vector<int>::iterator i_point;
      for (i_point = i_clas->indices.begin (); i_point != i_clas->indices.end (); i_point++)
            {
            int rgb;
            rgb = *i_point;
      //Azul 006-057-113
            clasificada->points[rgb].r = 006;
            clasificada->points[rgb].g = 057;
            clasificada->points[rgb].b = 113;
            }
          }

    if (colorc [k] == 3)
    {
            std::vector<int>::iterator i_point;
      for (i_point = i_clas->indices.begin (); i_point != i_clas->indices.end (); i_point++)
            {
            int rgb;
            rgb = *i_point;
      //Verde 036-231-017 
            clasificada->points[rgb].r = 036;
            clasificada->points[rgb].g = 231;
            clasificada->points[rgb].b = 017;
            }
          } 
  k++;
      }

//---------------------------------------//


      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    int v1(0);
    viewer->createViewPort (0.0,0.0,0.5,0.5,v1);
    viewer->setBackgroundColor(0,0,0,v1); 
    viewer->addText("Original", 10, 10, "right", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud, 130, 130, 130);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color1, "Original", v1);

    int v2(0);
    viewer->createViewPort (0.5,0.0,1.0,0.5,v2);
    viewer->setBackgroundColor(0.1,0.1,0.1,v2); // background color light
    viewer->addText("Suelo", 10, 10, "left", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud_in, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_in, color2, "Suelo", v2);
/*
                int v3(0);
    viewer->createViewPort (0.0,0.5,0.5,1.0,v3);
    viewer->setBackgroundColor(0,0,0,v3); // background color light
    //viewer->addText("Suelo", 0, 10, "right",v3);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud_in, 0, 0, 100);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_in, color3, "New", v3);
*/
    int v3(0);
    viewer->createViewPort (0.0,0.5,0.5,1.0,v3);
    viewer->setBackgroundColor(0,0,0,v3); // background color light
    //viewer->addText("cec", 0, 10, "left",v4);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(color_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(color_cloud, rgb1,"Cluster", v3);

    int v4(0);
    viewer->createViewPort (0.5,0.5,1.0,1.0,v4);
    viewer->setBackgroundColor(0,0,0,v4); // background color light
    //viewer->addText("cec", 0, 10, "left",v4);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(clasificada);
    viewer->addPointCloud<pcl::PointXYZRGB>(clasificada, rgb,"Clasificada", v4);

    
    while (!viewer->wasStopped ())
    {   
          viewer->spinOnce (100);
          boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }   
    
    /*std::cerr << "Saving...\n", tt.tic ();
      pcl::io::savePCDFile ("output.pcd", *color_cloud);
      std::cerr << ">> Done: " << tt.toc () << " ms\n";*/

      return (0);
  
}

int leer_filas(const char* cadena, char delimeter){
        std::ifstream file(cadena);
        std::string line;
        //int Cols = 0;
        int Fils = 0;

        while(getline(file,line))
        {
            std::stringstream sep(line);
            std::string value;
            int Cols = 0;
          
            while(getline(sep,value,delimeter))
            {
               Cols++;
            }
            Fils++;
        }
        return Fils;
}

int leer_columnas(const char* cadena, char delimeter){
        std::ifstream file(cadena);
        std::string line;
        int Colss = 0;
        int Fils = 0;

        while(getline(file,line))
        {
            std::stringstream sep(line);
            std::string value;
            int Cols = 0;
          
            while(getline(sep,value,delimeter))
            {
               Colss = Cols++;
            }
            Fils++;
        }
        return Colss+1;
}

std::vector <std::vector <float> > leer_datos(const char* cadena, char delimeter, int Nfilas, int Ncolumnas){
        std::ifstream file(cadena);
        std::string line;
        string vect[Nfilas][Ncolumnas];
        std::vector <std::vector <float> > numeros;
        //int Cols = 0;
        int Fils = 0;

        numeros = crear_matriz(Nfilas, Ncolumnas);

        while(getline(file,line))
        {
            std::stringstream sep(line);
            std::string value;
            int Cols = 0;
          
            while(getline(sep,value,delimeter))
            {
               //std::cout << "Value(" << value << ")\n";
               vect[Fils][Cols] = value;
               Cols++;
            }
            //std::cout << "Line Finished" << std::endl;
            Fils++;
        }

        for(int i = 0; i < Nfilas; ++i)
        {
            for(int j = 0; j < Ncolumnas; ++j)
            {
                numeros[i][j] = atof(vect[i][j].c_str()); // string to float
                //cout << "Posicion " << i << "," << j << ": " << numeros[i][j] << endl;
            }            
        }
    return numeros;
}

std::vector <std::vector <float> > crear_matriz(int fil, int col){

    std::vector <std::vector<float> > mat_zeros(fil, std::vector<float>(col, 0.0));
    //...
    return mat_zeros;
}

std::vector <std::vector <float> > transpose(std::vector <std::vector <float> > matrix){
    std::vector <std::vector <float> > trans;
    int f = fil_size(matrix);
    int c = col_size(matrix);

    trans = crear_matriz(c, f);

    for(int i = 0; i < f; ++i)
    {
        for(int j = 0; j < c; ++j)
        {
           trans[j][i] = matrix[i][j];
        }

    }
    return trans; 
}

void imp_mat(std::vector <std::vector <float> > mat, string nombre){
    cout << "Matriz " << nombre << endl;
    for(int i = 0; i < fil_size(mat); ++i)
        {
            for(int j = 0; j < col_size(mat); ++j)
            {
                //cout << "Fila: " << i << ", Columna: " << j << "= " << dataset[i][j] << endl;
                cout << mat[i][j] << ",";
                if(j == col_size(mat)-1)
                {
                    cout << '\n';
                }
            }            
        }
}
std::vector <std::vector <float> > mat_mult(std::vector <std::vector <float> > m1, std::vector <std::vector <float> > m2){
    int r1 = fil_size(m1);
    int r2 = fil_size(m2);
    int c1 = col_size(m1);
    int c2 = col_size(m2);
    std::vector <std::vector <float> > mult;

    mult = crear_matriz(r1, c2);

    if(c1 == r2){
        //cout << "La multiplicacion resultante sera de: " << r1 << "x" << c2 << endl;
        for(int i = 0; i < r1; ++i){
            for(int j = 0; j < c2; ++j){
                for(int k = 0; k < c1; ++k)
                {
                    mult[i][j] += m1[i][k] * m2[k][j];
                }
            }
        }
    }
    else{
        cout << "No corresponden las columnas de M1 con las filas de M2" << endl;
    }
    return mult;
}

std::vector <std::vector <float> > mat_sum(std::vector <std::vector <float> > m1, std::vector <std::vector <float> > m2){
    int r1 = fil_size(m1);
    int r2 = fil_size(m2);
    int c1 = col_size(m1);
    int c2 = col_size(m2);
    std::vector <std::vector <float> > sum;

    sum = crear_matriz(r1, c2);

    if(r1 == r2 && c1 == c2){
        //cout << "La suma resultante sera de: " << r1 << "x" << c2 << endl;
        for(int i = 0; i < r1; ++i){
            for(int j = 0; j < c2; ++j)
            {
                sum[i][j] = m1[i][j] + m2[i][j];
            }
        }
    }
    else{
        cout << "No corresponden las filas o las columnas" << endl;
    }
    return sum;
}

std::vector <std::vector <float> > mat_res(std::vector <std::vector <float> > m1, std::vector <std::vector <float> > m2){
    int r1 = fil_size(m1);
    int r2 = fil_size(m2);
    int c1 = col_size(m1);
    int c2 = col_size(m2);
    std::vector <std::vector <float> > res;

    res = crear_matriz(r1, c2);
       if(r1 == r2 && c1 == c2){
        //cout << "La resta resultante sera de: " << r1 << "x" << c2 << endl;
        for(int i = 0; i < r1; ++i){
            for(int j = 0; j < c2; ++j)
            {
                res[i][j] = m1[i][j] - m2[i][j];
            }
        }
    }
    else{
        cout << "No corresponden las filas o las columnas" << endl;
    }
    return res;
}

