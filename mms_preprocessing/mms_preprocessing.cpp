#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

#include <liblas/liblas.hpp>
#include <fstream>  // std::ifstream
#include <iostream> // std::cout
#include <exception>

#include <vector>

using PointType = pcl::PointXYZ;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;

using namespace pcl;
using namespace pcl::outofcore;

using pcl::console::parse_argument;
using pcl::console::parse_file_extension_argument;
using pcl::console::find_switch;
using pcl::console::print_error;
using pcl::console::print_warn;
using pcl::console::print_info;

#define NBPTS_MIN 10U
class CloudsGrid
{
private:
  pcl::PointXYZ mGridMin;
  pcl::PointXYZ mGridMax;
  size_t mNbCellsX, mNbCellsY;
  float mGridSize, mGridSizeX, mGridSizeY;
  std::vector<PointCloudPtr>* mCloudsGrid_p;

public:

  CloudsGrid(pcl::PointXYZ i_GridMin, pcl::PointXYZ i_GridMax, float i_gridSize)
  {
    this->mGridMin = i_GridMin;
  	this->mGridMax = i_GridMax;
    this->mGridSize = i_gridSize;

    this->mGridSizeX = this->mGridMax.x - this->mGridMin.x;
    this->mGridSizeY = this->mGridMax.y - this->mGridMin.y;
    this->mNbCellsX = ceil((this->mGridSizeX)/i_gridSize);
    this->mNbCellsY = ceil((this->mGridSizeY)/i_gridSize);
    this->mCloudsGrid_p = new std::vector<PointCloudPtr>(this->mNbCellsX*this->mNbCellsY);
    for (size_t i = 0; i < this->mCloudsGrid_p->size(); i++)
    {
      PointCloudPtr cloud (new pcl::PointCloud<PointType>());
      this->mCloudsGrid_p->at(i) = cloud;
    }
    
  }

  ~CloudsGrid()
  {
  }

  pcl::PointXYZ getGridMin() {
  	return this->mGridMin;
  }
  
  pcl::PointXYZ getGridMax() {
  	return this->mGridMax;
  }

  float getGridSizeX(){
    return this->mGridSizeX;
  }

  float getGridSizeY(){
    return this->mGridSizeY;
  }

  size_t getNbCellsX(){
    return this->mNbCellsX;
  }
  
  size_t getNbCellsY(){
    return this->mNbCellsY;
  }

  float getGridSize(){
    return this->mGridSize;
  }
  std::vector<PointCloudPtr>* getCloudsGrid() {
  	return this->mCloudsGrid_p;
  }

  PointCloudPtr getCloud(size_t i_cellX, size_t i_cellY) {
  	return this->mCloudsGrid_p->at(i_cellX*this->mNbCellsY+i_cellY);
  }
  
  uint8_t addPoint(liblas::Point const& i_p) {
    PointType  pt(i_p.GetX()-this->mGridMin.x,i_p.GetY()-this->mGridMin.y,i_p.GetZ()-this->mGridMin.z);
    size_t cellX = floor(pt.x/this->mGridSize);
    size_t cellY = floor(pt.y/this->mGridSize);
    this->getCloud(cellX,cellY)->points.push_back(pt);
  	return 0;
  }
};


using octree_disk = OutofcoreOctreeBase<>;

const int OCTREE_DEPTH (0);
const int OCTREE_RESOLUTION (1);

CloudsGrid*
getCloudsGridFromFile (boost::filesystem::path las_path, float grid_size)
{
  print_info ("Reading: %s ", las_path.c_str ());

  std::ifstream ifs;
  ifs.open(las_path.c_str(), std::ios::in | std::ios::binary);
  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);
  liblas::Header const& header = reader.GetHeader();

  CloudsGrid* outGrid = new CloudsGrid(pcl::PointXYZ(floor(header.GetMinX()),floor(header.GetMinY()),floor(header.GetMinZ())),
                                       pcl::PointXYZ(ceil(header.GetMaxX()),ceil(header.GetMaxY()),ceil(header.GetMaxZ())), grid_size);

  std::cout << "Compressed: " << (header.Compressed() == true) ? "true":"false";
  std::cout << "Signature: " << header.GetFileSignature() << '\n';
  std::cout << "Points count: " << header.GetPointRecordsCount() << '\n';
  while (reader.ReadNextPoint())
  {
      outGrid->addPoint(reader.GetPoint());
  }
  return outGrid;
}

int
outofcoreProcess (std::vector<boost::filesystem::path> las_paths, boost::filesystem::path root_dir, 
                  int depth, double resolution, int build_octree_with, bool gen_lod, bool overwrite, bool multiresolution, float grid_size)
{
  // Bounding box min/max pts
  pcl::PointXYZ min_pt, max_pt;

  CloudsGrid* cloudsGrid_p;

  // Iterate over all las files resizing min/max
  for (std::size_t i = 0; i < las_paths.size (); i++)
  {

    cloudsGrid_p = getCloudsGridFromFile (las_paths[i], grid_size);
    
    std::uint64_t total_pts = 0;

    for (size_t cloudID = 0; cloudID < cloudsGrid_p->getCloudsGrid()->size(); cloudID++)
    {
      if(cloudsGrid_p->getCloudsGrid()->at(cloudID)->points.size() <= NBPTS_MIN){
        std::cout << "WARNING: Grid: " << cloudID << " NbPoints " << cloudsGrid_p->getCloudsGrid()->at(cloudID)->points.size() << std::endl;
        continue;
      }
      pcl::PointXYZ tmp_min_pt, tmp_max_pt;

      // Get min/max
      getMinMax3D (*cloudsGrid_p->getCloudsGrid()->at(cloudID), min_pt, max_pt);
      std::cout << "Bounds: " << min_pt << " - " << max_pt << std::endl;

      // The bounding box of the root node of the out-of-core octree must be specified
      const Eigen::Vector3d bounding_box_min (min_pt.x, min_pt.y, min_pt.z);
      const Eigen::Vector3d bounding_box_max (max_pt.x, max_pt.y, max_pt.z);

      //specify the directory and the root node's meta data file with a
      //".oct_idx" extension (currently it must be this extension)
      std::string cellFolderName(las_paths[i].stem().c_str());
      cellFolderName.append("_").append(std::to_string(cloudID));
      boost::filesystem::path cell_path_on_disk (root_dir / cellFolderName);
      
      boost::filesystem::path octree_path_on_disk (cell_path_on_disk / "tree.oct_idx");

      print_info ("Writing: %s\n", octree_path_on_disk.c_str ());
      //make sure there isn't an octree there already
      if (boost::filesystem::exists (octree_path_on_disk))
      {
        if (overwrite)
        {
          boost::filesystem::remove_all (root_dir);
        }
        else
        {
          PCL_ERROR ("There's already an octree in the default location. Check the tree directory\n");
          return (-1);
        }
      }

      octree_disk *outofcore_octree;

      //create the out-of-core data structure (typedef'd above)
      if (build_octree_with == OCTREE_DEPTH)
      {
        outofcore_octree = new octree_disk (depth, bounding_box_min, bounding_box_max, octree_path_on_disk, "ECEF");
      }
      else
      {
        outofcore_octree = new octree_disk (bounding_box_min, bounding_box_max, resolution, octree_path_on_disk, "ECEF");
      }


      PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);

      pcl::toPCLPointCloud2(*cloudsGrid_p->getCloudsGrid()->at(cloudID),*cloud);

      std::uint64_t pts = 0;
      
      if (gen_lod && !multiresolution)
      {
        print_info ("  Generating LODs\n");
        pts = outofcore_octree->addPointCloud_and_genLOD (cloud);
      }
      else
      {
        pts = outofcore_octree->addPointCloud (cloud, false);
      }
      
      print_info ("Successfully added %lu points\n", pts);
      print_info ("%lu Points were dropped (probably NaN)\n", cloud->width*cloud->height - pts);
      
      
      total_pts += pts;

      double x, y;
      outofcore_octree->getBinDimension (x, y);

      print_info ("  Depth: %i\n", outofcore_octree->getDepth ());
      print_info ("  Resolution: [%f, %f]\n", x, y);

      if(multiresolution)
      {
        print_info ("Generating LOD...\n");
        outofcore_octree->setSamplePercent (0.25);
        outofcore_octree->buildLOD ();
      }

      //free outofcore data structure; the destructor forces buffer flush to disk
      delete outofcore_octree;
    }
   print_info ("Added a total of %lu from %d clouds\n",total_pts, las_paths.size ());

  }

  return 0;
}

void
printHelp (int, char **argv)
{
  print_info ("This program is used to process las files into an outofcore data structure viewable by the mms_viewer\n\n");
  print_info ("%s <options> <input>.las <output_treeGrid_dir>\n", argv[0]);
  print_info ("\n");
  print_info ("Options:\n");
  print_info ("\t -depth <resolution>           \t Octree depth\n");
  print_info ("\t -resolution <resolution>      \t Octree resolution\n");
  print_info ("\t -gen_lod                      \t Generate octree LODs\n");
  print_info ("\t -overwrite                    \t Overwrite existing octree\n");
  print_info ("\t -multiresolution              \t Generate multiresolutoin LOD\n");
  print_info ("\t -grid_size                    \t Segmentation grid size\n");
  print_info ("\t -h                            \t Display help\n");
  print_info ("\n");
}

int
main (int argc, char* argv[])
{
  // Check for help (-h) flag
  if (argc > 1)
  {
    if (find_switch (argc, argv, "-h"))
    {
      printHelp (argc, argv);
      return (-1);
    }
  }

  // If no arguments specified
  if (argc - 1 < 1)
  {
    printHelp (argc, argv);
    return (-1);
  }

  if (find_switch (argc, argv, "-debug"))
  {
    pcl::console::setVerbosityLevel ( pcl::console::L_DEBUG );
  }
  
  // Defaults
  int depth = 4;
  double resolution = .1;
  bool gen_lod = false;
  bool multiresolution = false;
  bool overwrite = false;
  int build_octree_with = OCTREE_DEPTH;
  float grid_size = 10.0F;

  // If both depth and resolution specified
  if (find_switch (argc, argv, "-depth") && find_switch (argc, argv, "-resolution"))
  {
    PCL_ERROR ("Both -depth and -resolution specified, please specify one (Mutually exclusive)\n");
  }
  // Just resolution specified (Update how we build the tree)
  else if (find_switch (argc, argv, "-resolution"))
  {
    build_octree_with = OCTREE_RESOLUTION;
  }

  // Parse options
  parse_argument (argc, argv, "-depth", depth);
  parse_argument (argc, argv, "-resolution", resolution);
  parse_argument (argc, argv, "-grid_size", grid_size);
  gen_lod = find_switch (argc, argv, "-gen_lod");
  overwrite = find_switch (argc, argv, "-overwrite");

  if (gen_lod && find_switch (argc, argv, "-multiresolution"))
  {
    multiresolution = true;
  }
  

  // Parse non-option arguments for las files
  std::vector<int> file_arg_indices = parse_file_extension_argument (argc, argv, ".las");

  std::vector<boost::filesystem::path> las_paths;
  for (const int &file_arg_index : file_arg_indices)
  {
    boost::filesystem::path las_path (argv[file_arg_index]);
    if (!boost::filesystem::exists (las_path))
    {
      PCL_WARN ("File %s doesn't exist", las_path.string ().c_str ());
      continue;
    }
    las_paths.push_back (las_path);

  }

  // Check if we should process any files
  if (las_paths.empty ())
  {
    PCL_ERROR ("No .las files specified\n");
    return -1;
  }

  // Get root directory
  boost::filesystem::path root_dir (argv[argc - 1]);

  // Check if a root directory was specified, use directory of las file
  if (root_dir.extension () == ".las")
    root_dir = root_dir.parent_path () / (root_dir.stem().string() + "_tree").c_str();

  boost::filesystem::create_directory(root_dir);
  return outofcoreProcess (las_paths, root_dir, depth, resolution, build_octree_with, gen_lod, overwrite, multiresolution, grid_size);
}
