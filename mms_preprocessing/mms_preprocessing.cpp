#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

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

// todo: Read clouds as PCLPointCloud2 so we don't need to define PointT explicitly.
//       This also requires our octree to take PCLPointCloud2 as an input.
using PointT = pcl::PointXYZ;

using namespace pcl;
using namespace pcl::outofcore;

using pcl::console::parse_argument;
using pcl::console::parse_file_extension_argument;
using pcl::console::find_switch;
using pcl::console::print_error;
using pcl::console::print_warn;
using pcl::console::print_info;

class CloudsGrid
{
private:
  pcl::PointXYZ mBBMin;
  pcl::PointXYZ mBBMax;
  size_t mNbCols, mNbRows;
  float mGridSize, mBBSizeX, mBBSizeY;
  PCLPointCloud2::Ptr* mCloudsGrid_p;

public:

  CloudsGrid(pcl::PointXYZ i_BBMin, pcl::PointXYZ i_BBMax, float i_gridSize)
  {
    this->mBBMin = i_BBMin;
  	this->mBBMax = i_BBMax;

    this->mBBSizeX = this->mBBMax.x - this->mBBMin.x;
    this->mBBSizeY = this->mBBMax.y - this->mBBMin.y;
    this->mNbRows = ceil((this->mbbSizeX)/i_gridSize);
    this->mNbCols = ceil((this->mBBSizeY)/i_gridSize);
    for (size_t col = 0; col < this->mNbCols; col++)
    {
      for (size_t row = 0; row < this->mNbRows; row++)
      {
        PCLPointCloud2::Ptr cloud_p(new PCLPointCloud2);
        this->mCloudsGrid_p[col*this->mNbRows+row]= cloud_p;
      }
    }
  }

  ~CloudsGrid()
  {
  }

  pcl::PointXYZ getBBMin() {
  	return this->mBBMin;
  }
  
  pcl::PointXYZ getBBMax() {
  	return this->mBBMax;
  }

  float getBBSizeX(){
    return this->mBBSizeX;
  }

  float getBBSizeY(){
    return this->mBBSizeY;
  }

  size_t getNbCols(){
    return this->mNbCols;
  }
  
  size_t getNbRows(){
    return this->mNbRows;
  }

  float getGridSize(){
    return this->mGridSize;
  }
  PCLPointCloud2::Ptr* getCloudsGrid() {
  	return this->mCloudsGrid_p;
  }

  PCLPointCloud2::Ptr getCloud(size_t i_col, size_t i_row) {
  	return this->mCloudsGrid_p[i_col*this->mNbRows+i_row];
  }
  
  uint8_t addPoint(liblas::Point const& i_p) {
    std::cout << i_p.GetX() << ", " << i_p.GetY() << ", " << i_p.GetZ() << "\n";
      this->getCloud(floor(i_p.GetX()-this->getBBMin().x)/this->getBBSizeX(),floor(i_p.GetY()-this->getBBMin().y)/this->getBBSizeY()).;
  	return 0;
  }
};


using octree_disk = OutofcoreOctreeBase<>;

const int OCTREE_DEPTH (0);
const int OCTREE_RESOLUTION (1);

PCLPointCloud2::Ptr
getCloudFromFile (boost::filesystem::path las_path)
{
  print_info ("Reading: %s ", las_path.c_str ());

  // Read las file
  PCLPointCloud2::Ptr cloud(new PCLPointCloud2);

  if (io::loadPCDFile (las_path.string (), *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file\n");
    exit (-1);
  }

  print_info ("(%d)\n", (cloud->width * cloud->height));

  return cloud;
}

CloudsGrid*
getCloudsGridFromFile (boost::filesystem::path las_path, float grid_size)
{
  print_info ("Reading: %s ", las_path.c_str ());

  //TODO: Split and generate OOCOctrees
  // // Read las file
  // PCLPointCloud2::Ptr cloud(new PCLPointCloud2);

  // if (io::loadPCDFile (pcd_path.string (), *cloud) == -1)
  // {
  //   PCL_ERROR ("Couldn't read file\n");
  //   exit (-1);
  // }

  // print_info ("(%d)\n", (cloud->width * cloud->height));

  // return cloud;
  std::ifstream ifs;
  ifs.open(las_path.c_str(), std::ios::in | std::ios::binary);
  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);
  liblas::Header const& header = reader.GetHeader();

  CloudsGrid* outGrid = new CloudsGrid(pcl::PointXYZ(header.GetMinX(),header.GetMinY(),header.GetMinZ()),
                                       pcl::PointXYZ(header.GetMaxX(),header.GetMaxY(),header.GetMaxZ()), grid_size);

  std::cout << "Compressed: " << (header.Compressed() == true) ? "true":"false";
  std::cout << "Signature: " << header.GetFileSignature() << '\n';
  std::cout << "Points count: " << header.GetPointRecordsCount() << '\n';
  while (reader.ReadNextPoint())
  {
      outGrid->addPoint(reader.GetPoint())
  }

}

int
outofcoreProcess (std::vector<boost::filesystem::path> las_paths, boost::filesystem::path root_dir, 
                  int depth, double resolution, int build_octree_with, bool gen_lod, bool overwrite, bool multiresolution, float grid_size)
{
  // Bounding box min/max pts
  PointT min_pt, max_pt;

  // Iterate over all las files resizing min/max
  for (std::size_t i = 0; i < las_paths.size (); i++)
  {

    CloudsGrid* cloudsGrid_p = getCloudsGridFromFile (las_paths[i], grid_size);

    // // Get cloud
    // PCLPointCloud2::Ptr cloud = getCloudFromFile (las_paths[i]);
    // PointCloud<PointXYZ>::Ptr cloudXYZ (new PointCloud<PointXYZ>);

    // fromPCLPointCloud2 (*cloud, *cloudXYZ);

    // PointT tmp_min_pt, tmp_max_pt;

    // if (i == 0)
    // {
    //   // Get initial min/max
    //   getMinMax3D (*cloudXYZ, min_pt, max_pt);
    // }
    // else
    // {
    //   getMinMax3D (*cloudXYZ, tmp_min_pt, tmp_max_pt);

    //   // Resize new min
    //   if (tmp_min_pt.x < min_pt.x)
    //     min_pt.x = tmp_min_pt.x;
    //   if (tmp_min_pt.y < min_pt.y)
    //     min_pt.y = tmp_min_pt.y;
    //   if (tmp_min_pt.z < min_pt.z)
    //     min_pt.z = tmp_min_pt.z;

    //   // Resize new max
    //   if (tmp_max_pt.x > max_pt.x)
    //     max_pt.x = tmp_max_pt.x;
    //   if (tmp_max_pt.y > max_pt.y)
    //     max_pt.y = tmp_max_pt.y;
    //   if (tmp_max_pt.z > max_pt.z)
    //     max_pt.z = tmp_max_pt.z;
    // }
  }

//   std::cout << "Bounds: " << min_pt << " - " << max_pt << std::endl;

//   // The bounding box of the root node of the out-of-core octree must be specified
//   const Eigen::Vector3d bounding_box_min (min_pt.x, min_pt.y, min_pt.z);
//   const Eigen::Vector3d bounding_box_max (max_pt.x, max_pt.y, max_pt.z);

//   //specify the directory and the root node's meta data file with a
//   //".oct_idx" extension (currently it must be this extension)
//   boost::filesystem::path octree_path_on_disk (root_dir / "tree.oct_idx");

//   print_info ("Writing: %s\n", octree_path_on_disk.c_str ());
//   //make sure there isn't an octree there already
//   if (boost::filesystem::exists (octree_path_on_disk))
//   {
//     if (overwrite)
//     {
//       boost::filesystem::remove_all (root_dir);
//     }
//     else
//     {
//       PCL_ERROR ("There's already an octree in the default location. Check the tree directory\n");
//       return (-1);
//     }
//   }

//   octree_disk *outofcore_octree;

//   //create the out-of-core data structure (typedef'd above)
//   if (build_octree_with == OCTREE_DEPTH)
//   {
//     outofcore_octree = new octree_disk (depth, bounding_box_min, bounding_box_max, octree_path_on_disk, "ECEF");
//   }
//   else
//   {
//     outofcore_octree = new octree_disk (bounding_box_min, bounding_box_max, resolution, octree_path_on_disk, "ECEF");
//   }

//   std::uint64_t total_pts = 0;

//   // Iterate over all las files adding points to the octree
//   for (const auto &las_path : las_paths)
//   {

//     PCLPointCloud2::Ptr cloud = getCloudFromFile (las_path);

//     std::uint64_t pts = 0;
    
//     if (gen_lod && !multiresolution)
//     {
//       print_info ("  Generating LODs\n");
//       pts = outofcore_octree->addPointCloud_and_genLOD (cloud);
//     }
//     else
//     {
//       pts = outofcore_octree->addPointCloud (cloud, false);
//     }
    
//     print_info ("Successfully added %lu points\n", pts);
//     print_info ("%lu Points were dropped (probably NaN)\n", cloud->width*cloud->height - pts);
    
// //    assert ( pts == cloud->width * cloud->height );
    
//     total_pts += pts;
//   }

//   print_info ("Added a total of %lu from %d clouds\n",total_pts, las_paths.size ());
  

//   double x, y;
//   outofcore_octree->getBinDimension (x, y);

//   print_info ("  Depth: %i\n", outofcore_octree->getDepth ());
//   print_info ("  Resolution: [%f, %f]\n", x, y);

//   if(multiresolution)
//   {
//     print_info ("Generating LOD...\n");
//     outofcore_octree->setSamplePercent (0.25);
//     outofcore_octree->buildLOD ();
//   }

//   //free outofcore data structure; the destructor forces buffer flush to disk
//   delete outofcore_octree;

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
  float grid_size = 100.0F;

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

  return outofcoreProcess (las_paths, root_dir, depth, resolution, build_octree_with, gen_lod, overwrite, multiresolution, grid_size);
}
