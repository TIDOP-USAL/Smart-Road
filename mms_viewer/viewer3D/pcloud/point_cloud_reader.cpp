#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <osg/Geode>

#include "pcloud/point_cloud_reader.h"

namespace cloud3D
{
    REGISTER_OSGPLUGIN(pcd, PointCloudReader);

    PointCloudReader::PointCloudReader()
    {
        supportsExtension("pcd","PCL Point Cloud Format");
    }

    PointCloudReader::PointCloudReader(const ReaderWriter& i_readWriter,
                                       const osg::CopyOp& i_copyOp)
    {
        supportsExtension("pcd","PCL Point Cloud Format");
    }

    PointCloudReader::~PointCloudReader()
    {
    }


    osgDB::ReaderWriter::ReadResult PointCloudReader::readNode (const std::string& i_filename,
                                                                const osgDB::ReaderWriter::Options* i_options_p) const
    {
        boost::filesystem::path filePath(i_filename);

        if(filePath.extension().string() !=".pcd"){
            return ReadResult();
        }

        osg::ref_ptr<Cloud3DOptions>  cOpts = dynamic_cast<Cloud3DOptions*>(const_cast<osgDB::Options*>(i_options_p));

        if(cOpts == NULL){
            cOpts = new Cloud3DOptions(new Cloud3DFactory_Coord<>());
        }

        if(!boost::filesystem::exists(filePath)){
            return ReadResult(ReaderWriter::ReadResult::FILE_NOT_FOUND);
        }

        pcl::PCLPointCloud2Ptr cloud(new pcl::PCLPointCloud2());

        pcl::PCDReader reader;
        if(reader.read(i_filename, *cloud) < 0){
            return ReadResult("Failed to read point cloud\n");
        }

        cOpts->getFactory()->setInputCloud(cloud);

        osg::Node* node = cOpts->getFactory()->buildNode();
        if(node == NULL){
            return ReadResult("Failed to build point cloud geometry\n");
        }
        node->setName(i_filename.c_str());

        return node;
    }

    osgDB::ReaderWriter::Features PointCloudReader::supportedFeatures() const
    {
        return FEATURE_READ_NODE;
    }

} 
