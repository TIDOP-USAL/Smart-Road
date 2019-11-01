#ifndef _POINT_CLOUD_READER_H_
#define _POINT_CLOUD_READER_H_

#include <osgDB/ReaderWriter>
#include <osgDB/Options>
#include <osgDB/Registry>

#include "pcloud/point_cloud.h"

namespace cloud3D
{
    /*
      * PointCloudReader
      * osgDB::ReaderWriter used to generate the point cloud model.
      */
    class PointCloudReader : public osgDB::ReaderWriter
    {
    public:
        PointCloudReader();
        PointCloudReader(const ReaderWriter& i_readerWriter, const osg::CopyOp& i_copyOp=osg::CopyOp::SHALLOW_COPY);
        virtual ~PointCloudReader();

        META_Object(cloud3D,PointCloudReader);

        virtual Features supportedFeatures() const;

        virtual ReadResult readNode(const std::string& i_fileName, const osgDB::ReaderWriter::Options* i_options_p) const;
    };
} 

#endif /* _POINT_CLOUD_READER_H_ */
