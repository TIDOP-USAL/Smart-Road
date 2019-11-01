/*
* viewer_3d.cpp
*
*/
#include "viewer_3d.h"

#include <osg/Geometry>
#include <osg/Geode>
#include <osgDB/ReadFile>

Viewer3DInterface* Viewer3DInterface::mInstance = NULL;

/////////////////////////
/// Singleton methods ///
//////////////////////////////////////////////////////////////////////////////////

/** Get singleton object instance
* @return Viewer3DInterface*
*/
Viewer3DInterface* Viewer3DInterface::getInstance()
{
    if(mInstance == NULL){
        mInstance = new Viewer3DInterface();
        osgDB::Registry::instance()->addReaderWriter(new cloud3D::PointCloudReader);
        osgDB::Registry::instance()->addReaderWriter(new cloud3D::OOCOctreeReader);
    }
    return mInstance;
}

/** Function to delete the instance
* @return void
*/
void Viewer3DInterface::clear()
{
    delete mInstance;
    mInstance = NULL;
}

/** Default constructor
* @return void
*/
Viewer3DInterface::Viewer3DInterface()
{

}

void Viewer3DInterface::plotCloud(OSGWidget* io_OSGViewer, pcl::PointCloud<pcl::PointXYZ>::Ptr i_cloud_ptr){
    osg::ref_ptr<osg::Geode> geode = osg::ref_ptr<osg::Geode>(new osg::Geode());
    osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

    osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());

    for (std::size_t i=0; i<i_cloud_ptr->points.size(); i++){
        vertices->push_back (osg::Vec3 (i_cloud_ptr->points[i].x, i_cloud_ptr->points[i].y, i_cloud_ptr->points[i].z));
    }

    geometry->setVertexArray (vertices.get());
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));

    geode->addDrawable (geometry.get());
    osg::StateSet* state = geometry->getOrCreateStateSet();
    state->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    io_OSGViewer->getViewer()->setSceneData(geode.get());
}


void Viewer3DInterface::plotCloud(OSGWidget* io_OSGViewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr i_cloud_ptr){
    osg::ref_ptr<osg::Geode> geode = osg::ref_ptr<osg::Geode>(new osg::Geode());
    osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

    osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
    osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());

    for (std::size_t i=0; i<i_cloud_ptr->points.size(); i++){
        vertices->push_back (osg::Vec3 (i_cloud_ptr->points[i].x, i_cloud_ptr->points[i].y, i_cloud_ptr->points[i].z));
        uint32_t rgb_val_;
        memcpy(&rgb_val_, &(i_cloud_ptr->points[i].rgb), sizeof(uint32_t));

        uint32_t red,green,blue;
        blue=rgb_val_ & 0x000000ff;
        rgb_val_ = rgb_val_ >> 8;
        green=rgb_val_ & 0x000000ff;
        rgb_val_ = rgb_val_ >> 8;
        red=rgb_val_ & 0x000000ff;

        colors->push_back (osg::Vec4f ((float)red/255.0f, (float)green/255.0f, (float)blue/255.0f,1.0f));
    }

    geometry->setVertexArray (vertices.get());
    geometry->setColorArray (colors.get());
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));

    geode->addDrawable (geometry.get());
    osg::StateSet* state = geometry->getOrCreateStateSet();
    state->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    io_OSGViewer->getViewer()->setSceneData(geode.get());
}

void Viewer3DInterface::plotCloud(OSGWidget* io_OSGViewer, pcl::PointCloud<pcl::PointXYZI>::Ptr i_cloud_ptr, float i_intensityMaxVal){
    osg::ref_ptr<osg::Geode> geode = osg::ref_ptr<osg::Geode>(new osg::Geode());
    osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

    osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
    osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());

    for (std::size_t i=0; i<i_cloud_ptr->points.size(); i++){
        vertices->push_back (osg::Vec3 (i_cloud_ptr->points[i].x, i_cloud_ptr->points[i].y, i_cloud_ptr->points[i].z));
        float intensity = i_cloud_ptr->points[i].intensity /i_intensityMaxVal;
        colors->push_back (osg::Vec4f (intensity,intensity,intensity,1.0f));
    }

    geometry->setVertexArray (vertices.get());
    geometry->setColorArray (colors.get());
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));

    geode->addDrawable (geometry.get());
    osg::StateSet* state = geometry->getOrCreateStateSet();
    state->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    io_OSGViewer->getViewer()->setSceneData(geode.get());
}

void Viewer3DInterface::plotOOCOctree_Solid(OSGWidget* io_OSGViewer, std::string i_OOCPath){
    mFactory =   new cloud3D::Cloud3DFactory_Colored<pcl::PointXYZ>();

    osg::ref_ptr<cloud3D::Cloud3DOptions> options = new cloud3D::Cloud3DOptions();

    if(CLOUD3D_OOCO_DEPTH != NULL){
        cloud3D::OOCOctreeReader::OOCOctreeOptions* opts = new cloud3D::OOCOctreeReader::OOCOctreeOptions();
        opts->setDepth(static_cast<int>(CLOUD3D_OOCO_DEPTH), static_cast<int>(CLOUD3D_OOCO_DEPTH));
        options = opts;
    }
    if(CLOUD3D_SAMPLING_RATE != NULL){
        options->setSamplingRate(static_cast<float>(CLOUD3D_SAMPLING_RATE));
    }
    if(CLOUD3D_PT_SIZE != NULL){
        mFactory->setPointSize(static_cast<int>(CLOUD3D_PT_SIZE));
    }
    options->setFactory(mFactory);

    io_OSGViewer->getViewer()->setSceneData(osgDB::readNodeFile(i_OOCPath, options));
}
void Viewer3DInterface::plotOOCOctree_Coord(OSGWidget* io_OSGViewer, std::string i_OOCPath, std::string i_label){
    mFactory =   new cloud3D::Cloud3DFactory_Coord<pcl::PointXYZ, pcl::PointXYZ>(i_label) ;
    osg::ref_ptr<cloud3D::Cloud3DOptions> options = new cloud3D::Cloud3DOptions();

    if(CLOUD3D_OOCO_DEPTH != NULL){
        cloud3D::OOCOctreeReader::OOCOctreeOptions* opts = new cloud3D::OOCOctreeReader::OOCOctreeOptions();
        opts->setDepth(static_cast<int>(CLOUD3D_OOCO_DEPTH), static_cast<int>(CLOUD3D_OOCO_DEPTH));
        options = opts;
    }
    if(CLOUD3D_SAMPLING_RATE != NULL){
        options->setSamplingRate(static_cast<float>(CLOUD3D_SAMPLING_RATE));
    }
    if(CLOUD3D_PT_SIZE != NULL){
        mFactory->setPointSize(static_cast<int>(CLOUD3D_PT_SIZE));
    }
    options->setFactory(mFactory);

    io_OSGViewer->getViewer()->setSceneData(osgDB::readNodeFile(i_OOCPath, options));

}	
void Viewer3DInterface::plotOOCOctree_RGB(OSGWidget* io_OSGViewer, std::string i_OOCPath){
    mFactory =  new cloud3D::Cloud3DFactory_RGB<pcl::PointXYZ, pcl::RGB>() ;
    osg::ref_ptr<cloud3D::Cloud3DOptions> options = new cloud3D::Cloud3DOptions();

    if(CLOUD3D_OOCO_DEPTH != NULL){
        cloud3D::OOCOctreeReader::OOCOctreeOptions* opts = new cloud3D::OOCOctreeReader::OOCOctreeOptions();
        opts->setDepth(static_cast<int>(CLOUD3D_OOCO_DEPTH), static_cast<int>(CLOUD3D_OOCO_DEPTH));
        options = opts;
    }
    if(CLOUD3D_SAMPLING_RATE != NULL){
        options->setSamplingRate(static_cast<float>(CLOUD3D_SAMPLING_RATE));
    }
    if(CLOUD3D_PT_SIZE != NULL){
        mFactory->setPointSize(static_cast<int>(CLOUD3D_PT_SIZE));
    }
    options->setFactory(mFactory);

    io_OSGViewer->getViewer()->setSceneData(osgDB::readNodeFile(i_OOCPath, options));
}	
void Viewer3DInterface::plotOOCOctree_Intensity(OSGWidget* io_OSGViewer, std::string i_OOCPath){
    mFactory =  new cloud3D::Cloud3DFactory_Intensity<pcl::PointXYZ, pcl::Intensity>() ;
    osg::ref_ptr<cloud3D::Cloud3DOptions> options = new cloud3D::Cloud3DOptions();

    if(CLOUD3D_OOCO_DEPTH != NULL){
        cloud3D::OOCOctreeReader::OOCOctreeOptions* opts = new cloud3D::OOCOctreeReader::OOCOctreeOptions();
        opts->setDepth(static_cast<int>(CLOUD3D_OOCO_DEPTH), static_cast<int>(CLOUD3D_OOCO_DEPTH));
        options = opts;
    }
    if(CLOUD3D_SAMPLING_RATE != NULL){
        options->setSamplingRate(static_cast<float>(CLOUD3D_SAMPLING_RATE));
    }
    if(CLOUD3D_PT_SIZE != NULL){
        mFactory->setPointSize(static_cast<int>(CLOUD3D_PT_SIZE));
    }
    options->setFactory(mFactory);

    io_OSGViewer->getViewer()->setSceneData(osgDB::readNodeFile(i_OOCPath, options));
}	
void Viewer3DInterface::plotOOCOctree_Label(OSGWidget* io_OSGViewer, std::string i_OOCPath){
    mFactory =   new cloud3D::Cloud3DFactory_Label<pcl::PointXYZ, pcl::Label>() ;
    osg::ref_ptr<cloud3D::Cloud3DOptions> options = new cloud3D::Cloud3DOptions();

    if(CLOUD3D_OOCO_DEPTH != NULL){
        cloud3D::OOCOctreeReader::OOCOctreeOptions* opts = new cloud3D::OOCOctreeReader::OOCOctreeOptions();
        opts->setDepth(static_cast<int>(CLOUD3D_OOCO_DEPTH), static_cast<int>(CLOUD3D_OOCO_DEPTH));
        options = opts;
    }
    if(CLOUD3D_SAMPLING_RATE != NULL){
        options->setSamplingRate(static_cast<float>(CLOUD3D_SAMPLING_RATE));
    }
    if(CLOUD3D_PT_SIZE != NULL){
        mFactory->setPointSize(static_cast<int>(CLOUD3D_PT_SIZE));
    }
    options->setFactory(mFactory);

    io_OSGViewer->getViewer()->setSceneData(osgDB::readNodeFile(i_OOCPath, options));
}	
