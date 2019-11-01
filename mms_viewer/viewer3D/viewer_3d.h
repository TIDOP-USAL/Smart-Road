/*
* viewer_3d.h
*
*/
#ifndef VIEWER_3D_INTERFACE_H
#define VIEWER_3D_INTERFACE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <OSGWidget/OSGWidget.h>

#include <viewer3D/pcloud/point_cloud_reader.h>
#include <viewer3D/pcloud/outofcore_octree_reader.h>

#define CLOUD3D_OOCO_DEPTH NULL
#define CLOUD3D_SAMPLING_RATE NULL
#define CLOUD3D_PT_SIZE NULL
/**
 * @brief The Viewer3DInterface class
 *
 * Singleton class used to publish in a OSGWidget viewer
 *
 */
class Viewer3DInterface
{

protected:

    static Viewer3DInterface* mInstance; //! The singleton instance

    osg::ref_ptr<cloud3D::Cloud3DFactory> mFactory;
    Viewer3DInterface();

public:

    static Viewer3DInterface* getInstance();

    void clear();

    void plotCloud(OSGWidget* io_OSGViewer, pcl::PointCloud<pcl::PointXYZ>::Ptr i_cloud_ptr);
    void plotCloud(OSGWidget* io_OSGViewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr i_cloud_ptr);
    void plotCloud(OSGWidget* io_OSGViewer, pcl::PointCloud<pcl::PointXYZI>::Ptr i_cloud_ptr, float i_intensityMaxVal);

    void plotOOCOctree_Solid(OSGWidget* io_OSGViewer, std::string i_OOCPath);
    void plotOOCOctree_Coord(OSGWidget* io_OSGViewer, std::string i_OOCPath, std::string i_label = "z");
    void plotOOCOctree_RGB(OSGWidget* io_OSGViewer, std::string i_OOCPath);
    void plotOOCOctree_Intensity(OSGWidget* io_OSGViewer, std::string i_OOCPath);
    void plotOOCOctree_Label(OSGWidget* io_OSGViewer, std::string i_OOCPath);

};

#endif
