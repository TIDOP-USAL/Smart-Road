#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#include <string>
#include <map>
#include <boost/any.hpp>
#include <osg/Geometry>
#include <osg/Uniform>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/PCLPointCloud2.h>
#include <osgDB/Options>
#include <osg/Object>


namespace cloud3D
{
    /*
       * Cloud3DFactory
       * Abstract base class for Point Cloud 3D model generation
       */
    class Cloud3DFactory : public osg::Referenced{
    public:
        Cloud3DFactory();
        virtual ~Cloud3DFactory(){}

        /*
           * buildGeometry
           * Build the 3D geometry for the point cloud
           *
           * i_uniqueStateSet -  controls wheather the OSG StateSet defining if rendering attributes
           *                     are shared between generated models.
           */
        virtual osg::Geometry* buildGeometry(bool i_uniqueStateSet = false) = 0;

        /*
           *  buildNode
           *  Packages a Geometry in a osg::Node for addition to a scene
           */
        virtual osg::Node* buildNode();

    private:
        std::map<std::string, boost::any> mInputClouds;

    public:
        /*
           * setInputCloud
           * Add a templated input cloud to the factory for constructing 3d model
           */
        template<typename PointT>
        void setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr& i_cloud){
            mInputClouds[pcl::getFieldsList(*i_cloud)] = i_cloud;
        }

        /*
           * setInputCloud
           * Add a  cloud to the factory as an input for constructing 3d model
           */
        virtual void setInputCloud(const pcl::PCLPointCloud2::ConstPtr& i_cloud) = 0;

        /*
           * clearInput
           * remove all inputs
           */
        void clearInput(){mInputClouds.clear();}

        /*
           * setIndices
           * select indices of the point clouds that will be rendered
           */
        void setIndices(const pcl::IndicesConstPtr& i_indices){
            mIndices = i_indices;
        }

    protected:
        osg::ref_ptr<osg::StateSet> mStateSet;
        pcl::IndicesConstPtr mIndices;

        template<typename PointT>
        typename pcl::PointCloud<PointT>::ConstPtr getInputCloud() const ;

        template<typename PointT>
        void addXYZToVertexBuffer(osg::Geometry& o_geom, const pcl::PointCloud<pcl::PointXYZ>& i_cloud) const;

    public:
        /*
           * setPointSize
           * set the rendering point size [pix]
           */
        void setPointSize(int i_size);
    };


    /*
       * Cloud3DFactory_Colored
       * Factory for generate single solid colored osg models of point clouds.
       */
    template<typename PointT = pcl::PointXYZ>
    class Cloud3DFactory_Colored : public Cloud3DFactory{

    public:
        Cloud3DFactory_Colored();

        virtual osg::Geometry* buildGeometry(bool i_uniqueStateSet=false);

        using Cloud3DFactory::setInputCloud;
        virtual void setInputCloud(const pcl::PCLPointCloud2::ConstPtr& i_cloud);

        void setColor(float i_r, float i_g, float i_b, float i_alpha =1);

    private:
    };


    /*
       * Cloud3DFactory_Coord
       * Factory for generating 3d visualizations of point clouds colored by coord
       * using a color map
       */
    template<typename PointTXYZ = pcl::PointXYZ, typename PointTF = pcl::PointXYZ>
    class Cloud3DFactory_Coord : public Cloud3DFactory{

    public:
        Cloud3DFactory_Coord(std::string i_field="");

        typedef boost::shared_ptr<Cloud3DFactory_Coord<PointTXYZ, PointTF>> Ptr;
        typedef boost::shared_ptr<typename pcl::PointCloud<PointTXYZ>::ConstPtr> CloudConstPtr;

        void setField(std::string i_field);
        void setRange(double i_min, double i_max);
        void setColorTable(const std::vector<osg::Vec4>& i_colorTable);

        virtual osg::Geometry* buildGeometry(bool i_uniqueStateSet=false)  ;
        void setPointSize(int i_size);
        virtual void setInputCloud(const pcl::PCLPointCloud2::ConstPtr& i_cloud);
        using Cloud3DFactory::setInputCloud;

    protected:
        std::string mFieldName;
        double mRangeMin, mRangeMax;
        std::vector<osg::Vec4> mColorTable;
    };


    /*
       * Cloud3DFactory_RGB
       * Factory for generating a osg 3D model of a point cloud with each point colored
       * by the RGB field.
       */
    template<typename PointTXYZ = pcl::PointXYZ, typename RGBT = pcl::RGB>
    class Cloud3DFactory_RGB : public Cloud3DFactory {
    public:
        virtual osg::Geometry* buildGeometry(bool i_uniqueStateSet = false)  ;
        virtual void setInputCloud(const pcl::PCLPointCloud2::ConstPtr& i_cloud);
        using Cloud3DFactory::setInputCloud;

    };

    /*
       * Cloud3DFactory_Intensity
       * Factory for generating a osg 3D model of a point cloud with each point colored
       * by the Intensity field of the point cloud.
       */
    template<typename PointTXYZ, typename IntensityT>
    class Cloud3DFactory_Intensity : public Cloud3DFactory{
    public:
        virtual osg::Geometry* buildGeometry(bool i_uniqueStateSet = false)  ;
        virtual void setInputCloud(const pcl::PCLPointCloud2::ConstPtr& i_cloud);
        using Cloud3DFactory::setInputCloud;
    };


    /*
       * Cloud3DFactory_Label
       * Factory for generating osg 3D models of point clouds with indexed coloring.
       * using a color map
       */
    template<typename PointTXYZ, typename LabelT>
    class Cloud3DFactory_Label : public Cloud3DFactory{

    public:
        Cloud3DFactory_Label();

        virtual osg::Geometry* buildGeometry(bool i_uniqueStateSet = false);

        using Cloud3DFactory::setInputCloud;
        virtual void setInputCloud(const pcl::PCLPointCloud2::ConstPtr& i_cloud);

        /*
         * setColorMap
         * set the color map.
         */
        typedef  std::map<uint32_t, osg::Vec4f> ColorMap;
        void setColorMap(const ColorMap& i_colorMap);

        /*
         * Enable random generation of colors
         */
        void enableRandomColoring(bool i_enableRandomColor);

    private:
        std::map<uint32_t, osg::Vec4f> mColorMap;
        bool mRandomColor;
    };

    class Cloud3DOptions : public osgDB::Options{
    public:
        META_Object(cloud3D::Cloud3DOptions, Cloud3DOptions);
        Cloud3DOptions(float i_samplingRate = 1.F);
        Cloud3DOptions(Cloud3DFactory* i_factory_p, float i_samplingRate = 1.F);
        Cloud3DOptions(const Cloud3DOptions& i_options, const osg::CopyOp& i_copyOp=osg::CopyOp::SHALLOW_COPY);

        /*
          //Get & Set the random sampling rate
          */
        void setSamplingRate(float i_samplingRate){
            mSamplingRte = i_samplingRate;
        }
        float getSamplingRate(){
            return mSamplingRte;
        }

        /*
          //Get & Set the current point cloud node factory
          */
        const osg::ref_ptr<cloud3D::Cloud3DFactory>& getFactory() const
        {
            return mFactory;
        }
        void setFactory(const osg::ref_ptr<cloud3D::Cloud3DFactory>& i_factory_p)
        {
            this->mFactory = i_factory_p;
        }

        //Get & Set the indices of the point cloud to load
        const pcl::IndicesConstPtr& getIndices() const
        {
            return mIndices;
        }
        void setIndices(const pcl::IndicesConstPtr& i_indices)
        {
            this->mIndices = i_indices;
        }

    protected:
        float mSamplingRte;
        pcl::IndicesConstPtr mIndices;
        osg::ref_ptr<cloud3D::Cloud3DFactory> mFactory;
    };
}
#endif /* _POINT_CLOUD_H_ */
