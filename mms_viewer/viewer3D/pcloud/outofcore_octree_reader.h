#ifndef _OUTOFCORE_OCTREE_READER_
#define _OUTOFCORE_OCTREE_READER_

#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/bernoulli_distribution.hpp>
#include <pcl/common/io.h>
#include <pcl/outofcore/outofcore.h>
#include <osgDB/ReaderWriter>
#include <osgDB/Options>
#include <osgDB/Registry>

#include "pcloud/point_cloud.h"

namespace cloud3D
{
    /*
      * OOCOctree
      * Untemplated Abstract base class used for querying the templated OutOfCore Octree structure.
      */
    class OOCOctree
    {
    public:
        virtual ~OOCOctree();

        typedef boost::shared_ptr<OOCOctree> Ptr;

        virtual boost::uint64_t getTreeDepth() const = 0;
        virtual void getBoundingBox(double* o_min, double* o_max) = 0;

        virtual  void queryBBIncludes(const double i_min[3], const double i_max[3], size_t i_queryDepth,
        const pcl::PCLPointCloud2::Ptr& o_destCloud) const = 0;

        virtual  void queryBBIncludes_subsample(const double i_min[3], const double i_max[3], size_t i_queryDepth,
        float i_samplingRate, const pcl::PCLPointCloud2::Ptr& o_destCloud) const = 0;
    };

    /*
      * OOCOctreeT
      * Templated wrapper around the standard PCL outofcore octree datastructure
      */
    template<typename PointT>
    class OOCOctreeT : public OOCOctree{

    public:
        typedef pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointT>,PointT> Octree;
        typedef boost::shared_ptr<Octree> OctreePtr;
        typedef boost::shared_ptr<OOCOctreeT<PointT>> Ptr;

        OOCOctreeT(const OctreePtr& i_octree);
        virtual  void queryBBIncludes(const double i_min[3], const double i_max[3], size_t i_queryDepth,
        const pcl::PCLPointCloud2::Ptr& o_destCloud) const;
        virtual  void queryBBIncludes_subsample(const double i_min[3], const double i_max[3], size_t i_queryDepth,
        float i_samplingRate, const pcl::PCLPointCloud2::Ptr& o_destCloud) const;
        virtual boost::uint64_t getTreeDepth() const {return mOctree_p->getDepth();};
        virtual void getBoundingBox(double* o_min, double* o_max){
            Eigen::Vector3d bbMin, bbMax;
            mOctree_p->getBoundingBox(bbMin, bbMax);
            for(int i=0;i<3; i++){o_min[i]=bbMin[i]; o_max[i]=bbMax[i];}
        }

    protected:
        OctreePtr mOctree_p;
        typedef Eigen::Map<const Eigen::Vector3d> ConstVec3dMap;
        typedef Eigen::Map< Eigen::Vector3d>  Vec3dMap;
    };

    /*
      * OOCOctreeReader
      * osgDB::Readwriter implementation for PCL OutOfCore trees (".oct_idx" extension)
      */
    class OOCOctreeReader : public osgDB::ReaderWriter
    {
    public:
        OOCOctreeReader();
        OOCOctreeReader(const OOCOctreeReader& i_readerWriter, const osg::CopyOp& i_copyOp=osg::CopyOp::SHALLOW_COPY);
        virtual ~OOCOctreeReader();

        META_Object(cloud3D,OOCOctreeReader);

        virtual Features supportedFeatures() const;


        /*
           * readNode
           * Recursively loads each octant outofcore octree data structure into a osg::Node
           */
        virtual ReadResult readNode(const std::string& i_fileName, const osgDB::ReaderWriter::Options* i_options_p) const;

        /*
          * OOCOctreeOptions
          * Provide additional options specific to OutOfCore Octrees.  
          */
        class OOCOctreeOptions : public Cloud3DOptions{

        public:
            OOCOctreeOptions(float i_samplingRate = 1.F);
            OOCOctreeOptions(cloud3D::Cloud3DFactory* i_factory_p, float i_samplingRate =1.F);
            OOCOctreeOptions(const OOCOctree::Ptr& i_octree_p, cloud3D::Cloud3DFactory* i_factory_p);
            OOCOctreeOptions(const OOCOctreeOptions& i_options,const osg::CopyOp& i_copyOp=osg::CopyOp::SHALLOW_COPY);

            META_Object(cloud3D::OOCOctreeReader::OOCOctreeOptions, OOCOctreeOptions);

            bool init(const OOCOctree::Ptr& i_octree_p);

            void setDepth(boost::uint64_t i_depth, boost::uint64_t i_maxDepth);
            bool depthIsSet();

            boost::uint64_t getDepth();
            boost::uint64_t getMaxDepth();

            bool isRoot();
            void setRoot(bool i_rootNode);

            void setBoundingBox(const osg::Vec3d& i_bbMin, const osg::Vec3d& i_bbMax);
            void getBoundingBox(osg::Vec3d& o_bbMin, osg::Vec3d& o_bbMax);

            OOCOctree::Ptr getOctree(){return mOctree_p;}
            cloud3D::Cloud3DFactory* getFactory(){return mFactory.get();}
            const osg::Vec3d& getBBmax(){return mBBMax;}
            const osg::Vec3d& getBBmin(){return mBBMin;}
            bool isLeaf(){return mIsLeafNode;}
            void setLeaf(bool i_leafNode){mIsLeafNode = i_leafNode;}

        private:
            bool mDepthSet;
            OOCOctree::Ptr mOctree_p;
            boost::uint64_t mDepth;
            boost::uint64_t mMaxDepth;
            osg::Vec3d mBBMin, mBBMax;
            bool mIsRootNode;
            bool mIsLeafNode;
        };
    };
}

template<typename PointT>
inline cloud3D::OOCOctreeT<PointT>::OOCOctreeT(const OctreePtr& i_octree)
{
    mOctree_p = i_octree;
}

template<typename PointT>
inline void cloud3D::OOCOctreeT<PointT>::queryBBIncludes(
        const double i_min[3], const double i_max[3], size_t i_queryDepth,
const pcl::PCLPointCloud2::Ptr& o_destCloud) const
{
    mOctree_p->queryBBIncludes(ConstVec3dMap(i_min), ConstVec3dMap(i_max), i_queryDepth, o_destCloud);
}

template<typename PointT>
inline void cloud3D::OOCOctreeT<PointT>::queryBBIncludes_subsample(
        const double i_min[3], const double i_max[3], size_t i_queryDepth,
float i_samplingRate, const pcl::PCLPointCloud2::Ptr& o_destCloud) const
{
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    if(i_samplingRate>0.999F){
        std::cout << "Querying " << i_queryDepth << " \n";
        mOctree_p->queryBBIncludes(ConstVec3dMap(i_min), ConstVec3dMap(i_max), i_queryDepth, o_destCloud);
        return;
    }
    else mOctree_p->queryBBIncludes(ConstVec3dMap(i_min), ConstVec3dMap(i_max), i_queryDepth, cloud);

    std::vector<int> indices;
    indices.resize(cloud->width*cloud->height);

    boost::mt19937 rand(std::time(NULL));
    boost::uniform_int <uint64_t> filedist (0, indices.size() - 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<uint64_t>> filedie(rand, filedist);

    for(std::size_t i=0; i< indices.size(); i++){
        indices[i] = filedie();
    }
    pcl::copyPointCloud(*cloud, indices, *o_destCloud);
}

#endif /* _OUTOFCORE_OCTREE_READER_ */
