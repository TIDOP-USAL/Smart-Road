#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>

#include <pcl/outofcore/impl/octree_base.hpp>
#include <pcl/outofcore/impl/octree_disk_container.hpp>
#include <pcl/outofcore/impl/octree_base_node.hpp>
#include <pcl/outofcore/impl/octree_ram_container.hpp>

#include <osg/PagedLOD>
#include <osg/Geode>
#include <osg/ShapeDrawable>

#include "pcloud/outofcore_octree_reader.h"

using namespace boost::assign;
namespace cloud3D
{
    REGISTER_OSGPLUGIN(oct_idx, OOCOctreeReader);


    OOCOctree::~OOCOctree()
    {
    }

    OOCOctreeReader::OOCOctreeReader()
    {
        supportsExtension("oct_idx","PCL OutofCore Octree Format");
    }

    OOCOctreeReader::OOCOctreeReader(const OOCOctreeReader& i_readerWriter, const osg::CopyOp& i_copyOp)
    {
    }

    OOCOctreeReader::~OOCOctreeReader()
    {

    }

    osgDB::ReaderWriter::Features OOCOctreeReader::supportedFeatures() const
    {
        return osgDB::ReaderWriter::FEATURE_READ_NODE;
    }

    void printBB(std::ostream& i_cout, OOCOctreeReader::OOCOctreeOptions& i_opts){
        i_cout << " " << i_opts.getBBmin()[0] <<" " << i_opts.getBBmin()[1] << " " <<  i_opts.getBBmin()[2] << " to  ";
        std::cout <<   i_opts.getBBmax()[0] <<" " << i_opts.getBBmax()[1] << " " << i_opts.getBBmax()[2] << " \n";
    }

    osgDB::ReaderWriter::ReadResult OOCOctreeReader::readNode(const std::string& i_fileName,
                                                              const osgDB::ReaderWriter::Options* i_options_p) const
    {

        boost::filesystem::path filePath(i_fileName);

        if(filePath.extension().string() !=".oct_idx"){
            return ReadResult();
        }

        osg::ref_ptr<OOCOctreeOptions> cOpts = dynamic_cast<OOCOctreeOptions*>(const_cast<osgDB::Options*>(i_options_p));
        if(cOpts  != NULL){
            cOpts = new OOCOctreeOptions(*cOpts, osg::CopyOp::DEEP_COPY_ALL);
        }
        else if(dynamic_cast<Cloud3DOptions*>(const_cast<osgDB::Options*>(i_options_p)) !=NULL){
            Cloud3DOptions* cro = dynamic_cast<Cloud3DOptions*>(const_cast<osgDB::Options*>(i_options_p));
            cOpts = new OOCOctreeOptions(cro->getFactory(), cro->getSamplingRate());
        }
        else{
            cOpts = new OOCOctreeOptions();
        }

        if(cOpts->getOctree() == NULL){
            if(!boost::filesystem::exists(i_fileName))  return osgDB::ReaderWriter::ReadResult::FILE_NOT_FOUND;
            OOCOctreeT<pcl::PointXYZ>::OctreePtr ot (new OOCOctreeT<pcl::PointXYZ>::Octree(i_fileName, true));
            OOCOctreeT<pcl::PointXYZ>::Ptr tree (new OOCOctreeT<pcl::PointXYZ>(ot));
            cOpts->init(tree);
        }

        if(cOpts->getFactory() == NULL){
            cloud3D::Cloud3DFactory_Coord<>* fact = new cloud3D::Cloud3DFactory_Coord<>("z");
            fact->setRange(cOpts->getBBmin()[2], cOpts->getBBmax()[2]);
            cOpts->setFactory(fact);
        }



        const osg::Vec3d & bbMin =cOpts->getBBmin();
        const osg::Vec3d& bbMax =cOpts->getBBmax();
        osg::Vec3d size = cOpts->getBBmax() - cOpts->getBBmin();
        osg::Vec3d childSize = size/2; //size of this octants children
        double radius = childSize.length();

        if(cOpts->isLeaf()){
            pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
            if(cOpts->getSamplingRate() > 0.999){
                cOpts->getOctree()->queryBBIncludes(cOpts->getBBmin()._v, cOpts->getBBmax()._v,cOpts->getDepth(), cloud);
            }
            else{
                cOpts->getOctree()->queryBBIncludes_subsample(cOpts->getBBmin()._v, cOpts->getBBmax()._v,cOpts->getDepth(), cOpts->getSamplingRate(), cloud);
            }
            if(cloud->width*cloud->height == 0) return new osg::Node();
            cOpts->getFactory()->setInputCloud(cloud);
            return cOpts->getFactory()->buildNode();
        }


        //bounding geometries (boxes/spheres) for children
        osg::ref_ptr<osg::PagedLOD> lod = new osg::PagedLOD();
        lod->setCenterMode(osg::LOD::USER_DEFINED_CENTER);
        osg::Vec3d center = (bbMax + bbMin)/2.0f ;
        lod->setCenter(center);
        lod->setRadius(radius);

        std::vector<osg::Vec3d > minBBs;
        minBBs += bbMin, bbMin+ osg::Vec3d(childSize[0],0,0), bbMin+ osg::Vec3d(childSize[0],childSize[1],0),
                bbMin+ osg::Vec3d(0,childSize[1],0), bbMin+ osg::Vec3d(0, childSize[1], childSize[2]),
                bbMin+ osg::Vec3d(childSize[0], childSize[1], childSize[2]),  bbMin+osg::Vec3d(childSize[0], 0 , childSize[2]),
                bbMin+osg::Vec3d(0, 0, childSize[2]);

        float childRad = childSize.length()/2;
        int cDepth = cOpts->getDepth()+1;

        bool buildChildren =true;
        if(cDepth >= cOpts->getMaxDepth()) buildChildren = false;



        if(buildChildren){
            osg::Group* group = new osg::Group();

            //Load the children as LOD actors so that they will be automatically loaded
            //from the disk when the camera is looking at them
            for(int i=0; i<8; i++){
                osg::PagedLOD* clod = new osg::PagedLOD();

                OOCOctreeOptions* childOpts = new OOCOctreeOptions(*cOpts, osg::CopyOp::DEEP_COPY_ALL);

                osg::Vec3d vMax = minBBs[i]+childSize;
                osg::Vec3d ccenter = (vMax+ minBBs[i])/2.0f;
                childOpts->setBoundingBox(minBBs[i],  minBBs[i]+childSize);
                childOpts->setDepth(cDepth, cOpts->getMaxDepth());

                clod->setFileName(0, i_fileName);
                clod->setDatabaseOptions(childOpts);
                clod->setRange(0,0,childRad*3.0f);
                clod->setCenterMode(osg::LOD::USER_DEFINED_CENTER);
                clod->setCenter(ccenter);
                clod->setRadius(radius/2.0f);
                group->addChild(clod);
            }
            //Add the child group to the LOD actor
            if(! lod->addChild(group,0, childRad*2)){
                std::cout << "Failed to add group \n";
            }
        }


        int repID = (buildChildren) ?  1 :0; //place the current nodes visualization in the correct LOD slot
        {
            OOCOctreeOptions* childOpts = new OOCOctreeOptions(*cOpts, osg::CopyOp::DEEP_COPY_ALL);
            childOpts->setLeaf(true);
            lod->setDatabaseOptions(childOpts);
            lod->setFileName(repID, i_fileName);
        }


        if(cOpts->isRoot()){//root node always be visible
            lod->setRange(repID,  0, FLT_MAX);
            cOpts->setRoot(false);
        }
        else{
            lod->setRange(repID, 0, radius*3);
        }
        return lod.get();
    }

    OOCOctreeReader::OOCOctreeOptions::OOCOctreeOptions(float i_samplingRate) :
        Cloud3DOptions(i_samplingRate),
        mIsRootNode(true),mDepth(0), mMaxDepth(0), mDepthSet(false),
        mBBMin(0,0,0),mBBMax(0,0,0), mIsLeafNode(false)
    {
    }

    OOCOctreeReader::OOCOctreeOptions::OOCOctreeOptions(cloud3D::Cloud3DFactory*  i_factory_p, float i_samplingRate):
        Cloud3DOptions(i_factory_p, i_samplingRate),
        mIsRootNode(true),mDepth(0), mMaxDepth(0), mDepthSet(false),
        mBBMin(0,0,0),mBBMax(0,0,0), mIsLeafNode(false)
    {
    }

    OOCOctreeReader::OOCOctreeOptions::OOCOctreeOptions(
            const OOCOctree::Ptr& i_octree_p, cloud3D::Cloud3DFactory* i_factory_p):
        Cloud3DOptions(i_factory_p, 1), mIsRootNode(true),mDepth(0), mMaxDepth(0),
        mDepthSet(false),  mBBMin(0,0,0),mBBMax(0,0,0), mIsLeafNode(false)
    {
        this->init(i_octree_p);
    }

    bool OOCOctreeReader::OOCOctreeOptions::init(const OOCOctree::Ptr& i_octree_p)
    {
        if(!mDepthSet){
            mDepth =0;
            mMaxDepth = i_octree_p->getTreeDepth();
            mDepthSet =true;
        }
        this->mOctree_p = i_octree_p;
        if(mBBMax == mBBMin){
            this->mOctree_p->getBoundingBox(mBBMin._v, mBBMax._v);
        }
        return true;
    }

    void OOCOctreeReader::OOCOctreeOptions::setDepth(boost::uint64_t i_depth,
                                                     boost::uint64_t i_maxDepth)
    {
        mDepthSet = true;
        mDepth =i_depth;
        mMaxDepth = i_maxDepth;
    }

    bool OOCOctreeReader::OOCOctreeOptions::depthIsSet()
    {
        return mDepthSet;
    }

    boost::uint64_t OOCOctreeReader::OOCOctreeOptions::getDepth()
    {
        return mDepth;
    }

    boost::uint64_t OOCOctreeReader::OOCOctreeOptions::getMaxDepth()
    {
        return mMaxDepth;
    }

    bool OOCOctreeReader::OOCOctreeOptions::isRoot()
    {
        return mIsRootNode;
    }

    void OOCOctreeReader::OOCOctreeOptions::setRoot(bool i_rootNode)
    {
        mIsRootNode = i_rootNode;
    }

    void OOCOctreeReader::OOCOctreeOptions::setBoundingBox(
            const osg::Vec3d& i_bbMin, const osg::Vec3d& i_bbMax)
    {
        mBBMin = i_bbMin;
        mBBMax = i_bbMax;
    }

    OOCOctreeReader::OOCOctreeOptions::OOCOctreeOptions(
            const OOCOctreeOptions& i_options, const osg::CopyOp& i_copyOp): Cloud3DOptions(i_options, i_copyOp){
        this->mBBMax = i_options.mBBMax;
        this->mBBMin = i_options.mBBMin;
        this->mIsRootNode = i_options.mIsRootNode;
        this->mDepth = i_options.mDepth;
        this->mMaxDepth = i_options.mMaxDepth;
        this->mOctree_p = i_options.mOctree_p;
        this->mFactory = i_options.mFactory;
        this->mSamplingRte = i_options.mSamplingRte;
        this->mIsLeafNode = i_options.mIsLeafNode;
        this->mDepthSet = i_options.mDepthSet;
    }

    void OOCOctreeReader::OOCOctreeOptions::getBoundingBox(
            osg::Vec3d& i_bbMin, osg::Vec3d& i_bbMax)
    {
        i_bbMin = mBBMin;
        i_bbMax = mBBMax;
    }

}
