#include <iostream>
#include <stdlib.h>
#include <time.h>

#include <boost/type_traits.hpp>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/point_traits.h>
#include <pcl/common/concatenate.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <osg/Point>
#include <osg/Geode>
#include <osg/Point>
#include <osg/LineWidth>

#include "pcloud/point_cloud.h"


namespace cloud3D
{
    template class Cloud3DFactory_Colored<pcl::PointXYZ>;
    template class Cloud3DFactory_RGB<pcl::PointXYZ, pcl::RGB>;
    template class Cloud3DFactory_Coord<pcl::PointXYZ, pcl::PointXYZ>;
    template class Cloud3DFactory_Label<pcl::PointXYZ, pcl::Label>;
    template class Cloud3DFactory_Intensity<pcl::PointXYZ, pcl::Intensity>;
}

template<typename PointT>
inline typename pcl::PointCloud<PointT>::ConstPtr cloud3D::Cloud3DFactory::getInputCloud () const
{

    std::string key;
    {
        pcl::PointCloud<PointT> cloud;
        key = pcl::getFieldsList(cloud);
    }

    std::map<std::string, boost::any>::const_iterator iter= mInputClouds.find(key);

    if(iter == mInputClouds.end()){
        pcl::console::print_error("Cloud3DFactory trying to retrieve input %s that does not exist\n", key.c_str());
        return typename pcl::PointCloud<PointT>::ConstPtr();
    }
    try{
        return boost::any_cast< typename pcl::PointCloud<PointT>::ConstPtr>(iter->second);
    }
    catch(boost::bad_any_cast& e){
        pcl::console::print_error("Cloud3DFactory Exception: %s\n", e.what());
        return typename pcl::PointCloud<PointT>::ConstPtr();
    }

}

template<typename PointT>
inline void cloud3D::Cloud3DFactory::addXYZToVertexBuffer (osg::Geometry& o_geom,
                                                           const pcl::PointCloud<pcl::PointXYZ>& i_cloud) const
{
    osg::Vec3Array* pts = new osg::Vec3Array();
    pts->reserve(i_cloud.points.size());
    for(std::size_t i=0; i<i_cloud.points.size(); i++){
        const PointT& pt = i_cloud.points[i];
        pts->push_back(osg::Vec3(pt.x, pt.y, pt.z));
    }
    o_geom.setVertexArray(pts);
    o_geom.addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, pts->size()));
}

//////////////////////////////////////////////
//////////  PointCloudColorFactory  //////////
//////////////////////////////////////////////

template<typename PointT>
inline cloud3D::Cloud3DFactory_Colored<PointT>::Cloud3DFactory_Colored ()
{
    const char* vertSource = {
        "#version 120\n"
        "void main(void)\n"
        "{\n"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
        "}\n"
    };
    const char* fragSource = {
        "#version 120\n"
        "uniform vec4 color;\n"
        "void main(void)\n"
        "{\n"
        "    gl_FragColor = color;\n"
        "}\n"
    };

    osg::Program* pgm = new osg::Program();
    pgm->setName("UniformColor");

    pgm->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    pgm->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    mStateSet = new osg::StateSet();
    mStateSet->setAttribute(pgm);

    osg::Point* p = new osg::Point();
    p->setSize(4);

    mStateSet->setAttribute(p);

    osg::Vec4 color;
    color[0]=color[1]=color[2]=color[3]=1;

    osg::Uniform* ucolor(new osg::Uniform("color", color));
    mStateSet->addUniform(ucolor);

    mStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
}


template<typename PointT> osg::Geometry*
cloud3D::Cloud3DFactory_Colored<PointT>::buildGeometry (bool i_uniqueStateSet)
{
    typename pcl::PointCloud<PointT>::ConstPtr cloud = getInputCloud<PointT>();
    if(cloud ==NULL) return NULL;

    osg::Geometry* geom = new osg::Geometry();
    this->addXYZToVertexBuffer<PointT>(*geom, *cloud);

    osg::ref_ptr<osg::StateSet> stateSet;
    if(i_uniqueStateSet){
        stateSet = new osg::StateSet(*mStateSet);
    }
    else{
        stateSet = mStateSet;
    }
    geom->setStateSet(stateSet);

    return geom;
}

template<typename PointT> void
cloud3D::Cloud3DFactory_Colored<PointT>::setInputCloud (
        const pcl::PCLPointCloud2::ConstPtr& i_cloud)
{
    typename pcl::PointCloud<PointT>::Ptr pCloud(new pcl::PointCloud<PointT>());
    pcl::fromPCLPointCloud2(*i_cloud,*pCloud);
    Cloud3DFactory::setInputCloud<PointT>(pCloud);
}

template<typename PointT> void
cloud3D::Cloud3DFactory_Colored<PointT>::setColor (float i_r,float i_g,float i_b,
                                                   float i_alpha)
{
    osg::Vec4 color;
    color[0]=i_r; color[1]=i_g; color[2]=i_b; color[3]=i_alpha;
    mStateSet->getUniform("color")->set(color);
}


//////////////////////////////////////////////
//////////  Cloud3DFactory_RGB  //////////////
//////////////////////////////////////////////

template<typename PointTXYZ, typename RGBT> osg::Geometry*
cloud3D::Cloud3DFactory_RGB<PointTXYZ, RGBT>::buildGeometry (
        bool i_uniqueStateSet)
{
    osg::Geometry* geom(new osg::Geometry());

    typename pcl::PointCloud<PointTXYZ>::ConstPtr pCloud = getInputCloud<PointTXYZ>();
    typename pcl::PointCloud<RGBT>::ConstPtr rgbCloud = getInputCloud<RGBT>();
    if  ((rgbCloud == NULL) || (pCloud==NULL)){
        return NULL;
    }

    if(rgbCloud->points.size() != pCloud->points.size()){
        pcl::console::print_error("[Cloud3DFactory_RGB]  XYZ and Label Clouds have different # of points.\n");
        return NULL;
    }

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->reserve(rgbCloud->points.size());
    int nbPts = rgbCloud->points.size();

    srand(time(NULL));
    for(int i=0; i<nbPts; i++){
        osg::Vec4f c;
        c[0]= (float) rgbCloud->points[i].r/255.0f;
        c[1] = (float)rgbCloud->points[i].g/255.0f;
        c[2] = (float)rgbCloud->points[i].b/255.0f;
        c[3] = 1;
        colors->push_back(c);
    }


    this->addXYZToVertexBuffer<PointTXYZ>(*geom, *pCloud);

    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    if(i_uniqueStateSet){
        geom->setStateSet(new osg::StateSet(*mStateSet));
    }
    else{
        geom->setStateSet(mStateSet);
    }
    return geom;

    return geom;
}


template<typename PointTXYZ , typename RGBT>
inline void cloud3D::Cloud3DFactory_RGB<PointTXYZ, RGBT>::setInputCloud (
        const pcl::PCLPointCloud2::ConstPtr& i_cloud)
{
    typename pcl::PointCloud<PointTXYZ>::Ptr pCloud(new pcl::PointCloud<PointTXYZ>());
    pcl::fromPCLPointCloud2(*i_cloud,*pCloud);
    Cloud3DFactory::setInputCloud<PointTXYZ>(pCloud);

    if(!boost::is_same<PointTXYZ, RGBT>::value){
        typename pcl::PointCloud<RGBT>::Ptr rgbCloud(new pcl::PointCloud<RGBT>());
        pcl::fromPCLPointCloud2(*i_cloud,*rgbCloud);
        Cloud3DFactory::setInputCloud<RGBT>(rgbCloud);
    }
}


//////////////////////////////////////////////
//////////  PointCloudCRange   ///////////////
//////////////////////////////////////////////


template<typename PointTXYZ , typename PointTF > inline
cloud3D::Cloud3DFactory_Coord<PointTXYZ, PointTF>::Cloud3DFactory_Coord (std::string i_field) :
    mRangeMax(-1), mRangeMin(-1), mFieldName(i_field)
{
    mColorTable.push_back(osg::Vec4(1,1,1,1));
    mColorTable.push_back(osg::Vec4(1,0,0,1));
    mStateSet = new osg::StateSet();
    setPointSize(4);
    mStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
}

template<typename PointTXYZ, typename PointTF> void
cloud3D::Cloud3DFactory_Coord<PointTXYZ, PointTF>::setField (
        std::string i_field)
{
    mFieldName =i_field;
}

template<typename PointTXYZ , typename PointTF>
inline void cloud3D::Cloud3DFactory_Coord<PointTXYZ, PointTF>::setRange (
        double i_min, double i_max)
{
    mRangeMin = i_min;
    mRangeMax = i_max;
}

template<typename PointTXYZ, typename PointTF >
inline void cloud3D::Cloud3DFactory_Coord<PointTXYZ, PointTF>::setColorTable (
        const std::vector<osg::Vec4>& i_colorTable)
{
    mColorTable = i_colorTable;
}


template<typename PointTXYZ, typename PointTF> osg::Geometry*
cloud3D::Cloud3DFactory_Coord<PointTXYZ, PointTF>::buildGeometry (
        bool i_uniqueStateSet)
{

    typename pcl::PointCloud<PointTXYZ>::ConstPtr pCloud = getInputCloud<PointTXYZ>();
    typename pcl::PointCloud<PointTF>::ConstPtr fCloud = getInputCloud<PointTF>();
    if  ((fCloud == NULL) || (pCloud==NULL)){
        return NULL;
    }
    double minRange, maxRange;

    std::vector<pcl::PCLPointField> fieldList;
    pcl::getFields<PointTF>(*fCloud, fieldList);

    int idx=-1;

    if(mFieldName.empty()){
        idx=0;
    }
    else {
        for(std::size_t i=0; i< fieldList.size(); i++){
            if(fieldList[i].name==mFieldName){idx=i; break;}
        }
    }

    if(idx <0){
        pcl::console::print_debug("[Cloud3DFactory_Coord] Pointfield (%s)does not exist\n", mFieldName.c_str());
        return NULL;
    }
    int offset =fieldList[idx].offset;

    if(fabs(mRangeMin-mRangeMax) <0.001){
        minRange = std::numeric_limits<double>::infinity();
        maxRange = -std::numeric_limits<double>::infinity();
        for(std::size_t i=0; i< fCloud->points.size(); i++){
            double val = *((float*)(((uint8_t*) &fCloud->points[i])  + offset));
            if(val < minRange) minRange = val;
            if(val > maxRange) maxRange = val;
        }
    }
    else{
        minRange = mRangeMin;
        maxRange = mRangeMax;
    }
    double scale = (mColorTable.size()-1)/(maxRange-minRange);
    int maxIdx= mColorTable.size()-1;

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->resize(fCloud->points.size());
    int nbPts = fCloud->points.size();
    for(int i=0; i< nbPts; i++){
        double val = *((float*)(((uint8_t*) &fCloud->points[i])  + offset));
        double idx = (val-minRange)*scale;
        if(idx <0) idx=0;
        if(idx> maxIdx)  idx =maxIdx;
        double wl = idx-std::floor(idx);
        double wu = 1-wl;
        const osg::Vec4f& lpt = mColorTable[std::floor(idx)];
        const osg::Vec4f& upt = mColorTable[std::ceil(idx)];
        for(int j=0; j<4; j++)(*colors)[i][j] = lpt[j]*wl+ upt[j]*wu;
    }

    osg::Geometry* geom = new osg::Geometry();

    this->addXYZToVertexBuffer<PointTXYZ>(*geom, *pCloud);

    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    if(i_uniqueStateSet){
        geom->setStateSet(new osg::StateSet(*mStateSet));
    }
    else{
        geom->setStateSet(mStateSet);
    }

    return geom;
}

template<typename PointTXYZ , typename PointTF > void
cloud3D::Cloud3DFactory_Coord<PointTXYZ, PointTF>::setPointSize (
        int i_size)
{
    osg::Point* p = new osg::Point();
    p->setSize(i_size);
    mStateSet->setAttribute(p);
}

template<typename PointTXYZ , typename PointTF >
inline void cloud3D::Cloud3DFactory_Coord<PointTXYZ, PointTF>::setInputCloud (
        const pcl::PCLPointCloud2::ConstPtr& i_cloud)
{
    typename  pcl::PointCloud<PointTXYZ>::Ptr pCloud(new pcl::PointCloud<PointTXYZ>());
    pcl::fromPCLPointCloud2(*i_cloud,*pCloud);
    Cloud3DFactory::setInputCloud<PointTXYZ>(pCloud);

    if(!boost::is_same<PointTXYZ, PointTF>::value){
        typename pcl::PointCloud<PointTF>::Ptr fCloud(new pcl::PointCloud<PointTF>());
        pcl::fromPCLPointCloud2(*i_cloud,*fCloud);
        Cloud3DFactory::setInputCloud<PointTF>(fCloud);
    }
}

//////////////////////////////////////////////
//////////  Intensity Point Cloud   //////////
//////////////////////////////////////////////

template<typename PointTXYZ, typename IntensityT>  osg::Geometry*
cloud3D::Cloud3DFactory_Intensity<PointTXYZ, IntensityT>::buildGeometry (bool i_uniqueStateSet)
{
    typename pcl::PointCloud<PointTXYZ>::ConstPtr pCloud = getInputCloud<PointTXYZ>();
    typename pcl::PointCloud<IntensityT>::ConstPtr iCloud = getInputCloud<IntensityT>();
    if  ((iCloud == NULL) || (pCloud==NULL)){
        return NULL;
    }

    if(iCloud->points.size() != pCloud->points.size()){
        pcl::console::print_error("[PointCloudIntensityFactory]  XYZ and Label Clouds have different # of points.\n");
        return NULL;
    }

    //TODO just make this a single grayscale value and make a custom shader program
    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->reserve(iCloud->points.size());
    int nbPts = iCloud->points.size();

    for(int i=0; i<nbPts; i++){
        osg::Vec4f color;
        color[0] =color[1]=color[2] = iCloud->points[i].intensity*0.8+0.2;
        color[3]=1;
        colors->push_back(color);
    }

    osg::Geometry* geom = new osg::Geometry();

    this->addXYZToVertexBuffer<PointTXYZ>(*geom, *pCloud);

    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geom->setStateSet(mStateSet);
    return geom;
}


template<typename PointTXYZ, typename IntensityT>
void cloud3D::Cloud3DFactory_Intensity<PointTXYZ, IntensityT>::setInputCloud (
        const pcl::PCLPointCloud2::ConstPtr& i_cloud)
{
    typename pcl::PointCloud<PointTXYZ>::Ptr pCloud(new pcl::PointCloud<PointTXYZ>());
    pcl::fromPCLPointCloud2(*i_cloud,*pCloud);
    Cloud3DFactory::setInputCloud<PointTXYZ>(pCloud);

    if(!boost::is_same<PointTXYZ, IntensityT>::value){
        typename  pcl::PointCloud<IntensityT>::Ptr iCloud(new pcl::PointCloud<IntensityT>());
        pcl::fromPCLPointCloud2(*i_cloud,*iCloud);
        Cloud3DFactory::setInputCloud<IntensityT>(iCloud);
    }
}


//////////////////////////////////////////////
//////////  Label Point Cloud  ///////////////
//////////////////////////////////////////////

template<typename PointTXYZ, typename LabelT>
inline cloud3D::Cloud3DFactory_Label<PointTXYZ, LabelT>::Cloud3DFactory_Label ()
{
    //Color map
    mColorMap[0] = osg::Vec4f(0.2,0.2,0.2,1);
    mColorMap[1] = osg::Vec4f(1,0,0,1);
    mColorMap[2] = osg::Vec4f(0,1,0,1);
    mColorMap[3] = osg::Vec4f(0,0,1,1);
    mColorMap[4] = osg::Vec4f(1,0,1,1);
    mColorMap[5] = osg::Vec4f(1,1,0,1);
    mColorMap[6] = osg::Vec4f(0.4,0.1,0.1,1);
    mColorMap[7] = osg::Vec4f(0.4,0.4,0.1,1);
    mColorMap[8] = osg::Vec4f(0.4,0.1,0.4,1);
    mColorMap[9] = osg::Vec4f(0.1,0.8,0.1,1);
    mColorMap[10] = osg::Vec4f(0.8,0.1,0.1,1);
    mColorMap[11] = osg::Vec4f(0.8,0.8,0.1,1);
    mColorMap[12] = osg::Vec4f(0.8,0.1,0.8,1);
    mColorMap[13] = osg::Vec4f(0.1,0.8,0.1,1);
    mColorMap[14] = osg::Vec4f(0.6,0.3,0.3,1);
    mColorMap[15] = osg::Vec4f(0.6,0.6,0.3,1);
    mColorMap[16] = osg::Vec4f(0.6,0.3,0.6,1);
    mColorMap[17] = osg::Vec4f(0.3,0.6,0.3,1);

    mRandomColor =true;

    mStateSet = new osg::StateSet();
    mStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    osg::Point* p = new osg::Point();
    p->setSize(4);
    mStateSet->setAttribute(p);
}

template<typename PointTXYZ, typename LabelT>
inline osg::Geometry* cloud3D::Cloud3DFactory_Label<PointTXYZ, LabelT>::buildGeometry (
        bool i_uniqueStateSet)
{

    typename pcl::PointCloud<PointTXYZ>::ConstPtr pCloud = getInputCloud<PointTXYZ>();
    typename pcl::PointCloud<LabelT>::ConstPtr lCloud = getInputCloud<LabelT>();
    if  ((lCloud == NULL) || (pCloud==NULL)){
        return NULL;
    }

    if(lCloud->points.size() != pCloud->points.size()){
        pcl::console::print_error("[Cloud3DFactory_Label]  XYZ and Label Clouds have different # of points.\n");
        return NULL;
    }

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->reserve(lCloud->points.size());
    int nbPts = lCloud->points.size();

    ColorMap& colorMap = mColorMap;
    srand(time(NULL));
    for(int i=0; i<nbPts; i++){
        ColorMap::iterator iter =  colorMap.find(lCloud->points[i].label);
        if(iter ==  colorMap.end()){
            osg::Vec4f color;
            if(mRandomColor){
                for(int i=0; i<3; i++) color[i] = ((float)(rand()%900))/900.0f +0.1;
            }
            else {color = osg::Vec4f(0,0,0,1); }
            color[3]=1;
            colorMap[lCloud->points[i].label] = color;
            colors->push_back(color);
        }
        else{
            colors->push_back(iter->second);
        }
    }

    osg::Geometry* geom = new osg::Geometry();
    this->addXYZToVertexBuffer<PointTXYZ>(*geom, *pCloud);

    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    if(i_uniqueStateSet){
        geom->setStateSet(new osg::StateSet(*mStateSet));
    }
    else{
        geom->setStateSet(mStateSet);
    }
    return geom;
}

template<typename PointTXYZ, typename LabelT>
inline void cloud3D::Cloud3DFactory_Label<PointTXYZ, LabelT>::setInputCloud (
        const pcl::PCLPointCloud2::ConstPtr& i_cloud)
{
    typename pcl::PointCloud<PointTXYZ>::Ptr pCloud(new pcl::PointCloud<PointTXYZ>());
    pcl::fromPCLPointCloud2(*i_cloud,*pCloud);
    Cloud3DFactory::setInputCloud<PointTXYZ>(pCloud);

    if(!boost::is_same<PointTXYZ, LabelT>::value){
        typename  pcl::PointCloud<LabelT>::Ptr iCloud(new pcl::PointCloud<LabelT>());
        pcl::fromPCLPointCloud2(*i_cloud,*iCloud);
        Cloud3DFactory::setInputCloud<LabelT>(iCloud);
    }
}

template<typename PointTXYZ, typename LabelT>
inline void cloud3D::Cloud3DFactory_Label<PointTXYZ, LabelT>::setColorMap (
        const ColorMap& i_colorMap)
{
    mColorMap= i_colorMap;
}

template<typename PointTXYZ, typename LabelT>
inline void cloud3D::Cloud3DFactory_Label<PointTXYZ, LabelT>::enableRandomColoring (
        bool i_enable)
{
    mRandomColor = i_enable;
}


cloud3D::Cloud3DFactory::Cloud3DFactory(){
    mStateSet = new osg::StateSet();
    osg::Point* p = new osg::Point();
    p->setSize(4);
    mStateSet->setAttribute(p);
    mStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
}

void cloud3D::Cloud3DFactory:: setPointSize(int i_size){
    osg::Point* p = new osg::Point();
    p->setSize(i_size);
    mStateSet->setAttribute(p);
}

osg::Node* cloud3D::Cloud3DFactory::buildNode ()
{
    osg::Geode* geode = new osg::Geode();
    geode->getDescriptions().push_back("PointCloud");
    osg::Geometry * geom = buildGeometry();
    if(geom ==NULL) std::cout << "Could not build point cloud\n";
    geode->addDrawable(geom);
    return geode;
}

cloud3D::Cloud3DOptions::Cloud3DOptions(float i_samplingRate):
    mSamplingRte(i_samplingRate)
{
}

cloud3D::Cloud3DOptions::Cloud3DOptions(Cloud3DFactory* i_factory_p,float i_samplingRate):
    mFactory(i_factory_p), mSamplingRte(i_samplingRate)
{
}

cloud3D::Cloud3DOptions::Cloud3DOptions(
        const Cloud3DOptions& i_options, const osg::CopyOp& i_copyOp)
{
    mFactory = i_options.mFactory;
    mIndices = i_options.mIndices;
    mSamplingRte = i_options.mSamplingRte;
}

bool fieldPresent(const std::string& i_name, const std::vector<pcl::PCLPointField>& i_fieldList){
    for(std::size_t i=0; i<i_fieldList.size(); i++){
        if(i_fieldList[i].name == i_name) return true;
    }
    return false;
}

