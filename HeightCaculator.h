#pragma once
#include <osgViewer/Viewer>
#include <osgEarth/Style>
#include <osgEarth/AnnotationUtils>
#include <osgEarth/FeatureNode>
#include <osgEarth/FeatureIndex>
#include <osgEarth/LabelNode>

using namespace osgEarth;
using namespace std;

class HeightCaculator : public osgGA::GUIEventHandler
{
public:
	HeightCaculator(const SpatialReference* srs, osg::Group* annoGroup);
	~HeightCaculator();
};