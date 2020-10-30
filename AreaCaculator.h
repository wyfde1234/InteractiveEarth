#pragma once
#include <osgViewer/Viewer>
#include <osgEarth/Style>
#include <osgEarth/AnnotationUtils>
#include <osgEarth/FeatureNode>
#include <osgEarth/FeatureIndex>
#include <osgEarth/LabelNode>

using namespace osgEarth;
using namespace std;

class AreaCaculator :
	public osgGA::GUIEventHandler
{
public:
	AreaCaculator(const SpatialReference* srs, osg::Group* annoGroup);
	~AreaCaculator();
	void enable() { _isEnable = true; }
	void disable() { _isEnable = false; }
	bool isEnabled() { return _isEnable; }

protected:
	virtual void slotPicked(osg::Vec3d pos);
	virtual void slotMoving(osg::Vec3d pos);
	virtual void slotRightHandle();
private:
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
	osg::Vec3d getPos(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Vec3d& pos);
	double caculateArea(const Geometry *geo);
	string toString(const long long &dbNum)
	{
		char *chCode;
		chCode = new char[50];
		sprintf(chCode, "面积：%lld平方米", dbNum);
		string strCode(chCode);
		delete[] chCode;
		return strCode;
	}
private:
	Feature* _pFeature;
	FeatureNode* _pFeatureNode;
	std::vector<osg::Vec3d> _vecPoint;
	LabelNode* _label;
	const SpatialReference* _srs;
	osg::Group* _annoGroup;
	bool _isEnable;
};

