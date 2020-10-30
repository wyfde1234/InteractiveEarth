#pragma once
#include <osgViewer/Viewer>
#include <osgEarth/Style>
#include <osgEarth/AnnotationUtils>
#include <osgEarth/FeatureNode>
#include <osgEarth/FeatureIndex>
#include <osgEarth/LabelNode>

using namespace osgEarth;
using namespace std;

class DistanceCaculator : public osgGA::GUIEventHandler
{
public:
	DistanceCaculator(const SpatialReference* srs, osg::Group* annoGroup);
	~DistanceCaculator();
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
	string doubleToString(const double &dbNum)
	{
		char *chCode;
		chCode = new char[20];
		sprintf(chCode, "æ‡¿Î£∫%.2lf√◊", dbNum);
		string strCode(chCode);
		delete[] chCode;
		return strCode;
	}

private:
	Style _lineStyle;
	Feature* _pFeature;
	FeatureNode* _pFeatureNode;

	Style _stippleLineStyle;
	Feature* _pStippleFeature;
	FeatureNode* _pStippleFeatureNode;

	Style _labelStyle;
	std::vector<osg::Vec3d> _vecPoint;
	std::vector<LabelNode*> _vecLabel;
	const SpatialReference* _srs;
	osg::Group* _annoGroup;
	bool _isEnable;
};

