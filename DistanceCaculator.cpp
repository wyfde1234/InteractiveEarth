#include "DistanceCaculator.h"

DistanceCaculator::DistanceCaculator(const osgEarth::SpatialReference* srs, osg::Group* annoGroup):_srs(srs), _annoGroup(annoGroup), _isEnable(false)
{
	_pFeature = NULL;
	_pFeatureNode = NULL;
	_pStippleFeature = NULL;
	_pStippleFeatureNode = NULL;

	_vecPoint.clear();
	_vecLabel.clear();

	_lineStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color(Color::White, 0.8);
	_lineStyle.getOrCreate<LineSymbol>()->stroke()->width() = 3.0f;
	_lineStyle.getOrCreate<LineSymbol>()->stroke()->smooth() = true;
	_lineStyle.getOrCreate<LineSymbol>()->tessellationSize()->set(75000, Units::METERS);
	_lineStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
	_lineStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_GPU;

	_stippleLineStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color(Color::White, 0.8);
	_stippleLineStyle.getOrCreate<LineSymbol>()->stroke()->width() = 3.0f;
	_stippleLineStyle.getOrCreate<LineSymbol>()->stroke()->smooth() = true;
	_stippleLineStyle.getOrCreate<LineSymbol>()->tessellationSize()->set(75000, Units::METERS);
	_stippleLineStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
	_stippleLineStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_GPU;
	_stippleLineStyle.getOrCreate<LineSymbol>()->stroke()->stipple() = 255;

	TextSymbol* text = _labelStyle.getOrCreate<TextSymbol>();
	text->declutter() = false;
	text->pixelOffset()->set(0, 20);
	text->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM;
	text->size() = text->size()->eval() + 2.0f;
	text->fill()->color() = Color(Color::Yellow, 0.8);
	//set font to show Chinese
	text->font() = "simhei.ttf";
}

DistanceCaculator::~DistanceCaculator()
{
}

bool DistanceCaculator::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (!_isEnable) return false;
	osg::Vec3d vecPos;
	osg::Vec3d pos = getPos(ea, aa, vecPos);
	switch (ea.getEventType())
	{
		case osgGA::GUIEventAdapter::PUSH:
		{
			if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
			{
				slotPicked(vecPos);
			}
			else if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
			{
				slotRightHandle();
			}
			break;
		}
		case osgGA::GUIEventAdapter::MOVE:
		{
			slotMoving(vecPos);
			break;
		}
	}

	return false;
}

osg::Vec3d DistanceCaculator::getPos(const osgGA::GUIEventAdapter& ea,
	osgGA::GUIActionAdapter& aa, osg::Vec3d& pos)
{
	pos = osg::Vec3d(0, 0, 0);
	osgViewer::Viewer* pViewer = dynamic_cast<osgViewer::Viewer*>(&aa);
	if (pViewer == NULL)
	{
		return osg::Vec3d(0, 0, 0);
	}
	osgUtil::LineSegmentIntersector::Intersections intersection;
	double x = ea.getX();
	double y = ea.getY();
	pViewer->computeIntersections(ea.getX(), ea.getY(), intersection);
	osgUtil::LineSegmentIntersector::Intersections::iterator iter
		= intersection.begin();
	if (iter != intersection.end())
	{
		_srs->getGeodeticSRS()->getEllipsoid()->convertXYZToLatLongHeight(
			iter->getWorldIntersectPoint().x(), iter->getWorldIntersectPoint().y(), iter->getWorldIntersectPoint().z(),
			pos.y(), pos.x(), pos.z());
		pos.x() = osg::RadiansToDegrees(pos.x());
		pos.y() = osg::RadiansToDegrees(pos.y());
		return iter->getWorldIntersectPoint();
	}
	return osg::Vec3d(0, 0, 0);
}

void DistanceCaculator::slotPicked(osg::Vec3d pos)
{
	if (_vecPoint.size() <= 0 && _vecLabel.size() > 0)
	{
		//recaculate, empty old label list
		for each (LabelNode* ln in _vecLabel)
		{
			_annoGroup->removeChild(ln);
		}
		_vecLabel.clear();
	}
	_vecPoint.push_back(pos);

	if (_pFeatureNode == NULL)
	{
		_pFeature = new osgEarth::Feature(new LineString, _srs);
		_pFeatureNode = new osgEarth::FeatureNode(_pFeature, _lineStyle);
		_annoGroup->addChild(_pFeatureNode);
	}

	_pFeature->getGeometry()->clear();
	for (int i = 0; i < _vecPoint.size(); ++i)
	{
		_pFeature->getGeometry()->push_back(_vecPoint[i]);
	}
	_pFeatureNode->dirty();
	if (_pStippleFeatureNode != NULL)
	{
		_pStippleFeature->getGeometry()->clear();
	}

	LabelNode *label = new LabelNode("", _labelStyle);
	label->setDynamic(true);
	label->setHorizonCulling(false);
	_annoGroup->addChild(label);
	_vecLabel.push_back(label);
}

void DistanceCaculator::slotMoving(osg::Vec3d pos)
{
	if (_vecPoint.size() <= 0)
	{
		return;
	}
	if (_pStippleFeatureNode == NULL)
	{
		_pStippleFeature = new osgEarth::Feature(new LineString, _srs);
		_pStippleFeatureNode = new osgEarth::FeatureNode(_pStippleFeature, _stippleLineStyle);
		_annoGroup->addChild(_pStippleFeatureNode);
	}

	_pStippleFeature->getGeometry()->clear();
	osg::Vec3d frontPos = _vecPoint[_vecPoint.size() - 1];
	_pStippleFeature->getGeometry()->push_back(frontPos);
	_pStippleFeature->getGeometry()->push_back(pos);
	_pStippleFeatureNode->dirty();

	double midLat, midLon;
	osgEarth::GeoMath::midpoint(osg::DegreesToRadians(frontPos.y()), osg::DegreesToRadians(frontPos.x()), osg::DegreesToRadians(pos.y()), osg::DegreesToRadians(pos.x()), midLat, midLon);
	LabelNode *label = _vecLabel[_vecLabel.size() - 1];
	label->setPosition(GeoPoint(_srs, osg::RadiansToDegrees(midLon), osg::RadiansToDegrees(midLat), pos.z()));
	label->setText(doubleToString(osgEarth::GeoMath::distance(frontPos, pos, _srs)));
}

void DistanceCaculator::slotRightHandle()
{
	_vecPoint.clear();
	if (_pStippleFeatureNode != NULL)
	{
		_pStippleFeature->getGeometry()->clear();
	}
}