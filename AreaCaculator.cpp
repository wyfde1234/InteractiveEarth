#include "AreaCaculator.h"

AreaCaculator::AreaCaculator(const SpatialReference* srs, osg::Group* annoGroup):_srs(srs), _annoGroup(annoGroup), _isEnable(false)
{
	_pFeature = NULL;
	_pFeatureNode = NULL;
	_label = NULL;
	_vecPoint.clear();

	Style labelStyle;
	TextSymbol* text = labelStyle.getOrCreate<TextSymbol>();
	text->declutter() = false;
	text->pixelOffset()->set(0, 20);
	text->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM;
	text->size() = text->size()->eval() + 2.0f;
	text->fill()->color() = Color(Color::Yellow, 0.8);
	//set font to show Chinese
	text->font() = "simhei.ttf";
	_label = new LabelNode("", labelStyle);
	_label->setDynamic(true);
	_label->setHorizonCulling(false);
	_annoGroup->addChild(_label);
}


AreaCaculator::~AreaCaculator()
{
}

bool AreaCaculator::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
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

osg::Vec3d AreaCaculator::getPos(const osgGA::GUIEventAdapter& ea,
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

void AreaCaculator::slotPicked(osg::Vec3d pos)
{
	_vecPoint.push_back(pos);

	if (_pFeatureNode == NULL)
	{
		Style polygonStyle;
		polygonStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::White, 0.8);
		polygonStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
		polygonStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;

		_pFeature = new osgEarth::Feature(new Polygon, _srs);
		_pFeatureNode = new osgEarth::FeatureNode(_pFeature, polygonStyle);
		_annoGroup->addChild(_pFeatureNode);
	}
	_pFeature->getGeometry()->clear();
	for (int i = 0; i < _vecPoint.size(); ++i)
	{
		_pFeature->getGeometry()->push_back(_vecPoint[i]);
	}
	_pFeatureNode->dirty();
}

void AreaCaculator::slotMoving(osg::Vec3d pos)
{
	if (_vecPoint.size() <= 0)
	{
		return;
	}
	_pFeature->getGeometry()->clear();
	for (int i = 0; i < _vecPoint.size(); ++i)
	{
		_pFeature->getGeometry()->push_back(_vecPoint[i]);
	}
	_pFeature->getGeometry()->push_back(pos);
	_pFeatureNode->dirty();

	const Geometry *geo = _pFeature->getGeometry();
	if (geo->size() > 2)
	{
		_label->setPosition(GeoPoint(_srs, geo->getBounds().center()));
		double area = caculateArea(geo);
		_label->setText(toString(area));
	}
}

void AreaCaculator::slotRightHandle()
{
	_vecPoint.clear();
}

double AreaCaculator::caculateArea(const Geometry *geo)
{
	double dLowX = 0, dLowY = 0, dMiddleX = 0, dMiddleY = 0, dHighX = 0, dHighY = 0;
	double dSum1 = 0, dSum2 = 0, iCount1 = 0, iCount2 = 0;

	for (int i = 0; i < geo->size(); i++)
	{
		if (i == 0)
		{
			dLowX = osg::DegreesToRadians(geo->at(geo->size() - 1).x());
			dLowY = osg::DegreesToRadians(geo->at(geo->size() - 1).y());
			dMiddleX = osg::DegreesToRadians(geo->at(0).x());
			dMiddleY = osg::DegreesToRadians(geo->at(0).y());
			dHighX = osg::DegreesToRadians(geo->at(1).x());
			dHighY = osg::DegreesToRadians(geo->at(1).y());
		}
		else if (i == geo->size() - 1)
		{
			dLowX = osg::DegreesToRadians(geo->at(geo->size() - 2).x());
			dLowY = osg::DegreesToRadians(geo->at(geo->size() - 2).y());
			dMiddleX = osg::DegreesToRadians(geo->at(geo->size() - 1).x());
			dMiddleY = osg::DegreesToRadians(geo->at(geo->size() - 1).y());
			dHighX = osg::DegreesToRadians(geo->at(0).x());
			dHighY = osg::DegreesToRadians(geo->at(0).y());
		}
		else
		{
			dLowX = osg::DegreesToRadians(geo->at(i - 1).x());
			dLowY = osg::DegreesToRadians(geo->at(i - 1).y());
			dMiddleX = osg::DegreesToRadians(geo->at(i).x());
			dMiddleY = osg::DegreesToRadians(geo->at(i).y());
			dHighX = osg::DegreesToRadians(geo->at(i + 1).x());
			dHighY = osg::DegreesToRadians(geo->at(i + 1).y());
		}
		double AM = cos(dMiddleY) * cos(dMiddleX);
		double BM = cos(dMiddleY) * sin(dMiddleX);
		double CM = sin(dMiddleY);
		double AL = cos(dLowY) * cos(dLowX);
		double BL = cos(dLowY) * sin(dLowX);
		double CL = sin(dLowY);
		double AH = cos(dHighY) * cos(dHighX);
		double BH = cos(dHighY) * sin(dHighX);
		double CH = sin(dHighY);

		double dCoefficientL = (AM*AM + BM*BM + CM*CM) / (AM*AL + BM*BL + CM*CL);
		double dCoefficientH = (AM*AM + BM*BM + CM*CM) / (AM*AH + BM*BH + CM*CH);

		double dALtangent = dCoefficientL * AL - AM;
		double dBLtangent = dCoefficientL * BL - BM;
		double dCLtangent = dCoefficientL * CL - CM;
		double dAHtangent = dCoefficientH * AH - AM;
		double dBHtangent = dCoefficientH * BH - BM;
		double dCHtangent = dCoefficientH * CH - CM;

		double dAngleCos = (dAHtangent * dALtangent + dBHtangent * dBLtangent + dCHtangent * dCLtangent) /
			(sqrt(dAHtangent * dAHtangent + dBHtangent * dBHtangent + dCHtangent * dCHtangent) *
				sqrt(dALtangent * dALtangent + dBLtangent * dBLtangent + dCLtangent * dCLtangent));

		dAngleCos = acos(dAngleCos);

		double dANormalLine = dBHtangent * dCLtangent - dCHtangent * dBLtangent;
		double dBNormalLine = 0 - (dAHtangent * dCLtangent - dCHtangent * dALtangent);
		double dCNormalLine = dAHtangent * dBLtangent - dBHtangent * dALtangent;

		double dOrientationValue = 0;
		if (AM != 0)
		{
			dOrientationValue = dANormalLine / AM;
		}
		else if (BM != 0)
		{
			dOrientationValue = dBNormalLine / BM;
		}
		else
		{
			dOrientationValue = dCNormalLine / CM;
		}

		if (dOrientationValue > 0)
		{
			dSum1 += dAngleCos;
			iCount1++;
		}
		else
		{
			dSum2 += dAngleCos;
			iCount2++;
		}
	}
	double dSum = 0;
	if (dSum1 > dSum2)
	{
		dSum = dSum1 + (2 * osg::PI*iCount2 - dSum2);
	}
	else
	{
		dSum = (2 * osg::PI*iCount1 - dSum1) + dSum2;
	}

	double dTotalArea = (dSum - (geo->size() - 2)*osg::PI)* osg::WGS_84_RADIUS_EQUATOR *
		osg::WGS_84_RADIUS_EQUATOR;
	return dTotalArea;
}