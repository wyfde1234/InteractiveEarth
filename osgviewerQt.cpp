/*  -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2010 Robert Osfield

    This application is open source and may be redistributed and/or modified
    freely and without restriction, both in commercial and non commercial applications,
    as long as this copyright notice is maintained.

    This application is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <osgQOpenGL/osgQOpenGLWidget>
#include <QtWidgets/qboxlayout.h>
#include <qmainwindow.h>
#include <qmenubar.h>
#include <qdockwidget.h>

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>
#include <osg/ShapeDrawable>

#include <osg/Switch>
#include <osg/Types>
#include <osgText/Text>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>

#include <osgGA/Device>
#include <osgEarth/GeoMath>
#include <osgEarth/GeoTransform>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Viewpoint>
#include <osgEarth/EarthManipulator>
#include <osgEarth/Controls>
#include <osgEarth/ExampleResources>
#include <osgEarth/LogarithmicDepthBuffer>
#include <osgEarth/ViewFitter>
#include <osgEarth/AnnotationUtils>
#include <osgEarth/LabelNode>
#include <osgEarth/Style>
#include <osgEarth/ScreenSpaceLayout>
#include <osgEarth/ModelNode>
#include <osgEarth/LineDrawable>
#include <osgUtil/Optimizer>
#include <osgEarth/FeatureNode>
#include <osgEarth/FeatureIndex>
#include <osgEarth/IntersectionPicker>
#include <osgEarth/Registry>
#include <osgEarth/RTTPicker>
#include <osgEarth/GLUtils>
#include <osgEarth/GeoPositionNodeAutoScaler>
#include <osgEarth/ElevationQuery>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/OGRFeatureSource>
#include <QApplication>
#include <QSurfaceFormat>
#include <QTextCodec> 
#include <iostream>

#include "DistanceCaculator.h"
#include "AreaCaculator.h"
#include "AngleCaculator.h"

using namespace osgEarth;
#define D2R (osg::PI/180.0)
#define R2D (180.0/osg::PI)

struct PlaneTailPoint : public GeoPositionNode
{
	PlaneTailPoint(MapNode* mapnode, osg::Vec3d pos) : GeoPositionNode(), _mapnode(mapnode)
	{
		setNumChildrenRequiringEventTraversal(1);
		this->getOrCreateStateSet()->setRenderBinDetails(50, "DepthSortedBin");
		setMapNode(mapnode);

		setCullingActive(false);

		//Build the handle
		osg::Geode* geode = new osg::Geode();

		osg::Sphere* shape = new osg::Sphere(osg::Vec3(0, 0, 0), 3);
		_shapeDrawable = new osg::ShapeDrawable(shape);
		_shapeDrawable->setColor(osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
		_shapeDrawable->setDataVariance(osg::Object::DYNAMIC);
		geode->addDrawable(_shapeDrawable);

		getPositionAttitudeTransform()->addChild(geode);

		this->addCullCallback(new GeoPositionNodeAutoScaler());

		setPosition(GeoPoint(_mapnode->getMapSRS(), pos));
	}

	void createLineToEarth(osg::Vec3d pos, osg::Group *annoGroup)
	{
		Geometry* path = new LineString();
		// add this pos twice to create polygon
		path->push_back(pos);
		path->push_back(pos);
		_height = pos.z();

		Feature* pathFeature = new Feature(path, _mapnode->getMapSRS());
		pathFeature->geoInterp() = GEOINTERP_RHUMB_LINE;

		_lineToEarth = new FeatureNode(pathFeature, createStyle());
		_lineToEarth->setNodeMask(false);
		annoGroup->addChild(_lineToEarth);
	}

	Style createStyle()
	{
		Style pathStyle;
		pathStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color(Color::Red, 0.75f);
		pathStyle.getOrCreate<LineSymbol>()->stroke()->width() = 2.0f;
		pathStyle.getOrCreate<LineSymbol>()->stroke()->stipple() = 255;
		pathStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
		pathStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_GPU;
		pathStyle.getOrCreate<ExtrusionSymbol>()->height() = _height;// 250000.0f;
		pathStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Red, 0);
		return pathStyle;
	}

	void traverse(osg::NodeVisitor& nv)
	{
		if (nv.getVisitorType() == osg::NodeVisitor::EVENT_VISITOR)
		{
			osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(&nv);
			for (osgGA::EventQueue::Events::iterator itr = ev->getEvents().begin();
				itr != ev->getEvents().end();
				++itr)
			{
				osgGA::GUIEventAdapter* ea = dynamic_cast<osgGA::GUIEventAdapter*>(itr->get());
				if (ea && handle(*ea, *(ev->getActionAdapter())))
					ea->setHandled(true);
			}
		}
		GeoPositionNode::traverse(nv);
	}

	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		if (ea.getHandled()) return false;

		osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
		if (!view) return false;
		if (!_mapnode) return false;

		if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
		{
			IntersectionPicker picker(view, this);
			IntersectionPicker::Hits hits;

			if (picker.pick(ea.getX(), ea.getY(), hits))
			{
				setHover(true);
			}
			else
			{
				setHover(false);
			}
		}
		return false;
	}

	void setHover(bool hovered)
	{
		if (_hovered != hovered)
		{
			bool wasHovered = _hovered;
			_hovered = hovered;
			if (wasHovered)
			{
				updateColor();
			}
			else
			{
				updateColor();
			}
		}
	}

	void updateColor()
	{
		if (_hovered)
		{
			_shapeDrawable->setColor(osg::Vec4f(1.0f, 1.0f, 0.0f, 1.0f));
			_lineToEarth->setNodeMask(true);
		}
		else
		{
			_shapeDrawable->setColor(osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
			_lineToEarth->setNodeMask(false);
		}
	}

	bool                               _hovered;
	double                             _height;
	FeatureNode*                       _lineToEarth;
	MapNode*                           _mapnode;
	osg::ShapeDrawable*                _shapeDrawable;
};

struct PlaneTail
{
	PlaneTail(MapNode* mapnode, osg::Group* annoGroup) : _mapnode(mapnode), _annoGroup(annoGroup)
	{
		_geode = new osg::Geode();
		annoGroup->addChild(_geode);
	}

	void addTailPoint(osg::Vec3d pos)
	{
		PlaneTailPoint *planeTailPoint = new PlaneTailPoint(_mapnode, pos);
		_annoGroup->addChild(planeTailPoint);
		planeTailPoint->createLineToEarth(pos, _annoGroup);
	}

	void createLine(osg::Vec3d startline, osg::Vec3d endline)
	{
		osg::ref_ptr<osg::Geometry> linesGeom = new osg::Geometry();
		osg::Vec3d startWorld;
		osg::Vec3d endWorld;
		osg::ref_ptr<osg::Vec3dArray> vertices = new osg::Vec3dArray(2);
		GeoPoint(_mapnode->getMapSRS(), startline).toWorld(startWorld);
		GeoPoint(_mapnode->getMapSRS(), endline).toWorld(endWorld);
		(*vertices)[0] = startWorld;
		(*vertices)[1] = endWorld;
		linesGeom->setVertexArray(vertices);
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
		linesGeom->setColorArray(colors);
		linesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

		osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
		normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
		linesGeom->setNormalArray(normals);
		linesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

		linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
		_geode->addDrawable(linesGeom);
	}

	MapNode*                           _mapnode;
	osg::Geode*                        _geode;
	osg::Group*                        _annoGroup;
};

/**
* A simple simulator that moves an object around the Earth. We use this to
* demonstrate/test tethering.
*/
class Simulator : public osgGA::GUIEventHandler
{
public:
	Simulator(EarthManipulator* manip, MapNode* mapnode,
		osg::Group* annoGroup,
		const std::string name)
		: _manip(manip)
	{
		Style style1;
		style1.getOrCreate<ModelSymbol>()->autoScale() = true;
		style1.getOrCreate<ModelSymbol>()->url()->setLiteral("D:/3d_map/modules_x64/osgearth_build_release/bin/Release/data/su27.IVE");
		_model = new ModelNode(mapnode, style1);

		osg::Group *attachPoint = new osg::Group();
		attachPoint->addChild(_model);
		//offset model toward center point
		//_model->setLocalOffset(osg::Vec3d(0, -50000, 0));

		Style style;
		TextSymbol* text = style.getOrCreate<TextSymbol>();
		text->declutter() = false;
		text->pixelOffset()->set(0, 20);
		text->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM;
		text->size() = text->size()->eval() + 2.0f;
		//set font to show Chinese
		text->font() = "simhei.ttf";

		_label = new LabelNode(name, style);
		_label->setDynamic(true);
		_label->setHorizonCulling(false);
		attachPoint->addChild(_label);

		_geo = new GeoPositionNode();
		_geo->getPositionAttitudeTransform()->addChild(attachPoint);

		mapnode->addChild(_geo.get());

		// plane tail
		_planeTail = new PlaneTail(mapnode, annoGroup);
		_dataCount = 0;
		_dataNum = loadPositions();



		//TODO: delete test
		_geo->setPosition(GeoPoint(_model->getMapNode()->getMapSRS(), 117.43926116579506, 31.959264570876737, 500));

	}

	const GeoPoint& getPosition() const { return _geo->getPosition(); }

private:
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		if (ea.getEventType() == ea.FRAME)
		{
			//TODO: delete test
			double t = fmod(osg::Timer::instance()->time_s(), 600) / 600;
			GeoPoint start(_model->getMapNode()->getMapSRS(), 117.43926116579506, 31.959264570876737, 500);
			GeoPoint end(_model->getMapNode()->getMapSRS(), 117.00290028126817, 31.70746267637601, 500);
			GeoPoint r = start.interpolate(end, t);
			GeoPoint l = _geo->getPosition();
			//_planeTail->createLine(l.vec3d(), r.vec3d());
			setPlaneMatrix(start.y(), start.x(), end.y(), end.x());
			_planeTail->addTailPoint(r.vec3d());
			_geo->setPosition(r);

			//TODO: revert this code
			//if (_dataNum > 0)
			//{
			//	handlePosition();
			//}
		}
		/*else if (ea.getEventType() == ea.KEYDOWN)
		{
			if (ea.getKey() == _key)
			{
				Viewpoint vp = _manip->getViewpoint();
				vp.setNode(_label);
				vp.range()->set(1000.0f, Units::METERS);
				vp.pitch()->set(0, Units::DEGREES);
				_manip->setViewpoint(vp, 2.0);
			}
			return true;
		}*/
		else if (ea.getEventType() == ea.RELEASE)
		{
			osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
			if (!view) return false;

			IntersectionPicker picker(view, _geo);
			IntersectionPicker::Hits hits;

			if (picker.pick(ea.getX(), ea.getY(), hits))
			{
				//Style style1;
				//style1.getOrCreate<ModelSymbol>()->autoScale() = true;
				//style1.getOrCreate<ModelSymbol>()->url()->setLiteral("D:/3d_map/modules_x64/osgearth_build_release/bin/Release/data/su27_hl.IVE");
				//_model->setStyle(style1);

				Viewpoint vp = _manip->getViewpoint();
				vp.setNode(_label);
				vp.range()->set(1000.0f, Units::METERS);
				vp.pitch()->set(0, Units::DEGREES);
				_manip->setViewpoint(vp, 2.0);
			}
			else
			{
				//Style style2;
				//style2.getOrCreate<ModelSymbol>()->autoScale() = true;
				//style2.getOrCreate<ModelSymbol>()->url()->setLiteral("D:/3d_map/modules_x64/osgearth_build_release/bin/Release/data/su27.IVE");
				//_model->setStyle(style2);
			}
		}

		return false;
	}

	//focalpoint:照相机视线方向与地面的交点 。类型osg::Vec3d ，分别是经度，纬度以及高度值
	//heading：相机姿态（航向角），人站在地面上，正北为0，向东为正，向西为负
	//pitch：相机姿态（倾斜角），人站在地面上，平视0，向上昂为正，向下为负
	//range：相机到交点的直线距离
	void setPlaneMatrix(float latStart, float lonStart, float latEnd, float lonEnd)
	{
		double bearing = GeoMath::bearing(D2R*latStart, D2R*lonStart, D2R*latEnd, D2R*lonEnd);
		double pp = bearing * R2D;
		_geo->setLocalRotation(osg::Quat(-bearing, osg::Vec3d(0, 0, 1)));
	}

	int loadPositions()
	{
		FILE *trackfile = fopen("D:/3d_map/modules_x64/osgearth_build_release/bin/Release/data/trackdata.txt", "r");
		int dataNum = 0;
		float tmp;
		while (EOF != fscanf(trackfile, "%f", &tmp))
		{
			dataNum++;
		}

		if (dataNum % 7 == 0)
		{
			dataNum = dataNum / 7;
		}
		else
		{
			return 0;
		}
		fseek(trackfile, 0, SEEK_SET);
		for (int i = 0; i < dataNum; i++)
		{
			fscanf(trackfile, "%f%f%f%f%f%f%f", _longArray + i, _latArray + i, _hiArray + i, _heArray + i, _pArray + i, _rArray + i, _vArray + i);
		}
		fclose(trackfile);
		return dataNum;
	}

	void handlePosition()
	{
		if (_dataCount > 0)
		{
			osg::Vec3d startline(_longArray[_dataCount - 1], _latArray[_dataCount - 1], _hiArray[_dataCount - 1]);
			osg::Vec3d endline(_longArray[_dataCount], _latArray[_dataCount], _hiArray[_dataCount]);
			_planeTail->createLine(startline, endline);

			setPlaneMatrix(_latArray[_dataCount], _longArray[_dataCount], _latArray[_dataCount - 1], _longArray[_dataCount - 1]);
			_planeTail->addTailPoint(endline);
		}
		_geo->setPosition(GeoPoint(_model->getMapNode()->getMapSRS(), _longArray[_dataCount], _latArray[_dataCount], _hiArray[_dataCount]));
		_dataCount++;
		if (_dataCount == _dataNum)
		{
			osg::Vec3d startline(_longArray[_dataCount - 1], _latArray[_dataCount - 1], _hiArray[_dataCount - 1]);
			osg::Vec3d endline(_longArray[_dataCount - 1], _latArray[_dataCount - 1], 0);
			_planeTail->createLine(startline, endline);
			_geo->setPosition(GeoPoint(_model->getMapNode()->getMapSRS(), _longArray[_dataCount - 1], _latArray[_dataCount - 1], 0));
			_dataCount = 0;
		}
	}

private:
	PlaneTail*                         _planeTail;
	ModelNode*                         _model;
	//char                               _key;
	EarthManipulator*                  _manip;
	LabelNode*                         _label;
	osg::ref_ptr<GeoPositionNode>      _geo;
	int                                _dataNum;
	int                                _dataCount;
	float                              _hiArray[10000];
	float                              _longArray[10000];
	float                              _latArray[10000];
	float                              _heArray[10000];
	float                              _pArray[10000];
	float                              _rArray[10000];
	float                              _vArray[10000];
};

EarthManipulator* manip = NULL;
MapNode* mapNode = NULL;
osg::Group* annoGroup;
FeatureModelLayer* buildingLayer = NULL;
DistanceCaculator* distanceCal = NULL;
AreaCaculator* areaCal = NULL;
AngleCaculator* angleCal = NULL;

#define BUILDINGS_URL    "D:/3d_map/modules_x64/osgearth_build_release/bin/Release/data/cityBuildingData/hefei/hf.shp" 
#define RESOURCE_LIB_URL "D:/3d_map/modules_x64/osgearth_build_release/bin/Release/data/resource_data/resources/textures_us/catalog.xml"

FeatureModelLayer* createBuildings()
{
	// create a feature source to load the building footprint shapefile.
	OGRFeatureSource* data = new OGRFeatureSource();
	data->setName("buildings-data");
	data->setURL(BUILDINGS_URL);

	// a style for the building data:
	Style buildingStyle;
	buildingStyle.setName("default");

	// Extrude the shapes into 3D buildings.
	ExtrusionSymbol* extrusion = buildingStyle.getOrCreate<ExtrusionSymbol>();
	extrusion->heightExpression() = NumericExpression("10 * max([floor], 1)");
	extrusion->flatten() = true;
	extrusion->wallStyleName() = "building-wall";
	extrusion->roofStyleName() = "building-roof";

	PolygonSymbol* poly = buildingStyle.getOrCreate<PolygonSymbol>();
	poly->fill()->color() = Color::White;

	// Clamp the buildings to the terrain.
	AltitudeSymbol* alt = buildingStyle.getOrCreate<AltitudeSymbol>();
	alt->clamping() = alt->CLAMP_TO_TERRAIN;
	alt->binding() = alt->BINDING_VERTEX;

	// a style for the wall textures:
	Style wallStyle;
	wallStyle.setName("building-wall");
	SkinSymbol* wallSkin = wallStyle.getOrCreate<SkinSymbol>();
	wallSkin->library() = "us_resources";
	wallSkin->addTag("building");
	wallSkin->randomSeed() = 1;

	// a style for the rooftop textures:
	Style roofStyle;
	roofStyle.setName("building-roof");
	SkinSymbol* roofSkin = roofStyle.getOrCreate<SkinSymbol>();
	roofSkin->library() = "us_resources";
	roofSkin->addTag("rooftop");
	roofSkin->randomSeed() = 1;
	roofSkin->isTiled() = true;

	// assemble a stylesheet and add our styles to it:
	StyleSheet* styleSheet = new StyleSheet();
	styleSheet->addStyle(buildingStyle);
	styleSheet->addStyle(wallStyle);
	styleSheet->addStyle(roofStyle);

	// load a resource library that contains the building textures.
	ResourceLibrary* reslib = new ResourceLibrary("us_resources", RESOURCE_LIB_URL);
	styleSheet->addResourceLibrary(reslib);

	// set up a paging layout for incremental loading. The tile size factor and
	// the visibility range combine to determine the tile size, such that
	// tile radius = max range / tile size factor.
	FeatureDisplayLayout layout;
	layout.tileSize() = 500;

	FeatureModelLayer* buildingsLayer = new FeatureModelLayer();
	buildingsLayer->setName("Buildings");
	buildingsLayer->setFeatureSource(data);
	buildingsLayer->setStyleSheet(styleSheet);
	buildingsLayer->setLayout(layout);
	buildingsLayer->setMaxVisibleRange(20000.0);
	return buildingsLayer;
}

void initOsg(osgQOpenGLWidget &widget)
{
	osgViewer::Viewer *viewer = widget.getOsgViewer();

	// load the data
	osg::ref_ptr<osg::Node> earthNode = osgDB::readRefNodeFile("D:/3d_map/modules_x64/osgearth_build_release/bin/Release/data/earth_data/earth_ah.earth");

	manip = new EarthManipulator();
	viewer->setCameraManipulator(manip);

	osg::Group* root = new osg::Group();

	osgUtil::Optimizer optimizer;
	optimizer.optimize(earthNode);
	optimizer.optimize(root);
	optimizer.reset();

	root->addChild(earthNode);

	mapNode = MapNode::findMapNode(earthNode);
	annoGroup = new osg::Group();
	mapNode->addChild(annoGroup);

	viewer->setSceneData(root);

	buildingLayer = createBuildings();
	mapNode->getMap()->addLayer(buildingLayer);
	buildingLayer->setVisible(false);

	manip->getSettings()->getBreakTetherActions().push_back(EarthManipulator::ACTION_GOTO);
	//tether to the node and refresh viewpoint automatically
	manip->getSettings()->setTetherMode(EarthManipulator::TetherMode::TETHER_CENTER_AND_HEADING);
	// Set the minimum distance to something larger than the default
	manip->getSettings()->setMinMaxDistance(10.0, manip->getSettings()->getMaxDistance());

	// Sets the maximum focal point offsets (usually for tethering)
	manip->getSettings()->setMaxOffset(5000.0, 5000.0);
	manip->getSettings()->setArcViewpointTransitions(true);
	viewer->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
	LogarithmicDepthBuffer buf;
	buf.install(viewer->getCamera());

	double hours = mapNode->externalConfig().child("sky").value("hours", 5.0);
	osg::ref_ptr<osgEarth::Util::SkyNode> sky = osgEarth::SkyNode::create();
	sky->setDateTime(osgEarth::DateTime(2020, 3, 6, hours));
	sky->attach(viewer);
	root->addChild(sky);

	viewer->realize();
	manip->setViewpoint(osgEarth::Util::Viewpoint("", 117.43926116579506, 31.959264570876737, 500, 0.0f, -90.0f, 1200000), 0.1f);

	distanceCal = new DistanceCaculator(mapNode->getMapSRS(), annoGroup);
	viewer->addEventHandler(distanceCal);

	areaCal = new AreaCaculator(mapNode->getMapSRS(), annoGroup);
	viewer->addEventHandler(areaCal);

	angleCal = new AngleCaculator(mapNode->getMapSRS(), annoGroup);
	viewer->addEventHandler(angleCal);
}

void addPlane(osgQOpenGLWidget &widget)
{
	osgViewer::Viewer *viewer = widget.getOsgViewer();
	Simulator* sim1 = new Simulator(manip, mapNode, annoGroup, "飞机1");
	viewer->addEventHandler(sim1);
}

int main( int argc, char** argv )
{
	osgEarth::initialize();

    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
	QTextCodec *codec = QTextCodec::codecForLocale();
#ifdef OSG_GL3_AVAILABLE
    format.setVersion(3, 2);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setOption(QSurfaceFormat::DebugContext);
#else
    format.setVersion(2, 0);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setOption(QSurfaceFormat::DebugContext);
#endif
    format.setDepthBufferSize(24);
    //format.setAlphaBufferSize(8);
    format.setSamples(8);
    format.setStencilBufferSize(8);
    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication app(argc, argv);

    osgQOpenGLWidget widget;
	widget.setMouseTracking(true);
	QMainWindow mainWin;
	mainWin.setFixedWidth(1000);
	mainWin.setFixedHeight(800);

	QMenuBar *menuBar = mainWin.menuBar();
	QAction *actionHome = menuBar->addAction(codec->toUnicode("主视角"));
	QObject::connect(actionHome, &QAction::triggered, [=] { manip->setViewpoint(osgEarth::Util::Viewpoint("", 117.43926116579506, 31.959264570876737, 500, 0.0f, -90.0f, 1200000), 0.1f); });

	QMenu *menu1 = menuBar->addMenu(codec->toUnicode("飞机模型"));
	QAction *action11 = menu1->addAction(codec->toUnicode("显示"));
	QObject::connect(action11, &QAction::triggered, [&widget] { addPlane(widget); });

	QMenu *menu2 = menuBar->addMenu(codec->toUnicode("城市房屋模型"));
	QAction *action21 = menu2->addAction(codec->toUnicode("显示"));
	action21->setCheckable(true);
	QObject::connect(action21, &QAction::triggered, [=] { 
		if (action21->isChecked())
		{
			buildingLayer->setVisible(true);
		}
		else 
		{
			buildingLayer->setVisible(false);
		}
	});

	QMenu *menu3 = menuBar->addMenu(codec->toUnicode("测量"));
	QAction *action31 = menu3->addAction(codec->toUnicode("距离测量"));
	action31->setCheckable(true);
	QAction *action32 = menu3->addAction(codec->toUnicode("面积测量"));
	action32->setCheckable(true);
	QAction *action33 = menu3->addAction(codec->toUnicode("角度测量"));
	action33->setCheckable(true);

	QObject::connect(action31, &QAction::triggered, [=] { 
		action32->setChecked(false);
		areaCal->disable();
		action33->setChecked(false);
		angleCal->disable();
		if (action31->isChecked())
		{
			distanceCal->enable();
		}
		else
		{
			distanceCal->disable();
		}
	});

	QObject::connect(action32, &QAction::triggered, [=] { 
		action31->setChecked(false);
		distanceCal->disable();
		action33->setChecked(false);
		angleCal->disable();
		if (action32->isChecked())
		{
			areaCal->enable();
		}
		else
		{
			areaCal->disable();
		}
	});

	QObject::connect(action33, &QAction::triggered, [=] {
		action31->setChecked(false);
		distanceCal->disable();
		action32->setChecked(false);
		areaCal->disable();
		if (action33->isChecked())
		{
			angleCal->enable();
		}
		else
		{
			angleCal->disable();
		}
	});
	
	mainWin.setCentralWidget(&widget);
	QObject::connect(&widget, &osgQOpenGLWidget::initialized, [&widget] { initOsg(widget); });

	mainWin.show();
    return app.exec();

}
