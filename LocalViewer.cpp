#include "LocalViewer.h"
#include <osgDB/ReadFile>
#include "ScreenQuad.h"
#include <osg/PolygonMode>
#include <osg/MatrixTransform>
#include <osg/CullFace>
#include <osg/PolygonOffset>
#include <osg/StateSet>
#include <osg/LineWidth>
#include <osg/Array>
#include "DepthCallback.h"
#include <ctime>
#include <cstdlib>
#include <osgUtil/Optimizer>
//#include <osgPPU/UnitOut.h>
//#include <osgPPU/UnitTexture.h> 
//#include <osgPPU/UnitCamera.h>
//#include <osgPPU/UnitCameraAttachmentBypass.h>
#include "WarpingEventHandler.h"
//#include <osgDB/WriteFile>
#define  SHOW
#define  MODEL_ARMADILLO
//#define  ISMSDEPTH

#define ISTOG05 //选择显著度模型

#define ISSALIENCYREGION   //是否扩展显著度区域 方案1
const float regionThreshold=0.2;

//----------------------------------
//#define ISSALIENCYSAMPLE   //是否扩展显著度全采样 方案2
const float threshold=0.2;


#define ISEDGESAMPLE        //是否扩展边缘采样   方案3
//---------------------------------

const int R_edgeRegion=1;//边缘扩散区域半径

osg::Vec4f lightP=osg::Vec4(0,0,-1,0);  //David
//osg::Vec4f lightP=osg::Vec4(0,1,0,0);    //armdillo
//osg::Vec4f lightP=osg::Vec4(0,-1,0,0);  //bunny
//osg::Vec4f lightP=osg::Vec4(-1,0,0,0);  //dragon
//osg::Vec4f lightP=osg::Vec4(0,0,-1,0);  //maxplank
//osg::Vec4f lightP=osg::Vec4(0,-1,0,0); //gargoyle
//osg::Vec4f lightP=osg::Vec4(0,-1,0,0); //teeth

//osg::Vec4f lightP=osg::Vec4(1,0,0,0);  //elephant
//osg::Vec4f lightP=osg::Vec4(0,0,-1,0);    //xyzrgb_dragon
//osg::Vec4f lightP=osg::Vec4(0,0,-1,0);    //happy

//osg::Vec4f lightP=osg::Vec4(0,0,1,0);//buddha
//osg::Vec4f lightP=osg::Vec4(0,1,0,0);//bimba

//osg::Vec4f lightP=osg::Vec4(0,-1,0,0);//angel

//osg::Vec4f lightP=osg::Vec4(0,0,-1,0);//igea

//const std::string model_ms="gargoyle_ms.ply";
//const std::string modelply="gargoyle.ply";

//string model_ms="teeth_ms.ply";
//string modelply="teeth.ply";

//string model_ms="elephant_ms.ply";
//string modelply="elephant.ply";

//string modelply="xyzrgb_dragon.ply";
//string model_ms="xyzrgb_dragon_ms.ply";

//string modelply="happy.ply";
//string model_ms="happy_ms.ply";

//string modelply="buddha.ply";
//string model_ms="buddha_ms.ply";


//string modelply="bimba.ply";
//string model_ms="bimba_ms.ply";

//string modelply="angel.ply";
//string model_ms="angel_ms.ply";

//string modelply="amphora.ply";
//string model_ms="amphora_ms.ply";

//string modelply="igea.ply";
//string model_ms="igea_ms.ply";
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////


const osg::Vec4f  BACKCOLOR = osg::Vec4(0.153f, 0.659, 0.843, 1.0f);
const osg::Vec4f  BC2 = osg::Vec4f(228.0 / 255.0,228.0 / 255.0, 218.0 / 255.0, 1.0);
const osg::Vec4f  WHITE = osg::Vec4f(1.0, 1.0, 1.0, 1.0);
const osg::Vec4f  BLACK=osg::Vec4f(0,0,0,0);
const osg::Vec4f  CL1 = osg::Vec4f(139.0 / 255.0, 139.0 / 255.0, 139.0 / 255.0, 1.0);
const osg::Vec4f  GREYCOLOR=osg::Vec4f(75.0/256,87.0/256,99.0/256,1.0);//论文背景色

//2014/3/11
const int baseorder=120;
const int MAXLEVEL=10;

//const osg::Vec4f  BACKCOLOR = osg::Vec4(0.2f, 0.2f, 0.4f, 1.0f);
std::string LocalViewer::DATAPATH = "E:\\3PLib\\OpenSceneGraph-3.0.1\\OpenSceneGraph-Data-3.0.0\\";
//04年微软深度图像加载
//std::string LocalViewer::MSDATAPATH ="F:\\MultiviewDepthData\\3DVideos-distrib\\MSR3DVideo-Ballet\\cam0";
std::string LocalViewer::MSDATAPATH ="F:\\MultiviewDepthData\\3DVideos-distrib\\MSR3DVideo-Breakdancers\\cam0";
const std::string  SUBPATH = "shiyanshuju\\OBJ\\798\\";
const int MODELCNT = 20;

LocalViewer::LocalViewer(int width, int height)
	:_width(width),
	_height(height),
	_DsFactorX(2),
	_DsFactorY(2),
#ifdef MODEL_ARMADILLO
	//mNear(10),
	//mFar(50000)
	//和尚(1,100)
	//mNear(1),
	//mFar(100)
	//head
	//发动机
	//mNear(10),
	//mFar(50000)
	//scale 穿山甲
	mNear(10),
	mFar(500)

#else
	mNear(20000),//10000
	mFar(1800000)//200000
#endif
{

#ifdef SHOW
	mIsShow = true;
#else
	mIsShow  = false;
#endif
	//setLightingMode(osg::View::NO_LIGHT);
	createMasterCamera();
	scenePrepare();
	createShowCamera();///2014/3/23边缘显示
	createWarpingCamera();
	//createVisiblePointCamera();
	//createPoissionSample();
	
	createShowPoissonsampleCam();
	createShowPoissonsampleCamAnother();///实验视频Cam
	createPSNRCam();//计算信噪比
	

	/*createShowSampleCam1();//实验截图
	createShowSampleCam2();//实验截图
	createShowSampleCam3();//实验截图
	createShowSampleCam4();//实验截图
	createShowSampleCam5();//实验截图
	createshowSmooth5();
	createshowSmooth4();
	createshowSmooth3();
	createshowSmooth2();
	createshowSmooth1();
	createshowInterpolation5();
	createshowInterpolation4();
	createshowInterpolation3();
	createshowInterpolation2();
	createshowInterpolation1();//实验截图
	createPostDeal();*/
	//createPoissoncamera();
	//createLowToHighCamera();

	//普适性证明
	createMSRDepthImage();
	createCmpTextShow(root);
	createPoissonTextShow(root);
	this->addEventHandler(new WarpingEventHandler);
}
void LocalViewer::createTextShow(osg::Group* root)
{
	//
	
	font = osgText::readFontFile("fonts/arial.ttf");

	osg::Geode* geode  = new osg::Geode;

	float windowHeight = _width;
	float windowWidth = _height;
	float margin = 50.0f;

	osg::Vec4 layoutColor(1.0f,1.0f,0.0f,1.0f);
	float layoutCharacterSize = 20.0f;    

	{
		text = new osgText::Text;
		text->setFont(font);
		text->setColor(layoutColor);
		text->setCharacterSize(layoutCharacterSize);
		text->setPosition(osg::Vec3(margin,margin,0.0f));

		// the default layout is left to right, typically used in languages
		// originating from europe such as English, French, German, Spanish etc..
		text->setLayout(osgText::Text::LEFT_TO_RIGHT);

		text->setText("");
		geode->addDrawable(text);
	}
	camera = new osg::Camera;
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	camera->setProjectionMatrixAsOrtho2D(0,_width,0,_height);
	camera->setViewMatrix(osg::Matrix::identity());
	camera->setClearMask(GL_DEPTH_BUFFER_BIT);
	camera->addChild(geode);
	camera->setRenderOrder(osg::Camera::POST_RENDER,900000);
	camera->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

	root->addChild(camera);
}
void LocalViewer::createCmpTextShow(osg::Group* root)
{

	_cmpfont = osgText::readFontFile("fonts/arial.ttf");

	osg::Geode* geode  = new osg::Geode;

	float windowHeight = _width;
	float windowWidth = _height;
	float margin = 50.0f;

	osg::Vec4 layoutColor(1.0f,1.0f,0.0f,1.0f);
	float layoutCharacterSize = 20.0f;    

	{
		_cmptext = new osgText::Text;
		_cmptext->setFont(_cmpfont);
		_cmptext->setColor(layoutColor);
		_cmptext->setCharacterSize(layoutCharacterSize);
		_cmptext->setPosition(osg::Vec3(margin,margin,0.0f));

		// the default layout is left to right, typically used in languages
		// originating from europe such as English, French, German, Spanish etc..
		_cmptext->setLayout(osgText::Text::LEFT_TO_RIGHT);

		_cmptext->setText("");
		geode->addDrawable(_cmptext);
	}
	_cmptextcam= new osg::Camera;
	_cmptextcam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	_cmptextcam->setProjectionMatrixAsOrtho2D(0,_width,0,_height);
	_cmptextcam->setViewMatrix(osg::Matrix::identity());
	_cmptextcam->setClearMask(GL_DEPTH_BUFFER_BIT);
	_cmptextcam->addChild(geode);
	_cmptextcam->setRenderOrder(osg::Camera::POST_RENDER,900000);
	_cmptextcam->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

	//root->addChild(_cmptextcam);
	_warpingCmpCam->addChild(_cmptextcam);


}
void LocalViewer::createPoissonTextShow(osg::Group* root)
{

	_poissonfont = osgText::readFontFile("fonts/arial.ttf");

	osg::Geode* geode  = new osg::Geode;

	float windowHeight = _width;
	float windowWidth = _height;
	float margin = 50.0f;

	osg::Vec4 layoutColor(1.0f,1.0f,0.0f,1.0f);
	float layoutCharacterSize = 20.0f;    

	{
		_poissontext = new osgText::Text;
		_poissontext->setFont(_poissonfont);
		_poissontext->setColor(layoutColor);
		_poissontext->setCharacterSize(layoutCharacterSize);
		_poissontext->setPosition(osg::Vec3(margin,margin,0.0f));

		// the default layout is left to right, typically used in languages
		// originating from europe such as English, French, German, Spanish etc..
		_poissontext->setLayout(osgText::Text::LEFT_TO_RIGHT);

		_poissontext->setText("");
		geode->addDrawable(_poissontext);
	}
	_poissontextcam = new osg::Camera;
	_poissontextcam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	_poissontextcam->setProjectionMatrixAsOrtho2D(0,_width,0,_height);
	_poissontextcam->setViewMatrix(osg::Matrix::identity());
	_poissontextcam->setClearMask(GL_DEPTH_BUFFER_BIT);
	_poissontextcam->addChild(geode);
	_poissontextcam->setRenderOrder(osg::Camera::POST_RENDER,900000);
	_poissontextcam->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

	//root->addChild(_poissontextcam);
	_warpingpoissoncam->addChild(_poissontextcam);

}
void LocalViewer::createMSRDepth()
{
	_msdepth=osgDB::readImageFile(MSDATAPATH+"\\depth-cam0-f000.png");
	osgDB::writeImageFile(*(_msdepth.get()),"msdepth.bmp");
}
void LocalViewer::createMSRDepthImage()
{
	_msdepthImg=new osg::Image;
	_msdepthImg->allocateImage(_width,_height,1,GL_RGB,GL_UNSIGNED_BYTE);
	//_msdepthImg->setInternalTextureFormat(GL_RGB);

	osg::ref_ptr<osg::Texture2D> _depthTex = new osg::Texture2D;
	_depthTex->setImage(_ddfDepthImg.get());
	_depthTex->setTextureSize(_width, _height);
	_depthTex->setInternalFormat(GL_LUMINANCE32F_ARB);
	_depthTex->setSourceFormat(GL_LUMINANCE);
	_depthTex->setSourceType(GL_FLOAT);
	_depthTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::NEAREST);
	_depthTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::NEAREST);
	_depthTex->setResizeNonPowerOfTwoHint(false);
	_depthTex->setWrap(osg::Texture2D::WRAP_S,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	_depthTex->setWrap(osg::Texture2D::WRAP_T,
		osg::Texture2D::CLAMP_TO_EDGE
		);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, _depthTex.get());
	
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\ZToDepth.vert", 
		DATAPATH + "Myshaders\\ZToDepth.frag")
		);
	/*
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\esample.vert", 
		DATAPATH + "Myshaders\\esample.frag"));//2014/3/21*/
	osg::ref_ptr<osg::Uniform> texDepth = new osg::Uniform("texDepth", 0);
	osg::ref_ptr<osg::Uniform> texSize = new osg::Uniform("texSize", osg::Vec2(float(_width), float(_height)));
	//osg::ref_ptr<osg::Uniform> isShow = new osg::Uniform("isShow", false);
	//osg::ref_ptr<osg::Uniform> dNear = new osg::Uniform("near", mNear);
	//osg::ref_ptr<osg::Uniform> dFar = new osg::Uniform("far", mFar);
	//osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("tex",leveltexture[LEVEL-1][NGROUP-1].get());//2014/3/21
	//add laplace modul;

	quad1->getOrCreateStateSet()->addUniform(texDepth.get());
	quad1->getOrCreateStateSet()->addUniform(texSize.get());
	//quad1->getOrCreateStateSet()->addUniform(isShow.get());
	//quad1->getOrCreateStateSet()->addUniform(dNear.get());
	//quad1->getOrCreateStateSet()->addUniform(dFar.get());
	//quad1->getOrCreateStateSet()->addUniform(tex.get());//2014/3/21



	_detectCam = new osg::Camera;
	_detectCam->setViewport(0, 0, _width, _height);
	_detectCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	_detectCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	_detectCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	_detectCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_detectCam->setRenderOrder(osg::Camera::POST_RENDER, 300000);
	_detectCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	_detectCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	_detectCam->attach(osg::Camera::COLOR_BUFFER, _msdepthImg.get());
	
	_detectCam->addChild(quad1.get());

	root->addChild(_detectCam.get());
}
void LocalViewer::createMSRDepthEdge()
{

	/*
	colorTex->setImage(colorImg.get());
	colorTex->setInternalFormat(GL_RGB);
	colorTex->setResizeNonPowerOfTwoHint(false);
	*/

	_msedge=new osg::Image;
	//_msedge->allocateImage(_width,_height,1,GL_RGB,GL_UNSIGNED_BYTE);
	//_msedge->setInternalTextureFormat(GL_RGB);
	_msedge->allocateImage(_width, _height, 1, GL_LUMINANCE_ALPHA, GL_FLOAT);
	_msedge->setInternalTextureFormat(GL_LUMINANCE_ALPHA32F_ARB);
	//_msedge->allocateImage(_width,_height,0,GL_RGB,);
	osg::ref_ptr<osg::Texture2D> depthTex = new osg::Texture2D;
	depthTex->setImage(_msdepth.get());
	depthTex->setTextureSize(_width, _height);
	depthTex->setInternalFormat(GL_RGB);
	depthTex->setResizeNonPowerOfTwoHint(false);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, depthTex.get());
	
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\msdetect.vert", 
		DATAPATH + "Myshaders\\msdetect.frag")
		);
	/*
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\esample.vert", 
		DATAPATH + "Myshaders\\esample.frag"));//2014/3/21*/
	osg::ref_ptr<osg::Uniform> texDepth = new osg::Uniform("texDepth", 0);
	osg::ref_ptr<osg::Uniform> texSize = new osg::Uniform("texSize", osg::Vec2(float(_width), float(_height)));
	osg::ref_ptr<osg::Uniform> isShow = new osg::Uniform("isShow", false);
	osg::ref_ptr<osg::Uniform> dNear = new osg::Uniform("near", mNear);
	osg::ref_ptr<osg::Uniform> dFar = new osg::Uniform("far", mFar);
	//osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("tex",leveltexture[LEVEL-1][NGROUP-1].get());//2014/3/21
	//add laplace modul;
	quad1->getOrCreateStateSet()->addUniform(texDepth.get());
	quad1->getOrCreateStateSet()->addUniform(texSize.get());
	quad1->getOrCreateStateSet()->addUniform(isShow.get());
	quad1->getOrCreateStateSet()->addUniform(dNear.get());
	quad1->getOrCreateStateSet()->addUniform(dFar.get());
	//quad1->getOrCreateStateSet()->addUniform(tex.get());//2014/3/21

	osg::ref_ptr<osg::Camera> _detectCam = new osg::Camera;
	_detectCam->setViewport(0, 0, _width, _height);
	_detectCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	_detectCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	_detectCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	_detectCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_detectCam->setRenderOrder(osg::Camera::PRE_RENDER, 3);
	_detectCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	_detectCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	_detectCam->attach(osg::Camera::COLOR_BUFFER, _msedge.get());
	//_detectCam->setPostDrawCallback(new DepthCallback);
	_detectCam->addChild(quad1.get());

	root->addChild(_detectCam.get());

}
void LocalViewer::createPSNRCam()//实验截图
{
	/*
	osg::ref_ptr<osg::Image> _regularImg;
	osg::ref_ptr<osg::Image> _poissonImg;
	osg::ref_ptr<osg::Camera> regularcam;
	osg::ref_ptr<osg::Camera> poissoncam;
	*/
	_regularImg=new osg::Image;
	_regularImg->allocateImage(_width,_height,1,GL_RGB,GL_UNSIGNED_BYTE);
	_poissonImg=new osg::Image;
	_poissonImg->allocateImage(_width,_height,1,GL_RGB,GL_UNSIGNED_BYTE);
	_originalImg=new osg::Image;
	_originalImg->allocateImage(_width,_height,1,GL_RGB,GL_UNSIGNED_BYTE);
	regularcam = new osg::Camera;
	regularcam->setViewport(0, 0, _width, _height);
	regularcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	regularcam->setRenderOrder(osg::Camera::POST_RENDER, 2000);
	regularcam->setClearColor(BACKCOLOR);
	//regularcam->setProjectionMatrixAsPerspective(45.0, (double)(_width)/(double)(_height), mNear,mFar + 100);
	regularcam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	regularcam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	regularcam->attach(osg::Camera::COLOR_BUFFER,_regularImg.get());

	//this->addSlave(regularcam.get(), false);
	root->addChild(regularcam.get());

	poissoncam = new osg::Camera;
	poissoncam->setViewport(0, 0, _width, _height);
	poissoncam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	poissoncam->setRenderOrder(osg::Camera::POST_RENDER, 2001);
	poissoncam->setClearColor(BACKCOLOR);
	//poissoncam->setProjectionMatrixAsPerspective(45.0, (double)(_width)/(double)(_height), mNear,mFar + 100);
	poissoncam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	poissoncam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	poissoncam->attach(osg::Camera::COLOR_BUFFER,_poissonImg.get());
	//this->addSlave(poissoncam.get(), false);
	root->addChild(poissoncam.get());

	originalcam = new osg::Camera;
	originalcam->setViewport(0, 0, _width, _height);
	originalcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	originalcam->setRenderOrder(osg::Camera::POST_RENDER, 2002);
	originalcam->setClearColor(BACKCOLOR);
	//originalcam->setProjectionMatrixAsPerspective(45.0, (double)(_width)/(double)(_height), mNear,mFar + 100);
	originalcam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	originalcam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	originalcam->attach(osg::Camera::COLOR_BUFFER,_originalImg.get());
	//this->addSlave(originalcam.get(), false);
	root->addChild(originalcam.get());
	
}
void LocalViewer::createShowPoissonsampleCamAnother()
{

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	img=new osg::Image;
	img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 250;
	traits->y = 250;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="泊松碟采样Another";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	showpoissonsamplecamanother=new osg::Camera;

	showpoissonsamplecamanother->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showpoissonsamplecamanother->setDrawBuffer(buffer);
	showpoissonsamplecamanother->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (leveltexture[LEVEL-1][3]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\sampleshow.vert", 
		DATAPATH + "Myshaders\\sampleshowanother.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("tex",0);
	osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showpoissonsamplecamanother->getOrCreateStateSet()->addUniform(tex.get());
	showpoissonsamplecamanother->getOrCreateStateSet()->addUniform(w.get());
	showpoissonsamplecamanother->getOrCreateStateSet()->addUniform(h.get());

	showpoissonsamplecamanother->setViewport(0, 0, _width, _height);
	showpoissonsamplecamanother->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showpoissonsamplecamanother->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showpoissonsamplecamanother->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showpoissonsamplecamanother->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showpoissonsamplecamanother->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showpoissonsamplecamanother->addChild(quad1.get());

	this->addSlave(showpoissonsamplecamanother.get(), false);
	//root->addChild(showpoissonsamplecam.get());
}
void LocalViewer::createShowPoissonsampleCam()
{

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	img=new osg::Image;
	img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 250;
	traits->y = 250;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="泊松碟采样";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	showpoissonsamplecam=new osg::Camera;
	
	showpoissonsamplecam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showpoissonsamplecam->setDrawBuffer(buffer);
	showpoissonsamplecam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (leveltexture[LEVEL-1][3]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\sampleshow.vert", 
		DATAPATH + "Myshaders\\sampleshow.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("tex",0);
	osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showpoissonsamplecam->getOrCreateStateSet()->addUniform(tex.get());
	showpoissonsamplecam->getOrCreateStateSet()->addUniform(w.get());
	showpoissonsamplecam->getOrCreateStateSet()->addUniform(h.get());

	showpoissonsamplecam->setViewport(0, 0, _width, _height);
	showpoissonsamplecam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showpoissonsamplecam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showpoissonsamplecam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showpoissonsamplecam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showpoissonsamplecam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showpoissonsamplecam->addChild(quad1.get());

	this->addSlave(showpoissonsamplecam.get(), false);
	//root->addChild(showpoissonsamplecam.get());

}
LocalViewer::~LocalViewer()
{

}
void LocalViewer::createBUSampleCam()
{
	if(BLEVEL<ULEVEL)
		return;
	int blevel=BLEVEL;
	int ulevel=ULEVEL;
	int group=BUNGROUP;
	bur_group=new osg::Uniform(osg::Uniform::INT,"r_group",group*(BLEVEL-ULEVEL+1));
	//设置callback

	osg::ref_ptr<osg::Texture2D> showDepthTex = new osg::Texture2D;
	showDepthTex->setImage(_depthImg.get());
	showDepthTex->setTextureSize(_width,_height);
	showDepthTex->setInternalFormat(GL_LUMINANCE32F_ARB);
	showDepthTex->setSourceFormat(GL_LUMINANCE);
	showDepthTex->setSourceType(GL_FLOAT);
	showDepthTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::NEAREST);
	showDepthTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::NEAREST);
	showDepthTex->setWrap(osg::Texture2D::WRAP_S,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	showDepthTex->setWrap(osg::Texture2D::WRAP_T,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	showDepthTex->setResizeNonPowerOfTwoHint(false);
	int gid=0;
	srand(unsigned int(time(NULL)));
	for(int i=ULEVEL;i<=BLEVEL;i++)
	{
		//生成随机数
		if(i!=0)
		{
			//产生随机伪随机数

			//组内随机数的产生
			bur_group->setElement((i-ulevel)*group,rand()%BUNGROUP);
			bur_group->setElement((i-ulevel)*group+1,rand()%BUNGROUP);
			bur_group->setElement((i-ulevel)*group+2,rand()%BUNGROUP);
			bur_group->setElement((i-ulevel)*group+3,rand()%BUNGROUP);

			//r_group->setElement(i*group,0);
			//r_group->setElement(i*group+1,1);
			//r_group->setElement(i*group+2,2);
			//r_group->setElement(i*group+3,3);

		}else
		{
			bur_group->setElement((i-ulevel)*group,0);
			bur_group->setElement((i-ulevel)*group+1,0);
			bur_group->setElement((i-ulevel)*group+2,0);
			bur_group->setElement((i-ulevel)*group+3,0);
		}
		if(i>0)
			group=4;
		else
			group=1;
		for(int j=0;j<group;j++)
		{
			gid=j;
			bulevelcamera[i-ULEVEL][j]=new osg::Camera;
			bulevelimg[i-ULEVEL][j]=new osg::Image;
			bulevelimg[i-ULEVEL][j]->allocateImage(_width, _height, 1, GL_LUMINANCE_ALPHA, GL_FLOAT);
			bulevelimg[i-ULEVEL][j]->setInternalTextureFormat(GL_LUMINANCE_ALPHA32F_ARB);


			buleveltexture[i-ULEVEL][j]=new osg::Texture2D;
			buleveltexture[i-ULEVEL][j]->setImage(bulevelimg[i-ULEVEL][j].get());
			buleveltexture[i-ULEVEL][j]->setTextureSize(_width, _height);
			buleveltexture[i-ULEVEL][j]->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
			buleveltexture[i-ULEVEL][j]->setSourceFormat(GL_LUMINANCE_ALPHA);
			buleveltexture[i-ULEVEL][j]->setSourceType(GL_FLOAT);
			buleveltexture[i-ULEVEL][j]->setFilter(osg::Texture2D::MIN_FILTER,
				osg::Texture2D::NEAREST);
			buleveltexture[i-ULEVEL][j]->setFilter(osg::Texture2D::MAG_FILTER,
				osg::Texture2D::NEAREST);
			buleveltexture[i-ULEVEL][j]->setWrap(osg::Texture2D::WRAP_S,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			buleveltexture[i-ULEVEL][j]->setWrap(osg::Texture2D::WRAP_T,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			buleveltexture[i-ULEVEL][j]->setResizeNonPowerOfTwoHint(false);


			busquad=new ScreenQuad;

			osg::ref_ptr<osg::Texture2D> region=new osg::Texture2D;
			region->setImage(_edgeRegionImg.get());
			region->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
			region->setResizeNonPowerOfTwoHint(false);

			busquad->getOrCreateStateSet()->setTextureAttributeAndModes(1,showDepthTex.get());//原始深度纹理
			busquad->getOrCreateStateSet()->setTextureAttributeAndModes(2,region.get());///ROI区域
			if(ULEVEL==i && 0==j)
			{
				busquad->getOrCreateStateSet()->setTextureAttributeAndModes(0,showDepthTex.get());
			}
			else
			{
				if(0==j)
				{
					busquad->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[i-ULEVEL-1][group-1]).get());

				}else
				{

					busquad->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[i-ULEVEL][j-1]).get());
				}
			}
			osg::ref_ptr<osg::Uniform> id=new osg::Uniform("gid",gid);
			osg::ref_ptr<osg::Uniform> ptex=new osg::Uniform("prev_tex",0);
			osg::ref_ptr<osg::Uniform> dtex=new osg::Uniform("depth_tex",1);
			osg::ref_ptr<osg::Uniform> l=new osg::Uniform("level",BLEVEL-i+ULEVEL);
			osg::ref_ptr<osg::Uniform> s_w=new osg::Uniform("_width",_width);
			osg::ref_ptr<osg::Uniform> s_h=new osg::Uniform("_height",_height);
			osg::ref_ptr<osg::Uniform> seed=new osg::Uniform("seed",unsigned int(rand()));
			osg::ref_ptr<osg::Uniform> fl=new osg::Uniform("firstlevel",BLEVEL);
			osg::ref_ptr<osg::Uniform> roi=new osg::Uniform("roi",2);
			osg::ref_ptr<osg::Uniform> bl=new osg::Uniform("blevel",BLEVEL);
			osg::ref_ptr<osg::Uniform> ul=new osg::Uniform("ulevel",ULEVEL);
			seed->setUpdateCallback(new updateSeed(this));
			//cout<<unsigned int (rand())<<endl;
			busquad->getOrCreateStateSet()->addUniform(bl.get());
			busquad->getOrCreateStateSet()->addUniform(ul.get());
			busquad->getOrCreateStateSet()->addUniform(fl.get());
			busquad->getOrCreateStateSet()->addUniform(dtex.get());
			busquad->getOrCreateStateSet()->addUniform(ptex.get());
			busquad->getOrCreateStateSet()->addUniform(seed.get());
			busquad->getOrCreateStateSet()->addUniform(id.get());
			busquad->getOrCreateStateSet()->addUniform(l.get());
			busquad->getOrCreateStateSet()->addUniform(s_w.get());
			busquad->getOrCreateStateSet()->addUniform(s_h.get());
			busquad->getOrCreateStateSet()->addUniform(r_group.get());
			busquad->getOrCreateStateSet()->addUniform(roi.get());
			busquad->getOrCreateStateSet()->setAttributeAndModes(
				createProgram(DATAPATH + "Myshaders\\bupoissonsample.vert", 
				DATAPATH + "Myshaders\\bupoissonsample.frag")
				);
			bulevelcamera[i-ULEVEL][j]->setViewport(0,0,_width,_height);
			bulevelcamera[i-ULEVEL][j]->setClearColor(WHITE);
			bulevelcamera[i-ULEVEL][j]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			bulevelcamera[i-ULEVEL][j]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
			bulevelcamera[i-ULEVEL][j]->setProjectionMatrixAsOrtho2D(-1.0,1.0,-1.0,0.0);
			bulevelcamera[i-ULEVEL][j]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
			bulevelcamera[i-ULEVEL][j]->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
			bulevelcamera[i-ULEVEL][j]->setRenderOrder(osg::Camera::PRE_RENDER,baseorder+group*i+j);
			bulevelcamera[i-ULEVEL][j]->attach(osg::Camera::COLOR_BUFFER,bulevelimg[i-ULEVEL][j].get());
			bulevelcamera[i-ULEVEL][j]->addChild(busquad.get());
			root->addChild(bulevelcamera[i-ULEVEL][j].get());

		}
	}
}
void LocalViewer::createPoissoncamera()
{

	osg::Timer t=osg::Timer();
	t.setStartTick();
	int level=1;
	int group=1;
	int X=1024;
	int Y=1024;
	osg::ref_ptr<osg::Uniform> r_group=new osg::Uniform(osg::Uniform::INT,"r_group",4*MAXLEVEL);


	osg::ref_ptr<osg::Uniform> l_vec_0=new osg::Uniform(osg::Uniform::INT_VEC2,"level0",1);
	osg::ref_ptr<osg::Uniform> l_vec_1=new osg::Uniform(osg::Uniform::INT_VEC2,"level1",4);
	osg::ref_ptr<osg::Uniform> l_vec_2=new osg::Uniform(osg::Uniform::INT_VEC2,"level2",16);
	osg::ref_ptr<osg::Uniform> l_vec_3=new osg::Uniform(osg::Uniform::INT_VEC2,"level3",64);
	osg::ref_ptr<osg::Uniform> l_vec_4=new osg::Uniform(osg::Uniform::INT_VEC2,"level4",256);
	osg::ref_ptr<osg::Uniform> l_vec_5=new osg::Uniform(osg::Uniform::INT_VEC2,"level5",1024);
	osg::ref_ptr<osg::Uniform> l_vec_6=new osg::Uniform(osg::Uniform::INT_VEC2,"level6",4096);
	osg::ref_ptr<osg::Uniform> l_vec_7=new osg::Uniform(osg::Uniform::INT_VEC2,"level7",16384);
	

	
	srand(unsigned(time(NULL)));
	for(int i=0;i<=level;i++)
	{
		if(i!=0)
		{
			group=4;
			//产生随机伪随机数
			
			//组内随机数的产生
			r_group->setElement(i*group,rand()%4);
			r_group->setElement(i*group+1,rand()%4);
			r_group->setElement(i*group+2,rand()%4);
			r_group->setElement(i*group+3,rand()%4);

		}else
		{
			r_group->setElement(i*group,0);
			r_group->setElement(i*group+1,0);
			r_group->setElement(i*group+2,0);
			r_group->setElement(i*group+3,0);
		}
		//
		int p=1<<(i);//左移
		int _x=X/p;//每一个Level级别层的Grid大小
		int _y=Y/p;//每一个Level级别层的Grid大小
		
		if(i<1)
		{
			
			//level_vec_0[0]=vec;
			l_vec_0->setElement(0,rand()%_x,rand()%_y);
		}else
		{

			//int group_size=_x;
			//(0,_x),(0,_y)内产生一个随机数
			int group_size=1<<(2*i-2);//i>1
			for(int j=0;j<group;j++)
			{
				for(int k=0;k<group_size;k++)
				{
					int	rand_x=rand()%_x;
					int rand_y=rand()%_y;
					switch(i)
					{
					case 1:
					
						//level_vec_1[j]=vec;//4组
						l_vec_1->setElement(j,rand_x,rand_y);
						break;
					case  2:
						//level_vec_2[j*group_size+k]=vec;
						l_vec_2->setElement(j*group_size+k,rand_x,rand_y);
						break;
					case 3:
					
						//level_vec_3[j*group_size+k]=vec;
						l_vec_3->setElement(j*group_size+k,rand_x,rand_y);
						break;
					case 4:
					
						//level_vec_4[j*group_size+k]=vec;
						l_vec_4->setElement(j*group_size+k,rand_x,rand_y);
						break;
					case 5:
						//level_vec_5[j*group_size+k]=vec;
						l_vec_5->setElement(j*group_size+k,rand_x,rand_y);
						break;
					case 6:
					
						//level_vec_6[j*group_size+k]=vec;
						l_vec_6->setElement(j*group_size+k,rand_x,rand_y);
						break;
					case 7:
					
						//level_vec_7[j*group_size+k]=vec;
						l_vec_7->setElement(j*group_size+k,rand_x,rand_y);
						break;
					default:
						break;
					}
					
				}

			}
		}


	}
	cout<<"时间： "<<t.time_m()<<endl;
	
	
	osg::ref_ptr<osg::Texture2D> sample_texture[MAXLEVEL][4];//【层次】【组号】
	group=1;
	osg::ref_ptr<osg::Camera> cam_level[MAXLEVEL][4];//[层次][组号]
	//添加camera
	int gid=0;
	for(int i=0;i<=level;i++)
	{
		if(i>1)
			group=4;
		for(int j=0;j<group;j++)
		{
			//纹理
			//组内camera
			gid=j;
			cam_level[i][j]=new osg::Camera;
			sample_texture[i][j]=new osg::Texture2D;
			sample_texture[i][j]->setTextureSize(_width, _height);
			sample_texture[i][j]->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
			sample_texture[i][j]->setSourceFormat(GL_LUMINANCE_ALPHA);
			sample_texture[i][j]->setSourceType(GL_FLOAT);
			sample_texture[i][j]->setFilter(osg::Texture2D::MIN_FILTER,
				osg::Texture2D::NEAREST);
			sample_texture[i][j]->setFilter(osg::Texture2D::MAG_FILTER,
				osg::Texture2D::NEAREST);
			sample_texture[i][j]->setWrap(osg::Texture2D::WRAP_S,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			sample_texture[i][j]->setWrap(osg::Texture2D::WRAP_T,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			sample_texture[i][j]->setResizeNonPowerOfTwoHint(false);

			osg::ref_ptr<ScreenQuad> quad=new ScreenQuad;

			quad->getOrCreateStateSet()->setTextureAttributeAndModes(1,_depthTex.get());//原始深度纹理
			if(0==i)
				quad->getOrCreateStateSet()->setTextureAttributeAndModes(0,_depthTex.get());
			else
			{
				if(0==j)
				{
					quad->getOrCreateStateSet()->setTextureAttributeAndModes(0,sample_texture[(i-1)][group-1]);
				}else
				{
					quad->getOrCreateStateSet()->setTextureAttributeAndModes(0,sample_texture[i][j-1]);
				}
			}
			osg::ref_ptr<osg::Uniform> id=new osg::Uniform("gid",gid);
			osg::ref_ptr<osg::Uniform> ptex=new osg::Uniform("prev_tex",0);
			osg::ref_ptr<osg::Uniform> dtex=new osg::Uniform("depth_tex",1);
			osg::ref_ptr<osg::Uniform> l=new osg::Uniform("level",i);
			osg::ref_ptr<osg::Uniform> s_w=new osg::Uniform("width",_width);
			osg::ref_ptr<osg::Uniform> s_h=new osg::Uniform("height",_height);
			quad->getOrCreateStateSet()->addUniform(id.get());
			quad->getOrCreateStateSet()->addUniform(l.get());
			quad->getOrCreateStateSet()->addUniform(s_w.get());
			quad->getOrCreateStateSet()->addUniform(s_h.get());
			quad->getOrCreateStateSet()->addUniform(r_group.get());
			switch(i)
			{
			case 0:
				quad->getOrCreateStateSet()->addUniform(l_vec_0.get());
				break;
			case 1:
				quad->getOrCreateStateSet()->addUniform(l_vec_1.get());
				break;
			case 2:
				quad->getOrCreateStateSet()->addUniform(l_vec_2.get());
				break;
			case 3:
				quad->getOrCreateStateSet()->addUniform(l_vec_3.get());
				break;
			case 4:
				quad->getOrCreateStateSet()->addUniform(l_vec_4.get());
				break;
			case 5:
				quad->getOrCreateStateSet()->addUniform(l_vec_5.get());
				break;
			case 6:
				quad->getOrCreateStateSet()->addUniform(l_vec_6.get());
				break;
			case 7:
				quad->getOrCreateStateSet()->addUniform(l_vec_7.get());
				break;
			default:
				break;
			}
			quad->getOrCreateStateSet()->setAttributeAndModes(
				createProgram(DATAPATH + "Myshaders\\poisson.vert", 
				DATAPATH + "Myshaders\\poisson.frag")
				);
			cam_level[i][j]->setViewport(0,0,_width,_height);
			cam_level[i][j]->setClearColor(osg::Vec4(1,1,1,0));
			cam_level[i][j]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			cam_level[i][j]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
			cam_level[i][j]->setProjectionMatrixAsOrtho2D(-1.0,1.0,-1.0,0.0);
			cam_level[i][j]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
			cam_level[i][j]->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
			cam_level[i][j]->setRenderOrder(osg::Camera::PRE_RENDER,baseorder+group*i+j);
			cam_level[i][j]->attach(osg::Camera::COLOR_BUFFER,sample_texture[i][j].get());

			cam_level[i][j]->addChild(quad.get());
			root->addChild(cam_level[i][j].get());
		}

	}
	GLint* data;
	glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_BLOCKS,data);
	cout<<"...."<<*data<<endl;

	//////////////////////////////////////////////////////////////////////////
	//测试伪随机数的产生
	//////////////////////////////////////////////////////////////////////////
	/*
	for(int i=0;i<=level;i++)
	{
		if(i==0)
		{
			cout<<"level0:"<<endl;
			for(int j=0;j<1;j++)
			{
				//cout<<"level0:"<<endl;
				//osg::Vec2 v;
				int x,y;
				l_vec_0->getElement(j,x,y);
				cout<<x<<" "<<y<<endl;
			}
		}else if(i==1)
		{
			cout<<"level1:"<<endl;
			for(int j=0;j<4;j++)
			{
				//cout<<level_vec_1[j].x()<<level_vec_1[j].y()<<endl;
				int x,y;
				l_vec_1->getElement(j,x,y);
				cout<<x<<" "<<y<<endl;
			}
		}else if(i==2)
		{
			cout<<"level2:"<<endl;
			for(int j=0;j<16;j++)
			{
				//cout<<level_vec_2[j].x()<<level_vec_2[j].y()<<endl;
				int x,y;
				l_vec_2->getElement(j,x,y);
				cout<<x<<" "<<y<<endl;
			}
		}else if(i==3)
		{
			cout<<"level3:"<<endl;
			for(int j=0;j<64;j++)
			{
				//cout<<level_vec_3[j].x()<<level_vec_3[j].y()<<endl;
				int x,y;
				l_vec_3->getElement(j,x,y);
				cout<<x<<" "<<y<<endl;
			}
		}else if(i==4)
		{
			cout<<"level4:"<<endl;
			for(int j=0;j<256;j++)
			{
				//cout<<level_vec_4[j].x()<<level_vec_4[j].y()<<endl;
				int x,y;
				l_vec_4->getElement(j,x,y);
				cout<<x<<" "<<y<<endl;
			}
		}else if(i==5)
		{
			cout<<"level5:"<<endl;
			for(int j=0;j<1024;j++)
			{
				//cout<<level_vec_5[j].x()<<level_vec_5[j].y()<<endl;
				int x,y;
				l_vec_5->getElement(j,x,y);
				cout<<x<<" "<<y<<endl;
			}
		}
	}*/
	//////////////////////////////////////////////////////////////////////////
}
void LocalViewer::createPoissionSample()
{
	//
	int level=LEVEL;
	int group=NGROUP;
	r_group=new osg::Uniform(osg::Uniform::INT,"r_group",group*level);
	r_group->setUpdateCallback(new updateGroup(this));
	osg::ref_ptr<osg::Texture2D> showDepthTex = new osg::Texture2D;

	//2014/4/26
	osg::ref_ptr<osg::Texture2D> saliencyMap=new osg::Texture2D;
	saliencyMap->setImage(_saliencymap.get());
	saliencyMap->setInternalFormat(GL_RGB);
	saliencyMap->setResizeNonPowerOfTwoHint(false);
	//

	/*
	_depthImg = new osg::Image;
	_depthImg->allocateImage(_width, _height, 1, GL_LUMINANCE, GL_FLOAT);
	_depthImg->setInternalTextureFormat(GL_LUMINANCE32F_ARB);
	*/
#ifdef ISMSDEPTH
	showDepthTex->setImage(_msdepth.get());
	showDepthTex->setTextureSize(_width,_height);
	showDepthTex->setInternalFormat(GL_RGB);
	showDepthTex->setResizeNonPowerOfTwoHint(false);
#else
	showDepthTex->setImage(_depthImg.get());
	showDepthTex->setTextureSize(_width,_height);
	showDepthTex->setInternalFormat(GL_LUMINANCE32F_ARB);
	showDepthTex->setSourceFormat(GL_LUMINANCE);
	showDepthTex->setSourceType(GL_FLOAT);
	showDepthTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::NEAREST);
	showDepthTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::NEAREST);
	showDepthTex->setWrap(osg::Texture2D::WRAP_S,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	showDepthTex->setWrap(osg::Texture2D::WRAP_T,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	showDepthTex->setResizeNonPowerOfTwoHint(false);
#endif
	//showDepthTex->setImage(_depthImg.get());
	int gid=0;
	srand(unsigned int(time(NULL)));
	for(int i=0;i<level;i++)
	{
		//生成随机数
		if(i!=0)
		{
			//产生随机伪随机数

			//组内随机数的产生
			//2014_5_16
			r_group->setElement(i*group,rand()%NGROUP);
			r_group->setElement(i*group+1,rand()%NGROUP);
			r_group->setElement(i*group+2,rand()%NGROUP);
			r_group->setElement(i*group+3,rand()%NGROUP);

			//r_group->setElement(i*group,0);
			//r_group->setElement(i*group+1,1);
			//r_group->setElement(i*group+2,2);
			//r_group->setElement(i*group+3,3);

		}else
		{
			r_group->setElement(i*group,0);
			r_group->setElement(i*group+1,0);
			r_group->setElement(i*group+2,0);
			r_group->setElement(i*group+3,0);
		}
		if(i>0)
			group=4;
		else
			group=1;
		for(int j=0;j<group;j++)
		{
			gid=j;
			levelcamera[i][j]=new osg::Camera;
			levelimg[i][j]=new osg::Image;
			levelimg[i][j]->allocateImage(_width, _height, 1, GL_LUMINANCE_ALPHA, GL_FLOAT);
			levelimg[i][j]->setInternalTextureFormat(GL_LUMINANCE_ALPHA32F_ARB);


			leveltexture[i][j]=new osg::Texture2D;
			leveltexture[i][j]->setImage(levelimg[i][j].get());
			leveltexture[i][j]->setTextureSize(_width, _height);
			leveltexture[i][j]->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
			leveltexture[i][j]->setSourceFormat(GL_LUMINANCE_ALPHA);
			leveltexture[i][j]->setSourceType(GL_FLOAT);
			leveltexture[i][j]->setFilter(osg::Texture2D::MIN_FILTER,
				osg::Texture2D::NEAREST);
			leveltexture[i][j]->setFilter(osg::Texture2D::MAG_FILTER,
				osg::Texture2D::NEAREST);
			leveltexture[i][j]->setWrap(osg::Texture2D::WRAP_S,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			leveltexture[i][j]->setWrap(osg::Texture2D::WRAP_T,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			leveltexture[i][j]->setResizeNonPowerOfTwoHint(false);


			squad=new ScreenQuad;

			osg::ref_ptr<osg::Texture2D> region=new osg::Texture2D;
			region->setImage(_edgeRegionImg.get());
			region->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
			region->setResizeNonPowerOfTwoHint(false);

			osg::ref_ptr<osg::Texture2D> edgetexture=new osg::Texture2D;
#ifdef ISMSDEPTH
			edgetexture->setImage(_msedge.get());
#else
			edgetexture->setImage(_edgeImg.get());
#endif
			edgetexture->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
			edgetexture->setResizeNonPowerOfTwoHint(false);

			squad->getOrCreateStateSet()->setTextureAttributeAndModes(1,showDepthTex.get());//原始深度纹理
			squad->getOrCreateStateSet()->setTextureAttributeAndModes(2,region.get());///ROI区域
			//2014/4/26
			squad->getOrCreateStateSet()->setTextureAttributeAndModes(3,saliencyMap.get());
			squad->getOrCreateStateSet()->setTextureAttributeAndModes(4,edgetexture.get());
			//
			if(0==i)
			{
				squad->getOrCreateStateSet()->setTextureAttributeAndModes(0,showDepthTex.get());
			}
			else
			{
				if(0==j)
				{
					if(i==1)
					{
						squad->getOrCreateStateSet()->setTextureAttributeAndModes(0,(leveltexture[(i-1)][0]).get());
					}else
					{
						squad->getOrCreateStateSet()->setTextureAttributeAndModes(0,(leveltexture[(i-1)][group-1]).get());
					}
					
				}else
				{
					
					squad->getOrCreateStateSet()->setTextureAttributeAndModes(0,(leveltexture[i][j-1]).get());
				}
			}
			osg::ref_ptr<osg::Uniform> id=new osg::Uniform("gid",gid);
			osg::ref_ptr<osg::Uniform> ptex=new osg::Uniform("prev_tex",0);
			osg::ref_ptr<osg::Uniform> dtex=new osg::Uniform("depth_tex",1);
			osg::ref_ptr<osg::Uniform> etex=new osg::Uniform("edge_tex",4);
			osg::ref_ptr<osg::Uniform> l=new osg::Uniform("level",i);
			osg::ref_ptr<osg::Uniform> s_w=new osg::Uniform("_width",_width);
			osg::ref_ptr<osg::Uniform> s_h=new osg::Uniform("_height",_height);
			osg::ref_ptr<osg::Uniform> seed=new osg::Uniform("seed",unsigned int(rand()));
			osg::ref_ptr<osg::Uniform> roi=new osg::Uniform("roi",2);
			//2014/4/26
			bool isSaliencySample=true;
			bool isEdgeSample=true;
#ifdef ISSALIENCYSAMPLE
			isSaliencySample=true;
#else
			isSaliencySample=false;

#endif

#ifdef ISEDGESAMPLE
			isEdgeSample=true;
#else
			isEdgeSample=false;
#endif
			osg::ref_ptr<osg::Uniform> saliencyMapUniform=new osg::Uniform("saliency",3);
			osg::ref_ptr<osg::Uniform> isSaliencySampleUniform=new osg::Uniform("isSaliencySample",isSaliencySample);
			osg::ref_ptr<osg::Uniform> thresholdUniform=new osg::Uniform("threshold",threshold);
			osg::ref_ptr<osg::Uniform> isEdgesampleUniform=new osg::Uniform("isEdgeSample",isEdgeSample);

			squad->getOrCreateStateSet()->addUniform(saliencyMapUniform.get());
			squad->getOrCreateStateSet()->addUniform(isSaliencySampleUniform.get());
			squad->getOrCreateStateSet()->addUniform(thresholdUniform.get());
			squad->getOrCreateStateSet()->addUniform(isEdgesampleUniform.get());
			squad->getOrCreateStateSet()->addUniform(etex.get());

			//2014/4/26
			//osg::ref_ptr<osg::Uniform> fl=new osg::Uniform("firstlevel",0);
			seed->setUpdateCallback(new updateSeed(this));
			//cout<<unsigned int (rand())<<endl;
			//squad->getOrCreateStateSet()->addUniform(fl.get());
			squad->getOrCreateStateSet()->addUniform(dtex.get());
			squad->getOrCreateStateSet()->addUniform(ptex.get());
			squad->getOrCreateStateSet()->addUniform(seed.get());
			squad->getOrCreateStateSet()->addUniform(id.get());
			squad->getOrCreateStateSet()->addUniform(l.get());
			squad->getOrCreateStateSet()->addUniform(s_w.get());
			squad->getOrCreateStateSet()->addUniform(s_h.get());
			squad->getOrCreateStateSet()->addUniform(r_group.get());
			squad->getOrCreateStateSet()->addUniform(roi.get());
			squad->getOrCreateStateSet()->setAttributeAndModes(
				createProgram(DATAPATH + "Myshaders\\poissonsample.vert", 
				DATAPATH + "Myshaders\\poissonsample.frag")
				);
			levelcamera[i][j]->setViewport(0,0,_width,_height);
			levelcamera[i][j]->setClearColor(WHITE);
			levelcamera[i][j]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			levelcamera[i][j]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
			levelcamera[i][j]->setProjectionMatrixAsOrtho2D(-1.0,1.0,-1.0,0.0);
			levelcamera[i][j]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
			levelcamera[i][j]->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
			levelcamera[i][j]->setRenderOrder(osg::Camera::PRE_RENDER,baseorder+group*i+j);
			levelcamera[i][j]->attach(osg::Camera::COLOR_BUFFER,levelimg[i][j].get());
			//if(i==level-1 && j==NGROUP-1)
			//{
				//levelcamera[i][j]->setPostDrawCallback(new sampleCallback);
			//}
			levelcamera[i][j]->addChild(squad.get());
			root->addChild(levelcamera[i][j].get());

		}
	}


};

void LocalViewer::createVisiblePointCamera()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;

	traits->x = 50;
	traits->y = 50;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="可见点";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	//osg::ref_ptr<osg::Camera>
	_visiblePointCam=new osg::Camera;
	_visiblePointCam->setGraphicsContext(gc.get());
	
	_visiblePointCam->setViewport(0, 0, traits->width, traits->height);
	_visiblePointCam->setClearColor(WHITE);
	_visiblePointCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//保留深度信息
	_visiblePointCam->setRenderOrder(osg::Camera::POST_RENDER,1);
	_visiblePointCam->setProjectionMatrixAsPerspective( 45.0f, (double)(traits->width)/(double)(traits->height), mNear, mFar);
	_visiblePointCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	//_visiblePointCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	//_visiblePointCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	
	

	stex = new osg::Texture2D;
	stex->setTextureSize(_width, _height);
	stex->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
	stex->setSourceFormat(GL_LUMINANCE_ALPHA);
	stex->setSourceType(GL_FLOAT);
	stex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::NEAREST);
	stex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::NEAREST);
	stex->setWrap(osg::Texture2D::WRAP_S,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	//_visiblePointCam->attach(osg::Camera::COLOR_BUFFER,stex.get());


	stex->setWrap(osg::Texture2D::WRAP_T,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	stex->setResizeNonPowerOfTwoHint(false);

	pdepthImge=new osg::Image;
	pdepthImge->allocateImage(_width, _height, 1, GL_LUMINANCE, GL_FLOAT);
	pdepthImge->setInternalTextureFormat(GL_LUMINANCE32F_ARB);
	
	pcolorImage=new osg::Image;
	pcolorImage->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);

	//_visiblePointCam->attach(osg::Camera::COLOR_BUFFER,pdepthImge.get());
	/*
	_visiblePointCam->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\depth.vert", 
		DATAPATH + "Myshaders\\depth.frag")
		);*/
	/*
	_visiblePointCam->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\linearity_depth.vert", 
		DATAPATH + "Myshaders\\linear.frag")
		);*/
	//osg::ref_ptr<osg::Uniform> texture=new osg::Uniform("depth",stex.get());
	//osg::ref_ptr<osg::Uniform> texsize=new osg::Uniform("texsize",osg::Vec2(float(_width),float(_height)));
	
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	_visiblePointCam->setDrawBuffer(buffer);
	_visiblePointCam->setReadBuffer(buffer);

	//_visiblePointCam->setPostDrawCallback(new getNearFarCallback);
	osg::ref_ptr<osg::PolygonMode> pm=new osg::PolygonMode;
	
	pm->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
	
	//pm->setMode(osg::PolygonMode::BACK,osg::PolygonMode::LINE);
	//pm->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::CULLFACE);
	//2014/2/21
	osg::ref_ptr<osg::Group> g=new osg::Group;
	osg::ref_ptr<osg::MatrixTransform> node=new osg::MatrixTransform;
	node->addChild(_modelNode.get());
	node->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\background.vert", 
		DATAPATH + "Myshaders\\background.frag"));
	g->addChild(node.get());
	//
	osg::ref_ptr<osg::CullFace> cf=new osg::CullFace;
	cf->setMode(osg::CullFace::BACK);
	osg::ref_ptr<osg::MatrixTransform> tran=new osg::MatrixTransform;
	tran->addChild(_modelNode.get());

	osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;
	osg::ref_ptr<osg::PolygonOffset> polyoffset = new osg::PolygonOffset;
	osg::ref_ptr<osg::LineWidth> lw=new osg::LineWidth;
	lw->setWidth(2.0);
	polyoffset->setFactor(-0.75f);
	polyoffset->setUnits(-1.0f);
	osg::ref_ptr<osg::PolygonMode> polymode = new osg::PolygonMode;
	polymode->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
	
	
	//osg::ref_ptr<osg::LineWidth> lw=new osg::LineWidth;

	stateset->setAttributeAndModes(polyoffset,osg::StateAttribute::PROTECTED|osg::StateAttribute::ON);
	stateset->setAttributeAndModes(polymode,osg::StateAttribute::PROTECTED|osg::StateAttribute::ON);
	stateset->setAttributeAndModes(lw,osg::StateAttribute::PROTECTED | osg::StateAttribute::ON);
	tran->setStateSet(stateset.get());
	
	tran->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\point.vert", 
		DATAPATH + "Myshaders\\point.frag"));
	//tran->getOrCreateStateSet()->setAttribute(pm.get(),osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
	//tran->getOrCreateStateSet()->setAttributeAndModes(cf.get(),osg::StateAttribute::ON);
	//tran->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF| osg::StateAttribute::PROTECTED);
	//tran->getOrCreateStateSet()->setMode(GL_FRONT_FACE,osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
	//tran->getOrCreateStateSet()->addUniform(texture.get());
	//tran->getOrCreateStateSet()->addUniform(texsize.get());
	g->addChild(tran.get());
	_visiblePointCam->addChild(g.get());
	//_visiblePointCam->setPostDrawCallback(new DepthCallback);
	//root->addChild(_visiblePointCam.get());

	
	this->addSlave(_visiblePointCam.get(),false);

}
void LocalViewer::createEdgeCamera()
{
	_edgecam=new osg::Camera;



}
osg::Node* LocalViewer::loadModel()
{
	//选择不同的模型
#ifdef MODEL_ARMADILLO
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "aaaa.ply");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "engine2.osg");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "engine.osg");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "head.osg");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "qq.osg");//发动机模型
	
	
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "buddha.osg");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "bunny.ply");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "cow.osgt");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "dragon.ply");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "cessna.osg");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH+ "paris_no_tree.osg")
	//---------------------
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "armadillo_scale.ply");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "horse_scale.ply");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "bunny_scale.ply");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "dragon_scale.ply");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "isis_scale.ply");
	osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "david_scale.ply");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH + "maxplanck_scale.ply");
	//osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(DATAPATH +modelply);
	//---------------------
	
#else 
	osg::ref_ptr<osg::Group> root = new osg::Group;
	int i;
	for(i = 1; i <= MODELCNT; ++i)
	{
		std::string name = DATAPATH + SUBPATH + "model_";
		char c[5];
		itoa(i, c, 10);
		name += c;
		name += "\\";
		int j = 1;
		osg::ref_ptr<osg::Group> md = new osg::Group;
		while(true)
		{
			std::string file = "obj_";
			char c[5];
			itoa(j++, c, 10);
			file = name + file + c;
			file += ".obj";
			osg::ref_ptr<osg::Node> mode = osgDB::readNodeFile(file);
			if(!mode.get()) break;
			md->addChild(mode.get());
		}
		root->addChild(md.get());
		std::cout << i << " done" << std::endl;
	}
#endif
	return root.release();
}
void LocalViewer::createSampleCam()
{
	sampleImg=new osg::Image;
	sampleImg->allocateImage(_width, _height, 1, GL_LUMINANCE_ALPHA, GL_FLOAT);
	sampleImg->setInternalTextureFormat(GL_LUMINANCE_ALPHA32F_ARB);

	sampleTex = new osg::Texture2D;
	sampleTex->setImage(sampleImg.get());
	sampleTex->setTextureSize(_width, _height);
	sampleTex->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
	sampleTex->setSourceFormat(GL_LUMINANCE_ALPHA);
	sampleTex->setSourceType(GL_FLOAT);
	sampleTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::NEAREST);
	sampleTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::NEAREST);
	sampleTex->setWrap(osg::Texture2D::WRAP_S,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	sampleTex->setWrap(osg::Texture2D::WRAP_T,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	sampleTex->setResizeNonPowerOfTwoHint(false);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
#ifdef ISMSDEPTH
	osg::ref_ptr<osg::Texture2D> _msdepthTex= new osg::Texture2D;
	_msdepthTex->setImage(_msdepth.get());
	_msdepthTex->setTextureSize(_width, _height);
	_msdepthTex->setInternalFormat(GL_LUMINANCE32F_ARB);
	_msdepthTex->setSourceFormat(GL_LUMINANCE);
	_msdepthTex->setSourceType(GL_FLOAT);
	_msdepthTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::NEAREST);
	_msdepthTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::NEAREST);
	_msdepthTex->setResizeNonPowerOfTwoHint(false);
	_msdepthTex->setWrap(osg::Texture2D::WRAP_S,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	_msdepthTex->setWrap(osg::Texture2D::WRAP_T,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, _msdepthTex.get());
#else
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, _depthTex.get());
#endif
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(1,leveltexture[LEVEL-1][NGROUP-1].get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\esample.vert", 
		DATAPATH + "Myshaders\\esample.frag")
		);
	osg::ref_ptr<osg::Uniform> texDepth = new osg::Uniform("texDepth", 0);
	osg::ref_ptr<osg::Uniform> texSize = new osg::Uniform("texSize", osg::Vec2(float(_width), float(_height)));
	osg::ref_ptr<osg::Uniform> isShow = new osg::Uniform("isShow", false);
	osg::ref_ptr<osg::Uniform> dNear = new osg::Uniform("near", mNear);
	osg::ref_ptr<osg::Uniform> dFar = new osg::Uniform("far", mFar);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("tex",1);
	//add laplace modul;
	quad1->getOrCreateStateSet()->addUniform(texDepth.get());
	quad1->getOrCreateStateSet()->addUniform(texSize.get());
	quad1->getOrCreateStateSet()->addUniform(isShow.get());
	quad1->getOrCreateStateSet()->addUniform(dNear.get());
	quad1->getOrCreateStateSet()->addUniform(dFar.get());
	quad1->getOrCreateStateSet()->addUniform(tex.get());

	sampleCam = new osg::Camera;
	sampleCam->setViewport(0, 0, _width, _height);
	sampleCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	sampleCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	sampleCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	sampleCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	sampleCam->setRenderOrder(osg::Camera::POST_RENDER, 4);
	sampleCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	sampleCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	sampleCam->attach(osg::Camera::COLOR_BUFFER, sampleImg.get());

	sampleCam->addChild(quad1.get());

	root->addChild(sampleCam.get());

}

osg::Node* LocalViewer::loadSaliencyModel()
{

#ifdef ISTOG05
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "armadillo_scale_ms.ply");//05年TOG
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "bunny_scale_ms.ply");
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "dragon_scale_ms.ply");
	osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "david_scale_ms.ply");
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "maxplanck_scale_ms.ply");
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "bunny_scale_ms_test.ply");
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH +""+ model_ms);

#else
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "armadillo_gbms.ply");//胡三峰的方法
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "horse_scale_gbms.ply");
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "bunny_scale_gbms.ply");
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "dragon_scale_gbms.ply");
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "isis_scale_gbms.ply");
	osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "david_scale_gbms.ply");
	//osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(DATAPATH + "maxplanck_scale_gbms.ply");
#endif
	cout<<"加载显著度"<<endl;
	return node.release();
}
void LocalViewer::createSaliencyMap()
{
	_saliencyCamera=new osg::Camera;

	_saliencymap=new osg::Image;
	_saliencymap->allocateImage(_width,_height,1,GL_RGB,GL_UNSIGNED_BYTE);
	

	_saliencyCamera->setViewport(0, 0, _width, _height);
	_saliencyCamera->setClearColor(BLACK);
	_saliencyCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_saliencyCamera->setRenderOrder(osg::Camera::PRE_RENDER, 0);
	_saliencyCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	_saliencyCamera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	/*_saliencyCamera->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\saliency.vert", 
		DATAPATH + "Myshaders\\saliency.frag")
		);*/
	//_imageCam->getOrCreateStateSet()->setAttributeAndModes();
	_saliencyCamera->attach(osg::Camera::COLOR_BUFFER, _saliencymap.get());
	_saliencyCamera->addChild(loadSaliencyModel());

	root->addChild(_saliencyCamera.get());
	
}
void LocalViewer::scenePrepare()
{
	root = new osg::Group;

	if(!_modelNode.valid()) _modelNode = loadModel();

	osgUtil::Optimizer optimizer;       
	optimizer.optimize(_modelNode.get());

	/*osg::BoundingSphere bs=_modelNode->getBound();
	osg::BoundingBox bb;
	bb.expandBy(bs);
	for(int i=7;i>=0;i--)
	{
		osg::ref_ptr<osg::LightSource> ls=new osg::LightSource;
		osg::ref_ptr<osg::Light> lt=new osg::Light;
		lt->setLightNum(i);
		lt->setPosition(osg::Vec4(bb.corner(i),1.0));
		lt->setAmbient(osg::Vec4(0.5,0.5,0.5,1.0));
		lt->setDiffuse(osg::Vec4(1.0,1.0,1.0,1.0));
		ls->setLight(lt);
		ls->setStateSetModes(*_masterCam->getOrCreateStateSet(),osg::StateAttribute::ON);
	}*/



	//osg::ref_ptr<osg::MatrixTransform> scale_node=new osg::MatrixTransform;
	//scale_node->setMatrix(osg::Matrix::scale(100,100,100));
	//scale_node->addChild(_modelNode);
	//root->addChild(scale_node.get());
	root->addChild(_modelNode.get());

	//root->addChild(scale_node.get());
	/*(1)
	///2014/2/20  realDepth
	_realDepthImage=new osg::Image;
	_realDepthImage->allocateImage(_width, _height, 1, GL_LUMINANCE, GL_FLOAT);
	_realDepthImage->setInternalTextureFormat(GL_LUMINANCE32F_ARB);



	_realDepthCam = new osg::Camera;
	_realDepthCam->setViewport(0, 0, _width, _height);
	_realDepthCam->setClearColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
	_realDepthCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_realDepthCam->setRenderOrder(osg::Camera::PRE_RENDER, 1);
	_realDepthCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	_realDepthCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	_realDepthCam->attach(osg::Camera::COLOR_BUFFER, _realDepthImage.get());
	osg::ref_ptr<osg::Uniform> _ufar=new osg::Uniform("far",mFar);
	_realDepthCam->getOrCreateStateSet()->addUniform(_ufar.get());
	_realDepthCam->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\linearity_depth.vert", 
		DATAPATH + "Myshaders\\linear.frag")
		);
	_realDepthCam->addChild(_modelNode.get());
	_realDepthCam->setPreDrawCallback(new DepthCallback);
	root->addChild(_realDepthCam.get());*/
	
	///

 
 	_colorImg = new osg::Image;
	_edgeImg = new osg::Image;
	_edgeImg->allocateImage(_width, _height, 1, GL_LUMINANCE_ALPHA, GL_FLOAT);
	_edgeImg->setInternalTextureFormat(GL_LUMINANCE_ALPHA32F_ARB);
 	_colorImg->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);

	_imageTex = new osg::Texture2D;
	_imageTex->setTextureSize(_width, _height);
	_imageTex->setInternalFormat(GL_RGB);
	_imageTex->setSourceFormat(GL_RGB);
	_imageTex->setSourceType(GL_UNSIGNED_BYTE);
	_imageTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::LINEAR);
	_imageTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::LINEAR);
	_imageTex->setWrap(osg::Texture2D::WRAP_S,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	_imageTex->setWrap(osg::Texture2D::WRAP_T,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	_imageTex->setResizeNonPowerOfTwoHint(false);
 
 	_imageCam = new osg::Camera;
 	_imageCam->setViewport(0, 0, _width, _height);
 	_imageCam->setClearColor(WHITE);
 	_imageCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 	_imageCam->setRenderOrder(osg::Camera::PRE_RENDER, 0);
 	_imageCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	
	//2014/4/29
	
	/*_imageCam->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\phong.vert", 
		DATAPATH + "Myshaders\\phong.frag")
		*/
	
	//2014/4/29
	//_imageCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	//_imageCam->setViewMatrixAsLookAt(osg::Vec3(0,0,0),osg::Vec3(0,0,1),osg::Vec3(0,1,0));
	//_imageCam->setViewport(0,0,_width,_height);
	//_imageCam->setProjectionMatrixAsPerspective(30,0.75,10,500);
	//_imageCam->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	_imageCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	_imageCam->attach(osg::Camera::COLOR_BUFFER, _colorImg.get());
	

 	_imageCam->addChild(_modelNode.get());
 	root->addChild(_imageCam.get());
 
	_depthTex = new osg::Texture2D;
	_depthTex->setTextureSize(_width, _height);
	_depthTex->setInternalFormat(GL_LUMINANCE32F_ARB);
	_depthTex->setSourceFormat(GL_LUMINANCE);
	_depthTex->setSourceType(GL_FLOAT);
	_depthTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::NEAREST);
	_depthTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::NEAREST);
	_depthTex->setResizeNonPowerOfTwoHint(false);
	_depthTex->setWrap(osg::Texture2D::WRAP_S,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	_depthTex->setWrap(osg::Texture2D::WRAP_T,
		osg::Texture2D::CLAMP_TO_EDGE
		);

	_depthCam = new osg::Camera;
	_depthCam->setViewport(0, 0, _width, _height);
 	_depthCam->setClearColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
 	_depthCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 	_depthCam->setRenderOrder(osg::Camera::PRE_RENDER, 1);
	_depthCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
 	_depthCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
 	_depthCam->attach(osg::Camera::COLOR_BUFFER, _depthTex.get());
 	_depthCam->getOrCreateStateSet()->setAttributeAndModes(
 		createProgram(DATAPATH + "Myshaders\\depth.vert", 
 		DATAPATH + "Myshaders\\depth.frag")
 		);
 	_depthCam->addChild(_modelNode.get());
 	root->addChild(_depthCam.get());

	_depthImg = new osg::Image;
	_depthImg->allocateImage(_width, _height, 1, GL_LUMINANCE, GL_FLOAT);
	_depthImg->setInternalTextureFormat(GL_LUMINANCE32F_ARB);
	
	_depthCam2 = new osg::Camera;
	_depthCam2->setViewport(0, 0, _width, _height);
	_depthCam2->setClearColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
	_depthCam2->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_depthCam2->setRenderOrder(osg::Camera::PRE_RENDER, 2);
	_depthCam2->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	_depthCam2->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	_depthCam2->attach(osg::Camera::COLOR_BUFFER, _depthImg.get());
	_depthCam2->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\depth.vert", 
		DATAPATH + "Myshaders\\depth.frag")
		);
	_depthCam2->addChild(_modelNode.get());
	root->addChild(_depthCam2.get());

	//______________________________________//
	///方法的普适性证明
	createMSRDepth();
	createMSRDepthEdge();
	/// 
	//_______________________________________//
	createSaliencyMap();//计算显著度Map
	createEdgeRegion();
	createEdgeRegionShowCam();
	createPoissionSample();//2014/3/21
	/*createBUSampleCam();//3/25*/
	
	_edgeTex = new osg::Texture2D;
	_edgeTex->setTextureSize(_width, _height);
	_edgeTex->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
	_edgeTex->setSourceFormat(GL_LUMINANCE_ALPHA);
	_edgeTex->setSourceType(GL_FLOAT);
	_edgeTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::NEAREST);
	_edgeTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::NEAREST);
	_edgeTex->setWrap(osg::Texture2D::WRAP_S,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	_edgeTex->setWrap(osg::Texture2D::WRAP_T,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	_edgeTex->setResizeNonPowerOfTwoHint(false);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, _depthTex.get());
	
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\detect.vert", 
		DATAPATH + "Myshaders\\detect1.frag")
		);
	/*
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\esample.vert", 
		DATAPATH + "Myshaders\\esample.frag"));//2014/3/21*/
	osg::ref_ptr<osg::Uniform> texDepth = new osg::Uniform("texDepth", 0);
	osg::ref_ptr<osg::Uniform> texSize = new osg::Uniform("texSize", osg::Vec2(float(_width), float(_height)));
	osg::ref_ptr<osg::Uniform> isShow = new osg::Uniform("isShow", false);
	osg::ref_ptr<osg::Uniform> dNear = new osg::Uniform("near", mNear);
	osg::ref_ptr<osg::Uniform> dFar = new osg::Uniform("far", mFar);
	//osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("tex",leveltexture[LEVEL-1][NGROUP-1].get());//2014/3/21
	//add laplace modul;
	quad1->getOrCreateStateSet()->addUniform(texDepth.get());
	quad1->getOrCreateStateSet()->addUniform(texSize.get());
	quad1->getOrCreateStateSet()->addUniform(isShow.get());
	quad1->getOrCreateStateSet()->addUniform(dNear.get());
	quad1->getOrCreateStateSet()->addUniform(dFar.get());
	//quad1->getOrCreateStateSet()->addUniform(tex.get());//2014/3/21

	_detectCam = new osg::Camera;
	_detectCam->setViewport(0, 0, _width, _height);
	_detectCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	_detectCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	_detectCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	_detectCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_detectCam->setRenderOrder(osg::Camera::PRE_RENDER, 3);
	_detectCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	_detectCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	_detectCam->attach(osg::Camera::COLOR_BUFFER, _edgeTex.get());
	
	_detectCam->addChild(quad1.get());

	root->addChild(_detectCam.get());
	

	
	
	/*
	_depthImg = new osg::Image;
	_depthImg->allocateImage(_width, _height, 1, GL_LUMINANCE, GL_FLOAT);
	_depthImg->setInternalTextureFormat(GL_LUMINANCE32F_ARB);
	*/

	osg::ref_ptr<ScreenQuad> quad2;
	quad2 = new ScreenQuad;
	quad2->getOrCreateStateSet()->setTextureAttributeAndModes(0, _depthTex.get());
	quad2->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\nogriddetect.vert", 
		DATAPATH + "Myshaders\\nogriddetect.frag")
		);
	//add laplace modul;
	quad2->getOrCreateStateSet()->addUniform(texDepth.get());
	quad2->getOrCreateStateSet()->addUniform(texSize.get());
	quad2->getOrCreateStateSet()->addUniform(isShow.get());
	quad2->getOrCreateStateSet()->addUniform(dNear.get());
	quad2->getOrCreateStateSet()->addUniform(dFar.get());

	_detectCam2 = new osg::Camera;
	_detectCam2->setViewport(0, 0, _width, _height);
	_detectCam2->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	_detectCam2->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	_detectCam2->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	_detectCam2->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//_detectCam2->setRenderOrder(osg::Camera::POST_RENDER, 4);
	_detectCam2->setRenderOrder(osg::Camera::PRE_RENDER, 4);///2014/3/23
	_detectCam2->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	_detectCam2->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	_detectCam2->attach(osg::Camera::COLOR_BUFFER, _edgeImg.get());
	_detectCam2->setPostDrawCallback(new DepthCallback);//2014/3/2
	_detectCam2->addDescription("edgeImg");//2014/3/2
	_detectCam2->addChild(quad2.get());

	root->addChild(_detectCam2.get());

	int lowWidth = int(_width / _DsFactorX);
	int lowHight = int(_height / _DsFactorY);

	_lowColorImg = new osg::Image;
	_lowColorImg->allocateImage(lowWidth, lowHight, 1, GL_RGB, GL_UNSIGNED_BYTE);

	_imageCam2 = new osg::Camera;
	_imageCam2->setViewport(0, 0, lowWidth, lowHight);
	_imageCam2->setClearColor(BACKCOLOR);
	_imageCam2->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_imageCam2->setRenderOrder(osg::Camera::PRE_RENDER, 5);
	_imageCam2->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	_imageCam2->attach(osg::Camera::COLOR_BUFFER, _lowColorImg.get());

	_imageCam2->addChild(_modelNode.get());
	root->addChild(_imageCam2.get());


//  	osg::ref_ptr<osg::Node>  test = new osg::Node;
//  	test->getOrCreateStateSet()->setAttributeAndModes(
//  		createProgram(DATAPATH + "Myshaders\\psudoDepth.vert", 
//  		DATAPATH + "Myshaders\\psudoDepth.frag")
//  		);
//  
//  	root->addChild(test.get());
	//createEdgeRegion();
	//createEdgeRegionShowCam();

	/* 
	统计
	*/
	_staticEdgeImg = new osg::Image;
	_staticEdgeImg->allocateImage(_width, _height, 1, GL_LUMINANCE_ALPHA, GL_FLOAT);
	_staticEdgeImg->setInternalTextureFormat(GL_LUMINANCE_ALPHA32F_ARB);


	osg::ref_ptr<osg::Camera> _detectCam3 = new osg::Camera;
	_detectCam3->setViewport(0, 0, _width, _height);
	_detectCam3->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	_detectCam3->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	_detectCam3->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	_detectCam3->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//_detectCam2->setRenderOrder(osg::Camera::POST_RENDER, 4);
	_detectCam3->setRenderOrder(osg::Camera::PRE_RENDER, 4);///2014/3/23
	_detectCam3->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	_detectCam3->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	_detectCam3->attach(osg::Camera::COLOR_BUFFER,_staticEdgeImg.get());
	_detectCam3->setPostDrawCallback(new DepthCallback);//2014/3/2
	_detectCam3->addDescription("edgeImg");//2014/3/2
	_detectCam3->addChild(quad1.get());
	root->addChild(_detectCam3.get());

	createSampleCam();
	
	createDiffuseCameras(root.get());
	createDiffuseCamerasCmp(root.get()); //poisson采样扩散

	//createDiffuseCamerasCmpForShow(root.get()); //实验截图

	/*osg::ref_ptr<ScreenQuad> quad2;
	quad2 = new ScreenQuad;
	quad2->getOrCreateStateSet()->setTextureAttributeAndModes(0, _imageTex.get());
	quad2->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\test2.vert", 
		DATAPATH + "Myshaders\\test2.frag")
		);
	osg::ref_ptr<osg::Uniform> texColor = new osg::Uniform("texColor", 0);
	quad2->getOrCreateStateSet()->addUniform(texColor.get());
	quad2->getOrCreateStateSet()->addUniform(texSize.get());

	osg::ref_ptr<osg::Camera> testCam = new osg::Camera;
	testCam->setViewport(0, 0, _width, _height);
	testCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	testCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	testCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	testCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	testCam->setRenderOrder(osg::Camera::POST_RENDER, 80);
	testCam->addChild(quad2.get());
	root->addChild(testCam.get());*/

	//createTest(root.get());
	createTextShow(root);//深度图像压缩率的显示
	
	this->setSceneData(root.get());
}

osg::Program* LocalViewer::createProgram(const std::string vert, const std::string frag)
{
	osg::ref_ptr<osg::Program> program = new osg::Program;
	osg::ref_ptr<osg::Shader> vertShader = osgDB::readShaderFile(vert);
	osg::ref_ptr<osg::Shader> fragShader = osgDB::readShaderFile(frag);
	program->addShader(vertShader.get());
	program->addShader(fragShader.get());
	return program.release();
}

void LocalViewer::createMasterCamera()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;

	traits->x = 100;
	traits->y = 100;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	_masterCam = new osg::Camera;
	//2014/5/1
	this->setLightingMode(osg::View::SKY_LIGHT);

	this->getLight()->setPosition(lightP);
	//osg::ref_ptr<osg::Light> light=new osg::Light(3);
	
	//David头像正面
	//this->getLight()->setPosition(osg::Vec4(0,0,-1,0));

	//armadillo
	//this->getLight()->setPosition(osg::Vec4(0,1,0,0));

	//bunny
	//this->getLight()->setPosition(osg::Vec4(0,-1,0,0));

	//dragon
	//this->getLight()->setPosition(osg::Vec4(-1,0,0,0));

	//maxplank
	//this->getLight()->setPosition(osg::Vec4(0,0,-1,0));
	//
	_masterCam->setGraphicsContext(gc.get());
	_masterCam->setViewport(0, 0, traits->width, traits->height);
	_masterCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//_masterCam->setClearColor(BACKCOLOR);
	//_masterCam->setClearColor(osg::Vec4(1,1,1,0));
	_masterCam->setClearColor(osg::Vec4(75.0/256,87.0/256,99.0/256,0));
	_masterCam->setProjectionMatrixAsPerspective( 45.0f, (double)(traits->width)/(double)(traits->height), mNear, mFar);
	
	_masterCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR); //2014/4/20
	
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	_masterCam->setDrawBuffer(buffer);
	_masterCam->setReadBuffer(buffer);

	_masterCam->setPostDrawCallback(new getNearFarCallback);

	this->setCamera(_masterCam.get());
}

void LocalViewer::createShowCamera()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;

	traits->x = 200;
	traits->y = 200;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	_showSlaveCam = new osg::Camera;
	_showSlaveCam->setGraphicsContext(gc.get());
	_showSlaveCam->setViewport(0, 0, traits->width, traits->height);
	_showSlaveCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_showSlaveCam->setProjectionMatrixAsPerspective( 45.0f, (double)(traits->width)/(double)(traits->height), mNear,mFar);
	_showSlaveCam->setClearColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	_showSlaveCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	_showSlaveCam->setDrawBuffer(buffer);
	_showSlaveCam->setReadBuffer(buffer);
	
	_showSlaveCam->setRenderOrder(osg::Camera::POST_RENDER, 50);

 	/*if(!_modelNode.valid()) _modelNode = loadModel();
 	osg::ref_ptr<osg::Group> tmp = new osg::Group;
 	tmp->getOrCreateStateSet()->setAttributeAndModes(
 		createProgram(DATAPATH + "Myshaders\\depth.vert", 
 		DATAPATH + "Myshaders\\depth.frag")
 		);
 	tmp->addChild(_modelNode);
  	_showSlaveCam->addChild(tmp.get());*/
	osg::ref_ptr<osg::Texture2D> showDepthTex = new osg::Texture2D;

	
	showDepthTex->setImage(_depthImg.get());
	//showDepthTex->setImage(_msdepth.get());
	
	/*
	*目的：块状结果截图
	*时间：2013年12月15号
	*/
	//showDepthTex->setImage(_dfDepthImg.get());


	showDepthTex->setInternalFormat(GL_LUMINANCE32F_ARB);
	showDepthTex->setResizeNonPowerOfTwoHint(false);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, showDepthTex.get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\detect.vert", 
		DATAPATH + "Myshaders\\detect1.frag")
		);
	
	//quad1->getOrCreateStateSet()->setAttributeAndModes(
	//	createProgram(DATAPATH + "Myshaders\\detect.vert", 
	//	DATAPATH + "Myshaders\\detect2.frag")
	//	);

	/*
	*目的：块状结果截图
	*时间：2013年12月15号
	*/
 	//quad1->getOrCreateStateSet()->setAttributeAndModes(
		//createProgram(DATAPATH + "Myshaders\\detect.vert", 
 		//DATAPATH + "Myshaders\\detect2.frag")
 		//);


	osg::ref_ptr<osg::Uniform> texDepth = new osg::Uniform("texDepth", 0);
	osg::ref_ptr<osg::Uniform> texSize = new osg::Uniform("texSize", osg::Vec2(float(_width), float(_height)));
	osg::ref_ptr<osg::Uniform> isShow = new osg::Uniform("isShow", mIsShow);
	osg::ref_ptr<osg::Uniform> dNear = new osg::Uniform("near", mNear);
	osg::ref_ptr<osg::Uniform> dFar = new osg::Uniform("far", mFar);
	//add laplace modul;
	quad1->getOrCreateStateSet()->addUniform(texDepth.get());
	quad1->getOrCreateStateSet()->addUniform(texSize.get());
	quad1->getOrCreateStateSet()->addUniform(isShow.get());
	quad1->getOrCreateStateSet()->addUniform(dNear.get());
	quad1->getOrCreateStateSet()->addUniform(dFar.get());

	osg::ref_ptr<osg::Camera> showDctCam = new osg::Camera;
	showDctCam->setViewport(0, 0, _width, _height);
	showDctCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showDctCam->setClearColor(BACKCOLOR);
	showDctCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	//showDctCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showDctCam->setClearColor(osg::Vec4(75.0/256,87.0/256,99.0/256,0.0));
	showDctCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showDctCam->setRenderOrder(osg::Camera::POST_RENDER, 51);
	showDctCam->addChild(quad1.get());
	
	_showSlaveCam->addChild(showDctCam.get());
	//_showSlaveCam->addChild(camera.get());
	this->addSlave(_showSlaveCam.get(), false);

	
}


void LocalViewer::createWarpingCamera()
{

	double ss = tan(45.0 / 180.0 * osg::PI) * 0.8;
	ss = atan(ss);

	ss = ss / osg::PI * 180;
	std::cout << std::fixed << ss << std::endl;


	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;

	traits->x = 300;
	traits->y = 300;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="基于原始深度的重建";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	_warpingCam = new osg::Camera;
	_warpingCam->setGraphicsContext(gc.get());
	
	_warpingCam->setViewport(0, 0, traits->width, traits->height);
	_warpingCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_warpingCam->setRenderOrder(osg::Camera::POST_RENDER, 1000);
	//_warpingCam->setClearColor(CL1);
	_warpingCam->setClearColor(GREYCOLOR);

	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	_warpingCam->setDrawBuffer(buffer);
	_warpingCam->setReadBuffer(buffer);

	_warpingCam->setProjectionMatrixAsPerspective(45.0, (double)(traits->width)/(double)(traits->height), mNear,mFar+100);
	_warpingCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

	this->addSlave(_warpingCam.get(),false);

	traits->x = 350;
	traits->y = 350;
	traits->windowName="基于规则网格采样的重建";
	gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	_warpingCmpCam = new osg::Camera;
	_warpingCmpCam->setGraphicsContext(gc.get());
	
	_warpingCmpCam->setViewport(0, 0, traits->width, traits->height);
	_warpingCmpCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_warpingCmpCam->setRenderOrder(osg::Camera::POST_RENDER, 1005);
	//_warpingCmpCam->setClearColor(CL1);
	_warpingCmpCam->setClearColor(GREYCOLOR);

	_warpingCmpCam->setDrawBuffer(buffer);
	_warpingCmpCam->setReadBuffer(buffer);

	_warpingCmpCam->setProjectionMatrixAsPerspective(45.0, (double)(traits->width)/(double)(traits->height), mNear,mFar + 100);
	_warpingCmpCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

	this->addSlave(_warpingCmpCam.get(), false);



	traits->x = 400;
	traits->y = 400;
	traits->windowName="基于泊松碟随机采样的重建";
	gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	_warpingpoissoncam = new osg::Camera;
	_warpingpoissoncam->setGraphicsContext(gc.get());

	_warpingpoissoncam->setViewport(0, 0, traits->width, traits->height);
	_warpingpoissoncam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_warpingpoissoncam->setRenderOrder(osg::Camera::POST_RENDER, 1010);
	//_warpingCmpCam->setClearColor(CL1);
	_warpingpoissoncam->setClearColor(GREYCOLOR);

	_warpingpoissoncam->setDrawBuffer(buffer);
	_warpingpoissoncam->setReadBuffer(buffer);

	_warpingpoissoncam->setProjectionMatrixAsPerspective(45.0, (double)(traits->width)/(double)(traits->height), mNear,mFar + 100);
	_warpingpoissoncam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

	this->addSlave(_warpingpoissoncam.get(), false);

	//_warpingpoissoncam->addChild(camera);

	//可见顶点的显示//2014.2.19
	/*
	traits->x=400;
	traits->y=400;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="可见几何顶点";
	gc=osg::GraphicsContext::createGraphicsContext(traits.get());
	*/

}

void LocalViewer::createLowToHighCamera()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;

	traits->x = 300;
	traits->y = 300;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	_lowToHighCam = new osg::Camera;
	_lowToHighCam->setGraphicsContext(gc.get());
	_lowToHighCam->setViewport(0, 0, traits->width, traits->height);
	_lowToHighCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_lowToHighCam->setClearColor(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));

	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	_lowToHighCam->setDrawBuffer(buffer);
	_lowToHighCam->setReadBuffer(buffer);

	_lowToHighCam->setRenderOrder(osg::Camera::POST_RENDER, 60);

	osg::ref_ptr<osg::Texture2D> depthTex = new osg::Texture2D;
	depthTex->setImage(_dfDepthImg.get());
	depthTex->setInternalFormat(GL_LUMINANCE32F_ARB);
	depthTex->setResizeNonPowerOfTwoHint(false);

	osg::ref_ptr<osg::Texture2D> lowColorTex = new osg::Texture2D;
	lowColorTex->setImage(_lowColorImg.get());
	lowColorTex->setInternalFormat(GL_RGB);
	lowColorTex->setResizeNonPowerOfTwoHint(false);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, depthTex.get());
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(1, lowColorTex.get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\lowToHigh.vert", 
		DATAPATH + "Myshaders\\lowToHigh.frag")
		);
	osg::ref_ptr<osg::Uniform> texDepth = new osg::Uniform("texDepth", 0);
	osg::ref_ptr<osg::Uniform> texLow = new osg::Uniform("texLow", 1);
	osg::ref_ptr<osg::Uniform> texSize = new osg::Uniform("texSize", osg::Vec2(float(_width), float(_height)));

	quad1->getOrCreateStateSet()->addUniform(texDepth.get());
	quad1->getOrCreateStateSet()->addUniform(texLow.get());
	quad1->getOrCreateStateSet()->addUniform(texSize.get());

	osg::ref_ptr<osg::Camera> showLTHCam = new osg::Camera;
	showLTHCam->setViewport(0, 0, _width, _height);
	showLTHCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showLTHCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showLTHCam->setClearColor(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
	showLTHCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showLTHCam->setRenderOrder(osg::Camera::POST_RENDER, 61);
	showLTHCam->addChild(quad1.get());

	_lowToHighCam->addChild(showLTHCam.get());

	this->addSlave(_lowToHighCam.get(), false);
}

osg::Camera* LocalViewer::createCamera(int leftTopX, int leftTopY,int width, int height)
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;

	traits->x = leftTopX;
	traits->y = leftTopY;
	traits->width = width;
	traits->height = height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	osg::ref_ptr<osg::Camera> cam = new osg::Camera;
	cam->setGraphicsContext(gc.get());
	cam->setViewport(0, 0, traits->width, traits->height);
	cam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	cam->setClearColor(BACKCOLOR);

	//cam->setProjectionMatrixAsPerspective(45, double(width) / height, 1.0, 10000.0);
	//cam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	cam->setDrawBuffer(buffer);
	cam->setReadBuffer(buffer);

	return cam.release();
}
void LocalViewer::createShowSampleCam1()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 350;
	traits->y = 350;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="实验截图1";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (dsTex[1]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\sampleshow.vert", 
		DATAPATH + "Myshaders\\sampleshow.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("tex",0);
	osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(w.get());
	showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());

}
void LocalViewer::createShowSampleCam2()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 360;
	traits->y = 360;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="实验截图2";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (dsTex[2]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\sampleshow.vert", 
		DATAPATH + "Myshaders\\sampleshow.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("tex",0);
	osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(w.get());
	showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());

}
void LocalViewer::createShowSampleCam3()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 370;
	traits->y = 370;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="实验截图3";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (dsTex[3]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\sampleshow.vert", 
		DATAPATH + "Myshaders\\sampleshow.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("tex",0);
	osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(w.get());
	showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());

}
void LocalViewer::createShowSampleCam4()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 380;
	traits->y = 380;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="实验截图4";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (dsTex[4]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\sampleshow.vert", 
		DATAPATH + "Myshaders\\sampleshow.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("tex",0);
	osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(w.get());
	showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());

}
void LocalViewer::createShowSampleCam5()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 400;
	traits->y = 400;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="实验截图5";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (dsTex[5]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\sampleshow.vert", 
		DATAPATH + "Myshaders\\sampleshow.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("tex",0);
	osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(w.get());
	showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());

}
void LocalViewer::createshowSmooth5()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 410;
	traits->y = 410;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="smooth实验截图5";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (smTex[5]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\detect.vert", 
		DATAPATH + "Myshaders\\detect2.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("texDepth",0);
	osg::ref_ptr<osg::Uniform> texsize=new osg::Uniform("texSize",osg::Vec2(_width,_height));
	//osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	//osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(texsize.get());
	//showcam->getOrCreateStateSet()->addUniform(w.get());
	//showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());
}
void LocalViewer::createshowSmooth4()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 420;
	traits->y = 420;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="smooth实验截图4";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (smTex[4]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\detect.vert", 
		DATAPATH + "Myshaders\\detect2.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("texDepth",0);
	osg::ref_ptr<osg::Uniform> texsize=new osg::Uniform("texSize",osg::Vec2(_width,_height));
	//osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	//osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(texsize.get());
	//showcam->getOrCreateStateSet()->addUniform(w.get());
	//showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());
}
void LocalViewer::createshowSmooth3()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 430;
	traits->y = 430;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="smooth实验截图3";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (smTex[3]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\detect.vert", 
		DATAPATH + "Myshaders\\detect2.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("texDepth",0);
	osg::ref_ptr<osg::Uniform> texsize=new osg::Uniform("texSize",osg::Vec2(_width,_height));
	//osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	//osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(texsize.get());
	//showcam->getOrCreateStateSet()->addUniform(w.get());
	//showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());
}
void LocalViewer::createshowSmooth2()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 440;
	traits->y = 440;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="smooth实验截图2";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (smTex[2]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\detect.vert", 
		DATAPATH + "Myshaders\\detect2.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("texDepth",0);
	osg::ref_ptr<osg::Uniform> texsize=new osg::Uniform("texSize",osg::Vec2(_width,_height));
	//osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	//osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(texsize.get());
	//showcam->getOrCreateStateSet()->addUniform(w.get());
	//showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());
}
void LocalViewer::createshowSmooth1()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 410;
	traits->y = 410;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="smooth实验截图1";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (smTex[1]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\detect.vert", 
		DATAPATH + "Myshaders\\detect2.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("texDepth",0);
	osg::ref_ptr<osg::Uniform> texsize=new osg::Uniform("texSize",osg::Vec2(_width,_height));
	//osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	//osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(texsize.get());
	//showcam->getOrCreateStateSet()->addUniform(w.get());
	//showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());
}
void LocalViewer::createshowInterpolation5()
{

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 500;
	traits->y = 500;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="5diffusion实验截图";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (dfTex[4]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\detect.vert", 
		DATAPATH + "Myshaders\\detect2.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("texDepth",0);
	osg::ref_ptr<osg::Uniform> texsize=new osg::Uniform("texSize",osg::Vec2(_width,_height));
	//osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	//osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(texsize.get());
	//showcam->getOrCreateStateSet()->addUniform(w.get());
	//showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());

}
void LocalViewer::createshowInterpolation4()
{

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 500;
	traits->y = 500;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="4diffusion实验截图4";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (dfTex[3]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\detect.vert", 
		DATAPATH + "Myshaders\\detect2.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("texDepth",0);
	osg::ref_ptr<osg::Uniform> texsize=new osg::Uniform("texSize",osg::Vec2(_width,_height));
	//osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	//osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(texsize.get());
	//showcam->getOrCreateStateSet()->addUniform(w.get());
	//showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());

}
void LocalViewer::createshowInterpolation3()
{

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 500;
	traits->y = 500;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="3diffusion实验截图3";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (dfTex[2]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\detect.vert", 
		DATAPATH + "Myshaders\\detect2.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("texDepth",0);
	osg::ref_ptr<osg::Uniform> texsize=new osg::Uniform("texSize",osg::Vec2(_width,_height));
	//osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	//osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(texsize.get());
	//showcam->getOrCreateStateSet()->addUniform(w.get());
	//showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());

}
void LocalViewer::createshowInterpolation2()
{

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 500;
	traits->y = 500;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="2diffusion实验截图2";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (dfTex[1]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\detect.vert", 
		DATAPATH + "Myshaders\\detect2.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("texDepth",0);
	osg::ref_ptr<osg::Uniform> texsize=new osg::Uniform("texSize",osg::Vec2(_width,_height));
	//osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	//osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(texsize.get());
	//showcam->getOrCreateStateSet()->addUniform(w.get());
	//showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());

}
void LocalViewer::createshowInterpolation1()
{

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 500;
	traits->y = 500;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="1diffusion实验截图1";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (dfTex[0]).get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\detect.vert", 
		DATAPATH + "Myshaders\\detect2.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("texDepth",0);
	osg::ref_ptr<osg::Uniform> texsize=new osg::Uniform("texSize",osg::Vec2(_width,_height));
	//osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	//osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(texsize.get());
	//showcam->getOrCreateStateSet()->addUniform(w.get());
	//showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());

}
void LocalViewer::createPostDeal()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	//img=new osg::Image;
	//img->allocateImage(_width,_height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	traits->x = 500;
	traits->y = 500;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="postDeal";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	osg::ref_ptr<osg::Camera> showcam=new osg::Camera;

	showcam->setGraphicsContext(gc.get());
	GLenum buffer =gc->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	showcam->setDrawBuffer(buffer);
	showcam->setReadBuffer(buffer);

	osg::ref_ptr<osg::Texture2D> colorTex=new osg::Texture2D;
	colorTex->setImage(_colorImg.get());
	colorTex->setInternalFormat(GL_RGB);
	colorTex->setResizeNonPowerOfTwoHint(false);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, (dfTex[0]).get());
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(1,colorTex.get());
	//quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0,(buleveltexture[BLEVEL-ULEVEL][BUNGROUP-1]).get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\postdeal.vert", 
		DATAPATH + "Myshaders\\postdeal.frag")
		);
	osg::ref_ptr<osg::Uniform> tex=new osg::Uniform("texDepth",0);
	osg::ref_ptr<osg::Uniform> texsize=new osg::Uniform("texSize",osg::Vec2(_width,_height));
	osg::ref_ptr<osg::Uniform> color=new osg::Uniform("color",1);
	//osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	//osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	showcam->getOrCreateStateSet()->addUniform(tex.get());
	showcam->getOrCreateStateSet()->addUniform(texsize.get());
	showcam->getOrCreateStateSet()->addUniform(color.get());
	//showcam->getOrCreateStateSet()->addUniform(w.get());
	//showcam->getOrCreateStateSet()->addUniform(h.get());

	showcam->setViewport(0, 0, _width, _height);
	showcam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	showcam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	showcam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	showcam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	showcam->setRenderOrder(osg::Camera::POST_RENDER, 300);
	//showpoissonsamplecam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//showpoissonsamplecam->attach(osg::Camera::COLOR_BUFFER,img.get());
	showcam->addChild(quad1.get());

	this->addSlave(showcam.get(), false);
	//root->addChild(showpoissonsamplecam.get());
}
void LocalViewer::createDiffuseCamerasCmpForShow(osg::Group* root)//实验截图
{
	osg::ref_ptr<osg::Camera> realDepthCam;
	osg::ref_ptr<osg::Texture2D> realDepthTex;
	osg::ref_ptr<ScreenQuad> realDepthQuad;
	realDepthQuad = new ScreenQuad;
	realDepthQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0, _edgeTex.get());
	realDepthQuad->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\realDepth.vert", 
		DATAPATH + "Myshaders\\realDepth.frag")
		);

	//2014.2.19
	//std::cout<<DATAPATH+"Myshaders\\realDepth.vert"<<std::endl;
	//
	osg::ref_ptr<osg::Uniform> pDepth = new osg::Uniform("pDepth", 0);
	osg::ref_ptr<osg::Uniform> size = new osg::Uniform("size", osg::Vec2(float(_width), float(_height)));
	osg::ref_ptr<osg::Uniform> near = new osg::Uniform("near", float(mNear));
	osg::ref_ptr<osg::Uniform> far = new osg::Uniform("far", float(mFar));
	
	near->setUpdateCallback(new updataNear(this));
	far->setUpdateCallback(new updataFar(this));

	realDepthQuad->getOrCreateStateSet()->addUniform(pDepth.get());
	realDepthQuad->getOrCreateStateSet()->addUniform(size.get());
	realDepthQuad->getOrCreateStateSet()->addUniform(near.get());
	realDepthQuad->getOrCreateStateSet()->addUniform(far.get());

	realDepthTex = new osg::Texture2D;
	realDepthTex->setTextureSize(_width,_height);
	realDepthTex->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
	realDepthTex->setSourceFormat(GL_LUMINANCE_ALPHA);
	realDepthTex->setSourceType(GL_FLOAT);
	realDepthTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::LINEAR);
	realDepthTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::LINEAR);
	realDepthTex->setResizeNonPowerOfTwoHint(false);


	realDepthCam = new osg::Camera;
	realDepthCam->setViewport(0, 0, _width, _height);
	realDepthCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	realDepthCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	realDepthCam->setClearColor(osg::Vec4(1.0f, 1.0f, 1.0f, 0.0f));
	realDepthCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	realDepthCam->setRenderOrder(osg::Camera::POST_RENDER, 1);
	realDepthCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	realDepthCam->attach(osg::Camera::COLOR_BUFFER, realDepthTex.get());

	realDepthCam->setPreDrawCallback(new getProcessTime);
	realDepthCam->addChild(realDepthQuad.get());
	root->addChild(realDepthCam);

	osg::ref_ptr<osg::Camera> dsCam[5];
	osg::ref_ptr<osg::Camera> edsCam[5];
	//osg::ref_ptr<osg::Texture2D> dsTex[6];
	
	
	

	int width[6], height[6];

	osg::ref_ptr<osg::Program> dsProg = createProgram(DATAPATH+"Myshaders\\downSample.vert", 
		DATAPATH + "Myshaders\\downSample.frag");

	osg::ref_ptr<osg::Program> edsProg=createProgram(DATAPATH+"Myshaders\\sampleedge.vert",
		DATAPATH+"Myshaders\\sampleedge.frag");
	//直接对非线性的Zbuffer进行diffuse;
	//dsTex[0] = _edgeTex;//2014/2/25
	//dsTex[0]=stex;
	//dsTex[0]=realDepthTex;
	dsTex[0] = sampleTex;//2014/3/20
	int i;
	int t1 = 5;
	//int t1 = 12;//2014/3/20
	width[0] = _width;
	height[0] = _height;

	for(i = 0;  i < t1; ++i)
	{
		osg::ref_ptr<ScreenQuad> quad;
		quad = new ScreenQuad;
		quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dsTex[i].get());
		quad->getOrCreateStateSet()->setAttributeAndModes(dsProg.get());

		osg::ref_ptr<osg::Uniform> preTex = new osg::Uniform("preTex", 0);
		osg::ref_ptr<osg::Uniform> preSize = new osg::Uniform("preSize", osg::Vec2(float(width[i]), float(height[i])));

		quad->getOrCreateStateSet()->addUniform(preTex.get());
		quad->getOrCreateStateSet()->addUniform(preSize.get());

		width[i + 1] = (width[i] + 1) / 2;
		height[i + 1] = (height[i] + 1) /2;

		dsImg[i + 1]=new osg::Image;
		dsImg[i + 1]->allocateImage(width[i+1],height[i+1], 1, GL_LUMINANCE_ALPHA, GL_FLOAT);//3/31
		dsImg[i + 1]->setInternalTextureFormat(GL_LUMINANCE_ALPHA32F_ARB);//3/31
		dsTex[i + 1] = new osg::Texture2D;
		dsTex[i + 1]->setTextureSize(width[i + 1], height[i + 1]);
		dsTex[i + 1]->setImage((dsImg[i+1]).get());
		dsTex[i + 1]->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
		dsTex[i + 1]->setSourceFormat(GL_LUMINANCE_ALPHA);
		dsTex[i + 1]->setSourceType(GL_FLOAT);
		dsTex[i + 1]->setFilter(osg::Texture2D::MIN_FILTER,
			osg::Texture2D::NEAREST);
		dsTex[i + 1]->setFilter(osg::Texture2D::MAG_FILTER,
			osg::Texture2D::NEAREST);
		dsTex[i + 1]->setWrap(osg::Texture2D::WRAP_S,
			osg::Texture2D::CLAMP_TO_EDGE
			);
		dsTex[i + 1]->setWrap(osg::Texture2D::WRAP_T,
			osg::Texture2D::CLAMP_TO_EDGE
			);
		dsTex[i + 1]->setResizeNonPowerOfTwoHint(false);

		//2014/2/24
		osg::ref_ptr<ScreenQuad> equad;
		equad = new ScreenQuad;
		equad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dsTex[i].get());
		equad->getOrCreateStateSet()->setAttributeAndModes(edsProg.get());
		osg::ref_ptr<osg::Uniform> epreTex = new osg::Uniform("preTex", 0);
		osg::ref_ptr<osg::Uniform> epreSize = new osg::Uniform("preSize", osg::Vec2(float(width[i]), float(height[i])));
		equad->getOrCreateStateSet()->addUniform(epreTex.get());
		equad->getOrCreateStateSet()->addUniform(epreSize.get());

		edsTex[i + 1]=new osg::Texture2D;
		edsImg[i + 1]=new osg::Image;//3/31
		edsImg[i + 1]->allocateImage(width[i+1],height[i+1], 1, GL_LUMINANCE_ALPHA, GL_FLOAT);//3/31
		edsImg[i + 1]->setInternalTextureFormat(GL_LUMINANCE_ALPHA32F_ARB);//3/31
		edsTex[i + 1]->setTextureSize(width[i+1],height[i+1]);
		edsTex[i + 1]->setImage((edsImg[i+1]).get());//3/31
		edsTex[i + 1]->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
		edsTex[i + 1]->setSourceFormat(GL_LUMINANCE_ALPHA);
		edsTex[i + 1]->setSourceType(GL_FLOAT);
		edsTex[i + 1]->setFilter(osg::Texture2D::MIN_FILTER,
			osg::Texture2D::NEAREST);
		edsTex[i + 1]->setFilter(osg::Texture2D::MAG_FILTER,
			osg::Texture2D::NEAREST);
		edsTex[i + 1]->setWrap(osg::Texture2D::WRAP_S,
			osg::Texture2D::CLAMP_TO_EDGE
			);
		edsTex[i + 1]->setWrap(osg::Texture2D::WRAP_T,
			osg::Texture2D::CLAMP_TO_EDGE
			);
		edsTex[i + 1]->setResizeNonPowerOfTwoHint(false);

		//
		dsCam[i] = new osg::Camera;
		dsCam[i]->setViewport(0, 0, width[i + 1], height[i + 1]);
		dsCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
		dsCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
		dsCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
		dsCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		dsCam[i]->setRenderOrder(osg::Camera::POST_RENDER, i + 2);
		dsCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
		dsCam[i]->attach(osg::Camera::COLOR_BUFFER, dsImg[i + 1].get());
		dsCam[i]->addChild(quad.get());

		//2014/2/24
		edsCam[i] = new osg::Camera;
		edsCam[i]->setViewport(0, 0, width[i + 1], height[i + 1]);
		edsCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
		edsCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
		edsCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
		edsCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		edsCam[i]->setRenderOrder(osg::Camera::POST_RENDER, i + 2);
		edsCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
		edsCam[i]->attach(osg::Camera::COLOR_BUFFER, edsImg[i + 1].get());
		edsCam[i]->addChild(equad.get());
		//
		root->addChild(dsCam[i].get());
		//root->addChild(edsCam[i].get());
	}


	osg::ref_ptr<osg::Camera> dfCam[5];
		osg::ref_ptr<osg::Camera> smCam[5];
		//osg::ref_ptr<osg::Texture2D> dfTex[6];
		//osg::ref_ptr<osg::Texture2D> smTex[6];

		osg::ref_ptr<osg::Program> dfProg = createProgram(DATAPATH + "Myshaders\\diffuse.vert", 
			DATAPATH + "Myshaders\\diffuse.frag");
		
		//这里smooth 也有两个版本
		
		osg::ref_ptr<osg::Program> smProg = createProgram(DATAPATH + "Myshaders\\smooth1.vert", 
			DATAPATH + "Myshaders\\smooth1.frag");
			

		//osg::ref_ptr<osg::Program> smProg = createProgram(DATAPATH + "Myshaders\\esmooth.vert", 
			//DATAPATH + "Myshaders\\esmooth.frag");

		//osg::ref_ptr<osg::Program> smProg = createProgram(DATAPATH + "Myshaders\\ela.vert", 
		//DATAPATH + "Myshaders\\ela.frag");
	
		//smTex[5] = dsTex[5];
		dfTex[5] = dsTex[5];
	
		int t2 = 0;
		for(i = 4; i >= t2; --i)
		{
 			osg::ref_ptr<ScreenQuad> smQuad;
 			smQuad = new ScreenQuad;
 			smQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dfTex[i + 1].get());
			smQuad->getOrCreateStateSet()->setTextureAttributeAndModes(1, edsTex[i + 1].get());//2014/2/24
 			smQuad->getOrCreateStateSet()->setAttributeAndModes(smProg.get());
 	
 			osg::ref_ptr<osg::Uniform> tex = new osg::Uniform("tex", 0);
			osg::ref_ptr<osg::Uniform> eTex=new osg::Uniform("eTex", 1);//2014/2/24
			osg::ref_ptr<osg::Uniform> size = new osg::Uniform("size", osg::Vec2(float(width[i + 1]), float(height[i + 1])));
			osg::ref_ptr<osg::Uniform> near = new osg::Uniform("near", mNear);
			osg::ref_ptr<osg::Uniform> far = new osg::Uniform("far", mFar);
			osg::ref_ptr<osg::Uniform> imvp=new osg::Uniform("imvp",this->_iMVPW);//2/26
			osg::ref_ptr<osg::Uniform> level=new osg::Uniform("level",i+1);//2/26

			smQuad->getOrCreateStateSet()->addUniform(tex.get());
			smQuad->getOrCreateStateSet()->addUniform(size.get());
			smQuad->getOrCreateStateSet()->addUniform(near.get());
			smQuad->getOrCreateStateSet()->addUniform(far.get());
			smQuad->getOrCreateStateSet()->addUniform(eTex.get());//2014/2/24
			smQuad->getOrCreateStateSet()->addUniform(imvp.get());//2/26
			smQuad->getOrCreateStateSet()->addUniform(level.get());//2/26

			smImg[i+1]=new osg::Image;
			smImg[i+1]->allocateImage(width[i + 1], height[i + 1], 1, GL_LUMINANCE, GL_FLOAT);
			smImg[i+1]->setInternalTextureFormat(GL_LUMINANCE32F_ARB);
			smTex[i+1] = new osg::Texture2D;
			smTex[i+1]->setTextureSize(width[i + 1], height[i + 1]);
			smTex[i+1]->setImage((smImg[i+1]).get());
			smTex[i+1]->setInternalFormat(GL_LUMINANCE32F_ARB);
			smTex[i+1]->setSourceFormat(GL_LUMINANCE);
			smTex[i+1]->setSourceType(GL_FLOAT);
			smTex[i+1]->setFilter(osg::Texture2D::MIN_FILTER,
				osg::Texture2D::NEAREST);
			smTex[i+1]->setFilter(osg::Texture2D::MAG_FILTER,
				osg::Texture2D::NEAREST);
			smTex[i + 1]->setWrap(osg::Texture2D::WRAP_S,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			smTex[i + 1]->setWrap(osg::Texture2D::WRAP_T,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			smTex[i+1]->setResizeNonPowerOfTwoHint(false);
			//2014/2/22边缘保留的高斯双边滤波

			//
			smCam[i] = new osg::Camera;
			smCam[i]->setViewport(0, 0, width[i + 1], height[i + 1]);
			smCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
			smCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
			smCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
			smCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			smCam[i]->setRenderOrder(osg::Camera::POST_RENDER, t1 + 2 + 3* (4 - i));
			smCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
			smCam[i]->attach(osg::Camera::COLOR_BUFFER, smImg[i + 1].get());
			smCam[i]->addChild(smQuad.get());
			root->addChild(smCam[i].get());
	
			osg::ref_ptr<ScreenQuad> quad;
			quad = new ScreenQuad;

			/*
			*目的：块状结果截图
			*时间：2013年12月15号
			*有或者没有smooth，这里切换参数，smTex[i + 1]或者dfTex[i + 1]
			*/
			//2014/3/13
			
				
			
			//quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dfTex[i + 1].get());

		
			quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, smTex[i + 1].get());
			quad->getOrCreateStateSet()->setTextureAttributeAndModes(1, dsTex[i].get());
			quad->getOrCreateStateSet()->setAttributeAndModes(dfProg.get());
	
			osg::ref_ptr<osg::Uniform> posTex = new osg::Uniform("posTex", 0);
			osg::ref_ptr<osg::Uniform> curTex = new osg::Uniform("curTex", 1);
			osg::ref_ptr<osg::Uniform> curSize = new osg::Uniform("curSize", osg::Vec2(float(width[i]), float(height[i])));
	
			quad->getOrCreateStateSet()->addUniform(posTex.get());
			quad->getOrCreateStateSet()->addUniform(curTex.get());
			quad->getOrCreateStateSet()->addUniform(curSize.get());

			dfImg[i]=new osg::Image;
			dfImg[i]->allocateImage(width[i], height[i], 1, GL_LUMINANCE, GL_FLOAT);
			dfImg[i]->setInternalTextureFormat(GL_LUMINANCE32F_ARB);

			dfTex[i] = new osg::Texture2D;
			dfTex[i]->setTextureSize(width[i], height[i]);
			dfTex[i]->setImage(dfImg[i].get());
			dfTex[i]->setInternalFormat(GL_LUMINANCE32F_ARB);
			dfTex[i]->setSourceFormat(GL_LUMINANCE);
			dfTex[i]->setSourceType(GL_FLOAT);
			dfTex[i]->setFilter(osg::Texture2D::MIN_FILTER,
				osg::Texture2D::NEAREST);
			dfTex[i]->setFilter(osg::Texture2D::MAG_FILTER,
				osg::Texture2D::NEAREST);
			dfTex[i]->setWrap(osg::Texture2D::WRAP_S,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			dfTex[i]->setWrap(osg::Texture2D::WRAP_T,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			dfTex[i]->setResizeNonPowerOfTwoHint(false);
	
			dfCam[i] = new osg::Camera;
			dfCam[i]->setViewport(0, 0, width[i], height[i]);
			dfCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
			dfCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
			dfCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
			dfCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			dfCam[i]->setRenderOrder(osg::Camera::POST_RENDER, t1 + 2 + 3*(4 - i) + 1);
			dfCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
			dfCam[i]->attach(osg::Camera::COLOR_BUFFER, dfImg[i].get());//i==0
			dfCam[i]->addChild(quad.get());
			root->addChild(dfCam[i].get());
		}
}


void LocalViewer::createDiffuseCamerasCmp(osg::Group* root)
{
	osg::ref_ptr<osg::Camera> realDepthCam;
	osg::ref_ptr<osg::Texture2D> realDepthTex;
	osg::ref_ptr<ScreenQuad> realDepthQuad;
	realDepthQuad = new ScreenQuad;
	realDepthQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0, _edgeTex.get());
	realDepthQuad->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\realDepth.vert", 
		DATAPATH + "Myshaders\\realDepth.frag")
		);

	//2014.2.19
	//std::cout<<DATAPATH+"Myshaders\\realDepth.vert"<<std::endl;
	//
	osg::ref_ptr<osg::Uniform> pDepth = new osg::Uniform("pDepth", 0);
	osg::ref_ptr<osg::Uniform> size = new osg::Uniform("size", osg::Vec2(float(_width), float(_height)));
	osg::ref_ptr<osg::Uniform> near = new osg::Uniform("near", float(mNear));
	osg::ref_ptr<osg::Uniform> far = new osg::Uniform("far", float(mFar));
	
	near->setUpdateCallback(new updataNear(this));
	far->setUpdateCallback(new updataFar(this));

	realDepthQuad->getOrCreateStateSet()->addUniform(pDepth.get());
	realDepthQuad->getOrCreateStateSet()->addUniform(size.get());
	realDepthQuad->getOrCreateStateSet()->addUniform(near.get());
	realDepthQuad->getOrCreateStateSet()->addUniform(far.get());

	realDepthTex = new osg::Texture2D;
	realDepthTex->setTextureSize(_width,_height);
	realDepthTex->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
	realDepthTex->setSourceFormat(GL_LUMINANCE_ALPHA);
	realDepthTex->setSourceType(GL_FLOAT);
	realDepthTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::LINEAR);
	realDepthTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::LINEAR);
	realDepthTex->setResizeNonPowerOfTwoHint(false);


	realDepthCam = new osg::Camera;
	realDepthCam->setViewport(0, 0, _width, _height);
	realDepthCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	realDepthCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	realDepthCam->setClearColor(osg::Vec4(1.0f, 1.0f, 1.0f, 0.0f));
	realDepthCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	realDepthCam->setRenderOrder(osg::Camera::POST_RENDER, 1);
	realDepthCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	realDepthCam->attach(osg::Camera::COLOR_BUFFER, realDepthTex.get());

	realDepthCam->setPreDrawCallback(new getProcessTime);
	realDepthCam->addChild(realDepthQuad.get());
	root->addChild(realDepthCam);

	osg::ref_ptr<osg::Camera> dsCam[5];
	osg::ref_ptr<osg::Camera> edsCam[5];
	osg::ref_ptr<osg::Texture2D> dsTex[6];
	
	

	int width[6], height[6];

	osg::ref_ptr<osg::Program> dsProg = createProgram(DATAPATH+"Myshaders\\downSample.vert", 
		DATAPATH + "Myshaders\\downSample.frag");

	osg::ref_ptr<osg::Program> edsProg=createProgram(DATAPATH+"Myshaders\\sampleedge.vert",
		DATAPATH+"Myshaders\\sampleedge.frag");
	//直接对非线性的Zbuffer进行diffuse;
	//dsTex[0] = _edgeTex;//2014/2/25
	//dsTex[0]=stex;
	//dsTex[0]=realDepthTex;
	dsTex[0] = sampleTex;//2014/3/20
	int i;
	int t1 = 5;
	//int t1 = 12;//2014/3/20
	width[0] = _width;
	height[0] = _height;

	for(i = 0;  i < t1; ++i)
	{
		osg::ref_ptr<ScreenQuad> quad;
		quad = new ScreenQuad;
		quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dsTex[i].get());
		quad->getOrCreateStateSet()->setAttributeAndModes(dsProg.get());

		osg::ref_ptr<osg::Uniform> preTex = new osg::Uniform("preTex", 0);
		osg::ref_ptr<osg::Uniform> preSize = new osg::Uniform("preSize", osg::Vec2(float(width[i]), float(height[i])));

		quad->getOrCreateStateSet()->addUniform(preTex.get());
		quad->getOrCreateStateSet()->addUniform(preSize.get());

		width[i + 1] = (width[i] + 1) / 2;
		height[i + 1] = (height[i] + 1) /2;

		dsTex[i + 1] = new osg::Texture2D;
		dsTex[i + 1]->setTextureSize(width[i + 1], height[i + 1]);
		dsTex[i + 1]->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
		dsTex[i + 1]->setSourceFormat(GL_LUMINANCE_ALPHA);
		dsTex[i + 1]->setSourceType(GL_FLOAT);
		dsTex[i + 1]->setFilter(osg::Texture2D::MIN_FILTER,
			osg::Texture2D::NEAREST);
		dsTex[i + 1]->setFilter(osg::Texture2D::MAG_FILTER,
			osg::Texture2D::NEAREST);
		dsTex[i + 1]->setWrap(osg::Texture2D::WRAP_S,
			osg::Texture2D::CLAMP_TO_EDGE
			);
		dsTex[i + 1]->setWrap(osg::Texture2D::WRAP_T,
			osg::Texture2D::CLAMP_TO_EDGE
			);
		dsTex[i + 1]->setResizeNonPowerOfTwoHint(false);

		//2014/2/24
		osg::ref_ptr<ScreenQuad> equad;
		equad = new ScreenQuad;
		equad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dsTex[i].get());
		equad->getOrCreateStateSet()->setAttributeAndModes(edsProg.get());
		osg::ref_ptr<osg::Uniform> epreTex = new osg::Uniform("preTex", 0);
		osg::ref_ptr<osg::Uniform> epreSize = new osg::Uniform("preSize", osg::Vec2(float(width[i]), float(height[i])));
		equad->getOrCreateStateSet()->addUniform(epreTex.get());
		equad->getOrCreateStateSet()->addUniform(epreSize.get());

		edsTex[i + 1]=new osg::Texture2D;
		edsImg[i + 1]=new osg::Image;//3/31
		edsImg[i + 1]->allocateImage(width[i+1],height[i+1], 1, GL_LUMINANCE_ALPHA, GL_FLOAT);//3/31
		edsImg[i + 1]->setInternalTextureFormat(GL_LUMINANCE_ALPHA32F_ARB);//3/31
		edsTex[i + 1]->setTextureSize(width[i+1],height[i+1]);
		edsTex[i + 1]->setImage((edsImg[i+1]).get());//3/31
		edsTex[i + 1]->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
		edsTex[i + 1]->setSourceFormat(GL_LUMINANCE_ALPHA);
		edsTex[i + 1]->setSourceType(GL_FLOAT);
		edsTex[i + 1]->setFilter(osg::Texture2D::MIN_FILTER,
			osg::Texture2D::NEAREST);
		edsTex[i + 1]->setFilter(osg::Texture2D::MAG_FILTER,
			osg::Texture2D::NEAREST);
		edsTex[i + 1]->setWrap(osg::Texture2D::WRAP_S,
			osg::Texture2D::CLAMP_TO_EDGE
			);
		edsTex[i + 1]->setWrap(osg::Texture2D::WRAP_T,
			osg::Texture2D::CLAMP_TO_EDGE
			);
		edsTex[i + 1]->setResizeNonPowerOfTwoHint(false);

		//
		dsCam[i] = new osg::Camera;
		dsCam[i]->setViewport(0, 0, width[i + 1], height[i + 1]);
		dsCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
		dsCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
		dsCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
		dsCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		dsCam[i]->setRenderOrder(osg::Camera::POST_RENDER, i + 2);
		dsCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
		dsCam[i]->attach(osg::Camera::COLOR_BUFFER, dsTex[i + 1].get());
		dsCam[i]->addChild(quad.get());

		//2014/2/24
		edsCam[i] = new osg::Camera;
		edsCam[i]->setViewport(0, 0, width[i + 1], height[i + 1]);
		edsCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
		edsCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
		edsCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
		edsCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		edsCam[i]->setRenderOrder(osg::Camera::POST_RENDER, i + 2);
		edsCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
		edsCam[i]->attach(osg::Camera::COLOR_BUFFER, edsImg[i + 1].get());
		edsCam[i]->addChild(equad.get());
		//
		root->addChild(dsCam[i].get());
		root->addChild(edsCam[i].get());
	}

// 	osg::ref_ptr<osg::Program> dfProg = createProgram(DATAPATH + "Myshaders\\diffuse.vert", 
// 		DATAPATH + "Myshaders\\diffuse.frag");
// 
// 	osg::ref_ptr<ScreenQuad> quad;
// 	quad = new ScreenQuad;
// 	int k = 2;
// 	quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dsTex[k].get());
// 	quad->getOrCreateStateSet()->setTextureAttributeAndModes(1, dsTex[k + 1].get());
// 	quad->getOrCreateStateSet()->setAttributeAndModes(dfProg.get());
// 
// 	osg::ref_ptr<osg::Uniform> curTex = new osg::Uniform("curTex", 0);
// 	osg::ref_ptr<osg::Uniform> posTex = new osg::Uniform("posTex", 1);
// 	osg::ref_ptr<osg::Uniform> curSize = new osg::Uniform("curSize", osg::Vec2(float(width[k]), float(height[k])));
// 
// 	quad->getOrCreateStateSet()->addUniform(curTex.get());
// 	quad->getOrCreateStateSet()->addUniform(posTex.get());
// 	quad->getOrCreateStateSet()->addUniform(curSize.get());
// 
// 	osg::ref_ptr<osg::Camera> testCam;
// 	testCam = new osg::Camera;
// 	testCam->setViewport(0, 0, width[k], height[k]);
// 	testCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
// 	testCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
// 	testCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
// 	testCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
// 	testCam->setRenderOrder(osg::Camera::POST_RENDER, 7);
// 
// 	testCam->addChild(quad.get());
// 	root->addChild(testCam.get());

	osg::ref_ptr<osg::Camera> dfCam[5];
		osg::ref_ptr<osg::Camera> smCam[5];
		osg::ref_ptr<osg::Texture2D> dfTex[6];
		osg::ref_ptr<osg::Texture2D> smTex[6];

		 	_ddfDepthImg = new osg::Image;
		 	_ddfDepthImg->allocateImage(_width, _height, 1, GL_LUMINANCE, GL_FLOAT);
		 	_ddfDepthImg->setInternalTextureFormat(GL_LUMINANCE32F_ARB);

		osg::ref_ptr<osg::Program> dfProg = createProgram(DATAPATH + "Myshaders\\diffuse.vert", 
			DATAPATH + "Myshaders\\diffuse.frag");

		//这里smooth 也有两个版本
		//2014/5/30
		//osg::ref_ptr<osg::Program> smProg = createProgram(DATAPATH + "Myshaders\\smooth1.vert", 
			//DATAPATH + "Myshaders\\smooth1.frag");

		//2014/5/19  改高斯核
		osg::ref_ptr<osg::Program> smProg = createProgram(DATAPATH + "Myshaders\\smooth2.vert", 
			DATAPATH + "Myshaders\\smooth2.frag");
			

		//osg::ref_ptr<osg::Program> smProg = createProgram(DATAPATH + "Myshaders\\esmooth.vert", 
			//DATAPATH + "Myshaders\\esmooth.frag");

		//osg::ref_ptr<osg::Program> smProg = createProgram(DATAPATH + "Myshaders\\ela.vert", 
		//DATAPATH + "Myshaders\\ela.frag");
	
		//smTex[5] = dsTex[5];
		dfTex[5] = dsTex[5];
	
		int t2 = 0;
		for(i = 4; i >= t2; --i)
		{
 			osg::ref_ptr<ScreenQuad> smQuad;
 			smQuad = new ScreenQuad;
 			smQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dfTex[i + 1].get());
			smQuad->getOrCreateStateSet()->setTextureAttributeAndModes(1, edsTex[i + 1].get());//2014/2/24
 			smQuad->getOrCreateStateSet()->setAttributeAndModes(smProg.get());
 	
 			osg::ref_ptr<osg::Uniform> tex = new osg::Uniform("tex", 0);
			osg::ref_ptr<osg::Uniform> eTex=new osg::Uniform("eTex", 1);//2014/2/24
			osg::ref_ptr<osg::Uniform> size = new osg::Uniform("size", osg::Vec2(float(width[i + 1]), float(height[i + 1])));
			osg::ref_ptr<osg::Uniform> near = new osg::Uniform("near", mNear);
			osg::ref_ptr<osg::Uniform> far = new osg::Uniform("far", mFar);
			osg::ref_ptr<osg::Uniform> imvp=new osg::Uniform("imvp",this->_iMVPW);//2/26
			osg::ref_ptr<osg::Uniform> level=new osg::Uniform("level",i+1);//2/26

			smQuad->getOrCreateStateSet()->addUniform(tex.get());
			smQuad->getOrCreateStateSet()->addUniform(size.get());
			smQuad->getOrCreateStateSet()->addUniform(near.get());
			smQuad->getOrCreateStateSet()->addUniform(far.get());
			smQuad->getOrCreateStateSet()->addUniform(eTex.get());//2014/2/24
			smQuad->getOrCreateStateSet()->addUniform(imvp.get());//2/26
			smQuad->getOrCreateStateSet()->addUniform(level.get());//2/26

			smTex[i+1] = new osg::Texture2D;
			smTex[i+1]->setTextureSize(width[i + 1], height[i + 1]);
			smTex[i+1]->setInternalFormat(GL_LUMINANCE32F_ARB);
			smTex[i+1]->setSourceFormat(GL_LUMINANCE);
			smTex[i+1]->setSourceType(GL_FLOAT);
			smTex[i+1]->setFilter(osg::Texture2D::MIN_FILTER,
				osg::Texture2D::NEAREST);
			smTex[i+1]->setFilter(osg::Texture2D::MAG_FILTER,
				osg::Texture2D::NEAREST);
			smTex[i + 1]->setWrap(osg::Texture2D::WRAP_S,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			smTex[i + 1]->setWrap(osg::Texture2D::WRAP_T,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			smTex[i+1]->setResizeNonPowerOfTwoHint(false);
			//2014/2/22边缘保留的高斯双边滤波

			//
			smCam[i] = new osg::Camera;
			smCam[i]->setViewport(0, 0, width[i + 1], height[i + 1]);
			smCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
			smCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
			smCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
			smCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			smCam[i]->setRenderOrder(osg::Camera::POST_RENDER, t1 + 2 + 3* (4 - i));
			smCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
			smCam[i]->attach(osg::Camera::COLOR_BUFFER, smTex[i + 1].get());
			smCam[i]->addChild(smQuad.get());
			root->addChild(smCam[i].get());
	
			osg::ref_ptr<ScreenQuad> quad;
			quad = new ScreenQuad;

			/*
			*目的：块状结果截图
			*时间：2013年12月15号
			*有或者没有smooth，这里切换参数，smTex[i + 1]或者dfTex[i + 1]
			*/
			//2014/3/13
			
				
			
			//quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dfTex[i + 1].get());

		
			quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, smTex[i + 1].get());
			quad->getOrCreateStateSet()->setTextureAttributeAndModes(1, dsTex[i].get());
			quad->getOrCreateStateSet()->setAttributeAndModes(dfProg.get());
	
			osg::ref_ptr<osg::Uniform> posTex = new osg::Uniform("posTex", 0);
			osg::ref_ptr<osg::Uniform> curTex = new osg::Uniform("curTex", 1);
			osg::ref_ptr<osg::Uniform> curSize = new osg::Uniform("curSize", osg::Vec2(float(width[i]), float(height[i])));
	
			quad->getOrCreateStateSet()->addUniform(posTex.get());
			quad->getOrCreateStateSet()->addUniform(curTex.get());
			quad->getOrCreateStateSet()->addUniform(curSize.get());
	
			dfTex[i] = new osg::Texture2D;
			dfTex[i]->setTextureSize(width[i], height[i]);
			dfTex[i]->setInternalFormat(GL_LUMINANCE32F_ARB);
			dfTex[i]->setSourceFormat(GL_LUMINANCE);
			dfTex[i]->setSourceType(GL_FLOAT);
			dfTex[i]->setFilter(osg::Texture2D::MIN_FILTER,
				osg::Texture2D::NEAREST);
			dfTex[i]->setFilter(osg::Texture2D::MAG_FILTER,
				osg::Texture2D::NEAREST);
			dfTex[i]->setWrap(osg::Texture2D::WRAP_S,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			dfTex[i]->setWrap(osg::Texture2D::WRAP_T,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			dfTex[i]->setResizeNonPowerOfTwoHint(false);
	
			dfCam[i] = new osg::Camera;
			dfCam[i]->setViewport(0, 0, width[i], height[i]);
			dfCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
			dfCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
			dfCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
			dfCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			dfCam[i]->setRenderOrder(osg::Camera::POST_RENDER, t1 + 2 + 3*(4 - i) + 1);
			dfCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
			if(i != 0)
				dfCam[i]->attach(osg::Camera::COLOR_BUFFER, dfTex[i].get());
			else
				dfCam[i]->attach(osg::Camera::COLOR_BUFFER, _ddfDepthImg.get());
			
			dfCam[i]->addChild(quad.get());
			root->addChild(dfCam[i].get());
	
			
			/*osg::ref_ptr<ScreenQuad> smQuad;
			smQuad = new ScreenQuad;
			smQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dfTex[i].get());
			smQuad->getOrCreateStateSet()->setAttributeAndModes(smProg.get());
	
			osg::ref_ptr<osg::Uniform> tex = new osg::Uniform("tex", 0);
			osg::ref_ptr<osg::Uniform> size = new osg::Uniform("size", osg::Vec2(float(width[i]), float(height[i])));
	
			smQuad->getOrCreateStateSet()->addUniform(tex.get());
			smQuad->getOrCreateStateSet()->addUniform(size.get());
	
			smTex[i] = new osg::Texture2D;
			smTex[i]->setTextureSize(width[i], height[i]);
			smTex[i]->setInternalFormat(GL_LUMINANCE32F_ARB);
			smTex[i]->setSourceFormat(GL_LUMINANCE);
			smTex[i]->setSourceType(GL_FLOAT);
			smTex[i]->setFilter(osg::Texture2D::MIN_FILTER,
				osg::Texture2D::LINEAR);
			smTex[i]->setFilter(osg::Texture2D::MAG_FILTER,
				osg::Texture2D::LINEAR);
			smTex[i]->setResizeNonPowerOfTwoHint(false);
	
			smCam[i] = new osg::Camera;
			smCam[i]->setViewport(0, 0, width[i], height[i]);
			smCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
			smCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
			smCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
			smCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			smCam[i]->setRenderOrder(osg::Camera::POST_RENDER, t1 + 2 + 3* (4 - i) + 2);
			//if(i != t2)
			//{
				smCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
				smCam[i]->attach(osg::Camera::COLOR_BUFFER, smTex[i].get());
			//}
			smCam[i]->addChild(smQuad.get());
			root->addChild(smCam[i].get());*/
		}

	//这里可以切换pesudoDepth shader 和 pesudoDepth1 shader;
// 	osg::ref_ptr<osg::Camera>    psudoDepthCam;
// 	osg::ref_ptr<ScreenQuad> psudoDepthQuad;
// 	psudoDepthQuad = new ScreenQuad;
// 	psudoDepthQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dfTex[0].get());
// 	psudoDepthQuad->getOrCreateStateSet()->setAttributeAndModes(
// 		createProgram(DATAPATH + "Myshaders\\psudoDepth1.vert", 
// 		DATAPATH + "Myshaders\\psudoDepth1.frag")
// 		);
// 
// 	osg::ref_ptr<osg::Uniform> rDepth = new osg::Uniform("rDepth", 0);
// 
// 	psudoDepthQuad->getOrCreateStateSet()->addUniform(rDepth.get());
// 	psudoDepthQuad->getOrCreateStateSet()->addUniform(size.get());
// 	psudoDepthQuad->getOrCreateStateSet()->addUniform(near.get());
// 	psudoDepthQuad->getOrCreateStateSet()->addUniform(far.get());
// 
// 
// 	_dfDepthImg = new osg::Image;
// 	_dfDepthImg->allocateImage(_width, _height, 1, GL_LUMINANCE, GL_FLOAT);
// 	_dfDepthImg->setInternalTextureFormat(GL_LUMINANCE32F_ARB);
// 
// 	psudoDepthCam = new osg::Camera;
// 	psudoDepthCam->setViewport(0, 0, _width, _height);
// 	psudoDepthCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
// 	psudoDepthCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
// 	psudoDepthCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
// 	psudoDepthCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
// 	psudoDepthCam->setRenderOrder(osg::Camera::POST_RENDER, 22);
// 	psudoDepthCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
// 	psudoDepthCam->attach(osg::Camera::COLOR_BUFFER, _dfDepthImg.get());
// 	psudoDepthCam->addChild(psudoDepthQuad.get());
// 	psudoDepthCam->setPostDrawCallback(new getProcessTime);
// 	root->addChild(psudoDepthCam.get());


}
void LocalViewer::createDiffuseCameras(osg::Group* root)
{
	osg::ref_ptr<osg::Camera> realDepthCam;
	osg::ref_ptr<osg::Texture2D> realDepthTex;
	osg::ref_ptr<ScreenQuad> realDepthQuad;
	realDepthQuad = new ScreenQuad;
	realDepthQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0, _edgeTex.get());
	realDepthQuad->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\realDepth.vert", 
		DATAPATH + "Myshaders\\realDepth.frag")
		);

	//2014.2.19
	std::cout<<DATAPATH+"Myshaders\\realDepth.vert"<<std::endl;
	//
	osg::ref_ptr<osg::Uniform> pDepth = new osg::Uniform("pDepth", 0);
	osg::ref_ptr<osg::Uniform> size = new osg::Uniform("size", osg::Vec2(float(_width), float(_height)));
	osg::ref_ptr<osg::Uniform> near = new osg::Uniform("near", float(mNear));
	osg::ref_ptr<osg::Uniform> far = new osg::Uniform("far", float(mFar));
	
	near->setUpdateCallback(new updataNear(this));
	far->setUpdateCallback(new updataFar(this));

	realDepthQuad->getOrCreateStateSet()->addUniform(pDepth.get());
	realDepthQuad->getOrCreateStateSet()->addUniform(size.get());
	realDepthQuad->getOrCreateStateSet()->addUniform(near.get());
	realDepthQuad->getOrCreateStateSet()->addUniform(far.get());

	realDepthTex = new osg::Texture2D;
	realDepthTex->setTextureSize(_width,_height);
	realDepthTex->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
	realDepthTex->setSourceFormat(GL_LUMINANCE_ALPHA);
	realDepthTex->setSourceType(GL_FLOAT);
	realDepthTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::LINEAR);
	realDepthTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::LINEAR);
	realDepthTex->setResizeNonPowerOfTwoHint(false);


	realDepthCam = new osg::Camera;
	realDepthCam->setViewport(0, 0, _width, _height);
	realDepthCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	realDepthCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	realDepthCam->setClearColor(osg::Vec4(1.0f, 1.0f, 1.0f, 0.0f));
	realDepthCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	realDepthCam->setRenderOrder(osg::Camera::POST_RENDER, 1);
	realDepthCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	realDepthCam->attach(osg::Camera::COLOR_BUFFER, realDepthTex.get());

	realDepthCam->setPreDrawCallback(new getProcessTime);
	realDepthCam->addChild(realDepthQuad.get());
	root->addChild(realDepthCam);

	osg::ref_ptr<osg::Camera> dsCam[5];
	osg::ref_ptr<osg::Camera> edsCam[5];
	osg::ref_ptr<osg::Texture2D> dsTex[6];
	osg::ref_ptr<osg::Texture2D> edsTex[6];//2014/2/13边缘纹理只保留边缘

	int width[6], height[6];

	osg::ref_ptr<osg::Program> dsProg = createProgram(DATAPATH+"Myshaders\\downSample.vert", 
		DATAPATH + "Myshaders\\downSample.frag");

	osg::ref_ptr<osg::Program> edsProg=createProgram(DATAPATH+"Myshaders\\sampleedge.vert",
		DATAPATH+"Myshaders\\sampleedge.frag");
	//直接对非线性的Zbuffer进行diffuse;
	dsTex[0] = _edgeTex;//2014/2/25
	//dsTex[0]=stex;
	//dsTex[0]=realDepthTex;
	//dsTex[0] = sampleTex;//2014/3/20
	int i;
	int t1 = 5;
	//int t1 = 12;//2014/3/20
	width[0] = _width;
	height[0] = _height;

	for(i = 0;  i < t1; ++i)
	{
		osg::ref_ptr<ScreenQuad> quad;
		quad = new ScreenQuad;
		quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dsTex[i].get());
		quad->getOrCreateStateSet()->setAttributeAndModes(dsProg.get());

		osg::ref_ptr<osg::Uniform> preTex = new osg::Uniform("preTex", 0);
		osg::ref_ptr<osg::Uniform> preSize = new osg::Uniform("preSize", osg::Vec2(float(width[i]), float(height[i])));

		quad->getOrCreateStateSet()->addUniform(preTex.get());
		quad->getOrCreateStateSet()->addUniform(preSize.get());

		width[i + 1] = (width[i] + 1) / 2;
		height[i + 1] = (height[i] + 1) /2;

		dsTex[i + 1] = new osg::Texture2D;
		dsTex[i + 1]->setTextureSize(width[i + 1], height[i + 1]);
		dsTex[i + 1]->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
		dsTex[i + 1]->setSourceFormat(GL_LUMINANCE_ALPHA);
		dsTex[i + 1]->setSourceType(GL_FLOAT);
		dsTex[i + 1]->setFilter(osg::Texture2D::MIN_FILTER,
			osg::Texture2D::NEAREST);
		dsTex[i + 1]->setFilter(osg::Texture2D::MAG_FILTER,
			osg::Texture2D::NEAREST);
		dsTex[i + 1]->setWrap(osg::Texture2D::WRAP_S,
			osg::Texture2D::CLAMP_TO_EDGE
			);
		dsTex[i + 1]->setWrap(osg::Texture2D::WRAP_T,
			osg::Texture2D::CLAMP_TO_EDGE
			);
		dsTex[i + 1]->setResizeNonPowerOfTwoHint(false);

		//2014/2/24
		osg::ref_ptr<ScreenQuad> equad;
		equad = new ScreenQuad;
		equad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dsTex[i].get());
		equad->getOrCreateStateSet()->setAttributeAndModes(edsProg.get());
		osg::ref_ptr<osg::Uniform> epreTex = new osg::Uniform("preTex", 0);
		osg::ref_ptr<osg::Uniform> epreSize = new osg::Uniform("preSize", osg::Vec2(float(width[i]), float(height[i])));
		equad->getOrCreateStateSet()->addUniform(epreTex.get());
		equad->getOrCreateStateSet()->addUniform(epreSize.get());
		edsTex[i + 1]=new osg::Texture2D;
		edsTex[i + 1]->setTextureSize(width[i+1],height[i+1]);
		edsTex[i + 1]->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
		edsTex[i + 1]->setSourceFormat(GL_LUMINANCE_ALPHA);
		edsTex[i + 1]->setSourceType(GL_FLOAT);
		edsTex[i + 1]->setFilter(osg::Texture2D::MIN_FILTER,
			osg::Texture2D::NEAREST);
		edsTex[i + 1]->setFilter(osg::Texture2D::MAG_FILTER,
			osg::Texture2D::NEAREST);
		edsTex[i + 1]->setWrap(osg::Texture2D::WRAP_S,
			osg::Texture2D::CLAMP_TO_EDGE
			);
		edsTex[i + 1]->setWrap(osg::Texture2D::WRAP_T,
			osg::Texture2D::CLAMP_TO_EDGE
			);
		edsTex[i + 1]->setResizeNonPowerOfTwoHint(false);

		//
		dsCam[i] = new osg::Camera;
		dsCam[i]->setViewport(0, 0, width[i + 1], height[i + 1]);
		dsCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
		dsCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
		dsCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
		dsCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		dsCam[i]->setRenderOrder(osg::Camera::POST_RENDER, i + 2);
		dsCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
		dsCam[i]->attach(osg::Camera::COLOR_BUFFER, dsTex[i + 1].get());
		dsCam[i]->addChild(quad.get());

		//2014/2/24
		edsCam[i] = new osg::Camera;
		edsCam[i]->setViewport(0, 0, width[i + 1], height[i + 1]);
		edsCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
		edsCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
		edsCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
		edsCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		edsCam[i]->setRenderOrder(osg::Camera::POST_RENDER, i + 2);
		edsCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
		edsCam[i]->attach(osg::Camera::COLOR_BUFFER, edsTex[i + 1].get());
		edsCam[i]->addChild(equad.get());
		//
		root->addChild(dsCam[i].get());
		//root->addChild(edsCam[i].get());//2014/3/23
	}

// 	osg::ref_ptr<osg::Program> dfProg = createProgram(DATAPATH + "Myshaders\\diffuse.vert", 
// 		DATAPATH + "Myshaders\\diffuse.frag");
// 
// 	osg::ref_ptr<ScreenQuad> quad;
// 	quad = new ScreenQuad;
// 	int k = 2;
// 	quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dsTex[k].get());
// 	quad->getOrCreateStateSet()->setTextureAttributeAndModes(1, dsTex[k + 1].get());
// 	quad->getOrCreateStateSet()->setAttributeAndModes(dfProg.get());
// 
// 	osg::ref_ptr<osg::Uniform> curTex = new osg::Uniform("curTex", 0);
// 	osg::ref_ptr<osg::Uniform> posTex = new osg::Uniform("posTex", 1);
// 	osg::ref_ptr<osg::Uniform> curSize = new osg::Uniform("curSize", osg::Vec2(float(width[k]), float(height[k])));
// 
// 	quad->getOrCreateStateSet()->addUniform(curTex.get());
// 	quad->getOrCreateStateSet()->addUniform(posTex.get());
// 	quad->getOrCreateStateSet()->addUniform(curSize.get());
// 
// 	osg::ref_ptr<osg::Camera> testCam;
// 	testCam = new osg::Camera;
// 	testCam->setViewport(0, 0, width[k], height[k]);
// 	testCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
// 	testCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
// 	testCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
// 	testCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
// 	testCam->setRenderOrder(osg::Camera::POST_RENDER, 7);
// 
// 	testCam->addChild(quad.get());
// 	root->addChild(testCam.get());

	osg::ref_ptr<osg::Camera> dfCam[5];
		osg::ref_ptr<osg::Camera> smCam[5];
		osg::ref_ptr<osg::Texture2D> dfTex[6];
		osg::ref_ptr<osg::Texture2D> smTex[6];

		 	_dfDepthImg = new osg::Image;
		 	_dfDepthImg->allocateImage(_width, _height, 1, GL_LUMINANCE, GL_FLOAT);
		 	_dfDepthImg->setInternalTextureFormat(GL_LUMINANCE32F_ARB);

		osg::ref_ptr<osg::Program> dfProg = createProgram(DATAPATH + "Myshaders\\diffuse.vert", 
			DATAPATH + "Myshaders\\diffuse.frag");
		
		//这里smooth 也有两个版本
		
		osg::ref_ptr<osg::Program> smProg = createProgram(DATAPATH + "Myshaders\\smooth1.vert", 
			DATAPATH + "Myshaders\\smooth1.frag");
			

		//osg::ref_ptr<osg::Program> smProg = createProgram(DATAPATH + "Myshaders\\esmooth.vert", 
			//DATAPATH + "Myshaders\\esmooth.frag");

		//osg::ref_ptr<osg::Program> smProg = createProgram(DATAPATH + "Myshaders\\ela.vert", 
		//DATAPATH + "Myshaders\\ela.frag");
	
		//smTex[5] = dsTex[5];
		dfTex[5] = dsTex[5];
	
		int t2 = 0;
		for(i = 4; i >= t2; --i)
		{
 			osg::ref_ptr<ScreenQuad> smQuad;
 			smQuad = new ScreenQuad;
 			smQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dfTex[i + 1].get());
			smQuad->getOrCreateStateSet()->setTextureAttributeAndModes(1, edsTex[i + 1].get());//2014/2/24
 			smQuad->getOrCreateStateSet()->setAttributeAndModes(smProg.get());
 	
 			osg::ref_ptr<osg::Uniform> tex = new osg::Uniform("tex", 0);
			osg::ref_ptr<osg::Uniform> eTex=new osg::Uniform("eTex", 1);//2014/2/24
			osg::ref_ptr<osg::Uniform> size = new osg::Uniform("size", osg::Vec2(float(width[i + 1]), float(height[i + 1])));
			osg::ref_ptr<osg::Uniform> near = new osg::Uniform("near", mNear);
			osg::ref_ptr<osg::Uniform> far = new osg::Uniform("far", mFar);
			osg::ref_ptr<osg::Uniform> imvp=new osg::Uniform("imvp",this->_iMVPW);//2/26
			osg::ref_ptr<osg::Uniform> level=new osg::Uniform("level",i+1);//2/26

			smQuad->getOrCreateStateSet()->addUniform(tex.get());
			smQuad->getOrCreateStateSet()->addUniform(size.get());
			smQuad->getOrCreateStateSet()->addUniform(near.get());
			smQuad->getOrCreateStateSet()->addUniform(far.get());
			smQuad->getOrCreateStateSet()->addUniform(eTex.get());//2014/2/24
			smQuad->getOrCreateStateSet()->addUniform(imvp.get());//2/26
			smQuad->getOrCreateStateSet()->addUniform(level.get());//2/26

			smTex[i+1] = new osg::Texture2D;
			smTex[i+1]->setTextureSize(width[i + 1], height[i + 1]);
			smTex[i+1]->setInternalFormat(GL_LUMINANCE32F_ARB);
			smTex[i+1]->setSourceFormat(GL_LUMINANCE);
			smTex[i+1]->setSourceType(GL_FLOAT);
			smTex[i+1]->setFilter(osg::Texture2D::MIN_FILTER,
				osg::Texture2D::NEAREST);
			smTex[i+1]->setFilter(osg::Texture2D::MAG_FILTER,
				osg::Texture2D::NEAREST);
			smTex[i + 1]->setWrap(osg::Texture2D::WRAP_S,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			smTex[i + 1]->setWrap(osg::Texture2D::WRAP_T,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			smTex[i+1]->setResizeNonPowerOfTwoHint(false);
			//2014/2/22边缘保留的高斯双边滤波

			//
			smCam[i] = new osg::Camera;
			smCam[i]->setViewport(0, 0, width[i + 1], height[i + 1]);
			smCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
			smCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
			smCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
			smCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			smCam[i]->setRenderOrder(osg::Camera::POST_RENDER, t1 + 2 + 3* (4 - i));
			smCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
			smCam[i]->attach(osg::Camera::COLOR_BUFFER, smTex[i + 1].get());
			smCam[i]->addChild(smQuad.get());
			root->addChild(smCam[i].get());
	
			osg::ref_ptr<ScreenQuad> quad;
			quad = new ScreenQuad;

			/*
			*目的：块状结果截图
			*时间：2013年12月15号
			*有或者没有smooth，这里切换参数，smTex[i + 1]或者dfTex[i + 1]
			*/
			//2014/3/13
			
				
			//
			//quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dfTex[i + 1].get());

		
			quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, smTex[i + 1].get());
			quad->getOrCreateStateSet()->setTextureAttributeAndModes(1, dsTex[i].get());
			quad->getOrCreateStateSet()->setAttributeAndModes(dfProg.get());
	
			osg::ref_ptr<osg::Uniform> posTex = new osg::Uniform("posTex", 0);
			osg::ref_ptr<osg::Uniform> curTex = new osg::Uniform("curTex", 1);
			osg::ref_ptr<osg::Uniform> curSize = new osg::Uniform("curSize", osg::Vec2(float(width[i]), float(height[i])));
	
			quad->getOrCreateStateSet()->addUniform(posTex.get());
			quad->getOrCreateStateSet()->addUniform(curTex.get());
			quad->getOrCreateStateSet()->addUniform(curSize.get());
	
			dfTex[i] = new osg::Texture2D;
			dfTex[i]->setTextureSize(width[i], height[i]);
			dfTex[i]->setInternalFormat(GL_LUMINANCE32F_ARB);
			dfTex[i]->setSourceFormat(GL_LUMINANCE);
			dfTex[i]->setSourceType(GL_FLOAT);
			dfTex[i]->setFilter(osg::Texture2D::MIN_FILTER,
				osg::Texture2D::NEAREST);
			dfTex[i]->setFilter(osg::Texture2D::MAG_FILTER,
				osg::Texture2D::NEAREST);
			dfTex[i]->setWrap(osg::Texture2D::WRAP_S,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			dfTex[i]->setWrap(osg::Texture2D::WRAP_T,
				osg::Texture2D::CLAMP_TO_EDGE
				);
			dfTex[i]->setResizeNonPowerOfTwoHint(false);
	
			dfCam[i] = new osg::Camera;
			dfCam[i]->setViewport(0, 0, width[i], height[i]);
			dfCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
			dfCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
			dfCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
			dfCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			dfCam[i]->setRenderOrder(osg::Camera::POST_RENDER, t1 + 2 + 3*(4 - i) + 1);
			dfCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
			if(i != 0)
				dfCam[i]->attach(osg::Camera::COLOR_BUFFER, dfTex[i].get());
			else
				dfCam[i]->attach(osg::Camera::COLOR_BUFFER, _dfDepthImg.get());
			
			dfCam[i]->addChild(quad.get());
			root->addChild(dfCam[i].get());
	
			
			/*osg::ref_ptr<ScreenQuad> smQuad;
			smQuad = new ScreenQuad;
			smQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dfTex[i].get());
			smQuad->getOrCreateStateSet()->setAttributeAndModes(smProg.get());
	
			osg::ref_ptr<osg::Uniform> tex = new osg::Uniform("tex", 0);
			osg::ref_ptr<osg::Uniform> size = new osg::Uniform("size", osg::Vec2(float(width[i]), float(height[i])));
	
			smQuad->getOrCreateStateSet()->addUniform(tex.get());
			smQuad->getOrCreateStateSet()->addUniform(size.get());
	
			smTex[i] = new osg::Texture2D;
			smTex[i]->setTextureSize(width[i], height[i]);
			smTex[i]->setInternalFormat(GL_LUMINANCE32F_ARB);
			smTex[i]->setSourceFormat(GL_LUMINANCE);
			smTex[i]->setSourceType(GL_FLOAT);
			smTex[i]->setFilter(osg::Texture2D::MIN_FILTER,
				osg::Texture2D::LINEAR);
			smTex[i]->setFilter(osg::Texture2D::MAG_FILTER,
				osg::Texture2D::LINEAR);
			smTex[i]->setResizeNonPowerOfTwoHint(false);
	
			smCam[i] = new osg::Camera;
			smCam[i]->setViewport(0, 0, width[i], height[i]);
			smCam[i]->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
			smCam[i]->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
			smCam[i]->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
			smCam[i]->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			smCam[i]->setRenderOrder(osg::Camera::POST_RENDER, t1 + 2 + 3* (4 - i) + 2);
			//if(i != t2)
			//{
				smCam[i]->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
				smCam[i]->attach(osg::Camera::COLOR_BUFFER, smTex[i].get());
			//}
			smCam[i]->addChild(smQuad.get());
			root->addChild(smCam[i].get());*/
		}

	//这里可以切换pesudoDepth shader 和 pesudoDepth1 shader;
// 	osg::ref_ptr<osg::Camera>    psudoDepthCam;
// 	osg::ref_ptr<ScreenQuad> psudoDepthQuad;
// 	psudoDepthQuad = new ScreenQuad;
// 	psudoDepthQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0, dfTex[0].get());
// 	psudoDepthQuad->getOrCreateStateSet()->setAttributeAndModes(
// 		createProgram(DATAPATH + "Myshaders\\psudoDepth1.vert", 
// 		DATAPATH + "Myshaders\\psudoDepth1.frag")
// 		);
// 
// 	osg::ref_ptr<osg::Uniform> rDepth = new osg::Uniform("rDepth", 0);
// 
// 	psudoDepthQuad->getOrCreateStateSet()->addUniform(rDepth.get());
// 	psudoDepthQuad->getOrCreateStateSet()->addUniform(size.get());
// 	psudoDepthQuad->getOrCreateStateSet()->addUniform(near.get());
// 	psudoDepthQuad->getOrCreateStateSet()->addUniform(far.get());
// 
// 
// 	_dfDepthImg = new osg::Image;
// 	_dfDepthImg->allocateImage(_width, _height, 1, GL_LUMINANCE, GL_FLOAT);
// 	_dfDepthImg->setInternalTextureFormat(GL_LUMINANCE32F_ARB);
// 
// 	psudoDepthCam = new osg::Camera;
// 	psudoDepthCam->setViewport(0, 0, _width, _height);
// 	psudoDepthCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
// 	psudoDepthCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
// 	psudoDepthCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
// 	psudoDepthCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
// 	psudoDepthCam->setRenderOrder(osg::Camera::POST_RENDER, 22);
// 	psudoDepthCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
// 	psudoDepthCam->attach(osg::Camera::COLOR_BUFFER, _dfDepthImg.get());
// 	psudoDepthCam->addChild(psudoDepthQuad.get());
// 	psudoDepthCam->setPostDrawCallback(new getProcessTime);
// 	root->addChild(psudoDepthCam.get());
}


void LocalViewer::getNearFarCallback::operator() (osg::RenderInfo& renderInfo) const
{
	LocalViewer* vw = dynamic_cast<LocalViewer*> (renderInfo.getView());
	double fov;
	double ratio;
	if(vw) 
	{
		
		vw->_uMVP = vw->getCamera()->getViewMatrix() * vw->getCamera()->getProjectionMatrix();
		vw->_iMVPW = vw->_uMVP * vw->getCamera()->getViewport()->computeWindowMatrix();
		vw->_iMVPW = osg::Matrix::inverse(vw->_iMVPW);

	    //std::cout << std::fixed << "near:" << vw->_autoNear << "," << "far:" << vw->_autoFar << std::endl;
 		//double tnear;
		//double tfar;
		//vw->getCamera()->getProjectionMatrixAsPerspective(fov,ratio, tnear, tfar);
       //std::cout << std::fixed << "near:" << tnear << "," << "far:" << tfar << std::endl;
	}
}

void LocalViewer::updataNear::operator ()(osg::Uniform* uniform, osg::NodeVisitor* nv)
{
	float tmp = float(view->mFar);
	uniform->set(tmp);
}
void LocalViewer::updateSeed::operator()(osg::Uniform* uniform, osg::NodeVisitor* nv)
{
	srand(unsigned int(time(NULL)));
	uniform->set(unsigned int (rand()));


}
void LocalViewer::updataFar::operator ()(osg::Uniform* uniform, osg::NodeVisitor* nv)
{
	float tmp = float(view->mNear);
	uniform->set(tmp);
}
void LocalViewer::updateGroup::operator()(osg::Uniform* uniform,osg::NodeVisitor* nv)
{
	int group=view->NGROUP;
	srand(unsigned int(time(NULL)));
	for(int i=0;i<view->LEVEL;i++)
	{
		//生成随机数
		if(i!=0)
		{
			//产生随机伪随机数

			//组内随机数的产生
			uniform->setElement(i*group,rand()%group);
			uniform->setElement(i*group+1,rand()%group);
			uniform->setElement(i*group+2,rand()%group);
			uniform->setElement(i*group+3,rand()%group);

			//uniform->setElement(i*group,0);
			//uniform->setElement(i*group+1,1);
			//uniform->setElement(i*group+2,2);
			//uniform->setElement(i*group+3,3);

		}else
		{
			uniform->setElement(i*group,0);
			uniform->setElement(i*group+1,0);
			uniform->setElement(i*group+2,0);
			uniform->setElement(i*group+3,0);
		}
	}
}

void LocalViewer::getProcessTime::operator () (osg::RenderInfo& renderInfo) const
{
	LocalViewer* vw = dynamic_cast<LocalViewer*> (renderInfo.getView());
	if(vw)
	{
		if(!isFinished)
		{
			gTime = osg::Timer::instance()->tick();
			isFinished = true;
		}
		else
		{
			osg::Timer_t tmp = gTime;
			gTime = osg::Timer::instance()->tick();
			double ss = osg::Timer::instance()->delta_m(tmp, gTime);
			isFinished = false;
			//std::cout << std::fixed << "processing time:" << ss << std::endl;
		}
	}
}

bool LocalViewer::getProcessTime::isFinished = false;

osg::Timer_t LocalViewer::getProcessTime::gTime = 0;

void LocalViewer::createTest(osg::Group* root)
{
	osg::ref_ptr<osg::Geometry>  geom = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

	int i, j;
	for(i = 0 ; i < _width; ++i)
		for(j = 0 ; j < _height; ++j)
		{
			float x = i + 0.5;
			float y = j + 0.5;
			vertices->push_back(osg::Vec3(x, y, 0.0f));
		}
		geom->setVertexArray(vertices.get());
		geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, _width * _height));
		osg::ref_ptr<osg::Geode> testGeode = new osg::Geode;
		testGeode->addDrawable(geom.get());
		testGeode->getOrCreateStateSet()->setAttributeAndModes(
			createProgram(DATAPATH + "Myshaders\\test.vert", 
			DATAPATH + "Myshaders\\test.frag")
			);
		testGeode->getOrCreateStateSet()->setTextureAttributeAndModes(0, _imageTex.get());
		osg::ref_ptr<osg::Uniform> size = new osg::Uniform("size", osg::Vec2(float(_width),float(_height)));
		osg::ref_ptr<osg::Uniform> color  = new osg::Uniform("color", 0);

		testGeode->getOrCreateStateSet()->addUniform(size.get());
		testGeode->getOrCreateStateSet()->addUniform(color.get());
	    osg::ref_ptr<osg::Camera> testCam = new osg::Camera;
		testCam->setViewport(0, 0, _width, _height);
		testCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
		testCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		testCam->setRenderOrder(osg::Camera::POST_RENDER, 80);
		testCam->addChild(testGeode.get());
		root->addChild(testCam.get());
}
void LocalViewer::createEdgeRegionShowCam()
{
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;

	traits->x = 450;
	traits->y = 450;
	traits->width = _width;
	traits->height = _height;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->supportsResize = false;
	traits->windowName="showRegion";
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());



	osg::ref_ptr<osg::Texture2D> showDepthTex = new osg::Texture2D;
	showDepthTex->setImage(_edgeRegionImg.get());
	showDepthTex->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
	showDepthTex->setResizeNonPowerOfTwoHint(false);

	osg::ref_ptr<ScreenQuad> quad1;
	quad1 = new ScreenQuad;
	quad1->getOrCreateStateSet()->setTextureAttributeAndModes(0, showDepthTex.get());
	quad1->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\showregion.vert", 
		DATAPATH + "Myshaders\\showregion.frag")
		);

	osg::ref_ptr<osg::Uniform> region = new osg::Uniform("edgeregion", 0);
	osg::ref_ptr<osg::Uniform> texSize = new osg::Uniform("texSize", osg::Vec2(float(_width), float(_height)));
	//add laplace modul;
	quad1->getOrCreateStateSet()->addUniform(region.get());
	quad1->getOrCreateStateSet()->addUniform(texSize.get());
	

	_edgeRegionShowCam = new osg::Camera;
	_edgeRegionShowCam->setGraphicsContext(gc.get());
	_edgeRegionShowCam->setViewport(0, 0, _width, _height);
	_edgeRegionShowCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	_edgeRegionShowCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	_edgeRegionShowCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	_edgeRegionShowCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_edgeRegionShowCam->setRenderOrder(osg::Camera::POST_RENDER, 52);
	_edgeRegionShowCam->addChild(quad1.get());
	this->addSlave(_edgeRegionShowCam.get(), false);

};
void LocalViewer::createEdgeRegion()
{
	_edgeRegionImg = new osg::Image;
	_edgeRegionImg->allocateImage(_width, _height, 1, GL_LUMINANCE_ALPHA, GL_FLOAT);
	_edgeRegionImg->setInternalTextureFormat(GL_LUMINANCE_ALPHA32F_ARB);
	

	osg::ref_ptr<osg::Texture2D> edgetexture=new osg::Texture2D;
	edgetexture->setImage(_edgeImg.get());
	edgetexture->setInternalFormat(GL_LUMINANCE_ALPHA32F_ARB);
	edgetexture->setResizeNonPowerOfTwoHint(false);
	
	//2014/4/26
	osg::ref_ptr<osg::Texture2D> saliencyTex=new osg::Texture2D;
	saliencyTex->setImage(_saliencymap.get());
	saliencyTex->setInternalFormat(GL_RGB);
	saliencyTex->setResizeNonPowerOfTwoHint(false);
	//2014/4/26

	osg::ref_ptr<ScreenQuad> quad;
	quad = new ScreenQuad;
	quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, edgetexture.get());
	quad->getOrCreateStateSet()->setTextureAttributeAndModes(1,saliencyTex.get());
	quad->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(DATAPATH + "Myshaders\\edgeRegion.vert", 
		DATAPATH + "Myshaders\\edgeRegion.frag")
		);
	osg::ref_ptr<osg::Uniform> edge=new osg::Uniform("edge",0);
	osg::ref_ptr<osg::Uniform> saliencyUniform=new osg::Uniform("saliency",1);
	osg::ref_ptr<osg::Uniform> w=new osg::Uniform("width",_width);
	osg::ref_ptr<osg::Uniform> h=new osg::Uniform("height",_height);
	bool isSaliencyRegion=true;
	bool isEdgeSample=true;
#ifdef ISSALIENCYREGION
	isSaliencyRegion=true;
#else
	isSaliencyRegion=false;
#endif


#ifdef ISEDGESAMPLE
	isEdgeSample=true;
#else
	isEdgeSample=false;
#endif
	osg::ref_ptr<osg::Uniform> isEdgeSampleUniform=new osg::Uniform("isEdgeSample",isEdgeSample);
	osg::ref_ptr<osg::Uniform> issaliency=new osg::Uniform("issaliency",isSaliencyRegion);
	osg::ref_ptr<osg::Uniform> regionThresholdUniform=new osg::Uniform("threshold",regionThreshold);
	osg::ref_ptr<osg::Uniform> runiform = new osg::Uniform("r",R_edgeRegion);
	quad->getOrCreateStateSet()->addUniform(regionThresholdUniform.get());
	quad->getOrCreateStateSet()->addUniform(edge.get());
	quad->getOrCreateStateSet()->addUniform(saliencyUniform.get());
	quad->getOrCreateStateSet()->addUniform(w.get());
	quad->getOrCreateStateSet()->addUniform(h.get());
	quad->getOrCreateStateSet()->addUniform(issaliency.get());
	quad->getOrCreateStateSet()->addUniform(isEdgeSampleUniform.get());
	quad->getOrCreateStateSet()->addUniform(runiform.get());
	_edgeRegionCam = new osg::Camera;
	_edgeRegionCam->setViewport(0, 0, _width, _height);
	_edgeRegionCam->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
	_edgeRegionCam->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 0.0);
	_edgeRegionCam->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
	_edgeRegionCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//_edgeRegionCam->setRenderOrder(osg::Camera::POST_RENDER, 5);
	_edgeRegionCam->setRenderOrder(osg::Camera::PRE_RENDER, 5);//2014/3/23
	_edgeRegionCam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	_edgeRegionCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	_edgeRegionCam->attach(osg::Camera::COLOR_BUFFER, _edgeRegionImg.get());
	_edgeRegionCam->addChild(quad.get());

	root->addChild(_edgeRegionCam.get());

}


void LocalViewer::frame(double simulationTime)
{
	
	static int cnt =0;
	static osg::Timer_t pre=osg::Timer::instance()->tick();
	++cnt;
	if(cnt == 10)
	{
		osg::Timer_t tt=osg::Timer::instance()->tick();
		double tmp=osg::Timer::instance()->delta_m(pre,tt);
		pre =tt;
		//cout<<"绘制时间"<<tmp/10<<"\n";
		cnt=0;
	}


	if (_done) return;

	// OSG_NOTICE<<std::endl<<"CompositeViewer::frame()"<<std::endl<<std::endl;

	if (_firstFrame)
	{
		viewerInit();

		if (!isRealized())
		{
			realize();
		}

		_firstFrame = false;
	}
	
	advance(simulationTime);
	eventTraversal();
	updateTraversal();
	renderingTraversals();
}