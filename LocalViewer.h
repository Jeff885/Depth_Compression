#ifndef  __LOCAL_VIEWER_H__
#define  __LOCAL_VIEWER_H__

#include <osgViewer/Viewer>
#include <osg/Texture2D>
//#include <osgPPU/Processor.h>
#include <osg/Camera>
#include <string>
#include "sampleCallback.h"
#include "ScreenQuad.h"
#include <osgText/Font>
#include <osgText/Text>

class  LocalViewer: public osgViewer::Viewer
{
public:
	class getNearFarCallback: public osg::Camera::DrawCallback
	{
	public:
		virtual void operator () (osg::RenderInfo& renderInfo) const;
	};

	class getProcessTime: public osg::Camera::DrawCallback
	{
	public:
		static osg::Timer_t  gTime;
		static bool isFinished;
		virtual void operator() (osg::RenderInfo& renderInfo) const;
	};

	class updataNear: public osg::Uniform::Callback
	{
	public:
		updataNear(LocalViewer* view)
			:view(view) {}
		virtual void operator() (osg::Uniform* uniform, osg::NodeVisitor* nv);
	private:
		LocalViewer* view;
	};

	class updateSeed :public osg::Uniform::Callback
	{
	public:
		updateSeed(LocalViewer* view)
			:view(view){
				
		}
		virtual void operator()(osg::Uniform* uniform,osg::NodeVisitor* nv);
	private:
		LocalViewer* view;
	};
	class updateGroup:public osg::Uniform::Callback
	{
	public:
		updateGroup(LocalViewer* view):view(view){}
		virtual void operator() (osg::Uniform* uniform,osg::NodeVisitor* nv);
	private:
		LocalViewer* view;
	};

	class updataFar: public osg::Uniform::Callback
	{
	public:
		updataFar(LocalViewer* view)
			:view(view) {}
		virtual void operator() (osg::Uniform* uniform, osg::NodeVisitor* nv);
	private:
		LocalViewer* view;
	};
	virtual void frame(double simulationTime);
	static std::string DATAPATH;
	static std::string MSDATAPATH;//微软深度图像加载
	osg::ref_ptr<osg::Image>  _msedge;
	osg::ref_ptr<osg::Image>  _msdepthImg;
	LocalViewer(int width, int height);

	~LocalViewer();

protected:

	osg::Camera* createCamera(int leftTopX, int leftTopY,int width, int height);

	osg::Node*  loadModel();
	osg::Node*  loadSaliencyModel();//载入网格显著度
	//2014_8_5
	void createTextShow(osg::Group* root);
	void createCmpTextShow(osg::Group* root);
	void createPoissonTextShow(osg::Group* root);

	osgText::Text* text;
	osgText::Font* font;
	//2014_8_14
	osgText::Text* _cmptext;
	osgText::Text* _poissontext;
	osgText::Font* _cmpfont;
	osgText::Font* _poissonfont;

	osg::Camera* camera;
	osg::Camera* _cmptextcam;
	osg::Camera* _poissontextcam;

	void createSaliencyMap();//创建显著度Camera;
	//void createSaliencyRegion();

	void scenePrepare();

	/*2014_5_15  微软深度信息*/
	void createMSRDepth();//微软深度数据加载处理
	void createMSRDepthEdge();//微软深度数据边缘
	void createMSRDepthImage();
	
	void createMasterCamera();

	//slave camera
	void createShowCamera();
	void createWarpingCamera();
	void createLowToHighCamera();

	osg::Program* createProgram(const std::string vert, const std::string frag);

	void createDiffuseCameras(osg::Group* root);

	void createTest(osg::Group* root);
	void createVisiblePointCamera();
	void createEdgeCamera();
	void createPoissoncamera();
	void testRand();
	void createPoissionSample();
	void createShowPoissonsampleCam();
	void createShowPoissonsampleCamAnother();
	void createSampleCam();
	void createDiffuseCamerasCmp(osg::Group* root);
	void createEdgeRegion();
	void createEdgeRegionShowCam();
	void createBUSampleCam();//自底向上采样，快速收敛
	void createShowSampleCam1();//实验截图
	void createShowSampleCam2();//实验截图
	void createShowSampleCam3();//实验截图
	void createShowSampleCam4();//实验截图
	void createShowSampleCam5();//实验截图
	void createDiffuseCamerasCmpForShow(osg::Group* root);//实验截图
	void createshowSmooth5();
	void createshowSmooth4();
	void createshowSmooth3();
	void createshowSmooth2();
	void createshowSmooth1();//实验截图
	void createshowInterpolation5();
	void createshowInterpolation4();
	void createshowInterpolation3();
	void createshowInterpolation2();
	void createshowInterpolation1();//实验截图
	void createPostDeal();//实验截图
	void createPSNRCam();//实验截图
private:

	//微软深度数据
	osg::ref_ptr<osg::Image> _msdepth;
	osg::ref_ptr<osg::Camera> _msDepthToZCam;
	osg::ref_ptr<osg::Camera> _msZToDepthCam;

	//SaliencyCamera
	osg::ref_ptr<osg::Camera> _saliencyCamera;
	osg::ref_ptr<osg::Image>  _saliencymap;//可见的显著度map
	//osg::ref_ptr<osg::Camera> _saliencyRegionCamera;
	//osg::ref_ptr<osg::Image>  _saliencyRegion;
	//统计
	osg::ref_ptr<osg::Image> _staticEdgeImg;
	//实验截图
	osg::ref_ptr<osg::Image> _regularImg;
	osg::ref_ptr<osg::Image> _poissonImg;
	osg::ref_ptr<osg::Image> _originalImg;
	osg::ref_ptr<osg::Camera> regularcam;
	osg::ref_ptr<osg::Camera> poissoncam;
	osg::ref_ptr<osg::Camera> originalcam;
	//实验截图

	osg::ref_ptr<osg::Image> edsImg[6];//2014/3/31
	osg::ref_ptr<osg::Texture2D> edsTex[6];//2014/2/13边缘纹理只保留边缘
	osg::ref_ptr<osg::Texture2D> dsTex[6];//
	osg::ref_ptr<osg::Texture2D> dfTex[6];
	osg::ref_ptr<osg::Texture2D> smTex[6];
	osg::ref_ptr<osg::Image> dsImg[6];//实验截图
	osg::ref_ptr<osg::Image> dfImg[6];//实验截图 
	osg::ref_ptr<osg::Image> smImg[6];//实验截图
	//
	//2014/3/25
	osg::ref_ptr<osg::Image> img;//3/27
	const static int BLEVEL=8;
	const static int ULEVEL=5;
	const static int BUNGROUP=4;
	osg::ref_ptr<osg::Camera> bulevelcamera[BLEVEL-ULEVEL+2][BUNGROUP];
	osg::ref_ptr<osg::Texture2D> buleveltexture[BLEVEL-ULEVEL+2][BUNGROUP];
	osg::ref_ptr<osg::Image>     bulevelimg[BLEVEL-ULEVEL+2][BUNGROUP];
	osg::ref_ptr<ScreenQuad> busquad;
	osg::ref_ptr<osg::Uniform> bur_group;
	//
	//2014/3/16 Poissonsample
	const static int LEVEL=10;
	const static  int NGROUP=4;
	osg::ref_ptr<osg::Camera> levelcamera[LEVEL][NGROUP];
	osg::ref_ptr<osg::Texture2D> leveltexture[LEVEL][NGROUP];
	osg::ref_ptr<osg::Image>     levelimg[LEVEL][NGROUP];
	//osg::ref_ptr<osg::Uniform> seed[LEVEL][NGROUP];
	osg::ref_ptr<ScreenQuad> squad;
 	osg::ref_ptr<osg::Uniform> r_group;
	//int timeseed;//不需要一个变量
	//2014/3/18
	//2014/3/20 poisson disk edge sample
	osg::ref_ptr<osg::Camera>    sampleCam;
	osg::ref_ptr<osg::Texture2D> sampleTex;
	osg::ref_ptr<osg::Image>     sampleImg;
	//2014/3/20
	//2014/3/22显著度区域生成
	osg::ref_ptr<osg::Camera>  _edgeRegionCam;
	osg::ref_ptr<osg::Image>   _edgeRegionImg;
	osg::ref_ptr<osg::Camera>  _edgeRegionShowCam;
	//
	osg::ref_ptr<osg::Camera> showpoissonsamplecam;
	osg::ref_ptr<osg::Camera> showpoissonsamplecamanother;
	//
	osg::ref_ptr<osg::Camera> _masterCam;
	
	//a slave camera for showing intermediate result;
	osg::ref_ptr<osg::Camera>		_showSlaveCam;

	osg::ref_ptr<osg::Camera>		_warpingCam;

	osg::ref_ptr<osg::Camera>		_warpingCmpCam;
	osg::ref_ptr<osg::Camera>       _warpingpoissoncam;
	
	osg::ref_ptr<osg::Camera>		_lowToHighCam;

	//camera for capture downsample  color frame, it is different from master camera
	// which shows original color frame;
	osg::ref_ptr<osg::Camera>		_imageCam;

	osg::ref_ptr<osg::Camera>		_imageCam2;

	//camera for capture original depth buffer(frame) corresponding to master camera
	osg::ref_ptr<osg::Camera>		_depthCam;

	osg::ref_ptr<osg::Camera>		_depthCam2;

	//camera for edge detection on depth buffer, just using  a laplace detector;
	//for simple, it uses another camera for post processing;
	osg::ref_ptr<osg::Camera>		_detectCam;
	osg::ref_ptr<osg::Camera>	     _detectCam2;
	osg::ref_ptr<osg::Camera>        _edgecam;//2014/2/28


	//processor for depth reconstruction from edge sample texture from detect camera;
	//it is a complex process, so use a osgPPU processor.
	//osg::ref_ptr<osgPPU::Processor>  _diffuseProc;

	//2014.2.19
	//可见顶点的提取
	osg::ref_ptr<osg::Camera>    _visiblePointCam;
	//2014.2.20
	osg::ref_ptr<osg::Camera>    _realDepthCam;
	osg::ref_ptr<osg::Image>     _realDepthImage;
	osg::ref_ptr<osg::Image>     pdepthImge;
	osg::ref_ptr<osg::Image>     pcolorImage;
	///2.10
	osg::ref_ptr<osg::Texture2D>	_imageTex;
	osg::ref_ptr<osg::Texture2D>	_depthTex;
	osg::ref_ptr<osg::Texture2D>	_edgeTex;
	osg::ref_ptr<osg::Texture2D>    stex;///2014/2/25

	osg::Matrixf _iMVPW;
	osg::Matrixf _uMVP;

	osg::ref_ptr<osg::Image>	_colorImg;
	osg::ref_ptr<osg::Image>	_lowColorImg;
	osg::ref_ptr<osg::Image>  _edgeImg;

	osg::ref_ptr<osg::Image>	_depthImg;
	osg::ref_ptr<osg::Image>	_dfDepthImg;
	osg::ref_ptr<osg::Image>    _ddfDepthImg;

	osg::ref_ptr<osg::Node>  _modelNode;
	osg::ref_ptr<osg::Group> root;

	float  _DsFactorX;
	float  _DsFactorY;

	int  _width;
	int  _height;


	float  mNear;
	float mFar;

	bool  mIsShow;

	friend class getNearFarCallback;
	friend class updataNear;
	friend class updataFar;
	friend class updataMVP;
	friend class WarpingEventHandler;
	friend class WarpingGeode;
	friend class DepthCallback;
	friend class sampleCallback;
	friend class updateSeed;
	friend class updateGroup;
};
#endif