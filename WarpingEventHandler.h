#ifndef __WARPING_EVENT_HANDLE_H__
#define __WARPING_EVENT_HANDLE_H__

#include<osgGA/GUIEventHandler>
#include<osg/Geode>
#include<osg/Texture2D>
#include<osg/image>




class LocalViewer;

class WarpingEventHandler: public osgGA::GUIEventHandler
{
public:
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
protected:
	float statistic(osg::Image* img);
	void output(osg::Image* origine, osg::Image* cmp, float near, float far);
	void fullStistic(osg::Image* edge, osg::Image* depth);
	float statisticDepthImg(osg::Image* depth);
};

class WarpingGeode: public osg::Geode
{
public:
	WarpingGeode(bool tag, LocalViewer* vw,int a=0);
protected:
	osg::Geometry* createGeometry(int width, int height);
	osg::Program* createProgram(const std::string vert, const std::string frag);
};

class updataMVP: public osg::Uniform::Callback
{
public:
	updataMVP(LocalViewer* view)
		:view(view) {}
	virtual void operator() (osg::Uniform* uniform, osg::NodeVisitor* nv);
private:
	LocalViewer* view;
};
#endif