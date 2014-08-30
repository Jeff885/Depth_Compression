#ifndef DEPTH_CALLBACK_H
#define DEPTH_CALLBACK_H
#include <osg/Camera>
#include <osgDB/WriteFile>
#include <osg/Image>
#include <osg/Matrixf>
#include <string>

using namespace std;

const int MAX_FRAMES=5;
class DepthCallback: public osg::Camera::DrawCallback 
{
public:
	DepthCallback();
	osg::ref_ptr<osg::Image> reference[MAX_FRAMES];
	mutable int _iframe;

	mutable osg::Matrixf mat;
	mutable bool _isfirst;

	mutable osg::ref_ptr<osg::Image> _previousDepth;

	void DepthResidual(osg::ref_ptr<osg::Image>& depth,osg::Matrixf& m) const;
	virtual void operator () (osg::RenderInfo& renderInfo) const;
	void restore(osg::Image& src,string& filename,int w,int h) const;
};
#endif