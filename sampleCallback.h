#ifndef SAMPLE_CALLBACK_H
#define SAMPLE_CALLBACK_H
#include <osg/Camera>
#include <osgDB/WriteFile>
#include <osg/Image>
#include <osg/Matrixf>
#include <string>

class sampleCallback: public osg::Camera::DrawCallback 
{
public:
	sampleCallback();
	virtual void operator () (osg::RenderInfo& renderInfo);
	
};
#endif