#ifndef __SCREEN_QUAD_H__
#define __SCREEN_QUAD_H__

#include <osg/Geode>
#include <osg/Texture2D>
//#include "sampleCallback.h"
class ScreenQuad: public osg::Geode
{
public:
	ScreenQuad();
	/*void setProgram(osg::Program* program);
	void setTextureToUniform(osg::Texture2D* tex, int num, std::string name);*/
	//friend class sampleCallback
};
#endif