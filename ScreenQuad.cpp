#include "ScreenQuad.h"
#include <osg/Geometry>


ScreenQuad::ScreenQuad()
{
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	vertices->push_back(osg::Vec3(-1.0f, -1.0f, 0.0f));
	vertices->push_back(osg::Vec3(1.0f, -1.0f, 0.0f));
	vertices->push_back(osg::Vec3(1.0f, 1.0f, 0.0f));
	vertices->push_back(osg::Vec3(-1.0f, 1.0f, 0.0f));

	osg::ref_ptr<osg::Vec2Array> texcoords = new osg::Vec2Array;
	texcoords->push_back(osg::Vec2(0.0f, 0.0f));
	texcoords->push_back(osg::Vec2(1.0f, 0.0f));
	texcoords->push_back(osg::Vec2(1.0f, 1.0f));
	texcoords->push_back(osg::Vec2(0.0f, 1.0f));

	osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
	quad->setVertexArray(vertices.get());
	quad->setTexCoordArray(0, texcoords.get());
	quad->addPrimitiveSet( new osg::DrawArrays(GL_QUADS, 0, 4) );
	this->addDrawable(quad.get());
}

//void ScreenQuad::setProgram(osg::Program* program)
//{
//	this->getOrCreateStateSet()->setAttributeAndModes(program);
//}
//
//void ScreenQuad::setTextureToUniform(osg::Texture2D* tex, int num, std::string name)
//{
//	this->getOrCreateStateSet()->setTextureAttributeAndModes(num, tex);
//	osg::ref_ptr<osg::Uniform> uniformName = new osg::Uniform(name.c_str(), num);
//	this->getOrCreateStateSet()->addUniform(uniformName.get());
//}

