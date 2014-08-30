#include "sampleCallback.h"
#include <iostream>
#include "LocalViewer.h"
#include <osg/Image>
#include <osgDB/WriteFile>
#include <fstream>
#include "ScreenQuad.h"

sampleCallback::sampleCallback()
{

}
void sampleCallback::operator () (osg::RenderInfo& renderInfo)
{
	std::cout<<"...."<<std::endl;
	LocalViewer* vw=dynamic_cast<LocalViewer*>(renderInfo.getView());
	int group=vw->NGROUP;
	srand(unsigned int(time(NULL)));
	for(int i=0;i<vw->LEVEL;i++)
	{
		if(i!=0)
		{
			vw->r_group->setElement(i*group,rand()%vw->NGROUP);
			vw->r_group->setElement(i*group+1,rand()%vw->NGROUP);
			vw->r_group->setElement(i*group+2,rand()%vw->NGROUP);
			vw->r_group->setElement(i*group+3,rand()%vw->NGROUP);
		}else 
		{
		   vw->r_group->setElement(i*group,0);
		   vw->r_group->setElement(i*group+1,0);
		   vw->r_group->setElement(i*group+2,0);
		   vw->r_group->setElement(i*group+3,0);
		}
		if(i>0)
			group=4;
		else 
			group=1;
		//for(int j=0;j<group;j++)
		//{
			osg::ref_ptr<osg::Uniform> seed=new osg::Uniform("seed",unsigned int(rand()));
			vw->squad->getOrCreateStateSet()->addUniform(seed.get());
			vw->squad->getOrCreateStateSet()->addUniform(vw->r_group.get());
			
		//}

	}
	
	
}