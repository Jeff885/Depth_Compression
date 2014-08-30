#include "DepthCallback.h"
#include <iostream>
#include "LocalViewer.h"
#include <osg/Image>
#include <osgDB/WriteFile>
#include <fstream>
DepthCallback::DepthCallback():_iframe(0),_isfirst(false)
{

}
void DepthCallback::operator () (osg::RenderInfo& renderInfo) const
{
	//LocalViewer* lv=dynamic_cast<LocalViewer* >(renderInfo.getView());
	//osgDB::writeImageFile(*(*lv)._msedge,"msedge.bmp");
	//depth to disparity
	//LocalViewer* lv=dynamic_cast<LocalViewer* >(renderInfo.getView());
	//osg::ref_ptr<osg::Image> cp=new osg::Image(*(lv->_edgeImg.get()),osg::CopyOp::DEEP_COPY_IMAGES);
	//int dataCnt=0;
	//int i,j;
	//int dataSize = cp->s() * cp->t(), dataCnt = 0;
	//float* srcData = reinterpret_cast<float*>( cp->data());
	//int minx=10000,miny=10000;
	//int maxx=0,maxy=0;
	//bool isfirst=true;
	//int index_a=0;
	//int index_b=0;
	/*
	for(i = 0; i < dataSize; ++i)
	{	
		if(srcData[2*i + 1] > 0.001)
		{

			++dataCnt;
		}
	}*/
	//i:y;j:x
	/*
	for(int i=0;i<cp->s();i++)
	{
		for(int j=0;j<cp->t();j++)
		{
			if(srcData[2*(i*cp->s()+j)+1]>0.001)
			{
				++dataCnt;
				
				if(j<minx)
				{
					minx=j;
				}
				
				if(j>maxx)
				{
					maxx=j;
				}

				if(i<miny)
				{
					miny=i;
				}
				
				if(i>maxy)
				{
					maxy=i;
				}
			}
		}
	}*/
	//cout <<"采样点个数: "<<dataCnt<</*" minx: "<<minx<<" maxx: "<<maxx<<" miny: "<<miny<<" maxy: "<<maxy<<*/endl;

	
	///2014/3/2注销
	/*2014/3/2
	osg::ref_ptr<osg::Image> depth=new osg::Image(*(lv->_depthImg.get()),osg::CopyOp::DEEP_COPY_IMAGES);
	osg::ref_ptr<osg::Image> disparity=new osg::Image;
	int w=lv->_width;
	int h=lv->_height;
	disparity->allocateImage(w,h,1,GL_RGB,GL_UNSIGNED_BYTE);
	unsigned char * disp=reinterpret_cast<unsigned char*>(disparity->data());
	float * tmp= reinterpret_cast<float*>(depth->data());
	float minz=lv->mNear,maxz=lv->mFar;
	unsigned char z;
	for(int i=0;i<w*h;i++)
	{
		//z=255*((tmp[i]-1/maxz)/(1/minz-1/maxz));
		//z=255*((tmp[i]-minz)*1.0/(maxz-minz)*1.0);
		z=255*tmp[i];
		disp[3*i]=z;
		disp[3*i+1]=z;
		disp[3*i+2]=z;
	}
	osg::Matrixf mat=lv->getCamera()->getViewMatrix()*lv->getCamera()->getProjectionMatrix()*lv->getCamera()->getViewport()->computeWindowMatrix();
	mat=osg::Matrixf::inverse(mat);2014/3/2*/
	//lv->getCamera()->getwi
	//DepthResidual(depth,mat);
	//restore(*(depth.get()),string("depth.txt"),w,h);
	//osgDB::writeImageFile(*(disparity.get()),"disparity.bmp");


	//2014.2.19

	//osg::ref_ptr<osg::Image> depth1=new osg::Image(*(lv->pdepthImge.get()),osg::CopyOp::DEEP_COPY_IMAGES);
	//restore(*(depth1.get()),string("dfdepth.txt"),w,h);
	//

}
void DepthCallback::restore(osg::Image& src,string& filename,int w,int h) const
{
	float * tmp= reinterpret_cast<float*>(src.data());
	fstream fout;
	fout.open(filename,ios::out);
	for(int i=0;i<h;i++)
	{
		for(int j=0;j<w;j++)
		{
			fout<<tmp[i*w+j]<<" ";
		}
		fout<<endl;
	}
	fout.close();
}
//计算Depth Residual
void DepthCallback::DepthResidual(osg::ref_ptr<osg::Image>& depth,osg::Matrixf& m) const
{
	//std::cout<<"计算深度差: "<<std::endl;
	if(!_isfirst)
	{
		_previousDepth=depth;
		osgDB::writeImageFile(*_previousDepth.get(),"original"+_iframe);
		mat=m;
		_isfirst=true;
	}
	else
	{	//存储原始的深度图像

		osgDB::writeImageFile(*depth.get(),"original"+_iframe);

		//计算差两帧之间的深度差


	}
	_iframe++;
}