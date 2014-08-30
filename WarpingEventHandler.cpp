#include "WarpingEventHandler.h"
#include "LocalViewer.h"
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <osgDB/WriteFile>
//#include <zlib.h>
using namespace std;
std::string  PATH = "E:\\3PLib\\OpenSceneGraph-3.0.1\\OpenSceneGraph-Data-3.0.0\\";

bool WarpingEventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	LocalViewer* vw = dynamic_cast<LocalViewer*> (&aa);
	if(vw)
	{
		switch(ea.getEventType())
		{
		case osgGA::GUIEventAdapter::KEYDOWN:
			switch(ea.getKey())
			{
			case 'w': case 'W':
				{
					double fov, aspect, near, far;
					vw->getCamera()->getProjectionMatrixAsPerspective(fov, aspect, near, far);
					std::cout << std::fixed << "near:"<<near << "far: " << far << std::endl; 
					osg::Vec3d eye, center, up;
					float distance;
					osgGA::CameraManipulator *cman = vw->getCameraManipulator();
					osgGA::TrackballManipulator * tt = dynamic_cast<osgGA::TrackballManipulator*>(cman);
					if(tt)
					{
						distance = tt->getDistance();
						std::cout << std::fixed << "distance: " << distance << std::endl;
						tt->getTransformation(eye, center, up);
						std::cout << std::fixed << "eye: <" << eye._v[0]<<" , " << eye._v[1] << "," << eye._v[2] << " >" << std::endl;
						std::cout << std::fixed << "center: <" << center._v[0]<<" , " << center._v[1] << "," << center._v[2] << " >" << std::endl;
						std::cout << std::fixed << "up: <" << up._v[0]<<" , " << up._v[1] << "," << up._v[2] << " >" << std::endl;
					}
					//output(vw->_depthImg, vw->_dfDepthImg, near, far);
					//output(vw->_depthImg, vw->_ddfDepthImg, near, far);
					//output(vw->_depthImg, vw->sampleImg, near, far);
					//output(vw->_depthImg,vw->_edgeImg,near,far);
					//fullStistic(vw->_edgeImg, vw->_depthImg);
					//cout<<"边缘:"<<endl;
					//statistic(vw->_msedge);
					cout<<"..............."<<endl;
					float a=statistic(vw->_staticEdgeImg);
					cout<<"..............."<<endl;
					//fullStistic(vw->sampleImg, vw->_depthImg);
					float b=statistic(vw->sampleImg);
					cout<<"________________"<<endl;
					float c=statisticDepthImg(vw->_depthImg);
					//float a=vw->_st,b;
					char cs[10];
					char cm[10];
					sprintf(cs,"%.3f",a/c);
					sprintf(cm,"%.3f",b/c);
					string es(cs),ms(cm);
					//vw->text->setText("CR2:"+ms+" "+"CR1:"+es);

					vw->_cmptext->setText("CR: "+ es);
					vw->_poissontext->setText("CR: "+ ms);

					//statisticDepthImg(vw->_msdepth);
					//statistic(vw->_depthImg);
					
					/*vw->_iMVPW = vw->getCamera()->getViewMatrix() * vw->getCamera()->getProjectionMatrix()*
						vw->getCamera()->getViewport()->computeWindowMatrix();
					vw->_iMVPW = osg::Matrix::inverse(vw->_iMVPW);*/
					//2014/6/6
					/*int i;
					for(i = 0; i< vw->_warpingCam->getNumChildren();++i)
						vw->_warpingCam->removeChild(i);
					for(i = 0; i< vw->_warpingCmpCam->getNumChildren();++i)
						vw->_warpingCmpCam->removeChild(i);
					for(i = 0; i< vw->_warpingpoissoncam->getNumChildren();++i)
						vw->_warpingpoissoncam->removeChild(i);

					for(i=0;i<vw->originalcam->getNumChildren();i++)
						vw->originalcam->removeChild(i);
					for(i=0;i<vw->regularcam->getNumChildren();i++)
						vw->regularcam->removeChild(i);
					for(i=0;i<vw->poissoncam->getNumChildren();i++)
						vw->poissoncam->removeChild(i);*/

					vw->_warpingCam->addChild(new WarpingGeode(true, vw));
					vw->_warpingCmpCam->addChild(new WarpingGeode(false, vw));
					vw->_warpingpoissoncam->addChild(new WarpingGeode(false,vw,2));
					//vw->text->setText()
					///截图
					//vw->originalcam->addChild(new WarpingGeode(true, vw));
					//vw->regularcam->addChild(new WarpingGeode(false, vw));
					//vw->poissoncam->addChild(new WarpingGeode(false,vw,2));
					break;
				}
			case 'S': case 's':
				{
					osg::Vec3d eye, center, up;
					float distance;
					osgGA::CameraManipulator *cman = vw->getCameraManipulator();
					osgGA::TrackballManipulator * tt = dynamic_cast<osgGA::TrackballManipulator*>(cman);
					if(tt)
					{
						distance = tt->getDistance();
						std::cout << std::fixed << "distance: " << distance << std::endl;
						tt->getTransformation(eye, center, up);
						std::cout << std::fixed << "eye: <" << eye._v[0]<<" , " << eye._v[1] << "," << eye._v[2] << " >" << std::endl;
						std::cout << std::fixed << "center: <" << center._v[0]<<" , " << center._v[1] << "," << center._v[2] << " >" << std::endl;
						std::cout << std::fixed << "up: <" << up._v[0]<<" , " << up._v[1] << "," << up._v[2] << " >" << std::endl;
					}
					break;
				}
			case 'A':case'a'://初始视点
				{
					//arm
					//eye: <2.720753 , 95.302281,32.240155 >
                    //center: <0.000000 , 0.000000,0.000000 >
                    //up: <-0.010572 , -0.320165,0.947303 >

					//david
					
					//eye: <55.200101 , 33.310480,-75.968029 >
					//center: <0.000000 , 0.000000,0.000000 >
					//up: <-0.453255 , 0.889318,0.060603 >

					//bunny
					//eye: <-4.922177 , -87.266594,-41.317331 >
					//center: <0.000000 , 0.000000,0.000000 >
					//up: <-0.951793 , -0.085754,0.294509 >

					//gargoyle
					//osg::Vec3d eye(9.388852, -30.636539,88.684721), center(0.000000 , 0.000000,0.000000), up(0.968091, 0.250080,-0.016098);

					//buddha.ply
					//osg::Vec3d eye(-17.015578, 96.332009,40.384904), center(0.000000 , 0.000000,0.000000), up(-0.980250, -0.102159,-0.169330);
					//osg::Vec3d eye(2.720753 , 95.302281,32.240155), center(0.000000 , 0.000000,0.000000), up(-0.010572 , -0.320165,0.947303);
					//igea.ply
					//osg::Vec3d eye(-62.955920, 44.499968,-37.481057), center(0.000000 , 0.000000,0.000000), up(-0.641233, -0.319585,0.697629);
					//angel.ply
					//osg::Vec3d eye(-59.995420, -50.534752,4.925380), center(6.411436, 2.282095,3.249767), up(0.503649, -0.613958,0.607778);
					//teeth.ply
					osg::Vec3d eye(-32.065239, -52.543163,58.423758), center(0.000000 , 0.000000,0.000000), up(-0.924784, 0.216293,-0.313036);
					//bimba
					//osg::Vec3d eye(17.130279 , 91.971103,11.814963), center(0.000000 , 0.000000,0.000000), up(0.982956, -0.176456,-0.051583);
					osgGA::CameraManipulator *cman = vw->getCameraManipulator();
					osgGA::TrackballManipulator * tt = dynamic_cast<osgGA::TrackballManipulator*>(cman);
					if(tt)
					{
						tt->setTransformation(eye,center,up);
					}
					break;
				}
			case 'Z':case 'z'://Warping视点
				{
					//gargoyle.ply
					//osg::Vec3d eye(19.760107 , -64.205457,66.173448), center(0.000000 , 0.000000,0.000000), up(0.945401 , 0.324300,0.032348);

					//buddha.ply
					//osg::Vec3d eye(-32.106762, 35.379993,94.433777), center(0.000000 , 0.000000,0.000000), up(-0.950644, -0.042201,-0.307401);
					
					//osg::Vec3d eye(-17.270310,-164.991029,-102.838749), center(0.000008,21.419441,-0.000252), up(0.060410,0.477867,-0.876352);

					//igea.ply
					//osg::Vec3d eye(-67.235687, -0.220535,-53.177425), center(0.000000 , 0.000000,0.000000), up(-0.586958, -0.320547,0.743458);
					//angel.ply
					osg::Vec3d eye(-31.784643, -73.475398,5.289204), center(6.411436, 2.282095,3.249767), up(0.743802, -0.359853,0.563263);
					//teeth.ply
					//osg::Vec3d eye(-32.735374, -77.495477,11.186844), center(0.000000, 0.000000,0.000000), up(-0.922054, 0.376572,-0.089494 );
					osgGA::CameraManipulator *cman = vw->getCameraManipulator();
					osgGA::TrackballManipulator * tt = dynamic_cast<osgGA::TrackballManipulator*>(cman);
					if(tt)
					{
						tt->setTransformation(eye,center,up);
					}
					break;
				}
			case 'B':case 'b':
				{
					if(vw->originalcam->getNumChildren()<1)
					{
						break;
					}

					osg::ref_ptr<osg::Image> o=new osg::Image(*(vw->_originalImg).get(),osg::CopyOp::DEEP_COPY_IMAGES);
					osg::ref_ptr<osg::Image> r=new osg::Image(*(vw->_regularImg).get(),osg::CopyOp::DEEP_COPY_IMAGES);
					osg::ref_ptr<osg::Image> p=new osg::Image(*(vw->_poissonImg).get(),osg::CopyOp::DEEP_COPY_IMAGES);

					/*unsigned char* o_data=reinterpret_cast<unsigned char*>(o->data());
					unsigned char* r_data=reinterpret_cast<unsigned char*>(r->data());
					unsigned char* p_data=reinterpret_cast<unsigned char*>(p->data());
					int width=o->s(),height=o->t();
					int or_mse[3]={0,0,0},op_mse[3]={0,0,0};//0:R,1:G;2:B
					for(int i=0;i<3*width*height;i+=3)
					{
						or_mse[0]+=(int(r_data[i+0])-int(o_data[i+0]))*(int(r_data[i+0])-int(o_data[i+0]));
						or_mse[1]+=((int(r_data[i+1])-int(o_data[i+1])))*(int(r_data[i+1])-int(o_data[i+1]));
						or_mse[2]+=int((r_data[i+2]-o_data[i+2]))*int((r_data[i+2]-o_data[i+2]));

						op_mse[0]+=int((p_data[i+0]-o_data[i+0]))*int((p_data[i+0]-o_data[i+0]));
						op_mse[1]+=int((p_data[i+1]-o_data[i+1]))*int((p_data[i+1]-o_data[i+1]));
						op_mse[2]+=int((p_data[i+2]-o_data[i+2]))*int((p_data[i+2]-o_data[i+2]));
					}
					//cout<<"int(r_data[0])"<<int(r_data[0])-int(o_data[0])<<endl;
					//cout<<"和：    "<<or_mse[0]/(width*height)<<endl;
					cout<<"!!!!!!!:::"<<20*log10(255*1.0/sqrt(double(double(or_mse[0])*1.0/(width*height))))<<endl;
					//cout<<or_mse[0]<<"       "<<or_mse[1]<<"        "<<or_mse[2]<<endl;
					cout<<"规则采样的PSNR："<<
						(20*log10(255*1.0/sqrt(double(double(or_mse[0])*1.0/(width*height))))
						+20*log10(255*1.0/sqrt(double(double(or_mse[1])*1.0/(width*height))))
						+20*log10(255*1.0/sqrt(double(double(or_mse[2])*1.0/(width*height)))))/3<<endl;
					cout<<"Poisson采样的PSNR："<<
						(20*log10(255*1.0/sqrt(double(double(op_mse[0])*1.0/(width*height))))
						+20*log10(255*1.0/sqrt(double(double(op_mse[1])*1.0/(width*height))))
						+20*log10(255*1.0/sqrt(double(double(op_mse[2])*1.0/(width*height)))))/3<<endl;*/
					osgDB::writeImageFile(*o,"original.bmp");
					osgDB::writeImageFile(*r,"regular.bmp");
					osgDB::writeImageFile(*p,"poisson.bmp");
					
					break;
				}
			case 'M':case 'm':
				{
					//arm
					//eye: <16.695519 , 94.222671,31.188183 >
					//center: <0.000000 , 0.000000,0.000000 >
					//up: <-0.030994 , -0.309133,0.950514 >

					//david
					//eye: <45.396080 , 27.684363,-84.264776 >
					//center: <0.000000 , 0.000000,0.000000 >
					//up: <-0.444591 , 0.894090,0.054229 >

					//bunny
					//eye: <18.405135 , -91.493477,-25.238915 >
                    //center: <0.000000 , 0.000000,0.000000 >
                    //up: <-0.945487 , -0.248318,0.210694 >

					//gargoyle
					//eye: <20.984785 , -75.020168,53.135120 >
                    //center: <0.000000 , 0.000000,0.000000 >
                    //up: <0.951619 , 0.302883,0.051808 >
					//osg::Vec3d eye(20.984785, -75.020168,53.135120), center(0.000000 , 0.000000,0.000000), up(0.951619, 0.302883,0.051808);

					//buddha.ply
					//osg::Vec3d eye(-23.777140, 76.361135,69.310540), center(0.000000 , 0.000000,0.000000), up(-0.969455, -0.097646,-0.224994);
					//osg::Vec3d eye(18.405135 , -91.493477,-25.238915), center(0.000000 , 0.000000,0.000000), up(-0.945487 , -0.248318,0.210694);
					
					//igea.ply
					//osg::Vec3d eye(-59.984151, -22.078388,-57.122422), center(0.000000 , 0.000000,0.000000), up(-0.590920, -0.315512,0.742473);
					//angel.ply
					//osg::Vec3d eye(-16.290968, -79.453923,0.775573), center(6.411436, 2.282095,3.249767), up(0.784377, -0.235237,0.573948);
					//teeth.ply
					osg::Vec3d eye(-27.307857, -78.204884,-18.454705), center(0.000000, 0.000000,0.000000), up(-0.940606, 0.337381,-0.037871 );

					//bimba
					//osg::Vec3d eye(14.897848 , 79.456521,48.543166), center(0,0,0), up(0.987216 , -0.123674,-0.100543);
					osgGA::CameraManipulator *cman = vw->getCameraManipulator();
					osgGA::TrackballManipulator * tt = dynamic_cast<osgGA::TrackballManipulator*>(cman);
					if(tt)
					{
						tt->setTransformation(eye,center,up);
					}

					break;
				}
			case 'C':case 'c':
				{
					output(vw->_msedge.get(),NULL,42,130);
					osg::ref_ptr<osg::Image> _msdep=new osg::Image(*vw->_msdepthImg.get(),osg::CopyOp::DEEP_COPY_IMAGES);
					osgDB::writeImageFile(*_msdep,"msdepRecover.png");
					break;
				}
			case 'D': case 'd':
				{

					osg::ref_ptr<osg::Image> df=new osg::Image(*vw->_ddfDepthImg.get(),osg::CopyOp::DEEP_COPY_IMAGES);
					osg::ref_ptr<osg::Image> ds=new osg::Image(*vw->_depthImg.get(),osg::CopyOp::DEEP_COPY_IMAGES);
					output(ds,df,vw->mNear,vw->mFar);
					break;
				}
			case 'p':case 'P':
				{
					osg::ref_ptr<osg::Image> colorImg = new osg::Image(*(vw->_saliencymap), osg::CopyOp::DEEP_COPY_IMAGES);
					osg::ref_ptr<osg::Image> imgtmp=new osg::Image(*(vw->img).get(),osg::CopyOp::DEEP_COPY_IMAGES);
					osg::ref_ptr<osg::Image> img=new osg::Image(*(vw->_depthImg).get(),osg::CopyOp::DEEP_COPY_IMAGES);
					osg::ref_ptr<osg::Image> thecolor=new osg::Image(*(vw->_colorImg).get(),osg::CopyOp::DEEP_COPY_IMAGES);
					osgDB::writeImageFile(*thecolor,"color_buffer.bmp");
					osgDB::writeImageFile(*colorImg,"color.bmp");

					/*float * tmp= reinterpret_cast<float*>(imgtmp->data());
					fstream fout;
					int h=vw->_height;
					int w=vw->_width;
					fout.open("sample",ios::out);
					for(int i=0;i<h;i++)
					{
						for(int j=0;j<w;j++)
						{
							fout<<tmp[i*w+j]<<" ";
						}
						fout<<endl;
					}
					fout.close();

					float * tmp1= reinterpret_cast<float*>(img->data());
					//fstream fout;
					//int h=vw->_height;
					//int w=vw->_width;
					fout.open("depth",ios::out);
					for(int i=0;i<h;i++)
					{
						for(int j=0;j<w;j++)
						{
							fout<<tmp1[i*w+j]<<" ";
						}
						fout<<endl;
					}
					fout.close();
					
					break;*/
				}
			default:
				break;
			}
			break;
		default:
			break;
		}
	}
	return false;
}
float WarpingEventHandler::statisticDepthImg(osg::Image* depth)
{
	float* data=reinterpret_cast<float*>(depth->data());
	int width = depth->s();
	int height = depth->t();
	int i,cnt=0;
	for(i=0;i<width*height;i++)
		if(1-data[i] >0.00001) cnt++;
	std::cout << "The total depth point is: " << cnt << std::endl;
	return cnt;
}

float WarpingEventHandler::statistic(osg::Image* img)
{
	float* data = reinterpret_cast<float*>(img->data());
	int width = img->s();
	int height = img->t();
	int i , cnt = 0;
	for(i = 0; i < width*height; ++i)
		if(data[2*i + 1] > 0.00005) cnt++;
	std::cout << "total sample number is: " << cnt << std::endl;
	return cnt;
}

void WarpingEventHandler::fullStistic(osg::Image* edge, osg::Image* depth)
{
	static unsigned char * source = new unsigned char[2200000];
	static unsigned char * dst = new unsigned char[2200000];
	static int cnt = 0;
	static double d_tot_samples = 0.0;
	static const int MAX = (1 << 16) -1 ;


	static double z_tot_size = 0.0;
	static  double  z_tot_time = 0.0;
	static double   z_tot_time_d = 0.0;
	static int  edge_max_size = 0;
	static int  zlib_max_size = 0;
	
	int width = edge->s();
	int height = edge->t();
	
	float* data = reinterpret_cast<float*>(edge->data());
	int i, j;
	int scnt = 0;
	for(i = 0; i < width*height; ++i)
		if(data[2*i + 1] > 0.00005) ++scnt;
	
	if(edge_max_size < scnt) edge_max_size = scnt;

	d_tot_samples += scnt;

	data = reinterpret_cast<float*>(depth->data());
	int idx = 0;
	for(i = 0; i < width*height; ++i)
	{
		int val = int(data[i] * MAX);
		source[idx] = ((val >> 8) & 0xff);
		source[idx + 1] = (val & 0xff);
		idx += 2;
	}
	unsigned long dstSize;
	unsigned long srcSize;

	clock_t stt = clock();
	//compress(dst, &dstSize, source, width* height * 2);
	clock_t fsh = clock();
	if(zlib_max_size < dstSize) zlib_max_size = dstSize;

//	uncompress(source, &srcSize, dst, dstSize);

	clock_t  fsh2 = clock();

	z_tot_size += dstSize;
	z_tot_time += fsh - stt;
	z_tot_time_d = fsh2 - fsh;

	++cnt;
	
	std::cout << "Avg. edge size  " << std::fixed << (d_tot_samples * 3/ cnt) / 1024.0 << "kb" << std::endl;
	std::cout << "Max edge size" << std::fixed << edge_max_size * 3/ 1024.0 << "kb" << std::endl;
	std::cout << "Avg. zlib size  " << std::fixed << (z_tot_size / cnt) / 1024.0 << "kb" << std::endl;
	std::cout << "Max zlib size" << std::fixed << zlib_max_size / 1024.0 << "kb" << std::endl;
	std::cout << "Avg. zlib time   " << std::fixed << (z_tot_time / cnt) << std::endl;
	std::cout << "Avg. zlib dd time   " << std::fixed << (z_tot_time_d / cnt) << std::endl;
}
void WarpingEventHandler::output(osg::Image* origine, osg::Image* cmp, float near, float far)
{
	static int fileCnt = 0;
	char str[10];
	itoa(fileCnt, str, 10);
	std::string folder = "model2_1\\";
	std::cout << str << std::endl;
	std::string oName = folder +"M2O" + std::string(str) + ".txt";
	std::string cName = folder + "M2C" + std::string(str)  + ".txt";
	int width = origine->s();
	int height = origine->t();
	std::fstream  outfile;
	outfile.open(oName.c_str(), std::fstream::out);
	//outfile << std::fixed << near <<" " << far;
	int i, j;
	float* data = reinterpret_cast<float*> (origine->data());
	for(i = 0; i < height; i++)
	{
		for(j = 0; j < width; j++)
		{
			int pos = (i*width + j);
			outfile << std::fixed << " " << data[pos];
		}
		outfile<<std::endl;
	}
	outfile.close();

	if(cmp==NULL)
	{
		fileCnt++;
		return;
	}
	outfile.open(cName.c_str(), std::fstream::out);
	//outfile << std::fixed << near <<" " << far;
	data = reinterpret_cast<float*> (cmp->data());
	for(i = 0; i < height; i++)
	{
		for(j = 0; j < width; j++)
		{
			int pos = (i*width + j);
			outfile << std::fixed << " " << data[pos];
		}
		outfile<<std::endl;
	}
	outfile.close();
	
	fileCnt ++;
}
WarpingGeode::WarpingGeode(bool tag, LocalViewer* vw,int a)
{
	osg::ref_ptr<osg::Texture2D> depthTex = new osg::Texture2D;
	osg::ref_ptr<osg::Texture2D> colorTex = new osg::Texture2D;
	osg::ref_ptr<osg::Texture2D> edgeTex = new osg::Texture2D;

	osg::ref_ptr<osg::Image> tmpDepth;
	tmpDepth = vw->_depthImg;
	if(!tag) tmpDepth = vw->_dfDepthImg;

	if(a!=0)
		tmpDepth=vw->_ddfDepthImg;

	osg::ref_ptr<osg::Image> colorImg = new osg::Image(*(vw->_colorImg), osg::CopyOp::DEEP_COPY_IMAGES);
	osg::ref_ptr<osg::Image> depthImg = new osg::Image(*tmpDepth, osg::CopyOp::DEEP_COPY_IMAGES);
	osg::ref_ptr<osg::Image> edgeImg = new osg::Image(*(vw->_edgeImg), osg::CopyOp::DEEP_COPY_IMAGES);
	colorTex->setImage(colorImg.get());
	colorTex->setInternalFormat(GL_RGB);
	colorTex->setResizeNonPowerOfTwoHint(false);

	depthTex->setImage(depthImg.get());
	depthTex->setInternalFormat(GL_LUMINANCE32F_ARB);
	depthTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::NEAREST);
	depthTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::NEAREST);
	depthTex->setWrap(osg::Texture2D::WRAP_S,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	depthTex->setWrap(osg::Texture2D::WRAP_T,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	depthTex->setResizeNonPowerOfTwoHint(false);

	edgeTex->setImage(edgeImg.get());
	edgeTex->setInternalFormat(GL_LUMINANCE_ALPHA);
	edgeTex->setFilter(osg::Texture2D::MIN_FILTER,
		osg::Texture2D::NEAREST);
	edgeTex->setFilter(osg::Texture2D::MAG_FILTER,
		osg::Texture2D::NEAREST);
	edgeTex->setWrap(osg::Texture2D::WRAP_S,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	edgeTex->setWrap(osg::Texture2D::WRAP_T,
		osg::Texture2D::CLAMP_TO_EDGE
		);
	edgeTex->setResizeNonPowerOfTwoHint(false);
	int width = vw->_width, height = vw->_height;

	this->addDrawable(createGeometry(width,height));

	this->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	this->getOrCreateStateSet()->setAttributeAndModes(
		createProgram(PATH + "Myshaders\\showWarping.vert",
		PATH + "Myshaders\\showWarping.frag")
		); 

	this->getOrCreateStateSet()->setTextureAttributeAndModes(0, depthTex.get());
	this->getOrCreateStateSet()->setTextureAttributeAndModes(1,  colorTex.get());
	this->getOrCreateStateSet()->setTextureAttributeAndModes(2,  edgeTex.get());

	osg::ref_ptr<osg::Uniform> depth = new osg::Uniform("depth", 0);
	this->getOrCreateStateSet()->addUniform(depth.get());

	osg::ref_ptr<osg::Uniform> color = new osg::Uniform("color", 1);
	this->getOrCreateStateSet()->addUniform(color.get());

	osg::ref_ptr<osg::Uniform> edge = new osg::Uniform("edge", 2);
	this->getOrCreateStateSet()->addUniform(edge.get());

	osg::ref_ptr<osg::Uniform> size = new osg::Uniform("size", osg::Vec2(float(width),float(height)));
	this->getOrCreateStateSet()->addUniform(size.get());

	osg::Matrixf mat=vw->_iMVPW;
	osg::ref_ptr<osg::Uniform> iMVPW = new osg::Uniform("iMVPW", mat/*vw->_iMVPW*/);
	this->getOrCreateStateSet()->addUniform(iMVPW.get());
	

	osg::ref_ptr<osg::Uniform> uMVP = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "uMVP");
	uMVP->setUpdateCallback(new updataMVP(vw));
	this->getOrCreateStateSet()->addUniform(uMVP.get());

	osg::ref_ptr<osg::Uniform> flag = new osg::Uniform("flag", tag);
	this->getOrCreateStateSet()->addUniform(flag.get());

}

osg::Geometry* WarpingGeode::createGeometry(int width, int height)
{
	osg::ref_ptr<osg::Geometry>  geom = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

	int i, j;
	for(i = 0 ; i < width; ++i)
		for(j = 0 ; j < height; ++j)
			vertices->push_back(osg::Vec3(i + 0.5, j + 0.5, 0.0f));
			//vertices->push_back(osg::Vec3(i , j , 0.0f));
	geom->setVertexArray(vertices.get());
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, width*height));
	return geom.release();
}

osg::Program* WarpingGeode::createProgram(const std::string vert, const std::string frag)
{
	osg::ref_ptr<osg::Program> program = new osg::Program;
	osg::ref_ptr<osg::Shader> vertShader = osgDB::readShaderFile(vert);
	osg::ref_ptr<osg::Shader> fragShader = osgDB::readShaderFile(frag);
	program->addShader(vertShader.get());
	program->addShader(fragShader.get());
	return program.release();
}

void updataMVP::operator() (osg::Uniform* uniform, osg::NodeVisitor* nv)
{
	uniform->set(view->_uMVP);
}


