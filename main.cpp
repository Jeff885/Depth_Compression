#include "LocalViewer.h"
#include <osgDB/ReadFile>

int main()
{
	//int width = 480;
	//int height = 640;

	int width = 450;
 	int height = 450;
	//int width = 300;
	//int height = 300;
	//
	//450
	//
	//int width = 600;
	//int height = 600;

	//width=1000;
	//height=1000;
	//int width = 250;
	//int height = 250;

	//int width=640;
	//int height=480;
	

	//int width = 768;
 	//int height = 1024;

	//int preWidth = int(width * 1.15);
	//int preHeight = int(height * 1.15);

	//int width=1024;
	//int height=768;
	int preWidth=width;
	int preHeight=height;
	LocalViewer viewer(preWidth, preHeight);
	viewer.run();
	return 0;
}