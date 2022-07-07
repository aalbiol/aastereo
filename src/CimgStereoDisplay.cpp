/*
 * CimgStereoDisplay.cpp
 *
 *  Created on: Sep 20, 2012
 *      Author: jmmossi
 */

#include <CImg.h>
#include "CimgStereoDisplay.h"

using namespace cimg_library;
using namespace std;

#ifdef HAVE_PCL

void CimgStereoDisplay::MyPointPicking_callback (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
	if (event.getPointIndex () == -1)
	{
		return;
	}
	event.getPoint(posSphere.x,posSphere.y,posSphere.z);
	if( Q_inv.size() != 16)
	{
		CImg<float> cimg_Q( CimgSt_p->Q_Matrix() );   //CimgStereo::Q is a cv::Mat. Convert here to CImg
		Q_inv=cimg_Q.get_invert();

		if(CimgSt_p->exist_rotationMatrix())
			rotMatrix_inv=( CimgSt_p->rotationMatrix() ).get_invert();
	}

	viewer->removeShape("sphere1");
	viewer->addSphere(posSphere, Sphere_radio, 255.0, 255.0, 0.0, "sphere1");

	CImg<float>ppw(1,4,1,1);
	float zz;
	if(CimgSt_p->exist_rotationMatrix())
	{
		zz=posSphere.z - CimgSt_p->camHeight_mm()/1000.0;
		CImg<float>pp(1,3,1,1, posSphere.x,posSphere.y,zz);
		pp=rotMatrix_inv*pp;

		// ??? atencion al cambio de signo en la componente Y. Ver la funcion int CimgStereo::reproject3D() en CimgStereo.cpp
		ppw(0)=pp(0);  ppw(1) = -pp(1); ppw(2)=pp(2); ppw(3)=1;
		ppw=Q_inv*ppw;
		ppw(0) = ppw(0)/ppw(3);
		ppw(1) = ppw(1)/ppw(3);
		ppw(2) = (ppw(2)/ppw(3))/1000;
	}
	else
	{

		// ??? atencion al cambio de signo en la componente Y. Ver la funcion int CimgStereo::reproject3D() en CimgStereo.cpp
		ppw(0)=posSphere.x;  ppw(1) = -posSphere.y; ppw(2)=posSphere.z; ppw(3)=1;
		ppw=Q_inv*ppw;
		ppw(0) = ppw(0)/ppw(3);
		ppw(1) = ppw(1)/ppw(3);
		ppw(2) = (ppw(2)/ppw(3))/1000;
	}

	int xx,yy;
	xx= (int) ppw(0);
	yy= (int) ppw(1);
	if(xx >= 0 && xx < (CimgSt_p->xyz().width()) &&	yy >=0 && yy < (CimgSt_p->xyz().height() ))
	{
		refresh_mosaico(xx,yy);
	}
}


#endif

CimgStereoDisplay::CimgStereoDisplay(CimgStereo *CimgSt) {
	_playing=1;
	_initialised=0;
	pos=0;
	bufsize(40);
	_exist_Center = 0;

	CimgSt_p = CimgSt;

#ifdef HAVE_PCL
	viewerPCL2 = 0;
#endif

}

void CimgStereoDisplay::to_buffer(StereoMosaic &StMosaic)
{
	if(0==bufsize())
		return;

	buffer.push_back(StMosaic);

	if(buffer.size()>unsigned(bufsize()))
	{
		buffer.pop_front();
	}
	pos=buffer.size()-1;
}
int CimgStereoDisplay::go_forward()
{
	if(!bufsize())
		return 0;
	pos++;

	if(unsigned(pos)>=buffer.size())
	{
		pos=buffer.size()-1;
		return 0;
	}
	return 1;
}
int CimgStereoDisplay::go_backward()
{
	if(!bufsize())
		return 0;

	pos--;

	if(pos<0)
	{
		pos=0;
		return 0;
	}
	return 1;

}


#ifdef HAVE_PCL
void CimgStereoDisplay::init_viewerPCL2()
{
	// PCL display
	viewerPCL2=1;

	viewer= boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));

	viewer->setBackgroundColor (0, 0, 0);
	viewer->setSize(640,480);

	viewer->initCameraParameters ();
	// Set position
	viewer->camera_.pos[0] = 0;
	viewer->camera_.pos[1] = 0;
	viewer->camera_.pos[2] = 10;//-0.5;

	// Set "rotation"
	viewer->camera_.view[0] = 0;
	viewer->camera_.view[1] = 3.14;//0;
	viewer->camera_.view[2] = 0;

	viewer->addCoordinateSystem (1.0);
	viewer->updateCamera();

	drawSpherePCL = 1;
	Sphere_radio = 0.05;
	if(CimgSt_p->exist_rotationMatrix())
	{
		pcl::PointXYZ basic_point;
		pcl::PointCloud<pcl::PointXYZ> basic_cloud;

		basic_point.x=2.0; basic_point.y=0.0;basic_point.z=0.0;
		basic_cloud.points.push_back(basic_point);

		basic_point.x=-2.0; basic_point.y=0.0;basic_point.z=0.0;
		basic_cloud.points.push_back(basic_point);

		basic_point.x=-2.0; basic_point.y=5.0;basic_point.z=0.0;
		basic_cloud.points.push_back(basic_point);

		basic_point.x=2.0; basic_point.y=5.0;basic_point.z=0.0;

		basic_cloud.points.push_back(basic_point);
		basic_cloud.width = 4;
		basic_cloud.height = 1;
		ground_polygon.setContour(basic_cloud);
		/*
	viewer->addPolygon<pcl::PointXYZ> (ground_polygon, 0.37, 0.3, 0.41, "ground polygon");
	viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "ground polygon");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
			pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "ground polygon");
		 */

		pcl::PointCloud<pcl::PointXYZ>::Ptr floor_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		float xmin, xmax, ymin, ymax;
		xmin =-2.0; xmax = +2.0; ymin = 0.0; ymax =5.0;

		float distance=0.1;
		basic_point.z=0.0;
		for (float jj= ymin; jj<=ymax; jj+=distance)
		{
			basic_point.y=jj;
			for (float ii=xmin; ii<=xmax; ii+=distance)
			{
				basic_point.x=ii;
				floor_cloud->points.push_back(basic_point);
			}
		}
		floor_cloud->width = (int)floor_cloud->points.size();
		floor_cloud->height = 1;

		viewer->addPointCloud<pcl::PointXYZ> (floor_cloud, "ground polygon");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ground polygon");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.37, 0.3, 0.41,"ground polygon");

		// polygon on Y=0
		basic_point.x=-1.3; basic_point.y=0.0;basic_point.z=0.0;
		basic_cloud.points[0]=basic_point;
		basic_point.x=-1.3; basic_point.y=0.0;basic_point.z=2.0;
		basic_cloud.points[1]=basic_point;
		basic_point.x=0.8; basic_point.y=0.0;basic_point.z=2.0;
		basic_cloud.points[2]=basic_point;
		basic_point.x=0.8; basic_point.y=0.0;basic_point.z=0.0;
		basic_cloud.points[3]=basic_point;

		basic_cloud.width = 4;
		basic_cloud.height = 1;
		Ycero_polygon.setContour(basic_cloud);
		/*	viewer->addPolygon<pcl::PointXYZ> (Ycero_polygon, 255.0, 255.0, 255.0 , "Ycero polygon");
	viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "Ycero polygon");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
			pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "Ycero polygon");
		 */

		// Cristal_polygon
		basic_point.x=-1.3; basic_point.y=0.0;basic_point.z=0.0;
		basic_cloud.points[0]=basic_point;
		basic_point.x=-1.3; basic_point.y=0.0;basic_point.z=2.0;
		basic_cloud.points[1]=basic_point;
		basic_point.x=-1.3; basic_point.y=5.0;basic_point.z=2.0;
		basic_cloud.points[2]=basic_point;
		basic_point.x=-1.3; basic_point.y=5.0;basic_point.z=0.0;
		basic_cloud.points[3]=basic_point;

		basic_cloud.width = 4;
		basic_cloud.height = 1;
		Cristal_polygon.setContour(basic_cloud);
		/*	viewer->addPolygon<pcl::PointXYZ> (Cristal_polygon, 1.0,0.0,0.0, "Cristal polygon");
	viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "Cristal polygon");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
			pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "Cristal polygon");
		 */
	}
	viewer->registerPointPickingCallback (&CimgStereoDisplay::MyPointPicking_callback, *this,0);

}
#endif


void CimgStereoDisplay::display(std::string &texto_A)
{
	StereoMosaic StMosaic;
	CimgSt_p->getLeftRectifiedGray(StMosaic.Left);
	CimgSt_p->getRightRectifiedGray(StMosaic.Right);
	CimgSt_p->getDisparity(StMosaic.disparity);
	CimgSt_p->getDisparity_type(StMosaic.disparity_type);
	StMosaic.DepthColor = StMosaic.Left.get_resize(-100,-100,-100,3);

	if(CimgSt_p->exist_rotationMatrix())
		CimgSt_p->Y_AsColor(StMosaic.DepthColor);
	else
		CimgSt_p->depthAsColor(StMosaic.DepthColor);
	CimgSt_p->getXYZ(StMosaic.xyz);
	StMosaic.texto = texto_A;

	rectLeft = &(StMosaic.Left);
	rectRight = &(StMosaic.Right);
	depthColor = &(StMosaic.DepthColor);
	disparity = &(StMosaic.disparity);
	disparity_type = &(StMosaic.disparity_type);
	xyz = &(StMosaic.xyz);
	texto = &texto_A;

	if(CimgSt_p->getNumCameras() == 3){
		_exist_Center = 1;
		CimgSt_p->getCenterRectifiedGray(StMosaic.Center);
		rectCenter = &(StMosaic.Center);
	}
	else
		_exist_Center = 0;


	to_buffer(StMosaic);

	if(!_initialised)
	{
		font_size = FONT_SIZE_DEFAULT;
		font_interline = FONT_SIZE_DEFAULT + FONT_SIZE_SEPARATION;

		if(((*rectLeft).height()-6) < 6*font_interline)
		{
			font_interline = (((*rectLeft).height()-6)) / 6;
			font_size = font_interline - FONT_SIZE_SEPARATION;
		}


		display_mosaico (disp.mouse_x(), disp.mouse_y());
		_initialised=1;

	}

	if(disp.is_closed()|| disp.is_keyQ())
		exit(0);

	if(!_playing)
	{
	esperar:

		display_mosaico (disp.mouse_x(), disp.mouse_y());

//TODO mirar esto del wait y del flush

		//disp.wait(80);
		//disp.flush();
		disp.wait(150);
		if(disp.is_keyENTER () || disp.is_keySPACE())
		{
			_playing=1;
			disp.wait(200);
			disp.flush();
			return;
		}
		if(disp.is_closed()|| disp.is_keyQ())
		{
			exit(0);
		}
		if(disp.is_keyARROWDOWN())
		{
			int r;
			r=go_forward();
			if(r)
			{
				//display_mosaico (disp.mouse_x(), disp.mouse_y());
				goto esperar;
			}
			else
				return;
		}
		if(disp.is_keyARROWUP())
		{
			int r;
			r=go_backward();
/*
			if(r)
			{
				display_mosaico (disp.mouse_x(), disp.mouse_y());
	 		}
	 		*/
			goto esperar;

		}

		//Cualquier otra tecla hace esperar
		goto esperar;
	}
	else
	{
		display_mosaico (disp.mouse_x(), disp.mouse_y());
//TODO mirar esto del wait y del flush
		disp.wait(200);
		if(disp.is_keyENTER () || disp.is_keySPACE())
		{
			_playing=0;
			goto esperar;
		}
	}
} //void CimgStereoDisplay::display




void CimgStereoDisplay::display_mosaico(int xx, int yy)
{

	const unsigned char black[] = { 0,0,0 }, white[] = { 255,255,255 }, red[]={255,0,0}, yellow[]={255,255,0},green[]={0,255,0};
	char ss[100];
	int anch,alt, anch_display, alt_display, xpos_text;
	//int diezmado = 1;   // scale font size according to image zoom


	if(bufsize()>0)
	{
		rectLeft = &(buffer[pos].Left);//&rectLeft_A;
		rectRight = &(buffer[pos].Right);//&rectRight_A;
		depthColor = &(buffer[pos].DepthColor);
		disparity = &(buffer[pos].disparity);
		disparity_type = &(buffer[pos].disparity_type);
		xyz = &(buffer[pos].xyz);
		texto = &(buffer[pos].texto);
		if(_exist_Center){
			rectCenter = &(buffer[pos].Center);
		}
	}

	anch = (*rectLeft).width();
	alt = (*rectLeft).height();

	CImg<unsigned char>upperRow;
	CImg<unsigned char>Black;
	CImg<unsigned char>img_disp_type;
	CImg<unsigned char>lowerRow;

	if(_exist_Center)
	{
		upperRow = (*rectLeft).get_append(*rectCenter, 'x').append(*rectRight, 'x').resize(-100,-100,-100,3);
		img_disp_type.assign((*rectLeft).width(),(*rectLeft).height(),1,3).fill(0);
		cimg_forXY(img_disp_type,x,y)
		{
			switch ((*disparity_type)(x,y))
			{
			case CAM3_LC_DECIMATE_2:
				img_disp_type(x,y,0) = 255; //red
			break;
			case CAM3_LC_DECIMATE_1:
				img_disp_type(x,y,0) = 255; //yellow
				img_disp_type(x,y,1) = 255;
			break;
			case CAM3_LR_DECIMATE_1:
				img_disp_type(x,y,1) = 255;  //green
			break;
			}
		}
		img_disp_type = img_disp_type*0.4+ 0.6*((*rectLeft).get_resize(-100,-100,-100,3));

		Black.assign((*rectLeft).width(),(*rectLeft).height(),1,3).fill(0);
		lowerRow = (*depthColor).get_append(img_disp_type, 'x').append(Black,'x');
		anch_display = 3*anch;
		alt_display = 3*alt;
		xpos_text = anch*2+3;
	}
	else
	{
		upperRow = (*rectLeft).get_append(*rectRight, 'x').resize(-100,-100,-100,3);
		img_disp_type.assign((*rectLeft).width(),(*rectLeft).height(),1,3).fill(0);
		cimg_forXY(img_disp_type,x,y)
		{
			switch ((*disparity_type)(x,y))
			{
			case CAM2_DECIMATE_2:
				img_disp_type(x,y,0) = 255;  //red
			break;
			case CAM2_DECIMATE_1:
				img_disp_type(x,y,0) = 255;  //yellow
				img_disp_type(x,y,1) = 255;
			break;
			}
		}
		img_disp_type = img_disp_type*0.4+ 0.6*((*rectLeft).get_resize(-100,-100,-100,3));
		lowerRow = (*depthColor).get_append(img_disp_type, 'x');
		anch_display = 2*anch;
		alt_display = 2*alt;
		xpos_text = anch+3;
	}

	CImg<unsigned char>mosaic = upperRow.get_append(lowerRow, 'y');

	for (int ff=20; ff<alt; ff+=20)
	{
		mosaic.draw_line(0,ff, anch_display-1,ff, black, 0.7);
	}

	if(xx >= 0 && xx < anch &&  yy >= alt)
	{
		int xpos = xx;
		int ypos = yy -alt;
		float XX,YY,ZZ,RR;

		sprintf(ss,"(%d,%d)",xpos,ypos);
		mosaic.draw_text(xpos_text, alt+3,ss,white,black,0.8f,font_size);

		ZZ=(*xyz)(xpos,ypos,2);
		if (!isnan(ZZ))
		{
			sprintf(ss,"D:%1.2f", (*disparity)(xpos,ypos,0,0));
			mosaic.draw_text(xpos_text,alt+font_interline,ss,white,black,0.8f,font_size);
			XX=(*xyz)(xpos,ypos,0);
			YY=(*xyz)(xpos,ypos,1);
			RR=(float)sqrt(XX*XX+YY*YY+ZZ*ZZ);
			sprintf(ss,"R:%1.2f", RR);
			mosaic.draw_text(xpos_text,alt+2*font_interline,ss,white,black,0.8f,font_size);
			sprintf(ss,"X:%1.2f", XX);
			mosaic.draw_text(xpos_text,alt+3*font_interline,ss,white,black,0.8f,font_size);
			sprintf(ss,"Y:%1.2f", YY);
			mosaic.draw_text(xpos_text,alt+4*font_interline,ss,white,black,0.8f,font_size);
			sprintf(ss,"Z:%1.2f", ZZ);
			mosaic.draw_text(xpos_text,alt+5*font_interline,ss,white,black,0.8f,font_size);

			int xxt = xpos-(int)((*disparity)(xpos,ypos));
			if(_exist_Center)  // 3 cameras
			{
				switch ((*disparity_type)(xpos,ypos))
				{
				case CAM3_LC_DECIMATE_2:
				{
					mosaic.draw_circle(xpos,ypos,3,black,1,1).draw_circle(xpos,ypos,4,white,1,1).draw_circle(xpos,ypos,5,red,1,1);
					mosaic.draw_circle(xpos+anch,ypos+alt,3,black,1,1).draw_circle(xpos+anch,ypos+alt,4,white,1,1);
					mosaic.draw_circle(xxt+anch,ypos,3,black,1,1).draw_circle(xxt+anch,ypos,4,white,1,1).draw_circle(xxt+anch,ypos,5,red,1,1);

				}
				break;
				case CAM3_LC_DECIMATE_1:
				{
					mosaic.draw_circle(xpos,ypos,3,black,1,1).draw_circle(xpos,ypos,4,white,1,1).draw_circle(xpos,ypos,5,yellow,1,1);
					mosaic.draw_circle(xpos+anch,ypos+alt,3,black,1,1).draw_circle(xpos+anch,ypos+alt,4,white,1,1);
					mosaic.draw_circle(xxt+anch,ypos,3,black,1,1).draw_circle(xxt+anch,ypos,4,white,1,1).draw_circle(xxt+anch,ypos,5,yellow,1,1);
				}
				break;
				case CAM3_LR_DECIMATE_1:
				{
					mosaic.draw_circle(xpos,ypos,3,black,1,1).draw_circle(xpos,ypos,4,white,1,1).draw_circle(xpos,ypos,5,green,1,1);
					mosaic.draw_circle(xpos+anch,ypos+alt,3,black,1,1).draw_circle(xpos+anch,ypos+alt,4,white,1,1);
					mosaic.draw_circle(xxt+2*anch,ypos,3,black,1,1).draw_circle(xxt+2*anch,ypos,4,white,1,1).draw_circle(xxt+2*anch,ypos,5,green,1,1);
				}
				break;
				}
			}
			else  // 2 cameras
			{
				switch ((*disparity_type)(xpos,ypos))
				{
				case CAM2_DECIMATE_2:
				{
					mosaic.draw_circle(xpos,ypos,3,black,1,1).draw_circle(xpos,ypos,4,white,1,1).draw_circle(xpos,ypos,5,red,1,1);
					mosaic.draw_circle(xpos+anch,ypos+alt,3,black,1,1).draw_circle(xpos+anch,ypos+alt,4,white,1,1);
					mosaic.draw_circle(xxt+anch,ypos,3,black,1,1).draw_circle(xxt+anch,ypos,4,white,1,1).draw_circle(xxt+anch,ypos,5,red,1,1);

				}
				break;
				case CAM2_DECIMATE_1:
				{
					mosaic.draw_circle(xpos,ypos,3,black,1,1).draw_circle(xpos,ypos,4,white,1,1).draw_circle(xpos,ypos,5,yellow,1,1);
					mosaic.draw_circle(xpos+anch,ypos+alt,3,black,1,1).draw_circle(xpos+anch,ypos+alt,4,white,1,1);
					mosaic.draw_circle(xxt+anch,ypos,3,black,1,1).draw_circle(xxt+anch,ypos,4,white,1,1).draw_circle(xxt+anch,ypos,5,yellow,1,1);
				}
				break;
				}
			}
		}
		else
		{
			mosaic.draw_circle(xpos,ypos,3,black,1,1).draw_circle(xpos,ypos,4,white,1,1);
		}
	}
	std::string txtmp=*texto;
	if(_playing)
		disp.display(mosaic).set_title(txtmp.c_str());
	else
	{
		txtmp="|P|" + *texto;
		disp.display(mosaic).set_title(txtmp.c_str());
	}

#ifdef HAVE_PCL
	if(viewerPCL2)
	{
		if(xx >= 0 && xx < anch &&  yy >= alt && yy< 2*alt)  // mouse inside the lower-left rectangle
		{
			int xpos = xx;
			int ypos = yy -alt;
			displayPCL(xpos, ypos);
		}
		else
		{
			int xpos = -1;
			int ypos = -1;
			displayPCL(xpos, ypos);
		}
	}
#endif


}

#ifdef HAVE_PCL

void CimgStereoDisplay::displayPCL(int xpos, int ypos)
{
	CImg<unsigned char>tmp_color = (*rectLeft).get_resize(-100,-100,-100,3);
	pointcloud = upvsoft::pointcloud::cimg2pcl(tmp_color, (*xyz));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud);

	viewer->removePointCloud("sample cloud");
	viewer->addPointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

	if(drawSpherePCL && xpos != -1 && -1 != ypos)
	{
		posSphere.x = (*xyz)(xpos,ypos,0);
		posSphere.y	= (*xyz)(xpos,ypos,1);
		posSphere.z = (*xyz)(xpos,ypos,2);
		viewer->removeShape("sphere1");
		viewer->addSphere(posSphere, Sphere_radio, 255.0, 0.0, 0.0, "sphere1");
	}

	/*
	viewer->removeShape("ground polygon");
	viewer->addPolygon<pcl::PointXYZ> (ground_polygon, 0.6, 0.5, 0.7, "ground polygon");
	//viewer->addPolygon<pcl::PointXYZ> (ground_polygon, 95.0,80.0,105.0, "ground polygon");
	viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "ground polygon");
	viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "ground polygon");
	//viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 95.0, 80.0, 105.0, "ground polygon");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
		                                        //pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "ground polygon");
			pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "ground polygon");


	viewer->removeShape("Ycero polygon");
	viewer->addPolygon<pcl::PointXYZ> (Ycero_polygon, 1.0,1.0,1.0, "Ycero polygon");
	viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "Ycero polygon");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
			pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "Ycero polygon");

	viewer->removeShape("Cristal polygon");
		viewer->addPolygon<pcl::PointXYZ> (Cristal_polygon, 1.0,0.6,0.0, "Cristal polygon");
		viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "Cristal polygon");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
				pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "Cristal polygon");

*/

	if(!viewer->wasStopped ()) {
		viewer->spinOnce (100);
		//	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
}

void CimgStereoDisplay::refresh_mosaico(int xx, int yy)
{
	const unsigned char black[] = { 0,0,0 }, white[] = { 255,255,255 }, red[]={255,0,0}, yellow[]={255,255,0},green[]={0,255,0},blue[]={0,0,255};
	char ss[100];
	int anch,alt, anch_display, alt_display, xpos_text;
	int diezmado = 1;   // scale font size according to image zoom


	if(bufsize()>0)
	{
		rectLeft = &(buffer[pos].Left);//&rectLeft_A;
		rectRight = &(buffer[pos].Right);//&rectRight_A;
		depthColor = &(buffer[pos].DepthColor);
		disparity = &(buffer[pos].disparity);
		disparity_type = &(buffer[pos].disparity_type);
		xyz = &(buffer[pos].xyz);
		texto = &(buffer[pos].texto);
		if(_exist_Center){
			rectCenter = &(buffer[pos].Center);
		}
	}

	anch = (*rectLeft).width();
	alt = (*rectLeft).height();


	yy = yy +alt; // display_mosaico receives yy from alt to 2*alt-1 but this function receives yy from 0 to alt-1 and the next code is made for alt to 2*alt-1

	CImg<unsigned char>upperRow;
	CImg<unsigned char>Black;
	CImg<unsigned char>img_disp_type;
	CImg<unsigned char>lowerRow;

	if(_exist_Center)
	{
		upperRow = (*rectLeft).get_append(*rectCenter, 'x').append(*rectRight, 'x').resize(-100,-100,-100,3);
		img_disp_type.assign((*rectLeft).width(),(*rectLeft).height(),1,3).fill(0);
		cimg_forXY(img_disp_type,x,y)
		{
			switch ((*disparity_type)(x,y))
			{
			case CAM3_LC_DECIMATE_2:
				img_disp_type(x,y,0) = 255; //red
			break;
			case CAM3_LC_DECIMATE_1:
				img_disp_type(x,y,0) = 255; //yellow
				img_disp_type(x,y,1) = 255;
			break;
			case CAM3_LR_DECIMATE_1:
				img_disp_type(x,y,1) = 255;  //green
			break;
			}
		}
		img_disp_type = img_disp_type*0.4+ 0.6*((*rectLeft).get_resize(-100,-100,-100,3));

		Black.assign((*rectLeft).width(),(*rectLeft).height(),1,3).fill(0);
		lowerRow = (*depthColor).get_append(img_disp_type, 'x').append(Black,'x');
		anch_display = 3*anch;
		alt_display = 3*alt;
		xpos_text = anch*2+3;
	}
	else
	{
		upperRow = (*rectLeft).get_append(*rectRight, 'x').resize(-100,-100,-100,3);
		img_disp_type.assign((*rectLeft).width(),(*rectLeft).height(),1,3).fill(0);
		cimg_forXY(img_disp_type,x,y)
		{
			switch ((*disparity_type)(x,y))
			{
			case CAM2_DECIMATE_2:
				img_disp_type(x,y,0) = 255;  //red
			break;
			case CAM2_DECIMATE_1:
				img_disp_type(x,y,0) = 255;  //yellow
				img_disp_type(x,y,1) = 255;
			break;
			}
		}
		img_disp_type = img_disp_type*0.4+ 0.6*((*rectLeft).get_resize(-100,-100,-100,3));
		lowerRow = (*depthColor).get_append(img_disp_type, 'x');
		anch_display = 2*anch;
		alt_display = 2*alt;
		xpos_text = anch+3;
	}

	CImg<unsigned char>mosaic = upperRow.get_append(lowerRow, 'y');

	for (int ff=20; ff<alt; ff+=20)
	{
		mosaic.draw_line(0,ff, anch_display-1,ff, black, 0.7);
	}

	if(xx >= 0 && xx < anch &&  yy >= alt)
	{
		int xpos = xx;
		int ypos = yy -alt;
		float XX,YY,ZZ,RR;

		sprintf(ss,"(%d,%d)",xpos,ypos);
		mosaic.draw_text(xpos_text, alt+3,ss,white,black,0.8f,font_size);


		ZZ=(*xyz)(xpos,ypos,2);
		if (!isnan(ZZ))
		{
			sprintf(ss,"D:%1.2f", (*disparity)(xpos,ypos,0,0));
			mosaic.draw_text(xpos_text,alt+font_interline,ss,white,black,0.8f,font_size);
			XX=(*xyz)(xpos,ypos,0);
			YY=(*xyz)(xpos,ypos,1);
			RR=(float)sqrt(XX*XX+YY*YY+ZZ*ZZ);
			sprintf(ss,"R:%1.2f", RR);
			mosaic.draw_text(xpos_text,alt+2*font_interline,ss,white,black,0.8f,font_size);
			sprintf(ss,"X:%1.2f", XX);
			mosaic.draw_text(xpos_text,alt+3*font_interline,ss,white,black,0.8f,font_size);
			sprintf(ss,"Y:%1.2f", YY);
			mosaic.draw_text(xpos_text,alt+4*font_interline,ss,white,black,0.8f,font_size);
			sprintf(ss,"Z:%1.2f", ZZ);
			mosaic.draw_text(xpos_text,alt+5*font_interline,ss,white,black,0.8f,font_size);

			int xxt = xpos-(int)((*disparity)(xpos,ypos));
			if(_exist_Center)  // 3 cameras
			{
				switch ((*disparity_type)(xpos,ypos))
				{
				case CAM3_LC_DECIMATE_2:
				{
					mosaic.draw_circle(xpos,ypos,3,black,1,1).draw_circle(xpos,ypos,4,white,1,1).draw_circle(xpos,ypos,5,red,1,1);
					mosaic.draw_circle(xpos+anch,ypos+alt,3,black,1,1).draw_circle(xpos+anch,ypos+alt,4,white,1,1);
					mosaic.draw_circle(xxt+anch,ypos,3,black,1,1).draw_circle(xxt+anch,ypos,4,white,1,1).draw_circle(xxt+anch,ypos,5,red,1,1);

				}
				break;
				case CAM3_LC_DECIMATE_1:
				{
					mosaic.draw_circle(xpos,ypos,3,black,1,1).draw_circle(xpos,ypos,4,white,1,1).draw_circle(xpos,ypos,5,yellow,1,1);
					mosaic.draw_circle(xpos+anch,ypos+alt,3,black,1,1).draw_circle(xpos+anch,ypos+alt,4,white,1,1);
					mosaic.draw_circle(xxt+anch,ypos,3,black,1,1).draw_circle(xxt+anch,ypos,4,white,1,1).draw_circle(xxt+anch,ypos,5,yellow,1,1);
				}
				break;
				case CAM3_LR_DECIMATE_1:
				{
					mosaic.draw_circle(xpos,ypos,3,black,1,1).draw_circle(xpos,ypos,4,white,1,1).draw_circle(xpos,ypos,5,green,1,1);
					mosaic.draw_circle(xpos+anch,ypos+alt,3,black,1,1).draw_circle(xpos+anch,ypos+alt,4,white,1,1);
					mosaic.draw_circle(xxt+2*anch,ypos,3,black,1,1).draw_circle(xxt+2*anch,ypos,4,white,1,1).draw_circle(xxt+2*anch,ypos,5,green,1,1);
				}
				break;
				}
			}
			else  // 2 cameras
			{
				switch ((*disparity_type)(xpos,ypos))
				{
				case CAM2_DECIMATE_2:
				{
					mosaic.draw_circle(xpos,ypos,3,black,1,1).draw_circle(xpos,ypos,4,white,1,1).draw_circle(xpos,ypos,5,red,1,1);
					mosaic.draw_circle(xpos+anch,ypos+alt,3,black,1,1).draw_circle(xpos+anch,ypos+alt,4,white,1,1);
					mosaic.draw_circle(xxt+anch,ypos,3,black,1,1).draw_circle(xxt+anch,ypos,4,white,1,1).draw_circle(xxt+anch,ypos,5,red,1,1);

				}
				break;
				case CAM2_DECIMATE_1:
				{
					mosaic.draw_circle(xpos,ypos,3,black,1,1).draw_circle(xpos,ypos,4,white,1,1).draw_circle(xpos,ypos,5,yellow,1,1);
					mosaic.draw_circle(xpos+anch,ypos+alt,3,black,1,1).draw_circle(xpos+anch,ypos+alt,4,white,1,1);
					mosaic.draw_circle(xxt+anch,ypos,3,black,1,1).draw_circle(xxt+anch,ypos,4,white,1,1).draw_circle(xxt+anch,ypos,5,yellow,1,1);
				}
				break;
				}
			}
		}
		else
		{
			mosaic.draw_circle(xpos,ypos,3,black,1,1).draw_circle(xpos,ypos,4,white,1,1);
		}
		mosaic.draw_circle(xpos,ypos+alt,3,black,1,1).draw_circle(xpos,ypos+alt,4,white,1,1);

	}
	std::string txtmp=*texto;
	if(_playing)
		disp.display(mosaic).set_title(txtmp.c_str());
	else
	{
		txtmp="|P|" + *texto;
		disp.display(mosaic).set_title(txtmp.c_str());
	}
}

#endif
