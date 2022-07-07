
#ifndef D_SMARTDISPLAY3_H

#ifdef __cplusplus
extern "C" {
#endif    /* __cplusplus */
	
	/* Prototipos de funciones */
	
#ifdef __cplusplus
}
#endif    /* __cplusplus */

#include "CImg.h"


#include <stdio.h>
#include <iostream>
#include <string>
#include <deque>

#define D_SMARTDISPLAY3_H

const unsigned char black[] = { 0,0,0 }, green[] = { 0,255,0 }, white[] = {255,255,255};

template <typename T>
class smartdisplay3{
private:
	
	int _pausa;//in ms
	int _zoom;
	int _bufsize;
	int _paso;
	
	bool _record;
	bool _help;
	
	std::deque<cimg_library::CImg<T> > buffer;
	std::deque<std::string> cadenas;
	
	//cimg_library::CImg<float> linea;//Para pintar una polinlinea
	
	int pos;
	int tantieco;
	int save_counter;
	int rec_counter;
	
	int nf;
	
	int zoomFlag;
	int zoom0;
	int width0;
	int height0;
	
	
	std::string filename;
	
	cimg_library::CImgDisplay disp;
	cimg_library::CImg<T> img;
	cimg_library::CImg<T> zimg;
	cimg_library::CImg<T> himg;
	
	cimg_library::CImg<T> help_img;
	
	
	/**
     Si playing =1, pinta la imagen, comprueba si se pulsa una tecla y vuelve inmediatamente
     Si playing =0, pinta la imagen y se espera a que se pulse una tecla. En ese caso antepone |P| al texto en el caption
	 */
	int _playing;
	
	//private functions
const	cimg_library::CImg<T> & zoomimage(const cimg_library::CImg<T> &img)
	{
		if (0==zoomFlag)
		{
			zoomFlag=1;
			zoom0 = _zoom;
			width0 = img.width();
			height0 = img.height();
		}
		
	//	if( linea.size() ){
//			unsigned char color[] = {255,0,0};
//			img.draw_line( linea, color );
//		}

		if ( _zoom==0 )
		  return img;
		
		
		if(_zoom==1 )
		  return img;
		  //zimg=img.get_resize(width0,height0);
		else if(_zoom==2)
		  //	zimg=img.get_resize(width0,height0).resize_doubleXY();
		  zimg=img.get_resize_doubleXY();
		else if(_zoom==-2)
		  //zimg=img.get_resize(width0,height0).resize_halfXY();
		  zimg=img.get_resize_halfXY();
		else if(_zoom==4)
		  //zimg=img.get_resize(width0,height0).resize_doubleXY().resize_doubleXY();
		  zimg=img.get_resize_doubleXY().resize_doubleXY();
		else if(_zoom==-4)
		  //zimg=img.get_resize(width0,height0).resize_halfXY().resize_halfXY();
		  zimg=img.get_resize_halfXY().resize_halfXY();
		return zimg;
	}
	
	cimg_library::CImg<T> & drawHelp(const cimg_library::CImg<T> &img)
	{
		himg.assign(img);
		himg.draw_image(help_img,0.6f);
		return himg;
	}
	
	
	void decrease_speed()
	{
		if(paso()>1)
			
			_paso--;
		else
		{
			int p=pausa();
			p+=10;
			pausa(p);
		}
		std::cout<<"Setting delay to "<< pausa() <<" ms. ";
		std::cout<<"and step to "<< paso() <<" frames\n";  
		
		
		
	}
	
	
	
	void increase_speed()
	{
		if(pausa()>0)
		{
			int p=pausa()-10;
			if(p<0)
				p=0;
			pausa(p);
		}
		
		else
			_paso++;
		
		
		std::cout<<"Setting delay to "<< pausa() <<" ms. ";
		std::cout<<"and step to "<< paso() <<" frames\n";  
	}
	
	
	
	void increase_zoom()
	{
		if ( 0 == zoom())
			zoom(1);
		if (2==zoom())
			zoom(4);
		else if(1==zoom())
			zoom(2);
		else if (-2==zoom())
			zoom(1);
		else if (-4==zoom())
			zoom(-2);
		else if(4==zoom())
			std::cout<<"Maximum zoom value set. ";
		
		disp.wait(_pausa);
		
		std::cout<<"Setting zoom to "<< zoom() <<"\n";
	}
	
	void decrease_zoom()
	{
		if ( 0 == zoom())
			zoom(1);
		
		if(4==zoom())
			zoom(2);
		else if (2==zoom())
			zoom(1);
		else if(1==zoom())
			zoom(-2);
		else if (-2==zoom())
			zoom(-4);
		else if (-4==zoom())
			std::cout<<"Minimum zoom value set. ";
		
		disp.wait(_pausa);
		
		std::cout<<"Setting zoom to "<< zoom() <<"\n";
	}
	
	void check_keys	(const cimg_library::CImg<T> &img, const std::string & str)
	{	
		
		if(disp.is_closed()|| disp.is_keyQ())
			if(nf)
				exit(0);
		
		
		if(!_playing)
		{
		esperar:
			
			display_interno (img,str);
			
			disp.wait(tantieco);
			tantieco=80;
			disp.flush();
			disp.wait();
			if(disp.is_keyENTER () || disp.is_keySPACE())
			{
				_playing=1;
				_help=0;
				disp.wait(200);
				disp.flush();
				return;
			}
			//Teclas que cambian parametros pero no imagenes
			if(disp.is_closed()|| disp.is_keyQ())
			{
				exit(0);
			}
			if(disp.is_keyA())
			{
				increase_speed();
				goto esperar; 
			}
			if(disp.is_keyD ())
			{
				decrease_speed();
				goto esperar;
			}
			if(disp.is_keyV ())
			{
				buffer[pos].display(str.c_str());
				goto esperar;
			}
			if(disp.is_keyW())
			{
				increase_zoom();
				display_interno(img,str);
				
				goto esperar; 
			}
			if(disp.is_keyX ())
			{
				decrease_zoom();
				display_interno(img,str);
				goto esperar;
			}
			
			if(disp.is_keyS ())
			{
				_zoom=1;
				display_interno(img,str);
				std::cout<<"Restoring to initial size\n";
				goto esperar;
			}
			
			
			if(disp.is_keyESC())
			{
				save(img,str);		
				goto esperar;
			}
			
			if(disp.is_keyH())
			{
				help();		
				_help = !_help;
				display_interno(img,str);
				goto esperar;
			}
			
			if(disp.is_keyARROWDOWN())
			{
				_help=0;
				int r;
				r=go_forward();
				if(r)
				{
					display_interno(img,str);
					goto esperar;
				}
				else 
					return;
				
				
			}
			if(disp.is_keyARROWUP())
			{
				_help=0;
				int r;
				r=go_backward();
				if(r)
				{
					display_interno(img,str);
					
				}
				goto esperar;					
				
			}
			if(disp.is_keyR())
			{
				_record = !_record;
				goto esperar; 
			}
			
			if (disp.is_resized()) 
			{
				_zoom=0;
				disp.resize(false);
			}
			
			
			
			//Cualquier otra tecla hace esperar
			goto esperar;
		}
		else
		{
			
			int r=1;
			
			while(r)
			{
				
				
				r=go_forward();
				if(r)
				{
					display_interno(img,str);
					
				}
				
				
				
				disp.wait(_pausa);
				
				
				if(disp.is_keyENTER () || disp.is_keySPACE())
				{
					_playing=0;
					tantieco=500;
					display_interno(img,str);
					goto esperar;
				}
				
				if(disp.is_keyW())
				{
					increase_zoom();
					disp.wait(_pausa);
				}
				
				if(disp.is_keyX ())
				{
					decrease_zoom();
					disp.wait(_pausa);
				}
				
				if(disp.is_keyS ())
				{
					_zoom=1;
					display_interno(img,str);
					std::cout<<"Restoring to initial size\n";
					disp.wait(_pausa);
					
				}
				
				if(disp.is_keyR())
				{
					_record = !_record;
					disp.wait(_pausa);
					
				}
				
				
			}
		}
		
		
	}
	
	
	
	
	
	int go_backward()
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
	
	int go_forward()
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
	
	
	void display_interno(const cimg_library::CImg<T> &img, const std::string & str)
	{	
		const cimg_library::CImg<T> *i;
		std::string currtext="";
		
		

		if(0==bufsize())
		{
			
			if(!_playing)
				currtext=+"|P| "+str;
			else
				currtext+=str;
			
			
			i=&(zoomimage(img));
			
		}
		
		else {
			
			if(_record)
				currtext="REC ON  ";
			
			if(!_playing)
				currtext+="|P| "+cadenas[pos];
			else
				currtext+=cadenas[pos];
			
			
			i=&(zoomimage(buffer[pos]));
			
			
			
		}
		
		//		if(!i->is_sameXY(disp))
		//			disp.assign(i->width(),i->height());
		
		if ( 0 == _zoom )
		{
			(*i).get_resize(disp.width(),disp.height());
			disp.resize(false);
		}
		else
			
			disp.resize(*i);	
		
		if(_help)
			i = &(drawHelp (*i));
		
		
		disp.display(*i).set_title(currtext.c_str());		
		
		if (disp.is_resized()) 
		{
			_zoom=0;
			disp.resize(false);
		}
		if(_record)
		{
			char a[30];
			//snprintf(a,28,filename.c_str(),rec_counter);
			sprintf(a,filename.c_str(),rec_counter);
			cimg_library::CImg<unsigned char> recimg(disp);
            recimg.save(a);
			rec_counter++;
		}	
		
	}
	
	void to_buffer(const cimg_library::CImg<T> &img, const std::string &s)
	{
		if(0==bufsize())
			return;
		
		buffer.push_back(img);
		
		cadenas.push_back(s);
		
		if(buffer.size()>unsigned(bufsize()))
		{
			buffer.pop_front();
			cadenas.pop_front();
		}
		
		pos=buffer.size()-1;
		
	}
	
	
	void save(const cimg_library::CImg<T> &img, const std::string &str)
	{
		char a[30];
		//snprintf(a,28,"smrtdsp3_%02d.jpg",save_counter);
		sprintf(a,"smrtdsp3_%02d.jpg",save_counter);
		cimg_library::CImg<unsigned char> tmpdisp(disp);
		tmpdisp.save(a);
		std::cout << a <<" saved.\n";
 		//		std::string s;
//		if(0==bufsize())
//		{
//			s=str;
//			s+=a;
//
//			img.save(s.c_str());
//		}
//		else
//		{
//			s=cadenas[pos];
//			s+=a;
//
//			cimg_library::CImg<unsigned char> aa=buffer[pos];
//			aa.save(s.c_str());
//		}
		save_counter++;
		
	}	
	
public:
	void normalization(int n)
	{
		disp._normalization = n;
	}

	smartdisplay3()
	{
		_zoom=1;
		_paso=1;
		tantieco=500;
		_playing=1;
		record(false);
		_help = false;
		
		pausa(30);
		nf=0;
		bufsize(40);
		save_counter=0;
		rec_counter=0;
		filename="aa_%04d.jpg";  // Formato por defecto 
		
		pos=0;
		
		zoomFlag=0;
		zoom0 = -1;
		width0 = -1;
		height0 = -1;
		
		help_img.assign(220,220,1,3,0).
		draw_text(5,5,
				  "------------------------------------------\n"
				  "SmartDisplay3\n"
				  "------------------------------------------\n"
				  "Key 'H' : Show/hide this help\n"
				  "ENTER or SPACE: Pause\n"
				  "Key 'Q' : Close window and quit program\n"
				  "Key 'A' : Increase speed\n"
				  "Key 'D' : Decrease speed\n"
				  "Key 'W' : Zoom in\n"
				  "Key 'X' : Zoom out\n"
				  "Key 'S' : Restore zoom\n"
				  "Key 'V' : Open current image in a separate display\n"
				  "Key 'R' : Start/stop recording frames\n"
				  "Key ESC : Save current image to file\n"
				  "UP ARROW: Go to next image\n"
				  "DOWN ARROW: Go to previous image\n",
				  white);
		
		
	}
	
	smartdisplay3(const smartdisplay3 & e)
	{
		_zoom=e._zoom;
		_paso=e._paso;
		tantieco=e.tantieco;
		_playing=e._playing;
		_record=e._record;
		_help=e._help;
		
		_pausa=e._pausa;
		nf=e.nf;
		_bufsize=e._bufsize;
		save_counter=e.save_counter;
		rec_counter=e.rec_counter;
		filename=e.filename;  // Formato por defecto 
		pos=e.pos;
		
		zoomFlag = e.zoomFlag;
		zoom0 = e.zoom0;
		width0 = e.width0;
		height0 = e.height0;
		
		buffer(e.buffer);
		cadenas=e.cadenas;
		
		linea(e.linea);//Para pintar una polinlinea
		
		filename=e.filename;
		
		//disp(e.disp);
		img(e.img);
		zimg(e.img);
    himg(e.himg);
		
		help_img(e.help_img);
		
		
	}
	
	
	smartdisplay3 & operator=(const smartdisplay3 & e)
	{
		_zoom=e._zoom;
		_paso=e._paso;
		tantieco=e.tantieco;
		_playing=e._playing;
		_record=e._record;
		_help=e._help;
		
		_pausa=e._pausa;
		nf=e.nf;
		_bufsize=e._bufsize;
		save_counter=e.save_counter;
		rec_counter=e.rec_counter;
		filename=e.filename;  // Formato por defecto 
		pos=e.pos;
		
		zoomFlag = e.zoomFlag;
		zoom0 = e.zoom0;
		width0 = e.width0;
		height0 = e.height0;
		
		
		buffer=e.buffer;
		cadenas=e.cadenas;
		
//		linea=e.linea;//Para pintar una polinlinea
		
		filename=e.filename;
		
		//disp=e.disp;
		img=e.img;
		zimg=e.img;
    himg=e.himg;
		
		help_img=e.help_img;
		
		return *this;
		
	}
	
	~smartdisplay3()
	{
		disp.assign();
	}
	
	
	void display(const cimg_library::CImg<T> &img, const std::string & str)
	{
		
		
		if(nf%paso()==0U)
		{
			to_buffer(img,str);
			
			display_interno(img,str);
			check_keys(img,str);
		}
		
		nf++;
	}
	
	void help()
	{
		std::cout<<"\n\n\n------ Smartdisplay3 help -------\n\n\t- Keyboard Functions -\n";
		std::cout<<"\n\t\tH: Show this help\n\n\t\tENTER or SPACE: Pause\n\n\t\tQ: Close window and exit program\n\n\t\t";
		std::cout<<"A: Increase speed\n\t\tD: Decrease speed\n\n\t\t";
		std::cout<<"W: Zoom in\n\t\tX: Zoom out\n\n\t\tV: Open current image in a separate display\n\n\t\tESC: Save current image to file\n\t\tR: Start/stop recording frames\n\n";
		std::cout<<"\t\tUP ARROW: Go to next image\n\t\tDOWN ARROW: Go to previous image\n\n";
		std::cout<<"--------------------------------------\n\n";
	}
	
	void record (std::string str){ _record = true; filename=str;};
	
	void record (bool b) { _record = b; }; 
	bool record(){return _record;};
	
	void zoom(int z){_zoom=z;};
	int zoom(){return _zoom;};
	
	
	void pausa(int p){if(p>=0) _pausa=p;};
	int pausa(){return _pausa;};
	
	void paso(int p){ _paso=p;};
	int paso(){return _paso;};
	
	void playing(int p){ _playing=p;};
	int playing(){return _playing;};
	
//	void lines(cimg_library::CImg<float> & l){linea=l.get_transpose();linea.print();};
	
	void bufsize(int a){ _bufsize=a;buffer.clear();};
	int bufsize(){return _bufsize;};
	void move(int pos_x, int pos_y){disp.move(pos_x,pos_y);};
	
	cimg_library::CImg<unsigned char> grabDisplay(){ cimg_library::CImg<unsigned char> tmp(disp); return tmp; }

	
};
#endif /* D_SMARTLAY_H */
