
//#define CV_NO_BACKWARD_COMPATIBILITY
//jmmossi
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>


//#include "highgui.h"
#include <stdio.h>
#include <unistd.h>

#include <sys/time.h>
#include <time.h>

#include <vicdef.h>
//#include <midemonio.h>
#include <iostream>
#include <fstream>

int timeval_subtract (timeval *x, timeval *y, timeval *result)
{
	/* Perform the carry for the later subtraction by updating y. */ 
	if (x->tv_usec < y->tv_usec) {
		int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1; 
		y->tv_usec -= 1000000 * nsec; 
		y->tv_sec += nsec;
	} if (x->tv_usec - y->tv_usec > 1000000) {
		int nsec = (x->tv_usec - y->tv_usec) / 1000000;
		y->tv_usec += 1000000 * nsec; 
		y->tv_sec -= nsec;
	}
	/* Compute the time remaining to wait. tv_usec is certainly positive. */
	result->tv_sec = x->tv_sec - y->tv_sec; 
	result->tv_usec = x->tv_usec - y->tv_usec;
	/* Return 1 if result is negative. */ 
	return x->tv_sec < y->tv_sec;
}

void ayuda() 
{
	printf("stero_capture\n"
			"    captura de dos camaras, guarda las imagenes y manda a demonio\n"
			"   -h   help\n"
			"   -d   Activa el display. Muestra las imagenes. DEFAULT: 0\n"
			"   -W   Anchura de imagenes deseada\n"
			"   -H   Altura de imagenes deseada\n"
			"   -g #:  modo de grabacion (DEFAULT: 3)\n"
			"        1: graba en disco imagenes sueltas cada vez que se pulsa SPACE,\n"
			"           con el nombre imLnum, imRnum \n"
			"        2: graba en disco dos ficheros de video llamados camL.avi y camR.avi.\n"
			"             Se puede parar y continuar la grabacion con SPACE\n"
			"           SI se activa grabacion pero el DISPLAY NO ESTA ACTIVO\n"
			"           graba continuamente desde el arranque y CTRL-C para terminar\n"
			"        3: graba imgs sueltas llamadas im_num.\n"
			"              Para cada instante una imagen formada por un mosaico\n HORIZONTAL de dos imgs\n"
			"        4: graba un video llamado cam.avi\n"
			"              Para cada instante una imagen formada por un mosaico\n HORIZONTAL de dos imgs\n"
			"        5: graba imgs sueltas llamadas im_num.\n"
			"              Para cada instante una imagen formada por un mosaico\n VERTICAL de dos imgs\n"
			"        6: graba un video llamado cam.avi\n"
			"              Para cada instante una imagen formada por un mosaico\n VERTICAL de dos imgs\n"
			"   -l # num for /dev/video_left (defecto 0) \n"
			"   -r # num for /dev/video_right (defecto 1) \n"
	);
}

/****************************************************************/


// A Simple Camera Capture Framework
int main(int argc, char *argv[]) {


	cv::VideoWriter grabarL;
	cv::VideoWriter grabarR;

	//	timeval tiempo1, tiempo2, tiempo3, tiempo4, tiempo5, tiempo6;
	//	timeval time_dif1,time_dif2,time_dif3, time_dif4, time_dif5;

	int grabando=0;
	int key;

	char buff[100];

	cv::VideoCapture captureL;
	cv::VideoCapture captureR;

	int opt;
	int display_activo = 0;
	int color_mode = 1;
	int convertir_agris;
	int usar_mosaico = 0;
	int record_mode = 3;

	int left_dev_video = 0;
	int right_dev_video = 1;

	double capt_width = 640.0;
	double capt_height = 480.0;

	while((opt=getopt(argc,argv,"hg:l:r:dW:H:"))!=-1) //
	{
		switch(opt)
		{
		case 'h':
			ayuda();
			exit(0);
			break;
		case 'g':
			record_mode=atoi(optarg);
			if (record_mode<0 || record_mode>6) {
				std::cerr << "record_mode erroneo\n";
				exit(-1);
			}
			break;
		case 'W':
			capt_width = atof(optarg);
			break;
		case 'H':
			capt_height = atof(optarg);
			break;
		case 'l':
			left_dev_video=atoi(optarg);
			break;
		case 'r':
			right_dev_video=atoi(optarg);
			break;

		case 'd':
			display_activo = 1;
			std::cerr << "display activado\n";
			break;


		} 
	}//while((opt=getopt())!=-1)

	if (0==display_activo) {
		std::cout << "no se ha activado el display\n";
		if (0==record_mode ) {
			std::cerr << " ni grabar Exit\n";
			exit(0);
		}
		if (record_mode>0) {  // si display no activo y grabar comenzar desde el arranque
			grabando=1;       //no se puede controlar la grabacion con la tecla espacio
		}
	}

	captureL.open(left_dev_video);
	captureR.open(right_dev_video);
	captureL.set(cv::CAP_PROP_FRAME_WIDTH , capt_width);
	captureL.set(cv::CAP_PROP_FRAME_HEIGHT, capt_height);

	captureR.set(cv::CAP_PROP_FRAME_WIDTH , capt_width);
	captureR.set(cv::CAP_PROP_FRAME_HEIGHT, capt_height);

	if( !captureL.isOpened() ) {
		std::cerr <<"ERROR: capture L " << left_dev_video<< "is not open \n" ;
		exit( -1);
	} 

	if( !captureR.isOpened() ) {
		std::cerr <<"ERROR: capture R " << right_dev_video<< "is not open \n" ;
		exit( -1);
	} 


	if (display_activo) {

		cv::namedWindow( "RIGHT", cv::WINDOW_AUTOSIZE );
		cv::namedWindow( "LEFT", cv::WINDOW_AUTOSIZE );
		cv::namedWindow( "Mosaico", cv::WINDOW_AUTOSIZE );
	}

	//================= BUCLE PRINCIPAL

	// Show the image captured from the camera in the window and repeat 
	int  ii=0;

	for( int n=0; 1; n++) {

		cv::Mat imL_cam;
		cv::Mat imR_cam;
		cv::Mat mosaico;

		if(0==n){
			for(int k=0; k < 4; k++){ //Repetir varias veces hasta que se estabilice

				int hasleft = captureL.read(imL_cam);

				int hasright = captureR.read(imR_cam);

			}
		}

		//		gettimeofday (&tiempo1, NULL);
		int hasleft = captureL.read(imL_cam);
		//		gettimeofday (&tiempo2, NULL);
		int hasright = captureR.read(imR_cam);
		//		gettimeofday (&tiempo3, NULL);

		if(! hasleft || ! hasright){
			std::cerr << "Failed to grab from one camera\n";
			break;
		}

		if(0==n)
		{
			cv::Size tam(imL_cam.cols,imL_cam.rows);
			int forcc = cv::VideoWriter::fourcc('F', 'M','P','4');  // tipo de codificador
			std::ofstream fs;
			switch (record_mode) {
			case 1:
			case 3:
				break;
			case 2:
				grabarL.open( "camL.avi", forcc,
						10, //fps
						tam);
				grabarR.open( "camR.avi", forcc,
						10, //fps
						tam);
				if(!grabarL.isOpened()){
					std::cerr << "no se puede abrir camL.avi,\n";
					exit(-1);
				}
				if(!grabarR.isOpened()){
					std::cerr << "no se puede abrir caRL.avi,\n";
					exit(-1);
				}
				break;
			case 4:
				grabarL.open( "camL.avi", forcc,
						10, //fps
						cv::Size(2*imL_cam.cols,imL_cam.rows));

				if(!grabarL.isOpened()){
					std::cerr << "no se puede abrir camL.avi,\n";
					exit(-1);
				}
				break;
			case 6:
				grabarL.open( "camL.avi", forcc,
						10, //fps
						cv::Size(imL_cam.cols,2*imL_cam.rows));
				if(!grabarL.isOpened()){
					std::cerr << "no se puede abrir camL.avi,\n";
					exit(-1);
				}
				break;
			}// switch (record_mode)



		}// if(1==primera_vez)

		std::cout << n <<"\n";
		if (3==record_mode || 4== record_mode) {
			usar_mosaico = 1;  //horizontal
			cv::hconcat(imL_cam,imR_cam,mosaico);
		}

		else if (5==record_mode || 6 == record_mode){
			usar_mosaico = 2;  // vertical
			cv::vconcat(imL_cam,imR_cam,mosaico);
		}

		if (display_activo) {
			if(0==usar_mosaico){
				cv::imshow( "RIGHT", imL_cam );
				cv::imshow( "LEFT", imR_cam );
			}
			else
				cv::imshow("Mosaico",mosaico);
			if (0==record_mode) {  //si se graba, la espera se hace mas adelante
				key=cv::waitKey(10);    //si no hay un cvWaitKey de varios ms despues de un cvShowImage el display no se actualiza
				if( key == 27 || key == 'q') // escape or q
					break;  
			}
		}


		// VIDEO FILE RECORDING MODE
		if (0==record_mode) {
			ii++;
		}
		else if(2==record_mode || 4 == record_mode || 6 == record_mode) {     
			if (1 == grabando) {
				if (2==record_mode) {
					grabarL.write( imL_cam);
					grabarR.write( imR_cam);
				}
				else  { //mosaico
					grabarL.write(mosaico);
				}
				ii++;

				std::cerr<< "imagen:"<<  ii<< "\n";
			}


		} //// VIDEO FILE RECORDING MODE

		//IMAGE FILES RECORDING MODE
		else if(1==record_mode || 3 == record_mode || 5 == record_mode){    


			if (1==record_mode) {
				sprintf(buff,"im_left_%08d.jpg",ii);
				std::string filename = buff;
				cv::imwrite(filename, imL_cam);

				sprintf(buff,"im_right_%08d.jpg",ii);
				filename = buff;
				cv::imwrite(filename, imR_cam);
			}else{
				sprintf(buff,"mosaicST_%08d.jpg",ii);
				std::string filename = buff;
				cv::imwrite(filename, mosaico);
			}
			std::cerr << "imagen: "<< ii<< "\n";
			ii++;

		} //IMAGE FILES RECORDING MODE

		if (display_activo) {
			key = cv::waitKey(20);
			if( key == 27 || key == 'q') // escape or q
				break;
			else if( key == ' ')
			{
				if (0==grabando) {
					grabando = 1;
					std::cerr <<"GRABANDO\n";
				}
				else
				{
					grabando = 0;
					std::cerr <<"GRABACION PARADA\n";
				}
			}
		}
	} //while 1

	terminar:

	if(grabarL.isOpened())
		grabarL.release();
	if(grabarR.isOpened())
		grabarR.release();


	if(captureL.isOpened())
		captureL.release();
	if(captureR.isOpened())
		captureR.release();
	return 0; 

}   //main
