
#include "xml_utilities.h"
#include "opencv2/core.hpp"



//#define cimg_plugin "cimg_gpiv.h"

#include <CImg.h>
using namespace cimg_library;

#include "string_utilities.h"
#include "file_utilities.h"
//#include "cimg_gpivfunctions.h"
#include "CimgStereo.h"

#define STEREOCALDIR "stereoCalDir"
#define MINDISTANCE "minDistance"
#define MAXDISTANCE "maxDistance"
#define SADWINSIZE "SADWinSize"
#define SADWINSIZEREL "SADWinSizeRel"
#define TEXTURETH "textureTh"
#define SPECKLEWINSIZE "speckleWinSize"
#define SPECKLERANGE "speckleRange"
#define UNIQUERATIO "uniqueRatio"
#define DECIMATE "decimate"
#define STEREOINPUTFORMAT "stereoInputFormat"
#define MAX_DISP_LR "maxDispLR"
#define DISPARITY_METHOD "DisparityMethod"
#define SCALE "scale"
#define MULTISCALEMODE "MultiscaleMode"
#define ALPHARECTIFY  "AlphaRectify"

using namespace upvsoft::misc::stringutilities;

// *******************************************************************************************

int CimgStereo::read_input_config( const char * filename){


	xmlDoc *doc = NULL;
	xmlNode *root_element = NULL;
	xmlNode *sensor_node = NULL;
	std::string tmp;


	char *buf;

	doc = xmlReadFile(filename, NULL, 0);


	if (doc == NULL) {
		std::cerr << "Can not open " << filename << " \n";
		return -1;
	}

	/*Get the root element node */
	root_element = xmlDocGetRootElement(doc);
	sensor_node = root_element;

	if( sensor_node->type != XML_ELEMENT_NODE){
		return -2;
	}

	xmlNode *node, *node_level2;


	//sensor input format
	node = GetSon( sensor_node, (char*) STEREOINPUTFORMAT );
	if ( node == NULL ){
		std::cerr << "Info: Missing " << STEREOINPUTFORMAT << "\n";
		return -2;
	}

	node_level2 = GetSon( node, (char*) "left" );
	if ( node_level2 == NULL ){
		std::cerr << "Missing " << "left" << "\n";
		return -2;
	}

	buf = (char*) xmlNodeGetContent( node_level2 );
	tmp = buf;
	tmp = trim(tmp); //Remove initial and trailing white spaces
	layoutLeft_ = layoutDecode(tmp);
	if( layoutLeft_ < 0)
	{
		std::cerr << "Invalid input format for left\n";
		return -2;
	}


	node_level2 = GetSon( node, (char*) "right" );
	if ( node_level2 == NULL ){
		std::cerr << "Missing " << "right" << "\n";
		return -2;
	}

	buf = (char*) xmlNodeGetContent( node_level2 );
	tmp = buf;
	tmp = trim(tmp); //Remove initial and trailing white spaces
	layoutRight_ = layoutDecode(tmp);
	if( layoutRight_ < 0)
	{
		std::cerr << "Invalid input format for left\n";
		return -2;
	}


	node_level2 = GetSon( node, (char*) "center" );
	if ( node_level2 == NULL ){ //It is normal to have 2 cameras ;-)
		//return -0;
	}
	else {
		buf = (char*) xmlNodeGetContent( node_level2 );
		tmp = buf;
		tmp = trim(tmp); //Remove initial and trailing white spaces
		layoutCenter_ = layoutDecode(tmp);
		if( layoutCenter_ < 0)
		{
			std::cerr << "Invalid input format for center\n";
			return -2;
		}
	}
	return 0;

}

int CimgStereo::read_config( const char * filename){

	xmlDoc *doc = NULL;
	xmlNode *root_element = NULL;
	xmlNode *sensor_node = NULL;
	std::string tmp;


	char *buf;

	doc = xmlReadFile(filename, NULL, 0);


	if (doc == NULL) {
		std::cerr << "Can not open " << filename << " \n";
		return -1;
	}

	/*Get the root element node */
	root_element = xmlDocGetRootElement(doc);
	sensor_node = root_element;

	if( sensor_node->type != XML_ELEMENT_NODE){
		return -2;
	}

	xmlNode *node, *node_level2;



	//Camera Extrinsics:OPTIONAL
	node = GetSon( sensor_node, (char*) "camera_extrinsics" );
	if ( node != NULL ){
		node_level2 = GetSon( node, (char*) "cameraHeight_metres" );
		if ( node_level2 == NULL ){
			std::cerr << "Missing " << "cameraHeight_metres" << "\n";
			return -2;
		}
		buf = (char*) xmlNodeGetContent( node_level2 );
		float camHeightMetres = atof(buf);
		if(camHeightMetres < 0)
		{
			std::cerr << "Error: Negative cameraHeight_metres value\n";
			return -2;
		}

		node_level2 = GetSon( node, (char*) "matrix_filename" );
		if ( node_level2 == NULL ){
			std::cerr << "Missing " << "matrix_filename" << "\n";
			return -2;
		}
		buf = (char*) xmlNodeGetContent( node_level2 );
		tmp = buf;
		tmp = trim(tmp); //Remove initial and trailing white spaces
		if( !upvsoft::misc::fileutilities::FileExists(tmp.c_str()) ) {
			std::cerr<<"Missing " << tmp.c_str() << std::endl;
			return -2;
		}
		CImg<float> matrix (tmp.c_str());
		if(matrix.size()!= 9)
		{
			std::cerr << "Error: Invalid matrix\n";
			matrix.print("Matrix");
			return -2;
		}
		rotationMatrix(matrix);
		camHeight_mm(camHeightMetres * 1000.0);
	}


	//sensor input format
	node = GetSon( sensor_node, (char*) STEREOINPUTFORMAT );
	if ( node == NULL ){
		std::cerr << "Info: Missing " << STEREOINPUTFORMAT << "\n";
		return -2;
	}

	node_level2 = GetSon( node, (char*) "left" );
	if ( node_level2 == NULL ){
		std::cerr << "Missing " << "left" << "\n";
		return -2;
	}

	buf = (char*) xmlNodeGetContent( node_level2 );
	tmp = buf;
	tmp = trim(tmp); //Remove initial and trailing white spaces
	layoutLeft_ = layoutDecode(tmp);
	if( layoutLeft_ < 0)
	{
		std::cerr << "Invalid input format for left\n";
		return -2;
	}


	node_level2 = GetSon( node, (char*) "right" );
	if ( node_level2 == NULL ){
		std::cerr << "Missing " << "right" << "\n";
		return -2;
	}

	buf = (char*) xmlNodeGetContent( node_level2 );
	tmp = buf;
	tmp = trim(tmp); //Remove initial and trailing white spaces
	layoutRight_ = layoutDecode(tmp);
	if( layoutRight_ < 0)
	{
		std::cerr << "Invalid input format for left\n";
		return -2;
	}


	node_level2 = GetSon( node, (char*) "center" );
	if ( node_level2 == NULL ){ //It is normal to have 2 cameras ;-)
		//return -0;
	}
	else {
		buf = (char*) xmlNodeGetContent( node_level2 );
		tmp = buf;
		tmp = trim(tmp); //Remove initial and trailing white spaces
		layoutCenter_ = layoutDecode(tmp);
		if( layoutCenter_ < 0)
		{
			std::cerr << "Invalid input format for center\n";
			return -2;
		}
	}

	// sensor params
	node = GetSon( sensor_node, (char*) STEREOCALDIR );
	if ( node == NULL ){
		std::cerr << "Missing " << STEREOCALDIR << "\n";
		return -2;
	}
	buf = (char*) xmlNodeGetContent( node );
	tmp = buf;
	tmp = trim(tmp); //Remove initial and trailing white spaces
	loadStereoCalibration(tmp.c_str());


	node = GetSon( sensor_node, (char*) DISPARITY_METHOD ); //optional Default BM
	if ( node != NULL ){
		buf = (char*) xmlNodeGetContent( node );
		tmp = buf;
		tmp = trim(tmp); //Remove initial and trailing white spaces
		if ( strcasecmp(tmp.c_str(), "bm" ) == 0)
			method(METHOD_BM);
		else	if ( strcasecmp(tmp.c_str(), "sgbm" ) == 0)
			method(METHOD_SGBM);
		else	if ( strcasecmp(tmp.c_str(), "sv" ) == 0 || strcasecmp(tmp.c_str(), "svar" ) == 0)
			method(METHOD_SVAR);
		else	if ( strcasecmp(tmp.c_str(), "oflow" ) == 0 || strcasecmp(tmp.c_str(), "of" ) == 0)
			method(METHOD_OFLOW);
		else{
			std::cerr << "Error setting disparity method to " << tmp << "\n";
			exit(0);
		}

	}
	else
		std::cout << "Info: no DisparityMethod Section Found. Setting to BM\n";


	node = GetSon( sensor_node, (char*) "SGBM_Smooth_Factor" );
	if ( node != NULL ){
		buf = (char*) xmlNodeGetContent( node );
		SGBM_SmoothFactor(atof(buf) );

	}



	node = GetSon( sensor_node, (char*) MINDISTANCE );
	if ( node == NULL ){
		std::cerr << "Missing " << MINDISTANCE << "\n";
		return -2;
	}
	buf = (char*) xmlNodeGetContent( node );
	minDistance(atof(buf) );

	node = GetSon( sensor_node, (char*) MAXDISTANCE );
	if ( node == NULL ){
		std::cerr << "Missing " << MAXDISTANCE << "\n";
		return -2;
	}
	buf = (char*) xmlNodeGetContent( node );
	maxDistance(atof(buf) );


	node = GetSon( sensor_node, (char*) SADWINSIZE );
	if ( node == NULL ){
		std::cerr << "Missing " << SADWINSIZE << "\n";
		SADWindowSize( 13 ); //Default Value
	}
	else {
		buf = (char*) xmlNodeGetContent( node );
		SADWindowSize(atoi(buf) );
	}

	node = GetSon( sensor_node, (char*) ALPHARECTIFY );
	if ( node == NULL ){
		std::cerr << "Missing " << ALPHARECTIFY << " using default 0\n";

	}
	else {
		buf = (char*) xmlNodeGetContent( node );
		alphaRectify_ = (atof(buf) );
	}

	node = GetSon( sensor_node, (char*) SADWINSIZEREL );
	if ( node ){
		buf = (char*) xmlNodeGetContent( node );
		SADWindowSizeRel(atof(buf) );
	}



	node = GetSon( sensor_node, (char*) TEXTURETH );
	if ( node == NULL ){
		std::cerr << "Missing " << TEXTURETH << "\n";
		//return -2;
		textureThreshold(3);
	}
	buf = (char*) xmlNodeGetContent( node );
	textureThreshold( atoi(buf) );


	node = GetSon( sensor_node, (char*) SPECKLEWINSIZE );
	if ( node == NULL ){
		std::cerr << "Missing " << SPECKLEWINSIZE << "\n";
		return -2;
	}
	buf = (char*) xmlNodeGetContent( node );
	speckleWindowSize( atoi(buf) );



	node = GetSon( sensor_node, (char*) SPECKLERANGE );
	if ( node == NULL ){
		std::cerr << "Missing " << SPECKLERANGE << "\n";
		return -2;
	}
	buf = (char*) xmlNodeGetContent( node );
	speckleRange( atoi(buf) );



	node = GetSon( sensor_node, (char*) UNIQUERATIO );
	if ( node == NULL ){
		std::cerr << "Missing " << UNIQUERATIO << "\n";
		return -2;
	}
	buf = (char*) xmlNodeGetContent( node );
	uniquenessRatio( atoi(buf) );



	node = GetSon( sensor_node, (char*) MAX_DISP_LR );
	if ( node == NULL ){
		disp12MaxDiff( -1 ); //Not use it
	}
	else {
		buf = (char*) xmlNodeGetContent( node );
		disp12MaxDiff( atoi(buf) );
	}



	node = GetSon( sensor_node, (char*) DECIMATE );
	if ( node == NULL ){
		std::cerr << "Missing " << DECIMATE << "\n";
		return -2;
	}
	buf = (char*) xmlNodeGetContent( node );
	decimateDisparity( atoi(buf) );

	node = GetSon( sensor_node, (char*) SCALE );
	if ( node != NULL ){
		buf = (char*) xmlNodeGetContent( node );
		scale( atof(buf) );
	}

	node = GetSon( sensor_node, (char*) MULTISCALEMODE );
	if ( node != NULL ){
		buf = (char*) xmlNodeGetContent( node );
		tmp = buf;
		tmp = trim(tmp); //Remove initial and trailing white spaces
		if (strcasecmp( tmp.c_str(), "classic") == 0 )
			multiscaleMode_ = MODE_CLASSIC;
		else if (strcasecmp( tmp.c_str(), "robust") == 0)
			multiscaleMode_ = MODE_ROBUST;
		else
			std::cerr << " \nINFO: Unknown MultiScaleMode\n";

	}
	else {
		if(decimate == 0)
			std::cerr << " \nINFO: Missing MultiScaleMode using decimate=0. Default: Classic\n";
	}

	return 0;
}
